/*
 * Copyright (c) 2009, Atheros Communications Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

/*
 * A-MSDU
 */
#include "if_athvar.h"
#include "if_ath_amsdu.h"
#include "if_llc.h"
#include "if_ethersubr.h"
#include "ath_timer.h"

#ifdef ATH_AMSDU

#define AMSDU_TIMEOUT 15
#define AMSDU_MAX_SUBFRM_LEN 100
#define AMSDU_BUFFER_HEADROOM 64
#define AMSDU_MAX_BUFFER_SIZE 2332
#define AMSDU_MAX_LEN (AMSDU_MAX_BUFFER_SIZE - AMSDU_BUFFER_HEADROOM)


/*
 * This routine picks an AMSDU buffer, calls the platform specific 802.11 layer for
 * WLAN encapsulation and then dispatches it to hardware for transmit.
 * This routine can be called from Transmit DPC and timer context (both are softIRQ
 * in Linux, and therefore they need appropriate "bh" locking).
 */
void
ath_amsdu_stageq_flush(struct ath_softc_net80211 *scn, struct ath_amsdu_tx *amsdutx)
{
    struct ieee80211_node *ni;
    struct ieee80211com *ic;
    wbuf_t wbuf;

    ATH_AMSDU_TXQ_LOCK(scn);
    wbuf = amsdutx->amsdu_tx_buf;
    if (!wbuf) {
        ATH_AMSDU_TXQ_UNLOCK(scn);
        return;
    }

    amsdutx->amsdu_tx_buf = NULL;
    ATH_AMSDU_TXQ_UNLOCK(scn);

    ni = wbuf_get_node(wbuf);

    ic = ni->ni_ic;

    /*
     * Encapsulate the packet for transmission
     */
    wbuf = ieee80211_encap(ni, wbuf);
    if (wbuf == NULL) {
        printk ("ERROR: ieee80211_encap ret NULL\n");
        return;
    }

    /* There is only one wbuf to send */
    if (wbuf != NULL) {
        ieee80211_tx_control_t txctl;

        txctl.iseap=0;

        /* prepare this frame */
        if (ath_tx_prepare(scn, wbuf, 0, &txctl) != 0)
            goto bad;

        /* send this frame to hardware */
        txctl.an = (ATH_NODE_NET80211(ni))->an_sta;
        if (scn->sc_ops->tx(scn->sc_dev, wbuf, &txctl) != 0)
            goto bad;
    }

    return;

bad:
    /* drop rest of the un-sent fragments */
    if (wbuf != NULL) {
        IEEE80211_TX_COMPLETE_WITH_ERROR(wbuf, 0);
    }

}

void
ath_amsdu_tx_drain(struct ath_softc_net80211 *scn)
{
    struct ath_amsdu_tx *amsdutx;
    wbuf_t wbuf;

    do {
        ATH_AMSDU_TXQ_LOCK(scn);
        amsdutx = TAILQ_FIRST(&scn->sc_amsdu_txq);
        if (amsdutx == NULL) {
            /* nothing to process */
            ATH_AMSDU_TXQ_UNLOCK(scn);
            break;
        }

        TAILQ_REMOVE(&scn->sc_amsdu_txq, amsdutx, amsdu_qelem);
        amsdutx->sched = 0;

        wbuf = amsdutx->amsdu_tx_buf;
        amsdutx->amsdu_tx_buf = NULL;
        ATH_AMSDU_TXQ_UNLOCK(scn);

        if (wbuf != NULL) {
            IEEE80211_TX_COMPLETE_WITH_ERROR(wbuf, 0);
        }

    } while (!TAILQ_EMPTY(&scn->sc_amsdu_txq));
}

/*
 * This global timer routine checks a queue for posted AMSDU events.
 * For every AMSDU that we started, we add an event to this queue.
 * When the timer expires, it sends out all outstanding AMSDU bufffers.
 * We want to make sure that no buffer is sitting around for too long.
 */
int
ath_amsdu_flush_timer(void *arg)
{
    struct ath_softc_net80211 *scn = (struct ath_softc_net80211 *)arg;
    struct ath_amsdu_tx *amsdutx;

    do {
        ATH_AMSDU_TXQ_LOCK(scn);
        amsdutx = TAILQ_FIRST(&scn->sc_amsdu_txq);
        if (amsdutx == NULL) {
            /* nothing to process */
            ATH_AMSDU_TXQ_UNLOCK(scn);
            break;
        }

        TAILQ_REMOVE(&scn->sc_amsdu_txq, amsdutx, amsdu_qelem);
        amsdutx->sched = 0;
        ATH_AMSDU_TXQ_UNLOCK(scn);

        ath_amsdu_stageq_flush(scn, amsdutx);

    } while (!TAILQ_EMPTY(&scn->sc_amsdu_txq));

    return 1;   /* dont re-schedule */
}

/*
 * If the receive phy rate is lower than the threshold or our transmit queue has at least one
 * frame to work on, we keep building the AMSDU.
 */
int
ath_amsdu_sched_check(struct ath_softc_net80211 *scn, struct ath_node_net80211 *anode,
                      int priority)
{
    if ((ATH_RATE_OUT(anode->an_avgrxrate) <= 270000) ||
        (scn->sc_ops->txq_depth(scn->sc_dev, scn->sc_ac2q[priority]) >= 1))
    {
        return 1;
    }

    return 0;
}

/*
 * This routine either starts a new AMSDU or it adds data to an existing one.
 * Each buffer is AMSDU encapsulated by our platform specific 802.11 protocol layer.
 * Once we have completed preparing the AMSDU, we call the flush routine to send it
 * out. We can stop filling an AMSDU for the following reasons -
 *    - the 2K AMSDU buffer is full
 *    - the hw transmit queue has only one frame to work on
 *    - we want to transmit a frame that cannot be in an AMSDU. i.e. frame larger than
 *      max subframe limit, EAPOL or multicast frame
 *    - the no activity timer expires and flushes any outstanding AMSDUs.
 */
wbuf_t
ath_amsdu_send(wbuf_t wbuf)
{
    struct ieee80211_node *ni = wbuf_get_node(wbuf);
    struct ieee80211com *ic = ni->ni_ic;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    struct ath_node_net80211 *anode = (struct ath_node_net80211 *)ni;
    struct ieee80211vap *vap = ni->ni_vap;
    wbuf_t amsdu_wbuf;
    u_int8_t tidno = wbuf_get_tid(wbuf);
    u_int32_t framelen;
    struct ath_amsdu_tx *amsdutx;
    int amsdu_deny;

    /* AMSDU handling not initialized */
    if (anode->an_amsdu == NULL) {
        printk("ERROR: ath_amsdu_attach not called\n");
        return wbuf;
    }

    if (ieee80211vap_get_fragthreshold(vap) < 2346 ||
        !(ieee80211vap_get_opmode(vap) == IEEE80211_M_STA ||
        ieee80211vap_get_opmode(vap) == IEEE80211_M_HOSTAP))
    {
        return wbuf;
    }

    amsdu_deny = ieee80211_amsdu_check(wbuf);

    framelen = roundup(wbuf_get_pktlen(wbuf), 4);

    ATH_AMSDU_TXQ_LOCK(scn);

    amsdutx = &(anode->an_amsdu->amsdutx[tidno]);
    amsdu_wbuf = amsdutx->amsdu_tx_buf;

    ATH_AMSDU_TXQ_UNLOCK(scn);


    if (amsdu_wbuf) {
        /* If AMSDU staging buffer exists, we need to add this wbuf
         * to it.
         */

        /*
         * Is there enough room in the AMSDU staging buffer?
         * Is the newly arrived wbuf larger than our amsdu limit?
         * If not, dispatch the AMSDU and return the current wbuf.
         */
        if ((framelen > AMSDU_MAX_SUBFRM_LEN) || amsdu_deny) {
            /* Flush the amsdu q */
            ath_amsdu_stageq_flush(scn, amsdutx);
            return wbuf;
        }

        if (ath_amsdu_sched_check(scn, anode, wbuf_get_priority(amsdu_wbuf)) &&
            (AMSDU_MAX_SUBFRM_LEN + framelen + wbuf_get_pktlen(amsdu_wbuf)) < AMSDU_MAX_LEN)
        {
            /* We are still building the AMSDU */
            ieee80211_amsdu_encap(amsdu_wbuf, wbuf, framelen, 0);
            /* Free the tx buffer */
            ieee80211_complete_wbuf(wbuf, NULL, 0);
            return NULL;
        } else {
            /*
             * This is the last wbuf to be added to the AMSDU
             * No pad for this frame.
             * Return the AMSDU wbuf back.
             */
            ieee80211_amsdu_encap(amsdu_wbuf, wbuf, wbuf_get_pktlen(wbuf), 0);

            /* Free the tx buffer */
            ieee80211_complete_wbuf(wbuf, NULL, 0);

            ath_amsdu_stageq_flush(scn, amsdutx);
            return NULL;
        }
    } else {
        /* Begin building the AMSDU */

        /* AMSDU for small frames only */
        if ((framelen > AMSDU_MAX_SUBFRM_LEN) || amsdu_deny)
            return wbuf;

        amsdu_wbuf = wbuf_alloc(scn->sc_osdev, WBUF_TX_INTERNAL, AMSDU_MAX_BUFFER_SIZE);

        /* No AMSDU buffer available */
        if (amsdu_wbuf == NULL)
            return wbuf;

        /* Perform 802.11 AMSDU encapsulation */
        ieee80211_amsdu_encap(amsdu_wbuf, wbuf, framelen, 1);

        /*
         * Copy information from buffer
         * Bump reference count for the node.
         */
        wbuf_set_priority(amsdu_wbuf, wbuf_get_priority(wbuf));
        wbuf_set_tid(amsdu_wbuf, tidno);
#ifdef NODE_FREE_DEBUG
        wbuf_set_node(amsdu_wbuf, ieee80211_ref_node(ni, __func__));
#else
        wbuf_set_node(amsdu_wbuf, ieee80211_ref_node(ni));
#endif
        wbuf_set_amsdu(amsdu_wbuf);

        ATH_AMSDU_TXQ_LOCK(scn);

        amsdutx->amsdu_tx_buf = amsdu_wbuf;

        if (!amsdutx->sched) {
            amsdutx->sched = 1;
            TAILQ_INSERT_TAIL(&scn->sc_amsdu_txq, amsdutx, amsdu_qelem);
        }
        ATH_AMSDU_TXQ_UNLOCK(scn);

        /* Free the tx buffer */
        ieee80211_complete_wbuf(wbuf, NULL, 0);

        if (!ath_timer_is_active(&scn->sc_amsdu_flush_timer))
            ath_start_timer(&scn->sc_amsdu_flush_timer);

        return NULL;
    }
}

int
ath_amsdu_attach(struct ath_softc_net80211 *scn)
{
    TAILQ_INIT(&scn->sc_amsdu_txq);
    ATH_AMSDU_TXQ_LOCK_INIT(scn);

    /* Initialize the no-activity timer */
    ath_initialize_timer(scn->sc_osdev, &scn->sc_amsdu_flush_timer, AMSDU_TIMEOUT,
                             ath_amsdu_flush_timer, scn);
    return 0;
}

void
ath_amsdu_detach(struct ath_softc_net80211 *scn)
{
    ATH_AMSDU_TXQ_LOCK_DESTROY(scn);
    ath_cancel_timer(&scn->sc_amsdu_flush_timer, CANCEL_NO_SLEEP);
}

int
ath_amsdu_node_attach(struct ath_softc_net80211 *scn, struct ath_node_net80211 *anode)
{
    int tidno;
    struct ath_amsdu_tx *amsdutx;

    anode->an_amsdu = (struct ath_amsdu *)OS_MALLOC(scn->sc_osdev,
                                               sizeof(struct ath_amsdu),
                                               GFP_ATOMIC);
    if (anode->an_amsdu == NULL) {
		return ENOMEM;
	}

    OS_MEMZERO(anode->an_amsdu, sizeof(struct ath_amsdu));

    for (tidno = 0; tidno < WME_NUM_TID; tidno++) {
        amsdutx = &(anode->an_amsdu->amsdutx[tidno]);
    }

    return 0;
}

void
ath_amsdu_node_detach(struct ath_softc_net80211 *scn, struct ath_node_net80211 *anode)
{
    /* Cleanup resources */
    if (anode->an_amsdu)
        OS_FREE(anode->an_amsdu);
}
#endif /* ATH_AMSDU */
