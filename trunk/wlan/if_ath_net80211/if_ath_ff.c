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
 * SuperG Fast Frame 
 */
#include "if_athvar.h"
#include "if_ath_ff.h"

#ifdef ATH_SUPERG_FF

int 
ath_ff_attach(struct ath_softc_net80211 *scn, struct ath_node_net80211 *anode)
{
    anode->an_ff = (struct ath_ff *)OS_MALLOC(scn->sc_osdev, 
                                               sizeof(struct ath_ff),
                                               GFP_KERNEL);
    if (anode->an_ff == NULL) {
		printk("%s: no memory for ff attach\n", __func__);
		return ENOMEM;
	}
    OS_MEMZERO(anode->an_ff, sizeof(struct ath_ff));
    ATH_FF_LOCK_INIT(anode->an_ff);

    return 0;
}

void
ath_ff_detach(struct ath_node_net80211 *anode)
{
    ATH_FF_LOCK_DESTROY(anode->an_ff);
    /* Cleanup resources */
    if (anode->an_ff)
        OS_FREE(anode->an_ff);
}

void
ath_ff_stageq_flush(struct ieee80211_node *ni, int priority)
                   {
    struct ath_node_net80211 *anode = (struct ath_node_net80211 *)ni;
    struct ieee80211com *ic = ni->ni_ic;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    ieee80211_tx_control_t *txctl;
    wbuf_t wbuf, next_wbuf;


    ATH_FF_LOCK(anode->an_ff);
    wbuf = anode->an_ff->aff_tx_buf[priority];
    if (!wbuf) {
        ATH_FF_UNLOCK(anode->an_ff);
        return;
    }

    anode->an_ff->aff_tx_buf[priority] = NULL;
    ATH_FF_UNLOCK(anode->an_ff);
    
    /*
     * Encapsulate the packet for transmission
     */
    wbuf = ieee80211_encap(ni, wbuf);
    if (wbuf == NULL) {
        return;
    }

    /* send down each fragment */
    while (wbuf != NULL) {
        int nextfraglen = 0;
        ATH_DEFINE_TXCTL(txctl, wbuf);

        next_wbuf = wbuf_next(wbuf);
        if (next_wbuf != NULL)
            nextfraglen = wbuf_get_pktlen(next_wbuf);

        /* prepare this frame */
        txctl = (ieee80211_tx_control_t *)wbuf_get_context(wbuf);
        txctl->iseap=0;
        if (ath_tx_prepare(scn, wbuf, nextfraglen, txctl) != 0)
            goto bad;

        /* send this frame to hardware */
        txctl->an = (ATH_NODE_NET80211(ni))->an_sta;
        if (scn->sc_ops->tx(scn->sc_dev, wbuf, txctl) != 0)
            goto bad;

        wbuf = next_wbuf;
    }

    return;

bad:
    /* drop rest of the un-sent fragments */
    while (wbuf != NULL) {
        next_wbuf = wbuf_next(wbuf);
        IEEE80211_TX_COMPLETE_WITH_ERROR(wbuf, 0);

        wbuf = next_wbuf;
    }

}


wbuf_t
ath_ff(wbuf_t wbuf)
{
    struct ieee80211_node *ni = wbuf_get_node(wbuf);
    struct ieee80211com *ic = ni->ni_ic;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    struct ath_node_net80211 *anode = (struct ath_node_net80211 *)ni;
    struct ieee80211vap *vap = ni->ni_vap;
    wbuf_t next_wbuf, prev_wbuf;
    int ff_flush, priority = wbuf_get_priority(wbuf);
    int qnum = scn->sc_ac2q[priority];
    struct wmeParams *wmep = ieee80211_wmm_chanparams(ic, priority);
    u_int32_t framelen, txoplimit = IEEE80211_TXOP_TO_US(wmep->wmep_txopLimit);
    struct ieee80211_frame *wh = (struct ieee80211_frame *)wbuf_header(wbuf);

    if (IEEE80211_NODE_USEAMPDU(ni)) {
        return wbuf;
    }

    if (ieee80211vap_get_fragthreshold(vap) < IEEE80211_FRAGMT_THRESHOLD_MAX) {
        return wbuf;
    }
    
    if (!(ieee80211vap_get_opmode(vap) == IEEE80211_M_STA || ieee80211vap_get_opmode(vap) == IEEE80211_M_HOSTAP)) {
        return wbuf;
    }

#ifdef ATHR_RNWF
    if (!(wh->i_fc[0] | IEEE80211_FC0_TYPE_DATA))
        return wbuf;
#endif

    ATH_FF_LOCK(anode->an_ff);
    if (anode->an_ff->aff_tx_buf[priority]) {
        if (wbuf_get_exemption_type(anode->an_ff->aff_tx_buf[priority]) != wbuf_get_exemption_type(wbuf)) {
            ATH_FF_UNLOCK(anode->an_ff);
            ath_ff_stageq_flush(ni, priority);
            return wbuf;
        }
    }

    /*
     * Approximate the frame length to be transmitted. 
     *   - 12: WEP_QOS_LEN
     *   - 8:  LLC_SNAPFRAMELEN
     *   - 4 + 4: fast-frame header and padding
     *   - 32: 2 ether headers + pad
     */
    framelen = wbuf_get_pktlen(wbuf) + 12 + 8 + 8 + 32;
    if (ieee80211vap_has_flag(vap, IEEE80211_F_PRIVACY)) {
        framelen += 24;
    }

    if (anode->an_ff->aff_tx_buf[priority]){
        framelen += wbuf_get_pktlen(anode->an_ff->aff_tx_buf[priority]);
        if (scn->sc_ops->check_ff(scn->sc_dev, ATH_NODE_NET80211(ni)->an_sta, qnum,
            FALSE, txoplimit, framelen)) {

            prev_wbuf = anode->an_ff->aff_tx_buf[priority];
            anode->an_ff->aff_tx_buf[priority] = NULL;
            ATH_FF_UNLOCK(anode->an_ff);
            /*
             * chain wbuf and add FF magic
             */
            
            prev_wbuf->wb_next = wbuf;
            wbuf = prev_wbuf;
            wbuf_set_fastframe(wbuf);

            DPRINTF(scn, ATH_DEBUG_XMIT | ATH_DEBUG_FF,
                    "%s: aggregating fast-frame\n", __func__);
        } else {
            ATH_FF_UNLOCK(anode->an_ff);
            ath_ff_stageq_flush(ni, priority);
        }
    } else {
        if (scn->sc_ops->check_ff(scn->sc_dev, ATH_NODE_NET80211(ni)->an_sta, qnum,
            TRUE, txoplimit, framelen)) {
            anode->an_ff->aff_tx_buf[priority] = wbuf;

            DPRINTF(scn, ATH_DEBUG_XMIT | ATH_DEBUG_FF,
                "%s: adding to fast-frame stage Q\n", __func__);
            ATH_FF_UNLOCK(anode->an_ff);
            return NULL; 
        }
        ATH_FF_UNLOCK(anode->an_ff);
    }
    
    return wbuf;
}
#endif
