
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
 */

#include "ath_internal.h"
#include "if_athrate.h"

#ifndef REMOVE_PKT_LOG
#include "pktlog.h"
extern struct ath_pktlog_funcs *g_pktlog_funcs;
#endif

#ifdef ATH_SUPPORT_UAPSD

#define ATH_QOSNULL_TXDESC 64

/*
 ******************************************************************************
 * UAPSD Trigger searching routine
 ******************************************************************************
 */

/*
 * Check for UAPSD Triggers.
 * Context: Rx Interrupt
 */
void
ath_check_uapsdtriggers(ath_dev_t dev)
{
#define PA2DESC(_sc, _pa)                                               \
    ((struct ath_desc *)((caddr_t)(_sc)->sc_rxdma.dd_desc +             \
                         ((_pa) - (_sc)->sc_rxdma.dd_desc_paddr)))

    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;
    struct ath_buf *bf, *bf_first, *bf_last;
    struct ath_desc *ds;
    wbuf_t wbuf;
    HAL_STATUS retval;
    bf_first = TAILQ_FIRST(&sc->sc_rxbuf);

    /* Search for UAPSD triffer event in the Rx descriptor list. */
    for (bf = bf_first; bf; bf = TAILQ_NEXT(bf, bf_list)) {

        bf_last = TAILQ_NEXT(bf, bf_list);
        if ((bf_last == NULL) && (bf->bf_status & ATH_BUFSTATUS_DONE)) {
            /*
             * The holding descriptor is the last descriptor in queue.
             */
            break;
        }

        if (bf->bf_status & ATH_BUFSTATUS_DONE)
            continue;

        ds = bf->bf_desc;

        if (ds->ds_link == bf->bf_daddr) {
            break;
        }

        retval = ath_hal_rxprocdesc(ah, ds, bf->bf_daddr, PA2DESC(sc, ds->ds_link), 0);
        if (HAL_EINPROGRESS == retval) {
            /*
             * h/w bug, the RXS could get corrupted. Hence, check
             * if the next descriptor's done bit is set or not.
             */
            struct ath_buf *tbf = TAILQ_NEXT(bf, bf_list);
            if (tbf) {
                struct ath_desc *tds = tbf->bf_desc;

                retval = ath_hal_rxprocdesc(ah, tds, tbf->bf_daddr, PA2DESC(sc, tds->ds_link), 0);

                if (HAL_EINPROGRESS == retval)
                    break;
            } else {
                break;
            }
            /* process desc again to get the updated value */
            retval = ath_hal_rxprocdesc(ah, ds, bf->bf_daddr, PA2DESC(sc, ds->ds_link), 0);
        }

        bf->bf_status |= ATH_BUFSTATUS_DONE;

        wbuf = bf->bf_mpdu;
        if (wbuf == NULL) {  /* XXX ??? can this happen */
            continue;
        }

        /* Skip frames with error */
        if (ds->ds_rxstat.rs_status != 0) {
            continue;
        }

        /* Sync before using rx buffer */
        OS_SYNC_SINGLE(sc->sc_osdev,
                       bf->bf_buf_addr, wbuf_get_len(wbuf), BUS_DMA_FROMDEVICE,
                       OS_GET_DMA_MEM_CONTEXT(bf, bf_dmacontext));


        /* Check for valid UAPSD trigger */
        sc->sc_ieee_ops->check_uapsdtrigger(sc->sc_ieee, wbuf, ds->ds_rxstat.rs_keyix);

    }
}


/*
 ******************************************************************************
 * UAPSD Queuing and Transmission routines.
 ******************************************************************************
 */

/*
 * Returns number for frames queued in software for a given node.
 * Context: Tasklet
 */
u_int32_t
ath_tx_uapsd_depth(ath_node_t node)
{
    struct ath_node *an = ATH_NODE(node);

    return an->an_uapsd_qdepth;
}

/*
 * UAPSD initialization.
 */
int
ath_tx_uapsd_init(struct ath_softc *sc)
{
    int error = 0;
    struct ath_buf *bf;
    wbuf_t wbuf;
    struct ath_desc *ds;

    ATH_UAPSD_LOCK_INIT(sc);

    sc->sc_uapsdqnuldepth = 0;

    /* Setup tx descriptors */
    error = ath_descdma_setup(sc, &sc->sc_uapsdqnuldma, &sc->sc_uapsdqnulbf,
                                  "uapsd_qnull", ATH_QOSNULL_TXDESC, ATH_TXDESC);
    if (error != 0) {
        printk("failed to allocate UAPSD QoS NULL tx descriptors: %d\n", error);
        return error;
    }

    /* Format QoS NULL frame */
    TAILQ_FOREACH(bf, &sc->sc_uapsdqnulbf, bf_list)
    {

        ATH_TXBUF_RESET(bf);
        wbuf = sc->sc_ieee_ops->uapsd_allocqosnullframe();
        if (wbuf == NULL) {
            error = -ENOMEM;
            printk("failed to allocate UAPSD QoS NULL wbuf\n");
            break;
        }

        bf->bf_frmlen = sizeof(struct ieee80211_qosframe) + IEEE80211_CRC_LEN;
        bf->bf_mpdu = wbuf;
        bf->bf_isdata = 1;
        bf->bf_lastfrm = bf;
        bf->bf_lastbf = bf->bf_lastfrm; /* one single frame */
        bf->bf_isaggr  = 0;
        bf->bf_isbar  = bf->bf_isampdu = 0;
        bf->bf_next = NULL;
        bf->bf_qosnulleosp = 1;

        bf->bf_buf_addr = wbuf_map_single(sc->sc_osdev, wbuf, BUS_DMA_TODEVICE,
                                          OS_GET_DMA_MEM_CONTEXT(bf, bf_dmacontext));

        /* setup descriptor */
        ds = bf->bf_desc;
        ds->ds_link = 0;
        ds->ds_data = bf->bf_buf_addr;
#ifndef REMOVE_PKT_LOG
        ds->ds_vdata = wbuf_header(wbuf);
#endif

        /*
         * Formulate first tx descriptor with tx controls.
         */
        ath_hal_set11n_txdesc(sc->sc_ah, ds
                              , bf->bf_frmlen           /* frame length */
                              , HAL_PKT_TYPE_NORMAL     /* Atheros packet type */
                              , MIN(sc->sc_curtxpow, 60)/* txpower */
                              , HAL_TXKEYIX_INVALID     /* key cache index */
                              , HAL_KEY_TYPE_CLEAR      /* key type */
                              , HAL_TXDESC_INTREQ
                                | HAL_TXDESC_CLRDMASK   /* flags */
                            );

        ath_hal_filltxdesc(sc->sc_ah, ds, bf->bf_frmlen, AH_TRUE, AH_TRUE, ds);

        /* NB: The desc swap function becomes void,
         * if descriptor swapping is not enabled
         */
        ath_desc_swap(ds);
    }

    if (error) {
        ath_tx_uapsd_cleanup(sc);
    }

    return error;
}

/*
 * Reclaim all UAPSD resources.
 * Context: Tasklet
 */
void
ath_tx_uapsd_cleanup(struct ath_softc *sc)
{
    ieee80211_tx_status_t tx_status;
    struct ath_buf *bf;
    wbuf_t wbuf;

    tx_status.flags = 0;

    TAILQ_FOREACH(bf, &sc->sc_uapsdqnulbf, bf_list)
    {
        wbuf = bf->bf_mpdu;
        /* Unmap this frame */
        wbuf_unmap_sg(sc->sc_osdev, wbuf,
                      OS_GET_DMA_MEM_CONTEXT(bf, bf_dmacontext));

        wbuf_release(sc->sc_osdev, WBUF_TX_INTERNAL, wbuf);
    }

    if (sc->sc_uapsdqnuldma.dd_desc_len != 0)
        ath_descdma_cleanup(sc, &sc->sc_uapsdqnuldma, &sc->sc_uapsdqnulbf);

    ATH_UAPSD_LOCK_DESTROY(sc);
}

void
ath_tx_uapsd_draintxq(struct ath_softc *sc)
{
    struct ath_buf *bf, *lastbf;
    ath_bufhead bf_head;
    struct ath_txq *txq = sc->sc_uapsdq;

    TAILQ_INIT(&bf_head);
    /*
     * NB: this assumes output has been stopped and
     *     we do not need to block ath_tx_tasklet
     */
    ATH_UAPSD_LOCK_IRQ(sc);
    for (;;) {
        bf = TAILQ_FIRST(&txq->axq_q);

        if (bf == NULL) {
            txq->axq_link = NULL;
            txq->axq_linkbuf = NULL;
            break;
        }

        if (bf->bf_status & ATH_BUFSTATUS_STALE) {
            ATH_TXQ_REMOVE_STALE_HEAD(txq, bf, bf_list);

            if (bf->bf_qosnulleosp) {
                TAILQ_INSERT_TAIL(&sc->sc_uapsdqnulbf, bf, bf_list);
                sc->sc_uapsdqnuldepth--;
            } else {
                ATH_UAPSD_UNLOCK_IRQ(sc);
                ATH_TXBUF_LOCK(sc);
                TAILQ_INSERT_TAIL(&sc->sc_txbuf, bf, bf_list);
                ATH_TXBUF_UNLOCK(sc);
                ATH_UAPSD_LOCK_IRQ(sc);
            }
            continue;
        }

        KASSERT(bf->bf_lastbf, ("bf->bf_lastbf is NULL"));
        lastbf = bf->bf_lastbf;
        lastbf->bf_desc->ds_txstat.ts_flags = HAL_TX_SW_ABORTED;

        /* remove ath_buf's of the same mpdu from txq */
        ATH_TXQ_MOVE_HEAD_UNTIL(txq, &bf_head, lastbf, bf_list);

        ATH_UAPSD_UNLOCK_IRQ(sc);
        ath_tx_uapsd_complete(sc, bf->bf_node, bf, &bf_head, 0);
        ATH_UAPSD_LOCK_IRQ(sc);
    }
    ATH_UAPSD_UNLOCK_IRQ(sc);
}


/*
 * Reclaim all UAPSD node resources.
 * Context: Tasklet
 */
void
ath_tx_uapsd_node_cleanup(struct ath_softc *sc, struct ath_node *an)
{
    struct ath_buf *bf;
    ath_bufhead bf_head;
    struct ath_txq *txq;

    txq = sc->sc_uapsdq;

    ATH_UAPSD_LOCK_IRQ(sc);
    /*
     * Drain the uapsd software queue.
     */
    for (;;) {
        bf = TAILQ_FIRST(&an->an_uapsd_q);

        if (bf == NULL)
            break;

        TAILQ_REMOVE_HEAD_UNTIL(&an->an_uapsd_q, &bf_head, bf->bf_lastfrm, bf_list);

        /* complete this sub-frame */
        ATH_UAPSD_UNLOCK_IRQ(sc);
        ath_tx_complete_buf(sc, bf, &bf_head, 0);
        ATH_UAPSD_LOCK_IRQ(sc);
    }

    ATH_UAPSD_UNLOCK_IRQ(sc);
}

/*
 * Add frames to per node UAPSD queue.
 * Frames will be transmitted on receiving trigger.
 * Context: Tasklet
 */
void
ath_tx_queue_uapsd(struct ath_softc *sc, struct ath_txq *txq, ath_bufhead *bf_head, ieee80211_tx_control_t *txctl)
{
    struct ath_buf *bf, *bf_prev;
    struct ath_node *an = txctl->an;

    bf = TAILQ_FIRST(bf_head);

    if (txctl->ht && sc->sc_txaggr) {
        bf->bf_isampdu = 0;
        bf->bf_seqno = txctl->seqno; /* save seqno and tidno in buffer */
        bf->bf_tidno = txctl->tidno;
    }

    bf->bf_nframes = 1;
    bf->bf_lastfrm->bf_desc->ds_link = 0; // XXX
    bf->bf_lastbf = bf->bf_lastfrm; /* one single frame */
    bf->bf_next = NULL;

    /*
     * Lock out interrupts since this queue shall be accessed
     * in interrupt context.
     */
    ATH_UAPSD_LOCK_IRQ(sc);
    if ((bf_prev = TAILQ_LAST(&an->an_uapsd_q, ath_bufhead_s))) {
        bf_prev->bf_desc->ds_link = bf->bf_daddr;
        bf_prev->bf_next = bf;
    }

    TAILQ_CONCAT(&an->an_uapsd_q, bf_head, bf_list);
    an->an_uapsd_qdepth++;

    sc->sc_stats.ast_uapsddataqueued++;

    ATH_UAPSD_UNLOCK_IRQ(sc);
}


/*
 * Reclaim used QoS NULL buffer, requeue in free pool.
 * Context: Tasklet
 */
void
ath_tx_uapsdqnulbf_reclaim(struct ath_softc *sc, struct ath_buf *bf, ath_bufhead *bf_q)
{
    ATH_UAPSD_LOCK_IRQ(sc);

    TAILQ_CONCAT(&sc->sc_uapsdqnulbf, bf_q, bf_list);
    sc->sc_uapsdqnuldepth--;

    ATH_UAPSD_UNLOCK_IRQ(sc);
}

/*
 * Handle transmit completion for UAPSD frames.
 * Context: Tx tasklet
 */
void
ath_tx_uapsd_complete(struct ath_softc *sc, struct ath_node *an, struct ath_buf *bf, ath_bufhead *bf_q, int txok)
{
    ath_bufhead bf_head;
    struct ath_buf *bf_next, *bf_lastq = NULL;

    if(!an) {
        DPRINTF(sc, ATH_DEBUG_UAPSD, "%s: ath_node is null in bf %p\n", __func__, bf);
        return;
    }

    TAILQ_INIT(&bf_head);

    while (bf) {

        bf_next = bf->bf_next;

        /*
         * Indicate EOSP to upper layer for appropriate handling.
         */
        ATH_UAPSD_LOCK_IRQ(sc);
        sc->sc_ieee_ops->uapsd_eospindicate(an->an_node, bf->bf_mpdu, txok);
        ATH_UAPSD_UNLOCK_IRQ(sc);

        /*
         * Is this the last ath_buf in the txq
         */
        if (bf_next == NULL) {
            KASSERT(bf->bf_lastfrm == bf->bf_lastbf, ("bf_lastfrm != bf->bf_lastbf"));

            bf_lastq = TAILQ_LAST(bf_q, ath_bufhead_s);
            if (bf_lastq) {
                TAILQ_REMOVE_HEAD_UNTIL(bf_q, &bf_head, bf_lastq, bf_list);
            } else {
                KASSERT(TAILQ_EMPTY(bf_q), ("bf_q NOT EMPTY"));
                TAILQ_INIT(&bf_head);
            }
        } else {
            KASSERT(!TAILQ_EMPTY(bf_q), ("bf_q EMPTY"));
            TAILQ_REMOVE_HEAD_UNTIL(bf_q, &bf_head, bf->bf_lastfrm, bf_list);
        }

        if (bf->bf_qosnulleosp) {
            if (!TAILQ_EMPTY(&bf_head)) {
                ath_tx_uapsdqnulbf_reclaim(sc, bf, &bf_head);
            }
            sc->sc_stats.ast_uapsdqnulcomp++;
        } else {
            /* complete this frame */
            ath_tx_complete_buf(sc, bf, &bf_head, txok);
            sc->sc_stats.ast_uapsddatacomp++;
        }

        bf = bf_next;
    }
}

/*
 * Send a QoS NULL frame with EOSP set.  Used when we want to terminate the service
 * period because we have no data frames to send to this node.
 * Context: Interrupt
 */
static void
ath_uapsd_sendqosnull(struct ath_softc *sc, struct ath_node *an, u_int8_t ac)
{
    struct ath_buf *bf;
    ath_bufhead bf_head;
    int prate;

    if (!TAILQ_EMPTY(&sc->sc_uapsdqnulbf)) {
        bf = TAILQ_FIRST(&sc->sc_uapsdqnulbf);
        TAILQ_REMOVE(&sc->sc_uapsdqnulbf, bf, bf_list);
    } else {
        sc->sc_stats.ast_uapsdqnulbf_unavail++;
        return;
    }

    sc->sc_uapsdqnuldepth++;

    DPRINTF(sc, ATH_DEBUG_UAPSD, "%s: bf %p ds %p depth %d\n",
            __func__, bf, bf->bf_desc, sc->sc_uapsdqnuldepth);
    TAILQ_INIT(&bf_head);
    TAILQ_INSERT_TAIL(&bf_head, bf, bf_list);

    ath_rate_findrate(sc, an, AH_TRUE, 0, ATH_11N_TXMAXTRY,
                      ATH_RC_PROBE_ALLOWED, ac, bf->bf_rcs, &prate, AH_FALSE);

    bf->bf_node = an;
    ath_buf_set_rate(sc, bf);

    /*
     * Format a QoS NULL frame for this node and ac
     */
    bf->bf_mpdu = sc->sc_ieee_ops->uapsd_getqosnullframe(an->an_node, bf->bf_mpdu, ac);

    bf->bf_desc->ds_link = 0;

    /*
     * This buffer needs cache sync because we modified it here.
     */
    OS_SYNC_SINGLE(sc->sc_osdev, bf->bf_buf_addr, wbuf_get_pktlen(bf->bf_mpdu),
                    BUS_DMA_TODEVICE, OS_GET_DMA_MEM_CONTEXT(bf, bf_dmacontext));

    ath_tx_txqaddbuf(sc, sc->sc_uapsdq, &bf_head);

    sc->sc_stats.ast_uapsdqnul_pkts++;
}

/*
 * This function will send UAPSD frames out to a destination node.
 * Context: Interrupt
 */
int
ath_process_uapsd_trigger(ath_dev_t dev, ath_node_t node, u_int8_t maxsp, u_int8_t ac, u_int8_t flush)
{
    struct ath_node *an = ATH_NODE(node);
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_buf *bf, *first_bf, *last_bf = NULL;
    struct ath_txq *txq;
    u_int8_t count;
    struct ieee80211_qosframe *whqos;
    ath_bufhead bf_q, bf_head;
    struct ath_desc *last_ds;
    struct ath_rc_series rcs[4];
    int prate;

    sc->sc_stats.ast_uapsdtriggers++;

    if (!(an->an_flags & ATH_NODE_UAPSD)) {
        sc->sc_stats.ast_uapsdnodeinvalid++;
        return 0;
    }

    /*
     * UAPSD queue is empty. Send QoS NULL if this is
     * not a flush operation.
     */
    if (TAILQ_EMPTY(&an->an_uapsd_q)) {
        if (!flush)
            ath_uapsd_sendqosnull(sc, an, ac);
        return 0;
    }

    TAILQ_INIT(&bf_q);
    TAILQ_INIT(&bf_head);

    /*
     * Send all frames
     */
    if (maxsp == WME_UAPSD_NODE_MAXQDEPTH)
        maxsp = an->an_uapsd_qdepth;

    txq = sc->sc_uapsdq;

    ath_rate_findrate(sc, an, AH_TRUE, 0, ATH_11N_TXMAXTRY,
                      ATH_RC_PROBE_ALLOWED, ac, rcs, &prate, AH_FALSE);

    for (count = 0; count < maxsp; count++) {
        bf = TAILQ_FIRST(&an->an_uapsd_q);

        if (bf == NULL)
            break;

        /* Copy the rates into each buffer */
        memcpy(bf->bf_rcs, rcs, sizeof(rcs));
        ath_buf_set_rate(sc, bf);

        last_bf = bf;

        TAILQ_REMOVE_HEAD_UNTIL(&an->an_uapsd_q, &bf_head, bf->bf_lastfrm, bf_list);

        TAILQ_CONCAT(&bf_q, &bf_head, bf_list);
    }

    first_bf = TAILQ_FIRST(&bf_q);
    first_bf->bf_lastbf = last_bf->bf_lastbf;
    last_bf->bf_next = NULL;
    an->an_uapsd_qdepth -= count;

    /*
     * Request Tx Interrupt for last descriptor
     */
    last_ds = last_bf->bf_lastbf->bf_desc;
    ath_hal_txreqintrdesc(sc->sc_ah, last_ds);
    last_ds->ds_link = 0;

    /*
     * Mark EOSP flag at the last frame in this SP
     */
    whqos = (struct ieee80211_qosframe *)wbuf_header(last_bf->bf_mpdu);
    if (!flush) {
        whqos->i_qos[0] |= IEEE80211_QOS_EOSP;
        sc->sc_stats.ast_uapsdeospdata++;
    }

    sc->sc_stats.ast_uapsddata_pkts += count;

    /*
     * Clear More data bit for EOSP frame if we have
     * no pending frames
     */
    if (TAILQ_EMPTY(&an->an_uapsd_q)) {
        whqos->i_fc[1] &= ~IEEE80211_FC1_MORE_DATA;
    }

    /*
     * The last buffer needs cache sync because we modified it here.
     */
    OS_SYNC_SINGLE(sc->sc_osdev, last_bf->bf_buf_addr, wbuf_get_pktlen(last_bf->bf_mpdu),
                    BUS_DMA_TODEVICE, OS_GET_DMA_MEM_CONTEXT(last_bf, bf_dmacontext));

    ath_tx_txqaddbuf(sc, txq, &bf_q);

    return (an->an_uapsd_qdepth);
}


#endif /* ATH_SUPPORT_UAPSD */
