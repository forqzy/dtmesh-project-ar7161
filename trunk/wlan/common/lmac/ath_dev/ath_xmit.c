
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

void
ath_buf_set_rate(struct ath_softc *sc, struct ath_buf *bf);

#ifdef ATH_SUPPORT_IQUE
void
ath_rateseries_update_retry(struct ath_softc *sc, struct ath_buf *bf, HAL_11N_RATE_SERIES *series, int retry_duration);
#endif

static INLINE int
ath_aggr_query(struct ath_softc *sc, struct ath_node *an, u_int8_t tidno);

static int
ath_tx_send_normal(struct ath_softc *sc, struct ath_txq *txq, struct ath_atx_tid *tid, ath_bufhead *bf_head);

static INLINE int
ath_tx_send_ampdu(struct ath_softc *sc, struct ath_txq *txq, struct ath_atx_tid *tid,
                  ath_bufhead *bf_head, ieee80211_tx_control_t *txctl);

static void
ath_tx_complete_aggr_rifs(struct ath_softc *sc, struct ath_txq *txq, struct ath_buf *bf, ath_bufhead *bf_q, int txok);

static INLINE int
ath_tx_num_badfrms(struct ath_softc *sc, struct ath_buf *bf, int txok);

static void
ath_tx_complete_bar(struct ath_softc *sc, struct ath_buf *bf, ath_bufhead *bf_q, int txok);

void
ath_txq_schedule(struct ath_softc *sc, struct ath_txq *txq);

static void
ath_txq_drain_pending_buffers(struct ath_softc *sc, struct ath_txq *txq);

static INLINE void ath_tx_queue_tid(struct ath_txq *txq, struct ath_atx_tid *tid);
static void ath_tx_pause_tid(struct ath_softc *sc, struct ath_atx_tid *tid);
static void ath_tx_resume_tid(struct ath_softc *sc, struct ath_atx_tid *tid);
static void ath_tx_flush_tid(struct ath_softc *sc, struct ath_atx_tid *tid);

void ath_enq_legacy_buf(struct ath_softc *, struct ath_txq *, struct ath_buf *);
void ath_deq_legacy_buf(struct ath_softc *, struct ath_txq *);
void ath_drain_legacy_buf(struct ath_softc *, struct ath_txq *);
static OS_TIMER_FUNC(ath_tx_legacy);

#define IS_HT_RATE(_rate)     ((_rate) & 0x80)

#ifdef ATH_SUPPORT_IQUE
static u_int8_t min_qdepth_per_ac[WME_NUM_AC] = { 2, 2, 1, 1};
#endif

/*
 * Initialize TX queue and h/w
 */
int
ath_tx_init(ath_dev_t dev, int nbufs)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    int error = 0;

    do {
        ATH_TXBUF_LOCK_INIT(sc);

        /* Setup tx descriptors */
        error = ath_descdma_setup(sc, &sc->sc_txdma, &sc->sc_txbuf,
                                  "tx", nbufs * ATH_FRAG_PER_MSDU, ATH_TXDESC);
        if (error != 0) {
            printk("failed to allocate tx descriptors: %d\n", error);
            break;
        }

        /* XXX allocate beacon state together with vap */
        error = ath_descdma_setup(sc, &sc->sc_bdma, &sc->sc_bbuf,
                                  "beacon", ATH_BCBUF, 1);
        if (error != 0) {
            printk("failed to allocate beacon descripotrs: %d\n", error);
            break;
        }

#ifdef ATH_SUPPORT_UAPSD
        /* Initialize uapsd descriptors */
        error = ath_tx_uapsd_init(sc);
        if (error != 0) {
            printk("failed to allocate UAPSD descripotrs: %d\n", error);
            break;
        }
#endif
    } while (0);

    if (error != 0)
        ath_tx_cleanup(sc);

    OS_INIT_TIMER(sc->sc_osdev, &sc->sc_tx_leg, ath_tx_legacy, sc);
    OS_SET_TIMER(&sc->sc_tx_leg, 1000);

    return error;
}

/*
 * Reclaim all tx queue resources.
 */
int
ath_tx_cleanup(ath_dev_t dev)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);

    ATH_PS_WAKEUP(sc);

#ifdef ATH_SUPPORT_UAPSD
    /* cleanup uapsd descriptors */
    ath_tx_uapsd_cleanup(sc);
#endif

    /* cleanup beacon descriptors */
    if (sc->sc_bdma.dd_desc_len != 0)
        ath_descdma_cleanup(sc, &sc->sc_bdma, &sc->sc_bbuf);

    /* cleanup tx descriptors */
    if (sc->sc_txdma.dd_desc_len != 0)
        ath_descdma_cleanup(sc, &sc->sc_txdma, &sc->sc_txbuf);

    ATH_TXBUF_LOCK_DESTROY(sc);
    ATH_PS_SLEEP(sc);

    OS_CANCEL_TIMER(&sc->sc_tx_leg);

    return 0;
}

/*
 * Setup a h/w transmit queue.
 */
struct ath_txq *
ath_txq_setup(struct ath_softc *sc, int qtype, int subtype)
{
#define	N(a)	(sizeof(a)/sizeof(a[0]))
    struct ath_hal *ah = sc->sc_ah;
    HAL_TXQ_INFO qi;
    int qnum;
#ifdef ATH_SUPERG_COMP
    u_int compbufsz = 0;
    char *compbuf = NULL;
    dma_addr_t compbufp = 0;
    OS_DMA_MEM_CONTEXT(dmacontext)
#endif
    
    OS_MEMZERO(&qi, sizeof(qi));
    qi.tqi_subtype = subtype;
    qi.tqi_aifs = HAL_TXQ_USEDEFAULT;
    qi.tqi_cwmin = HAL_TXQ_USEDEFAULT;
    qi.tqi_cwmax = HAL_TXQ_USEDEFAULT;
    qi.tqi_compBuf = 0;
#ifdef ATH_SUPERG_XR
    if(subtype == HAL_XR_DATA) {
        qi.tqi_aifs  = XR_DATA_AIFS;
        qi.tqi_cwmin = XR_DATA_CWMIN;
        qi.tqi_cwmax = XR_DATA_CWMAX;
    }
#endif

#ifdef ATH_SUPERG_COMP
    /* allocate compression scratch buffer for data queues */
    if (((qtype == HAL_TX_QUEUE_DATA)|| (qtype == HAL_TX_QUEUE_UAPSD)) 
        && ath_hal_compressionsupported(ah)) {
        compbufsz = roundup(HAL_COMP_BUF_MAX_SIZE, 
                            HAL_COMP_BUF_ALIGN_SIZE) + HAL_COMP_BUF_ALIGN_SIZE;
        compbuf = (char *)OS_MALLOC_CONSISTENT(sc->sc_osdev,
                                               compbufsz, &compbufp,
                                               dmacontext, sc->sc_reg_parm.shMemAllocRetry);
        if (compbuf == NULL) {
            /* clear compression flag */
            sc->sc_hascompression = 0;
        } else {
            qi.tqi_compBuf = (u_int32_t)compbufp;
        }
    } 
#endif
    /*
     * Enable interrupts only for EOL and DESC conditions.
     * We mark tx descriptors to receive a DESC interrupt
     * when a tx queue gets deep; otherwise waiting for the
     * EOL to reap descriptors.  Note that this is done to
     * reduce interrupt load and this only defers reaping
     * descriptors, never transmitting frames.  Aside from
     * reducing interrupts this also permits more concurrency.
     * The only potential downside is if the tx queue backs
     * up in which case the top half of the kernel may backup
     * due to a lack of tx descriptors.
     *
     * The UAPSD queue is an exception, since we take a desc-
     * based intr on the EOSP frames.
     */
    if (qtype == HAL_TX_QUEUE_UAPSD)
        qi.tqi_qflags = TXQ_FLAG_TXDESCINT_ENABLE;
    else
        qi.tqi_qflags = TXQ_FLAG_TXEOLINT_ENABLE | TXQ_FLAG_TXDESCINT_ENABLE;
    qnum = ath_hal_setuptxqueue(ah, qtype, &qi);
    if (qnum == -1) {
        /*
         * NB: don't print a message, this happens
         * normally on parts with too few tx queues
         */
#ifdef ATH_SUPERG_COMP
        if (compbuf) {
            OS_FREE_CONSISTENT(sc->sc_osdev, compbufsz,
                               compbuf, compbufp, dmacontext);
        }
#endif
        return NULL;
    }
    if (qnum >= N(sc->sc_txq)) {
        printk("hal qnum %u out of range, max %u!\n",
               qnum, (unsigned int)N(sc->sc_txq));
#ifdef ATH_SUPERG_COMP
        if (compbuf) {
            OS_FREE_CONSISTENT(sc->sc_bdev, compbufsz,
                               compbuf, compbufp, dmacontext);
        }
#endif
        ath_hal_releasetxqueue(ah, qnum);
        return NULL;
    }
    if (!ATH_TXQ_SETUP(sc, qnum)) {
        struct ath_txq *txq = &sc->sc_txq[qnum];

        txq->axq_qnum = qnum;
        txq->axq_link = NULL;
        TAILQ_INIT(&txq->axq_q);
        TAILQ_INIT(&txq->axq_acq);
        ATH_TXQ_LOCK_INIT(txq);
        txq->axq_depth = 0;
        txq->axq_aggr_depth = 0;
        TAILQ_INIT(&txq->axq_ll);
        ATH_TXQ_LL_LOCK_INIT(txq);
        txq->axq_ll_count = 0;
        txq->axq_aggr_nbuf = 0;
        txq->axq_lim = (10 - subtype) * 5;
        txq->axq_totalqueued = 0;
        txq->axq_intrcnt = 0;
        txq->axq_linkbuf = NULL;
        //TAILQ_INIT(&txq->axq_stageq);
        sc->sc_txqsetup |= 1<<qnum;
#ifdef ATH_SUPERG_COMP
        txq->axq_compbuf = compbuf;
        txq->axq_compbufsz = compbufsz;
        txq->axq_compbufp = compbufp;
#ifdef ATH_SWRETRY
        txq->axq_destmask = AH_TRUE;
#endif
        OS_COPY_DMA_MEM_CONTEXT(OS_GET_DMA_MEM_CONTEXT(txq, axq_dmacontext),
                                dmacontext);
#endif
    }
    return &sc->sc_txq[qnum];
#undef N
}

/*
 * Reclaim resources for a setup queue.
 */
void
ath_tx_cleanupq(struct ath_softc *sc, struct ath_txq *txq)
{
#ifdef ATH_SUPERG_COMP
    /* Release compression buffer */
    if (txq->axq_compbuf) {
        OS_FREE_CONSISTENT(sc->sc_bdev, txq->axq_compbufsz,
                           txq->axq_compbuf, txq->axq_compbufp,
                           OS_GET_DMA_MEM_CONTEXT(txq, axq_dmacontext));
        txq->axq_compbuf = NULL;
    }
#endif
    ath_hal_releasetxqueue(sc->sc_ah, txq->axq_qnum);
    ATH_TXQ_LOCK_DESTROY(txq);
    sc->sc_txqsetup &= ~(1<<txq->axq_qnum);
}

/*
 * Setup a hardware data transmit queue for the specified
 * access control.  The hal may not support all requested
 * queues in which case it will return a reference to a
 * previously setup queue.  We record the mapping from ac's
 * to h/w queues for use by ath_tx_start and also track
 * the set of h/w queues being used to optimize work in the
 * transmit interrupt handler and related routines.
 */
int
ath_tx_setup(struct ath_softc *sc, int haltype)
{
#define	N(a)	(sizeof(a)/sizeof(a[0]))
    struct ath_txq *txq;

    if (haltype >= N(sc->sc_haltype2q)) {
        printk("HAL AC %u out of range, max %zu!\n",
               haltype, N(sc->sc_haltype2q));
        return 0;
    }
    txq = ath_txq_setup(sc, HAL_TX_QUEUE_DATA, haltype);
    if (txq != NULL) {
        sc->sc_haltype2q[haltype] = txq->axq_qnum;
        return 1;
    } else
        return 0;
#undef N
}

int
ath_tx_get_qnum(ath_dev_t dev, int qtype, int haltype)
{
#define	N(a)	(sizeof(a)/sizeof(a[0]))
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    int qnum;
    
    switch (qtype) {
    case HAL_TX_QUEUE_DATA:
        if (haltype >= N(sc->sc_haltype2q)) {
            printk("HAL AC %u out of range, max %zu!\n",
                   haltype, N(sc->sc_haltype2q));
            return -1;
        }
        qnum = sc->sc_haltype2q[haltype];
        break;
    case HAL_TX_QUEUE_BEACON:
        qnum = sc->sc_bhalq;
        break;
#ifdef ATH_SUPPORT_UAPSD
    case HAL_TX_QUEUE_UAPSD:
        qnum = sc->sc_uapsdq->axq_qnum;
        break;
#endif
    case HAL_TX_QUEUE_CAB:
        qnum = sc->sc_cabq->axq_qnum;
        break;
    default:
        qnum = -1;
    }
    return qnum;
#undef N
}

/*
 * Update parameters for a transmit queue.
 */
int
ath_txq_update(ath_dev_t dev, int qnum, HAL_TXQ_INFO *qi0)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;
    int error = 0;
    HAL_TXQ_INFO qi;

    if (qnum == sc->sc_bhalq) {
        /*
         * XXX: for beacon queue, we just save the parameter. It will be picked
         * up by ath_beaconq_config when it's necessary.
         */
        sc->sc_beacon_qi = *qi0;
        return 0;
    }
    
    ATH_PS_WAKEUP(sc);

    ASSERT(sc->sc_txq[qnum].axq_qnum == qnum);

    ath_hal_gettxqueueprops(ah, qnum, &qi);
    qi.tqi_aifs = qi0->tqi_aifs;
    qi.tqi_cwmin = qi0->tqi_cwmin;
    qi.tqi_cwmax = qi0->tqi_cwmax;
    qi.tqi_burstTime = qi0->tqi_burstTime;
    qi.tqi_readyTime = qi0->tqi_readyTime;

    if (!ath_hal_settxqueueprops(ah, qnum, &qi)) {
        printk("%s: unable to update hardware queue %u!\n",
               __func__, qnum);
        error = -EIO;
    } else {
        ath_hal_resettxqueue(ah, qnum); /* push to h/w */
    }

    ATH_PS_SLEEP(sc);
    return error;
}

int
ath_cabq_update(struct ath_softc *sc)
{
    HAL_TXQ_INFO qi;
    int qnum = sc->sc_cabq->axq_qnum;
    ieee80211_beacon_config_t conf;

    ath_hal_gettxqueueprops(sc->sc_ah, qnum, &qi);
    /*
     * Ensure the readytime % is within the bounds.
     */
    if (sc->sc_config.cabqReadytime < HAL_READY_TIME_LO_BOUND) {
        sc->sc_config.cabqReadytime = HAL_READY_TIME_LO_BOUND;
    } else if (sc->sc_config.cabqReadytime > HAL_READY_TIME_HI_BOUND) {
        sc->sc_config.cabqReadytime = HAL_READY_TIME_HI_BOUND;
    }

    sc->sc_ieee_ops->get_beacon_config(sc->sc_ieee, ATH_IF_ID_ANY, &conf);
    qi.tqi_readyTime = (conf.beacon_interval * sc->sc_config.cabqReadytime)/100;
    ath_txq_update(sc, qnum, &qi);

    return 0;
}

void
ath_set_protmode(ath_dev_t dev, PROT_MODE mode)
{
    ATH_DEV_TO_SC(dev)->sc_protmode = mode;
}

/*
 * Insert a chain of ath_buf (descriptors) on a multicast txq
 * but do NOT start tx DMA on this queue.
 * NB: must be called with txq lock held
 */
static INLINE void
ath_tx_mcastqaddbuf(struct ath_softc *sc, struct ath_txq *txq, ath_bufhead *head)
{
    struct ath_hal *ah = sc->sc_ah;
    struct ath_buf *bf;

    /*
     * Insert the frame on the outbound list and
     * pass it on to the hardware.
     */
    bf = TAILQ_FIRST(head);
    if (bf == NULL)
        return;

    /*
     * The CAB queue is started from the SWBA handler since
     * frames only go out on DTIM and to avoid possible races.
     */
    ath_hal_intrset(ah, 0);
    
    /*
    ** If there is anything in the mcastq, we want to set the "more data" bit
    ** in the last item in the queue to indicate that there is "more data".  This
    ** is an alternate implementation of changelist 289513 put within the code
    ** to add to the mcast queue.  It makes sense to add it here since you are
    ** *always* going to have more data when adding to this queue, no matter where
    ** you call from.
    */
    
    if (txq->axq_depth) {
        struct ath_buf *lbf;
        struct ieee80211_frame  *wh;
        
        /*
        ** Add the "more data flag" to the last frame
        */
        
        lbf = TAILQ_LAST(&txq->axq_q,ath_bufhead_s);
        wh = (struct ieee80211_frame *)wbuf_header(lbf->bf_mpdu);
        wh->i_fc[1] |= IEEE80211_FC1_MORE_DATA;
    }
    
    /*
    ** Now, concat the frame onto the queue
    */

    ATH_TXQ_CONCAT(txq, head);
    DPRINTF(sc, ATH_DEBUG_TX_PROC, "%s: txq depth = %d\n", __func__, txq->axq_depth);
    if (txq->axq_link != NULL) {
#ifdef AH_NEED_DESC_SWAP
        *txq->axq_link = cpu_to_le32(bf->bf_daddr);
#else
        *txq->axq_link = bf->bf_daddr;
#endif
        DPRINTF(sc, ATH_DEBUG_XMIT, "%s: link[%u](%p)=%llx (%p)\n",
                __func__,
                txq->axq_qnum, txq->axq_link,
                ito64(bf->bf_daddr), bf->bf_desc);
    }
    txq->axq_link = &(bf->bf_lastbf->bf_desc->ds_link);
    ath_hal_intrset(ah, sc->sc_imask);

    //sc->sc_devstats.tx_packets++;
    //sc->sc_devstats.tx_bytes += framelen;
}

/*
 * Insert a chain of ath_buf (descriptors) on a txq and
 * assume the descriptors are already chained together by caller.
 * NB: must be called with txq lock held
 */
void
ath_tx_txqaddbuf(struct ath_softc *sc, struct ath_txq *txq, ath_bufhead *head)
{
    struct ath_hal *ah = sc->sc_ah;
    struct ath_buf *bf;
#ifdef ATH_SWRETRY
    struct ath_node *an;
    struct ieee80211_frame *wh;
#endif

    /*
     * Insert the frame on the outbound list and
     * pass it on to the hardware.
     */

    bf = TAILQ_FIRST(head);
    if (bf == NULL)
        return;

#ifdef ATH_SWRETRY
    an = ATH_NODE(bf->bf_node);
    wh = (struct ieee80211_frame *)wbuf_header(bf->bf_mpdu);

    if (an && an->an_total_swrtx_pendfrms && !(bf->bf_status & ATH_BUFSTATUS_MARKEDSWRETRY)) {
        ATH_NODE_SWRETRY_TXBUF_LOCK(an);
        TAILQ_CONCAT(&(an->an_softxmit_q), head, bf_list);
        an->an_softxmit_qdepth++;
        DPRINTF(sc, ATH_DEBUG_SWR, "%s: Queuing frm with SeqNo%d to SxQ: SxQdepth %d pendfrms %d\n",__func__, 
                    (*(u_int16_t *)&wh->i_seq[0]) >> 4, an->an_softxmit_qdepth, an->an_total_swrtx_pendfrms);
        ATH_NODE_SWRETRY_TXBUF_UNLOCK(an);

        return;
    }

    if (bf->bf_isswretry)
        DPRINTF(sc, ATH_DEBUG_SWR, "%s: Queuing frm with SeqNo %d to HwQ\n",__func__, (*(u_int16_t *)&wh->i_seq[0]) >> 4);
#endif

    ATH_TXQ_CONCAT(txq, head);
    DPRINTF(sc, ATH_DEBUG_TX_PROC, "%s: txq depth = %d\n", __func__, txq->axq_depth);

    if (txq->axq_link == NULL) {
        ath_hal_puttxbuf(ah, txq->axq_qnum, bf->bf_daddr);
        DPRINTF(sc, ATH_DEBUG_XMIT, "%s: TXDP[%u] = %llx (%p)\n",
                __func__, txq->axq_qnum, ito64(bf->bf_daddr), bf->bf_desc);
    } else {
#ifdef AH_NEED_DESC_SWAP
        *txq->axq_link = cpu_to_le32(bf->bf_daddr);
#else
        *txq->axq_link = bf->bf_daddr;
#endif
#ifdef CAVIUM_FLUSH_CACHE
          asm volatile ("syncw\nsyncw\n":::"memory");
#endif
        DPRINTF(sc, ATH_DEBUG_XMIT, "%s: link[%u] (%p)=%llx (%p)\n",
                __func__,
                txq->axq_qnum, txq->axq_link,
                ito64(bf->bf_daddr), bf->bf_desc);
    }
    txq->axq_link = &(bf->bf_lastbf->bf_desc->ds_link);
    ath_hal_txstart(ah, txq->axq_qnum);
    // sc->sc_dev->trans_start = jiffies;

    //sc->sc_devstats.tx_packets++;
    //sc->sc_devstats.tx_bytes += framelen;
}

/*
 * Get transmit rate index using rate in Kbps
 */
static INLINE int
ath_tx_findindex(const HAL_RATE_TABLE *rt, int rate)
{
    int i;
    int ndx = 0;

    for (i = 0; i < rt->rateCount; i++) {
        if (rt->info[i].rateKbps == rate) {
            ndx = i;
            break;
        }
    }

    return ndx;
}

/*
 * Check if it's okay to send out aggregates
 */
static INLINE int
ath_aggr_query(struct ath_softc *sc, struct ath_node *an, u_int8_t tidno)
{
    ath_atx_tid_t *tid;
    tid = ATH_AN_2_TID(an, tidno);

    if (tid->addba_exchangecomplete || tid->addba_exchangeinprogress)
        return 1;
    else
        return 0;
}

/*
 * This function will setup additional txctl information, mostly rate stuff
 */
static INLINE int
__ath_tx_prepare(struct ath_softc *sc, wbuf_t wbuf, ieee80211_tx_control_t *txctl)
{
    struct ath_node *an;
    u_int8_t rix;
#ifdef ATH_SUPERG_COMP
    int comp = ATH_COMP_PROC_NO_COMP_NO_CCS;
#endif
    struct ath_txq *txq = NULL;
    struct ieee80211_frame *wh;
    const HAL_RATE_TABLE *rt;
    u_int8_t antenna;
    struct ath_rc_series *rcs;
    int subtype;

    txctl->dev = sc;

    wh = (struct ieee80211_frame *)wbuf_header(wbuf);

#ifdef ATH_SUPERG_XR
    if (ieee80211vap_has_flags(vap, IEEE80211_F_XR))
        rt = sc->sc_xr_rates;
    else
        rt = sc->sc_currates;
#else
    rt = sc->sc_currates;
#endif
    KASSERT(rt != NULL, ("no rate table, mode %u", sc->sc_curmode));

    an = txctl->an;
    txq = &sc->sc_txq[txctl->qnum];

    /*
     * Setup for rate calculations.
     */
    rcs = (struct ath_rc_series *)&txctl->priv[0];
    OS_MEMZERO(rcs, sizeof(struct ath_rc_series) * 4);

    if (txctl->isdata && !txctl->use_minrate) {
        if (txctl->ismcast) {
            rcs[0].rix = (u_int8_t)ath_tx_findindex(rt, txctl->mcast_rate);

            /*
             * mcast packets are not re-tried.
             */
            rcs[0].tries = 1;
        } else {
            /*
             * For aggregation enabled nodes there is no need to do rate find
             * on each of these frames.
             */
            if (
#ifdef ATH_RIFS
                !txctl->ht || (!sc->sc_txaggr && !sc->sc_txrifs) ||
#else
                !txctl->ht || !sc->sc_txaggr ||
#endif
                !ath_aggr_query(sc, an, wbuf_get_tid(wbuf)))
            {

                int isProbe;
                /*
                 * Data frames; consult the rate control module.
                 */
                sc->sc_log_rcfind = 1;
                ath_rate_findrate(sc, an, txctl->shortPreamble, txctl->frmlen,
                              ATH_11N_TXMAXTRY, ATH_RC_PROBE_ALLOWED,
                              TID_TO_WME_AC(wbuf_get_tid(wbuf)),
                              rcs, &isProbe, AH_FALSE);

                /* Ratecontrol sometimes returns invalid rate index */
                if (rcs[0].rix != 0xff)
                    an->an_prevdatarix = rcs[0].rix;
                else {
                    rcs[0].rix = an->an_prevdatarix;
                }
            } else {
                /*
                 * For HT capable stations, we save tidno for later use.
                 * We also override seqno set by upper layer with the one
                 * in tx aggregation state.
                 *
                 * First, the fragmentation stat is determined.  If fragmentation
                 * is on, the sequence number is not overridden, since it has been
                 * incremented by the fragmentation routine.
                 */
#ifdef ATH_SUPPORT_IQUE
				/* If this frame is a HBR (headline block removal) probing QoSNull frame,
				 * it should be sent at the min rate which is cached in ath_node->an_minRate[ac]
				 */
				if (M_FLAG_GET((struct sk_buff *)wbuf, M_PROBING)) {
					int isProbe;	
					int ac = TID_TO_WME_AC(wbuf_get_tid(wbuf));
			        ath_rate_findrate(sc, an, AH_FALSE, txctl->frmlen,
			             1, 0, ac, rcs, &isProbe, AH_FALSE);
					rcs[0].tries = 1;
					rcs[1].tries = 0;
					rcs[2].tries = 0;
					rcs[3].tries = 0;
				}
#endif

				if (likely(!(txctl->flags & HAL_TXDESC_FRAG_IS_ON))) {
                    struct ath_atx_tid *tid;

                    txctl->tidno = wbuf_get_tid(wbuf);
                    tid = ATH_AN_2_TID(an, txctl->tidno);
                    ATH_TXQ_LOCK(txq);
                    *(u_int16_t *)wh->i_seq = htole16(tid->seq_next << IEEE80211_SEQ_SEQ_SHIFT);
                    txctl->seqno = tid->seq_next;
                    INCR(tid->seq_next, IEEE80211_SEQ_MAX);
                    ATH_TXQ_UNLOCK(txq);
                }
            }
        }
    } else {
        /* for management and control frames, or for NULL and EAPOL frames */
        if (txctl->min_rate != 0)
            rcs[0].rix = ath_rate_findrateix(sc, an, txctl->min_rate);
        else
            rcs[0].rix = sc->sc_minrateix;
        rcs[0].tries = ATH_MGT_TXMAXTRY;
    }
    rix = rcs[0].rix;

    /*
     * Calculate duration.  This logically belongs in the 802.11
     * layer but it lacks sufficient information to calculate it.
     */
    if ((txctl->flags & HAL_TXDESC_NOACK) == 0 &&
        (wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) != IEEE80211_FC0_TYPE_CTL) {
        u_int16_t dur;
        /*
         * XXX not right with fragmentation.
         */
        if (txctl->shortPreamble)
            dur = rt->info[rix].spAckDuration;
        else
            dur = rt->info[rix].lpAckDuration;

        if (wh->i_fc[1] & IEEE80211_FC1_MORE_FRAG) {
            dur += dur;  /* Add additional 'SIFS + ACK' */

            /*
            ** Compute size of next fragment in order to compute
            ** durations needed to update NAV.
            ** The last fragment uses the ACK duration only.
            ** Add time for next fragment.
            */
            dur += ath_hal_computetxtime(sc->sc_ah, rt, txctl->nextfraglen, 
                                         rix, txctl->shortPreamble);
        }

        if (txctl->istxfrag) {
            /*
            **  Force hardware to use computed duration for next
            **  fragment by disabling multi-rate retry, which
            **  updates duration based on the multi-rate
            **  duration table.
            */
            rcs[1].tries = rcs[2].tries = rcs[3].tries = 0;
            rcs[1].rix = rcs[2].rix = rcs[3].rix = 0;
            rcs[0].tries = ATH_TXMAXTRY; /* reset tries but keep rate index */
        }

        *(u_int16_t *)wh->i_dur = cpu_to_le16(dur);
    }

    /*
     * Determine if a tx interrupt should be generated for
     * this descriptor.  We take a tx interrupt to reap
     * descriptors when the h/w hits an EOL condition or
     * when the descriptor is specifically marked to generate
     * an interrupt.  We periodically mark descriptors in this
     * way to insure timely replenishing of the supply needed
     * for sending frames.  Defering interrupts reduces system
     * load and potentially allows more concurrent work to be
     * done but if done to aggressively can cause senders to
     * backup.
     *
     * NB: use >= to deal with sc_txintrperiod changing
     *     dynamically through sysctl.
     */
    ATH_TXQ_LOCK(txq);
    if (
#ifdef ATH_SUPPORT_UAPSD
       (!txctl->isuapsd) &&
#endif
       (++txq->axq_intrcnt >= sc->sc_txintrperiod)) {
        txctl->flags |= HAL_TXDESC_INTREQ;
        txq->axq_intrcnt = 0;
    }
    ATH_TXQ_UNLOCK(txq);

#ifdef ATH_SUPERG_COMP
    if (ATH_NODE(ni)->an_decomp_index != INVALID_DECOMP_INDEX && 
        !ismcast &&
        ((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) == IEEE80211_FC0_TYPE_DATA)
        && ((wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK) 
            != IEEE80211_FC0_SUBTYPE_NODATA)) {
        if (txctl->frmlen > ATH_COMP_THRESHOLD) {
            comp = ATH_COMP_PROC_COMP_OPTIMAL;
        } else {
            comp = ATH_COMP_PROC_NO_COMP_ADD_CCS;
        }
    } 
#endif

    if (txctl->ismcast) {
        antenna = sc->sc_mcastantenna + 1;
        sc->sc_mcastantenna = (sc->sc_mcastantenna + 1) & 0x1;
    } else
        antenna = sc->sc_txantenna;

#ifdef USE_LEGACY_HAL
    txctl->antenna = antenna;
    txctl->compression = comp;
#endif

#ifdef ATH_SWRETRY            
    /* Management frames will not go for SW Retry
     * process unless they are failed with filtered
     * error.
     */
    if (!(an) || (an && !(an->an_swrenabled))) {
        txctl->flags |= HAL_TXDESC_CLRDMASK;
    } else {
        ATH_TXQ_LOCK(txq);
        if (txq->axq_destmask) {
            txctl->flags |= HAL_TXDESC_CLRDMASK;
            if (txctl->isdata) {
                txq->axq_destmask = AH_FALSE; /*Turn-off destmask only for subsequent data frames*/
            }   
        }
        ATH_TXQ_UNLOCK(txq);
    }
#endif
    
    /* report LED module about byte count */
    subtype = wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK;
    if (txctl->isdata &&
        subtype != IEEE80211_FC0_SUBTYPE_NODATA &&
        subtype != IEEE80211_FC0_SUBTYPE_QOS_NULL)
        ath_led_report_data_flow(&sc->sc_led_control, txctl->frmlen);
   
    /*
     * XXX: Update some stats ???
     */
    if (txctl->shortPreamble)
        sc->sc_stats.ast_tx_shortpre++;
    if (txctl->flags & HAL_TXDESC_NOACK)
        sc->sc_stats.ast_tx_noack++;
    
    return 0;
}

int
ath_tx_start(ath_dev_t dev, wbuf_t wbuf, ieee80211_tx_control_t *txctl)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    int error = 0;

    error = __ath_tx_prepare(sc, wbuf, txctl);
    if (error == 0) {
        /*
         * Start DMA mapping.
         * ath_tx_start_dma() will be called either synchronously
         * or asynchrounsly once DMA is complete.
         */
        wbuf_map_sg(sc->sc_osdev, wbuf,
                    OS_GET_DMA_MEM_CONTEXT(txctl, dmacontext),
                    txctl);
    }
    /* failed packets will be dropped by the caller */
    return error;
}

/*
 * The function that actually starts the DMA.
 * It will either be called by the wbuf_map() function,
 * or called in a different thread if asynchronus DMA
 * mapping is used (NDIS 6.0).
 */
int
ath_tx_start_dma(wbuf_t wbuf, sg_t *sg, u_int32_t n_sg, void *arg)
{
    ieee80211_tx_control_t *txctl = (ieee80211_tx_control_t *)arg;
    struct ath_softc *sc = (struct ath_softc *)txctl->dev;
    struct ath_node *an = txctl->an;
    struct ath_buf *bf = NULL, *firstbf=NULL;
    ath_bufhead bf_head;
    struct ath_desc *ds, *firstds = NULL, *lastds = NULL;
    struct ath_hal *ah = sc->sc_ah;
    struct ath_txq *txq = &sc->sc_txq[txctl->qnum];
    size_t i;
    struct ath_rc_series *rcs;

   

    if(txctl->iseapol)
    {
	    txq = &sc->sc_txq[txctl->iseapol];
    }

    /* For each sglist entry, allocate an ath_buf for DMA */
    

    TAILQ_INIT(&bf_head);
    for (i = 0; i < n_sg; i++, sg++) {
        ATH_TXBUF_LOCK(sc);
        bf = TAILQ_FIRST(&sc->sc_txbuf);
        if (bf == NULL) {
            ATH_TXBUF_UNLOCK(sc);
            if (txctl->isbar) {
                struct ath_atx_tid *tid = ATH_AN_2_TID(an, txctl->tidno);
                ath_tx_resume_tid(sc, tid);
            }
            goto bad;
        }
        TAILQ_REMOVE(&sc->sc_txbuf, bf, bf_list);
        ATH_TXBUF_UNLOCK(sc);

        TAILQ_INSERT_TAIL(&bf_head, bf, bf_list);

        /* set up this buffer */
        ATH_TXBUF_RESET(bf);
#ifdef ATH_SWRETRY
        ATH_TXBUF_SWRETRY_RESET(bf);
#endif
        bf->bf_frmlen = txctl->frmlen;
        bf->bf_isdata = txctl->isdata;
        bf->bf_isbar = txctl->isbar;
        bf->bf_ispspoll = txctl->ispspoll;
        bf->bf_calcairtime = txctl->calcairtime;
        bf->bf_flags = txctl->flags;
        bf->bf_shpreamble = txctl->shortPreamble;
        bf->bf_keytype = txctl->keytype;
		bf->bf_tidno = txctl->tidno;
#ifdef ATH_SWRETRY
        bf->bf_qnum = txctl->qnum;
#endif
        rcs = (struct ath_rc_series *)&txctl->priv[0];
        bf->bf_rcs[0] = rcs[0];
        bf->bf_rcs[1] = rcs[1];
        bf->bf_rcs[2] = rcs[2];
        bf->bf_rcs[3] = rcs[3];
        bf->bf_node = an;
        bf->bf_mpdu = wbuf;
        bf->bf_buf_addr = sg_dma_address(sg);

#ifdef ATH_SUPPORT_IQUE
		if(!bf->bf_isretried) {
			bf->bf_txduration = 0;
		}
#endif
        /* setup descriptor */
        ds = bf->bf_desc;
        ds->ds_link = 0;
        ds->ds_data = bf->bf_buf_addr;
#ifndef REMOVE_PKT_LOG
        ds->ds_vdata = wbuf_header(wbuf);
#endif

        if (i == 0) {
            /*
             * Save the DMA context in the first ath_buf
             */
            OS_COPY_DMA_MEM_CONTEXT(OS_GET_DMA_MEM_CONTEXT(bf, bf_dmacontext),
                                    OS_GET_DMA_MEM_CONTEXT(txctl, dmacontext));

            /*
             * Formulate first tx descriptor with tx controls.
             */
            ath_hal_set11n_txdesc(ah, ds
                                  , bf->bf_frmlen           /* frame length */
                                  , txctl->atype            /* Atheros packet type */
                                  , MIN(txctl->txpower, 60) /* txpower */
                                  , txctl->keyix            /* key cache index */
                                  , txctl->keytype          /* key type */
                                  , txctl->flags            /* flags */
                );

            firstds = ds;
            firstbf = bf;

            ath_hal_filltxdesc(ah, ds
                               , sg_dma_len(sg)     /* segment length */
                               , AH_TRUE            /* first segment */
                               , (n_sg == 1) ? AH_TRUE : AH_FALSE /* last segment */
                               , ds                 /* first descriptor */
                );
        } else {
            /* chain descriptor together */
            lastds->ds_link = bf->bf_daddr;

            ath_hal_filltxdesc(ah, ds
                               , sg_dma_len(sg)     /* segment length */
                               , AH_FALSE           /* first segment */
                               , (i == n_sg-1) ? AH_TRUE : AH_FALSE /* last segment */
                               , firstds           /* first descriptor */
                );
        }

        /* NB: The desc swap function becomes void, 
         * if descriptor swapping is not enabled
         */
        ath_desc_swap(ds);

        lastds = ds;
    }

    if (firstbf) {
        firstbf->bf_lastfrm = bf;
        firstbf->bf_ht = txctl->ht;

#ifdef ATH_SUPPORT_UAPSD
        if (txctl->isuapsd) {
            ath_tx_queue_uapsd(sc, txq, &bf_head, txctl);
            return 0;
        }
#endif
        ATH_TXQ_LOCK(txq);

#ifdef ATH_RIFS
        if (txctl->ht && (sc->sc_txaggr || sc->sc_txrifs)) {
#else
        if (txctl->ht && sc->sc_txaggr) {
#endif
            struct ath_atx_tid *tid = ATH_AN_2_TID(an, txctl->tidno);
            if (ath_aggr_query(sc, an, txctl->tidno)) {
                /*
                 * Try aggregation if it's a unicast data frame
                 * and the destination is HT capable.
                 */
                ath_tx_send_ampdu(sc, txq, tid, &bf_head, txctl);
            } else {
                /*
                 * Send this frame as regular when ADDBA exchange
                 * is neither complete nor pending.
                 */
                ath_tx_send_normal(sc, txq, tid, &bf_head);
            }
#if defined(ATH_ADDITIONAL_STATS) || defined(ATH_SUPPORT_IQUE)
            sc->sc_stats.ast_txq_packets[txq->axq_qnum]++;
#endif
        } else {
            firstbf->bf_lastbf = bf;
            firstbf->bf_nframes = 1;
            ath_buf_set_rate(sc, firstbf);

            if (txctl->isbar) {
                /* This is required for resuming tid during BAR completion */
                firstbf->bf_tidno = wbuf_get_tid(wbuf);
            }

            if (txctl->ismcast) {
                struct ath_vap *avp = sc->sc_vaps[txctl->if_id];

                /*
                 * When servicing one or more stations in power-save mode (or)
                 * if there is some mcast data waiting on mcast queue
                 * (to prevent out of order delivery of mcast,bcast packets)
                 * multicast frames must be buffered until after the beacon.
                 * We use the private mcast queue for that.
                 */
                /* XXX? more bit in 802.11 frame header */
                if (txctl->ps || avp->av_mcastq.axq_depth)
                    ath_tx_mcastqaddbuf(sc, &avp->av_mcastq, &bf_head);
                else
                    ath_tx_txqaddbuf(sc, txq, &bf_head);
            } else {
                if (txq->axq_aggr_nbuf) {
                    ath_enq_legacy_buf(sc, txq, TAILQ_FIRST(&bf_head));
                } else {
                    ath_tx_txqaddbuf(sc, txq, &bf_head);
                }
            }
        }

        ATH_TXQ_UNLOCK(txq);
        return 0;
    }

bad:
    /*
     * XXX: In other OS's, we can probably drop the frame. But in de-serialized
     * windows driver (NDIS6.0), we're not allowd to tail drop frame when out
     * of resources. So we just return NOMEM here and let OS shim to do whatever
     * OS wants.
     */
    ATH_TXBUF_LOCK(sc);
    TAILQ_CONCAT(&sc->sc_txbuf, &bf_head, bf_list);
    ATH_TXBUF_UNLOCK(sc);

    sc->sc_stats.ast_tx_nobuf++;
    sc->sc_stats.ast_txq_nobuf[txctl->qnum]++;

    return -ENOMEM;
}

#ifdef debug_wds_endurance
void
ath_dump_mcastbuf(struct ath_softc *sc)
{
    struct ath_vap      *avp = sc->sc_vaps[0];
    struct ath_buf      *buf;

    printk("\nMulticast Q: ");
    TAILQ_FOREACH(buf, &avp->av_mcastq.axq_q, bf_list)
        printk("%p ", buf);
    printk("\n");
}

void
ath_dump_buf_state(struct ath_buf *buf)
{
    printk("%p \n", buf);
    printk("buf flags - 0x%08x --------- ", buf->bf_flags);
    printk("buf status - 0x%08x\n", buf->bf_status);
    printk("# frames in aggr - %d, length of aggregate - %d, length of frame - %d, sequence number - %d, tidno - %d\n", 
        buf->bf_state.bfs_nframes, buf->bf_state.bfs_al, buf->bf_state.bfs_frmlen, buf->bf_state.bfs_seqno, buf->bf_state.bfs_tidno);
    printk("isdata: %d isaggr: %d isampdu: %d ht: %d isretried: %d isxretried: %d shpreamble: %d isbar: %d ispspoll: %d aggrburst: %d calcairtime: %d qosnulleosp: %d\n",
		buf->bf_state.bfs_isdata,
		buf->bf_state.bfs_isaggr,
		buf->bf_state.bfs_isampdu,
		buf->bf_state.bfs_ht,
		buf->bf_state.bfs_isretried,
		buf->bf_state.bfs_isxretried,
		buf->bf_state.bfs_shpreamble,
		buf->bf_state.bfs_isbar,
		buf->bf_state.bfs_ispspoll,
		buf->bf_state.bfs_aggrburst,
		buf->bf_state.bfs_calcairtime,
		buf->bf_state.bfs_qosnulleosp);
}

void
ath_dump_tx_desc(struct ath_desc *ds)
{
    u_int32_t   *p = (u_int32_t *)ds;

    printk("%p: 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n",
        p, p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9]);
    printk("0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n",
        p[10], p[11], p[12], p[13], p[14], p[15], p[16], p[17], p[18], p[19]);
    printk("0x%08x 0x%08x 0x%08x 0x%08x\n",
        p[20], p[21], p[22], p[23]);
}

void
ath_dump_txbufs(struct ath_softc *sc)
{
    int                 j, i;
    struct ath_atx_ac   *ac;
    struct ath_atx_tid  *tid;
    struct ath_txq      *txq;
    struct ath_buf      *buf;

    for (i = 0; i < HAL_NUM_TX_QUEUES; i++) {
        if (ATH_TXQ_SETUP(sc, i)) {
            txq = &sc->sc_txq[i];
            printk("\nsc_txq[%d] : ", i);
            TAILQ_FOREACH(ac, &txq->axq_acq, ac_qelem) {
                TAILQ_FOREACH(tid, &ac->tid_q, tid_qelem) {
                    printk("tid %p pause %d : ", tid, tid->paused);
                    for (j = 0;j < ATH_TID_MAX_BUFS; j++) {
                        if (tid->tx_buf[j]) {
                            printk("%d: %p ", j, tid->tx_buf[j]);
                        }
                    }
                    TAILQ_FOREACH(buf, &tid->buf_q, bf_list) {
                        printk("%p ", buf);
                    }
                    printk("\n");
                }
            }
            printk("axq_q:\n");
            TAILQ_FOREACH(buf, &txq->axq_q, bf_list) {
                ath_dump_buf_state(buf);
                ath_dump_tx_desc(buf->bf_desc);
            }
            printk("\n");
        }
    }
}
#endif

/*
 * To complete a chain of buffers associated a frame
 */
void
ath_tx_complete_buf(struct ath_softc *sc, struct ath_buf *bf, ath_bufhead *bf_q, int txok)
{
    wbuf_t wbuf = bf->bf_mpdu;
    ieee80211_tx_status_t tx_status;

    /*
     * Set retry information.
     * NB: Don't use the information in the descriptor, because the frame
     * could be software retried.
     */
#ifdef ATH_SWRETRY
    if (bf->bf_isswretry)
        tx_status.retries = bf->bf_totaltries;
    else
        tx_status.retries = bf->bf_retries;
#else
    tx_status.retries = bf->bf_retries;
#endif
    tx_status.flags = 0;

    if (txok) {
        /* increment count of transmitted bytes */
        sc->sc_stats.ast_tx_bytes += bf->bf_frmlen;
    }
    else {
        tx_status.flags |= ATH_TX_ERROR;

        if (bf->bf_isxretried) {
            tx_status.flags |= ATH_TX_XRETRY;
        }
    }
    /* Unmap this frame */
    wbuf_unmap_sg(sc->sc_osdev, wbuf,
                  OS_GET_DMA_MEM_CONTEXT(bf, bf_dmacontext));

    sc->sc_ieee_ops->tx_complete(wbuf, &tx_status); /* complete this frame */

    /*
     * Return the list of ath_buf of this mpdu to free queue
     */
    ATH_TXBUF_LOCK(sc);
    TAILQ_CONCAT(&sc->sc_txbuf, bf_q, bf_list);
    ATH_TXBUF_UNLOCK(sc);
}

/*
 * Process completed xmit descriptors from the specified queue.
 */
static int
ath_tx_processq(struct ath_softc *sc, struct ath_txq *txq)
{
    struct ath_hal *ah = sc->sc_ah;
    struct ath_buf *bf, *lastbf, *bf_held = NULL;
    ath_bufhead bf_head;
    struct ath_desc *ds;
    struct ath_node *an;
    int sr, lr;
    HAL_STATUS status;
    int uapsdq = 0, nacked;
    u_int8_t rateCode;
    int txok, nbad = 0;
    int isrifs = 0;
#ifdef ATH_SWRETRY    
    struct ath_swretry_info *pInfo;
    struct ieee80211_frame  *wh;
    struct ath_buf *lfbf = NULL;
#endif
    DPRINTF(sc, ATH_DEBUG_TX_PROC, "%s: tx queue %d (%x), link %p\n", __func__,
            txq->axq_qnum, ath_hal_gettxbuf(sc->sc_ah, txq->axq_qnum),
            txq->axq_link);

    if (txq == sc->sc_uapsdq) {
        DPRINTF(sc, ATH_DEBUG_UAPSD, "%s: reaping U-APSD txq\n", __func__);
        uapsdq = 1;
    }

    nacked = 0;
    for (;;) {
        ATH_TXQ_LOCK(txq);
        txq->axq_intrcnt = 0; /* reset periodic desc intr count */
        bf = TAILQ_FIRST(&txq->axq_q);
        if (bf == NULL) {
            txq->axq_link = NULL;
            txq->axq_linkbuf = NULL;
            ATH_TXQ_UNLOCK(txq);
            break;
        }

        /*
         * There is a race condition that DPC gets scheduled after sw writes TxE
         * and before hw re-load the last descriptor to get the newly chained one.
         * Software must keep the last DONE descriptor as a holding descriptor -
         * software does so by marking it with the STALE flag.
         */
        bf_held = NULL;
        if (bf->bf_status & ATH_BUFSTATUS_STALE) {
            bf_held = bf;
            bf = TAILQ_NEXT(bf_held, bf_list);
            if (bf == NULL) {
#ifdef ATH_SWRETRY
                if (sc->sc_swRetryEnabled)
                    txq->axq_destmask = AH_TRUE;
#endif

#ifdef notyet
                /*
                 * The holding descriptor is the last descriptor in queue.
                 * It's safe to remove the last holding descriptor in DPC context.
                 */
                ATH_TXQ_REMOVE_STALE_HEAD(txq, bf_held, bf_list);
                txq->axq_link = NULL;
                txq->axq_linkbuf = NULL;
                ATH_TXQ_UNLOCK(txq);

                ATH_TXBUF_LOCK(sc);
                TAILQ_INSERT_TAIL(&sc->sc_txbuf, bf_held, bf_list);
                ATH_TXBUF_UNLOCK(sc);
#else
                ATH_TXQ_UNLOCK(txq);
#endif
                break;
            }
        }

        isrifs = (ATH_RIFS_SUBFRAME_FIRST == bf->bf_rifsburst_elem) ? 1 : 0;
        lastbf = bf->bf_lastbf;
        ds = lastbf->bf_desc;    /* NB: last decriptor */

        status = ath_hal_txprocdesc(ah, ds);
#ifdef AR_DEBUG
        if (sc->sc_debug & ATH_DEBUG_XMIT_DESC)
            ath_printtxbuf(bf, status == HAL_OK);
#endif
        if (status == HAL_EINPROGRESS) {
            ATH_TXQ_UNLOCK(txq);
            break;
        }
        if (bf->bf_desc == txq->axq_lastdsWithCTS) {
            txq->axq_lastdsWithCTS = NULL;
        }
        if (ds == txq->axq_gatingds) {
            txq->axq_gatingds = NULL;
        }

        /*
         * Remove ath_buf's of the same transmit unit from txq,
         * however leave the last descriptor back as the holding
         * descriptor for hw.
         */
        lastbf->bf_status |= ATH_BUFSTATUS_STALE;
        ATH_TXQ_MOVE_HEAD_BEFORE(txq, &bf_head, lastbf, bf_list);

        if (bf->bf_isaggr) {
            txq->axq_aggr_depth--;
        }
        if (bf->bf_isn) {
            txq->axq_aggr_nbuf -= bf->bf_nframes;
        }
        if (!txq->axq_aggr_nbuf) {
            OS_SET_TIMER(&sc->sc_tx_leg, 1);
        }
        txok = (ds->ds_txstat.ts_status == 0);

#ifdef ATH_SWRETRY
        if (txok || (ath_check_swretry_req(sc, bf) == AH_FALSE)) {
            /* Change the status of the frame and complete
             * this frame as normal frame
             */
            bf->bf_status &= ~ATH_BUFSTATUS_MARKEDSWRETRY;

        } else {
            /* This frame is going through SW retry mechanism
             */    
            bf->bf_status |= ATH_BUFSTATUS_MARKEDSWRETRY;

            bf = ath_form_swretry_frm(sc, txq, &bf_head, bf);
            /* Here bf will be changed only when there is single
             * buffer for the current frame. 
             */
            lastbf = bf->bf_lastfrm;
            ds = lastbf->bf_desc;    
        }
#endif
        ATH_TXQ_UNLOCK(txq);

        /* Put the old holding descriptor to the free queue */
        if (bf_held) {
            TAILQ_REMOVE(&bf_head, bf_held, bf_list);
#ifdef ATH_SUPPORT_UAPSD
            if (bf_held->bf_qosnulleosp) {
                ATH_UAPSD_LOCK_IRQ(sc);
                TAILQ_INSERT_TAIL(&sc->sc_uapsdqnulbf, bf_held, bf_list);
                sc->sc_uapsdqnuldepth--;
                ATH_UAPSD_UNLOCK_IRQ(sc);
            } else
#endif
            {
                ATH_TXBUF_LOCK(sc);
                TAILQ_INSERT_TAIL(&sc->sc_txbuf, bf_held, bf_list);
                ATH_TXBUF_UNLOCK(sc);
            }
        }

        an = bf->bf_node;
        if (an != NULL) {
            ieee80211_tx_stat_t txstatus;
            int noratectrl;
            noratectrl = an->an_flags & (ATH_NODE_CLEAN | ATH_NODE_PWRSAVE);
            if (txok) {
                u_int8_t txant = ds->ds_txstat.ts_antenna;
                sc->sc_cur_txant = txant; /* save current tx antenna */
               if (txant <  8 )
               {
                       sc->sc_stats.ast_ant_tx[txant]++;
                       sc->sc_ant_tx[txant]++;
               }
                if (ds->ds_txstat.ts_rateindex != 0)
                    sc->sc_stats.ast_tx_altrate++;
                sc->sc_stats.ast_tx_rssi =
                    ds->ds_txstat.ts_rssi;
                rateCode = ds->ds_txstat.ts_ratecode;
                txstatus.rssi = ds->ds_txstat.ts_rssi;
                txstatus.flags=0;

                if (sc->sc_hashtsupport) {
                   /*
                    * Update 11n stats
                    */
                    sc->sc_stats.ast_tx_rssi_ctl0 = ds->ds_txstat.ts_rssi_ctl0;
                    sc->sc_stats.ast_tx_rssi_ctl1 = ds->ds_txstat.ts_rssi_ctl1;
                    sc->sc_stats.ast_tx_rssi_ctl2 = ds->ds_txstat.ts_rssi_ctl2;
                    sc->sc_stats.ast_tx_rssi_ext0 = ds->ds_txstat.ts_rssi_ext0;
                    sc->sc_stats.ast_tx_rssi_ext1 = ds->ds_txstat.ts_rssi_ext1;
                    sc->sc_stats.ast_tx_rssi_ext2 = ds->ds_txstat.ts_rssi_ext2;
                   /*
                    * Update rate average
                    */
                    txstatus.rssictl[0] = ds->ds_txstat.ts_rssi_ctl0;
                    txstatus.rssictl[1] = ds->ds_txstat.ts_rssi_ctl1;
                    txstatus.rssictl[2] = ds->ds_txstat.ts_rssi_ctl2;
                    txstatus.rssiextn[0] = ds->ds_txstat.ts_rssi_ext0;
                    txstatus.rssiextn[1] = ds->ds_txstat.ts_rssi_ext1;
                    txstatus.rssiextn[2] = ds->ds_txstat.ts_rssi_ext2;
                    txstatus.flags |= ATH_TX_CHAIN_RSSI_VALID;
                }

                if (bf->bf_isdata) {
                    txstatus.rateieee = sc->sc_hwmap[rateCode].ieeerate;
                    txstatus.rateKbps = sc->sc_hwmap[rateCode].rateKbps;
                    txstatus.ratecode = rateCode;

                    if (IS_HT_RATE(rateCode)) {
                        u_int8_t    rateFlags = bf->bf_rcs[ds->ds_txstat.ts_rateindex].flags;

                        /* TODO - add table to avoid division */
                        if (rateFlags & ATH_RC_CW40_FLAG) {
                            txstatus.rateKbps = (txstatus.rateKbps * 27) / 13;
                        }
                        if (rateFlags & ATH_RC_SGI_FLAG) {
                            txstatus.rateKbps = (txstatus.rateKbps * 10) / 9;
                        }
                    }
                    /*
                     * Calculate the average of the IEEE rates, which
                     * is in 500kbps units
                     */
                } else {
                    /*
                     * Filter out management frames sent at the lowest rate, 
                     * especially ProbeRequests sent during scanning.
                     */
                    txstatus.rateieee = 0;
                    txstatus.rateKbps = 0;
                    txstatus.ratecode = 0;
                }

                /*
                 * Calculate air time of packet on air only when requested.
                 */
                txstatus.airtime = bf->bf_calcairtime ? ath_hal_txcalcairtime(ah, ds) : 0;

                sc->sc_ieee_ops->tx_status(an->an_node,&txstatus);
                
                if (bf->bf_flags & HAL_TXDESC_RTSENA) {
                    /* XXX: how to get RTS success count ???
                     * We assume once the RTS gets an CTS, normally the frame would
                     * transmit succesfully coz no one would contend with us. */
                    sc->sc_phy_stats[sc->sc_curmode].ast_tx_rts++;
                }
#ifdef ATH_SWRETRY
               if (bf->bf_isswretry) {
                   pInfo = &an->an_swretry_info[txq->axq_qnum];
                   ASSERT(pInfo->swr_num_pendfrms || an->an_total_swrtx_pendfrms);                   

                   /* Decrement the counters, as we successfully
                    * transmitted the swretry frame
                    */
                   pInfo->swr_num_pendfrms--;
                   an->an_total_swrtx_pendfrms--;
                   an->an_total_swrtx_successfrms++;
                   
                   if (!an->an_total_swrtx_pendfrms)
                       ath_tx_drain_sxmitq(sc, an);
                          
               }    
#endif                
                
            } else {

                if (ds->ds_txstat.ts_status & HAL_TXERR_XRETRY) {
                    sc->sc_stats.ast_tx_xretries++;
#if defined(ATH_ADDITIONAL_STATS) || defined(ATH_SUPPORT_IQUE)
                    sc->sc_stats.ast_txq_xretries[txq->axq_qnum]++;
#endif
                }
                if (ds->ds_txstat.ts_status & HAL_TXERR_FIFO) {
                    sc->sc_stats.ast_tx_fifoerr++;
#if defined(ATH_ADDITIONAL_STATS) || defined(ATH_SUPPORT_IQUE)
                    sc->sc_stats.ast_txq_fifoerr[txq->axq_qnum]++;
#endif
                }
                if (ds->ds_txstat.ts_status & HAL_TXERR_FILT) {
                    sc->sc_stats.ast_tx_filtered++;
#if defined(ATH_ADDITIONAL_STATS) || defined(ATH_SUPPORT_IQUE)
                    sc->sc_stats.ast_txq_filtered[txq->axq_qnum]++;
#endif
                }
                if (ds->ds_txstat.ts_status & HAL_TXERR_XTXOP)
                    __11nstats(sc, txaggr_xtxop);
                if (ds->ds_txstat.ts_status & HAL_TXERR_TIMER_EXPIRED)
                    __11nstats(sc, txaggr_timer_exp);
                
#ifdef ATH_SWRETRY
                /* Decrement the number of pendfrms when we are
                 * trying to come out of sw retrying mechanism
                 */
                if ((sc->sc_scanning || (!an->an_swrenabled)) && /*If scanning or Bmiss happens*/
                        bf->bf_isswretry && /*If frm passed through swRetry mechanism*/
                        !(bf->bf_status & ATH_BUFSTATUS_MARKEDSWRETRY)) { 
                    pInfo = &an->an_swretry_info[txq->axq_qnum];    
                    pInfo->swr_num_pendfrms--;
                    an->an_total_swrtx_pendfrms--;
                    sc->sc_halstats.ns_swretryfailcount++;
                }
#endif                        
                
            }
            sr = ds->ds_txstat.ts_shortretry;
            lr = ds->ds_txstat.ts_longretry;
            sc->sc_phy_stats[sc->sc_curmode].ast_tx_shortretry += sr;
            sc->sc_phy_stats[sc->sc_curmode].ast_tx_longretry += lr;

            /*
             * Hand the descriptor to the rate control algorithm
             * if the frame wasn't dropped for filtering or sent
             * w/o waiting for an ack.  In those cases the rssi
             * and retry counts will be meaningless.
             */
            if (! bf->bf_isampdu) {
                /*
                 * This frame is sent out as a single frame. Use hardware retry
                 * status for this frame.
                 */
                bf->bf_retries = ds->ds_txstat.ts_longretry;
                if (ds->ds_txstat.ts_status & HAL_TXERR_XRETRY) {
                      __11nstats(sc,tx_sf_hw_xretries);
                     bf->bf_isxretried = 1;
                }
                nbad = 0;
            } else {
                nbad = ath_tx_num_badfrms(sc, bf, txok);
            }

            if ((ds->ds_txstat.ts_status & HAL_TXERR_FILT) == 0 &&
                (bf->bf_flags & HAL_TXDESC_NOACK) == 0) {
                /*
                 * If frame was ack'd update the last rx time
                 * used to workaround phantom bmiss interrupts.
                 */
                if (ds->ds_txstat.ts_status == 0)
                    nacked++;

                if (bf->bf_isdata && !noratectrl) {
                    ath_rate_tx_complete(sc,
                                         an,
                                         isrifs ? bf->bf_rifslast->bf_desc : ds,
                                         bf->bf_rcs,
                                         TID_TO_WME_AC(bf->bf_tidno),
                                         bf->bf_nframes,
                                         nbad);
                }
            }

#ifdef ATH_SWRETRY
            if ((sc->sc_debug & ATH_DEBUG_SWR) && (ds->ds_txstat.ts_status || bf->bf_isswretry)) {
                wh = (struct ieee80211_frame *)wbuf_header(bf->bf_mpdu);
                DPRINTF(sc, ATH_DEBUG_SWR, "%s: SeqNo%d --> rate %02X, swretry %d retrycnt %d totaltries %d\n",__func__, 
                                                (*(u_int16_t *)&wh->i_seq[0]) >> 4, ds->ds_txstat.ts_ratecode, (bf->bf_isswretry)?1:0, 
                                                bf->bf_swretries, bf->bf_totaltries + ds->ds_txstat.ts_longretry + ds->ds_txstat.ts_shortretry);
                DPRINTF(sc, ATH_DEBUG_SWR, "%s: SeqNo%d --> status %08X, destmask %d\n",__func__, 
                                                (*(u_int16_t *)&wh->i_seq[0]) >> 4, ds->ds_txstat.ts_status, (bf->bf_desc->ds_ctl0 & 0x01000000)? 1:0);
            }
            
            if (bf->bf_status & ATH_BUFSTATUS_MARKEDSWRETRY) {
                ath_tx_mpdu_resend(sc, txq, &bf_head);
                /* We have completed the buf in resend in case of
                 * failure and hence not needed and will be fatal
                 * if we fall through this                
                 */
                //XXX TBD txFFDrain();
               continue;
            }   
#endif                  
            /*
             * Complete this transmit unit
             *
             * Node cannot be referenced past this point since it can be freed
             * here.
             */
            if (bf->bf_isampdu) {
                if (ds->ds_txstat.ts_flags & HAL_TX_DESC_CFG_ERR)
                    __11nstats(sc, txaggr_desc_cfgerr);
                if (ds->ds_txstat.ts_flags & HAL_TX_DATA_UNDERRUN)
                    __11nstats(sc, txaggr_data_urun);
                if (ds->ds_txstat.ts_flags & HAL_TX_DELIM_UNDERRUN)
                    __11nstats(sc, txaggr_delim_urun);

                ath_tx_complete_aggr_rifs(sc, txq, bf, &bf_head, txok);
            } else {                
#ifndef REMOVE_PKT_LOG
                /* do pktlog */
                {
                    struct log_tx log_data;
                    struct ath_buf *tbf;

                    TAILQ_FOREACH(tbf, &bf_head, bf_list) {
                        log_data.firstds = tbf->bf_desc;
                        log_data.bf = tbf;
                        ath_log_txctl(sc, &log_data, 0);
                    }

                    /* log the last descriptor. */
                    log_data.firstds = lastbf->bf_desc;
                    log_data.bf = lastbf;
                    ath_log_txctl(sc, &log_data, 0);
                }
#endif

#ifdef ATH_SUPPORT_UAPSD
                if (uapsdq) {
                    ath_tx_uapsd_complete(sc, an, bf, &bf_head, txok);
                } else {
                    if (bf->bf_isbar)
                        ath_tx_complete_bar(sc, bf, &bf_head, txok);
                    else
                        ath_tx_complete_buf(sc, bf, &bf_head, txok);
                }
#else
                if (bf->bf_isbar)
                    ath_tx_complete_bar(sc, bf, &bf_head, txok);
                else
                    ath_tx_complete_buf(sc, bf, &bf_head, txok);
#endif
            }

#ifndef REMOVE_PKT_LOG
            /* do pktlog */
            {
                struct log_tx log_data;
                log_data.lastds = ds;
                ath_log_txstatus(sc, &log_data, 0);
            }
#endif
        }

        /*
         * schedule any pending packets if aggregation is enabled
         */
#ifdef ATH_RIFS
        if (sc->sc_txaggr || sc->sc_txrifs)
#else
        if (sc->sc_txaggr)
#endif
        {
            ATH_TXQ_LOCK(txq);
            ath_txq_schedule(sc, txq);
            ATH_TXQ_UNLOCK(txq);
        }
    }
    return nacked;
}

/*
 * Deferred processing of transmit interrupt.
 */
void
ath_tx_tasklet(ath_dev_t dev)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    u_int64_t tsf = ath_hal_gettsf64(sc->sc_ah);
    int i, nacked=0, qdepth = 0;
    u_int32_t qcumask = ((1 << HAL_NUM_TX_QUEUES) - 1);

    ath_hal_gettxintrtxqs(sc->sc_ah, &qcumask);

#ifdef AH_WAR_52640
    /*
     * Check if this function was invoked from ath_inact. If yes,
     * process the 'stuck' queues also, in addition to the queues
     * present in `qcumask'
     */
    if (sc->sc_qcumask) {
        qcumask |= sc->sc_qcumask;
        sc->sc_qcumask = 0;
    } else {
        sc->sc_tx_inact |= qcumask;
    }
#endif

    /*
     * Process each active queue.
     */
    for (i = 0; i < HAL_NUM_TX_QUEUES; i++) {
        if (ATH_TXQ_SETUP(sc, i) && (qcumask & (1 << i))) {
            nacked += ath_tx_processq(sc, &sc->sc_txq[i]);
            qdepth += ath_txq_depth(sc, i);
        }
    }
#ifdef ATH_SUPERG_XR
    if (sc->sc_xrtxq && txqactive(sc->sc_ah, sc->sc_xrtxq->axq_qnum))
        nacked += ath_tx_processq(sc, sc->sc_xrtxq);
#endif
    if (nacked)
        sc->sc_lastrx = tsf;

    if (sc->sc_ieee_ops->notify_txq_status)
        sc->sc_ieee_ops->notify_txq_status(sc->sc_ieee, qdepth);

    //netif_wake_queue(dev);
}

void
ath_tx_draintxq(struct ath_softc *sc, struct ath_txq *txq, HAL_BOOL retry_tx)
{
    struct ath_buf *bf, *lastbf;
    ath_bufhead bf_head;

    /*
     * NB: this assumes output has been stopped and
     *     we do not need to block ath_tx_tasklet
     */
    for (;;) {
        ATH_TXQ_LOCK(txq);
        bf = TAILQ_FIRST(&txq->axq_q);
        if (bf == NULL) {
            txq->axq_link = NULL;
            txq->axq_linkbuf = NULL;
            ATH_TXQ_UNLOCK(txq);
            break;
        }

        if (bf->bf_status & ATH_BUFSTATUS_STALE) {
            ATH_TXQ_REMOVE_STALE_HEAD(txq, bf, bf_list);
            ATH_TXQ_UNLOCK(txq);

            ATH_TXBUF_LOCK(sc);
            TAILQ_INSERT_TAIL(&sc->sc_txbuf, bf, bf_list);
            ATH_TXBUF_UNLOCK(sc);
            continue;
        }

        lastbf = bf->bf_lastbf;
        if (!retry_tx)
            lastbf->bf_desc->ds_txstat.ts_flags = HAL_TX_SW_ABORTED;

        /* remove ath_buf's of the same mpdu from txq */
        ATH_TXQ_MOVE_HEAD_UNTIL(txq, &bf_head, lastbf, bf_list);
        ATH_TXQ_UNLOCK(txq);

#ifdef AR_DEBUG
        if (sc->sc_debug & ATH_DEBUG_RESET)
            ath_printtxbuf(bf,
                           ath_hal_txprocdesc(ah, bf->bf_desc) == HAL_OK);
#endif /* AR_DEBUG */

        /*
         * Fail this transmit unit
         */
        if (bf->bf_isaggr) {
           __11nstats(sc,tx_comperror);
        }

        if (bf->bf_isampdu) {
            if (!bf->bf_isaggr) {
                __11nstats(sc,tx_unaggr_comperror);
            }

            ath_tx_complete_aggr_rifs(sc, txq, bf, &bf_head, 0);
        } else {
            if (bf->bf_isbar) {
                ath_tx_complete_bar(sc, bf, &bf_head, 0);
            } else {
                ath_tx_complete_buf(sc, bf, &bf_head, 0);
            }
        }
    }
    txq->axq_aggr_nbuf = 0;
    txq->axq_aggr_depth = 0;
    ath_drain_legacy_buf(sc, txq);

    /* flush any pending frames if aggregation is enabled */
#ifdef ATH_RIFS
    if (sc->sc_txaggr || sc->sc_txrifs) {
#else
    if (sc->sc_txaggr) {
#endif
        if (!retry_tx) {
            ATH_TXQ_LOCK(txq);
            ath_txq_drain_pending_buffers(sc, txq);
            sc->sc_ieee_ops->drain_amsdu(sc->sc_ieee); /* drain amsdu buffers */
            ATH_TXQ_UNLOCK(txq);
        }
    }

#ifdef ATH_SWRETRY
    if (sc->sc_swRetryEnabled)
        txq->axq_destmask = AH_TRUE;
#endif
}

static void
ath_tx_stopdma(struct ath_softc *sc, struct ath_txq *txq)
{
    struct ath_hal *ah = sc->sc_ah;

    (void) ath_hal_stoptxdma(ah, txq->axq_qnum);
    DPRINTF(sc, ATH_DEBUG_RESET, "%s: tx queue [%u] %x, link %p\n",
            __func__, txq->axq_qnum,
            ath_hal_gettxbuf(ah, txq->axq_qnum), txq->axq_link);
}

/*
 * Drain only the data queues.
 */
static void
ath_drain_txdataq(struct ath_softc *sc, HAL_BOOL retry_tx)
{
    struct ath_hal *ah = sc->sc_ah;
    int i;
    int npend = 0;
    HAL_HT_MACMODE ht_macmode = sc->sc_ieee_ops->cwm_macmode(sc->sc_ieee);

    /* XXX return value */
    if (!sc->sc_invalid) {
        for (i = 0; i < HAL_NUM_TX_QUEUES; i++) {
            if (ATH_TXQ_SETUP(sc, i)) {
                ath_tx_stopdma(sc, &sc->sc_txq[i]);

                /* The TxDMA may not really be stopped.
                 * Double check the hal tx pending count */
                npend += ath_hal_numtxpending(ah, sc->sc_txq[i].axq_qnum);
            }
        }
    }

    if (npend) {
        HAL_STATUS status;

        /* TxDMA not stopped, reset the hal */
        DPRINTF(sc, ATH_DEBUG_RESET, "%s: Unable to stop TxDMA. Reset HAL!\n", __func__);

        ATH_RESET_LOCK(sc);
        if (!ath_hal_reset(ah, sc->sc_opmode,
                           &sc->sc_curchan, ht_macmode,
                           sc->sc_tx_chainmask, sc->sc_rx_chainmask,
                           sc->sc_ht_extprotspacing, AH_FALSE, &status)) {
            printk("%s: unable to reset hardware; hal status %u\n", __func__, status);
        }
        ATH_RESET_UNLOCK(sc);
    }

    //sc->sc_dev->trans_start = jiffies;
    // RNWF Need this?? netif_start_queue(sc->sc_dev);
    for (i = 0; i < HAL_NUM_TX_QUEUES; i++) {
        if (ATH_TXQ_SETUP(sc, i)) {
#ifdef ATH_SUPPORT_UAPSD
            if (&sc->sc_txq[i] == sc->sc_uapsdq) {
                ath_tx_uapsd_draintxq(sc);
                continue;
            }
#endif
            ath_tx_draintxq(sc, &sc->sc_txq[i], retry_tx);
        }
    }
}

/*
 * Drain the transmit queues and reclaim resources.
 */
void
ath_draintxq(struct ath_softc *sc, HAL_BOOL retry_tx)
{
    /* stop beacon queue. The beacon will be freed when we go to INIT state */
    if (!sc->sc_invalid) {
        (void) ath_hal_stoptxdma(sc->sc_ah, sc->sc_bhalq);
        DPRINTF(sc, ATH_DEBUG_RESET, "%s: beacon queue %x\n", __func__,
                ath_hal_gettxbuf(sc->sc_ah, sc->sc_bhalq));
    }
    
    ath_drain_txdataq(sc, retry_tx);

#ifdef ATH_SWRETRY
    ath_tx_reset_swretry(sc);
    ath_tx_flush_sxmitq(sc);
#endif
}

/*
 * Flush the pending traffic in tx queues. Beacon queue is not stopped.
 */
void
ath_tx_flush(ath_dev_t dev)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);

    ATH_PS_WAKEUP(sc);
    ath_drain_txdataq(sc, FALSE);
    ATH_PS_SLEEP(sc);
}

u_int32_t
ath_txq_depth(ath_dev_t dev, int qnum)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);

    return sc->sc_txq[qnum].axq_depth +
           sc->sc_txq[qnum].axq_ll_count;
#ifdef ATH_SWRETRY
    /* XXX TODO the num of frames present in SW Retry queue
     * are not reported. No problems are forseen at this
     * moment due to this. Need to revisit this if problem
     * occurs
     */
#endif
}

u_int32_t
ath_txq_aggr_depth(ath_dev_t dev, int qnum)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);

    return sc->sc_txq[qnum].axq_aggr_depth;
}

u_int32_t
ath_txq_aggr_nbuf(ath_dev_t dev, int qnum)
{
    return ATH_DEV_TO_SC(dev)->sc_txq[qnum].axq_aggr_nbuf;
}

u_int32_t
ath_txq_lim(ath_dev_t dev, int qnum)
{
    return ATH_DEV_TO_SC(dev)->sc_txq[qnum].axq_lim;
}


#define BITS_PER_BYTE           8
#define OFDM_PLCP_BITS          22
#define HT_RC_2_MCS(_rc)        ((_rc) & 0x0f)
#define HT_RC_2_STREAMS(_rc)    ((((_rc) & 0x78) >> 3) + 1)
#define L_STF                   8
#define L_LTF                   8
#define L_SIG                   4
#define HT_SIG                  8
#define HT_STF                  4
#define HT_LTF(_ns)             (4 * (_ns))
#define SYMBOL_TIME(_ns)        ((_ns) << 2)            // ns * 4 us
#define SYMBOL_TIME_HALFGI(_ns) (((_ns) * 18 + 4) / 5)  // ns * 3.6 us
#define NUM_SYMBOLS_PER_USEC(_usec) (_usec >> 2)
#define NUM_SYMBOLS_PER_USEC_HALFGI(_usec) (((_usec*5)-4)/18)

static const u_int32_t bits_per_symbol[][2] = {
    /* 20MHz 40MHz */
    {    26,   54 },     //  0: BPSK
    {    52,  108 },     //  1: QPSK 1/2
    {    78,  162 },     //  2: QPSK 3/4
    {   104,  216 },     //  3: 16-QAM 1/2
    {   156,  324 },     //  4: 16-QAM 3/4
    {   208,  432 },     //  5: 64-QAM 2/3
    {   234,  486 },     //  6: 64-QAM 3/4
    {   260,  540 },     //  7: 64-QAM 5/6
    {    52,  108 },     //  8: BPSK
    {   104,  216 },     //  9: QPSK 1/2
    {   156,  324 },     // 10: QPSK 3/4
    {   208,  432 },     // 11: 16-QAM 1/2
    {   312,  648 },     // 12: 16-QAM 3/4
    {   416,  864 },     // 13: 64-QAM 2/3
    {   468,  972 },     // 14: 64-QAM 3/4
    {   520, 1080 },     // 15: 64-QAM 5/6
};

/*
 * ath_pkt_dur - compute packet duration (NB: not NAV)
 * ref: depot/chips/owl/2.0/rtl/mac/doc/rate_to_duration_ht.xls
 *
 * rix - rate index
 * pktlen - total bytes (delims + data + fcs + pads + pad delims)
 * width  - 0 for 20 MHz, 1 for 40 MHz
 * half_gi - to use 4us v/s 3.6 us for symbol time
 */
static u_int32_t
ath_pkt_duration(struct ath_softc *sc, u_int8_t rix, struct ath_buf *bf,
                 int width, int half_gi, HAL_BOOL shortPreamble)
{
    const HAL_RATE_TABLE    *rt = sc->sc_currates;
    u_int32_t               nbits, nsymbits, duration, nsymbols;
    u_int8_t                rc;
    int                     streams;
    int                     pktlen;
    
    pktlen = bf->bf_isaggr ? bf->bf_al : bf->bf_frmlen;
    rc = rt->info[rix].rateCode;

    /*
     * for legacy rates, use old function to compute packet duration
     */
    if (!IS_HT_RATE(rc))
        return ath_hal_computetxtime(sc->sc_ah, rt, pktlen, rix,
                                     shortPreamble);

    /*
     * find number of symbols: PLCP + data
     */
    nbits = (pktlen << 3) + OFDM_PLCP_BITS;
    nsymbits = bits_per_symbol[HT_RC_2_MCS(rc)][width];
    nsymbols = (nbits + nsymbits - 1) / nsymbits;

    if (!half_gi)
        duration = SYMBOL_TIME(nsymbols);
    else
        duration = SYMBOL_TIME_HALFGI(nsymbols);

    /*
     * addup duration for legacy/ht training and signal fields
     */
    streams = HT_RC_2_STREAMS(rc);
    duration += L_STF + L_LTF + L_SIG + HT_SIG + HT_STF + HT_LTF(streams);
    return duration;
}

/*
 * Rate module function to set rate related fields in tx descriptor.
 */
void
ath_buf_set_rate(struct ath_softc *sc, struct ath_buf *bf)
{
    struct ath_hal       *ah = sc->sc_ah;
    const HAL_RATE_TABLE *rt;
    struct ath_desc      *ds = bf->bf_desc;
    struct ath_desc      *lastds = bf->bf_lastbf->bf_desc;
    HAL_11N_RATE_SERIES  series[4];
    int                  i, flags, rtsctsena = 0, dynamic_mimops = 0;
    u_int8_t             rix = 0, cix, ctsrate = 0;
    u_int                ctsduration = 0;
    u_int32_t		aggr_limit_with_rts = sc->sc_rtsaggrlimit;
    struct 		ath_node *an = (struct ath_node *) bf->bf_node;

#ifdef ATH_SUPPORT_IQUE
    u_int               retry_duration = 0;
    ath_atx_tid_t       *tid;
    int                 qnum;
#endif

    /*
     * get the cix for the lowest valid rix.
     */
    rt = sc->sc_currates;
    for (i = 4; i--;) {
	    if (bf->bf_rcs[i].tries) {
		    rix = bf->bf_rcs[i].rix;
		    break;
	    }
    }
    flags = (bf->bf_flags & (HAL_TXDESC_RTSENA | HAL_TXDESC_CTSENA));
    cix = rt->info[rix].controlRate;

    if (flags & HAL_TXDESC_RTSENA) {
        rtsctsena = 1;
    }
    /*
     * If 802.11g protection is enabled, determine whether
     * to use RTS/CTS or just CTS.  Note that this is only
     * done for OFDM/HT unicast frames.
     */
    else if (sc->sc_protmode != PROT_M_NONE &&
        (rt->info[rix].phy == IEEE80211_T_OFDM ||
         rt->info[rix].phy == IEEE80211_T_HT) &&
        (bf->bf_flags & HAL_TXDESC_NOACK) == 0)
    {
        if (sc->sc_protmode == PROT_M_RTSCTS)
            flags = HAL_TXDESC_RTSENA;
        else if (sc->sc_protmode == PROT_M_CTSONLY)
            flags = HAL_TXDESC_CTSENA;

        cix = rt->info[sc->sc_protrix].controlRate;
        sc->sc_stats.ast_tx_protect++;
        rtsctsena = 1;
    }

    /* For 11n, the default behavior is to enable RTS for
     * hw retried frames. We enable the global flag here and
     * let rate series flags determine which rates will actually
     * use RTS.
     */
    if (sc->sc_hashtsupport && bf->bf_isdata) {
        KASSERT(an != NULL, ("an == null"));
        /*
         * 802.11g protection not needed, use our default behavior
         */
        if (!rtsctsena)
            flags = HAL_TXDESC_RTSENA;
        /*
         * For dynamic MIMO PS, RTS needs to precede the first aggregate
         * and the second aggregate should have any protection at all.
         */
        if (an->an_smmode == ATH_SM_PWRSAV_DYNAMIC) {
            if (!bf->bf_aggrburst) {
                flags = HAL_TXDESC_RTSENA;
                dynamic_mimops = 1;
            } else {
                flags = 0;
            }
        }
    }

    /*
     * Set protection if aggregate protection on
     */
    if (sc->sc_config.ath_aggr_prot && (!bf->bf_isaggr ||
        (bf->bf_isaggr && bf->bf_al < sc->sc_config.ath_aggr_prot_max)))
    {
        flags = HAL_TXDESC_RTSENA;
        cix = rt->info[sc->sc_protrix].controlRate;
        rtsctsena = 1;
    }

    /*
     * OWL 2.0 WAR, RTS cannot be followed by a frame larger than 8K.
     */
    if (bf->bf_isaggr && (bf->bf_al > aggr_limit_with_rts)) {
        /*
         * Ensure that in the case of SM Dynamic power save
         * while we are bursting the second aggregate the
         * RTS is cleared.
         */
        flags &= ~(HAL_TXDESC_RTSENA);
    }

    /*
     * CTS transmit rate is derived from the transmit rate
     * by looking in the h/w rate table.  We must also factor
     * in whether or not a short preamble is to be used.
     */
    /* NB: cix is set above where RTS/CTS is enabled */
    KASSERT(cix != 0xff, ("cix not setup"));
    ctsrate = rt->info[cix].rateCode |
        (bf->bf_shpreamble ? rt->info[cix].shortPreamble : 0);

    /*
     * Setup HAL rate series
     */
    OS_MEMZERO(series, sizeof(HAL_11N_RATE_SERIES) * 4);

    for (i = 0; i < 4; i++) {
        if (!bf->bf_rcs[i].tries)
            continue;

        rix = bf->bf_rcs[i].rix;

        series[i].Rate = rt->info[rix].rateCode |
                         (bf->bf_shpreamble ? rt->info[rix].shortPreamble : 0);

        series[i].Tries = bf->bf_rcs[i].tries;

        series[i].RateFlags = (
            (bf->bf_rcs[i].flags & ATH_RC_RTSCTS_FLAG) ? HAL_RATESERIES_RTS_CTS : 0) |
            ((bf->bf_rcs[i].flags & ATH_RC_CW40_FLAG) ? HAL_RATESERIES_2040 : 0)     |
            ((bf->bf_rcs[i].flags & ATH_RC_SGI_FLAG) ? HAL_RATESERIES_HALFGI : 0)    |
            ((bf->bf_rcs[i].flags & ATH_RC_TX_STBC_FLAG) ? HAL_RATESERIES_STBC: 0);

        series[i].PktDuration = ath_pkt_duration(
            sc, rix, bf,
            (bf->bf_rcs[i].flags & ATH_RC_CW40_FLAG) != 0,
            (bf->bf_rcs[i].flags & ATH_RC_SGI_FLAG),
            bf->bf_shpreamble);

#ifdef ATH_SUPPORT_IQUE
        retry_duration += series[i].PktDuration * series[i].Tries;
#endif

        if ((an->an_smmode == ATH_SM_PWRSAV_STATIC) &&
            (bf->bf_rcs[i].flags & ATH_RC_DS_FLAG) == 0)
        {
            /*
             * When sending to an HT node that has enabled static
             * SM/MIMO power save, send at single stream rates but use
             * maximum allowed transmit chains per user, hardware,
             * regulatory, or country limits for better range.
             */
            series[i].ChSel = sc->sc_tx_chainmask;
        } else {
#ifdef ATH_CHAINMASK_SELECT
            if (bf->bf_ht)
                series[i].ChSel = ath_chainmask_sel_logic(sc, an);
            else
                series[i].ChSel = sc->sc_tx_chainmask;
#else
            series[i].ChSel = sc->sc_tx_chainmask;
#endif
        }

        if (rtsctsena)
            series[i].RateFlags |= HAL_RATESERIES_RTS_CTS;

        /*
         * Set RTS for all rates if node is in dynamic powersave
         * mode and we are using dual stream rates.
         */
        if (dynamic_mimops && (bf->bf_rcs[i].flags & ATH_RC_DS_FLAG))
            series[i].RateFlags |= HAL_RATESERIES_RTS_CTS;
    }

#ifdef ATH_SUPPORT_IQUE
    /*
     * Calculate and update the latency due to retries for VO/VI data queues only
     */
    if (bf->bf_isdata && sc->sc_txaggr && sc->sc_retry_duration > 0) {
        tid = ATH_AN_2_TID(an, bf->bf_tidno);
        qnum = tid->ac->qnum;
        if (qnum == sc->sc_haltype2q[HAL_WME_AC_VI] ||
            qnum == sc->sc_haltype2q[HAL_WME_AC_VO])
        {
            ath_rateseries_update_retry(sc, bf, series, retry_duration);
        }
    }
#endif

	/*
     * For non-HT devices, calculate RTS/CTS duration in software
     * and disable multi-rate retry.
     */
    if (flags && !sc->sc_hashtsupport) {
        /*
         * Compute the transmit duration based on the frame
         * size and the size of an ACK frame.  We call into the
         * HAL to do the computation since it depends on the
         * characteristics of the actual PHY being used.
         *
         * NB: CTS is assumed the same size as an ACK so we can
         *     use the precalculated ACK durations.
         */
        if (flags & HAL_TXDESC_RTSENA) {    /* SIFS + CTS */
            ctsduration += bf->bf_shpreamble ?
                rt->info[cix].spAckDuration : rt->info[cix].lpAckDuration;
        }

        ctsduration += series[0].PktDuration;

        if ((bf->bf_flags & HAL_TXDESC_NOACK) == 0) {  /* SIFS + ACK */
            ctsduration += bf->bf_shpreamble ?
                rt->info[rix].spAckDuration : rt->info[rix].lpAckDuration;
        }

        /*
         * Disable multi-rate retry when using RTS/CTS by clearing
         * series 1, 2 and 3.
         */
        OS_MEMZERO(&series[1], sizeof(HAL_11N_RATE_SERIES) * 3);
    }

    /*
     * set dur_update_en for l-sig computation except for PS-Poll frames
     */
    ath_hal_set11n_ratescenario(ah, ds, lastds,
                                !bf->bf_ispspoll,
                                ctsrate,
                                ctsduration,
                                series, 4, flags);
    if (sc->sc_config.ath_aggr_prot && flags)
        ath_hal_set11n_burstduration(ah, ds, sc->sc_config.ath_aggr_prot_duration);
}

#ifdef ATH_SUPPORT_IQUE
/*
 * Calculate the accumulative latency of the frame due to sw retries which is updated per sw retry
 * and the hw retries from the series for the worst cast, and compare the latency with the threshold.
 * If the accumulative latency is larger than the threshold, decrease the hw retries from the
 * lowest rate to the highest rate step by step.
 */
void
ath_rateseries_update_retry(struct ath_softc *sc, struct ath_buf *bf, HAL_11N_RATE_SERIES *series, int retry_duration)
{
    u_int       duration, delay_gap;
    u_int32_t   adjust[4];
    int8_t      i, total_tries=0;

    /*
     * If this is a sw retry frame and the accumulative tx duration is larger than the
     * threshold of tx duration, this frame will be dropped.
     */
    if (bf->bf_txduration > sc->sc_retry_duration) {
        series[0].Tries = 1;
        series[1].Tries = 0;
        series[2].Tries = 0;
        series[3].Tries = 0;
        return ;
    }

    /*
     * Here 'duration' is the accumulative latency due to
     * sw retries (bf->bf_txduration in us) and the hw
     * retries (retry_duration in us).
     */
    duration = retry_duration + bf->bf_txduration;
    if (duration > sc->sc_retry_duration) {
        delay_gap = duration - sc->sc_retry_duration;
        for (i = 3; i >= 0; i --) {
            adjust[i] = (u_int32_t)delay_gap/series[i].PktDuration;

            /*
             * Check for some boundary conditions
             */
            if (delay_gap < adjust[i] * series[i].PktDuration) {
                adjust[i] --;
            }
            if (adjust[i] < 0) {
                adjust[i] = 0;
            }
            if (adjust[i] > series[i].Tries) {
                adjust[i] = series[i].Tries;
            }

            /*
             * Adjust the hw retries and update the delay_gap
             */
            series[i].Tries -= adjust[i];
            delay_gap -= adjust[i] * series[i].PktDuration;
            total_tries += series[i].Tries;
        }
    }
    if (total_tries == 0) {
        series[0].Tries = 1;
    }
    return;
}
#endif

#define ADDBA_EXCHANGE_ATTEMPTS     10
#define ADDBA_TIMEOUT               200 /* 200 milliseconds */

/*
 * Check if an ADDBA is required.
 */
int
ath_aggr_check(ath_dev_t dev, ath_node_t node, u_int8_t tidno)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_node *an = ATH_NODE(node);
    struct ath_atx_tid *tid;

#ifdef ATH_RIFS
    if (!sc->sc_txaggr && !sc->sc_txrifs)
#else
    if (!sc->sc_txaggr)
#endif
        return 0;
    
    /* ADDBA exchange must be completed before sending aggregates */
    tid = ATH_AN_2_TID(an, tidno);

    if (tid->cleanup_inprogress)
        return 0;
    
    if (!tid->addba_exchangecomplete) {
        if (!tid->addba_exchangeinprogress &&
            (tid->addba_exchangeattempts < ADDBA_EXCHANGE_ATTEMPTS)) {
            tid->addba_exchangeattempts++;
            return 1;
        }
    }
    return 0;
}

#ifdef ATH_SUPERG_FF
int
ath_ff_check(ath_dev_t dev, 
             ath_node_t node, 
             int qnum,
             int check_qdepth, 
             u_int32_t txoplimit,
             u_int32_t framelen)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_node *an = ATH_NODE(node);
    struct ath_txq *txq = &sc->sc_txq[qnum];
    u_int32_t txtime;

    if (sc->sc_curchan.channelFlags & (CHANNEL_HALF | CHANNEL_QUARTER)) {
        return AH_FALSE;
    }

    if (check_qdepth) {
        ATH_TXQ_LOCK(txq);
        if (txq->axq_depth < sc->sc_fftxqmin) {
            ATH_TXQ_UNLOCK(txq);
            return AH_FALSE;
        }
        ATH_TXQ_UNLOCK(txq);
    }

    /* if the 4msec limit is set on the channel, take it in to account */
    if(sc->sc_curchan.privFlags & CHANNEL_4MS_LIMIT)
        txoplimit = MIN(txoplimit, US_PER_4MS); 

    txtime = ath_hal_computetxtime(sc->sc_ah, sc->sc_currates, framelen,
                       an->an_prevdatarix, AH_FALSE);
    if (txoplimit != 0 && txtime > txoplimit) {
        DPRINTF(sc, ATH_DEBUG_XMIT | ATH_DEBUG_FF,
            "%s: FF TxOp violation\n", __func__);
        return AH_FALSE;
    }

    return AH_TRUE;

}
#endif

int
ath_get_amsdusupported(ath_dev_t dev, ath_node_t node, int tidno)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_node *an = ATH_NODE(node);
    ath_atx_tid_t *tid = ATH_AN_2_TID(an, tidno);

    if (sc->sc_txamsdu && tid->addba_exchangecomplete) {
        return (tid->addba_amsdusupported);
    }
    return (FALSE);
}

/*
 * ADDBA request timer - timeout
 */
static int
ath_addba_timer(void *arg)
{
    ath_atx_tid_t *tid = (ath_atx_tid_t *)arg;
    struct ath_softc *sc = tid->an->an_sc;

    ATH_PS_WAKEUP(sc);

    if (cmpxchg(&tid->addba_exchangeinprogress, 1, 0) == 1) {
        /* ADDBA exchange timed out, send pending frames as regular frames */
        ath_tx_flush_tid(sc, tid);
    }

    ATH_PS_SLEEP(sc);

    return 1;   /* don't re-arm itself */
}

/*
 * Setup ADDBA request
 */
void
ath_addba_requestsetup(
    ath_dev_t dev, ath_node_t node,
    u_int8_t tidno,
    struct ieee80211_ba_parameterset *baparamset,
    u_int16_t *batimeout,
    struct ieee80211_ba_seqctrl *basequencectrl,
    u_int16_t buffersize
    )
{
    struct ath_node *an = ATH_NODE(node);
    ath_atx_tid_t *tid = ATH_AN_2_TID(an, tidno);

    baparamset->amsdusupported = 0;
    baparamset->bapolicy       = IEEE80211_BA_POLICY_IMMEDIATE;
    baparamset->tid            = tidno;
    baparamset->buffersize     = buffersize;
    *batimeout                 = 0;
    basequencectrl->fragnum    = 0;
    basequencectrl->startseqnum = tid->seq_start;

    /* Start ADDBA request timer */
    if (cmpxchg(&tid->addba_exchangeinprogress, 0, 1) == 0) {
        if (!ath_timer_is_active(&tid->addba_requesttimer))
            ath_start_timer(&tid->addba_requesttimer);
    }

    ath_tx_pause_tid(ATH_DEV_TO_SC(dev), tid);
}

/*
 * Process ADDBA response
 */
void
ath_addba_responseprocess(
    ath_dev_t dev, ath_node_t node,
    u_int16_t statuscode,
    struct ieee80211_ba_parameterset *baparamset,
    u_int16_t batimeout
    )
{
    struct ath_node *an = ATH_NODE(node);
    u_int16_t tidno = baparamset->tid;
    ath_atx_tid_t *tid = ATH_AN_2_TID(an, tidno);
    int resume = 1;

    /* Stop ADDBA request timer */
    if (cmpxchg(&tid->addba_exchangeinprogress, 1, 0) == 1) {
        ath_cancel_timer(&tid->addba_requesttimer, CANCEL_NO_SLEEP);
    } else {
        resume = 0;
    }

    tid->addba_exchangestatuscode = statuscode;

    if (statuscode == IEEE80211_STATUS_SUCCESS) {
        /* Enable aggregation! */
        tid->addba_exchangecomplete = 1;

        /* adjust transmitter's BAW size according to ADDBA response */
        tid->baw_size = MIN(baparamset->buffersize, tid->baw_size);

        /* This field indicates whether the receiver accepts AMSDUs
         * carried in QoS data AMPDU under this BlockAck aggrement.
         */
        tid->addba_amsdusupported = baparamset->amsdusupported;

        if (resume) {
            ath_tx_resume_tid(ATH_DEV_TO_SC(dev), tid);
        }
    } else {
        /* ADDBA exchange failed, send pending frames as regular frames */
        if (resume) {
            ath_tx_flush_tid(ATH_DEV_TO_SC(dev), tid);
        }
    }
}

/*
 * Return status of ADDBA request
 */
u_int16_t
ath_addba_status(ath_dev_t dev, ath_node_t node, u_int8_t tidno)
{
    struct ath_node *an = ATH_NODE(node);
    struct ath_atx_tid *tid = ATH_AN_2_TID(an, tidno);
    u_int16_t status;

    /*
     * Report the ADDBA response status code.  Return a special status to indicate
     * that either ADDBA was not initiated, or the response has not been received yet.
     */
    if ((tid->addba_exchangestatuscode == IEEE80211_STATUS_SUCCESS) &&
        !tid->addba_exchangecomplete) {
        status = 0xFFFF;
    } else {
        status = tid->addba_exchangestatuscode;
    }

    return status;
}

/*
 * Clear ADDBA for all tids in this node
 */
void
ath_addba_clear(ath_dev_t dev, ath_node_t node)
{
    struct ath_node *an = ATH_NODE(node);
    int i;
    struct ath_atx_tid *tid;
    struct ath_arx_tid *rxtid;
#define	N(a)	(sizeof (a) / sizeof (a[0]))

    for (i = 0; i < N(an->an_tx_tid); i++) {
        tid = &an->an_tx_tid[i];
        if (tid->addba_exchangecomplete) {
            tid->addba_exchangecomplete = 0;
            tid->addba_exchangeattempts = 0;
            tid->addba_exchangestatuscode = IEEE80211_STATUS_UNSPECIFIED;
            tid->paused = AH_FALSE;
        }
    }

    for (i = 0; i < N(an->an_rx_tid); i++) {
        rxtid = &an->an_rx_tid[i];
        if (rxtid->addba_exchangecomplete)
            rxtid->addba_exchangecomplete = 0;
    }
#undef N
}

/*
 * Add a sub-frame to block ack window
 */
static void
ath_tx_addto_baw(struct ath_softc *sc, struct ath_atx_tid *tid, struct ath_buf *bf)
{
    int index, cindex;

    if (bf->bf_isretried) {
        __11nstats(sc, tx_bawretries);
        return;
    }
    
    __11nstats(sc, tx_bawnorm);

    index  = ATH_BA_INDEX(tid->seq_start, bf->bf_seqno);
    cindex = (tid->baw_head + index) & (ATH_TID_MAX_BUFS - 1);

    ASSERT(tid->tx_buf[cindex] == NULL);
#ifdef NODE_FREE_DEBUG
    sc->sc_ieee_ops->add_trace(((struct ath_node *)bf->bf_node)->an_node,
                        (char *)__func__, "Index", cindex);
    sc->sc_ieee_ops->add_trace(((struct ath_node *)bf->bf_node)->an_node,
                        (char *)__func__, "AthBuf", (u_int64_t)(u_int32_t)bf);
#endif
    tid->tx_buf[cindex] = bf;

    if (index >= ((tid->baw_tail - tid->baw_head) & (ATH_TID_MAX_BUFS - 1))) {
        __11nstats(sc, tx_bawadv);
        tid->baw_tail = cindex;
        INCR(tid->baw_tail, ATH_TID_MAX_BUFS);
    }
}

/*
 * Update block ack window
 */
static void
ath_tx_update_baw(struct ath_softc *sc, struct ath_atx_tid *tid, int seqno)
{
    int index, cindex;

    __11nstats(sc, tx_bawupdates);

    index  = ATH_BA_INDEX(tid->seq_start, seqno);
    cindex = (tid->baw_head + index) & (ATH_TID_MAX_BUFS - 1);

#ifdef NODE_FREE_DEBUG
    {
    struct ath_buf *bf = (struct ath_buf *)tid->tx_buf[cindex];
    sc->sc_ieee_ops->add_trace(((struct ath_node *)bf->bf_node)->an_node,
                        (char *)__func__, "Index", cindex);
    sc->sc_ieee_ops->add_trace(((struct ath_node *)bf->bf_node)->an_node,
                        (char *)__func__, "AthBuf",
                        (u_int64_t)(u_int32_t)tid->tx_buf[cindex]);
    }
#endif
    tid->tx_buf[cindex] = NULL;

    while (tid->baw_head != tid->baw_tail && !tid->tx_buf[tid->baw_head]) {
        __11nstats(sc, tx_bawupdtadv);
        INCR(tid->seq_start, IEEE80211_SEQ_MAX);
        INCR(tid->baw_head, ATH_TID_MAX_BUFS);
    }
}

static void
ath_bar_tx(struct ath_softc *sc, struct ath_node *an, struct ath_atx_tid *tid)
{
    __11nstats(sc, tx_bars);

    /* pause TID until BAR completes */
    ath_tx_pause_tid(sc, tid);

    if (sc->sc_ieee_ops->send_bar) {
        if (sc->sc_ieee_ops->send_bar(an->an_node, tid->tidno, tid->seq_start)) {
            /* resume tid if send bar failed. */
            ath_tx_resume_tid(sc, tid);
        }
    }
}

static void
ath_tx_complete_bar(struct ath_softc *sc, struct ath_buf *bf, ath_bufhead *bf_q, int txok)
{
    struct ath_node *an = bf->bf_node;
    struct ath_atx_tid *tid = ATH_AN_2_TID(an, bf->bf_tidno);

    ath_tx_resume_tid(sc, tid);

    ath_tx_complete_buf(sc, bf, bf_q, txok);
}

/*
 * pause a tid
 */
static void
ath_tx_pause_tid(struct ath_softc *sc, struct ath_atx_tid *tid)
{
    struct ath_txq *txq = &sc->sc_txq[tid->ac->qnum];

    ATH_TXQ_LOCK(txq);   

    tid->paused++;

    __11nstats(sc, tx_tidpaused);

    ATH_TXQ_UNLOCK(txq);
}

/*
 * resume a tid and schedule aggregate
 */
static void
ath_tx_resume_tid(struct ath_softc *sc, struct ath_atx_tid *tid)
{
    struct ath_txq *txq = &sc->sc_txq[tid->ac->qnum];

    ASSERT(tid->paused > 0);
    ATH_TXQ_LOCK(txq);

    tid->paused--;
    __11nstats(sc, tx_tidresumed);

    if (tid->paused > 0) {
        ATH_TXQ_UNLOCK(txq);
        return;
    }


    if (TAILQ_EMPTY(&tid->buf_q)) {
        ATH_TXQ_UNLOCK(txq);
        return;
    }

    /*
     * Add this TID to scheduler and try to send out aggregates
     */
    ath_tx_queue_tid(txq, tid);
    ath_txq_schedule(sc, txq);
    ATH_TXQ_UNLOCK(txq);
}

/*
 * flush tid's software queue and send frames as non-ampdu's
 */
static void
ath_tx_flush_tid(struct ath_softc *sc, struct ath_atx_tid *tid)
{
    struct ath_txq *txq = &sc->sc_txq[tid->ac->qnum];
    struct ath_buf *bf;
    ath_bufhead bf_head;
    int isProbe;

    ASSERT(tid->paused > 0);
    ATH_TXQ_LOCK(txq);

    tid->paused--;
    __11nstats(sc, tx_tidresumed);

    if (tid->paused > 0) {
        ATH_TXQ_UNLOCK(txq);
        return;
    }

    while (!TAILQ_EMPTY(&tid->buf_q)) {
        bf = TAILQ_FIRST(&tid->buf_q);
        ASSERT(!bf->bf_isretried);
        TAILQ_REMOVE_HEAD_UNTIL(&tid->buf_q, &bf_head, bf->bf_lastfrm, bf_list);
        /*
         * Frames in tid queue do not have rate series set, so do it here
         * before transmitting.
         */
        sc->sc_log_rcfind = 1;
        ath_rate_findrate(sc, tid->an, bf->bf_shpreamble, bf->bf_frmlen,
                        ATH_11N_TXMAXTRY, ATH_RC_PROBE_ALLOWED,
                        TID_TO_WME_AC(tid->tidno), bf->bf_rcs, &isProbe, AH_FALSE);
        ath_tx_send_normal(sc, txq, tid, &bf_head);
    }

    ATH_TXQ_UNLOCK(txq);
}

/*
 * Performs transmit side cleanup when TID changes from aggregated to
 * unaggregated.
 * - Pause the TID and mark cleanup in progress
 * - Stop ADDBA timer if it's armed.
 * - Discard all retry frames from the s/w queue.
 */
void
ath_tx_aggr_teardown(struct ath_softc *sc, struct ath_node *an, u_int8_t tidno)
{
    struct ath_atx_tid *tid = ATH_AN_2_TID(an, tidno);
    struct ath_txq *txq = &sc->sc_txq[tid->ac->qnum];
    struct ath_buf *bf;
    ath_bufhead bf_head;

    if ( tid->cleanup_inprogress ) /* cleanup is in progress */
        return;

    if (!tid->addba_exchangecomplete) {
        /* Clear the addba_exchangeattempts
         * we might have exceeded the exchageattempts limit
         */
        tid->addba_exchangeattempts = 0;
        tid->addba_exchangestatuscode = IEEE80211_STATUS_UNSPECIFIED;
        return;
    }

    /* TID must be paused first */
    ath_tx_pause_tid(sc, tid);

    /* stop ADDBA request timer (if ADDBA in progress) */
    if (cmpxchg(&tid->addba_exchangeinprogress, 1, 0) == 1)
        ath_cancel_timer(&tid->addba_requesttimer, CANCEL_NO_SLEEP);

    /*
     * drop all software retried frames and mark this TID
     */
    ATH_TXQ_LOCK(txq);
    while (!TAILQ_EMPTY(&tid->buf_q)) {
        bf = TAILQ_FIRST(&tid->buf_q);
        if (!bf->bf_isretried) {
            /*
             * NB: it's based on the assumption that
             * software retried frame will always stay
             * at the head of software queue.
             */
            break;
        }

        TAILQ_REMOVE_HEAD_UNTIL(&tid->buf_q, &bf_head, bf->bf_lastfrm, bf_list);
        ath_tx_update_baw(sc, tid, bf->bf_seqno);

        /* complete this sub-frame */
        ath_tx_complete_buf(sc, bf, &bf_head, 0);
    }


    if (tid->baw_head != tid->baw_tail) {
        ATH_TXQ_UNLOCK(txq);
        tid->cleanup_inprogress = AH_TRUE;
    } else {
        tid->addba_exchangecomplete = 0;
        tid->addba_exchangeattempts = 0;
        tid->addba_exchangestatuscode = IEEE80211_STATUS_UNSPECIFIED;
        ATH_TXQ_UNLOCK(txq);
        ath_tx_flush_tid(sc, tid);
    }


}

/*
 * Tear down either tx or rx aggregation. This is usually called
 * before protocol layer sends a DELBA.
 */
void
ath_aggr_teardown(ath_dev_t dev, ath_node_t node, u_int8_t tidno, u_int8_t initiator)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_node *an = ATH_NODE(node);

    if (initiator)
        ath_tx_aggr_teardown(sc, an, tidno);
    else
        ath_rx_aggr_teardown(sc, an, tidno);
}

/*
 * queue up a dest/ac pair for tx scheduling
 * NB: must be called with txq lock held
 */
static INLINE void
ath_tx_queue_tid(struct ath_txq *txq, struct ath_atx_tid *tid)
{
    struct ath_atx_ac *ac = tid->ac;

    /*
     * if tid is paused, hold off
     */
    if (tid->paused)
        return;
    
    /*
     * add tid to ac atmost once
     */
    if (tid->sched)
        return;

    tid->sched = AH_TRUE;
    TAILQ_INSERT_TAIL(&ac->tid_q, tid, tid_qelem);
    
    /*
     * add node ac to txq atmost once
     */
    if (ac->sched)
        return;

    ac->sched = AH_TRUE;
    TAILQ_INSERT_TAIL(&txq->axq_acq, ac, ac_qelem);
}

/*
 * Function to send a normal HT (non-AMPDU) frame
 * NB: must be called with txq lock held
 */
static int
ath_tx_send_normal(struct ath_softc *sc, struct ath_txq *txq, struct ath_atx_tid *tid, ath_bufhead *bf_head)
{
    struct ath_buf *bf;

    bf = TAILQ_FIRST(bf_head);
    bf->bf_isampdu = 0; /* regular HT frame */
    bf->bf_isn = 1;
    txq->axq_aggr_nbuf ++;

    /* update starting sequence number for subsequent ADDBA request */
    INCR(tid->seq_start, IEEE80211_SEQ_MAX);

    /* Queue to h/w without aggregation */
    __11nstats(sc, tx_pkts);
    bf->bf_nframes = 1;
    bf->bf_lastbf = bf->bf_lastfrm; /* one single frame */
    ath_buf_set_rate(sc, bf);
    ath_tx_txqaddbuf(sc, txq, bf_head);

    return 0;
}

/*
 * Function to send an A-MPDU
 * NB: must be called with txq lock held
 */
static INLINE int
ath_tx_send_ampdu(struct ath_softc *sc, struct ath_txq *txq, struct ath_atx_tid *tid, 
                  ath_bufhead *bf_head, ieee80211_tx_control_t *txctl)
{
    struct ath_buf *bf;
    int isProbe;
    u_int8_t ac = TID_TO_WME_AC(tid->tidno);

    bf = TAILQ_FIRST(bf_head);
    bf->bf_isampdu = 1;
    bf->bf_seqno = txctl->seqno; /* save seqno and tidno in buffer */
    bf->bf_tidno = txctl->tidno;

    __11nstats(sc, tx_pkts);

    /*
     * Do not queue to h/w when any of the following conditions is true:
     * - there are pending frames in software queue
     * - the TID is currently paused for ADDBA/BAR request
     * - seqno is not within block-ack window
     * - h/w queue depth exceeds low water mark
     */
    if (!TAILQ_EMPTY(&tid->buf_q) || tid->paused ||
#ifdef ATH_SUPPORT_IQUE
        txq->axq_depth >= min_qdepth_per_ac[ac] ||
#else
        txq->axq_depth >= ATH_AGGR_MIN_QDEPTH ||
#endif
        !BAW_WITHIN(tid->seq_start, tid->baw_size, bf->bf_seqno)) {
        /*
         * Add this frame to software queue for scheduling later
         * for aggregation.
         */
        __11nstats(sc, tx_queue);
        TAILQ_CONCAT(&tid->buf_q, bf_head, bf_list);
        ath_tx_queue_tid(txq, tid);
        return 0;
    }

    sc->sc_log_rcfind = 1;
    ath_rate_findrate(sc, tid->an, bf->bf_shpreamble, bf->bf_frmlen,
                      ATH_11N_TXMAXTRY, ATH_RC_PROBE_ALLOWED,
                      ac, bf->bf_rcs, &isProbe, AH_FALSE);
    /* Add sub-frame to BAW */
    ath_tx_addto_baw(sc, tid, bf);

    /* Queue to h/w without aggregation */
    __11nstats(sc, tx_minqdepth);
    bf->bf_nframes = 1;
    bf->bf_lastbf = bf->bf_lastfrm; /* one single frame */
    ath_buf_set_rate(sc, bf);
    ath_tx_txqaddbuf(sc, txq, bf_head);
    return 0;
}

/*
 * looks up the rate
 * returns aggr limit based on lowest of the rates
 */
static u_int32_t
ath_lookup_rate(struct ath_softc *sc, struct ath_node *an, struct ath_buf *bf, int mimoburst)
{
    int                     i, prate = 0;
    u_int32_t               max4msframelen, frame_length;
    u_int16_t               aggr_limit, legacy=0;
    const HAL_RATE_TABLE    *rt = sc->sc_currates;
    u_int16_t               maxampdu;
    u_int8_t                ac;

    /*
     * Log the rate lookup.
     */
    sc->sc_log_rcfind = 1;
    ac = TID_TO_WME_AC(bf->bf_tidno);

    /* XXX: we know shortPreamble and pktlen is not used for 11n rate control */
    ath_rate_findrate(sc, an, AH_TRUE, 0, ATH_11N_TXMAXTRY, ATH_RC_PROBE_ALLOWED,
                      ac, bf->bf_rcs, &prate, AH_FALSE);



    /*
     * Find the lowest frame length among the rate series that will have a
     * 4ms transmit duration.
     * TODO - TXOP limit needs to be considered.
     */
    max4msframelen = IEEE80211_AMPDU_LIMIT_MAX;

    for (i = 0; i < 4; i++) {
        if (bf->bf_rcs[i].tries) {
            frame_length = bf->bf_rcs[i].max4msframelen;

            if (rt->info[bf->bf_rcs[i].rix].phy != IEEE80211_T_HT) {
                legacy = 1;
                break;
            }

            max4msframelen = MIN(max4msframelen, frame_length);

            if (mimoburst)
                break;
        }
    }

    /*
     * limit aggregate size by the minimum rate if rate selected is
     * not a probe rate, if rate selected is a probe rate then
     * avoid aggregation of this packet.
     */
    if (prate || legacy)
        return 0;

#ifdef ATH_SUPPORT_IQUE
    if (sc->sc_ac_params[ac].aggrsize_scaling) {
        u_int16_t vi_depth = 0, vo_depth = 0;

        if (ATH_TXQ_SETUP(sc, WME_AC_VI)) {
            vi_depth = sc->sc_txq[WME_AC_VI].axq_depth;
        }

        if (ATH_TXQ_SETUP(sc, WME_AC_VO)) {
            vo_depth = sc->sc_txq[WME_AC_VO].axq_depth;
        }

        if (vi_depth > 0 || vo_depth > 0)
            max4msframelen = max4msframelen >> (sc->sc_ac_params[ac].aggrsize_scaling);
    }
#endif
    aggr_limit = MIN(max4msframelen, sc->sc_config.ampdu_limit);

    /*
     * h/w can accept aggregates upto 16 bit lengths (65535). The IE, however
     * can hold upto 65536, which shows up here as zero. Ignore 65536 since we
     * are constrained by hw.
     */
    maxampdu = an->an_aggr.tx.maxampdu;
    if (maxampdu)
        aggr_limit = MIN(aggr_limit, maxampdu);

    return aggr_limit;
}

/*
 * returns the number of delimiters to be added to
 * meet the minimum required mpdudensity.
 * caller should make sure that the rate is  HT rate .
 */
static INLINE int
ath_compute_num_delims(struct ath_softc *sc, struct ath_buf *bf, u_int16_t frmlen)
{
    const HAL_RATE_TABLE    *rt = sc->sc_currates;
    u_int32_t               nsymbits, nsymbols;
    int                     width, half_gi;
    int                     ndelim, mindelim;
    u_int16_t               minlen;
    u_int8_t                rc, flags, rix;
    u_int32_t               mpdudensity;

    /* Select standard number of delimiters based on frame length alone */
    ndelim = ATH_AGGR_GET_NDELIM(frmlen);

    /*
     * If encryption enabled, hardware requires some more padding between
     * subframes.
     * TODO - this could be improved to be dependent on the rate.
     *      The hardware can keep up at lower rates, but not higher rates
     *      See bug 20205.
     */
    switch (bf->bf_keytype) {
        case HAL_KEY_TYPE_AES:
            ndelim += ATH_AGGR_ENCRYPTDELIM;
            break;
        case HAL_KEY_TYPE_WEP:
        case HAL_KEY_TYPE_TKIP:
            ndelim += ATH_NODE(bf->bf_node)->an_aggr.tx.weptkipdelim;
            break;
        default:
            break;
    }

    /*
     * Convert desired mpdu density from microeconds to bytes based
     * on highest rate in rate series (i.e. first rate) to determine
     * required minimum length for subframe. Take into account
     * whether high rate is 20 or 40Mhz and half or full GI.
     */
    mpdudensity = ATH_NODE(bf->bf_node)->an_aggr.tx.mpdudensity;

    /*
     * If there is no mpdu density restriction, no further calculation
     * is needed.
     */
    if (mpdudensity == 0) {
        return ndelim;
    }

    rix = bf->bf_rcs[0].rix;
    flags = bf->bf_rcs[0].flags;
    rc = rt->info[rix].rateCode;
    width = (flags & ATH_RC_CW40_FLAG) ? 1 : 0;
    half_gi = (flags & ATH_RC_SGI_FLAG) ? 1 : 0;

    if (half_gi) {
        nsymbols=NUM_SYMBOLS_PER_USEC_HALFGI(mpdudensity);
    } else {
        nsymbols=NUM_SYMBOLS_PER_USEC(mpdudensity);
    }

    if (nsymbols == 0) {
        nsymbols = 1;
    }

    nsymbits = bits_per_symbol[HT_RC_2_MCS(rc)][width];
    minlen = (nsymbols * nsymbits) / BITS_PER_BYTE;

    /* Is frame shorter than required minimum length? */
    if (frmlen < minlen) {
        /* Get the minimum number of delimiters required. */
        mindelim = (minlen - frmlen) / ATH_AGGR_DELIM_SZ;
        ndelim = MAX(mindelim, ndelim);
    }

    return ndelim;
}

/*
 * For aggregation from software buffer queue.
 * NB: must be called with txq lock held
 */
static ATH_AGGR_STATUS
ath_tx_form_aggr(struct ath_softc *sc, struct ath_atx_tid *tid, 
	ath_bufhead *bf_q, struct ath_buf **bf_last, 
	struct aggr_rifs_param *param, int mimoburst, int *prev_frames)
{
    struct ath_buf *bf, *tbf, *bf_first, *bf_prev = NULL;
    ath_bufhead bf_head;
    int rl = 0, nframes = 0, ndelim;
    u_int16_t aggr_limit = 0, al = 0, bpad = 0, al_delta, h_baw = tid->baw_size/2;
    ATH_AGGR_STATUS status = ATH_AGGR_DONE;
    u_int32_t aggr_limit_with_rts = sc->sc_rtsaggrlimit;
    int prev_al = 0, is_ds_rate = 0;
    int is_ap = (sc->sc_opmode == HAL_M_HOSTAP);
    struct ath_node *an;

#ifdef ATH_RIFS
    if (sc->sc_txrifs) {
        rl = param->param_rl;
        aggr_limit = param->param_max_len;
    }
#endif

    bf_first = TAILQ_FIRST(&tid->buf_q);


    /*
     * Get the RTS aggr limit.
     */
    if (mimoburst) {
        DPRINTF(sc, ATH_DEBUG_PWR_SAVE, "%s:forming aggregate for burst\n",
            __func__);
        if (aggr_limit_with_rts) {
            prev_al = aggr_limit_with_rts;
        }
    }
    do {
        bf = TAILQ_FIRST(&tid->buf_q);

        /*
         * do not step over block-ack window
         */
        if (!BAW_WITHIN(tid->seq_start, tid->baw_size, bf->bf_seqno)) {
            status = ATH_AGGR_BAW_CLOSED;
            break;
        }

        if (!rl) {
            aggr_limit = ath_lookup_rate(sc, tid->an, bf, mimoburst);
            rl = 1;
            /*
             * Is rate dual stream
             */
            is_ds_rate = (bf->bf_rcs[0].flags & ATH_RC_DS_FLAG)? 1 : 0;
#ifdef ATH_RIFS
            if (sc->sc_txrifs) {
                param->param_rcs[0] = bf->bf_rcs[0];
                param->param_rcs[1] = bf->bf_rcs[1];
                param->param_rcs[2] = bf->bf_rcs[2];
                param->param_rcs[3] = bf->bf_rcs[3];
                param->param_max_len = aggr_limit;
                param->param_rl = rl;
            }
#endif
        }
 
        /*
         * do not exceed aggregation limit
         */
        al_delta = ATH_AGGR_DELIM_SZ + bf->bf_frmlen;

        /*
         * Check whether the packets are preceded with RTS in the case of
         * dynamic SM/MIMO power save. RTS needs to be applied in dual stream
         * rates only.
         */
        if (is_ap && !mimoburst && is_ds_rate) {
            an = bf_first->bf_node;

            if (an->an_smmode == ATH_SM_PWRSAV_DYNAMIC) {
                if (nframes &&
                (aggr_limit_with_rts < (al + bpad + al_delta))) {
                    status = ATH_AGGR_8K_LIMITED;
                    *prev_frames = nframes;
                    DPRINTF(sc, ATH_DEBUG_PWR_SAVE, "%s "
                        "AGGR_8K_LIMITED %d frames "
                        " al %d bpad %d al_delta %d\n", __func__,
                        nframes, al, bpad, al_delta);
                    break;
                }
            }
        }

        if (nframes && (aggr_limit < (al + bpad + al_delta + prev_al))) {
            status = ATH_AGGR_LIMITED;
            break;
        }

        /*
         * do not exceed subframe limit
         */
#ifdef ATH_RIFS
        if ((nframes +*prev_frames) >= MIN(h_baw, sc->sc_config.ampdu_subframes)
	    || (sc->sc_txrifs && ((nframes + *prev_frames) >=
			    param->param_max_frames)))
#else
        if ((nframes + *prev_frames) >=
		MIN(h_baw, sc->sc_config.ampdu_subframes))
#endif
        {
            status = ATH_AGGR_LIMITED;
            break;
        }

        /*
         * add padding for previous frame to aggregation length
         */
        al += bpad + al_delta;

        /*
         * Get the delimiters needed to meet the MPDU density for this node.
         */
        ndelim = ath_compute_num_delims(sc, bf_first, bf->bf_frmlen);

        bpad = PADBYTES(al_delta) + (ndelim << 2);

        bf->bf_next = NULL;
        bf->bf_lastfrm->bf_desc->ds_link = 0;

        /*
         * this packet is part of an aggregate
         * - remove all descriptors belonging to this frame from software queue
         * - add it to block ack window
         * - set up descriptors for aggregation
         */
        TAILQ_REMOVE_HEAD_UNTIL(&tid->buf_q, &bf_head, bf->bf_lastfrm, bf_list);
        ath_tx_addto_baw(sc, tid, bf);

        TAILQ_FOREACH(tbf, &bf_head, bf_list) {
            ath_hal_set11n_aggr_middle(sc->sc_ah, tbf->bf_desc, ndelim);
        }

#ifdef ATH_RIFS
        if (sc->sc_txrifs) {
            ATH_SET_TX_SET_NOACK_POLICY(sc, ATH_MPDU_2_QOS_WH(bf->bf_mpdu));
            TAILQ_FOREACH(tbf, &bf_head, bf_list) {
                ath_hal_set11n_aggr_rifs_burst(sc->sc_ah, tbf->bf_desc);
            }
        }
#endif

        /*
         * link buffers of this frame to the aggregate
         */
        TAILQ_CONCAT(bf_q, &bf_head, bf_list);
        nframes ++;

        if (bf_prev) {
            bf_prev->bf_next = bf;
            bf_prev->bf_lastfrm->bf_desc->ds_link = bf->bf_daddr;
        }
        bf_prev = bf;

#if AGGR_NOSHORT
        /*
         * terminate aggregation on a small packet boundary
         */
        if (bf->bf_frmlen < ATH_AGGR_MINPLEN) {
            status = ATH_AGGR_SHORTPKT;
            break;
        }
#endif
    } while (!TAILQ_EMPTY(&tid->buf_q));

    bf_first->bf_al = al;
    bf_first->bf_nframes = nframes;
    *bf_last = bf_prev;
    return status;
}

#ifdef ATH_RIFS

/*
 * ath_tx_get_rifsframe ()
 *
 * Purpose:
 *  - Get a buffer from the tid q. If we don't exceed the length limit,
 *    or overstep the BAW add this to the list of frames in the RIFS burst.
 * Locks:
 *  - none
 *
 */

static INLINE ATH_AGGR_STATUS
ath_tx_get_rifsframe(struct ath_softc *sc, struct ath_atx_tid *tid,
                     ath_bufhead *bf_q, struct ath_buf **bf_last,
                     struct aggr_rifs_param *param, int nframes)
{
    struct ath_buf *bf;
    ATH_AGGR_STATUS status = ATH_AGGR_DONE;
    ath_bufhead bf_head;

    bf = TAILQ_FIRST(&tid->buf_q);

    /*
     * do not step over block-ack window
     */
    if (!BAW_WITHIN(tid->seq_start, tid->baw_size, bf->bf_seqno)) {
        status = ATH_AGGR_BAW_CLOSED;
    } else {
        if (!param->param_rl) {
            param->param_max_len = ath_lookup_rate(sc, tid->an, bf, 0);
            param->param_rcs[0] = bf->bf_rcs[0];
            param->param_rcs[1] = bf->bf_rcs[1];
            param->param_rcs[2] = bf->bf_rcs[2];
            param->param_rcs[3] = bf->bf_rcs[3];         
            param->param_rl = 1;
        }

        /*
         * do not exceed length limit
         */
        param->param_al += bf->bf_frmlen;
        if (nframes && (param->param_al > param->param_max_len)) {
            return ATH_AGGR_LIMITED;
        }
 
        /*
         * this packet is part of an RIFS burst
         * - remove all descriptors belonging to this frame from software queue
         * - add it to block ack window
         */
        TAILQ_REMOVE_HEAD_UNTIL(&tid->buf_q, &bf_head,
                                bf->bf_lastfrm, bf_list);

        ATH_SET_TX_SET_NOACK_POLICY(sc, ATH_MPDU_2_QOS_WH(bf->bf_mpdu));

        ath_tx_addto_baw(sc, tid, bf);
        *bf_last = bf;

        TAILQ_CONCAT(bf_q, &bf_head, bf_list);
        bf->bf_nframes = 1;
    }
    return status;
}


/*
 * ath_tx_form_rifsburst ()
 *
 * Purpose:
 *  - Form a RIFS burst of aggregates or singles.
 * Locks:
 *  - none
 *
 */

static ATH_AGGR_STATUS
ath_tx_form_rifsburst(struct ath_softc *sc, struct ath_atx_tid *tid,
                      ath_bufhead *bf_q, struct ath_buf **bf_last_frame)
{
    int    h_baw = tid->baw_size/2, nframes = 0, nrifsubframes = 0;
    int    nframes_rifs_limit  = MIN(h_baw, sc->sc_config.ampdu_subframes),
           nframes_aggr_limit  = MIN(h_baw,
                  sc->sc_config.ampdu_subframes/sc->sc_config.rifs_ampdu_div);
    ATH_AGGR_STATUS status = ATH_AGGR_DONE;
    ath_bufhead bf_head;
    struct ath_buf *bf_last_sub, *bf_first = NULL, *bf_last = NULL,
                   *tbf, *bf_prev = NULL;
    struct ath_rc_series rcs[4];
    struct aggr_rifs_param param = {0, 0, 0, 0, NULL};
    int prev_frames = 0;

    param.param_rcs = rcs;

    do {
        TAILQ_INIT(&bf_head);

        if (sc->sc_txaggr) {
            /* Set the max # of frames we want in the aggregate in param */
            param.param_max_frames = MIN(nframes_aggr_limit,
                                         nframes_rifs_limit - nframes);
            /* Form the aggregate */
            status = ath_tx_form_aggr(sc, tid, &bf_head, &bf_last_sub,
                                      &param, 0, &prev_frames);
        } else {
            /* Get a single frame from the tid q */
            status = ath_tx_get_rifsframe(sc, tid, &bf_head, &bf_last_sub,
                                          &param, nframes);

            if (ATH_AGGR_LIMITED == status)
                break;
        }

        /*
         * no frames picked up to be bursted
         */
        if (TAILQ_EMPTY(&bf_head))
            break;

        bf_first = TAILQ_FIRST(&bf_head);
        bf_last = TAILQ_LAST(&bf_head, ath_bufhead_s);

        bf_first->bf_lastbf = bf_last;
        bf_last_sub->bf_next = NULL;
        bf_last->bf_desc->ds_link = 0;

        /*
         * Copy the rate series acquired from the first lookup into every
         * aggregate/single.
         */
        bf_first->bf_rcs[0] = rcs[0];
        bf_first->bf_rcs[1] = rcs[1];
        bf_first->bf_rcs[2] = rcs[2];
        bf_first->bf_rcs[3] = rcs[3];

        ath_buf_set_rate(sc, bf_first);

        /* Handle the single vs aggregate case when aggregation is enabled */
        if (sc->sc_txaggr) {
            if (bf_first->bf_nframes == 1) {
                TAILQ_FOREACH(tbf, &bf_head, bf_list) {
                    ath_hal_clr11n_aggr(sc->sc_ah, tbf->bf_desc);
                }
            } else {
                bf_first->bf_isaggr = 1;
                ath_hal_set11n_aggr_first(sc->sc_ah, bf_first->bf_desc,
                                          bf_first->bf_al);
                tbf = bf_last_sub;
                do {
                    ath_hal_set11n_aggr_last(sc->sc_ah, tbf->bf_desc);
                    tbf = TAILQ_NEXT(tbf, bf_list);
                } while (tbf != NULL);
            }
        }

        /*
         * Set More RIFS in every descriptor of the last frame in the
         * aggregate or in the lone frame in the non-aggregate case.
         */
        tbf = bf_last_sub;
        do {
            ath_hal_set11n_rifs_burst_middle(sc->sc_ah, tbf->bf_desc);
            tbf = TAILQ_NEXT(tbf, bf_list);
        } while (tbf != NULL);

        TAILQ_CONCAT(bf_q, &bf_head, bf_list);

        /* Link this sub-frame with the prior frames of the RIFS burst */
        if (bf_prev) {
            bf_prev->bf_next = bf_first;
            bf_prev->bf_lastfrm->bf_desc->ds_link = bf_first->bf_daddr;
        }
        bf_prev = bf_last_sub;

        /*
         * Count every aggregate or single frame as an element within the
         * RIFS burst.
         */
        nrifsubframes++;
        nframes += bf_first->bf_nframes;

    } while (!TAILQ_EMPTY(&tid->buf_q) && (nframes < nframes_rifs_limit) &&
             status != ATH_AGGR_BAW_CLOSED);

    /*
     * no frames picked up to be bursted
     */
    if (TAILQ_EMPTY(bf_q))
        return status;

    bf_first = TAILQ_FIRST(bf_q);
    bf_last_sub = *bf_last_frame = bf_prev;

    bf_first->bf_rifsburst_elem = ATH_RIFS_SUBFRAME_FIRST;
    bf_last_sub->bf_rifsburst_elem = ATH_RIFS_SUBFRAME_LAST;

    /* Clear the More RIFS bit on every descriptor of the last sub-frame */
    tbf = bf_last_sub;
    do {
        ath_hal_set11n_rifs_burst_last(sc->sc_ah, tbf->bf_desc);
        tbf = TAILQ_NEXT(tbf, bf_list);
    } while (tbf != NULL);

    bf_first->bf_nframes = nframes;
    bf_first->bf_nrifsubframes = nrifsubframes;

    return status;
}


/*
 * ath_rifsburst_bar_buf_alloc ()
 *
 * Purpose:
 *  - allocates wbuf
 *  - remove an ath_buf from the free buffer list
 * Locks:
 *  - acquires sc_txbuflock while manipulating the free buffer list
 *
 */

static struct ath_buf *
ath_rifsburst_bar_buf_alloc(struct ath_softc *sc)
{
    struct ath_buf *bf = NULL;
    wbuf_t wbuf;
    size_t wbuf_len = sizeof(struct ieee80211_frame_bar);

    /* XXX: allocating with type beacon since it provides single map/unmap */
    wbuf = wbuf_alloc(sc->sc_osdev, WBUF_TX_BEACON, wbuf_len);
    if (NULL == wbuf)
        return bf;

    ATH_TXBUF_LOCK(sc);

    if (!TAILQ_EMPTY(&sc->sc_txbuf)) {
        bf = TAILQ_FIRST(&sc->sc_txbuf);
        TAILQ_REMOVE(&sc->sc_txbuf, bf, bf_list);
    }

    ATH_TXBUF_UNLOCK(sc);

    if (NULL == bf) {
        wbuf_complete(wbuf);
        return bf;
    }

    ATH_TXBUF_RESET(bf);

    __11nstats(sc, txrifs_bar_alloc);
    bf->bf_mpdu = wbuf;
    bf->bf_frmlen = wbuf_len;

    return bf;
}


/*
 * ath_rifsburst_bar_buf_free ()
 *
 * Purpose:
 *  - unmap the wbuf associated with this ath_buf
 *  - free the wbuf associated with this ath_buf
 * Locks:
 *  -none
 *
 */

static void
ath_rifsburst_bar_buf_free(struct ath_softc *sc, struct ath_buf *bf)
{
    wbuf_t wbuf = (wbuf_t)bf->bf_mpdu;

    __11nstats(sc, txrifs_bar_freed);
    wbuf_unmap_single(sc->sc_osdev, wbuf, BUS_DMA_TODEVICE,
                      OS_GET_DMA_MEM_CONTEXT(bf, bf_dmacontext));
    wbuf_complete(wbuf);
}


/*
 * ath_rifsburst_bar_tx ()
 *
 * Purpose:
 *  - Setup the BAR frame for transmit
 * Locks:
 *  - none
 *
 */

static void
ath_rifsburst_bar_tx(struct ath_softc *sc, ath_atx_tid_t *tid,
                     struct ath_buf *bf, struct ath_buf *bf_last)
{
    wbuf_t                      wbuf = bf->bf_mpdu;
    struct ieee80211_frame_bar  *bar;
    struct ath_desc             *ds;
    u_int8_t                    minrate = 0x0b;
    HAL_11N_RATE_SERIES         series[4] = {{ 0 }};
    int                         i;


    bar = (struct ieee80211_frame_bar *) wbuf_header(wbuf);

    /* Inherit frame header fields from the last frame of the RIFS burst */
    OS_MEMCPY(bar, wbuf_header(bf_last->bf_mpdu), bf->bf_frmlen);

    /*
     * form the bar frame
     */
    bar->i_fc[1]  = IEEE80211_FC1_DIR_NODS;
    bar->i_fc[0]  = IEEE80211_FC0_VERSION_0 |
                    IEEE80211_FC0_TYPE_CTL  |
                    IEEE80211_FC0_SUBTYPE_BAR;
    bar->i_ctl    = (tid->tidno << IEEE80211_BAR_CTL_TID_S) |
                                   IEEE80211_BAR_CTL_COMBA;
    bar->i_seq    = htole16(tid->seq_start << IEEE80211_SEQ_SEQ_SHIFT);

    bf->bf_seqno  = tid->seq_start;
    bf->bf_tidno = bf_last->bf_tidno;
    bf->bf_node = bf_last->bf_node;
    bf->bf_isaggr  = 0;
    bf->bf_isbar  = bf->bf_isampdu = 0;
    bf->bf_lastbf = bf;
    bf->bf_rifsburst_elem = ATH_RIFS_BAR;
    bf->bf_desc->ds_link = 0;
    bf->bf_next = NULL;

    /*
     * setup buf
     */
    bf->bf_buf_addr = wbuf_map_single(sc->sc_osdev, wbuf, BUS_DMA_TODEVICE,
                                     OS_GET_DMA_MEM_CONTEXT(bf, bf_dmacontext));

    /*
     * setup desc
     */
    ds = bf->bf_desc;

    /* XXX this routine will be depracated, it should be eventually changed */
    ath_hal_setuptxdesc(sc->sc_ah
                        , ds
                        , bf->bf_frmlen + IEEE80211_CRC_LEN     /* pktlen   */
                        , 0                                     /* hdrlen   */
                        , HAL_PKT_TYPE_NORMAL                   /* atype    */
                        , MIN(sc->sc_curtxpow, 60)              /* tx power */
                        , minrate                               /* rate0    */
                        , 1                                     /* tries    */
                        , HAL_TXKEYIX_INVALID                   /* keyindex */
                        , 0                                     /* antenna  */
                        , HAL_TXDESC_INTREQ
                        | HAL_TXDESC_CLRDMASK                   /* flags    */
                        , 0, 0, 0, 0
                        , ATH_COMP_PROC_NO_COMP_NO_CCS
                        );

    ds->ds_data = bf->bf_buf_addr;
    ds->ds_link = 0;
    ath_hal_filltxdesc(sc->sc_ah, ds, bf->bf_frmlen, AH_TRUE, AH_TRUE, ds);

    for(i=0; i<4;i++) {
        series[i].Tries = bf_last->bf_rcs[i].tries;
        if(series[i].Tries) {
            series[i].Rate = minrate;
            series[i].ChSel = sc->sc_tx_chainmask;
        }
    }

    ath_hal_set11n_ratescenario(sc->sc_ah, ds, ds, 0, 0, 0,
                                series, 4, 0);
}

/*
 * process pending frames possibly doing RIFS bursting
 * NB: must be called with txq lock held
 */

static void
ath_tx_sched_rifs(struct ath_softc *sc, struct ath_txq *txq, ath_atx_tid_t *tid)
{
    struct ath_buf *bf, *tbf, *bf_last, *bf_last_rifs;
    struct ath_buf *bar_bf = NULL;
    ATH_AGGR_STATUS status;
    ath_bufhead bf_q;

    /* Loop through TID buffer queue looking for frames to schedule */
    do {

	/* If TID queue is empty, break */
        if (TAILQ_EMPTY(&tid->buf_q))
            break;

        /* If RIFS is configured and no BAR allocated, allocate a BAR */
        if (sc->sc_txrifs && (NULL == bar_bf)) {
            bar_bf = ath_rifsburst_bar_buf_alloc(sc);

            /* Return if there is no buf available for BAR */
            if (bar_bf == NULL) {
                return;
            }
        }

        TAILQ_INIT(&bf_q);

        /*
         * Insert into bf_q, one of the following:
         * 1. a RIFS burst
         * 2. a RIFS burst of aggregates
         */
        status = ath_tx_form_rifsburst(sc, tid, &bf_q, &bf_last_rifs);

        /*
         * no frames picked up to be bursted;
         * block-ack window is not open
         */
        if (TAILQ_EMPTY(&bf_q))
            break;

        bf = TAILQ_FIRST(&bf_q);
        bf_last = TAILQ_LAST(&bf_q, ath_bufhead_s);
        bf->bf_lastbf = bf_last;

        /*
         * if only one RIFS sub-frame, send
         */
        if (bf->bf_nrifsubframes == 1) {
            /* Clear No Ack policy set in Qos that is done blindly in
             * ath_tx_form_rifsburst().
             */
            tbf = TAILQ_FIRST(&bf_q);
            do {
                ATH_SET_TX_CLR_NOACK_POLICY(sc,
                                            ATH_MPDU_2_QOS_WH(tbf->bf_mpdu));
                tbf = tbf->bf_next;
            } while (tbf != NULL);

            TAILQ_FOREACH(tbf, &bf_q, bf_list) {
                bf->bf_rifsburst_elem = ATH_RIFS_NONE;
                ath_hal_clr11n_rifs_burst(sc->sc_ah, tbf->bf_desc);
            }

            ath_tx_txqaddbuf(sc, txq, &bf_q);
            continue;
        }

        /* If we have allocated a BAR, set it up leveraging information from
         * the last frame in bf_q, bf_last_rifs.
         */
        if (bar_bf && (bf->bf_nrifsubframes > 1)) {
            /* Setup the BAR */
            ath_rifsburst_bar_tx(sc, tid, bar_bf, bf_last_rifs);
            /* Set rifslast: used for rate completion, last ds in RIFS burst */
            bf->bf_rifslast = bf_last;
            /* Link the BAR to the RIFS burst */
            TAILQ_INSERT_TAIL(&bf_q, bar_bf, bf_list);
            bf_last->bf_desc->ds_link = bar_bf->bf_daddr;
            bf_last_rifs->bf_next = bar_bf;
            bf->bf_lastbf = bar_bf;
            /* Set BAR to NULL as to avoid freeing it below */
            bar_bf = NULL;
        }

        /*
         * queue burst to hardware
         */
        ath_tx_txqaddbuf(sc, txq, &bf_q);

    } while (txq->axq_depth < ATH_AGGR_MIN_QDEPTH &&
             status != ATH_AGGR_BAW_CLOSED);

    /*
     * Result of allocating a BAR and not tx'ing it.
     */
    if (bar_bf) {
        ath_rifsburst_bar_buf_free(sc, bar_bf);
        ATH_TXBUF_LOCK(sc);
        TAILQ_INSERT_TAIL(&sc->sc_txbuf, bar_bf, bf_list);
        ATH_TXBUF_UNLOCK(sc);
    }
}
#endif

static INLINE u_int32_t
ath_buf_get_ba_period(struct ath_softc *sc, struct ath_buf *bf)
{
	u_int32_t	ba_period;
	u_int8_t	rix, cix;
	const HAL_RATE_TABLE *rt;

	rt = sc->sc_currates;

	/*
	 * The second aggregate has only one rate, so use it to get
	 * the control rate.
	 */
	rix = bf->bf_rcs[0].rix;
	cix = rt->info[rix].controlRate;

	switch (rt->info[cix].rateCode) {
	default:
	case 0x0b: /* 6 Mb OFDM */
		ba_period = 68;
		break;
	case 0x0a: /* 12 Mb OFDM */
		ba_period = 44;
		break;
	case 0x09: /* 24 Mb OFDM */
		ba_period = 32;
		break;
	}

	return (ba_period);
}

/*
 * Do MIMO Bursting. Form two aggregates and hand-over to the hardware.
 */
static void
ath_tx_sched_mimoburst(struct ath_softc *sc, struct ath_txq *txq, 
    ath_atx_tid_t *tid, struct aggr_rifs_param *param, 
    ath_bufhead *bf_qfirst, int *prev_frames)
{
    int burst_duration = 0, status;
    struct ath_buf *bf_firstaggr, *bf, *bf_last, *tbf, *bf_lastaggr = NULL;
    ath_bufhead bf_q;

    bf_firstaggr = TAILQ_FIRST(bf_qfirst);
    TAILQ_INIT(&bf_q);

    do {
        if (TAILQ_EMPTY(&tid->buf_q))
            break;

        status = ath_tx_form_aggr(sc, tid, &bf_q, &bf_lastaggr, param,
             1, prev_frames);

        /*
         * no frames picked up to be aggregated; block-ack window is not open
         */
        if (TAILQ_EMPTY(&bf_q))
            break;

        bf = TAILQ_FIRST(&bf_q);
        bf_last = TAILQ_LAST(&bf_q, ath_bufhead_s);
        bf->bf_lastbf = bf_last;

        txq->axq_aggr_nbuf += bf->bf_nframes;
        bf->bf_isn = 1;
        /*
         * if only one frame, send as non-aggregate
         */
        if (bf->bf_nframes == 1) {
            ASSERT(bf->bf_lastfrm == bf_last);

            bf->bf_isaggr = 0;
            /*
             * clear aggr bits for every descriptor
             * XXX TODO: is there a way to optimize it?
             */
            TAILQ_FOREACH(tbf, &bf_q, bf_list) {
                ath_hal_clr11n_aggr(sc->sc_ah, tbf->bf_desc);
            }
        } else {
            /*
             * setup first desc with rate and aggr info
             */
            bf->bf_isaggr  = 1;
            ath_hal_set11n_aggr_first(sc->sc_ah, bf->bf_desc, bf->bf_al);

            /*
             * anchor last frame of aggregate correctly
             */
            ASSERT(bf_lastaggr);
            ASSERT(bf_lastaggr->bf_lastfrm == bf_last);
            tbf = bf_lastaggr;
            do {
                ath_hal_set11n_aggr_last(sc->sc_ah, tbf->bf_desc);
                tbf = TAILQ_NEXT(tbf, bf_list);
            } while (tbf != NULL);

            txq->axq_aggr_depth++;
        }

        /* Tag the second frame in the burst */
        bf->bf_aggrburst = 1;

        /*
         * 1 try, 1 rate
         */
        bf->bf_rcs[0].tries = 1;
        bf->bf_rcs[1].tries = bf->bf_rcs[2].tries = 
            bf->bf_rcs[3].tries = 0;
        bf->bf_rcs[1].rix = bf->bf_rcs[2].rix = bf->bf_rcs[3].rix = 0;

        /*
         * Remove RTS enable flags if present.
         */
        bf->bf_flags &= ~(HAL_TXDESC_RTSENA);

        ath_buf_set_rate(sc, bf);
        /*
         * Burst duration is calculated based on the duration of second aggr.
         */
        burst_duration = ath_pkt_duration(sc, bf->bf_rcs[0].rix, bf,
            (bf->bf_rcs[0].flags & ATH_RC_CW40_FLAG) != 0,
            (bf->bf_rcs[0].flags & ATH_RC_SGI_FLAG), bf->bf_shpreamble);

        /*
         * Protect the bar and SIFSs for the second aggr.
         */
#define    OFDM_SIFS_TIME    16
        burst_duration += ath_buf_get_ba_period(sc, bf) +
            2 * OFDM_SIFS_TIME;

        /*
         * Set the burst duration and vmf required in the first aggr.
         */
        ath_hal_set11n_burstduration(sc->sc_ah, bf_firstaggr->bf_desc, 
            burst_duration);
        ath_hal_set11n_virtualmorefrag(sc->sc_ah, \
                bf_firstaggr->bf_desc,1);
        bf->bf_aggrburst = 0;

    } while (0);

    /*
     * queue both aggregates to hardware
     */
    DPRINTF(sc, ATH_DEBUG_PWR_SAVE, "%s Queuing both the aggrs\n",
        __func__);

    ath_tx_txqaddbuf(sc, txq, bf_qfirst);
    if (!TAILQ_EMPTY(&bf_q))
        ath_tx_txqaddbuf(sc, txq, &bf_q);
}

/*
 * process pending frames possibly doing a-mpdu aggregation
 * NB: must be called with txq lock held
 */
static INLINE void
ath_tx_sched_aggr(struct ath_softc *sc, struct ath_txq *txq, ath_atx_tid_t *tid)
{
    struct ath_buf *bf, *tbf, *bf_last, *bf_lastaggr = NULL;
    ATH_AGGR_STATUS status;
    ath_bufhead bf_q;
    struct aggr_rifs_param param = {0, 0, 0, 0, NULL};
    int prev_frames = 0;

    do {
        if (TAILQ_EMPTY(&tid->buf_q))
            break;

        TAILQ_INIT(&bf_q);

        status = ath_tx_form_aggr(sc, tid, &bf_q, &bf_lastaggr, &param,
	    0, &prev_frames);

        /*
         * no frames picked up to be aggregated; block-ack window is not open
         */
        if (TAILQ_EMPTY(&bf_q))
            break;

        bf = TAILQ_FIRST(&bf_q);
        bf_last = TAILQ_LAST(&bf_q, ath_bufhead_s);
        bf->bf_lastbf = bf_last;

        txq->axq_aggr_nbuf += bf->bf_nframes;
        bf->bf_isn = 1;
        /*
         * if only one frame, send as non-aggregate
         */
        if (bf->bf_nframes == 1) {
            __11nstats(sc, txaggr_single);
            ASSERT(bf->bf_lastfrm == bf_last);

            bf->bf_isaggr = 0;
            /*
             * clear aggr bits for every descriptor
             * XXX TODO: is there a way to optimize it?
             */
            TAILQ_FOREACH(tbf, &bf_q, bf_list) {
                ath_hal_clr11n_aggr(sc->sc_ah, tbf->bf_desc);
            }

            ath_buf_set_rate(sc, bf);
            ath_tx_txqaddbuf(sc, txq, &bf_q);
            continue;
        }

        /*
         * setup first desc with rate and aggr info
         */
        bf->bf_isaggr  = 1;
        ath_hal_set11n_aggr_first(sc->sc_ah, bf->bf_desc, bf->bf_al);
        ath_buf_set_rate(sc, bf);

        __11nstats(sc,tx_aggregates);
        __11nstatsn(sc,tx_aggr_frames,bf->bf_nframes);

        /*
         * anchor last frame of aggregate correctly
         */
        ASSERT(bf_lastaggr);
        ASSERT(bf_lastaggr->bf_lastfrm == bf_last);
        tbf = bf_lastaggr;
        do {
            ath_hal_set11n_aggr_last(sc->sc_ah, tbf->bf_desc);
            tbf = TAILQ_NEXT(tbf, bf_list);
        } while (tbf != NULL);

        txq->axq_aggr_depth++;

        if (status == ATH_AGGR_8K_LIMITED) {
            /*
             * Mimo Burst, Queue aggregate in pairs
             */
            ath_tx_sched_mimoburst(sc, txq, tid, &param, &bf_q, &prev_frames);
            __11nstats(sc, txaggr_mimo);
        } else {
            /*
             * Normal aggregate, queue to hardware
             */
            ath_tx_txqaddbuf(sc, txq, &bf_q);
        }
    } while (
#ifdef ATH_SUPPORT_IQUE
             txq->axq_depth < min_qdepth_per_ac[TID_TO_WME_AC(tid->tidno)] &&
#else
             txq->axq_depth < ATH_AGGR_MIN_QDEPTH &&
#endif
             status != ATH_AGGR_BAW_CLOSED);
}

/*
 * Tx scheduling logic
 * NB: must be called with txq lock held
 */
void
ath_txq_schedule(struct ath_softc *sc, struct ath_txq *txq)
{
    struct ath_atx_ac *ac;
    struct ath_atx_tid *tid;

    /*
     * get the first node/ac pair on the queue
     */
    ac = TAILQ_FIRST(&txq->axq_acq);
    if (ac == NULL) {
        /* nothing to schedule */
        __11nstats(sc, tx_schednone);
        return;
    }

    TAILQ_REMOVE(&txq->axq_acq, ac, ac_qelem);
    ac->sched = AH_FALSE;

    /*
     * process a single tid per destination
     */
    do {
        tid = TAILQ_FIRST(&ac->tid_q);
        if (tid == NULL) {
            /* nothing to schedule */
            __11nstats(sc, tx_schednone);
            return;
        }
        TAILQ_REMOVE(&ac->tid_q, tid, tid_qelem);
        tid->sched = AH_FALSE;

        if (tid->paused)    /* check next tid to keep h/w busy */
            continue;

        /*
         * schedule rifs or aggregation for this tid
         */
#ifdef ATH_RIFS
        if (sc->sc_txrifs)
            ath_tx_sched_rifs(sc, txq, tid);
        else
#endif
        if (!(tid->an->an_smmode == ATH_SM_PWRSAV_DYNAMIC) ||
            ((txq->axq_depth % 2) == 0)) {
            ath_tx_sched_aggr(sc, txq, tid);
        }

        /*
         * add tid to round-robin queue if more frames are pending for the tid
         */
        if (!TAILQ_EMPTY(&tid->buf_q))
            ath_tx_queue_tid(txq, tid);

        /* only schedule one TID at a time */
        break;
    } while (!TAILQ_EMPTY(&ac->tid_q));

    /*
     * schedule AC if more TIDs need processing
     */
    if (!TAILQ_EMPTY(&ac->tid_q)) {
        /*
         * add dest ac to txq if not already added
         */
        if (ac->sched == AH_FALSE) {
            ac->sched = AH_TRUE;
            TAILQ_INSERT_TAIL(&txq->axq_acq, ac, ac_qelem);
        }
    }
}

static INLINE void
ath_tx_set_retry(struct ath_softc *sc, struct ath_buf *bf)
{
    wbuf_t wbuf;
    struct ieee80211_frame *wh;

    __11nstats(sc, tx_retries);

    bf->bf_isretried = 1;
    bf->bf_retries ++;

    wbuf = bf->bf_mpdu;
    wh = (struct ieee80211_frame *)wbuf_header(wbuf);
    wh->i_fc[1] |= IEEE80211_FC1_RETRY;
}

/*
 * Compute the number of bad frames
 */
static INLINE int
ath_tx_num_badfrms(struct ath_softc *sc, struct ath_buf *bf, int txok)
{
    struct ath_node *an = bf->bf_node;
    int isnodegone = (an->an_flags & ATH_NODE_CLEAN);
    struct ath_buf *bf_last = bf->bf_lastbf;
    struct ath_desc *ds = bf_last->bf_desc;
    u_int16_t seq_st = 0;
    u_int32_t ba[WME_BA_BMP_SIZE >> 5];
    int ba_index;
    int nbad = 0;
    int isaggr = 0;
#ifdef ATH_RIFS
    int isrifs = 0;
#endif

    if (isnodegone || ds->ds_txstat.ts_flags == HAL_TX_SW_ABORTED) 
        return 0;

    isaggr = bf->bf_isaggr;
#ifdef ATH_RIFS
    isrifs = (ATH_RIFS_SUBFRAME_FIRST == bf->bf_rifsburst_elem) ? 1:0;
    
    if (isaggr || isrifs) {
#else
    if (isaggr) {
#endif
        seq_st = ATH_DS_BA_SEQ(ds);
        OS_MEMCPY(ba, ATH_DS_BA_BITMAP(ds), WME_BA_BMP_SIZE >> 3);
    }

#ifdef ATH_RIFS 
    while (bf && bf->bf_rifsburst_elem != ATH_RIFS_BAR) {
#else
    while (bf) {
#endif
        ba_index = ATH_BA_INDEX(seq_st, bf->bf_seqno);
#ifdef ATH_RIFS
        if (!txok || ((isaggr || isrifs) && && !ATH_BA_ISSET(ba, ba_index))) {
#else
        if (!txok || (isaggr && !ATH_BA_ISSET(ba, ba_index))) {
#endif
            nbad++;
        }
        
        bf = bf->bf_next;
    }
    
    return nbad;
}    

/*
 * Completion routine of an aggregate
 */
static void 
ath_tx_complete_aggr_rifs(struct ath_softc *sc, struct ath_txq *txq, struct ath_buf *bf, ath_bufhead *bf_q, int txok)
{
#ifdef ATH_SUPPORT_IQUE
    struct ath_hal *ah = sc->sc_ah;
    struct ath_node *tan;
    struct ieee80211_node *node;
#endif
    struct ath_node *an = bf->bf_node;
    struct ath_atx_tid *tid = ATH_AN_2_TID(an, bf->bf_tidno);
    struct ath_buf *bf_last = bf->bf_lastbf;
    struct ath_desc *ds = bf_last->bf_desc;
    struct ath_buf *bf_next, *bf_lastq = NULL;
    ath_bufhead bf_head, bf_pending;
    u_int16_t seq_st = 0;
    u_int32_t ba[WME_BA_BMP_SIZE >> 5];
    int ba_index;
    int isaggr, txfail, txpending, sendbar = 0, needreset = 0;
    int isnodegone= (an->an_flags & ATH_NODE_CLEAN);

#ifdef ATH_RIFS
    int isrifs = 0;
    struct ath_buf *bar_bf = NULL;
#endif

    isaggr = bf->bf_isaggr;
#ifdef ATH_RIFS
    isrifs = (ATH_RIFS_SUBFRAME_FIRST == bf->bf_rifsburst_elem) ? 1:0;


    if (isrifs) {
        bar_bf = bf->bf_lastbf;
        ASSERT(ATH_RIFS_BAR == bar_bf->bf_rifsburst_elem);
    }

    if (isaggr || isrifs)
#else
    if (isaggr)
#endif
    {
#ifdef ATH_RIFS
        isrifs ? __11nstats(sc, tx_comprifs) : __11nstats(sc, tx_compaggr);
#else
        __11nstats(sc, tx_compaggr);
#endif
        if (txok) {
            if (ATH_DS_TX_BA(ds)) {
                /*
                 * extract starting sequence and block-ack bitmap
                 */
                seq_st = ATH_DS_BA_SEQ(ds);
                OS_MEMCPY(ba, ATH_DS_BA_BITMAP(ds), WME_BA_BMP_SIZE >> 3);
                ba_index = ATH_BA_INDEX(seq_st, bf->bf_seqno);
                if (ba_index >= WME_BA_BMP_SIZE) {
                    DPRINTF(sc, ATH_DEBUG_TX_PROC, "%s: Invalid bit map: seq_st=0x%x, bf_seqno=0x%x\n", 
                            __func__, seq_st, bf->bf_seqno);
                    needreset = 1;
                }
            } else {
#ifdef ATH_RIFS
                isrifs ? __11nstats(sc, txrifs_babug) :
                         __11nstats(sc, txaggr_babug);
#else
                __11nstats(sc, txaggr_babug);
#endif
                OS_MEMZERO(ba, WME_BA_BMP_SIZE >> 3);
                DPRINTF(sc, ATH_DEBUG_TX_PROC, "%s: BA bit not set.\n", __func__);

                /*
                 * Owl can become deaf/mute when BA bug happens.
                 * Chip needs to be reset. See bug 32789.
                 */
                needreset = 1;
            }
        } else {
            OS_MEMZERO(ba, WME_BA_BMP_SIZE >> 3);
        }
    }

    TAILQ_INIT(&bf_pending);

#ifdef ATH_RIFS 
    while (bf && bf->bf_rifsburst_elem != ATH_RIFS_BAR)
#else
    while (bf)
#endif
    {
        txfail = txpending = 0;
        bf_next = bf->bf_next;

        if (ATH_BA_ISSET(ba, ATH_BA_INDEX(seq_st, bf->bf_seqno))) {
            /*
             * transmit completion, subframe is acked by block ack
             */
#ifdef ATH_RIFS
            isrifs ? __11nstats(sc, txrifs_compgood) :
                     __11nstats(sc, txaggr_compgood);
#else
            __11nstats(sc, txaggr_compgood);
#endif
        }
#ifdef ATH_RIFS
        else if ((!isaggr && !isrifs) && txok) {
#else
        else if (!isaggr && txok) {
#endif
            /*
             * transmit completion
             */
#ifdef ATH_RIFS
            isrifs ? __11nstats(sc, tx_compnorifs) :
                     __11nstats(sc, tx_compunaggr);
#else
            __11nstats(sc, tx_compunaggr);
#endif
        } else {
            /*
             * retry the un-acked ones
             */
            if (ds->ds_txstat.ts_flags & HAL_TXERR_XRETRY) {
                __11nstats(sc,tx_sf_hw_xretries);
            }

#ifdef ATH_RIFS
            isrifs ? __11nstats(sc, txrifs_compretries) :
                     __11nstats(sc, txaggr_compretries);
#else
            __11nstats(sc, txaggr_compretries);
#endif

#ifdef ATH_SUPPORT_IQUE
            /* For the frames to be droped who block the headline of the AC_VI queue,
             * these frames should not be sw-retried. So mark them as already xretried.
             */
            tan = ATH_NODE(bf->bf_node);
            node = (struct ieee80211_node *)tan->an_node;
            if (node->ni_hbr_block && 
                            TID_TO_WME_AC(bf->bf_tidno) == WME_AC_VI) {
                bf->bf_retries = ATH_MAX_SW_RETRIES;
            }
#endif            
            
            if (!tid->cleanup_inprogress && !isnodegone &&
                ds->ds_txstat.ts_flags != HAL_TX_SW_ABORTED) {
                if (bf->bf_retries < ATH_MAX_SW_RETRIES) {
                    ath_tx_set_retry(sc, bf);

#ifdef ATH_SUPPORT_IQUE
					bf->bf_txduration += ath_hal_txcalcairtime(ah, ds);
					if (bf->bf_txduration >= sc->sc_retry_duration &&
						sc->sc_retry_duration > 0)
					{
	                    __11nstats(sc, tx_xretries);
	                    bf->bf_isxretried = 1;
	                    txfail = 1;
	                    sendbar = 1;
					} else {
                    	txpending = 1;
					}
#else
                    txpending = 1;
#endif
                } else {
                    __11nstats(sc, tx_xretries);
                    bf->bf_isxretried = 1;
                    txfail = 1;
                    sendbar = 1;
                }
            } else {
                /*
                 * the entire aggregate is aborted by software due to
                 * reset, channel change, node left and etc.
                 */
                if (ds->ds_txstat.ts_flags == HAL_TX_SW_ABORTED) {
                    __11nstats(sc, txaggr_comperror); 
                }

                /*
                 * cleanup in progress, just fail
                 * the un-acked sub-frames
                 */
                txfail = 1;
            }
        }

        /*
         * Remove ath_buf's of this sub-frame from aggregate queue.
         */
        if (bf_next == NULL) {  /* last subframe in the aggregate */
            ASSERT(bf->bf_lastfrm == bf_last);

            /*
             * The last descriptor of the last sub frame could be a holding descriptor
             * for h/w. If that's the case, bf->bf_lastfrm won't be in the bf_q.
             * Make sure we handle bf_q properly here.
             */
            bf_lastq = TAILQ_LAST(bf_q, ath_bufhead_s);
            if (bf_lastq) {
                TAILQ_REMOVE_HEAD_UNTIL(bf_q, &bf_head, bf_lastq, bf_list);
            } else {
                /*
                 * XXX: if the last subframe only has one descriptor which is also being used as
                 * a holding descriptor. Then the ath_buf is not in the bf_q at all.
                 */
                ASSERT(TAILQ_EMPTY(bf_q));
                TAILQ_INIT(&bf_head);
            }
        } else {
            ASSERT(!TAILQ_EMPTY(bf_q));
            TAILQ_REMOVE_HEAD_UNTIL(bf_q, &bf_head, bf->bf_lastfrm, bf_list);
        }
        
#ifndef REMOVE_PKT_LOG
        /* do pktlog */
        {
            struct log_tx log_data;
            struct ath_buf *tbf;

            TAILQ_FOREACH(tbf, &bf_head, bf_list) {
                log_data.firstds = tbf->bf_desc;
                log_data.bf = tbf;
                ath_log_txctl(sc, &log_data, 0);
            }

            if (bf->bf_next == NULL &&
                bf_last->bf_status & ATH_BUFSTATUS_STALE) {
                log_data.firstds = bf_last->bf_desc;
                log_data.bf = bf_last;
                ath_log_txctl(sc, &log_data, 0);
            }
        }
#endif

        if (!txpending) {
            /*
             * complete the acked-ones/xretried ones; update block-ack window
             */
            ATH_TXQ_LOCK(txq);
            ath_tx_update_baw(sc, tid, bf->bf_seqno);
            if ((isnodegone) && (tid->cleanup_inprogress)) {
                if (tid->baw_head == tid->baw_tail) {
                    tid->addba_exchangecomplete = 0;
                    tid->addba_exchangeattempts = 0;
                    tid->addba_exchangestatuscode = IEEE80211_STATUS_UNSPECIFIED;
                    /* resume the tid */
                    tid->paused--;
                    __11nstats(sc, tx_tidresumed);
                    tid->cleanup_inprogress = AH_FALSE;
                }
            }
            ATH_TXQ_UNLOCK(txq);

            /* complete this sub-frame */
            ath_tx_complete_buf(sc, bf, &bf_head, !txfail);
        } else {
            /*
             * retry the un-acked ones
             */
            /*
             * XXX: if the last descriptor is holding descriptor, in order to requeue
             * the frame to software queue, we need to allocate a new descriptor and
             * copy the content of holding descriptor to it.
             */
            if (bf->bf_next == NULL &&
                bf_last->bf_status & ATH_BUFSTATUS_STALE) {
                struct ath_buf *tbf;

                /* allocate new descriptor */
                ATH_TXBUF_LOCK(sc);
                tbf = TAILQ_FIRST(&sc->sc_txbuf);
                if (tbf == NULL) {
                    /*
                     * We are short on memory, release the wbuf
                     * and bail out.
                     * Complete the packet with status *Not* OK.
                     */
                    wbuf_t wbuf = bf->bf_mpdu;
                    ieee80211_tx_status_t tx_status;
                    
                    ATH_TXQ_LOCK(txq);
                    ath_tx_update_baw(sc, tid, bf->bf_seqno);
                    ATH_TXQ_UNLOCK(txq);

                    ASSERT(wbuf != NULL);  

#ifdef ATH_SWRETRY
                    if (bf->bf_isswretry)
                        tx_status.retries = bf->bf_totaltries;
                    else
                        tx_status.retries = bf->bf_retries;
#else
                    tx_status.retries = bf->bf_retries;
#endif
                    tx_status.flags |= ATH_TX_ERROR;

                    if (bf->bf_isxretried) {
                        tx_status.flags |= ATH_TX_XRETRY;
                    }
                    /* Unmap this frame */
                    wbuf_unmap_sg(sc->sc_osdev, wbuf,
                        OS_GET_DMA_MEM_CONTEXT(bf, bf_dmacontext));

                    sc->sc_ieee_ops->tx_complete(wbuf, &tx_status); /* complete this frame */
                    bf->bf_mpdu = NULL;

                    ATH_TXBUF_UNLOCK(sc);
                    // At this point, bf_next is NULL: We are done with this aggregate.
                    break;
                }
                TAILQ_REMOVE(&sc->sc_txbuf, tbf, bf_list);
                ATH_TXBUF_UNLOCK(sc);

                ATH_TXBUF_RESET(tbf);
                
                /* copy descriptor content */
                tbf->bf_mpdu = bf_last->bf_mpdu;
                tbf->bf_node = bf_last->bf_node;
                tbf->bf_buf_addr = bf_last->bf_buf_addr;
                *(tbf->bf_desc) = *(bf_last->bf_desc);

                /* link it to the frame */
                if (bf_lastq) {
                    bf_lastq->bf_desc->ds_link = tbf->bf_daddr;
                    bf->bf_lastfrm = tbf;
                    ath_hal_cleartxdesc(sc->sc_ah, bf->bf_lastfrm->bf_desc);
                } else {
                    tbf->bf_state = bf_last->bf_state;
                    tbf->bf_lastfrm = tbf;
                    ath_hal_cleartxdesc(sc->sc_ah, tbf->bf_lastfrm->bf_desc);

                    /* copy the DMA context */
                    OS_COPY_DMA_MEM_CONTEXT(OS_GET_DMA_MEM_CONTEXT(tbf, bf_dmacontext),
                                            OS_GET_DMA_MEM_CONTEXT(bf_last, bf_dmacontext));
                }
                TAILQ_INSERT_TAIL(&bf_head, tbf, bf_list);
            } else {
                /*
                 * Clear descriptor status words for software retry
                 */
                ath_hal_cleartxdesc(sc->sc_ah, bf->bf_lastfrm->bf_desc);
            }

            /*
             * Put this buffer to the temporary pending queue to retain ordering
             */
            TAILQ_CONCAT(&bf_pending, &bf_head, bf_list);
        }

        bf = bf_next;
    }

    /*
     * node is already gone. no more assocication
     * with the node. the node might have been freed
     * any  node acces can result in panic.note tid
     * is part of the node. 
     */  
    if (isnodegone) return;

    if (tid->cleanup_inprogress) {
        /* check to see if we're done with cleaning the h/w queue */
        ATH_TXQ_LOCK(txq);

        if (tid->baw_head == tid->baw_tail) {
            tid->addba_exchangecomplete = 0;
            tid->addba_exchangeattempts = 0;
            tid->addba_exchangestatuscode = IEEE80211_STATUS_UNSPECIFIED;
            ATH_TXQ_UNLOCK(txq);
            
            tid->cleanup_inprogress = AH_FALSE;
 
            /* send buffered frames as singles */
            ath_tx_flush_tid(sc, tid);
        } else {
            ATH_TXQ_UNLOCK(txq);
        }

        return;
    }
    
    if (sendbar)
        ath_bar_tx(sc, an, tid);

#ifdef ATH_RIFS
    if (isrifs)
        ath_rifsburst_bar_buf_free(sc, bar_bf);
#endif
    /*
     * prepend un-acked frames to the beginning of the pending frame queue
     */
    if (!TAILQ_EMPTY(&bf_pending)) {
#ifdef ATH_RIFS
        isrifs ? __11nstats(sc, txrifs_prepends) :
                 __11nstats(sc, txaggr_prepends);
#else
        __11nstats(sc, txaggr_prepends);
#endif
        
        ATH_TXQ_LOCK(txq);
        TAILQ_INSERTQ_HEAD(&tid->buf_q, &bf_pending, bf_list);
        ath_tx_queue_tid(txq, tid);
        ATH_TXQ_UNLOCK(txq);
    }

    if (needreset) {
        /*
         * AP code may have sychronization issues
         * when perform internal reset in this routine.
         * Only enable reset in STA mode for now.
         */
        if (sc->sc_opmode == HAL_M_STA)
            ath_internal_reset(sc);
    }

    return;
}


/* This function clear buffers in software queue for this tid */
static void
ath_tid_swq_cleanup(struct ath_softc *sc, struct ath_txq *txq, struct ath_atx_tid *tid)
{
    struct ath_buf *bf;
    ath_bufhead bf_head;

    for (;;) {
        bf = TAILQ_FIRST(&tid->buf_q);
        if (bf == NULL)
            break;

        TAILQ_REMOVE_HEAD_UNTIL(&tid->buf_q, &bf_head, bf->bf_lastfrm, bf_list);

        /* update baw for software retried frame */
        if (bf->bf_isretried)
            ath_tx_update_baw(sc, tid, bf->bf_seqno);

        /* do not indicate packets while holding txq spinlock */
        ATH_TXQ_UNLOCK(txq);

        /* complete this sub-frame */
        ath_tx_complete_buf(sc, bf, &bf_head, 0);

        ATH_TXQ_LOCK(txq);

        __11nstats(sc, tx_drain_bufs);
    }

}

/* This function is called in the process of chip reset - and
 * the assumption is all the buffers in the HW queue are
 * removed already.
 * Called with txq lock held */
static void
ath_tid_drain(struct ath_softc *sc, struct ath_txq *txq, struct ath_atx_tid *tid)
{

    __11nstats(sc, tx_drain_tid);

    ath_tid_swq_cleanup(sc, txq, tid);
    /*
     * TODO: For frame(s) that are in the retry state, we will reuse the 
     * sequence number(s) without setting the retry bit. The alternative is to
     * give up on these and BAR the receiver's window forward.
     */
    tid->seq_next = tid->seq_start;
    tid->baw_tail = tid->baw_head;
}


/* This function is called in the process of node cleanup.
 * Need to check for pending buffers in the HW queue.
 */
static void
ath_tid_cleanup(struct ath_softc *sc, struct ath_txq *txq, struct ath_atx_tid *tid)
{
    __11nstats(sc, tx_cleanup_tid);

    ath_tid_swq_cleanup(sc, txq, tid);

    if (tid->cleanup_inprogress)
        return;
    /* Check for pending packets in HW queue */
    if (tid->baw_head != tid->baw_tail) {
        /* Frames in HW queue */
        /* Pause the tid and set cleanup in progress to True */
        tid->paused++;
        __11nstats(sc, tx_tidpaused);
        tid->cleanup_inprogress = AH_TRUE;
    } else {
        tid->addba_exchangecomplete = 0;
        tid->addba_exchangeattempts = 0;
        tid->addba_exchangestatuscode = IEEE80211_STATUS_UNSPECIFIED;
    }
}

/*
 * Drain all pending buffers
 * NB: must be called with txq lock held
 */
static void
ath_txq_drain_pending_buffers(struct ath_softc *sc, struct ath_txq *txq)
{
    struct ath_atx_ac *ac;
    struct ath_atx_tid *tid;
    
    __11nstats(sc,tx_resetq);
    while ((ac = TAILQ_FIRST(&txq->axq_acq)) != NULL) {
        TAILQ_REMOVE(&txq->axq_acq, ac, ac_qelem);
        ac->sched = AH_FALSE;

        while ((tid = TAILQ_FIRST(&ac->tid_q)) != NULL) {
            TAILQ_REMOVE(&ac->tid_q, tid, tid_qelem);
            tid->sched = AH_FALSE;

            ath_tid_drain(sc, txq, tid);
        }
    }
}

/*
 * Initialize per-node transmit state
 */
void
ath_tx_node_init(struct ath_softc *sc, struct ath_node *an)
{
#ifdef ATH_RIFS
    if (sc->sc_txaggr || sc->sc_txrifs) {
#else
    if (sc->sc_txaggr) {
#endif
        struct ath_atx_tid *tid;
        struct ath_atx_ac *ac;
        int tidno, acno;

        an->an_aggr.tx.maxampdu = sc->sc_config.ampdu_limit;

        /*
         * Init per tid tx state
         */
        for (tidno = 0, tid = &an->an_tx_tid[tidno]; tidno < WME_NUM_TID;
             tidno++, tid++) {
            tid->an        = an;
            tid->tidno     = tidno;
            tid->seq_start = tid->seq_next = 0;
            tid->baw_size  = WME_MAX_BA;
            tid->baw_head  = tid->baw_tail = 0;
            tid->sched     = AH_FALSE;
            tid->filtered  = AH_FALSE;
            tid->paused = AH_FALSE;
            tid->cleanup_inprogress = AH_FALSE;
            TAILQ_INIT(&tid->buf_q);

            acno = TID_TO_WME_AC(tidno);
            tid->ac = &an->an_tx_ac[acno];

            ath_initialize_timer(sc->sc_osdev, &tid->addba_requesttimer, ADDBA_TIMEOUT,
                                 ath_addba_timer, tid);

            /* ADDBA state */
            tid->addba_exchangecomplete     = 0;
            tid->addba_exchangeinprogress   = 0;
            tid->addba_exchangeattempts     = 0;
            tid->addba_exchangestatuscode   = IEEE80211_STATUS_UNSPECIFIED;
        }

        /*
         * Init per ac tx state
         */
        for (acno = 0, ac = &an->an_tx_ac[acno]; acno < WME_NUM_AC; acno++, ac++) {
            ac->sched    = AH_FALSE;
            ac->filtered = AH_FALSE;
            ac->hwqcnt   = 0;
            TAILQ_INIT(&ac->tid_q);

            switch(acno) {
            case WME_AC_BE:
                ac->qnum = ath_tx_get_qnum(sc, HAL_TX_QUEUE_DATA, HAL_WME_AC_BE);
                break;
            case WME_AC_BK:
                ac->qnum = ath_tx_get_qnum(sc, HAL_TX_QUEUE_DATA, HAL_WME_AC_BK);
                break;
            case WME_AC_VI:
                ac->qnum = ath_tx_get_qnum(sc, HAL_TX_QUEUE_DATA, HAL_WME_AC_VI);
                break;
            case WME_AC_VO:
                ac->qnum = ath_tx_get_qnum(sc, HAL_TX_QUEUE_DATA, HAL_WME_AC_VO);
                break;
            }
        }
    }
}

/*
 * Cleanupthe pending buffers for the node. 
 */
void
ath_tx_node_cleanup(struct ath_softc *sc, struct ath_node *an)
{
    int i;
    struct ath_atx_ac *ac;
    struct ath_atx_tid *tid;
    struct ath_txq *txq;
    for (i = 0; i < HAL_NUM_TX_QUEUES; i++) {
        if (ATH_TXQ_SETUP(sc, i)) {
            txq = &sc->sc_txq[i];
            ATH_TXQ_LOCK(txq);

            TAILQ_FOREACH(ac,&txq->axq_acq,ac_qelem) {
                tid = TAILQ_FIRST(&ac->tid_q);
                if (tid && tid->an != an) {
                    continue;
                }
                TAILQ_REMOVE(&txq->axq_acq, ac, ac_qelem);
                ac->sched = AH_FALSE;

                while ((tid = TAILQ_FIRST(&ac->tid_q)) != NULL) {
                    TAILQ_REMOVE(&ac->tid_q, tid, tid_qelem);
                    tid->sched = AH_FALSE;
                    /* stop ADDBA request timer (if ADDBA in progress) */
                    if (cmpxchg(&tid->addba_exchangeinprogress, 1, 0) == 1) {
                        ath_cancel_timer(&tid->addba_requesttimer, CANCEL_NO_SLEEP);
                        /* Tid is paused - resume the tid */
                        tid->paused--;
                        __11nstats(sc, tx_tidresumed);
                    }
                    ath_tid_cleanup(sc, txq, tid);
                }
            }
            ATH_TXQ_UNLOCK(txq);
        }
    }
}

/*
 * Cleanup per node transmit state
 */
void
ath_tx_node_free(struct ath_softc *sc, struct ath_node *an)
{
#ifdef ATH_RIFS
    if (sc->sc_txaggr || sc->sc_txrifs)
#else
    if (sc->sc_txaggr)
#endif
    {
        struct ath_atx_tid *tid;
        int tidno, i;

        /* Init per tid rx state */
        for (tidno = 0, tid = &an->an_tx_tid[tidno]; tidno < WME_NUM_TID;
             tidno++, tid++) {

            /* better safe than sorry */
            ath_cancel_timer(&tid->addba_requesttimer, CANCEL_NO_SLEEP);

            for (i = 0; i < ATH_TID_MAX_BUFS; i++){
#ifdef NODE_FREE_DEBUG
                if (tid->tx_buf[i] != NULL) {
                    printk("*********************\n");
                    printk("****ASSERTION HIT****\n");
                    printk("MacAddr=%s\n",
                    ether_sprintf(sc->sc_ieee_ops->get_macaddr(an->an_node)));
                    printk("TxBufIdx=%d\n", i);
                    printk("Tid=%d\n", tidno);
                    printk("AthBuf=%p\n", tid->tx_buf[i]);
                    sc->sc_ieee_ops->node_print(an->an_node);
                }
#else
                //ASSERT(tid->tx_buf[i] == NULL);
#endif
                
            }
        }
    }
}

void
ath_tx_node_pause(struct ath_softc *sc, struct ath_node *an)
{
    int tidno;
    struct ath_atx_tid *tid;

#ifdef ATH_RIFS
    if (!sc->sc_txaggr && !sc->sc_txrifs)
#else
    if (!sc->sc_txaggr)
#endif
        return;

    for (tidno = 0, tid = &an->an_tx_tid[tidno]; tidno < WME_NUM_TID;
             tidno++, tid++) {
        ath_tx_pause_tid(sc,tid);
    }
}

void
ath_tx_node_resume(struct ath_softc *sc, struct ath_node *an)
{
    int tidno;
    struct ath_atx_tid *tid;

#ifdef ATH_RIFS
    if (!sc->sc_txaggr && !sc->sc_txrifs)
#else
    if (!sc->sc_txaggr)
#endif
        return;

    for (tidno = 0, tid = &an->an_tx_tid[tidno]; tidno < WME_NUM_TID;
         tidno++, tid++) {
        if (tid->addba_exchangecomplete) {
            ath_tx_resume_tid(sc,tid);
        } else {
            ath_tx_flush_tid(sc,tid);
        }
    }
}
void
ath_txto_tasklet(struct ath_softc *sc)
{
    DPRINTF(sc, ATH_DEBUG_XMIT, "%s\n", __func__);

#if 0
    /* Notify CWM */
    ath_cwm_txtimeout(sc);
#endif
}

void
ath_set_ampduparams(ath_dev_t dev, ath_node_t node, u_int16_t maxampdu,
                   u_int32_t mpdudensity)
{
    ATH_NODE(node)->an_aggr.tx.maxampdu = maxampdu;
    ATH_NODE(node)->an_aggr.tx.mpdudensity = mpdudensity;
}

void
ath_set_weptkip_rxdelim(ath_dev_t dev, ath_node_t node, u_int8_t rxdelim)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    u_int32_t txdelim = 0;

    /* 
     * Delimiter count for WEP/TKIP is the maximum of 
     * the delim count required by the receiver
     * and the delim count required by the device for transmitting.
     */
    (void)ath_hal_gettxdelimweptkipaggr(sc->sc_ah, &txdelim);
    ATH_NODE(node)->an_aggr.tx.weptkipdelim = MAX(rxdelim, txdelim);
}

void
ath_enq_legacy_buf(struct ath_softc *sc, struct ath_txq *txq, struct ath_buf *bf)
{
    ATH_TXQ_LL_LOCK(txq);
    TAILQ_INSERT_TAIL(&txq->axq_ll, bf, bf_list);
    txq->axq_ll_count ++;
    ATH_TXQ_LL_UNLOCK(txq);
}

void
ath_deq_legacy_buf(struct ath_softc *sc, struct ath_txq *txq)
{
    int                 i;
    ath_bufhead         bf_head;
    struct ath_buf      *bf = NULL;

    for (i = 0;i < 7; i++)
    {

        ATH_TXQ_LL_LOCK(txq);
        bf = TAILQ_FIRST(&txq->axq_ll);
        if (!bf) {
            ATH_TXQ_LL_UNLOCK(txq);
            return;
        }
        TAILQ_REMOVE(&txq->axq_ll, bf, bf_list);
        txq->axq_ll_count --;
        ATH_TXQ_LL_UNLOCK(txq);

        TAILQ_INIT(&bf_head);
        TAILQ_INSERT_TAIL(&bf_head, bf, bf_list);
        ath_tx_txqaddbuf(sc, txq, &bf_head);
    }

    return;
}

void
ath_drain_legacy_buf(struct ath_softc *sc, struct ath_txq *txq)
{
    ath_bufhead         bf_head;
    struct ath_buf      *bf = NULL;

    for (;;)
    {

        ATH_TXQ_LL_LOCK(txq);
        bf = TAILQ_FIRST(&txq->axq_ll);
        if (!bf) {
            ATH_TXQ_LL_UNLOCK(txq);
            return;
        }
        TAILQ_REMOVE(&txq->axq_ll, bf, bf_list);
        txq->axq_ll_count --;
        ATH_TXQ_LL_UNLOCK(txq);

        TAILQ_INIT(&bf_head);
        TAILQ_INSERT_TAIL(&bf_head, bf, bf_list);
        ath_tx_complete_buf(sc, bf, &bf_head, 0);
    }

    return;
}

static OS_TIMER_FUNC(ath_tx_legacy)
{
    struct ath_softc    *sc;
    int i;

    OS_GET_TIMER_ARG(sc, struct ath_softc *);

    for (i = 0; i < HAL_NUM_TX_QUEUES; i++) {
        if (ATH_TXQ_SETUP(sc, i)) {
            ath_deq_legacy_buf(sc, &sc->sc_txq[i]);
        }
    }

    OS_SET_TIMER(&sc->sc_tx_leg, 200);
}

