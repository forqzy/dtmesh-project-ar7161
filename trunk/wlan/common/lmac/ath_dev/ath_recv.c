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
 *  Implementation of receive path in atheros OS-independent layer.
 */

#include "ath_internal.h"
#include "ath_antdiv.h"

#ifndef REMOVE_PKT_LOG
#include "pktlog.h"
extern struct ath_pktlog_funcs *g_pktlog_funcs;
#endif

#ifdef ATH_SUPPORT_DFS
#include "dfs.h"
#endif

static int ath_ampdu_input(struct ath_softc *sc, struct ath_node *an, wbuf_t wbuf, ieee80211_rx_status_t *rx_status);
static u_int8_t ath_rx_detect_antenna(struct ath_softc *sc, struct ath_rx_status *rxstat);
static void ath_rx_flush_tid(struct ath_softc *sc, struct ath_arx_tid *rxtid, int drop);

int
ath_rx_init(ath_dev_t dev, int nbufs)
{
	struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    wbuf_t wbuf;
    struct ath_buf *bf;
    int error = 0;

    do {
        ATH_RXFLUSH_LOCK_INIT(sc);
        sc->sc_rxflush = 0;
        ATH_RXBUF_LOCK_INIT(sc);

        /*
         * Cisco's VPN software requires that drivers be able to
         * receive encapsulated frames that are larger than the MTU.
         * Since we can't be sure how large a frame we'll get, setup
         * to handle the larges on possible.
         */
        sc->sc_rxbufsize = IEEE80211_MAX_MPDU_LEN;

        DPRINTF(sc,ATH_DEBUG_RESET, "%s: cachelsz %u rxbufsize %u\n",
                __func__, sc->sc_cachelsz, sc->sc_rxbufsize);

        /*
         * Initialize rx descriptors
         */
        error = ath_descdma_setup(sc, &sc->sc_rxdma, &sc->sc_rxbuf,
                                  "rx", nbufs, 1);
        if (error != 0) {
            printk("failed to allocate rx descriptors: %d\n", error);
            break;
        }

        /*
         * Pre-allocate a wbuf for each rx buffer
         */
        TAILQ_FOREACH(bf, &sc->sc_rxbuf, bf_list) {
            wbuf = ath_rxbuf_alloc(sc, sc->sc_rxbufsize);
            if (wbuf == NULL) {
                error = -ENOMEM;
                break;
            }

            bf->bf_mpdu = wbuf;
            bf->bf_buf_addr = wbuf_map_single(sc->sc_osdev, wbuf, BUS_DMA_FROMDEVICE,
                                              OS_GET_DMA_MEM_CONTEXT(bf, bf_dmacontext));
            ATH_RX_CONTEXT(wbuf)->ctx_rxbuf = bf;
        }
        sc->sc_rxlink = NULL;

    } while (0);

    if (error) {
        ath_rx_cleanup(sc);
    }

    return error;
}

/*
 * Reclaim all rx queue resources
 */
void
ath_rx_cleanup(ath_dev_t dev)
{
	struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    wbuf_t wbuf;
    struct ath_buf *bf;
    
    TAILQ_FOREACH(bf, &sc->sc_rxbuf, bf_list) {
        wbuf = bf->bf_mpdu;
        if (wbuf)
            wbuf_release(sc->sc_osdev, WBUF_RX, wbuf);
    }
    
    /* cleanup rx descriptors */
    if (sc->sc_rxdma.dd_desc_len != 0)
        ath_descdma_cleanup(sc, &sc->sc_rxdma, &sc->sc_rxbuf);

    ATH_RXBUF_LOCK_DESTROY(sc);
    ATH_RXFLUSH_LOCK_DESTROY(sc);
}

/*
 * Setup and link descriptors.
 * 
 * 11N: we can no longer afford to self link the last descriptor.
 * MAC acknowledges BA status as long as it copies frames to host
 * buffer (or rx fifo). This can incorrectly acknowledge packets
 * to a sender if last desc is self-linked.
 * 
 * NOTE: Caller should hold the rxbuf lock.
 */
static void
ath_rx_buf_link(struct ath_softc *sc, struct ath_buf *bf)
{
    struct ath_hal *ah = sc->sc_ah;
    struct ath_desc *ds;
    wbuf_t wbuf;

    ATH_RXBUF_RESET(bf); 
    
    ds = bf->bf_desc;
    ds->ds_link = 0;    /* link to null */
    ds->ds_data = bf->bf_buf_addr;

    /* XXX For RADAR?
     * virtual addr of the beginning of the buffer. */
    wbuf = bf->bf_mpdu;
    ASSERT(wbuf != NULL);
    ds->ds_vdata = wbuf_raw_data(wbuf);

    /* setup rx descriptors */
    ath_hal_setuprxdesc(ah, ds
                        , wbuf_get_len(wbuf)   /* buffer size */
                        , 0
        );

    if (sc->sc_rxlink == NULL)
        ath_hal_putrxbuf(ah, bf->bf_daddr);
    else
        *sc->sc_rxlink = bf->bf_daddr;

    sc->sc_rxlink = &ds->ds_link;
    ath_hal_rxena(ah);
}

/*
 * Calculate the receive filter according to the
 * operating mode and state:
 *
 * o always accept unicast, broadcast, and multicast traffic
 * o maintain current state of phy error reception (the hal
 *   may enable phy error frames for noise immunity work)
 * o probe request frames are accepted only when operating in
 *   hostap, adhoc, or monitor modes
 * o enable promiscuous mode according to the interface state
 * o accept beacons:
 *   - when operating in adhoc mode so the 802.11 layer creates
 *     node table entries for peers,
 *   - when operating in station mode for collecting rssi data when
 *     the station is otherwise quiet, or
 *   - when operating as a repeater so we see repeater-sta beacons
 *   - when scanning
 */
u_int32_t
ath_calcrxfilter(struct ath_softc *sc)
{
#define	RX_FILTER_PRESERVE	(HAL_RX_FILTER_PHYERR | HAL_RX_FILTER_PHYRADAR)
    u_int32_t rfilt;
    int netif = 0;

    rfilt = (ath_hal_getrxfilter(sc->sc_ah) & RX_FILTER_PRESERVE)
        | HAL_RX_FILTER_UCAST | HAL_RX_FILTER_BCAST | HAL_RX_FILTER_MCAST;
        
    /*
    ** If not a STA, enable processing of Probe Requests
    */
    
    if (sc->sc_opmode != HAL_M_STA)
        rfilt |= HAL_RX_FILTER_PROBEREQ;
        
    /*
    ** Can't set HOSTAP into promiscous mode
    */
        
    if ((sc->sc_opmode != HAL_M_HOSTAP &&
        (netif = sc->sc_ieee_ops->get_netif_settings(sc->sc_ieee)) & ATH_NETIF_PROMISCUOUS) ||
        (sc->sc_opmode == HAL_M_MONITOR)) {
        rfilt |= HAL_RX_FILTER_PROM;
        if (sc->sc_opmode == HAL_M_MONITOR) {
            rfilt &= ~HAL_RX_FILTER_UCAST; /* ??? To prevent from sending ACK */
        }
    }
    if ((sc->sc_opmode == HAL_M_STA && !(netif & ATH_NETIF_NO_BEACON)) ||
        sc->sc_opmode == HAL_M_IBSS ||
        sc->sc_nostabeacons || sc->sc_scanning)
        rfilt |= HAL_RX_FILTER_BEACON;

    /*
    ** If in HOSTAP mode, want to enable reception of PSPOLL frames & beacon frames
    */
    
    if (sc->sc_opmode == HAL_M_HOSTAP)
        rfilt |= (HAL_RX_FILTER_BEACON | HAL_RX_FILTER_PSPOLL);
    return rfilt;
#undef RX_FILTER_PRESERVE
}

void
ath_opmode_init(ath_dev_t dev)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;
    u_int32_t rfilt, mfilt[2];

    /* configure rx filter */
    rfilt = ath_calcrxfilter(sc);
    ath_hal_setrxfilter(ah, rfilt);

    /* configure bssid mask */
    if (sc->sc_hasbmask) {
        ath_hal_setbssidmask(ah, sc->sc_bssidmask);
    }

    /* configure operational mode */
    ath_hal_setopmode(ah);

	/* Handle any link-level address change. */
    ath_hal_setmac(ah, sc->sc_myaddr);
    
    /* calculate and install multicast filter */
    if (sc->sc_ieee_ops->get_netif_settings(sc->sc_ieee) & ATH_NETIF_ALLMULTI) {
        mfilt[0] = mfilt[1] = ~0;
    }
    else {
        sc->sc_ieee_ops->netif_mcast_merge(sc->sc_ieee, mfilt);
    }

    ath_hal_setmcastfilter(ah, mfilt[0], mfilt[1]);
    DPRINTF(sc, ATH_DEBUG_RECV ,
            "%s: RX filter 0x%x, MC filter %08x:%08x\n",
            __func__, rfilt, mfilt[0], mfilt[1]);    
}

/*
 * Enable the receive h/w following a reset.
 */
int
ath_startrecv(struct ath_softc *sc)
{
    struct ath_hal *ah = sc->sc_ah;
    struct ath_buf *bf, *tbf;

#ifdef ATH_RB
    /* Reset RB state machine */
    ath_rb_reset(sc);
#endif

    ATH_RXBUF_LOCK(sc);
    sc->sc_rxlink = NULL;
    TAILQ_FOREACH_SAFE(bf, &sc->sc_rxbuf, bf_list, tbf) {
        if (bf->bf_status & ATH_BUFSTATUS_STALE) {
            bf->bf_status &= ~ATH_BUFSTATUS_STALE; /* restarting h/w, no need for holding descriptors */
            /*
             * Upper layer may not be done with the frame yet so we can't
             * just re-queue it to hardware. Remove it from h/w queue. It'll be
             * re-queued when upper layer returns the frame and ath_rx_requeue_mpdu
             * is called.
             */
            if (!(bf->bf_status & ATH_BUFSTATUS_FREE)) {
                TAILQ_REMOVE(&sc->sc_rxbuf, bf, bf_list);
                continue;
            }
        }
        /* chain descriptors */
        ath_rx_buf_link(sc, bf);
    }

    bf = TAILQ_FIRST(&sc->sc_rxbuf);
    if (bf != NULL) {
        ath_hal_putrxbuf(ah, bf->bf_daddr);
        ath_hal_rxena(ah);      /* enable recv descriptors */
    }
    ATH_RXBUF_UNLOCK(sc);

    ath_opmode_init(sc);        /* set filters, etc. */
    ath_hal_startpcurecv(ah);	/* re-enable PCU/DMA engine */
    return 0;
}

/*
 * Disable the receive h/w in preparation for a reset.
 */
HAL_BOOL
ath_stoprecv(struct ath_softc *sc)
{
#define	PA2DESC(_sc, _pa)                                               \
    ((struct ath_desc *)((caddr_t)(_sc)->sc_rxdma.dd_desc +             \
                         ((_pa) - (_sc)->sc_rxdma.dd_desc_paddr)))
    struct ath_hal *ah = sc->sc_ah;
    u_int64_t tsf;
    HAL_BOOL stopped;

    ath_hal_stoppcurecv(ah);	/* disable PCU */
    ath_hal_setrxfilter(ah, 0);	/* clear recv filter */
    stopped = ath_hal_stopdmarecv(ah);	/* disable DMA engine */
    if (sc->sc_opmode == HAL_M_HOSTAP) {
        /* Adding this delay back, only for chip in AP mode
         * since not having this delay causing pci data bus error 
         * in WDS mode - See bug 37962
         */
        OS_DELAY(3000); /* 3ms is long enough for 1 frame */
    }
    tsf = ath_hal_gettsf64(ah);
#ifdef AR_DEBUG
    if (sc->sc_debug & (ATH_DEBUG_RESET | ATH_DEBUG_FATAL)) {
        struct ath_buf *bf;

        printk("ath_stoprecv: rx queue %p, link %p\n",
               (caddr_t) ath_hal_getrxbuf(ah), sc->sc_rxlink);
        TAILQ_FOREACH(bf, &sc->sc_rxbuf, bf_list) {
            struct ath_desc *ds = bf->bf_desc;
            HAL_STATUS status = ath_hal_rxprocdesc(ah, ds,
                                                   bf->bf_daddr, PA2DESC(sc, ds->ds_link), tsf);
            if (status == HAL_OK || (sc->sc_debug & ATH_DEBUG_FATAL))
                ath_printrxbuf(bf, status == HAL_OK);
        }
    }
#endif
    sc->sc_rxlink = NULL;		/* just in case */
    return stopped;
#undef PA2DESC
}

/*
 * Flush receive queue
 */
void
ath_flushrecv(struct ath_softc *sc)
{
    /*
     * ath_rx_tasklet may be used to hande rx interrupt and flush receive
     * queue at the same time. Use a lock to serialize the access of rx
     * queue.
     * ath_rx_tasklet cannot hold the spinlock while indicating packets.
     * Instead, do not claim the spinlock but check for a flush in
     * progress (see references to sc_rxflush)
     */

    /*
     * Since we cannot hold a lock, use busy wait.
     */
    while (cmpxchg(&sc->sc_rxflush, 0, 1) == 1);
    ath_rx_tasklet(sc, RX_DROP);
    sc->sc_rxflush = 0;
    if (cmpxchg(&sc->sc_rxflush, 1, 0) != 1) {
        DPRINTF(sc, ATH_DEBUG_RX_PROC, "%s: rx queue is not protected.\n", __func__);
    }
}

void
ath_rx_requeue(ath_dev_t dev, wbuf_t wbuf)
{
	struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_buf *bf = ATH_RX_CONTEXT(wbuf)->ctx_rxbuf;

    ASSERT(bf != NULL);

    ATH_RXBUF_LOCK(sc);
    if (bf->bf_status & ATH_BUFSTATUS_STALE) {
        /*
         * This buffer is still held for hw acess.
         * Mark it as free to be re-queued it later.
         */
        bf->bf_status |= ATH_BUFSTATUS_FREE;
    } else {
        TAILQ_INSERT_TAIL(&sc->sc_rxbuf, bf, bf_list);
        ath_rx_buf_link(sc, bf);
    }
    ATH_RXBUF_UNLOCK(sc);
}

/*
 * Process an individual frame
 */
int
ath_rx_input(ath_dev_t dev, ath_node_t node, int is_ampdu,
             wbuf_t wbuf, ieee80211_rx_status_t *rx_status,
             ATH_RX_TYPE *status)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_node *an = ATH_NODE(node);

#ifdef AP_SLOW_ANT_DIV
    struct ieee80211_frame *wh;

    wh = (struct ieee80211_frame *) wbuf_raw_data(wbuf);
    if (sc->sc_antdiv.antdiv_flags && IEEE80211_IS_DATA(wh) && 
        (rx_status->flags & ATH_RX_RSSI_VALID)) {
        an->an_antdiv_rssictl[sc->sc_antdiv.antdiv_curcfg] += rx_status->rssictl[0];
        an->an_antdiv_bytes[sc->sc_antdiv.antdiv_curcfg] += wbuf_get_pktlen(wbuf);
        an->an_antdiv_rxcnt[sc->sc_antdiv.antdiv_curcfg] ++;
    }
#endif

    if (is_ampdu && sc->sc_rxaggr) {
        *status = ATH_RX_CONSUMED;
        return ath_ampdu_input(sc, an, wbuf, rx_status);
    } else {
        *status = ATH_RX_NON_CONSUMED;
        return -1;
    }
}

/*
 * Process receive queue, as well as LED, etc.
 * Arg "flush":
 * 0: Process rx frames in rx interrupt.
 * 1: Drop rx frames in flush routine.
 * 2: Flush and indicate rx frames, must be synchronized with other flush threads.
 */
int
ath_rx_tasklet(ath_dev_t dev, int flush)
{
#define PA2DESC(_sc, _pa)                                               \
    ((struct ath_desc *)((caddr_t)(_sc)->sc_rxdma.dd_desc +             \
                         ((_pa) - (_sc)->sc_rxdma.dd_desc_paddr)))

    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_buf *bf, *bf_held = NULL;
#if defined(ATH_SUPPORT_DFS) || !defined(ATH_SUPPORT_UAPSD)
    struct ath_hal *ah = sc->sc_ah;
#endif
    struct ath_desc *ds;
    int type;
#ifndef ATH_SUPPORT_UAPSD
    HAL_STATUS retval;
#endif
    u_int phyerr;
    struct ieee80211_frame *wh;
    wbuf_t wbuf = NULL;
    ieee80211_rx_status_t rx_status;
    struct ath_phy_stats *phy_stats = &sc->sc_phy_stats[sc->sc_curmode];
    u_int8_t rxchainmask, chainreset = 0;
    int rx_processed = 0;

    DPRINTF(sc, ATH_DEBUG_RX_PROC, "%s\n", __func__);
    do {
        u_int16_t buf_len;
        u_int8_t frame_fc0;

        /* If handling rx interrupt and flush is in progress => exit */
        if (sc->sc_rxflush && (flush == RX_PROCESS)) {
            break;
        }

        ATH_RXBUF_LOCK(sc);
        bf = TAILQ_FIRST(&sc->sc_rxbuf);
        if (bf == NULL) {
            sc->sc_rxlink = NULL;
            ATH_RXBUF_UNLOCK(sc);
            break;
        }

        bf_held = NULL;

        /*
         * There is a race condition that DPC gets scheduled after sw writes RxE
         * and before hw re-load the last descriptor to get the newly chained one.
         * Software must keep the last DONE descriptor as a holding descriptor -
         * software does so by marking it with the STALE flag. 
         */
        if (bf->bf_status & ATH_BUFSTATUS_STALE) {
            bf_held = bf;
            bf = TAILQ_NEXT(bf_held, bf_list);
            if (bf == NULL) {
                /*
                 * The holding descriptor is the last descriptor in queue.
                 * It's safe to remove the last holding descriptor in DPC context.
                 */
                TAILQ_REMOVE(&sc->sc_rxbuf, bf_held, bf_list);
                bf_held->bf_status &= ~ATH_BUFSTATUS_STALE;
                sc->sc_rxlink = NULL;

                if (bf_held->bf_status & ATH_BUFSTATUS_FREE) {
                    TAILQ_INSERT_TAIL(&sc->sc_rxbuf, bf_held, bf_list);
                    ath_rx_buf_link(sc, bf_held); /* try to requeue this descriptor */
                }
                ATH_RXBUF_UNLOCK(sc);
                break;
            }
        }

#ifdef ATH_SUPPORT_UAPSD
        /*
         * Descriptors are now processed in the first-level
         * interrupt handler to support U-APSD trigger search.
         * This must also be done even when U-APSD is not active to support
         * other error handling that requires immediate attention.
         * We check bf_status to find out if the bf's descriptors have 
         * been processed by the HAL.
         */
        if (!(bf->bf_status & ATH_BUFSTATUS_DONE)) {
            ATH_RXBUF_UNLOCK(sc);
            break;
        }
#endif

        ds = bf->bf_desc;
        ++rx_processed;

#ifndef ATH_SUPPORT_UAPSD
        /* Non-UAPSD case: descriptor is processed here at DPC level */
    
        /*
         * Must provide the virtual address of the current
         * descriptor, the physical address, and the virtual
         * address of the next descriptor in the h/w chain.
         * This allows the HAL to look ahead to see if the
         * hardware is done with a descriptor by checking the
         * done bit in the following descriptor and the address
         * of the current descriptor the DMA engine is working
         * on.  All this is necessary because of our use of
         * a self-linked list to avoid rx overruns.
         */
        retval = ath_hal_rxprocdesc(ah, ds, bf->bf_daddr, PA2DESC(sc, ds->ds_link), 0);
        if (HAL_EINPROGRESS == retval) {
            struct ath_buf *tbf = TAILQ_NEXT(bf, bf_list);

            /*
             * Due to a h/w bug the descriptor status words could
             * get corrupted, including the done bit. Hence, check
             * if the next descriptor's done bit is set or not.
             *
             * If the next descriptor's done bit is set, the current
             * descriptor has been corrupted. Force s/w to discard this
             * descriptor and continue...
             */

            if (tbf) {
                struct ath_desc *tds = tbf->bf_desc;

                retval = ath_hal_rxprocdesc(ah, tds, tbf->bf_daddr, PA2DESC(sc, tds->ds_link), 0);

                if (HAL_EINPROGRESS == retval) {
#ifdef ATH_ADDITIONAL_STATS
                    sc->sc_stats.ast_rx_hal_in_progress++;
#endif
                    ATH_RXBUF_UNLOCK(sc);
                    break;
                }
            } else {
                ATH_RXBUF_UNLOCK(sc);
                break;
            }
            /* Read again to update rx desc status correctly */
            retval = ath_hal_rxprocdesc(ah, ds, bf->bf_daddr, PA2DESC(sc, ds->ds_link), 0);
        }

        /* XXX: we do not support frames spanning multiple descriptors */
        bf->bf_status |= ATH_BUFSTATUS_DONE;
#endif /* ! ATH_SUPPORT_UAPSD */

        /* Force PPM tracking */
        ath_force_ppm_logic(&sc->sc_ppm_info, bf, HAL_OK, &ds->ds_rxstat);

        wbuf = bf->bf_mpdu;
        if (wbuf == NULL) {		/* XXX ??? can this happen */
            printk("no mpdu (%s)\n", __func__);
            ATH_RXBUF_UNLOCK(sc);
            continue;
        }

#ifdef AR_DEBUG
        if (sc->sc_debug & ATH_DEBUG_RECV_DESC)
            ath_printrxbuf(bf, 1);
#endif

        /*
         * Now we know it's a completed frame, we can indicate the frame.
         * Remove the previous holding descriptor and leave this one
         * in the queue as the new holding descriptor.
         */
        if (bf_held) {
            TAILQ_REMOVE(&sc->sc_rxbuf, bf_held, bf_list);
            bf_held->bf_status &= ~ATH_BUFSTATUS_STALE;
            if (bf_held->bf_status & ATH_BUFSTATUS_FREE) {
                TAILQ_INSERT_TAIL(&sc->sc_rxbuf, bf_held, bf_list);
                ath_rx_buf_link(sc, bf_held); /* try to requeue this descriptor */
            }
        }

        bf->bf_status |= ATH_BUFSTATUS_STALE;
        bf_held = bf;

        /*
         * Release the lock here in case ieee80211_input() return
         * the frame immediately by calling ath_rx_mpdu_requeue().
         */
        ATH_RXBUF_UNLOCK(sc);

        if (flush == RX_DROP) {
            /*
             * If we're asked to flush receive queue, directly
             * chain it back at the queue without processing it.
             */
            goto rx_next;
        }
        
        wh = (struct ieee80211_frame *)wbuf_raw_data(wbuf);
        frame_fc0 = wh->i_fc[0];
        OS_MEMZERO(&rx_status, sizeof(ieee80211_rx_status_t));
 
#ifndef REMOVE_PKT_LOG
        /* do pktlog */
        {
            struct log_rx log_data;
            log_data.ds = ds;
            log_data.status = &ds->ds_rxstat;
            ath_log_rx(sc, &log_data, 0);
        }
#endif
 
        if (ds->ds_rxstat.rs_more) {
            /*
             * Frame spans multiple descriptors; this
             * cannot happen yet as we don't support
             * jumbograms.  If not in monitor mode,
             * discard the frame.
             */
#ifndef ERROR_FRAMES
            /*
             * Enable this if you want to see
             * error frames in Monitor mode.
             */
            if (sc->sc_opmode != HAL_M_MONITOR) {
                phy_stats->ast_rx_toobig++;
                goto rx_next;
            }
#endif
            /* fall thru for monitor mode handling... */
        } else if (ds->ds_rxstat.rs_status != 0) {
            phy_stats->ast_rx_err++;
            if (ds->ds_rxstat.rs_status & HAL_RXERR_INCOMP) {
                __11nstats(sc, rx_dsstat_err);
                goto rx_next;
            }
            if (ds->ds_rxstat.rs_status & HAL_RXERR_CRC) {
                rx_status.flags |= ATH_RX_FCS_ERROR;
                phy_stats->ast_rx_crcerr++;
            }
            if (ds->ds_rxstat.rs_status & HAL_RXERR_FIFO)
                phy_stats->ast_rx_fifoerr++;
            if (ds->ds_rxstat.rs_status & HAL_RXERR_PHY) {
                phy_stats->ast_rx_phyerr++;
                phyerr = ds->ds_rxstat.rs_phyerr & 0x1f;
                phy_stats->ast_rx_phy[phyerr]++;
#ifdef ATH_SUPPORT_DFS
                {
                    u_int64_t tsf = ath_hal_gettsf64(ah);
                    /* Process phyerrs */
                    ath_process_phyerr(sc, ds, tsf);
                }
#endif
                goto rx_next;
            }

            if (ds->ds_rxstat.rs_status & HAL_RXERR_DECRYPT) {
                /*
                 * Decrypt error. We only mark packet status here
                 * and always push up the frame up to let NET80211 layer
                 * handle the actual error case, be it no decryption key
                 * or real decryption error.
                 * This let us keep statistics there.
                 */
                 phy_stats->ast_rx_decrypterr++;
                rx_status.flags |= ATH_RX_DECRYPT_ERROR;
            } else if (ds->ds_rxstat.rs_status & HAL_RXERR_MIC) {
                /*
                 * Demic error. We only mark frame status here
                 * and always push up the frame up to let NET80211 layer
                 * handle the actual error case.
                 * This let us keep statistics there and also apply the
                 * WAR for bug 6903: (Venice?) Hardware may
                 * post a false-positive MIC error.  Need to expose this
                 * error to tkip_demic() to really declare a failure.
                 */
                if ((frame_fc0 & IEEE80211_FC0_TYPE_MASK) == IEEE80211_FC0_TYPE_CTL) {
                    /*
                     * As doc. in hardware bug 30127, sometimes, we get invalid
                     * MIC failures on valid control frames. Remove these mic errors.
                     */
                    ds->ds_rxstat.rs_status &= ~HAL_RXERR_MIC;
                    phy_stats->ast_rx_demicok++;
                }
                else {
                    phy_stats->ast_rx_demicerr++;
                    rx_status.flags |= ATH_RX_MIC_ERROR;
                }
            } else {
                phy_stats->ast_rx_demicok++;
            }

            /*
             * Reject error frames with the exception of
             * decryption and MIC failures. For monitor mode, we also
             * ignore the CRC error.
             */
            if (sc->sc_opmode == HAL_M_MONITOR) {
                if (ds->ds_rxstat.rs_status &
                    ~(HAL_RXERR_DECRYPT | HAL_RXERR_MIC | HAL_RXERR_CRC))
                    goto rx_next;
            } else {
                if (ds->ds_rxstat.rs_status &
                    ~(HAL_RXERR_DECRYPT | HAL_RXERR_MIC)) {
                    goto rx_next;
                }
            }
        }

        /*
         * To workaround a h/w issue due to which the status
         * portion of the descriptor could get corrupted.
         * Refer to BUG#28934 for additional details.
         */
        /*
         * Check for 0 length also, for frames with corrupted 
         * descriptor status 
         */
        if ((sc->sc_rxbufsize < ds->ds_rxstat.rs_datalen) 
             || (0 == ds->ds_rxstat.rs_datalen)) {
            __11nstats(sc, rx_dsstat_err);
            DPRINTF(sc,ATH_DEBUG_ANY, 
                "%s: Incorrect frame length %d due to rx ds status corruption\n",
                __func__, ds->ds_rxstat.rs_datalen);
            goto rx_next;
        }

        /*
         * Sync and unmap the frame.  At this point we're
         * committed to passing the sk_buff somewhere so
         * clear buf_skb; this means a new sk_buff must be
         * allocated when the rx descriptor is setup again
         * to receive another frame.
         */
        wbuf_init(wbuf, ds->ds_rxstat.rs_datalen);
        buf_len = wbuf_get_pktlen(wbuf);
        rx_status.tsf = ath_extend_tsf(sc, ds->ds_rxstat.rs_tstamp);
        rx_status.rateieee = sc->sc_hwmap[ds->ds_rxstat.rs_rate].ieeerate;
        rx_status.rateKbps = sc->sc_hwmap[ds->ds_rxstat.rs_rate].rateKbps;
        rx_status.ratecode = ds->ds_rxstat.rs_rate;

        /* HT rate */
        if (rx_status.ratecode & 0x80) {
            /* TODO - add table to avoid division */
            if (ds->ds_rxstat.rs_flags & HAL_RX_2040) {
                rx_status.flags |= ATH_RX_40MHZ;
                rx_status.rateKbps = (rx_status.rateKbps * 27) / 13;
            }
            if (ds->ds_rxstat.rs_flags & HAL_RX_GI) {
                rx_status.rateKbps = (rx_status.rateKbps * 10) / 9;
            } else {
                rx_status.flags |= ATH_RX_SHORT_GI;
            }
        }

        /* sc->sc_noise_floor is only available when the station attaches to an AP,
         * so we use a default value if we are not yet attached.
         */
        /* XXX we should use either sc->sc_noise_floor or
         * ath_hal_getChanNoise(ah, &sc->sc_curchan) 
         * to calculate the noise floor.
         * However, the value returned by ath_hal_getChanNoise seems to be incorrect
         * (-31dBm on the last test), so we will use a hard-coded value until we 
         * figure out what is going on.
         */
        rx_status.abs_rssi = ds->ds_rxstat.rs_rssi + ATH_DEFAULT_NOISE_FLOOR;

        OS_SYNC_SINGLE(sc->sc_osdev,
                       bf->bf_buf_addr, wbuf_get_pktlen(wbuf), BUS_DMA_FROMDEVICE,
                       OS_GET_DMA_MEM_CONTEXT(bf, bf_dmacontext));
        OS_UNMAP_SINGLE(sc->sc_osdev, bf->bf_buf_addr,
                        sc->sc_rxbufsize, BUS_DMA_FROMDEVICE,
                        OS_GET_DMA_MEM_CONTEXT(bf, bf_dmacontext));

        if (ds->ds_rxstat.rs_antenna <  8 )
       {
               sc->sc_stats.ast_ant_rx[ds->ds_rxstat.rs_antenna]++;
       }
        if (sc->sc_hashtsupport) {
            if (ds->ds_rxstat.rs_moreaggr == 0) {
                /*
                 * do we really need to store these in sc ?
                 */
                sc->sc_stats.ast_rx_rssi       = ds->ds_rxstat.rs_rssi;
                sc->sc_stats.ast_rx_rssi_ctl0  = ds->ds_rxstat.rs_rssi_ctl0;
                sc->sc_stats.ast_rx_rssi_ctl1  = ds->ds_rxstat.rs_rssi_ctl1;
                sc->sc_stats.ast_rx_rssi_ctl2  = ds->ds_rxstat.rs_rssi_ctl2;
                sc->sc_stats.ast_rx_rssi_ext0  = ds->ds_rxstat.rs_rssi_ext0;
                sc->sc_stats.ast_rx_rssi_ext1  = ds->ds_rxstat.rs_rssi_ext1;
                sc->sc_stats.ast_rx_rssi_ext2  = ds->ds_rxstat.rs_rssi_ext2;
                rx_status.rssictl[0]  = ds->ds_rxstat.rs_rssi_ctl0;
                rx_status.rssictl[1]  = ds->ds_rxstat.rs_rssi_ctl1;
                rx_status.rssictl[2]  = ds->ds_rxstat.rs_rssi_ctl2;
                rx_status.rssi = ds->ds_rxstat.rs_rssi;
                if (ds->ds_rxstat.rs_flags & HAL_RX_2040) {
                    rx_status.rssiextn[0]  = ds->ds_rxstat.rs_rssi_ext0;
                    rx_status.rssiextn[1]  = ds->ds_rxstat.rs_rssi_ext1;
                    rx_status.rssiextn[2]  = ds->ds_rxstat.rs_rssi_ext2;
                    rx_status.flags |= ATH_RX_RSSI_EXTN_VALID;
                }
                rx_status.flags |= ATH_RX_RSSI_VALID | ATH_RX_CHAIN_RSSI_VALID;
            }
        } else {
            /*
             * Need to insert the "combined" rssi into the status structure
             * for upper layer processing
             */
            
            rx_status.rssi = ds->ds_rxstat.rs_rssi;
            rx_status.flags |= ATH_RX_RSSI_VALID;
        }

        if (ds->ds_rxstat.rs_flags & HAL_RX_HI_RX_CHAIN)
            rx_status.flags |= ATH_RX_SM_ENABLE;

#ifdef ATH_ADDITIONAL_STATS
                        do {
                                u_int8_t frm_type;
                                u_int8_t frm_subtype;
                                wh = (struct ieee80211_frame *) wbuf_raw_data(wbuf);
                                frm_type = wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK;
                                frm_subtype = wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK;
                                if (frm_type == IEEE80211_FC0_TYPE_DATA ) {
                                        if (frm_subtype == IEEE80211_FC0_SUBTYPE_QOS) {
                                                struct ieee80211_qosframe       *whqos;
                                                int                             tid;
                                                whqos = (struct ieee80211_qosframe *) wh;
                                                tid = whqos->i_qos[0] & IEEE80211_QOS_TID;
                                                sc->sc_stats.ast_rx_num_qos_data[tid]++;
                                        } else {
                                                sc->sc_stats.ast_rx_num_nonqos_data++;
                                        }
                                }
                        } while(0);
#endif

        if (sc->sc_diversity) {
            /*
             * When using hardware fast diversity, change the default rx
             * antenna if rx diversity chooses the other antenna 3
             * times in a row.
             */
            if ((ds->ds_rxstat.rs_antenna < 8 ) && (sc->sc_defant != ds->ds_rxstat.rs_antenna)){
                if (++sc->sc_rxotherant >= 3)
                    ath_setdefantenna(sc,
                                      ds->ds_rxstat.rs_antenna);
            } else {
                sc->sc_rxotherant = 0;
            }
        }
        /* increment count of received bytes */
        sc->sc_stats.ast_rx_bytes += wbuf_get_pktlen(wbuf);
	
        /*
         * Increment rx_pkts count.
         */
        __11nstats(sc, rx_pkts);

        /*
         * WAR 25033: redo antenna detection for Lenovo devices
         */
        if (sc->sc_rx_chainmask_detect && sc->sc_rx_chainmask_start) {
            rxchainmask = ath_rx_detect_antenna(sc, &ds->ds_rxstat);
            if (rxchainmask) {
                sc->sc_rx_chainmask_detect = 0;
                sc->sc_rx_chainmask_start  = 0;
                if (sc->sc_rx_chainmask != rxchainmask) {
                    sc->sc_rx_chainmask = rxchainmask;

                    /* we have to do an reset to change chain mask */
                    chainreset = 1;
                }
            }
        }

#ifdef SLOW_ANT_DIV
        if (sc->sc_slowAntDiv && (rx_status.flags & ATH_RX_RSSI_VALID) && IEEE80211_IS_BEACON(wh)) {
            ath_slow_ant_div(&sc->sc_antdiv, wh, &ds->ds_rxstat);
        }
#endif

        if (sc->sc_antDivComb)
        {
            ath_ant_div_comb_scan(&sc->sc_antcomb, &ds->ds_rxstat);   
        }

        /*
         * Pass frames up to the stack.
         * Note: After calling ath_rx_indicate(), we should not assumed that the
         * contents of wbuf, wh, and ds are valid.
         */
        type = ath_rx_indicate(sc, wbuf, &rx_status, ds->ds_rxstat.rs_keyix);

        if (type == IEEE80211_FC0_TYPE_DATA) {
#ifdef ATH_ADDITIONAL_STATS
            sc->sc_stats.ast_rx_num_data++;
#endif
        }
#ifdef ATH_ADDITIONAL_STATS
        else if (type == IEEE80211_FC0_TYPE_MGT) {
            sc->sc_stats.ast_rx_num_mgmt++;
        }
        else if (type == IEEE80211_FC0_TYPE_CTL) {
            sc->sc_stats.ast_rx_num_ctl++;
        }
        else {
            sc->sc_stats.ast_rx_num_unknown++;
        }
#endif
 
        /* report data flow to LED module */
        if (type == IEEE80211_FC0_TYPE_DATA) {
            int subtype = frame_fc0 & IEEE80211_FC0_SUBTYPE_MASK;
            if (subtype != IEEE80211_FC0_SUBTYPE_NODATA &&
                subtype != IEEE80211_FC0_SUBTYPE_QOS_NULL)
                ath_led_report_data_flow(&sc->sc_led_control, buf_len);
        }

        /*
         * For frames successfully indicated, the buffer will be
         * returned to us by upper layers by calling ath_rx_mpdu_requeue,
         * either synchronusly or asynchronously.
         * So we don't want to do it here in this loop.
         */
        continue;
        
rx_next:
        bf->bf_status |= ATH_BUFSTATUS_FREE;
    } while (TRUE);

#ifdef ATH_SUPPORT_DFS
    if (sc->sc_dfs != NULL) {
                if (!STAILQ_EMPTY(&sc->sc_dfs->dfs_arq))
                        dfs_process_ar_event(sc, &sc->sc_curchan);
                if (!STAILQ_EMPTY(&sc->sc_dfs->dfs_radarq)) {
                        sc->sc_rtasksched = 1;
                         OS_SET_TIMER(&sc->sc_dfs->sc_dfs_task_timer, 0);
                }
        }
#endif 
#ifdef notyet
    /* rx signal state monitoring */
    ath_hal_rxmonitor(ah, &sc->sc_halstats, &sc->sc_curchan);
#endif

#ifdef ATH_ADDITIONAL_STATS
        if (rx_processed < ATH_RXBUF ) {
                sc->sc_stats.ast_pkts_per_intr[rx_processed]++;
        }
        else {
                sc->sc_stats.ast_pkts_per_intr[ATH_RXBUF]++;
        }
#else
#endif

    if (chainreset) {
        printk("Reset rx chain mask. Do internal reset. (%s)\n", __func__);
        ASSERT(flush == 0);
        ath_internal_reset(sc);
    }
    
    return 0;
#undef PA2DESC
}

/*
 * Set a user defined ADDBA response status code.
 */
void
ath_set_addbaresponse(ath_dev_t dev, ath_node_t node,
                      u_int8_t tidno, u_int16_t statuscode)
{
    struct ath_node *an = ATH_NODE(node);
    struct ath_arx_tid *rxtid = &an->an_aggr.rx.tid[tidno];

    /*
     * Set the user defined ADDBA response for this TID
     */
    rxtid->userstatuscode = statuscode;
}

/*
 * Clear the user defined ADDBA response status code.
 */
void
ath_clear_addbaresponsestatus(ath_dev_t dev, ath_node_t node)
{
#define	N(a)	(sizeof(a)/sizeof(a[0]))
    struct ath_node *an = ATH_NODE(node);
    struct ath_arx_tid *rxtid;
    int i;

    for (i = 0; i < N(an->an_rx_tid); i++) {
        rxtid = &an->an_rx_tid[i];
	    rxtid->userstatuscode = IEEE80211_STATUS_SUCCESS;
    }
#undef N
}

/*
 * Process ADDBA request and save response information in per-TID data structure
 */
int
ath_addba_requestprocess(
    ath_dev_t dev, ath_node_t node,
    u_int8_t dialogtoken,
    struct ieee80211_ba_parameterset *baparamset,
    u_int16_t batimeout,
    struct ieee80211_ba_seqctrl basequencectrl
    )
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_node *an = ATH_NODE(node);
    u_int16_t tidno = baparamset->tid;
    struct ath_arx_tid *rxtid  = &an->an_aggr.rx.tid[tidno];

    ATH_RXTID_LOCK(rxtid);

    if (!sc->sc_rxaggr) {
        /* decline addba request */
        rxtid->statuscode = IEEE80211_STATUS_REFUSED;
    } else if (rxtid->userstatuscode != IEEE80211_STATUS_SUCCESS) {
        /*
         * XXX: we imply it's manual ADDBA mode if userstatuscode
         * is set to non-success code
         */
        rxtid->statuscode = rxtid->userstatuscode;
    } else {                    /* Allow aggregation reception */
        if (rxtid->addba_exchangecomplete && rxtid->rxbuf != NULL) {
            ath_cancel_timer(&rxtid->timer, CANCEL_NO_SLEEP);
            ath_rx_flush_tid(sc, rxtid, 0);
            rxtid->addba_exchangecomplete = 0;
        }

        /*
         * Adjust rx BA window size. Peer might indicate a zero buffer size for
         * a _dont_care_ condition.
         */
        if (baparamset->buffersize)
            rxtid->baw_size = MIN(baparamset->buffersize, rxtid->baw_size);

        /* set rx sequence number */
        rxtid->seq_next = basequencectrl.startseqnum;

        /* save ADDBA response parameters in rx TID */
        rxtid->statuscode               = IEEE80211_STATUS_SUCCESS;
        rxtid->baparamset.bapolicy      = IEEE80211_BA_POLICY_IMMEDIATE;
        rxtid->baparamset.buffersize    = rxtid->baw_size;
        rxtid->batimeout                = 0;

        /*
        ** Allocate the receive buffers for this TID
        */

        DPRINTF(sc,ATH_DEBUG_AGGR_MEM,"%s: Allcating rxbuffer for TID %d\n",__func__,tidno);

        if (rxtid->rxbuf == NULL) {
            /*
            ** If the rxbuff is not NULL at this point, we *probably* already allocated the
            ** buffer on a previous ADDBA, and this is a subsequent ADDBA that got through.
            ** Don't allocate, but use the value in the pointer (we zero it out when we de-allocate)
            */

            rxtid->rxbuf = (struct ath_rxbuf *)OS_MALLOC(sc->sc_osdev ,
                                                         ATH_TID_MAX_BUFS*sizeof(struct ath_rxbuf),
                                                         GFP_ATOMIC);
        }

        if (rxtid->rxbuf == NULL) {
            /*
            ** If malloc is unsuccessful, treat this as an ADDBA Reject
            */
           rxtid->statuscode = IEEE80211_STATUS_REFUSED;
           DPRINTF(sc,ATH_DEBUG_AGGR_MEM,"%s: Unable to allocate RX buffer, refusing ADDBA\n",__func__);

        } else {

            /*
            ** Ensure the memory is zeroed out (all internal pointers are null)
            */

            OS_MEMZERO(rxtid->rxbuf, ATH_TID_MAX_BUFS*sizeof(struct ath_rxbuf));

            DPRINTF(sc,ATH_DEBUG_AGGR_MEM, "%s: Allocated @%p\n",__func__,
                                                                rxtid->rxbuf);

            /* Allow aggregation reception */
            rxtid->addba_exchangecomplete = 1;
        }
    }

    rxtid->dialogtoken              = dialogtoken;
    rxtid->baparamset.amsdusupported = IEEE80211_BA_AMSDU_SUPPORTED;
    rxtid->baparamset.tid           = tidno;

    ATH_RXTID_UNLOCK(rxtid);
    return 0;
}

/*
 * Setup ADDBA response
 *
 * Output: status code, BA parameter set and BA timeout
 *         for response
 */
void
ath_addba_responsesetup(
    ath_dev_t dev, ath_node_t node,
    u_int8_t tidno,
    u_int8_t *dialogtoken,
    u_int16_t *statuscode,
    struct ieee80211_ba_parameterset *baparamset,
    u_int16_t *batimeout
    )
{
    struct ath_node *an = ATH_NODE(node);
    struct ath_arx_tid *rxtid  = &an->an_aggr.rx.tid[tidno];

    /* setup ADDBA response paramters */
    *dialogtoken = rxtid->dialogtoken;
    *statuscode = rxtid->statuscode;
    *baparamset = rxtid->baparamset;
    *batimeout  = rxtid->batimeout;
}

/*
 * Process DELBA
 */
void
ath_delba_process(
    ath_dev_t dev, ath_node_t node,
    struct ieee80211_delba_parameterset *delbaparamset,
    u_int16_t reasoncode
    )
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_node *an = ATH_NODE(node);
    u_int16_t tidno = delbaparamset->tid;

    if (delbaparamset->initiator)
        ath_rx_aggr_teardown(sc, an, tidno);
   else
        ath_tx_aggr_teardown(sc, an, tidno);
}

/*
 * Process received BAR frame
 */
static int
ath_bar_rx(struct ath_softc *sc, struct ath_node *an, wbuf_t wbuf)
{
    struct ieee80211_frame_bar *bar;
    int tidno;
    u_int16_t seqno;
    struct ath_arx_tid *rxtid;
    int index, cindex;
    wbuf_t twbuf;
    ieee80211_rx_status_t *rx_status;

    __11nstats(sc, rx_bars);

    /*
     * look at BAR contents
     */
    bar = (struct ieee80211_frame_bar *) wbuf_header(wbuf);
    tidno = (bar->i_ctl & IEEE80211_BAR_CTL_TID_M) >> IEEE80211_BAR_CTL_TID_S;
    seqno = le16toh(bar->i_seq) >> IEEE80211_SEQ_SEQ_SHIFT;

    /*
     * process BAR - indicate all pending RX frames till the BAR seqno
     */
    rxtid = &an->an_aggr.rx.tid[tidno];

    ATH_RXTID_LOCK(rxtid);

    /*
     * get relative index
     */
    index = ATH_BA_INDEX(rxtid->seq_next, seqno);

    /*
     * drop BAR if old sequence (index is too large)
     */
    if ((index > rxtid->baw_size) &&
        (index > (IEEE80211_SEQ_MAX - (rxtid->baw_size << 2)))) {
        /*
         * discard frame, ieee layer may not treat frame as a dup
         */
        ATH_RXTID_UNLOCK(rxtid);
        __11nstats(sc, rx_bardiscard);
        wbuf_free(wbuf);
        return IEEE80211_FC0_TYPE_CTL;
    }

    /*
     * complete receive processing for all pending frames upto BAR seqno
     */
    cindex = (rxtid->baw_head + index) & (ATH_TID_MAX_BUFS - 1);
    while ((rxtid->baw_head != rxtid->baw_tail) &&
           (rxtid->baw_head != cindex)) {
        twbuf = rxtid->rxbuf[rxtid->baw_head].rx_wbuf;
        rx_status = &rxtid->rxbuf[rxtid->baw_head].rx_status;
        rxtid->rxbuf[rxtid->baw_head].rx_wbuf = NULL;

        if (twbuf != NULL) {
            __11nstats(sc, rx_barcomps);
            sc->sc_ieee_ops->rx_subframe(an->an_node, twbuf, rx_status);
        }

        INCR(rxtid->baw_head, ATH_TID_MAX_BUFS);
        INCR(rxtid->seq_next, IEEE80211_SEQ_MAX);
    }

    /*
     * ... and indicate rest of the frames in-order
     */
    while (rxtid->baw_head != rxtid->baw_tail &&
           rxtid->rxbuf[rxtid->baw_head].rx_wbuf != NULL) {
        twbuf = rxtid->rxbuf[rxtid->baw_head].rx_wbuf;
        rx_status = &rxtid->rxbuf[rxtid->baw_head].rx_status;
        rxtid->rxbuf[rxtid->baw_head].rx_wbuf = NULL;

        __11nstats(sc, rx_barrecvs);
        sc->sc_ieee_ops->rx_subframe(an->an_node, twbuf, rx_status);

        INCR(rxtid->baw_head, ATH_TID_MAX_BUFS);
        INCR(rxtid->seq_next, IEEE80211_SEQ_MAX);
    }

    ATH_RXTID_UNLOCK(rxtid);

    /*
     * free bar itself
     */
    wbuf_free(wbuf);
    return IEEE80211_FC0_TYPE_CTL;
}

/*
 * Function to handle a subframe of aggregation when HT is enabled
 */
static int
ath_ampdu_input(struct ath_softc *sc, struct ath_node *an, wbuf_t wbuf, ieee80211_rx_status_t *rx_status)
{
    struct ieee80211_frame             *wh;
    struct ieee80211_qosframe          *whqos;
    struct ieee80211_qosframe_addr4    *whqos_4addr;
    u_int8_t                           type, subtype;
#ifdef ATH_RB
    u_int8_t                           wep, qos_noack;
#endif
    int                                ismcast;
    int                                tid;
    struct ath_arx_tid                 *rxtid;
    int                                index, cindex, rxdiff;
    u_int16_t                          rxseq;
    struct ath_rxbuf                   *rxbuf;
    int                                is4addr;

    wh = (struct ieee80211_frame *) wbuf_header(wbuf);
    is4addr = (wh->i_fc[1] & IEEE80211_FC1_DIR_MASK) == IEEE80211_FC1_DIR_DSTODS;

    __11nstats(sc, rx_aggr);
    /*
     * collect stats of frames with non-zero version
     */
    if ((wh->i_fc[0] & IEEE80211_FC0_VERSION_MASK) != IEEE80211_FC0_VERSION_0) {
        __11nstats(sc, rx_aggrbadver);
        wbuf_free(wbuf);
        return -1;
    }

    type = wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK;
    subtype = wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK;
    ismcast = IEEE80211_IS_MULTICAST(wh->i_addr1);
#ifdef ATH_RB
    wep = wh->i_fc[1] & IEEE80211_FC1_WEP;
#endif

    if ((type == IEEE80211_FC0_TYPE_CTL) &&
        (subtype == IEEE80211_FC0_SUBTYPE_BAR)) {
        return ath_bar_rx(sc, an, wbuf);
    }

    /*
     * special aggregate processing only for qos unicast data frames
     */
    if (type != IEEE80211_FC0_TYPE_DATA ||
        subtype != IEEE80211_FC0_SUBTYPE_QOS || (ismcast)) {
        __11nstats(sc, rx_nonqos);
        return sc->sc_ieee_ops->rx_subframe(an->an_node, wbuf, rx_status);
    }

    /*
     * lookup rx tid state
     */
    if (is4addr) { /* special qos check for 4 address frames */
        whqos_4addr = (struct ieee80211_qosframe_addr4 *) wh;
        tid = whqos_4addr->i_qos[0] & IEEE80211_QOS_TID;
    } else {
        whqos = (struct ieee80211_qosframe *) wh;
        tid = whqos->i_qos[0] & IEEE80211_QOS_TID;
#ifdef ATH_RB
        qos_noack = (whqos->i_qos[0] & IEEE80211_QOS_ACKPOLICY) >>
                     IEEE80211_QOS_ACKPOLICY_S;
        if (sc->sc_do_rb_war && sc->sc_rxrifs == ATH_RB_MODE_DETECT &&
            type == IEEE80211_FC0_TYPE_DATA &&
            subtype == IEEE80211_FC0_SUBTYPE_QOS && !wep && qos_noack) {
            KASSERT(!ismcast, ("mcast frames in qos-noack rb processing"));
            ath_rb_detect(&sc->sc_rb, whqos);
        }
#endif
    	if (sc->sc_opmode == HAL_M_STA) {
	    /* Drop the frame not belonging to me. Refer to Bug 34218*/
	    if (OS_MEMCMP(wh->i_addr1, sc->sc_myaddr, IEEE80211_ADDR_LEN)) {
		wbuf_free(wbuf);
		return -1;
	    }
	}
    }

    rxtid = &an->an_aggr.rx.tid[tid];

    ATH_RXTID_LOCK(rxtid);

    /*
     * If the ADDBA exchange has not been completed by the source,
     * process via legacy path (i.e. no reordering buffer is needed)
     */
    if (!rxtid->addba_exchangecomplete) {
        ATH_RXTID_UNLOCK(rxtid);
        __11nstats(sc, rx_nonqos);
        return sc->sc_ieee_ops->rx_subframe(an->an_node, wbuf, rx_status);
    }

    /*
     * extract sequence number from recvd frame
     */
    rxseq = le16toh(*(u_int16_t *)wh->i_seq) >> IEEE80211_SEQ_SEQ_SHIFT;

    if (rxtid->seq_reset) {
        __11nstats(sc, rx_seqreset);
        rxtid->seq_reset = 0;
        rxtid->seq_next = rxseq;
    }

    index = ATH_BA_INDEX(rxtid->seq_next, rxseq);

    /*
     * drop frame if old sequence (index is too large)
     */
    if (index > (IEEE80211_SEQ_MAX - (rxtid->baw_size << 2))) {
        /*
         * discard frame, ieee layer may not treat frame as a dup
         */
        ATH_RXTID_UNLOCK(rxtid);
        __11nstats(sc, rx_oldseq);
        wbuf_free(wbuf);
        return IEEE80211_FC0_TYPE_DATA;
    }

    /*
     * sequence number is beyond block-ack window
     */
    if (index >= rxtid->baw_size) {

        __11nstats(sc, rx_bareset);

        /*
         * complete receive processing for all pending frames
         */
        while (index >= rxtid->baw_size) {

            rxbuf = rxtid->rxbuf + rxtid->baw_head;

            if (rxbuf->rx_wbuf != NULL) {
                __11nstats(sc, rx_baresetpkts);
                sc->sc_ieee_ops->rx_subframe(an->an_node, rxbuf->rx_wbuf,
                                             &rxbuf->rx_status);
                __11nstats(sc, rx_recvcomp);
                rxbuf->rx_wbuf = NULL;
            }

            INCR(rxtid->baw_head, ATH_TID_MAX_BUFS);
            INCR(rxtid->seq_next, IEEE80211_SEQ_MAX);

            index --;
        }
    }

    /*
     * add buffer to the recv ba window
     */
    cindex = (rxtid->baw_head + index) & (ATH_TID_MAX_BUFS - 1);
    rxbuf = rxtid->rxbuf + cindex;

    if (rxbuf->rx_wbuf != NULL) {
        /*
         *duplicate frame
         */
        ATH_RXTID_UNLOCK(rxtid);
        __11nstats(sc, rx_dup);
        wbuf_free(wbuf);
        return IEEE80211_FC0_TYPE_DATA;
    }

    rxdiff = (rxtid->baw_tail - rxtid->baw_head) &
             (ATH_TID_MAX_BUFS - 1);
    rxbuf->rx_wbuf = wbuf;
    rxbuf->rx_time = OS_GET_TIMESTAMP();
    rxbuf->rx_status = *rx_status;

    /*
     * advance tail if sequence received is newer than any received so far
     */
    if (index >= rxdiff) {
        __11nstats(sc, rx_baadvance);
        rxtid->baw_tail = cindex;
        INCR(rxtid->baw_tail, ATH_TID_MAX_BUFS);
    }

    /*
     * indicate all in-order received frames
     */
    while (rxtid->baw_head != rxtid->baw_tail) {
        rxbuf = rxtid->rxbuf + rxtid->baw_head;
        if (!rxbuf->rx_wbuf)
            break;

        __11nstats(sc, rx_recvcomp);
        sc->sc_ieee_ops->rx_subframe(an->an_node, rxbuf->rx_wbuf, &rxbuf->rx_status);
        rxbuf->rx_wbuf = NULL;

        INCR(rxtid->baw_head, ATH_TID_MAX_BUFS);
        INCR(rxtid->seq_next, IEEE80211_SEQ_MAX);
    }

    /*
     * start a timer to flush all received frames if there are pending
     * receive frames
     */
    if (rxtid->baw_head != rxtid->baw_tail) {
        if (!ath_timer_is_active(&rxtid->timer)) {
            __11nstats(sc,rx_timer_starts);
            ath_set_timer_period(&rxtid->timer, ATH_RX_TIMEOUT);
            ath_start_timer(&rxtid->timer);
        }
    } else {
        if (ath_timer_is_active(&rxtid->timer)) {
            __11nstats(sc,rx_timer_stops);
        }
        ath_cancel_timer(&rxtid->timer, CANCEL_NO_SLEEP);
    }
    
    ATH_RXTID_UNLOCK(rxtid);
    return IEEE80211_FC0_TYPE_DATA;
}

/*
 * Timer to flush all received sub-frames
 */
static int
ath_rx_timer(void *context)
{
    struct ath_arx_tid *rxtid = (struct ath_arx_tid *) context;
    struct ath_node *an = rxtid->an;
    struct ath_softc *sc = an->an_sc;
    int nosched;
    struct ath_rxbuf *rxbuf;
    int count = 0;
    systime_t diff;
    int baw_head;

    ATH_PS_WAKEUP(sc);

    __11nstats(sc, rx_timer_run);
    ATH_RXTID_LOCK(rxtid);
    baw_head = rxtid->baw_head;

    while (baw_head != rxtid->baw_tail) {
        rxbuf = rxtid->rxbuf + baw_head;
        if (!rxbuf->rx_wbuf) {
            count++;
            INCR(baw_head, ATH_TID_MAX_BUFS);
            continue;
        }

        /*
         * Stop if the next one is a very recent frame.
         *
         * Call OS_GET_TIMESTAMP in every iteration to protect against the
         * case in which a new frame is received while we are executing this
         * function. Using a timestamp obtained before entering the loop could
         * lead to a very large time interval (a negative value typecast to
         * unsigned), breaking the function's logic.
         */
        diff = OS_GET_TIMESTAMP() - rxbuf->rx_time;
        if (diff < ATH_RX_TIMEOUT) {
            ath_set_timer_period(&rxtid->timer, ATH_RX_TIMEOUT - diff);
            break;
        }

        __11nstats(sc, rx_recvcomp);
        __11nstats(sc, rx_comp_to);
        __11nstatsn(sc, rx_skipped, count);
        sc->sc_ieee_ops->rx_subframe(an->an_node, rxbuf->rx_wbuf,
                                     &rxbuf->rx_status);
        rxbuf->rx_wbuf = NULL;

        count++;

        INCR(baw_head, ATH_TID_MAX_BUFS);
        ADD(rxtid->baw_head, count, ATH_TID_MAX_BUFS);
        ADD(rxtid->seq_next, count, IEEE80211_SEQ_MAX);
        count = 0;
    }

    /*
     * start a timer to flush all received frames if there are pending
     * receive frames
     */
    if (rxtid->baw_head != rxtid->baw_tail) {
        __11nstats(sc, rx_timer_more);
        nosched = 0;
    } else {
        nosched = 1; /* no need to re-arm the timer again */
    }

    ATH_RXTID_UNLOCK(rxtid);

    ATH_PS_SLEEP(sc);

    return nosched;
}

/*
 * Free all pending sub-frames in the re-ordering buffer
 * ATH_RXTID_LOCK must be held
 */
static void
ath_rx_flush_tid(struct ath_softc *sc, struct ath_arx_tid *rxtid, int drop)
{
    struct ath_rxbuf *rxbuf;

    while (rxtid->baw_head != rxtid->baw_tail) {
        rxbuf = rxtid->rxbuf + rxtid->baw_head;
        if (!rxbuf->rx_wbuf) {
            INCR(rxtid->baw_head, ATH_TID_MAX_BUFS);
            INCR(rxtid->seq_next, IEEE80211_SEQ_MAX);
            __11nstats(sc, rx_skipped);
            continue;
        }

        __11nstats(sc, rx_recvcomp);

        if (drop) {
            wbuf_free(rxbuf->rx_wbuf);
        } else {
            __11nstats(sc, rx_comp_to);
            sc->sc_ieee_ops->rx_subframe(rxtid->an->an_node, rxbuf->rx_wbuf,
                                         &rxbuf->rx_status);
        }
        rxbuf->rx_wbuf = NULL;

        INCR(rxtid->baw_head, ATH_TID_MAX_BUFS);
        INCR(rxtid->seq_next, IEEE80211_SEQ_MAX);
    }
}

/*
 * Rx aggregation tear down
 */
void
ath_rx_aggr_teardown(struct ath_softc *sc, struct ath_node *an, u_int8_t tidno)
{
    struct ath_arx_tid *rxtid = &an->an_rx_tid[tidno];

    ATH_RXTID_LOCK(rxtid);

    if (!rxtid->addba_exchangecomplete) {
        ATH_RXTID_UNLOCK(rxtid);
        return;
    }

    ath_cancel_timer(&rxtid->timer, CANCEL_NO_SLEEP);
    ath_rx_flush_tid(sc, rxtid, 0);
    rxtid->addba_exchangecomplete = 0;
    
    /* 
    ** De-allocate the receive buffer array allocated when addba 
    ** started 
    */ 
    
    if (rxtid->rxbuf != 0) { 
        DPRINTF(sc,ATH_DEBUG_AGGR_MEM, "%s: Deallocating TID %d rxbuff @%p\n", 
                   __func__, tidno, rxtid->rxbuf); 
        OS_FREE( rxtid->rxbuf );
        
        /*
        ** Set pointer to null to avoid reuse
        */
        
        rxtid->rxbuf = NULL; 
    }
    ATH_RXTID_UNLOCK(rxtid);
}

/*
 * Initialize per-node receive state
 */
void
ath_rx_node_init(struct ath_softc *sc, struct ath_node *an)
{
    if (sc->sc_rxaggr) {
        struct ath_arx_tid *rxtid;
        int tidno;

        /* Init per tid rx state */
        for (tidno = 0, rxtid = &an->an_rx_tid[tidno]; tidno < WME_NUM_TID;
             tidno++, rxtid++) {
            rxtid->an        = an;
            rxtid->seq_reset = 1;
            rxtid->seq_next  = 0;
            rxtid->baw_size  = WME_MAX_BA;
            rxtid->baw_head  = rxtid->baw_tail = 0;

            /*
            ** Ensure the buffer pointer is null at this point (needs to be allocated
            ** when addba is received)
            */

            rxtid->rxbuf     = NULL;

            ath_initialize_timer(sc->sc_osdev, &rxtid->timer, ATH_RX_TIMEOUT,
                                 ath_rx_timer, rxtid);

            ATH_RXTID_LOCK_INIT(rxtid);
            
            /* ADDBA state */
            rxtid->addba_exchangecomplete = 0;
            rxtid->userstatuscode = IEEE80211_STATUS_SUCCESS;
        }
    }
}

void ath_rx_node_cleanup(struct ath_softc *sc, struct ath_node *an)
{
    if (sc->sc_rxaggr) {
        struct ath_arx_tid *rxtid;
        int tidno,i;

        /* Init per tid rx state */
        for (tidno = 0, rxtid = &an->an_rx_tid[tidno]; tidno < WME_NUM_TID;
             tidno++, rxtid++) {

            ATH_RXTID_LOCK(rxtid);

            if (!rxtid->addba_exchangecomplete) {
                ATH_RXTID_UNLOCK(rxtid);
                continue;
            }
            /* must cancel timer first */
            ath_cancel_timer(&rxtid->timer, CANCEL_NO_SLEEP);

            /* drop any pending sub-frames */
            ath_rx_flush_tid(sc, rxtid, 1);

            for (i = 0; i < ATH_TID_MAX_BUFS; i++){
                ASSERT(rxtid->rxbuf[i].rx_wbuf == NULL);
            }

            if (rxtid->rxbuf) {
               OS_FREE(rxtid->rxbuf);
               rxtid->rxbuf = NULL;
            }
            rxtid->addba_exchangecomplete = 0;

            ATH_RXTID_UNLOCK(rxtid);
            ATH_RXTID_LOCK_DESTROY(rxtid);
        }
    }
}

/*
 * Cleanup per-node receive state
 */
void
ath_rx_node_free(struct ath_softc *sc, struct ath_node *an)
{
    ath_rx_node_cleanup(sc,an);
}

static u_int8_t
ath_rx_detect_antenna(struct ath_softc *sc, struct ath_rx_status *rxstat)
{
#define	IS_CHAN_5GHZ(_c)    (((_c).channelFlags & CHANNEL_5GHZ) != 0)
#define ATH_RX_CHAINMASK_CLR(_chainmask, _chain) ((_chainmask) &= ~(1 << (_chain)))
    u_int8_t rx_chainmask = sc->sc_rx_chainmask;
    int rssiRef, detectThresh, detectDelta;

    if (IS_CHAN_5GHZ(sc->sc_curchan)) {
        detectThresh = sc->sc_rxchaindetect_thresh5GHz;
        detectDelta = sc->sc_rxchaindetect_delta5GHz;
    } else {
        detectThresh = sc->sc_rxchaindetect_thresh2GHz;
        detectDelta = sc->sc_rxchaindetect_delta2GHz;
    }

    switch (sc->sc_rxchaindetect_ref) {
    case 0:
        rssiRef = rxstat->rs_rssi;
        if (rssiRef < detectThresh) {
            return 0;
        }

        if (rssiRef - rxstat->rs_rssi_ctl1 > detectDelta) {
            ATH_RX_CHAINMASK_CLR(rx_chainmask, 1);
        }

        if (rssiRef - rxstat->rs_rssi_ctl2 > detectDelta) {
            ATH_RX_CHAINMASK_CLR(rx_chainmask, 2);
        }
        break;

    case 1:
        rssiRef = rxstat->rs_rssi_ctl1;
        if (rssiRef < detectThresh) {
            return 0;
        }

        if (rssiRef - rxstat->rs_rssi_ctl2 > detectDelta) {
            ATH_RX_CHAINMASK_CLR(rx_chainmask, 2);
        }
        break;

    case 2:
        rssiRef = rxstat->rs_rssi_ctl2;
        if (rssiRef < detectThresh) {
            return 0;
        }

        if (rssiRef - rxstat->rs_rssi_ctl1 > detectDelta) {
            ATH_RX_CHAINMASK_CLR(rx_chainmask, 1);
        }
        break;
    }

    return rx_chainmask;
#undef IS_CHAN_5GHZ
#undef ATH_RX_CHAINMASK_CLR
}
