/*! \file ath_beacon.c
**  \brief ATH Beacon Processing
**    
**  This file contains the implementation of the common beacon support code for
**  the ATH layer, including any tasklets/threads used for beacon support.
**
** Copyright (c) 2009, Atheros Communications Inc.
**
** Permission to use, copy, modify, and/or distribute this software for any
** purpose with or without fee is hereby granted, provided that the above
** copyright notice and this permission notice appear in all copies.
**
** THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
** WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
** MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
** ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
** WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
** ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
** OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
**
*/

#include "ath_internal.h"
#ifdef ATH_SUPPORT_DFS
#include "dfs.h"
#endif

#ifndef REMOVE_PKT_LOG
#include "pktlog.h"
extern struct ath_pktlog_funcs *g_pktlog_funcs;
#endif

#define IEEE80211_MS_TO_TU(x)   (((x) * 1000) / 1024)

#define ATH_DEFAULT_BINTVAL     100 /* default beacon interval in TU */
#define ATH_DEFAULT_BMISS_LIMIT 10

#ifdef ATH_SUPPORT_DFS
static u_int32_t ath_get_phy_err_rate(struct ath_softc *sc);
int get_dfs_hang_war_timeout(struct ath_softc *sc);
#endif

/******************************************************************************/
/*!
**  \brief Setup a h/w transmit queue for beacons.
**
**  This function allocates an information structure (HAL_TXQ_INFO) on the stack,
**  sets some specific parameters (zero out channel width min/max, and enable aifs)
**  The info structure does not need to be persistant.
**
**  \param sc Pointer to the SoftC structure for the ATH object
**  \param nbufs Number of buffers to allocate for the transmit queue
**
**  \return Returns the queue index number (priority), or -1 for failure
*/

int
ath_beaconq_setup(struct ath_hal *ah)
{
    HAL_TXQ_INFO qi;

    OS_MEMZERO(&qi, sizeof(qi));
    qi.tqi_aifs = 1;
    qi.tqi_cwmin = 0;
    qi.tqi_cwmax = 0;
#ifdef ATH_SUPERG_DYNTURBO
    qi.tqi_qflags = TXQ_FLAG_TXDESCINT_ENABLE;
#endif
    /* NB: don't enable any interrupts */
    return ath_hal_setuptxqueue(ah, HAL_TX_QUEUE_BEACON, &qi);
}


/******************************************************************************/
/*!
**  \brief Configure parameters for the beacon queue
**
**  This function will modify certain transmit queue properties depending on
**  the operating mode of the station (AP or AdHoc).  Parameters are AIFS
**  settings and channel width min/max
**
**  \param sc Pointer to ATH object ("this" pointer)
**
**  \return zero for failure, or 1 for success
*/

static int
ath_beaconq_config(struct ath_softc *sc)
{
    struct ath_hal *ah = sc->sc_ah;
    HAL_TXQ_INFO qi;

    ath_hal_gettxqueueprops(ah, sc->sc_bhalq, &qi);
    if (sc->sc_opmode == HAL_M_HOSTAP)
    {
        /*
         * Always burst out beacon and CAB traffic.
         */
        qi.tqi_aifs = 1;
        qi.tqi_cwmin = 0;
        qi.tqi_cwmax = 0;
    }
    else
    {
        /*
         * Adhoc mode; important thing is to use 2x cwmin.
         */
        qi.tqi_aifs = sc->sc_beacon_qi.tqi_aifs;
        qi.tqi_cwmin = 2*sc->sc_beacon_qi.tqi_cwmin;
        qi.tqi_cwmax = sc->sc_beacon_qi.tqi_cwmax;
    }

    if (!ath_hal_settxqueueprops(ah, sc->sc_bhalq, &qi))
    {
        printk("%s: unable to update h/w beacon queue parameters\n",
               __func__);
        return 0;
    }
    else
    {
        ath_hal_resettxqueue(ah, sc->sc_bhalq); /* push to h/w */
        return 1;
    }
}

/******************************************************************************/
/*!
**  \brief Allocate and setup an initial beacon frame.
**
**  Allocate a beacon state variable for a specific VAP instance created on
**  the ATH interface.  This routine also calculates the beacon "slot" for
**  staggared beacons in the mBSSID case.
**
**  \param sc Pointer to ATH object ("this" pointer).
**  \param if_id Index of the specific VAP of interest.
**
**  \return -EINVAL if there is no function in the upper layer assigned to
**  \return         beacon transmission
**  \return -ENOMEM if no wbuf is available
**  \return   0     for success
*/

int
ath_beacon_alloc(struct ath_softc *sc, int if_id)
{
    struct ath_vap *avp;
    struct ieee80211_frame *wh;
    struct ath_buf *bf;
    wbuf_t wbuf;

    /*
    ** Code Begins
    */

    avp = sc->sc_vaps[if_id];
    ASSERT(avp);
    
    /* Allocate a beacon descriptor if we haven't done so. */
    if(!avp->av_bcbuf)
    {
        /*
         * Allocate beacon state for hostap/ibss.  We know
         * a buffer is available.
         */
        
        avp->av_bcbuf = TAILQ_FIRST(&sc->sc_bbuf);
        TAILQ_REMOVE(&sc->sc_bbuf, avp->av_bcbuf, bf_list);
        
        if (sc->sc_opmode == HAL_M_HOSTAP || !sc->sc_hasveol)
        {
            int slot;
            /*
             * Assign the vap to a beacon xmit slot.  As
             * above, this cannot fail to find one.
             */
            avp->av_bslot = 0;
            for (slot = 0; slot < ATH_BCBUF; slot++)
                if (sc->sc_bslot[slot] == ATH_IF_ID_ANY)
                {
                    /*
                     * XXX hack, space out slots to better
                     * deal with misses
                     */
                    if (slot+1 < ATH_BCBUF &&
                        sc->sc_bslot[slot+1] == ATH_IF_ID_ANY)
                    {
                        avp->av_bslot = slot+1;
                        break;
                    }
                    avp->av_bslot = slot;
                    /* NB: keep looking for a double slot */
                }
            KASSERT(sc->sc_bslot[avp->av_bslot] == ATH_IF_ID_ANY,
                    ("beacon slot %u not empty?", avp->av_bslot));
            sc->sc_bslot[avp->av_bslot] = if_id;
            sc->sc_nbcnvaps++;
        }
    }
    
    /*
     * release the previous beacon frame , if it already exists.
     */
    bf = avp->av_bcbuf;
    if (bf->bf_mpdu != NULL)
    {
        ieee80211_tx_status_t tx_status;
        
        wbuf = (wbuf_t)bf->bf_mpdu;
        wbuf_unmap_single(sc->sc_osdev, wbuf, BUS_DMA_TODEVICE,
                          OS_GET_DMA_MEM_CONTEXT(bf, bf_dmacontext));
        tx_status.flags = 0;
        tx_status.retries = 0;
        sc->sc_ieee_ops->tx_complete(wbuf, &tx_status);
        bf->bf_mpdu = NULL;
    }
    
    /*
     * NB: the beacon data buffer must be 32-bit aligned;
    * we assume the wbuf routines will return us something
     * with this alignment (perhaps should assert).
     */
    if (!sc->sc_ieee_ops->get_beacon)
    {
        /* Protocol layer doesn't support beacon generation for host driver */
        return -EINVAL;
    }
    wbuf = sc->sc_ieee_ops->get_beacon(sc->sc_ieee, if_id, &avp->av_boff, &avp->av_btxctl);
    if (wbuf == NULL)
    {
        DPRINTF(sc, ATH_DEBUG_BEACON, "%s: cannot get wbuf\n",
                __func__);
        sc->sc_stats.ast_be_nobuf++;
        return -ENOMEM;
    }

    /*
     * Calculate a TSF adjustment factor required for
     * staggered beacons.  Note that we assume the format
     * of the beacon frame leaves the tstamp field immediately
     * following the header.
     */
    if (sc->sc_stagbeacons && avp->av_bslot > 0)
    {
        u_int64_t tsfadjust;
        int intval;

        if (sc->sc_ieee_ops->get_beacon_config)
        {
            ieee80211_beacon_config_t conf;
            
            sc->sc_ieee_ops->get_beacon_config(sc->sc_ieee, if_id, &conf);
            intval = conf.beacon_interval;
        }
        else
            intval = ATH_DEFAULT_BINTVAL;
        
        /*
         * The beacon interval is in TU's; the TSF in usecs.
         * We figure out how many TU's to add to align the
         * timestamp then convert to TSF units and handle
         * byte swapping before writing it in the frame.
         * The hardware will then add this each time a beacon
         * frame is sent.  Note that we align vap's 1..N
         * and leave vap 0 untouched.  This means vap 0
         * has a timestamp in one beacon interval while the
         * others get a timestamp aligned to the next interval.
         */
        tsfadjust = (intval * (ATH_BCBUF - avp->av_bslot)) / ATH_BCBUF;
        tsfadjust = cpu_to_le64(tsfadjust<<10);     /* TU->TSF */

        DPRINTF(sc, ATH_DEBUG_BEACON,
                "%s: %s beacons, bslot %d intval %u tsfadjust %llu\n",
                __func__, sc->sc_stagbeacons ? "stagger" : "burst",
                avp->av_bslot, intval, (unsigned long long)tsfadjust);

        wh = (struct ieee80211_frame *)wbuf_header(wbuf);
        OS_MEMCPY(&wh[1], &tsfadjust, sizeof(tsfadjust));
    }

    bf->bf_buf_addr = wbuf_map_single(sc->sc_osdev, wbuf, BUS_DMA_TODEVICE,
                                      OS_GET_DMA_MEM_CONTEXT(bf, bf_dmacontext));
    bf->bf_mpdu = wbuf;

    return 0;
}

/******************************************************************************/
/*!
**  \brief Setup the beacon frame for transmit.
**
**  Associates the beacon frame buffer with a transmit descriptor.  Will set
**  up all required antenna switch parameters, rate codes, and channel flags.
**  Beacons are always sent out at the lowest rate, and are not retried.
**
**  \param sc Pointer to ATH object ("this" pointer)
**  \param avp Pointer to VAP object (ieee802.11 layer object)for the beacon
**  \param bf Pointer to ATH buffer containing the beacon frame.
**  \return N/A
*/

static void
ath_beacon_setup(struct ath_softc *sc, struct ath_vap *avp, struct ath_buf *bf)
{
    wbuf_t wbuf = (wbuf_t)bf->bf_mpdu;
    struct ath_hal *ah = sc->sc_ah;
    struct ath_desc *ds;
    int flags, antenna;
    const HAL_RATE_TABLE *rt;
    u_int8_t rix, rate;
    int ctsrate = 0;
    int ctsduration = 0;
    HAL_11N_RATE_SERIES  series[4];

    /*
    ** Code Begins
    */

    DPRINTF(sc, ATH_DEBUG_BEACON_PROC, "%s: m %p len %u\n",
            __func__, wbuf, wbuf_get_pktlen(wbuf));

    /* setup descriptors */
    ds = bf->bf_desc;

    flags = HAL_TXDESC_NOACK;
#ifdef ATH_SUPERG_DYNTURBO
    if (sc->sc_dturbo_switch)
        flags |= HAL_TXDESC_INTREQ;
#endif

    if (sc->sc_opmode == HAL_M_IBSS && sc->sc_hasveol)
    {
        ds->ds_link = bf->bf_daddr; /* self-linked */
        flags |= HAL_TXDESC_VEOL;
        /*
         * Let hardware handle antenna switching.
         */
        antenna = 0;
    }
    else
    {
        ds->ds_link = 0;
        /*
         * Switch antenna every beacon.
         * Should only switch every beacon period, not for every
         * SWBA's
         * XXX assumes two antenna
         */
        if (sc->sc_stagbeacons)
            antenna = ((sc->sc_stats.ast_be_xmit / sc->sc_nbcnvaps) & 1 ? 2 : 1);
        else
            antenna = (sc->sc_stats.ast_be_xmit & 1 ? 2 : 1);
    }

    ds->ds_data = bf->bf_buf_addr;
    
    /*
     * Calculate rate code.
     * XXX everything at min xmit rate
     */
    rix = sc->sc_minrateix;
    rt = sc->sc_currates;
    rate = rt->info[rix].rateCode;
    if (avp->av_btxctl.shortPreamble)
        rate |= rt->info[rix].shortPreamble;

#ifndef REMOVE_PKT_LOG
        ds->ds_vdata = wbuf_header(wbuf);
#endif

    ath_hal_set11n_txdesc(ah, ds
                          , wbuf_get_pktlen(wbuf) + IEEE80211_CRC_LEN /* frame length */
                          , HAL_PKT_TYPE_BEACON                 /* Atheros packet type */
                          , avp->av_btxctl.txpower              /* txpower XXX */
                          , HAL_TXKEYIX_INVALID                 /* no encryption */
                          , HAL_KEY_TYPE_CLEAR                  /* no encryption */
                          , flags                               /* no ack, veol for beacons */
                          );

    /* NB: beacon's BufLen must be a multiple of 4 bytes */
    ath_hal_filltxdesc(ah, ds
                       , roundup(wbuf_get_pktlen(wbuf), 4)      /* buffer length */
                       , AH_TRUE                                /* first segment */
                       , AH_TRUE                                /* last segment */
                       , ds                                     /* first descriptor */
                       );

    OS_MEMZERO(series, sizeof(HAL_11N_RATE_SERIES) * 4);
    series[0].Tries = 1;
    series[0].Rate = rate;
    series[0].ChSel = sc->sc_tx_chainmask;
    series[0].RateFlags = (ctsrate) ? HAL_RATESERIES_RTS_CTS : 0;
    ath_hal_set11n_ratescenario(ah, ds, ds, 0, ctsrate, ctsduration, series, 4, 0);

    /* NB: The desc swap function becomes void, 
     * if descriptor swapping is not enabled
     */
    ath_desc_swap(ds);
}

/******************************************************************************/
/*!
**  \brief Generate beacon frame and queue cab data for a vap.
**
**  Updates the contents of the beacon frame.  It is assumed that the buffer for
**  the beacon frame has been allocated in the ATH object, and simply needs to
**  be filled for this cycle.  Also, any CAB (crap after beacon?) traffic will
**  be added to the beacon frame at this point.
**
**  \param sc Pointer to ATH object ("this" pointer)
**  \param if_id Index to VAP object that is sending the beacon
**  \param needmark Pointer to integer.  Only used in dynturbo mode, and deprecated at that!
**  \return a pointer to an allocated ath buffer containing the beacon frame,
**  \return or NULL if not successful
*/
/*
 * 
 */
static struct ath_buf *
ath_beacon_generate(struct ath_softc *sc, int if_id, int *needmark)
{
    struct ath_hal *ah = sc->sc_ah;
    struct ath_buf *bf;
    struct ath_vap *avp;
    wbuf_t wbuf;
    int ncabq;
    unsigned int curlen;
    
#ifdef ATH_SUPERG_XR
    if (ieee80211vap_has_flag(vap, IEEE80211_F_XR))
    {
        vap->iv_xrbcnwait++;
        /* wait for XR_BEACON_FACTOR times before sending the beacon */
        if (vap->iv_xrbcnwait < IEEE80211_XR_BEACON_FACTOR)
            return NULL;
        vap->iv_xrbcnwait = 0;
    }
#endif
    
    avp = sc->sc_vaps[if_id];
    ASSERT(avp);
    
    if (avp->av_bcbuf == NULL)
    {
        DPRINTF(sc, ATH_DEBUG_ANY, "%s: avp=%p av_bcbuf=%p\n",
                __func__, avp, avp->av_bcbuf);
        return NULL;
    }
    bf = avp->av_bcbuf;
    wbuf = (wbuf_t)bf->bf_mpdu;

#ifdef ATH_SUPERG_DYNTURBO
    /* 
     * If we are using dynamic turbo, update the
     * capability info and arrange for a mode change
     * if needed.
     */
    if (sc->sc_dturbo)
    {
        u_int8_t dtim;
        dtim = ((avp->av_boff.bo_tim[2] == 1) || (avp->av_boff.bo_tim[3] == 1));
#ifdef notyet        
        ath_beacon_dturbo_update(vap, needmark, dtim);
#endif        
    }
#endif
    /*
     * Update dynamic beacon contents.  If this returns
     * non-zero then we need to remap the memory because
     * the beacon frame changed size (probably because
     * of the TIM bitmap).
     */
    curlen = wbuf_get_pktlen(wbuf);
    ncabq = avp->av_mcastq.axq_depth;

    if (!sc->sc_ieee_ops->update_beacon)
    {
        /*
         * XXX: if protocol layer doesn't support update beacon at run-time,
         * we have to free the old beacon and allocate a new one.
         */
        if (sc->sc_ieee_ops->get_beacon)
        {
            ieee80211_tx_status_t tx_status;
            
            wbuf_unmap_single(sc->sc_osdev, wbuf, BUS_DMA_TODEVICE,
                              OS_GET_DMA_MEM_CONTEXT(bf, bf_dmacontext));
            tx_status.flags = 0;
            tx_status.retries = 0;
            sc->sc_ieee_ops->tx_complete(wbuf, &tx_status);

            wbuf = sc->sc_ieee_ops->get_beacon(sc->sc_ieee, if_id, &avp->av_boff, &avp->av_btxctl);
            if (wbuf == NULL)
                return NULL;

            bf->bf_mpdu = wbuf;
            bf->bf_buf_addr = wbuf_map_single(sc->sc_osdev, wbuf, BUS_DMA_TODEVICE,
                                              OS_GET_DMA_MEM_CONTEXT(bf, bf_dmacontext));
        }
    }
    else
    {
        if (sc->sc_ieee_ops->update_beacon(sc->sc_ieee, if_id, &avp->av_boff, wbuf, ncabq) == 1) {
            wbuf_unmap_single(sc->sc_osdev, wbuf, BUS_DMA_TODEVICE,
                              OS_GET_DMA_MEM_CONTEXT(bf, bf_dmacontext));
            bf->bf_buf_addr = wbuf_map_single(sc->sc_osdev, wbuf, BUS_DMA_TODEVICE,
                                              OS_GET_DMA_MEM_CONTEXT(bf, bf_dmacontext));
        } else {
            OS_SYNC_SINGLE(sc->sc_osdev,
                       bf->bf_buf_addr, wbuf_get_pktlen(wbuf), BUS_DMA_TODEVICE,
                       OS_GET_DMA_MEM_CONTEXT(bf, bf_dmacontext));
        }
    }

    /*
     * if the CABQ traffic from previous DTIM is pending and the current
     *  beacon is also a DTIM. 
     *  1) if there is only one vap let the cab traffic continue. 
     *  2) if there are more than one vap and we are using staggered
     *     beacons, then drain the cabq by dropping all the frames in
     *     the cabq so that the current vaps cab traffic can be scheduled.
     */
    if (ncabq && (avp->av_boff.bo_tim[4] & 1) && sc->sc_cabq->axq_depth)
    {
        if (sc->sc_nvaps > 1 && sc->sc_stagbeacons)
        {
            /* stop dma since ath_tx_draintxq() assumes that 
             * output has been stopped
             */
            if (!ath_hal_stoptxdma(ah, sc->sc_cabq->axq_qnum))
            {
                DPRINTF(sc, ATH_DEBUG_ANY, "%s: cabq queue did not stop?\n",
__func__);
                /* NB: the HAL still stops DMA, so proceed */
            }

            ath_tx_draintxq(sc, sc->sc_cabq, AH_FALSE);
            DPRINTF(sc, ATH_DEBUG_BEACON,
                    "%s: flush previous cabq traffic\n", __func__);
        }
    }

    /*
     * Construct tx descriptor.
     */
    ath_beacon_setup(sc, avp, bf);

    /*
     * Enable the CAB queue before the beacon queue to
     * insure cab frames are triggered by this beacon.
     */
    if (avp->av_boff.bo_tim[4] & 1)
    {   /* NB: only at DTIM */
        struct ath_txq *cabq = sc->sc_cabq;
        struct ath_buf *bfmcast;
        /*
         * Move everything from the vap's mcast queue 
         * to the hardware cab queue.
         * XXX MORE_DATA bit?
         */
        ATH_TXQ_LOCK(&avp->av_mcastq);
        ATH_TXQ_LOCK(cabq);
        if ((bfmcast = TAILQ_FIRST(&avp->av_mcastq.axq_q)) != NULL)
        {
            /* link the descriptors */
            if (cabq->axq_link == NULL)
            {
                ath_hal_puttxbuf(ah, cabq->axq_qnum, bfmcast->bf_daddr);
            }
            else
            {
#ifdef AH_NEED_DESC_SWAP
                *cabq->axq_link = cpu_to_le32(bfmcast->bf_daddr);
#else
                *cabq->axq_link = bfmcast->bf_daddr;
#endif
            }
            /* append the private vap mcast list to  the cabq */
            ATH_TXQ_MOVE_MCASTQ(&avp->av_mcastq, cabq);
#ifdef ATH_ADDITIONAL_STATS
            sc->sc_stats.ast_txq_packets[cabq->axq_qnum]++;
#endif
        }
        /* NB: gated by beacon so safe to start here */
        if (!TAILQ_EMPTY(&(cabq->axq_q)))
        {
            ath_hal_txstart(ah, cabq->axq_qnum);
        }
        ATH_TXQ_UNLOCK(cabq);
        ATH_TXQ_UNLOCK(&avp->av_mcastq);
    }

    return bf;
}

/******************************************************************************/
/*!
**  \brief Startup beacon transmission for adhoc mode
**
** Startup beacon transmission for adhoc mode when they are sent entirely
** by the hardware using the self-linked descriptor + veol trick.
**
**  \param sc Pointer to ATH object. "This" pointer.
**  \param if_id Integer containing index of VAP interface to start the beacon on.
**
**  \return N/A
*/

static void
ath_beacon_start_adhoc(struct ath_softc *sc, int if_id)
{
    struct ath_hal *ah = sc->sc_ah;
    struct ath_buf *bf;
    struct ath_vap *avp;
    wbuf_t wbuf;

    /*
    ** Code Begins
    */

    avp = sc->sc_vaps[if_id];
    ASSERT(avp);
    
    if (avp->av_bcbuf == NULL)
    {
        DPRINTF(sc, ATH_DEBUG_ANY, "%s: avp=%p av_bcbuf=%p\n",
                __func__, avp, avp != NULL ? avp->av_bcbuf : NULL);
        return;
    }
    bf = avp->av_bcbuf;
    wbuf = (wbuf_t)bf->bf_mpdu;

    /* XXX: We don't do ATIM, so we don't need to update beacon contents */
#if 0
    /*
     * Update dynamic beacon contents.  If this returns
     * non-zero then we need to remap the memory because
     * the beacon frame changed size (probably because
     * of the TIM bitmap).
     */
    /*
     * XXX: This function should be called from ath_beacon_tasklet,
     * which is in ISR context, thus use the locked version.
     * Better to have an assertion to verify that.
     */
    if (sc->sc_ieee_ops->update_beacon &&
        sc->sc_ieee_ops->update_beacon(ni, &avp->av_boff, wbuf, 0) == 0)
    {
        wbuf_unmap_single(sc->sc_osdev, wbuf,
                          BUS_DMA_TODEVICE,
                          OS_GET_DMA_MEM_CONTEXT(bf, bf_dmacontext));
        bf->bf_buf_addr = wbuf_map_single(sc->sc_osdev, wbuf,
                                          BUS_DMA_TO_DEVICE,
                                          OS_GET_DMA_MEM_CONTEXT(bf, bf_dmacontext));
    }
#endif

    /*
     * Construct tx descriptor.
     */
    ath_beacon_setup(sc, avp, bf);

    /* NB: caller is known to have already stopped tx dma */
    ath_hal_puttxbuf(ah, sc->sc_bhalq, bf->bf_daddr);
    ath_hal_txstart(ah, sc->sc_bhalq);
    DPRINTF(sc, ATH_DEBUG_BEACON_PROC, "%s: TXDP%u = %llx (%p)\n", __func__,
            sc->sc_bhalq, ito64(bf->bf_daddr), bf->bf_desc);
}

/******************************************************************************/
/*!
**  \brief Reclaim beacon resources and return buffer to the pool.
**
**  Checks the VAP to put the beacon frame buffer back to the ATH object
**  queue, and de-allocates any wbuf frames that were sent as CAB traffic.
**
**  \param sc Pointer to ATH object, "this" pointer.
**  \param avp Pointer to VAP object that sent the beacon
**
**  \return N/A
*/

void
ath_beacon_return(struct ath_softc *sc, struct ath_vap *avp)
{
    if (avp->av_bcbuf != NULL)
    {
        struct ath_buf *bf;

        if (avp->av_bslot != -1)
        {
            sc->sc_bslot[avp->av_bslot] = ATH_IF_ID_ANY;
            sc->sc_nbcnvaps--;
        }

        bf = avp->av_bcbuf;
        if (bf->bf_mpdu != NULL)
        {
            wbuf_t wbuf = (wbuf_t)bf->bf_mpdu;
            ieee80211_tx_status_t tx_status;

            wbuf_unmap_single(sc->sc_osdev, wbuf, BUS_DMA_TODEVICE,
                              OS_GET_DMA_MEM_CONTEXT(bf, bf_dmacontext));
            tx_status.flags = 0;
            tx_status.retries = 0;
            sc->sc_ieee_ops->tx_complete(wbuf, &tx_status);
            bf->bf_mpdu = NULL;
        }
        TAILQ_INSERT_TAIL(&sc->sc_bbuf, bf, bf_list);

        avp->av_bcbuf = NULL;
    }
}

/******************************************************************************/
/*!
**  \brief Reclaim beacon resources and return buffer to the pool.
**
**  This function will free any wbuf frames that are still attached to the
**  beacon buffers in the ATH object.  Note that this does not de-allocate
**  any wbuf objects that are in the transmit queue and have not yet returned
**  to the ATH object.
**
**  \param sc Pointer to ATH object, "this" pointer
**
**  \return N/A
*/

void
ath_beacon_free(struct ath_softc *sc)
{
    struct ath_buf *bf;

    TAILQ_FOREACH(bf, &sc->sc_bbuf, bf_list) {
        if (bf->bf_mpdu != NULL)
        {
            wbuf_t wbuf = (wbuf_t)bf->bf_mpdu;
            ieee80211_tx_status_t tx_status;

            wbuf_unmap_single(sc->sc_osdev, wbuf, BUS_DMA_TODEVICE,
                              OS_GET_DMA_MEM_CONTEXT(bf, bf_dmacontext));
            tx_status.flags = 0;
            tx_status.retries = 0;
            sc->sc_ieee_ops->tx_complete(wbuf, &tx_status);
            bf->bf_mpdu = NULL;
        }
    }
}

#ifdef ATH_SUPPORT_DFS
static int
ath_handle_dfs_bb_hang(struct ath_softc *sc)
{
    if (sc->sc_dfs_hang.hang_war_activated) {
        sc->sc_bmisscount=0;
        printk("ALREADY ACTIVATED\n");
        return 0;
    }

    if (sc->sc_curchan.channelFlags & CHANNEL_HT20) {
        /*=========DFS HT20 HANG========*/
        /* moved marking PHY inactive to HAL reset */
        sc->sc_dfs_hang.hang_war_ht20count++;
        return 0;
    } else {
        /*=========DFS HT40 HANG========*/
        sc->sc_dfs_hang.hang_war_ht40count++;
        sc->sc_dfs_hang.hang_war_activated = 1;
        OS_CANCEL_TIMER(&sc->sc_dfs_hang.hang_war_timer);
                        sc->sc_dfs_hang.hang_war_timeout =
        get_dfs_hang_war_timeout(sc);
        OS_SET_TIMER(&sc->sc_dfs_hang.hang_war_timer,
                     sc->sc_dfs_hang.hang_war_timeout);
        /* Switch to static20 mode, clear 0x981C before
         * channel change
         */
        /* moved marking PHY inactive to HAL reset */
        sc->sc_ieee_ops->ath_net80211_switch_mode_static20(sc->sc_ieee);
        sc->sc_bmisscount=0;
        return 1;
    } /*End HT40 WAR*/
}
#endif

/*
 * Determines if the device currently has a BB or MAC hang.
 */
int
ath_hw_hang_check(struct ath_softc *sc)
{
    sc->sc_hang_check = AH_FALSE;

    /* Do we have a BB hang?
     */
    if (ATH_BB_HANG_WAR_REQ(sc)) {
        if (AH_TRUE == ath_hal_is_bb_hung(sc->sc_ah)) {
#ifdef ATH_SUPPORT_DFS
            /* Found a DFS related BB hang */
            if (ATH_DFS_BB_HANG_WAR_REQ(sc) && !ath_get_phy_err_rate(sc)) {
                if (ath_handle_dfs_bb_hang(sc)) {
                    ATH_BB_GENERIC_HANG(sc);
                    return 1;
                }
            }
#endif
            /* Found a generic BB hang */
            if (ATH_BB_GENERIC_HANG_WAR_REQ(sc)) {
                ATH_BB_GENERIC_HANG(sc);
                return 1;
            }
        }
    }

    /* Do we have a MAC hang?
     */
    if (ATH_MAC_HANG_WAR_REQ(sc)) {
        if (AH_TRUE == ath_hal_is_mac_hung(sc->sc_ah)) {
            ATH_MAC_GENERIC_HANG(sc);
            return 1;
        }
    }

    return 0; 
}

/******************************************************************************/
/*!
**  \brief Task for Sending Beacons
**
** Transmit one or more beacon frames at SWBA.  Dynamic updates to the frame
** contents are done as needed and the slot time is also adjusted based on
** current state.
**
** \warning This task is not scheduled, it's called in ISR context.
**
**  \param sc Pointer to ATH object ("this" pointer).
**  \param needmark Pointer to integer value indicating that the beacon miss
**                  threshold exceeded
**  \return Describe return value, or N/A for void
*/

void
ath_beacon_tasklet(struct ath_softc *sc, int *needmark)
{
#define TSF_TO_TU(_h,_l) \
    ((((u_int32_t)(_h)) << 22) | (((u_int32_t)(_l)) >> 10))
    
    struct ath_hal *ah = sc->sc_ah;
    struct ath_buf *bf=NULL;
    int slot, if_id;
    u_int32_t bfaddr;
    u_int32_t rx_clear = 0, rx_frame = 0, tx_frame = 0;
    u_int32_t show_cycles = 0;
    u_int32_t bc = 0; /* beacon count */

#ifdef ATH_SUPPORT_DFS
    /* Make sure no beacons go out during the DFS wait period (CAC timer)*/
    if (sc->sc_dfs && sc->sc_dfs->sc_dfswait) {
       return;
    }
#endif

    /*
     * Check if the previous beacon has gone out.  If
     * not don't try to post another, skip this period
     * and wait for the next.  Missed beacons indicate
     * a problem and should not occur.  If we miss too
     * many consecutive beacons reset the device.
     */
    if (ath_hal_numtxpending(ah, sc->sc_bhalq) != 0)
    {
        show_cycles = ath_hal_getMibCycleCountsPct(ah, 
                      &rx_clear, &rx_frame, &tx_frame);

        sc->sc_bmisscount++;
        /* XXX: doth needs the chanchange IE countdown decremented.
         *      We should consider adding a net80211 call to indicate
         *      a beacon miss so appropriate action could be taken
         *      (in that layer).
         */
        if (sc->sc_bmisscount < sc->sc_bsthresh) {

            if (show_cycles && !tx_frame && (rx_clear >= 99)) {
                /* Meant to reduce PHY errors and potential false detects */
                if (sc->sc_toggle_immunity)
                    ath_hal_set_immunity(ah, AH_TRUE);

                sc->sc_noise++;
            }

            if (sc->sc_noreset) {
                printk("%s: missed %u consecutive beacons\n",
                       __func__, sc->sc_bmisscount);
                if (show_cycles) {
                    /*
                     * Display cycle counter stats from HAL to
                     * aide in debug of stickiness.
                     */
                    printk("%s: busy times: rx_clear=%d, rx_frame=%d, tx_frame=%d\n", __func__, rx_clear, rx_frame, tx_frame);
                } else {
                    printk("%s: unable to obtain busy times\n", __func__);
                }
            } else {
                DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
                        "%s: missed %u consecutive beacons\n",
                        __func__, sc->sc_bmisscount);
            }

            /* If the HW requires hang detection, notify caller
             * and detect in the tasklet context.
             */
            if (ATH_HANG_WAR_REQ(sc)) {
                sc->sc_hang_check = AH_TRUE;
                *needmark = 1;
                return;
            }
        } else if (sc->sc_bmisscount >= sc->sc_bsthresh) {
            if (sc->sc_noreset) {
                if (sc->sc_bmisscount == sc->sc_bsthresh) {
                    printk("%s: beacon is officially stuck\n",
                           __func__);
                    ath_hal_dmaRegDump(ah);
                }
            } else {
                DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
                        "%s: beacon is officially stuck\n",
                        __func__);

                if (show_cycles && rx_clear >= 99) {
                    sc->sc_hang_war |= HAL_BB_HANG_DETECTED;

                    if (rx_frame >= 99) {
                        printk("Busy environment detected\n");
                    } else {
                        printk("Inteference detected\n");
                    }

                    printk("rx_clear=%d, rx_frame=%d, tx_frame=%d\n",
                           rx_clear, rx_frame, tx_frame);

                    sc->sc_noise = 0;
                }
                *needmark = 1;
            }
        }

        return;
    }

    if (sc->sc_toggle_immunity)
        ath_hal_set_immunity(ah, AH_FALSE);
    sc->sc_noise = 0;
    sc->sc_hang_check = AH_FALSE;

    if (sc->sc_bmisscount != 0)
    {
        if (sc->sc_noreset) {
            printk("%s: resume beacon xmit after %u misses\n",
                   __func__, sc->sc_bmisscount);
        } else {
            DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
                    "%s: resume beacon xmit after %u misses\n",
                    __func__, sc->sc_bmisscount);
        }
        sc->sc_bmisscount = 0;
    }

    /*
     * Generate beacon frames.  If we are sending frames
     * staggered then calculate the slot for this frame based
     * on the tsf to safeguard against missing an swba.
     * Otherwise we are bursting all frames together and need
     * to generate a frame for each vap that is up and running.
     */
    if (sc->sc_stagbeacons)
    {       /* staggered beacons */
        u_int64_t tsf;
        u_int32_t tsftu;
        u_int16_t intval;

        if (sc->sc_ieee_ops->get_beacon_config)
        {
            ieee80211_beacon_config_t conf;
            sc->sc_ieee_ops->get_beacon_config(sc->sc_ieee, ATH_IF_ID_ANY, &conf);
            intval = conf.beacon_interval;
        }
        else
            intval = ATH_DEFAULT_BINTVAL;
        
        tsf = ath_hal_gettsf64(ah);
        tsftu = TSF_TO_TU(tsf>>32, tsf);
        if (intval == 0) {
            /*
             * This should not happen. We're seeing zero bintval sometimes 
             * in WDS testing but is not easily reproducible 
             */
            return;
        }
        slot = ((tsftu % intval) * ATH_BCBUF) / intval;
        if_id = sc->sc_bslot[(slot + 1) % ATH_BCBUF];
        DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
                "%s: slot %d [tsf %llu tsftu %u intval %u] if_id %d\n",
                __func__, slot, (unsigned long long)tsf, tsftu, intval, if_id);
        bfaddr = 0;
        if (if_id != ATH_IF_ID_ANY)
        {
            bf = ath_beacon_generate(sc, if_id, needmark);
            if (bf != NULL) {
                bfaddr = bf->bf_daddr;
                bc = 1;
            }
        }
    }
    else
    {                        /* burst'd beacons */
#ifdef _WIN64
        u_int32_t __unaligned *bflink;
#else
        u_int32_t *bflink;
#endif


        bflink = &bfaddr;
        /* XXX rotate/randomize order? */
        for (slot = 0; slot < ATH_BCBUF; slot++)
        {
            if_id = sc->sc_bslot[slot];
            if (if_id != ATH_IF_ID_ANY)
            {
                bf = ath_beacon_generate(sc, if_id, needmark);
                if (bf != NULL)
                {
#ifdef AH_NEED_DESC_SWAP
                    if(bflink != &bfaddr)
                        *bflink = cpu_to_le32(bf->bf_daddr);
                    else
                        *bflink = bf->bf_daddr;
#else
                    *bflink = bf->bf_daddr;
#endif
                    bflink = &bf->bf_desc->ds_link;
                    bc ++;
                }
            }
        }
        *bflink = 0;    /* link of last frame */
    }

    /*
     * Handle slot time change when a non-ERP station joins/leaves
     * an 11g network.  The 802.11 layer notifies us via callback,
     * we mark updateslot, then wait one beacon before effecting
     * the change.  This gives associated stations at least one
     * beacon interval to note the state change.
     *
     * NB: The slot time change state machine is clocked according
     *     to whether we are bursting or staggering beacons.  We
     *     recognize the request to update and record the current
     *     slot then don't transition until that slot is reached
     *     again.  If we miss a beacon for that slot then we'll be
     *     slow to transition but we'll be sure at least one beacon
     *     interval has passed.  When bursting slot is always left
     *     set to ATH_BCBUF so this check is a noop.
     */
    /* XXX locking */
    if (sc->sc_updateslot == UPDATE)
    {
        sc->sc_updateslot = COMMIT; /* commit next beacon */
        sc->sc_slotupdate = slot;
    }
    else if (sc->sc_updateslot == COMMIT && sc->sc_slotupdate == slot)
        ath_setslottime(sc);        /* commit change to hardware */

    if ((!sc->sc_stagbeacons || slot == 0) && (!sc->sc_diversity))
    {
        int otherant;
        /*
         * Check recent per-antenna transmit statistics and flip
         * the default rx antenna if noticeably more frames went out
         * on the non-default antenna.  Only do this if rx diversity
         * is off.
         * XXX assumes 2 anntenae
         */
        otherant = sc->sc_defant & 1 ? 2 : 1;
        if (sc->sc_ant_tx[otherant] > sc->sc_ant_tx[sc->sc_defant] + ATH_ANTENNA_DIFF)
        {
            DPRINTF(sc, ATH_DEBUG_BEACON,
                    "%s: flip defant to %u, %u > %u\n",
                    __func__, otherant, sc->sc_ant_tx[otherant],
                    sc->sc_ant_tx[sc->sc_defant]);
            ath_setdefantenna(sc, otherant);
        }
        sc->sc_ant_tx[1] = sc->sc_ant_tx[2] = 0;
    }

    if (bfaddr != 0)
    {
        /*
         * Stop any current dma and put the new frame(s) on the queue.
         * This should never fail since we check above that no frames
         * are still pending on the queue.
         */
        if (!ath_hal_stoptxdma(ah, sc->sc_bhalq))
        {
            DPRINTF(sc, ATH_DEBUG_ANY,
                    "%s: beacon queue %u did not stop?\n",
                    __func__, sc->sc_bhalq);
            /* NB: the HAL still stops DMA, so proceed */
        }

#ifndef REMOVE_PKT_LOG

        {
            struct log_tx log_data;
            log_data.firstds = bf->bf_desc;
            log_data.bf = bf;
            ath_log_txctl(sc, &log_data, 0);
            log_data.lastds = bf->bf_desc;
            ath_log_txstatus(sc, &log_data, 0);
        }

#endif
        /* NB: cabq traffic should already be queued and primed */
        ath_hal_puttxbuf(ah, sc->sc_bhalq, bfaddr);
        ath_hal_txstart(ah, sc->sc_bhalq);

        sc->sc_stats.ast_be_xmit += bc;     /* XXX per-vap? */
#ifdef ATH_ADDITIONAL_STATS
        sc->sc_stats.ast_txq_packets[sc->sc_bhalq]++;
#endif
    }
#undef TSF_TO_TU
}

/******************************************************************************/
/*!
**  \brief Task or Beacon Stuck processing
**
**  Processing for Beacon Stuck. 
**  Basically calls the ath_internal_reset function to reset the chip.
**
**  \param data Pointer to the ATH object (ath_softc).
**
**  \return N/A
*/

void
ath_bstuck_tasklet(struct ath_softc *sc)
{
#ifdef ATH_SUPPORT_DFS
    sc->sc_dfs_hang.total_stuck++;
#endif
    if (!ATH_HANG_DETECTED(sc)) {
        printk("%s: stuck beacon; resetting (bmiss count %u)\n",
               __func__, sc->sc_bmisscount);
    }
    ath_internal_reset(sc);
    sc->sc_stats.ast_resetOnError++;
}

/*
 * Configure the beacon and sleep timers.
 *
 * When operating as an AP this resets the TSF and sets
 * up the hardware to notify us when we need to issue beacons.
 *
 * When operating in station mode this sets up the beacon
 * timers according to the timestamp of the last received
 * beacon and the current TSF, configures PCF and DTIM
 * handling, programs the sleep registers so the hardware
 * will wakeup in time to receive beacons, and configures
 * the beacon miss handling so we'll receive a BMISS
 * interrupt when we stop seeing beacons from the AP
 * we've associated with.
 */
void
ath_beacon_config(struct ath_softc *sc, int if_id)
{
#define TSF_TO_TU(_h,_l) \
    ((((u_int32_t)(_h)) << 22) | (((u_int32_t)(_l)) >> 10))
    struct ath_hal *ah = sc->sc_ah;
    u_int32_t nexttbtt, intval;
    ieee80211_beacon_config_t conf;
    HAL_OPMODE av_opmode;
 
    if ( if_id != ATH_IF_ID_ANY ) 
    {
        av_opmode = sc->sc_vaps[if_id]->av_opmode;
    }
    else
    {
        av_opmode = sc->sc_opmode;
    }

    if ((av_opmode == HAL_M_STA) &&
        (sc->sc_opmode == HAL_M_HOSTAP)) {
        /*
         * no need to program beacon timers for a STA vap
         * when the chip is operating in AP mode (WDS,Direct Connect).
         */
        DPRINTF(sc, ATH_DEBUG_BEACON,"%s\n", "chip op mode is AP, ignore beacon config request from sta vap");
        return;
    }

    OS_MEMZERO(&conf, sizeof(ieee80211_beacon_config_t));

    /* Query beacon configuration first */
    if (sc->sc_ieee_ops->get_beacon_config)
    {
        sc->sc_ieee_ops->get_beacon_config(sc->sc_ieee, if_id, &conf);
    }
    else
    {
        /*
         * Protocol stack doesn't support dynamic beacon configuration,
         * use default configurations.
         */
        conf.beacon_interval = ATH_DEFAULT_BINTVAL;
        conf.listen_interval = 1;
        conf.dtim_period = conf.beacon_interval;
        conf.dtim_count = 1;
        conf.bmiss_timeout = ATH_DEFAULT_BMISS_LIMIT * conf.beacon_interval;
    }
    
    /* extract tstamp from last beacon and convert to TU */
    nexttbtt = TSF_TO_TU(LE_READ_4(conf.u.last_tstamp + 4),
                         LE_READ_4(conf.u.last_tstamp));
    /* XXX conditionalize multi-bss support? */
    if (sc->sc_opmode == HAL_M_HOSTAP)
    {
        /*
         * For multi-bss ap support beacons are either staggered
         * evenly over N slots or burst together.  For the former
         * arrange for the SWBA to be delivered for each slot.
         * Slots that are not occupied will generate nothing. 
         */
        /* NB: the beacon interval is kept internally in TU's */
        intval = conf.beacon_interval & HAL_BEACON_PERIOD;
        if (sc->sc_stagbeacons)
            intval /= ATH_BCBUF;    /* for staggered beacons */
    }
    else
    {
        intval = conf.beacon_interval & HAL_BEACON_PERIOD;
    }
    if (nexttbtt == 0)      /* e.g. for ap mode */
        nexttbtt = intval;
    else if (intval)        /* NB: can be 0 for monitor mode */
        nexttbtt = roundup(nexttbtt, intval);
    DPRINTF(sc, ATH_DEBUG_BEACON, "%s: av_opmode %d sc_opmode %d nexttbtt %u intval %u (%u)\n",
            __func__, av_opmode, sc->sc_opmode, nexttbtt, intval, conf.beacon_interval);
    /* Check for HAL_M_HOSTAP and sc_nostabeacons for WDS client */
    if (sc->sc_opmode == HAL_M_STA) 
    {
        HAL_BEACON_STATE bs;
        u_int64_t tsf;
        u_int32_t tsftu;
        int dtimperiod, dtimcount, sleepduration;
        int cfpperiod, cfpcount;

        /*
         * Setup dtim and cfp parameters according to
         * last beacon we received (which may be none).
         */
        dtimperiod = conf.dtim_period;
        if (dtimperiod <= 0)        /* NB: 0 if not known */
            dtimperiod = 1;
        dtimcount = conf.dtim_count;
        if (dtimcount >= dtimperiod)    /* NB: sanity check */
            dtimcount = 0;      /* XXX? */
        cfpperiod = 1;          /* NB: no PCF support yet */
        cfpcount = 0;

        sleepduration = conf.listen_interval * intval;
        if (sleepduration <= 0)  
            sleepduration = intval;

#define FUDGE   2
        /*
         * Pull nexttbtt forward to reflect the current
         * TSF and calculate dtim+cfp state for the result.
         */
        tsf = ath_hal_gettsf64(ah);
        tsftu = TSF_TO_TU(tsf>>32, tsf) + FUDGE;
        do
        {
            nexttbtt += intval;
            if (--dtimcount < 0)
            {
                dtimcount = dtimperiod - 1;
                if (--cfpcount < 0)
                    cfpcount = cfpperiod - 1;
            }
        } while (nexttbtt < tsftu);
#undef FUDGE
        OS_MEMZERO(&bs, sizeof(bs));
        bs.bs_intval = intval;
        bs.bs_nexttbtt = nexttbtt;
        bs.bs_dtimperiod = dtimperiod*intval;
        bs.bs_nextdtim = bs.bs_nexttbtt + dtimcount*intval;
        bs.bs_cfpperiod = cfpperiod*bs.bs_dtimperiod;
        bs.bs_cfpnext = bs.bs_nextdtim + cfpcount*bs.bs_dtimperiod;
        bs.bs_cfpmaxduration = 0;
#if 0
        /*
         * The 802.11 layer records the offset to the DTIM
         * bitmap while receiving beacons; use it here to
         * enable h/w detection of our AID being marked in
         * the bitmap vector (to indicate frames for us are
         * pending at the AP).
         * XXX do DTIM handling in s/w to WAR old h/w bugs
         * XXX enable based on h/w rev for newer chips
         */
        bs.bs_timoffset = conf.tim_offset;
#endif
        /*
         * Calculate the number of consecutive beacons to miss
         * before taking a BMISS interrupt.  The configuration
         * is specified in TU so we only need calculate based
         * on the beacon interval.  Note that we clamp the
         * result to at most 15 beacons.
         */
        if (sleepduration > intval)
        {
            bs.bs_bmissthreshold = conf.listen_interval * ATH_DEFAULT_BMISS_LIMIT/2;
        }
        else
        {
            bs.bs_bmissthreshold = howmany(conf.bmiss_timeout, intval);
            if (bs.bs_bmissthreshold > 15)
                bs.bs_bmissthreshold = 15;
            else if (bs.bs_bmissthreshold <= 0)
                bs.bs_bmissthreshold = 1;
        }

        /*
         * Calculate sleep duration.  The configuration is
         * given in ms.  We insure a multiple of the beacon
         * period is used.  Also, if the sleep duration is
         * greater than the DTIM period then it makes senses
         * to make it a multiple of that.
         *
         * XXX fixed at 100ms
         */

        bs.bs_sleepduration =
            roundup(IEEE80211_MS_TO_TU(100), sleepduration);
        if (bs.bs_sleepduration > bs.bs_dtimperiod)
            bs.bs_sleepduration = bs.bs_dtimperiod;

        DPRINTF(sc, ATH_DEBUG_BEACON, 
                "%s: tsf %llu tsf:tu %u intval %u nexttbtt %u dtim %u nextdtim %u bmiss %u sleep %u cfp:period %u maxdur %u next %u timoffset %u\n"
                , __func__
                , (unsigned long long)tsf, tsftu
                , bs.bs_intval
                , bs.bs_nexttbtt
                , bs.bs_dtimperiod
                , bs.bs_nextdtim
                , bs.bs_bmissthreshold
                , bs.bs_sleepduration
                , bs.bs_cfpperiod
                , bs.bs_cfpmaxduration
                , bs.bs_cfpnext
                , bs.bs_timoffset
                );
        
         ath_hal_intrset(ah, 0);
         ath_hal_beacontimers(ah, &bs);
         sc->sc_imask |= HAL_INT_BMISS;
         ath_hal_intrset(ah, sc->sc_imask);
    }
    else
    {
        u_int64_t tsf;
        u_int32_t tsftu;
        ath_hal_intrset(ah, 0);
        if (nexttbtt == intval)
            intval |= HAL_BEACON_RESET_TSF;
        if (sc->sc_opmode == HAL_M_IBSS)
        {
            /*
             * Pull nexttbtt forward to reflect the current
             * TSF .
             */
#define FUDGE   2
            if (!(intval & HAL_BEACON_RESET_TSF))
            {
                tsf = ath_hal_gettsf64(ah);
                tsftu = TSF_TO_TU((u_int32_t)(tsf>>32), (u_int32_t)tsf) + FUDGE;
                do
                {
                    nexttbtt += intval;
                } while (nexttbtt < tsftu);
            }
#undef FUDGE
            DPRINTF(sc, ATH_DEBUG_BEACON, "%s: IBSS nexttbtt %u intval %u (%u)\n",
                    __func__, nexttbtt, intval & ~HAL_BEACON_RESET_TSF,
                    conf.beacon_interval);

            /*
             * In IBSS mode enable the beacon timers but only
             * enable SWBA interrupts if we need to manually
             * prepare beacon frames.  Otherwise we use a
             * self-linked tx descriptor and let the hardware
             * deal with things.
             */
            intval |= HAL_BEACON_ENA;
            if (!sc->sc_hasveol)
                sc->sc_imask |= HAL_INT_SWBA;
            ath_beaconq_config(sc);
        }
        else if (sc->sc_opmode == HAL_M_HOSTAP)
        {
            /*
             * In AP mode we enable the beacon timers and
             * SWBA interrupts to prepare beacon frames.
             */
            intval |= HAL_BEACON_ENA;
            sc->sc_imask |= HAL_INT_SWBA;   /* beacon prepare */
            ath_beaconq_config(sc);
        }
#ifdef ATH_SUPERG_DYNTURBO
#ifdef notyet
        ath_beacon_dturbo_config(vap, intval & 
                                 ~(HAL_BEACON_RESET_TSF | HAL_BEACON_ENA));
#endif                                
#endif
        ath_hal_beaconinit(ah, nexttbtt, intval);
        sc->sc_bmisscount = 0;
        sc->sc_noise = 0;
        ath_hal_intrset(ah, sc->sc_imask);
        /*
         * When using a self-linked beacon descriptor in
         * ibss mode load it once here.
         */
        if (sc->sc_opmode == HAL_M_IBSS && sc->sc_hasveol)
            ath_beacon_start_adhoc(sc, 0);
    }
#undef TSF_TO_TU
}

/*
 * Function to collect beacon rssi data and resync beacon if necessary
 */
void
ath_beacon_sync(ath_dev_t dev, int if_id)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    HAL_OPMODE av_opmode;


    ATH_FUNC_ENTRY_VOID(sc);

    if ( if_id != ATH_IF_ID_ANY )
    {
        av_opmode = sc->sc_vaps[if_id]->av_opmode;
        if ((av_opmode == HAL_M_STA) &&
            (sc->sc_opmode == HAL_M_HOSTAP)) {
            /*
             * no need to program beacon timers for a STA vap
             * when the chip is operating in AP mode (WDS,Direct Connect).
             */
            DPRINTF(sc, ATH_DEBUG_BEACON,"%s\n", "chip op mode is AP, ignore beacon config request from sta vap");
            return;
        }
    }
    /*
     * Resync beacon timers using the tsf of the
     * beacon frame we just received.
     */
    ATH_PS_WAKEUP(sc);
    ath_beacon_config(sc, if_id);
    ATH_PS_SLEEP(sc);
    sc->sc_beacons = 1;
}

/******************************************************************************/
/*!
**  \brief Task for Beacon Miss Interrupt
**
**  This tasklet will notify the upper layer when a beacon miss interrupt
**  has occurred.  The upper layer will perform any required processing.  If
**  the upper layer does not supply a function for this purpose, no other action
**  is taken.
**
**  \param sc Pointer to ATH object, "this" pointer
**
**  \return N/A
*/

void
ath_bmiss_tasklet(struct ath_softc *sc)
{
    DPRINTF(sc, ATH_DEBUG_ANY, "%s\n", __func__);
    if (sc->sc_ieee_ops->notify_beacon_miss)
        sc->sc_ieee_ops->notify_beacon_miss(sc->sc_ieee);
}

#ifdef ATH_SUPPORT_DFS
/* Return PHY error rate per second */
static u_int32_t
ath_get_phy_err_rate(struct ath_softc *sc)
{
    u_int64_t cur_tstamp;
    u_int32_t tstamp_diff, phy_err_rate, total_phy_errors;

    cur_tstamp = ath_hal_gettsf64(sc->sc_ah);
    tstamp_diff = (cur_tstamp - sc->sc_dfs->ath_dfs_stats.last_reset_tstamp);

    total_phy_errors = sc->sc_dfs->ath_dfs_stats.total_phy_errors;

    if (total_phy_errors == 0) {
            return 0;
    }

    /* Clear PHY error stats*/
    dfs_clear_stats(sc);

    tstamp_diff /= 1000000;

    if (tstamp_diff) {
          phy_err_rate = (total_phy_errors / tstamp_diff);
    } else {
                /* not enough time difference*/
                phy_err_rate = 1;
    }
    return phy_err_rate;
}

int get_dfs_hang_war_timeout(struct ath_softc *sc)
{
    u_int64_t cur_tstamp = OS_GET_TIMESTAMP();     
    u_int64_t diff_tstamp = 0;     
    int timeout = 0;

    timeout = sc->sc_dfs_hang.hang_war_timeout;
    diff_tstamp = cur_tstamp - sc->sc_dfs_hang.last_dfs_hang_war_tstamp;

   /* Check if we should increase the WAR timeout, can only increase upto MAX value*/
    if (diff_tstamp <= ATH_DFS_HANG_FREQ_US) {
       if ((timeout*2) < ATH_DFS_HANG_WAR_MAX_TIMEOUT_MS){
           /* double hang war timeout*/
           timeout = sc->sc_dfs_hang.hang_war_timeout * 2;
        } else {
           /* hang war timeout at MAX value*/
          timeout = ATH_DFS_HANG_WAR_MAX_TIMEOUT_MS;
        }
    } else {
    /* Check if we should decrease the WAR timeout, can only decrease upto MIN value*/
       if (diff_tstamp >= (ATH_DFS_HANG_FREQ_US + 
                           (sc->sc_dfs_hang.hang_war_timeout * 1000))) {
             /* decrease hang war timeout */
            timeout = sc->sc_dfs_hang.hang_war_timeout / 2;
            if (timeout < ATH_DFS_HANG_WAR_MIN_TIMEOUT_MS) {
                     timeout = ATH_DFS_HANG_WAR_MIN_TIMEOUT_MS;
            }
        } else { 
           /* Keep the WAR timeout the same */
           timeout = sc->sc_dfs_hang.hang_war_timeout;
        }
    }
    /* Store the current timestamp as the last time the WAR kicked in*/
    sc->sc_dfs_hang.last_dfs_hang_war_tstamp = cur_tstamp;
    return timeout;
}

#endif



