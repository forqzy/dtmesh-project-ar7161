/*-
 * Copyright (c) 2004 Video54 Technologies, Inc.
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
 * $Id: //depot/sw/releases/7.3_AP/wlan/common/lmac/ratectrl/if_athrate.c#3 $
 */
/*
 * Atheros rate control algorithm
 */

#include <osdep.h>

#include "opt_ah.h"
#include "ah.h"
#include "ratectrl.h"
#include "ratectrl11n.h"
#include "ah_desc.h"
#include "ath_internal.h"

#include <_ieee80211.h>

#ifndef REMOVE_PKT_LOG
#include "pktlog_rc.h"
struct ath_pktlog_rcfuncs *g_pktlog_rcfuncs;
#endif

static int ath_rate_newassoc_11n(struct ath_softc *sc, struct ath_node *an, int isnew,
                                 unsigned int capflag, 
                                 struct ieee80211_rateset *negotiated_rates,
                                 struct ieee80211_rateset *negotiated_htrates);
static void ath_rate_tx_complete_11n(struct ath_softc *sc, struct ath_node *an,
                                     const struct ath_desc *ds,
                                     struct ath_rc_series rcs[], u_int8_t ac, int nframes,
                                     int nbad);
static void ath_rate_findrate_11n(struct ath_softc *sc,
                                  struct ath_node *an,
                                  size_t frameLen,
                                  int numTries,
                                  int numRates,
                                  unsigned int rcflag,
                                  u_int8_t ac,
                                  struct ath_rc_series series[],
                                  int *isProbe,
                                  int isretry);

/*
 * Attach to a device instance.  Setup the public definition
 * of how much per-node space we need and setup the private
 * phy tables that have rate control parameters.  These tables
 * are normally part of the Atheros hal but are not included
 * in our hal as the rate control data was not being used and
 * was considered proprietary (at the time).
 */
struct atheros_softc *
ath_rate_attach(struct ath_hal *ah)
{
    struct atheros_softc *asc;

    RC_OS_MALLOC(&asc, sizeof(struct atheros_softc), RATECTRL_MEMTAG);
    if (asc == NULL)
        return NULL;

    OS_MEMZERO(asc, sizeof(struct atheros_softc));

    /*
     * Use the magic number to figure out the chip type.
     * There's probably a better way to do this but for
     * now this suffices.
     *
     * NB: We don't have a separate set of tables for the
     *     5210; treat it like a 5211 since it has the same
     *     tx descriptor format and (hopefully) sufficiently
     *     similar operating characteristics to work ok.
     */
    switch (ah->ah_magic) {
#ifdef AH_SUPPORT_AR5212
    case 0x19570405:
    case 0x19541014:	/* 5212 */
        ar5212AttachRateTables(asc);
        break;
#endif
    case 0x19641014:    /* 5416 */
        ar5416AttachRateTables(asc);
        break;
    default:
        ASSERT(0);
        break;
    }

    /* Save Maximum TX Trigger Level (used for 11n) */
    ath_hal_getcapability(ah, HAL_CAP_TX_TRIG_LEVEL_MAX, 0, &asc->txTrigLevelMax);

    /*  return alias for atheros_softc * */
    return asc;
}

struct atheros_vap *
ath_rate_create_vap(struct atheros_softc *asc, struct ath_vap *athdev_vap)
{
    struct atheros_vap *avp;

    RC_OS_MALLOC(&avp, sizeof(struct atheros_vap), RATECTRL_MEMTAG);
    if (avp == NULL)
        return NULL;

    OS_MEMZERO(avp, sizeof(struct atheros_vap));
    avp->asc = asc;
    avp->athdev_vap = athdev_vap;
    return avp;
}

struct atheros_node *
ath_rate_node_alloc(struct atheros_vap *avp)
{
    struct atheros_node *anode;

    RC_OS_MALLOC(&anode, sizeof(struct atheros_node), RATECTRL_MEMTAG);
    if (anode == NULL)
        return NULL;

    OS_MEMZERO(anode, sizeof(struct atheros_node));
    anode->avp = avp;
    anode->asc = avp->asc;
#ifdef ATH_SUPPORT_IQUE
    anode->rcFunc[WME_AC_VI].rcUpdate = &rcUpdate_11nViVo;
    anode->rcFunc[WME_AC_VO].rcUpdate = &rcUpdate_11nViVo;
    anode->rcFunc[WME_AC_BE].rcUpdate = &rcUpdate_11n;
    anode->rcFunc[WME_AC_BK].rcUpdate = &rcUpdate_11n;

    anode->rcFunc[WME_AC_VI].rcFind = &rcRateFind_11nViVo;
    anode->rcFunc[WME_AC_VO].rcFind = &rcRateFind_11nViVo;
    anode->rcFunc[WME_AC_BE].rcFind = &rcRateFind_11n;
    anode->rcFunc[WME_AC_BK].rcFind = &rcRateFind_11n;
#endif
    return anode;
}

void
ath_rate_node_free(struct atheros_node *anode)
{
    if (anode != NULL)
        RC_OS_FREE(anode, sizeof(*anode));
}

void
ath_rate_free_vap(struct atheros_vap *avp)
{
    if (avp != NULL)
        RC_OS_FREE(avp, sizeof(*avp));
}

void
ath_rate_detach(struct atheros_softc *asc)
{
    if (asc != NULL)
        RC_OS_FREE(asc, sizeof(*asc));
}

/*
 * Initialize per-node rate control state.
 */
void
ath_rate_node_init(struct atheros_node *pSib)
{
    rcSibInit(pSib);
}

/*
 * Cleanup per-node rate control state.
 */
void
ath_rate_node_cleanup(struct atheros_node *an)
{
    /* NB: nothing to do */
    UNREFERENCED_PARAMETER(an);
}

/*
 * Return the next series 0 transmit rate and setup for a callback
 * to install the multi-rate transmit data if appropriate.  We cannot
 * install the multi-rate transmit data here because the caller is
 * going to initialize the tx descriptor and so would clobber whatever
 * we write. Note that we choose an arbitrary series 0 try count to
 * insure we get called back; this permits us to defer calculating
 * the actual number of tries until the callback at which time we
 * can just copy the pre-calculated series data.
 */
void
ath_rate_findrate(struct ath_softc *sc, struct ath_node *an,
                  int shortPreamble, u_int32_t frameLen, int numTries,
                  unsigned int rcflag, u_int8_t ac, struct ath_rc_series rcs[],
                  int *isProbe, int isretry)
{
    struct atheros_node *oan = an->an_rc_node;
    struct atheros_softc *asc = oan->asc;
    struct atheros_vap *avap = oan->avp;
    const RATE_TABLE *pRateTable = avap->rateTable;
    u_int32_t *retrySched;

    ASSERT(rcs != NULL);

    if (sc->sc_ah->ah_magic == 0x19641014) {
            ath_rate_findrate_11n(sc, an, frameLen, numTries, 4,
                                  rcflag, ac, rcs, isProbe, isretry);
            return;
    }

    if (asc->fixedrix == IEEE80211_FIXED_RATE_NONE) {
        rcs[0].rix = rcRateFind(sc, oan, frameLen, pRateTable, &sc->sc_curchan, isretry);
        if (sc->sc_mrretry)
            rcs[0].tries = ATH_TXMAXTRY-1;	/* anything != ATH_TXMAXTRY */
        else
            rcs[0].tries = ATH_TXMAXTRY;
    } else {
        rcs[0].rix = asc->fixedrix;
        rcs[0].tries = ATH_TXMAXTRY;
    }

    ASSERT(rcs[0].rix != (u_int8_t)-1);

    if (rcs[0].tries != ATH_TXMAXTRY) {

        ASSERT(sc->sc_mrretry != 0);

        /* NB: only called for data frames */
        if (oan->txRateCtrl.probeRate) {
            retrySched = shortPreamble ?
                (u_int32_t *)&pRateTable->info[rcs[0].rix].probeShortSched :
                (u_int32_t *)&pRateTable->info[rcs[0].rix].probeSched;
        } else {
            retrySched = shortPreamble ?
                (u_int32_t *)&pRateTable->info[rcs[0].rix].shortSched :
                (u_int32_t *)&pRateTable->info[rcs[0].rix].normalSched;
        }

        /* Update rate seies based on retry schedule */
        rcs[0].tries = (retrySched[0] & 0x000f0000) >> 16;
        rcs[1].tries = (retrySched[0] & 0x00f00000) >> 20;
        rcs[2].tries = (retrySched[0] & 0x0f000000) >> 24;
        rcs[3].tries = (retrySched[0] & 0xf0000000) >> 28;
        
        if (!IS_CHAN_TURBO(&sc->sc_curchan))
            ASSERT(rcs[0].rix == pRateTable->rateCodeToIndex[(retrySched[1] & 0x0000001f)]);
        rcs[1].rix = pRateTable->rateCodeToIndex[(retrySched[1] & 0x000003e0) >> 5];
        rcs[2].rix = pRateTable->rateCodeToIndex[(retrySched[1] & 0x00007c00) >> 10];
        rcs[3].rix = pRateTable->rateCodeToIndex[(retrySched[1] & 0x000f8000) >> 15];

        if (sc->sc_curchan.privFlags & CHANNEL_4MS_LIMIT) {
            u_int32_t i, prevrix = rcs[0].rix;

            for (i=1; i<4; i++) {
                if (rcs[i].tries) {
                    if (pRateTable->info[rcs[i].rix].max4msFrameLen < frameLen) {
                        rcs[i].rix = prevrix;
                    } else {
                        prevrix = rcs[i].rix;
                    }
                } else {
                    /* Retries are 0 from here */
                    break;
                }
            }
        }

    }

    if (sc->sc_ieee_ops->update_txrate) {
        sc->sc_ieee_ops->update_txrate(an->an_node, oan->rixMap[rcs[0].rix]);
    }
    
#if ATH_SUPERG_DYNTURBO
    /* XXX map from merged table to split for driver */
    if (IS_CHAN_TURBO(&sc->sc_curchan) && rcs[0].rix >=
        (pRateTable->rateCount - pRateTable->numTurboRates))
    {
        u_int32_t numCCKRates = 5;
        u_int32_t i;

         rcs[0].rix -= (pRateTable->rateCount-pRateTable->numTurboRates);
         if (IS_CHAN_2GHZ(&sc->sc_curchan)) {
            for (i=1;i<=3;i++) { /*Mapping for retry rates from merged table to split table*/
                 if (rcs[i].rix >= numCCKRates) {
                     rcs[i].rix -= numCCKRates;
                 } else { /* For 6Mbps Turbo rate */
                     rcs[i].rix -= numCCKRates-1;
                 }                                
            }
        }

    }
#endif
}

u_int8_t
ath_rate_findrateix(struct ath_softc *sc, struct ath_node *an, u_int8_t dot11Rate)
{
    struct atheros_node *oan = an->an_rc_node;
    struct atheros_vap *avap = oan->avp;
    const RATE_TABLE *pRateTable;
    const RATE_TABLE_11N *pRateTable_11n;
    int i;

    if (sc->sc_ah->ah_magic == 0x19641014) {
            pRateTable_11n = (const RATE_TABLE_11N *)avap->rateTable;
            for (i=0 ; i < pRateTable_11n->rateCount; i++) {
                if ((pRateTable_11n->info[i].dot11Rate & 0x7f) ==
                                    (dot11Rate & 0x7f)) {
                    return i;
                }
            }
            return 0;
    }

    pRateTable = (const RATE_TABLE *)avap->rateTable;

    for (i=0 ; i < pRateTable->rateCount; i++) {
        if ((pRateTable->info[i].dot11Rate & 0x7f) == (dot11Rate & 0x7f)) {
            return i;
        }
    }

    return 0;
}

#ifdef AH_SUPPORT_AR5212
#include <ar5212/ar5212desc.h>
#endif

#define	MS(_v, _f)	(((_v) & _f) >> _f##_S)

/*
 * Process a tx descriptor for a completed transmit (success or failure).
 */
void
ath_rate_tx_complete(struct ath_softc *sc, struct ath_node *an,
                     const struct ath_desc *ds, struct ath_rc_series rcs[],
                     u_int8_t ac, int nframes, int nbad)
{
    struct atheros_node    *atn = an->an_rc_node;
    struct atheros_vap     *avap = atn->avp;
    const RATE_TABLE       *pRateTable = avap->rateTable;
    u_int8_t               txRateCode = ds->ds_txstat.ts_ratecode;
    u_int8_t               totalTries = 0;

    if (sc->sc_ah->ah_magic == 0x19641014) {
        ath_rate_tx_complete_11n(sc, an, ds, rcs, ac, nframes, nbad);
        return;
    }

    if (pRateTable->rateCodeToIndex[txRateCode] == (u_int8_t) -1) {
        /*
         * This can happen, for example, when switching bands
         * and pending tx's are processed before the queue
         * is flushed (should fix mode switch to ensure this
         * does not happen).
         */
        // DPRINTF(sc, "%s: no mapping for rate code 0x%x",
        //         __func__, txRate);
        return;
    }
#ifdef AH_SUPPORT_AR5212
    if (ds->ds_txstat.ts_rateindex != 0) {
        const struct ar5212_desc *ads = AR5212DESC(ds);
        int finalTSIdx = MS(ads->ds_txstatus1, AR_FinalTSIndex);
        int series;
        
        /*
         * Process intermediate rates that failed.
         */
        for (series = 0; series < finalTSIdx; series++) {
            int rate, tries;

            /* NB: we know series <= 2 */
            switch (series) {
            case 0:
                rate = MS(ads->ds_ctl3, AR_XmitRate0);
                tries = MS(ads->ds_ctl2, AR_XmitDataTries0);
                break;
            case 1:
                rate = MS(ads->ds_ctl3, AR_XmitRate1);
                tries = MS(ads->ds_ctl2, AR_XmitDataTries1);
                break;
            default:
                rate = MS(ads->ds_ctl3, AR_XmitRate2);
                tries = MS(ads->ds_ctl2, AR_XmitDataTries2);
                break;
            }

            if (pRateTable->rateCodeToIndex[rate] != (u_int8_t) -1) {
                /*
                 * This can happen, for example, when switching bands
                 * and pending tx's are processed before the queue
                 * is flushed (should fix mode switch to ensure this
                 * does not happen).
                 */
                // DPRINTF(sc, "%s: no mapping for rate code 0x%x",
                //         __func__, txRate);
                rcUpdate(atn
                         , 2 // Huh? Indicates an intermediate rate failure. Should use a macro instead.
                         , pRateTable->rateCodeToIndex[rate]
                         , tries
                         , ds->ds_txstat.ts_rssi
                         , ds->ds_txstat.ts_antenna
                         , pRateTable
                         , sc->sc_opmode 
                         , sc->sc_diversity
                         , sc->sc_setdefantenna
                         , (void *)sc
                    );
            }

            /* Account for retries on intermediate rates */
            totalTries += tries;
        }
    }
#endif

    /* 
     * Exclude intermediate rate retries, or the last rate, which may have 
     * succeeded, will incur a penalty higher than the intermediate rates that
     * failed.
     */
    rcUpdate(atn
             , (ds->ds_txstat.ts_status & HAL_TXERR_XRETRY) != 0
             , pRateTable->rateCodeToIndex[txRateCode]
             , ds->ds_txstat.ts_longretry - totalTries
             , ds->ds_txstat.ts_rssi
             , ds->ds_txstat.ts_antenna
             , pRateTable
             , sc->sc_opmode
             , sc->sc_diversity
             , sc->sc_setdefantenna
             , (void *)sc
	);
}

/*
 * Update rate-control state on station associate/reassociate.
 */
int
ath_rate_newassoc(struct ath_softc *sc, struct ath_node *an,
                  int isnew, unsigned int capflag,
                  struct ieee80211_rateset *negotiated_rates,
                  struct ieee80211_rateset *negotiated_htrates)
{
    struct ieee80211_rateset *pRates = negotiated_rates;
    struct atheros_node *oan = an->an_rc_node;
    struct atheros_softc *asc = oan->asc;
    int txrate = 0;

    if (sc->sc_ah->ah_magic == 0x19641014) {
        return (ath_rate_newassoc_11n(sc, an, isnew, capflag, negotiated_rates, negotiated_htrates));
    }

    if (isnew) {
        rcSibUpdate(asc, oan, 0, pRates, sc->sc_curmode);
        /*
         * Set an initial tx rate for the net80211 layer.
         * Even though noone uses it, it wants to validate
         * the setting before entering RUN state so if there
         * was a pervious setting from a different node it
         * may be invalid.
         */
        if (asc->fixedrix != IEEE80211_FIXED_RATE_NONE) {
            if (sc->sc_rixmap[asc->fixedrix] != 0xff)
                txrate = sc->sc_rixmap[asc->fixedrix];
        }
        if (sc->sc_ieee_ops->update_txrate) {
            sc->sc_ieee_ops->update_txrate(an->an_node, txrate);
    }
    }

    return 0;
}

void
ath_rate_node_update(struct ath_node *an)
{
    /* notify net80211 layer */
    an->an_sc->sc_ieee_ops->rate_node_update(an->an_sc->sc_ieee, an->an_node, 1);
}

/*
 * Update rate-control state on a device state change.  When
 * operating as a station this includes associate/reassociate
 * with an AP.  Otherwise this gets called, for example, when
 * the we transition to run state when operating as an AP.
 */
void
ath_rate_newstate(struct ath_softc *sc, struct ath_vap *avp, int up)
{
    struct atheros_softc *asc = sc->sc_rc;
    struct atheros_vap *avap = avp->av_atvp;

    switch (sc->sc_ah->ah_magic) {
#ifdef AH_SUPPORT_AR5212
    case 0x19541014:	/* 5212 */
        /* For half and quarter rate channles use different
         * rate tables
         */
        if (sc->sc_curchan.channelFlags & CHANNEL_HALF) {
            ar5212SetHalfRateTable(asc);
        } else if (sc->sc_curchan.channelFlags & CHANNEL_QUARTER) {
            ar5212SetQuarterRateTable(asc);
        } else { /* full rate */
            ar5212SetFullRateTable(asc);
        }
        break;
#endif
    case 0x19641014:
        /* For half and quarter rate channles use different
         * rate tables
         */
#ifndef ATH_NO_5G_SUPPORT
        if (sc->sc_curchan.channelFlags & CHANNEL_HALF) {
            ar5416SetHalfRateTable(asc);
        } else if (sc->sc_curchan.channelFlags & CHANNEL_QUARTER) {
            ar5416SetQuarterRateTable(asc);
        } else { /* full rate */
            ar5416SetFullRateTable(asc);
        }
#endif  /* #ifndef ATH_NO_5G_SUPPORT */
        break;
    default:		/* XXX 5210 */
        break;
    }

    /*
     * Calculate index of any fixed rate configured.  It is safe
     * to do this only here as changing/setting the fixed rate
     * causes the 802.11 state machine to transition (which causes
     * us to be notified).
     */
#ifdef ATH_SUPERG_XR 
    if(vap->iv_flags & IEEE80211_F_XR) {
        avap->rateTable = asc->hwRateTable[WIRELESS_MODE_XR];
    } else {
        avap->rateTable = asc->hwRateTable[sc->sc_curmode];
    }
#else
    avap->rateTable = asc->hwRateTable[sc->sc_curmode];
#endif

    if (avp->av_config.av_fixed_rateset != IEEE80211_FIXED_RATE_NONE) {
        asc->fixedrix = sc->sc_rixmap[avp->av_config.av_fixed_rateset & 0xff];
        /* NB: check the fixed rate exists */
        if (asc->fixedrix == 0xff)
            asc->fixedrix = IEEE80211_FIXED_RATE_NONE;
    } else
        asc->fixedrix = IEEE80211_FIXED_RATE_NONE;

    if (up) {
        /* Notify net80211 layer */
        sc->sc_ieee_ops->rate_newstate(sc->sc_ieee, avp->av_if_data);
	}
}

void
atheros_setuptable(RATE_TABLE *rt)
{
    int i;

    for (i = 0; i < RATE_TABLE_SIZE; i++)
        rt->rateCodeToIndex[i] = (u_int8_t) -1;
    for (i = 0; i < rt->rateCount; i++) {
        u_int8_t code = rt->info[i].rateCode;

        /* Do not re-initialize rateCodeToIndex when using combined
         * base + turbo rate tables. i.e the rateCodeToIndex should
         * always point to base rate index. The ratecontrol module
         * adjusts the index based on turbo mode.
         */
        if(rt->rateCodeToIndex[code] == (u_int8_t) -1) {
            rt->rateCodeToIndex[code] = i;
            rt->rateCodeToIndex[ code | rt->info[i].shortPreamble] = i;
        }
    }
}

int
ath_rate_table_init(void)
{
#ifdef AH_SUPPORT_AR5212
    ar5212SetupRateTables();
    /* ar5416SetupRateTables(); */
#endif
    return 0;
}

/* 11N speicific routines */

/*
 * Return the next series 0 transmit rate and setup for a callback
 * to install the multi-rate transmit data if appropriate.  We cannot
 * install the multi-rate transmit data here because the caller is
 * going to initialize the tx descriptor and so would clobber whatever
 * we write. Note that we choose an arbitrary series 0 try count to
 * insure we get called back; this permits us to defer calculating
 * the actual number of tries until the callback at which time we
 * can just copy the pre-calculated series data.
 */

static void
ath_rate_findrate_11n(struct ath_softc *sc,
                      struct ath_node *an,
                      size_t frameLen,
                      int numTries,
                      int numRates,
                      unsigned int rcflag,
                      u_int8_t ac,
                      struct ath_rc_series series[],
                      int *isProbe,
                      int isretry
                      )
{
    struct atheros_node    *oan = ATH_NODE_ATHEROS(an);
    struct ath_vap         *avp = oan->avp->athdev_vap;

    if (!numRates || !numTries) {
        return;
    }

    if (avp->av_config.av_fixed_rateset == IEEE80211_FIXED_RATE_NONE) {
#ifdef ATH_SUPPORT_IQUE
        oan->rcFunc[ac].rcFind(sc, an, ac, numTries, numRates, rcflag, series, isProbe, isretry);
#else
        rcRateFind_11n(sc, an, ac, numTries, numRates, rcflag, series, isProbe, isretry);
#endif
    } else {
        /* Fixed rate */
        int idx;
        A_UINT8 flags;
        A_UINT32 rix;
        struct atheros_softc *asc = oan->asc;
        RATE_TABLE_11N *pRateTable = (RATE_TABLE_11N *)asc->hwRateTable[sc->sc_curmode];

        for (idx = 0; idx < 4; idx++) {
            unsigned int    mcs;

            series[idx].tries =
                IEEE80211_RATE_IDX_ENTRY(avp->av_config.av_fixed_retryset, idx);

            mcs = IEEE80211_RATE_IDX_ENTRY(avp->av_config.av_fixed_rateset, idx);

            if (idx == 3 && (mcs & 0xf0) == 0x70)
                mcs = (mcs & ~0xf0)|0x80;

            if (!(mcs & 0x80)) {
                flags = 0;
            } else {
                flags = ((oan->htcap & WLAN_RC_DS_FLAG) ? ATH_RC_DS_FLAG : 0) |
                        ((oan->htcap & WLAN_RC_40_FLAG) ? ATH_RC_CW40_FLAG : 0) |
                        ((oan->htcap & WLAN_RC_SGI_FLAG)? ((oan->htcap & WLAN_RC_40_FLAG)? ATH_RC_SGI_FLAG : 0) : 0);

                if (oan->stbc) {
                    /* For now, only single stream STBC is supported */
                    if (mcs >= 0x80 && mcs <= 0x87)
                        flags |= ATH_RC_TX_STBC_FLAG;
                }
            }

            series[idx].rix = sc->sc_rixmap[mcs];

            if ((flags & ATH_RC_CW40_FLAG) && (flags & ATH_RC_SGI_FLAG)) {
                rix = pRateTable->info[series[idx].rix].htIndex;
            } else if (flags & ATH_RC_SGI_FLAG) {
                rix = pRateTable->info[series[idx].rix].sgiIndex;
            } else if (flags & ATH_RC_CW40_FLAG) {
                rix = pRateTable->info[series[idx].rix].cw40Index;
            } else {
                rix = pRateTable->info[series[idx].rix].baseIndex;
            }
            series[idx].max4msframelen = pRateTable->info[rix].max4msframelen;
            series[idx].flags = flags;
        }
    }
}

/*
 * Process a tx descriptor for a completed transmit (success or failure).
 */
static void
ath_rate_tx_complete_11n(struct ath_softc *sc,
                         struct ath_node *an,
                         const struct ath_desc *ds,
                         struct ath_rc_series rcs[],
                         u_int8_t ac,
                         int nframes,
                         int nbad)
{
    int    finalTSIdx = ds->ds_txstat.ts_rateindex;
    int    tx_status = 0, is_underrun = 0;
    struct atheros_node    *oan = ATH_NODE_ATHEROS(an);
    struct ath_vap         *avp = oan->avp->athdev_vap;

    if ((avp->av_config.av_fixed_rateset != IEEE80211_FIXED_RATE_NONE)
            || ds->ds_txstat.ts_status & HAL_TXERR_FILT) {
        return;
    }

#ifdef ATH_CHAINMASK_SELECT
    if (ds->ds_txstat.ts_rssi > 0) {
        ATH_RSSI_LPF(an->an_chainmask_sel.tx_avgrssi,
                             ds->ds_txstat.ts_rssi);
    }
#endif

    /*
     * If underrun error is seen assume it as an excessive retry only if prefetch
     * trigger level have reached the max (0x3f for 5416)
     * Adjust the long retry as if the frame was tried ATH_11N_TXMAXTRY times. This
     * affects how ratectrl updates PER for the failed rate.
     */
    if ((HAL_IS_TX_UNDERRUN(ds)) &&
        (ath_hal_gettxtriglevel(sc->sc_ah) >= sc->sc_rc->txTrigLevelMax)) {
        tx_status = 1;
        is_underrun = 1;
    }

    if ((ds->ds_txstat.ts_status & HAL_TXERR_XRETRY) ||
        (ds->ds_txstat.ts_status & HAL_TXERR_FIFO)) {
        tx_status = 1;
    }

#ifdef ATH_SUPPORT_IQUE
    oan->rcFunc[ac].rcUpdate(sc, an, ds->ds_txstat.ts_rssi, ac,
                        finalTSIdx, tx_status, rcs, nframes , nbad,
                        (is_underrun) ? ATH_11N_TXMAXTRY:ds->ds_txstat.ts_longretry);
#else
    rcUpdate_11n(sc, an, ds->ds_txstat.ts_rssi , ac,
                    finalTSIdx, tx_status, rcs, nframes , nbad,
                    (is_underrun) ? ATH_11N_TXMAXTRY:ds->ds_txstat.ts_longretry);
#endif
}

/*
 * Update rate-control state on station associate/reassociate.
 */
static int
ath_rate_newassoc_11n(struct ath_softc *sc, struct ath_node *an, int isnew,
                      unsigned int capflag,
                      struct ieee80211_rateset *negotiated_rates,
                      struct ieee80211_rateset *negotiated_htrates)
{
    struct atheros_node    *oan = ATH_NODE_ATHEROS(an);

	if (isnew) {

	    /* FIX ME:XXXX Looks like this not used at all.	*/
	    oan->htcap =
                    ((capflag & ATH_RC_DS_FLAG) ? WLAN_RC_DS_FLAG : 0) |
                    ((capflag & ATH_RC_SGI_FLAG) ? WLAN_RC_SGI_FLAG : 0) |
                    ((capflag & ATH_RC_HT_FLAG)  ? WLAN_RC_HT_FLAG : 0) |
                    ((capflag & ATH_RC_CW40_FLAG) ? WLAN_RC_40_FLAG : 0) |
                    ((capflag & ATH_RC_WEP_TKIP_FLAG) ? WLAN_RC_WEP_TKIP_FLAG : 0);

        /* Rx STBC is a 2-bit mask. Needs to convert from ath definition to wlan definition. */ 
        oan->htcap |= (((capflag & ATH_RC_RX_STBC_FLAG) >> ATH_RC_RX_STBC_FLAG_S)
                       << WLAN_RC_STBC_FLAG_S);

        /* Enable stbc only for more than one tx chain */
        if (sc->sc_txstbcsupport && (sc->sc_tx_chainmask != 1)) {
            oan->stbc = (capflag & ATH_RC_RX_STBC_FLAG) >> ATH_RC_RX_STBC_FLAG_S;
        } else {
            oan->stbc = 0;
        }

	    rcSibUpdate_11n(sc, an, oan->htcap, 0, negotiated_rates, negotiated_htrates);

#ifdef ATH_SUPPORT_IQUE
	    rcSibUpdate_11nViVo(sc, an, oan->htcap, 0, negotiated_rates, negotiated_htrates);
#endif

	    /*
	     * Set an initial tx rate for the net80211 layer.
	     * Even though noone uses it, it wants to validate
	     * the setting before entering RUN state so if there
	     * was a pervious setting from a different node it
	     * may be invalid.
	     */

        if (sc->sc_ieee_ops->update_txrate) {
            sc->sc_ieee_ops->update_txrate(an->an_node, 0);
        }
    }

    return 0;
}

#ifdef __linux__
#ifndef ATH_WLAN_COMBINE
/*
 * Linux module glue
 */
static char *dev_info = "ath_rate_atheros";

MODULE_AUTHOR("Atheros Communications, Inc.");
MODULE_DESCRIPTION("Rate control support for Atheros devices");
#ifdef MODULE_LICENSE
MODULE_LICENSE("Proprietary");
#endif

static int __init
init_ath_rate_atheros(void)
{
    /* XXX version is a guess; where should it come from? */
    printk(KERN_INFO "%s: "
           "Copyright (c) 2001-2005 Atheros Communications, Inc, "
           "All Rights Reserved\n", dev_info);
    return 0;
}
module_init(init_ath_rate_atheros);

static void __exit
exit_ath_rate_atheros(void)
{
	printk(KERN_INFO "%s: driver unloaded\n", dev_info);
}
module_exit(exit_ath_rate_atheros);

#ifndef EXPORT_SYMTAB
#define	EXPORT_SYMTAB
#endif

EXPORT_SYMBOL(ath_rate_table_init);
EXPORT_SYMBOL(ath_rate_attach);
EXPORT_SYMBOL(ath_rate_detach);
EXPORT_SYMBOL(ath_rate_create_vap);
EXPORT_SYMBOL(ath_rate_free_vap);
EXPORT_SYMBOL(ath_rate_node_alloc);
EXPORT_SYMBOL(ath_rate_node_free);
EXPORT_SYMBOL(ath_rate_node_init);
EXPORT_SYMBOL(ath_rate_node_cleanup);
EXPORT_SYMBOL(ath_rate_newassoc);
EXPORT_SYMBOL(ath_rate_newstate);
EXPORT_SYMBOL(ath_rate_findrate);
EXPORT_SYMBOL(ath_rate_node_update);
EXPORT_SYMBOL(ath_rate_findrateix);
EXPORT_SYMBOL(ath_rate_tx_complete);
#endif /* #ifndef ATH_WLAN_COMBINE */

EXPORT_SYMBOL(g_pktlog_rcfuncs);

#endif

#ifdef ATH_CCX
u_int8_t
ath_rcRateValueToPer(ath_dev_t dev, struct ath_node *an, int txRateKbps)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    switch (sc->sc_ah->ah_magic) {
    case 0x19570405:
    case 0x19541014:	/* 5212 */
        return(rcRateValueToPer(sc, an, txRateKbps));
    case 0x19641014:    /* 5416 */
        return(rcRateValueToPer_11n(sc, an, txRateKbps));
    default:
        ASSERT(0);
        return(100);
    }

}
#endif
