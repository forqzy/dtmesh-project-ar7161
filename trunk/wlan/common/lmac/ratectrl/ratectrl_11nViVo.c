/*
 * $Id: //depot/sw/releases/7.3_AP/wlan/common/lmac/ratectrl/ratectrl_11nViVo.c#3 $
 *
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
 * DESCRIPTION
 *
 * This file contains the data structures and routines for transmit rate
 * control operating on the linearized rates of different phys
 *
 *
 */


#ifdef __FreeBSD__
#include <dev/ath/ath_rate/atheros/ratectrl.h>
#else
#include <osdep.h>

#include "ah.h"
#include "ratectrl.h"
#include "ratectrl11n.h"
#endif

#ifndef REMOVE_PKT_LOG
#include "pktlog_rc.h"
extern struct ath_pktlog_rcfuncs *g_pktlog_rcfuncs;
#endif

#ifdef ATH_SUPPORT_IQUE

#define MULTI_RATE_RETRY_ENABLE 1

#define MIN_PROBE_INTERVAL 2
/* Dump all the valid rate index */

#if 0
static void
dump_valid_rate_index(RATE_TABLE_11N *pRateTable, A_UINT8 *validRateIndex)
{
    A_UINT32 i=0;

    printk("Valid Rate Table:-\n");
    for(i=0; i<MAX_TX_RATE_TBL; i++)
    printk(" Index:%d, value:%d, code:%x, rate:%d, flag:%x\n", i, (int)validRateIndex[i],
           pRateTable->info[(int)validRateIndex[i]].rateCode,
           pRateTable->info[(int)validRateIndex[i]].rateKbps,
           WLAN_RC_PHY_40(pRateTable->info[(int)validRateIndex[i]].phy));
}
#endif

static void
rcSortValidRates(const RATE_TABLE_11N *pRateTable, TX_RATE_CTRL *pRc)
{
    A_UINT8 i,j;

    if (!pRc->maxValidRate) {
        return;
    }

    for (i=pRc->maxValidRate-1; i > 0; i--) {
        for (j=0; j <= i-1; j++) {
            if (pRateTable->info[pRc->validRateIndex[j]].rateKbps >
                pRateTable->info[pRc->validRateIndex[j+1]].rateKbps)
            {
                A_UINT8 tmp;
                tmp = pRc->validRateIndex[j];
                pRc->validRateIndex[j] = pRc->validRateIndex[j+1];
                pRc->validRateIndex[j+1] = tmp;
            }
        }
    }

    return;
}


/* Access functions for validTxRateMask */

static void
rcInitValidTxMask(TX_RATE_CTRL *pRc)
{
    A_UINT8     i;

    for (i = 0; i < pRc->rateTableSize; i++) {
        pRc->validRateIndex[i] = FALSE;
    }
}

static INLINE  void
rcSetValidTxMask(TX_RATE_CTRL *pRc, A_UINT8 index, int validTxRate)
{
    ASSERT(index < pRc->rateTableSize);
    pRc->validRateIndex[index] = validTxRate ? TRUE : FALSE;

}

static INLINE int
rcIsValidTxMask(TX_RATE_CTRL *pRc, A_UINT8 index)
{
    ASSERT(index < pRc->rateTableSize);
    return (pRc->validRateIndex[index]);
}

/* Iterators for validTxRateMask */
static INLINE int
rcGetNextValidTxRate(const RATE_TABLE_11N *pRateTable, TX_RATE_CTRL *pRc,
                     A_UINT8 curValidTxRate, A_UINT8 *pNextIndex)
{
    A_UINT8     i;

    for (i = 0; i < pRc->maxValidRate-1; i++) {
        if (pRc->validRateIndex[i] == curValidTxRate) {
            *pNextIndex = pRc->validRateIndex[i+1];
            return TRUE;
        }
    }

    /* No more valid rates */
    *pNextIndex = 0;
    return FALSE;
}


/* Return true only for single stream */

static int
rcIsValidPhyRate(A_UINT32 phy, A_UINT32 capflag, int ignoreCW)
{
    if (WLAN_RC_PHY_HT(phy) & !(capflag & WLAN_RC_HT_FLAG)) {
        return FALSE;
    }
    if (WLAN_RC_PHY_DS(phy) && !(capflag & WLAN_RC_DS_FLAG))  {
        return FALSE;
    }
    if (WLAN_RC_PHY_SGI(phy) && !(capflag & WLAN_RC_SGI_FLAG)) {
        return FALSE;
    }
    if (!ignoreCW && WLAN_RC_PHY_HT(phy)) {
        if (WLAN_RC_PHY_40(phy) && !(capflag & WLAN_RC_40_FLAG)) {
            return FALSE;
        }
        if (!WLAN_RC_PHY_40(phy) && (capflag & WLAN_RC_40_FLAG)) {
            return FALSE;
        }
    }
    return TRUE;
}

static INLINE int
rcGetNextLowerValidTxRate(const RATE_TABLE_11N *pRateTable, TX_RATE_CTRL *pRc,
                          A_UINT8 curValidTxRate, A_UINT8 *pNextIndex)
{
    A_INT8     i;

    for (i = 1; i < pRc->maxValidRate ; i++) {
        if (pRc->validRateIndex[i] == curValidTxRate) {
            *pNextIndex = pRc->validRateIndex[i-1];
            return TRUE;
        }
    }

    return FALSE;
}

/*
 * Initialize the Valid Rate Index from valid entries in Rate Table
 */
static A_UINT8
rcSibInitValidRates(struct atheros_node *pSib, const RATE_TABLE_11N *pRateTable, A_UINT32 capflag)
{
    TX_RATE_CTRL    *pRc  = (TX_RATE_CTRL *)(&pSib->txRateCtrlViVo);
    A_UINT8         i, hi = 0;
    A_UINT32        valid;

    for (i = 0; i < pRateTable->rateCount; i++) {
        valid = (pSib->singleStream ? pRateTable->info[i].validSingleStream :
                                      pRateTable->info[i].valid);
        if (valid == TRUE)
        {
            A_UINT32 phy = pRateTable->info[i].phy;

            if (!rcIsValidPhyRate(phy, capflag, FALSE))
                continue;
            pRc->validPhyRateIndex[phy][pRc->validPhyRateCount[phy]] = i;
            pRc->validPhyRateCount[phy] += 1;
            rcSetValidTxMask(pRc, i, TRUE);
            hi = A_MAX(hi, i);
        }
    }
    return hi;
}

/*
 * Initialize the Valid Rate Index from Rate Set
 */
static A_UINT8
rcSibSetValidRates(struct atheros_node *pSib, const RATE_TABLE_11N *pRateTable,
                   struct ieee80211_rateset *pRateSet, A_UINT32 capflag)
{
    A_UINT8      i, j, hi = 0;
    TX_RATE_CTRL *pRc  = (TX_RATE_CTRL *)(&pSib->txRateCtrlViVo);

    /* Use intersection of working rates and valid rates */
    for (i = 0; i < pRateSet->rs_nrates; i++) {
        for (j = 0; j < pRateTable->rateCount; j++) {
            A_UINT32 phy = pRateTable->info[j].phy;
            A_UINT32 valid = (pSib->singleStream ? pRateTable->info[j].validSingleStream :
                                                   pRateTable->info[j].valid);

            /* We allow a rate only if its valid and the capflag matches one of
             * the validity (TRUE/TRUE_20/TRUE_40) flags
             */
            if (((pRateSet->rs_rates[i] & 0x7F) == (pRateTable->info[j].dot11Rate & 0x7F))
                && ((valid & WLAN_RC_CAP_MODE(capflag)) == WLAN_RC_CAP_MODE(capflag))
                && !WLAN_RC_PHY_HT(phy))
            {

                if (!rcIsValidPhyRate(phy, capflag, FALSE))
                    continue;

                pRc->validPhyRateIndex[phy][pRc->validPhyRateCount[phy]] = j;
                pRc->validPhyRateCount[phy] += 1;
                rcSetValidTxMask(pRc, j, TRUE);
                hi = A_MAX(hi, j);
            }
        }
    }
    return hi;
}

static A_UINT8
rcSibSetValidHtRates(struct atheros_node *pSib, const RATE_TABLE_11N *pRateTable,
                     A_UINT8 *pMcsSet, A_UINT32 capflag)
{
    A_UINT8     i, j, hi = 0;
    TX_RATE_CTRL *pRc  = (TX_RATE_CTRL *)(&pSib->txRateCtrlViVo);

    /* Use intersection of working rates and valid rates */
    for (i = 0; i <  ((struct ieee80211_rateset *)pMcsSet)->rs_nrates; i++) {
        for (j = 0; j < pRateTable->rateCount; j++) {
            A_UINT32 phy = pRateTable->info[j].phy;
            A_UINT32 valid = (pSib->singleStream ? pRateTable->info[j].validSingleStream :
                                                       pRateTable->info[j].valid);

            if (((((struct ieee80211_rateset *)pMcsSet)->rs_rates[i] & 0x7F)
                != (pRateTable->info[j].dot11Rate & 0x7F)) || !WLAN_RC_PHY_HT(phy) ||
                !WLAN_RC_PHY_HT_VALID(valid, capflag))
            {
                continue;
            }

            if (!rcIsValidPhyRate(phy, capflag, FALSE))
                continue;

            pRc->validPhyRateIndex[phy][pRc->validPhyRateCount[phy]] = j;
            pRc->validPhyRateCount[phy] += 1;
            rcSetValidTxMask(pRc, j, TRUE);
            hi = A_MAX(hi, j);
        }
    }
    return hi;
}

/*
 *  Update the SIB's rate control information
 *
 *  This should be called when the supported rates change
 *  (e.g. SME operation, wireless mode change)
 *
 *  It will determine which rates are valid for use.
 */
static void
rcSibUpdate_ht(struct ath_softc *sc, struct ath_node *an, A_UINT32 capflag, int keepState,
               struct ieee80211_rateset *negotiated_rates,
               struct ieee80211_rateset *negotiated_htrates)
{
    RATE_TABLE_11N            *pRateTable;
    struct atheros_node	      *pSib	  = ATH_NODE_ATHEROS(an);
    struct atheros_softc      *asc 	  = (struct atheros_softc*)sc->sc_rc;
    struct ieee80211_rateset  *pRateSet = negotiated_rates;
    A_UINT8 *phtMcs = (A_UINT8 *)negotiated_htrates;
    TX_RATE_CTRL *pRc  = (TX_RATE_CTRL *)(&pSib->txRateCtrlViVo);
    A_UINT8 i, j, k, hi = 0, htHi = 0;

    pRateTable = (RATE_TABLE_11N*)asc->hwRateTable[sc->sc_curmode];

    /* Initial rate table size. Will change depending on the working rate set */
    pRc->rateTableSize = MAX_TX_RATE_TBL;

    /* Initialize thresholds according to the global rate table */
    for (i = 0 ; (i < pRc->rateTableSize) && (!keepState); i++) {
        pRc->state[i].rssiThres = pRateTable->info[i].rssiAckValidMin;
        pRc->state[i].per       = 0;
    }

    /* Determine the valid rates */
    rcInitValidTxMask(pRc);

    for (i = 0; i < WLAN_RC_PHY_MAX; i++) {
        for (j = 0; j < MAX_TX_RATE_PHY; j++) {
            pRc->validPhyRateIndex[i][j] = 0;
        }
        pRc->validPhyRateCount[i] = 0;
    }

    pRc->rcPhyMode = (capflag & WLAN_RC_40_FLAG);

    /* Set stream capability */
    pSib->singleStream = (capflag & WLAN_RC_DS_FLAG) ? 0 : 1;

    if (!pRateSet->rs_nrates) {
        /* No working rate, just initialize valid rates */
        hi = rcSibInitValidRates(pSib, pRateTable, capflag);
    } else {
        /* Use intersection of working rates and valid rates */
        hi = rcSibSetValidRates(pSib, pRateTable, pRateSet, capflag);
        if (capflag & WLAN_RC_HT_FLAG) {
            htHi = rcSibSetValidHtRates(pSib, pRateTable, phtMcs, capflag);
        }
        hi = A_MAX(hi, htHi);
    }

    pRc->rateTableSize = hi + 1;
    ASSERT(pRc->rateTableSize <= MAX_TX_RATE_TBL);

    for (i = 0, k = 0; i < WLAN_RC_PHY_MAX; i++) {
        for (j = 0; j < pRc->validPhyRateCount[i]; j++) {
            pRc->validRateIndex[k++] = pRc->validPhyRateIndex[i][j];
        }

        if (!rcIsValidPhyRate(i, pRateTable->initialRateMax, TRUE) || !pRc->validPhyRateCount[i])
           continue;

        //pRc->rateMaxPhy = pRc->validPhyRateIndex[i][j-1];
    }

    ASSERT(pRc->rateTableSize <= MAX_TX_RATE_TBL);
    ASSERT(k <= MAX_TX_RATE_TBL);

    pRc->maxValidRate = k;
    /*
     * Some third party vendors don't send the supported rate series in order. So sorting to
     * make sure its in order, otherwise our RateFind Algo will select wrong rates
     */
    rcSortValidRates(pRateTable, pRc);
    pRc->rateMaxPhy = pRc->validRateIndex[k-4];
    pRc->probeInterval = sc->sc_rc_params[1].probe_interval;

    //dump_valid_rate_index(pRateTable, pRc->validRateIndex);
    //printk("RateTable:%d, maxvalidrate:%d, ratemax:%d\n", pRc->rateTableSize,k,pRc->rateMaxPhy);

}

void
rcSibUpdate_11nViVo( struct ath_softc *sc, struct ath_node *pSib,
                      A_UINT32 capflag, int keepState,
                      struct ieee80211_rateset *negotiated_rates,
                      struct ieee80211_rateset *negotiated_htrates)
{
    rcSibUpdate_ht(sc, pSib, capflag,
                   keepState, negotiated_rates, negotiated_htrates);
}


static A_UINT8
rcRateFind_ht(struct ath_softc *sc, struct atheros_node *pSib, A_UINT8 ac,
		  const RATE_TABLE_11N *pRateTable, int probeAllowed, int *isProbing, int isretry)
{
    A_UINT32      bestThruput, thisThruput;
    A_UINT32      nowMsec;
    A_UINT8       rate, nextRate, thisRate, minRate;
    A_INT8        index, rc_index;
    TX_RATE_CTRL  *pRc;

    ASSERT (pSib);
    pRc  = (TX_RATE_CTRL *)(&pSib->txRateCtrlViVo);

    *isProbing = FALSE;

    nowMsec = A_MS_TICKGET();

    bestThruput = 0;
    rate = 0;
    minRate = 0;
    /*
     * Try the higher rate first. It will reduce memory moving time
     * if we have very good channel characteristics.
     */

    for (index = pRc->maxValidRate-1; index >= 0 ; index--) {
        A_UINT8 per;

        thisRate = pRc->validRateIndex[index];

        if (thisRate > pRc->rateMaxPhy) {
            continue;
        }

        if (pRateTable->info[thisRate].userRateKbps >
            sc->sc_ac_params[ac].min_kbps)
        {
            minRate = thisRate;
        }

        /*
         * For TCP the average collision rate is around 11%,
         * so we ignore PERs less than this.  This is to
         * prevent the rate we are currently using (whose
         * PER might be in the 10-15 range because of TCP
         * collisions) looking worse than the next lower
         * rate whose PER has decayed close to 0.  If we
         * used to next lower rate, its PER would grow to
         * 10-15 and we would be worse off then staying
         * at the current rate.
         */
        per = pRc->state[thisRate].per;

        if (per < 12) {
            per = 12;
        }

        thisThruput = pRateTable->info[thisRate].userRateKbps *
                      (100 - per);

        if (bestThruput <= thisThruput) {
            bestThruput = thisThruput;
            rate = thisRate;
        }
    }

    if (minRate > rate) {
        rate = minRate;
    }

	/*
	 * For the headline block removal feature, if 
	 * the minRate for a node is MCS2, and PER >= 55 
	 * (for video/voice), then the data to be 
	 * transmitted to this node maight block 
	 * xmission to other nodes with higher rates 
	 * and lower PER. Then the headline block 
	 * removal state machine will be triggered, 
	 * by blocking further xmission to this node 
	 * untill the probing to this node succeedes.
	 * Probing means checking the status of the 
	 * link to the blocked node by sending out QoS 
	 * Null frames. Either N continuous DoSNull 
	 * frames been sent successfully, or the PER 
	 * drops below 35, the "probing" succeeds, and the 
	 * transmission to this node resumes.
	 */
	pSib->an->an_minRate[ac] = minRate;

	if (sc->sc_hbr_params[ac].hbr_enable && 
		rate == minRate &&
		pRc->state[rate].per >= sc->sc_hbr_per_high) /* PER default as 55 by default */
	{
		if (pSib && pSib->an) {
			sc->sc_ieee_ops->hbr_settrigger(pSib->an->an_node, HBR_SIGNAL_PROBING);
		}
	}

#if 0
    /* if we are retrying for more than half the number
     * of max retries, use the min rate for the next retry
     */
    if (isretry)
        rate = pRc->validRateIndex[minIndex];
#endif

    /*
     * If we are not picking rateMax for more than 10ms,
     * we should reduce rateMax.
     */

    if (rate < pRc->rateMaxPhy) {
        if ((nowMsec - pRc->rateMaxLastUsed) > 10) {
            pRc->rateMaxPhy = rate;
            pRc->rateMaxLastUsed = nowMsec;
        }
    } else {
        pRc->rateMaxLastUsed = nowMsec;
    }

    if (rate == pRc->rateMaxPhy && probeAllowed) {
        /*
         * attempt to probe only if this rate is lower than the max valid rate
         */
        if (rate < pRc->validRateIndex[pRc->maxValidRate-1]) {
            /* Probe the next allowed phy state */
            if (rcGetNextValidTxRate(pRateTable, pRc, rate, &nextRate) &&
                (nowMsec - pRc->probeTime > pRc->probeInterval) &&
                (pRc->hwMaxRetryPktCnt >= 1))
            {
                rate                  = nextRate;
                pRc->probeRate        = rate;
                pRc->probeTime        = nowMsec;
                pRc->hwMaxRetryPktCnt = 0;
                *isProbing            = TRUE;
            }
        } else {
            /* Reset Probe interval to configured value */
            rc_index = (ac >= WME_AC_VI) ? 1 : 0;
            pRc->probeInterval = sc->sc_rc_params[rc_index].probe_interval;
        }
    }

    /*
     * Make sure rate is not higher than the allowed maximum.
     */
    ASSERT (rate <= (pRc->rateTableSize - 1));

#ifndef REMOVE_PKT_LOG
    if (!sc->sc_txaggr || (sc->sc_txaggr && sc->sc_log_rcfind))
    {
        struct log_rcfind log_data;
        log_data.rc = pRc;
        log_data.rateCode = pRateTable->info[rate].rateCode;
        log_data.rate = rate;
        log_data.rssiReduce = 0;
        log_data.misc[0] = pRc->state[rate].per;
        log_data.misc[1] = 0;
        log_data.ac = ac;
        log_data.isProbing = *isProbing;
        ath_log_rcfind(sc, &log_data, 0);
        sc->sc_log_rcfind = 0;
    }
#endif

    /* record selected rate, which is used to decide if we want to do fast
     * frame
     */
    if (!(*isProbing)) {
        pSib->lastRateKbps = pRateTable->info[rate].rateKbps;
    }
    ASSERT((pRateTable->info[rate].valid && !pSib->singleStream) ||
           (pRateTable->info[rate].validSingleStream && pSib->singleStream));

    return rate;
}

static void
rcRateSetseries(const RATE_TABLE_11N *pRateTable ,
                    struct ath_rc_series *series, A_UINT8 tries, A_UINT8 rix,
                    int rtsctsenable, int stbc)
{
	series->tries = tries;
	series->flags = (rtsctsenable? ATH_RC_RTSCTS_FLAG : 0) |
			(WLAN_RC_PHY_DS(pRateTable->info[rix].phy) ? ATH_RC_DS_FLAG : 0) |
			(WLAN_RC_PHY_40(pRateTable->info[rix].phy) ? ATH_RC_CW40_FLAG : 0) |
			(WLAN_RC_PHY_SGI(pRateTable->info[rix].phy) ? ATH_RC_SGI_FLAG : 0);

    if (stbc) {
        /* For now, only single stream STBC is supported */
        if (pRateTable->info[rix].rateCode >= ATH_MCS0 &&
            pRateTable->info[rix].rateCode <= ATH_MCS7)
        {
            series->flags |= ATH_RC_TX_STBC_FLAG;
        }
    }

	series->rix = pRateTable->info[rix].baseIndex;
	series->max4msframelen = pRateTable->info[rix].max4msframelen;
}

static A_UINT8
rcRateGetIndex(struct ath_softc *sc, struct ath_node *an,
                A_UINT8 ac, const RATE_TABLE_11N *pRateTable,
                A_UINT8 rix, A_UINT16 stepDown, A_UINT16 minRate)
{
    A_UINT32 j;
    A_UINT8 nextIndex;
    struct atheros_node *pSib;
    TX_RATE_CTRL *pRc;

    pSib = ATH_NODE_ATHEROS(an);
    pRc = (TX_RATE_CTRL *)(&pSib->txRateCtrlViVo);

	if (minRate) {
		for (j = RATE_TABLE_SIZE_11N; j > 0; j-- ) {
			if (rcGetNextLowerValidTxRate(pRateTable, pRc, rix, &nextIndex)) {
				rix = nextIndex;
			} else {
				break;
			}
		}
	} else {
		for (j = stepDown; j > 0; j-- ) {
			if (rcGetNextLowerValidTxRate(pRateTable, pRc, rix, &nextIndex)) {
                if (pRateTable->info[nextIndex].userRateKbps > sc->sc_ac_params[ac].min_kbps) {
				    rix = nextIndex;
                } else {
				    break;
                }
			} else {
				break;
			}
		}
	}
	return rix;
}

void
rcRateFind_11nViVo(struct ath_softc *sc, struct ath_node *an, A_UINT8 ac,
              int numTries, int numRates, unsigned int rcflag,
              struct ath_rc_series series[], int *isProbe, int isretry)
{
    A_UINT8 i=0;
    A_UINT8 tryPerRate = 0;
    A_UINT8 rix, nrix, use_rts;
    struct atheros_softc  *asc = (struct atheros_softc*)sc->sc_rc;
    RATE_TABLE_11N        *pRateTable = (RATE_TABLE_11N *)asc->hwRateTable[sc->sc_curmode];
    struct atheros_node   *asn = ATH_NODE_ATHEROS(an);
	asn->an = an;

    rix = rcRateFind_ht(sc, asn, ac, pRateTable, (rcflag & ATH_RC_PROBE_ALLOWED) ? 1 : 0, isProbe, isretry);
    nrix = rix;

    use_rts = sc->sc_ac_params[ac].use_rts;

    if ((rcflag & ATH_RC_PROBE_ALLOWED) && (*isProbe)) {
        /* set one try for probe rates. For the probes don't enable rts */
        rcRateSetseries(pRateTable, &series[i++], 1, nrix, (use_rts) ? TRUE:FALSE, asn->stbc);

        tryPerRate = (numTries/numRates);
        /* Get the next tried/allowed rate. No RTS for the next series
         * after the probe rate
         */
        nrix = rcRateGetIndex( sc, an, ac, pRateTable, nrix, 1, FALSE);
        rcRateSetseries(pRateTable, &series[i++], tryPerRate, nrix, (use_rts) ? TRUE:FALSE, asn->stbc);
    } else {
        tryPerRate = (numTries/numRates);
        /* Set the choosen rate. No RTS for first series entry. */
        rcRateSetseries(pRateTable, &series[i++], tryPerRate, nrix, (use_rts) ? TRUE:FALSE, asn->stbc);
    }

    /* Fill in the other rates for multirate retry */
    for ( ; i < numRates; i++ ) {
        A_UINT8 tryNum;
        A_UINT8 minRate;

        tryNum  = ((i + 1) == numRates) ? numTries - (tryPerRate * i) : tryPerRate ;
        minRate = (((i + 1) == numRates) && (rcflag & ATH_RC_MINRATE_LASTRATE)) ? 1 : 0;

        nrix = rcRateGetIndex(sc, an, ac, pRateTable, nrix, 1, minRate);
        /* All other rates in the series have RTS enabled */
        rcRateSetseries(pRateTable, &series[i], tryNum, nrix, TRUE, asn->stbc);
    }

    /*
     * BUG 26545:
     * Change rate series to enable aggregation when operating at lower MCS rates.
     * When first rate in series is MCS2 in HT40 @ 2.4GHz, series should look like:
     *    {MCS2, MCS1, MCS0, MCS0}.
     * When first rate in series is MCS3 in HT20 @ 2.4GHz, series should look like:
     *    {MCS3, MCS2, MCS1, MCS1}
     * So, set fourth rate in series to be same as third one for above conditions.
     */
    if ((sc->sc_curmode == WIRELESS_MODE_11NG_HT20) ||
        (sc->sc_curmode == WIRELESS_MODE_11NG_HT40PLUS) ||
        (sc->sc_curmode == WIRELESS_MODE_11NG_HT40MINUS))
    {
        A_UINT8  dot11Rate = pRateTable->info[rix].dot11Rate;
        WLAN_PHY phy = pRateTable->info[rix].phy;
        if (i == 4 &&
            ((dot11Rate == 2 && phy == WLAN_RC_PHY_HT_40_SS) ||
             (dot11Rate == 3 && phy == WLAN_RC_PHY_HT_20_SS)))
        {
            series[3].rix = series[2].rix;
            series[3].flags = series[2].flags;
            series[3].max4msframelen = series[2].max4msframelen;
        }
    }
}


/*
 * Calculate PER updates
 * new_PER = 7/8*old_PER + 1/8*(currentPER)
 */
static inline A_UINT8
perCal(A_UINT8 per, int retries, A_UINT16 nFrames, A_UINT16 nBad)
{
    A_UINT8 newPer;
    int i;

    newPer = per;

    /* Update per for all intermediate failure attempts */
    for (i = 0; i < retries; i++) {
        newPer = ((7*newPer) + 100) >> 3;
    }

    /* Update per for the final successful attempt, if any */
    if (nBad < nFrames) {
        newPer = ((7*newPer) + (100 * nBad)/nFrames) >> 3;
    }

    return newPer;
}

static void
rcUpdate_ht(struct ath_softc *sc, struct ath_node *an, A_UINT8 ac, int txRate,
            int Xretries, int retries, A_RSSI rssiAck, A_UINT32 nowMsec,
            A_UINT16 nFrames, A_UINT16 nBad)
{
    TX_RATE_CTRL *pRc;
    A_UINT8      lastPer, lastminRatePer, rc_index;
    int          rate;
    struct atheros_node *pSib;
    struct atheros_softc *asc;
    RATE_TABLE_11N *pRateTable;

    pSib = ATH_NODE_ATHEROS(an);
    asc = (struct atheros_softc*)sc->sc_rc;
    rc_index = (ac >= WME_AC_VI) ? 1 : 0;

    if (!pSib || !asc) {
        return;
    }

    pRc = (TX_RATE_CTRL *)(&pSib->txRateCtrlViVo);
    pRateTable = (RATE_TABLE_11N *)asc->hwRateTable[sc->sc_curmode];

    ASSERT(txRate >= 0);

	/* To compensate for some imbalance between ctrl and ext. channel */

    if (WLAN_RC_PHY_40(pRateTable->info[txRate].phy))
		rssiAck = rssiAck < 3? 0: rssiAck - 3;

    lastPer = pRc->state[txRate].per;
	lastminRatePer = pRc->state[an->an_minRate[ac]].per;

    pRc->state[txRate].per = perCal(pRc->state[txRate].per, retries, nFrames, nBad);
		

    if (!Xretries) {
        /*
         * If we got at most one retry then increase the max rate if
         * this was a probe.  Otherwise, ignore the probe.
         */
        if (pRc->probeRate && pRc->probeRate == txRate) {
            /*
             * Since we probed with just a single attempt,
             * any retries means the probe failed.  Also,
             * if the attempt worked, but more than half
             * the subframes were bad then also consider
             * the probe a failure.
             */
            if (retries == 0 && nBad <= nFrames/2) {
                A_UINT8 prevRate;
                A_UINT32 interval;

                pRc->rateMaxPhy = pRc->probeRate;

                if (rcGetNextLowerValidTxRate(pRateTable, pRc, (A_UINT8)pRc->probeRate, &prevRate)) {
                    pRc->state[pRc->probeRate].per = pRc->state[prevRate].per;
                }

                /*
                 * Since this probe succeeded, we allow the next probe
                 * twice as soon.  This allows the maxRate to move up
                 * faster if the probes are succesful. Dont let the
                 * probe interval fall below MIN_PROBE_INTERVAL ms.
                 */
                interval = pRc->probeInterval/2;
                pRc->probeInterval = (interval > MIN_PROBE_INTERVAL) ? interval : MIN_PROBE_INTERVAL;
                pRc->probeTime = nowMsec;
            } else {
                /* Reset Probe interval to configured value  on probe failure */
                pRc->probeInterval = sc->sc_rc_params[rc_index].probe_interval;
            }
        }

        if (retries > 0) {
            /*
             * Don't update anything.  We don't know if this was because
             * of collisions or poor signal.
             *
             */
            pRc->hwMaxRetryPktCnt = 0;
        } else {
            /* XXX */
            if (txRate == pRc->rateMaxPhy && pRc->hwMaxRetryPktCnt < 255) {
                pRc->hwMaxRetryPktCnt++;
            }
        }
    }

    if (pRc->probeRate == txRate) {
        pRc->probeRate = 0;
    }

	/* For all cases */

    /*
     * If this rate looks bad (high PER) then stop using it for
     * a while (except if we are probing).
     */
    if (pRc->state[txRate].per >= sc->sc_rc_params[rc_index].per_threshold && txRate > 0 &&
        pRateTable->info[txRate].rateKbps <=
				pRateTable->info[pRc->rateMaxPhy].rateKbps)
    {
        A_UINT8 nextRate;

        if (rcGetNextLowerValidTxRate(pRateTable, pRc, (A_UINT8) txRate, &nextRate)) {
            if (pRateTable->info[nextRate].userRateKbps > sc->sc_ac_params[ac].min_kbps) {
                pRc->rateMaxPhy = nextRate;
            }
        }

        /* Don't probe for a little while. */
        pRc->probeTime = nowMsec;
    }


    /* Make sure the rates below this have lower PER */
    /* Monotonicity is kept only for rates below the current rate. */
    if (pRc->state[txRate].per < lastPer) {
        for (rate = txRate - 1; rate >= 0; rate--) {
            if (pRateTable->info[rate].phy != pRateTable->info[txRate].phy) {
                break;
            }

            if (pRc->state[rate].per > pRc->state[rate+1].per) {
                pRc->state[rate].per = pRc->state[rate+1].per;
            }
        }
    }

    if (pRc->state[txRate].per > lastPer) {
        A_UINT8 nextRate;
        rate = txRate;
        /*
         * If PER of current rate is higher than the next upper valid rate
         * the upper rate PER data is likely stale. Reduce rateMaxPhy.
         */
        if (rcGetNextValidTxRate(pRateTable, pRc, rate, &nextRate)) {
            if ((pRc->state[nextRate].per < pRc->state[rate].per) &&
                (pRc->rateMaxPhy > rate))
            {
                pRc->rateMaxPhy = rate;
            }
        }
    }

    /* Every so often, we reduce the thresholds and PER (different for CCK and OFDM). */
    if (nowMsec - pRc->perDownTime >= pRateTable->rssiReduceInterval) {
        for (rate = 0; rate < pRc->rateTableSize; rate++) {
            pRc->state[rate].per = ((7*pRc->state[rate].per) >> 3);
        }

        pRc->perDownTime = nowMsec;
    }

	/*
	 * Send signal back to headline block removal state machine: hey, the PER
	 * is lower than the threshold to go back to ACTIVE state
	 */
	
	if (sc->sc_hbr_params[ac].hbr_enable &&
//	 	lastminRatePer > sc->sc_hbr_per_low && 
//		((struct ieee80211_node*)(an->an_node))->ni_hbr_block &&
		pRc->state[an->an_minRate[ac]].per <= sc->sc_hbr_per_low) 
	{
		sc->sc_ieee_ops->hbr_settrigger(an->an_node, HBR_SIGNAL_ACTIVE);
	}


#ifndef REMOVE_PKT_LOG
    {
        struct log_rcupdate log_data;

        log_data.rc = pRc;
        log_data.txRate = txRate;
        log_data.rateCode = pRateTable->info[txRate].rateCode;
        log_data.Xretries = Xretries;
        log_data.currentBoostState = 0;
        log_data.useTurboPrime = 0;
        log_data.retries = retries;
        log_data.rssiAck = rssiAck;
        log_data.ac = ac;
        ath_log_rcupdate(sc, &log_data, 0);
   }
#endif
}

static inline A_UINT32
getRateIndex(RATE_TABLE_11N * pRateTable, A_UINT32 index, A_UINT8 flags)
{
    A_UINT32 rix;

    if ((flags & ATH_RC_CW40_FLAG) && (flags & ATH_RC_SGI_FLAG)) {
        rix = pRateTable->info[index].htIndex;
    } else if (flags & ATH_RC_SGI_FLAG) {
        rix = pRateTable->info[index].sgiIndex;
    } else if (flags & ATH_RC_CW40_FLAG) {
        rix = pRateTable->info[index].cw40Index;
    } else {
        rix = pRateTable->info[index].baseIndex;
    }

    return rix;
}

/*
 * This routine is called by the Tx interrupt service routine to give
 * the status of previous frames.
 */
void
rcUpdate_11nViVo(struct ath_softc *sc, struct ath_node *an, A_RSSI rssiAck, A_UINT8 ac,
            int finalTSIdx, int Xretries, struct ath_rc_series rcs[], int nFrames,
            int  nBad, int long_retry)
{
    A_UINT32 series, tries;
    A_UINT32 rix, index;
    A_UINT32 nowMsec;
    struct atheros_softc *asc;
    RATE_TABLE_11N *pRateTable;
    struct atheros_node *pSib;
    TX_RATE_CTRL *pRc;
    A_UINT8 flags;
    int total_frames, bad_frames;

    ASSERT(an);

    pSib = ATH_NODE_ATHEROS(an);
    pRc = (TX_RATE_CTRL *)(&pSib->txRateCtrlViVo);
    asc = (struct atheros_softc*)sc->sc_rc;
    pRateTable = (RATE_TABLE_11N *)asc->hwRateTable[sc->sc_curmode];
    nowMsec = A_MS_TICKGET();
    ASSERT (rcs[0].tries != 0);

    /* Process the entire rate series */
    for (series = 0; series <= finalTSIdx ; series++) {
        if (rcs[series].tries != 0) {
            flags = rcs[series].flags;

            /* If HT40 and we have switched mode from 40 to 20 => don't update */
            if ((flags & ATH_RC_CW40_FLAG) &&
                (pRc->rcPhyMode != (flags & ATH_RC_CW40_FLAG)))
            {
                return;
            }

            index = rcs[series].rix;
            rix = getRateIndex(pRateTable, index, flags);

            if ((series == finalTSIdx) && !Xretries) {
                tries = long_retry;
                total_frames = nFrames;
                bad_frames = nBad;
            } else {
                /* transmit failed completely */
                tries = rcs[series].tries;
                total_frames = nFrames;
                bad_frames = nFrames;
            }

            rcUpdate_ht(sc, an, ac, rix, Xretries, tries, rssiAck, nowMsec, total_frames, bad_frames);

        }
    }

}

#endif /* ATH_SUPPORT_IQUE */
