/*
 * $Id: //depot/sw/releases/7.3_AP/wlan/common/lmac/ratectrl/ratectrl_11n.c#10 $
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
#include "ath_internal.h"
#endif

#ifndef REMOVE_PKT_LOG
#include "pktlog_rc.h"
extern struct ath_pktlog_rcfuncs *g_pktlog_rcfuncs;
#endif

#define MULTI_RATE_RETRY_ENABLE 1

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
                        pRateTable->info[pRc->validRateIndex[j+1]].rateKbps) {
                        A_UINT8 tmp=0;
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
    // Bug 54801: Only check for 40 Phy & 20 Cap. Removed 20 Phy & 40 Cap check.
    if (!ignoreCW && WLAN_RC_PHY_HT(phy)) {
        if (WLAN_RC_PHY_40(phy) && !(capflag & WLAN_RC_40_FLAG)) {
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
    TX_RATE_CTRL    *pRc  = (TX_RATE_CTRL *)(pSib);
    A_UINT8         i, hi = 0;
    A_UINT32        valid;

    for (i = 0; i < pRateTable->rateCount; i++) {
        /* Check for single stream, to avoid dual stream rates for 
         * single stream device */
        if (pSib->singleStream) {
            valid = pRateTable->info[i].validSingleStream;
        } else if (pSib->stbc) {
            valid = pRateTable->info[i].validSTBC;
        } else {
            valid = pRateTable->info[i].valid;
        }

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
    TX_RATE_CTRL *pRc  = (TX_RATE_CTRL *)(pSib);

    /* Use intersection of working rates and valid rates */
    for (i = 0; i < pRateSet->rs_nrates; i++) {
        for (j = 0; j < pRateTable->rateCount; j++) {
            A_UINT32 phy = pRateTable->info[j].phy;
            A_UINT32 valid;

            /* Check for single stream, to avoid dual stream rates for 
             * single stream device */
            if (pSib->singleStream) {
                valid = pRateTable->info[j].validSingleStream;
            } else if (pSib->stbc) {
                valid = pRateTable->info[j].validSTBC;
            } else {
                valid = pRateTable->info[j].valid;
            }

		/* We allow a rate only if its valid and the capflag matches one of
		 * the validity (TRUE/TRUE_20/TRUE_40) flags */

            if (((pRateSet->rs_rates[i] & 0x7F) == (pRateTable->info[j].dot11Rate & 0x7F))
		   && ((valid & WLAN_RC_CAP_MODE(capflag)) ==
		   WLAN_RC_CAP_MODE(capflag)) && !WLAN_RC_PHY_HT(phy))
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
    TX_RATE_CTRL *pRc  = (TX_RATE_CTRL *)(pSib);

    /* Use intersection of working rates and valid rates */
    for (i = 0; i <  ((struct ieee80211_rateset *)pMcsSet)->rs_nrates; i++) {
        for (j = 0; j < pRateTable->rateCount; j++) {
            A_UINT32 phy = pRateTable->info[j].phy;
            A_UINT32 valid;

            /* Check for single stream, to avoid dual stream rates for 
             * single stream device */
            if (pSib->singleStream) {
                valid = pRateTable->info[j].validSingleStream;
            } else if (pSib->stbc) {
                valid = pRateTable->info[j].validSTBC;
            } else {
                valid = pRateTable->info[j].valid;
            }

		    if (((((struct ieee80211_rateset *)pMcsSet)->rs_rates[i] & 0x7F)
			    != (pRateTable->info[j].dot11Rate & 0x7F)) ||
                !WLAN_RC_PHY_HT(phy) || !WLAN_RC_PHY_HT_VALID(valid, capflag) || 
                ((pRateTable->info[j].dot11Rate >= 13) && 
      		 (capflag & WLAN_RC_WEP_TKIP_FLAG))) 
                    continue;

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

static A_UINT8
rcSibSingleStream(struct ath_softc *sc,struct ieee80211_rateset * negotiated_htrates)
{
    /* 
     * Currently only Transmit capability is being considered. Need to 
     * consider the capability of the receiving node as well.
     */

	int i,ds_rates=0;
	for (i=8;i<=15;i++)
	{
		if(negotiated_htrates->rs_rates[i]) 
		{
			ds_rates++;
			break;
		}
	}
	/*  Return Single stream if any one of AP or STA is
	 *  only Single stream capable 
	 */
	return ((ds_rates)? FALSE:TRUE) ||  ((sc->sc_tx_chainmask == 1) ? TRUE : FALSE);
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
    RATE_TABLE_11N                *pRateTable = 0;
    struct atheros_node	      *pSib	  = ATH_NODE_ATHEROS(an);
    struct atheros_softc      *asc 	  = (struct atheros_softc*)sc->sc_rc;
    struct ieee80211_rateset  *pRateSet = negotiated_rates;
    A_UINT8		      *phtMcs = (A_UINT8 *)negotiated_htrates;
    TX_RATE_CTRL *pRc  = (TX_RATE_CTRL *)(pSib);
    A_UINT8                 i, j, k, hi = 0, htHi = 0;

    pRateTable = (RATE_TABLE_11N*)asc->hwRateTable[sc->sc_curmode];

    /* Initial rate table size. Will change depending on the working rate set */
    pRc->rateTableSize = MAX_TX_RATE_TBL;

    pRc->switchCount = 0;
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
    pSib->singleStream = rcSibSingleStream(sc,negotiated_htrates);

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
    pRc->rateMaxPhy = 0;
    ASSERT(pRc->rateTableSize <= MAX_TX_RATE_TBL);

    for (i = 0, k = 0; i < WLAN_RC_PHY_MAX; i++) {
        for (j = 0; j < pRc->validPhyRateCount[i]; j++) {
            pRc->validRateIndex[k++] = pRc->validPhyRateIndex[i][j];
        }

        if (!rcIsValidPhyRate(i, pRateTable->initialRateMax, TRUE) || !pRc->validPhyRateCount[i])
               continue;

        pRc->rateMaxPhy = pRc->validPhyRateIndex[i][j-1];
    }
    ASSERT(pRc->rateTableSize <= MAX_TX_RATE_TBL);
    ASSERT(k <= MAX_TX_RATE_TBL);

    /*Clear the probeRate of previous rateTable */
    pRc->probeRate = 0;

    pRc->maxValidRate = k;
    /*
     * Some third party vendors don't send the supported rate series in order. So sorting to
     * make sure its in order, otherwise our RateFind Algo will select wrong rates
     */
    rcSortValidRates(pRateTable, pRc);
    pRc->rateMaxPhy = pRc->validRateIndex[k-4];

    //dump_valid_rate_index(pRateTable, pRc->validRateIndex);
    //printk("RateTable:%d, maxvalidrate:%d, ratemax:%d\n", pRc->rateTableSize,k,pRc->rateMaxPhy);

}

void
rcSibUpdate_11n( struct ath_softc *sc, struct ath_node *pSib,
                      A_UINT32 capflag, int keepState,
                      struct ieee80211_rateset *negotiated_rates,
                      struct ieee80211_rateset *negotiated_htrates)
{
    rcSibUpdate_ht(sc, pSib, capflag,
                   keepState, negotiated_rates, negotiated_htrates);
}

#ifdef notyet
/*
 *  This routine is called to initialize the rate control parameters
 *  in the SIB. It is called initially during system initialization
 *  or when a station is associated with the AP.
 */
void
rcSibInit(struct ath_softc *sc, struct ath_node *an)
{
    struct atheros_node    *pSib = ATH_NODE_ATHEROS(an);
    TX_RATE_CTRL    *pRc = (TX_RATE_CTRL *)(pSib);

    A_MEM_ZERO((char *)pRc, sizeof(*pRc));
    pRc->rssiDownTime = A_MS_TICKGET();
    //pSib->rateInit = FALSE;
}
#endif


/*
 * Return the median of three numbers
 */
static INLINE A_RSSI
median(A_RSSI a, A_RSSI b, A_RSSI c)
{
    if (a >= b) {
        if (b >= c) {
            return b;
        } else if (a > c) {
            return c;
        } else {
            return a;
        }
    } else {
        if (a >= c) {
            return a;
        } else if (b >= c) {
            return c;
        } else {
            return b;
        }
    }
}

static A_UINT8
rcRateFind_ht(struct ath_softc *sc, struct atheros_node *pSib, A_UINT8 ac,
		  const RATE_TABLE_11N *pRateTable, int probeAllowed, int *isProbing, int isretry)
{
    A_UINT32             dt;
    A_UINT32             bestThruput, thisThruput;
    A_UINT32             nowMsec;
    A_UINT8              rate, nextRate, bestRate;
    A_RSSI               rssiLast, rssiReduce=0;
    A_UINT8              maxIndex, minIndex;
    A_INT8               index;
    TX_RATE_CTRL  *pRc         = NULL;

    pRc = (TX_RATE_CTRL *)(pSib ? (pSib) : NULL);

    *isProbing = FALSE;

    rssiLast   = median(pRc->rssiLast, pRc->rssiLastPrev, pRc->rssiLastPrev2);

    /*
     * Age (reduce) last ack rssi based on how old it is.
     * The bizarre numbers are so the delta is 160msec,
     * meaning we divide by 16.
     *   0msec   <= dt <= 25msec:   don't derate
     *   25msec  <= dt <= 185msec:  derate linearly from 0 to 10dB
     *   185msec <= dt:             derate by 10dB
     */

    nowMsec = A_MS_TICKGET();
    dt = nowMsec - pRc->rssiTime;

    if (dt >= 185) {
        rssiReduce = 10;
    } else if (dt >= 25) {
        rssiReduce = (A_UINT8)((dt - 25) >> 4);
    }

    /* Now reduce rssiLast by rssiReduce */
    if (rssiLast < rssiReduce) {
        rssiLast = 0;
    } else {
        rssiLast -= rssiReduce;
    }

    /*
     * Now look up the rate in the rssi table and return it.
     * If no rates match then we return 0 (lowest rate)
     */

    bestThruput = 0;
    maxIndex    = pRc->maxValidRate-1;

    /* FIXME: XXX */
    minIndex    = 0;
    bestRate    = minIndex;
    /*
     * Try the higher rate first. It will reduce memory moving time
     * if we have very good channel characteristics.
     */

    for (index = maxIndex; index >= minIndex ; index--) {
        A_UINT8 perThres;

        rate = pRc->validRateIndex[index];

        if (rate > pRc->rateMaxPhy) {
            continue;
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
        perThres = pRc->state[rate].per;
        if ( perThres < 12 ) {
            perThres = 12;
        }

        thisThruput = pRateTable->info[rate].userRateKbps *
                      (100 - perThres);

        if (bestThruput <= thisThruput) {
            bestThruput = thisThruput;
            bestRate    = rate;
        }
    }

    rate = bestRate;

    /* if we are retrying for more than half the number
     * of max retries, use the min rate for the next retry
     */
    if (isretry)
        rate = pRc->validRateIndex[minIndex];

    pRc->rssiLastLkup = rssiLast;

    /*
     * Must check the actual rate (rateKbps) to account for non-monoticity of
     * 11g's rate table
     */

    if (rate >= pRc->rateMaxPhy && probeAllowed) {

	    rate = pRc->rateMaxPhy;

            /* Probe the next allowed phy state */
	    /* FIXME:XXXX Check to make sure ratMax is checked properly */
            if (rcGetNextValidTxRate( pRateTable, pRc, rate, &nextRate) &&
		(nowMsec - pRc->probeTime > pRateTable->probeInterval) &&
                (pRc->hwMaxRetryPktCnt >= 1))
            {
		rate                  = nextRate;
		pRc->probeRate        = rate;
		pRc->probeTime        = nowMsec;
		pRc->hwMaxRetryPktCnt = 0;
		*isProbing            = TRUE;

            }
   }

    /*
     * Make sure rate is not higher than the allowed maximum.
     * We should also enforce the min, but I suspect the min is
     * normally 1 rather than 0 because of the rate 9 vs 6 issue
     * in the old code.
     */
    if (rate > (pRc->rateTableSize - 1)) {
        rate = pRc->rateTableSize - 1;
    }

#ifndef REMOVE_PKT_LOG
    if (!sc->sc_txaggr || (sc->sc_txaggr && sc->sc_log_rcfind))
    {
        struct log_rcfind log_data;
        log_data.rc = pRc;
        log_data.rateCode = pRateTable->info[rate].rateCode;
        log_data.rate = rate;
        log_data.rssiReduce = rssiReduce;
        log_data.misc[0] = pRc->state[rate].per;
        log_data.misc[1] = pRc->state[rate].rssiThres;
        log_data.ac = ac;
        log_data.isProbing = *isProbing;
        ath_log_rcfind(sc, &log_data, 0);
        sc->sc_log_rcfind = 0;
    }
#endif

    /* record selected rate, which is used to decide if we want to do fast 
     * frame
     */
    if (!(*isProbing) && pSib) {
        pSib->lastRateKbps = pRateTable->info[rate].rateKbps;
    }
    ASSERT(pRateTable->info[rate].valid ||
           pRateTable->info[rate].validSingleStream ||
           pRateTable->info[rate].validSTBC);

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
        if (pRateTable->info[rix].rateCode >= 0x80 && 
            pRateTable->info[rix].rateCode <= 0x87)
            series->flags |= ATH_RC_TX_STBC_FLAG;
    }

	series->rix = pRateTable->info[rix].baseIndex;
	series->max4msframelen = pRateTable->info[rix].max4msframelen;
}

static A_UINT8
rcRateGetIndex(struct ath_softc *sc, struct ath_node *an,
               const RATE_TABLE_11N *pRateTable,
		   A_UINT8 rix, A_UINT16 stepDown, A_UINT16 minRate)
{
    A_UINT32                j;
    A_UINT8                 nextIndex;
    struct atheros_node     *pSib = ATH_NODE_ATHEROS(an);
    TX_RATE_CTRL     *pRc = (TX_RATE_CTRL *)(pSib);

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
				rix = nextIndex;
			} else {
				break;
			}
		}
	}
	return rix;
}

void
rcRateFind_11n(struct ath_softc *sc, struct ath_node *an, A_UINT8 ac,
              int numTries, int numRates, unsigned int rcflag,
              struct ath_rc_series series[], int *isProbe, int isretry)
{
    A_UINT8               i=0;
    A_UINT8               tryPerRate = 0;
    struct atheros_softc  *asc        = (struct atheros_softc*)sc->sc_rc;
    RATE_TABLE_11N            *pRateTable = (RATE_TABLE_11N *)asc->hwRateTable[sc->sc_curmode];
    struct atheros_node   *asn	      = ATH_NODE_ATHEROS(an);
    A_UINT8               rix, nrix;

    rix = rcRateFind_ht(sc, asn, ac, pRateTable, (rcflag & ATH_RC_PROBE_ALLOWED) ? 1 : 0, isProbe, isretry);
    nrix = rix;

    if ((rcflag & ATH_RC_PROBE_ALLOWED) && (*isProbe)) {
        /* set one try for probe rates. For the probes don't enable rts */
        rcRateSetseries(pRateTable, &series[i++], 1, nrix, FALSE, asn->stbc);

        tryPerRate = (numTries/numRates);
        /* Get the next tried/allowed rate. No RTS for the next series
         * after the probe rate
         */
        nrix = rcRateGetIndex( sc, an, pRateTable, nrix, 1, FALSE);
        rcRateSetseries(pRateTable, &series[i++], tryPerRate, nrix, 0, asn->stbc);
    } else {
        tryPerRate = (numTries/numRates);
        /* Set the choosen rate. No RTS for first series entry. */
        rcRateSetseries(pRateTable, &series[i++], tryPerRate, nrix, FALSE, asn->stbc);
    }

    /* Fill in the other rates for multirate retry */
    for ( ; i < numRates; i++ ) {
        A_UINT8 tryNum;
        A_UINT8 minRate;

        tryNum  = ((i + 1) == numRates) ? numTries - (tryPerRate * i) : tryPerRate ;
        minRate = (((i + 1) == numRates) && (rcflag & ATH_RC_MINRATE_LASTRATE)) ? 1 : 0;

        nrix = rcRateGetIndex(sc, an, pRateTable, nrix, 1, minRate);
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

static void
rcUpdate_ht(struct ath_softc *sc, struct ath_node *an,  A_UINT8 ac,
            int txRate, int Xretries, int retries, A_RSSI rssiAck,
            A_UINT16 nFrames, A_UINT16 nBad)
{
    TX_RATE_CTRL   *pRc;
    A_UINT32              nowMsec     = A_MS_TICKGET();
    int                stateChange = FALSE;
    A_UINT8               lastPer;
    int                   rate,count;
    //struct ieee80211com   *ic         = &sc->sc_ic;
    struct atheros_node   *pSib       = ATH_NODE_ATHEROS(an);
    struct atheros_softc  *asc        = (struct atheros_softc*)sc->sc_rc;
    RATE_TABLE_11N            *pRateTable = (RATE_TABLE_11N *)asc->hwRateTable[sc->sc_curmode];

    static const A_UINT32 nRetry2PerLookup[10] = {
        100 * 0 / 1,
        100 * 1 / 4,
        100 * 1 / 2,
        100 * 3 / 4,
        100 * 4 / 5,
        100 * 5 / 6,
        100 * 6 / 7,
        100 * 7 / 8,
        100 * 8 / 9,
        100 * 9 / 10
    };

    if (!pSib) {
        return;
    }

    pRc        = (TX_RATE_CTRL *)(pSib);

    // ASSERT(retries >= 0 && retries < MAX_TX_RETRIES);
    ASSERT(txRate >= 0);
    if (txRate < 0) {
            //printk("%s: txRate value of 0x%x is bad.\n", __FUNCTION__, txRate);
            return;
    }

	/* To compensate for some imbalance between ctrl and ext. channel */

    if (WLAN_RC_PHY_40(pRateTable->info[txRate].phy))
		rssiAck = rssiAck < 3? 0: rssiAck - 3;

    lastPer = pRc->state[txRate].per;

    if (Xretries) {
        /* Update the PER. */
        if (Xretries == 1) {
            pRc->state[txRate].per += 30;
            if (pRc->state[txRate].per > 100) {
                pRc->state[txRate].per = 100;
            }

        } else {
	        /* Xretries == 2 */
#ifdef MULTI_RATE_RETRY_ENABLE
            count = sizeof(nRetry2PerLookup) / sizeof(nRetry2PerLookup[0]);
            if (retries >= count) {
                retries = count - 1;
            }
            /* new_PER = 7/8*old_PER + 1/8*(currentPER) */
            pRc->state[txRate].per = (A_UINT8)(pRc->state[txRate].per - (pRc->state[txRate].per >> 3) +
                                     ((100) >> 3));
#endif
        }

	    /* Xretries == 1 or 2 */

        if (pRc->probeRate == txRate)
            pRc->probeRate = 0;

    } else {	/* Xretries == 0 */

        /* Update the PER. */
        /* Make sure it doesn't index out of array's bounds. */
        count = sizeof(nRetry2PerLookup) / sizeof(nRetry2PerLookup[0]);
        if (retries >= count) {
            retries = count - 1;
        }
        if (nBad) {
            /* new_PER = 7/8*old_PER + 1/8*(currentPER)  */
	    /*
             * Assuming that nFrames is not 0.  The current PER
             * from the retries is 100 * retries / (retries+1),
             * since the first retries attempts failed, and the
             * next one worked.  For the one that worked, nBad
             * subframes out of nFrames wored, so the PER for
             * that part is 100 * nBad / nFrames, and it contributes
             * 100 * nBad / (nFrames * (retries+1)) to the above
             * PER.  The expression below is a simplified version
             * of the sum of these two terms.
             */
	    if (nFrames > 0)
            	pRc->state[txRate].per = (A_UINT8)(pRc->state[txRate].per -
                                     (pRc->state[txRate].per >> 3) +
				((100*(retries*nFrames + nBad)/(nFrames*(retries+1))) >> 3));
        } else {
            /* new_PER = 7/8*old_PER + 1/8*(currentPER) */

            pRc->state[txRate].per = (A_UINT8)(pRc->state[txRate].per -
			(pRc->state[txRate].per >> 3) + (nRetry2PerLookup[retries] >> 3));
        }

        pRc->rssiLastPrev2 = pRc->rssiLastPrev;
        pRc->rssiLastPrev  = pRc->rssiLast;
        pRc->rssiLast      = rssiAck;
        pRc->rssiTime      = nowMsec;

        /*
         * If we got at most one retry then increase the max rate if
         * this was a probe.  Otherwise, ignore the probe.
         */

        if (pRc->probeRate && pRc->probeRate == txRate) {
            if (retries > 0 || 2 * nBad > nFrames) {
                /*
                 * Since we probed with just a single attempt,
                 * any retries means the probe failed.  Also,
                 * if the attempt worked, but more than half
                 * the subframes were bad then also consider
                 * the probe a failure.
                 */
                pRc->probeRate = 0;
            } else {

                pRc->rateMaxPhy = pRc->probeRate;

                if (pRc->state[pRc->probeRate].per > 30) {
                    pRc->state[pRc->probeRate].per = 20;
                }

                pRc->probeRate = 0;

                /*
                 * Since this probe succeeded, we allow the next probe
                 * twice as soon.  This allows the maxRate to move up
                 * faster if the probes are succesful.
                 */
                pRc->probeTime = nowMsec - pRateTable->probeInterval / 2;
            }
        }

        if (retries > 0) {
            /*
             * Don't update anything.  We don't know if this was because
             * of collisions or poor signal.
             *
             * Later: if rssiAck is close to pRc->state[txRate].rssiThres
             * and we see lots of retries, then we could increase
             * pRc->state[txRate].rssiThres.
             */
	    pRc->hwMaxRetryPktCnt = 0;
        } else {
            /*
             * It worked with no retries.  First ignore bogus (small)
             * rssiAck values.
             */
            if (txRate == pRc->rateMaxPhy && pRc->hwMaxRetryPktCnt < 255) {
                pRc->hwMaxRetryPktCnt++;
            }

            if (rssiAck >= pRateTable->info[txRate].rssiAckValidMin) {
                /* Average the rssi */
                if (txRate != pRc->rssiSumRate) {
                    pRc->rssiSumRate = txRate;
                    pRc->rssiSum     = pRc->rssiSumCnt = 0;
                }

                pRc->rssiSum += rssiAck;
                pRc->rssiSumCnt++;

                if (pRc->rssiSumCnt > 4) {
                    A_RSSI32 rssiAckAvg = (pRc->rssiSum + 2) / 4;

                    pRc->rssiSum = pRc->rssiSumCnt = 0;

                    /* Now reduce the current rssi threshold. */
                    if ((rssiAckAvg < pRc->state[txRate].rssiThres + 2) &&
                        (pRc->state[txRate].rssiThres >
					pRateTable->info[txRate].rssiAckValidMin))
                    {
                        pRc->state[txRate].rssiThres--;
                    }

                    stateChange = TRUE;
                }
            }
        }
    }


	/* For all cases */

    // ASSERT((pRc->rateMaxPhy >= 0 && pRc->rateMaxPhy <= pRc->rateTableSize && pRc->rateMaxPhy != INVALID_RATE_MAX));

    /*
     * If this rate looks bad (high PER) then stop using it for
     * a while (except if we are probing).
     */
    if (pRc->state[txRate].per >= 55 && txRate > 0 &&
        pRateTable->info[txRate].rateKbps <=
				pRateTable->info[pRc->rateMaxPhy].rateKbps)
    {
        rcGetNextLowerValidTxRate(pRateTable, pRc, (A_UINT8) txRate,
								&pRc->rateMaxPhy);

        /* Don't probe for a little while. */
        pRc->probeTime = nowMsec;
    }

    if (stateChange) {
        /*
         * Make sure the rates above this have higher rssi thresholds.
         * (Note:  Monotonicity is kept within the OFDM rates and within the CCK rates.
         *         However, no adjustment is made to keep the rssi thresholds monotonically
         *         increasing between the CCK and OFDM rates.)
         */
        for (rate = txRate; rate < pRc->rateTableSize - 1; rate++) {
            if (pRateTable->info[rate+1].phy != pRateTable->info[txRate].phy) {
                break;
            }

            if (pRc->state[rate].rssiThres + pRateTable->info[rate].rssiAckDeltaMin >
                pRc->state[rate+1].rssiThres)
            {
                pRc->state[rate+1].rssiThres =
                    pRc->state[rate].rssiThres + pRateTable->info[rate].rssiAckDeltaMin;
            }
        }

        /* Make sure the rates below this have lower rssi thresholds. */
        for (rate = txRate - 1; rate >= 0; rate--) {
            if (pRateTable->info[rate].phy != pRateTable->info[txRate].phy) {
                break;
            }

            if (pRc->state[rate].rssiThres + pRateTable->info[rate].rssiAckDeltaMin >
                pRc->state[rate+1].rssiThres)
            {
                if (pRc->state[rate+1].rssiThres < pRateTable->info[rate].rssiAckDeltaMin) {
                    pRc->state[rate].rssiThres = 0;
                } else {
                    pRc->state[rate].rssiThres =
                        pRc->state[rate+1].rssiThres - pRateTable->info[rate].rssiAckDeltaMin;
                }

                if (pRc->state[rate].rssiThres < pRateTable->info[rate].rssiAckValidMin) {
                    pRc->state[rate].rssiThres = pRateTable->info[rate].rssiAckValidMin;
                }
            }
        }
    }

    /* Make sure the rates below this have lower PER */
    /* Monotonicity is kept only for rates below the current rate. */
    if (pRc->state[txRate].per < lastPer) {
        for (rate = txRate - 1; rate >= 0; rate--) {
            if (pRc->state[rate].per > pRc->state[rate+1].per) {
                pRc->state[rate].per = pRc->state[rate+1].per;
            }
        }
    }

    /* Maintain monotonicity for rates above the current rate*/
    for (rate = txRate; rate < pRc->rateTableSize - 1; rate++) {
        if (pRc->state[rate+1].per < pRc->state[rate].per) {
            pRc->state[rate+1].per = pRc->state[rate].per;
        }
    }

    /* Every so often, we reduce the thresholds and PER (different for CCK and OFDM). */
    if (nowMsec - pRc->rssiDownTime >= pRateTable->rssiReduceInterval) {

        for (rate = 0; rate < pRc->rateTableSize; rate++) {
            if (pRc->state[rate].rssiThres > pRateTable->info[rate].rssiAckValidMin) {
                pRc->state[rate].rssiThres -= 1;
            }
//            pRc->state[rate].per = 7*pRc->state[rate].per/8;
        }
        pRc->rssiDownTime = nowMsec;
    }

    /* Every so often, we reduce the thresholds and PER (different for CCK and OFDM). */
    if (nowMsec - pRc->perDownTime >= pRateTable->rssiReduceInterval) {
        for (rate = 0; rate < pRc->rateTableSize; rate++) {
            pRc->state[rate].per = 7*pRc->state[rate].per/8;
        }

        pRc->perDownTime = nowMsec;
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

/*
 * This routine is called by the Tx interrupt service routine to give
 * the status of previous frames.
 */
void
rcUpdate_11n(struct ath_softc *sc, struct ath_node *an, A_RSSI rssiAck, A_UINT8 ac,
               int finalTSIdx, int Xretries, struct ath_rc_series rcs[], int nFrames,
	       int  nBad, int long_retry)
{
    A_UINT32              series = 0;
    A_UINT32              rix;
    struct atheros_softc  *asc        = (struct atheros_softc*)sc->sc_rc;
    RATE_TABLE_11N            *pRateTable = (RATE_TABLE_11N *)asc->hwRateTable[sc->sc_curmode];
    struct atheros_node   *pSib       = ATH_NODE_ATHEROS(an);
    TX_RATE_CTRL   	  *pRc 	      = (TX_RATE_CTRL *)(pSib);
    A_UINT8               flags;


    if (!an) {
	// panic ("rcUpdate an is NULL");
        return;
    }

    ASSERT (rcs[0].tries != 0);

    /*
     * If the first rate is not the final index, there are intermediate rate failures
     * to be processed.
     */
    if (finalTSIdx != 0) {

	/* Process intermediate rates that failed.*/
      for (series = 0; series < finalTSIdx ; series++) {
        if (rcs[series].tries != 0) {
           flags = rcs[series].flags;
           /* If HT40 and we have switched mode from 40 to 20 => don't update */
	   if ((flags & ATH_RC_CW40_FLAG) && (pRc->rcPhyMode != (flags & ATH_RC_CW40_FLAG))) {
		return;
           }
           if ((flags & ATH_RC_CW40_FLAG) && (flags & ATH_RC_SGI_FLAG)) {
               rix = pRateTable->info[rcs[series].rix].htIndex;
           } else if (flags & ATH_RC_SGI_FLAG) {
               rix = pRateTable->info[rcs[series].rix].sgiIndex;
           } else if (flags & ATH_RC_CW40_FLAG) {
               rix = pRateTable->info[rcs[series].rix].cw40Index;
           } else {
               rix = pRateTable->info[rcs[series].rix].baseIndex;
           }
           /* FIXME:XXXX, too many args! */
           rcUpdate_ht(sc, an, ac, rix, Xretries? 1 : 2, rcs[series].tries, rssiAck,
                             nFrames, nFrames);
        }
      }
    } else {
        /*
         * Handle the special case of MIMO PS burst, where the second aggregate is sent
         *  out with only one rate and one try. Treating it as an excessive retry penalizes
         * the rate inordinately.
         */
        if (rcs[0].tries == 1 && Xretries == 1) {
            Xretries = 2;
        }
    }

    flags = rcs[series].flags;
    /* If HT40 and we have switched mode from 40 to 20 => don't update */
    if ((flags & ATH_RC_CW40_FLAG) && (pRc->rcPhyMode != (flags & ATH_RC_CW40_FLAG))) {
	 return;
    }
    if ((flags & ATH_RC_CW40_FLAG) && (flags & ATH_RC_SGI_FLAG)) {
	  rix = pRateTable->info[rcs[series].rix].htIndex;
    } else if (flags & ATH_RC_SGI_FLAG) {
 	  rix = pRateTable->info[rcs[series].rix].sgiIndex;
    } else if (flags & ATH_RC_CW40_FLAG) {
  	  rix = pRateTable->info[rcs[series].rix].cw40Index;
    } else {
	  rix = pRateTable->info[rcs[series].rix].baseIndex;
    }
    /* FIXME:XXXX, too many args! */
    rcUpdate_ht(sc, an, ac, rix, Xretries, long_retry, rssiAck, nFrames, nBad);
}

#if ATH_CCX
u_int8_t
rcRateValueToPer_11n(struct ath_softc *sc, struct ath_node *an, int txRateKbps)
{
    const RATE_TABLE_11N *pRateTable;
    u_int8_t             rate  = 0;
    u_int8_t             index = 0;
    struct atheros_node *oan = an->an_rc_node;
    struct TxRateCtrl_s *pRc = &oan->txRateCtrl;
    HAL_BOOL turboFlag = IS_CHAN_TURBO(&(sc->sc_curchan));

    pRateTable = sc->sc_rc->hwRateTable[sc->sc_curmode];
    if(pRateTable){
        while (rate < pRateTable->rateCount) {
            A_UINT32 valid;

            /* Check for single stream, to avoid dual stream rates for 
             * single stream device */
            if (oan->singleStream) {
                valid = pRateTable->info[rate].validSingleStream;
            } else if (oan->stbc) {
                valid = pRateTable->info[rate].validSTBC;
            } else {
                valid = pRateTable->info[rate].valid;
            }
            if (valid && txRateKbps == pRateTable->info[rate].rateKbps && 
                ((turboFlag && pRateTable->info[rate].phy == WLAN_PHY_TURBO) || 
                (!turboFlag && pRateTable->info[rate].phy != WLAN_PHY_TURBO ))) {

                index = rate;
                break;
            }
            rate++;
        }
	} else {
        return (100);
    }
    if (pRc == NULL || index >= MAX_TX_RATE_TBL) {
        return (100);
    } else {
        return (pRc->state[index].per);
    }
}
#endif
