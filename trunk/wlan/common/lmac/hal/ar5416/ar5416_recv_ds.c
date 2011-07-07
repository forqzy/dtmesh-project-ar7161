/*
 * Copyright (c) 2002-2005 Sam Leffler, Errno Consulting
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
#include "opt_ah.h"

#ifdef AH_SUPPORT_AR5416

#include "ah.h"
#include "ah_desc.h"
#include "ah_internal.h"

#include "ar5416/ar5416.h"
#include "ar5416/ar5416reg.h"
#include "ar5416/ar5416desc.h"

/*
 * Initialize RX descriptor, by clearing the status and setting
 * the size (and any other flags).
 */
HAL_BOOL
ar5416SetupRxDesc(struct ath_hal *ah, struct ath_desc *ds,
    u_int32_t size, u_int flags)
{
    struct ar5416_desc *ads = AR5416DESC(ds);
    HAL_CAPABILITIES *pCap = &AH_PRIVATE(ah)->ah_caps;

    HALASSERT((size &~ AR_BufLen) == 0);

    ads->ds_ctl1 = size & AR_BufLen;
    if (flags & HAL_RXDESC_INTREQ)
        ads->ds_ctl1 |= AR_RxIntrReq;

    /* this should be enough */
    ads->ds_rxstatus8 &= ~AR_RxDone;
    if (! pCap->halAutoSleepSupport) {
        /* If AutoSleep not supported, clear all fields. */
        OS_MEMZERO(&(ads->u), sizeof(ads->u));
    }
    return AH_TRUE;
}

/*
 * Process an RX descriptor, and return the status to the caller.
 * Copy some hardware specific items into the software portion
 * of the descriptor.
 *
 * NB: the caller is responsible for validating the memory contents
 *     of the descriptor (e.g. flushing any cached copy).
 */
HAL_STATUS
ar5416ProcRxDesc(struct ath_hal *ah, struct ath_desc *ds,
    u_int32_t pa, struct ath_desc *nds, u_int64_t tsf)
{
    struct ar5416_desc ads;
    struct ar5416_desc *adsp = AR5416DESC(ds);

    if ((adsp->ds_rxstatus8 & AR_RxDone) == 0) {
        ds->ds_rxstat.rs_status = HAL_RXERR_INCOMP;
        return HAL_EINPROGRESS;
    }

    /*
     * Now we need to get the stats from the descriptor. Since desc are
     * uncached, lets make a copy of the stats first. Note that, since we
     * touch most of the rx stats, a memcpy would always be more efficient
     *
     * Next we fill in all values in a caller passed stack variable.
     * This reduces the number of uncached accesses.
     * Do this copy here, after the check so that when the checks fail, we
     * dont end up copying the entire stats uselessly.
     */
    ads.u.rx = adsp->u.rx;

    ds->ds_rxstat.rs_status = 0;
    ds->ds_rxstat.rs_flags =  0;

    ds->ds_rxstat.rs_datalen = ads.ds_rxstatus1 & AR_DataLen;
    ds->ds_rxstat.rs_tstamp =  ads.AR_RcvTimestamp;

    /* If post delim CRC error happens,  */
    if (ads.ds_rxstatus8 & AR_PostDelimCRCErr) {
        ds->ds_rxstat.rs_rssi      = HAL_RSSI_BAD;
        ds->ds_rxstat.rs_rssi_ctl0 = HAL_RSSI_BAD;
        ds->ds_rxstat.rs_rssi_ctl1 = HAL_RSSI_BAD;
        ds->ds_rxstat.rs_rssi_ctl2 = HAL_RSSI_BAD;
        ds->ds_rxstat.rs_rssi_ext0 = HAL_RSSI_BAD;
        ds->ds_rxstat.rs_rssi_ext1 = HAL_RSSI_BAD;
        ds->ds_rxstat.rs_rssi_ext2 = HAL_RSSI_BAD;
    } else {
        ds->ds_rxstat.rs_rssi =
                                MS(ads.ds_rxstatus4, AR_RxRSSICombined);
        ds->ds_rxstat.rs_rssi_ctl0 = MS(ads.ds_rxstatus0, AR_RxRSSIAnt00);
        ds->ds_rxstat.rs_rssi_ctl1 = MS(ads.ds_rxstatus0, AR_RxRSSIAnt01);
        ds->ds_rxstat.rs_rssi_ctl2 = MS(ads.ds_rxstatus0, AR_RxRSSIAnt02);
        ds->ds_rxstat.rs_rssi_ext0 = MS(ads.ds_rxstatus4, AR_RxRSSIAnt10);
        ds->ds_rxstat.rs_rssi_ext1 = MS(ads.ds_rxstatus4, AR_RxRSSIAnt11);
        ds->ds_rxstat.rs_rssi_ext2 = MS(ads.ds_rxstatus4, AR_RxRSSIAnt12);
    }
    if (ads.ds_rxstatus8 & AR_RxKeyIdxValid)
        ds->ds_rxstat.rs_keyix = MS(ads.ds_rxstatus8, AR_KeyIdx);
    else
        ds->ds_rxstat.rs_keyix = HAL_RXKEYIX_INVALID;
    /* NB: caller expected to do rate table mapping */
    ds->ds_rxstat.rs_rate = RXSTATUS_RATE(ah, (&ads));
    ds->ds_rxstat.rs_more = (ads.ds_rxstatus1 & AR_RxMore) ? 1 : 0;

    ds->ds_rxstat.rs_isaggr = (ads.ds_rxstatus8 & AR_RxAggr) ? 1 : 0;
    ds->ds_rxstat.rs_moreaggr = (ads.ds_rxstatus8 & AR_RxMoreAggr) ? 1 : 0;
    ds->ds_rxstat.rs_antenna = MS(ads.ds_rxstatus3, AR_RxAntenna);
    ds->ds_rxstat.rs_flags  = (ads.ds_rxstatus3 & AR_GI) ? HAL_RX_GI : 0;
    ds->ds_rxstat.rs_flags  |= (ads.ds_rxstatus3 & AR_2040) ? HAL_RX_2040 : 0;

    if (ads.ds_rxstatus8 & AR_PreDelimCRCErr)
        ds->ds_rxstat.rs_flags |= HAL_RX_DELIM_CRC_PRE;
    if (ads.ds_rxstatus8 & AR_PostDelimCRCErr)
        ds->ds_rxstat.rs_flags |= HAL_RX_DELIM_CRC_POST;
    if (ads.ds_rxstatus8 & AR_DecryptBusyErr)
        ds->ds_rxstat.rs_flags |= HAL_RX_DECRYPT_BUSY;
    if (ads.ds_rxstatus8 & AR_HiRxChain) {
        ds->ds_rxstat.rs_flags |= HAL_RX_HI_RX_CHAIN;
    }

    if ((ads.ds_rxstatus8 & AR_RxFrameOK) == 0) {
        /*
         * These four bits should not be set together.  The
         * 5416 spec states a Michael error can only occur if
         * DecryptCRCErr not set (and TKIP is used).  Experience
         * indicates however that you can also get Michael errors
         * when a CRC error is detected, but these are specious.
         * Consequently we filter them out here so we don't
         * confuse and/or complicate drivers.
         */
        if (ads.ds_rxstatus8 & AR_CRCErr)
            ds->ds_rxstat.rs_status |= HAL_RXERR_CRC;
        else if (ads.ds_rxstatus8 & AR_PHYErr) {
            u_int phyerr;

            ds->ds_rxstat.rs_status |= HAL_RXERR_PHY;
            phyerr = MS(ads.ds_rxstatus8, AR_PHYErrCode);
            ds->ds_rxstat.rs_phyerr = phyerr;
#if 0
            if ((!AH5416(ah)->ah_hasHwPhyCounters) &&
                (phyerr != HAL_PHYERR_RADAR))
                ar5416AniPhyErrReport(ah, rx_stats);
            if (phyerr == HAL_PHYERR_RADAR)
                ar5416ProcessRadar(ah, ds, rx_stats);
#endif
        } else if (ads.ds_rxstatus8 & AR_DecryptCRCErr)
            ds->ds_rxstat.rs_status |= HAL_RXERR_DECRYPT;
        else if (ads.ds_rxstatus8 & AR_MichaelErr)
            ds->ds_rxstat.rs_status |= HAL_RXERR_MIC;
    }

    return HAL_OK;
}
#endif /* AH_SUPPORT_AR5416 */
