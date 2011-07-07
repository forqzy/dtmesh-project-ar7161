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

#include "opt_ah.h"

#ifdef AH_SUPPORT_AR5416

#include "ah.h"
#include "ah_desc.h"
#include "ah_internal.h"

#include "ar5416/ar5416phy.h"
#include "ar5416/ar5416.h"
#include "ar5416/ar5416reg.h"
#include "ar5416/ar5416desc.h"

#define HAL_RADAR_WAIT_TIME		60*1000000	/* 1 minute in usecs */
/*
 * Default 5413/5416 radar phy parameters
 */
#define AR5416_DFS_FIRPWR	-33
#define AR5416_DFS_RRSSI	20
#define AR5416_DFS_HEIGHT	10
#define AR5416_DFS_PRSSI	15
#define AR5416_DFS_INBAND	15
#define AR5416_DFS_RELPWR   8
#define AR5416_DFS_RELSTEP  12
#define AR5416_DFS_MAXLEN   255

struct dfs_pulse ar5416_etsi_radars[] = {
        /* TYPE 1 */
        {15, 2,   750,  750, 0,  24, 7,  0,  2, 21,  0, -10, 0},
        /* TYPE 2 */
        {10,  5,   200, 200,   0,  24, 5,  1,  9, 21,  0, 3, 1},
        {10,  5,   300, 300,  0,  24, 8,  1,  9, 21,  0, 3, 2},
        {10,  5,   500, 500,  0,  24, 8,  1,  9, 21,  1, 3, 3},
        {10,  5,   800, 800,  0,  24, 8,  1,  9, 21,  1, 3, 4},
        {10,  5,  1001, 1001,  0,  30, 8,  1,  9, 21,  0, 3, 5},
        /* TYPE 3 */
        {15, 16,  200,   200, 0,  24, 6, 10, 19, 22, 0, 3, 6},
        {15, 16,  300,   300, 0,  24, 6, 10, 19, 22, 0, 3, 7},
        {15, 16,  503,   503, 0,  24, 7, 10, 19, 22, 0, 3, 8},
        {15, 16,  809,   809, 0,  24, 7, 10, 19, 22, 0, 3, 9},
        {15, 16, 1014,   1014, 0,  30, 7, 10, 19, 22, 0, 3, 10},
        /* TYPE 4 */
        {15, 5,  1200,   1200, 0,  24, 7,  1,  9, 21,  0, 3, 11},
        {15, 5,  1500,   1500, 0,  30, 7,  1,  9, 21,  0, 3, 12},
        {15, 5,  1600,   1600, 0,  24, 7,  1,  9, 21,  0, 3, 13},
        {15, 16,  1200,  1200, 0,  30, 7, 10,  19, 22, 0, 3, 14},
        {15, 16,  1500,  1500, 0,  24, 7, 10,  19, 22, 0, 3, 15},
        {15, 16,  1600,  1600, 0,  24, 7, 10,  19, 22, 0, 3, 16},
        /* TYPE 5 */
        {25, 5,  2305,   2305, 0,  24, 12,  1,  9, 21,  0, 3, 17},
        {25, 5,  3009,   3009, 0,  24, 12,  1,  9, 21,  0, 3, 18},
        {25, 5,  3500,   3500, 0,  24, 12,  1,  9, 21,  0, 3, 19},
        {25, 5,  4000,   4000, 0,  24, 12,  1,  9, 21,  0, 3, 20},
        {25, 16, 2300,   2300, 0,  24, 12, 10, 20, 22,  0, 3, 21},
        {25, 16, 3000,   3000, 0,  24, 12, 10, 20, 22,  0, 3, 22},
        {25, 16, 3500,   3500, 0,  24, 12, 10, 20, 22,  0, 3, 23},
        {25, 16, 3850,   3850, 0,  24, 12, 10, 20, 22,  0, 3, 24},
        /* TYPE 6 */
        {20, 25, 2000,   2000, 0,  24, 10, 20, 26, 22,  0, 3, 25},
        {20, 25, 3000,   3000, 0,  24, 10, 20, 26, 22,  0, 3, 26},
        {20, 25, 4000,   4000, 0,  24, 10, 20, 26, 22,  0, 3, 27},
        {20, 37, 2000,   2000, 0,  24, 10, 30, 36, 22,  0, 3, 28},
        {20, 37, 3000,   3000, 0,  24, 10, 30, 36, 22,  0, 3, 29},
        {20, 37, 4000,   4000, 0,  24, 10, 30, 36, 22,  0, 3, 30},
        
        /* TYPE staggered pulse */
        {20, 2, 300,    400, 2,  30, 10, 0, 2, 22,  0, 3, 31}, //0.8-2us, 2-3 bursts,300-400 PRF, 10 pulses each 
        {30, 2, 400,   1200, 2,  30, 15, 0, 2, 22,  0, 3, 32}, //0.8-2us, 2-3 bursts, 400-1200 PRF, 15 pulses each

        /* constant PRF based */ 
        {10, 5,   200,  1000, 0,  24, 6,  0,  8, 21,  0, -10, 33}, /*0.8-5us , 200-1000 PRF, 10 pulses */
        {15, 15,   200,  1600, 0,  24, 7,  0,  18, 21,  0, -10, 34}, /*0.8-15us , 200-1600 PRF, 15 pulses */
        {25, 15,   2300,  4000, 0,  24, 12,  0,  18, 21,  0, -10, 35}, /* 0.8-15 us, 2300-4000 PRF, 25 pulses*/
        {20, 30,   2000,  4000, 0,  24, 10,  19,  33, 21,  0, -10, 36}, /* 20-30us, 2000-4000 PRF, 20 pulses*/

};

/* The following are for FCC Bin 1-4 pulses */
struct dfs_pulse ar5416_fcc_radars[] = {
        /* following two filters are specific to Japan/MKK4 */
        {18,   1,  720,  720, 1,  6,   6,  0,  1, 18,  0, 3, 17}, // 1389 +/- 6 us
        {18,   4,  250,  250, 1,  10,  5,  1,  6, 18,  0, 3, 18}, // 4000 +/- 6 us
        {18,   5,  260,  260, 1,  10,  6,  1,  6, 18,  0, 3, 19}, // 3846 +/- 7 us
        /* following filters are common to both FCC and JAPAN */
        {18,  1,  700,   700, 1,  6,  7,  0,  1, 18,  0, 3,  0}, // 1428 +/- 7 us
        {9,   1, 3003,   3003, 1,  7,  5,  0,  1, 18,  0, 0,  1}, // 333 +/- 7 us
       
        {23,  5, 6250,   6250, 0, 15, 16,  0,  7, 22,  0, 3,  2}, // 160 +/- 15 us
        {23,  5, 5263,   5263, 0, 18, 11,  0,  7, 22,  0, 3,  3}, // 190 +/- 15 us
        {23,  5, 4545,   4545, 0, 18, 11,  0,  7, 22,  0, 3,  4}, // 220 +/- 15 us
        
        {18, 10, 4444,   4444, 0, 35,  6,  7, 13, 22,  0, 3,  5}, // 225 +/- 30 us
        {18, 10, 3636,   3636, 0, 25,  6,  7, 13, 22,  0, 3,  6}, // 275 +/- 25 us
        {18, 10, 3076,   3076, 0, 25,  8,  7, 13, 22,  0, 3,  7}, // 325 +/- 25 us
        {18, 10, 2666,   2666, 0, 25,  8,  7, 13, 22,  0, 3,  8}, // 375 +/- 25 us
        {18, 10, 2352,   2352, 0, 25,  8,  7, 13, 22,  0, 3,  9}, // 425 +/- 25 us
        {18, 10, 2105,   2105, 0, 30,  8,  7, 13, 22,  0, 3, 10}, // 475 +/- 30 us
       
        {14, 15, 4444,   4444, 0, 35,  5, 13, 21, 22,  0, 3, 11}, // 225 +/- 30 us
        {14, 15, 3636,   3636, 0, 25,  5, 13, 24, 22,  0, 3, 12}, // 275 +/- 25 us
        {14, 15, 3076,   3076, 0, 25,  7, 13, 23, 22,  0, 3, 13}, // 325 +/- 25 us
        {14, 15, 2666,   2666, 0, 25,  7, 13, 23, 22,  0, 3, 14}, // 375 +/- 25 us
        {14, 15, 2352,   2352, 0, 25,  7, 13, 21, 22,  0, 3, 15}, // 425 +/- 25 us
        {12, 15, 2105,   2105, 0, 30,  7, 13, 21, 22,  0, 3, 16}, // 475 +/- 30 us
};

struct dfs_bin5pulse ar5416_bin5pulses[] = {
        {4, 40, 105, 12, 22, 5},
};


/*
 * Find the internal HAL channel corresponding to the
 * public HAL channel specified in c
 */

static HAL_CHANNEL_INTERNAL *
getchannel(struct ath_hal *ah, const HAL_CHANNEL *c)
{
#define CHAN_FLAGS	(CHANNEL_ALL|CHANNEL_HALF|CHANNEL_QUARTER)
    HAL_CHANNEL_INTERNAL *base, *cc;
    int flags = c->channelFlags & CHAN_FLAGS;
    int n, lim;

    /*
     * Check current channel to avoid the lookup.
     */
    cc = AH_PRIVATE(ah)->ah_curchan;
    if (cc != AH_NULL && cc->channel == c->channel &&
        (cc->channelFlags & CHAN_FLAGS) == flags) {
        return cc;
    }

    /* binary search based on known sorting order */
    base = AH_PRIVATE(ah)->ah_channels;
    n = AH_PRIVATE(ah)->ah_nchan;
    /* binary search based on known sorting order */
    for (lim = n; lim != 0; lim >>= 1) {
        int d;
        cc = &base[lim>>1];
        d = c->channel - cc->channel;
        if (d == 0) {
            if ((cc->channelFlags & CHAN_FLAGS) == flags) {
                return cc;
            }
            d = flags - (cc->channelFlags & CHAN_FLAGS);
        }
        HDPRINTF(ah, HAL_DBG_DFS, "%s: channel %u/0x%x d %d\n", __func__,
                cc->channel, cc->channelFlags, d);
        if (d > 0) {
            base = cc + 1;
            lim--;
        }
    }
    HDPRINTF(ah, HAL_DBG_DFS, "%s: no match for %u/0x%x\n",
            __func__, c->channel, c->channelFlags);
    return AH_NULL;
#undef CHAN_FLAGS
}

/* Check the internal channel list to see if the desired channel
 * is ok to release from the NOL.  If not, then do nothing.  If so,
 * mark the channel as clear and reset the internal tsf time
 */

void
ar5416CheckDfs(struct ath_hal *ah, HAL_CHANNEL *chan)
{
    HAL_CHANNEL_INTERNAL *ichan=AH_NULL;

    ichan = getchannel(ah, chan);
    if (ichan == AH_NULL) {
        return;
    }
    if (!(ichan->privFlags & CHANNEL_INTERFERENCE)) {
        return;
    }

    ichan->privFlags &= ~CHANNEL_INTERFERENCE;
    ichan->dfsTsf = 0;
}

/*
 * This function marks the channel as having found a dfs event
 * It also marks the end time that the dfs event should be cleared
 * If the channel is already marked, then tsf end time can only
 * be increased
 */

void
ar5416DfsFound(struct ath_hal *ah, HAL_CHANNEL *chan, u_int64_t nolTime)
{
    HAL_CHANNEL_INTERNAL *ichan;

    ichan = getchannel(ah, chan);
    if (ichan == AH_NULL) {
        return;
    }
    if (!(ichan->privFlags & CHANNEL_INTERFERENCE)) {
        ichan->dfsTsf = ar5416GetTsf64(ah);
    }
    ichan->dfsTsf += nolTime;
    ichan->privFlags |= CHANNEL_INTERFERENCE;
    chan->privFlags |= CHANNEL_INTERFERENCE;
}

/*
 * Enable radar detection and set the radar parameters per the
 * values in pe
 */

void
ar5416EnableDfs(struct ath_hal *ah, HAL_PHYERR_PARAM *pe)
{
    u_int32_t val;
    struct ath_hal_private  *ahp = AH_PRIVATE(ah);
    HAL_CHANNEL_INTERNAL *ichan=ahp->ah_curchan;

    val = OS_REG_READ(ah, AR_PHY_RADAR_0);
    if (pe->pe_firpwr != HAL_PHYERR_PARAM_NOVAL) {
        val &= ~AR_PHY_RADAR_0_FIRPWR;
        val |= SM(pe->pe_firpwr, AR_PHY_RADAR_0_FIRPWR);
    }
    if (pe->pe_rrssi != HAL_PHYERR_PARAM_NOVAL) {
        val &= ~AR_PHY_RADAR_0_RRSSI;
        val |= SM(pe->pe_rrssi, AR_PHY_RADAR_0_RRSSI);
    }
    if (pe->pe_height != HAL_PHYERR_PARAM_NOVAL) {
        val &= ~AR_PHY_RADAR_0_HEIGHT;
        val |= SM(pe->pe_height, AR_PHY_RADAR_0_HEIGHT);
    }
    if (pe->pe_prssi != HAL_PHYERR_PARAM_NOVAL) {
        val &= ~AR_PHY_RADAR_0_PRSSI;
        val |= SM(pe->pe_prssi, AR_PHY_RADAR_0_PRSSI);
    }
    if (pe->pe_inband != HAL_PHYERR_PARAM_NOVAL) {
        val &= ~AR_PHY_RADAR_0_INBAND;
        val |= SM(pe->pe_inband, AR_PHY_RADAR_0_INBAND);
    }
    
    /*Enable FFT data*/
    val |= AR_PHY_RADAR_0_FFT_ENA;

    OS_REG_WRITE(ah, AR_PHY_RADAR_0, val | AR_PHY_RADAR_0_ENA);
    
    val = OS_REG_READ(ah, AR_PHY_RADAR_1);
    val |=( AR_PHY_RADAR_1_MAX_RRSSI |
            AR_PHY_RADAR_1_BLOCK_CHECK );

    if (pe->pe_maxlen != HAL_PHYERR_PARAM_NOVAL) {
        val &= ~AR_PHY_RADAR_1_MAXLEN;
        val |= SM(pe->pe_maxlen, AR_PHY_RADAR_1_MAXLEN);
    }

    OS_REG_WRITE(ah, AR_PHY_RADAR_1, val);
            
    if (ath_hal_getcapability(ah, HAL_CAP_EXT_CHAN_DFS, 0, 0) == HAL_OK) {
        if (IS_CHAN_HT40(ichan)) {
            /*Enable extension channel radar detection*/
            val = OS_REG_READ(ah, AR_PHY_RADAR_EXT);
            OS_REG_WRITE(ah, AR_PHY_RADAR_EXT, val | AR_PHY_RADAR_EXT_ENA);
        } else {
            /*HT20 mode, disable extension channel radar detect*/
            val = OS_REG_READ(ah, AR_PHY_RADAR_EXT);
            OS_REG_WRITE(ah, AR_PHY_RADAR_EXT, val & ~AR_PHY_RADAR_EXT_ENA);
        }
    }

    if (pe->pe_relstep != HAL_PHYERR_PARAM_NOVAL) {
        val = OS_REG_READ(ah, AR_PHY_RADAR_1);
        val &= ~AR_PHY_RADAR_1_RELSTEP_THRESH;
        val |= SM(pe->pe_relstep, AR_PHY_RADAR_1_RELSTEP_THRESH);
        OS_REG_WRITE(ah, AR_PHY_RADAR_1, val);
    }
    if (pe->pe_relpwr != HAL_PHYERR_PARAM_NOVAL) {
        val = OS_REG_READ(ah, AR_PHY_RADAR_1);
        val &= ~AR_PHY_RADAR_1_RELPWR_THRESH;
        val |= SM(pe->pe_relpwr, AR_PHY_RADAR_1_RELPWR_THRESH);
        OS_REG_WRITE(ah, AR_PHY_RADAR_1, val);
    }
}

/*
 * Get the radar parameter values and return them in the pe
 * structure
 */

void
ar5416GetDfsThresh(struct ath_hal *ah, HAL_PHYERR_PARAM *pe)
{
    u_int32_t val,temp;
    
    val = OS_REG_READ(ah, AR_PHY_RADAR_0);

    temp = MS(val,AR_PHY_RADAR_0_FIRPWR);
    temp |= 0xFFFFFF80;
    pe->pe_firpwr = temp;
    pe->pe_rrssi = MS(val, AR_PHY_RADAR_0_RRSSI);
    pe->pe_height =  MS(val, AR_PHY_RADAR_0_HEIGHT);
    pe->pe_prssi = MS(val, AR_PHY_RADAR_0_PRSSI);
    pe->pe_inband = MS(val, AR_PHY_RADAR_0_INBAND);

    val = OS_REG_READ(ah, AR_PHY_RADAR_1);
    temp = val & AR_PHY_RADAR_1_RELPWR_ENA;
    pe->pe_relpwr = MS(val, AR_PHY_RADAR_1_RELPWR_THRESH);
    if (temp)
        pe->pe_relpwr |= HAL_PHYERR_PARAM_ENABLE;
    temp = val & AR_PHY_RADAR_1_RELSTEP_CHECK;
    pe->pe_relstep = MS(val, AR_PHY_RADAR_1_RELSTEP_THRESH);
    if (temp)
        pe->pe_relstep |= HAL_PHYERR_PARAM_ENABLE;
    pe->pe_maxlen = MS(val, AR_PHY_RADAR_1_MAXLEN);
}

HAL_BOOL
ar5416RadarWait(struct ath_hal *ah, HAL_CHANNEL *chan)
{
    struct ath_hal_private *ahp= AH_PRIVATE(ah);
    u_int64_t tsf;

    if(!ahp->ah_curchan) {
        return AH_TRUE;
    }
    tsf = ar5416GetTsf64(ah);
    /* WAR: appears tsf is being set after ah_tsf_last
     * has been set in reset function. */
    if (tsf > ahp->ah_curchan->ah_tsf_last) {
        ahp->ah_curchan->ah_channel_time  += (tsf - ahp->ah_curchan->ah_tsf_last);
    }
    ahp->ah_curchan->ah_tsf_last = tsf;
    chan->channel = ahp->ah_curchan->channel;
    chan->channelFlags = ahp->ah_curchan->channelFlags;
    chan->privFlags  = ahp->ah_curchan->privFlags;
    chan->maxRegTxPower = ahp->ah_curchan->maxRegTxPower;
    if (ahp->ah_curchan->ah_channel_time > (HAL_RADAR_WAIT_TIME)) {
        ahp->ah_curchan->privFlags |= CHANNEL_DFS_CLEAR;
        chan->privFlags  = ahp->ah_curchan->privFlags;
        return AH_FALSE;
    }
    return AH_TRUE;
}

struct dfs_pulse *
ar5416GetDfsRadars(struct ath_hal *ah, u_int32_t dfsdomain, int *numradars,
               struct dfs_bin5pulse **bin5pulses, int *numb5radars, HAL_PHYERR_PARAM *pe)
{
#define N(a)    (sizeof(a)/sizeof(a[0]))
        struct dfs_pulse *dfs_radars = AH_NULL;
        switch (dfsdomain) {
        case DFS_FCC_DOMAIN:
                        dfs_radars = &ar5416_fcc_radars[3];
                        *numradars= N(ar5416_fcc_radars)-3;
                        *bin5pulses = &ar5416_bin5pulses[0];
                        *numb5radars = N(ar5416_bin5pulses);
                         HDPRINTF(ah, HAL_DBG_DFS, "%s: DFS_FCC_DOMAIN_5416\n", __func__);
                break;
        case DFS_ETSI_DOMAIN:
                        dfs_radars = &ar5416_etsi_radars[0];
                        *numradars = N(ar5416_etsi_radars);
                        *bin5pulses = &ar5416_bin5pulses[0];
                        *numb5radars = N(ar5416_bin5pulses);
                         HDPRINTF(ah, HAL_DBG_DFS, "%s: DFS_ETSI_DOMAIN_5416\n", __func__);
                         break;
        case DFS_MKK4_DOMAIN:
                        dfs_radars = &ar5416_fcc_radars[0];
                        *numradars = N(ar5416_fcc_radars);
                        *bin5pulses = &ar5416_bin5pulses[0];
                        *numb5radars = N(ar5416_bin5pulses);
                         HDPRINTF(ah, HAL_DBG_DFS, "%s: DFS_MKK4_DOMAIN_5416\n", __func__);
                        break;
        default:
                HDPRINTF(ah, HAL_DBG_DFS, "%s: no domain\n", __func__);
                return AH_NULL;
        }
        /* Set the default phy parameters per chip */
                pe->pe_firpwr = AR5416_DFS_FIRPWR;
                pe->pe_rrssi = AR5416_DFS_RRSSI;
                pe->pe_height = AR5416_DFS_HEIGHT;
                pe->pe_prssi = AR5416_DFS_PRSSI;
                pe->pe_inband = AR5416_DFS_INBAND;
                pe->pe_relpwr = AR5416_DFS_RELPWR;
                pe->pe_relstep = AR5416_DFS_RELSTEP;
                pe->pe_maxlen = AR5416_DFS_MAXLEN;
                return dfs_radars;
#undef N
}

HAL_CHANNEL *ar5416GetExtensionChannel(struct ath_hal *ah)
{
    struct ath_hal_private  *ahp = AH_PRIVATE(ah);
    int i=0;

    HAL_CHANNEL_INTERNAL *ichan=AH_NULL;
    CHAN_CENTERS centers;

    ichan = ahp->ah_curchan;
    ar5416GetChannelCenters(ah, ichan, &centers);
    if (centers.ctl_center == centers.ext_center) {
        return AH_NULL;
    }
    for (i = 0; i < ahp->ah_nchan; i++) {
        ichan = &ahp->ah_channels[i];
        if (ichan->channel == centers.ext_center) {
            return (HAL_CHANNEL*)ichan;
        }
    }
    return AH_NULL;
}


HAL_BOOL ar5416IsFastClockEnabled(struct ath_hal *ah)
{
    struct ath_hal_private *ahp= AH_PRIVATE(ah);

    if (AR_SREV_MERLIN_20(ah) && IS_5GHZ_FAST_CLOCK_EN(ah, ahp->ah_curchan)) {
        return AH_TRUE;
    }
    return AH_FALSE;
}
#endif
