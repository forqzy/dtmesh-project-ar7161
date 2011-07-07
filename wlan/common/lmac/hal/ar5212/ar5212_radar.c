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

#ifdef AH_SUPPORT_AR5212
#include "ah.h"
#include "ah_internal.h"

#include "ar5212/ar5212.h"
#include "ar5212/ar5212reg.h"
#include "ar5212/ar5212desc.h"
#include "ar5212/ar5212phy.h"



#define HAL_RADAR_WAIT_TIME		60*1000000	/* 1 minute in usecs */

/* 
 * Default 5212/5312 radar phy parameters
 */
#define AR5212_DFS_FIRPWR	-41
#define AR5212_DFS_RRSSI	12
#define AR5212_DFS_HEIGHT	20
#define AR5212_DFS_PRSSI	22
#define AR5212_DFS_INBAND	6

struct dfs_pulse ar5212_etsi_radars[] = {
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
struct dfs_pulse ar5212_fcc_radars[] = {
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

struct dfs_bin5pulse ar5212_bin5pulses[] = {
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
ar5212CheckDfs(struct ath_hal *ah, HAL_CHANNEL *chan)
{
	HAL_CHANNEL_INTERNAL *ichan=AH_NULL;
	u_int64_t tsf;

	ichan = getchannel(ah, chan);
	if (ichan == AH_NULL)
		return;
	if (!(ichan->privFlags & CHANNEL_INTERFERENCE))
		return;
	tsf = ar5212GetTsf64(ah);
	if (tsf >= ichan->dfsTsf) {
		ichan->privFlags &= ~CHANNEL_INTERFERENCE;
		ichan->dfsTsf = 0;
		chan->privFlags &= ~CHANNEL_INTERFERENCE;
	}
}

/*
 * This function marks the channel as having found a dfs event
 * It also marks the end time that the dfs event should be cleared
 * If the channel is already marked, then tsf end time can only
 * be increased
 */

void
ar5212DfsFound(struct ath_hal *ah, HAL_CHANNEL *chan, u_int64_t nolTime)
{
	HAL_CHANNEL_INTERNAL *ichan;

	ichan = getchannel(ah, chan);
	if (ichan == AH_NULL)
		return;
	if (!(ichan->privFlags & CHANNEL_INTERFERENCE))
		ichan->dfsTsf = ar5212GetTsf64(ah);
	ichan->dfsTsf += nolTime;
	ichan->privFlags |= CHANNEL_INTERFERENCE;
	chan->privFlags |= CHANNEL_INTERFERENCE;
}


void
ar5212EnableDfs(struct ath_hal *ah, HAL_PHYERR_PARAM *pe)
{
	u_int32_t val;
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
	OS_REG_WRITE(ah, AR_PHY_RADAR_0, val | AR_PHY_RADAR_0_ENA);

}

void
ar5212GetDfsThresh(struct ath_hal *ah, HAL_PHYERR_PARAM *pe)
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

	pe->pe_relpwr = 0;
	pe->pe_relstep = 0;
	pe->pe_maxlen = 0;
}

HAL_BOOL
ar5212RadarWait(struct ath_hal *ah, HAL_CHANNEL *chan)
{
	struct ath_hal_private *ahp= AH_PRIVATE(ah);
#ifdef AH_SUPPORT_DFS
	u_int64_t tsf;

	if(!ahp->ah_curchan) return AH_TRUE;
	tsf = ar5212GetTsf64(ah);
	ahp->ah_curchan->ah_channel_time  += (tsf - ahp->ah_curchan->ah_tsf_last);
	ahp->ah_curchan->ah_tsf_last = tsf;
	chan->channel = ahp->ah_curchan->channel;
	chan->channelFlags = ahp->ah_curchan->channelFlags;
	chan->privFlags  = ahp->ah_curchan->privFlags;
	chan->maxRegTxPower = ahp->ah_curchan->maxRegTxPower;
	if (ahp->ah_curchan->ah_channel_time > (HAL_RADAR_WAIT_TIME)) {
		ahp->ah_curchan->privFlags |= CHANNEL_DFS_CLEAR;
		chan->privFlags  = ahp->ah_curchan->privFlags;
		ar5212TxEnable(ah,AH_TRUE);
		return AH_FALSE;
	} 
	return AH_TRUE;
#else
	chan->channel = ahp->ah_curchan->channel;
	ahp->ah_curchan->privFlags |= CHANNEL_DFS_CLEAR;
	chan->privFlags |= CHANNEL_DFS_CLEAR;
	chan->channelFlags = ahp->ah_curchan->channelFlags;
	chan->privFlags  = ahp->ah_curchan->privFlags;
	chan->maxRegTxPower = ahp->ah_curchan->maxRegTxPower;
	return AH_FALSE;
#endif /* AH_SUPPORT_DFS */
}

struct dfs_pulse *
ar5212GetDfsRadars(struct ath_hal *ah, u_int32_t dfsdomain, int *numradars,
               struct dfs_bin5pulse **bin5pulses, int *numb5radars, HAL_PHYERR_PARAM *pe)
{
#define N(a)    (sizeof(a)/sizeof(a[0]))
        struct dfs_pulse *dfs_radars = AH_NULL;

        switch (dfsdomain) {
        case DFS_FCC_DOMAIN:
                        dfs_radars = &ar5212_fcc_radars[2];
                        *numradars= N(ar5212_fcc_radars)-2;
                        *bin5pulses = &ar5212_bin5pulses[0];
                        *numb5radars = N(ar5212_bin5pulses);
                         HDPRINTF(ah, HAL_DBG_DFS, "%s: DFS_FCC_DOMAIN_5212\n", __func__);
                break;
        case DFS_ETSI_DOMAIN:
                        dfs_radars = &ar5212_etsi_radars[0];
                        *numradars = N(ar5212_etsi_radars);
                        *bin5pulses = &ar5212_bin5pulses[0];
                        *numb5radars = N(ar5212_bin5pulses);
                         HDPRINTF(ah, HAL_DBG_DFS, "%s: DFS_ETSI_DOMAIN_5212\n", __func__);
                         break;
        case DFS_MKK4_DOMAIN:
                        dfs_radars = &ar5212_fcc_radars[0];
                        *numradars = N(ar5212_fcc_radars);
                        *bin5pulses = &ar5212_bin5pulses[0];
                        *numb5radars = N(ar5212_bin5pulses);
                         HDPRINTF(ah, HAL_DBG_DFS, "%s: DFS_MKK4_DOMAIN_5212\n", __func__);
                        break;
        default:
                HDPRINTF(ah, HAL_DBG_DFS, "%s: no domain\n", __func__);
                return AH_NULL;
        }
        /* Set the default phy parameters per chip */
                pe->pe_firpwr = AR5212_DFS_FIRPWR;
                pe->pe_rrssi = AR5212_DFS_RRSSI;
                pe->pe_height = AR5212_DFS_HEIGHT;
                pe->pe_prssi = AR5212_DFS_PRSSI;
                pe->pe_inband = AR5212_DFS_INBAND;

                return dfs_radars;
#undef N
}

HAL_CHANNEL *ar5212GetExtensionChannel(struct ath_hal *ah)
{
    return AH_NULL;
}

HAL_BOOL ar5212IsFastClockEnabled(struct ath_hal *ah)
{
    return AH_FALSE;
}


#endif /* AH_SUPPORT_AR5212 */

