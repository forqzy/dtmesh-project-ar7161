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

#include <osdep.h>
#include "sys/queue.h"
#ifdef __NetBSD__
#include <net/if_media.h>
#else
#include <net80211/if_media.h>
#endif

#ifdef UMAC
#include "ieee80211_var.h"
#else
#include <net80211/ieee80211_var.h>
#endif

#include <ath_dev.h>

#include "ath_internal.h"
#include "if_athioctl.h"
#include "if_athvar.h"
#include "ah.h"
#include "ah_desc.h"
#include "dfs.h"
#include "dfs_ioctl.h"

static	u_int32_t _round(int32_t val);
static	void dfs_print_delayline(struct ath_softc *sc, struct dfs_delayline *dl);
static void dfs_print_nol(struct ath_softc *sc);
void dfs_print_filters(struct ath_softc *sc);

static	void dfs_reset_alldelaylines(struct ath_softc *sc);
static	void dfs_reset_delayline(struct dfs_delayline *dl);
void dfs_reset_filter_delaylines(struct dfs_filtertype *dft);
static	void dfs_reset_radarq(struct ath_softc *sc);

static  void  dfs_add_pulse(struct ath_softc *sc, struct dfs_filter *rf,
                           struct dfs_event *re, u_int32_t deltaT);

static  int dfs_bin_fixedpattern_check(struct ath_softc *sc, struct dfs_filter *rf, u_int32_t dur);
static int dfs_bin_check(struct ath_softc *sc, struct dfs_filter *rf,
                            u_int32_t deltaT, u_int32_t dur);
static  int  dfs_bin5_addpulse(struct ath_softc *sc, struct dfs_bin5radars *br,
                               struct dfs_event *re, u_int64_t thists);
static	int dfs_bin5_check(struct ath_softc *sc);

static	void dfs_reset_ar(struct ath_softc *sc);
static	void dfs_reset_arq(struct ath_softc *sc);
static	int dfs_set_thresholds(struct ath_softc *sc, const u_int32_t threshtype,
			       const u_int32_t value);
static void dfs_nol_addchan(struct ath_softc *sc, HAL_CHANNEL *chan);

static  int dfs_bin_pri_check(struct ath_softc *sc, struct dfs_filter *rf,
                             struct dfs_delayline *dl, u_int32_t score,
                             u_int32_t refpri, u_int32_t refdur);
static  int dfs_staggered_check(struct ath_softc *sc, struct dfs_filter *rf,
                             u_int32_t deltaT, u_int32_t width);

static char debug_dup[33];
static int debug_dup_cnt;
static int domainoverride=DFS_UNINIT_DOMAIN;
static int usenol=1;
static u_int32_t dfs_debug_level=ATH_DEBUG_DFS;

#define DFS_DPRINTK(sc, _m, _fmt, ...) do {             \
    if ((_m) <= dfs_debug_level) {               \
        printk(_fmt, __VA_ARGS__);  \
    }        \
} while (0)

#define DFS_MIN(a,b) ((a)<(b)?(a):(b))
#define DFS_MAX(a,b) ((a)>(b)?(a):(b))
#define DFS_DIFF(a,b) (DFS_MAX(a,b) - DFS_MIN(a,b))
/*
 * Maximum number of radar events to be processed in a single iteration.
 * Allows soft watchdog to run.
 */
#define MAX_EVENTS 100

/* Constants to use for chirping detection */
#define MIN_BIN5_DUR  50 /* 40 * 1.25*/
#define MAYBE_BIN5_DUR 37 /* 30 * 1.25*/

#define DFS_MARGIN_EQUAL(a, b, margin) ((DFS_DIFF(a,b)) <= margin)
#define DFS_MAX_STAGGERED_BURSTS 3

static int is_pri_multiple(u_int32_t sample_pri, u_int32_t refpri)
{
#define MAX_ALLOWED_MISSED 3
    int i;

    if (sample_pri < refpri || (!refpri)) 
        return 0;

    for (i=1; i<= MAX_ALLOWED_MISSED; i++) {
        if((sample_pri%(i*refpri) <= 5)) {
            //printk("sample_pri=%d is a multiple of refpri=%d\n", sample_pri, refpri);
            return 1;
        }
    }
    return 0;
#undef MAX_ALLOWED_MISSED
}
static int is_unique_pri(u_int32_t highestpri , u_int32_t midpri, 
        u_int32_t lowestpri , u_int32_t refpri ) 
{
#define DFS_STAGGERED_PRI_MARGIN_MIN  20
#define DFS_STAGGERED_PRI_MARGIN_MAX  400
    if ((DFS_DIFF(lowestpri, refpri) >= DFS_STAGGERED_PRI_MARGIN_MIN) &&
       (DFS_DIFF(midpri, refpri) >= DFS_STAGGERED_PRI_MARGIN_MIN) &&
       (DFS_DIFF(highestpri, refpri) >= DFS_STAGGERED_PRI_MARGIN_MIN)) {
        return 1;
    } else {
        if ((is_pri_multiple(refpri, highestpri)) || (is_pri_multiple(refpri, lowestpri)) ||
           (is_pri_multiple(refpri, midpri))) 
        return 0;
    }
    return 0;
#undef DFS_STAGGERED_PRI_MARGIN_MIN
#undef DFS_STAGGERED_PRI_MARGIN_MAX
}

static u_int32_t
_round(int32_t val)
{
	u_int32_t ival,rem;

	if (val < 0)
		return 0;
	ival = val/100;
	rem = val-(ival*100);
	if (rem <50)
		return ival;
	else
		return(ival+1);
}

static  int dfs_bin_fixedpattern_check(struct ath_softc *sc, struct dfs_filter *rf, u_int32_t dur)
{
        struct ath_dfs *dfs=sc->sc_dfs;
        struct dfs_pulseline *pl = dfs->pulses;
        int i, n, refpri, primargin, numpulses=0;
        u_int64_t start_ts, end_ts, event_ts, prev_event_ts, next_event_ts, window_start, window_end;
        u_int32_t index, next_index, deltadur;
        
        primargin = 6;
        refpri = (rf->rf_minpri + rf->rf_maxpri)/2;
        index = pl->pl_lastelem;
        end_ts = pl->pl_elems[index].p_time;
        start_ts = end_ts - (refpri*rf->rf_numpulses);
        
        DFS_DPRINTK(sc, ATH_DEBUG_DFS3, "lastelem ts=%llu start_ts=%llu, end_ts=%llu\n", (unsigned long long)pl->pl_elems[index].p_time, (unsigned long long)start_ts, (unsigned long long)end_ts);
        /* find the index of first element in our window of interest */
        for(i=0;i<pl->pl_numelems;i++) {
                index = (index-1) & DFS_MAX_PULSE_BUFFER_MASK;
                if(pl->pl_elems[index].p_time >= start_ts )
                        continue;
                else {
                        index = (index) & DFS_MAX_PULSE_BUFFER_MASK;
                        break;
                }
        }
        for (n=0;n<=rf->rf_numpulses; n++) {
                window_start = (start_ts + (refpri*n))-(primargin+n);
                window_end = window_start + 2*(primargin+n);
                DFS_DPRINTK(sc, ATH_DEBUG_DFS2,
                "window_start %u window_end %u \n",
                (u_int32_t)window_start, (u_int32_t)window_end);
                for(i=0;i<pl->pl_numelems;i++) {
                        prev_event_ts = pl->pl_elems[index].p_time;
                        index = (index+1) & DFS_MAX_PULSE_BUFFER_MASK;
                        event_ts = pl->pl_elems[index].p_time;
                        next_index = (index+1) & DFS_MAX_PULSE_BUFFER_MASK;
                        next_event_ts = pl->pl_elems[next_index].p_time;
                     DFS_DPRINTK(sc, ATH_DEBUG_DFS2,
                        "ts %u \n", (u_int32_t)event_ts);
                        if( (event_ts <= window_end) && (event_ts >= window_start)){
                                deltadur = DFS_DIFF(pl->pl_elems[index].p_dur, dur);
                                if( (pl->pl_elems[index].p_dur == 1) ||
                                        ((dur != 1) && (deltadur <= 2))) {
                                        numpulses++;
                                        DFS_DPRINTK(sc, ATH_DEBUG_DFS2,
                                        "numpulses %u \n", numpulses);
                                        break;
                                }
                        }
                        else if( event_ts > window_end) {
                                index = (index-1) & DFS_MAX_PULSE_BUFFER_MASK;
                                break;
                        }
                        else if( event_ts == prev_event_ts) {
                                if( ((next_event_ts - event_ts) > refpri) ||
                                        ((next_event_ts - event_ts) == 0)) {
                                        deltadur = DFS_DIFF(pl->pl_elems[index].p_dur, dur);
                                        if( (pl->pl_elems[index].p_dur == 1) ||
                                                ((pl->pl_elems[index].p_dur != 1) && (deltadur <= 2))) {
                                                numpulses++;
                                                DFS_DPRINTK(sc, ATH_DEBUG_DFS2,
                                                "zero PRI: numpulses %u \n", numpulses);
                                                break;
                                        }
                                }
                        }
                }
        }
        if (numpulses >= rf->rf_threshold) {
           DFS_DPRINTK(sc, ATH_DEBUG_DFS2, "%s FOUND filterID=%u\n", __func__, rf->rf_pulseid);
           return 1;
        }
        else 
                return 0;
}

static void
dfs_print_delayline(struct ath_softc *sc, struct dfs_delayline *dl)
{
	int i=0,index;
	struct dfs_delayelem *de;

	index = dl->dl_lastelem;
 	for (i=0; i<dl->dl_numelems; i++) {
		de = &dl->dl_elems[index];
		DFS_DPRINTK(sc, ATH_DEBUG_DFS2, 
			"Elem %d: ts = %u (0x%x) dur=%u\n",i,
		       de->de_time, de->de_time, de->de_dur);
		index = (index - 1)& DFS_MAX_DL_MASK;
	}
}

void
dfs_print_filters(struct ath_softc *sc)
{
        struct ath_dfs *dfs=sc->sc_dfs;
        struct dfs_filtertype *ft = NULL;
        struct dfs_filter *rf;
        int i,j;

        if (dfs == NULL) {
                DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s: sc_dfs is NULL\n", __func__);
                return;
        }
        for (i=0; i<DFS_MAX_RADAR_TYPES; i++) {
		if (dfs->dfs_radarf[i] != NULL) {
                    ft = dfs->dfs_radarf[i];
                    if((ft->ft_numfilters > DFS_MAX_NUM_RADAR_FILTERS) || (!ft->ft_numfilters)) 
                        continue;
                    printk("===========ft->ft_numfilters=%u===========\n", ft->ft_numfilters);
                    for (j=0; j<ft->ft_numfilters; j++) {
                        rf = &(ft->ft_filters[j]);
                        printk("filter[%d] filterID = %d rf_numpulses=%u; rf->rf_minpri=%u; rf->rf_maxpri=%u; rf->rf_threshold=%u; rf->rf_filterlen=%u; rf->rf_mindur=%u; rf->rf_maxdur=%u\n",j, rf->rf_pulseid,
                        rf->rf_numpulses, rf->rf_minpri, rf->rf_maxpri, rf->rf_threshold, rf->rf_filterlen, rf->rf_mindur, rf->rf_maxdur);
                    }
            }
    }
}

/*
 * Clear all delay lines for all filter types
 */
static void
dfs_reset_alldelaylines(struct ath_softc *sc)
{
        struct ath_dfs *dfs=sc->sc_dfs;
        struct dfs_filtertype *ft = NULL;
        struct dfs_filter *rf;
        struct dfs_delayline *dl;
        struct dfs_pulseline *pl = dfs->pulses;
        int i,j;

        if (dfs == NULL) {
                DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s: sc_dfs is NULL\n", __func__);
                return;
        }
         /* reset the pulse log */
        pl->pl_firstelem = pl->pl_numelems = 0;
        pl->pl_lastelem = DFS_MAX_PULSE_BUFFER_MASK;

        for (i=0; i<DFS_MAX_RADAR_TYPES; i++) {
		if (dfs->dfs_radarf[i] != NULL) {
                    ft = dfs->dfs_radarf[i];
                    for (j=0; j<ft->ft_numfilters; j++) {
                            rf = &(ft->ft_filters[j]);
                            dl = &(rf->rf_dl);
                            if(dl != NULL) {
                                    OS_MEMZERO(dl, sizeof(struct dfs_delayline));
                                    dl->dl_lastelem = (0xFFFFFFFF) & DFS_MAX_DL_MASK;
                            }
                    }
            }
        }
        for (i=0; i<dfs->dfs_rinfo.rn_numbin5radars; i++) {
                OS_MEMZERO(&(dfs->dfs_b5radars[i].br_elems[0]), sizeof(struct dfs_bin5elem)*DFS_MAX_B5_SIZE);
                dfs->dfs_b5radars[i].br_firstelem = 0;
                dfs->dfs_b5radars[i].br_numelems = 0;
                dfs->dfs_b5radars[i].br_lastelem = (0xFFFFFFFF)&DFS_MAX_B5_MASK;
        }
}
/*
 * Clear only a single delay line
 */

static void
dfs_reset_delayline(struct dfs_delayline *dl)
{
	OS_MEMZERO(&(dl->dl_elems[0]), sizeof(dl->dl_elems));
	dl->dl_lastelem = (0xFFFFFFFF)&DFS_MAX_DL_MASK;
}
	

void
dfs_reset_filter_delaylines(struct dfs_filtertype *dft)
{
        int i;
        struct dfs_filter *df;
        for (i=0; i< DFS_MAX_NUM_RADAR_FILTERS; i++) {
                df = &dft->ft_filters[i];
                dfs_reset_delayline(&(df->rf_dl));
        }
}
	
/*
 * Clear/Reset radar information for a single radar state/channel
 * Also clears all radar delay lines for all filter types
 */

#if 0
static void
dfs_reset_radarchannel(struct ath_softc *sc, struct dfs_state *rs)
{
	dfs_reset_alldelaylines(sc);
	rs->rs_numradarevents=0;
}
#endif

static void
dfs_reset_radarq(struct ath_softc *sc)
{
	struct ath_dfs *dfs=sc->sc_dfs;
	struct dfs_event *event;
	if (dfs == NULL) {
		DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s: sc_dfs is NULL\n", __func__);
		return;
	}
	ATH_DFSQ_LOCK(dfs);
	ATH_DFSEVENTQ_LOCK(dfs);
	while (!STAILQ_EMPTY(&(dfs->dfs_radarq))) {
		event = STAILQ_FIRST(&(dfs->dfs_radarq));
		STAILQ_REMOVE_HEAD(&(dfs->dfs_radarq), re_list);
		OS_MEMZERO(event, sizeof(struct dfs_event));
		STAILQ_INSERT_TAIL(&(dfs->dfs_eventq), event, re_list);
	}
	ATH_DFSEVENTQ_UNLOCK(dfs);
	ATH_DFSQ_UNLOCK(dfs);
}


/*
 * Add the radar event to the appropriate linked list which represents
 * the pulses which are currently in the tapped delay line with time
 * markers representing their current position in the filter.
 * pulseindex is the index of the pulse which we are trying to match,
 * so it is the index of the pulse whose filter we are convolving.
 */

static void
dfs_add_pulse(struct ath_softc *sc, struct dfs_filter *rf, struct dfs_event *re,
	      u_int32_t deltaT)
{
	u_int32_t index,n, window, pri;
        struct dfs_delayline *dl;

        dl = &rf->rf_dl;
	/* Circular buffer of size 2^n */
	index = (dl->dl_lastelem + 1) & DFS_MAX_DL_MASK;
	//if ((dl->dl_numelems+1) == DFS_MAX_DL_SIZE)
	if ((dl->dl_numelems) == DFS_MAX_DL_SIZE)
		dl->dl_firstelem = (dl->dl_firstelem + 1) & DFS_MAX_DL_MASK;
	else
		dl->dl_numelems++;
	dl->dl_lastelem = index;
	dl->dl_elems[index].de_time = deltaT;
        window = deltaT;
	dl->dl_elems[index].de_dur = re->re_dur;
	dl->dl_elems[index].de_rssi = re->re_rssi;

	for (n=0;n<dl->dl_numelems-1; n++) {
                index = (index-1) & DFS_MAX_DL_MASK;
                pri = dl->dl_elems[index].de_time;
                window += pri;
                    if (window > rf->rf_filterlen) {
			dl->dl_firstelem = (index+1) & DFS_MAX_DL_MASK;
			dl->dl_numelems = n+1;
		}
	}
	DFS_DPRINTK(sc, ATH_DEBUG_DFS2,
		"dl firstElem = %d  lastElem = %d\n",dl->dl_firstelem,
		dl->dl_lastelem);
}

static int dfs_bin_check(struct ath_softc *sc, struct dfs_filter *rf,
                             u_int32_t deltaT, u_int32_t width)
{
        u_int32_t refpri, refdur, searchpri, deltapri, averagerefpri;
        u_int32_t n, i, primargin, durmargin, highscore, highscoreindex;
        int score[DFS_MAX_DL_SIZE], delayindex, dindex, found=0;
        struct dfs_delayline *dl;
        u_int32_t scoreindex, lowpriindex= 0, lowpri = 0xffff;
        int numpulses=0;

        dl = &rf->rf_dl;
        if( dl->dl_numelems < (rf->rf_threshold-1)) {
                return 0; 
        }
        if( deltaT > rf->rf_filterlen)
                return 0;
        primargin = 10;
        if(rf->rf_maxdur < 10) {
                durmargin = 4;
        }
        else {
                durmargin = 6;
        }

        if( rf->rf_patterntype == 1 ){
                found = dfs_bin_fixedpattern_check(sc, rf, width);
                if(found) {
                        dl->dl_numelems = 0;
                }
                return found;
        }

        OS_MEMZERO(score, sizeof(int)*DFS_MAX_DL_SIZE);
        /* find out the lowest pri */
        for (n=0;n<dl->dl_numelems; n++) {
                delayindex = (dl->dl_firstelem + n) & DFS_MAX_DL_MASK;
                refpri = dl->dl_elems[delayindex].de_time;
                if( refpri == 0)
                        continue;
                else if(refpri < lowpri) {
                        lowpri = dl->dl_elems[delayindex].de_time;
                        lowpriindex = n;
                }
 }
        /* find out the each delay element's pri score */
        for (n=0;n<dl->dl_numelems; n++) {
                delayindex = (dl->dl_firstelem + n) & DFS_MAX_DL_MASK;
                refpri = dl->dl_elems[delayindex].de_time;
                if( refpri == 0)
                        continue;
                for (i=0;i<dl->dl_numelems; i++) {
                        dindex = (dl->dl_firstelem + i) & DFS_MAX_DL_MASK;
                        searchpri = dl->dl_elems[dindex].de_time;
                        deltapri = DFS_DIFF(searchpri, refpri);
                        if( deltapri < primargin)
                                score[n]++;
                }
                if( score[n] > rf->rf_threshold) {
                        /* we got the most possible candidate,
                         * no need to continue further */
                        break;
                }
        }
        /* find out the high scorer */
        highscore = 0;
        highscoreindex = 0;
        for (n=0;n<dl->dl_numelems; n++) {
                if( score[n] > highscore) {
                        highscore = score[n];
                        highscoreindex = n;
                }
                else if( score[n] == highscore ) {
                        /*more than one pri has highscore take the least pri */
                        delayindex = (dl->dl_firstelem + highscoreindex) & DFS_MAX_DL_MASK;
                        dindex = (dl->dl_firstelem + n) & DFS_MAX_DL_MASK;
                        if( dl->dl_elems[dindex].de_time <=
                                dl->dl_elems[delayindex].de_time ) {
                                highscoreindex = n;
                        }
                }
        }
        /* find the average pri of pulses around the pri of highscore or
         * the pulses around the lowest pri */
        if( highscore < 3) {
                scoreindex = lowpriindex;
        }
        else {
                scoreindex = highscoreindex;
        }
        /* We got the possible pri, save its parameters as reference */
        delayindex = (dl->dl_firstelem + scoreindex) & DFS_MAX_DL_MASK;
        refdur = dl->dl_elems[delayindex].de_dur;
        refpri = dl->dl_elems[delayindex].de_time;
        averagerefpri = 0;
        
        numpulses = dfs_bin_pri_check(sc, rf, dl, score[scoreindex], refpri, refdur);
        if (numpulses >= rf->rf_threshold) {
            found = 1;
            DFS_DPRINTK(sc, ATH_DEBUG_DFS2, "MATCH filter=%u numpulses=%u thresh=%u\n", rf->rf_pulseid, numpulses,rf->rf_threshold);     
        }
        return found;
}


static  int dfs_staggered_check(struct ath_softc *sc, struct dfs_filter *rf,
                             u_int32_t deltaT, u_int32_t width)
{

        u_int32_t refpri, refdur, searchpri=0, deltapri;//, averagerefpri;
        u_int32_t n, i, primargin, durmargin;
        int score[DFS_MAX_DL_SIZE], delayindex, dindex, found=0;
        struct dfs_delayline *dl;
        u_int32_t scoreindex, lowpriindex= 0, lowpri = 0xffff;
        int numpulses=0, higherthan, lowerthan, numscores;
        u_int32_t lowestscore=0, lowestscoreindex=0, lowestpri=0;
        u_int32_t midscore=0, midscoreindex=0, midpri=0;
        u_int32_t highestscore=0, highestscoreindex=0, highestpri=0;

        dl = &rf->rf_dl;
        if( dl->dl_numelems < (rf->rf_threshold-1)) {
            DFS_DPRINTK(sc, ATH_DEBUG_DFS2, "numelems %d < threshold for filter %d\n", dl->dl_numelems, rf->rf_pulseid);    
            return 0; 
        }
        if( deltaT > rf->rf_filterlen) {
            DFS_DPRINTK(sc, ATH_DEBUG_DFS2, "numelems %d < threshold for filter %d\n", dl->dl_numelems, rf->rf_pulseid);    
            return 0;
        }
        primargin = 10;
        if(rf->rf_maxdur < 10) {
                durmargin = 4;
        }
        else {
                durmargin = 6;
        }

        OS_MEMZERO(score, sizeof(int)*DFS_MAX_DL_SIZE);
        /* find out the lowest pri */
        for (n=0;n<dl->dl_numelems; n++) {
                delayindex = (dl->dl_firstelem + n) & DFS_MAX_DL_MASK;
                refpri = dl->dl_elems[delayindex].de_time;
                if( refpri == 0)
                        continue;
                else if(refpri < lowpri) {
                        lowpri = dl->dl_elems[delayindex].de_time;
                        lowpriindex = n;
                }
 }
        /* find out the each delay element's pri score */
        for (n=0;n<dl->dl_numelems; n++) {
                delayindex = (dl->dl_firstelem + n) & DFS_MAX_DL_MASK;
                refpri = dl->dl_elems[delayindex].de_time;
                if( refpri == 0)
                        continue;
                for (i=0;i<dl->dl_numelems; i++) {
                        dindex = (dl->dl_firstelem + i) & DFS_MAX_DL_MASK;
                        searchpri = dl->dl_elems[dindex].de_time;
                        deltapri = DFS_DIFF(searchpri, refpri);
                        if( deltapri < primargin)
                                score[n]++;
                }
                if( score[n] > rf->rf_threshold) {
                        /* we got the most possible candidate,
                         * no need to continue further */
                        DFS_DPRINTK(sc, ATH_DEBUG_DFS2, "THRESH score[%d]=%d pri=%d\n", n, score[n], searchpri);                       
                         break;
                }
        }
        for (n=0;n<dl->dl_numelems; n++) {
            delayindex = (dl->dl_firstelem + n) & DFS_MAX_DL_MASK;
            refdur = dl->dl_elems[delayindex].de_time;
            DFS_DPRINTK(sc, ATH_DEBUG_DFS2, "score[%d]=%d pri=%d\n", n, score[n], refdur);
        }

        /* find out the 2 or 3 highest scorers */
        scoreindex = 0;
        highestscore=0;
        highestscoreindex=0;
        highestpri=0; numscores=0; lowestscore=0;

        for (n=0;n<dl->dl_numelems; n++) {
                higherthan=0;
                lowerthan=0;
                delayindex = (dl->dl_firstelem + n) & DFS_MAX_DL_MASK;
                refpri = dl->dl_elems[delayindex].de_time;

                if ((score[n] >= highestscore) && 
                        (is_unique_pri(highestpri, midpri, lowestpri, refpri))) {
                            lowestscore = midscore;
                            lowestpri = midpri;
                            lowestscoreindex = midscoreindex;
                            midscore = highestscore;
                            midpri = highestpri;
                            midscoreindex = highestscoreindex;
                            highestscore = score[n];
                            highestpri = refpri;
                            highestscoreindex = n;
                    } else {
                        if ((score[n] >= midscore) &&
                            (is_unique_pri(highestpri, midpri, lowestpri, refpri))) {
                            lowestscore = midscore;
                            lowestpri = midpri;
                            lowestscoreindex = midscoreindex;
                            midscore = score[n];
                            midpri = refpri;
                            midscoreindex = n;
                        } else if ((score[n] >= lowestscore) &&
                            (is_unique_pri(highestpri, midpri, lowestpri, refpri))) {
                            lowestscore = score[n];
                            lowestpri = refpri;
                            lowestscoreindex = n;
                        }
                    } 
                    
                }

             DFS_DPRINTK(sc, ATH_DEBUG_DFS1, "FINAL highestscore=%d highestscoreindex=%d highestpri=%d\n", highestscore, highestscoreindex, highestpri);
            DFS_DPRINTK(sc, ATH_DEBUG_DFS1, "FINAL lowestscore=%d lowestscoreindex=%d lowpri=%d\n", lowestscore, lowestscoreindex, lowestpri);
            DFS_DPRINTK(sc, ATH_DEBUG_DFS1, "FINAL midscore=%d midscoreindex=%d midpri=%d\n", midscore, midscoreindex, midpri);
    
            delayindex = (dl->dl_firstelem + highestscoreindex) & DFS_MAX_DL_MASK;
            refdur = dl->dl_elems[delayindex].de_dur;
            refpri = dl->dl_elems[delayindex].de_time;
            DFS_DPRINTK(sc, ATH_DEBUG_DFS1, "highscoreindex=%d refdur=%d refpri=%d\n", highestscoreindex, refdur, refpri);
        
            numpulses += dfs_bin_pri_check(sc, rf, dl, highestscore, refpri, refdur);

;           delayindex = (dl->dl_firstelem + midscoreindex) & DFS_MAX_DL_MASK;
            refdur = dl->dl_elems[delayindex].de_dur;
            refpri = dl->dl_elems[delayindex].de_time;
            DFS_DPRINTK(sc, ATH_DEBUG_DFS1, "midscoreindex=%d refdur=%d refpri=%d\n", midscoreindex, refdur, refpri);
        
            numpulses += dfs_bin_pri_check(sc, rf, dl, midscore, refpri, refdur);
; 
            delayindex = (dl->dl_firstelem + lowestscoreindex) & DFS_MAX_DL_MASK;
            refdur = dl->dl_elems[delayindex].de_dur;
            refpri = dl->dl_elems[delayindex].de_time;
            DFS_DPRINTK(sc, ATH_DEBUG_DFS1, "lowestscoreindex=%d refdur=%d refpri=%d\n", lowestscoreindex, refdur, refpri);
        
            numpulses += dfs_bin_pri_check(sc, rf, dl, lowestscore, refpri, refdur);
            DFS_DPRINTK(sc, ATH_DEBUG_DFS2, "numpulses=%d\n",numpulses);

        if (numpulses >= rf->rf_threshold) {
            found = 1;
            DFS_DPRINTK(sc, ATH_DEBUG_DFS1, "MATCH filter=%u numpulses=%u thresh=%u\n", rf->rf_pulseid, numpulses,rf->rf_threshold);     
        }
        return found;
    }


static  int dfs_bin_pri_check(struct ath_softc *sc, struct dfs_filter *rf,
                             struct dfs_delayline *dl, u_int32_t score,
                             u_int32_t refpri, u_int32_t refdur)
{
        u_int32_t searchpri, searchdur, searchrssi, deltapri, deltadur, averagerefpri=0;
        int primultiples[6];
        int delayindex, dindex;
        u_int32_t i, j, primargin, durmargin, highscore=score, highscoreindex=0;
        int numpulses=1;  //first pulse in the burst is most likely being filtered out based on maxfilterlen
        primargin = 10;
        if(rf->rf_maxdur < 10) {
                durmargin = 4;
        }
        else {
                durmargin = 6;
        }
        if( score > 1) {
                for (i=0;i<dl->dl_numelems; i++) {
                        dindex = (dl->dl_firstelem + i) & DFS_MAX_DL_MASK;
                        searchpri = dl->dl_elems[dindex].de_time;
                        deltapri = DFS_DIFF(searchpri, refpri);
                        if( deltapri < primargin)
                                averagerefpri += searchpri;
                }
                refpri = (averagerefpri/score);  //average
        }
        /* Note: Following primultiple calculation should be done once per filter
         * during initialization stage (dfs_attach) and stored in its array
         * atleast for fixed frequency types like FCC Bin1 to save some CPU cycles.
         * multiplication, devide operators in the following code are left as it is
         * for readability hoping the complier will use left/right shifts wherever possible
         */
        if( refpri > rf->rf_maxpri) {
                primultiples[0] = (refpri - refdur)/2;
                primultiples[1] = refpri;
                primultiples[2] = refpri + primultiples[0];
                primultiples[3] = (refpri - refdur)*2;
                primultiples[4] = primultiples[3] + primultiples[0];
                primultiples[5] = primultiples[3] + refpri;
        }
        else {
                primultiples[0] = refpri;
                primultiples[1] = refpri + primultiples[0];
                primultiples[2] = refpri + primultiples[1];
                primultiples[3] = refpri + primultiples[2];
                primultiples[4] = refpri + primultiples[3];
                primultiples[5] = refpri + primultiples[4];
        }
        DFS_DPRINTK(sc, ATH_DEBUG_DFS2,
                "pri0 = %d pri1 = %d pri2 = %d pri3 = %d pri4 = %d pri5 = %d\n",
                primultiples[0], primultiples[1], primultiples[2],
                primultiples[3], primultiples[4], primultiples[5]);
        DFS_DPRINTK(sc, ATH_DEBUG_DFS2,
                "refpri = %d high score = %d index = %d numpulses = %d\n",
                refpri, highscore, highscoreindex, numpulses);
   /* Count the other delay elements that have  pri and dur with in the
         * acceptable range from the reference one */
        for (i=0; i<dl->dl_numelems; i++) {
                delayindex = (dl->dl_firstelem + i) & DFS_MAX_DL_MASK;
                searchpri = dl->dl_elems[delayindex].de_time;
                if( searchpri == 0) {
                        /* This events PRI is zero, take it as a
                         * valid pulse but decrement next event's PRI by refpri
                         */
                        dindex = (delayindex+1)& DFS_MAX_DL_MASK;
                        dl->dl_elems[dindex].de_time -=  refpri;
                        searchpri = refpri;
                }
                searchdur = dl->dl_elems[delayindex].de_dur;
                searchrssi = dl->dl_elems[delayindex].de_rssi;
                deltadur = DFS_DIFF(searchdur, refdur);
                for(j=0; j<6; j++) {
                        deltapri = DFS_DIFF(searchpri, primultiples[j]);
                        if( deltapri < (primargin+j)) {
                                if( deltadur < durmargin) {
                                    if( (refdur < 8) || ((refdur >=8)&&
                                        (searchrssi < 250))) {

                                        numpulses++;
                                        DFS_DPRINTK(sc, ATH_DEBUG_DFS2,
                                        "searchpri = %d index = %d numpulses = %d\n",
                                        searchpri, i, numpulses);
                                    }
                                }
                                break;
                        }
                }
        }
        return numpulses;
}

static
OS_TIMER_FUNC(dfs_remove_from_nol)
{
        struct ath_softc *sc;
        uint32_t nchans=0;
        HAL_CHANNEL *chans = NULL;
        struct dfs_nol_timer_arg *nol_arg;
        u_int16_t delfreq=0;

        OS_GET_TIMER_ARG(nol_arg, struct dfs_nol_timer_arg *);

        sc = nol_arg->sc;
        delfreq= nol_arg->delfreq;

        if (sc->sc_opmode == HAL_M_HOSTAP)
        {
            chans = (HAL_CHANNEL *)OS_MALLOC(sc->sc_osdev,
                                         IEEE80211_CHAN_MAX * sizeof(HAL_CHANNEL),
                                         GFP_ATOMIC);
            if (chans == NULL) 
            {
                DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s: unable to allocate channel table\n", __func__);
                goto done;
            }

            nchans = dfs_check_nol(sc, chans, IEEE80211_CHAN_MAX, delfreq);
            if (nchans > 0 && sc->sc_ieee_ops->setup_channel_list) 
            {
                sc->sc_ieee_ops->setup_channel_list(sc->sc_ieee, CLIST_DFS_UPDATE,
                                                chans, nchans, NULL, 0,
                                                CTRY_DEFAULT);
            }
            OS_FREE(chans);
        }
        done: 
             OS_FREE(nol_arg);
            return;

}

static void
dfs_print_nol(struct ath_softc *sc)
{
	struct ath_dfs *dfs=sc->sc_dfs;
	struct dfs_nolelem *nol;
        int i=0;
        uint32_t diff=0;
#define TIME_IN_MS 1000
#define TIME_IN_US (TIME_IN_MS * 1000)

	if (dfs == NULL) {
		DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s: sc_dfs is NULL\n",
			__func__);
		return;
	}
	nol = dfs->dfs_nol;
        printk("NOL\n");
	while (nol != NULL) {
		diff=(int)(nol->nol_timer.expires - OS_GET_TICKS());
                diff=CONVERT_SYSTEM_TIME_TO_SEC(diff);
                DFS_DPRINTK(sc, ATH_DEBUG_DFS, 
                                "nol:%d channel=%d time left=%u seconds \n", 
                                i++, nol->nol_chan.channel, diff);
                nol=nol->nol_next;
        }
#undef TIME_IN_MS
#undef TIME_IN_US
}
static void
dfs_nol_addchan(struct ath_softc *sc, HAL_CHANNEL *chan)
{
	struct ath_dfs *dfs=sc->sc_dfs;
	struct dfs_nolelem *nol, *elem, *prev;
        struct dfs_nol_timer_arg *dfs_nol_arg;

	if (dfs == NULL) {
		DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s: sc_dfs is NULL\n",
			__func__);
		return;
	}
	nol = dfs->dfs_nol;
	prev = dfs->dfs_nol;
        elem=NULL;
	while (nol != NULL) {
                    if ((nol->nol_chan.channel == chan->channel) &&
		    (nol->nol_chan.channelFlags == chan->channelFlags) &&
		    (nol->nol_chan.privFlags == chan->privFlags)) {
                        DFS_DPRINTK(sc, ATH_DEBUG_DFS2, "%s: Update TSF for NOL channel %d\n", __func__, nol->nol_chan.channel);
                        OS_CANCEL_TIMER(&nol->nol_timer);
                        OS_SET_TIMER(&nol->nol_timer, DFS_NOL_TIMEOUT_MS);
			ath_hal_dfsfound(sc->sc_ah, chan, DFS_NOL_TIMEOUT_US);
			return;
		}
		prev = nol;
		nol = nol->nol_next;
	}
        /* Add a new element to the NOL*/
	elem = (struct dfs_nolelem *)OS_MALLOC(sc->sc_osdev, sizeof(struct dfs_nolelem),GFP_ATOMIC);
	dfs_nol_arg = (struct dfs_nol_timer_arg *)OS_MALLOC(sc->sc_osdev, sizeof(struct dfs_nol_timer_arg), GFP_ATOMIC);
	if (elem == NULL || dfs_nol_arg == NULL) {
		DFS_DPRINTK(sc, ATH_DEBUG_DFS,
			"%s: failed to allocate memory for nol entry\n",
			 __func__);
		return;
	}
	elem->nol_chan = *chan;
	elem->nol_next = NULL;
	if (prev) {
            prev->nol_next = elem;
        } else {
            /* This is the first element in the NOL */
            dfs->dfs_nol = elem;
        }
        dfs_nol_arg->sc = sc;
        dfs_nol_arg->delfreq = chan->channel;

        OS_INIT_TIMER(sc->sc_osdev, &elem->nol_timer, dfs_remove_from_nol, dfs_nol_arg);
        OS_SET_TIMER(&elem->nol_timer, DFS_NOL_TIMEOUT_MS);
        DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s: new NOL channel %d\n", __func__, elem->nol_chan.channel);
	ath_hal_dfsfound(sc->sc_ah, chan, DFS_NOL_TIMEOUT_US);
}

static int
dfs_bin5_addpulse(struct ath_softc *sc, struct dfs_bin5radars *br,
		  struct dfs_event *re, u_int64_t thists)
{
	u_int32_t index,stop;
	u_int64_t tsDelta;

	/* Check if this pulse is a valid pulse in terms of repetition, 
	 * if not, return without adding it to the queue.
	 * PRI : Pulse Repitetion Interval
	 * BRI : Burst Repitetion Interval */ 
	if( br->br_numelems != 0){
		index = br->br_lastelem;
		tsDelta = thists - br->br_elems[index].be_ts;
		if( (tsDelta < DFS_BIN5_PRI_LOWER_LIMIT) ||
			( (tsDelta > DFS_BIN5_PRI_HIGHER_LIMIT) &&
			  (tsDelta < DFS_BIN5_BRI_LOWER_LIMIT))) {
			    return 0;
		}
	}
	/* Circular buffer of size 2^n */
	index = (br->br_lastelem +1) & DFS_MAX_B5_MASK;
	br->br_lastelem = index;
        if (br->br_numelems == DFS_MAX_B5_SIZE)
		br->br_firstelem = (br->br_firstelem+1)&DFS_MAX_B5_MASK;
	else
		br->br_numelems++;
	br->br_elems[index].be_ts = thists;
	br->br_elems[index].be_rssi = re->re_rssi;
	br->br_elems[index].be_dur = re->re_dur;
	stop = 0;
	index = br->br_firstelem;
	while ((!stop) && (br->br_numelems-1) > 0) {
		if ((thists - br->br_elems[index].be_ts) > 
		    ((u_int64_t) br->br_pulse.b5_timewindow)) {
			br->br_numelems--;
			br->br_firstelem = (br->br_firstelem +1) & DFS_MAX_B5_MASK;
			index = br->br_firstelem;
		} else
			stop = 1;
	}
	return 1;
}

/*
 * If the dfs structure is NULL (which should be illegal if everyting is working
 * properly, then signify that a bin5 radar was found
 */

static int
dfs_bin5_check(struct ath_softc *sc)
{
	struct ath_dfs *dfs=sc->sc_dfs;
	struct dfs_bin5radars *br;
        int index[DFS_MAX_B5_SIZE];
	u_int32_t n, i, this, prev, rssi_diff, width_diff, bursts= 0;
        u_int32_t total_diff=0, average_diff, total_width=0, average_width, numevents=0;
	u_int64_t pri;

	if (dfs == NULL) {
		DFS_DPRINTK(sc, ATH_DEBUG_DFS,"%s: sc_dfs is NULL\n",
			__func__);
		return 1;
	}
	for (n=0;n<dfs->dfs_rinfo.rn_numbin5radars; n++) {
		br = &(dfs->dfs_b5radars[n]);
		DFS_DPRINTK(sc, ATH_DEBUG_DFS1,
			"Num elems = %d\n", br->br_numelems);
		prev = br->br_firstelem;
		for(i=0;i<br->br_numelems;i++){
			this = ((br->br_firstelem +i) & DFS_MAX_B5_MASK);
			/* Rule 1: 1000 <= PRI <= 2000 + some margin */
			if( br->br_elems[this].be_ts >= br->br_elems[prev].be_ts ) {
				pri = br->br_elems[this].be_ts - br->br_elems[prev].be_ts;
			}
			else {//roll over case
				//pri = (0xffffffffffffffff - br->br_elems[prev].be_ts) + br->br_elems[this].be_ts;
				pri = br->br_elems[this].be_ts;
			}
			if(( (pri >= DFS_BIN5_PRI_LOWER_LIMIT) && (pri <= DFS_BIN5_PRI_HIGHER_LIMIT))) {  //pri: pulse repitition interval in us
				/* Rule 2: pulse width of the pulses in the burst should be same (+/- margin) */
				if( br->br_elems[this].be_dur >= br->br_elems[prev].be_dur) {
					width_diff = br->br_elems[this].be_dur - br->br_elems[prev].be_dur;
				}
				else {
					width_diff = br->br_elems[prev].be_dur - br->br_elems[this].be_dur;
				}
				if( width_diff <= DFS_BIN5_WIDTH_MARGIN ) {
					/* Rule 3: RSSI of the pulses in the burst should be same (+/- margin) */
					if( br->br_elems[this].be_rssi >= br->br_elems[prev].be_rssi) {
						rssi_diff = br->br_elems[this].be_rssi - br->br_elems[prev].be_rssi;
					}
					else {
						rssi_diff = br->br_elems[prev].be_rssi - br->br_elems[this].be_rssi;
					}
					if( rssi_diff <= DFS_BIN5_RSSI_MARGIN ) {
						bursts++;
                                                /* Save the indexes of this pair for later width variance check */
                                                if( numevents >= 2 ) {
                                                        /* make sure the event is not duplicated,
                                                         * possible in a 3 pulse burst */
                                                        if( index[numevents-1] != prev) {
                                                                index[numevents++] = prev;
                                                        }
                                                }
                                                else {
                                                        index[numevents++] = prev;                                                }
                                                index[numevents++] = this;
					}
				} 
			} 
			prev = this;
		}
                DFS_DPRINTK(sc, ATH_DEBUG_DFS2, "bursts=%u numevents=%u\n", bursts, numevents);
		if ( bursts >= br->br_pulse.b5_threshold) {
                        if( (br->br_elems[br->br_lastelem].be_ts - br->br_elems[br->br_firstelem].be_ts) < 3000000 ) {
                                return 0;
                        }
                        else {
                                for (i=0; i<numevents; i++){
                                        total_width += br->br_elems[index[i]].be_dur;
                                }
                                average_width = total_width/numevents;
                                for (i=0; i<numevents; i++){
                                        total_diff += DFS_DIFF(br->br_elems[index[i]].be_dur, average_width);
                                }
                                average_diff = total_diff/numevents;
                                if( average_diff > DFS_BIN5_WIDTH_MARGIN ) {
                                        return 1;
                                } 

                        }
		}
	}

	return 0;
}

static inline u_int8_t
dfs_process_pulse_dur(struct ath_softc *sc, u_int8_t re_dur) 
{
	struct ath_dfs *dfs=sc->sc_dfs;
	if (re_dur == 0) {
		return 1;
	} else {
		/* Convert 0.8us durations to TSF ticks (usecs) */
		return (u_int8_t)_round((int32_t)((dfs->dur_multiplier)*re_dur));
	}
}

/*
 * Finds the radar state entry that matches the current channel
 */
struct dfs_state *
dfs_getchanstate(struct ath_softc *sc, u_int8_t *index, int ext_chan_flag)
{
	struct ath_dfs *dfs=sc->sc_dfs;
	struct dfs_state *rs=NULL;
	int i;
        HAL_CHANNEL *cmp_ch;

	if (dfs == NULL) {
		DFS_DPRINTK(sc, ATH_DEBUG_DFS,"%s: sc_dfs is NULL\n",
			__func__);
		return NULL;
	}

        if (ext_chan_flag) {
            cmp_ch = (ath_hal_get_extension_channel(sc->sc_ah));    
            if (cmp_ch) {
                DFS_DPRINTK(sc, ATH_DEBUG_DFS2, "Extension channel freq = %u flags=0x%x\n", cmp_ch->channel, cmp_ch->privFlags);
            } else {
                return NULL;
            }
            
        } else {
            cmp_ch = &sc->sc_curchan;
            DFS_DPRINTK(sc, ATH_DEBUG_DFS2, "Primary channel freq = %u flags=0x%x\n", cmp_ch->channel, cmp_ch->privFlags);
        }      
	for (i=0;i<DFS_NUM_RADAR_STATES; i++) {
                if ((dfs->dfs_radar[i].rs_chan.channel == cmp_ch->channel) &&
		    (dfs->dfs_radar[i].rs_chan.channelFlags == cmp_ch->channelFlags)) {
			if (index != NULL)
				*index = (u_int8_t) i;
			return &(dfs->dfs_radar[i]);
		}
	}
	/* No existing channel found, look for first free channel state entry */
	for (i=0; i<DFS_NUM_RADAR_STATES; i++) {
		if (dfs->dfs_radar[i].rs_chan.channel == 0) {
			rs = &(dfs->dfs_radar[i]);
			/* Found one, set channel info and default thresholds */
			rs->rs_chan = *cmp_ch;
			rs->rs_firpwr = dfs->dfs_defaultparams.pe_firpwr;
			rs->rs_radarrssi = dfs->dfs_defaultparams.pe_rrssi;
			rs->rs_height = dfs->dfs_defaultparams.pe_height;
			rs->rs_pulserssi = dfs->dfs_defaultparams.pe_prssi;
			rs->rs_inband = dfs->dfs_defaultparams.pe_inband;
			/* 5413 specific */
			rs->rs_relpwr = dfs->dfs_defaultparams.pe_relpwr;
			rs->rs_relstep = dfs->dfs_defaultparams.pe_relstep;
			rs->rs_maxlen = dfs->dfs_defaultparams.pe_maxlen;

			if (index != NULL)
				*index = (u_int8_t) i;
			return (rs);
		}
	}
	DFS_DPRINTK(sc, ATH_DEBUG_DFS2, "%s: No more radar states left.\n", __func__);
	return(NULL);
}


int
dfs_process_radarevent(struct ath_softc *sc, HAL_CHANNEL *chan)
{
	struct ath_dfs *dfs=sc->sc_dfs;
	struct ath_hal *ah=sc->sc_ah;
	struct dfs_event re,*event;
	struct dfs_state *rs=NULL;
	struct dfs_filtertype *ft;
	struct dfs_filter *rf;
	int found, retval=0,p, empty;
	int events_processed=0;
        u_int32_t tabledepth,rfilt, index;
	u_int64_t deltafull_ts = 0,this_ts, deltaT;
	HAL_CHANNEL *thischan;
	HAL_PHYERR_PARAM pe;
        struct dfs_pulseline *pl = dfs->pulses;
        static u_int32_t  test_ts  = 0;
        static u_int32_t  diff_ts  = 0;

	if (dfs == NULL) {
		DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s: sc_sfs is NULL\n",
			__func__);
		return 0;
	}
	if ( ! (sc->sc_curchan.privFlags & CHANNEL_DFS)) {
	        DFS_DPRINTK(sc, ATH_DEBUG_DFS2, "%s: radar event on non-DFS chan\n",
                        __func__);
                dfs_reset_radarq(sc);
                dfs_reset_alldelaylines(sc);
        	return 0;
        }
 	/* TEST : Simulate radar bang, make sure we add the channel to NOL (bug 29968) */
        if (dfs->dfs_bangradar) {
                    /* bangradar will always simulate radar found on the primary channel */
		     rs = &dfs->dfs_radar[dfs->dfs_curchan_radindex];
 		     dfs->dfs_bangradar = 0; /* reset */
	             DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s: bangradar\n", __func__);
 		     retval = 1;                    
                     goto dfsfound;
 	 }

	ATH_DFSQ_LOCK(dfs);
	empty = STAILQ_EMPTY(&(dfs->dfs_radarq));
	ATH_DFSQ_UNLOCK(dfs);

	while ((!empty) && (!retval) && (events_processed < MAX_EVENTS)) {
		ATH_DFSQ_LOCK(dfs);
		event = STAILQ_FIRST(&(dfs->dfs_radarq));
		if (event != NULL)
			STAILQ_REMOVE_HEAD(&(dfs->dfs_radarq), re_list);
		ATH_DFSQ_UNLOCK(dfs);

		if (event == NULL) {
			empty = 1;
			break;
		}
                events_processed++;
                re = *event;

		OS_MEMZERO(event, sizeof(struct dfs_event));
		ATH_DFSEVENTQ_LOCK(dfs);
		STAILQ_INSERT_TAIL(&(dfs->dfs_eventq), event, re_list);
		ATH_DFSEVENTQ_UNLOCK(dfs);

		found = 0;
		if (re.re_chanindex < DFS_NUM_RADAR_STATES)
			rs = &dfs->dfs_radar[re.re_chanindex];
		else {
			ATH_DFSQ_LOCK(dfs);
			empty = STAILQ_EMPTY(&(dfs->dfs_radarq));
			ATH_DFSQ_UNLOCK(dfs);
			continue;
		}
		if (rs->rs_chan.privFlags & CHANNEL_INTERFERENCE) {
			ATH_DFSQ_LOCK(dfs);
			empty = STAILQ_EMPTY(&(dfs->dfs_radarq));
			ATH_DFSQ_UNLOCK(dfs);
			continue;
		}

		if (dfs->dfs_rinfo.rn_lastfull_ts == 0) {
			/*
			 * Either not started, or 64-bit rollover exactly to zero
			 * Just prepend zeros to the 15-bit ts
			 */
			dfs->dfs_rinfo.rn_ts_prefix = 0;
			this_ts = (u_int64_t) re.re_ts;
		} else {
                         /* WAR 23031- patch duplicate ts on very short pulses */
                        /* This pacth has two problems in linux environment.
                         * 1)The time stamp created and hence PRI depends entirely on the latency.
                         *   If the latency is high, it possibly can split two consecutive
                         *   pulses in the same burst so far away (the same amount of latency)
                         *   that make them look like they are from differenct bursts. It is
                         *   observed to happen too often. It sure makes the detection fail.
                         * 2)Even if the latency is not that bad, it simply shifts the duplicate
                         *   timestamps to a new duplicate timestamp based on how they are processed.
                         *   This is not worse but not good either.
                         *
                         *   Take this pulse as a good one and create a probable PRI later
                         */
                        if (re.re_dur == 0 && re.re_ts == dfs->dfs_rinfo.rn_last_unique_ts) {
                                debug_dup[debug_dup_cnt++] = '1';
                                DFS_DPRINTK(sc, ATH_DEBUG_DFS1, "\n %s deltaT is 0 \n", __func__);
                        } else {
                                dfs->dfs_rinfo.rn_last_unique_ts = re.re_ts;
                                debug_dup[debug_dup_cnt++] = '0';
                        }
                        if (debug_dup_cnt >= 32){
                                 debug_dup_cnt = 0;
                        }


			if (re.re_ts <= dfs->dfs_rinfo.rn_last_ts) {
				dfs->dfs_rinfo.rn_ts_prefix += 
					(((u_int64_t) 1) << DFS_TSSHIFT);
				/* Now, see if it's been more than 1 wrap */
				deltafull_ts = re.re_full_ts - dfs->dfs_rinfo.rn_lastfull_ts;
				if (deltafull_ts > 
				    ((u_int64_t)((DFS_TSMASK - dfs->dfs_rinfo.rn_last_ts) + 1 + re.re_ts)))
					deltafull_ts -= (DFS_TSMASK - dfs->dfs_rinfo.rn_last_ts) + 1 + re.re_ts;
				deltafull_ts = deltafull_ts >> DFS_TSSHIFT;
				if (deltafull_ts > 1) {
					dfs->dfs_rinfo.rn_ts_prefix += 
						((deltafull_ts - 1) << DFS_TSSHIFT);
				}
			} else {
				deltafull_ts = re.re_full_ts - dfs->dfs_rinfo.rn_lastfull_ts;
				if (deltafull_ts > (u_int64_t) DFS_TSMASK) {
					deltafull_ts = deltafull_ts >> DFS_TSSHIFT;
					dfs->dfs_rinfo.rn_ts_prefix += 
						((deltafull_ts - 1) << DFS_TSSHIFT);
				}
			}
			this_ts = dfs->dfs_rinfo.rn_ts_prefix | ((u_int64_t) re.re_ts);
		}
		dfs->dfs_rinfo.rn_lastfull_ts = re.re_full_ts;
		dfs->dfs_rinfo.rn_last_ts = re.re_ts;

		re.re_dur = dfs_process_pulse_dur(sc, re.re_dur);
	        if (re.re_dur != 1) {
                	this_ts -= re.re_dur;
                }

              /* Save the pulse parameters in the pulse buffer(pulse line) */
                index = (pl->pl_lastelem + 1) & DFS_MAX_PULSE_BUFFER_MASK;
                if (pl->pl_numelems == DFS_MAX_PULSE_BUFFER_SIZE)
                        pl->pl_firstelem = (pl->pl_firstelem+1) & DFS_MAX_PULSE_BUFFER_MASK;
                else
                        pl->pl_numelems++;
                pl->pl_lastelem = index;
                pl->pl_elems[index].p_time = this_ts;
                pl->pl_elems[index].p_dur = re.re_dur;
                pl->pl_elems[index].p_rssi = re.re_rssi;
                diff_ts = (u_int32_t)this_ts - test_ts;
                test_ts = (u_int32_t)this_ts;
                DFS_DPRINTK(sc, ATH_DEBUG_DFS1,"ts%u %u %u diff %u pl->pl_lastelem.p_time=%llu\n",(u_int32_t)this_ts, re.re_dur, re.re_rssi, diff_ts, (unsigned long long)pl->pl_elems[index].p_time);

		found = 0;
		for (p=0; (p<dfs->dfs_rinfo.rn_numbin5radars)&&(!found); p++) {
			struct dfs_bin5radars *br;
			u_int32_t b5_rssithresh;
			br = &(dfs->dfs_b5radars[p]);
			b5_rssithresh = br->br_pulse.b5_rssithresh;

                       /* Adjust the filter threshold for rssi in non TURBO mode*/
                        if( ! (sc->sc_curchan.channelFlags & CHANNEL_TURBO) ) {
                                b5_rssithresh += br->br_pulse.b5_rssimargin;
                        }

			if ((re.re_dur >= br->br_pulse.b5_mindur) &&
			    (re.re_dur <= br->br_pulse.b5_maxdur) &&
			    (re.re_rssi >= b5_rssithresh)) {
				if( dfs_bin5_addpulse(sc, br, &re, this_ts) ) {
					found |= dfs_bin5_check(sc);
				}
			} 
		}
		if (found) {
			DFS_DPRINTK(sc, ATH_DEBUG_DFS1, "%s: Found bin5 radar\n", __func__);
			retval |= found;
			goto dfsfound;
		}
		tabledepth = 0;
		rf = NULL;
		DFS_DPRINTK(sc, ATH_DEBUG_DFS1,"  *** chan freq (%d): ts %llu dur %u rssi %u\n",
			rs->rs_chan.channel, (unsigned long long)this_ts, re.re_dur, re.re_rssi);

		while ((tabledepth < DFS_MAX_RADAR_OVERLAP) &&
		       ((dfs->dfs_radartable[re.re_dur])[tabledepth] != -1) &&
		       (!retval)) {
			ft = dfs->dfs_radarf[((dfs->dfs_radartable[re.re_dur])[tabledepth])];
			DFS_DPRINTK(sc, ATH_DEBUG_DFS2,"  ** RD (%d): ts %x dur %u rssi %u\n",
				       rs->rs_chan.channel,
				       re.re_ts, re.re_dur, re.re_rssi);

			if (re.re_rssi < ft->ft_rssithresh && re.re_dur > 4) {
	        		DFS_DPRINTK(sc, ATH_DEBUG_DFS2,"%s : Rejecting on rssi rssi=%u thresh=%u\n", __func__, re.re_rssi, ft->ft_rssithresh);
				tabledepth++;
				ATH_DFSQ_LOCK(dfs);
				empty = STAILQ_EMPTY(&(dfs->dfs_radarq));
				ATH_DFSQ_UNLOCK(dfs);
				continue;
			}
			deltaT = this_ts - ft->ft_last_ts;
			DFS_DPRINTK(sc, ATH_DEBUG_DFS2,"deltaT = %lld (ts: 0x%llx) (last ts: 0x%llx)\n",(unsigned long long)deltaT, (unsigned long long)this_ts, (unsigned long long)ft->ft_last_ts);
			if ((deltaT < ft->ft_minpri) && (deltaT !=0)){
                                /* This check is for the whole filter type. Individual filters
                                 will check this again. This is first line of filtering.*/
				DFS_DPRINTK(sc, ATH_DEBUG_DFS2, "%s: Rejecting on pri pri=%lld minpri=%u\n", __func__, (unsigned long long)deltaT, ft->ft_minpri);
                                tabledepth++;
				continue;
			}
			for (p=0, found = 0; (p<ft->ft_numfilters) && (!found); p++) {
                                    rf = &(ft->ft_filters[p]);
                                    if ((re.re_dur >= rf->rf_mindur) && (re.re_dur <= rf->rf_maxdur)) {
                                        /* The above check is probably not necessary */
                                        deltaT = this_ts - rf->rf_dl.dl_last_ts;
                                        if (deltaT < 0)
                                                deltaT = (int64_t) ((DFS_TSF_WRAP - rf->rf_dl.dl_last_ts) + this_ts +1);
                                        if ((deltaT < rf->rf_minpri) && (deltaT != 0)) {
                                                /* Second line of PRI filtering. */
                                                DFS_DPRINTK(sc, ATH_DEBUG_DFS2,
                                                "filterID %d : Rejecting on individual filter min PRI deltaT=%lld rf->rf_minpri=%u\n",
                                                rf->rf_pulseid, (unsigned long long)deltaT, rf->rf_minpri);
                                                continue;
                                        }

                                        if ((rf->rf_patterntype==2) && (deltaT > rf->rf_maxpri) ) {
                                                DFS_DPRINTK(sc, ATH_DEBUG_DFS2,
                                                "filterID %d : Staggered - Rejecting on individual filter max PRI deltaT=%lld rf->rf_maxpri=%u\n",
                                                rf->rf_pulseid, (unsigned long long)deltaT, rf->rf_maxpri);
                                                /* But update the last time stamp */
                                                rf->rf_dl.dl_last_ts = this_ts;
                                                continue;
                                        }

                                        if ((rf->rf_patterntype!= 2) && (deltaT > rf->rf_maxpri) && (deltaT < (2*rf->rf_minpri))) {
                                                DFS_DPRINTK(sc, ATH_DEBUG_DFS2,
                                                "filterID %d : Rejecting on individual filter max PRI deltaT=%lld rf->rf_minpri=%u\n",
                                                rf->rf_pulseid, (unsigned long long)deltaT, rf->rf_minpri);
                                                /* But update the last time stamp */
                                                rf->rf_dl.dl_last_ts = this_ts;
                                                continue;
                                        }
                                        dfs_add_pulse(sc, rf, &re, deltaT);

                                        if (rf->rf_patterntype == 2)
                                            found = dfs_staggered_check(sc, rf, (u_int32_t) deltaT, re.re_dur);
                                       else
                                         found = dfs_bin_check(sc, rf, (u_int32_t) deltaT, re.re_dur);
                                        if ((sc->sc_debug) & ATH_DEBUG_DFS2) {
                                                dfs_print_delayline(sc, &rf->rf_dl);
                                        }
                                        rf->rf_dl.dl_last_ts = this_ts;
                                }
                            } 
			ft->ft_last_ts = this_ts;
			retval |= found;
			if (found) {
				DFS_DPRINTK(sc, ATH_DEBUG_DFS1,
					"Found on channel minDur = %d, filterId = %d\n",ft->ft_mindur,
					rf != NULL ? rf->rf_pulseid : -1);
                        }
			tabledepth++;
		}
		ATH_DFSQ_LOCK(dfs);
		empty = STAILQ_EMPTY(&(dfs->dfs_radarq));
		ATH_DFSQ_UNLOCK(dfs);
	}
dfsfound:
	if (retval) {
                /* Collect stats */
                dfs->ath_dfs_stats.num_radar_detects++;
		thischan = &rs->rs_chan;
		if (dfs->dfs_rinfo.rn_use_nol) rs->rs_chan.privFlags |= CHANNEL_INTERFERENCE;
		DFS_DPRINTK(sc, ATH_DEBUG_DFS, "Found on channel %d\n", thischan->channel);
                /* If radar is detected in 40MHz mode, add both the primary and the 
                   extension channels to the NOL. chan is the channel data we return 
                   to the ath_dev layer which passes it on to the 80211 layer. 
                   As we want the AP to change channels and send out a CSA, 
                   we always pass back the primary channel data to the ath_dev layer.*/
                chan->channel = sc->sc_curchan.channel;
		chan->channelFlags = sc->sc_curchan.channelFlags;
		chan->privFlags  = sc->sc_curchan.privFlags;
		chan->maxRegTxPower = sc->sc_curchan.maxRegTxPower;
		chan->privFlags |= CHANNEL_INTERFERENCE;

		if ((dfs->dfs_rinfo.rn_use_nol) &&
		    (sc->sc_opmode == HAL_M_HOSTAP)) {
                        HAL_CHANNEL *ext_ch;
			/* If HT40 mode, always add both primary and extension channels 
                           to the NOL irrespective of where radar was detected */    
			sc->sc_curchan.privFlags |= CHANNEL_INTERFERENCE;
			    dfs_nol_addchan(sc, &sc->sc_curchan);

	                ext_ch=ath_hal_get_extension_channel(ah);
                        if (ext_ch) {
			            sc->sc_extchan.privFlags |= CHANNEL_INTERFERENCE;
        			    dfs_nol_addchan(sc, &sc->sc_extchan);
                        } 
                }
		/* Disable radar for now */
		rfilt = ath_hal_getrxfilter(ah);
		rfilt &= ~HAL_RX_FILTER_PHYRADAR;
		ath_hal_setrxfilter(ah, rfilt);

		dfs_reset_radarq(sc);
		dfs_reset_alldelaylines(sc);
		/* XXX Should we really enable again? Maybe not... */
		pe.pe_firpwr = rs->rs_firpwr;
		pe.pe_rrssi = rs->rs_radarrssi;
		pe.pe_height = rs->rs_height;
		pe.pe_prssi = rs->rs_pulserssi;
		pe.pe_inband = rs->rs_inband;
		/* 5413 specific */
		pe.pe_relpwr = rs->rs_relpwr;
		pe.pe_relstep = rs->rs_relstep;
		pe.pe_maxlen = rs->rs_maxlen;

		ath_hal_enabledfs(ah, &pe);
		rfilt |= HAL_RX_FILTER_PHYRADAR;
		ath_hal_setrxfilter(ah, rfilt);

                DFS_DPRINTK(sc, ATH_DEBUG_DFS1, "Primary channel freq = %u flags=0x%x\n", chan->channel, chan->privFlags);
                if ((sc->sc_curchan.channel != thischan->channel)) {
                    DFS_DPRINTK(sc, ATH_DEBUG_DFS1, "Ext channel freq = %u flags=0x%x\n", thischan->channel, thischan->privFlags);
                }
	}
	return retval;
}
u_int32_t
dfs_check_nol(struct ath_softc *sc, HAL_CHANNEL *chans, u_int32_t nchans, u_int16_t delfreq)
{
	struct ath_dfs *dfs=sc->sc_dfs;
	struct dfs_nolelem *nol;
	u_int32_t free_index=0;
	struct dfs_state *rs=NULL;
	int i;

	if (dfs == NULL) {
		DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s: sc_dfs is NULL\n",
			__func__);
		return 0;
	}
	
	nol = dfs->dfs_nol;
	if (nol != NULL) {
	    nol->nol_chan.privFlags &= ~CHANNEL_INTERFERENCE;
	    if (free_index < nchans) {
	        chans[free_index].channel = nol->nol_chan.channel;
	        chans[free_index].channelFlags = nol->nol_chan.channelFlags;
	        chans[free_index].privFlags = nol->nol_chan.privFlags;
	        free_index++;
	    }
	    ath_hal_checkdfs(sc->sc_ah, &nol->nol_chan);
	    DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s removing channel %d from NOL tstamp=%d\n", 
			__func__, nol->nol_chan.channel, CONVERT_SYSTEM_TIME_TO_SEC(OS_GET_TIMESTAMP()));
            OS_CANCEL_TIMER(&nol->nol_timer);
            dfs->dfs_nol = nol->nol_next;
            for (i=0;i<64; i++)
            {
                    rs = &dfs->dfs_radar[i];
                    if (rs->rs_chan.channel == nol->nol_chan.channel) 
                    {
                            rs->rs_chan.privFlags &=~CHANNEL_INTERFERENCE;
                            break;
                    }
            }
            OS_FREE(nol);
            nol=NULL;
        }

	return free_index;
}

/* This function Initialize the radar filter tables 
 * if the ath dfs domain is uninitalized or
 *    ath dfs domain is different from hal dfs domain
 */
int dfs_init_radar_filters( struct ath_softc *sc )
{
    u_int32_t T, Tmax, haldfsdomain;
    int numpulses,p,n, i;
    int numradars = 0, numb5radars = 0;
    struct ath_dfs *dfs = sc->sc_dfs;
    struct dfs_filtertype *ft = NULL;
    struct dfs_filter *rf=NULL;
    struct dfs_pulse *dfs_radars;
    struct dfs_bin5pulse *b5pulses=NULL;
    int32_t min_rssithresh=DFS_MAX_RSSI_VALUE;
    u_int32_t max_pulsedur=0;

    if ( dfs == NULL ) {
        DFS_DPRINTK(sc, ATH_DEBUG_DFS, "dfs is NULL %s",__func__);
    	return 1;
    }

    if (ath_hal_getcapability(sc->sc_ah, HAL_CAP_PHYDIAG,
				  HAL_CAP_RADAR, NULL) == HAL_OK) {
    /* Init DFS radar filters */
    ath_hal_getcapability(sc->sc_ah, HAL_CAP_DFS_DMN, 0, &haldfsdomain);

    if ( (dfs->dfsdomain == DFS_UNINIT_DOMAIN) ||
	 ( dfs->dfsdomain != haldfsdomain ) )
    {
            /* set the dfs domain from HAL */
            dfs->dfsdomain = haldfsdomain;
	    DFS_DPRINTK(sc, ATH_DEBUG_DFS2, "DFS domain=%d\n",dfs->dfsdomain);
	    if (domainoverride != DFS_UNINIT_DOMAIN) {
		DFS_DPRINTK(sc, ATH_DEBUG_DFS, 
			   "Overriding DFS domain with %d\n",domainoverride); 
		dfs->dfsdomain = domainoverride;
	    }
	    dfs_radars = ath_hal_getdfsradars(sc->sc_ah, dfs->dfsdomain,
			     &numradars, &b5pulses, &numb5radars,
			     &(dfs->dfs_defaultparams));
	    if (dfs_radars == NULL) {
		DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s: Unknown dfs domain %d\n",
			    __func__, dfs->dfsdomain);
		/* Disable radar detection since we don't have a radar domain */
		dfs->dfs_proc_phyerr &= ~DFS_RADAR_EN;
		return 0;
	    }
	    dfs->dfs_rinfo.rn_numradars = 0;
	    /* Clear filter type table */
	    for (n=0; n<256; n++) {
		for (i=0;i<DFS_MAX_RADAR_OVERLAP; i++)
		    (dfs->dfs_radartable[n])[i] = -1;
            }
	    /* Now, initialize the radar filters */
	    for (p=0; p<numradars; p++) {
		ft = NULL;
		for (n=0; n<dfs->dfs_rinfo.rn_numradars; n++) {
		    if ((dfs_radars[p].rp_pulsedur == dfs->dfs_radarf[n]->ft_filterdur) &&
		       (dfs_radars[p].rp_numpulses == dfs->dfs_radarf[n]->ft_numpulses) &&
		       (dfs_radars[p].rp_mindur == dfs->dfs_radarf[n]->ft_mindur) &&
		       (dfs_radars[p].rp_maxdur == dfs->dfs_radarf[n]->ft_maxdur)) {
		       ft = dfs->dfs_radarf[n];
		       break;
		    }
		}
		if (ft == NULL) {
		    /* No filter of the appropriate dur was found */
		    if ((dfs->dfs_rinfo.rn_numradars+1) >DFS_MAX_RADAR_TYPES) {
			DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s: Too many filter types\n",
			__func__);
			goto bad4;
		    }
		    ft = dfs->dfs_radarf[dfs->dfs_rinfo.rn_numradars];
		    ft->ft_numfilters = 0;
		    ft->ft_numpulses = dfs_radars[p].rp_numpulses;
		    ft->ft_patterntype = dfs_radars[p].rp_patterntype;
		    ft->ft_mindur = dfs_radars[p].rp_mindur;
		    ft->ft_maxdur = dfs_radars[p].rp_maxdur;
		    ft->ft_filterdur = dfs_radars[p].rp_pulsedur;
		    ft->ft_rssithresh = dfs_radars[p].rp_rssithresh;
		    ft->ft_rssimargin = dfs_radars[p].rp_rssimargin;
		    ft->ft_minpri = 1000000;

		    if (ft->ft_rssithresh < min_rssithresh)
		    min_rssithresh = ft->ft_rssithresh;
		    if (ft->ft_maxdur > max_pulsedur)
		    max_pulsedur = ft->ft_maxdur;
		    for (i=ft->ft_mindur; i<=ft->ft_maxdur; i++) {
			u_int32_t stop=0,tableindex=0;
			while ((tableindex < DFS_MAX_RADAR_OVERLAP) && (!stop)) {
			    if ((dfs->dfs_radartable[i])[tableindex] == -1)
				stop = 1;
			    else
				tableindex++;
			}
			if (stop) {
			    (dfs->dfs_radartable[i])[tableindex] =
				(int8_t) (dfs->dfs_rinfo.rn_numradars);
			} else {
			    DFS_DPRINTK(sc, ATH_DEBUG_DFS,
				"%s: Too many overlapping radar filters\n",
				__func__);
			    goto bad4;
			}
		    }
		    dfs->dfs_rinfo.rn_numradars++;
		}
		rf = &(ft->ft_filters[ft->ft_numfilters++]);
		dfs_reset_delayline(&rf->rf_dl);
		numpulses = dfs_radars[p].rp_numpulses;

		rf->rf_numpulses = numpulses;
		rf->rf_patterntype = dfs_radars[p].rp_patterntype;
		rf->rf_pulseid = dfs_radars[p].rp_pulseid;
		rf->rf_mindur = dfs_radars[p].rp_mindur;
		rf->rf_maxdur = dfs_radars[p].rp_maxdur;
		rf->rf_numpulses = dfs_radars[p].rp_numpulses;

		T = (100000000/dfs_radars[p].rp_max_pulsefreq) -
        		100*(dfs_radars[p].rp_meanoffset);
                rf->rf_minpri =
		        _round((int32_t)T - (100*(dfs_radars[p].rp_pulsevar)));
		Tmax = (100000000/dfs_radars[p].rp_pulsefreq) -
		        100*(dfs_radars[p].rp_meanoffset);
        	rf->rf_maxpri =
	        	_round((int32_t)Tmax + (100*(dfs_radars[p].rp_pulsevar)));

	        if( rf->rf_minpri < ft->ft_minpri )
		    ft->ft_minpri = rf->rf_minpri;

		rf->rf_threshold = dfs_radars[p].rp_threshold;
		rf->rf_filterlen = rf->rf_maxpri * rf->rf_numpulses;

		DFS_DPRINTK(sc, ATH_DEBUG_DFS2, "minprf = %d maxprf = %d pulsevar = %d thresh=%d\n",
		dfs_radars[p].rp_pulsefreq, dfs_radars[p].rp_max_pulsefreq, dfs_radars[p].rp_pulsevar, rf->rf_threshold);
		DFS_DPRINTK(sc, ATH_DEBUG_DFS2,
		"minpri = %d maxpri = %d filterlen = %d filterID = %d\n",
		rf->rf_minpri, rf->rf_maxpri, rf->rf_filterlen, rf->rf_pulseid);
	    }

#ifdef DFS_DEBUG
	    dfs_print_filters(sc);
#endif
	    dfs->dfs_rinfo.rn_numbin5radars  = numb5radars;
	    if ( dfs->dfs_b5radars == NULL ) {
		dfs->dfs_b5radars = (struct dfs_bin5radars *)OS_MALLOC(sc->sc_osdev, numb5radars * sizeof(struct dfs_bin5radars),
		GFP_KERNEL);
		if (dfs->dfs_b5radars == NULL) {
		    DFS_DPRINTK(sc, ATH_DEBUG_DFS, 
		    "%s: cannot allocate memory for bin5 radars\n",
		    __func__);
		    goto bad4;
		}
	    }
	    for (n=0; n<numb5radars; n++) {
		dfs->dfs_b5radars[n].br_pulse = b5pulses[n];
		dfs->dfs_b5radars[n].br_pulse.b5_timewindow *= 1000000;
		if (dfs->dfs_b5radars[n].br_pulse.b5_rssithresh < min_rssithresh)
		min_rssithresh = dfs->dfs_b5radars[n].br_pulse.b5_rssithresh;
		if (dfs->dfs_b5radars[n].br_pulse.b5_maxdur > max_pulsedur )
		max_pulsedur = dfs->dfs_b5radars[n].br_pulse.b5_maxdur;
	    }
	    dfs_reset_alldelaylines(sc);
	    dfs_reset_radarq(sc);
	    dfs->dfs_curchan_radindex = -1;
	    dfs->dfs_extchan_radindex = -1;
	    dfs->dfs_rinfo.rn_minrssithresh = min_rssithresh;
	    /* Convert durations to TSF ticks */
	    dfs->dfs_rinfo.rn_maxpulsedur = _round((int32_t)((max_pulsedur*100/80)*100));
	    DFS_DPRINTK (sc, ATH_DEBUG_DFS, "DFS min filter rssiThresh = %d\n",min_rssithresh);
	    DFS_DPRINTK (sc, ATH_DEBUG_DFS, "DFS max pulse dur = %d ticks\n", dfs->dfs_rinfo.rn_maxpulsedur);
	    return 0;

	bad4:
	     for (n=0; n<ft->ft_numfilters; n++) {
		 rf = &(ft->ft_filters[n]);
	     }
	     return 1;
        }
     }
     return 0;
}

void
dfs_clear_stats(struct ath_softc *sc)
{
    struct ath_dfs *dfs = sc->sc_dfs;
    if (dfs == NULL) 
            return;
    OS_MEMZERO(&dfs->ath_dfs_stats, sizeof (struct dfs_stats));
    dfs->ath_dfs_stats.last_reset_tstamp = ath_hal_gettsf64(sc->sc_ah);
}

int
dfs_attach(struct ath_softc *sc)
{
	int i, n;
	struct ath_dfs *dfs = sc->sc_dfs;
#define	N(a)	(sizeof(a)/sizeof(a[0]))
        struct ieee80211com *ic = (struct ieee80211com *)sc->sc_ieee; /* XXX */

	if (dfs != NULL) {
		DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s: sc_dfs was not NULL\n",
			__func__);
		return 1;
	}
	dfs = (struct ath_dfs *)OS_MALLOC(sc->sc_osdev, sizeof(struct ath_dfs), GFP_KERNEL);
	if (dfs == NULL) {
		DFS_DPRINTK(sc, ATH_DEBUG_DFS,
			"%s: ath_dfs allocation failed\n", __func__);
		return 1;
	}

	OS_MEMZERO(dfs, sizeof (struct ath_dfs));
        sc->sc_dfs = dfs;
        dfs->dfs_nol=NULL;
        dfs_clear_stats(sc);

        /* Get capability information - can extension channel radar be detected and should we use combined radar RSSI or not.*/
        if (ath_hal_getcapability(sc->sc_ah, HAL_CAP_COMBINED_RADAR_RSSI, 0, 0) 
                                   == HAL_OK) {
                sc->sc_dfs->sc_dfs_combined_rssi_ok = 1;
        } else {
                sc->sc_dfs->sc_dfs_combined_rssi_ok = 0;
        }
        if (ath_hal_getcapability(sc->sc_ah, HAL_CAP_EXT_CHAN_DFS, 0, 0) 
                                    == HAL_OK) {
            sc->sc_dfs->sc_dfs_ext_chan_ok = 1;
        } else {
            sc->sc_dfs->sc_dfs_ext_chan_ok = 0;
        }

        sc->sc_dfs->sc_dfs_cac_time = ATH_DFS_WAIT_MS;
        sc->sc_dfs->sc_dfstesttime = ATH_DFS_TEST_RETURN_PERIOD_MS;
	ATH_DFSQ_LOCK_INIT(dfs);
	STAILQ_INIT(&dfs->dfs_radarq);
	ATH_ARQ_LOCK_INIT(dfs);
	STAILQ_INIT(&dfs->dfs_arq);
	STAILQ_INIT(&(dfs->dfs_eventq));
	ATH_DFSEVENTQ_LOCK_INIT(dfs);
	dfs->events = (struct dfs_event *)OS_MALLOC(sc->sc_osdev,
                       sizeof(struct dfs_event)*DFS_MAX_EVENTS,
                       GFP_KERNEL);
	if (dfs->events == NULL) {
		OS_FREE(dfs);
                sc->sc_dfs = NULL;
		DFS_DPRINTK(sc, ATH_DEBUG_DFS,
			"%s: events allocation failed\n", __func__);
		return 1;
	}
	for (i=0; i<DFS_MAX_EVENTS; i++) {
		STAILQ_INSERT_TAIL(&(dfs->dfs_eventq), &dfs->events[i], re_list);
	}

        dfs->pulses = (struct dfs_pulseline *)OS_MALLOC(sc->sc_osdev, sizeof(struct dfs_pulseline), GFP_KERNEL);

        if (dfs->pulses == NULL) {
                OS_FREE(dfs->events);   
                dfs->events = NULL;
                OS_FREE(dfs);
                sc->sc_dfs = NULL;
                DFS_DPRINTK(sc, ATH_DEBUG_DFS,
                        "%s: pulse buffer allocation failed\n", __func__);
                return 1;
        }

        dfs->pulses->pl_lastelem = DFS_MAX_PULSE_BUFFER_MASK;
#ifdef ATH_ENABLE_AR
	if (ath_hal_getcapability(sc->sc_ah, HAL_CAP_PHYDIAG,
				  HAL_CAP_AR, NULL) == HAL_OK) {
		dfs_reset_ar(sc);
		dfs_reset_arq(sc);
		dfs->dfs_proc_phyerr |= DFS_AR_EN;
	}
#endif
	if (ath_hal_getcapability(sc->sc_ah, HAL_CAP_PHYDIAG,
				  HAL_CAP_RADAR, NULL) == HAL_OK) {
		u_int32_t val;
		/* 
		 * If we have fast diversity capability, read off
		 * Strong Signal fast diversity count set in the ini
		 * file, and store so we can restore the value when
		 * radar is disabled
		 */
		if (ath_hal_getcapability(sc->sc_ah, HAL_CAP_DIVERSITY, HAL_CAP_STRONG_DIV,
					  &val) == HAL_OK) {
			dfs->dfs_rinfo.rn_fastdivGCval = val;
		}
		ic->ic_isdfsregdomain = 1;
		dfs->dfs_proc_phyerr |= DFS_RADAR_EN;

                /* Allocate memory for radar filters */
		for (n=0; n<DFS_MAX_RADAR_TYPES; n++) {
			dfs->dfs_radarf[n] = (struct dfs_filtertype *)OS_MALLOC(sc->sc_osdev, sizeof(struct dfs_filtertype),GFP_KERNEL);
			if (dfs->dfs_radarf[n] == NULL) {
				DFS_DPRINTK(sc,ATH_DEBUG_DFS,
					"%s: cannot allocate memory for radar filter types\n",
					__func__);
				goto bad1;
			}
		}
                /* Allocate memory for radar table */
		dfs->dfs_radartable = (int8_t **)OS_MALLOC(sc->sc_osdev, 256*sizeof(int8_t *), GFP_KERNEL);
		if (dfs->dfs_radartable == NULL) {
			DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s: cannot allocate memory for radar table\n",
				__func__);
			goto bad1;
		}
		for (n=0; n<256; n++) {
			dfs->dfs_radartable[n] = OS_MALLOC(sc->sc_osdev, DFS_MAX_RADAR_OVERLAP*sizeof(int8_t),
							 GFP_KERNEL);
			if (dfs->dfs_radartable[n] == NULL) {
				DFS_DPRINTK(sc, ATH_DEBUG_DFS,
					"%s: cannot allocate memory for radar table entry\n",
					__func__);
				goto bad2;
			}
		}
		if (!usenol) {
			DFS_DPRINTK(sc, ATH_DEBUG_DFS, " %s: Disabling Channel NOL\n", __func__);

                }
		dfs->dfs_rinfo.rn_use_nol = usenol;

		if (dfs->dfs_rinfo.rn_use_nol) {
			ic->ic_flags_ext |= IEEE80211_FEXT_MARKDFS;
		} else {
			ic->ic_flags_ext &= ~IEEE80211_FEXT_MARKDFS;
                }
                dfs->dfs_b5radars = NULL;
                if ( dfs_init_radar_filters( sc ) ) {
			DFS_DPRINTK(sc, ATH_DEBUG_DFS, 
                            " %s: Radar Filter Intialization Failed \n", 
                            __func__);
                    return 1;
                }
	}
	return 0;
bad2:
	OS_FREE(dfs->dfs_radartable);
	dfs->dfs_radartable = NULL;
bad1:	
        for (n=0; n<DFS_MAX_RADAR_TYPES; n++) {
		if (dfs->dfs_radarf[n] != NULL) {
			OS_FREE(dfs->dfs_radarf[n]);
			dfs->dfs_radarf[n] = NULL;
		}
	}
        if (dfs->pulses) {
		OS_FREE(dfs->pulses);
		dfs->pulses = NULL;
	}
	if (dfs->events) {
		OS_FREE(dfs->events);
		dfs->events = NULL;
	}

	if (sc->sc_dfs) {
		OS_FREE(sc->sc_dfs);
		sc->sc_dfs = NULL;
	}
	return 1;
#undef N
}

void
dfs_detach(struct ath_softc *sc)
{
	struct ath_dfs *dfs = sc->sc_dfs;
	int n, empty;
        struct ieee80211com *ic = (struct ieee80211com *)sc->sc_ieee; /* XXX */

	if (dfs == NULL) {
	        DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s: sc_dfs is NULL\n",
			__func__);
		return;
	}
  
        /* Bug 29099 make sure all outstanding timers are cancelled*/

        if (dfs->sc_rtasksched) {
            OS_CANCEL_TIMER(&dfs->sc_dfs_task_timer);
            dfs->sc_rtasksched = 0;
        }
        if (dfs->sc_dfswait) {
            OS_CANCEL_TIMER(&dfs->sc_dfswaittimer);
            dfs->sc_dfswait = 0;
        }
        if (dfs->sc_dfstest) {
            OS_CANCEL_TIMER(&dfs->sc_dfstesttimer);
            dfs->sc_dfstest = 0;
        }

	if (dfs->dfs_nol != NULL) {
	    struct dfs_nolelem *nol, *next;
	    nol = dfs->dfs_nol;
                /* Bug 29099 - each NOL element has its own timer, cancel it and 
                   free the element*/
		while (nol != NULL) {
                       OS_CANCEL_TIMER(&nol->nol_timer);
		       next = nol->nol_next;
		       OS_FREE(nol);
		       nol = next;
		}
		dfs->dfs_nol = NULL;
	}
        /* Return radar events to free q*/
        dfs_reset_radarq(sc);
	dfs_reset_alldelaylines(sc);

        /* Free up pulse log*/
        if (dfs->pulses != NULL) {
                OS_FREE(dfs->pulses);
                dfs->pulses = NULL;
        }

	for (n=0; n<DFS_MAX_RADAR_TYPES;n++) {
		if (dfs->dfs_radarf[n] != NULL) {
			OS_FREE(dfs->dfs_radarf[n]);
			dfs->dfs_radarf[n] = NULL;
		}
	}


	if (dfs->dfs_radartable != NULL) {
		for (n=0; n<256; n++) {
			if (dfs->dfs_radartable[n] != NULL) {
				OS_FREE(dfs->dfs_radartable[n]);
				dfs->dfs_radartable[n] = NULL;
			}
		}
		OS_FREE(dfs->dfs_radartable);
		dfs->dfs_radartable = NULL;
		ic->ic_isdfsregdomain = 0;
	}
        
	if (dfs->dfs_b5radars != NULL) {
		OS_FREE(dfs->dfs_b5radars);
		dfs->dfs_b5radars=NULL;
	}

	dfs_reset_ar(sc);

	ATH_ARQ_LOCK(dfs);
	empty = STAILQ_EMPTY(&(dfs->dfs_arq));
	ATH_ARQ_UNLOCK(dfs);
	if (!empty) {
		dfs_reset_arq(sc);
	}
        if (dfs->events != NULL) {
                OS_FREE(dfs->events);
                dfs->events = NULL;
        }
	OS_FREE(dfs);
        sc->sc_dfs = NULL;
}

static void
dfs_reset_ar(struct ath_softc *sc)
{
	struct ath_dfs *dfs=sc->sc_dfs;

	if (dfs == NULL) {
		DFS_DPRINTK(sc,ATH_DEBUG_DFS, "%s: sc_dfs is NULL\n",
			__func__);
		return;
	}
	OS_MEMZERO(&dfs->dfs_ar_state, sizeof(dfs->dfs_ar_state));
	dfs->dfs_ar_state.ar_packetthreshold = DFS_AR_PKT_COUNT_THRESH;
	dfs->dfs_ar_state.ar_parthreshold = DFS_AR_ACK_DETECT_PAR_THRESH;
}

static void
dfs_reset_arq(struct ath_softc *sc)
{
	struct ath_dfs *dfs = sc->sc_dfs;
	struct dfs_event *event;

	if (dfs == NULL) {
		DFS_DPRINTK(sc,ATH_DEBUG_DFS, "%s: sc_dfs is NULL\n",
			__func__);
		return;
	}
	ATH_ARQ_LOCK(dfs);
	ATH_DFSEVENTQ_LOCK(dfs);
	while (!STAILQ_EMPTY(&(dfs->dfs_arq))) {
		event = STAILQ_FIRST(&(dfs->dfs_arq));
		STAILQ_REMOVE_HEAD(&(dfs->dfs_arq), re_list);
		OS_MEMZERO(event, sizeof(struct dfs_event));
		STAILQ_INSERT_TAIL(&(dfs->dfs_eventq), event, re_list);
	}
	ATH_DFSEVENTQ_UNLOCK(dfs);
	ATH_ARQ_UNLOCK(dfs);
}

void
ath_ar_enable(struct ath_softc *sc)
{
	struct ath_dfs *dfs=sc->sc_dfs;
	struct dfs_ar_state *ar;
	HAL_CHANNEL *chan= &sc->sc_curchan;
	HAL_PHYERR_PARAM pe;
	u_int32_t rfilt=0;

	if (dfs == NULL) {
		DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s: sc_dfs is NULL\n",
			__func__);
		return;
	}

	ar = (struct dfs_ar_state *) &dfs->dfs_ar_state;
	if (dfs->dfs_proc_phyerr & (DFS_RADAR_EN | DFS_AR_EN)) {
		if ((chan->channelFlags & CHANNEL_108G) == CHANNEL_108G) {
			/* We are in turbo G, so enable AR */
			DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s: Enabling AR\n", __func__);
			dfs_reset_ar(sc);
			ar->ar_radarrssi = DFS_AR_RADAR_RSSI_THR;
			rfilt = ath_hal_getrxfilter(sc->sc_ah);
			pe.pe_firpwr = HAL_PHYERR_PARAM_NOVAL;
			pe.pe_height = HAL_PHYERR_PARAM_NOVAL;
			pe.pe_prssi = HAL_PHYERR_PARAM_NOVAL;
			pe.pe_inband = HAL_PHYERR_PARAM_NOVAL;
			pe.pe_relpwr = HAL_PHYERR_PARAM_NOVAL;
			pe.pe_maxlen = HAL_PHYERR_PARAM_NOVAL;
			pe.pe_relstep = HAL_PHYERR_PARAM_NOVAL;
			pe.pe_usefir128 = 0;
			pe.pe_blockradar = 0;
			pe.pe_enmaxrssi = 0;
			pe.pe_rrssi = ar->ar_radarrssi;
			ath_hal_enabledfs(sc->sc_ah, &pe);
			/* Enable strong signal fast antenna diversity since accurate 
			 * 1-2us radar
			 * detection is not important for AR anyways.
			 */
			ath_hal_setcapability(sc->sc_ah, HAL_CAP_DIVERSITY, 
					      HAL_CAP_STRONG_DIV, dfs->dfs_rinfo.rn_fastdivGCval, NULL);
			rfilt |= HAL_RX_FILTER_PHYRADAR;
			ath_hal_setrxfilter(sc->sc_ah, rfilt);
		}
	} else {
		DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s: Disabling AR\n", __func__);
		rfilt = ath_hal_getrxfilter(sc->sc_ah);
		rfilt &= ~HAL_RX_FILTER_PHYRADAR;
		ath_hal_setrxfilter(sc->sc_ah,rfilt);
		/* Enable strong signal fast antenna diversity */
		ath_hal_setcapability(sc->sc_ah, HAL_CAP_DIVERSITY, 
				      HAL_CAP_STRONG_DIV, dfs->dfs_rinfo.rn_fastdivGCval, NULL);

	}
}

int dfs_radar_enable(struct ath_softc *sc)
{
	struct ath_dfs *dfs=sc->sc_dfs;
	u_int32_t rfilt;
	HAL_CHANNEL *chan=&sc->sc_curchan;
	struct ath_hal *ah = sc->sc_ah;
	HAL_CHANNEL *ext_ch=ath_hal_get_extension_channel(ah);

	if (dfs == NULL) {
		DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s: sc_dfs is NULL\n",
			__func__);
		return -EIO;
	}
	rfilt = ath_hal_getrxfilter(ah);
	if ((dfs->dfs_proc_phyerr & (DFS_RADAR_EN | DFS_AR_EN)) &&
	    (sc->sc_opmode == HAL_M_HOSTAP)) {
            	if (chan->privFlags & CHANNEL_DFS) {
			struct dfs_state *rs_pri=NULL, *rs_ext=NULL;
			u_int8_t index_pri, index_ext;
			HAL_PHYERR_PARAM pe;
                        dfs->sc_dfs_cac_time = ATH_DFS_WAIT_MS;

                        if (dfs->dfsdomain == DFS_ETSI_DOMAIN) {
                            if(IS_CHANNEL_WEATHER_RADAR(chan)) {
                                dfs->sc_dfs_cac_time = ATH_DFS_WEATHER_CHANNEL_WAIT_MS;
                            } else {
                                 if (ext_ch && IS_CHANNEL_WEATHER_RADAR(ext_ch)) {
                                     dfs->sc_dfs_cac_time = ATH_DFS_WEATHER_CHANNEL_WAIT_MS;
                                 }
                            }
                        }
                        if(dfs->sc_dfs_cac_time != ATH_DFS_WAIT_MS)
                            printk("WARNING!!! 10 minute CAC period as channel is a weather radar channel\n");
			/*
			  Disable radar detection in case we need to setup
			 * a new channel state and radars are somehow being
			 * reported. Avoids locking problem.
			 */

			rfilt = ath_hal_getrxfilter(ah);
			rfilt &= ~HAL_RX_FILTER_PHYRADAR;
			ath_hal_setrxfilter(ah, rfilt);
			/* Enable strong signal fast antenna diversity */
			ath_hal_setcapability(ah, HAL_CAP_DIVERSITY, 
					      HAL_CAP_STRONG_DIV, dfs->dfs_rinfo.rn_fastdivGCval, NULL);
			dfs_reset_alldelaylines(sc);

			rs_pri = dfs_getchanstate(sc, &index_pri, 0);
			if (ext_ch) {
                            rs_ext = dfs_getchanstate(sc, &index_ext, 1);
                            sc->sc_extchan = *ext_ch;
                        }
    			if (rs_pri != NULL && ((ext_ch==NULL)||(rs_ext != NULL))) {
				if (index_pri != dfs->dfs_curchan_radindex)
					dfs_reset_alldelaylines(sc);

				dfs->dfs_curchan_radindex = (int16_t) index_pri;
                                
                                if (rs_ext)
			            dfs->dfs_extchan_radindex = (int16_t) index_ext;

				pe.pe_firpwr = rs_pri->rs_firpwr;
				pe.pe_rrssi = rs_pri->rs_radarrssi;
				pe.pe_height = rs_pri->rs_height;
				pe.pe_prssi = rs_pri->rs_pulserssi;
				pe.pe_inband = rs_pri->rs_inband;
				/* 5413 specific */
				pe.pe_relpwr = rs_pri->rs_relpwr;
				pe.pe_relstep = rs_pri->rs_relstep;
				pe.pe_maxlen = rs_pri->rs_maxlen;


				/* Disable strong signal fast antenna diversity */
				ath_hal_setcapability(ah, HAL_CAP_DIVERSITY,
						      HAL_CAP_STRONG_DIV, 1, NULL);

				ath_hal_enabledfs(sc->sc_ah, &pe);
				rfilt |= HAL_RX_FILTER_PHYRADAR;
				ath_hal_setrxfilter(ah, rfilt);
				DFS_DPRINTK(sc, ATH_DEBUG_DFS, "Enabled radar detection on channel %d\n",
					chan->channel);

                                dfs->dur_multiplier =  (ath_hal_is_fast_clock_enabled(sc->sc_ah) ? (DFS_FAST_CLOCK_MULTIPLIER) : (DFS_NO_FAST_CLOCK_MULTIPLIER));
                    	DFS_DPRINTK(sc, ATH_DEBUG_DFS3,
			"%s: duration multiplier is %d\n", __func__, dfs->dur_multiplier);

				/*
				 * Fast antenna diversity for strong signals disturbs
				 * radar detection of 1-2us pusles. Enable fast diveristy
				 * but disable the strong signal aspect of it
				 */
				if (ath_hal_getcapability(ah, HAL_CAP_DIVERSITY, 1, NULL) == HAL_OK) {
					ath_hal_setcapability(ah, HAL_CAP_DIVERSITY, 
							      HAL_CAP_STRONG_DIV, 0, NULL);
				}
			} else
				DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s: No more radar states left\n",
					__func__);
		} else {
			if (!(chan->channelFlags & CHANNEL_2GHZ)) {
				/* Enable strong signal fast antenna diversity */
				ath_hal_setcapability(ah, HAL_CAP_DIVERSITY, 
						      HAL_CAP_STRONG_DIV, dfs->dfs_rinfo.rn_fastdivGCval, NULL);
				/* Disable Radar if not 2GHZ channel and not DFS */
				rfilt &= ~HAL_RX_FILTER_PHYRADAR;
				ath_hal_setrxfilter(ah, rfilt);
			}
		}
	} else {
		/* Enable strong signal fast antenna diversity */
		ath_hal_setcapability(ah, HAL_CAP_DIVERSITY, 
				      HAL_CAP_STRONG_DIV, dfs->dfs_rinfo.rn_fastdivGCval, NULL);
		/* Disable Radar if RADAR or AR not enabled */
		rfilt &= ~HAL_RX_FILTER_PHYRADAR;
		ath_hal_setrxfilter(ah, rfilt);
	}
	return 0;
}

/*
 * Disable AR if AR channel
 */

void
ath_ar_disable(struct ath_softc *sc)
{
	u_int32_t rfilt;
	HAL_CHANNEL *chan=&sc->sc_curchan;
	struct ath_dfs *dfs=sc->sc_dfs;
	if (dfs == NULL) {
		DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s: sc_dfs is NULL\n",
			__func__);
		return;
	}
	if ((chan->channelFlags & CHANNEL_108G) == CHANNEL_108G) {
		/* Enable strong signal fast antenna diversity */
		ath_hal_setcapability(sc->sc_ah, HAL_CAP_DIVERSITY, 
				      HAL_CAP_STRONG_DIV, dfs->dfs_rinfo.rn_fastdivGCval, NULL);
		rfilt = ath_hal_getrxfilter(sc->sc_ah);
		rfilt &= ~HAL_RX_FILTER_PHYERR;
		ath_hal_setrxfilter(sc->sc_ah, rfilt);
		dfs_reset_ar(sc);
		dfs_reset_arq(sc);
	}
}

/* Return TRUE if chirping pulse, FALSE if not.
   Decision is made based on processing the FFT data included with the PHY error.
   Calculate the slope using the maximum bin index reported in the FFT data.
   Calculate slope between FFT packet 0 and packet n-1. Also calculate slope between
   packet 1 and packet n. 
   If a pulse is chirping, a slope of 5 and greater is seen.
   Non-chirping pulses have slopes of 0, 1, 2 or 3.   
*/

int dfs_check_chirping (struct ath_softc *sc, struct ath_desc *ds, int is_ctl, int is_ext, int *slope, int *is_dc)
{
#define FFT_LEN 70
#define FFT_LOWER_BIN_MAX_INDEX_BYTE 66
#define FFT_UPPER_BIN_MAX_INDEX_BYTE 69
#define MIN_CHIRPING_SLOPE 4

    int is_chirp=0;
    int p, num_fft_packets=0, datalen;
    int ctl_slope=0, ext_slope=0;
    int ctl_high0, ctl_low0, ctl_slope0=0, ext_high0, ext_low0, ext_slope0=0;
    int ctl_high1, ctl_low1, ctl_slope1=0, ext_high1, ext_low1, ext_slope1=0;
    u_int8_t *fft_data_ptr; 


    *slope = 0;
    *is_dc = 0;

    datalen = ds->ds_rxstat.rs_datalen;
    num_fft_packets = datalen / FFT_LEN;
    fft_data_ptr = ((u_int8_t*)ds->ds_vdata);

    /* DEBUG - Print relevant portions of the FFT data*/
    for (p=0; p < num_fft_packets; p++) {

        DFS_DPRINTK(sc, ATH_DEBUG_DFS3,"fft_data_ptr=0x%p\t", fft_data_ptr);
        DFS_DPRINTK(sc, ATH_DEBUG_DFS3, "[66]=%d [69]=%d\n",*(fft_data_ptr+FFT_LOWER_BIN_MAX_INDEX_BYTE) >> 2, *(fft_data_ptr+FFT_UPPER_BIN_MAX_INDEX_BYTE) >> 2);  

        fft_data_ptr += FFT_LEN;
    }

    DFS_DPRINTK(sc, ATH_DEBUG_DFS3, "datalen=%d num_fft_packets=%d\n", datalen, num_fft_packets);  

    /* There is not enough FFT data to figure out whether the pulse is chirping or not*/
    if (num_fft_packets < 4) {
        return 0;
    }

    fft_data_ptr = ((u_int8_t*)ds->ds_vdata);

    if (is_ctl) {
    
        fft_data_ptr = ((u_int8_t*)ds->ds_vdata);
        ctl_low0 = *(fft_data_ptr+FFT_LOWER_BIN_MAX_INDEX_BYTE) >>  2;
        fft_data_ptr += FFT_LEN;
        ctl_low1 = *(fft_data_ptr+FFT_LOWER_BIN_MAX_INDEX_BYTE) >>  2;

        // last packet with first packet
        fft_data_ptr = ((u_int8_t*)ds->ds_vdata) + (FFT_LEN*(num_fft_packets - 1));
        ctl_high1 = *(fft_data_ptr+FFT_LOWER_BIN_MAX_INDEX_BYTE) >> 2;

        // second last packet with 0th packet
        fft_data_ptr = ((u_int8_t*)ds->ds_vdata) + (FFT_LEN*(num_fft_packets - 2));
        ctl_high0 = *(fft_data_ptr+FFT_LOWER_BIN_MAX_INDEX_BYTE) >> 2;

        ctl_slope0 = ctl_high0 - ctl_low0;
        if (ctl_slope0 < 0) ctl_slope0 *= (-1);

        ctl_slope1 = ctl_high1 - ctl_low1;
        if (ctl_slope1 < 0) ctl_slope1 *= (-1);

        ctl_slope = ((ctl_slope0 > ctl_slope1) ? ctl_slope0: ctl_slope1);
        *slope = ctl_slope;
        DFS_DPRINTK(sc, ATH_DEBUG_DFS3, "ctl_slope0=%d ctl_slope1=%d ctl_slope=%d\n",
            ctl_slope0, ctl_slope1, ctl_slope);

    } else if (is_ext) {

        fft_data_ptr = ((u_int8_t*)ds->ds_vdata);
        ext_low0 = *(fft_data_ptr+FFT_UPPER_BIN_MAX_INDEX_BYTE) >>  2;

        fft_data_ptr += FFT_LEN;
        ext_low1 = *(fft_data_ptr+FFT_UPPER_BIN_MAX_INDEX_BYTE) >>  2;

        fft_data_ptr = ((u_int8_t*)ds->ds_vdata) + (FFT_LEN*(num_fft_packets - 1));
        ext_high1 = *(fft_data_ptr+FFT_UPPER_BIN_MAX_INDEX_BYTE) >> 2;
        fft_data_ptr = ((u_int8_t*)ds->ds_vdata) + (FFT_LEN*(num_fft_packets - 2));
        ext_high0 = *(fft_data_ptr+FFT_UPPER_BIN_MAX_INDEX_BYTE) >> 2;

        ext_slope0 = ext_high0 - ext_low0;
        if (ext_slope0 < 0) ext_slope0 *= (-1);

        ext_slope1 = ext_high1 - ext_low1;
        if (ext_slope1 < 0) ext_slope1 *= (-1);

        ext_slope = ((ext_slope0 > ext_slope1) ? ext_slope0: ext_slope1);
        *slope = ext_slope;
        DFS_DPRINTK(sc, ATH_DEBUG_DFS2, "ext_slope0=%d ext_slope1=%d ext_slope=%d\n",
            ext_slope0, ext_slope1, ext_slope);
    } else 
        return 0;

    if ((ctl_slope >= MIN_CHIRPING_SLOPE) || (ext_slope >= MIN_CHIRPING_SLOPE)) {
        is_chirp = 1;
        DFS_DPRINTK(sc, ATH_DEBUG_DFS3, "is_chirp=%d is_dc=%d\n", is_chirp, *is_dc);
    }
    return is_chirp;

#undef FFT_LEN
#undef FFT_LOWER_BIN_MAX_INDEX_BYTE 
#undef FFT_UPPER_BIN_MAX_INDEX_BYTE
#undef MIN_CHIRPING_SLOPE
}
void
ath_process_phyerr(struct ath_softc *sc, struct ath_desc *ds, u_int64_t fulltsf)
{
#define EXT_CH_RADAR_FOUND 0x02
#define PRI_CH_RADAR_FOUND 0x01
#define EXT_CH_RADAR_EARLY_FOUND 0x04
        struct ath_dfs *dfs=sc->sc_dfs;
	HAL_CHANNEL *chan=&sc->sc_curchan;
	struct dfs_event *event;
	u_int8_t rssi;
	u_int8_t ext_rssi=0;
        u_int8_t pulse_bw_info=0, pulse_length_ext=0, pulse_length_pri=0;
	u_int32_t dur=0;
        u_int16_t datalen;
        int pri_found=1, ext_found=0, dc_found=0, early_ext=0, slope=0, add_dur=0;

	int empty;
        u_int32_t *last_word_ptr, *secondlast_word_ptr;
        u_int8_t *byte_ptr, last_byte_0, last_byte_1, last_byte_2, last_byte_3; 
        u_int8_t secondlast_byte_0, secondlast_byte_1, secondlast_byte_2, secondlast_byte_3; 

	if ((!(ds->ds_rxstat.rs_phyerr & HAL_PHYERR_RADAR)) &&
	    (!(ds->ds_rxstat.rs_phyerr & HAL_PHYERR_FALSE_RADAR_EXT)))  {
		DFS_DPRINTK(sc, ATH_DEBUG_DFS3, "%s: rs_phyer=0x%x not a radar error\n",__func__, ds->ds_rxstat.rs_phyerr);
        	return;
        }

	if (dfs == NULL) {
		DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s: sc_dfs is NULL\n",__func__);
		return;
	}
        dfs->ath_dfs_stats.total_phy_errors++;
        datalen = ds->ds_rxstat.rs_datalen;
        /* WAR: Never trust combined RSSI on radar pulses for <=
         * OWL2.0. For short pulses only the chain 0 rssi is present
         * and remaining descriptor data is all 0x80, for longer
         * pulses the descriptor is present, but the combined value is
         * inaccurate. This HW capability is queried in dfs_attach and stored in
         * the sc_dfs_combined_rssi_ok flag.*/

        if (sc->sc_dfs->sc_dfs_combined_rssi_ok) {
                rssi = (u_int8_t) ds->ds_rxstat.rs_rssi;
        } else {
            rssi = (u_int8_t) ds->ds_rxstat.rs_rssi_ctl0;
        }

        ext_rssi = (u_int8_t) ds->ds_rxstat.rs_rssi_ext0;

        last_word_ptr = (u_int32_t *)(((u_int8_t*)ds->ds_vdata) + datalen - (datalen%4));
        secondlast_word_ptr = last_word_ptr-1;

        byte_ptr = (u_int8_t*)last_word_ptr; 
        last_byte_0=(*(byte_ptr) & 0xff); 
        last_byte_1=(*(byte_ptr+1) & 0xff); 
        last_byte_2=(*(byte_ptr+2) & 0xff); 
        last_byte_3=(*(byte_ptr+3) & 0xff); 

        byte_ptr = (u_int8_t*)secondlast_word_ptr; 
        secondlast_byte_0=(*(byte_ptr) & 0xff); 
        secondlast_byte_1=(*(byte_ptr+1) & 0xff); 
        secondlast_byte_2=(*(byte_ptr+2) & 0xff); 
        secondlast_byte_3=(*(byte_ptr+3) & 0xff); 
       
        /* If radar can be detected on the extension channel (for SOWL onwards), we have to read radar data differently as the HW supplies bwinfo and duration for both primary and extension channel.*/
        if (sc->sc_dfs->sc_dfs_ext_chan_ok) {
        
        /* If radar can be detected on the extension channel, datalen zero pulses are bogus, discard them.*/
        if (!datalen) {
            dfs->ath_dfs_stats.datalen_discards++;
            return;
        }
        switch((datalen & 0x3)) {
        case 0:
            pulse_bw_info = secondlast_byte_3;
            pulse_length_ext = secondlast_byte_2;
            pulse_length_pri = secondlast_byte_1;
            break;
        case 1:
            pulse_bw_info = last_byte_0;
            pulse_length_ext = secondlast_byte_3;
            pulse_length_pri = secondlast_byte_2;
            break;
        case 2:
            pulse_bw_info = last_byte_1;
            pulse_length_ext = last_byte_0;
            pulse_length_pri = secondlast_byte_3;
           break;
        case 3:
            pulse_bw_info = last_byte_2;
            pulse_length_ext = last_byte_1;
            pulse_length_pri = last_byte_0;
            break;
        default:
            DFS_DPRINTK(sc, ATH_DEBUG_DFS, "datalen mod4=%d\n", (datalen%4));
        }

        /* Only the last 3 bits of the BW info are relevant, they indicate
        which channel the radar was detected in.*/
        pulse_bw_info &= 0x07;
        /* If pulse on DC, both primary and extension flags will be set */
         if (((pulse_bw_info & EXT_CH_RADAR_FOUND) && (pulse_bw_info & PRI_CH_RADAR_FOUND))) {

            /* Conducted testing, when pulse is on DC, both pri and ext durations are reported to be same
               Radiated testing, when pulse is on DC, different pri and ext durations are reported, so take the larger of the two */
            if (pulse_length_ext >= pulse_length_pri) {
                dur = pulse_length_ext;
                ext_found = 1;
            } else {
                dur = pulse_length_pri;
                pri_found = 1;
            }
            dfs->ath_dfs_stats.dc_phy_errors++;         

    } else {
        if (pulse_bw_info & EXT_CH_RADAR_FOUND) {
            dur = pulse_length_ext;
            pri_found = 0;
            ext_found = 1;
            dfs->ath_dfs_stats.ext_phy_errors++;         
        } 
        if (pulse_bw_info & PRI_CH_RADAR_FOUND) {
            dur = pulse_length_pri;
            pri_found = 1;
            ext_found = 0;
            dfs->ath_dfs_stats.pri_phy_errors++;         
        } 
        if (pulse_bw_info & EXT_CH_RADAR_EARLY_FOUND) {
             dur = pulse_length_ext;
             pri_found = 0;
             ext_found = 1; early_ext = 1;
             dfs->ath_dfs_stats.early_ext_phy_errors++;         
	     DFS_DPRINTK(sc, ATH_DEBUG_DFS2, "EARLY ext channel dur=%u rssi=%u datalen=%d\n",dur, rssi, datalen);
        } 
        if (!pulse_bw_info) {
	    DFS_DPRINTK(sc, ATH_DEBUG_DFS3, "ERROR channel dur=%u rssi=%u pulse_bw_info=0x%x datalen MOD 4 = %d\n",dur, rssi, pulse_bw_info, (datalen & 0x3));
            /* Bogus bandwidth info received in descriptor, 
            so ignore this PHY error */
            dfs->ath_dfs_stats.bwinfo_errors++;
            return; 
        }
    }
        /* Always use combined RSSI reported, unless RSSI reported on 
           extension is stronger */
        if ((ext_rssi > rssi) && (ext_rssi < 128)) {
                    rssi = ext_rssi;
        } 

        DFS_DPRINTK(sc, ATH_DEBUG_DFS1, "pulse_bw_info=0x%x pulse_length_ext=%u pulse_length_pri=%u rssi=%u ext_rssi=%u phyerr=0x%x\n", pulse_bw_info, pulse_length_ext, pulse_length_pri, rssi, ext_rssi, ds->ds_rxstat.rs_phyerr);

        /* HW has a known issue with chirping pulses injected at or around DC in 40MHz 
           mode. Such pulses are reported with much lower durations and SW then discards 
           them because they do not fit the minimum bin5 pulse duration.

           To work around this issue, if a pulse is within a 10us range of the 
           bin5 min duration, check if the pulse is chirping. If the pulse is chirping,
           bump up the duration to the minimum bin5 duration. 

           This makes sure that a valid chirping pulse will not be discarded because of 
           incorrect low duration.

            TBD - Is it possible to calculate the 'real' duration of the pulse using the 
            slope of the FFT data?
            TBD - Use FFT data to differentiate between radar pulses and false PHY errors.
            This will let us reduce the number of false alarms seen.
        */
        if (dur >= MAYBE_BIN5_DUR && dur < MIN_BIN5_DUR) {
            add_dur = dfs_check_chirping(sc, ds, pri_found, ext_found, &slope, &dc_found);
            if (add_dur) {
                DFS_DPRINTK(sc, ATH_DEBUG_DFS2, "old dur %d slope =%d\n", dur, slope);
                    // bump up to the mimimum bin5 pulse duration
                    if (dur < MIN_BIN5_DUR) {
                        dur = MIN_BIN5_DUR;
                    }
                DFS_DPRINTK(sc, ATH_DEBUG_DFS2, "new dur %d\n", dur);
            }
        }
      } else {
        dfs->ath_dfs_stats.owl_phy_errors++;
     /* HW cannot detect extension channel radar so it only passes us primary channel radar data*/
            dur = (ds->ds_rxstat.rs_datalen && ds->ds_vdata != NULL ?
               (u_int32_t)(*((u_int8_t *) ds->ds_vdata)) : 0) & 0xff;

            if ((rssi == 0) && (dur== 0)){
               return;
            }
            pri_found = 1;
            ext_found = 0;
        }

	ATH_DFSEVENTQ_LOCK(dfs);
	empty = STAILQ_EMPTY(&(dfs->dfs_eventq));
	ATH_DFSEVENTQ_UNLOCK(dfs);
	if (empty) {
		return;
        }

	if ((chan->channelFlags & CHANNEL_108G) == CHANNEL_108G) {
	        if (!(dfs->dfs_proc_phyerr & DFS_AR_EN)) {
                		DFS_DPRINTK(sc, ATH_DEBUG_DFS2, "%s: DFS_AR_EN not enabled\n",
				__func__);
                                return;
                }
		ATH_DFSEVENTQ_LOCK(dfs);
		event = STAILQ_FIRST(&(dfs->dfs_eventq));
		if (event == NULL) {
			ATH_DFSEVENTQ_UNLOCK(dfs);
			DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s: no more events space left\n",
				__func__);
			return;
		}
		STAILQ_REMOVE_HEAD(&(dfs->dfs_eventq), re_list);
		ATH_DFSEVENTQ_UNLOCK(dfs);
		event->re_rssi = rssi;
		event->re_dur = dur;
		event->re_full_ts = fulltsf;
		event->re_ts = (ds->ds_rxstat.rs_tstamp) & DFS_TSMASK;
        	event->re_chanindex = dfs->dfs_curchan_radindex;
		ATH_ARQ_LOCK(dfs);
		STAILQ_INSERT_TAIL(&(dfs->dfs_arq), event, re_list);
		ATH_ARQ_UNLOCK(dfs);
	}
	else {
		if (chan->privFlags & CHANNEL_DFS) {
	                if (!(dfs->dfs_proc_phyerr & DFS_RADAR_EN)) {
                		DFS_DPRINTK(sc, ATH_DEBUG_DFS3, "%s: DFS_RADAR_EN not enabled\n",
				__func__);
                                return;
                        }
                        /* rssi is not accurate for short pulses, so do not filter based on that for short duration pulses*/
                        if (sc->sc_dfs->sc_dfs_ext_chan_ok) {
			    if ((rssi < dfs->dfs_rinfo.rn_minrssithresh && (dur > 4))||
			        dur > (dfs->dfs_rinfo.rn_maxpulsedur) ) {
                                    dfs->ath_dfs_stats.rssi_discards++;
				    return;
                            }
                        } else {

			    if (rssi < dfs->dfs_rinfo.rn_minrssithresh ||
			        dur > dfs->dfs_rinfo.rn_maxpulsedur) {
                                    dfs->ath_dfs_stats.rssi_discards++;
				    return;
                            }
                        }

			ATH_DFSEVENTQ_LOCK(dfs);
			event = STAILQ_FIRST(&(dfs->dfs_eventq));
			if (event == NULL) {
				ATH_DFSEVENTQ_UNLOCK(dfs);
				DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s: no more events space left\n",
					__func__);
				return;
			}
			STAILQ_REMOVE_HEAD(&(dfs->dfs_eventq), re_list);
			ATH_DFSEVENTQ_UNLOCK(dfs);
			event->re_dur = dur;
			event->re_full_ts = fulltsf;
			event->re_ts = (ds->ds_rxstat.rs_tstamp) & DFS_TSMASK;
			event->re_rssi = rssi;
                        if (pri_found == 1) {
        		    event->re_chanindex = dfs->dfs_curchan_radindex;
                        } else {
                            if (dfs->dfs_extchan_radindex == -1) { 
                                DFS_DPRINTK(sc, ATH_DEBUG_DFS3, "%s - phyerr on ext channel\n", __func__);
                            }
        		    event->re_chanindex = dfs->dfs_extchan_radindex;
                        }
			ATH_DFSQ_LOCK(dfs);
			STAILQ_INSERT_TAIL(&(dfs->dfs_radarq), event, re_list);
			ATH_DFSQ_UNLOCK(dfs);
		}
    }
#undef EXT_CH_RADAR_FOUND
#undef PRI_CH_RADAR_FOUND
#undef EXT_CH_RADAR_EARLY_FOUND
}
#define UPDATE_TOP_THREE_PEAKS(_histo, _peakPtrList, _currWidth) { \
	if ((_histo)[(_peakPtrList)[0]] < (_histo)[(_currWidth)]) {	\
		(_peakPtrList)[2] = (_currWidth != (_peakPtrList)[1]) ?	\
					(_peakPtrList)[1] : (_peakPtrList)[2];  \
		(_peakPtrList)[1] = (_peakPtrList)[0]; \
		(_peakPtrList)[0] = (_currWidth); \
	} else if ((_currWidth != (_peakPtrList)[0])	\
			&& ((_histo)[(_peakPtrList)[1]] < (_histo)[(_currWidth)])) { \
		(_peakPtrList)[2] = (_peakPtrList)[1]; \
		(_peakPtrList)[1] = (_currWidth);      \
	} else if ((_currWidth != (_peakPtrList)[1])   \
			&& (_currWidth != (_peakPtrList)[0])  \
			&& ((_histo)[(_peakPtrList)[2]] < (_histo)[(_currWidth)])) { \
		(_peakPtrList)[2] = (_currWidth);  \
	} \
}

/*
 * This routine builds the histogram based on radar duration and does pattern matching
 * on incoming radars to determine if neighboring traffic is present.
 */

void
dfs_process_ar_event(struct ath_softc *sc, HAL_CHANNEL *chan)
{
	struct ath_dfs *dfs=sc->sc_dfs;
	struct dfs_ar_state *ar;
	struct dfs_event *re=NULL;
	u_int32_t sumpeak=0,numpeaks,rssi,width,origregionsum=0, i;
	u_int16_t thistimestamp;
	int empty;

	if (dfs == NULL) {
		DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s: sc_dfs is NULL\n",
			__func__);
		return;
	}
	ar = (struct dfs_ar_state *) &(dfs->dfs_ar_state);
	ATH_ARQ_LOCK(dfs);
	empty = STAILQ_EMPTY(&(dfs->dfs_arq));
	ATH_ARQ_UNLOCK(dfs);
	while (!empty) {
		ATH_ARQ_LOCK(dfs);
		re = STAILQ_FIRST(&(dfs->dfs_arq));
		if (re != NULL)
			STAILQ_REMOVE_HEAD(&(dfs->dfs_arq), re_list);
		ATH_ARQ_UNLOCK(dfs);
		if (re == NULL)
			return;

		thistimestamp = re->re_ts;
		rssi = re->re_rssi;
		width = re->re_dur;

		/* Return the dfs event to the free event list */
		OS_MEMZERO(re, sizeof(struct dfs_event));
		ATH_DFSEVENTQ_LOCK(dfs);
		STAILQ_INSERT_TAIL(&(dfs->dfs_eventq), re, re_list);
		ATH_DFSEVENTQ_UNLOCK(dfs);

		/* determine if current radar is an extension of previous radar */
		if (ar->ar_prevwidth == 255) {
			/* tag on previous width for consideraion of low data rate ACKs */
			ar->ar_prevwidth += width;
			width = (width == 255) ? 255 : ar->ar_prevwidth;
		} else if ((width == 255) &&
			   (ar->ar_prevwidth == 510 ||
			    ar->ar_prevwidth == 765 ||
			    ar->ar_prevwidth == 1020)) {
			/* Aggregate up to 5 consecuate max radar widths
			 * to consider 11Mbps long preamble 1500-byte pkts
			 */
			ar->ar_prevwidth += width;
		} else if (ar->ar_prevwidth == 1275 && width != 255) {
			/* Found 5th consecute maxed out radar, reset history */
			width += ar->ar_prevwidth;
			ar->ar_prevwidth = 0;
		} else if (ar->ar_prevwidth > 255) {
			/* Ignore if there are less than 5 consecutive maxed out radars */
			ar->ar_prevwidth = width;
			width = 255;
		} else {
			ar->ar_prevwidth = width;
		}
		/* For ignoring noises with radar duration in ranges of 3-30: AP4x */
		if ((width >= 257 && width <= 278) ||	/* Region 7 - 5.5Mbps (long pre) ACK = 270 = 216 us */
		    (width >= 295 && width <= 325) ||	/* Region 8 - 2Mbps (long pre) ACKC = 320 = 256us */
		    (width >= 1280 && width <= 1300)) {
			u_int16_t wraparoundadj=0;
			u_int16_t base = (width >= 1280) ? 1275 : 255;
			if (thistimestamp < ar->ar_prevtimestamp) {
				wraparoundadj = 32768;
			}
			if ((thistimestamp + wraparoundadj - ar->ar_prevtimestamp) !=
			    (width - base)) {
				width = 1;
			}
		}
		if (width <= 10) {
			ATH_ARQ_LOCK(dfs);
			empty = STAILQ_EMPTY(&(dfs->dfs_arq));
			ATH_ARQ_UNLOCK(dfs);
			continue;
		}
		/*
		 * Overloading the width=2 in: Store a count of radars w/max duration
		 * and high RSSI (not noise)
		 */
		if ((width == 255) && (rssi > DFS_AR_RSSI_THRESH_STRONG_PKTS))
			width = 2;
		/*
		 * Overloading the width=3 bin:
		 *   Double and store a count of rdars of durtaion that matches 11Mbps (long preamble)
		 *   TCP ACKs or 1500-byte data packets
		 */
		if ((width >= 1280 && width <= 1300) ||
		    (width >= 318 && width <= 325)) {
			width = 3;
			ar->ar_phyerrcount[3] += 2;
			ar->ar_acksum += 2;
		}
		/* build histogram of radar duration */
		if (width > 0 && width <= 510)
			ar->ar_phyerrcount[width]++;
		else {
			/* invalid radar width, throw it away */
			ATH_ARQ_LOCK(dfs);
			empty = STAILQ_EMPTY(&(dfs->dfs_arq));
			ATH_ARQ_UNLOCK(dfs);
			continue;
		}
		/* Received radar of interest (i.e., signature match), proceed to check if
		 * there is enough neighboring traffic to drop out of Turbo 
		 */
		if ((width >= 33 && width <= 38) ||          /* Region 0: 24Mbps ACK = 35 = 28us */
		    (width >= 39 && width <= 44) ||          /* Region 1: 12Mbps ACK = 40 = 32us */
		    (width >= 53 && width <= 58) ||          /* Region 2:  6Mbps ACK = 55 = 44us */
		    (width >= 126 && width <= 140) ||        /* Region 3: 11Mbps ACK = 135 = 108us */
		    (width >= 141 && width <= 160) ||        /* Region 4: 5.5Mbps ACK = 150 = 120us */
		    (width >= 189 && width <= 210) ||        /* Region 5:  2Mbps ACK = 200 = 160us */
		    (width >= 360 && width <= 380) ||        /* Region 6   1Mbps ACK = 400 = 320us */
		    (width >= 257 && width <= 270) ||        /* Region 7   5.5Mbps (Long Pre) ACK = 270 = 216us */
		    (width >= 295 && width <= 302) ||        /* Region 8   2Mbps (Long Pre) ACK = 320 = 256us */
		    /* Ignoring Region 9 due to overlap with 255 which is same as board noise */
		    /* Region 9  11Mbps (Long Pre) ACK = 255 = 204us */            
		    (width == 3)) {
			ar->ar_acksum++;
			/* double the count for strong radars that match one of the ACK signatures */
			if (rssi > DFS_AR_RSSI_DOUBLE_THRESHOLD) {
				ar->ar_phyerrcount[width]++;
				ar->ar_acksum++;
			}
			UPDATE_TOP_THREE_PEAKS(ar->ar_phyerrcount,
					       ar->ar_peaklist, width);
			/* sum the counts of these peaks */
			numpeaks = DFS_AR_MAX_NUM_PEAKS;
			origregionsum = ar->ar_acksum;
			for (i=0; i<= DFS_AR_MAX_NUM_PEAKS; i++) {
				if (ar->ar_peaklist[i] > 0) {
					if ((i==0) &&
					    (ar->ar_peaklist[i] == 3) &&
					    (ar->ar_phyerrcount[3] <
					     ar->ar_phyerrcount[2]) &&
					    (ar->ar_phyerrcount[3] > 6)) {
						/*
						 * If the top peak is one that
						 * maches the 11Mbps long
						 * preamble TCP Ack/1500-byte
						 * data, include the count for
						 * radars that hav emax
						 * duration and high rssi
						 * (width = 2) to boost the
						 * sum for the PAR test that
						 * follows */
						sumpeak += (ar->ar_phyerrcount[2]
							    + ar->ar_phyerrcount[3]);
						ar->ar_acksum += (ar->ar_phyerrcount[2]
								  + ar->ar_phyerrcount[3]);
					} else {
						sumpeak += ar->ar_phyerrcount[ar->ar_peaklist[i]];
					}
				} else 
					numpeaks--;
			}
			/*
			 * If sum of patterns matches exceeds packet threshold,
			 * perform comparison between peak-to-avg ratio against parThreshold
			 */
			if ((ar->ar_acksum > ar->ar_packetthreshold) &&
			    ((sumpeak * DFS_AR_REGION_WIDTH) > (ar->ar_parthreshold * numpeaks *
								ar->ar_acksum))) {
				/* neighboring traffic detected, get out of Turbo */
				chan->privFlags |= CHANNEL_INTERFERENCE;
				OS_MEMZERO(ar->ar_peaklist, sizeof(ar->ar_peaklist));
				ar->ar_acksum = 0;
				OS_MEMZERO(ar->ar_phyerrcount, sizeof(ar->ar_phyerrcount));
			} else {
				/*
				 * reset sum of matches to discount the count of
				 * strong radars with max duration
				 */
				ar->ar_acksum = origregionsum;
			}
		}
		ar->ar_prevtimestamp = thistimestamp;
		ATH_ARQ_LOCK(dfs);
		empty = STAILQ_EMPTY(&(dfs->dfs_arq));
		ATH_ARQ_UNLOCK(dfs);
	}
}

int
dfs_control(ath_dev_t dev, u_int id,
                void *indata, u_int32_t insize,
                void *outdata, u_int32_t *outsize)
{
	struct ath_softc *sc = ATH_DEV_TO_SC(dev);
	int error = 0;
	u_int32_t *data = NULL;
	HAL_PHYERR_PARAM peout;
	struct ath_dfs *dfs = sc->sc_dfs;
	struct dfs_ioctl_params *dfsparams;
	struct ieee80211com *ic = (struct ieee80211com *)sc->sc_ieee; /* XXX */
        u_int32_t val=0;

	if (dfs == NULL) {
		error = -EINVAL;
                DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s DFS is null\n", __func__);
		goto bad;
	}

	switch (id) {
	case DFS_MUTE_TIME:
		if (insize < sizeof(u_int32_t) || !indata) {
			error = -EINVAL;
			break;
		}
		data = (u_int32_t *) indata;
		sc->sc_dfs->sc_dfstesttime = *data;
		sc->sc_dfs->sc_dfstesttime *= (1000); //convert sec into ms
		break;
	case DFS_SET_THRESH:
		if (insize < sizeof(HAL_PHYERR_PARAM) || !indata) {
			error = -EINVAL;
			break;
		}
		dfsparams = (struct dfs_ioctl_params *) indata;
		if (!dfs_set_thresholds(sc, DFS_PARAM_FIRPWR, dfsparams->dfs_firpwr))
			error = -EINVAL;
		if (!dfs_set_thresholds(sc, DFS_PARAM_RRSSI, dfsparams->dfs_rrssi))
			error = -EINVAL;
		if (!dfs_set_thresholds(sc, DFS_PARAM_HEIGHT, dfsparams->dfs_height))
			error = -EINVAL;
		if (!dfs_set_thresholds(sc, DFS_PARAM_PRSSI, dfsparams->dfs_prssi))
			error = -EINVAL;
		if (!dfs_set_thresholds(sc, DFS_PARAM_INBAND, dfsparams->dfs_inband))
			error = -EINVAL;
		/* 5413 speicfic */
		if (!dfs_set_thresholds(sc, DFS_PARAM_RELPWR, dfsparams->dfs_relpwr))
			error = -EINVAL;
		if (!dfs_set_thresholds(sc, DFS_PARAM_RELSTEP, dfsparams->dfs_relstep))
			error = -EINVAL;
		if (!dfs_set_thresholds(sc, DFS_PARAM_MAXLEN, dfsparams->dfs_maxlen))
			error = -EINVAL;
		break;
	case DFS_GET_THRESH:
		if (!outdata || !outsize || *outsize <sizeof(struct dfs_ioctl_params)) {
			error = -EINVAL;
			break;
		}
		*outsize = sizeof(struct dfs_ioctl_params);
		ath_hal_getdfsthresh(sc->sc_ah, &peout);
		dfsparams = (struct dfs_ioctl_params *) outdata;
		dfsparams->dfs_firpwr = peout.pe_firpwr;
		dfsparams->dfs_rrssi = peout.pe_rrssi;
		dfsparams->dfs_height = peout.pe_height;
		dfsparams->dfs_prssi = peout.pe_prssi;
		dfsparams->dfs_inband = peout.pe_inband;
		/* 5413 specific */
		dfsparams->dfs_relpwr = peout.pe_relpwr;
		dfsparams->dfs_relstep = peout.pe_relstep;
		dfsparams->dfs_maxlen = peout.pe_maxlen;
                break;
	case DFS_GET_USENOL:
		if (!outdata || !outsize || *outsize < sizeof(u_int32_t)) {
			error = -EINVAL;
			break;
		}
		*outsize = sizeof(u_int32_t);
		*((u_int32_t *)outdata) = dfs->dfs_rinfo.rn_use_nol;
		break;
	case DFS_SET_USENOL:
		if (insize < sizeof(u_int32_t) || !indata) {
			error = -EINVAL;
			break;
		}
		dfs->dfs_rinfo.rn_use_nol = *(u_int32_t *)indata;
		/* iwpriv markdfs in linux can do the same thing... */
		if (dfs->dfs_rinfo.rn_use_nol) {
			ic->ic_flags_ext |= IEEE80211_FEXT_MARKDFS;
		} else {
			ic->ic_flags_ext &= ~IEEE80211_FEXT_MARKDFS;
                }
		break;
        case DFS_SHOW_NOL:
                dfs_print_nol(sc);
                break;
	case DFS_BANGRADAR:
		dfs->dfs_bangradar = 1;     
                sc->sc_rtasksched = 1;
                OS_SET_TIMER(&sc->sc_dfs->sc_dfs_task_timer, 0);
		break;
	case DFS_RADARDETECTS:
		if (!outdata || !outsize || *outsize < sizeof(u_int32_t)) {
			error = -EINVAL;
			break;
		}
		*outsize = sizeof (u_int32_t);
		*((u_int32_t *)outdata) = dfs->ath_dfs_stats.num_radar_detects;
		break;
        case DFS_DISABLE_DETECT:
                printk("%s disable detects\n", __func__);
                dfs->dfs_proc_phyerr &= ~DFS_RADAR_EN;
                break;
        case DFS_ENABLE_DETECT:
		dfs->dfs_proc_phyerr |= DFS_RADAR_EN;
                printk("%s enable detects\n", __func__);
                break;
        case DFS_DISABLE_FFT:
#define AR_PHY_RADAR_0      0x9954      /* radar detection settings */
#define AR_PHY_RADAR_DISABLE_FFT 0x7FFFFFFF
                val = OS_REG_READ(sc->sc_ah, AR_PHY_RADAR_0);
                val &= AR_PHY_RADAR_DISABLE_FFT;
                OS_REG_WRITE(sc->sc_ah, AR_PHY_RADAR_0, val);
                val = OS_REG_READ(sc->sc_ah, AR_PHY_RADAR_0);
#undef AR_PHY_RADAR_0
#undef AR_PHY_RADAR_DISABLE_FFT
                printk("%s disable FFT val=0x%x \n", __func__, val);
                break;
        case DFS_ENABLE_FFT:
#define AR_PHY_RADAR_0      0x9954      /* radar detection settings */
#define AR_PHY_RADAR_ENABLE_FFT 0x80000000
                val = OS_REG_READ(sc->sc_ah, AR_PHY_RADAR_0);
                val |= AR_PHY_RADAR_ENABLE_FFT;
                OS_REG_WRITE(sc->sc_ah, AR_PHY_RADAR_0, val);
                val = OS_REG_READ(sc->sc_ah, AR_PHY_RADAR_0);
#undef AR_PHY_RADAR_ENABLE_FFT
#undef AR_PHY_RADAR_0            /* radar detection settings */
                printk("%s enable FFT val=0x%x \n", __func__, val);
                break;
        case DFS_SET_DEBUG_LEVEL:
		if (insize < sizeof(u_int32_t) || !indata) {
                        error = -EINVAL;
			break;
		}
		dfs_debug_level = *(u_int32_t *)indata;
		dfs_debug_level = (ATH_DEBUG_DFS << dfs_debug_level);
                printk("%s debug level now = 0x%x \n", __func__, dfs_debug_level);
                break;
	default:
		error = -EINVAL;
	}
bad:
	return error;
}

static int
dfs_set_thresholds(struct ath_softc *sc, const u_int32_t threshtype,
		   const u_int32_t value)
{
	struct ath_dfs *dfs=sc->sc_dfs;
	int16_t chanindex;
	struct dfs_state *rs;
	HAL_PHYERR_PARAM pe;

	if (dfs == NULL) {
		DFS_DPRINTK(sc, ATH_DEBUG_DFS, "%s: sc_dfs is NULL\n",
			__func__);
		return 0;
	}

	chanindex = dfs->dfs_curchan_radindex;
	if ((chanindex <0) || (chanindex >= DFS_NUM_RADAR_STATES))
		return 0;
	pe.pe_firpwr = HAL_PHYERR_PARAM_NOVAL;
	pe.pe_height = HAL_PHYERR_PARAM_NOVAL;
	pe.pe_prssi = HAL_PHYERR_PARAM_NOVAL;
	pe.pe_inband = HAL_PHYERR_PARAM_NOVAL;
	pe.pe_rrssi = HAL_PHYERR_PARAM_NOVAL;
	/* 5413 specific */
	pe.pe_relpwr = HAL_PHYERR_PARAM_NOVAL;
	pe.pe_relstep = HAL_PHYERR_PARAM_NOVAL;
	pe.pe_maxlen = HAL_PHYERR_PARAM_NOVAL;

	rs = &(dfs->dfs_radar[chanindex]);
	switch (threshtype) {
	case DFS_PARAM_FIRPWR:
		rs->rs_firpwr = (int32_t) value;
		pe.pe_firpwr = rs->rs_firpwr;
		break;
	case DFS_PARAM_RRSSI:
		rs->rs_radarrssi = value;
		pe.pe_rrssi = value;
		break;
	case DFS_PARAM_HEIGHT:
		rs->rs_height = value;
		pe.pe_height = value;
		break;
	case DFS_PARAM_PRSSI:
		rs->rs_pulserssi = value;
		pe.pe_prssi = value;
		break;
	case DFS_PARAM_INBAND:
		rs->rs_inband = value;
		pe.pe_inband = value;
		break;
	/* 5413 specific */
	case DFS_PARAM_RELPWR:
		rs->rs_relpwr = value;
		pe.pe_relpwr = value;
		break;
	case DFS_PARAM_RELSTEP:
		rs->rs_relstep = value;
		pe.pe_relstep = value;
		break;
	case DFS_PARAM_MAXLEN:
		rs->rs_maxlen = value;
		pe.pe_maxlen = value;
		break;
	}
	ath_hal_enabledfs(sc->sc_ah, &pe);
	return 1;
}

int
dfs_get_thresholds(struct ath_softc *sc, HAL_PHYERR_PARAM *param)
{
	ath_hal_getdfsthresh(sc->sc_ah, param);
	return 1;
}

#ifndef __NetBSD__
#ifdef __linux__
/*
 * Linux Module glue.
 */
static char *dev_info = "ath_dfs";

MODULE_AUTHOR("Atheros Communications, Inc.");
MODULE_DESCRIPTION("DFS Support for Atheros 802.11 wireless LAN cards.");
MODULE_SUPPORTED_DEVICE("Atheros WLAN cards");
#ifdef MODULE_LICENSE
MODULE_LICENSE("Proprietary");
#endif

static int __init
init_ath_dfs(void)
{
	printk (KERN_INFO "%s: Version 2.0.0\n"
		"Copyright (c) 2005-2006 Atheros Communications, Inc. "
		"All Rights Reserved\n",dev_info);
	return 0;
}
module_init(init_ath_dfs);

static void __exit
exit_ath_dfs(void)
{
	printk (KERN_INFO "%s: driver unloaded\n", dev_info);
}
module_exit(exit_ath_dfs);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,52))
MODULE_PARM(domainoverride, "i");
MODULE_PARM(usenol, "i");
MODULE_PARM_DESC(domainoverride, "Override dfs domain");
MODULE_PARM_DESC(usenol, "Override the use of channel NOL");
#else
#include <linux/moduleparam.h>
module_param(domainoverride, int, 0600);
module_param(usenol, int, 0600);
#endif

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#endif

EXPORT_SYMBOL(dfs_getchanstate);
EXPORT_SYMBOL(dfs_process_radarevent);
EXPORT_SYMBOL(dfs_check_nol);
EXPORT_SYMBOL(dfs_attach);
EXPORT_SYMBOL(dfs_detach);
EXPORT_SYMBOL(ath_ar_enable);
EXPORT_SYMBOL(dfs_radar_enable);
EXPORT_SYMBOL(ath_ar_disable);
EXPORT_SYMBOL(ath_process_phyerr);
EXPORT_SYMBOL(dfs_process_ar_event);
EXPORT_SYMBOL(dfs_control);
EXPORT_SYMBOL(dfs_get_thresholds);
EXPORT_SYMBOL(dfs_init_radar_filters);
EXPORT_SYMBOL(dfs_clear_stats);

#endif /* __linux__ */
#endif /* __netbsd__ */
