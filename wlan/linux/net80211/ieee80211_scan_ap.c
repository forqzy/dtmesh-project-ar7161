/*! \file
**  \brief 
**
** Copyright (c) 2002-2005 Sam Leffler, Errno Consulting
** Copyright (c) 2004-2007 Atheros Communications, Inc.
**
** All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef EXPORT_SYMTAB
#define	EXPORT_SYMTAB
#endif

__FBSDID("$FreeBSD$");

/*
 * IEEE 802.11 ap scanning support.
 */
#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif
#include <linux/version.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/init.h>
#include <linux/random.h>

#include "if_media.h"

#include <net80211/ieee80211_var.h>

struct ap_scan_entry
{
    struct ieee80211_scan_entry base;
    TAILQ_ENTRY(ap_scan_entry) ase_list;
    int rssi;
};

struct ap_scan_list
{
    TAILQ_HEAD(, ap_scan_entry) asl_head;
};

struct ap_state
{
    spinlock_t	as_lock;		/* on scan table */
    struct	ap_scan_list as_ht_list[IEEE80211_CHAN_MAX];
    struct	ap_scan_list as_nonht_list[IEEE80211_CHAN_MAX];
};

#define	IEEE80211_AP_LOCK_BH(_as)			\
	if (irqs_disabled()) {				\
		spin_lock(&(_as)->as_lock);		\
	} else {					\
		spin_lock_bh(&(_as)->as_lock);		\
	}

#define	IEEE80211_AP_UNLOCK_BH(_as)			\
	if (irqs_disabled()) {				\
		spin_unlock(&(_as)->as_lock);		\
	} else {					\
		spin_unlock_bh(&(_as)->as_lock);	\
	}

#define	IEEE80211_AP_LOCK(_as)		spin_lock(&(_as)->as_lock)
#define	IEEE80211_AP_UNLOCK(_as)	spin_unlock(&(_as)->as_lock)

static int ap_flush(struct ieee80211_scan_state *);
static int check_interference(struct ieee80211_channel *chan, struct ieee80211vap *vap);
static int find_best_11na_centerchan(struct ieee80211_scan_state *ss, struct ieee80211vap *vap);
static int find_best_11ng_centerchan(struct ieee80211_scan_state *ss, struct ieee80211vap *vap);
#define find_best_11a_centerchan	find_best_11na_centerchan	/* xxx: will change */
#define find_best_legacy_centerchan	find_best_11na_centerchan	/* xxx: will change */

/*
 * Attach prior to any scanning work.
 */
static int
ap_attach(struct ieee80211_scan_state *ss)
{
    struct ap_state *as;
    int i;

    _MOD_INC_USE(THIS_MODULE, return 0);

    MALLOC(as, struct ap_state *, sizeof(struct ap_state),
        M_SCANCACHE, M_NOWAIT);
    if (as == NULL)
    {
        return 0;
    }
    for (i=0; i<IEEE80211_CHAN_MAX; i++)
    {
        TAILQ_INIT(&as->as_ht_list[i].asl_head);
        TAILQ_INIT(&as->as_nonht_list[i].asl_head);
    }
    ss->ss_priv = as;
    ap_flush(ss);
    return 1;
}

/*
 * Flush all per-scan state.
 */
static int
ap_flush_scantable(struct ieee80211_scan_state *ss)
{
    struct ap_state *as = ss->ss_priv;
    struct ap_scan_entry *se, *next;
    int i;

    IEEE80211_AP_LOCK_BH(as);
    for (i=0; i<IEEE80211_CHAN_MAX; i++)
    {
        TAILQ_FOREACH_SAFE(se, &as->as_ht_list[i].asl_head, ase_list, next) {
            TAILQ_REMOVE(&as->as_ht_list[i].asl_head, se, ase_list);
            FREE(se, M_80211_SCAN);
        }
        TAILQ_FOREACH_SAFE(se, &as->as_nonht_list[i].asl_head, ase_list, next) {
            TAILQ_REMOVE(&as->as_nonht_list[i].asl_head, se, ase_list);
            FREE(se, M_80211_SCAN);
        }
    }

    IEEE80211_AP_UNLOCK_BH(as);
    return 0;
}

/*
 * Cleanup any private state.
 */
static int
ap_detach(struct ieee80211_scan_state *ss)
{
    struct ap_state *as = ss->ss_priv;

    if (as != NULL)
    {
        ap_flush_scantable(ss);
        FREE(as, M_SCANCACHE);
    }

    _MOD_DEC_USE(THIS_MODULE);
    return 1;
}

/*
 * Flush all per-scan state.
 */
static int
ap_flush(struct ieee80211_scan_state *ss)
{
    ap_flush_scantable(ss);
    ss->ss_last = 0;		/* insure no channel will be picked */
    return 0;
}

static int
finddot11channel(struct ieee80211com *ic, int i, int freq, u_int flags)
{
    struct ieee80211_channel *c;
    int j;

    for (j = i+1; j < ic->ic_nchans; j++)
    {
        c = &ic->ic_channels[j];
        if ((c->ic_freq == freq) && ((c->ic_flags & flags) == flags))
            return 1;
    }
    for (j = 0; j < i; j++)
    {
        c = &ic->ic_channels[j];
        if ((c->ic_freq == freq) && ((c->ic_flags & flags) == flags))
            return 1;
    }
    return 0;
}

static int
find11gchannel(struct ieee80211com *ic, int i, int freq)
{
    const struct ieee80211_channel *c;
    int j;

    /*
    * The normal ordering in the channel list is b channel
    * immediately followed by g so optimize the search for
    * this.  We'll still do a full search just in case.
    */
    for (j = i+1; j < ic->ic_nchans; j++)
    {
        c = &ic->ic_channels[j];
        if (c->ic_freq == freq && IEEE80211_IS_CHAN_ANYG(c))
            return 1;
    }
    for (j = 0; j < i; j++)
    {
        c = &ic->ic_channels[j];
        if (c->ic_freq == freq && IEEE80211_IS_CHAN_ANYG(c))
            return 1;
    }
    return 0;
}


/*
 * Start an ap 20/40 coext scan by populating the channel list.
 */
static int
ap_coext_start(struct ieee80211_scan_state *ss, struct ieee80211vap *vap)
{
    struct ieee80211com *ic = vap->iv_ic;
    struct ieee80211_channel *c;
    int i;

    IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN," %s: Entering\n", __func__);
    ap_flush_scantable(ss);
    ss->ss_last = 0;
    for (i = 0; i < ic->ic_nchans; i++)
    {
        c = &ic->ic_channels[i];
        if (IEEE80211_IS_CHAN_2GHZ(c) &&
            IEEE80211_IS_CHAN_11NG_HT20(c)) {
            ss->ss_chans[ss->ss_last++] = c;
            IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN,"%s: Adding channel %d\n", __func__, c->ic_ieee);
        }
    }
    ss->ss_next = 0;
    /* XXX tunables */
    ss->ss_mindwell = msecs_to_jiffies(20);	/* 20ms */
    ss->ss_maxdwell = msecs_to_jiffies(30);	/* 30ms */


    return 0;
}


/*
 * Start an ap channel selction scan by populating the channel list.
 */
static int
ap_chansel_start(struct ieee80211_scan_state *ss, struct ieee80211vap *vap)
{
    struct ieee80211com *ic = vap->iv_ic;
    struct ieee80211_channel *c;
    int i;

    ap_flush_scantable(ss);
    ss->ss_last = 0;
    if (vap->iv_des_mode != IEEE80211_MODE_AUTO)
    {
        static const u_int chanflags[] = {
			0,			/* IEEE80211_MODE_AUTO */
			IEEE80211_CHAN_A,   /* IEEE80211_MODE_11A */
			IEEE80211_CHAN_B,   /* IEEE80211_MODE_11B */
			IEEE80211_CHAN_PUREG,   /* IEEE80211_MODE_11G */
			IEEE80211_CHAN_FHSS,    /* IEEE80211_MODE_FH */
			IEEE80211_CHAN_108A,    /* IEEE80211_MODE_TURBO_A */
			IEEE80211_CHAN_108G,    /* IEEE80211_MODE_TURBO_G */
			IEEE80211_CHAN_11NA_HT20,      /* IEEE80211_MODE_11NA_HT20 */
			IEEE80211_CHAN_11NG_HT20,      /* IEEE80211_MODE_11NG_HT20 */
			IEEE80211_CHAN_11NA_HT40PLUS,  /* IEEE80211_MODE_11NA_HT40PLUS */
			IEEE80211_CHAN_11NA_HT40MINUS, /* IEEE80211_MODE_11NA_HT40MINUS */
			IEEE80211_CHAN_11NG_HT40PLUS,  /* IEEE80211_MODE_11NG_HT40PLUS */
			IEEE80211_CHAN_11NG_HT40MINUS, /* IEEE80211_MODE_11NG_HT40MINUS */
            IEEE80211_CHAN_11NA_HT40,      /* IEEE80211_MODE_11NA_HT40 */
            IEEE80211_CHAN_11NG_HT40,      /* IEEE80211_MODE_11NG_HT40 */
		};
        u_int modeflags;

        modeflags = chanflags[vap->iv_des_mode];
        if (vap->iv_ath_cap & IEEE80211_ATHC_TURBOP && modeflags != IEEE80211_CHAN_ST)
        {
            if (vap->iv_des_mode == IEEE80211_MODE_11G)
                modeflags = IEEE80211_CHAN_108G;
            else
                modeflags = IEEE80211_CHAN_108A;
        }
        for (i = 0; i < ic->ic_nchans; i++)
        {
            u_int cflags; 

            c = &ic->ic_channels[i];

            cflags = c->ic_flags & modeflags;
            if (modeflags == IEEE80211_CHAN_11NG_HT40) {
                if ((cflags != IEEE80211_CHAN_11NG_HT40PLUS) &&
                    (cflags != IEEE80211_CHAN_11NG_HT40MINUS))
                continue;
            }
            else if (modeflags == IEEE80211_CHAN_11NA_HT40) {
                if ((cflags != IEEE80211_CHAN_11NA_HT40PLUS) &&
                    (cflags != IEEE80211_CHAN_11NA_HT40MINUS))
                continue;
            }
            else if (cflags != modeflags) 
                continue;

            /* XR is not supported on turbo channels */
            if (IEEE80211_IS_CHAN_TURBO(c) && vap->iv_ath_cap & IEEE80211_ATHC_XR)
                continue;
            if (ss->ss_last >= IEEE80211_SCAN_MAX)
                break;
            /* 
            * do not select static turbo channels if the mode is not
            * static turbo .
            */
            if (IEEE80211_IS_CHAN_STURBO(c) && vap->iv_des_mode != IEEE80211_MODE_MAX ) 
                continue;
            /* No dfs interference detected channels */
            if (c->ic_flags & IEEE80211_CHAN_RADAR)
                continue;
            ss->ss_chans[ss->ss_last++] = c;
        }
    }


    /* switch to auto mode if no channels matching the mode were found*/
    if (!ss->ss_last)
    {
        printf("%s: No channels found for mode %d, switching to auto mode\n", 
               vap->iv_dev->name, vap->iv_des_mode);
        vap->iv_des_mode = IEEE80211_MODE_AUTO;
    }

    if (vap->iv_des_mode == IEEE80211_MODE_AUTO)
    {
        for (i = 0; i < ic->ic_nchans; i++)
        {
            c = &ic->ic_channels[i];
            if (IEEE80211_IS_CHAN_TURBO(c))
            {
                /* XR is not supported on turbo channels */
                if (vap->iv_ath_cap & IEEE80211_ATHC_XR)
                    continue;
                /* dynamic channels are scanned in base mode */
                if (!IEEE80211_IS_CHAN_ST(c))
                    continue;
            }
            else
            {
                /*
                * Use any 11g channel instead of 11b one.
                */
                if (IEEE80211_IS_CHAN_ANYG(c) &&
                    (finddot11channel(ic, i, c->ic_freq, IEEE80211_CHAN_11NG_HT20) ||
                    finddot11channel(ic, i, c->ic_freq, IEEE80211_CHAN_11NG_HT40PLUS) ||
                    finddot11channel(ic, i, c->ic_freq, IEEE80211_CHAN_11NG_HT40MINUS)))
                    continue;
                if (IEEE80211_IS_CHAN_A(c) &&
                    (finddot11channel(ic, i, c->ic_freq, IEEE80211_CHAN_11NA_HT20) ||
                    finddot11channel(ic, i, c->ic_freq, IEEE80211_CHAN_11NA_HT40PLUS) ||
                    finddot11channel(ic, i, c->ic_freq, IEEE80211_CHAN_11NA_HT40MINUS)))
                    continue;
                if (IEEE80211_IS_CHAN_B(c) &&
                    finddot11channel(ic, i, c->ic_freq, IEEE80211_CHAN_11NG_HT20))
                    continue;
                if (IEEE80211_IS_CHAN_B(c) &&
                    find11gchannel(ic, i, c->ic_freq))
                    continue;
            }
            if (c->ic_flags & IEEE80211_CHAN_RADAR)
                continue;
            if (ss->ss_last >= IEEE80211_SCAN_MAX)
                break;
            ss->ss_chans[ss->ss_last++] = c;
        }
    }

    ss->ss_next = 0;
    /* XXX tunables */
    ss->ss_mindwell = msecs_to_jiffies(200);	/* 200ms */
    ss->ss_maxdwell = msecs_to_jiffies(300);	/* 300ms */

#ifdef IEEE80211_DEBUG
    if (ieee80211_msg_scan(vap))
    {
        printf("%s: scan set ", vap->iv_dev->name);
        ieee80211_scan_dump_channels(ss);
        printf(" dwell min %ld max %ld\n",
            ss->ss_mindwell, ss->ss_maxdwell);
    }
#endif /* IEEE80211_DEBUG */

    return 0;
}

/*
 * Start an ap scan.
 */
static int
ap_start(struct ieee80211_scan_state *ss, struct ieee80211vap *vap)
{
    if (ss->ss_flags & IEEE80211_SCAN_OBSS)
        return ap_coext_start(ss, vap);
    else
        return ap_chansel_start(ss, vap);
}

/*
 * Restart a bg scan.
 */
static int
ap_restart(struct ieee80211_scan_state *ss, struct ieee80211vap *vap)
{
    return 0;
}

/*
 * Cancel an ongoing scan.
 */
static int
ap_cancel(struct ieee80211_scan_state *ss, struct ieee80211vap *vap)
{
    return 0;
}

static void
savenie(u_int8_t **iep, const u_int8_t *ie, size_t len)
{

    if (ie != NULL)
    {
        ieee80211_savenie(iep, ie, len);
    }
}


/*
 * Record max rssi on channel.
 */
static int
ap_add(struct ieee80211_scan_state *ss,
    const struct ieee80211_scanparams *sp,
    const struct ieee80211_frame *wh,
    int subtype, int rssi, int rstamp)
{
    struct ap_state *as = ss->ss_priv;
    struct ieee80211vap *vap = ss->ss_vap;
    struct ieee80211com *ic = vap->iv_ic;
    const u_int8_t *macaddr = wh->i_addr2;
    struct ap_scan_entry *se, *next;
    struct ieee80211_scan_entry *ise;
    int chan;

    chan = ieee80211_chan2ieee(ic, ic->ic_curchan);
    /* XXX better quantification of channel use? */
    /* XXX count bss's? */

    IEEE80211_AP_LOCK_BH(as);
    if (sp->htinfo)
    {
        TAILQ_FOREACH_SAFE(se, &as->as_ht_list[chan].asl_head, ase_list, next) {
            if (IEEE80211_ADDR_EQ(se->base.se_macaddr, macaddr))
                goto found;
        }
    }
    else
    {
        TAILQ_FOREACH_SAFE(se, &as->as_nonht_list[chan].asl_head, ase_list, next) {
            if (IEEE80211_ADDR_EQ(se->base.se_macaddr, macaddr))
                goto found;
        }
    }

    MALLOC(se, struct ap_scan_entry *, sizeof(struct ap_scan_entry),
        M_80211_SCAN, M_NOWAIT | M_ZERO);
    if (se == NULL)
    {
        IEEE80211_AP_UNLOCK_BH(as);
        return 0;
    }

    if (sp->htinfo)
    {
        TAILQ_INSERT_TAIL(&as->as_ht_list[chan].asl_head, se, ase_list);
    }
    else
    {
        TAILQ_INSERT_TAIL(&as->as_nonht_list[chan].asl_head, se, ase_list);
    }

    se->rssi = rssi;

    IEEE80211_ADDR_COPY(se->base.se_macaddr, macaddr);
found:
    ise = &se->base;
    savenie(&ise->se_htinfo_ie, sp->htinfo, 
        sizeof(struct ieee80211_ie_htinfo_cmn));
    if (sp->ssid[1] != 0 &&
        ise->se_ssid[1] == 0)
        memcpy(ise->se_ssid, sp->ssid, 2+sp->ssid[1]);
    ise->se_intval = sp->bintval;
    ise->se_chan = ic->ic_curchan;
    ise->se_capinfo = sp->capinfo;
    ise->se_rssi = se->rssi;

    IEEE80211_AP_UNLOCK_BH(as);
    return 1;
}

/*
 * set the 20/40 ap operation.
 */
static int
ap_coext_end(struct ieee80211_scan_state *ss, struct ieee80211vap *vap,
    int (*action)(struct ieee80211vap *, const struct ieee80211_scan_entry *))
{
    struct ieee80211_scan_entry se;
    struct ieee80211com *ic = vap->iv_ic;
    int i, chan;

    for (i = 0; i < ss->ss_last; i++) {
        chan = ieee80211_chan2ieee(ic, ss->ss_chans[i]);
        if (ss->ss_chans[i]->ic_flags & IEEE80211_CHAN_HT40INTOL) {
                IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN,"%s: Detected legacy BSS on channel %d\n", __func__, chan);
                vap->iv_des_chan->ic_flags |= IEEE80211_CHAN_HT40INTOL;
        }
    }
    se.se_chan = vap->iv_des_chan;
    if (action == NULL)
        action = ss->ss_ops->scan_default;
    return ((*action)(vap, &se));
}


/*
 * Pick a quiet channel to use for ap operation.
 */
static int
ap_chansel_end(struct ieee80211_scan_state *ss, struct ieee80211vap *vap,
    int (*action)(struct ieee80211vap *, const struct ieee80211_scan_entry *))
{
    struct ap_state *as = ss->ss_priv;
    struct ieee80211com *ic = vap->iv_ic;
    int i, chan, bestchan, bestchanix;
    unsigned int random;
    struct ap_scan_entry *se, *next;
    struct ieee80211_ie_htinfo_cmn *htinfo;

    KASSERT(vap->iv_opmode == IEEE80211_M_HOSTAP,
        ("wrong opmode %u", vap->iv_opmode));

    if (ss->ss_flags & IEEE80211_SCAN_NOPICK)
    {
        /*
         * Manual scan, don't select the channel, just return.  
         */
        ss->ss_flags &= ~IEEE80211_SCAN_NOPICK;
        return 1;
    }
    /* XXX select channel more intelligently, e.g. channel spread, power */
    bestchan = -1;
    bestchanix = 0;		/* NB: silence compiler */

    if (ic->ic_isdfsregdomain &&
        ((ic->ic_curmode == IEEE80211_MODE_11A) ||
        (ic->ic_curmode == IEEE80211_MODE_11NA_HT20) ||
        (ic->ic_curmode == IEEE80211_MODE_11NA_HT40PLUS) ||
        (ic->ic_curmode == IEEE80211_MODE_11NA_HT40MINUS)))
    {
        if (ss->ss_last)
        {
try_again:
            /* Pick up a random channel to start */
            get_random_bytes(&random, sizeof(random));

            i = random % ss->ss_last;

            if ( (ss->ss_chans[i]->ic_flags & IEEE80211_CHAN_RADAR) ||
                (ss->ss_chans[i]->ic_flags & IEEE80211_CHAN_DFS &&
                ic->ic_flags_ext & IEEE80211_FEXT_BLKDFSCHAN) )
                goto try_again;

            chan = ieee80211_chan2ieee(ic, ss->ss_chans[i]);

            IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN,
                "%s: picked random channel %u\n",
                __func__, chan);

            bestchan = chan;
            bestchanix = i;
        }
    }
    else
    {
        IEEE80211_AP_LOCK_BH(as);
#ifdef IEEE80211_DEBUG
        for (i = 0; i < ss->ss_last; i++)
        {
            chan = ieee80211_chan2ieee(ic, ss->ss_chans[i]);

            TAILQ_FOREACH_SAFE(se, &as->as_ht_list[chan].asl_head, ase_list, next) {
                htinfo = (struct ieee80211_ie_htinfo_cmn *) se->base.se_htinfo_ie;
                KASSERT(htinfo != NULL, ("htinfo should be present"));
                IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN,
                    "ht bss: chan: %d, ctrlchan: %d, extoffset: %d\n", 
                    chan,
                    htinfo->hi_ctrlchannel,
                    htinfo->hi_extchoff);
            }

            TAILQ_FOREACH_SAFE(se, &as->as_nonht_list[chan].asl_head, ase_list, next) {
                IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN, "non-ht bss: chan: %d\n", chan);
            }
        }
#endif /* IEEE80211_DEBUG */

        switch (ic->ic_curmode)
        {
        case IEEE80211_MODE_11NA_HT20:
        case IEEE80211_MODE_11NA_HT40PLUS:
        case IEEE80211_MODE_11NA_HT40MINUS:
            bestchanix = find_best_11na_centerchan(ss, vap);
            ic->ic_cwm.cw_extprotspacing = IEEE80211_CWM_EXTPROTSPACING20;
            ic->ic_cwm.cw_mode = 1;
            break;

        case IEEE80211_MODE_11A:
        case IEEE80211_MODE_TURBO_A:
            bestchanix = find_best_11a_centerchan(ss, vap);
            break;

        case IEEE80211_MODE_11NG_HT20:
        case IEEE80211_MODE_11NG_HT40PLUS:
        case IEEE80211_MODE_11NG_HT40MINUS:
            bestchanix = find_best_11ng_centerchan(ss, vap);
            ic->ic_cwm.cw_extprotspacing = IEEE80211_CWM_EXTPROTSPACING25;
            ic->ic_cwm.cw_mode = 0;
            break;

        default:
            bestchanix = find_best_legacy_centerchan(ss, vap);
            break;
        }
        IEEE80211_AP_UNLOCK_BH(as);
    }

    if (bestchanix == -1)
    {
        /* no suitable channel, should not happen */
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN,
            "%s: no suitable channel! (should not happen)\n", __func__);
        return 0; /* restart scan */
    }
    else
    {
        struct ieee80211_channel *c;
        struct ieee80211_scan_entry se;
        /* XXX notify all vap's? */
        /* if this is a dynamic turbo frequency , start with normal mode first */

        c = ss->ss_chans[bestchanix];
        if (IEEE80211_IS_CHAN_TURBO(c) && !IEEE80211_IS_CHAN_STURBO(c))
        {
            if ((c = ieee80211_find_channel(ic, c->ic_freq, 
                c->ic_flags & ~IEEE80211_CHAN_TURBO)) == NULL)
            {
                /* should never happen ?? */
                return 0;
            }
        }
        memset(&se, 0, sizeof(se));
        se.se_chan = c;
        if (action == NULL)
            action = ss->ss_ops->scan_default;
        return ((*action)(vap, &se));
    }
}

/*
 * End AP scan 
 */
static int
ap_end(struct ieee80211_scan_state *ss, struct ieee80211vap *vap,
    int (*action)(struct ieee80211vap *, const struct ieee80211_scan_entry *))
{
    if (ss->ss_flags & IEEE80211_SCAN_OBSS)
        return ap_coext_end(ss, vap, action);
    else
        return ap_chansel_end(ss, vap, action);
}

/*
 * Check for channel interference.
 */
static int
check_interference(struct ieee80211_channel *chan, struct ieee80211vap *vap)
{
    struct ieee80211com *ic = vap->iv_ic;

    if (chan->ic_flags & IEEE80211_CHAN_RADAR ||
        (chan->ic_flags & IEEE80211_CHAN_DFS &&
        ic->ic_flags_ext & IEEE80211_FEXT_BLKDFSCHAN) )
    {
        return (1);
    }
    else
    {
        return  (0);
    }
}


static int
find_average_rssi(struct ieee80211_scan_state *ss, struct ieee80211vap *vap, const int *chanlist, int chancount)
{
    struct ap_state *as = ss->ss_priv;
    struct ap_scan_entry *se, *next;
    int chan;
    int i;
    int average=0;
    int maxrssi; /* max rssi for this channel */
    int totalrssi=0; /* total rssi for all channels so far */

    if (chancount <= 0)
    {
        return (0);
    }

    for (i=0; i<chancount; i++)
    {
        chan = chanlist[i];
        maxrssi = 0;

        //look for max rssi in ht beacons found in this channel
        TAILQ_FOREACH_SAFE(se, &as->as_ht_list[chan].asl_head, ase_list, next) {
            if (se->rssi > maxrssi)
            {
                maxrssi = se->rssi;
            }
        }

        //now look for max rssi in all of ht beacons found in this channel
        TAILQ_FOREACH_SAFE(se, &as->as_nonht_list[chan].asl_head, ase_list, next) {
            if (se->rssi > maxrssi)
            {
                maxrssi = se->rssi;
            }
        }

        totalrssi += maxrssi;
    }

    average = totalrssi/chancount;
    return (average);
}


/*
 * In the auto-scan mode, we are looking for best possible 20MHz band with center
 * channels 1, 6 and 11.
 *
 * We determine the best possible channel by averaging the rssi in 20MHz bands around
 * the center frequency channel.
 */
static int
find_best_11ng_centerchan(struct ieee80211_scan_state *ss, struct ieee80211vap *vap)
{
    int best_center_chanix = -1;
    struct ieee80211com *ic = vap->iv_ic;
    int minrssi = 0xffff, avg_rssi, i, chan, band;

    /*
    * The following center chan data structures are invented to allow calculating
    * the average rssi in 20Mhz band for a certain center chan. 
    *
    * We would then pick the band which has the minimum rsi of all the 20Mhz bands.
    */

    /* For 20Mhz band with center chan 1, we would see beacons only on channels 1,2. */
    const u_int center1[] = { 1, 2 };

    /* For 20Mhz band with center chan 6, we would see beacons on channels 4,5,6 & 7. */
    const u_int center6[] = { 4, 5, 6, 7 };

    /* For 20Mhz band with center chan 11, we would see beacons on channels 9,10 & 11. */
    const u_int center11[] = { 9, 10, 11 };

    struct centerchan
    {
        int count;               /* number of chans to average the rssi */
        const u_int *chanlist;   /* the possible beacon channels around center chan */
    };

#define X(a)    .count = sizeof(a)/sizeof(a[0]), .chanlist = a

    struct centerchan centerchans[] = {
		{ X(center1) },
		{ X(center6) },
		{ X(center11) }
	};


    for (i=0; i < ss->ss_last; i++)
    {
        /* Check for channel interference. If found, skip the channel */
        if (check_interference(ss->ss_chans[i], vap))
        {
            continue;
        }

        chan = ieee80211_chan2ieee(ic, ss->ss_chans[i]);
        if ((chan != 1) && (chan != 6) && (chan != 11))
        {
            /* Don't bother with other center channels except for 1, 6 & 11 */
            continue;
        }

        switch (chan)
        {
        case 1:
            band = 0;
            break;
        case 6:
            band = 1;
            break;
        case 11:
            band = 2;
            break;
        default:
            band = 0;
            break;
        }

        /* find the average rssi for this 20Mhz band */
        avg_rssi = find_average_rssi(ss, vap, centerchans[band].chanlist, centerchans[band].count);

        if (avg_rssi == 0)
        {
            best_center_chanix = i;
            break;
        }

        if (avg_rssi < minrssi)
        {
            minrssi = avg_rssi;
            best_center_chanix = i;
        }
    }

    if (best_center_chanix != -1)
    {
        chan = ieee80211_chan2ieee(ic, ss->ss_chans[best_center_chanix]);
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN, 
            "find_best_11ng_centerchan: found best center chan: %d", chan);
    }
    return (best_center_chanix);

}


static int
find_best_11na_centerchan(struct ieee80211_scan_state *ss, struct ieee80211vap *vap)
{
    int best_center_chanix, chanix[] = { -1, -1 };
    struct ap_state *as = ss->ss_priv;
    struct ap_scan_entry *se, *next;
    struct ieee80211com *ic = vap->iv_ic;
    int edge, minrssi[] = { 0xffff, 0xffff } , chanmaxrssi, i, chan;

#define NON_EDGE	0
#define EDGE		1

    for (i=0; i < ss->ss_last; i++)
    {
        register struct ieee80211_channel *ss_chans;

        ss_chans = ss->ss_chans[i];

        /* Check for channel interference. If found, skip the channel */
        if (check_interference(ss_chans, vap))
        {
            continue;
        }

        chan = ieee80211_chan2ieee(ic, ss_chans);

        chanmaxrssi = 0;

        //look for max rssi in ht beacons found in this channel
        TAILQ_FOREACH_SAFE(se, &as->as_ht_list[chan].asl_head, ase_list, next) {
            if (se->rssi > chanmaxrssi)
            {
                chanmaxrssi = se->rssi;
            }
        }

        //now look for max rssi in all of non-ht beacons found in this channel
        TAILQ_FOREACH_SAFE(se, &as->as_nonht_list[chan].asl_head, ase_list, next) {
            if (se->rssi > chanmaxrssi)
            {
                chanmaxrssi = se->rssi;
            }
        }

        if ((vap->iv_flags_ext & IEEE80211_FEXT_NO_EDGE_CH) &&
            (ss_chans->ic_flagext & IEEE80211_CHAN_EDGE_CH)) {
            edge = EDGE;
        } else {
            edge = NON_EDGE;
        }

        if (chanmaxrssi == 0)
        {
            chanix[edge] = i;
            if (edge == NON_EDGE) {
                minrssi[NON_EDGE] = 0;
                break;
            }
        }

        if (chanmaxrssi < minrssi[edge])
        {
            minrssi[edge] = chanmaxrssi;
            chanix[edge] = i;
        }
    }

    if (minrssi[EDGE] < minrssi[NON_EDGE]) {
        best_center_chanix = chanix[EDGE];
    } else {
        best_center_chanix = chanix[NON_EDGE];
    }

    if (best_center_chanix != -1)
    {
        chan = ieee80211_chan2ieee(ic, ss->ss_chans[i]);
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN, 
            "find_best_11na_centerchan: found best center chan: %d", chan);
    }
    return (best_center_chanix);
}

static void
ap_age(struct ieee80211_scan_state *ss)
{
    /* XXX is there anything meaningful to do? */
}

static int
ap_iterate(struct ieee80211_scan_state *ss,
    ieee80211_scan_iter_func *f, void *arg)
{
    struct ap_state *as = ss->ss_priv;
    struct ieee80211com *ic = ss->ss_vap->iv_ic;
    struct ap_scan_entry *se, *next;
    int i, chan;

    /* Wait if a scan is in progress */
    while (ic->ic_flags & IEEE80211_F_SCAN) {
        schedule_timeout_interruptible(HZ);
    }

    IEEE80211_AP_LOCK_BH(as);
    for (i = 0; i < ss->ss_last; i++) {

        chan = ieee80211_chan2ieee(ic, ss->ss_chans[i]);

        TAILQ_FOREACH_SAFE(se, &as->as_ht_list[chan].asl_head, ase_list, next) {
            (*f)(arg, &se->base);
        }
        TAILQ_FOREACH_SAFE(se, &as->as_nonht_list[chan].asl_head, ase_list, next) {
            (*f)(arg, &se->base);
        }
    }
    IEEE80211_AP_UNLOCK_BH(as);
    return 0;
}

static void
ap_assoc_success(struct ieee80211_scan_state *ss,
    const u_int8_t macaddr[IEEE80211_ADDR_LEN])
{
    /* should not be called */
}

static void
ap_assoc_fail(struct ieee80211_scan_state *ss,
    const u_int8_t macaddr[IEEE80211_ADDR_LEN], int reason)
{
    /* should not be called */
}

/*
 * Default action to execute when a scan entry is found for ap
 * mode.  Return 1 on success, 0 on failure
 */
static int
ap_default_action(struct ieee80211vap *vap,
    const struct ieee80211_scan_entry *se)
{
    ieee80211_create_ibss(vap, se->se_chan);
    return 1;
}

/*
 * Module glue.
 */

PRIVATE_C const struct ieee80211_scanner ap_default = {
	.scan_name		= "default",
	.scan_attach		= ap_attach,
	.scan_detach		= ap_detach,
	.scan_start		= ap_start,
	.scan_restart		= ap_restart,
	.scan_cancel		= ap_cancel,
	.scan_end		= ap_end,
	.scan_flush		= ap_flush,
	.scan_add		= ap_add,
	.scan_age		= ap_age,
	.scan_iterate		= ap_iterate,
	.scan_assoc_success	= ap_assoc_success,
	.scan_assoc_fail	= ap_assoc_fail,
	.scan_default		= ap_default_action,
};

PRIVATE_C int __init
init_scanner_ap(void)
{
    ieee80211_scanner_register(IEEE80211_M_HOSTAP, &ap_default);
    return 0;
}

PRIVATE_C void __exit
exit_scanner_ap(void)
{
    ieee80211_scanner_unregister_all(&ap_default);
}

#ifndef ATH_WLAN_COMBINE
module_init(init_scanner_ap);
module_exit(exit_scanner_ap);
MODULE_AUTHOR("Errno Consulting, Sam Leffler");
MODULE_DESCRIPTION("802.11 wireless support: default ap scanner");
#ifdef MODULE_LICENSE
MODULE_LICENSE("BSD");
#endif
#endif /* #ifndef ATH_WLAN_COMBINE */
