/*! \file
**  \brief 
**
** Copyright (c) 2001 Atsushi Onoe
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
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
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
 *
 * $Id: //depot/sw/releases/7.3_AP/wlan/linux/net80211/ieee80211_beacon.c#7 $
 */
#ifndef EXPORT_SYMTAB
#define	EXPORT_SYMTAB
#endif

/*
 * IEEE 802.11 beacon handling routines
 */
#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif
#include <linux/version.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/if_vlan.h>

#include "if_media.h"
#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_interfaces.h>
/* 
 *  XXX: Include an intra-module function from ieee80211_input.c.
 *       When we move regdomain code out to separate .h/.c files
 *       this should go to that .h file.
 */
struct ieee80211_channel *
    ieee80211_doth_findchan(struct ieee80211vap *vap, u_int8_t chan);

extern u_int32_t ath_htvendorie_enable;
extern u_int32_t ath_htdupie_enable;

static u_int8_t *
ieee80211_beacon_init(struct ieee80211_node *ni, struct ieee80211_beacon_offsets *bo,
    u_int8_t *frm)
{
    struct ieee80211vap *vap = ni->ni_vap;
    struct ieee80211com *ic = ni->ni_ic;
    u_int16_t capinfo;
    struct ieee80211_rateset *rs = &ni->ni_rates;

    KASSERT(ic->ic_bsschan != IEEE80211_CHAN_ANYC, ("no bss chan"));

    memset(frm, 0, 8);	/* XXX timestamp is set by hardware/driver */
    frm += 8;
    *(u_int16_t *)frm = htole16(ni->ni_intval);
    frm += 2;
    
    if (vap->iv_opmode == IEEE80211_M_IBSS)
        capinfo = IEEE80211_CAPINFO_IBSS;
    else
        capinfo = IEEE80211_CAPINFO_ESS;
    if (vap->iv_flags & IEEE80211_F_PRIVACY)
        capinfo |= IEEE80211_CAPINFO_PRIVACY;
    if ((ic->ic_flags & IEEE80211_F_SHPREAMBLE) &&
        IEEE80211_IS_CHAN_2GHZ(ic->ic_bsschan))
        capinfo |= IEEE80211_CAPINFO_SHORT_PREAMBLE;
    if (ic->ic_flags & IEEE80211_F_SHSLOT)
        capinfo |= IEEE80211_CAPINFO_SHORT_SLOTTIME;
    if (ic->ic_flags & IEEE80211_F_DOTH)
        capinfo |= IEEE80211_CAPINFO_SPECTRUM_MGMT;
    bo->bo_caps = (u_int16_t *)frm;
    *(u_int16_t *)frm = htole16(capinfo);
    frm += 2;
    *frm++ = IEEE80211_ELEMID_SSID;
    if ((vap->iv_flags & IEEE80211_F_HIDESSID) == 0)
    {
        *frm++ = ni->ni_esslen;
        memcpy(frm, ni->ni_essid, ni->ni_esslen);
        frm += ni->ni_esslen;
    }
    else
        *frm++ = 0;

    bo->bo_rates = frm;
    frm = ieee80211_add_rates(frm, rs);

    /* XXX better way to check this? */
    if (!IEEE80211_IS_CHAN_FHSS(ic->ic_bsschan))
    {
        *frm++ = IEEE80211_ELEMID_DSPARMS;
        *frm++ = 1;
        *frm++ = ieee80211_chan2ieee(ic, ic->ic_bsschan);
    }
    
    bo->bo_tim = frm;
    
    if (vap->iv_opmode == IEEE80211_M_IBSS)
    {
        *frm++ = IEEE80211_ELEMID_IBSSPARMS;
        *frm++ = 2;
        *frm++ = 0;
        *frm++ = 0;		/* TODO: ATIM window */
        bo->bo_tim_len = 0;
    }
    else
    {
        struct ieee80211_tim_ie *tie = (struct ieee80211_tim_ie *) frm;

        tie->tim_ie = IEEE80211_ELEMID_TIM;
        tie->tim_len = 4;	/* length */
        tie->tim_count = 0;	/* DTIM count */
        tie->tim_period = vap->iv_dtim_period;	/* DTIM period */
        tie->tim_bitctl = 0;	/* bitmap control */
        tie->tim_bitmap[0] = 0;	/* Partial Virtual Bitmap */
        frm += sizeof(struct ieee80211_tim_ie);
        bo->bo_tim_len = 1;
    }
    bo->bo_tim_trailer = frm;

    if (vap->iv_opmode == IEEE80211_M_HOSTAP && ((ic->ic_flags & IEEE80211_F_DOTH) || 
	     (ic->ic_flags_ext & IEEE80211_FEXT_COUNTRYIE))) 
    {
             frm = ieee80211_add_country(frm, ic);
    }
    if (vap->iv_opmode == IEEE80211_M_HOSTAP && (ic->ic_flags & IEEE80211_F_DOTH))
    {
        *frm++ = IEEE80211_ELEMID_PWRCNSTR;
        *frm++ = 1;
        *frm++ = IEEE80211_PWRCONSTRAINT_VAL(ic);
    }
    bo->bo_chanswitch = frm;
    if (IEEE80211_IS_CHAN_ANYG(ic->ic_bsschan) ||
        IEEE80211_IS_CHAN_11NG(ic->ic_bsschan))
    {
        bo->bo_erp = frm;
        frm = ieee80211_add_erp(frm, ic);
    }
    if (vap->iv_flags & IEEE80211_F_WPA)
        frm = ieee80211_add_wpa(frm, vap);
    frm = ieee80211_add_xrates(frm, rs);
#ifdef E_CSA
    bo->bo_extchanswitch = frm;
#endif /* E_CSA */
    if (vap->iv_flags & IEEE80211_F_WME)
    {
        bo->bo_wme = frm;
        frm = ieee80211_add_wme_param(frm, &ic->ic_wme, IEEE80211_VAP_UAPSD_ENABLED(vap));
        vap->iv_flags &= ~IEEE80211_F_WMEUPDATE;
    }
    if (IEEE80211_IS_CHAN_11N(ic->ic_bsschan) && ieee80211vap_is_htallowed(vap))
    {
        bo->bo_htcap = frm;
        frm = ieee80211_add_htcap(frm, ni);

        bo->bo_htinfo = frm;
        frm = ieee80211_add_htinfo(frm, ni);

       if (!(ic->ic_flags_ext & IEEE80211_FEXT_COEXT_DISABLE)) {
            bo->bo_obss_scan = frm;
            frm = ieee80211_add_obss_scan(frm, ni);

            bo->bo_extcap = frm;
            frm = ieee80211_add_extcap(frm, ni);
        }
    }
    bo->bo_ath_caps = frm;

    if (vap->iv_bss && vap->iv_bss->ni_ath_flags)
    {
        frm = ieee80211_add_athAdvCap(frm, vap->iv_bss->ni_ath_flags,
            vap->iv_bss->ni_ath_defkeyindex);
    }
    else
    {
        frm = ieee80211_add_athAdvCap(frm, 0, IEEE80211_INVAL_DEFKEY);
    }

    /* Insert ieee80211_ie_ath_extcap IE to beacon */
    frm = ieee80211_add_athextcap(frm, ic->ic_ath_extcap, ic->ic_weptkipaggr_rxdelim);

    bo->bo_xr = frm;
#ifdef ATH_SUPERG_XR
    if(vap->iv_ath_cap & IEEE80211_ATHC_XR)	/* XR */
        frm=ieee80211_add_xr_param(frm,vap);
#endif

    bo->bo_appie_buf = frm;
    bo->bo_appie_buf_len = 0;

    bo->bo_tim_trailerlen = frm - bo->bo_tim_trailer;
    bo->bo_chanswitch_trailerlen = frm - bo->bo_chanswitch;
#ifdef E_CSA
    bo->bo_extchanswitch_trailerlen = frm - bo->bo_extchanswitch;
#endif /* E_CSA */

    return frm;
}


/*
 * Allocate a beacon frame and fillin the appropriate bits.
 */
struct sk_buff *
    ieee80211_beacon_alloc(struct ieee80211_node *ni,
        struct ieee80211_beacon_offsets *bo)
{
    struct ieee80211vap *vap = ni->ni_vap;
    struct ieee80211com *ic = ni->ni_ic;
    struct ieee80211_frame *wh;
    struct sk_buff *skb;
    int pktlen;
    u_int8_t *frm;
    struct ieee80211_rateset *rs;
    int htcaplen, htinfolen;

    /*
     * beacon frame format
     *	[8] time stamp
     *	[2] beacon interval
     *	[2] cabability information
     *	[tlv] ssid
     *	[tlv] supported rates
     *	[3] parameter set (DS)
     *	[tlv] parameter set (IBSS/TIM)
     *	[tlv] country code, if present
     *	[3] power constraint
     *	[5] channel switch announcement
     *	[tlv] extended rate phy (ERP)
     *	[tlv] extended supported rates
     *	[tlv] WME parameters
     *	[tlv] WPA/RSN parameters
     *	[tlv] HT Capabilities
     *	[tlv] HT Information
     *	[tlv] Atheros Advanced Capabilities
     *	[tlv] AtherosXR parameters
     * XXX Vendor-specific OIDs (e.g. Atheros)
     * NB: we allocate the max space required for the TIM bitmap.
     */
    rs = &ni->ni_rates;
    if(ath_htvendorie_enable)
    {
        htcaplen  = sizeof(struct vendor_ie_htcap);
        htinfolen = sizeof(struct vendor_ie_htinfo);
    }
    else
    {
        htcaplen  = sizeof(struct ieee80211_ie_htcap);
        htinfolen = sizeof(struct ieee80211_ie_htinfo);
    }

    if(ath_htdupie_enable)
    {
        htcaplen  += sizeof(struct ieee80211_ie_htcap);
        htinfolen += sizeof(struct ieee80211_ie_htinfo);
    }

    pktlen =   8					/* time stamp */
        + sizeof(u_int16_t)			/* beacon interval */
        + sizeof(u_int16_t)			/* capabilities */
        + 2 + ni->ni_esslen			/* ssid */
        + 2 + IEEE80211_RATE_SIZE		/* supported rates */
        + 2 + 1				/* DS parameters */
        + 2 + 4 + vap->iv_tim_len		/* DTIM/IBSSPARMS */
        + ic->ic_country_ie.country_len + 2	/* country code */
        + 3					/* power constraint */
        + 5					/* channel switch */
        + 2 + 1				/* ERP */
        + 2 + (IEEE80211_RATE_MAXSIZE - IEEE80211_RATE_SIZE)
        + (vap->iv_caps & IEEE80211_C_WME ?	/* WME */
        sizeof(struct ieee80211_wme_param) : 0)
            + (vap->iv_caps & IEEE80211_C_WPA ?	/* WPA 1+2 */
            2*sizeof(struct ieee80211_ie_wpa) : 0)
                + htcaplen 
                + htinfolen
                + sizeof(struct ieee80211_ie_obss_scan)
                + sizeof(struct ieee80211_ie_ext_cap)
                + sizeof(struct ieee80211_ie_athAdvCap)
#ifdef ATH_SUPERG_XR
                + (ic->ic_ath_cap & IEEE80211_ATHC_XR ?	/* XR */
                    sizeof(struct ieee80211_xr_param) : 0)
#endif
                /* Atheros Extended Capabilities IE */
                +  sizeof (struct ieee80211_ie_ath_extcap)
                    + IEEE80211_APPIE_MAX  /* APP_IE buffer */
                        ;
    skb = ieee80211_getmgtframe(&frm, pktlen);
    if (skb == NULL)
    {
        IEEE80211_NOTE(vap, IEEE80211_MSG_ANY, ni,
            "%s: cannot get buf; size %u", __func__, pktlen);
        vap->iv_stats.is_tx_nobuf++;
        return NULL;
    }

    frm = ieee80211_beacon_init(ni, bo, frm);

    skb_trim(skb, frm - skb->data);

    wh = (struct ieee80211_frame *)
            skb_push(skb, sizeof(struct ieee80211_frame));
    wh->i_fc[0] = IEEE80211_FC0_VERSION_0 | IEEE80211_FC0_TYPE_MGT |
        IEEE80211_FC0_SUBTYPE_BEACON;
    wh->i_fc[1] = IEEE80211_FC1_DIR_NODS;
    *(u_int16_t *)wh->i_dur = 0;
    IEEE80211_ADDR_COPY(wh->i_addr1, ic->ic_dev->broadcast);
    IEEE80211_ADDR_COPY(wh->i_addr2, vap->iv_myaddr);
    IEEE80211_ADDR_COPY(wh->i_addr3, ni->ni_bssid);
    *(u_int16_t *)wh->i_seq = 0;

    return skb;
}
EXPORT_SYMBOL_C(ieee80211_beacon_alloc);

/*
 * Update the dynamic parts of a beacon frame based on the current state.
 */
int
ieee80211_beacon_update(struct ieee80211_node *ni,
    struct ieee80211_beacon_offsets *bo, struct sk_buff *skb, int mcast)
{
    struct ieee80211vap *vap = ni->ni_vap;
    struct ieee80211com *ic = ni->ni_ic;
    int len_changed = 0;
    u_int16_t capinfo;

    IEEE80211_BEACON_LOCK(ic);

    if ((ic->ic_flags & IEEE80211_F_DOTH) && (vap->iv_flags & IEEE80211_F_CHANSWITCH) &&
        (vap->iv_chanchange_count == ic->ic_chanchange_tbtt))
    {
        u_int8_t *frm;
        struct ieee80211_channel *c;

        vap->iv_chanchange_count = 0;

        IEEE80211_DPRINTF(vap, IEEE80211_MSG_DOTH, "%s: reinit beacon\n", __func__);

        /* 
         * NB: ic_bsschan is in the DSPARMS beacon IE, so must set this
         *     prior to the beacon re-init, below.
         */
        c = ieee80211_doth_findchan(vap, ic->ic_chanchange_chan);
        if (c == NULL)
        {
            IEEE80211_DPRINTF(vap, IEEE80211_MSG_DOTH, "%s: find channel failure\n", __func__);
            return 0;
        }
        ic->ic_bsschan = c;

        skb_pull(skb, sizeof(struct ieee80211_frame));
        skb_trim(skb, 0);
        frm = skb->data;
        skb_put(skb, ieee80211_beacon_init(ni, bo, frm) - frm);
        skb_push(skb, sizeof(struct ieee80211_frame));

        vap->iv_flags &= ~IEEE80211_F_CHANSWITCH;
        ic->ic_flags &= ~IEEE80211_F_CHANSWITCH;

        /* NB: only for the first VAP to get here */
        if (ic->ic_curchan != c)
        {
            ic->ic_curchan = c;
            ic->ic_set_channel(ic);
        }

        len_changed = 1;
    }

    /* XXX faster to recalculate entirely or just changes? */
    if (vap->iv_opmode == IEEE80211_M_IBSS)
        capinfo = IEEE80211_CAPINFO_IBSS;
    else
        capinfo = IEEE80211_CAPINFO_ESS;
    if (vap->iv_flags & IEEE80211_F_PRIVACY)
        capinfo |= IEEE80211_CAPINFO_PRIVACY;
    if ((ic->ic_flags & IEEE80211_F_SHPREAMBLE) &&
        IEEE80211_IS_CHAN_2GHZ(ic->ic_bsschan))
        capinfo |= IEEE80211_CAPINFO_SHORT_PREAMBLE;
    if (ic->ic_flags & IEEE80211_F_SHSLOT)
        capinfo |= IEEE80211_CAPINFO_SHORT_SLOTTIME;
    if (ic->ic_flags & IEEE80211_F_DOTH)
        capinfo |= IEEE80211_CAPINFO_SPECTRUM_MGMT;

    *bo->bo_caps = htole16(capinfo);

    if (vap->iv_flags & IEEE80211_F_WME)
    {
        struct ieee80211_wme_state *wme = &ic->ic_wme;

        /*
        * Check for agressive mode change.  When there is
        * significant high priority traffic in the BSS
        * throttle back BE traffic by using conservative
        * parameters.  Otherwise BE uses agressive params
        * to optimize performance of legacy/non-QoS traffic.
        */
        if (vap->iv_ath_cap & IEEE80211_ATHC_WME)
        {
            if (wme->wme_flags & WME_F_AGGRMODE)
            {
                if (wme->wme_hipri_traffic >
                    wme->wme_hipri_switch_thresh)
                {
                    IEEE80211_NOTE(vap, IEEE80211_MSG_WME, ni,
                        "%s: traffic %u, disable aggressive mode",
                        __func__, wme->wme_hipri_traffic);
                    wme->wme_flags &= ~WME_F_AGGRMODE;
                    ieee80211_wme_updateparams_locked(vap);
                    wme->wme_hipri_traffic =
                        wme->wme_hipri_switch_hysteresis;
                }
                else
                    wme->wme_hipri_traffic = 0;
            }
            else
            {
                if (wme->wme_hipri_traffic <=
                    wme->wme_hipri_switch_thresh)
                {
                    IEEE80211_NOTE(vap, IEEE80211_MSG_WME, ni,
                        "%s: traffic %u, enable aggressive mode",
                        __func__, wme->wme_hipri_traffic);
                    wme->wme_flags |= WME_F_AGGRMODE;
                    ieee80211_wme_updateparams_locked(vap);
                    wme->wme_hipri_traffic = 0;
                }
                else
                    wme->wme_hipri_traffic =
                        wme->wme_hipri_switch_hysteresis;
            }
        }
        /* XXX multi-bss */
        if (vap->iv_flags & IEEE80211_F_WMEUPDATE)
        {
            (void) ieee80211_add_wme_param(bo->bo_wme, wme, IEEE80211_VAP_UAPSD_ENABLED(vap));
            vap->iv_flags &= ~IEEE80211_F_WMEUPDATE;
        }
    }

    /* update beacon sequence number information */
    {
        struct ieee80211_frame  *wh;
        wh = (struct ieee80211_frame  *)skb->data;
        *(u_int16_t *)(wh->i_seq) = htole16(vap->sequence << IEEE80211_SEQ_SEQ_SHIFT);
        /* max 4095 for sequence number[12 bits] */
        vap->sequence = (vap->sequence < 4095) ? (vap->sequence+1):0;
        /* END update beacon sequence number information */
    }



    /* update 11n info */
    if (IEEE80211_IS_CHAN_11N(ic->ic_bsschan) && ieee80211vap_is_htallowed(vap))
    {
        struct ieee80211_ie_htinfo_cmn *htinfo;
        int txchwidth =
            (ic->ic_cwm.cw_width == IEEE80211_CWM_WIDTH40) ?
            IEEE80211_HTINFO_TXWIDTH_2040 : IEEE80211_HTINFO_TXWIDTH_20;

        switch(vap->iv_chwidth)
        {
            case 1:
                txchwidth = IEEE80211_HTINFO_TXWIDTH_20;
                break;         
            case 2:
                txchwidth = IEEE80211_HTINFO_TXWIDTH_2040;
                break;
            default:
                break;
        }
        if(ath_htvendorie_enable)
            htinfo = &((struct vendor_ie_htinfo *)bo->bo_htinfo)->hi_ie;
        else
            htinfo = &((struct ieee80211_ie_htinfo *)bo->bo_htinfo)->hi_ie;

        htinfo->hi_txchwidth = txchwidth;

        if(ath_htdupie_enable)
        {
            struct ieee80211_ie_htinfo *duphtinfo =
                    (struct ieee80211_ie_htinfo *)(htinfo + 1);

            duphtinfo->hi_ie.hi_txchwidth = txchwidth;
        }
    } 

    if (vap->iv_opmode == IEEE80211_M_HOSTAP)
    {	/* NB: no IBSS support*/
        struct ieee80211_tim_ie *tie =
                (struct ieee80211_tim_ie *) bo->bo_tim;
        if (vap->iv_flags & IEEE80211_F_TIMUPDATE)
        {
            u_int timlen, timoff, i;
            /*
            * ATIM/DTIM needs updating.  If it fits in the
            * current space allocated then just copy in the
            * new bits.  Otherwise we need to move any trailing
            * data to make room.  Note that we know there is
            * contiguous space because ieee80211_beacon_allocate
            * insures there is space in the mbuf to write a
            * maximal-size virtual bitmap (based on ic_max_aid).
            */
            /*
            * Calculate the bitmap size and offset, copy any
            * trailer out of the way, and then copy in the
            * new bitmap and update the information element.
            * Note that the tim bitmap must contain at least
            * one byte and any offset must be even.
            */
            if (vap->iv_ps_pending != 0)
            {
                timoff = 128;		/* impossibly large */
                for (i = 0; i < vap->iv_tim_len; i++)
                    if (vap->iv_tim_bitmap[i])
                    {
                        timoff = i &~ 1;
                        break;
                    }
                    KASSERT(timoff != 128, ("tim bitmap empty!"));
                for (i = vap->iv_tim_len-1; i >= timoff; i--)
                    if (vap->iv_tim_bitmap[i])
                        break;
                    timlen = 1 + (i - timoff);
            }
            else
            {
                timoff = 0;
                timlen = 1;
            }
            if (timlen != bo->bo_tim_len)
            {
                /* copy up/down trailer */
                int trailer_adjust = (int)(long)(tie->tim_bitmap+timlen) - (int)(long)(bo->bo_tim_trailer);
                memmove(tie->tim_bitmap+timlen, bo->bo_tim_trailer,
                    bo->bo_tim_trailerlen);
                bo->bo_tim_trailer = tie->tim_bitmap+timlen;
                bo->bo_chanswitch += trailer_adjust;
#ifdef E_CSA
                bo->bo_extchanswitch += trailer_adjust;
#endif /* E_CSA */
                bo->bo_wme += trailer_adjust;
                bo->bo_erp += trailer_adjust;
                bo->bo_ath_caps += trailer_adjust;
                bo->bo_xr += trailer_adjust;
                bo->bo_appie_buf += trailer_adjust;
                if (timlen > bo->bo_tim_len)
                    skb_put(skb, timlen - bo->bo_tim_len);
                else
                    skb_trim(skb, skb->len - (bo->bo_tim_len - timlen));
                bo->bo_tim_len = timlen;

                /* update information element */
                tie->tim_len = 3 + timlen;
                tie->tim_bitctl = timoff;
                len_changed = 1;
            }
            memcpy(tie->tim_bitmap, vap->iv_tim_bitmap + timoff,
                bo->bo_tim_len);

            vap->iv_flags &= ~IEEE80211_F_TIMUPDATE;

            IEEE80211_NOTE(vap, IEEE80211_MSG_POWER, ni,
                "%s: TIM updated, pending %u, off %u, len %u",
                __func__, vap->iv_ps_pending, timoff, timlen);
        }
        /* count down DTIM period */
        if (tie->tim_count == 0)
            tie->tim_count = tie->tim_period - 1;
        else
            tie->tim_count--;
        /* update state for buffered multicast frames on DTIM */
        if (mcast && (tie->tim_count == 0 || tie->tim_period == 1))
            tie->tim_bitctl |= 1;
        else
            tie->tim_bitctl &= ~1;

        if ((ic->ic_flags & IEEE80211_F_DOTH) && (ic->ic_flags & IEEE80211_F_CHANSWITCH))
        {

            if (!vap->iv_chanchange_count)
            {
                vap->iv_flags |= IEEE80211_F_CHANSWITCH;

                /* copy out trailer to open up a slot */
                memmove(bo->bo_chanswitch + IEEE80211_CHANSWITCHANN_BYTES, 
                    bo->bo_chanswitch, bo->bo_chanswitch_trailerlen);

                /* add ie in opened slot */
                bo->bo_chanswitch[0] = IEEE80211_ELEMID_CHANSWITCHANN;
                bo->bo_chanswitch[1] = 3; /* fixed length */
                bo->bo_chanswitch[2] = 1; /* stations get off for now */
                bo->bo_chanswitch[3] = ic->ic_chanchange_chan;
                bo->bo_chanswitch[4] = ic->ic_chanchange_tbtt;

                /* update the trailer lens */
                bo->bo_chanswitch_trailerlen += IEEE80211_CHANSWITCHANN_BYTES;
                bo->bo_tim_trailerlen += IEEE80211_CHANSWITCHANN_BYTES;
                bo->bo_wme += IEEE80211_CHANSWITCHANN_BYTES;
                bo->bo_erp += IEEE80211_CHANSWITCHANN_BYTES;
#ifdef E_CSA
                bo->bo_extchanswitch += IEEE80211_CHANSWITCHANN_BYTES;
#endif /* E_CSA */
                bo->bo_ath_caps += IEEE80211_CHANSWITCHANN_BYTES;
                bo->bo_xr += IEEE80211_CHANSWITCHANN_BYTES;
                bo->bo_appie_buf += IEEE80211_CHANSWITCHANN_BYTES;

                /* indicate new beacon length so other layers may manage memory */
                skb_put(skb, IEEE80211_CHANSWITCHANN_BYTES);
                len_changed = 1;
            }
            else
            {
                bo->bo_chanswitch[4]--;
            }
            IEEE80211_DPRINTF(vap, IEEE80211_MSG_DOTH, "%s: CHANSWITCH IE, change in %d\n",
                __func__, bo->bo_chanswitch[4]);
        }
#ifdef ATH_SUPERG_XR
        if(vap->iv_flags & IEEE80211_F_XRUPDATE)
        {
            if(vap->iv_xrvap)
                (void) ieee80211_add_xr_param(bo->bo_xr,vap);
            vap->iv_flags &= ~IEEE80211_F_XRUPDATE;
        }
#endif
        if (((jiffies - ic->ic_time_nonerp_present) >= IEEE80211_INACT_NONERP * HZ)
            && ic->ic_nonerpsta == 0 && (ic->ic_flags & IEEE80211_F_USEPROT))
        {
            struct ieee80211vap *tmpvap;

            ic->ic_flags &= ~IEEE80211_F_USEPROT;
            ieee80211_enumerate_vaps(tmpvap, ic) {
                tmpvap->iv_flags_ext |= IEEE80211_FEXT_ERPUPDATE;
            }
            ic->ic_update_protmode(ic);
            ieee80211_set_shortslottime(ic, 1);
        }
        if (vap->iv_flags_ext & IEEE80211_FEXT_ERPUPDATE)
        {
            (void) ieee80211_add_erp(bo->bo_erp, ic);
            vap->iv_flags_ext &= ~IEEE80211_FEXT_ERPUPDATE;
        }

#ifdef E_CSA
        if ((ic->ic_flags & IEEE80211_F_DOTH) && (ic->ic_flags & IEEE80211_F_CHANSWITCH))
        {

            if (!vap->iv_chanchange_count)
            {
                /* copy out trailer to open up a slot */
                memmove(bo->bo_extchanswitch + IEEE80211_EXTCHANSWITCHANN_BYTES, 
                    bo->bo_extchanswitch, bo->bo_extchanswitch_trailerlen);

                /* add ie in opened slot */
                bo->bo_extchanswitch[0] = IEEE80211_ELEMID_EXTCHANSWITCHANN;
                bo->bo_extchanswitch[1] = 4; /* fixed length */
                bo->bo_extchanswitch[2] = 1; /* stations get off for now */
                /*
                 * XXX: bo_extchanswitch[3] should be regulatory class instead
                 * of country code. Currently there is no functionality to retrieve
                 * the regulatory classe from HAL. Need to correct this later when
                 * we fix the IEEE80211_ELEMID_EXTCHANSWITCHANN to ANA defined
                 * value.
                 */
                bo->bo_extchanswitch[3] = ic->ic_country_code;
                bo->bo_extchanswitch[4] = ic->ic_chanchange_chan;
                bo->bo_extchanswitch[5] = ic->ic_chanchange_tbtt;

                /* update the trailer lens */
                bo->bo_extchanswitch_trailerlen += IEEE80211_EXTCHANSWITCHANN_BYTES;
                bo->bo_ath_caps += IEEE80211_EXTCHANSWITCHANN_BYTES;
                bo->bo_appie_buf += IEEE80211_EXTCHANSWITCHANN_BYTES;

                /* indicate new beacon length so other layers may manage memory */
                skb_put(skb, IEEE80211_EXTCHANSWITCHANN_BYTES);
                len_changed = 1;
            }
            else
            {
                bo->bo_extchanswitch[5]--;
            }
            IEEE80211_DPRINTF(vap, IEEE80211_MSG_DOTH, "%s: EXT CHANSWITCH IE, change in %d\n",
                __func__, bo->bo_extchanswitch[5]);
        }
#endif /* E_CSA */
        if ((ic->ic_flags & IEEE80211_F_DOTH) && (ic->ic_flags & IEEE80211_F_CHANSWITCH))
        {
            vap->iv_chanchange_count++;
        }

        /* Age out non-HT protection if we haven't detected beacons
        * from a neighboring non-HT AP in the last 10 seconds and
        * there aren't any non-HT stations currently associated.
        */
        if (((jiffies - ic->ic_time_noht_present) >=
            IEEE80211_INACT_HT * HZ) &&
            (ic->ic_flags_ext & IEEE80211_F_NONHT_AP))
        {

            ic->ic_flags_ext &= ~(IEEE80211_F_NONHT_AP);

            /*Schedule a change to the protection opmode in the HT IE*/
            ieee80211_ht_prot(ic);
        }

        /* Implement a change to the protection opmode in the HT IE */
        if (IEEE80211_IS_CHAN_11N(ic->ic_bsschan) && ieee80211vap_is_htallowed(vap)) {
            if (ic->ic_flags_ext & IEEE80211_FEXT_HTUPDATE)
            {
                ic->ic_flags_ext &= ~IEEE80211_FEXT_HTUPDATE;
                ieee80211_add_htinfo(bo->bo_htinfo, ni);
                ieee80211_add_htcap(bo->bo_htcap, ni);
                if (!(ic->ic_flags_ext & IEEE80211_FEXT_COEXT_DISABLE))
                    ieee80211_add_obss_scan(bo->bo_obss_scan, ni);
            }
        }
    }

    if (vap->iv_flags_ext & IEEE80211_FEXT_BR_UPDATE) {
        int n;
        struct ieee80211_rateset *rs = &vap->iv_bss->ni_rates;
        n = rs->rs_nrates;
        if (n > IEEE80211_RATE_SIZE) {
            n = IEEE80211_RATE_SIZE;
        }
        memcpy(bo->bo_rates + 2, rs->rs_rates, n);
        bo->bo_rates[1] = n;
        vap->iv_flags_ext &= ~IEEE80211_FEXT_BR_UPDATE;
    }

    /* if it is a mode change beacon for dynamic turbo case */
    if (((ic->ic_ath_cap & IEEE80211_ATHC_BOOST) != 0) ^
        IEEE80211_IS_CHAN_TURBO(ic->ic_curchan))
        ieee80211_add_athAdvCap(bo->bo_ath_caps, vap->iv_bss->ni_ath_flags,
            vap->iv_bss->ni_ath_defkeyindex);

    /* add APP_IE buffer if app updated it */
    if (vap->iv_flags_ext & IEEE80211_FEXT_APPIE_UPDATE)
    {
        /* adjust the buffer size if the size is changed */
        if(vap->app_ie[IEEE80211_APPIE_FRAME_BEACON].length != bo-> bo_appie_buf_len)
        {
            int diff_len;
            diff_len = vap->app_ie[IEEE80211_APPIE_FRAME_BEACON].length - bo-> bo_appie_buf_len;

            if(diff_len > 0) 
                skb_put(skb,diff_len);
            else
                skb_trim(skb,skb->len + diff_len);

            bo->bo_appie_buf_len = vap->app_ie[IEEE80211_APPIE_FRAME_BEACON].length;
            /* update the trailer lens */
            bo->bo_chanswitch_trailerlen += diff_len;
#ifdef E_CSA
            bo->bo_extchanswitch_trailerlen += diff_len;
#endif /* E_CSA */
            bo->bo_tim_trailerlen += diff_len;

            len_changed = 1;
        }

        memcpy(bo->bo_appie_buf,vap->app_ie[IEEE80211_APPIE_FRAME_BEACON].ie,
            vap->app_ie[IEEE80211_APPIE_FRAME_BEACON].length);

        vap->iv_flags_ext &= ~IEEE80211_FEXT_APPIE_UPDATE;
    }

    IEEE80211_BEACON_UNLOCK(ic);

    return len_changed;
}
EXPORT_SYMBOL_C(ieee80211_beacon_update);
