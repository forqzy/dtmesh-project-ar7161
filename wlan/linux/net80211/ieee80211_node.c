#define macfmt "%02x:%02x:%02x:%02x:%02x:%02x"
#define macstr(m)	m[0], m[1], m[2], m[3], m[4], m[5]
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
 */
#ifndef EXPORT_SYMTAB
#define	EXPORT_SYMTAB
#endif

__FBSDID("$FreeBSD: src/sys/net80211/ieee80211_node.c,v 1.31 2005/01/01 17:48:27 sam Exp $");

/*
 * IEEE 802.11 node handling support.
 */
#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif
#include <linux/version.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/random.h>

#include <osdep.h>
#include "if_media.h"
#include "ieee80211_interfaces.h"
#include <net80211/ieee80211_var.h>
#include <net80211/if_athproto.h>
#include <net80211/ieee80211_sm.h>
#include <net80211/ieee80211_node.h>
#include <net80211/ieee80211_interfaces.h>
/*
 * Association id's are managed with a bit vector.
 */
#define	IEEE80211_AID_SET(_vap, _b) \
	((_vap)->iv_aid_bitmap[IEEE80211_AID(_b) / 32] |= \
		(1 << (IEEE80211_AID(_b) % 32)))
#define	IEEE80211_AID_CLR(_vap, _b) \
	((_vap)->iv_aid_bitmap[IEEE80211_AID(_b) / 32] &= \
		~(1 << (IEEE80211_AID(_b) % 32)))
#define	IEEE80211_AID_ISSET(_vap, _b) \
	((_vap)->iv_aid_bitmap[IEEE80211_AID(_b) / 32] & (1 << (IEEE80211_AID(_b) % 32)))

static int ieee80211_sta_join1(struct ieee80211_node *);

static struct ieee80211_node *node_alloc(struct ieee80211_node_table *,struct ieee80211vap *);
static void node_cleanup(struct ieee80211_node *);
static void node_free(struct ieee80211_node *);
static int8_t node_getrssi(const struct ieee80211_node *ni,  int8_t chain, u_int8_t flags);

static void _ieee80211_free_node(struct ieee80211_node *);
static void node_reclaim(struct ieee80211_node_table *, struct ieee80211_node*);

static void ieee80211_node_timeout(unsigned long);
static void ieee80211_free_node_delay(unsigned long);
static void ieee80211_free_node_delay_cleanup(struct ieee80211com *ic);

static	void ieee80211_node_table_init(struct ieee80211com *ic,
    struct ieee80211_node_table *nt, const char *name, int inact);
static	void ieee80211_node_table_cleanup(struct ieee80211_node_table *nt);
static	void ieee80211_node_table_reset(struct ieee80211_node_table *,
    struct ieee80211vap *);
static void ieee80211_node_wds_ageout(unsigned long data);
void ieee80211_create_adhocbssid(struct ieee80211vap *, u_int8_t *);

static void ieee80211_send_deauth(void *, struct ieee80211_node *);

MALLOC_DEFINE(M_80211_NODE, "80211node", "802.11 node state");

void
ieee80211_node_attach(struct ieee80211com *ic)
{

    ieee80211_node_table_init(ic, &ic->ic_sta, "station",
        IEEE80211_INACT_INIT);
    init_timer(&ic->ic_inact);
    ic->ic_inact.function = ieee80211_node_timeout;
    ic->ic_inact.data = (unsigned long) ic;
    ic->ic_inact.expires = jiffies + IEEE80211_INACT_WAIT*HZ;
    add_timer(&ic->ic_inact);

    init_timer(&ic->ic_free);
    ic->ic_free.function = ieee80211_free_node_delay;
    ic->ic_free.data = (unsigned long) ic;
    ic->ic_free.expires = jiffies + IEEE80211_NODE_FREE_WAIT*HZ;
    add_timer(&ic->ic_free);

    ic->ic_node_alloc = node_alloc;
    ic->ic_node_free = node_free;
    ic->ic_node_cleanup = node_cleanup;
    ic->ic_node_getrssi = node_getrssi;
}

void
ieee80211_node_detach(struct ieee80211com *ic)
{

    del_timer(&ic->ic_inact);
    del_timer(&ic->ic_free);

    /*
     * In case if there are nodes in the deleted node
     * list to be deleted and timer is not active and deleted
     * then we've to do the node cleanup before exit, otherwise
     * the kernel free mem will deplete gradually with every apup/apdown
     * finally resulting in page allocation/out of memory error and
     * kernel panic.
     *
     * The cleanup call below solves the mem leak issue described 
     * above.
     */
    ieee80211_free_node_delay_cleanup(ic);

    ieee80211_node_table_cleanup(&ic->ic_sta);
}

void
ieee80211_node_vattach(struct ieee80211vap *vap)
{
    /* default station inactivity timer setings */
    vap->iv_inact_init = IEEE80211_INACT_INIT;
    vap->iv_inact_auth = IEEE80211_INACT_AUTH;
    vap->iv_inact_run = IEEE80211_INACT_RUN;
    vap->iv_inact_probe = IEEE80211_INACT_PROBE;
}

void
ieee80211_node_latevattach(struct ieee80211vap *vap)
{
    struct ieee80211com *ic = vap->iv_ic;
    struct ieee80211_rsnparms *rsn;

    /*
    * Allocate these only if needed.  Beware that we
    * know adhoc mode doesn't support ATIM yet...
    */
    if (vap->iv_opmode == IEEE80211_M_HOSTAP)
    {
        if (vap->iv_max_aid == 0)
            vap->iv_max_aid = IEEE80211_AID_DEF;
        else if (vap->iv_max_aid > IEEE80211_AID_MAX)
            vap->iv_max_aid = IEEE80211_AID_MAX;
        MALLOC(vap->iv_aid_bitmap, u_int32_t *,
            howmany(vap->iv_max_aid, 32) * sizeof(u_int32_t),
            M_DEVBUF, M_NOWAIT | M_ZERO);
        if (vap->iv_aid_bitmap == NULL)
        {
            /* XXX no way to recover */
            printf("%s: no memory for AID bitmap!\n", __func__);
            vap->iv_max_aid = 0;
        }
    }

    ieee80211_reset_bss(vap);
    /*
     * Setup "global settings" in the bss node so that
     * each new station automatically inherits them.
     */
    rsn = &vap->iv_bss->ni_rsn;
    /* WEP, TKIP, and AES-CCM are always supported */
    rsn->rsn_ucastcipherset |= 1<<IEEE80211_CIPHER_WEP;
    rsn->rsn_ucastcipherset |= 1<<IEEE80211_CIPHER_TKIP;
    rsn->rsn_ucastcipherset |= 1<<IEEE80211_CIPHER_AES_CCM;
    if (ic->ic_caps & IEEE80211_C_AES)
        rsn->rsn_ucastcipherset |= 1<<IEEE80211_CIPHER_AES_OCB;
    if (ic->ic_caps & IEEE80211_C_CKIP)
        rsn->rsn_ucastcipherset |= 1<<IEEE80211_CIPHER_CKIP;
    /*
     * Default unicast cipher to WEP for 802.1x use.  If
     * WPA is enabled the management code will set these
     * values to reflect.
     */
    rsn->rsn_ucastcipher = IEEE80211_CIPHER_WEP;
    rsn->rsn_ucastkeylen = 104 / NBBY;
    /*
     * WPA says the multicast cipher is the lowest unicast
     * cipher supported.  But we skip WEP which would
     * otherwise be used based on this criteria.
     */
    rsn->rsn_mcastcipher = IEEE80211_CIPHER_TKIP;
    rsn->rsn_mcastkeylen = 128 / NBBY;

    /*
     * We support both WPA-PSK and 802.1x; the one used
     * is determined by the authentication mode and the
     * setting of the PSK state.
     */
    rsn->rsn_keymgmtset = WPA_ASE_8021X_UNSPEC | WPA_ASE_8021X_PSK;
    rsn->rsn_keymgmt = WPA_ASE_8021X_PSK;

    vap->iv_auth = ieee80211_authenticator_get(vap->iv_bss->ni_authmode);
}

void
ieee80211_node_vdetach(struct ieee80211vap *vap)
{
    struct ieee80211com *ic = vap->iv_ic;

    ieee80211_node_table_reset(&ic->ic_sta, vap);
    if (vap->iv_bss != NULL)
    {
        ieee80211_free_node(vap->iv_bss);
        vap->iv_bss = NULL;
    }
    if (vap->iv_aid_bitmap != NULL)
    {
        FREE(vap->iv_aid_bitmap, M_DEVBUF);
        vap->iv_aid_bitmap = NULL;
    }
}

void
ieee80211_iterate_wds_nodes(struct ieee80211_node_table *nt,
	ieee80211_iter_func *f, void *arg)
{
	int			i;
	u_int			gen;
	struct ieee80211_node	*ni;
	struct ieee80211_wds_addr *wds;

	IEEE80211_SCAN_LOCK(nt);
	gen = nt->nt_scangen++;
restart:
	IEEE80211_NODE_LOCK(nt);
	for (i = 0; i < IEEE80211_NODE_HASHSIZE; i++) {
		LIST_FOREACH(wds, &nt->nt_wds_hash[i], wds_hash) {
			ni = wds->wds_ni;
			if (ni->ni_scangen != gen) {
				ni->ni_scangen = gen;
#ifdef NODE_FREE_DEBUG
				ieee80211_ref_node(ni, __func__);
#else
				ieee80211_ref_node(ni);
#endif
				IEEE80211_NODE_UNLOCK(nt);
				if (f != NULL) {
					(*f)(arg, ni);
				}
				ieee80211_free_node(ni);
				goto restart;
			}
		}
	}
	IEEE80211_NODE_UNLOCK(nt);

	IEEE80211_SCAN_UNLOCK(nt);
}

void
ieee80211_proxy_deauth(struct ieee80211_node *ni, int rc)
{
	ieee80211_pd_t pd = { ni, rc };
	ieee80211_iterate_wds_nodes(
		&ni->ni_ic->ic_sta,
		ieee80211_send_deauth,
		&pd);
}

/*
 * ieee80211_iterate_nodes will hold and free the node's reference
 */
static void
ieee80211_send_deauth(void *arg, struct ieee80211_node *i_ni)
{
#define senderr(_x, _v) do { vap->iv_stats._v++; ret = _x; goto bad; } while (0)

	struct ieee80211_node	*ni = ((ieee80211_pd_t *)arg)->ni;
	int			rc = ((ieee80211_pd_t *)arg)->rc;
	struct ieee80211vap	*vap = i_ni->ni_vap;
	struct ieee80211com	*ic = i_ni->ni_ic;
	struct ieee80211_frame	*wh;
	struct ieee80211_cb	*cb;
	struct sk_buff		*skb;
	u_int8_t		*frm, mac[IEEE80211_ADDR_LEN];
	int			ret;

	KASSERT(i_ni != NULL, ("null node"));

	if (!vap) {
		return;
	}

	skb = ieee80211_getmgtframe(&frm, sizeof(u_int16_t));

	if (skb == NULL) {
		senderr(ENOMEM, is_tx_nobuf);
	}

#ifdef NODE_FREE_DEBUG
	ieee80211_ref_node(i_ni, __func__);
#else
	ieee80211_ref_node(i_ni);
#endif
	/*
	 * Copy the sta-vap MAC address 
	 */
	IEEE80211_ADDR_COPY(mac, i_ni->ni_macaddr);

	*(u_int16_t *)frm = htole16(rc);       /* reason */

	cb = (struct ieee80211_cb *)skb->cb;

	cb->ni = i_ni;

	wh = (struct ieee80211_frame *)
		skb_push(skb, sizeof(struct ieee80211_frame));

	wh->i_fc[0] = IEEE80211_FC0_VERSION_0 | IEEE80211_FC0_SUBTYPE_DEAUTH;

	wh->i_fc[1] = IEEE80211_FC1_DIR_NODS;

	IEEE80211_ADDR_COPY(wh->i_addr1, mac);	// da
	IEEE80211_ADDR_COPY(wh->i_addr2, ni->ni_macaddr);	// sa
	IEEE80211_ADDR_COPY(wh->i_addr3, mac);	// bssid

	*(u_int16_t *)&wh->i_dur[0] = 0;
	/* NB: use non-QoS tid */
	*(u_int16_t *)&wh->i_seq[0] =
		htole16(i_ni->ni_txseqs[0] << IEEE80211_SEQ_SEQ_SHIFT);
	i_ni->ni_txseqs[0]++;
	IEEE80211_NOTE(vap, IEEE80211_MSG_AUTH, ni,
		"%s: deauthenticate " macfmt " " macfmt " (reason %d)\n", __func__, macstr(mac), macstr(ni->ni_macaddr), rc);

	/* XXX power management */

	if ((cb->flags & M_LINK0) != 0 && i_ni->ni_challenge != NULL)
	{
		cb->flags &= ~M_LINK0;
		IEEE80211_NOTE_MAC(vap, IEEE80211_MSG_AUTH, wh->i_addr1,
			"encrypting frame (%s)", __func__);
		wh->i_fc[1] |= IEEE80211_FC1_WEP;
	}

	if (IEEE80211_VAP_IS_SLEEPING(i_ni->ni_vap))
		wh->i_fc[1] |= IEEE80211_FC1_PWR_MGT;

	IEEE80211_NODE_STAT(i_ni, tx_mgmt);
	ieee80211_pwrsave_wakeup(vap, TRANSMIT);
	vap->iv_lastdata = jiffies;
	(void) ic->ic_mgtstart(ic, skb);

	return;
bad:
	return;
#undef senderr
}

/*
 * Port authorize/unauthorize interfaces for use by an authenticator.
 */

void
ieee80211_node_authorize(struct ieee80211_node *ni)
{
    ni->ni_flags |= IEEE80211_NODE_AUTH;
    ni->ni_inact_reload = ni->ni_vap->iv_inact_run;
}
EXPORT_SYMBOL_C(ieee80211_node_authorize);

void
ieee80211_node_unauthorize(struct ieee80211_node *ni)
{
    ni->ni_flags &= ~IEEE80211_NODE_AUTH;
}
EXPORT_SYMBOL_C(ieee80211_node_unauthorize);

/*
 * Set/change the channel.  The rate set is also updated
 * to insure a consistent view by drivers.
 */
static __inline void
ieee80211_node_set_chan(struct ieee80211com *ic, struct ieee80211_node *ni)
{
    struct ieee80211_channel *chan = ic->ic_bsschan;

    KASSERT(chan != IEEE80211_CHAN_ANYC, ("bss channel not setup"));
    ni->ni_chan = chan;
#ifdef ATH_SUPERG_XR 
    if(ni->ni_vap->iv_flags & IEEE80211_F_XR)
    {
        ni->ni_rates = ic->ic_sup_xr_rates;
    }
    else
#endif
        ni->ni_rates = ic->ic_sup_rates[ieee80211_chan2mode(chan)];
    ni->ni_htrates = ic->ic_sup_ht_rates[ieee80211_chan2mode(chan)];
}

static __inline void
copy_bss(struct ieee80211_node *nbss, const struct ieee80211_node *obss)
{
    /* propagate useful state */
    nbss->ni_authmode = obss->ni_authmode;
    nbss->ni_ath_flags = obss->ni_ath_flags;
    nbss->ni_txpower = obss->ni_txpower;
    nbss->ni_vlan = obss->ni_vlan;
    nbss->ni_rsn = obss->ni_rsn;
    /* XXX statistics? */
}

void
ieee80211_create_ibss(struct ieee80211vap* vap, struct ieee80211_channel *chan)
{
    struct ieee80211com *ic = vap->iv_ic;
    struct ieee80211_node *ni;

    IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN,
        "%s: creating ibss on channel %u\n", __func__,
        ieee80211_chan2ieee(ic, chan));

    ni = ieee80211_alloc_node(&ic->ic_sta, vap, vap->iv_myaddr);
    if (ni == NULL)
    {
        /* XXX recovery? */
        return;
    }
    IEEE80211_ADDR_COPY(ni->ni_bssid, vap->iv_myaddr);
    ni->ni_esslen = vap->iv_des_ssid[0].len;
    memcpy(ni->ni_essid, vap->iv_des_ssid[0].ssid, ni->ni_esslen);
    if (vap->iv_bss != NULL)
        copy_bss(ni, vap->iv_bss);
    ni->ni_intval = ic->ic_lintval;
#ifdef ATH_SUPERG_XR 
    if(vap->iv_flags & IEEE80211_F_XR)
    {
        ni->ni_intval *= IEEE80211_XR_BEACON_FACTOR;
    }
#endif
    if (vap->iv_flags & IEEE80211_F_PRIVACY)
        ni->ni_capinfo |= IEEE80211_CAPINFO_PRIVACY;
    if (ic->ic_phytype == IEEE80211_T_FH)
    {
        ni->ni_fhdwell = 200;	/* XXX */
        ni->ni_fhindex = 1;
    }
    if (vap->iv_opmode == IEEE80211_M_IBSS)
    {
        vap->iv_flags |= IEEE80211_F_SIBSS;
        ni->ni_capinfo |= IEEE80211_CAPINFO_IBSS;	/* XXX */
        if (vap->iv_flags & IEEE80211_F_DESBSSID)
            IEEE80211_ADDR_COPY(ni->ni_bssid, vap->iv_des_bssid);
        else
	      /* Create a random bssid for Adhoc network as done in NDIS */
                ieee80211_create_adhocbssid(vap, ni->ni_bssid);

    }
#ifdef ATH_SUPERG_DYNTURBO
    if (vap->iv_opmode == IEEE80211_M_HOSTAP)
    {
        ni->ni_ath_flags = vap->iv_ath_cap;
        /*
        * no dynamic turbo and AR on a static turbo channel.
        * no dynamic turbo and AR on non-turbo channel.
        * no AR on 5GHZ channel .
        */
        if (IEEE80211_IS_CHAN_STURBO(chan) ||
            !ieee80211_find_channel(ic, chan->ic_freq, chan->ic_flags | IEEE80211_CHAN_TURBO))
        {
            ni->ni_ath_flags &= ~(IEEE80211_ATHC_TURBOP | IEEE80211_ATHC_AR);
        }
        if (IEEE80211_IS_CHAN_5GHZ(chan)) 
            ni->ni_ath_flags &= ~IEEE80211_ATHC_AR;
    }
#endif
    /* 
     * Fix the channel and related attributes.
     */
    ic->ic_bsschan = chan;
    ieee80211_node_set_chan(ic, ni);
    ic->ic_curmode = ieee80211_chan2mode(chan);
    if (ic->ic_curmode == IEEE80211_MODE_11NG_HT40PLUS ||
        ic->ic_curmode == IEEE80211_MODE_11NA_HT40PLUS) 
    {
        ic->ic_cwm.cw_extoffset = 1;
    } else if (ic->ic_curmode == IEEE80211_MODE_11NG_HT40MINUS ||
               ic->ic_curmode == IEEE80211_MODE_11NA_HT40MINUS) {
        ic->ic_cwm.cw_extoffset = -1;
    }
    ni->ni_extoffset = ic->ic_cwm.cw_extoffset;


    /* Update country ie information */
    ieee80211_build_countryie(ic);

    if (IEEE80211_IS_CHAN_HALF(chan))
    {
        ni->ni_rates = ic->ic_sup_half_rates;
    }
    else if (IEEE80211_IS_CHAN_QUARTER(chan))
    {
        ni->ni_rates = ic->ic_sup_quarter_rates;
    }

    if ((vap->iv_flags & IEEE80211_F_PUREG) &&
        (IEEE80211_IS_CHAN_ANYG(chan) || IEEE80211_IS_CHAN_11NG(chan)))
    {
        ieee80211_setpuregbasicrates(&ni->ni_rates);
    }

#ifdef NODE_FREE_DEBUG
    (void) ieee80211_sta_join1(ieee80211_ref_node(ni, __func__));
#else
    (void) ieee80211_sta_join1(ieee80211_ref_node(ni));
#endif
}
EXPORT_SYMBOL_C(ieee80211_create_ibss);

/*
 * Reset bss state on transition to the INIT state.
 * Clear any stations from the table (they have been
 * deauth'd) and reset the bss node (clears key, rate,
 * etc. state).
 */
void
ieee80211_reset_bss(struct ieee80211vap *vap)
{
    struct ieee80211com *ic = vap->iv_ic;
    struct ieee80211_node *ni, *obss;

    IEEE80211_DPRINTF(vap, IEEE80211_MSG_NODE, "%s: old bss %p<%s>\n",
        __func__, vap->iv_bss, ether_sprintf(vap->iv_bss->ni_macaddr));

    ieee80211_node_table_reset(&ic->ic_sta, vap);
    /* XXX multi-bss wrong */
    ieee80211_reset_erp(ic, ic->ic_curmode);
    ieee80211_reset_ht(ic);

    ni = ieee80211_alloc_node(&ic->ic_sta, vap, vap->iv_myaddr);
    KASSERT(ni != NULL, ("unable to setup inital BSS node"));
    obss = vap->iv_bss;
#ifdef NODE_FREE_DEBUG
    vap->iv_bss = ieee80211_ref_node(ni, __func__);
#else
    vap->iv_bss = ieee80211_ref_node(ni);
#endif
    if (obss != NULL)
    {
        copy_bss(ni, obss);
        ni->ni_intval = ic->ic_lintval;
        ieee80211_free_node(obss);
    }
}

static int
match_ssid(const struct ieee80211_node *ni,
    int nssid, const struct ieee80211_scan_ssid ssids[])
{
    int i;

    for (i = 0; i < nssid; i++)
    {
        if (ni->ni_esslen == ssids[i].len &&
            memcmp(ni->ni_essid, ssids[i].ssid, ni->ni_esslen) == 0)
            return 1;
    }
    return 0;
}

/*
 * Test a node for suitability/compatibility.
 */
static int
check_bss(struct ieee80211vap *vap, struct ieee80211_node *ni)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ieee80211_rateset *srs;

    if (isclr(ic->ic_chan_active, ieee80211_chan2ieee(ic, ni->ni_chan)))
        return 0;
    if (vap->iv_opmode == IEEE80211_M_IBSS)
    {
        if ((ni->ni_capinfo & IEEE80211_CAPINFO_IBSS) == 0)
            return 0;
    }
    else
    {
        if ((ni->ni_capinfo & IEEE80211_CAPINFO_ESS) == 0)
            return 0;
    }
    if (vap->iv_flags & IEEE80211_F_PRIVACY)
    {
        if ((ni->ni_capinfo & IEEE80211_CAPINFO_PRIVACY) == 0)
            return 0;
    }
    else
    {
        /* XXX does this mean privacy is supported or required? */
        if (ni->ni_capinfo & IEEE80211_CAPINFO_PRIVACY)
            return 0;
    }

    srs =  &ic->ic_sup_rates[ieee80211_chan2mode(ni->ni_chan)];
    if (!ieee80211_brs_rate_check(srs, &ni->ni_rates))
        return 0;
    if (!ieee80211_fixed_rate_check(ni, &ni->ni_rates))
        return 0;
    if (vap->iv_des_nssid != 0 &&
        !match_ssid(ni, vap->iv_des_nssid, vap->iv_des_ssid))
        return 0;
    if ((vap->iv_flags & IEEE80211_F_DESBSSID) &&
        !IEEE80211_ADDR_EQ(vap->iv_des_bssid, ni->ni_bssid))
        return 0;
    return 1;
}

#ifdef IEEE80211_DEBUG
/*
 * Display node suitability/compatibility.
 */
static void
check_bss_debug(struct ieee80211vap *vap, struct ieee80211_node *ni)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ieee80211_rateset *srs;
    int fail;

    fail = 0;
    if (isclr(ic->ic_chan_active, ieee80211_chan2ieee(ic, ni->ni_chan)))
        fail |= 0x01;
    if (vap->iv_opmode == IEEE80211_M_IBSS)
    {
        if ((ni->ni_capinfo & IEEE80211_CAPINFO_IBSS) == 0)
            fail |= 0x02;
    }
    else
    {
        if ((ni->ni_capinfo & IEEE80211_CAPINFO_ESS) == 0)
            fail |= 0x02;
    }
    if (vap->iv_flags & IEEE80211_F_PRIVACY)
    {
        if ((ni->ni_capinfo & IEEE80211_CAPINFO_PRIVACY) == 0)
            fail |= 0x04;
    }
    else
    {
        /* XXX does this mean privacy is supported or required? */
        if (ni->ni_capinfo & IEEE80211_CAPINFO_PRIVACY)
            fail |= 0x04;
    }
    srs =  &ic->ic_sup_rates[ieee80211_chan2mode(ni->ni_chan)];
    if (!ieee80211_brs_rate_check(srs, &ni->ni_rates))
        fail |= 0x08;
    if (!ieee80211_fixed_rate_check(ni, &ni->ni_rates))
        fail |= 0x08;
    if (vap->iv_des_nssid != 0 &&
        !match_ssid(ni, vap->iv_des_nssid, vap->iv_des_ssid))
        fail |= 0x10;
    if ((vap->iv_flags & IEEE80211_F_DESBSSID) &&
        !IEEE80211_ADDR_EQ(vap->iv_des_bssid, ni->ni_bssid))
        fail |= 0x20;

    printf(" %c %s", fail ? '-' : '+', ether_sprintf(ni->ni_macaddr));
    printf(" %s%c", ether_sprintf(ni->ni_bssid), fail & 0x20 ? '!' : ' ');
    printf(" %3d%c",
        ieee80211_chan2ieee(ic, ni->ni_chan), fail & 0x01 ? '!' : ' ');
    printf(" %+4d", ni->ni_rssi);
    printf(" M%c", fail & 0x08 ? '!' : ' ');
    printf(" %4s%c",
        (ni->ni_capinfo & IEEE80211_CAPINFO_ESS) ? "ess" :
        (ni->ni_capinfo & IEEE80211_CAPINFO_IBSS) ? "ibss" :
        "????",
        fail & 0x02 ? '!' : ' ');
    printf(" %3s%c ",
        (ni->ni_capinfo & IEEE80211_CAPINFO_PRIVACY) ?  "wep" : "no",
        fail & 0x04 ? '!' : ' ');
    ieee80211_print_essid(ni->ni_essid, ni->ni_esslen);
    printf("%s\n", fail & 0x10 ? "!" : "");
}
#endif /* IEEE80211_DEBUG */

/*
 * Handle 802.11 ad hoc network merge.  The
 * convention, set by the Wireless Ethernet Compatibility Alliance
 * (WECA), is that an 802.11 station will change its BSSID to match
 * the "oldest" 802.11 ad hoc network, on the same channel, that
 * has the station's desired SSID.  The "oldest" 802.11 network
 * sends beacons with the greatest TSF timestamp.
 *
 * The caller is assumed to validate TSF's before attempting a merge.
 *
 * Return !0 if the BSSID changed, 0 otherwise.
 */
int
ieee80211_ibss_merge(struct ieee80211_node *ni)
{
    struct ieee80211vap *vap = ni->ni_vap;
    struct ieee80211com *ic = ni->ni_ic;

    if (ni == vap->iv_bss ||
        IEEE80211_ADDR_EQ(ni->ni_bssid, vap->iv_bss->ni_bssid))
    {
        /* unchanged, nothing to do */
        return 0;
    }
    if (!check_bss(vap, ni))
    {
        /* capabilities mismatch */
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_ASSOC,
            "%s: merge failed, capabilities mismatch\n", __func__);
#ifdef IEEE80211_DEBUG
        if (ieee80211_msg_assoc(vap))
            check_bss_debug(vap, ni);
#endif
        vap->iv_stats.is_ibss_capmismatch++;
        return 0;
    }
    IEEE80211_DPRINTF(vap, IEEE80211_MSG_ASSOC,
        "%s: new bssid %s: %s preamble, %s slot time%s\n", __func__,
        ether_sprintf(ni->ni_bssid),
        ic->ic_flags&IEEE80211_F_SHPREAMBLE ? "short" : "long",
        ic->ic_flags&IEEE80211_F_SHSLOT ? "short" : "long",
        ic->ic_flags&IEEE80211_F_USEPROT ? ", protection" : ""
        );
#ifdef NODE_FREE_DEBUG
    return ieee80211_sta_join1(ieee80211_ref_node(ni, __func__));
#else
    return ieee80211_sta_join1(ieee80211_ref_node(ni));
#endif
}
EXPORT_SYMBOL_C(ieee80211_ibss_merge);

static __inline int
ssid_equal(const struct ieee80211_node *a, const struct ieee80211_node *b)
{
    return (a->ni_esslen == b->ni_esslen &&
        memcmp(a->ni_essid, b->ni_bssid, a->ni_esslen) == 0);
}

/*
 * Join the specified IBSS/BSS network.  The node is assumed to
 * be passed in with a reference already held for use in assigning
 * to iv_bss.
 */
static int
ieee80211_sta_join1(struct ieee80211_node *selbs)
{
    struct ieee80211vap *vap = selbs->ni_vap;
    struct ieee80211com *ic = selbs->ni_ic;
    struct ieee80211_node *obss;
    int canreassoc;

    /*
     * Committed to selbs, setup state.
     */
    obss = vap->iv_bss;
    /*
     * Check if old+new node have the same ssid in which
     * case we can reassociate when operating in sta mode.
     */
    canreassoc = (obss != NULL &&
        vap->iv_state == IEEE80211_S_RUN && ssid_equal(obss, selbs));
    vap->iv_bss = selbs;
    if (obss != NULL)
    {
        ieee80211_node_removeall_wds(&ic->ic_sta,obss);
        ieee80211_free_node(obss);
    }

    /* Setup 11n state */
    /* adopt to selbs chwidth and extoffset, only if sta is in
     * dyn2040 mode */
    if (ic->ic_cwm.cw_mode == IEEE80211_CWM_MODE2040) {
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_ASSOC,
            "%s: adopting to bss cw_width %d and cw_extoffset\n",
             __func__, selbs->ni_chwidth, selbs->ni_extoffset);
        ic->ic_cwm.cw_width = selbs->ni_chwidth;
        ic->ic_cwm.cw_extoffset = selbs->ni_extoffset;
    }
    else {
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_ASSOC, 
            "%s: operating in static mode - cw_width and cw_extoffset not changed\n",
            __func__);
    }


    ic->ic_bsschan = selbs->ni_chan;
    ic->ic_curchan = ic->ic_bsschan;
    ic->ic_curmode = ieee80211_chan2mode(ic->ic_curchan);
    ieee80211_new_state(vap, IEEE80211_S_JOIN, 0);
    ic->ic_set_channel(ic);
    if (selbs->ni_tpc_ie != NULL)
        ieee80211_process_tpcie(selbs);
    /*
     * Set the erp state (mostly the slot time) to deal with
     * the auto-select case; this should be redundant if the
     * mode is locked.
     */
    ieee80211_reset_erp(ic, ic->ic_curmode);
    ieee80211_reset_ht(ic);
    ieee80211_wme_initparams(vap);

    if (vap->iv_opmode == IEEE80211_M_STA)
    {
        /*
        * Act as if we received a DEAUTH frame in case we are
        * invoked from the RUN state.  This will cause us to try
        * to re-authenticate if we are operating as a station.
        */
        if (canreassoc)
            ieee80211_new_state(vap, IEEE80211_S_ASSOC, 0);
        else
            ieee80211_new_state(vap, IEEE80211_S_AUTH,
                IEEE80211_FC0_SUBTYPE_DEAUTH);
    }
    else
        ieee80211_new_state(vap, IEEE80211_S_RUN, -1);
    return 1;
}

int
ieee80211_sta_join(struct ieee80211vap *vap,
    const struct ieee80211_scan_entry *se)
{
    struct ieee80211com *ic = vap->iv_ic;
    struct ieee80211_node *ni;

    ni = ieee80211_alloc_node(&ic->ic_sta, vap, se->se_macaddr);
    if (ni == NULL)
    {
        /* XXX msg */
        return 0;
    }

    /*
     * Expand scan state into node's format.
     * XXX may not need all this stuff
     */
    ni->ni_authmode = vap->iv_bss->ni_authmode;		/* inherit authmode from iv_bss */
    IEEE80211_ADDR_COPY(ni->ni_bssid, se->se_bssid);
    ni->ni_esslen = se->se_ssid[1];
    memcpy(ni->ni_essid, se->se_ssid+2, ni->ni_esslen);
    ni->ni_rstamp = se->se_rstamp;
    ni->ni_tstamp.tsf = se->se_tstamp.tsf;
    ni->ni_intval = se->se_intval;
    ni->ni_capinfo = se->se_capinfo;
    ni->ni_chan = se->se_chan;
    ni->ni_timoff = se->se_timoff;
    ni->ni_fhdwell = se->se_fhdwell;
    ni->ni_fhindex = se->se_fhindex;
    ni->ni_erp = se->se_erp;
    ni->ni_rssi = se->se_rssi;
    if (se->se_wpa_ie != NULL)
        ieee80211_saveie(&ni->ni_wpa_ie, se->se_wpa_ie);
    if (se->se_wme_ie != NULL)
        ieee80211_saveie(&ni->ni_wme_ie, se->se_wme_ie);
    if ((se->se_htcap_ie != NULL) && ieee80211vap_is_htallowed(vap))
        ieee80211_parse_htcap(ni, se->se_htcap_ie);
    if ((se->se_htinfo_ie != NULL) && ieee80211vap_is_htallowed(vap))
        ieee80211_parse_htinfo(ni, se->se_htinfo_ie);
    if (se->se_ath_ie != NULL)
        ieee80211_saveath(ni, se->se_ath_ie);
#ifdef ATH_WPS_IE
	if (se->se_rsn_ie != NULL)
		ieee80211_saveie(&ni->ni_rsn_ie, se->se_rsn_ie);
	if (se->se_wps_ie != NULL)
		ieee80211_saveie(&ni->ni_wps_ie, se->se_wps_ie);
#endif /* ATH_WPS_IE */
    if (se->se_tpc_ie != NULL)
        ieee80211_saveie(&ni->ni_tpc_ie, se->se_tpc_ie);
    ic->ic_11dinfo = (struct ieee80211_country_ie *)se->se_country_ie;
    if (ic->ic_11dinfo != NULL) {
        ni->ni_cc[0] = ic->ic_11dinfo->cc[0];
        ni->ni_cc[1] = ic->ic_11dinfo->cc[1];
        ni->ni_cc[2] = ic->ic_11dinfo->cc[2];
    } else {
        ni->ni_cc[0] = 0;
        ni->ni_cc[1] = 0;
        ni->ni_cc[2] = 0;
    }
    vap->iv_dtim_period = se->se_dtimperiod;
    vap->iv_dtim_count = 0;

    /* NB: must be after ni_chan is setup */
    ieee80211_setup_rates(ni, se->se_rates, se->se_xrates,
        IEEE80211_F_DOSORT | IEEE80211_F_DOXSECT);
    if (se->se_htcap_ie != NULL) 
        ieee80211_setup_ht_rates(ni, se->se_htcap_ie,IEEE80211_F_DOXSECT);
    if (se->se_htinfo_ie != NULL)
        ieee80211_setup_basic_ht_rates(ni, se->se_htinfo_ie);

    /*
     * Validate and set country code which is received
     * in 11D Beacon
     */
    ieee80211_11D_handler(vap, ni);
#ifdef NODE_FREE_DEBUG
    return ieee80211_sta_join1(ieee80211_ref_node(ni, __func__));
#else
    return ieee80211_sta_join1(ieee80211_ref_node(ni));
#endif
}
EXPORT_SYMBOL_C(ieee80211_sta_join);

/*
 * Leave the specified IBSS/BSS network.  The node is assumed to
 * be passed in with a held reference.
 */
void
ieee80211_sta_leave(struct ieee80211_node *ni, int reassoc)
{
    struct ieee80211vap *vap = ni->ni_vap;
    struct ieee80211com *ic = vap->iv_ic;

    /*
     * Set the country code setting to default if 11d has been in use
     */
    if (ieee80211com_has_extflag(ni->ni_ic, IEEE80211_FEXT_DOT11D))
        ieee80211com_set_country_code(ni->ni_ic, NULL);

    /* WDS/Repeater: Stop software beacon timer for STA */
    if (vap->iv_opmode == IEEE80211_M_STA &&
        vap->iv_flags_ext & IEEE80211_FEXT_SWBMISS)
    {
        del_timer(&vap->iv_swbmiss);
    }

    ic->ic_node_cleanup(ni);
    ieee80211_notify_node_leave(ni, reassoc);
}

/*
 * Node table support.
 */

static void
ieee80211_node_table_init(struct ieee80211com *ic,
    struct ieee80211_node_table *nt,
    const char *name, int inact)
{
    nt->nt_ic = ic;
    IEEE80211_NODE_LOCK_INIT(nt, ic->ic_dev->name);
    IEEE80211_SCAN_LOCK_INIT(nt, ic->ic_dev->name);
    TAILQ_INIT(&nt->nt_node);
    nt->nt_name = name;
    nt->nt_scangen = 1;
    nt->nt_inact_init = inact;
    init_timer(&nt->nt_wds_aging_timer);
    nt->nt_wds_aging_timer.function = ieee80211_node_wds_ageout;
    nt->nt_wds_aging_timer.data = (unsigned long) nt;
    mod_timer(&nt->nt_wds_aging_timer,jiffies+HZ*WDS_AGING_TIMER_VAL);
}

static struct ieee80211_node *
    node_alloc(struct ieee80211_node_table *nt,struct ieee80211vap *vap)
{
    struct ieee80211_node *ni;

    MALLOC(ni, struct ieee80211_node *, sizeof(struct ieee80211_node),
        M_80211_NODE, M_NOWAIT | M_ZERO);
    return ni;
}

/*
 * Reclaim any resources in a node and reset any critical
 * state.  Typically nodes are free'd immediately after,
 * but in some cases the storage may be reused so we need
 * to insure consistent state (should probably fix that).
 */
static void
node_cleanup(struct ieee80211_node *ni)
{
#define	N(a)	(sizeof(a)/sizeof(a[0]))
    struct ieee80211vap *vap = ni->ni_vap;
    int i;

    /* NB: preserve ni_table */
    if (ni->ni_flags & IEEE80211_NODE_PWR_MGT)
    {
        if (vap->iv_opmode != IEEE80211_M_STA)
            vap->iv_ps_sta--;
        ni->ni_flags &= ~IEEE80211_NODE_PWR_MGT;
        IEEE80211_NOTE(vap, IEEE80211_MSG_POWER, ni,
            "power save mode off, %u sta's in ps mode", vap->iv_ps_sta);

        if (ni->ni_flags & IEEE80211_NODE_UAPSD_TRIG)
        {
            ni->ni_flags &= ~IEEE80211_NODE_UAPSD_TRIG;
            IEEE80211_UAPSD_LOCK(ni->ni_ic);
            ni->ni_ic->ic_uapsdmaxtriggers--;
            IEEE80211_UAPSD_UNLOCK(ni->ni_ic);
        }
    }
    /*
     * Clear AREF flag that marks the authorization refcnt bump
     * has happened.  This is probably not needed as the node
     * should always be removed from the table so not found but
     * do it just in case.
     */
    ni->ni_flags &= ~IEEE80211_NODE_AREF;

    /*
    * Drain power save queue and, if needed, clear TIM.
    */
    if (ieee80211_node_saveq_drain(ni) != 0 && vap->iv_set_tim != NULL)
        vap->iv_set_tim(ni, 0);

    ni->ni_associd = 0;
    if (ni->ni_challenge != NULL)
    {
        FREE(ni->ni_challenge, M_DEVBUF);
        ni->ni_challenge = NULL;
    }
    /*
    * Preserve SSID, WPA, and WME ie's so the bss node is
    * reusable during a re-auth/re-assoc state transition.
    * If we remove these data they will not be recreated
    * because they come from a probe-response or beacon frame
    * which cannot be expected prior to the association-response.
    * This should not be an issue when operating in other modes
    * as stations leaving always go through a full state transition
    * which will rebuild this state.
    *
    * XXX does this leave us open to inheriting old state?
    */
    for (i = 0; i < N(ni->ni_rxfrag); i++)
        if (ni->ni_rxfrag[i] != NULL)
        {
            kfree_skb(ni->ni_rxfrag[i]);
            ni->ni_rxfrag[i] = NULL;
        }
        ieee80211_crypto_delkey(vap, &ni->ni_ucastkey, ni);
    ni->ni_rxkeyoff = 0;
#undef N
}

static void
node_free(struct ieee80211_node *ni)
{
    if (ni->ni_wpa_ie != NULL)
        FREE(ni->ni_wpa_ie, M_DEVBUF);


    if (ni->ni_wme_ie != NULL)
        FREE(ni->ni_wme_ie, M_DEVBUF);
    if (ni->ni_ath_ie != NULL)
        FREE(ni->ni_ath_ie, M_DEVBUF);
#ifdef ATH_WPS_IE
	if (ni->ni_rsn_ie != NULL)
		FREE(ni->ni_rsn_ie, M_DEVBUF);
	if (ni->ni_wps_ie != NULL)
		FREE(ni->ni_wps_ie, M_DEVBUF);
#endif /* ATH_WPS_IE */
    if (ni->ni_tpc_ie != NULL)
        FREE(ni->ni_tpc_ie, M_DEVBUF);
    IEEE80211_NODE_SAVEQ_DESTROY(ni);
    FREE(ni, M_80211_NODE);
}

static int8_t
node_getrssi(const struct ieee80211_node *ni,  int8_t chain, u_int8_t flags)
{
    return ni->ni_rssi;
}

/*
 * Create an entry in the specified node table.  The node
 * is setup with the mac address, an initial reference count,
 * and some basic parameters obtained from global state.
 * This interface is not intended for general use, it is
 * used by the routines below to create entries with a
 * specific purpose.
 */
struct ieee80211_node *
    ieee80211_alloc_node(struct ieee80211_node_table *nt,
        struct ieee80211vap *vap, const u_int8_t *macaddr)
{
    struct ieee80211com *ic = nt->nt_ic;
    struct ieee80211_node *ni;
    int hash;
    TAILQ_FOREACH(ni, &nt->nt_node, ni_list)
    if (ni != NULL) {
        if (IEEE80211_ADDR_EQ(macaddr, ni->ni_bssid))
	    return ni;
    }

    ni = ic->ic_node_alloc(nt,vap);
    if (ni == NULL)
    {
        /* XXX msg */
        vap->iv_stats.is_rx_nodealloc++;
        return NULL;
    }

#ifdef NODE_FREE_DEBUG
    MALLOC(ni->node_trace, ieee80211_trace_t *,
           sizeof(ieee80211_trace_t)*MAX_TRACE_SIZE,
           0, M_NOWAIT | M_ZERO);
    ni->nix=0;
    spin_lock_init(&ni->buff_lock);
#endif
    IEEE80211_DPRINTF(vap, IEEE80211_MSG_NODE,
        "%s %p<%s> in %s table\n", __func__, ni,
        ether_sprintf(macaddr), nt->nt_name);

    IEEE80211_ADDR_COPY(ni->ni_macaddr, macaddr);
    hash = IEEE80211_NODE_HASH(macaddr);
    ieee80211_node_initref(ni);		/* mark referenced */
    ni->ni_chan = IEEE80211_CHAN_ANYC;
    ni->ni_authmode = IEEE80211_AUTH_OPEN;
    ni->ni_txpower = ic->ic_txpowlimit;	/* max power */
    ieee80211_crypto_resetkey(vap, &ni->ni_ucastkey, IEEE80211_KEYIX_NONE);
    ni->ni_inact_reload = nt->nt_inact_init;
    ni->ni_inact = ni->ni_inact_reload;
    ni->ni_ath_defkeyindex = IEEE80211_INVAL_DEFKEY;
    ni->ni_rxkeyoff = 0;
    IEEE80211_NODE_SAVEQ_INIT(ni, "unknown");
    /* 11n */
    ni->ni_chwidth = ic->ic_cwm.cw_width;
    ni->ni_newchwidth = 0;
    ni->ni_updaterates = 0;
    ni->ni_extoffset = ic->ic_cwm.cw_extoffset;

    IEEE80211_NODE_LOCK_BH(nt);
    TAILQ_INSERT_TAIL(&nt->nt_node, ni, ni_list);
    LIST_INSERT_HEAD(&nt->nt_hash[hash], ni, ni_hash);
    ni->ni_table = nt;
    ni->ni_vap = vap;
    ni->ni_ic = ic;
    IEEE80211_NODE_UNLOCK_BH(nt);

    WME_UAPSD_NODE_TRIGSEQINIT(ni);

    return ni;
}
EXPORT_SYMBOL_C(ieee80211_alloc_node);

/* Add wds address to the node table */
int
ieee80211_add_wds_addr(struct ieee80211_node_table *nt,
    struct ieee80211_node *ni, const u_int8_t *macaddr)
{
    int hash;
    struct ieee80211_wds_addr *wds;

    MALLOC(wds, struct ieee80211_wds_addr *, sizeof(struct ieee80211_wds_addr),
        M_80211_WDS, M_NOWAIT | M_ZERO);
    if (wds == NULL)
    {
        /* XXX msg */
        return 1;
    }
    wds->wds_agingcount = WDS_AGING_COUNT;
    hash = IEEE80211_NODE_HASH(macaddr);
    IEEE80211_ADDR_COPY(wds->wds_macaddr, macaddr);
#ifdef NODE_FREE_DEBUG
    ieee80211_ref_node(ni, __func__);		/* Reference node */
#else
    ieee80211_ref_node(ni);		/* Reference node */
#endif
    wds->wds_ni = ni;
    IEEE80211_NODE_LOCK_BH(nt);
    LIST_INSERT_HEAD(&nt->nt_wds_hash[hash], wds, wds_hash);
    IEEE80211_NODE_UNLOCK_BH(nt);
    return 0;
}
EXPORT_SYMBOL_C(ieee80211_add_wds_addr);

/* remove wds address from the wds hash table */
void
ieee80211_remove_wds_addr(struct ieee80211_node_table *nt, 
    const u_int8_t *macaddr)
{
    int hash;
    struct ieee80211_wds_addr *wds;

    hash = IEEE80211_NODE_HASH(macaddr);
    LIST_FOREACH(wds, &nt->nt_wds_hash[hash], wds_hash) {
        if (IEEE80211_ADDR_EQ(wds->wds_macaddr, macaddr))
        {
            ieee80211_free_node(wds->wds_ni);  /* Decrement ref count */
            LIST_REMOVE(wds, wds_hash);
            FREE(wds, M_80211_WDS);
            break;
        }
    }
}
EXPORT_SYMBOL_C(ieee80211_remove_wds_addr);


/* Remove node references from wds table */
void
ieee80211_del_wds_node(struct ieee80211_node_table *nt,
    struct ieee80211_node *ni)
{
    int hash;
    struct ieee80211_wds_addr *wds;

    for (hash=0; hash<IEEE80211_NODE_HASHSIZE; hash++)
    {
        LIST_FOREACH(wds, &nt->nt_wds_hash[hash], wds_hash) {
            if (wds->wds_ni == ni)
            {
                ieee80211_free_node(ni);  /* Decrement ref count */
                LIST_REMOVE(wds, wds_hash);
                FREE(wds, M_80211_WDS);
            }
        }
    }
}
EXPORT_SYMBOL_C(ieee80211_del_wds_node);

static void
ieee80211_node_wds_ageout(unsigned long data)
{
    struct ieee80211_node_table *nt= (struct ieee80211_node_table *)data;
    int hash;
    struct ieee80211_wds_addr *wds;

    IEEE80211_NODE_LOCK_BH(nt);
    for (hash=0; hash<IEEE80211_NODE_HASHSIZE; hash++)
    {
        LIST_FOREACH(wds, &nt->nt_wds_hash[hash], wds_hash) {
            if (!wds->wds_agingcount)
            {
                ieee80211_free_node(wds->wds_ni);  /* Decrement ref count */
                LIST_REMOVE(wds, wds_hash);
                FREE(wds, M_80211_WDS);
            }
            else
            {
                wds->wds_agingcount--;
            }
        }
    }
    IEEE80211_NODE_UNLOCK_BH(nt);
    mod_timer(&nt->nt_wds_aging_timer,jiffies+HZ*WDS_AGING_TIMER_VAL);
}


/*
 * Craft a temporary node suitable for sending a management frame
 * to the specified station.  We craft only as much state as we
 * need to do the work since the node will be immediately reclaimed
 * once the send completes.
 */
struct ieee80211_node *
    ieee80211_tmp_node(struct ieee80211vap *vap, const u_int8_t *macaddr)
{
    struct ieee80211com *ic = vap->iv_ic;
    struct ieee80211_node *ni;

    /*
    * No node table since this is a tmp node.
    */
    ni = ic->ic_node_alloc(NULL ,vap);
    if (ni != NULL)
    {
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_NODE,
            "%s %p<%s>\n", __func__, ni, ether_sprintf(macaddr));

        ni->ni_tmp_node = 1;
        IEEE80211_ADDR_COPY(ni->ni_macaddr, macaddr);
        IEEE80211_ADDR_COPY(ni->ni_bssid, vap->iv_bss->ni_bssid);
        ieee80211_node_initref(ni);		/* mark referenced */
        ni->ni_txpower = vap->iv_bss->ni_txpower;
        ni->ni_vap = vap;
        /* NB: required by ieee80211_fix_rate */
        ieee80211_node_set_chan(ic, ni);
        ieee80211_crypto_resetkey(vap, &ni->ni_ucastkey,
            IEEE80211_KEYIX_NONE);
        /* XXX optimize away */
        IEEE80211_NODE_SAVEQ_INIT(ni, "unknown");

        ni->ni_table = NULL;		/* NB: pedantic */
        ni->ni_ic = ic;
    }
    else
    {
        /* XXX msg */
        vap->iv_stats.is_rx_nodealloc++;
    }
    return ni;
}

/*
 * Add the specified station to the station table.
 */
struct ieee80211_node *
    ieee80211_dup_bss(struct ieee80211vap *vap, const u_int8_t *macaddr)
{
    struct ieee80211com *ic = vap->iv_ic;
    struct ieee80211_node *ni;

    ni = ieee80211_alloc_node(&ic->ic_sta, vap, macaddr);
    if (ni != NULL)
    {
        /*
         * Inherit from iv_bss.
         */
        ni->ni_authmode = vap->iv_bss->ni_authmode;
        ni->ni_txpower = vap->iv_bss->ni_txpower;
        ni->ni_vlan = vap->iv_bss->ni_vlan;	/* XXX?? */
        IEEE80211_ADDR_COPY(ni->ni_bssid, vap->iv_bss->ni_bssid);
        ieee80211_node_set_chan(ic, ni);
        ni->ni_rsn = vap->iv_bss->ni_rsn;
    }
    return ni;
}
EXPORT_SYMBOL_C(ieee80211_dup_bss);

static struct ieee80211_node *
    _ieee80211_find_wds_node(struct ieee80211_node_table *nt,
        const u_int8_t *macaddr)
{
    struct ieee80211_node *ni;
    struct ieee80211_wds_addr *wds;
    int hash;
    IEEE80211_NODE_LOCK_ASSERT(nt);

    hash = IEEE80211_NODE_HASH(macaddr);
    LIST_FOREACH(wds, &nt->nt_wds_hash[hash], wds_hash) {
        if (IEEE80211_ADDR_EQ(wds->wds_macaddr, macaddr))
        {
            ni = wds->wds_ni;
            wds->wds_agingcount = WDS_AGING_COUNT; /* reset the aging count */
#ifdef NODE_FREE_DEBUG
            ieee80211_ref_node(ni, __func__);
#else
            ieee80211_ref_node(ni);
#endif
            return ni;
        }
    }
    return NULL;
}

static struct ieee80211_node *
#ifdef IEEE80211_DEBUG_REFCNT
    _ieee80211_find_node_debug(struct ieee80211_node_table *nt,
        const u_int8_t *macaddr, const char *func, int line)
#else
    _ieee80211_find_node(struct ieee80211_node_table *nt,
        const u_int8_t *macaddr)
#endif
    {
        struct ieee80211_node *ni;
        int hash;
        struct ieee80211_wds_addr *wds;

        IEEE80211_NODE_LOCK_ASSERT(nt);

        hash = IEEE80211_NODE_HASH(macaddr);
        LIST_FOREACH(ni, &nt->nt_hash[hash], ni_hash) {
            if (IEEE80211_ADDR_EQ(ni->ni_macaddr, macaddr))
            {
#ifdef NODE_FREE_DEBUG
                ieee80211_ref_node(ni, __func__);	/* mark referenced */
#else
                ieee80211_ref_node(ni);	/* mark referenced */
#endif

#ifdef IEEE80211_DEBUG_REFCNT
                IEEE80211_DPRINTF(ni->ni_vap, IEEE80211_MSG_NODE,
                    "%s (%s:%u) %p<%s> refcnt %d\n", __func__,
                    func, line,
                    ni, ether_sprintf(ni->ni_macaddr),
                    ieee80211_node_refcnt(ni));
#endif
                return ni;
            }
        }

        /* Now, we look for the desired mac address in the 4 address
           nodes. */
        LIST_FOREACH(wds, &nt->nt_wds_hash[hash], wds_hash) {
            if (IEEE80211_ADDR_EQ(wds->wds_macaddr, macaddr))
            {
                ni = wds->wds_ni;
#ifdef NODE_FREE_DEBUG
                ieee80211_ref_node(ni, __func__);
#else
                ieee80211_ref_node(ni);
#endif
                return ni;
            }
        }
        return NULL;
    }
#ifdef IEEE80211_DEBUG_REFCNT
#define	_ieee80211_find_node(nt, mac) \
	_ieee80211_find_node_debug(nt, mac, func, line)
#endif

/* Remove all the wds entries associated with the AP when the AP to 
 * which STA is associated goes down
 */
int ieee80211_node_removeall_wds (struct ieee80211_node_table *nt,struct ieee80211_node *ni)
{
    unsigned int hash;
    struct ieee80211_wds_addr *wds;
    IEEE80211_NODE_LOCK_BH(nt);
    for (hash=0 ;hash < IEEE80211_NODE_HASHSIZE;hash++)
    {
        LIST_FOREACH(wds, &nt->nt_wds_hash[hash], wds_hash) {
            if (wds->wds_ni == ni)
            {
                ieee80211_free_node(wds->wds_ni);
                LIST_REMOVE(wds, wds_hash);
                FREE(wds, M_80211_WDS);
            }
        }
    }
    IEEE80211_NODE_UNLOCK_BH(nt);
    return 0;
}

struct ieee80211_node *
    ieee80211_find_wds_node(struct ieee80211_node_table *nt, const u_int8_t *macaddr)
{
    struct ieee80211_node *ni;

    IEEE80211_NODE_LOCK_BH(nt);
    ni = _ieee80211_find_wds_node(nt, macaddr);
    IEEE80211_NODE_UNLOCK_BH(nt);
    return ni;
}
EXPORT_SYMBOL_C(ieee80211_find_wds_node);

struct ieee80211_node *
#ifdef IEEE80211_DEBUG_REFCNT
    ieee80211_find_node_debug(struct ieee80211_node_table *nt,
        const u_int8_t *macaddr, const char *func, int line)
#else
    ieee80211_find_node(struct ieee80211_node_table *nt, const u_int8_t *macaddr)
#endif
    {
        struct ieee80211_node *ni;

        IEEE80211_NODE_LOCK_BH(nt);
        ni = _ieee80211_find_node(nt, macaddr);
        IEEE80211_NODE_UNLOCK_BH(nt);
        return ni;
    }
#ifdef IEEE80211_DEBUG_REFCNT
EXPORT_SYMBOL_C(ieee80211_find_node_debug);
#else
EXPORT_SYMBOL_C(ieee80211_find_node);
#endif

/*
 * Fake up a node; this handles node discovery in adhoc mode.
 * Note that for the driver's benefit we we treat this like
 * an association so the driver has an opportunity to setup
 * it's private state.
 */
struct ieee80211_node *
    ieee80211_fakeup_adhoc_node(struct ieee80211vap *vap,
        const u_int8_t macaddr[IEEE80211_ADDR_LEN])
{
    struct ieee80211_node *ni;

    ni = ieee80211_dup_bss(vap, macaddr);
    if (ni != NULL)
    {
        /* XXX no rate negotiation; just dup */
        ni->ni_rates = vap->iv_bss->ni_rates;
        if (vap->iv_ic->ic_newassoc != NULL)
            vap->iv_ic->ic_newassoc(ni, 1);
        /* XXX not right for 802.1x/WPA */
        ieee80211_node_authorize(ni);
    }
    return ni;
}

/*
 * Do node discovery in adhoc mode on receipt of a beacon
 * or probe response frame.  Note that for the driver's
 * benefit we we treat this like an association so the
 * driver has an opportunity to setup it's private state.
 */
struct ieee80211_node *
    ieee80211_add_neighbor(struct ieee80211vap *vap,
        const struct ieee80211_frame *wh,
        const struct ieee80211_scanparams *sp)
{
    struct ieee80211com *ic = vap->iv_ic;
    struct ieee80211_node *ni;

    ni = ieee80211_dup_bss(vap, wh->i_addr2);	/* XXX alloc_node? */
    if (ni != NULL)
    {
        ni->ni_esslen = sp->ssid[1];
        memcpy(ni->ni_essid, sp->ssid + 2, sp->ssid[1]);
        IEEE80211_ADDR_COPY(ni->ni_bssid, wh->i_addr3);
        memcpy(ni->ni_tstamp.data, sp->tstamp, sizeof(ni->ni_tstamp));
        ni->ni_intval = sp->bintval;
        ni->ni_capinfo = sp->capinfo;
        ni->ni_chan = ic->ic_curchan;
        ni->ni_fhdwell = sp->fhdwell;
        ni->ni_fhindex = sp->fhindex;
        ni->ni_erp = sp->erp;
        ni->ni_timoff = sp->timoff;
        if (sp->wme != NULL)
            ieee80211_saveie(&ni->ni_wme_ie, sp->wme);
        if (sp->wpa != NULL)
            ieee80211_saveie(&ni->ni_wpa_ie, sp->wpa);
        if (sp->ath != NULL)
            ieee80211_saveath(ni, sp->ath);
#ifdef ATH_WPS_IE
		if (sp->rsn != NULL)
			ieee80211_saveie(&ni->ni_rsn_ie, sp->rsn);
		if (sp->wps != NULL)
			ieee80211_saveie(&ni->ni_wps_ie, sp->wps);
#endif /* ATH_WPS_IE */

        /* NB: must be after ni_chan is setup */
        ieee80211_setup_rates(ni, sp->rates, sp->xrates, IEEE80211_F_DOSORT);

        if (ic->ic_newassoc != NULL)
            ic->ic_newassoc(ni, 1);
        /* XXX not right for 802.1x/WPA */
        ieee80211_node_authorize(ni);
    }
    return ni;
}

/*
 * Locate the node for sender, track state, and then pass the
 * (referenced) node up to the 802.11 layer for its use.  We
 * return NULL when the sender is unknown; the driver is required
 * locate the appropriate virtual ap in that case; possibly
 * sending it to all (using ieee80211_input_all).
 */
struct ieee80211_node *
#ifdef IEEE80211_DEBUG_REFCNT
    ieee80211_find_rxnode_debug(struct ieee80211com *ic,
        const struct ieee80211_frame_min *wh, const char *func, int line)
#else
    ieee80211_find_rxnode(struct ieee80211com *ic,
        const struct ieee80211_frame_min *wh)
#endif
    {
#define	IS_CTL(wh) \
	((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) == IEEE80211_FC0_TYPE_CTL)
#define	IS_PSPOLL(wh) \
	((wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK) == IEEE80211_FC0_SUBTYPE_PS_POLL)
#define	IS_BAR(wh) \
	((wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK) == IEEE80211_FC0_SUBTYPE_BAR)
        struct ieee80211_node_table *nt;
        struct ieee80211_node *ni;

        /* XXX check ic_bss first in station mode */
        /* XXX 4-address frames? */
        nt = &ic->ic_sta;
        IEEE80211_NODE_LOCK(nt);
        if (IS_CTL(wh) && !IS_PSPOLL(wh) && !IS_BAR(wh))
            ni = _ieee80211_find_node(nt, wh->i_addr1);
        else
            ni = _ieee80211_find_node(nt, wh->i_addr2);
        IEEE80211_NODE_UNLOCK(nt);

        return ni;
#undef IS_BAR
#undef IS_PSPOLL
#undef IS_CTL
    }
#ifdef IEEE80211_DEBUG_REFCNT
EXPORT_SYMBOL_C(ieee80211_find_rxnode_debug);
#else
EXPORT_SYMBOL_C(ieee80211_find_rxnode);
#endif

/*
 * Return a reference to the appropriate node for sending
 * a data frame.  This handles node discovery in adhoc networks.
 */
struct ieee80211_node *
#ifdef IEEE80211_DEBUG_REFCNT
    ieee80211_find_txnode_debug(struct ieee80211vap *vap, const u_int8_t *mac,
        const char *func, int line)
#else
    ieee80211_find_txnode(struct ieee80211vap *vap, const u_int8_t *mac)
#endif
    {
        struct ieee80211_node_table *nt;
        struct ieee80211_node *ni;

        /*
        * The destination address should be in the node table
        * unless we are operating in station mode or this is a
        * multicast/broadcast frame.
        */
        if (vap->iv_opmode == IEEE80211_M_STA) {
#ifdef NODE_FREE_DEBUG
            return ieee80211_ref_node(vap->iv_bss, __func__);
#else
            return ieee80211_ref_node(vap->iv_bss);
#endif
        }

		/*
		 * If it's mcast frame and there's no STA associated with this vap, 
		 * NULL should be returned.
		 */
		if(IEEE80211_IS_MULTICAST(mac)) {
			if(vap->iv_sta_assoc > 0) {
#ifdef NODE_FREE_DEBUG
				return ieee80211_ref_node(vap->iv_bss, __func__);
#else
				return ieee80211_ref_node(vap->iv_bss);
#endif
			} else {
                vap->iv_stats.is_tx_nonode++;
				return NULL;
			}
		}

        /* XXX can't hold lock across dup_bss 'cuz of recursive locking */
        nt = &vap->iv_ic->ic_sta;
        IEEE80211_NODE_LOCK_BH(nt);
        ni = _ieee80211_find_node(nt, mac);
        IEEE80211_NODE_UNLOCK_BH(nt);

        if (ni == NULL)
        {
            if (vap->iv_opmode == IEEE80211_M_IBSS ||
                vap->iv_opmode == IEEE80211_M_AHDEMO)
            {
                /*
                 * In adhoc mode cons up a node for the destination.
                 * Note that we need an additional reference for the
                 * caller to be consistent with _ieee80211_find_node.
                 */
                ni = ieee80211_fakeup_adhoc_node(vap, mac);
                if (ni != NULL) {
#ifdef NODE_FREE_DEBUG
                    (void) ieee80211_ref_node(ni, __func__);
#else
                    (void) ieee80211_ref_node(ni);
#endif
                }
            }
            else
            {
                IEEE80211_NOTE_MAC(vap, IEEE80211_MSG_OUTPUT, mac,
                    "no node, discard frame (%s)", __func__);
                vap->iv_stats.is_tx_nonode++;
            }
        }
        else
        {
            /* 
            * Since all vaps share the same node table, we may find someone else's
            * node (sigh!).
            */
            if (ni->ni_vap != vap)
            {
                /*
                * :-( since _ieee80211_find_node(..) ups the ref count
                */
#ifdef NODE_FREE_DEBUG
                ieee80211_unref_node(&ni, __func__);
#else
                ieee80211_unref_node(&ni);
#endif
                IEEE80211_NOTE_MAC(vap, IEEE80211_MSG_OUTPUT, mac,
                    "no node, discard frame (%s)", __func__);
                vap->iv_stats.is_tx_nonode++;
                return NULL;
            }
        }
        return ni;
    }
#ifdef IEEE80211_DEBUG_REFCNT
EXPORT_SYMBOL_C(ieee80211_find_txnode_debug);
#else
EXPORT_SYMBOL_C(ieee80211_find_txnode);
#endif

static void
ieee80211_free_node_delay(unsigned long arg)
{
    struct ieee80211com   *ic = (struct ieee80211com *) arg;
    struct node_entry     *del_node;
    struct ieee80211_node *ni;

    IEEE80211_NODE_FREE_LOCK(ic);
    while (!TAILQ_EMPTY(&ic->ic_free_entryq)) {
        del_node = TAILQ_FIRST(&ic->ic_free_entryq);
        if ((jiffies - del_node->queue_time) > (IEEE80211_NODE_FREE_WAIT*HZ)) {
            TAILQ_REMOVE(&ic->ic_free_entryq, del_node, entry);
            IEEE80211_NODE_FREE_UNLOCK(ic);

            ni = del_node->node;

            ic->ic_node_free(ni);
            FREE(del_node, M_DEVBUF);

            IEEE80211_NODE_FREE_LOCK(ic);
        } else {
            break;
        }
    }
    IEEE80211_NODE_FREE_UNLOCK(ic);

    ic->ic_free.expires = jiffies + IEEE80211_NODE_FREE_WAIT*HZ;
    add_timer(&ic->ic_free);
}

static void
ieee80211_free_node_delay_cleanup(struct ieee80211com *ic)
{
    struct node_entry     *del_node;
    struct ieee80211_node *ni;

    IEEE80211_NODE_FREE_LOCK(ic);

    while (!TAILQ_EMPTY(&ic->ic_free_entryq)) {
        del_node = TAILQ_FIRST(&ic->ic_free_entryq);
        if (del_node != NULL) {
            TAILQ_REMOVE(&ic->ic_free_entryq, del_node, entry);
            IEEE80211_NODE_FREE_UNLOCK(ic);

            ni = del_node->node;

            ic->ic_node_free(ni);
            FREE(del_node, M_DEVBUF);

            IEEE80211_NODE_FREE_LOCK(ic);
        }
    }
    IEEE80211_NODE_FREE_UNLOCK(ic);
}

static void
_ieee80211_free_node(struct ieee80211_node *ni)
{
    struct ieee80211vap *vap = ni->ni_vap;
    struct ieee80211_node_table *nt = ni->ni_table;
    struct ieee80211com * ic = (struct ieee80211com *)ni->ni_ic;
    struct node_entry *del_node;

    if (!ni->ni_tmp_node) {
        MALLOC(del_node, struct node_entry *, sizeof(struct node_entry *),
               M_DEVBUF, M_NOWAIT | M_ZERO);

        if (del_node) {
            del_node->node = ni;

            IEEE80211_DPRINTF(vap, IEEE80211_MSG_NODE,
                              "%s %p<%s> in %s table\n", __func__, ni,
                              ether_sprintf(ni->ni_macaddr),
                              nt != NULL ? nt->nt_name : "<gone>");
            if (vap->iv_aid_bitmap != NULL)
                IEEE80211_AID_CLR(vap, ni->ni_associd);
            if (nt != NULL)
            {
                TAILQ_REMOVE(&nt->nt_node, ni, ni_list);
                LIST_REMOVE(ni, ni_hash);
            }

            IEEE80211_NODE_FREE_LOCK(ic);
            del_node->queue_time = jiffies;
            TAILQ_INSERT_TAIL(&ic->ic_free_entryq, del_node, entry);
            IEEE80211_NODE_FREE_UNLOCK(ic);

            ic->ic_node_cleanup(ni);
            return;
        }
    }

    IEEE80211_DPRINTF(vap, IEEE80211_MSG_NODE,
        "%s %p<%s> in %s table\n", __func__, ni,
        ether_sprintf(ni->ni_macaddr),
        nt != NULL ? nt->nt_name : "<gone>");
    if (vap->iv_aid_bitmap != NULL)
        IEEE80211_AID_CLR(vap, ni->ni_associd);
    if (nt != NULL)
    {
        TAILQ_REMOVE(&nt->nt_node, ni, ni_list);
        LIST_REMOVE(ni, ni_hash);
    }
    ic->ic_node_cleanup(ni);
    ic->ic_node_free(ni);
}

void
#ifdef IEEE80211_DEBUG_REFCNT
ieee80211_free_node_debug(struct ieee80211_node *ni, const char *func, int line)
#else
#ifdef NODE_FREE_DEBUG
orig_ieee80211_free_node(struct ieee80211_node *ni)
#else
ieee80211_free_node(struct ieee80211_node *ni)
#endif
#endif
{
    struct ieee80211_node_table *nt = ni->ni_table;
    struct ieee80211com *ic = ni->ni_ic;

#ifdef IEEE80211_DEBUG_REFCNT
    IEEE80211_DPRINTF(ni->ni_vap, IEEE80211_MSG_NODE,
        "%s (%s:%u) %p<%s> refcnt %d\n", __func__, func, line, ni,
        ether_sprintf(ni->ni_macaddr), ieee80211_node_refcnt(ni)-1);
#endif
    /*
     * XXX: may need to lock out the following race. we dectestref
     *      and determine it's time to free the node. between the if()
     *      and lock, we take an rx intr to receive a frame from this
     *      node. the rx path (tasklet or intr) bumps this node's
     *      refcnt and xmits a response frame. eventually that response
     *      will get reaped, and the reaping code will attempt to use
     *      the node. the code below will delete the node prior
     *      to the reap and we could get a crash.
     *
     *      as a stopgap before delving deeper, lock intrs to
     *      prevent this case.
     */
    IEEE80211_LOCK(ic);
#ifdef NODE_FREE_DEBUG
        ieee80211_add_trace(ni, (char *)__func__, "RefcntBefore", ieee80211_node_refcnt(ni));
#endif
    if (ieee80211_node_dectestref(ni))
    {
        /*
        * Beware; if the node is marked gone then it's already
        * been removed from the table and we cannot assume the
        * table still exists.  Regardless, there's no need to lock
        * the table.
        */
        if (ni->ni_table != NULL)
        {
#ifdef NODE_FREE_DEBUG
        ieee80211_add_trace(ni, (char *)__func__, "NT!NULL", ieee80211_node_refcnt(ni));
#endif
            IEEE80211_NODE_LOCK_BH(nt);
            _ieee80211_free_node(ni);
            IEEE80211_NODE_UNLOCK_BH(nt);
        }
        else {
#ifdef NODE_FREE_DEBUG
        ieee80211_add_trace(ni, (char *)__func__, "NTNULL", ieee80211_node_refcnt(ni));
#endif
            _ieee80211_free_node(ni);
        }
    }
    IEEE80211_UNLOCK(ic);
}
#ifdef IEEE80211_DEBUG_REFCNT
EXPORT_SYMBOL_C(ieee80211_free_node_debug);
#else
#ifdef NODE_FREE_DEBUG
EXPORT_SYMBOL_C(orig_ieee80211_free_node);
#else
EXPORT_SYMBOL_C(ieee80211_free_node);
#endif
#endif

/*
 * Reclaim a node.  If this is the last reference count then
 * do the normal free work.  Otherwise remove it from the node
 * table and mark it gone by clearing the back-reference.
 */
static void
node_reclaim(struct ieee80211_node_table *nt, struct ieee80211_node *ni)
{

    IEEE80211_DPRINTF(ni->ni_vap, IEEE80211_MSG_NODE,
        "%s: remove %p<%s> from %s table, refcnt %d\n",
        __func__, ni, ether_sprintf(ni->ni_macaddr),
        nt->nt_name, ieee80211_node_refcnt(ni)-1);
    if (!ieee80211_node_dectestref(ni))
    {
        /*
         * Other references are present, just remove the
         * node from the table so it cannot be found.  When
         * the references are dropped storage will be
         * reclaimed.  This normally only happens for ic_bss.
         */
        TAILQ_REMOVE(&nt->nt_node, ni, ni_list);
        LIST_REMOVE(ni, ni_hash);
        ni->ni_table = NULL;		/* clear reference */
    }
    else
        _ieee80211_free_node(ni);
}

static void
ieee80211_node_table_reset(struct ieee80211_node_table *nt,
    struct ieee80211vap *match)
{
    struct ieee80211_node *ni, *next;

    IEEE80211_NODE_LOCK_BH(nt);
    TAILQ_FOREACH_SAFE(ni, &nt->nt_node, ni_list, next) {
        if (match != NULL && ni->ni_vap != match)
            continue;
        if (ni->ni_associd != 0)
        {
            struct ieee80211vap *vap = ni->ni_vap;

            if (vap->iv_auth->ia_node_leave != NULL)
                vap->iv_auth->ia_node_leave(ni);
            if (vap->iv_aid_bitmap != NULL)
                IEEE80211_AID_CLR(vap, ni->ni_associd);
        }
        node_reclaim(nt, ni);
        /* Remove the node reference from wds table if it's recorded in.
         * This will insure the node is not reprocessed for inactivity.
         * fix for bug #32448
         */
        ieee80211_remove_wds_addr(nt,ni->ni_macaddr);
        ieee80211_del_wds_node(nt,ni);
    }
    IEEE80211_NODE_UNLOCK_BH(nt);
}

static void
ieee80211_node_table_cleanup(struct ieee80211_node_table *nt)
{
    struct ieee80211_node *ni, *next;

    TAILQ_FOREACH_SAFE(ni, &nt->nt_node, ni_list, next) {
        if (ni->ni_associd != 0)
        {
            struct ieee80211vap *vap = ni->ni_vap;

            if (vap->iv_auth->ia_node_leave != NULL)
                vap->iv_auth->ia_node_leave(ni);
            if (vap->iv_aid_bitmap != NULL)
                IEEE80211_AID_CLR(vap, ni->ni_associd);
        }
        node_reclaim(nt, ni);
    }
    del_timer(&nt->nt_wds_aging_timer);
    IEEE80211_SCAN_LOCK_DESTROY(nt);
    IEEE80211_NODE_LOCK_DESTROY(nt);
    IEEE80211_SCAN_LOCK_DESTROY(nt);
    IEEE80211_NODE_LOCK_DESTROY(nt);
}

/*
 * Timeout inactive stations and do related housekeeping.
 * Note that we cannot hold the node lock while sending a
 * frame as this would lead to a LOR.  Instead we use a
 * generation number to mark nodes that we've scanned and
 * drop the lock and restart a scan if we have to time out
 * a node.  Since we are single-threaded by virtue of
 * controlling the inactivity timer we can be sure this will
 * process each node only once.
 */
static void
ieee80211_timeout_stations(struct ieee80211_node_table *nt)
{
    struct ieee80211_node *ni;
    struct ieee80211com *ic = nt->nt_ic;
    struct sk_buff_head skb_freeq;
    struct sk_buff *skb;
    u_int gen;

    skb_queue_head_init(&skb_freeq);

    /* In case non-HT & HT20 station count get out of whack */
    IEEE80211_LOCK(ic);
    ieee80211_ht_prot(ic);
    IEEE80211_UNLOCK(ic);

    IEEE80211_SCAN_LOCK(nt);
    gen = nt->nt_scangen++;
restart:
    IEEE80211_NODE_LOCK(nt);
    TAILQ_FOREACH(ni, &nt->nt_node, ni_list) {
        if (ni->ni_scangen == gen)	/* previously handled */
            continue;
        /*
        * Ignore entries for which have yet to receive an
        * authentication frame.  These are transient and
        * will be reclaimed when the last reference to them
        * goes away (when frame xmits complete).
        */
        if ((ni->ni_flags & IEEE80211_NODE_AREF) == 0)
            continue;

	
#ifdef ATH_WDS_INTEROP
	/* 
	 * ignore Repeaters and Base stations added by configuration
	 * they cannot be idled out 
	 */
	if ((ni->ni_flags & IEEE80211_NODE_REPEATER)==IEEE80211_NODE_REPEATER)
	{
		continue;
	}

#endif
	ni->ni_scangen = gen;
        /*
        * Free fragment if not needed anymore
        * (last fragment older than 1s).
        * XXX doesn't belong here
        */
        if (ni->ni_rxfrag[0] != NULL &&
            jiffies > ni->ni_rxfragstamp + HZ)
        {
            kfree_skb(ni->ni_rxfrag[0]);
            ni->ni_rxfrag[0] = NULL;
        }
        /*
        * Special case ourself; we may be idle for extended periods
        * of time and regardless reclaiming our state is wrong.
        */
        if (ni == ni->ni_vap->iv_bss)
        {
            /* NB: don't permit it to go negative */
            if (ni->ni_inact > 0)
                ni->ni_inact--;
            continue;
        }
        ni->ni_inact--;
        if (ni->ni_associd != 0)
        {
            struct ieee80211vap *vap = ni->ni_vap;
            /*
            * Age frames on the power save queue.
            */
            if (ieee80211_node_saveq_age(ni, &skb_freeq) != 0 &&
                IEEE80211_NODE_SAVEQ_QLEN(ni) == 0 &&
                vap->iv_set_tim != NULL)
                vap->iv_set_tim(ni, 0);
            /*
            * Probe the station before time it out.  We
            * send a null data frame which may not be
            * universally supported by drivers (need it
            * for ps-poll support so it should be...).
            */
            if (0 < ni->ni_inact &&
                ni->ni_inact <= vap->iv_inact_probe)
            {
                IEEE80211_NOTE(vap,
                    IEEE80211_MSG_INACT | IEEE80211_MSG_NODE,
                    ni, "%s",
                    "probe station due to inactivity");
                /*
                 * Grab a reference before unlocking the table
                 * so the node cannot be reclaimed before we
                 * send the frame. ieee80211_send_nulldata
                 * understands we've done this and reclaims the
                 * ref for us as needed.
                 */
#ifdef NODE_FREE_DEBUG
                ieee80211_ref_node(ni, __func__);
#else
                ieee80211_ref_node(ni);
#endif
                IEEE80211_NODE_UNLOCK(nt);
                ieee80211_send_nulldata(ni);
                /* XXX stat? */
                goto restart;
            }
        }
        if (ni->ni_inact <= 0)
        {
            IEEE80211_NOTE(ni->ni_vap,
                IEEE80211_MSG_INACT | IEEE80211_MSG_NODE, ni,
                "station timed out due to inactivity (refcnt %u)",
                ieee80211_node_refcnt(ni));
            /*
             * Send a deauthenticate frame and drop the station.
             * We grab a reference before unlocking the table so
             * the node cannot be reclaimed before we complete our
             * work.
             *
             * Separately we must drop the node lock before sending
             * in case the driver takes a lock, as this may result
             * in a LOR between the node lock and the driver lock.
             */
            ni->ni_vap->iv_stats.is_node_timeout++;
#ifdef NODE_FREE_DEBUG
            ieee80211_ref_node(ni, __func__);
#else
            ieee80211_ref_node(ni);
#endif
            IEEE80211_NODE_UNLOCK(nt);
            if (ni->ni_associd != 0)
            {
                int arg = IEEE80211_REASON_AUTH_EXPIRE;
                IEEE80211_SEND_MGMT(ni,
                    IEEE80211_FC0_SUBTYPE_DEAUTH, (void *)&arg);
            }
            ieee80211_node_leave(ni);
            ieee80211_free_node(ni);
            goto restart;
        }
    }
    IEEE80211_NODE_UNLOCK(nt);

    IEEE80211_SCAN_UNLOCK(nt);

    while ((skb = skb_peek(&skb_freeq)) != NULL) {
        skb = __skb_dequeue(&skb_freeq);
        kfree_skb(skb);
    }
}

/*
 * Per-ieee80211com inactivity timer callback.
 */
static void
ieee80211_node_timeout(unsigned long arg)
{
    struct ieee80211com *ic = (struct ieee80211com *) arg;

    ieee80211_scan_timeout(ic);
    ieee80211_timeout_stations(&ic->ic_sta);

    ic->ic_inact.expires = jiffies + IEEE80211_INACT_WAIT*HZ;
    add_timer(&ic->ic_inact);
}

/*
 * This function is called with IEEE80211_LOCK held.
 */
void
ieee80211_iterate_nodes_nolock(struct ieee80211_node_table *nt, ieee80211_iter_func *f, void *arg)
{
    struct ieee80211_node *ni;
    u_int gen;

    IEEE80211_SCAN_LOCK(nt);
    gen = nt->nt_scangen++;
restart:
    IEEE80211_NODE_LOCK(nt);
    TAILQ_FOREACH(ni, &nt->nt_node, ni_list) {
        if (ni->ni_scangen != gen)
        {
            ni->ni_scangen = gen;
#ifdef NODE_FREE_DEBUG
            (void) ieee80211_ref_node(ni, __func__);
#else
            (void) ieee80211_ref_node(ni);
#endif
            IEEE80211_NODE_UNLOCK(nt);
            if (f != NULL)
                (*f)(arg, ni);
            if (ieee80211_node_dectestref(ni)) {
                _ieee80211_free_node(ni);
            }
            goto restart;
        }
    }
    IEEE80211_NODE_UNLOCK(nt);

    IEEE80211_SCAN_UNLOCK(nt);
}
EXPORT_SYMBOL_C(ieee80211_iterate_nodes_nolock);

void
ieee80211_iterate_nodes(struct ieee80211_node_table *nt, ieee80211_iter_func *f, void *arg)
{
    struct ieee80211_node *ni;
    u_int gen;

    IEEE80211_SCAN_LOCK(nt);
    gen = nt->nt_scangen++;
restart:
    IEEE80211_NODE_LOCK(nt);
    TAILQ_FOREACH(ni, &nt->nt_node, ni_list) {
        if (ni->ni_scangen != gen)
        {
            ni->ni_scangen = gen;
#ifdef NODE_FREE_DEBUG
            (void) ieee80211_ref_node(ni, __func__);
#else
            (void) ieee80211_ref_node(ni);
#endif
            IEEE80211_NODE_UNLOCK(nt);
   	    if (f != NULL)
       	       (*f)(arg, ni);
            ieee80211_free_node(ni);
            goto restart;
        }
    }
    IEEE80211_NODE_UNLOCK(nt);

    IEEE80211_SCAN_UNLOCK(nt);
}
EXPORT_SYMBOL_C(ieee80211_iterate_nodes);

void
ieee80211_iterate_vap_nodes(struct ieee80211vap *reqvap, ieee80211_iter_func *f, void *arg)
{
    struct ieee80211com *ic = reqvap->iv_ic;
    struct ieee80211_node_table *nt = &ic->ic_sta;
    struct ieee80211_node *ni;

    IEEE80211_NODE_LOCK(nt);
    TAILQ_FOREACH(ni, &nt->nt_node, ni_list) {
        struct ieee80211vap *vap=ni->ni_vap;
	if (vap && vap != reqvap) /* only entries for this vap (or) xrvap */
		continue ;
#ifdef NODE_FREE_DEBUG
	(void) ieee80211_ref_node(ni, __func__);
#else
	(void) ieee80211_ref_node(ni);
#endif
	(*f)(arg, ni);
	ieee80211_free_node(ni);
    }
    IEEE80211_NODE_UNLOCK(nt);
}
EXPORT_SYMBOL_C(ieee80211_iterate_vap_nodes);
	
void
ieee80211_dump_node(struct ieee80211_node_table *nt, struct ieee80211_node *ni)
{
    int i;

    printf("0x%p: mac %s refcnt %d\n", ni,
        ether_sprintf(ni->ni_macaddr), ieee80211_node_refcnt(ni));
    printf("\tscangen %u authmode %u flags 0x%x\n",
        ni->ni_scangen, ni->ni_authmode, ni->ni_flags);
    printf("\tassocid 0x%x txpower %u vlan %u\n",
        ni->ni_associd, ni->ni_txpower, ni->ni_vlan);
    printf ("rxfragstamp %u\n", ni->ni_rxfragstamp);
    for (i=0; i<17; i++)
    {
        printf("\t%d: txseq %u rxseq %u fragno %u\n", i, 
            ni->ni_txseqs[i],
            ni->ni_rxseqs[i] >> IEEE80211_SEQ_SEQ_SHIFT,
            ni->ni_rxseqs[i] & IEEE80211_SEQ_FRAG_MASK);
    }
    printf("\trstamp %llu rssi %u intval %u capinfo 0x%x\n",
        (unsigned long long)ni->ni_rstamp, ni->ni_rssi, ni->ni_intval, ni->ni_capinfo);
    printf("\tbssid %s essid \"%.*s\" channel %u:0x%x\n",
        ether_sprintf(ni->ni_bssid),
        ni->ni_esslen, ni->ni_essid,
        ni->ni_chan != IEEE80211_CHAN_ANYC ?
        ni->ni_chan->ic_freq : IEEE80211_CHAN_ANY,
        ni->ni_chan != IEEE80211_CHAN_ANYC ? ni->ni_chan->ic_flags : 0);
    printf("\tinact %u txrate %u\n",
        ni->ni_inact, ni->ni_txrate);
    /* 11n */
    printf("\tchwidth %u\n", ni->ni_chwidth);

}

void
ieee80211_dump_nodes(struct ieee80211_node_table *nt)
{
    ieee80211_iterate_nodes(nt,
        (ieee80211_iter_func *) ieee80211_dump_node, nt);
}

/*
 * Handle a station joining an 11g network.
 */
static void
ieee80211_node_join_11g(struct ieee80211_node *ni)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ieee80211vap *vap = ni->ni_vap;

    IEEE80211_LOCK_ASSERT(ic);

    KASSERT((IEEE80211_IS_CHAN_ANYG(ic->ic_bsschan) ||
        IEEE80211_IS_CHAN_11NG(ic->ic_bsschan)),
        ("not in 11g, bss %u:0x%x, curmode %u", ic->ic_bsschan->ic_freq,
        ic->ic_bsschan->ic_flags, ic->ic_curmode));

    /*
    * Station isn't capable of short slot time.  Bump
    * the count of long slot time stations and disable
    * use of short slot time.  Note that the actual switch
    * over to long slot time use may not occur until the
    * next beacon transmission (per sec. 7.3.1.4 of 11g).
    */
    if ((ni->ni_capinfo & IEEE80211_CAPINFO_SHORT_SLOTTIME) == 0)
    {
        ic->ic_longslotsta++;
        IEEE80211_NOTE(vap, IEEE80211_MSG_ASSOC, ni,
            "station needs long slot time, count %d",
            ic->ic_longslotsta);
        /* XXX vap's w/ conflicting needs won't work */
        if (!IEEE80211_IS_CHAN_108G(ic->ic_bsschan))
        {
            /*
             * Don't force slot time when switched to turbo
             * mode as non-ERP stations won't be present; this
             * need only be done when on the normal G channel.
             */
            ieee80211_set_shortslottime(ic, 0);
        }
    }
    /*
    * If the new station is not an ERP station
    * then bump the counter and enable protection
    * if configured.
    */
    if (!ieee80211_iserp_rateset(ic, &ni->ni_rates))
    {
        ic->ic_nonerpsta++;
        IEEE80211_NOTE(vap, IEEE80211_MSG_ASSOC, ni,
            "station is !ERP, %d non-ERP stations associated",
            ic->ic_nonerpsta);
        /*
        * If protection is configured, enable it.
        */
        if (ic->ic_protmode != IEEE80211_PROT_NONE)
        {
            IEEE80211_DPRINTF(vap, IEEE80211_MSG_ASSOC,
                "%s: enable use of protection\n", __func__);
            ic->ic_flags |= IEEE80211_F_USEPROT;
            ic->ic_update_protmode(ic);
        }
        /*
        * If station does not support short preamble
        * then we must enable use of Barker preamble.
        */
        if ((ni->ni_capinfo & IEEE80211_CAPINFO_SHORT_PREAMBLE) == 0)
        {
            IEEE80211_NOTE(vap, IEEE80211_MSG_ASSOC, ni,
                "%s", "station needs long preamble");
            ic->ic_flags |= IEEE80211_F_USEBARKER;
            ic->ic_flags &= ~IEEE80211_F_SHPREAMBLE;
        }

        /* Update ERP element if this is first non ERP station */
        if (ic->ic_nonerpsta == 1)
        {
            struct ieee80211vap *tmpvap;

            ieee80211_enumerate_vaps(tmpvap, ic) {
                tmpvap->iv_flags_ext |= IEEE80211_FEXT_ERPUPDATE;
            }

        }
    }
    else
        ni->ni_flags |= IEEE80211_NODE_ERP;
}

void
ieee80211_node_join(struct ieee80211_node *ni, int resp)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ieee80211vap *vap = ni->ni_vap;
    int newassoc, arg;

    if (ni->ni_associd == 0)
    {
        u_int16_t aid;

        KASSERT(vap->iv_aid_bitmap != NULL, ("no aid bitmap"));
        /*
        * It would be good to search the bitmap
        * more efficiently, but this will do for now.
        */
        for (aid = 1; aid < vap->iv_max_aid; aid++)
        {
            if (!IEEE80211_AID_ISSET(vap, aid))
                break;
        }
        if (aid >= vap->iv_max_aid)
        {
            arg = IEEE80211_REASON_ASSOC_TOOMANY;
            IEEE80211_SEND_MGMT(ni, resp, (void *)&arg);
            ieee80211_node_leave(ni);
            return;
        }
        ni->ni_associd = aid | 0xc000;

        IEEE80211_LOCK(ic);
        IEEE80211_AID_SET(vap, ni->ni_associd);
        vap->iv_sta_assoc++;
        ic->ic_sta_assoc++;
        if ((ni->ni_flags & IEEE80211_NODE_HT) &&
            (ni->ni_ath_flags & IEEE80211_NODE_40_INTOLERANT)) {
                ieee80211_change_cw(ic, 1);
        }
#ifdef ATH_SUPERG_XR
        if(ni->ni_vap->iv_flags & IEEE80211_F_XR)
            ic->ic_xr_sta_assoc++;
#endif
        if (IEEE80211_ATH_CAP(vap, ni, IEEE80211_ATHC_TURBOP))
            ic->ic_dt_sta_assoc++;

        if (IEEE80211_IS_CHAN_ANYG(ic->ic_bsschan) ||
            IEEE80211_IS_CHAN_11NG(ic->ic_bsschan))
            ieee80211_node_join_11g(ni);

        ieee80211_ht_prot(ic);
        IEEE80211_UNLOCK(ic);

        newassoc = 1;
    }
    else
        newassoc = 0;

    IEEE80211_NOTE(vap, IEEE80211_MSG_ASSOC | IEEE80211_MSG_DEBUG, ni,
        "station %sassociated at aid %d: %s preamble, %s slot time"
        "%s%s%s%s%s%s%s",
        newassoc ? "" : "re",
        IEEE80211_NODE_AID(ni),
        ic->ic_flags & IEEE80211_F_SHPREAMBLE ? "short" : "long",
        ic->ic_flags & IEEE80211_F_SHSLOT ? "short" : "long",
        ic->ic_flags & IEEE80211_F_USEPROT ? ", protection" : "",
        ni->ni_flags & IEEE80211_NODE_QOS ? ", QoS" : "",
        IEEE80211_ATH_CAP(vap, ni, IEEE80211_NODE_TURBOP) ?
        ", turbo" : "",
        IEEE80211_ATH_CAP(vap, ni, IEEE80211_NODE_COMP) ?
        ", compression" : "",
        IEEE80211_ATH_CAP(vap, ni, IEEE80211_NODE_FF) ?
        ", fast-frames" : "",
        IEEE80211_ATH_CAP(vap, ni, IEEE80211_NODE_XR) ? ", XR" : "",
        IEEE80211_ATH_CAP(vap, ni, IEEE80211_NODE_AR) ? ", AR" : ""
        );

    /*
     * For an existing node reassociation, reset key to NONE.
     */
    if (ni->ni_ucastkey.wk_keyix != IEEE80211_KEYIX_NONE) {
        ieee80211_crypto_delkey(ni->ni_vap, &ni->ni_ucastkey, ni);
    }

	/* Multicast enhancement: If the entry with the node's address exists in 
	 * the snoop table, it should be removed.
	 */
#ifdef IEEE80211_MCAST_ENHANCEMENT
	if(vap->iv_me_ops && vap->iv_me_ops->ieee80211_me_cleanup) {
		vap->iv_me_ops->ieee80211_me_cleanup(ni);
	}
#endif

    /* give driver a chance to setup state like ni_txrate */
    if (ic->ic_newassoc != NULL)
        ic->ic_newassoc(ni, newassoc);
    ni->ni_inact_reload = vap->iv_inact_auth;
    ni->ni_inact = ni->ni_inact_reload;
#ifdef ATH_WDS_INTEROP
    /* do not send mgmt frame for configured repeaters */
    if ((ni->ni_flags & IEEE80211_NODE_REPEATER) != IEEE80211_NODE_REPEATER)
#endif
    {
	    arg = IEEE80211_STATUS_SUCCESS;
	    IEEE80211_SEND_MGMT(ni, resp, (void *)&arg);
    }
    /* tell the authenticator about new station */
    if (vap->iv_auth->ia_node_join != NULL)
        vap->iv_auth->ia_node_join(ni);
    ieee80211_notify_node_join(ni, newassoc);
#ifdef ATH_SUPPORT_IQUE
	/*
	 * Init the state machine for headline block removal
	 */
	ni->ni_hbr_block = 0;
	ni->ni_ique_flag = 0;
	ieee80211_hbr_addentry(vap, ni->ni_macaddr);
#endif	
	if ((ni->ni_vap->iv_flags_ext & IEEE80211_FEXT_WDS) &&
	    (!IEEE80211_ADDR_EQ(ni->ni_macaddr, ni->ni_bssid)) &&
        (vap->iv_opmode == IEEE80211_M_HOSTAP) ) {
		struct ieee80211vap *v;
		int nvaps = 0;
    	/* do a full search to mark all the allocated vaps */
    	ieee80211_enumerate_vaps(v, ic) {
       		nvaps++;
    	}

		if (nvaps == 1) {
            /* Grab a reference */
			ieee80211_ref_node(ni);
			ieee80211_proxy_deauth(ni, IEEE80211_REASON_AUTH_EXPIRE);
			/* Dereference */
			ieee80211_free_node(ni);
		}
	}
}

/*
 * Handle a station leaving an 11g network.
 */
static void
ieee80211_node_leave_11g(struct ieee80211_node *ni)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ieee80211vap *vap = ni->ni_vap;

    IEEE80211_LOCK_ASSERT(ic);

    KASSERT((IEEE80211_IS_CHAN_ANYG(ic->ic_bsschan) 
        || IEEE80211_IS_CHAN_11NG(ic->ic_bsschan)),
        ("not in 11g, bss %u:0x%x, curmode %u", ic->ic_bsschan->ic_freq,
        ic->ic_bsschan->ic_flags, ic->ic_curmode));

    /*
     * If a long slot station do the slot time bookkeeping.
     */
    if ((ni->ni_capinfo & IEEE80211_CAPINFO_SHORT_SLOTTIME) == 0)
    {
        KASSERT(ic->ic_longslotsta > 0,
            ("bogus long slot station count %d", ic->ic_longslotsta));
        ic->ic_longslotsta--;
        IEEE80211_NOTE(vap, IEEE80211_MSG_ASSOC, ni,
            "long slot time station leaves, count now %d",
            ic->ic_longslotsta);
        if (ic->ic_longslotsta == 0)
        {
            /*
            * Re-enable use of short slot time if supported
            * and not operating in IBSS mode (per spec).
            */
            if ((ic->ic_caps & IEEE80211_C_SHSLOT) &&
                vap->iv_opmode != IEEE80211_M_IBSS)
            {
                IEEE80211_DPRINTF(vap, IEEE80211_MSG_ASSOC,
                    "%s: re-enable use of short slot time\n",
                    __func__);
                ieee80211_set_shortslottime(ic, 1);
            }
        }
    }
    /*
    * If a non-ERP station do the protection-related bookkeeping.
    */
    if ((ni->ni_flags & IEEE80211_NODE_ERP) == 0)
    {
        KASSERT(ic->ic_nonerpsta > 0,
            ("bogus non-ERP station count %d", ic->ic_nonerpsta));
        ic->ic_nonerpsta--;
        IEEE80211_NOTE(vap, IEEE80211_MSG_ASSOC, ni,
            "non-ERP station leaves, count now %d", ic->ic_nonerpsta);
        if (ic->ic_nonerpsta == 0)
        {
            
            struct ieee80211vap *tmpvap;

            if ((jiffies - ic->ic_time_nonerp_present) >= IEEE80211_INACT_NONERP * HZ)
            {
                IEEE80211_DPRINTF(vap, IEEE80211_MSG_ASSOC,
                    "%s: disable use of protection\n", __func__);
                ic->ic_flags &= ~IEEE80211_F_USEPROT;
                ic->ic_update_protmode(ic);
            }
            /* XXX verify mode? */
            if (ic->ic_caps & IEEE80211_C_SHPREAMBLE)
            {
                IEEE80211_DPRINTF(vap, IEEE80211_MSG_ASSOC,
                    "%s: re-enable use of short preamble\n",
                    __func__);
                ic->ic_flags |= IEEE80211_F_SHPREAMBLE;
                ic->ic_flags &= ~IEEE80211_F_USEBARKER;
            }

            ieee80211_enumerate_vaps(tmpvap, ic) {
                tmpvap->iv_flags_ext |= IEEE80211_FEXT_ERPUPDATE;
            }
        }
    }
}

/*
 * Handle bookkeeping for a station/neighbor leaving
 * the bss when operating in ap or adhoc modes.
 */
void
ieee80211_node_leave(struct ieee80211_node *ni)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ieee80211vap *vap = ni->ni_vap;
    struct ieee80211_node_table *nt = ni->ni_table;

    IEEE80211_NOTE(vap, IEEE80211_MSG_ASSOC | IEEE80211_MSG_DEBUG, ni,
        "station with aid %d leaves (refcnt %u)",
        IEEE80211_NODE_AID(ni), ieee80211_node_refcnt(ni));

	/* Multicast enhancement: If the entry with the node's address exists in 
	 * the snoop table, it should be removed.
	 */
#ifdef IEEE80211_MCAST_ENHANCEMENT
	if(vap->iv_me_ops && vap->iv_me_ops->ieee80211_me_cleanup) {
		vap->iv_me_ops->ieee80211_me_cleanup(ni);
	}
#endif	
#ifdef ATH_SUPPORT_IQUE
	ieee80211_hbr_delentry(vap, ni->ni_macaddr);
#endif	
	/*
    * If node wasn't previously associated all
    * we need to do is reclaim the reference.
    */
    /* XXX ibss mode bypasses 11g and notification */
    if (ni->ni_associd == 0)
        goto done;
    /*
    * Tell the authenticator the station is leaving.
    * Note that we must do this before yanking the
    * association id as the authenticator uses the
    * associd to locate it's state block.
    */
    if (vap->iv_auth->ia_node_leave != NULL)
        vap->iv_auth->ia_node_leave(ni);

    IEEE80211_LOCK(ic);
    IEEE80211_AID_CLR(vap, ni->ni_associd);
    ni->ni_associd = 0;
    vap->iv_sta_assoc--;
    ic->ic_sta_assoc--;

    if ((ni->ni_flags & IEEE80211_NODE_HT) && 
        (ni->ni_ath_flags & IEEE80211_NODE_40_INTOLERANT)) {
            ieee80211_change_cw(ic, 0);
    }

#ifdef ATH_SUPERG_XR
    if(ni->ni_vap->iv_flags & IEEE80211_F_XR)
        ic->ic_xr_sta_assoc--;
#endif
    if (IEEE80211_ATH_CAP(vap, ni, IEEE80211_ATHC_TURBOP))
        ic->ic_dt_sta_assoc--;

    if (IEEE80211_IS_CHAN_ANYG(ic->ic_bsschan) ||
        IEEE80211_IS_CHAN_11NG(ic->ic_bsschan))
        ieee80211_node_leave_11g(ni);

    ieee80211_ht_prot(ic);
    IEEE80211_UNLOCK(ic);
    /*
     * Cleanup station state.  In particular clear various
     * state that might otherwise be reused if the node
     * is reused before the reference count goes to zero
     * (and memory is reclaimed).
     */
    ieee80211_sta_leave(ni, 0);
done:
    /*
    * Remove the node from any table it's recorded in and
    * drop the caller's reference.  Removal from the table
    * is important to insure the node is not reprocessed
    * for inactivity.
    */
    if (nt != NULL)
    {
        IEEE80211_NODE_LOCK_BH(nt);
        node_reclaim(nt, ni);
        ieee80211_remove_wds_addr(nt,ni->ni_macaddr);
        ieee80211_del_wds_node(nt,ni);
        IEEE80211_NODE_UNLOCK_BH(nt);	
    }
    else
        ieee80211_free_node(ni);
}
EXPORT_SYMBOL_C(ieee80211_node_leave);

u_int8_t
ieee80211_getrssi(struct ieee80211com *ic, int chain, u_int32_t flags)
{
#define	NZ(x)	((x) == 0 ? 1 : (x))
    struct ieee80211_node_table *nt = &ic->ic_sta;
    struct ieee80211vap *vap;
    u_int32_t rssi_samples, rssi_total;
    struct ieee80211_node *ni;

    rssi_total = 0;
    rssi_samples = 0;
    switch (ic->ic_opmode)
    {
    case IEEE80211_M_IBSS:		/* average of all ibss neighbors */
        /* XXX locking */
        TAILQ_FOREACH(ni, &nt->nt_node, ni_list)
            if (ni->ni_capinfo & IEEE80211_CAPINFO_IBSS)
        {
            rssi_samples++;
            rssi_total += ic->ic_node_getrssi(ni,chain,flags);
        }
        break;
    case IEEE80211_M_AHDEMO:	/* average of all neighbors */
        /* XXX locking */
        TAILQ_FOREACH(ni, &nt->nt_node, ni_list) {
            rssi_samples++;
            rssi_total += ic->ic_node_getrssi(ni,chain,flags);
        }
        break;
    case IEEE80211_M_HOSTAP:	/* average of all associated stations */
        /* XXX locking */
        TAILQ_FOREACH(ni, &nt->nt_node, ni_list)
            if (IEEE80211_AID(ni->ni_associd) != 0)
        {
            rssi_samples++;
            rssi_total += ic->ic_node_getrssi(ni,chain,flags);
        }
        break;
    case IEEE80211_M_MONITOR:	/* XXX */
    case IEEE80211_M_STA:		/* use stats from associated ap */
    default:
        TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next)
            if (vap->iv_bss != NULL)
        {
            rssi_samples++;
            rssi_total += ic->ic_node_getrssi(vap->iv_bss,chain,flags);
        }
        break;
    }
    return rssi_total / NZ(rssi_samples);
#undef NZ
}
EXPORT_SYMBOL_C(ieee80211_getrssi);

void ieee80211_node_reset(struct ieee80211_node *ni, struct ieee80211vap *vap)
{
    if (ni != NULL)
    {
        struct ieee80211_node_table *nt = ni->ni_table;
        if(!nt) nt=&vap->iv_ic->ic_sta;
        IEEE80211_ADDR_COPY(ni->ni_bssid, vap->iv_bss->ni_bssid);
        ni->ni_prev_vap = ni->ni_vap;
        ni->ni_vap = vap;
        ni->ni_ic = vap->iv_ic;
        /* 
        * if node not found in the node table
        * add it to the node table .
        */
        if(nt && ieee80211_find_node(nt, ni->ni_macaddr) != ni)
        {
            int hash = IEEE80211_NODE_HASH(ni->ni_macaddr);
            IEEE80211_NODE_LOCK_BH(nt);
            TAILQ_INSERT_TAIL(&nt->nt_node, ni, ni_list);
            LIST_INSERT_HEAD(&nt->nt_hash[hash], ni, ni_hash);
            ni->ni_table=nt;
            IEEE80211_NODE_UNLOCK_BH(nt);
        }

    }
}

/* Due to OWL specific HW bug: Deny aggregation if
* we're Owl in WDS mode and
* the remote node hasn't sent us an IE indicating they're Atheros Owl or later
* and we're a WDS client or we're WDS AP and the remote node is discovered
* to be a WDS client.
* Disablement of this is controlled by toggling IEEE80211_C_WDS_AUTODETECT
* via "iwpriv athN wdsdetect 0".
*/
int ieee80211_node_wdswar_isaggrdeny(struct ieee80211_node *ni)
{
    return (((ni->ni_vap->iv_flags_ext & IEEE80211_FEXT_WDS) &&
         (ni->ni_vap->iv_flags_ext & IEEE80211_C_WDS_AUTODETECT) &&
         (ni->ni_vap->iv_ic->ic_ath_extcap & IEEE80211_ATHEC_OWLWDSWAR) &&
         !(ni->ni_flags & IEEE80211_NODE_ATH) &&
         ((ni->ni_vap->iv_opmode & IEEE80211_M_STA) ||
          ((ni->ni_vap->iv_opmode & IEEE80211_M_HOSTAP) &&
           (ni->ni_flags & IEEE80211_NODE_WDS)))) != 0 );

}
EXPORT_SYMBOL_C(ieee80211_node_wdswar_isaggrdeny);

/* Due to OWL specific HW bug, send a DELBA to remote node when we detect
 * that they're a WDS link potentially sending aggregates to us.
 * We do this if we're Owl in WDS mode and
 * the remote node hasn't sent us an IE indicating they're Atheros Owl or later
 * and we're a WDS AP and the remote node is discovered to be a WDS client.
 * Disablement of this is controlled by toggling IEEE80211_C_WDS_AUTODETECT
 * via "iwpriv athN wdsdetect 0".
 */
int ieee80211_node_wdswar_issenddelba(struct ieee80211_node *ni)
{
    return (((ni->ni_vap->iv_flags_ext & IEEE80211_FEXT_WDS) &&
               (ni->ni_vap->iv_flags_ext & IEEE80211_C_WDS_AUTODETECT) &&
               (ni->ni_ic->ic_ath_extcap & IEEE80211_ATHEC_OWLWDSWAR) &&
               !(ni->ni_flags & IEEE80211_NODE_ATH) &&
               (ni->ni_vap->iv_opmode & IEEE80211_M_HOSTAP)) != 0 );
}
EXPORT_SYMBOL_C(ieee80211_node_wdswar_issenddelba);

/* 
 * Create Adhoc bssid using NDIS logic 
 */
void
ieee80211_create_adhocbssid(struct ieee80211vap *vap, u_int8_t *ni_bssid)
{

	union {
       	    u_int8_t tmac[IEEE80211_ADDR_LEN];
            u_int32_t     tstamp;
   	} randVal;
   	u_int8_t i;

   randVal.tstamp = jiffies / HZ * 1000;
   for (i = 0; i < IEEE80211_ADDR_LEN; i++) {
       ni_bssid[i] = randVal.tmac[i] ^ vap->iv_myaddr[i];
   }
   ni_bssid[0] = (ni_bssid[0] & 0xfc) | 0x02; 
}

#ifdef NODE_FREE_DEBUG
void
ieee80211_add_trace(struct ieee80211_node *ni, char *funcp, char *descp, u_int64_t value)
{
    unsigned long tmp_val=0;

    spin_lock_irqsave(&ni->buff_lock, tmp_val);
    if (ni->node_trace == NULL)
        return;
    memset(ni->node_trace[ni->nix].funcp, 0, MAX_FUNC_LEN);
    memset(ni->node_trace[ni->nix].descp, 0, MAX_DESC_LEN);
    strncpy(ni->node_trace[ni->nix].funcp, funcp, MIN(strlen(funcp), MAX_FUNC_LEN));
    strncpy(ni->node_trace[ni->nix].descp, descp, MIN(strlen(descp), MAX_DESC_LEN));
    memcpy(ni->node_trace[ni->nix].macaddrp, ni->ni_macaddr, IEEE80211_ADDR_LEN);
    ni->node_trace[ni->nix].value = value;
    ni->node_trace[ni->nix].valid = 1;
    ni->nix = (ni->nix+1)%MAX_TRACE_SIZE;
    spin_unlock_irqrestore(&ni->buff_lock, tmp_val);
}
EXPORT_SYMBOL_C(ieee80211_add_trace);

void
ieee80211_print_trace(struct ieee80211_node *ni)
{
    int i, j;
    unsigned long tmp_val=0;

    spin_lock_irqsave(&ni->buff_lock, tmp_val);
    if (ni->node_trace == NULL)
        return;
    for (i=ni->nix, j=0; j<MAX_TRACE_SIZE; j++, i++) {
        i=i%MAX_TRACE_SIZE;
        if (ni->node_trace[i].valid) {
            ni->node_trace[i].funcp[MAX_FUNC_LEN-1]='\0';
            ni->node_trace[i].descp[MAX_DESC_LEN-1]='\0';
            printk("[%d]\tFunction\t%s\n", j, ni->node_trace[i].funcp);
            printk("[%d]\tMacAddr\t%s\n", j,
                   ether_sprintf(ni->node_trace[i].macaddrp));
            printk("[%d]\tDescp\t\t%s\n", j, ni->node_trace[i].descp);
            printk("[%d]\tValue\t\t%llu(0x%llx)\n", j, ni->node_trace[i].value,
                   ni->node_trace[i].value);
        }
    }
    spin_unlock_irqrestore(&ni->buff_lock, tmp_val);
}
EXPORT_SYMBOL_C(ieee80211_print_trace);
#endif
