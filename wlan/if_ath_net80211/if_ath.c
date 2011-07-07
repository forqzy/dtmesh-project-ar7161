/*****************************************************************************/
/* \file if_ath.c
** \brief Shim Layer implementation between ATH and MAC layers
**
**  This provides the "ic" object support for the shim interface between the
**  LMAC layers and the NET80211 UMAC.
**
** Copyright (c) 2009, Atheros Communications Inc.
**
** Permission to use, copy, modify, and/or distribute this software for any
** purpose with or without fee is hereby granted, provided that the above
** copyright notice and this permission notice appear in all copies.
**
** THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
** WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
** MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
** ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
** WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
** ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
** OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/

#include "if_athvar.h"
#include "if_ath_cwm.h"
#include "if_ath_ff.h"
#include "if_ath_amsdu.h"
#include "if_ethersubr.h"
/*
 * Mapping between WIRELESS_MODE_XXX to IEEE80211_MODE_XXX
 */
static enum ieee80211_phymode
ath_mode_map[WIRELESS_MODE_MAX] = {
    IEEE80211_MODE_11A,
    IEEE80211_MODE_11B,
    IEEE80211_MODE_11G,
    IEEE80211_MODE_TURBO_A,
    IEEE80211_MODE_TURBO_G,
    IEEE80211_MODE_11NA_HT20,
    IEEE80211_MODE_11NG_HT20,
    IEEE80211_MODE_11NA_HT40PLUS,
    IEEE80211_MODE_11NA_HT40MINUS,
    IEEE80211_MODE_11NG_HT40PLUS,
    IEEE80211_MODE_11NG_HT40MINUS,
    IEEE80211_MODE_MAX          /* XXX: for XR */
};

int ath_newstate(struct ieee80211vap *vap, enum ieee80211_state nstate, int arg);
static void ath_net80211_rate_node_update(ieee80211_handle_t ieee, ieee80211_node_t node, int isnew);
static int ath_key_alloc(struct ieee80211vap *vap, struct ieee80211_key *k);
static int ath_key_delete(struct ieee80211vap *vap, const struct ieee80211_key *k,
                          struct ieee80211_node *ninfo);
static int ath_key_set(struct ieee80211vap *vap, const struct ieee80211_key *k,
                       const u_int8_t peermac[IEEE80211_ADDR_LEN]);
static void ath_key_update_begin(struct ieee80211vap *vap);
static void ath_key_update_end(struct ieee80211vap *vap);
static void ath_update_ps_mode(struct ieee80211vap *vap);
static void ath_net80211_set_config(struct ieee80211vap* vap);
static void ath_setTxPowerLimit(struct ieee80211com *ic, u_int32_t limit, u_int16_t tpcInDb);
static u_int8_t ath_net80211_get_common_power(struct ieee80211com *ic, struct ieee80211_channel *chan);
#ifdef ATH_CCX
static int  ath_getrmcounters(struct ieee80211com *ic, struct ath_mib_cycle_cnts *pCnts);
static void ath_clearrmcounters(struct ieee80211com *ic);
static int  ath_updatermcounters(struct ieee80211com *ic, struct ath_mib_mac_stats* pStats);
static int  ath_getmibmaccounters(struct ieee80211com *ic, struct ath_mib_mac_stats* pStats);
static u_int32_t ath_getTSF32(struct ieee80211com *ic);
static u_int64_t ath_getTSF64(struct ieee80211com *ic);
static void ath_setReceiveFilter(struct ieee80211com *ic);
static int ath_getMfgSerNum(struct ieee80211com *ic, u_int8_t *pSrn);
static int ath_net80211_get_chanData(struct ieee80211com *ic, struct ieee80211_channel *pChan, struct ath_chan_data *pData);
static u_int32_t ath_net80211_get_curRSSI(struct ieee80211com *ic);
#endif
static u_int16_t ath_net80211_find_countrycode(struct ieee80211com *ic, char* isoName);
#ifdef ATH_SWRETRY
static void ath_setup_keycacheslot(struct ieee80211_node *ni);
#endif
#ifdef ATH_SUPPORT_IQUE
void ath_net80211_hbr_settrigger(ieee80211_node_t node, int state);
int ath_net80211_hbr_getstate(ieee80211_node_t node);
#endif
void ath_net80211_printreg(struct ieee80211com *ic, u_int32_t printctrl);
u_int32_t ath_net80211_wpsPushButton(struct ieee80211com *ic);
u_int32_t ath_net80211_getmfpsupport(struct ieee80211com *ic);

/*---------------------
 * Support routines
 *---------------------
 */
static u_int
ath_chan2flags(struct ieee80211_channel *chan)
{
    u_int flags;
    static const u_int modeflags[] = {
        0,                   /* IEEE80211_MODE_AUTO           */
        CHANNEL_A,           /* IEEE80211_MODE_11A            */
        CHANNEL_B,           /* IEEE80211_MODE_11B            */
        CHANNEL_PUREG,       /* IEEE80211_MODE_11G            */
        0,                   /* IEEE80211_MODE_FH             */
        CHANNEL_108A,        /* IEEE80211_MODE_TURBO_A        */
        CHANNEL_108G,        /* IEEE80211_MODE_TURBO_G        */
        CHANNEL_A_HT20,      /* IEEE80211_MODE_11NA_HT20      */
        CHANNEL_G_HT20,      /* IEEE80211_MODE_11NG_HT20      */
        CHANNEL_A_HT40PLUS,  /* IEEE80211_MODE_11NA_HT40PLUS  */
        CHANNEL_A_HT40MINUS, /* IEEE80211_MODE_11NA_HT40MINUS */
        CHANNEL_G_HT40PLUS,  /* IEEE80211_MODE_11NG_HT40PLUS  */
        CHANNEL_G_HT40MINUS, /* IEEE80211_MODE_11NG_HT40MINUS */
    };

    flags = modeflags[ieee80211_chan2mode(chan)];

    if (IEEE80211_IS_CHAN_HALF(chan)) {
        flags |= CHANNEL_HALF;
    } else if (IEEE80211_IS_CHAN_QUARTER(chan)) {
        flags |= CHANNEL_QUARTER;
    }

    return flags;
}


WIRELESS_MODE
ath_ieee2wmode(enum ieee80211_phymode mode)
{
    WIRELESS_MODE wmode;
    
    for (wmode = 0; wmode < WIRELESS_MODE_MAX; wmode++) {
        if (ath_mode_map[wmode] == mode)
            break;
    }

    return wmode;
}

/* Query ATH layer for tx/rx chainmask and set in the com object via OS stack */
static void
ath_set_chainmask(struct ieee80211com *ic)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    int tx_chainmask = 0, rx_chainmask = 0;

    if (!scn->sc_ops->ath_get_config_param(scn->sc_dev, ATH_PARAM_TXCHAINMASK,
                                           &tx_chainmask))
        ieee80211com_set_tx_chainmask(ic, (u_int8_t) tx_chainmask);

    if (!scn->sc_ops->ath_get_config_param(scn->sc_dev, ATH_PARAM_RXCHAINMASK,
                                           &rx_chainmask))
        ieee80211com_set_rx_chainmask(ic, (u_int8_t) rx_chainmask);
}

/*------------------------------------------------------------
 * Callbacks for net80211 module, which will be hooked up as
 * ieee80211com vectors (ic->ic_xxx) accordingly.
 *------------------------------------------------------------
 */

static int
ath_init(struct ieee80211com *ic)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    struct ieee80211_channel *cc;
    HAL_CHANNEL hchan;
    int error = 0;

    /* stop protocol stack first */
    ieee80211_stop_running(ic);
    
    /* setup initial channel */
    cc = ieee80211com_get_curchan(ic);
    hchan.channel = cc->ic_freq;
    hchan.channelFlags = ath_chan2flags(cc);
    
    /* open ath_dev */
    error = scn->sc_ops->open(scn->sc_dev, &hchan);
    if (error)
        return error;

    /* Set tx/rx chainmask */
    ath_set_chainmask(ic);
    
    /* Initialize CWM (Channel Width Management) */
    ath_cwm_init(scn);

    /* kick start 802.11 state machine */
    ieee80211_start_running(ic);

    /* update max channel power to max regpower of current channel */
    ieee80211com_set_curchanmaxpwr(ic, cc->ic_maxregpower);

    return error;
}

static struct ieee80211vap *
ath_vap_create(struct ieee80211com *ic, const char *name, int unit,
               int opmode, int flags, const u_int8_t bssid[IEEE80211_ADDR_LEN])
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    struct ath_vap_net80211 *avn;
    struct ieee80211vap *vap, *v;
    u_int8_t myaddr[IEEE80211_ADDR_LEN];
    int id = 0, id_mask = 0;
    int nvaps = 0;
    int ic_opmode;
    int nostabeacons = 0;
    struct ath_vap_config ath_vap_config;

    /* XXX ic unlocked and race against add */

    /* do a full search to mark all the allocated vaps */
    ieee80211_enumerate_vaps(v, ic) {
        ieee80211vap_get_macaddr(v, myaddr);
     
        if((scn->sc_ops->has_capability)(scn->sc_dev,ATH_CAP_MBSSID_AGGR_SUPPORT) & UL_BIT_MASK) {
        	id_mask |= (1 << ATH_GET_VAP_ID_ULB(myaddr));
		}
		else {
            ATH_GET_VAP_ID(myaddr, ic->ic_myaddr, id);
            id_mask |= (1<<id);
		}
        nvaps++;
    }

    switch (opmode) {
    case IEEE80211_M_STA:   /* ap+sta for repeater application */
        if (scn->sc_nstavaps != 0)  /* only one sta regardless */
            return NULL;
        if ((nvaps != 0) && (!(flags & IEEE80211_NO_STABEACONS)))
            return NULL;   /* If using station beacons, must first up */
        if (flags & IEEE80211_NO_STABEACONS) {
            nostabeacons = 1;
            ic_opmode = IEEE80211_M_HOSTAP;	/* Run with chip in AP mode */
        } else
            ic_opmode = opmode;
        break;
    case IEEE80211_M_IBSS:
    case IEEE80211_M_MONITOR:
        if (nvaps != 0)     /* only one */
            return NULL;
        ic_opmode = opmode;
        break;
    case IEEE80211_M_HOSTAP:
    case IEEE80211_M_WDS:
        /* permit multiple ap's and/or wds links */
        /* XXX sta+ap for repeater/bridge application */
        if ((nvaps != 0) && (ic->ic_opmode == IEEE80211_M_STA))
            return NULL;

        /*
         * XXX Not sure if this is correct when operating only
         * with WDS links.
         */
        ic_opmode = IEEE80211_M_HOSTAP;
        break;
    default:
        return NULL;
    }

    /* calculate an interface id */
    if ((flags & IEEE80211_CLONE_BSSID) &&
        nvaps != 0 && opmode != IEEE80211_M_WDS &&
        (scn->sc_ops->has_capability)(scn->sc_dev, ATH_CAP_BSSIDMASK)) {
        /*
         * Hardware supports the bssid mask and a unique bssid was
         * requested.  Assign a new mac address and expand our bssid
         * mask to cover the active virtual ap's with distinct
         * addresses.
         */
        KASSERT(nvaps <= ATH_BCBUF, ("too many virtual ap's: %d", nvaps));

        for (id = 0; id < ATH_BCBUF; id++) {
            /* get the first available slot */
            if ((id_mask & (1 << id)) == 0)
                break;
        }
    }

    /* create the corresponding VAP */
    avn = (struct ath_vap_net80211 *)OS_ALLOC_VAP(scn->sc_osdev,
                                                    sizeof(struct ath_vap_net80211));
    if (avn == NULL) {
        printk("Can't allocate memory for ath_vap.\n");
        return NULL;
    }
    
    avn->av_sc = scn;
    avn->av_if_id = id;

    vap = &avn->av_vap;

    /* add an interface in ath_dev */
    if (scn->sc_ops->add_interface(scn->sc_dev, id, vap, ic_opmode, opmode, nostabeacons)) {
        printk("Unable to add an interface for ath_dev.\n");
        OS_FREE_VAP(avn);
        return NULL;
    }

    ieee80211_vap_setup(ic, vap, name, unit, opmode, flags, bssid);

    /* override default ath_dev VAP configuration with IEEE VAP configuration */
    OS_MEMZERO(&ath_vap_config, sizeof(ath_vap_config));
    ath_vap_config.av_fixed_rateset = vap->iv_fixed_rateset;
    ath_vap_config.av_fixed_retryset = vap->iv_fixed_retryset; 
    scn->sc_ops->config_interface(scn->sc_dev, id, &ath_vap_config);

    /* set up MAC address */
    ieee80211vap_get_macaddr(vap, myaddr);
	if((scn->sc_ops->has_capability)(scn->sc_dev,ATH_CAP_MBSSID_AGGR_SUPPORT) & UL_BIT_MASK) {
		ATH_SET_VAP_BSSID_ULB(myaddr, id);
	}
	else {
        ATH_SET_VAP_BSSID(myaddr, ic->ic_myaddr, id);
	}
    ieee80211vap_set_macaddr(vap, myaddr);

    /* override with driver methods */
    avn->av_newstate = vap->iv_newstate;
    vap->iv_newstate = ath_newstate;
    vap->iv_key_alloc = ath_key_alloc;
    vap->iv_key_delete = ath_key_delete;
    vap->iv_key_set = ath_key_set;
    vap->iv_key_update_begin = ath_key_update_begin;
    vap->iv_key_update_end = ath_key_update_end;

    vap->iv_update_ps_mode = ath_update_ps_mode;

    /* complete setup */
    (void) ieee80211_vap_attach(vap,
                                (void *)ieee80211_media_change,
                                (void *)ieee80211_media_status);
    ic->ic_opmode = ic_opmode;
    if (opmode == IEEE80211_M_STA)
        scn->sc_nstavaps++;

    if((vap->iv_unit + 1) > 
       ((scn->sc_ops->has_capability)(scn->sc_dev,ATH_CAP_MBSSID_AGGR_SUPPORT) & ~(UL_BIT_MASK))) {
       vap->iv_flags_ext &= ~IEEE80211_FEXT_AMPDU;
    }
    
    return vap;
}

static void
ath_vap_delete(struct ieee80211vap *vap)
{
    struct ath_vap_net80211 *avn = ATH_VAP_NET80211(vap);
    struct ath_softc_net80211 *scn = avn->av_sc;
    int ret;

    KASSERT(vap->iv_state == IEEE80211_S_INIT, ("vap not stopped"));

    /* remove the interface from ath_dev */
    ret = scn->sc_ops->remove_interface(scn->sc_dev, avn->av_if_id);
    KASSERT(ret == 0, ("invalid interface id"));

    if (ieee80211vap_get_opmode(vap) == IEEE80211_M_STA)
        scn->sc_nstavaps--;

    /* detach VAP from the procotol stack */
    ieee80211_vap_detach(vap);

    OS_FREE_VAP(avn);
}

static void
ath_net80211_set_config(struct ieee80211vap* vap)
{
    struct ath_vap_net80211 *avn = ATH_VAP_NET80211(vap);
    struct ath_softc_net80211 *scn = avn->av_sc;
    struct ath_vap_config ath_vap_config;

    /* override default ath_dev VAP configuration with IEEE VAP configuration */
    OS_MEMZERO(&ath_vap_config, sizeof(ath_vap_config));
    ath_vap_config.av_fixed_rateset = vap->iv_fixed_rateset;
    ath_vap_config.av_fixed_retryset = vap->iv_fixed_retryset; 
    scn->sc_ops->config_interface(scn->sc_dev, avn->av_if_id, &ath_vap_config);
}

static struct ieee80211_node *
ath_net80211_node_alloc(struct ieee80211_node_table *nt, struct ieee80211vap *vap)
{
    struct ieee80211com *ic = vap->iv_ic;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    struct ath_vap_net80211 *avn = ATH_VAP_NET80211(vap);
    struct ath_node_net80211 *anode;
    int i;

    anode = (struct ath_node_net80211 *)OS_MALLOC(scn->sc_osdev,
                                                  sizeof(struct ath_node_net80211),
                                                  GFP_ATOMIC);
    if (anode == NULL)
        return NULL;

    OS_MEMZERO(anode, sizeof(struct ath_node_net80211));
    anode->an_avgbrssi  = ATH_RSSI_DUMMY_MARKER;
    anode->an_avgrssi   = ATH_RSSI_DUMMY_MARKER;
    anode->an_avgtxrssi = ATH_RSSI_DUMMY_MARKER;
    anode->an_lasttxrate = ATH_RATE_DUMMY_MARKER;
    anode->an_lastrxrate = ATH_RATE_DUMMY_MARKER;
    anode->an_avgrxrate = ATH_RATE_DUMMY_MARKER;
    for (i = 0; i < ATH_MAX_ANTENNA; ++i) {
        anode->an_avgtxchainrssi[i]    = ATH_RATE_DUMMY_MARKER;
        anode->an_avgtxchainrssiext[i] = ATH_RATE_DUMMY_MARKER;
        anode->an_avgchainrssi[i]      = ATH_RATE_DUMMY_MARKER;
        anode->an_avgchainrssiext[i]   = ATH_RATE_DUMMY_MARKER;
    }

    /* attach a node in ath_dev module */
    anode->an_sta = scn->sc_ops->alloc_node(scn->sc_dev, avn->av_if_id, anode);
    if (anode->an_sta == NULL) {
        OS_FREE(anode);
        return NULL;
    }
#ifdef ATH_SUPERG_FF
    /* attach fast frame module */
    ath_ff_attach(scn, anode);
#endif

#ifdef ATH_AMSDU
    ath_amsdu_node_attach(scn, anode);
#endif

    anode->an_node.ni_vap = vap;
    return &anode->an_node;
}

static void
ath_force_ppm_enable (void *arg, struct ieee80211_node *ni)
{
    struct ieee80211com          *ic  = arg;
    struct ath_softc_net80211    *scn = ATH_SOFTC_NET80211(ic);

    scn->sc_ops->force_ppm_notify(scn->sc_dev, ATH_FORCE_PPM_ENABLE, ni->ni_macaddr);
}

static void
ath_net80211_node_free(struct ieee80211_node *ni)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    ath_node_t an = ATH_NODE_NET80211(ni)->an_sta;
#ifdef NODE_FREE_DEBUG
    ieee80211_trace_t *trp = ni->node_trace;
#endif

    /*
     * If AP mode, enable ForcePPM if only one Station is connected, or
     * disable it otherwise.
     */
    if (ieee80211vap_get_opmode(ni->ni_vap) == IEEE80211_M_HOSTAP) {
        if (ieee80211com_can_enable_force_ppm(ic)) {
            ieee80211_iterate_node_nolock(ic, ath_force_ppm_enable, ic);
        }
        else {
            scn->sc_ops->force_ppm_notify(scn->sc_dev, ATH_FORCE_PPM_DISABLE, NULL);
        }
    }

#ifdef ATH_SUPERG_FF
    ath_ff_detach(ATH_NODE_NET80211(ni));
#endif

#ifdef ATH_AMSDU
    ath_amsdu_node_detach(scn, ATH_NODE_NET80211(ni));
#endif

    scn->sc_node_free(ni);
    scn->sc_ops->free_node(scn->sc_dev, an);
#ifdef NODE_FREE_DEBUG
    if (trp)
        kfree(trp);
#endif
}

static void
ath_net80211_node_cleanup(struct ieee80211_node *ni)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    struct ath_node_net80211 *anode = (struct ath_node_net80211 *)ni;

    scn->sc_ops->cleanup_node(scn->sc_dev, anode->an_sta);
#ifdef ATH_SWRETRY
    scn->sc_ops->set_swretrystate(scn->sc_dev, anode->an_sta, AH_FALSE);
#endif
    scn->sc_node_cleanup(ni);
}

static int8_t
ath_net80211_node_getrssi(const struct ieee80211_node *ni,int8_t chain, u_int8_t flags )
{
    struct ath_node_net80211 *anode = (struct ath_node_net80211 *)ni;
    int32_t avgrssi;
    int32_t rssi;


    /*
     * When only one frame is received there will be no state in
     * avgrssi so fallback on the value recorded by the 802.11 layer.
     */

    avgrssi = ATH_RSSI_DUMMY_MARKER;

    if (flags & IEEE80211_RSSI_BEACON) {
        avgrssi = anode->an_avgbrssi;
    } else if (flags & IEEE80211_RSSI_RX) {
        if (chain == -1) {
            avgrssi = anode->an_avgrssi;
        } else {
            if (!(flags & IEEE80211_RSSI_EXTCHAN)) {
                avgrssi = anode->an_avgchainrssi[chain];
            } else {
                avgrssi = anode->an_avgchainrssiext[chain];
            }
        }
    } else if (flags & IEEE80211_RSSI_TX) {
        if (chain == -1) {
            avgrssi = anode->an_avgtxrssi;
        } else {
            if (!(flags & IEEE80211_RSSI_EXTCHAN)) {
                avgrssi = anode->an_avgtxchainrssi[chain];
            } else {
                avgrssi = anode->an_avgtxchainrssiext[chain];
            }
        }
    }

    if (avgrssi != ATH_RSSI_DUMMY_MARKER)
        rssi = ATH_EP_RND(avgrssi, HAL_RSSI_EP_MULTIPLIER);
    else
        rssi =  -1; 

    return rssi;
}

static int
ath_net80211_reset(struct ieee80211com *ic)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    int error;

    scn->sc_ops->reset_start(scn->sc_dev, 0);
    error = scn->sc_ops->reset(scn->sc_dev);
    scn->sc_ops->reset_end(scn->sc_dev, 0);

    //netif_wake_queue(dev);        /* restart xmit */

    return error;
}

static int
ath_net80211_set_channel(struct ieee80211com *ic)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    struct ieee80211_channel *chan;
    HAL_CHANNEL hchan;

    /*
     * Convert to a HAL channel description with
     * the flags constrained to reflect the current
     * operating mode.
     */
    chan = ieee80211com_get_curchan(ic);
    hchan.channel = chan->ic_freq;
    hchan.channelFlags = ath_chan2flags(chan);
    KASSERT(hchan.channel != 0,
            ("bogus channel %u/0x%x", hchan.channel, hchan.channelFlags));

    /* set h/w channel */
    scn->sc_ops->set_channel(scn->sc_dev, &hchan);
    
    /* update max channel power to max regpower of current channel */
    ieee80211com_set_curchanmaxpwr(ic, chan->ic_maxregpower);

    /*
     * If we are returning to our bss channel then mark state
     * so the next recv'd beacon's tsf will be used to sync the
     * beacon timers.  Note that since we only hear beacons in
     * sta/ibss mode this has no effect in other operating modes.
     */
    if (!scn->sc_isscan &&
        (ieee80211com_get_curchan(ic) == ieee80211com_get_bsschan(ic)) &&
        ieee80211com_get_opmode(ic) != IEEE80211_M_HOSTAP) {

        struct ieee80211vap *vap;

        /* XXX: we know there's only ONE vap for sta/ibss mode. */
        vap = ieee80211com_first_vap(ic);
        
        if (ieee80211vap_get_state(vap) == IEEE80211_S_RUN) {
            /*
             * Always configure the beacon.
             * In ad-hoc, we may be the only peer in the network.
             * In infrastructure, we need to detect beacon miss
             * if the AP goes away while we are scanning.
             */
            scn->sc_ops->sync_beacon(scn->sc_dev, 0);

            if (ieee80211vap_get_opmode(vap) == IEEE80211_M_IBSS) {
                /*
                 * if tsf is 0. we are alone.
                 * no need to sync tsf from beacons.
                 */
                if (ieee80211node_get_tsf(ieee80211vap_get_bssnode(vap)) != 0) 
                    scn->sc_syncbeacon = 1;
            } else
                scn->sc_syncbeacon = 1;

            /* Notify CWM */
            DPRINTF(scn, ATH_DEBUG_CWM, "%s\n", __func__);

            TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) {
                ath_cwm_newstate(vap, IEEE80211_S_RUN);

                /* Resume ForcePPM operation as we return to home channel */
                if (ieee80211vap_get_opmode(vap) == IEEE80211_M_STA) {
                    scn->sc_ops->force_ppm_notify(scn->sc_dev, ATH_FORCE_PPM_RESUME, NULL);
                }
            }
        }
    }

    return 0;
}

static void
ath_net80211_newassoc(struct ieee80211_node *ni, int isnew)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    struct ath_node_net80211 *an = (struct ath_node_net80211 *)ni;

    ath_net80211_rate_node_update(ic, ni, isnew);
    scn->sc_ops->new_assoc(scn->sc_dev, an->an_sta, isnew, ((ni->ni_flags & IEEE80211_NODE_UAPSD)? 1: 0));

    /*
     * If AP mode, enable ForcePPM if only one Station is connected, or 
     * disable it otherwise. ForcePPM applies only to 2GHz channels.
     */
    if (ieee80211vap_get_opmode(ni->ni_vap) == IEEE80211_M_HOSTAP) {
        if (IEEE80211_IS_CHAN_2GHZ(ni->ni_chan)) {
            enum ath_force_ppm_event_t    event = ieee80211com_can_enable_force_ppm(ic) ?
                ATH_FORCE_PPM_ENABLE : ATH_FORCE_PPM_DISABLE;

            scn->sc_ops->force_ppm_notify(scn->sc_dev, event, ni->ni_macaddr);
        }
    }

#ifdef ATH_SUPERG_COMP
    if (!(ieee80211vap_has_athcap(vap, IEEE80211_ATHC_COMP) &&
          ieee80211node_has_athflag(ni, IEEE80211_NODE_COMP))) {
        ieee80211node_clear_athflag(ni, IEEE80211_NODE_COMP);
    }

    /* disable compression for TKIP */
    if ((ni->ni_ath_flags & IEEE80211_NODE_COMP) &&
        (ni->ni_wpa_ie != NULL) &&
        (ni->ni_rsn.rsn_ucastcipher == IEEE80211_CIPHER_TKIP)) {
        ni->ni_ath_flags &= ~IEEE80211_NODE_COMP;
    }
#endif
}

int
ath_newstate(struct ieee80211vap *vap, enum ieee80211_state nstate, int arg)
{
    struct ieee80211com *ic = vap->iv_ic;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    struct ath_vap_net80211 *avn = ATH_VAP_NET80211(vap);
    struct ieee80211_node *ni = vap->iv_bss;
    u_int flags = 0;
    int error = 0;
    int aid = 0;
    struct ieee80211vap *tmpvap;
    enum ieee80211_opmode opmode = ieee80211vap_get_opmode(vap);

    DPRINTF(scn, ATH_DEBUG_STATE, "%s: %s -> %s\n", __func__,
            ieee80211_state_name[ieee80211vap_get_state(vap)],
            ieee80211_state_name[nstate]);

    //netif_stop_queue(dev);			/* before we do anything else */

    /*
     * Notify CWM.
     */
    ath_cwm_newstate(vap, nstate);
  
#ifdef ATH_SWRETRY
    scn->sc_ops->set_swretrystate(scn->sc_dev, ATH_NODE_NET80211(ni)->an_sta, AH_FALSE);
#endif

    switch (nstate) {
    case IEEE80211_S_INIT:
        /*
         * if there is no vap left in RUN state, turn off hardware
         */
        ieee80211_enumerate_vaps(tmpvap, ic) {
            if (tmpvap != vap && ieee80211vap_get_state(tmpvap) == IEEE80211_S_RUN) {
                break;
            }
        }
        if (!tmpvap)
            flags |= ATH_IF_HW_OFF;

        error = scn->sc_ops->down(scn->sc_dev, avn->av_if_id, flags);
        break;
        
    case IEEE80211_S_SCAN:
        error = scn->sc_ops->listen(scn->sc_dev, avn->av_if_id);
        break;
        
    case IEEE80211_S_JOIN:
        if (opmode != IEEE80211_M_STA && opmode != IEEE80211_M_IBSS) {
            DPRINTF(scn, ATH_DEBUG_STATE,
                    "%s: JOIN state is only for STA/IBSS mode\n",
                    __func__);
            error = -EINVAL;
            goto bad;
        }
        if (vap->iv_flags_ext & IEEE80211_FEXT_VAP_IND) {
            flags |= ATH_IF_VAP_IND;
        }
        if (ieee80211node_has_flag(ni, IEEE80211_NODE_HT))
            flags |= ATH_IF_HT;

        error = scn->sc_ops->join(scn->sc_dev, avn->av_if_id, ni->ni_bssid, flags);
        break;

    case IEEE80211_S_RUN:
        switch (opmode) {
        case IEEE80211_M_HOSTAP:
        case IEEE80211_M_IBSS:
#ifndef ATHR_RNWF
            /* Set default key index for static wep case */
            ni->ni_ath_defkeyindex = IEEE80211_INVAL_DEFKEY;
            if (((vap->iv_flags & IEEE80211_F_WPA) == 0) &&
                (ni->ni_authmode != IEEE80211_AUTH_8021X) &&
                (vap->iv_def_txkey != IEEE80211_KEYIX_NONE)) {
                ni->ni_ath_defkeyindex = vap->iv_def_txkey;
            }
#endif

            if (ieee80211vap_has_athcap(vap, IEEE80211_ATHC_TURBOP))
                flags |= ATH_IF_DTURBO;
             
            if ( ieee80211vap_has_flag(vap, IEEE80211_F_PRIVACY) )
                flags |= ATH_IF_PRIVACY;

            /*
             * if it is the first AP VAP moving to RUN state then beacon
             * needs to be reconfigured.
             */
            ieee80211_enumerate_vaps(tmpvap, ic) {
                if ((tmpvap != vap) && (ieee80211vap_get_state(tmpvap) == IEEE80211_S_RUN)
                    && (ieee80211vap_get_opmode(tmpvap) == IEEE80211_M_HOSTAP)) {
                    break;
                }
            }
            if(!tmpvap) {
                flags |= ATH_IF_BEACON_ENABLE;

                if (ieee80211vap_get_state(vap) != IEEE80211_S_RUN)
                    flags |= ATH_IF_HW_ON;
                
                /*
                 * if tsf is 0. we are starting a new ad-hoc network.
                 * no need to wait and sync for beacons.
                 */
                if (ieee80211node_get_tsf(ni) != 0) {
                    scn->sc_syncbeacon = 1;
                    flags |= ATH_IF_BEACON_SYNC;
                } else {
                    /*  
                     *  Fix bug 27870.
                     *  When system wakes up from power save mode, we don't 
                     *  know if the peers have left the ad hoc network or not,
                     *  so we have to configure beacon (& ~ATH_IF_BEACON_SYNC)
                     *  and also synchronize to older beacons (sc_syncbeacon
                     *  = 1).
                     *  
                     *  The merge function should take care of it, but during
                     *  resume, sometimes the tsf in rx_status shows the
                     *  synchorized value, so merge routine does not get
                     *  called. It is safer we turn on sc_syncbeacon now.
                     *
                     *  There is no impact to synchronize twice, so just enable
                     *  sc_syncbeacon as long as it is ad hoc mode.
                     */
                    if (opmode == IEEE80211_M_IBSS)
                        scn->sc_syncbeacon = 1;
                    else
                        scn->sc_syncbeacon = 0;
                }
            }
            break;
            
        case IEEE80211_M_STA:
            aid = ni->ni_associd;
            scn->sc_syncbeacon = 1;
            flags |= ATH_IF_BEACON_SYNC; /* sync with next received beacon */

#ifdef ATH_SUPERG_COMP
            /* have we negotiated compression? */
            if (!(ieee80211vap_has_athcap(vap, IEEE80211_CAP_COMP) &&
                  ieee80211node_has_athflag(ni, IEEE80211_NODE_COMP))) {
                ieee80211node_clear_athflag(ni, IEEE80211_NODE_COMP);
            }
#endif
            if (ieee80211node_has_athflag(ni, IEEE80211_ATHC_TURBOP))
                flags |= ATH_IF_DTURBO;

            if (ieee80211node_has_flag(ni, IEEE80211_NODE_HT))
                flags |= ATH_IF_HT;

#ifdef ATH_SWRETRY
            if (scn->sc_ops->has_capability(scn->sc_dev, ATH_CAP_SWRETRY_SUPPORT)) {
                /* We need to allocate keycache slot here 
                 * only if we enable sw retry mechanism
                 */
                ath_setup_keycacheslot(ni);

                /* Enabling SW Retry mechanism only for Infrastructure 
                 * mode and only when STA associates to AP and entered into
                 * into RUN state.
                 */
                scn->sc_ops->set_swretrystate(scn->sc_dev, ATH_NODE_NET80211(ni)->an_sta, AH_TRUE);
            }
#endif
            break;
        default:
            break;
        }

        error = scn->sc_ops->up(scn->sc_dev, avn->av_if_id, ni->ni_bssid, aid, flags);
        break;
        
    default:
        break;
    }

    /*
     * If STA mode, enable ForcePPM only if we're connecting to a 2GHz channel,
     * otherwise disable it.
     */
    if ((ieee80211vap_get_state(vap) != nstate) &&
        (opmode == IEEE80211_M_STA)) {
        enum ath_force_ppm_event_t    event = ATH_FORCE_PPM_DISABLE;

        if ((nstate == IEEE80211_S_RUN) && IEEE80211_IS_CHAN_2GHZ(ni->ni_chan)) {
            event = ATH_FORCE_PPM_ENABLE;
        }
        scn->sc_ops->force_ppm_notify(scn->sc_dev, event, ieee80211node_get_bssid(vap->iv_bss));
    }

    if (!error)
        error = avn->av_newstate(vap, nstate, arg);
    
bad:
    //netif_start_queue(dev);
    //dev->watchdog_timeo = 5 * HZ;			/* set the timeout to normal */
    return error;
}

static void
ath_net80211_scan_start(struct ieee80211com *ic)
{
    struct ath_softc_net80211    *scn = ATH_SOFTC_NET80211(ic);
    struct ieee80211vap          *vap = ieee80211com_first_vap(ic);

    scn->sc_isscan = 1;
    scn->sc_syncbeacon = 0;
    scn->sc_ops->scan_start(scn->sc_dev);
    ath_cwm_scan_start(ic);

    /* Suspend ForcePPM since we are going off-channel */
    if (ieee80211vap_get_opmode(vap) == IEEE80211_M_STA) {
        scn->sc_ops->force_ppm_notify(scn->sc_dev, ATH_FORCE_PPM_SUSPEND, NULL);
    }
}

static void
ath_net80211_scan_end(struct ieee80211com *ic)
{
    struct ath_softc_net80211    *scn = ATH_SOFTC_NET80211(ic);

    scn->sc_isscan = 0;
    scn->sc_ops->scan_end(scn->sc_dev);
    ath_cwm_scan_end(ic);
}

static void
ath_net80211_led_enter_scan(struct ieee80211com *ic)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    scn->sc_ops->led_scan_start(scn->sc_dev);
}

static void
ath_net80211_led_leave_scan(struct ieee80211com *ic)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    scn->sc_ops->led_scan_end(scn->sc_dev);
}

static void
ath_beacon_update(struct ieee80211_node *ni, int rssi)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ni->ni_ic);

    /* Update beacon-related information - rssi and others */
    scn->sc_ops->update_beacon_info(scn->sc_dev,
                                    ATH_NODE_NET80211(ni)->an_avgbrssi);
    
    if (scn->sc_syncbeacon) {
        scn->sc_ops->sync_beacon(scn->sc_dev, (ATH_VAP_NET80211(ni->ni_vap))->av_if_id);
        scn->sc_syncbeacon = 0;
    }
}

static int
ath_wmm_update(struct ieee80211com *ic)
{
#define	ATH_EXPONENT_TO_VALUE(v)    ((1<<v)-1)
#define	ATH_TXOP_TO_US(v)           (v<<5)
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    int ac;
    struct wmeParams *wmep;
    HAL_TXQ_INFO qi;

    OS_MEMZERO(&qi, sizeof(HAL_TXQ_INFO));

    for (ac = 0; ac < WME_NUM_AC; ac++) {
        wmep = ieee80211_wmm_chanparams(ic, ac);

        qi.tqi_aifs = wmep->wmep_aifsn;
        qi.tqi_cwmin = ATH_EXPONENT_TO_VALUE(wmep->wmep_logcwmin);
        qi.tqi_cwmax = ATH_EXPONENT_TO_VALUE(wmep->wmep_logcwmax);
        qi.tqi_burstTime = ATH_TXOP_TO_US(wmep->wmep_txopLimit);
        /*
         * XXX Set the readyTime appropriately if used.
         */

        if (scn->sc_ops->txq_update(scn->sc_dev, scn->sc_ac2q[ac], &qi) != 0)
            return -EIO;

        if (ac == WME_AC_BE)
            scn->sc_ops->txq_update(scn->sc_dev, scn->sc_beacon_qnum, &qi);

#ifdef ATH_SUPPORT_UAPSD
        /*
         * set VO parameters in UAPSD queue
         */
        if (ac == WME_AC_VO)
            scn->sc_ops->txq_update(scn->sc_dev, scn->sc_uapsd_qnum, &qi);
#endif
    }
    return 0;
#undef ATH_TXOP_TO_US
#undef ATH_EXPONENT_TO_VALUE
}

#define AR_DEBUG
#ifdef AR_DEBUG
static void
ath_keyprint(const char *tag, u_int ix,
             const HAL_KEYVAL *hk, const u_int8_t mac[IEEE80211_ADDR_LEN])
{
    static const char *ciphers[] = {
        "WEP",
        "AES-OCB",
        "AES-CCM",
        "CKIP",
        "TKIP",
        "CLR",
    };
    int i, n;

    printk("%s: [%02u] %-7s ", tag, ix, ciphers[hk->kv_type]);
    for (i = 0, n = hk->kv_len; i < n; i++)
        printk("%02x", hk->kv_val[i]);
    if (mac) {
        printk(" mac %02x-%02x-%02x-%02x-%02x-%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    } else {
        printk(" mac 00-00-00-00-00-00");
    }
    if (hk->kv_type == HAL_CIPHER_TKIP) {
        printk(" mic ");
        for (i = 0; i < sizeof(hk->kv_mic); i++)
            printk("%02x", hk->kv_mic[i]);
        printk(" txmic ");
		for (i = 0; i < sizeof(hk->kv_txmic); i++)
			printk("%02x", hk->kv_txmic[i]);
    }
    printk("\n");
}
#endif

/*
 * Allocate one or more key cache slots for a uniacst key.  The
 * key itself is needed only to identify the cipher.  For hardware
 * TKIP with split cipher+MIC keys we allocate two key cache slot
 * pairs so that we can setup separate TX and RX MIC keys.  Note
 * that the MIC key for a TKIP key at slot i is assumed by the
 * hardware to be at slot i+64.  This limits TKIP keys to the first
 * 64 entries.
 */
static int
ath_key_alloc(struct ieee80211vap *vap, struct ieee80211_key *k)
{
    struct ieee80211com *ic = vap->iv_ic;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    int opmode;
    u_int keyix;

    if (k->wk_flags & IEEE80211_KEY_GROUP) {
        opmode = ieee80211vap_get_opmode(vap);

        switch (opmode) {
        case IEEE80211_M_STA:

            if (!((&vap->iv_nw_keys[0] <= k) &&
                  (k < &vap->iv_nw_keys[IEEE80211_WEP_NKID]))) {
                /* should not happen */
                DPRINTF(scn, ATH_DEBUG_KEYCACHE,
                        "%s: bogus group key\n", __func__);
                return IEEE80211_KEYIX_NONE;
            }
            keyix = k - vap->iv_nw_keys;
            return keyix;
            break;

        case IEEE80211_M_IBSS:
            //ASSERT(scn->sc_mcastkey);
            if ((k->wk_flags & IEEE80211_KEY_PERSTA) == 0) {

                /* 
                 * Multicast key search doesn't work on certain hardware, don't use shared key slot(0-3)
                 * for default Tx broadcast key in that case. This affects broadcast traffic reception with AES-CCMP.
                 */
               if (!(scn->sc_ops->has_capability)(scn->sc_dev, ATH_CAP_MCAST_KEYSEARCH) &&
					(k->wk_cipher->ic_cipher == IEEE80211_CIPHER_AES_CCM)) {
                    return scn->sc_ops->key_alloc_single(scn->sc_dev);
                }

                if (!((&vap->iv_nw_keys[0] <= k) &&
                      (k < &vap->iv_nw_keys[IEEE80211_WEP_NKID]))) {
                    /* should not happen */
                    DPRINTF(scn, ATH_DEBUG_KEYCACHE,
                            "%s: bogus group key\n", __func__);
                    return IEEE80211_KEYIX_NONE;
                }
                keyix = k - vap->iv_nw_keys;
                return keyix;

            } else if (!(k->wk_flags & IEEE80211_KEY_RECV)) {
                return IEEE80211_KEYIX_NONE;
            }

            if (k->wk_flags & IEEE80211_KEY_PERSTA) {
                if (k->wk_valid) {
                    return k->wk_keyix;
                }
            }
            /* fall thru to allocate a slot for _PERSTA keys */
            break;

        case IEEE80211_M_HOSTAP:
            /*
             * Group key allocation must be handled specially for
             * parts that do not support multicast key cache search
             * functionality.  For those parts the key id must match
             * the h/w key index so lookups find the right key.  On
             * parts w/ the key search facility we install the sender's
             * mac address (with the high bit set) and let the hardware
             * find the key w/o using the key id.  This is preferred as
             * it permits us to support multiple users for adhoc and/or
             * multi-station operation.
             */
            if (!scn->sc_mcastkey) {
                if (!(&vap->iv_nw_keys[0] <= k &&
                      k < &vap->iv_nw_keys[IEEE80211_WEP_NKID])) {
                    /* should not happen */
                    DPRINTF(scn, ATH_DEBUG_KEYCACHE,
                            "%s: bogus group key\n", __func__);
                    return IEEE80211_KEYIX_NONE;
                }
                keyix = k - vap->iv_nw_keys;
                /*
                 * XXX we pre-allocate the global keys so
                 * have no way to check if they've already been allocated.
                 */
                return keyix;
            }
            /* fall thru to allocate a key cache slot */
            break;
            
        default:
            return IEEE80211_KEYIX_NONE;
            break;
        }
    }


    /*
     * We allocate two pair for TKIP when using the h/w to do
     * the MIC.  For everything else, including software crypto,
     * we allocate a single entry.  Note that s/w crypto requires
     * a pass-through slot on the 5211 and 5212.  The 5210 does
     * not support pass-through cache entries and we map all
     * those requests to slot 0.
     *
     * Allocate 1 pair of keys for WEP case. Make sure the key
     * is not a shared-key.
     */
    if (k->wk_flags & IEEE80211_KEY_SWCRYPT) {
        keyix = scn->sc_ops->key_alloc_single(scn->sc_dev);
    } else if ((k->wk_cipher->ic_cipher == IEEE80211_CIPHER_TKIP) &&
               ((k->wk_flags & IEEE80211_KEY_SWMIC) == 0))
    {
        if (scn->sc_splitmic) {
            keyix = scn->sc_ops->key_alloc_2pair(scn->sc_dev);
        } else {
            keyix = scn->sc_ops->key_alloc_pair(scn->sc_dev);
        }
    } else {
        keyix = scn->sc_ops->key_alloc_single(scn->sc_dev);
    }

    // Allocate clear key slot only after the rx key slot is allocated.
    // It will ensure that key cache search for incoming frame will match
    // correct index.
    if (k->wk_flags & IEEE80211_KEY_MFP) {
        /* Allocate a clear key entry for sw encryption of mgmt frames */
        k->wk_clearkeyix = scn->sc_ops->key_alloc_single(scn->sc_dev);
    }
    return keyix;
}

/*
 * Delete an entry in the key cache allocated by ath_key_alloc.
 */
static int
ath_key_delete(struct ieee80211vap *vap, const struct ieee80211_key *k,
               struct ieee80211_node *ninfo)
{
    struct ieee80211com *ic = vap->iv_ic;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    const struct ieee80211_cipher *cip = k->wk_cipher;
    struct ieee80211_node *ni;
    u_int keyix = k->wk_keyix;
    int rxkeyoff = 0;
    int freeslot;

    DPRINTF(scn, ATH_DEBUG_KEYCACHE, "%s: delete key %u\n", __func__, keyix);

    /*
     * Don't touch keymap entries for global keys so
     * they are never considered for dynamic allocation.
     */
    freeslot = (keyix >= IEEE80211_WEP_NKID) ? 1 : 0;

    scn->sc_ops->key_delete(scn->sc_dev, keyix, freeslot);
    /*
     * Check the key->node map and flush any ref.
     */
    ni = scn->sc_keyixmap[keyix];
    if (ni != NULL) {
        ieee80211_free_node(ni);
        scn->sc_keyixmap[keyix] = NULL;
    }
    /*
     * Handle split tx/rx keying required for TKIP with h/w MIC.
     */
    if ((cip->ic_cipher == IEEE80211_CIPHER_TKIP) &&
        ((k->wk_flags & IEEE80211_KEY_SWMIC) == 0))
    {
        if (scn->sc_splitmic) {
            scn->sc_ops->key_delete(scn->sc_dev, keyix+32, freeslot);   /* RX key */
            ni = scn->sc_keyixmap[keyix+32];
            if (ni != NULL) {           /* as above... */
                ieee80211_free_node(ni);
                scn->sc_keyixmap[keyix+32] = NULL;
            }

            scn->sc_ops->key_delete(scn->sc_dev, keyix+32+64, freeslot);   /* RX key MIC */
            ASSERT(scn->sc_keyixmap[keyix+32+64] == NULL);
        }
        
        /* 
         * When splitmic, this key+64 is Tx MIC key. When non-splitmic, this
         * key+64 is Rx/Tx (combined) MIC key.
         */
        scn->sc_ops->key_delete(scn->sc_dev, keyix+64, freeslot);
        ASSERT(scn->sc_keyixmap[keyix+64] == NULL);
    }

    /* Remove the clear key allocated for MFP */
    if(k->wk_flags & IEEE80211_KEY_MFP) {
        scn->sc_ops->key_delete(scn->sc_dev, k->wk_clearkeyix, freeslot);
    }

    /* Remove receive key entry if one exists for static WEP case */
    if (ninfo != NULL) {
        rxkeyoff = ninfo->ni_rxkeyoff;
        if (rxkeyoff != 0) {
            ninfo->ni_rxkeyoff = 0;
            scn->sc_ops->key_delete(scn->sc_dev, keyix+rxkeyoff, freeslot);
            ni = scn->sc_keyixmap[keyix+rxkeyoff];
            if (ni != NULL) {   /* as above... */
                ieee80211_free_node(ni);
                scn->sc_keyixmap[keyix+rxkeyoff] = NULL;
            }
        }
    }

    return 1;
}

/*
 * Set a TKIP key into the hardware.  This handles the
 * potential distribution of key state to multiple key
 * cache slots for TKIP.
 * NB: return 1 for success, 0 otherwise.
 */
static int
ath_keyset_tkip(struct ath_softc_net80211 *scn, const struct ieee80211_key *k,
                HAL_KEYVAL *hk, const u_int8_t mac[IEEE80211_ADDR_LEN])
{
#define	IEEE80211_KEY_TXRX	(IEEE80211_KEY_XMIT | IEEE80211_KEY_RECV)

    KASSERT(k->wk_cipher->ic_cipher == IEEE80211_CIPHER_TKIP,
            ("got a non-TKIP key, cipher %u", k->wk_cipher->ic_cipher));

    if ((k->wk_flags & IEEE80211_KEY_TXRX) == IEEE80211_KEY_TXRX) {
        if (!scn->sc_splitmic) {
            /*
             * data key goes at first index,
             * the hal handles the MIC keys at index+64.
             */
            OS_MEMCPY(hk->kv_mic, k->wk_rxmic, sizeof(hk->kv_mic));
            OS_MEMCPY(hk->kv_txmic, k->wk_txmic, sizeof(hk->kv_txmic));
            KEYPRINTF(scn, k->wk_keyix, hk, mac);
            return (scn->sc_ops->key_set(scn->sc_dev, k->wk_keyix, hk, mac));
        } else {
            /*
             * TX key goes at first index, RX key at +32.
             * The hal handles the MIC keys at index+64.
             */
            OS_MEMCPY(hk->kv_mic, k->wk_txmic, sizeof(hk->kv_mic));
            KEYPRINTF(scn, k->wk_keyix, hk, NULL);
            if (!scn->sc_ops->key_set(scn->sc_dev, k->wk_keyix, hk, NULL)) {
		/*
		 * Txmic entry failed. No need to proceed further.
		 */
                return 0;
            }

            OS_MEMCPY(hk->kv_mic, k->wk_rxmic, sizeof(hk->kv_mic));
            KEYPRINTF(scn, k->wk_keyix+32, hk, mac);
            /* XXX delete tx key on failure? */
            return (scn->sc_ops->key_set(scn->sc_dev, k->wk_keyix+32, hk, mac)); 
        }
    } else if (k->wk_flags & IEEE80211_KEY_TXRX) {
        /*
         * TX/RX key goes at first index.
         * The hal handles the MIC keys are index+64.
         */
        if ((!scn->sc_splitmic) &&
            (k->wk_keyix >= IEEE80211_WEP_NKID)) {
                printk("Cannot support setting tx and rx keys individually\n");
                return 0;
        }
        OS_MEMCPY(hk->kv_mic, k->wk_flags & IEEE80211_KEY_XMIT ?
               k->wk_txmic : k->wk_rxmic, sizeof(hk->kv_mic));
        KEYPRINTF(scn, k->wk_keyix, hk, mac);
        return (scn->sc_ops->key_set(scn->sc_dev, k->wk_keyix, hk, mac));
    }
    /* XXX key w/o xmit/recv; need this for compression? */
    return 0;
#undef IEEE80211_KEY_TXRX
}

/*
 * Set the key cache contents for the specified key.  Key cache
 * slot(s) must already have been allocated by ath_key_alloc.
 * NB: return 1 for success, 0 otherwise.
 */
static int
ath_key_set(struct ieee80211vap *vap,
            const struct ieee80211_key *k,
            const u_int8_t peermac[IEEE80211_ADDR_LEN])
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(vap->iv_ic);
    static const u_int8_t ciphermap[] = {
        HAL_CIPHER_WEP,		/* IEEE80211_CIPHER_WEP     */
        HAL_CIPHER_TKIP,	/* IEEE80211_CIPHER_TKIP    */
        HAL_CIPHER_AES_OCB,	/* IEEE80211_CIPHER_AES_OCB */
        HAL_CIPHER_AES_CCM,	/* IEEE80211_CIPHER_AES_CCM */
        (u_int8_t) 0xff,	/* 4 is not allocated       */
        HAL_CIPHER_CKIP,	/* IEEE80211_CIPHER_CKIP    */
        HAL_CIPHER_CLR,		/* IEEE80211_CIPHER_NONE    */
    };
    const struct ieee80211_cipher *cip = k->wk_cipher;
    u_int8_t gmac[IEEE80211_ADDR_LEN];
    const u_int8_t *mac = NULL;
    HAL_KEYVAL hk;
    int opmode, status;

    ASSERT(cip != NULL);
    if (cip == NULL)
        return 0;

    if (k->wk_keyix == IEEE80211_KEYIX_NONE)
        return 0;

    opmode = ieee80211vap_get_opmode(vap);
    memset(&hk, 0, sizeof(hk));
    /*
     * Software crypto uses a "clear key" so non-crypto
     * state kept in the key cache are maintained and
     * so that rx frames have an entry to match.
     */
    if ((k->wk_flags & IEEE80211_KEY_SWCRYPT) == 0) {
        KASSERT(cip->ic_cipher < (sizeof(ciphermap)/sizeof(ciphermap[0])),
                ("invalid cipher type %u", cip->ic_cipher));
        hk.kv_type = ciphermap[cip->ic_cipher];
        hk.kv_len  = k->wk_keylen;
        OS_MEMCPY(hk.kv_val, k->wk_key, k->wk_keylen);
    } else
        hk.kv_type = HAL_CIPHER_CLR;

    /*
     *  Strategy:
     *   For _M_STA mc tx, we will not setup a key at all since we never tx mc.
     *       _M_STA mc rx, we will use the keyID.
     *   for _M_IBSS mc tx, we will use the keyID, and no macaddr.
     *   for _M_IBSS mc rx, we will alloc a slot and plumb the mac of the peer node. BUT we 
     *       will plumb a cleartext key so that we can do perSta default key table lookup
     *       in software.
     */
    if (k->wk_flags & IEEE80211_KEY_GROUP) {
        switch (opmode) {
        case IEEE80211_M_STA:
            /* default key:  could be group WPA key or could be static WEP key */
            mac = NULL;
            break;

        case IEEE80211_M_IBSS:
            if (k->wk_flags & IEEE80211_KEY_RECV) {
                if (k->wk_flags & IEEE80211_KEY_PERSTA) {
                    //ASSERT(scn->sc_mcastkey); /* require this for perSta keys */
                    ASSERT(k->wk_keyix >= IEEE80211_WEP_NKID);

                    /*
                     * Group keys on hardware that supports multicast frame
                     * key search use a mac that is the sender's address with
                     * the bit 0 set instead of the app-specified address.
                     * This is a flag to indicate to the HAL that this is 
                     * multicast key. Using any other bits for this flag will
                     * corrupt the MAC address.
                     * XXX: we should use a new parameter called "Multicast" and
                     * pass it to key_set routines instead of embedding this flag.
                     */
                    IEEE80211_ADDR_COPY(gmac, peermac);
                    gmac[0] |= 0x01;
                    mac = gmac;
                } else {
                    /* static wep */
                    mac = NULL;
                }
            } else if (k->wk_flags & IEEE80211_KEY_XMIT) {
                /*
                 * Owl and Merlin have problems in mcast key search.
                 * TX key will be allocated > IEEE80211_WEP_NKID.
                 */
                //ASSERT(k->wk_keyix < IEEE80211_WEP_NKID);
                mac = NULL;
            } else {
                ASSERT(0);
                status = 0;
                goto done;
            }
            break;
            
        case IEEE80211_M_HOSTAP:
            if (scn->sc_mcastkey) {
                /*
                 * Group keys on hardware that supports multicast frame
                 * key search use a mac that is the sender's address with
                 * the bit 0 set instead of the app-specified address.
                 * This is a flag to indicate to the HAL that this is 
                 * multicast key. Using any other bits for this flag will
                 * corrupt the MAC address.
                 * XXX: we should use a new parameter called "Multicast" and
                 * pass it to key_set routines instead of embedding this flag.
                 */
                IEEE80211_ADDR_COPY(gmac, vap->iv_bss->ni_macaddr);
                gmac[0] |= 0x01;
                mac = gmac;
            } else
                mac = peermac;
            break;
            
        default:
            ASSERT(0);
            break;
        }
    } else {
        /* key mapping key */
        ASSERT(k->wk_keyix >= IEEE80211_WEP_NKID);
        mac = peermac;
    }

    if (hk.kv_type == HAL_CIPHER_TKIP &&
        (k->wk_flags & IEEE80211_KEY_SWMIC) == 0)
    {
        status = ath_keyset_tkip(scn, k, &hk, mac);
    } else {
        KEYPRINTF(scn, k->wk_keyix, &hk, mac);
        status = (scn->sc_ops->key_set(scn->sc_dev, k->wk_keyix, &hk, mac) != 0);
    }

    if ((mac != NULL) && (cip->ic_cipher == IEEE80211_CIPHER_TKIP)) {
        struct ieee80211com *ic = vap->iv_ic;
        struct ieee80211_node *ni;
        if (opmode == IEEE80211_M_STA) {
#ifdef NODE_FREE_DEBUG
            ni = ieee80211_ref_node(vap->iv_bss, (char *)__func__);
#else
            ni = ieee80211_ref_node(vap->iv_bss);
#endif
        } else
            ni = ieee80211_find_node(&ic->ic_sta, mac);
        if (ni) {
            ni->ni_flags |= IEEE80211_NODE_TKIPCIPHER;
            ath_net80211_rate_node_update(ic, ni, 1);
            ieee80211_free_node(ni);
        }
    }

    if ((k->wk_flags & IEEE80211_KEY_MFP) && (opmode == IEEE80211_M_STA)) {
        /* Create a clear key entry to be used for MFP */
        hk.kv_type = HAL_CIPHER_CLR;
        KEYPRINTF(scn, k->wk_clearkeyix, &hk, mac);
        status = (scn->sc_ops->key_set(scn->sc_dev, k->wk_clearkeyix, &hk, NULL) != 0);
    }

done:
    return status;
}

#ifdef ATH_SWRETRY
/*
 * key cache management
 * Key cache slot is allocated for open and wep cases only 
 * if SW Retry feature is required.
 */

 /*
 * Allocate a key cache slot to the station so we can
 * setup a mapping from key index to node. The key cache
 * slot is needed for managing antenna state and for
 * compression when stations do not use crypto.  We do
 * it uniliaterally here; if crypto is employed this slot
 * will be reassigned.
 */
 
static void
ath_setup_stationkey(struct ieee80211_node *ni)
{
    struct ieee80211vap *vap = ni->ni_vap;
    u_int16_t keyix;

    keyix = ath_key_alloc(vap, &ni->ni_ucastkey);
    if (keyix == IEEE80211_KEYIX_NONE) {
        /*
         * Key cache is full; we'll fall back to doing
         * the more expensive lookup in software.  Note
         * this also means no h/w compression.
         */
        /* XXX msg+statistic */
        return;
    } else {
        ni->ni_ucastkey.wk_keyix = keyix;
        ni->ni_ucastkey.wk_valid = AH_TRUE;
        /* NB: this will create a pass-thru key entry */
        ath_key_set(vap, &ni->ni_ucastkey, ni->ni_bssid);

#ifdef ATH_SUPERG_COMP
        /* Enable de-compression logic */
        ath_setup_comp(ni, 1);
#endif
    }
	
    return;
}

/* Setup WEP key for the station if compression is negotiated.
 * When station and AP are using same default key index, use single key
 * cache entry for receive and transmit, else two key cache entries are
 * created. One for receive with MAC address of station and one for transmit
 * with NULL mac address. On receive key cache entry de-compression mask
 * is enabled.
 */

static void
ath_setup_stationwepkey(struct ieee80211_node *ni)
{
    struct ieee80211vap *vap = ni->ni_vap;
    struct ieee80211_key *ni_key;
    struct ieee80211_key tmpkey;
    struct ieee80211_key *rcv_key, *xmit_key;
    int    txkeyidx, rxkeyidx = IEEE80211_KEYIX_NONE,i;
    u_int8_t null_macaddr[IEEE80211_ADDR_LEN] = {0,0,0,0,0,0};

    KASSERT(ni->ni_ath_defkeyindex < IEEE80211_WEP_NKID,
            ("got invalid node key index 0x%x", ni->ni_ath_defkeyindex));
    KASSERT(vap->iv_def_txkey < IEEE80211_WEP_NKID,
            ("got invalid vap def key index 0x%x", vap->iv_def_txkey));

    /* Allocate a key slot first */
    if (!ieee80211_crypto_newkey(vap, 
                                 IEEE80211_CIPHER_WEP, 
                                 IEEE80211_KEY_XMIT|IEEE80211_KEY_RECV, 
                                 &ni->ni_ucastkey)) {
        goto error;
    }

    txkeyidx = ni->ni_ucastkey.wk_keyix;
    xmit_key = &vap->iv_nw_keys[vap->iv_def_txkey];

    /* Do we need seperate rx key? */
    if (ni->ni_ath_defkeyindex != vap->iv_def_txkey) {
        ni->ni_ucastkey.wk_keyix = IEEE80211_KEYIX_NONE;
        if (!ieee80211_crypto_newkey(vap, 
                                     IEEE80211_CIPHER_WEP, 
                                     IEEE80211_KEY_XMIT|IEEE80211_KEY_RECV,
                                     &ni->ni_ucastkey)) {
            ni->ni_ucastkey.wk_keyix = txkeyidx;
            ieee80211_crypto_delkey(vap, &ni->ni_ucastkey, ni);
            goto error;
        }
        rxkeyidx = ni->ni_ucastkey.wk_keyix;
        ni->ni_ucastkey.wk_keyix = txkeyidx;

        rcv_key = &vap->iv_nw_keys[ni->ni_ath_defkeyindex];
    } else {
        rcv_key = xmit_key;
        rxkeyidx = txkeyidx;
    }

    /* Remember receive key offset */
    ni->ni_rxkeyoff = rxkeyidx - txkeyidx;

    /* Setup xmit key */
    ni_key = &ni->ni_ucastkey;
    if (rxkeyidx != txkeyidx) {
        ni_key->wk_flags = IEEE80211_KEY_XMIT;
    } else {
        ni_key->wk_flags = IEEE80211_KEY_XMIT|IEEE80211_KEY_RECV;
    }
    ni_key->wk_keylen = xmit_key->wk_keylen;
    for(i=0;i<IEEE80211_TID_SIZE;++i)
        ni_key->wk_keyrsc[i] = xmit_key->wk_keyrsc[i];
    ni_key->wk_keytsc = 0; 
    OS_MEMZERO(ni_key->wk_key, sizeof(ni_key->wk_key));
    OS_MEMCPY(ni_key->wk_key, xmit_key->wk_key, xmit_key->wk_keylen);
    ieee80211_crypto_setkey(vap, &ni->ni_ucastkey, 
                            (rxkeyidx == txkeyidx) ? ni->ni_macaddr : null_macaddr, NULL);

    if (rxkeyidx != txkeyidx) {
        /* Setup recv key */
        ni_key = &tmpkey;
        ni_key->wk_keyix = rxkeyidx;
        ni_key->wk_flags = IEEE80211_KEY_RECV;
        ni_key->wk_keylen = rcv_key->wk_keylen;

        for(i = 0; i < IEEE80211_TID_SIZE; ++i)
            ni_key->wk_keyrsc[i] = rcv_key->wk_keyrsc[i];

        ni_key->wk_keytsc = 0;
        ni_key->wk_cipher = rcv_key->wk_cipher;
        ni_key->wk_private = rcv_key->wk_private;
        OS_MEMZERO(ni_key->wk_key, sizeof(ni_key->wk_key));
        OS_MEMCPY(ni_key->wk_key, rcv_key->wk_key, rcv_key->wk_keylen);
        ieee80211_crypto_setkey(vap, &tmpkey, ni->ni_macaddr, NULL);
    }

    return;

error:
    ni->ni_ath_flags &= ~IEEE80211_NODE_COMP;
    return;
}

/* Create a keycache entry for given node in clearcase as well as static wep.
 * Handle compression state if required.
 * For non clearcase/static wep case, the key is plumbed by hostapd.
 */
static void
ath_setup_keycacheslot(struct ieee80211_node *ni)
{
    struct ieee80211vap *vap = ni->ni_vap;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(vap->iv_ic);

    if (ni->ni_ucastkey.wk_keyix != IEEE80211_KEYIX_NONE) {
        ieee80211_crypto_delkey(vap, &ni->ni_ucastkey, ni);
    }

    /* Only for clearcase and WEP case */
    if (ieee80211vap_has_flag(vap, IEEE80211_F_PRIVACY) == 0 ||
        (ni->ni_ath_defkeyindex != IEEE80211_INVAL_DEFKEY)) {

        if (ieee80211vap_has_flag(vap, IEEE80211_F_PRIVACY) == 0) {
            KASSERT(ni->ni_ucastkey.wk_keyix  \
                    == IEEE80211_KEYIX_NONE, \
                    ("new node with a ucast key already setup (keyix %u)",\
                     ni->ni_ucastkey.wk_keyix));
            /* 
             * For now, all chips support clr key.
             * // NB: 5210 has no passthru/clr key support
             * if (scn->sc_ops->has_cipher(scn->sc_dev, HAL_CIPHER_CLR))
             *   ath_setup_stationkey(ni);
             */
            ath_setup_stationkey(ni);
        } else {
            ath_setup_stationwepkey(ni);
        }
    }

    return;
}
#endif /* ATH_SWRETRY */

/*
 * Block/unblock tx+rx processing while a key change is done.
 * We assume the caller serializes key management operations
 * so we only need to worry about synchronization with other
 * uses that originate in the driver.
 */
static void
ath_key_update_begin(struct ieee80211vap *vap)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(vap->iv_ic);

    DPRINTF(scn, ATH_DEBUG_KEYCACHE, "%s:\n", __func__);
    /*
     * When called from the rx tasklet we cannot use
     * tasklet_disable because it will block waiting
     * for us to complete execution.
     *
     * XXX Using in_softirq is not right since we might
     * be called from other soft irq contexts than
     * ath_rx_tasklet.
     */
#ifdef notyet
    // XXX Need to stop both rx and tx here, and drain queues
    // XXX Since this will usually be done in OID context, maybe
    // we should use NdisSyncWithIntrrupt()?
    if (!in_softirq())
        tasklet_disable(&sc->sc_rxtq);
    netif_stop_queue(dev);
#endif
}

static void
ath_key_update_end(struct ieee80211vap *vap)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(vap->iv_ic);

    DPRINTF(scn, ATH_DEBUG_KEYCACHE, "%s:\n", __func__);
#ifdef notyet
    // XXX Need to stop both rx and tx here, and drain queues
    // XXX Since this will usually be done in OID context, maybe
    // we should use NdisSyncWithIntrrupt()?
    netif_start_queue(dev);
    if (!in_softirq())		/* NB: see above */
        tasklet_enable(&sc->sc_rxtq);
#endif
}

static void
ath_update_ps_mode(struct ieee80211vap *vap)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(vap->iv_ic);
    struct ath_vap_net80211 *avn = ATH_VAP_NET80211(vap);
    
    /*
     * if not in RUN state (or)
     * if we are waiting for syncbeacon
     * nothing to do. 
     */
    if (vap->iv_state != IEEE80211_S_RUN || scn->sc_syncbeacon)
        return;

    /*
     * reconfigure the beacon timers.
     */
    scn->sc_ops->sync_beacon(scn->sc_dev, avn->av_if_id);
}

static void
ath_net80211_pwrsave_set_state(struct ieee80211com *ic, IEEE80211_PWRSAVE_STATE newstate)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);

    switch (newstate) {
    case IEEE80211_PWRSAVE_AWAKE:
        scn->sc_ops->awake(scn->sc_dev);
        break;
    case IEEE80211_PWRSAVE_NETWORK_SLEEP:
        scn->sc_ops->netsleep(scn->sc_dev);
        break;
    case IEEE80211_PWRSAVE_FULL_SLEEP:
        scn->sc_ops->fullsleep(scn->sc_dev);
        break;
    default:
        DPRINTF(scn, ATH_DEBUG_STATE, "%s: wrong power save state %u\n",
                __func__, newstate);
    }
}

int
ath_tx_prepare(struct ath_softc_net80211 *scn, wbuf_t wbuf, int nextfraglen,
               ieee80211_tx_control_t *txctl)
{
    struct ieee80211_node *ni = wbuf_get_node(wbuf);
    struct ieee80211com *ic = &scn->sc_ic;
    struct ath_softc *sc = ATH_DEV_TO_SC(scn->sc_dev);
    struct ieee80211vap *vap = ni->ni_vap;
    struct ieee80211_frame *wh;
    int keyix, hdrlen, pktlen;
    int type, subtype, use_minrate=0;
    HAL_KEY_TYPE keytype = HAL_KEY_TYPE_CLEAR;
    struct sk_buff *skb=wbuf;

    use_minrate=txctl->iseap;

    OS_MEMZERO(txctl, sizeof(ieee80211_tx_control_t));

    if (use_minrate) {
            /* Added a iwpriv to enable using the lowest data rate for EAP packets
               sc->sc_eap_lowest_rate reflects whether lowest rate should be used or not */
            txctl->use_minrate=sc->sc_eap_lowest_rate;
            txctl->iseap=1;
            txctl->min_rate=ni->ni_minbasicrate;
     }

    wh = (struct ieee80211_frame *)wbuf_header(wbuf);

    txctl->ismcast = IEEE80211_IS_MULTICAST(wh->i_addr1);
    txctl->istxfrag = (wh->i_fc[1] & IEEE80211_FC1_MORE_FRAG) ||
        (((le16toh(*((u_int16_t *)&(wh->i_seq[0]))) >>
           IEEE80211_SEQ_FRAG_SHIFT) & IEEE80211_SEQ_FRAG_MASK) > 0);
    type = wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK;
    subtype = wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK;

    /*
     * Packet length must not include any
     * pad bytes; deduct them here.
     */
    hdrlen = ieee80211_anyhdrsize(wh);
    pktlen = wbuf_get_pktlen(wbuf);
    pktlen -= (hdrlen & 3);

    if (IEEE80211_IS_SAFEMODE_ENABLED(ic)) {
        /* For Safe Mode, the encryption and its encap is already done
           by the upper layer software. Driver do not modify the packet. */
        keyix = HAL_TXKEYIX_INVALID;
    }
    else if (wh->i_fc[1] & IEEE80211_FC1_WEP) {
        const struct ieee80211_cipher *cip;
        struct ieee80211_key *k;

        /*
         * Construct the 802.11 header+trailer for an encrypted
         * frame. The only reason this can fail is because of an
         * unknown or unsupported cipher/key type.
         */

        /* FFXXX: change to handle linked wbufs */
        k = ieee80211_crypto_encap(ni, wbuf);

        if (k == NULL) {
            /*
             * This can happen when the key is yanked after the
             * frame was queued.  Just discard the frame; the
             * 802.11 layer counts failures and provides
             * debugging/diagnostics.
             */
            return -EIO;
        }
        /* update the value of wh since encap can reposition the header */
        wh = (struct ieee80211_frame *)wbuf_header(wbuf);

        /*
         * Adjust the packet + header lengths for the crypto
         * additions and calculate the h/w key index. When
         * a s/w mic is done the frame will have had any mic
         * added to it prior to entry so wbuf pktlen above will
         * account for it. Otherwise we need to add it to the
         * packet length.
         */
        cip = k->wk_cipher;
        hdrlen += cip->ic_header;
        pktlen += cip->ic_header + cip->ic_trailer;
        if ((k->wk_flags & IEEE80211_KEY_SWMIC) == 0) {
            if ( ! txctl->istxfrag)
                pktlen += cip->ic_miclen;
            else {
                if (cip->ic_cipher != IEEE80211_CIPHER_TKIP)
                    pktlen += cip->ic_miclen;
            }
        }

        switch (cip->ic_cipher) {
        case IEEE80211_CIPHER_WEP:
        case IEEE80211_CIPHER_CKIP:
            keytype = HAL_KEY_TYPE_WEP;
            break;
        case IEEE80211_CIPHER_TKIP:
            keytype = HAL_KEY_TYPE_TKIP;
            break;
        case IEEE80211_CIPHER_AES_OCB:
        case IEEE80211_CIPHER_AES_CCM:
            keytype = HAL_KEY_TYPE_AES;
            break;
        default:
            keytype = HAL_KEY_TYPE_CLEAR;
        }
        
        if (((k->wk_flags & IEEE80211_KEY_MFP) && IEEE80211_IS_MFP_FRAME(wh))) {
            keyix = k->wk_clearkeyix;
            keytype = HAL_KEY_TYPE_CLEAR;
            /* mfp packet len could be extended by MHDR IE */
            pktlen = wbuf_get_pktlen(wbuf);
        }
        else 
            keyix = k->wk_keyix;

#ifdef ATH_SUPERG_COMP
#ifdef USE_LEGACY_HAL
        txctl->ivlen  = k->wk_cipher->ic_header;
        txctl->icvlen = k->wk_cipher->ic_trailer;
#endif
#endif

    }  else if (ni->ni_ucastkey.wk_cipher == &ieee80211_cipher_none) {
        /*
         * Use station key cache slot, if assigned.
         */
        keyix = ni->ni_ucastkey.wk_keyix;
        if (keyix == IEEE80211_KEYIX_NONE)
            keyix = HAL_TXKEYIX_INVALID;
    } else
        keyix = HAL_TXKEYIX_INVALID;

    pktlen += IEEE80211_CRC_LEN;

    txctl->frmlen = pktlen;
    txctl->keyix = keyix;
    txctl->keytype = keytype;
    txctl->txpower = ni->ni_txpower;
    txctl->nextfraglen = nextfraglen;
#ifdef USE_LEGACY_HAL
    txctl->hdrlen = hdrlen;
#endif
#ifdef ATH_SUPPORT_IQUE
	txctl->tidno = wbuf_get_tid(wbuf);
#endif	
    
    /*
     * NB: the 802.11 layer marks whether or not we should
     * use short preamble based on the current mode and
     * negotiated parameters.
     */
    if (ieee80211com_has_flag(ic, IEEE80211_F_SHPREAMBLE) &&
        !ieee80211com_has_flag(ic, IEEE80211_F_USEBARKER) &&
        ieee80211node_has_cap(ni, IEEE80211_CAPINFO_SHORT_PREAMBLE)) {
        txctl->shortPreamble = 1;
    }

#ifndef ATH_SWRETRY 
    txctl->flags = HAL_TXDESC_CLRDMASK;    /* XXX needed for crypto errs */
#endif

    /*
     * Calculate Atheros packet type from IEEE80211
     * packet header and select h/w transmit queue.
     */
    switch (type) {
    case IEEE80211_FC0_TYPE_MGT:
        txctl->ismgmt = 1;
        txctl->use_minrate = 1;

        if (subtype == IEEE80211_FC0_SUBTYPE_BEACON)
            txctl->atype = HAL_PKT_TYPE_BEACON;
        else if (subtype == IEEE80211_FC0_SUBTYPE_PROBE_RESP)
            txctl->atype = HAL_PKT_TYPE_PROBE_RESP;
        else if (subtype == IEEE80211_FC0_SUBTYPE_ATIM)
            txctl->atype = HAL_PKT_TYPE_ATIM;
        else
            txctl->atype = HAL_PKT_TYPE_NORMAL;	/* XXX */

        /*
         * Send out all mangement frames except Probe request
         * at minimum rate set by AP.
         */
        if ((ieee80211vap_get_opmode(vap) == IEEE80211_M_STA) &&
            (ni->ni_minbasicrate != 0) &&
            (subtype != IEEE80211_FC0_SUBTYPE_PROBE_REQ)) {
            txctl->min_rate = ni->ni_minbasicrate;
        }
        
        /* NB: force all management frames to highest queue */
        txctl->qnum = scn->sc_ac2q[WME_AC_VO];

        break;
    case IEEE80211_FC0_TYPE_CTL:
        txctl->use_minrate = 1;

        switch (subtype) {
        case IEEE80211_FC0_SUBTYPE_PS_POLL:
            /*
             * Make sure hardware does not overwrite association id which
             * is already in duration field of PS-Poll frame header.
             */
            txctl->atype = HAL_PKT_TYPE_PSPOLL;
            txctl->ispspoll = 1;
            break;
        case IEEE80211_FC0_SUBTYPE_BAR:
            txctl->isbar = 1;
            /* fall through to default case */
        default:
            /* 
             * Duration field of all control frames except PS-Poll carries
             * duration information for updating the NAV in hardware.
             */
            txctl->atype = HAL_PKT_TYPE_NORMAL;
            break;
        }

        if (ieee80211node_has_flag(ni, IEEE80211_NODE_QOS)) {
            /* NB: force all ctl frames to highest queue */
            txctl->qnum = scn->sc_ac2q[WME_AC_VO];
        } else {
            txctl->qnum = scn->sc_ac2q[WME_AC_BE];
        }
        break;
    case IEEE80211_FC0_TYPE_DATA:
        txctl->isdata = 1;
        txctl->atype = HAL_PKT_TYPE_NORMAL;		/* default */
#ifdef ATH_SUPPORT_UAPSD
        txctl->isuapsd = wbuf_is_uapsd(wbuf);
#endif
        if (txctl->ismcast)
            txctl->mcast_rate = vap->iv_mcast_rate;
        
        if (wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_QOS) {
            int ac = wbuf_get_priority(wbuf);
            txctl->isqosdata = 1;
            
            /* XXX validate frame priority, remove mask */
            txctl->qnum = scn->sc_ac2q[ac & 0x3];
            if (ieee80211_wmm_chanparams(ic, ac)->wmep_noackPolicy)
                txctl->flags |= HAL_TXDESC_NOACK;

        } else {
            /*
             * Default all non-QoS traffic to the best-effort queue.
             */
            txctl->qnum = scn->sc_ac2q[WME_AC_BE];
            wbuf_set_priority(wbuf, WME_AC_BE);
        }

#ifdef ATH_SUPPORT_UAPSD
        /*
         * UAPSD frames go to a dedicated hardware queue.
         */
        if (txctl->isuapsd) {
            txctl->qnum = scn->sc_uapsd_qnum;
        }
#endif
        /*
         * For HT capable stations, we save tidno for later use.
         * We also override seqno set by upper layer with the one
         * in tx aggregation state.
         */
	if (!txctl->ismcast && !txctl->use_minrate && IEEE80211_NODE_USEAMPDU(ni))
		txctl->ht = 1;
        /*
         * Use lowest rate for NULL data frame and EAPOL frame.
         * Send out using VO queue if possible
         */
        if (subtype == IEEE80211_FC0_SUBTYPE_NODATA ||
	    wbuf_is_eapol(wbuf)) {
		

            /*
             * Send out all mangement frames except Probe request
             * at minimum rate set by AP.
             */
            if ((ieee80211vap_get_opmode(vap) == IEEE80211_M_STA) &&
                (ni->ni_minbasicrate != 0)) {
                txctl->min_rate = ni->ni_minbasicrate;
            }

            if (ieee80211node_has_flag(ni, IEEE80211_NODE_QOS))
                txctl->qnum = scn->sc_ac2q[WME_AC_VO];
            else
                txctl->qnum = scn->sc_ac2q[WME_AC_BE];
        txctl->iseapol = scn->sc_ac2q[WME_AC_VO];
	M_FLAG_CLR(skb, M_EAPOL);
        }

	

        if (!txctl->ismcast && !txctl->use_minrate && ieee80211node_has_flag(ni, IEEE80211_NODE_HT))
            txctl->ht = 1;
        
        break;
    default:
        printk("bogus frame type 0x%x (%s)\n",
               wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK, __func__);
        /* XXX statistic */
        return -EIO;
    }

    /*
     * If we are servicing one or more stations in power-save mode.
     */
    txctl->if_id = (ATH_VAP_NET80211(vap))->av_if_id;
    if (ieee80211vap_has_pssta(vap))
        txctl->ps = 1;
    
    /*
     * Calculate miscellaneous flags.
     */
    if (txctl->ismcast) {
        txctl->flags |= HAL_TXDESC_NOACK;	/* no ack on broad/multicast */
    } else if (pktlen > ieee80211vap_get_rtsthreshold(vap)) { 
#ifdef ATH_SUPERG_FF
        /* we could refine to only check that the frame of interest
         * is a FF, but this seems inconsistent.
         */
        if(!(ieee80211vap_has_athcap(vap, IEEE80211_ATHC_FF) &&
             ieee80211node_has_athflag(ni, IEEE80211_NODE_FF))) {
#endif
            txctl->flags |= HAL_TXDESC_RTSENA;	/* RTS based on frame length */
#ifdef ATH_SUPERG_FF
        }
#endif
    }

    /* Frame to enable SM power save */
    if (wbuf_is_smpsframe(wbuf)) {
        txctl->flags |= HAL_TXDESC_LOWRXCHAIN;
    }
    return 0;
}

/*
 * The function to send a frame (i.e., hardstart). The wbuf should already be
 * associated with the actual frame, and have a valid node instance.
 */

int
ath_tx_send(wbuf_t wbuf)
{
    struct ieee80211_node *ni = wbuf_get_node(wbuf);
    struct ieee80211com *ic = ni->ni_ic;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    wbuf_t next_wbuf;
    struct ether_header *eh = (struct ether_header *)wbuf_header(wbuf);
    int use_minrate=0;

    /* Use the lowest data rate for EAP packets */
    if (eh->ether_type == __constant_htons(ETHERTYPE_PAE)) {
            use_minrate=1;
    }

    /*
     * XXX TODO: Fast frame here
     */

#ifdef ATH_SUPERG_FF
    if (IEEE80211_ATH_CAP(ieee80211node_get_vap(ni), ni, IEEE80211_ATHC_FF)) {
        wbuf = ath_ff(wbuf);
        if (wbuf == NULL)
            return 0;
    }
#endif

#ifdef ATH_AMSDU
    /* Check whether AMSDU is supported in this BlockAck agreement */
    if (IEEE80211_NODE_USEAMPDU(ni) &&
        scn->sc_ops->get_amsdusupported(scn->sc_dev,
                                        ATH_NODE_NET80211(ni)->an_sta,
                                        wbuf_get_tid(wbuf)))
    {
        wbuf = ath_amsdu_send(wbuf);
        if (wbuf == NULL)
            return 0;
    }
#endif

    /*
     * Encapsulate the packet for transmission
     */
    wbuf = ieee80211_encap(ni, wbuf);
    if (wbuf == NULL) {
        goto bad;
    }

    /*
     * If node is HT capable, then send out ADDBA if
     * we haven't done so.
     *
     * XXX: send ADDBA here to avoid re-entrance of other
     * tx functions.
     */
    if (IEEE80211_NODE_USEAMPDU(ni) &&
        ic->ic_addba_mode == ADDBA_MODE_AUTO &&
        ni->ni_vap->iv_fragthreshold == IEEE80211_FRAGMT_THRESHOLD_MAX) {
        
        u_int8_t tidno = wbuf_get_tid(wbuf);
        struct ieee80211_action_mgt_args actionargs;

        if (
#ifdef ATH_SUPPORT_UAPSD
           (!IEEE80211_NODE_AC_UAPSD_ENABLED(ni, TID_TO_WME_AC(tidno))) &&
#endif
           (scn->sc_ops->check_aggr(scn->sc_dev, ATH_NODE_NET80211(ni)->an_sta, tidno)))
        {
            /* Send ADDBA request */
            actionargs.category = IEEE80211_ACTION_CAT_BA;
            actionargs.action   = IEEE80211_ACTION_BA_ADDBA_REQUEST;
            actionargs.arg1     = tidno;
            actionargs.arg2     = WME_MAX_BA;
            actionargs.arg3     = 0;

            ieee80211_send_action(ni, (void *)&actionargs);
        }
    }

    /* send down each fragment */
    while (wbuf != NULL) {
        int nextfraglen = 0;
        ATH_DEFINE_TXCTL(txctl, wbuf);

        next_wbuf = wbuf_next(wbuf);
        if (next_wbuf != NULL)
            nextfraglen = wbuf_get_pktlen(next_wbuf);

        if (use_minrate) {
            txctl->iseap=1;
        } else  {
            txctl->iseap=0;
        }
        /* prepare this frame */
        if (ath_tx_prepare(scn, wbuf, nextfraglen, txctl) != 0)
            goto bad;


        /* send this frame to hardware */
        txctl->an = (ATH_NODE_NET80211(ni))->an_sta;

        /*
        ** Check to see if the frag thereshold is set to something else.  If so
        ** set the frag flag in the control block
        */

        if (unlikely(ni->ni_vap->iv_fragthreshold != IEEE80211_FRAGMT_THRESHOLD_MAX)) {
            txctl->flags |= HAL_TXDESC_FRAG_IS_ON;
        }

        /*
        ** Transmit the Frame!
        */

        if (scn->sc_ops->tx(scn->sc_dev, wbuf, txctl) != 0)
            goto bad;

        wbuf = next_wbuf;
    }

    return 0;

bad:
    /* drop rest of the un-sent fragments */
    while (wbuf != NULL) {
        next_wbuf = wbuf_next(wbuf);
        IEEE80211_TX_COMPLETE_WITH_ERROR(wbuf, -EIO);

        wbuf = next_wbuf;
    }
    
    return -EIO;
}

/*
 * The function to send a management frame. The wbuf should already
 * have a valid node instance.
 */
int
ath_tx_mgt_send(struct ieee80211com *ic, wbuf_t wbuf)
{
    struct ieee80211_node *ni = wbuf_get_node(wbuf);
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    int error = 0;
    ATH_DEFINE_TXCTL(txctl, wbuf);

    txctl->iseap=0;

    /* Just bypass fragmentation and fast frame. */
    error = ath_tx_prepare(scn, wbuf, 0, txctl);
    if (!error) {
        /* send this frame to hardware */
        txctl->an = (ATH_NODE_NET80211(ni))->an_sta;
        error = scn->sc_ops->tx(scn->sc_dev, wbuf, txctl);
        if (!error)
            return 0;
    }

    /* fall thru... */
    IEEE80211_TX_COMPLETE_WITH_ERROR(wbuf, 0);
    return error;
}

static u_int32_t
ath_net80211_txq_depth(struct ieee80211com *ic)
{
    int ac, qdepth = 0;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    
    for (ac = WME_AC_BE; ac <= WME_AC_VO; ac++) {
        qdepth += scn->sc_ops->txq_depth(scn->sc_dev, scn->sc_ac2q[ac]);
    }
    return qdepth;
}

static void
ath_net80211_tx_complete(wbuf_t wbuf, ieee80211_tx_status_t *tx_status)
{
    struct ieee80211_tx_status ts;

    ts.ts_flags =
        ((tx_status->flags & ATH_TX_ERROR) ? IEEE80211_TX_ERROR : 0) |
        ((tx_status->flags & ATH_TX_XRETRY) ? IEEE80211_TX_XRETRY : 0);
    ts.ts_retries = tx_status->retries;
    ieee80211_complete_wbuf(wbuf, &ts, 0);
}

static void
ath_net80211_tx_status(ieee80211_node_t node, ieee80211_tx_stat_t *txstatus)
{
    struct ieee80211_node *ni = (struct ieee80211_node *)node;
    int i;
    struct ieee80211vap *vap = ni->ni_vap;

    ni->ni_inact = ni->ni_inact_reload;

    ATH_RSSI_LPF(ATH_NODE_NET80211(ni)->an_avgtxrssi, txstatus->rssi);
#ifdef notyet
    /* Unused code for now. Disable it since no one is using it. */
    if (txstatus->rateKbps)
        ATH_RATE_LPF(ATH_NODE_NET80211(ni)->an_avgtxrate, txstatus->rateKbps);
#endif

    if (txstatus->rateKbps > 0) {
        ATH_NODE_NET80211(ni)->an_lasttxrate = txstatus->rateKbps;
        ATH_NODE_NET80211(vap->iv_bss)->an_lasttxrate = txstatus->rateKbps ; 
    }

    if (txstatus->flags & ATH_TX_CHAIN_RSSI_VALID) {
        for(i=0;i<ATH_MAX_ANTENNA;++i) {
            ATH_RATE_LPF(ATH_NODE_NET80211(ni)->an_avgtxchainrssi[i], txstatus->rssictl[i]);
            ATH_RATE_LPF(ATH_NODE_NET80211(ni)->an_avgtxchainrssiext[i], txstatus->rssiextn[i]);
        }
    }
}

static void
ath_net80211_updateslot(struct ieee80211com *ic)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    int slottime;

    slottime = (ieee80211com_has_flag(ic, IEEE80211_F_SHSLOT)) ?
        HAL_SLOT_TIME_9 : HAL_SLOT_TIME_20;

    scn->sc_ops->set_slottime(scn->sc_dev, slottime);
}

static void
ath_net80211_update_protmode(struct ieee80211com *ic)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    PROT_MODE mode = PROT_M_NONE;

    if (ieee80211com_has_flag(ic, IEEE80211_F_USEPROT)) {
        if (ic->ic_protmode == IEEE80211_PROT_RTSCTS)
            mode = PROT_M_RTSCTS;
        else if (ic->ic_protmode == IEEE80211_PROT_CTSONLY)
            mode = PROT_M_CTSONLY;
    }
    scn->sc_ops->set_protmode(scn->sc_dev, mode);
}

static void
ath_net80211_set_ampduparams(struct ieee80211_node *ni)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);

    scn->sc_ops->set_ampdu_params(scn->sc_dev,
                                 ATH_NODE_NET80211(ni)->an_sta,
                                 ni->ni_maxampdu,
                                 ni->ni_mpdudensity);
}

static void
ath_net80211_set_weptkip_rxdelim(struct ieee80211_node *ni, u_int8_t rxdelim)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);

    scn->sc_ops->set_weptkip_rxdelim(scn->sc_dev,
                                 ATH_NODE_NET80211(ni)->an_sta,
                                 rxdelim);
}


static void
ath_net80211_addba_requestsetup(struct ieee80211_node *ni,
                                u_int8_t tidno,
                                struct ieee80211_ba_parameterset *baparamset,
                                u_int16_t *batimeout,
                                struct ieee80211_ba_seqctrl *basequencectrl,
                                u_int16_t buffersize
                                )
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    scn->sc_ops->addba_request_setup(scn->sc_dev, ATH_NODE_NET80211(ni)->an_sta,
                                     tidno, baparamset, batimeout, basequencectrl,
                                     buffersize);
}

static void
ath_net80211_addba_responsesetup(struct ieee80211_node *ni,
                                 u_int8_t tidno,
                                 u_int8_t *dialogtoken, u_int16_t *statuscode,
                                 struct ieee80211_ba_parameterset *baparamset,
                                 u_int16_t *batimeout
                                 )
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    scn->sc_ops->addba_response_setup(scn->sc_dev, ATH_NODE_NET80211(ni)->an_sta,
                                      tidno, dialogtoken, statuscode,
                                      baparamset, batimeout);
}

static int
ath_net80211_addba_requestprocess(struct ieee80211_node *ni,
                                  u_int8_t dialogtoken,
                                  struct ieee80211_ba_parameterset *baparamset,
                                  u_int16_t batimeout,
                                  struct ieee80211_ba_seqctrl basequencectrl
                                  )
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    return scn->sc_ops->addba_request_process(scn->sc_dev, ATH_NODE_NET80211(ni)->an_sta,
                                              dialogtoken, baparamset, batimeout,
                                              basequencectrl);
}

static void
ath_net80211_addba_responseprocess(struct ieee80211_node *ni,
                                   u_int16_t statuscode,
                                   struct ieee80211_ba_parameterset *baparamset,
                                   u_int16_t batimeout
                                   )
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    scn->sc_ops->addba_response_process(scn->sc_dev,
                                        ATH_NODE_NET80211(ni)->an_sta,
                                        statuscode, baparamset, batimeout);
}

static void
ath_net80211_addba_clear(struct ieee80211_node *ni)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    scn->sc_ops->addba_clear(scn->sc_dev, ATH_NODE_NET80211(ni)->an_sta);
}

static void
ath_net80211_delba_process(struct ieee80211_node *ni,
                           struct ieee80211_delba_parameterset *delbaparamset,
                           u_int16_t reasoncode
                           )
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    scn->sc_ops->delba_process(scn->sc_dev, ATH_NODE_NET80211(ni)->an_sta,
                               delbaparamset, reasoncode);
}

static int
ath_net80211_addba_send(struct ieee80211_node *ni,
                        u_int8_t tidno,
                        u_int16_t buffersize)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    struct ieee80211_action_mgt_args actionargs;

    if (IEEE80211_NODE_USEAMPDU(ni) &&
        scn->sc_ops->check_aggr(scn->sc_dev, ATH_NODE_NET80211(ni)->an_sta, tidno)) {

        actionargs.category = IEEE80211_ACTION_CAT_BA;
        actionargs.action   = IEEE80211_ACTION_BA_ADDBA_REQUEST;
        actionargs.arg1     = tidno;
        actionargs.arg2     = buffersize;
        actionargs.arg3     = 0;

        ieee80211_send_action(ni, (void *)&actionargs);
        return 0;
    }

    return 1;
}

static void
ath_net80211_addba_status(struct ieee80211_node *ni, u_int8_t tidno, u_int16_t *status)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);

    *status = scn->sc_ops->addba_status(scn->sc_dev, ATH_NODE_NET80211(ni)->an_sta, tidno);
}

static void
ath_net80211_delba_send(struct ieee80211_node *ni,
                        u_int8_t tidno,
                        u_int8_t initiator,
                        u_int16_t reasoncode)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    struct ieee80211_action_mgt_args actionargs;

    /* tear down aggregation first */
    scn->sc_ops->aggr_teardown(scn->sc_dev, ATH_NODE_NET80211(ni)->an_sta, tidno, initiator);

    /* Send DELBA request */
    actionargs.category = IEEE80211_ACTION_CAT_BA;
    actionargs.action   = IEEE80211_ACTION_BA_DELBA;
    actionargs.arg1     = tidno;
    actionargs.arg2     = initiator;
    actionargs.arg3     = reasoncode;

    ieee80211_send_action(ni, (void *)&actionargs);
}

static void
ath_net80211_addba_setresponse(struct ieee80211_node *ni, u_int8_t tidno, u_int16_t statuscode)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);

    scn->sc_ops->set_addbaresponse(scn->sc_dev, ATH_NODE_NET80211(ni)->an_sta,
                                   tidno, statuscode); 
}

static void
ath_net80211_addba_clearresponse(struct ieee80211_node *ni)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);

    scn->sc_ops->clear_addbaresponsestatus(scn->sc_dev, ATH_NODE_NET80211(ni)->an_sta);
}

static int
ath_net80211_set_country(struct ieee80211com *ic, char *isoName)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    int error;

    error = scn->sc_ops->set_country(scn->sc_dev, isoName, 0 /* cc ignored */);
    if (!error)
        ieee80211_update_country_channellist(ic);

    return error;
}

static void
ath_net80211_set_divant(struct ieee80211com *ic, int divant_value)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);

    scn->sc_ops->set_divant(scn->sc_dev, divant_value);
}


static void
ath_net80211_get_currentCountry(struct ieee80211com *ic, IEEE80211_COUNTRY_ENTRY *ctry)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    scn->sc_ops->get_current_country(scn->sc_dev, (HAL_COUNTRY_ENTRY *)ctry);
}

static int
ath_net80211_set_regdomain(struct ieee80211com *ic, int regdomain)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    return scn->sc_ops->set_regdomain(scn->sc_dev, regdomain);
}

static int
ath_net80211_set_quiet(struct ieee80211_node *ni, u_int8_t *quiet_elm)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ni->ni_ic);
    struct ieee80211_quiet_ie *quiet = (struct ieee80211_quiet_ie *)quiet_elm;

    return scn->sc_ops->set_quiet(scn->sc_dev, 
                                  quiet->period,
                                  quiet->period,
                                  quiet->offset + quiet->tbttcount*ni->ni_intval,
                                  1);
}

static u_int16_t ath_net80211_find_countrycode(struct ieee80211com *ic, char* isoName)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);

    return scn->sc_ops->find_countrycode(scn->sc_dev, isoName);
}

static u_int
ath_net80211_mhz2ieee(struct ieee80211com *ic, u_int freq, u_int flags)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    return scn->sc_ops->mhz2ieee(scn->sc_dev, freq, flags);
}

static void
ath_net80211_clone_macaddr(struct ieee80211com *ic, u_int8_t* macaddr)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    scn->sc_ops->set_macaddr(scn->sc_dev, macaddr);
}

#ifdef ATH_SUPPORT_DFS
static int
ath_net80211_check_dfs_wait(struct ieee80211_node *ni)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);

    return scn->sc_ops->check_dfs_wait(scn->sc_dev);
}
#endif

/*------------------------------------------------------------
 * Callbacks for ath_dev module, which calls net80211 API's
 * (ieee80211_xxx) accordingly.
 *------------------------------------------------------------
 */

static void
ath_net80211_channel_setup(ieee80211_handle_t ieee,
                           enum ieee80211_clist_cmd cmd,
                           const HAL_CHANNEL *chans, int nchan,
                           const u_int8_t *regclassids, u_int nregclass,
                           int countrycode)
{
    struct ieee80211com *ic = NET80211_HANDLE(ieee);
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    struct ieee80211_channel_list cl;
    struct ieee80211_channel *ichan;
    const HAL_CHANNEL *c;
    int i, j;

    ieee80211com_get_channel_list(ic, &cl);

    if (cmd == CLIST_DFS_UPDATE) {
        for (i = 0; i < nchan; i++) {
            for (j = 0; j < cl.cl_nchans; j++) {
                ichan = &cl.cl_channels[j];
                if (chans[i].channel == ichan->ic_freq)
                    ichan->ic_flags &= ~IEEE80211_CHAN_RADAR;
            }

            ichan = ieee80211_find_channel(ic, chans[i].channel,
                                           chans[i].channelFlags);
            if (ichan != NULL)
                ichan->ic_flags &= ~IEEE80211_CHAN_RADAR;
        }

        return;
    }

    if (countrycode == CTRY_DEFAULT || cmd == CLIST_NEW_COUNTRY) {
        /*
         * Convert HAL channels to ieee80211 ones.
         */
        for (i = 0, ichan = cl.cl_channels; i < nchan; i++, ichan++) {
            c = &chans[i];
            ichan->ic_ieee = scn->sc_ops->mhz2ieee(scn->sc_dev, c->channel, c->channelFlags);
            ichan->ic_freq = c->channel;
            ichan->ic_flags = c->channelFlags;
            ichan->ic_flagext = (c->privFlags & (CHANNEL_DFS|CHANNEL_DFS_CLEAR));
            if (c->privFlags & CHANNEL_DISALLOW_ADHOC)
                ichan->ic_flagext |= IEEE80211_CHAN_DISALLOW_ADHOC;
            if (c->privFlags & CHANNEL_EDGE_CH) {
                ichan->ic_flagext |= IEEE80211_CHAN_EDGE_CH;
            }
            ichan->ic_maxregpower = c->maxRegTxPower;	/* dBm */
            ichan->ic_maxpower = c->maxTxPower / 4;		/* 1/4 dBm */
            ichan->ic_minpower = c->minTxPower / 4;		/* 1/4 dBm */
        }
        ieee80211com_set_nchannels(ic, nchan);
    }
    else {
        /*
         * Logic AND the country channel and domain channels.
         */
        for (i = 0, ichan = cl.cl_channels; i < cl.cl_nchans; i++, ichan++) {
            ichan->ic_flagext |= IEEE80211_CHAN_11D_EXCLUDED;
            c = chans;
            for (j = 0; j < nchan; j++) {
                if ((ichan->ic_freq == c->channel) &&
                   ((ichan->ic_flags & (~CHANNEL_PASSIVE)) == (c->channelFlags & (~CHANNEL_PASSIVE)))) {
                    ichan->ic_flags = c->channelFlags;
                    ichan->ic_flagext = (c->privFlags & (CHANNEL_DFS|CHANNEL_DFS_CLEAR));
                    if (c->privFlags & CHANNEL_DISALLOW_ADHOC)
                        ichan->ic_flagext |= IEEE80211_CHAN_DISALLOW_ADHOC;
                    if (c->privFlags & CHANNEL_EDGE_CH) {
                        ichan->ic_flagext |= IEEE80211_CHAN_EDGE_CH;
                    }
                    ichan->ic_maxregpower = c->maxRegTxPower;	/* dBm */
                    ichan->ic_maxpower = c->maxTxPower / 4;		/* 1/4 dBm */
                    ichan->ic_minpower = c->minTxPower / 4;		/* 1/4 dBm */
                    break;
                 }
                 c++;
            }
        }
    }

    /*
     * Copy regclass ids
     */
    if (nregclass >= IEEE80211_REGCLASSIDS_MAX)
        nregclass = IEEE80211_REGCLASSIDS_MAX;

    ic->ic_nregclass = nregclass;
    for (i = 0; i < nregclass; i++)
        ic->ic_regclassids[i] = regclassids[i];
        
}

static void
ath_net80211_update_txpow(ieee80211_handle_t ieee,
                          u_int16_t txpowlimit, u_int16_t txpowlevel)
{
    struct ieee80211com *ic = NET80211_HANDLE(ieee);
    struct ieee80211vap *vap;

    ieee80211com_set_txpowerlimit(ic, txpowlimit);
    
    ieee80211_enumerate_vaps(vap, ic) {
        struct ieee80211_node *ni;

        ni = ieee80211vap_get_bssnode(vap);
        ASSERT(ni);
        
        /* We might have no ni */
        if (ni)
            ieee80211node_set_txpower(ni, txpowlevel);
    }
}

static struct ieee80211vap *
ath_get_vap_by_idx(struct ieee80211com *ic, int if_id)
{
    struct ieee80211vap *vap = NULL;
    ieee80211_enumerate_vaps(vap, ic) {
        if ((ATH_VAP_NET80211(vap))->av_if_id == if_id)
            break;
    }
    return vap;
}

static void
ath_net80211_get_beaconconfig(ieee80211_handle_t ieee, int if_id,
                              ieee80211_beacon_config_t *conf)
{
    struct ieee80211com *ic = NET80211_HANDLE(ieee);
    struct ieee80211vap *vap = NULL;
    struct ieee80211_node *ni;

    if (if_id == ATH_IF_ID_ANY) {
        ieee80211_enumerate_vaps(vap, ic) {
            if (ieee80211vap_get_opmode(vap) == ic->ic_opmode) {
                break;
            }
        }
    } else {
        vap = ath_get_vap_by_idx(ic, if_id);
    }

    if (vap == NULL)
        return;

    /* fill in beacon config data */
    ni = vap->iv_bss;
    conf->beacon_interval = ni->ni_intval;
    conf->listen_interval = ni->ni_lintval;
    conf->dtim_period = ni->ni_dtim_period;
    conf->dtim_count = ni->ni_dtim_count;
#if 0
    conf->tim_offset = ni->ni_timoff;
#endif
    conf->bmiss_timeout = ic->ic_bmisstimeout;
    conf->u.last_tsf = ni->ni_tstamp.tsf;
}

static int16_t
ath_net80211_get_noisefloor(struct ieee80211com *ic, struct ieee80211_channel *chan)
{

    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);

    return scn->sc_ops->get_noisefloor(scn->sc_dev, chan->ic_freq,  ath_chan2flags(chan));
}

static void
ath_net80211_amsdu_enable(struct ieee80211com *ic, int enable)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);

    scn->sc_ops->ath_set_config_param(scn->sc_dev, ATH_PARAM_AMSDU_ENABLE, &enable);
}

static wbuf_t
ath_net80211_beacon_alloc(ieee80211_handle_t ieee, int if_id,
                          ieee80211_beacon_offset_t *bo,
                          ieee80211_tx_control_t *txctl)
{
#define USE_SHPREAMBLE(_ic)                                 \
    (ieee80211com_has_flag(_ic, IEEE80211_F_SHPREAMBLE) &&  \
     !ieee80211com_has_flag(_ic, IEEE80211_F_USEBARKER))
    struct ieee80211com *ic = NET80211_HANDLE(ieee);
    struct ieee80211vap *vap = NULL;
    struct ath_vap_net80211 *avn;
    struct ieee80211_node *ni;
    wbuf_t wbuf;

    ASSERT(if_id != ATH_IF_ID_ANY);

    vap = ath_get_vap_by_idx(ic, if_id);
    if (vap == NULL)
        return NULL;

    ni = vap->iv_bss;
    avn = ATH_VAP_NET80211(vap);
    wbuf = ieee80211_beacon_alloc(ni, &avn->av_beacon_offsets);
    if (wbuf == NULL)
        return NULL;

    /* set up tim offset */
    bo->bo_tim = avn->av_beacon_offsets.bo_tim;

    /* setup tx control block for this beacon */
    txctl->txpower = ni->ni_txpower;
    if (USE_SHPREAMBLE(ic))
        txctl->shortPreamble = 1;
    return wbuf;
#undef USE_SHPREAMBLE
}

static int
ath_net80211_beacon_update(ieee80211_handle_t ieee, int if_id,
                           ieee80211_beacon_offset_t *bo, wbuf_t wbuf,
                           int mcast)
{
    struct ieee80211com *ic = NET80211_HANDLE(ieee);
    struct ieee80211vap *vap = NULL;
    struct ath_vap_net80211 *avn;
    int error = 0;

    ASSERT(if_id != ATH_IF_ID_ANY);

    vap = ath_get_vap_by_idx(ic, if_id);
    if (vap == NULL)
        return -EINVAL;

    avn = ATH_VAP_NET80211(vap);
    error = ieee80211_beacon_update(vap->iv_bss, &avn->av_beacon_offsets,
                                    wbuf, mcast);
    if (!error) {
        /* set up tim offset */
        bo->bo_tim = avn->av_beacon_offsets.bo_tim;
    }
    
    return error;
}

static void
ath_net80211_beacon_miss(ieee80211_handle_t ieee)
{
    ieee80211_beacon_miss(NET80211_HANDLE(ieee));
}

static void
ath_net80211_proc_tim(ieee80211_handle_t ieee)
{
    ieee80211_pwrsave_proc_tim(NET80211_HANDLE(ieee));
}







/******************************************************************************/
/*!
**  \brief shim callback to set VAP state
** 
** This routine is used by DFS to change the state of the VAP after a CAC
** period, or when doing a channel change.  Required for layer seperation.
**
**  \param ieee     Pointer to shim structure (this)
**  \param if_id    VAP Index (1-4).  Zero is invalid.
**  \param state    Flag indicating INIT (0) or RUN (1) state
**
**  \return N/A
*/

static void
ath_net80211_set_vap_state(ieee80211_handle_t ieee,u_int8_t if_id, u_int8_t state)
{
    struct ieee80211com *ic = NET80211_HANDLE(ieee);
    struct ieee80211vap *vap = NULL;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    
    /*
    ** Code Begins
    ** Get the VAP ID from the index passed, but check the index for validity
    */
    
    if ( if_id < 4) {
        vap = ath_get_vap_by_idx(ic, if_id);
        
        if ( vap ) {
            /*
            ** Set the state based on the flag passed
            */
            
            /* Integrate bug fix for 36126. Change VAP state
               to SCAN or RUN depending on whether the VAP 
               started the DFS wait*/
            vap->iv_newstate(vap,state ? state : IEEE80211_S_INIT,0);
            
        } else
            DPRINTF(scn, ATH_DEBUG_BEACON_PROC, "%s: Unable to find VAP %d\n",__func__,if_id);
    }
}    


static int
ath_net80211_send_bar(ieee80211_node_t node, u_int8_t tidno, u_int16_t seqno)
{
    return ieee80211_send_bar((struct ieee80211_node *)node, tidno, seqno);
}

static void
ath_net80211_notify_qstatus(ieee80211_handle_t ieee, u_int16_t qdepth)
{
#ifdef ATH_SUPERG_FF
    struct ieee80211com *ic = NET80211_HANDLE(ieee);
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    struct ieee80211vap *vap = ieee80211com_first_vap(ic);
    struct ieee80211_node *ni = vap->iv_bss;
    int    ac;

    for (ac = WME_AC_BE; ac <= WME_AC_VO; ac++) {
        if (scn->sc_ops->txq_depth(scn->sc_dev, scn->sc_ac2q[ac]) < scn->sc_fftxqmin) {
            ath_ff_stageq_flush(ni, ac);
        }
    }
#endif

    ieee80211_notify_queue_status(NET80211_HANDLE(ieee), qdepth);
}

static INLINE void
ath_rxstat2ieee(struct ieee80211com *ic,
                ieee80211_rx_status_t *rx_status,
                struct ieee80211_rx_status *rs)
{
    rs->rs_flags =
        ((rx_status->flags & ATH_RX_FCS_ERROR) ? IEEE80211_RX_FCS_ERROR : 0) |
        ((rx_status->flags & ATH_RX_MIC_ERROR) ? IEEE80211_RX_MIC_ERROR : 0) |
        ((rx_status->flags & ATH_RX_DECRYPT_ERROR) ? IEEE80211_RX_DECRYPT_ERROR : 0) |
        ((rx_status->flags & ATH_RX_SM_ENABLE) ? IEEE80211_RX_SM_ENABLE : 0);

    rs->rs_phymode = ic->ic_curmode;
    rs->rs_freq = ic->ic_curchan->ic_freq;
    rs->rs_rssi = rx_status->rssi;
    rs->rs_abs_rssi = rx_status->abs_rssi;
    rs->rs_datarate = rx_status->rateKbps;
    rs->rs_tstamp.tsf = rx_status->tsf;
}

static int
ath_net80211_input(ieee80211_node_t node, wbuf_t wbuf, ieee80211_rx_status_t *rx_status)
{
    struct ieee80211_node *ni = (struct ieee80211_node *)node;
    struct ieee80211_rx_status rs;

    ath_rxstat2ieee(ni->ni_ic, rx_status, &rs);
    return ieee80211_input(ni, wbuf, &rs);
}

#ifdef ATH_SUPPORT_UAPSD
wbuf_t
ath_net80211_uapsd_allocqosnullframe(void)
{
    u_int8_t *frm;
    wbuf_t wbuf;

    wbuf = ieee80211_getmgtframe(&frm, 2);
    wbuf_push(wbuf, sizeof(struct ieee80211_qosframe));

    return wbuf;
}

wbuf_t
ath_net80211_uapsd_getqosnullframe(ieee80211_node_t node, wbuf_t wbuf, int ac)
{
    struct ieee80211_node *ni;

    ni = (struct ieee80211_node *)node;

    ieee80211_prepare_qosnulldata(ni, wbuf, ac);

    return wbuf;
}

/*
 * This function is called when we have successsfully transmitted EOSP.
 * It clears the SP flag so that we are ready to accept more triggers
 * from this node.
 */
void
ath_net80211_uapsd_eospindicate(ieee80211_node_t node, wbuf_t wbuf, int txok)
{
    struct ieee80211_qosframe *qwh;
    struct ieee80211_node *ni;

    qwh = (struct ieee80211_qosframe *)wbuf_header(wbuf);
    ni = (struct ieee80211_node *)node;

    if ((qwh->i_fc[0] == (IEEE80211_FC0_SUBTYPE_QOS|IEEE80211_FC0_TYPE_DATA)) ||
        (qwh->i_fc[0] == (IEEE80211_FC0_SUBTYPE_QOS_NULL|IEEE80211_FC0_TYPE_DATA)))
    {
        if (qwh->i_qos[0] & IEEE80211_QOS_EOSP) {
            ni->ni_flags &= ~IEEE80211_NODE_UAPSD_SP;
            if (!txok)
                ni->ni_stats.ns_tx_eosplost++;
        }
    }

    return;
}

/*
 * This function determines whether the received frame is a valid UAPSD trigger.
 * Called from interrupt context.
 */
void
ath_net80211_check_uapsdtrigger(ieee80211_handle_t ieee, wbuf_t wbuf, u_int16_t keyix)
{
    struct ieee80211com *ic = NET80211_HANDLE(ieee);
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    struct ieee80211_node *ni;
    struct ieee80211_qosframe *qwh;
    int tid, ac;
    u_int16_t frame_seq;
    int queue_depth;

    /*
     * Locate the node for sender
     */
    ni = (keyix != HAL_RXKEYIX_INVALID) ? scn->sc_keyixmap[keyix] : NULL;
    if (ni == NULL) {
        /*
         * No key index or no entry, do a lookup
         */
        ni = ieee80211_find_rxnode(ic, (struct ieee80211_frame_min *)
                                       wbuf_header(wbuf));
        if (ni == NULL) {
            return;
        }
    } else {
#ifdef NODE_FREE_DEBUG
        ieee80211_ref_node(ni, (char *)__func__);
#else
        ieee80211_ref_node(ni);
#endif
    }


    if (!(ni->ni_flags & IEEE80211_NODE_UAPSD))
        goto end;

    qwh = (struct ieee80211_qosframe *)wbuf_raw_data(wbuf);

    /*
     * Must deal with change of state here, since otherwise there would
     * be a race (on two quick frames from STA) between this code and the
     * tasklet where we would:
     *   - miss a trigger on entry to PS if we're already trigger hunting
     *   - generate spurious SP on exit (due to frame following exit frame)
     */
    if ((((qwh->i_fc[1] & IEEE80211_FC1_PWR_MGT) == IEEE80211_FC1_PWR_MGT) ^
        ((ni->ni_flags & IEEE80211_NODE_UAPSD_TRIG) == IEEE80211_NODE_UAPSD_TRIG)))
    {
        ni->ni_flags &= ~IEEE80211_NODE_UAPSD_SP;

        if (qwh->i_fc[1] & IEEE80211_FC1_PWR_MGT) {
            WME_UAPSD_NODE_TRIGSEQINIT(ni);
            ni->ni_stats.ns_uapsd_triggerenabled++;
            ni->ni_flags |= IEEE80211_NODE_UAPSD_TRIG;
        } else {
            /*
             * Node transitioned from UAPSD -> Active state. Flush out UAPSD frames
             */
            ni->ni_stats.ns_uapsd_active++;
            ni->ni_flags &= ~IEEE80211_NODE_UAPSD_TRIG;
            scn->sc_ops->process_uapsd_trigger(scn->sc_dev,
                                               ATH_NODE_NET80211(ni)->an_sta,
                                               WME_UAPSD_NODE_MAXQDEPTH, 0, 1);
        }

        goto end;
    }

    /*
     * Check for a valid trigger frame i.e. QoS Data or QoS NULL
     */
      if ( ((qwh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) !=
		      IEEE80211_FC0_TYPE_DATA ) ||
		     !(qwh->i_fc[0] & IEEE80211_FC0_SUBTYPE_QOS) ) {
          goto end;
      }

    if (((qwh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) == IEEE80211_FC0_TYPE_DATA) &&
       (((qwh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK) == IEEE80211_FC0_SUBTYPE_QOS) ||
        ((qwh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK) == IEEE80211_FC0_SUBTYPE_QOS_NULL)))
    {
        tid = qwh->i_qos[0] & IEEE80211_QOS_TID;
        ac = TID_TO_WME_AC(tid);

        if (WME_UAPSD_AC_CAN_TRIGGER(ac, ni)) {
            /*
             * Detect duplicate triggers and drop if so.
             */
            frame_seq = le16toh(*(u_int16_t *)qwh->i_seq);
            if ((qwh->i_fc[1] & IEEE80211_FC1_RETRY) &&
                frame_seq == ni->ni_uapsd_trigseq[ac])
            {
                ni->ni_stats.ns_uapsd_duptriggers++;
                goto end;
            }

            /*
             * SP in progress for this node, discard trigger.
             */
            if (ni->ni_flags & IEEE80211_NODE_UAPSD_SP) {
                ni->ni_stats.ns_uapsd_ignoretriggers++;
                goto end;
            }

            /* start the SP */
            ni->ni_stats.ns_uapsd_triggers++;
            ni->ni_flags |= IEEE80211_NODE_UAPSD_SP;
            ni->ni_uapsd_trigseq[ac] = frame_seq;

            queue_depth = scn->sc_ops->process_uapsd_trigger(scn->sc_dev,
                                               ATH_NODE_NET80211(ni)->an_sta,
                                               ni->ni_uapsd_maxsp, ac, 0);
            if (!queue_depth &&
                (ni->ni_vap->iv_set_tim != NULL) &&
                IEEE80211_NODE_UAPSD_USETIM(ni))
            {
                ni->ni_vap->iv_set_tim(ni, 0);
            }
        }
    }
end:
    ieee80211_free_node(ni);
    return;
}
#endif /*ATH_SUPPORT_UAPSD */
int
ath_net80211_rx(ieee80211_handle_t ieee, wbuf_t wbuf, ieee80211_rx_status_t *rx_status, u_int16_t keyix)
{
    struct ieee80211com *ic = NET80211_HANDLE(ieee);
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    struct ieee80211_node *ni;
    struct ieee80211vap *vap ;
    struct ieee80211_frame *wh;
    int type;

	int frame_type, frame_subtype;

    ATH_RX_TYPE status;
    struct ieee80211_qosframe_addr4      *whqos_4addr;
    int tid;

    if (ic->ic_opmode == IEEE80211_M_MONITOR) {
        /*
         * Monitor mode: discard anything shorter than
         * an ack or cts, clean the skbuff, fabricate
         * the Prism header existing tools expect,
         * and dispatch.
         */
        if (wbuf_get_pktlen(wbuf) < IEEE80211_ACK_LEN) {
            DPRINTF(scn, ATH_DEBUG_RECV,
                    "%s: runt packet %d\n", __func__, wbuf_get_pktlen(wbuf));
            wbuf_free(wbuf);
        } else {
            struct ieee80211_rx_status rs;
            ath_rxstat2ieee(ic, rx_status, &rs);
            ieee80211_input_monitor(ic, wbuf, &rs);
        }

        return -1;
    }
   
    /*
     * From this point on we assume the frame is at least
     * as large as ieee80211_frame_min; verify that.
     */
    if (wbuf_get_pktlen(wbuf) < IEEE80211_MIN_LEN) {
        DPRINTF(scn, ATH_DEBUG_RECV, "%s: short packet %d\n",
                    __func__, wbuf_get_pktlen(wbuf));
        wbuf_free(wbuf);
        return -1;
    }
    
    /*
     * Normal receive.
     */
    wbuf_trim(wbuf, IEEE80211_CRC_LEN);

    if (scn->sc_debug & ATH_DEBUG_RECV) {
        ieee80211_dump_pkt(ic, wbuf_header(wbuf), wbuf_get_pktlen(wbuf) + IEEE80211_CRC_LEN,
                           rx_status->rateKbps, rx_status->rssi);
    }




	wh = (struct ieee80211_frame *)wbuf_header(wbuf);
	frame_type = wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK;
	frame_subtype = wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK;
	
	/* 
	** check if this is a PROBE REQ
	** dup a PROBE REQ- since it may be for other VAPs also
	*/
    if ((frame_type == IEEE80211_FC0_TYPE_MGT) && ( frame_subtype == IEEE80211_FC0_SUBTYPE_PROBE_REQ))
	{
        struct ieee80211_rx_status rs;
	    ath_rxstat2ieee(ic, rx_status, &rs);
	    return ieee80211_input_all(ic, wbuf, &rs);
    }

    /*
     * Locate the node for sender, track state, and then
     * pass the (referenced) node up to the 802.11 layer
     * for its use.  If the sender is unknown spam the
     * frame; it'll be dropped where it's not wanted.
     */
    ni = (keyix != HAL_RXKEYIX_INVALID) ? scn->sc_keyixmap[keyix] : NULL;
    if (ni == NULL) {
        /*
         * No key index or no entry, do a lookup and
         * add the node to the mapping table if possible.
         */
        ni = ieee80211_find_rxnode(ic, (struct ieee80211_frame_min *)
                                   wbuf_header(wbuf));
        if (ni == NULL) {
            struct ieee80211_rx_status rs;
            ath_rxstat2ieee(ic, rx_status, &rs);
            return ieee80211_input_all(ic, wbuf, &rs);
        }

        /*
         * If the station has a key cache slot assigned
         * update the key->node mapping table.
         */
        keyix = ni->ni_ucastkey.wk_keyix;
        if (keyix != IEEE80211_KEYIX_NONE && scn->sc_keyixmap[keyix] == NULL) {
#ifdef NODE_FREE_DEBUG
            scn->sc_keyixmap[keyix] = ieee80211_ref_node(ni, (char *)__func__);
#else
            scn->sc_keyixmap[keyix] = ieee80211_ref_node(ni);
#endif
        }
    }
    else {
#ifdef NODE_FREE_DEBUG
        ieee80211_ref_node(ni, (char *)__func__);
#else
        ieee80211_ref_node(ni);
#endif
    }

    /* ni has been located. If this STA is sending an AUTH to another VAP
     * in the same AP, we should not refer this ni, instead we must spam to all
     * The correct VAP will respond while others will disacrd the AUTH req
     * This is placed here so that the ni stats are not updated and 
     * the  rx_proc_frame are not called
	 */
    if ((frame_type == IEEE80211_FC0_TYPE_MGT) &&
        ((frame_subtype == IEEE80211_FC0_SUBTYPE_AUTH) || (frame_subtype == IEEE80211_FC0_SUBTYPE_DEAUTH)) &&
        !IEEE80211_ADDR_EQ(ni->ni_bssid, wh->i_addr3))
    {
        u_int8_t mac1[IEEE80211_ADDR_LEN], mac2[IEEE80211_ADDR_LEN];

        /*
         * Copy node's bssid and third address of 
         * received packet.
         */
        IEEE80211_ADDR_COPY(mac1, ni->ni_bssid);
        IEEE80211_ADDR_COPY(mac2, wh->i_addr3);

        /*
         * Mask the copied addresses.
         */
        ATH_SET_VAP_BSSID_MASK(mac1);
        ATH_SET_VAP_BSSID_MASK(mac2);

        /*
         * WAR for HW Bug: Frames with incorrect DA and/or BSSID
         * not getting rejected.
         * Compare the BSSID, to ensure that the received frame
         * is destined to us, thereby avoiding corrupted packets.
         */
        if (IEEE80211_ADDR_EQ(mac1, mac2) && (ni->ni_vap->iv_opmode == IEEE80211_M_HOSTAP)) {
            struct ieee80211_rx_status rs;

            ath_rxstat2ieee(ic, rx_status, &rs);
	        /* dec refcnt just now incremented by find_rx_node() */
            ieee80211_free_node(ni);
            ieee80211_node_leave(ni);
            if (frame_subtype == IEEE80211_FC0_SUBTYPE_DEAUTH) {
                return IEEE80211_FC0_TYPE_MGT;
            } else {
                return ieee80211_input_all(ic, wbuf, &rs);
            }
        }
    }

    /*
     * update node statistics
     */
    if (IEEE80211_IS_DATA(wh)) {
        ATH_RATE_LPF(ATH_NODE_NET80211(ni)->an_avgrxrate, rx_status->rateKbps);

        if (rx_status->rateKbps > 0) {
		    vap= ni->ni_vap;
            ATH_NODE_NET80211(vap->iv_bss)->an_lastrxrate = rx_status->rateKbps;
            ATH_NODE_NET80211(ni)->an_lastrxrate = rx_status->rateKbps;
        }
    }

    if (rx_status->flags & ATH_RX_RSSI_VALID) {
        int i;
        ATH_RSSI_LPF(ATH_NODE_NET80211(ni)->an_avgrssi, rx_status->rssi);
        if (IEEE80211_IS_BEACON(wh)) {
            ATH_RSSI_LPF(ATH_NODE_NET80211(ni)->an_avgbrssi, rx_status->rssi);
        }

        if (rx_status->flags & ATH_RX_CHAIN_RSSI_VALID) {
            for(i=0;i<ATH_MAX_ANTENNA;++i) {
                ATH_RSSI_LPF(ATH_NODE_NET80211(ni)->an_avgchainrssi[i], rx_status->rssictl[i]);
            }
            if (rx_status->flags & ATH_RX_RSSI_EXTN_VALID) {
                for(i=0;i<ATH_MAX_ANTENNA;++i) {
                    ATH_RSSI_LPF(ATH_NODE_NET80211(ni)->an_avgchainrssiext[i], rx_status->rssiextn[i]);
                }
            }
        }
    }

    /*
     * Let ath_dev do some special rx frame processing. If the frame is not
     * consumed by ath_dev, indicate it up to the stack.
     */
    type = scn->sc_ops->rx_proc_frame(scn->sc_dev, ATH_NODE_NET80211(ni)->an_sta,
                                      IEEE80211_NODE_USEAMPDU(ni),
                                      wbuf, rx_status, &status);
 

    /* For OWL specific HW bug, 4addr aggr needs to be denied in 
    * some cases. So check for delba send and send delba
    */
    if ( (wh->i_fc[1] & IEEE80211_FC1_DIR_MASK) == IEEE80211_FC1_DIR_DSTODS ) {
        if ( IEEE80211NODE_WDSWAR_ISSENDDELBA(ni) ) {
	    whqos_4addr = (struct ieee80211_qosframe_addr4 *) wh;
            tid = whqos_4addr->i_qos[0] & IEEE80211_QOS_TID;
	    ath_net80211_delba_send(ni, tid, 0, IEEE80211_REASON_UNSPECIFIED);
	}
    }

    if (status != ATH_RX_CONSUMED) {
        /*
         * Not consumed by ath_dev for out-of-order delivery,
         * indicate up the stack now.
         */
        type = ath_net80211_input(ni, wbuf, rx_status);
    }
    
    ieee80211_free_node(ni);
    return type;
}

static void
ath_net80211_drain_amsdu(ieee80211_handle_t ieee)
{
#ifdef ATH_AMSDU
    struct ieee80211com *ic = NET80211_HANDLE(ieee);
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);

    ath_amsdu_tx_drain(scn);
#endif
}

static void
ath_net80211_sm_pwrsave_update(struct ieee80211_node *ni, int smen, int dyn,
	int ratechg)
{
	struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ni->ni_ic);
	ATH_SM_PWRSAV mode;

	if (smen) {
		mode = ATH_SM_ENABLE;
	} else {
		if (dyn) {
			mode = ATH_SM_PWRSAV_DYNAMIC;
		} else {
			mode = ATH_SM_PWRSAV_STATIC;
		}
	} 
	DPRINTF(((struct ath_softc *)scn->sc_dev), ATH_DEBUG_PWR_SAVE,
	    "%s: smen: %d, dyn: %d, ratechg: %d\n",
	    __func__, smen, dyn, ratechg);
	(scn->sc_ops->ath_sm_pwrsave_update)(scn->sc_dev,
	    ATH_NODE_NET80211(ni)->an_sta, mode, ratechg);
}


static void
ath_net80211_node_ps_update(struct ieee80211_node *ni, int pwrsave)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ni->ni_ic);
    (scn->sc_ops->update_node_pwrsave)(scn->sc_dev,
                                       ATH_NODE_NET80211(ni)->an_sta, pwrsave);
}

static u_int32_t
ath_net80211_node_gettxrate(struct ieee80211_node *ni)
{
    u_int32_t rate = (ATH_NODE_NET80211(ni)->an_lasttxrate);
    return rate;
}

static u_int32_t
ath_net80211_node_getrxrate(struct ieee80211_node *ni)
{
    u_int32_t rate = (ATH_NODE_NET80211(ni)->an_lastrxrate);
    return rate;
}

static void
ath_net80211_set_doth(ieee80211_handle_t ieee,int doth)
{
    struct ieee80211com *ic = NET80211_HANDLE(ieee);

	if(doth)
	    ieee80211com_set_flag(ic, IEEE80211_F_DOTH);
	else
	    ieee80211com_clear_flag(ic, IEEE80211_F_DOTH);
}

static void
ath_net80211_rate_setup(ieee80211_handle_t ieee, WIRELESS_MODE wMode,
                        RATE_TYPE type, const HAL_RATE_TABLE *rt)
{
    struct ieee80211com *ic = NET80211_HANDLE(ieee);
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    enum ieee80211_phymode mode = ath_mode_map[wMode];
    struct ieee80211_rateset *rs;
    int i, maxrates, rix;

    if (mode >= IEEE80211_MODE_MAX) {
        DPRINTF(scn, ATH_DEBUG_ANY, "%s: unsupported mode %u\n",
                __func__, mode);
        return;
    }
    
    if (rt->rateCount > IEEE80211_RATE_MAXSIZE) {
        DPRINTF(scn, ATH_DEBUG_ANY,
                "%s: rate table too small (%u > %u)\n",
                __func__, rt->rateCount, IEEE80211_RATE_MAXSIZE);
        maxrates = IEEE80211_RATE_MAXSIZE;
    } else {
        maxrates = rt->rateCount;
    }

    switch (type) {
    case NORMAL_RATE:
        rs = ieee80211com_get_rates(ic, mode);
        break;
    case HALF_RATE:
        rs = ieee80211com_get_half_rates(ic);
        break;
    case QUARTER_RATE:
        rs = ieee80211com_get_quarter_rates(ic);
        break;
    default:
        DPRINTF(scn, ATH_DEBUG_ANY, "%s: unknown rate type%u\n",
                __func__, type);
        return;
    }
    
    /* supported rates (non HT) */
    rix = 0;
    for (i = 0; i < maxrates; i++) {
        if ((rt->info[i].phy == IEEE80211_T_HT))
            continue;
        rs->rs_rates[rix++] = rt->info[i].dot11Rate;
    }
    rs->rs_nrates = (u_int8_t)rix;

    if ((mode == IEEE80211_MODE_11NA_HT20)     || (mode == IEEE80211_MODE_11NG_HT20)      ||
        (mode == IEEE80211_MODE_11NA_HT40PLUS) || (mode == IEEE80211_MODE_11NA_HT40MINUS) ||
        (mode == IEEE80211_MODE_11NG_HT40PLUS) || (mode == IEEE80211_MODE_11NG_HT40MINUS)) {
        /* supported rates (HT) */
        rix = 0;
        rs = ieee80211com_get_ht_rates(ic, mode);
        for (i = 0; i < maxrates; i++) {
            if (rt->info[i].phy == IEEE80211_T_HT) {
                rs->rs_rates[rix++] = rt->info[i].dot11Rate;
            }
        }
        rs->rs_nrates = (u_int8_t)rix;
    }
}

static void ath_net80211_update_txrate(ieee80211_node_t node, int txrate)
{
    struct ieee80211_node *ni = (struct ieee80211_node *)node;
    ni->ni_txrate = txrate;
}

static void ath_net80211_rate_node_update(ieee80211_handle_t ieee, ieee80211_node_t node, int isnew)
{
    struct ieee80211com *ic = NET80211_HANDLE(ieee);
    struct ieee80211_node *ni = (struct ieee80211_node *)node;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    struct ath_node_net80211 *an = (struct ath_node_net80211 *)ni;
    struct ieee80211vap *vap = ieee80211node_get_vap(ni);
    u_int32_t capflag = 0;

    if (ni->ni_flags & IEEE80211_NODE_HT) {
        capflag |=  ATH_RC_HT_FLAG;
        /*
         * Currently, only HT20 is supported in IBSS. To be able to connect
         * at HT20 mode, HT40 need to be disabled.
         */
        if (ieee80211vap_get_opmode(vap) != IEEE80211_M_IBSS) {
            if ((ni->ni_chwidth == IEEE80211_CWM_WIDTH40) && 
                (ic->ic_cwm.cw_width == IEEE80211_CWM_WIDTH40))
            {
                capflag |=  ATH_RC_CW40_FLAG;
            }
            if (ni->ni_htcap & IEEE80211_HTCAP_C_SHORTGI40) {
                capflag |=  ATH_RC_SGI_FLAG;
            }
        }

        /* Rx STBC is a 2-bit mask. Needs to convert from ieee definition to ath definition. */ 
        capflag |= (((ni->ni_htcap & IEEE80211_HTCAP_C_RXSTBC) >> IEEE80211_HTCAP_C_RXSTBC_S) 
                    << ATH_RC_RX_STBC_FLAG_S);

        switch (ni->ni_htcap & IEEE80211_HTCAP_C_SM_MASK) {
        case IEEE80211_HTCAP_C_SMPOWERSAVE_DYNAMIC:
        case IEEE80211_HTCAP_C_SM_ENABLED:
            {
                int val;
                scn->sc_ops->ath_get_config_param(scn->sc_dev, ATH_PARAM_TXCHAINMASK, &val);
                if (!ATH_IS_SINGLE_CHAIN(val) && (ni->ni_ath_flags & IEEE80211_NODE_HT_DS) && 
                    !(vap->iv_flags_ext & IEEE80211_FEXT_SMPS) &&
                    (scn->sc_ops->has_capability)(scn->sc_dev, ATH_CAP_DS)) {
                    capflag |= ATH_RC_DS_FLAG;
                }
            }
            break;
        case IEEE80211_HTCAP_C_SMPOWERSAVE_STATIC:
        default:
            break;
        }

        if (ni->ni_flags & IEEE80211_NODE_TKIPCIPHER) {

            capflag |= ATH_RC_WEP_TKIP_FLAG;
            if (ieee80211com_has_extflag(ic, IEEE80211_FEXT_WEP_TKIP_HTRATE)) {
                /* TKIP supported at HT rates */
                if (ni->ni_flags & IEEE80211_NODE_WEPTKIPAGGR) {
                    /* Pass proprietary rx delimiter count for tkip w/aggr to ath_dev */
                    scn->sc_ops->set_weptkip_rxdelim(scn->sc_dev, an->an_sta, ni->ni_weptkipaggr_rxdelim);
                } else {
                    /* Atheros proprietary wep/tkip aggregation mode is not supported */
                    ni->ni_flags |= IEEE80211_NODE_NOAMPDU;
                }
            } else {
                /* no TKIP support at HT rates => disable HT and aggregation */
                capflag &= ~ATH_RC_HT_FLAG;
                ni->ni_flags |= IEEE80211_NODE_NOAMPDU;
            }
        }
    }
    ((struct ath_node *)an->an_sta)->an_cap = capflag; 
    scn->sc_ops->ath_rate_newassoc(scn->sc_dev, an->an_sta, isnew, capflag,
                                   &ni->ni_rates, &ni->ni_htrates);
}

/* Iterator function */
static void
rate_cb(void *arg, struct ieee80211_node *ni)
{
    struct ieee80211com *ic = (struct ieee80211com *)arg;
    ath_net80211_rate_node_update(ic, ni, 1);
}

static void ath_net80211_rate_newstate(ieee80211_handle_t ieee, ieee80211_if_t if_data)
{
    struct ieee80211com *ic = NET80211_HANDLE(ieee);
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    struct ieee80211vap *vap = (struct ieee80211vap *)if_data;
    struct ieee80211_node* ni = ieee80211vap_get_bssnode(vap);
    u_int32_t capflag = 0;
    ath_node_t an;

    if (ieee80211vap_get_opmode(vap) != IEEE80211_M_STA) {
        /*
         * Sync rates for associated stations and neighbors.
         */
        ieee80211_iterate_node(ic, rate_cb, ic);

        if (ic->ic_cwm.cw_width == IEEE80211_CWM_WIDTH40) {
            capflag |= ATH_RC_CW40_FLAG;
        }
        if (IEEE80211_IS_CHAN_11N(ic->ic_bsschan)) {
            capflag |= ATH_RC_HT_FLAG;
            if ((scn->sc_ops->has_capability)(scn->sc_dev, ATH_CAP_DS)) {
                capflag |= ATH_RC_DS_FLAG;
            }
        }
    } else {
        if (ni && ni->ni_flags & IEEE80211_NODE_HT) {
            int tx_chainmask;
            capflag |= ATH_RC_HT_FLAG;
            scn->sc_ops->ath_get_config_param(scn->sc_dev, ATH_PARAM_TXCHAINMASK, &tx_chainmask);
            if (!ATH_IS_SINGLE_CHAIN(tx_chainmask) && (ni->ni_ath_flags & IEEE80211_NODE_HT_DS) &&
                !(vap->iv_flags_ext & IEEE80211_FEXT_SMPS) &&
                (scn->sc_ops->has_capability)(scn->sc_dev, ATH_CAP_DS))
                capflag |= ATH_RC_DS_FLAG;
        }

        if ((vap->iv_bss->ni_chwidth == IEEE80211_CWM_WIDTH40) && 
            (ic->ic_cwm.cw_width == IEEE80211_CWM_WIDTH40)) 
        {
            capflag |= ATH_RC_CW40_FLAG;
        }
    }
    if ((ni->ni_htcap & IEEE80211_HTCAP_C_SHORTGI40)) {
        capflag |= ATH_RC_SGI_FLAG;
    }

    /*
     * Currently, only HT20 is supported in IBSS. To be able to connect
     * at HT20 mode, HT40 need to be disabled.
     */
    if (ieee80211vap_get_opmode(vap) == IEEE80211_M_IBSS) {
        capflag &= ~(ATH_RC_CW40_FLAG|ATH_RC_SGI_FLAG);
    }

    /* Rx STBC is a 2-bit mask. Needs to convert from ieee definition to ath definition. */
    capflag |= (((ni->ni_htcap & IEEE80211_HTCAP_C_RXSTBC) >> IEEE80211_HTCAP_C_RXSTBC_S)
                << ATH_RC_RX_STBC_FLAG_S);

    an = ((struct ath_node_net80211 *)ni)->an_sta;
    scn->sc_ops->ath_rate_newassoc(scn->sc_dev, an, 1, capflag,
                                   &ni->ni_rates, &ni->ni_htrates);
}

#ifdef ATH_SUPPORT_IQUE

void ath_net80211_hbr_settrigger(ieee80211_node_t node, int signal)
{
	struct ieee80211_node *ni;
	ni = (struct ieee80211_node *)node;
	/* Node is associated, and not the AP self */
	if (ni && ni->ni_associd && ni != ni->ni_vap->iv_bss) {
		ieee80211_hbr_settrigger_byaddr(ni->ni_vap, ni->ni_macaddr, signal);
	}
}

int ath_net80211_hbr_getstate(ieee80211_node_t node)
{
	struct ieee80211_node *ni;
	int state;
	ni = (struct ieee80211_node *)node;
	/* Node is associated, and not the AP self */
	if (ni && ni->ni_associd && ni != ni->ni_vap->iv_bss) {
		state = ieee80211_hbr_reportstate_byaddr(ni->ni_vap, ni->ni_macaddr);
		return state;	
	}
	return -1;
}
#endif

struct ieee80211_ops net80211_ops = {
    ath_get_netif_settings,                 /* get_netif_settings   */
    ath_mcast_merge,                        /* netif_mcast_merge    */
    ath_net80211_channel_setup,             /* setup_channel_list   */
    ath_net80211_update_txpow,              /* update_txpow         */
    ath_net80211_get_beaconconfig,          /* get_beacon_config    */
    ath_net80211_beacon_alloc,              /* get_beacon           */
    ath_net80211_beacon_update,             /* update_beacon        */
    ath_net80211_beacon_miss,               /* notify_beacon_miss   */
    ath_net80211_proc_tim,                  /* proc_tim             */
    ath_net80211_send_bar,                  /* send_bar             */
    ath_net80211_notify_qstatus,            /* notify_txq_status    */
    ath_net80211_tx_complete,               /* tx_complete          */
    ath_net80211_tx_status,                 /* tx_status            */
    ath_net80211_rx,                        /* rx_indicate          */
    ath_net80211_input,                     /* rx_subframe          */
    ath_net80211_cwm_macmode,               /* cwm_macmode          */
    ath_net80211_dfs_test_return,           /* dfs_test_return      */
    ath_net80211_mark_dfs,                  /* mark_dfs             */
    ath_net80211_set_vap_state,             /* set_vap_state        */
    ath_net80211_change_channel,            /* change channel       */
    ath_net80211_switch_mode_static20,      /* change mode to static20 */
    ath_net80211_switch_mode_dynamic2040,   /* change mode to dynamic2040 */
	ath_net80211_set_doth,
    ath_net80211_rate_setup,                /* setup_rate */
    ath_net80211_update_txrate,             /* update_txrate */
    ath_net80211_rate_newstate,             /* rate_newstate */
    ath_net80211_rate_node_update,          /* rate_node_update */
    ath_net80211_drain_amsdu,               /* drain_amsdu */
#ifdef ATH_SUPPORT_UAPSD
    ath_net80211_check_uapsdtrigger,        /* check_uapsdtrigger */
    ath_net80211_uapsd_eospindicate,        /* uapsd_eospindicate */
    ath_net80211_uapsd_allocqosnullframe,   /* uapsd_allocqosnull */
    ath_net80211_uapsd_getqosnullframe,     /* uapsd_getqosnullframe */
#endif

#ifdef ATH_SUPPORT_IQUE
	ath_net80211_hbr_settrigger,			/* hbr_settrigger */
	ath_net80211_hbr_getstate,				/* hbr_getstate */
#endif
    ath_net80211_get_macaddr,
#ifdef NODE_FREE_DEBUG
    ath_net80211_node_print,
    ath_net80211_add_trace,
#endif
};

u_int8_t *
ath_net80211_get_macaddr(ieee80211_node_t node)
{
    struct ieee80211_node *ni;
    ni = (struct ieee80211_node *)node;

    return ni->ni_macaddr;
}

#ifdef NODE_FREE_DEBUG
void
ath_net80211_add_trace(ieee80211_node_t node, char *funcp, char *descp,
                       u_int64_t value)
{
    ieee80211_add_trace(node, funcp, descp, value);
}

void
ath_net80211_node_print(ieee80211_node_t node)
{
    struct ieee80211_node *ni;
    ni = (struct ieee80211_node *)node;

    ieee80211_print_trace(ni); 
}     
#endif

void ath_net80211_switch_mode_dynamic2040(ieee80211_handle_t ieee)
{
    struct ieee80211com *ic = NET80211_HANDLE(ieee);
    ath_cwm_switch_mode_dynamic2040(ic);
}

void
ath_net80211_switch_mode_static20(ieee80211_handle_t ieee)
{
    struct ieee80211com *ic = NET80211_HANDLE(ieee);
    ath_cwm_switch_mode_static20(ic);
}


void
ath_net80211_dfs_test_return(ieee80211_handle_t ieee, u_int8_t ieeeChan)
{
    struct ieee80211com *ic = NET80211_HANDLE(ieee);
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ieee);

    /* Return to the original channel we were on before the test mute */
    DPRINTF(scn, ATH_DEBUG_ANY, "Returning to channel %d\n", ieeeChan);
    ic->ic_chanchange_chan = ieeeChan;
    ic->ic_chanchange_tbtt = IEEE80211_RADAR_11HCOUNT;
    ic->ic_flags |= IEEE80211_F_CHANSWITCH;
}

void        
ath_net80211_mark_dfs(ieee80211_handle_t ieee, struct ieee80211_channel *ichan)
{
    struct ieee80211com *ic = NET80211_HANDLE(ieee);
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ieee);

    DPRINTF(scn, ATH_DEBUG_ANY, "%s : Radar found on channel %d (%d MHz)\n", __func__, ichan->ic_ieee, ichan->ic_freq);
    ieee80211_mark_dfs(ic, ichan);
}

void
ath_net80211_change_channel(ieee80211_handle_t ieee, struct ieee80211_channel *chan)
{
    struct ieee80211com *ic = NET80211_HANDLE(ieee);
    ic->ic_curchan = chan;
    ic->ic_set_channel(ic);
}

void
ath_setTxPowerLimit(struct ieee80211com *ic, u_int32_t limit, u_int16_t tpcInDb)
{
    struct ath_softc_net80211    *scn = ATH_SOFTC_NET80211(ic);

    scn->sc_ops->ath_set_txPwrLimit(scn->sc_dev, limit, tpcInDb);
}

u_int8_t
ath_net80211_get_common_power(struct ieee80211com *ic, struct ieee80211_channel *chan)
{
    struct ath_softc_net80211    *scn = ATH_SOFTC_NET80211(ic);

    return scn->sc_ops->get_common_power(chan->ic_freq);
}
    
#ifdef ATH_CCX
int
ath_getrmcounters(struct ieee80211com *ic, struct ath_mib_cycle_cnts *pCnts)
{
    struct ath_softc_net80211    *scn = ATH_SOFTC_NET80211(ic);

    return (scn->sc_ops->ath_get_mibCycleCounts(scn->sc_dev, pCnts));
}

void
ath_clearrmcounters(struct ieee80211com *ic)
{
    struct ath_softc_net80211    *scn = ATH_SOFTC_NET80211(ic);

    scn->sc_ops->ath_clear_mibCounters(scn->sc_dev);
}

int
ath_updatermcounters(struct ieee80211com *ic, struct ath_mib_mac_stats *pStats)
{
    struct ath_softc_net80211    *scn = ATH_SOFTC_NET80211(ic);

    scn->sc_ops->ath_update_mibMacstats(scn->sc_dev);
    return scn->sc_ops->ath_get_mibMacstats(scn->sc_dev, pStats);
}
   
int
ath_getmibmaccounters(struct ieee80211com *ic, struct ath_mib_mac_stats* pStats)
{
    struct ath_softc_net80211    *scn = ATH_SOFTC_NET80211(ic);

    return scn->sc_ops->ath_get_mibMacstats(scn->sc_dev, pStats);
}

u_int8_t
ath_net80211_rcRateValueToPer(struct ieee80211com *ic, struct ieee80211_node *ni, int txRateKbps)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    return (scn->sc_ops->rcRateValueToPer(scn->sc_dev, (struct ath_node *)(ATH_NODE_NET80211(ni)->an_sta),
            txRateKbps));
}

u_int32_t
ath_getTSF32(struct ieee80211com *ic)
{
    struct ath_softc_net80211    *scn = ATH_SOFTC_NET80211(ic);

    return scn->sc_ops->ath_get_tsf32(scn->sc_dev);
}

u_int64_t
ath_getTSF64(struct ieee80211com *ic)
{
    struct ath_softc_net80211    *scn = ATH_SOFTC_NET80211(ic);

    return scn->sc_ops->ath_get_tsf64(scn->sc_dev);
}

void
ath_setReceiveFilter(struct ieee80211com *ic)
{
    struct ath_softc_net80211    *scn = ATH_SOFTC_NET80211(ic);

    scn->sc_ops->ath_set_rxfilter(scn->sc_dev);
}

int
ath_getMfgSerNum(struct ieee80211com *ic, u_int8_t *pSrn)
{
    struct ath_softc_net80211    *scn = ATH_SOFTC_NET80211(ic);

    return scn->sc_ops->ath_get_sernum(scn->sc_dev, pSrn);
}

int
ath_net80211_get_chanData(struct ieee80211com *ic, struct ieee80211_channel *pChan, struct ath_chan_data *pData)
{
    struct ath_softc_net80211    *scn = ATH_SOFTC_NET80211(ic);

    return scn->sc_ops->ath_get_chandata(scn->sc_dev, pChan, pData);

}

u_int32_t
ath_net80211_get_curRSSI(struct ieee80211com *ic)
{
    struct ath_softc_net80211    *scn = ATH_SOFTC_NET80211(ic);

    return scn->sc_ops->ath_get_curRSSI(scn->sc_dev);
}
#endif

#ifdef ATH_SWRETRY
/* Interface function for the IEEE layer to manipulate
 * the software retry state. Used during BMISS and 
 * scanning state machine in IEEE layer
 */
void 
ath_net80211_set_swretrystate(struct ieee80211_node *ni, int flag)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    ath_node_t node = ATH_NODE_NET80211(ni)->an_sta;

    scn->sc_ops->set_swretrystate(scn->sc_dev, node, flag);
}
#endif

void
ath_net80211_printreg(struct ieee80211com *ic, u_int32_t printctrl)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    scn->sc_ops->ath_printreg(scn->sc_dev, printctrl);
}

u_int32_t
ath_net80211_wpsPushButton(struct ieee80211com *ic)
{
    struct ath_softc_net80211  *scn = ATH_SOFTC_NET80211(ic);
    struct ath_ops             *ops = scn->sc_ops;

    return ((ops->has_capability)(scn->sc_dev, ATH_CAP_WPS_BUTTON));
}

#ifdef ATH_SUPPORT_IQUE
void
ath_net80211_set_acparams(struct ieee80211com *ic, u_int8_t ac, u_int8_t use_rts,
                          u_int8_t aggrsize_scaling, u_int32_t min_kbps)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);

    scn->sc_ops->ath_set_acparams(scn->sc_dev, ac, use_rts, aggrsize_scaling, min_kbps);
}

void
ath_net80211_set_rtparams(struct ieee80211com *ic, u_int8_t ac, u_int8_t perThresh,
                          u_int8_t probeInterval)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);

    scn->sc_ops->ath_set_rtparams(scn->sc_dev, ac, perThresh, probeInterval);
}

void
ath_net80211_get_iqueconfig(struct ieee80211com *ic)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);

    scn->sc_ops->ath_get_iqueconfig(scn->sc_dev);
}

void
ath_net80211_set_hbrparams(struct ieee80211vap *iv, u_int8_t ac, u_int8_t enable, u_int8_t per)
{
	struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(iv->iv_ic);

	scn->sc_ops->ath_set_hbrparams(scn->sc_dev, ac, enable, per);
	/* Send ACTIVE signal to all nodes. Otherwise, if the hbr_enable is turned off when
	 * one state machine is in BLOCKING or PROBING state, the ratecontrol module 
	 * will never send ACTIVE signals after hbr_enable is turned off, therefore
	 * the state machine will stay in the PROBING state forever 
	 */
	/* TODO ieee80211_hbr_setstate_all(iv, HBR_SIGNAL_ACTIVE); */
}
#endif
u_int32_t
ath_net80211_getmfpsupport(struct ieee80211com *ic)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    return(scn->sc_ops->ath_get_mfpsupport(scn->sc_dev));
}



int
ath_attach(u_int16_t devid, void *base_addr,
           struct ath_softc_net80211 *scn,
           osdev_t osdev, struct ath_reg_parm *ath_conf_parm,
           struct hal_reg_parm *hal_conf_parm)
{
    ath_dev_t               dev;
    struct ath_ops          *ops;
    struct ieee80211com     *ic;
    int                     error;
    int                     weptkipaggr_rxdelim = 0;

    scn->sc_osdev = osdev;
    scn->sc_debug = DBG_DEFAULT;
    ic = &scn->sc_ic;
    /*
     * Create an Atheros Device object
     */
    error = ath_dev_attach(devid, base_addr,
                           ic, &net80211_ops, osdev,
                           &scn->sc_dev, &scn->sc_ops,
                           ath_conf_parm, hal_conf_parm);
    if (error != 0)
        return error;

    dev = scn->sc_dev;
    ops = scn->sc_ops;

    /* attach channel width management */
    error = ath_cwm_attach(scn, ath_conf_parm);
    if (error) {
        ath_dev_free(dev);
        return error;
    }

#ifdef ATH_AMSDU
    /* attach amsdu transmit handler */
    ath_amsdu_attach(scn);
#endif
#ifndef ATH_WLAN_COMBINE
    ath_limit_legacy_attach(scn);
#endif /* #ifndef ATH_WLAN_COMBINE */

    /* setup ieee80211 flags */
    ieee80211com_clear_cap(ic, -1);
    ieee80211com_clear_athcap(ic, -1);
    ieee80211com_clear_athextcap(ic, -1);

    /* XXX not right but it's not used anywhere important */
    ieee80211com_set_phytype(ic, IEEE80211_T_OFDM);

    /* 
     * Set the Atheros Advanced Capabilities from station config before 
     * starting 802.11 state machine.
     */
#ifdef ATH_SUPERG_FF
    ieee80211com_set_athcap(ic, (ops->has_capability(dev, ATH_CAP_FF) ? IEEE80211_ATHC_FF : 0));
    scn->sc_fftxqmin = ATH_FF_TXQMIN;
#endif
    ieee80211com_set_athcap(ic, ((ops->has_capability)(dev, ATH_CAP_BURST) ? IEEE80211_ATHC_BURST : 0));

#ifdef ATH_SUPERG_COMP
    ieee80211com_set_athcap(ic, ((ops->has_capability)(dev, ATH_CAP_COMPRESSION) ? IEEE80211_ATHC_COMP : 0)); 
#endif

#ifdef ATH_SUPERG_DYNTURBO
    ieee80211com_set_athcap(ic, ((ops->has_capability)(dev, ATH_CAP_TURBO) ?
                                 (IEEE80211_ATHC_TURBOP | IEEE80211_ATHC_AR):0));
#endif
#ifdef ATH_SUPERG_XR
    ieee80211com_set_athcap(ic, ((ops->has_capability)(dev, ATH_CAP_XR) ? IEEE80211_ATHC_XR : 0));
#endif

    /* Set Atheros Extended Capabilities */
    ieee80211com_set_athextcap(ic,
        (((ops->has_capability)(dev, ATH_CAP_HT) &&
          !(ops->has_capability)(dev, ATH_CAP_4ADDR_AGGR))
         ? IEEE80211_ATHEC_OWLWDSWAR : 0));
        ieee80211com_set_athextcap(ic,
	    (((ops->has_capability)(dev, ATH_CAP_HT) &&
	      (ops->has_capability)(dev, ATH_CAP_WEP_TKIP_AGGR))
	    ? IEEE80211_ATHEC_WEPTKIPAGGR : 0));
        ieee80211com_set_athextcap(ic,
	    (((ops->has_capability)(dev, ATH_CAP_HT) &&
	      (ops->has_capability)(dev, ATH_CAP_SINGLE_AGGR_SAFE))
	    	? IEEE80211_ATHEC_SINGLEAGGRSAFE : 0));

    ieee80211com_set_cap(ic,
                         IEEE80211_C_IBSS           /* ibss, nee adhoc, mode */
                         | IEEE80211_C_HOSTAP       /* hostap mode */
                         | IEEE80211_C_MONITOR      /* monitor mode */
                         | IEEE80211_C_SHPREAMBLE   /* short preamble supported */
                         | IEEE80211_C_SHSLOT       /* short slot time supported */
                         | IEEE80211_C_WPA          /* capable of WPA1+WPA2 */
                         | IEEE80211_C_BGSCAN       /* capable of bg scanning */
        );

    /*
     * WMM enable
     */
    if ((ops->has_capability)(dev, ATH_CAP_WMM))
        ieee80211com_set_cap(ic, IEEE80211_C_WME);

    /* set up WMM AC to h/w qnum mapping */
    scn->sc_ac2q[WME_AC_BE] = ops->tx_get_qnum(dev, HAL_TX_QUEUE_DATA, HAL_WME_AC_BE);
    scn->sc_ac2q[WME_AC_BK] = ops->tx_get_qnum(dev, HAL_TX_QUEUE_DATA, HAL_WME_AC_BK);
    scn->sc_ac2q[WME_AC_VI] = ops->tx_get_qnum(dev, HAL_TX_QUEUE_DATA, HAL_WME_AC_VI);
    scn->sc_ac2q[WME_AC_VO] = ops->tx_get_qnum(dev, HAL_TX_QUEUE_DATA, HAL_WME_AC_VO);
    scn->sc_beacon_qnum = ops->tx_get_qnum(dev, HAL_TX_QUEUE_BEACON, 0);
#ifdef ATH_SUPPORT_UAPSD
    scn->sc_uapsd_qnum = ops->tx_get_qnum(dev, HAL_TX_QUEUE_UAPSD, 0);

    /*
     * UAPSD capable
     */
    if ((ops->has_capability)(dev, ATH_CAP_UAPSD)) {
        ieee80211com_set_cap(ic, IEEE80211_C_UAPSD);
        IEEE80211_COM_UAPSD_ENABLE(ic);
    }
#endif

    /*
     * Query the hardware to figure out h/w crypto support.
     */
    if (ops->has_cipher(dev, HAL_CIPHER_WEP))
        ieee80211com_set_cap(ic, IEEE80211_C_WEP);
    if (ops->has_cipher(dev, HAL_CIPHER_AES_OCB))
        ieee80211com_set_cap(ic, IEEE80211_C_AES);
    if (ops->has_cipher(dev, HAL_CIPHER_AES_CCM))
        ieee80211com_set_cap(ic, IEEE80211_C_AES_CCM);
    if (ops->has_cipher(dev, HAL_CIPHER_CKIP))
        ieee80211com_set_cap(ic, IEEE80211_C_CKIP);
    if (ops->has_cipher(dev, HAL_CIPHER_TKIP)) {
        ieee80211com_set_cap(ic, IEEE80211_C_TKIP);

        /* Check if h/w does the MIC. */
        if (ops->has_cipher(dev, HAL_CIPHER_MIC)) {
            ieee80211com_set_cap(ic, IEEE80211_C_TKIPMIC);
            /*
             * Check if h/w does MIC correctly when
             * WMM is turned on.  If not, then disallow WMM.
             */
            if ((ops->has_capability)(dev, ATH_CAP_TKIP_WMEMIC)) {
                ieee80211com_set_cap(ic, IEEE80211_C_WME_TKIPMIC);
            } else {
                ieee80211com_clear_cap(ic, IEEE80211_C_WME);
            }

            /*
             * Check whether the separate key cache entries
             * are required to handle both tx+rx MIC keys.
             * With split mic keys the number of stations is limited
             * to 27 otherwise 59.
             */
            if ((ops->has_capability)(dev, ATH_CAP_TKIP_SPLITMIC))
                scn->sc_splitmic = 1;
                DPRINTF(scn, ATH_DEBUG_KEYCACHE, "%s\n", __func__);
        }
    }

    if ((ops->has_capability)(dev, ATH_CAP_MCAST_KEYSEARCH))
        scn->sc_mcastkey = 1;

    /* TPC enabled */
    if ((ops->has_capability)(dev, ATH_CAP_TXPOWER))
        ieee80211com_set_cap(ic, IEEE80211_C_TXPMGT);

    /*
     * Default 11.h to disabled.
     */
    ieee80211com_clear_flag(ic, IEEE80211_F_DOTH);

    /* 11n Capabilities */
    ieee80211com_clear_htcap(ic, -1);
    ieee80211com_clear_htextcap(ic, -1);
    if ((ops->has_capability)(dev, ATH_CAP_HT)) {
        ieee80211com_set_cap(ic, IEEE80211_C_HT);
        ieee80211com_set_htcap(ic, IEEE80211_HTCAP_C_SHORTGI40
                        | IEEE80211_HTCAP_C_CHWIDTH40
                        | IEEE80211_HTCAP_C_DSSSCCK40);
        if ((ops->has_capability)(dev, ATH_CAP_DYNAMIC_SMPS)) {
            ieee80211com_set_htcap(ic, IEEE80211_HTCAP_C_SMPOWERSAVE_DYNAMIC);
        } else {
            ieee80211com_set_htcap(ic, IEEE80211_HTCAP_C_SM_ENABLED);
        }
        ieee80211com_set_htextcap(ic, IEEE80211_HTCAP_EXTC_TRANS_TIME_5000
                        | IEEE80211_HTCAP_EXTC_MCS_FEEDBACK_NONE);
        ieee80211com_set_maxampdu(ic, IEEE80211_HTCAP_MAXRXAMPDU_65536);
        ieee80211com_set_mpdudensity(ic, IEEE80211_HTCAP_MPDUDENSITY_8);
        ieee80211com_set_extflag(ic, IEEE80211_FEXT_AMPDU);



        if (!scn->sc_ops->ath_get_config_param(scn->sc_dev, ATH_PARAM_WEP_TKIP_AGGR_RX_DELIM,
                                           &weptkipaggr_rxdelim)) {
            ieee80211com_set_weptkipaggr_rxdelim(ic, (u_int8_t) weptkipaggr_rxdelim);
        }

        //ieee80211com_set_ampdu_limit(ic, ath_configuration_parameters.aggrLimit);
        //ieee80211com_set_ampdu_subframes(ic, ath_configuration_parameters.aggrSubframes);
    }

    if ((ops->has_capability)(dev, ATH_CAP_TX_STBC)) {
        ieee80211com_set_htcap(ic, IEEE80211_HTCAP_C_TXSTBC);
    }

    /* Rx STBC is a 2-bit mask. Needs to convert from ath definition to ieee definition. */ 
    ieee80211com_set_htcap(ic, IEEE80211_HTCAP_C_RXSTBC &
                           ((ops->has_capability)(dev, ATH_CAP_RX_STBC) << IEEE80211_HTCAP_C_RXSTBC_S));

    /* 11n configuration */
    ieee80211com_clear_htflags(ic, -1);
    if (ieee80211com_has_htcap(ic, IEEE80211_HTCAP_C_SHORTGI40)) {
	    ieee80211com_set_htflags(ic, IEEE80211_HTF_SHORTGI);
    }

    /*
     * Check for misc other capabilities.
     */
    if ((ops->has_capability)(dev, ATH_CAP_BURST))
        ieee80211com_set_cap(ic, IEEE80211_C_BURST);

    /*
     * Indicate we need the 802.11 header padded to a
     * 32-bit boundary for 4-address and QoS frames.
     */
    ieee80211com_set_flag(ic, IEEE80211_F_DATAPAD);

    /* get mac address from hardware */
    ops->get_macaddr(dev, ic->ic_myaddr);
    
#ifdef USE_HEADERLEN_RESV
    dev->hard_header_len += sizeof (struct ieee80211_qosframe) +
        sizeof(struct llc) + IEEE80211_ADDR_LEN + IEEE80211_WEP_IVLEN +
        IEEE80211_WEP_KIDLEN;
#ifdef ATH_SUPERG_FF
    dev->hard_header_len += ATH_FF_MAX_HDR;
#endif
#endif

    /* get default country info for 11d */
    ops->get_current_country(dev, (HAL_COUNTRY_ENTRY *)&ic->ic_country);
    
    /*
     * Setup some ieee80211com methods
     */
    ic->ic_mgtstart = ath_tx_mgt_send;
    ic->ic_init = ath_init;
    ic->ic_reset = ath_net80211_reset;
    ic->ic_newassoc = ath_net80211_newassoc;
    ic->ic_updateslot = ath_net80211_updateslot;

    ic->ic_wme.wme_update = ath_wmm_update;
    
    ic->ic_get_currentCountry = ath_net80211_get_currentCountry;
    ic->ic_set_country = ath_net80211_set_country;
    ic->ic_set_divant = ath_net80211_set_divant;
    ic->ic_set_regdomain = ath_net80211_set_regdomain;
    ic->ic_set_quiet = ath_net80211_set_quiet;
    ic->ic_find_countrycode = ath_net80211_find_countrycode;

    ic->ic_beacon_update = ath_beacon_update;
    ic->ic_txq_depth = ath_net80211_txq_depth;

    ic->ic_chwidth_change = ath_chwidth_change;
    ic->ic_sm_pwrsave_update = ath_net80211_sm_pwrsave_update;
    ic->ic_update_protmode = ath_net80211_update_protmode;
    ic->ic_set_config = ath_net80211_set_config;
    ic->ic_bss_to40 = ath_cwm_switch_to40;
    ic->ic_bss_to20 = ath_cwm_switch_to20;


    /*
     * Attach ieee80211com object to net80211 protocal stack.
     */
    ieee80211_ifattach(ic);

    /*
     * Override default methods
     */
    ic->ic_vap_create = ath_vap_create;
    ic->ic_vap_delete = ath_vap_delete;
    ic->ic_node_alloc = ath_net80211_node_alloc;
    scn->sc_node_free = ic->ic_node_free;
    ic->ic_node_free = ath_net80211_node_free;
    ic->ic_node_getrssi = ath_net80211_node_getrssi;
    ic->ic_node_psupdate = ath_net80211_node_ps_update;
    scn->sc_node_cleanup = ic->ic_node_cleanup;
    ic->ic_node_cleanup = ath_net80211_node_cleanup;

    ic->ic_scan_start = ath_net80211_scan_start;
    ic->ic_scan_end = ath_net80211_scan_end;
    ic->ic_led_scan_start = ath_net80211_led_enter_scan;
    ic->ic_led_scan_end = ath_net80211_led_leave_scan;
    ic->ic_set_channel = ath_net80211_set_channel;

    ic->ic_pwrsave_set_state = ath_net80211_pwrsave_set_state;
    
    ic->ic_mhz2ieee = ath_net80211_mhz2ieee;

    ic->ic_set_ampduparams = ath_net80211_set_ampduparams;
    ic->ic_set_weptkip_rxdelim = ath_net80211_set_weptkip_rxdelim;

    ic->ic_addba_requestsetup = ath_net80211_addba_requestsetup;
    ic->ic_addba_responsesetup = ath_net80211_addba_responsesetup;
    ic->ic_addba_requestprocess = ath_net80211_addba_requestprocess;
    ic->ic_addba_responseprocess = ath_net80211_addba_responseprocess;
    ic->ic_addba_clear = ath_net80211_addba_clear;
    ic->ic_delba_process = ath_net80211_delba_process;
    ic->ic_addba_send = ath_net80211_addba_send;
    ic->ic_addba_status = ath_net80211_addba_status;
    ic->ic_delba_send = ath_net80211_delba_send;
    ic->ic_addba_setresponse = ath_net80211_addba_setresponse;
    ic->ic_addba_clearresponse = ath_net80211_addba_clearresponse;
    ic->ic_get_noisefloor = ath_net80211_get_noisefloor;
    ic->ic_amsdu_enable = ath_net80211_amsdu_enable;
    ic->ic_set_txPowerLimit = ath_setTxPowerLimit;
    ic->ic_get_common_power = ath_net80211_get_common_power;
#ifdef ATH_CCX
    ic->ic_rmgetcounters = ath_getrmcounters;
    ic->ic_rmclearcounters = ath_clearrmcounters;
    ic->ic_rmupdatecounters = ath_updatermcounters;
    ic->ic_getmibmaccounters = ath_getmibmaccounters;
    ic->ic_rcRateValueToPer = ath_net80211_rcRateValueToPer;
    ic->ic_get_TSF32        = ath_getTSF32;
    ic->ic_get_TSF64        = ath_getTSF64;
    ic->ic_set_rxfilter     = ath_setReceiveFilter;
    ic->ic_get_mfgsernum    = ath_getMfgSerNum;
    ic->ic_get_chandata     = ath_net80211_get_chanData;
    ic->ic_get_curRSSI      = ath_net80211_get_curRSSI;
#endif
#ifdef ATH_SWRETRY
    ic->ic_set_swretrystate = ath_net80211_set_swretrystate;
#endif
    ic->ic_clone_macaddr = ath_net80211_clone_macaddr;
#ifdef ATH_SUPPORT_IQUE
    ic->ic_set_acparams = ath_net80211_set_acparams;
    ic->ic_set_rtparams = ath_net80211_set_rtparams;
    ic->ic_get_iqueconfig = ath_net80211_get_iqueconfig;
	ic->ic_set_hbrparams = ath_net80211_set_hbrparams;
#endif

    ic->ic_printreg = ath_net80211_printreg;
    ic->ic_get_wpsPushButton = ath_net80211_wpsPushButton;
    ic->ic_get_mfpsupport = ath_net80211_getmfpsupport;
    ic->ic_gettxrate = ath_net80211_node_gettxrate;
    ic->ic_getrxrate = ath_net80211_node_getrxrate;

#ifdef ATH_SUPPORT_DFS
    ic->ic_check_dfs_wait = ath_net80211_check_dfs_wait;
#endif
    return 0;
}

int
ath_detach(struct ath_softc_net80211 *scn)
{
    struct ieee80211com *ic = &scn->sc_ic;
    
    ieee80211_stop_running(ic);
    
    /*
     * NB: the order of these is important:
     * o call the 802.11 layer before detaching the hal to
     *   insure callbacks into the driver to delete global
     *   key cache entries can be handled
     * o reclaim the tx queue data structures after calling
     *   the 802.11 layer as we'll get called back to reclaim
     *   node state and potentially want to use them
     * o to cleanup the tx queues the hal is called, so detach
     *   it last
     * Other than that, it's straightforward...
     */
    ieee80211_ifdetach(ic);
    ath_cwm_detach(scn);
#ifdef ATH_AMSDU
    ath_amsdu_detach(scn);
#endif
#ifndef ATH_WLAN_COMBINE
    ath_limit_legacy_detach(scn);
#endif /* #ifndef ATH_WLAN_COMBINE */
    ath_dev_free(scn->sc_dev);
    scn->sc_dev = NULL;
    scn->sc_ops = NULL;
    return 0;
}

int
ath_resume(struct ath_softc_net80211 *scn)
{
    return ath_init(&scn->sc_ic);
}


