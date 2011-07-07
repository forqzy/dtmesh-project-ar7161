/*****************************************************************************/
/* \file ath_main.c
** \brief Main Rx/Tx code
**
**  This file contains the main implementation of the ATH layer.  Most member
**  functions of the ATH layer are defined here.
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
**
*/

#include "ath_internal.h"
#include "ath_antdiv.h"
#ifdef ATH_SUPPORT_DFS
#include "dfs.h"
#endif


#ifndef REMOVE_PKT_LOG
#include "pktlog.h"
struct ath_pktlog_funcs *g_pktlog_funcs = NULL;
#endif

/* Global configuration overrides */
static	const int countrycode = -1;
static	const int xchanmode = -1;
static	const int ath_outdoor = AH_FALSE;		/* enable outdoor use */

const u_int8_t ath_bcast_mac[IEEE80211_ADDR_LEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

#ifdef ATH_CHAINMASK_SELECT
u_int32_t   ath_chainmask_sel_up_rssi_thres = ATH_CHAINMASK_SEL_UP_RSSI_THRES;
u_int32_t   ath_chainmask_sel_down_rssi_thres = ATH_CHAINMASK_SEL_DOWN_RSSI_THRES;
u_int32_t   ath_chainmask_sel_period = ATH_CHAINMASK_SEL_TIMEOUT;

static void ath_chainmask_sel_timerstart(struct ath_chainmask_sel *cm);
static void ath_chainmask_sel_timerstop(struct ath_chainmask_sel *cm);
static void ath_chainmask_sel_init(struct ath_softc *sc, struct ath_node *an);
#endif

#ifdef ATH_SWRETRY
extern void ath_tx_flush_sxmitq(struct ath_softc *sc);
#endif

extern void ath_txq_schedule(struct ath_softc *sc, struct ath_txq *txq);

static void ath_printreg(ath_dev_t dev, u_int32_t printctrl);

static u_int32_t ath_getmfpsupport(ath_dev_t dev);
/*
** Internal Prototypes
*/

static int ath_vap_attach(ath_dev_t dev, int if_id, ieee80211_if_t if_data, HAL_OPMODE opmode, HAL_OPMODE iv_opmode, int nostabeacons);
static int ath_vap_detach(ath_dev_t dev, int if_id);
static int ath_vap_config(ath_dev_t dev, int if_id, struct ath_vap_config *if_config);

static ath_node_t ath_node_attach(ath_dev_t dev, int if_id, ieee80211_node_t ni);
static void ath_node_detach(ath_dev_t dev, ath_node_t an);
static void ath_node_cleanup(ath_dev_t dev, ath_node_t an);
static void ath_node_update_pwrsave(ath_dev_t dev, ath_node_t node, int pwrsave);

/* Key cache interfaces */
static u_int16_t ath_key_alloc_2pair(ath_dev_t);
static u_int16_t ath_key_alloc_pair(ath_dev_t);
static u_int16_t ath_key_alloc_single(ath_dev_t);
static void ath_key_reset(ath_dev_t, u_int16_t keyix, int freeslot);
static int ath_keyset(ath_dev_t, u_int16_t keyix, HAL_KEYVAL *hk,
                      const u_int8_t mac[IEEE80211_ADDR_LEN]);
static u_int ath_keycache_size(ath_dev_t);
static void ath_set_divant(ath_dev_t dev, int divant_value);

/* regdomain interfaces */
static int ath_set_country(ath_dev_t, char *isoName, u_int16_t);
static void ath_get_currentCountry(ath_dev_t, HAL_COUNTRY_ENTRY *ctry);
static int ath_set_regdomain(ath_dev_t, int regdomain);
static int ath_get_regdomain(ath_dev_t dev);
static int ath_set_quiet(ath_dev_t, u_int16_t period, u_int16_t duration,
                         u_int16_t nextStart, u_int16_t enabled);
static u_int16_t ath_find_countrycode(ath_dev_t, char* isoName);

static int ath_set_tx_antenna_select(ath_dev_t, u_int32_t antenna_select);
static u_int32_t ath_get_current_tx_antenna(ath_dev_t);
static u_int32_t ath_get_default_antenna(ath_dev_t);
static int16_t ath_get_noisefloor(ath_dev_t dev, u_int16_t	freq,  u_int chan_flags);

static void ath_update_sm_pwrsave(ath_dev_t dev, ath_node_t node, ATH_SM_PWRSAV mode,
	int ratechg);
/* Rate control interfaces */
static int athop_rate_newassoc(ath_dev_t dev, ath_node_t node, int isnew, unsigned int capflag,
                                  struct ieee80211_rateset *negotiated_rates,
                                  struct ieee80211_rateset *negotiated_htrates);
static void
athop_rate_newstate(ath_dev_t dev, int if_id, int up);
#ifdef DBG
static u_int32_t ath_register_read(ath_dev_t dev, u_int32_t offset);
static void ath_register_write(ath_dev_t dev, u_int32_t offset, u_int32_t value);
#endif

#ifdef ATH_SUPPORT_DFS
static OS_TIMER_FUNC(ath_radar_task);
static OS_TIMER_FUNC(ath_check_dfs_clear);
static OS_TIMER_FUNC(ath_dfs_test_return);
static OS_TIMER_FUNC(ath_dfs_hang_war)
{
        struct ath_softc *sc;
        OS_GET_TIMER_ARG(sc, struct ath_softc *);
        sc->sc_dfs_hang.hang_war_activated=0;
        sc->sc_ieee_ops->ath_net80211_switch_mode_dynamic2040(sc->sc_ieee);
}

#endif

/******************************************************************************/
/*!
**  \brief Expand time stamp to TSF
**
**  Extend 15-bit time stamp from rx descriptor to
**  a full 64-bit TSF using the current h/w TSF.
**
**  \param sc Pointer to ATH object (this)
**  \param rstamp Time stamp value
**
**  \return 64 bit TSF (Timer Synchronization Function) value
*/
u_int64_t
ath_extend_tsf(struct ath_softc *sc, u_int32_t rstamp)
{
    u_int64_t tsf;

    tsf = ath_hal_gettsf64(sc->sc_ah);
    if ((tsf & 0x7fff) < rstamp)
        tsf -= 0x8000;
    return ((tsf &~ 0x7fff) | rstamp);
}

/******************************************************************************/
/*!
**  \brief Set current operating mode
**
**  This function initializes and fills the rate table in the ATH object based
**  on the operating mode.  The blink rates are also set up here, although
**  they have been superceeded by the ath_led module.
**
**  \param sc Pointer to ATH object (this).
**  \param mode WIRELESS_MODE Enumerated value indicating the
**              wireless operating mode
**
**  \return N/A
*/

static void
ath_setcurmode(struct ath_softc *sc, WIRELESS_MODE mode)
{
#define	N(a)	(sizeof(a)/sizeof(a[0]))
    /* NB: on/off times from the Atheros NDIS driver, w/ permission */
    static const struct {
        u_int		rate;		/* tx/rx 802.11 rate */
        u_int16_t	timeOn;		/* LED on time (ms) */
        u_int16_t	timeOff;	/* LED off time (ms) */
    } blinkrates[] = {
        { 108,  40,  10 },
        {  96,  44,  11 },
        {  72,  50,  13 },
        {  48,  57,  14 },
        {  36,  67,  16 },
        {  24,  80,  20 },
        {  22, 100,  25 },
        {  18, 133,  34 },
        {  12, 160,  40 },
        {  10, 200,  50 },
        {   6, 240,  58 },
        {   4, 267,  66 },
        {   2, 400, 100 },
        {   0, 500, 130 },
    };
    const HAL_RATE_TABLE *rt;
    int i, j;

    OS_MEMSET(sc->sc_rixmap, 0xff, sizeof(sc->sc_rixmap));
    rt = sc->sc_rates[mode];
    KASSERT(rt != NULL, ("no h/w rate set for phy mode %u", mode));
    for (i = 0; i < rt->rateCount; i++) {
        sc->sc_rixmap[rt->info[i].rateCode] = (u_int8_t)i;
    }
    OS_MEMZERO(sc->sc_hwmap, sizeof(sc->sc_hwmap));
    for (i = 0; i < 256; i++) {
        u_int8_t ix = rt->rateCodeToIndex[i];
        if (ix == 0xff) {
            sc->sc_hwmap[i].ledon = 500;
            sc->sc_hwmap[i].ledoff = 130;
            continue;
        }
        sc->sc_hwmap[i].ieeerate =
            rt->info[ix].dot11Rate & IEEE80211_RATE_VAL;
        sc->sc_hwmap[i].rateKbps = rt->info[ix].rateKbps;

        if (rt->info[ix].shortPreamble ||
            rt->info[ix].phy == IEEE80211_T_OFDM) {
            //sc->sc_hwmap[i].flags |= IEEE80211_RADIOTAP_F_SHORTPRE;
        }
        /* setup blink rate table to avoid per-packet lookup */
        for (j = 0; j < N(blinkrates)-1; j++) {
            if (blinkrates[j].rate == sc->sc_hwmap[i].ieeerate) {
                break;
            }
        }
        /* NB: this uses the last entry if the rate isn't found */
        /* XXX beware of overlow */
        sc->sc_hwmap[i].ledon = blinkrates[j].timeOn;
        sc->sc_hwmap[i].ledoff = blinkrates[j].timeOff;
    }
    sc->sc_currates = rt;
    sc->sc_curmode = mode;
    /*
     * All protection frames are transmited at 2Mb/s for
     * 11g, otherwise at 1Mb/s.
     * XXX select protection rate index from rate table.
     */
    sc->sc_protrix = (mode == WIRELESS_MODE_11g ? 1 : 0);
    /* rate index used to send mgt frames */
    sc->sc_minrateix = 0;
}
#undef N

/******************************************************************************/
/*!
**  \brief Select Rate Table
**
**  Based on the wireless mode passed in, the rate table in the ATH object
**  is set to the mode specific rate table.  This also calls the callback
**  function to set the rate in the protocol layer object.
**
**  \param sc Pointer to ATH object (this)
**  \param mode Enumerated mode value used to select the specific rate table.
**
**  \return 0 if no valid mode is selected
**  \return 1 on success
*/

static int
ath_rate_setup(struct ath_softc *sc, WIRELESS_MODE mode)
{
    struct ath_hal *ah = sc->sc_ah;
    const HAL_RATE_TABLE *rt;

    switch (mode) {
    case WIRELESS_MODE_11a:
        sc->sc_rates[mode] = ath_hal_getratetable(ah, HAL_MODE_11A);
        break;
    case WIRELESS_MODE_11b:
        sc->sc_rates[mode] = ath_hal_getratetable(ah, HAL_MODE_11B);
        break;
    case WIRELESS_MODE_11g:
        sc->sc_rates[mode] = ath_hal_getratetable(ah, HAL_MODE_11G);
        break;
    case WIRELESS_MODE_108a:
        sc->sc_rates[mode] = ath_hal_getratetable(ah, HAL_MODE_TURBO);
        break;
    case WIRELESS_MODE_108g:
        sc->sc_rates[mode] = ath_hal_getratetable(ah, HAL_MODE_108G);
        break;
    case WIRELESS_MODE_11NA_HT20:
        sc->sc_rates[mode] = ath_hal_getratetable(ah, HAL_MODE_11NA_HT20);
        break;
    case WIRELESS_MODE_11NG_HT20:
        sc->sc_rates[mode] = ath_hal_getratetable(ah, HAL_MODE_11NG_HT20);
        break;
    case WIRELESS_MODE_11NA_HT40PLUS:
        sc->sc_rates[mode] = ath_hal_getratetable(ah, HAL_MODE_11NA_HT40PLUS);
        break;
    case WIRELESS_MODE_11NA_HT40MINUS:
        sc->sc_rates[mode] = ath_hal_getratetable(ah, HAL_MODE_11NA_HT40MINUS);
        break;
    case WIRELESS_MODE_11NG_HT40PLUS:
        sc->sc_rates[mode] = ath_hal_getratetable(ah, HAL_MODE_11NG_HT40PLUS);
        break;
    case WIRELESS_MODE_11NG_HT40MINUS:
        sc->sc_rates[mode] = ath_hal_getratetable(ah, HAL_MODE_11NG_HT40MINUS);
        break;
    default:
        DPRINTF(sc, ATH_DEBUG_ANY, "%s: invalid mode %u\n",
                __func__, mode);
        return 0;
    }
    rt = sc->sc_rates[mode];
    if (rt == NULL)
        return 0;

    /* setup rate set in 802.11 protocol layer */
    if (sc->sc_ieee_ops->setup_rate)
        sc->sc_ieee_ops->setup_rate(sc->sc_ieee, mode, NORMAL_RATE, rt);

    return 1;
}

/******************************************************************************/
/*!
**  \brief Setup half/quarter rate table support
**
**  Get pointers to the half and quarter rate tables.  Note that the mode
**  must have been selected previously.  Callbacks to the protocol object
**  are made if a table is selected.
**
**  \param sc Pointer to ATH object
**
**  \return N/A
*/

static void
ath_setup_subrates(struct ath_softc *sc)
{
    struct ath_hal *ah = sc->sc_ah;
    const HAL_RATE_TABLE *rt;

    sc->sc_half_rates = ath_hal_getratetable(ah, HAL_MODE_11A_HALF_RATE);
    rt = sc->sc_half_rates;
    if (rt != NULL) {
        if (sc->sc_ieee_ops->setup_rate)
            sc->sc_ieee_ops->setup_rate(sc->sc_ieee, WIRELESS_MODE_11a, HALF_RATE, rt);
    }

    sc->sc_quarter_rates = ath_hal_getratetable(ah, HAL_MODE_11A_QUARTER_RATE);
    rt = sc->sc_quarter_rates;
    if (rt != NULL) {
        if (sc->sc_ieee_ops->setup_rate)
            sc->sc_ieee_ops->setup_rate(sc->sc_ieee, WIRELESS_MODE_11a, QUARTER_RATE, rt);
    }
}

/******************************************************************************/
/*!
**  \brief Set Transmit power limit
**
**  This is intended as an "external" interface routine, since it uses the
**  ath_dev_t as input vice the ath_softc.
**
**  \param dev Void pointer to ATH object used by upper layers
**  \param txpowlimit 16 bit transmit power limit
**
**  \return N/A
*/

static void
ath_set_txpowlimit(ath_dev_t dev, u_int16_t txpowlimit)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);

    /*
     * Just save it in configuration blob. It will take effect
     * after a chip reset.
     */
    sc->sc_config.txpowlimit = txpowlimit;
}

void
ath_update_tpscale(struct ath_softc *sc)
{
    struct ath_hal *ah = sc->sc_ah;
    u_int32_t tpscale;

    /* Get current tpscale setting and compare with the newly configured one */
    ath_hal_gettpscale(ah, &tpscale);

    if (sc->sc_config.tpscale != tpscale) {
        ath_hal_settpscale(ah, sc->sc_config.tpscale);
        /* If user tries to set lower than MIN, the value will be set to MIN. 
        Do a read back to make sure sc_config reflects the actual value set. */
        ath_hal_gettpscale(ah, &tpscale);
        sc->sc_config.tpscale = tpscale;
    }
    /* Must do a reset for the new tpscale to take effect */
    ath_internal_reset(sc);
}

/******************************************************************************/
/*!
**  \brief Set Transmit power in HAL
**
**  This routine makes the actual HAL calls to set the new transmit power
**  limit.  This also calls back into the protocol layer setting the max
**  transmit power limit.
**
**  \param sc Pointer to ATH object (this)
**
**  \return N/A
*/

void
ath_update_txpow(struct ath_softc *sc, u_int16_t tpcInDb)
{
    struct ath_hal *ah = sc->sc_ah;
    u_int32_t txpow, txpowlimit;

    txpowlimit = (sc->sc_config.txpowlimit_override) ?
        sc->sc_config.txpowlimit_override : sc->sc_config.txpowlimit;

    if (sc->sc_curtxpow != txpowlimit) {
        ath_hal_settxpowlimit(ah, txpowlimit, tpcInDb);
        /* read back in case value is clamped */
        ath_hal_gettxpowlimit(ah, &txpow);
        sc->sc_curtxpow = txpow;
    }

    /*
     * Fetch max tx power level and update protocal stack
     */
    ath_hal_getmaxtxpow(sc->sc_ah, &txpow);
    
    if (sc->sc_ieee_ops->update_txpow)
        sc->sc_ieee_ops->update_txpow(sc->sc_ieee, sc->sc_curtxpow, txpow);
}

/******************************************************************************/
/*!
**  \brief Set up New Node
**
** Setup driver-specific state for a newly associated node.  This routine
** really only applies if compression or XR are enabled, there is no code
** covering any other cases.
**
**  \param dev Void pointer to ATH object, to be called from protocol layre
**  \param node ATH node object that represents the other station
**  \param isnew Flag indicating that this is a "new" association
**
**  \return xxx
*/
static void
ath_newassoc(ath_dev_t dev, ath_node_t node, int isnew, int isuapsd)
{

    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
	struct ath_node *an = ATH_NODE(node);
    int tidno;
    /*
     * if station reassociates, tear down the aggregation state.
     */
    if (!isnew) {
        for (tidno = 0; tidno < WME_NUM_TID; tidno++) {
            if (sc->sc_txaggr) {
                ath_tx_aggr_teardown(sc,an,tidno);
            }
            if (sc->sc_rxaggr) {
                ath_rx_aggr_teardown(sc,an,tidno);
            }
        }
    }
#ifdef ATH_SUPPORT_UAPSD
    an->an_flags = (isuapsd) ? ATH_NODE_UAPSD : 0;
#else
    an->an_flags=0;
#endif

#ifdef ATH_SUPERG_XR
    if(1) {
        struct ath_node *an = ATH_NODE(ni);
        if (ic->ic_ath_cap & an->an_node.ni_ath_flags & IEEE80211_ATHC_XR) {
            an->an_minffrate = ATH_MIN_FF_RATE;
        } else {
            an->an_minffrate = 0;
        }
        ath_grppoll_period_update(sc);
    }
#endif
}

/******************************************************************************/
/*!
**  \brief Convert Freq to channel number
**
**  Convert the frequency, provided in MHz, to an ieee 802.11 channel number.
**  This is done by calling a conversion routine in the ATH object.  The
**  sc pointer is required since this is a call from the protocol layer which
**  does not "know" about the internals of the ATH layer.
**
**  \param dev Void pointer to ATH object provided by protocol layer
**  \param freq Integer frequency, in MHz
**  \param flags Flag Value indicating if this is in the 2.4 GHz (CHANNEL_2GHZ)
**                    band or in the 5 GHz (CHANNEL_5GHZ).
**
**  \return Integer channel number
*/

static u_int
ath_mhz2ieee(ath_dev_t dev, u_int freq, u_int flags)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
 	return ath_hal_mhz2ieee(sc->sc_ah, freq, flags);
}

/******************************************************************************/
/*!
**  \brief Set up channel list
**
**  Determines the proper set of channelflags based on the selected mode,
**  allocates a channel array, and passes it to the HAL for initialization.
**  If successful, the list is passed to the upper layer, then de-allocated.
**
**  \param sc Pointer to ATH object (this)
**  \param cc Integer Country Code value (normally read from EEPROM)
**  \param outDoor Boolean indicating if this transmitter is outdoors.
**  \param xchanMode Boolean indicating if extension channels are enabled.
**
**  \return -ENOMEM If channel strucure cannot be allocated
**  \return -EINVAL If channel initialization fails
**  \return 0 on success
*/

static int
ath_getchannels(struct ath_softc *sc, u_int cc,
                HAL_BOOL outDoor, HAL_BOOL xchanMode)
{
    struct ath_hal *ah = sc->sc_ah;
    HAL_CHANNEL *chans;
    int nchan;
    u_int8_t regclassids[ATH_REGCLASSIDS_MAX];
    u_int nregclass = 0;
    u_int wMode;
    u_int netBand;

    wMode = sc->sc_reg_parm.wModeSelect;

    if (!(wMode & HAL_MODE_11A)) {
        wMode &= ~(HAL_MODE_TURBO|HAL_MODE_108A|HAL_MODE_11A_HALF_RATE);
    }
    if (!(wMode & HAL_MODE_11G)) {
        wMode &= ~(HAL_MODE_108G);
    }
    netBand = sc->sc_reg_parm.NetBand;
    if (!(netBand & HAL_MODE_11A)) {
        netBand &= ~(HAL_MODE_TURBO|HAL_MODE_108A|HAL_MODE_11A_HALF_RATE);
    }
    if (!(netBand & HAL_MODE_11G)) {
        netBand &= ~(HAL_MODE_108G);
    }
    wMode &= netBand;

    chans = (HAL_CHANNEL *)OS_MALLOC(sc->sc_osdev, 
                      IEEE80211_CHAN_MAX * sizeof(HAL_CHANNEL), 
                      GFP_KERNEL);

    if (chans == NULL) {
        printk("%s: unable to allocate channel table\n", __func__);
        return -ENOMEM;
    }
    
    if (!ath_hal_init_channels(ah, chans, IEEE80211_CHAN_MAX, (u_int *)&nchan,
                               regclassids, ATH_REGCLASSIDS_MAX, &nregclass,
                               cc, wMode, outDoor, xchanMode)) {
        u_int32_t rd;

        ath_hal_getregdomain(ah, &rd);
        printk("%s: unable to collect channel list from hal; "
               "regdomain likely %u country code %u\n",
               __func__, rd, cc);
        OS_FREE(chans);
        return -EINVAL;
    }

    if (sc->sc_ieee_ops->setup_channel_list) {
        sc->sc_ieee_ops->setup_channel_list(sc->sc_ieee, CLIST_UPDATE,
                                            chans, nchan, regclassids, nregclass,
                                            CTRY_DEFAULT);
    }

    OS_FREE(chans);
    return 0;
}

/******************************************************************************/
/*!
**  \brief Set Default Antenna
**
**  Call into the HAL to set the default antenna to use.  Not really valid for
**  MIMO technology.
**
**  \param context Void pointer to ATH object (called from Rate Control)
**  \param antenna Index of antenna to select
**
**  \return N/A
*/

void
ath_setdefantenna(void *context, u_int antenna)
{
    struct ath_softc *sc = (struct ath_softc *)context;
    struct ath_hal *ah = sc->sc_ah;

    /* XXX block beacon interrupts */
    ath_hal_setdefantenna(ah, antenna);
    if (sc->sc_defant != antenna)
        sc->sc_stats.ast_ant_defswitch++;
    sc->sc_defant = antenna;
    sc->sc_rxotherant = 0;
}

/******************************************************************************/
/*!
**  \brief Set Slot Time
**
**  This will wake up the chip if required, and set the slot time for the
**  frame (maximum transmit time).  Slot time is assumed to be already set
** in the ATH object member sc_slottime
**
**  \param sc Pointer to ATH object (this)
**
**  \return N/A
*/

void
ath_setslottime(struct ath_softc *sc)
{
    ATH_FUNC_ENTRY_VOID(sc);

    ATH_PS_WAKEUP(sc);
    ath_hal_setslottime(sc->sc_ah, sc->sc_slottime);
    sc->sc_updateslot = OK;
    ATH_PS_SLEEP(sc);
}

/******************************************************************************/
/*!
**  \brief Update slot time
**
**  This is a call interface used by the protocol layer to update the current
**  slot time setting based on updated settings.  This updates the hardware
**  also.
**
**  \param dev Void pointer to ATH object used by protocol layer
**  \param slottime Slot duration in microseconds
**
**  \return N/A
*/

static void
ath_updateslot(ath_dev_t dev, int slottime)
{
	struct ath_softc *sc = ATH_DEV_TO_SC(dev);

    ATH_FUNC_ENTRY_VOID(sc);
    ATH_PS_WAKEUP(sc);

    /* save slottime setting */
    sc->sc_slottime = slottime;
    
    /*!
    ** When not coordinating the BSS, change the hardware
    ** immediately.  For other operation we defer the change
    ** until beacon updates have propagated to the stations.
     */
    if (sc->sc_opmode == HAL_M_HOSTAP)
        sc->sc_updateslot = UPDATE;
    else
        ath_setslottime(sc);

    ATH_PS_SLEEP(sc);
}

/******************************************************************************/
/*!
**  \brief Determine mode from channel flags
**
**  This routine will provide the enumerated WIRELESSS_MODE value based
**  on the settings of the channel flags.  If ho valid set of flags
**  exist, the lowest mode (11b) is selected.
**
**  \param chan Pointer to channel flags value
**
**  \return Higest channel mode selected by the flags.
*/

static WIRELESS_MODE
ath_halchan2mode(HAL_CHANNEL *chan)
{
    if ((chan->channelFlags & CHANNEL_108G) == CHANNEL_108G)
        return WIRELESS_MODE_108g;
    else if ((chan->channelFlags & CHANNEL_108A) == CHANNEL_108A)
        return WIRELESS_MODE_108a;
    else if ((chan->channelFlags & CHANNEL_A) == CHANNEL_A)
        return WIRELESS_MODE_11a;
    else if ((chan->channelFlags & CHANNEL_G) == CHANNEL_G)
        return WIRELESS_MODE_11g;
    else if ((chan->channelFlags & CHANNEL_B) == CHANNEL_B)
        return WIRELESS_MODE_11b;
    else if ((chan->channelFlags & CHANNEL_A_HT20) == CHANNEL_A_HT20)
        return WIRELESS_MODE_11NA_HT20;
    else if ((chan->channelFlags & CHANNEL_G_HT20) == CHANNEL_G_HT20)
        return WIRELESS_MODE_11NG_HT20;
    else if ((chan->channelFlags & CHANNEL_A_HT40PLUS) == CHANNEL_A_HT40PLUS)
        return WIRELESS_MODE_11NA_HT40PLUS;
    else if ((chan->channelFlags & CHANNEL_A_HT40MINUS) == CHANNEL_A_HT40MINUS)
        return WIRELESS_MODE_11NA_HT40MINUS;
    else if ((chan->channelFlags & CHANNEL_G_HT40PLUS) == CHANNEL_G_HT40PLUS)
        return WIRELESS_MODE_11NG_HT40PLUS;
    else if ((chan->channelFlags & CHANNEL_G_HT40MINUS) == CHANNEL_G_HT40MINUS)
        return WIRELESS_MODE_11NG_HT40MINUS;

    /* NB: should not get here */
    printk("%s: cannot map channel to mode; freq %u flags 0x%x\n",
           __func__, chan->channel, chan->channelFlags);
    return WIRELESS_MODE_11b;
}

/******************************************************************************/
/*!
**  \brief Change Channels
**
**  Performs the actions to change the channel in the hardware, and set up
**  the current operating mode for the new channel.
**
**  \param sc Pointer to ATH object (this)
**  \param chan Pointer to channel descriptor for new channel
**
**  \return xxx
*/

static void
ath_chan_change(struct ath_softc *sc, HAL_CHANNEL *chan)
{
    WIRELESS_MODE mode;

    mode = ath_halchan2mode(chan);

    ath_rate_setup(sc, mode);
    ath_setcurmode(sc, mode);

#ifdef ATH_SUPPORT_DFS
#if 0 /* Temporarily commented bcoz of Hang Issue with G mode */
        if ((hchan.channelFlags & CHANNEL_108G) == CHANNEL_108G) {
            DPRINTF(sc, ATH_DEBUG_DFS2, "Calling ath_ar_enable \n");
            ath_ar_enable(sc);
        }
#endif
        if (sc->sc_dfs != NULL) {
            sc->sc_dfs->dfs_curchan_radindex = -1;
            sc->sc_dfs->dfs_extchan_radindex = -1;
        /* Only enable radar when we are in a DFS channel and are not scanning (see bug 29099)*/
        /* Also WDS clients must not enable radar detection */
            if((chan->privFlags & CHANNEL_DFS) && (!sc->sc_scanning) &&(!sc->sc_nostabeacons)) {
                u_int8_t index;
                int error;

				sc->sc_ieee_ops->ath_net80211_set_doth(sc->sc_ieee,1);
                error = dfs_radar_enable(sc);
                if (error) {
                    DPRINTF(sc, ATH_DEBUG_DFS, "%s: dfs_radar_enable failed\n",
                            __func__);
                }
                /* Get the index for both the primary and extension channels*/
                if (dfs_getchanstate(sc, &index, FALSE) != NULL)
                    sc->sc_dfs->dfs_curchan_radindex = index;
                if (dfs_getchanstate(sc, &index, TRUE) != NULL)
                    sc->sc_dfs->dfs_extchan_radindex = index;
            }
			else
			{
				sc->sc_ieee_ops->ath_net80211_set_doth(sc->sc_ieee,0);
			}
       }
#endif

#ifdef notyet
    /*
     * Update BPF state.
     */
    sc->sc_tx_th.wt_chan_freq = sc->sc_rx_th.wr_chan_freq =
        htole16(chan->ic_freq);
    sc->sc_tx_th.wt_chan_flags = sc->sc_rx_th.wr_chan_flags =
        htole16(chan->ic_flags);
#endif
}

/******************************************************************************/
/*!
**  \brief Set the current channel
**
** Set/change channels.  If the channel is really being changed, it's done
** by reseting the chip.  To accomplish this we must first cleanup any pending
** DMA, then restart stuff after a la ath_init.
**
**  \param sc Pointer to ATH object (this)
**  \param hchan Pointer to channel descriptor for the new channel.
**
**  \return -EIO on failure
**  \return 0 on success
*/

static int
ath_chan_set(struct ath_softc *sc, HAL_CHANNEL *hchan)
{
    struct ath_hal *ah = sc->sc_ah;
    u_int8_t chan_flags_changed=0;
    HAL_BOOL fastcc = AH_TRUE, stopped;
    HAL_HT_MACMODE ht_macmode = sc->sc_ieee_ops->cwm_macmode(sc->sc_ieee);
#ifdef ATH_SUPPORT_DFS
    int i;
    struct ath_vap *avp;
    int vap_in_wait=0;
#endif

    DPRINTF(sc, ATH_DEBUG_RESET, "%s: %u (%u MHz) -> %u (%u MHz)\n",
            __func__,
            ath_hal_mhz2ieee(ah, sc->sc_curchan.channel,
                             sc->sc_curchan.channelFlags),
            sc->sc_curchan.channel,
            ath_hal_mhz2ieee(ah, hchan->channel, hchan->channelFlags), hchan->channel);

    /* As part of the Sowl TX hang WAR, we switch from dynamic HT40 mode to 
       static HT20 mode when we detect the hang. In this case, only the channel 
       flags change, so no need to start DFS wait or any of the other good stuff.*/

     if (hchan->channel == sc->sc_curchan.channel &&
        hchan->channelFlags != sc->sc_curchan.channelFlags)
        chan_flags_changed = 1;

    if (hchan->channel != sc->sc_curchan.channel ||
        hchan->channelFlags != sc->sc_curchan.channelFlags ||
        sc->sc_update_chainmask ||
        sc->sc_full_reset)
    {
        HAL_STATUS status;

        /*!
        ** This is only performed if the channel settings have
        ** actually changed.
        **
        ** To switch channels clear any pending DMA operations;
        ** wait long enough for the RX fifo to drain, reset the
        ** hardware at the new frequency, and then re-enable
        ** the relevant bits of the h/w.
        */
        
        ath_hal_intrset(ah, 0);     /* disable interrupts */
        ath_draintxq(sc, AH_FALSE); /* clear pending tx frames */
        stopped = ath_stoprecv(sc); /* turn off frame recv */

        /* XXX: do not flush receive queue here. We don't want
         * to flush data frames already in queue because of
         * changing channel. */

        /* Set coverage class */
        if (sc->sc_scanning || !(hchan->channelFlags & CHANNEL_5GHZ))
            ath_hal_setcoverageclass(ah, 0, 0);
        else
            ath_hal_setcoverageclass(ah, sc->sc_coverageclass, 0);

        if (!stopped || sc->sc_full_reset)
            fastcc = AH_FALSE;

        ATH_RESET_LOCK(sc);
        if (!ath_hal_reset(ah, sc->sc_opmode, hchan,
                           ht_macmode, sc->sc_tx_chainmask, sc->sc_rx_chainmask,
                           sc->sc_ht_extprotspacing, fastcc, &status)) {
            printk("%s: unable to reset channel %u (%uMhz) "
                   "flags 0x%x hal status %u\n",
                   __func__,
                   ath_hal_mhz2ieee(ah, hchan->channel, hchan->channelFlags),
                   hchan->channel, hchan->channelFlags, status);
            ATH_RESET_UNLOCK(sc);
            return -EIO;
        }
        ATH_RESET_UNLOCK(sc);
        sc->sc_curchan = *hchan;
        sc->sc_update_chainmask = 0;
        sc->sc_full_reset = 0;

        /*
         * Re-enable rx framework.
         */
        if (ath_startrecv(sc) != 0) {
            printk("%s: unable to restart recv logic\n",
                   __func__);
            return -EIO;
        }

        /*
         * Change channels and update the h/w rate map
         * if we're switching; e.g. 11a to 11b/g.
         */
        ath_chan_change(sc, hchan);

        ath_update_txpow(sc, 0);		/* update tx power state */

#ifdef ATH_SUPPORT_DFS
        /* Only start DFS wait when we are in a DFS channel, in AP mode 
        and are not scanning (see bug 29099)*/

        /* Also if we are changing modes (in case of the Sowl TX hang WAR,
        only the channel flags have changed, so no need to start DFS wait again*/
        /* Also WDS clients must not start DFS wait */

        if((sc->sc_opmode == HAL_M_HOSTAP) && (!sc->sc_scanning) && (!chan_flags_changed) && (!sc->sc_nostabeacons)) {
            if(sc->sc_curchan.privFlags & CHANNEL_DFS) {
                if (!(sc->sc_curchan.privFlags & CHANNEL_DFS_CLEAR)) {

                    if(sc->sc_dfs->sc_dfswait)
                        OS_CANCEL_TIMER(&sc->sc_dfs->sc_dfswaittimer);

                    printk("%s: start DFS WAIT period on channel %d\n", __func__,sc->sc_curchan.channel);
                    sc->sc_dfs->sc_dfswait=1;

                    for (i=0; i < sc->sc_nvaps; i++) {
                        avp = sc->sc_vaps[i];
                        if (avp) {
                        /* Bug fix for 52582 and 54279 - do not set VAP state to INIT when starting DFS 
                          wait this can result in kernel panic. */

                            // Make sure that av_dfswait_run is set for only 1 VAP
                            if(!avp->av_dfswait_run && !vap_in_wait){
                                avp->av_dfswait_run=1;
                                vap_in_wait=1;
                            } else {
                                if(vap_in_wait)
                                    avp->av_dfswait_run=0;
                                if(avp->av_dfswait_run)
                                    vap_in_wait=1;
                            }
                        }
                    }
                    OS_SET_TIMER(&sc->sc_dfs->sc_dfswaittimer, sc->sc_dfs->sc_dfs_cac_time);
                } else {
                    if (sc->sc_dfs->sc_dfswait == 1) {
                        printk("%s: cancel DFS WAIT period on channel %d\n", __func__, sc->sc_curchan.channel);
                        OS_CANCEL_TIMER(&sc->sc_dfs->sc_dfswaittimer);
                        OS_SET_TIMER(&sc->sc_dfs->sc_dfswaittimer, 2);
                    }
                }
            } else {
                /* We are now in a non-DFS channel, but we may still have wait timer from a previous
                   DFS channel pending. This can happen when radar is detected before the end of
                   the CAC wait time and the AP moves to a non-DFS channel*/
                if (sc->sc_dfs->sc_dfswait) {
                    OS_CANCEL_TIMER(&sc->sc_dfs->sc_dfswaittimer);
                    printk("Non-DFS channel, cancelling previous DFS wait timer channel %d\n", sc->sc_curchan.channel);
                    /* Bug 32399 - When AP detects radar in CAC period, if it switches to a non-DFS channel, nothing sets the VAP to running state. So no beaconing will start. Fix is to cancel the DFS wait timer like before but do all the cleanup that it would do at the end of CAC.*/
                    OS_SET_TIMER(&sc->sc_dfs->sc_dfswaittimer, 0);
                } else {
                    /* Bug fix for 52582 and 54279 - reconfigure beacons here if non DFS channel, this
                       fixes the issue where VAP never starts beaconing after radar detection forces 
                       a switch to a non-DFS channel */
                    if(sc->sc_beacons)
                        ath_beacon_config(sc, ATH_IF_ID_ANY);
                }
        }
    }
#endif
        /*!
        ** re configure beacons when it is a turbo mode switch.
        ** HW seems to turn off beacons during turbo mode switch.
        ** This is only needed for AP or IBSS mode. Also, we don't
        ** want to send beacons on a non-home channel during a
        ** background scan in IBSS mode.
         */
        if (sc->sc_opmode == HAL_M_HOSTAP ||
            sc->sc_opmode == HAL_M_IBSS) {
            if ((sc->sc_beacons && !sc->sc_scanning) ||
                sc->sc_vap_ind) {
                ath_beacon_config(sc, ATH_IF_ID_ANY);
            }
        }
        /*
         * Re-enable interrupts.
         */
        ath_hal_intrset(ah, sc->sc_imask);
    }
    return 0;
}

/******************************************************************************/
/*!
**  \brief Calculate the Calibration Interval
**
**  The interval must be the shortest necessary to satisfy ANI, 
**  short calibration and long calibration.
**
**  \param sc_Caldone: indicates whether calibration has already completed
**         enableANI: indicates whether ANI is enabled
**
**  \return The timer interval
*/

static u_int32_t
ath_get_cal_interval(HAL_BOOL sc_Caldone, HAL_BOOL enableANI)
{
    u_int32_t    cal_interval;        

    cal_interval = ATH_LONG_CALINTERVAL;
    if (enableANI) {
        cal_interval = min(cal_interval, (u_int32_t)ATH_ANI_POLLINTERVAL);
    }
    if (! sc_Caldone) {
        cal_interval = min(cal_interval, (u_int32_t)ATH_SHORT_CALINTERVAL);
    }

    /*
     * If we wanted to be really precise we'd now compare cal_interval to the
     * time left before each of the calibrations must be run. 
     * For example, it is possible that sc_Caldone changed from FALSE to TRUE 
     * just 1 second before the long calibration interval would elapse. 
     * Is this case the timer interval is still set to ATH_LONG_CALINTERVAL, 
     * causing an extended delay before the long calibration is performed.
     *
     * Using values for ani and short and long calibrations that are not 
     * multiples of each other would cause a similar problem (try setting
     * ATH_SHORT_CALINTERVAL = 20000 and ATH_LONG_CALINTERVAL = 30000)
     */

    return cal_interval;
}

/******************************************************************************/
/*!
**  \brief Calibration Task
**
**  This routine performs the periodic noise floor calibration function
**  that is used to adjust and optimize the chip performance.  This
**  takes environmental changes (location, temperature) into account.
**  When the task is complete, it reschedules itself depending on the
**  appropriate interval that was calculated.
**
**  \param xxx Parameters are OS dependant.  See the definition of the
**             OS_TIMER_FUNC macro
**
**  \return N/A
*/

static
OS_TIMER_FUNC(ath_calibrate)
{
    struct ath_softc    *sc;
    struct ath_hal      *ah;
    HAL_BOOL            longcal    = AH_FALSE;
    HAL_BOOL            shortcal   = AH_FALSE;
    HAL_BOOL            aniflag    = AH_FALSE;
    HAL_BOOL            work_to_do = AH_FALSE;
    systime_t           timestamp  = OS_GET_TIMESTAMP();
    u_int32_t           now        = (u_int32_t) CONVERT_SYSTEM_TIME_TO_MS(timestamp);

    OS_GET_TIMER_ARG(sc, struct ath_softc *);
    ah = sc->sc_ah;

    /* 
     * don't calibrate when we're scanning.
     * we are most likely not on our home channel. 
     */
    if (! sc->sc_scanning) {
        /*
         * Long calibration runs independently of short calibration.
         */
        if (CONVERT_SYSTEM_TIME_TO_MS(timestamp - sc->sc_longcal_timer) >= ATH_LONG_CALINTERVAL) {
            longcal = AH_TRUE;
            sc->sc_longcal_timer = timestamp;
        }

        /*
         * Short calibration applies only while (sc->sc_Caldone == AH_FALSE)
         */
        if (! sc->sc_Caldone) {
            if (CONVERT_SYSTEM_TIME_TO_MS(timestamp - sc->sc_shortcal_timer) >= ATH_SHORT_CALINTERVAL) {
                shortcal = AH_TRUE;
                sc->sc_shortcal_timer = timestamp;
                sc->sc_resetcal_timer = timestamp;
            }
        } else {
            if (CONVERT_SYSTEM_TIME_TO_MS(timestamp - sc->sc_resetcal_timer) >= ATH_RESTART_CALINTERVAL) {
                ath_hal_reset_calvalid(ah, &sc->sc_curchan, &sc->sc_Caldone);
                if (sc->sc_Caldone == AH_TRUE)
                    sc->sc_resetcal_timer = timestamp;
            }
        }
 
        /*
         * Verify whether we must check ANI
         */
        if (ath_hal_enabledANI(ah)) {
            if (CONVERT_SYSTEM_TIME_TO_MS(timestamp - sc->sc_ani_timer) >= ATH_ANI_POLLINTERVAL) {
                aniflag = AH_TRUE;
                sc->sc_ani_timer = timestamp;
            }
        }

        /*
         * Decide whether there's any work to do
         */
        work_to_do = longcal || shortcal || aniflag;
    }

    /*
     * Skip all processing if there's nothing to do.
     */
    if (work_to_do) {
        ATH_PS_WAKEUP(sc);
        
        /* Call ANI routine if necessary */
        if (aniflag) {
            ath_hal_rxmonitor(ah, &sc->sc_halstats, &sc->sc_curchan);
        }
        
        /* Perform calibration if necessary */
        if (longcal || shortcal) {
            HAL_BOOL    isCaldone = AH_FALSE;

            sc->sc_stats.ast_per_cal++;
        
            if (ath_hal_getrfgain(ah) == HAL_RFGAIN_NEED_CHANGE) {
                /*
                 * Rfgain is out of bounds, reset the chip
                 * to load new gain values.
                 */
                DPRINTF(sc, ATH_DEBUG_CALIBRATE, "%s: RFgain out of bounds, resetting device.\n", __func__);
                sc->sc_stats.ast_per_rfgain++;
                ath_internal_reset(sc);
            }
        
            if (ath_hal_calibrate(ah, &sc->sc_curchan, sc->sc_rx_chainmask, longcal, &isCaldone)) {
                /* XXX Need to investigate why ath_hal_getChanNoise is returning a 
                 * value that seems to be incorrect (-66dBm on the last test using CB72).
                 */
                if (longcal) {
                    sc->sc_noise_floor = ath_hal_getChanNoise(ah, &sc->sc_curchan);
                }

                DPRINTF(sc, ATH_DEBUG_CALIBRATE, "%d.%03d | %s: channel %u/%x noise_floor=%d\n",
                    now / 1000, now % 1000,  __func__, 
                    sc->sc_curchan.channel, 
                    sc->sc_curchan.channelFlags,
                    sc->sc_noise_floor);
            } else {
                DPRINTF(sc, ATH_DEBUG_ANY, "%d.%03d | %s: calibration of channel %u/%x failed\n",
                    now / 1000, now % 1000,  __func__,
                    sc->sc_curchan.channel,
                    sc->sc_curchan.channelFlags);
                sc->sc_stats.ast_per_calfail++;
            }

            sc->sc_Caldone = isCaldone;
        }

        ATH_PS_SLEEP(sc);
    }

    /*
     * Set timer interval based on previous results.
     * The interval must be the shortest necessary to satisfy ANI,
     * short calibration and long calibration.
     *
     */
    OS_SET_TIMER(&sc->sc_cal_ch,
                 ath_get_cal_interval(sc->sc_Caldone, ath_hal_enabledANI(ah)));
}

/******************************************************************************/
/*!
**  \brief Start Scan
**
**  This function is called when starting a channel scan.  It will perform
**  power save wakeup processing, set the filter for the scan, and get the
**  chip ready to send broadcast packets out during the scan.
**
**  \param dev Void pointer to ATH object passed from protocol layer
**
**  \return N/A
*/

static void
ath_scan_start(ath_dev_t dev)
{
    struct ath_softc    *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal      *ah = sc->sc_ah;
    u_int32_t           rfilt;
    u_int32_t           now = (u_int32_t) CONVERT_SYSTEM_TIME_TO_MS(OS_GET_TIMESTAMP());

    /*
     * Make sure chip is awake before modifying any registers.
     */
    ATH_PS_WAKEUP(sc);
    sc->sc_scanning = 1;
    rfilt = ath_calcrxfilter(sc);
    ath_hal_setrxfilter(ah, rfilt);
    ath_hal_setassocid(ah, ath_bcast_mac, 0);

    /*
     * Restore previous power management state.
     */
    ATH_PS_SLEEP(sc);

    DPRINTF(sc, ATH_DEBUG_STATE, "%d.%03d | %s: RX filter 0x%x aid 0\n",
            now / 1000, now % 1000, __func__, rfilt);
}

/******************************************************************************/
/*!
**  \brief Scan End
**
**  This routine is called by the upper layer when the scan is completed.  This
**  will set the filters back to normal operating mode, set the BSSID to the
**  correct value, and restore the power save state.
**
**  \param dev Void pointer to ATH object passed from protocol layer
**
**  \return N/A
*/

static void
ath_scan_end(ath_dev_t dev)
{
    struct ath_softc    *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal      *ah = sc->sc_ah;
    u_int32_t           rfilt;
    u_int32_t           now = (u_int32_t) CONVERT_SYSTEM_TIME_TO_MS(OS_GET_TIMESTAMP());

    /*
     * Make sure chip is awake before modifying any registers.
     */
    ATH_PS_WAKEUP(sc);
    sc->sc_scanning = 0;

    /*
     * Request for a full reset due to rx packet filter changes.
     */
    sc->sc_full_reset = 1;
    rfilt = ath_calcrxfilter(sc);
    ath_hal_setrxfilter(ah, rfilt);
    ath_hal_setassocid(ah, sc->sc_curbssid, sc->sc_curaid);

    /*
     * Restore previous power management state.
     */
    ATH_PS_SLEEP(sc);

    DPRINTF(sc, ATH_DEBUG_STATE, "%d.%03d | %s: RX filter 0x%x aid 0x%x\n",
            now / 1000, now % 1000, __func__, rfilt, sc->sc_curaid);
}

/******************************************************************************/
/*!
**  \brief Set Channel
**
**  This is the entry point from the protocol level to chang the current
**  channel the AP is serving.
**
**  \param dev Void pointer to ATH object passed from the protocol layer
**  \param hchan Pointer to channel structure with new channel information
**
**  \return N/A
*/

static void
ath_set_channel(ath_dev_t dev, HAL_CHANNEL *hchan)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);

    if (sc->sc_invalid)         /* if the device is invalid or removed */
        return;

    ATH_FUNC_ENTRY_VOID(sc);

    /*
     * Make sure chip is awake before modifying any registers.
     */
    ATH_PS_WAKEUP(sc);
    
    (void) ath_chan_set(sc, hchan);

    /*
     * Restore previous power management state.
     */
    ATH_PS_SLEEP(sc);
}

/******************************************************************************/
/*!
**  \brief Get MAC address
**
**  Copy the MAC address into the buffer provided
**
**  \param dev Void pointer to ATH object passed from protocol layer
**  \param macaddr Pointer to buffer to place MAC address into
**
**  \return N/A
*/

static void
ath_get_mac_address(ath_dev_t dev, u_int8_t macaddr[IEEE80211_ADDR_LEN])
{
    OS_MEMCPY(macaddr, ATH_DEV_TO_SC(dev)->sc_myaddr, IEEE80211_ADDR_LEN);
}

/******************************************************************************/
/*!
**  \brief set MAC address
**
**  Copies the provided MAC address into the proper ATH member, and sets the
**  value into the chipset through the HAL.
**
**  \param dev Void pointer to ATH object passed from protocol layer
**  \param macaddr buffer containing MAC address to set into hardware
**
**  \return N/A
*/

static void
ath_set_mac_address(ath_dev_t dev, const u_int8_t macaddr[IEEE80211_ADDR_LEN])
{
	struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    OS_MEMCPY(sc->sc_myaddr, macaddr, IEEE80211_ADDR_LEN);
    ath_hal_setmac(sc->sc_ah, sc->sc_myaddr);
}

/******************************************************************************/
/*!
**  \brief set multi-cast addresses
**
**  Copies the provided MAC address into the proper ATH member, and sets the
**  value into the chipset through the HAL.
**
**  \param dev Void pointer to ATH object passed from protocol layer
**  \param macaddr buffer containing MAC address to set into hardware
**
**  \return N/A
*/
static void
ath_set_mcastlist(ath_dev_t dev)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;
    u_int32_t mfilt[2];

    if (sc->sc_invalid)         /* if the device is invalid or removed */
        return;

    ATH_FUNC_ENTRY_VOID(sc);

    /* calculate and install multicast filter */
    if (sc->sc_ieee_ops->get_netif_settings(sc->sc_ieee) & ATH_NETIF_ALLMULTI) {
        mfilt[0] = mfilt[1] = ~0;
    }
    else {
        sc->sc_ieee_ops->netif_mcast_merge(sc->sc_ieee, mfilt);
    }

    /*
     * Make sure chip is awake before modifying any registers.
     */
    ATH_PS_WAKEUP(sc);

    ath_hal_setmcastfilter(ah, mfilt[0], mfilt[1]);

    /*
     * Restore previous power management state.
     */
    ATH_PS_SLEEP(sc);
}

/******************************************************************************/
/*!
**  \brief Enter Scan State
**
**  This routine will call the LED routine that "blinks" the light indicating
**  the device is in Scan mode. This sets no other status.
**
**  \param dev Void pointer to ATH object passed from protocol layer
**
**  \return N/A
*/

static void 
ath_enter_led_scan_state(ath_dev_t dev)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    ath_led_scan_start(&sc->sc_led_control);
}

/******************************************************************************/
/*!
**  \brief Leave Scan State
**
**  Calls the LED Scan End function to change the LED state back to normal.
**  Does not set any other states.
**
**  \param dev Void pointer to ATH object passed from protocol layer
**
**  \return N/A
*/

static void
ath_leave_led_scan_state(ath_dev_t dev)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    ath_led_scan_end(&sc->sc_led_control);
}

/******************************************************************************/
/*!
**  \brief Activate ForcePPM
**
**  Activates/deactivates ForcePPM.
**  Rules for ForcePPM activation depend on the operation mode:
**      -STA:   active when connected.
**      -AP:    active when a single STA is connected to it.
**      -ADHOC: ??
**
**  \param dev      Void pointer to ATH object passed from protocol layer
**  \param enable   Flag indicating whether to enable ForcePPM
**  \param bssid    Pointer to BSSID. Must point to a valid BSSID if flag 'enable' is set.
**
**  \return N/A
*/

static void
ath_notify_force_ppm(ath_dev_t dev, int enable, u_int8_t *bssid)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);

    /* Notify PPM module that we are returning to the home channel; report connection status */
    ath_force_ppm_notify(&sc->sc_ppm_info, enable, bssid);
}

/******************************************************************************/
/*!
**  \brief Down VAP instance
**
**  This routine will stop the indicated VAP and put it in a "down" state.
**  The down state is basically an initialization state that can be brought
**  back up by callint the opposite up routine.
**  This routine will bring the interface out of power save mode, set the
**  LED states, update the rate control processing, stop DMA transfers, and
**  set the VAP into the down state.
**
**  \param dev      Void pointer to ATH object passed from protocol layer
**  \param if_id    Devicescape style interface identifier, integer
**  \param flags    Flags indicating device capabilities
**
**  \return -EINVAL if the ID is invalid
**  \return  0 on success
*/

static int
ath_vap_down(ath_dev_t dev, int if_id, u_int flags)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;
    struct ath_vap *avp;

    avp = sc->sc_vaps[if_id];
    if (avp == NULL) {
        DPRINTF(sc, ATH_DEBUG_STATE, "%s: invalid interface id %u\n",
                __func__, if_id);
        return -EINVAL;
    }

    ath_pwrsave_awake(sc);

    ATH_PS_WAKEUP(sc);

    OS_CANCEL_TIMER(&sc->sc_cal_ch);		/* periodic calibration timer */

#if defined(SLOW_ANT_DIV) || defined(AP_SLOW_ANT_DIV)
    if (sc->sc_slowAntDiv)
        ath_slow_ant_div_stop(&sc->sc_antdiv);
#ifdef AP_SLOW_ANT_DIV
    if (sc->sc_antdiv.enabled) {
        OS_CANCEL_TIMER(&sc->sc_slow_ant_div);
    }
#endif
#endif

    /* set LEDs to INIT state */
    ath_set_led_state(&sc->sc_led_control, HAL_LED_INIT);

    /* update ratectrl about the new state */
    ath_rate_newstate(sc, avp, 0);

    /* Reclaim beacon resources */
    if (sc->sc_opmode == HAL_M_HOSTAP || sc->sc_opmode == HAL_M_IBSS) {
        ath_hal_stoptxdma(ah, sc->sc_bhalq);
        ath_beacon_return(sc, avp);
    }

    if (flags & ATH_IF_HW_OFF) {
        HAL_BOOL gpio_intr = AH_FALSE;

        sc->sc_imask &= ~(HAL_INT_SWBA | HAL_INT_BMISS);
        /*
         * Disable interrupts other than GPIO to capture RfKill event.
         */
#ifdef ATH_RFKILL
        if (sc->sc_hasrfkill && ath_rfkill_hasint(sc))
            gpio_intr = AH_TRUE;
#endif
        if (gpio_intr)
            ath_hal_intrset(ah, HAL_INT_GLOBAL | HAL_INT_GPIO);
        else
            ath_hal_intrset(ah, sc->sc_imask &~ HAL_INT_GLOBAL);
        sc->sc_beacons=0;
    }

#ifdef ATH_SUPERG_XR
    if (ieee80211vap_has_flag(vap, IEEE80211_F_XR) && sc->sc_xrgrppoll) {
        ath_grppoll_stop(vap);
    }
#endif
    ATH_PS_SLEEP(sc);
    return 0;
}

/******************************************************************************/
/*!
**  \brief VAP in Listen mode
**
**  This routine brings the VAP out of the down state into a "listen" state
**  where it waits for association requests.  This is used in AP and AdHoc
**  modes.
**
**  \param dev      Void pointer to ATH object passed from protocol layer
**  \param if_id    Devicescape style interface identifier, integer
**
**  \return -EINVAL if interface ID is not valid
**  \return  0 on success
*/

static int
ath_vap_listen(ath_dev_t dev, int if_id)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;
    struct ath_vap *avp;
    u_int32_t rfilt = 0;

    avp = sc->sc_vaps[if_id];
    if (avp == NULL) {
        DPRINTF(sc, ATH_DEBUG_STATE, "%s: invalid interface id %u\n",
                __func__, if_id);
        return -EINVAL;
    }

    ath_pwrsave_awake(sc);

    ATH_PS_WAKEUP(sc);

    OS_CANCEL_TIMER(&sc->sc_cal_ch);		/* periodic calibration timer */

#if defined(SLOW_ANT_DIV) || defined(AP_SLOW_ANT_DIV)
    if (sc->sc_slowAntDiv)
        ath_slow_ant_div_stop(&sc->sc_antdiv);
#ifdef AP_SLOW_ANT_DIV
    if (sc->sc_antdiv.enabled) {
        OS_CANCEL_TIMER(&sc->sc_slow_ant_div);
    }
#endif
#endif

    /* set LEDs to READY state */
    ath_set_led_state(&sc->sc_led_control, HAL_LED_READY);

    /* update ratectrl about the new state */
    ath_rate_newstate(sc, avp, 0);

    rfilt = ath_calcrxfilter(sc);
    ath_hal_setrxfilter(ah, rfilt);

    if (sc->sc_opmode == HAL_M_STA || sc->sc_opmode == HAL_M_IBSS) {
        ATH_ADDR_COPY(sc->sc_curbssid, ath_bcast_mac);
        ath_hal_setassocid(ah, sc->sc_curbssid, sc->sc_curaid);
    } else
        sc->sc_curaid = 0;

    DPRINTF(sc, ATH_DEBUG_STATE, "%s: RX filter 0x%x bssid %02x:%02x:%02x:%02x:%02x:%02x aid 0x%x\n",
            __func__, rfilt,
            sc->sc_curbssid[0], sc->sc_curbssid[1], sc->sc_curbssid[2],
            sc->sc_curbssid[3], sc->sc_curbssid[4], sc->sc_curbssid[5],
            sc->sc_curaid);

    /*
     * XXXX
     * Disable BMISS interrupt when we're not associated
     */
    ath_hal_intrset(ah,sc->sc_imask &~ (HAL_INT_SWBA | HAL_INT_BMISS));
    sc->sc_imask &= ~(HAL_INT_SWBA | HAL_INT_BMISS);
    sc->sc_beacons = 0; /* need to reconfigure the beacons when it moves to RUN */

    avp->av_dfswait_run=0; /* reset the dfs wait flag */ 

    ATH_PS_SLEEP(sc);
    return 0;
}

/******************************************************************************/
/*!
**  \brief xxxxxe
**
**  -- Enter Detailed Description --
**
**  \param xx xxxxxxxxxxxxx
**  \param xx xxxxxxxxxxxxx
**
**  \return xxx
*/

static int
ath_vap_join(ath_dev_t dev, int if_id, const u_int8_t bssid[IEEE80211_ADDR_LEN], u_int flags)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;
    struct ath_vap *avp;
    u_int32_t rfilt = 0;

    avp = sc->sc_vaps[if_id];
    if (avp == NULL) {
        DPRINTF(sc, ATH_DEBUG_STATE, "%s: invalid interface id %u\n",
                __func__, if_id);
        return -EINVAL;
    }

    ath_pwrsave_awake(sc);

    ATH_PS_WAKEUP(sc);

    OS_CANCEL_TIMER(&sc->sc_cal_ch);		/* periodic calibration timer */

    /* set LEDs to READY state */
    ath_set_led_state(&sc->sc_led_control, HAL_LED_READY);

    /* update ratectrl about the new state */
    ath_rate_newstate(sc, avp, 0);

    rfilt = ath_calcrxfilter(sc);
    ath_hal_setrxfilter(ah, rfilt);

    ATH_ADDR_COPY(sc->sc_curbssid, bssid);
    sc->sc_curaid = 0;
    ath_hal_setassocid(ah, sc->sc_curbssid, sc->sc_curaid);

    DPRINTF(sc, ATH_DEBUG_STATE, "%s: RX filter 0x%x bssid %02x:%02x:%02x:%02x:%02x:%02x aid 0x%x\n",
            __func__, rfilt,
            sc->sc_curbssid[0], sc->sc_curbssid[1], sc->sc_curbssid[2],
            sc->sc_curbssid[3], sc->sc_curbssid[4], sc->sc_curbssid[5],
            sc->sc_curaid);

    /*
     * Update tx/rx chainmask. For legacy association, hard code chainmask to 1x1,
     * for 11n association, use the chainmask configuration.
     */
    sc->sc_update_chainmask = 1;
    if (flags & ATH_IF_HT) {
        sc->sc_tx_chainmask = sc->sc_config.txchainmask;
        sc->sc_rx_chainmask = sc->sc_config.rxchainmask;
    } else {
        sc->sc_tx_chainmask = sc->sc_config.txchainmasklegacy; 
        sc->sc_rx_chainmask = sc->sc_config.rxchainmasklegacy; 
    }
    
    /*
     * Enable rx chain mask detection if configured to do so
     */
    if (sc->sc_reg_parm.rxChainDetect)
        sc->sc_rx_chainmask_detect = 1;
    else
        sc->sc_rx_chainmask_detect = 0;
        
    /*
    ** Set aggregation protection mode parameters
    */
    
    sc->sc_config.ath_aggr_prot = sc->sc_reg_parm.aggrProtEnable;
    sc->sc_config.ath_aggr_prot_duration = sc->sc_reg_parm.aggrProtDuration;
    sc->sc_config.ath_aggr_prot_max = sc->sc_reg_parm.aggrProtMax;
    
    /*
     * Reset our TSF so that its value is lower than the beacon that we are
     * trying to catch. Only then hw will update its TSF register with the
     * new beacon. Reset the TSF before setting the BSSID to avoid allowing
     * in any frames that would update our TSF only to have us clear it
     * immediately thereafter.
     */
    ath_hal_resettsf(ah);

    /*
     * XXXX
     * Disable BMISS interrupt when we're not associated
     */
    if (flags & ATH_IF_VAP_IND) {
        ath_hal_intrset(ah, sc->sc_imask & ~HAL_INT_BMISS);
        sc->sc_imask &= ~HAL_INT_BMISS;
        sc->sc_vap_ind = 1;
    } else {
        ath_hal_intrset(ah,sc->sc_imask &~ (HAL_INT_SWBA | HAL_INT_BMISS));
        sc->sc_imask &= ~(HAL_INT_SWBA | HAL_INT_BMISS);
        sc->sc_beacons = 0; /* need to reconfigure the beacons when it moves to RUN */
    }

    avp->av_dfswait_run=0; /* reset the dfs wait flag */

    ATH_PS_SLEEP(sc);
    return 0;
}

static int
ath_vap_up(ath_dev_t dev, int if_id, const u_int8_t bssid[IEEE80211_ADDR_LEN],
           u_int8_t aid, u_int flags)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;
    struct ath_vap *avp;
    u_int32_t rfilt = 0;
    int i, error = 0;
    systime_t timestamp = OS_GET_TIMESTAMP();

    ASSERT(if_id != ATH_IF_ID_ANY);
    avp = sc->sc_vaps[if_id];
    if (avp == NULL) {
        DPRINTF(sc, ATH_DEBUG_STATE, "%s: invalid interface id %u\n",
                __func__, if_id);
        return -EINVAL;
    }

    ath_pwrsave_awake(sc);

    ATH_PS_WAKEUP(sc);

    OS_CANCEL_TIMER(&sc->sc_cal_ch);		/* periodic calibration timer */

    /* set LEDs to RUN state */
    ath_set_led_state(&sc->sc_led_control, HAL_LED_RUN);

    /* update ratectrl about the new state */
    ath_rate_newstate(sc, avp, 1);

    rfilt = ath_calcrxfilter(sc);
    ath_hal_setrxfilter(ah, rfilt);

    if (avp->av_opmode == HAL_M_STA || avp->av_opmode == HAL_M_IBSS) {
        ATH_ADDR_COPY(sc->sc_curbssid, bssid);
        sc->sc_curaid = aid;
        ath_hal_setassocid(ah, sc->sc_curbssid, sc->sc_curaid);
    }

    DPRINTF(sc, ATH_DEBUG_STATE, "%s: RX filter 0x%x bssid %02x:%02x:%02x:%02x:%02x:%02x aid 0x%x\n",
            __func__, rfilt,
            sc->sc_curbssid[0], sc->sc_curbssid[1], sc->sc_curbssid[2],
            sc->sc_curbssid[3], sc->sc_curbssid[4], sc->sc_curbssid[5],
            sc->sc_curaid);

    if ((avp->av_opmode != IEEE80211_M_STA) && (flags & ATH_IF_PRIVACY)) {
        for (i = 0; i < IEEE80211_WEP_NKID; i++)
            if (ath_hal_keyisvalid(ah, (u_int16_t)i))
                ath_hal_keysetmac(ah, (u_int16_t)i, bssid);
    }

    switch (avp->av_opmode) {
    case HAL_M_HOSTAP:
    case HAL_M_IBSS:
        /*
         * Allocate and setup the beacon frame.
         *
         * Stop any previous beacon DMA.  This may be
         * necessary, for example, when an ibss merge
         * causes reconfiguration; there will be a state
         * transition from RUN->RUN that means we may
         * be called with beacon transmission active.
         */
        ath_hal_stoptxdma(ah, sc->sc_bhalq);

        error = ath_beacon_alloc(sc, if_id);
        if (error != 0)
            goto bad;

        /*
         * if the turbo flags have changed, then beacon and turbo
         * need to be reconfigured.
         */
        if((sc->sc_dturbo && !(flags & ATH_IF_DTURBO)) ||
           (!sc->sc_dturbo && (flags & ATH_IF_DTURBO)))
            sc->sc_beacons = 0;

        if (flags & ATH_IF_BEACON_ENABLE)
            sc->sc_beacons = 0;

        break;

    case HAL_M_STA:
        /*
         * Record negotiated dynamic turbo state for
         * use by rate control modules.
         */
        sc->sc_dturbo = (flags & ATH_IF_DTURBO) ? 1 : 0;

        /*
         * start rx chain mask detection if it is enabled.
         * Use the default chainmask as starting point.
         */
        if (sc->sc_rx_chainmask_detect) {
            if (flags & ATH_IF_HT) {
                sc->sc_rx_chainmask = sc->sc_config.rxchainmask;
            } else {
                sc->sc_rx_chainmask = sc->sc_config.rxchainmasklegacy; 
            }
            sc->sc_rx_chainmask_start = 1;
        }

#ifdef SLOW_ANT_DIV
        /* Start slow antenna diversity */
        if (sc->sc_slowAntDiv)
        {
            u_int32_t num_antcfg;

            if (sc->sc_curchan.channelFlags & CHANNEL_5GHZ)
                ath_hal_getcapability(ah, HAL_CAP_ANT_CFG_5GHZ, 0, &num_antcfg);
            else
                ath_hal_getcapability(ah, HAL_CAP_ANT_CFG_2GHZ, 0, &num_antcfg);
     
            if (num_antcfg > 1)
                ath_slow_ant_div_start(&sc->sc_antdiv, num_antcfg, bssid);
            }
#endif
        break;

    default:
        break;
    }

#ifdef ATH_SUPPORT_DFS
    /*
     * if it is a DFS channel and has not been checked for radar 
     * do not let the 80211 state machine to go to RUN state.
     */
    if (sc->sc_dfs && sc->sc_dfs->sc_dfswait 
            && avp->av_opmode == HAL_M_HOSTAP) {
        /* push the vap to RUN state once DFS is cleared */
        DPRINTF(sc, ATH_DEBUG_STATE, "%s: avp  -> DFS_WAIT\n", __func__);
        avp->av_dfswait_run = 1; 
        error = EAGAIN;
        goto bad;
    }
#endif

    /* Moved beacon_config after dfs_wait check
     * so that ath_beacon_config won't be called duing dfswait
     * period - this will fix the beacon stuck afer DFS 
     * CAC period issue
     */
    /*
     * Configure the beacon and sleep timers.
     */
    if (!sc->sc_beacons && !(flags & ATH_IF_BEACON_SYNC)) {
        ath_beacon_config(sc, if_id);
        sc->sc_beacons = 1;
    }

    /*
     * Reset rssi stats; maybe not the best place...
     */
    if (flags & ATH_IF_HW_ON) {
        sc->sc_halstats.ns_avgbrssi = ATH_RSSI_DUMMY_MARKER;
        sc->sc_halstats.ns_avgrssi = ATH_RSSI_DUMMY_MARKER;
        sc->sc_halstats.ns_avgtxrssi = ATH_RSSI_DUMMY_MARKER;
        sc->sc_halstats.ns_avgtxrate = ATH_RATE_DUMMY_MARKER;
    }

    /* start periodic recalibration timer */
    sc->sc_longcal_timer  = timestamp;
    sc->sc_shortcal_timer = timestamp;
    sc->sc_ani_timer      = timestamp;
    OS_SET_TIMER(&sc->sc_cal_ch,
                 ath_get_cal_interval(sc->sc_Caldone, ath_hal_enabledANI(ah)));


#ifdef ATH_SUPERG_XR
    if (ieee80211vap_has_flag(vap, IEEE80211_F_XR) && nstate == IEEE80211_S_RUN ) {
        ATH_SETUP_XR_VAP(sc,vap,rfilt);
    }
#endif

bad:
    ATH_PS_SLEEP(sc);
    return error;
}

/*
************************************
*  PnP routines
************************************
*/

static int
ath_stop_locked(struct ath_softc *sc)
{
    struct ath_hal *ah = sc->sc_ah;
#ifdef ATH_RB
    ath_rb_t *rb = &sc->sc_rb;
#endif

    DPRINTF(sc, ATH_DEBUG_RESET, "%s: invalid %u\n",
            __func__, sc->sc_invalid);

// RNWF    if (dev->flags & IFF_RUNNING) {
    /*
     * Shutdown the hardware and driver:
     *    stop output from above
     *    reset 802.11 state machine
     *	(sends station deassoc/deauth frames)
     *    turn off timers
     *    disable interrupts
     *    clear transmit machinery
     *    clear receive machinery
     *    turn off the radio
     *    reclaim beacon resources
     *
     * Note that some of this work is not possible if the
     * hardware is gone (invalid).
     */

    /* Stop ForcePPM module */
    ath_halt_force_ppm_module(&sc->sc_ppm_info);

#ifdef ATH_RB
    /* Cancel RB timer */
    ath_cancel_timer(&rb->restore_timer, CANCEL_NO_SLEEP);
#endif

// RNWF upper layers will know to stop themselves
//        netif_stop_queue(dev);	/* XXX re-enabled by ath_newstate */
//        dev->flags &= ~IFF_RUNNING;	/* NB: avoid recursion */
    ATH_PS_WAKEUP(sc);

    // Stop LED module.
    // ath_stop_locked can be called multiple times, so we wait until we are
    // actually dettaching from the device (sc->sc_invalid = TRUE) before 
    // stopping the LED module.
    if (sc->sc_invalid) {
        ath_halt_led_control(&sc->sc_led_control);
    }

    if (!sc->sc_invalid) {
        ath_hal_intrset(ah, 0);
    }
    ath_draintxq(sc, AH_FALSE);
    if (!sc->sc_invalid) {
        ath_stoprecv(sc);
        ath_hal_phydisable(ah);
    } else
        sc->sc_rxlink = NULL;

//    }
    ATH_PS_SLEEP(sc);
    return 0;
}

static int
ath_open(ath_dev_t dev, HAL_CHANNEL *initial_chan)
{
	struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;
    HAL_STATUS status;
    int error = 0;
    HAL_HT_MACMODE ht_macmode = sc->sc_ieee_ops->cwm_macmode(sc->sc_ieee);

    //ATH_LOCK(sc);

    DPRINTF(sc, ATH_DEBUG_RESET, "%s: mode %d\n", __func__, sc->sc_opmode);

    ath_pwrsave_init(sc);

    /*
     * Stop anything previously setup.  This is safe
     * whether this is the first time through or not.
     */
    ath_stop_locked(sc);

#if ATH_CAP_TPC
    ath_hal_setcapability(ah, HAL_CAP_TPC, 0, 1, NULL);
#endif

    /* Initialize chanmask selection */
    sc->sc_tx_chainmask = sc->sc_config.txchainmask;
    sc->sc_rx_chainmask = sc->sc_config.rxchainmask;

    /* Start ForcePPM module */
    ath_start_force_ppm_module(&sc->sc_ppm_info);

    /* Reset SERDES registers */
    ath_hal_configpcipowersave(ah, 0);

    /*
     * The basic interface to setting the hardware in a good
     * state is ``reset''.  On return the hardware is known to
     * be powered up and with interrupts disabled.  This must
     * be followed by initialization of the appropriate bits
     * and then setup of the interrupt mask.
     */
    sc->sc_curchan = *initial_chan;

    ATH_RESET_LOCK(sc);
    if (!ath_hal_reset(ah, sc->sc_opmode, &sc->sc_curchan, ht_macmode,
                       sc->sc_tx_chainmask, sc->sc_rx_chainmask,
                       sc->sc_ht_extprotspacing, AH_FALSE, &status))
    {
        printk("%s: unable to reset hardware; hal status %u "
               "(freq %u flags 0x%x)\n", __func__, status,
               sc->sc_curchan.channel, sc->sc_curchan.channelFlags);
        error = -EIO;
        ATH_RESET_UNLOCK(sc);
        goto done;
    }
    ATH_RESET_UNLOCK(sc);
    /*
     * This is needed only to setup initial state
     * but it's best done after a reset.
     */
    ath_update_txpow(sc, 0);

    /*
     * Setup the hardware after reset: the key cache
     * is filled as needed and the receive engine is
     * set going.  Frame transmit is handled entirely
     * in the frame output path; there's nothing to do
     * here except setup the interrupt mask.
     */
#if 0
    ath_initkeytable(sc);		/* XXX still needed? */
#endif
    if (ath_startrecv(sc) != 0) {
        printk("%s: unable to start recv logic\n", __func__);
        error = -EIO;
        goto done;
    }
    /*
     *  Setup our intr mask.
     */
    sc->sc_imask = HAL_INT_RX | HAL_INT_TX
        | HAL_INT_RXEOL | HAL_INT_RXORN
        | HAL_INT_FATAL | HAL_INT_GLOBAL;

    if (ath_hal_gttsupported(ah))
        sc->sc_imask |= HAL_INT_GTT;

    if (sc->sc_hashtsupport)
        sc->sc_imask |= HAL_INT_CST;

    /*
     * Enable MIB interrupts when there are hardware phy counters.
     * Note we only do this (at the moment) for station mode.
     */
    if (sc->sc_needmib &&
        ((sc->sc_opmode == HAL_M_STA) ||
        (sc->sc_opmode == HAL_M_IBSS))) {
        sc->sc_imask |= HAL_INT_MIB;
    }

    /*
     * Some hardware processes the TIM IE and fires an
     * interrupt when the TIM bit is set.  For hardware
     * that does, if not overridden by configuration,
     * enable the TIM interrupt when operating as station.
     */
    if (ath_hal_hasenhancedpmsupport(ah) &&
        sc->sc_opmode == HAL_M_STA &&
        !sc->sc_config.swBeaconProcess) {
	    sc->sc_imask |= HAL_INT_TIM;
    }

#ifdef ATH_RFKILL
    if (sc->sc_hasrfkill) {
        if (ath_rfkill_hasint(sc))
            sc->sc_imask |= HAL_INT_GPIO;

        /*
         * WAR for Bug 33276: In certain systems, the RfKill signal is slow to
         * stabilize when waking up from power suspend. The RfKill is an
         * active-low GPIO signal and the problem is the slow rise from 0V to VCC.
         * For this workaround, we will delayed implementing the new RfKill
         * state if there is a change in RfKill during the sleep. This WAR is only
         * when the previous RfKill state is OFF and the new awaken state is ON.
         */
        if (sc->sc_reg_parm.rfKillDelayDetect) {
            ath_rfkill_delay_detect(sc);
        }

        /*
         * Bug 36651
         * Hw switch might have been changed while rfkill polling is stopped. sc_hw_phystate
         * needs to be updated here. Call ath_rfkill_poll() to get the latest state. Instead
         * of just updating sc_hw_phystate, which doesn't call any radio enable/disable function
         * if sc_hw_phystate is changed.
         */
        ath_rfkill_poll(sc);
        ath_rfkill_start_polling(sc);
    }
#endif
    
    if (sc->sc_wpsgpiointr)
        sc->sc_imask |= HAL_INT_GPIO;

#ifdef ATH_BT_COEX
    if (sc->sc_btinfo.bt_gpioIntEnabled) {
        sc->sc_imask |= HAL_INT_GPIO;
    }
#endif
    /*
     *  Don't enable interrupts here as we've not yet built our
     *  vap and node data structures, which will be needed as soon
     *  as we start receiving.
     */
  
    ath_chan_change(sc, initial_chan);

    if (!sc->sc_reg_parm.pcieDisableAspmOnRfWake) {
        ath_pcie_pwrsave_enable(sc, 1);
    } else {
        ath_pcie_pwrsave_enable(sc, 0);
    }

    /* XXX: we must make sure h/w is ready and clear invalid flag
     * before turning on interrupt. */
    sc->sc_invalid = 0;
    
    /* Start LED module; pass radio state as parameter */
    ath_start_led_control(&sc->sc_led_control, 
                          sc->sc_hw_phystate && sc->sc_sw_phystate);

done:
    //ATH_UNLOCK(sc);
    return error;
}

/*
 * Stop the device, grabbing the top-level lock to protect
 * against concurrent entry through ath_init (which can happen
 * if another thread does a system call and the thread doing the
 * stop is preempted).
 */
static int
ath_stop(ath_dev_t dev)
{
	struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    int error;

    //ATH_LOCK(sc);

    if (!sc->sc_invalid) {
        ath_pwrsave_awake(sc);

        ath_led_disable(&sc->sc_led_control);
    }

    error = ath_stop_locked(sc);
#if 0
    if (error == 0 && !sc->sc_invalid) {
        /*
         * Set the chip in full sleep mode.  Note that we are
         * careful to do this only when bringing the interface
         * completely to a stop.  When the chip is in this state
         * it must be carefully woken up or references to
         * registers in the PCI clock domain may freeze the bus
         * (and system).  This varies by chip and is mostly an
         * issue with newer parts that go to sleep more quickly.
         */
        ath_hal_setpower(sc->sc_ah, HAL_PM_FULL_SLEEP);
    }
#endif
    //ATH_UNLOCK(sc);
#ifdef ATH_RFKILL
    if (sc->sc_hasrfkill) {
        ath_rfkill_stop_polling(sc);
    }
#endif
    return error;
}

/*
 * Reset the hardware w/o losing operational state.  This is
 * basically a more efficient way of doing ath_stop, ath_init,
 * followed by state transitions to the current 802.11
 * operational state.  Used to recover from errors rx overrun
 * and to reset the hardware when rf gain settings must be reset.
 */
int
ath_reset_start(ath_dev_t dev, u_int32_t flag)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;

    ATH_PS_WAKEUP(sc);

    ath_hal_intrset(ah, 0);                     /* disable interrupts */
    ath_draintxq(sc, flag & RESET_RETRY_TXQ);   /* stop xmit side */
    ath_stoprecv(sc);                           /* stop recv side */

    /*
     * We cannot indicate received packets while holding a lock.
     * Use busy wait here.
     */
    while (cmpxchg(&sc->sc_rxflush, 0, 1) == 1);
    if (flag & RESET_RETRY_TXQ) {
        ath_rx_tasklet(sc, RX_FORCE_PROCESS);
    }
    else {
        ath_rx_tasklet(sc, RX_DROP);
    }
    if (cmpxchg(&sc->sc_rxflush, 1, 0) != 1) {
        DPRINTF(sc, ATH_DEBUG_RX_PROC, "%s: rx queue is not protected.\n", __func__);
    }

    /* Suspend ForcePPM when entering a reset */
    ath_force_ppm_notify(&sc->sc_ppm_info, ATH_FORCE_PPM_SUSPEND, NULL);

    ATH_PS_SLEEP(sc);
    return 0;
}

int
ath_reset_end(ath_dev_t dev, u_int32_t flag)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;

    ATH_PS_WAKEUP(sc);

    if (ath_startrecv(sc) != 0)	/* restart recv */
        printk("%s: unable to start recv logic\n", __func__);

    /*
     * We may be doing a reset in response to an ioctl
     * that changes the channel so update any state that
     * might change as a result.
     */
    ath_chan_change(sc, &sc->sc_curchan);

    ath_update_txpow(sc, 0);		/* update tx power state */

    if (sc->sc_beacons)
        ath_beacon_config(sc, ATH_IF_ID_ANY);   /* restart beacons */
    ath_hal_intrset(ah, sc->sc_imask);

#ifdef ATH_SUPERG_XR
    /*
     * restart the group polls.
     */
    if (sc->sc_xrgrppoll) {
        struct ieee80211vap *vap;
        TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next)
            if(vap && (vap->iv_flags & IEEE80211_F_XR)) break;
        ath_grppoll_stop(vap);
        ath_grppoll_start(vap,sc->sc_xrpollcount);
    }
#endif

    /* Resume ForcePPM after reset is completed */
    ath_force_ppm_notify(&sc->sc_ppm_info, ATH_FORCE_PPM_RESUME, NULL);
   
    /* Restart the txq */
    if (flag & RESET_RETRY_TXQ) {
        int i;
        for (i = 0; i < HAL_NUM_TX_QUEUES; i++) {
            if (ATH_TXQ_SETUP(sc, i)) {
                ATH_TXQ_LOCK(&sc->sc_txq[i]);
                ath_txq_schedule(sc, &sc->sc_txq[i]);
                ATH_TXQ_UNLOCK(&sc->sc_txq[i]);
            }
        }
    }

    ATH_PS_SLEEP(sc);
    return 0;
}

int
ath_reset(ath_dev_t dev)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;
    HAL_STATUS status;
    int error = 0;
    HAL_HT_MACMODE ht_macmode = sc->sc_ieee_ops->cwm_macmode(sc->sc_ieee);

    ATH_PS_WAKEUP(sc);

    /* NB: indicate channel change so we do a full reset */
    ATH_RESET_LOCK(sc);
    if (!ath_hal_reset(ah, sc->sc_opmode, &sc->sc_curchan,
                       ht_macmode,
                       sc->sc_tx_chainmask, sc->sc_rx_chainmask,
                       sc->sc_ht_extprotspacing,
                       AH_FALSE, &status))
    {
        printk("%s: unable to reset hardware; hal status %u\n",
               __func__, status);
        error = -EIO;
    }
    ATH_RESET_UNLOCK(sc);

    ATH_PS_SLEEP(sc);
    return error;
}

static int
ath_switch_opmode(ath_dev_t dev, HAL_OPMODE opmode)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    if (sc->sc_opmode != opmode) {
        DPRINTF(sc, ATH_DEBUG_STATE, "%s: switch opmode from %d to %d\n", __func__,sc->sc_opmode, opmode);
        sc->sc_opmode = opmode;
        ath_reset_start(dev,0);
        ath_reset(dev);
        ath_reset_end(dev,0);

    }
    return 0;
}

static int
ath_suspend(ath_dev_t dev)
{
	struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    
    /*
     * No I/O if device has been surprise removed
     */
    if (sc->sc_invalid)
        return -EIO;

    if (ath_get_sw_phystate(sc) && ath_get_hw_phystate(sc)) {
        /* stop the hardware */
        ath_stop(sc);
    }
    else {
        struct ath_hal *ah = sc->sc_ah;

        /*
         * Work around for bug #24058: OS hang during restart after software RF off.
         * Condore has problem waking up on some machines (no clue why) if ath_suspend is called
         * in which nothing is done when RF is off.
         */
        ath_pwrsave_awake(sc);

        /* Shut off the interrupt before setting sc->sc_invalid to '1' */
        ath_hal_intrset(ah, 0);

#ifdef ATH_RFKILL
        /* Even though the radio is OFF, this timer might still be alive. Stop it. */
        if (sc->sc_hasrfkill) {
            ath_rfkill_stop_polling(sc);
        }
#endif
        
        /* Even though the radio is OFF, the LED might still be alive. Disable it. */
        ath_led_disable(&sc->sc_led_control);
    }
    
    /* XXX: we must make sure h/w will not generate any interrupt
     * before setting the invalid flag. */
    sc->sc_invalid = 1;

    
    /* disable HAL and put h/w to sleep */
    ath_hal_disable(sc->sc_ah);

    ath_hal_configpcipowersave(sc->sc_ah, 1);

    ath_pwrsave_fullsleep(sc);

    return 0;
}

#ifdef notyet
static void
ath_fatal_tasklet(void *data)
{
    struct ath_softc *sc = (struct ath_softc *)data;

    printk("hardware error; reseting\n");
    ath_internal_reset(sc);
    //TBD
    //sc->sc_stats.ast_resetOnError++;
}

static void
ath_rxorn_tasklet(void *data)
{
    struct ath_softc *sc = (struct ath_softc *)data;

    printk("rx FIFO overrun; reseting\n");
    ath_internal_reset(sc);
    //TBD
    //sc->sc_stats.ast_resetOnError++;
}
#endif

static void
ath_enable_intr(ath_dev_t dev)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    ath_hal_intrset(sc->sc_ah, sc->sc_imask);
}

static void
ath_disable_intr(ath_dev_t dev)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    ath_hal_intrset(sc->sc_ah, 0);
}

#ifdef ATH_MIB_INTR_FILTER
/*
 * Filter excessive MIB interrupts so that the system doesn't freeze because
 * it cannot handle the lower-priority threads.
 *
 * Algorithm:
 *     -The beginning of a possible burst is characterized by two consecutive
 *      MIB interrupts arriving within MIB_FILTER_MAX_INTR_ELAPSED_TIME (20ms) 
 *      of each other.
 *
 *     -A MIB interrupt is considered part of a burst if it arrives within
 *      MIB_FILTER_MAX_INTR_ELAPSED_TIME (20ms) of the previous one AND within
 *      MIB_FILTER_MAX_BURST_ELAPSED_TIME (100ms) of the beginning of the burst.
 *
 *     -Once the number of consecutive MIB interrupts reaches
 *      MIB_FILTER_COUNT_THRESHOLD (500) we disable reception of MIB interrupts.
 *
 *     -Reception of a MIB interrupt that is longer part of the burst or 
 *      reception of a different interrupt cause the counting to start over.
 *
 *     -Once the MIB interrupts have been disabled, we wait for 
 *      MIB_FILTER_RECOVERY_TIME (50ms) and then reenable MIB interrupts upon   
 *      reception of the next non-MIB interrupt.
 *
 * Miscellaneous:
 *     -The algorithm is always enabled if ATH_MIB_INTR_FILTER is defined.
 */
static void ath_filter_mib_intr(struct ath_softc *sc, u_int8_t is_mib_intr)
{
    struct ath_intr_filter    *intr_filter  = &sc->sc_intr_filter;
    u_int32_t                 current_time  = (u_int32_t) CONVERT_SYSTEM_TIME_TO_MS(OS_GET_TIMESTAMP());
    u_int32_t                 intr_interval = (u_int32_t) CONVERT_SYSTEM_TIME_TO_MS(current_time - intr_filter->last_intr_time);
    u_int32_t                 burst_dur     = (u_int32_t) CONVERT_SYSTEM_TIME_TO_MS(current_time - intr_filter->burst_start_time);

    switch (intr_filter->state) {
    case INTR_FILTER_OFF:
        /*
         * Two MIB interrupts arriving too close to each other may mark the
         * beggining of a burst of MIB interrupts.
         */
        if (is_mib_intr && (intr_interval <= MIB_FILTER_MAX_INTR_ELAPSED_TIME)) {
            intr_filter->state            = INTR_FILTER_DEGLITCHING;
            intr_filter->burst_start_time = current_time;
            intr_filter->intr_count++;
        }
        break;

    case INTR_FILTER_DEGLITCHING:
        /*
         * Consider a MIB interrupt arrived within a short time of the
         * previous one and withing a short time of the beginning of the burst
         * to be still part of the same burst.
         */
        if (is_mib_intr && 
            (intr_interval <= MIB_FILTER_MAX_INTR_ELAPSED_TIME) &&
            (burst_dur <= MIB_FILTER_MAX_BURST_ELAPSED_TIME)) {
            intr_filter->intr_count++;

            /*
             * Too many consecutive within a short time of each other 
             * characterize a MIB burst ("storm")
             */
            if (intr_filter->intr_count >= MIB_FILTER_COUNT_THRESHOLD) {
                /* MIB burst identified */
                intr_filter->activation_count++;
                intr_filter->state = INTR_FILTER_ON;

                /* Disable MIB interrupts */
                sc->sc_imask &= ~HAL_INT_MIB;

                DPRINTF(sc, ATH_DEBUG_INTR, "%d.%03d | %s: Start Mib Storm index=%3d intrCount=%8d start=%d.%03d duration=%6u ms\n",
                    current_time / 1000, current_time % 1000, __func__, 
                    intr_filter->activation_count,
                    intr_filter->intr_count,
                    ((u_int32_t) CONVERT_SYSTEM_TIME_TO_MS(intr_filter->burst_start_time)) / 1000, 
                    ((u_int32_t) CONVERT_SYSTEM_TIME_TO_MS(intr_filter->burst_start_time)) % 1000,
                    burst_dur);

                /* Want a register dump? */
                /* ath_printreg(sc, HAL_DIAG_PRINT_REG_COUNTER | HAL_DIAG_PRINT_REG_ALL); */
            }
        }
        else {
            /*
             * The gap from the previous interrupt is long enough to 
             * characterize the latest as not being part of a burst.
             * Go back to the initial state.
             */
            intr_filter->state      = INTR_FILTER_OFF;
            intr_filter->intr_count = 1;
        }
        break;

    case INTR_FILTER_ON:
        /* 
         * Wait for a non-MIB interrupt to be received more than a certain time
         * after the last MIB interrupt to consider it the end of the interrupt 
         * burst                                                      UE
         */
        if (! is_mib_intr) {
            if (intr_interval > MIB_FILTER_RECOVERY_TIME) {
                DPRINTF(sc, ATH_DEBUG_INTR, "%d.%03d | %s: Mib Storm index=%3d intrCount=%8d start=%d.%03d duration=%6u ms\n",
                    current_time / 1000, current_time % 1000, __func__, 
                    intr_filter->activation_count,
                    intr_filter->intr_count,
                    ((u_int32_t) CONVERT_SYSTEM_TIME_TO_MS(intr_filter->burst_start_time)) / 1000, 
                    ((u_int32_t) CONVERT_SYSTEM_TIME_TO_MS(intr_filter->burst_start_time)) % 1000,
                    burst_dur);

                /* Re-enable MIB interrupt */
                sc->sc_imask |= HAL_INT_MIB;

                /* Return to the initial state and monitor for new bursts */
                intr_filter->state      = INTR_FILTER_OFF;
                intr_filter->intr_count = 1;
            }
        }
        else {
            /* Another MIB interrupt generated was we were disabling them */
            intr_filter->intr_count++;
        }
        break;

    default:
        /* Bad state */
        ASSERT(0);
        break;
    }

    /* Do not save timestamp of non-MIB interrupts */
    if (is_mib_intr) {
        intr_filter->last_intr_time = current_time;
    }
}
#endif

/*
 * Interrupt handler.  Most of the actual processing is deferred.
 *
 * It's the caller's responsibility to ensure the chip is awake.
 */
static int
ath_intr(ath_dev_t dev)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;
    HAL_INT status;
    int sched = ATH_ISR_NOSCHED;

    do {
        if (sc->sc_invalid) {
            /*
             * The hardware is not ready/present, don't touch anything.
             * Note this can happen early on if the IRQ is shared.
             */
            return (ATH_ISR_NOTMINE);
        }
        if (!ath_hal_intrpend(ah)) {	/* shared irq, not for us */
            return (ATH_ISR_NOTMINE); 
        }

        /*
         * Figure out the reason(s) for the interrupt.  Note
         * that the hal returns a pseudo-ISR that may include
         * bits we haven't explicitly enabled so we mask the
         * value to insure we only process bits we requested.
         */
        ath_hal_getisr(ah, &status);		/* NB: clears ISR too */
        DPRINTF(sc, ATH_DEBUG_INTR, "%s: status 0x%x  Mask: 0x%x\n",
                __func__, status, sc->sc_imask);

        status &= sc->sc_imask;			/* discard unasked-for bits */

        /*
        ** If there are no status bits set, then this interrupt was not
        ** for me (should have been caught above).
        */

        if(!status)
        {
            DPRINTF(sc, ATH_DEBUG_INTR, "%s: Not My Interrupt\n",__func__);
            return(ATH_ISR_NOTMINE);
        }

        sc->sc_intrstatus |= status;

#ifdef ATH_MIB_INTR_FILTER
        /* Notify the MIB interrupt filter that we received some other interrupt. */
        if (! (status & HAL_INT_MIB)) {
            ath_filter_mib_intr(sc, AH_FALSE);
        }
#endif

        if (status & HAL_INT_FATAL) {
            /* need a chip reset */
            sc->sc_stats.ast_hardware++;
            sched = ATH_ISR_SCHED;

        } else if (status & HAL_INT_RXORN) {
            /* need a chip reset? */
#ifdef ATH_SUPPORT_UAPSD
            ath_check_uapsdtriggers(dev);
#endif
            sc->sc_stats.ast_rxorn++;
            sched = ATH_ISR_SCHED;
        } else {
            if (status & HAL_INT_SWBA) {
                int needmark = 0;

                /*
                 * Software beacon alert--time to send a beacon.
                 * Handle beacon transmission directly; deferring
                 * this is too slow to meet timing constraints
                 * under load.
                 */
                ath_beacon_tasklet(sc, &needmark);

                if (needmark) {
                    /* We have a beacon stuck. Beacon stuck processing
                     * should be done in DPC instead of here. */
                    sched = ATH_ISR_SCHED;
                }
            }
            if (status & HAL_INT_RXEOL) {
                /*
                 * NB: the hardware should re-read the link when
                 *     RXE bit is written, but it doesn't work at
                 *     least on older hardware revs.
                 */
#ifdef ATH_SUPPORT_UAPSD
                ath_check_uapsdtriggers(dev);
#endif
                sc->sc_stats.ast_rxeol++;
                sched = ATH_ISR_SCHED;
            }

            if (status & HAL_INT_TXURN) {
                sc->sc_stats.ast_txurn++;
                /* bump tx trigger level */
                ath_hal_updatetxtriglevel(ah, AH_TRUE);
            }
            if (status & HAL_INT_RX) {
#ifdef ATH_SUPPORT_UAPSD
                ath_check_uapsdtriggers(dev);
#endif
                sched = ATH_ISR_SCHED;
            }

            if (status & HAL_INT_TX) {
#ifdef ATH_SUPERG_DYNTURBO
                /*
                 * Check if the beacon queue caused the interrupt
                 * when a dynamic turbo switch
                 * is pending so we can initiate the change.
                 * XXX must wait for all vap's beacons
                 */

                if (sc->sc_opmode == HAL_M_HOSTAP && sc->sc_dturbo_switch) {
                    u_int32_t txqs= (1 << sc->sc_bhalq);
                    ath_hal_gettxintrtxqs(ah,&txqs);
                    if(txqs & (1 << sc->sc_bhalq)) {
                        sc->sc_dturbo_switch = 0;
                        /*
                         * Hack: defer switch for 10ms to permit slow
                         * clients time to track us.  This especially
                         * noticeable with Windows clients.
                         */
#ifdef notyet
                        mod_timer(&sc->sc_dturbo_switch_mode,
                                  jiffies + ((HZ * 10) / 1000));
#endif

                    }
                }
#endif
                sched = ATH_ISR_SCHED;
            }
            if (status & HAL_INT_BMISS) {
                sc->sc_stats.ast_bmiss++;
#if ATH_WOW
                if (sc->sc_wow_sleep) {
                    /* 
                     * The system is in WOW sleep and we used the BMISS intr to
                     * wake the system up. Note this BMISS by setting the 
                     * sc_wow_bmiss_intr flag but do not process this interrupt.
                     */
                    printk("%s: During Wow Sleep and got BMISS\n", __func__);
                    sc->sc_wow_bmiss_intr = 1;
                    sc->sc_wow_sleep = 0;
                    sched = ATH_ISR_NOSCHED;
                } else
#endif
                sched = ATH_ISR_SCHED;
            }
            if (status & HAL_INT_GTT) { /* tx timeout interrupt */
                sc->sc_stats.ast_txto++;
            }
            if (status & HAL_INT_CST) { /* carrier sense timeout */
                sc->sc_stats.ast_cst++;
                sched = ATH_ISR_SCHED;
            }

            if (status & HAL_INT_MIB) {
                sc->sc_stats.ast_mib++;
                /*
                 * Disable interrupts until we service the MIB
                 * interrupt; otherwise it will continue to fire.
                 */
                ath_hal_intrset(ah, 0);

#ifdef ATH_MIB_INTR_FILTER
                /* Check for bursts of MIB interrupts */
                ath_filter_mib_intr(sc, AH_TRUE);
#endif

                /*
                 * Let the hal handle the event.  We assume it will
                 * clear whatever condition caused the interrupt.
                 */
                ath_hal_mibevent(ah, &sc->sc_halstats);
                ath_hal_intrset(ah, sc->sc_imask);
            }
            if (status & HAL_INT_GPIO) {
                /* Check if this GPIO interrupt is caused by RfKill */
#ifdef ATH_RFKILL
                if (ath_rfkill_gpio_isr(sc))
                    sched = ATH_ISR_SCHED;
#endif
                if (sc->sc_wpsgpiointr) {
                    /* Check for WPS push button press (GPIO polarity low) */
                    if (ath_hal_gpioget(sc->sc_ah, sc->sc_reg_parm.wpsButtonGpio) == 0) {
                        sc->sc_wpsbuttonpushed = 1;

                        /* Disable associated GPIO interrupt to prevent flooding */
                        ath_hal_gpioSetIntr(ah, sc->sc_reg_parm.wpsButtonGpio, HAL_GPIO_INTR_DISABLE);
                        sc->sc_wpsgpiointr = 0;
                    }
				}
#ifdef ATH_BT_COEX
                if (sc->sc_btinfo.bt_gpioIntEnabled) {
                    sched = ATH_ISR_SCHED;
                }
#endif
            }
            if (status & HAL_INT_TIM_TIMER) {
                if (! sc->sc_hasautosleep) {
                    /* Clear RxAbort bit so that we can receive frames */
                    ath_hal_setrxabort(ah, 0);
                    /* Set flag indicating we're waiting for a beacon */
                    sc->sc_waitbeacon = 1;

                    sched = ATH_ISR_SCHED;
                }
            }
        }
    } while (0);

    if (sched == ATH_ISR_SCHED) {
        DPRINTF(sc, ATH_DEBUG_INTR, "%s: Scheduling BH/DPC\n",__func__);
        /* turn off every interrupt except SWBA */
        ath_hal_intrset(ah, (sc->sc_imask & HAL_INT_SWBA));
    }

    return sched;

}

/*
 * Deferred interrupt processing
 */
static void
ath_handle_intr(ath_dev_t dev)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    u_int32_t status = sc->sc_intrstatus;
    sc->sc_intrstatus &= (~status);

    ATH_PS_WAKEUP(sc);

    if (status & HAL_INT_FATAL) {
        /* need a chip reset */
        DPRINTF(sc, ATH_DEBUG_INTR, "%s: Got fatal intr\n", __func__);
        ath_internal_reset(sc);
        ATH_PS_SLEEP(sc);
        return;
    } else {
        if (((AH_TRUE == sc->sc_hang_check) && ath_hw_hang_check(sc)) ||
            (!sc->sc_noreset && (sc->sc_bmisscount >= sc->sc_bsthresh))) {
            ath_bstuck_tasklet(sc);
            ATH_CLEAR_HANGS(sc);
            ATH_PS_SLEEP(sc);
            return;
        }
		/*
		 * Howl needs DDR FIFO flush before any desc/dma data can be read.
		 */
		ATH_FLUSH_FIFO();

        if (status & (HAL_INT_RX | HAL_INT_RXEOL | HAL_INT_RXORN)) {
            if (status & HAL_INT_RXORN) {
                DPRINTF(sc, ATH_DEBUG_INTR, "%s: Got RXORN intr\n", __func__);
            }
            if (status & HAL_INT_RXEOL) {
                DPRINTF(sc, ATH_DEBUG_INTR, "%s: Got RXEOL intr\n", __func__);
            }
            ath_handle_rx_intr(sc);
        }
        if (status & HAL_INT_TX) {
            ath_handle_tx_intr(sc);
        }
        if (status & HAL_INT_BMISS) {
            ath_bmiss_tasklet(sc);
        }
        if (status & HAL_INT_CST) {
            ath_txto_tasklet(sc);
        }
        if (status & (HAL_INT_TIM | HAL_INT_DTIMSYNC)) {
            if (status & HAL_INT_TIM) {
                if (sc->sc_ieee_ops->proc_tim)
                    sc->sc_ieee_ops->proc_tim(sc->sc_ieee);
            }
            if (status & HAL_INT_DTIMSYNC) {
                DPRINTF(sc, ATH_DEBUG_INTR, "%s: Got DTIMSYNC intr\n", __func__);
            }
        }
        if (status & HAL_INT_GPIO) {
#ifdef ATH_RFKILL
            ath_rfkill_gpio_intr(sc);
#endif
#ifdef ATH_BT_COEX
            if (sc->sc_btinfo.bt_gpioIntEnabled) {
                ath_bt_coex_gpio_intr(sc);
            }
#endif
        }
    }

    /* re-enable hardware interrupt */
    ath_hal_intrset(sc->sc_ah, sc->sc_imask);
    ATH_PS_SLEEP(sc);
}

/*
 * ATH symbols exported to the HAL.
 * Make table static so that clients can keep a pointer to it
 * if they so choose.
 */
static u_int32_t
ath_read_pci_config_space(struct ath_softc *sc, u_int32_t offset, void *buffer, u_int32_t len)
{
    return OS_PCI_READ_CONFIG(sc->sc_osdev, offset, buffer, len);
}

static const struct ath_hal_callback halCallbackTable = {
    /* Callback Functions */
    /* read_pci_config_space */ (HAL_BUS_CONFIG_READER)ath_read_pci_config_space
};

static int
ath_get_caps(ath_dev_t dev, ATH_CAPABILITY_TYPE type)
{
	struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;
    int supported = 0;

    switch (type) {
    case ATH_CAP_FF:
        supported = ath_hal_fastframesupported(ah);
        break;
    case ATH_CAP_BURST:
        supported = ath_hal_burstsupported(ah);
        break;
    case ATH_CAP_COMPRESSION:
        supported = sc->sc_hascompression;
        break;
    case ATH_CAP_TURBO:
        supported = ath_hal_turboagsupported(ah, sc->sc_config.ath_countrycode);
        break;
    case ATH_CAP_XR:
        supported = ath_hal_xrsupported(ah);
        break;
    case ATH_CAP_TXPOWER:
        supported = (sc->sc_hastpc || ath_hal_hastxpowlimit(ah));
        break;
    case ATH_CAP_DIVERSITY:
        supported = sc->sc_diversity;
        break;
    case ATH_CAP_BSSIDMASK:
        supported = sc->sc_hasbmask;
        break;
    case ATH_CAP_TKIP_SPLITMIC:
        supported = ath_hal_tkipsplit(ah);
        break;
    case ATH_CAP_MCAST_KEYSEARCH:
        supported = ath_hal_getmcastkeysearch(ah);
        break;
    case ATH_CAP_TKIP_WMEMIC:
        supported = ath_hal_wmetkipmic(ah);
        break;
    case ATH_CAP_WMM:
        supported = sc->sc_haswme;
        break;
    case ATH_CAP_HT:
        supported = sc->sc_hashtsupport;
        break;
    case ATH_CAP_RX_STBC:
        supported = sc->sc_rxstbcsupport;
        break;
    case ATH_CAP_TX_STBC:
        supported = sc->sc_txstbcsupport;
        break;
    case ATH_CAP_4ADDR_AGGR:
        supported = ath_hal_4addraggrsupported(ah);
        break;
    case ATH_CAP_ENHANCED_PM_SUPPORT:
        supported = ath_hal_hasenhancedpmsupport(ah);
        break;
    case ATH_CAP_WPS_BUTTON:
        if (ath_hal_haswpsbutton(ah) && sc->sc_reg_parm.wpsButtonGpio) {

            /* Overload push button status when reporting capability */
            supported |= (ATH_WPS_BUTTON_EXISTS | ATH_WPS_BUTTON_DOWN_SUP | ATH_WPS_BUTTON_STATE_SUP);

            supported |= (sc->sc_wpsbuttonpushed ? ATH_WPS_BUTTON_PUSHED : 0);
            sc->sc_wpsbuttonpushed = 0;  /* Clear status after query */

            /* Get current push button status, GPIO polarity low */
            if (ath_hal_gpioget(sc->sc_ah, sc->sc_reg_parm.wpsButtonGpio)) {
                /* GPIO line normal, re-enable GPIO interrupt */
                ath_hal_gpioSetIntr(ah, sc->sc_reg_parm.wpsButtonGpio, HAL_GPIO_INTR_LOW);
                sc->sc_wpsgpiointr = 1;
            } else {
                supported |= ATH_WPS_BUTTON_PUSHED_CURR;
            }
        }
        break;
    case ATH_CAP_MBSSID_AGGR_SUPPORT:/* MBSSID Aggregation support capability*/
          ath_hal_hasMbssidAggrSupport(ah,&supported);
          break;
#ifdef ATH_SWRETRY
    case ATH_CAP_SWRETRY_SUPPORT:
        supported = sc->sc_swRetryEnabled;
        break;
#endif
#ifdef ATH_SUPPORT_UAPSD
    case ATH_CAP_UAPSD:
        supported = sc->sc_uapsdsupported;
        break;
#endif
    case ATH_CAP_DYNAMIC_SMPS:
        supported = ath_hal_hasdynamicsmps(ah);
        break;
    case ATH_CAP_WEP_TKIP_AGGR:
        supported = ath_hal_weptkipaggrsupport(ah);
        break;
    case ATH_CAP_DS:
        supported = ath_hal_getcapability(ah, HAL_CAP_DS, 0, NULL) == HAL_OK ? TRUE : FALSE;
        break;
    case ATH_CAP_SINGLE_AGGR_SAFE:
        supported = ath_hal_singleframeaggrsupport(ah);
        break; 	    
    default:
        supported = 0;
        break;
    }
    return supported;
}

static int
ath_get_ciphers(ath_dev_t dev, HAL_CIPHER cipher)
{
    return ath_hal_ciphersupported(ATH_DEV_TO_SC(dev)->sc_ah, cipher);
}

static struct ath_phy_stats *
ath_get_phy_stats(ath_dev_t dev, WIRELESS_MODE wmode)
{
    return &(ATH_DEV_TO_SC(dev)->sc_phy_stats[wmode]);
}

static struct ath_stats *
ath_get_ath_stats(ath_dev_t dev)
{
    return &(ATH_DEV_TO_SC(dev)->sc_stats);
}

static struct ath_11n_stats *
ath_get_11n_stats(ath_dev_t dev)
{
    return &(ATH_DEV_TO_SC(dev)->sc_stats.ast_11n_stats);
}

static void
ath_clear_stats(ath_dev_t dev)
{
    OS_MEMZERO(&(ATH_DEV_TO_SC(dev)->sc_phy_stats[0]),
               sizeof(struct ath_phy_stats) * WIRELESS_MODE_MAX);
}

static void
ath_update_beacon_info(ath_dev_t dev, int avgbrssi)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);

    /* Update beacon RSSI */
    sc->sc_halstats.ns_avgbrssi = avgbrssi;

    if (! sc->sc_hasautosleep) {
        sc->sc_waitbeacon = 0;
    }
}

static void
ath_set_macmode(ath_dev_t dev, HAL_HT_MACMODE macmode)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);

    ATH_FUNC_ENTRY_VOID(sc);
    ATH_PS_WAKEUP(sc);
    ath_hal_set11nmac2040(sc->sc_ah, macmode);
    ATH_PS_SLEEP(sc);
}

static void
ath_set_extprotspacing(ath_dev_t dev, HAL_HT_EXTPROTSPACING extprotspacing)
{
    ATH_DEV_TO_SC(dev)->sc_ht_extprotspacing = extprotspacing;
}

static int
ath_get_extbusyper(ath_dev_t dev)
{
    return ath_hal_get11nextbusy(ATH_DEV_TO_SC(dev)->sc_ah);
}

#ifndef REMOVE_PKT_LOG
static int
ath_start_pktlog(ath_dev_t dev, int log_state)
{
    int error;
    ath_pktlog_start(ATH_DEV_TO_SC(dev), log_state, error);
    return error;
}

static int
ath_read_pktlog_hdr(ath_dev_t dev, void *buf, u_int32_t buf_len,
                    u_int32_t *required_len,
                    u_int32_t *actual_len)
{
    int error;
    ath_pktlog_read_hdr(ATH_DEV_TO_SC(dev), buf, buf_len,
                        required_len, actual_len, error);
    return error;
}

static int
ath_read_pktlog_buf(ath_dev_t dev, void *buf, u_int32_t buf_len,
                    u_int32_t *required_len,
                    u_int32_t *actual_len)
{
    int error;
    ath_pktlog_read_buf(ATH_DEV_TO_SC(dev), buf, buf_len,
                        required_len, actual_len, error);
    return error;
}
#endif

#ifdef ATH_SUPPORT_IQUE
static void
ath_setacparams(ath_dev_t dev, u_int8_t ac, u_int8_t use_rts,
                u_int8_t aggrsize_scaling, u_int32_t min_kbps)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);

    if (ac < WME_NUM_AC) {
        sc->sc_ac_params[ac].use_rts = use_rts;
        sc->sc_ac_params[ac].min_kbps = min_kbps;
        sc->sc_ac_params[ac].aggrsize_scaling = aggrsize_scaling;
    }
}

static void
ath_setrtparams(ath_dev_t dev, u_int8_t rt_index, u_int8_t perThresh,
                u_int8_t probeInterval)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);

    if (rt_index < 2) {
        sc->sc_rc_params[rt_index].per_threshold = perThresh;
        sc->sc_rc_params[rt_index].probe_interval = probeInterval;
    }
}

static void
ath_sethbrparams(ath_dev_t dev, u_int8_t ac, u_int8_t enable, u_int8_t perlow)
{
	struct ath_softc *sc = ATH_DEV_TO_SC(dev);
	u_int8_t perhigh;
	perhigh = sc->sc_rc_params[0].per_threshold;
	if (perhigh < sc->sc_rc_params[1].per_threshold)
		perhigh = sc->sc_rc_params[1].per_threshold;

	/* Now we only support HBR for VI */
	if (ac == WME_AC_VI) {
		sc->sc_hbr_params[ac].hbr_enable = enable;
		sc->sc_hbr_params[ac].hbr_per_low = (perlow > (perhigh - 10))? (perhigh-10):perlow;
	}
}

/*
 * Dump all AC and rate control parameters related to IQUE.
 */
static void
ath_getiqueconfig(ath_dev_t dev)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);

    printk("\n========= Rate Control Table Config========\n");
    printk("AC\tPER\tProbe Interval\n\n");
    printk("BE&BK\t%d\t\t%d \n",
           sc->sc_rc_params[0].per_threshold,
           sc->sc_rc_params[0].probe_interval);
    printk("VI&VO\t%d\t\t%d \n",
           sc->sc_rc_params[1].per_threshold,
           sc->sc_rc_params[1].probe_interval);

    printk("\n========= Access Category Config===========\n");
    printk("AC\tRTS \tAggr Scaling\tMin Rate(Kbps)\tHBR \tPER LOW THRESHOLD\n\n");
    printk("BE\t%s\t\t%d\t%6d\t\t%s\t%d\n",
           sc->sc_ac_params[0].use_rts?"Yes":"No",
           sc->sc_ac_params[0].aggrsize_scaling,
           sc->sc_ac_params[0].min_kbps,
		   sc->sc_hbr_params[0].hbr_enable?"Yes":"No",
		   sc->sc_hbr_params[0].hbr_per_low);
    printk("BK\t%s\t\t%d\t%6d\t\t%s\t%d\n",
           sc->sc_ac_params[1].use_rts?"Yes":"No",
           sc->sc_ac_params[1].aggrsize_scaling,
           sc->sc_ac_params[1].min_kbps,
		   sc->sc_hbr_params[1].hbr_enable?"Yes":"No",
		   sc->sc_hbr_params[1].hbr_per_low);
    printk("VI\t%s\t\t%d\t%6d\t\t%s\t%d\n",
           sc->sc_ac_params[2].use_rts?"Yes":"No",
           sc->sc_ac_params[2].aggrsize_scaling,
           sc->sc_ac_params[2].min_kbps,
		   sc->sc_hbr_params[2].hbr_enable?"Yes":"No",
		   sc->sc_hbr_params[2].hbr_per_low);
    printk("VO\t%s\t\t%d\t%6d\t\t%s\t%d\n",
           sc->sc_ac_params[3].use_rts?"Yes":"No",
           sc->sc_ac_params[3].aggrsize_scaling,
           sc->sc_ac_params[3].min_kbps,
		   sc->sc_hbr_params[3].hbr_enable?"Yes":"No",
		   sc->sc_hbr_params[3].hbr_per_low);
}
#endif

/*
 * Callback table for Atheros ARXXXX MACs
 */
struct ath_ops ath_ar_ops = {
    ath_get_caps,               /* has_capability */
    ath_get_ciphers,            /* has_cipher */
    ath_open,                   /* open */
    ath_suspend,                /* stop */
    ath_vap_attach,             /* add_interface */
    ath_vap_detach,             /* remove_interface */
    ath_vap_config,             /* config_interface */
    ath_vap_down,               /* down */
    ath_vap_listen,             /* listen */
    ath_vap_join,               /* join */
    ath_vap_up,                 /* up */
    ath_node_attach,            /* alloc_node */
    ath_node_detach,            /* free_node */
    ath_node_cleanup,           /* cleanup_node */
    ath_node_update_pwrsave,	/* update_node_pwrsave */
    ath_newassoc,               /* new_assoc */
    ath_enable_intr,            /* enable_interrupt */
    ath_disable_intr,           /* disable_interrupt */
    ath_intr,                   /* isr */
    ath_handle_intr,            /* handle_intr */
    ath_reset_start,            /* reset_start */
    ath_reset,                  /* reset */
    ath_reset_end,              /* reset_end */
    ath_switch_opmode,          /* switch_opmode */
    ath_set_channel,            /* set_channel */
    ath_scan_start,             /* scan_start */
    ath_scan_end,               /* scan_end */
    ath_enter_led_scan_state,   /* led_scan_start */
    ath_leave_led_scan_state,   /* led_scan_end */
    ath_notify_force_ppm,       /* force_ppm_notify */
    ath_beacon_sync,            /* sync_beacon */
    ath_update_beacon_info,     /* update_beacon_info */
    ath_tx_init,                /* tx_init */
    ath_tx_cleanup,             /* tx_cleanup */
    ath_tx_get_qnum,            /* tx_get_qnum */
    ath_txq_update,             /* txq_update */
    ath_tx_start,               /* tx */
    ath_tx_tasklet,             /* tx_proc */
    ath_tx_flush,               /* tx_flush */
    ath_txq_depth,              /* txq_depth */
    ath_txq_aggr_depth,         /* txq_aggr_depth */
    ath_txq_aggr_nbuf,          /* txq_aggr_nbuf */
    ath_txq_lim,                /* txq_lim */
    ath_rx_init,                /* rx_init */
    ath_rx_cleanup,             /* rx_cleanup */
    ath_rx_tasklet,             /* rx_proc */
    ath_rx_requeue,             /* rx_requeue */
    ath_rx_input,               /* rx_proc_frame */
    ath_aggr_check,             /* check_aggr */
    ath_set_ampduparams,        /* set_ampdu_params */
    ath_set_weptkip_rxdelim,    /* set_weptkip_rxdelim */
    ath_addba_requestsetup,     /* addba_request_setup */
    ath_addba_responsesetup,    /* addba_response_setup */
    ath_addba_requestprocess,   /* addba_request_process */
    ath_addba_responseprocess,  /* addba_response_process */
    ath_addba_clear,            /* addba_clear */
    ath_delba_process,          /* delba_process */
    ath_addba_status,           /* addba_status */
    ath_aggr_teardown,          /* aggr_teardown */
    ath_set_addbaresponse,      /* set_addbaresponse */
    ath_clear_addbaresponsestatus, /* clear_addbaresponsestatus */
    ath_updateslot,             /* set_slottime */
    ath_set_protmode,           /* set_protmode */
    ath_set_txpowlimit,         /* set_txpowlimit */
    ath_get_mac_address,        /* get_macaddr */
    ath_set_mac_address,        /* set_macaddr */
    ath_set_mcastlist,          /* set_mcastlist */
    ath_opmode_init,            /* mc_upload */
    ath_key_alloc_2pair,        /* key_alloc_2pair */
    ath_key_alloc_pair,         /* key_alloc_pair */
    ath_key_alloc_single,       /* key_alloc_single */
    ath_key_reset,              /* key_delete */
    ath_keyset,                 /* key_set */
    ath_keycache_size,          /* keycache_size */
    ath_get_sw_phystate,        /* get_sw_phystate */
    ath_get_hw_phystate,        /* get_hw_phystate */
    ath_set_sw_phystate,        /* set_sw_phystate */
    ath_radio_enable,           /* radio_enable */
    ath_radio_disable,          /* radio_disable */
    ath_led_suspend,            /* led_suspend */
    ath_pwrsave_awake,          /* awake */
    ath_pwrsave_netsleep,       /* netsleep */
    ath_pwrsave_fullsleep,      /* fullsleep */
    ath_set_country,            /* set_country */
    ath_set_divant,             /* set_diversity_antenna */
    ath_get_currentCountry,     /* get_current_country */
    ath_set_regdomain,          /* set_regdomain */
    ath_get_regdomain,          /* get_regdomain */
    ath_set_quiet,              /* set_quiet */
    ath_find_countrycode,       /* find_countrycode */
    ath_set_tx_antenna_select,  /* set_antenna_select */
    ath_get_current_tx_antenna, /* get_current_tx_antenna */
    ath_get_default_antenna,    /* get_default_antenna */
    ath_notify_device_removal,  /* notify_device_removal */
    ath_detect_card_present,    /* detect_card_present */
    ath_mhz2ieee,               /* mhz2ieee */
    ath_get_phy_stats,          /* get_phy_stats */
    ath_get_ath_stats,          /* get_ath_stats */
    ath_get_11n_stats,          /* get_11n_stats */
    ath_clear_stats,            /* clear_stats */
    ath_set_macmode,            /* set_macmode */
    ath_set_extprotspacing,     /* set_extprotspacing */
    ath_get_extbusyper,         /* get_extbusyper */
#ifdef ATH_SUPERG_FF
    ath_ff_check,               /* check_ff */
#endif
#ifdef ATH_SUPPORT_DFS
    ath_dfs_control,            /* ath_dfs_control */
#endif
#ifdef ATH_WOW    
    ath_get_wow_support,        /* ath_get_wow_support */
    ath_set_wow_enable,         /* ath_set_wow_enable */
    ath_wow_wakeup,             /* ath_wow_wakeup */
    ath_set_wow_events,         /* ath_set_wow_events */
    ath_get_wow_events,         /* ath_get_wow_events */
    ath_wow_add_wakeup_pattern, /* ath_wow_add_wakeup_pattern */
    ath_wow_remove_wakeup_pattern, /* ath_wow_remove_wakeup_pattern */
    ath_get_wow_wakeup_reason,  /* ath_get_wow_wakeup_reason */
    ath_wow_matchpattern_exact, /* ath_wow_matchpattern_exact */
    ath_wow_set_duration,       /* ath_wow_set_duration */
#endif

    ath_get_config,             /* ath_get_config_param */
    ath_set_config,             /* ath_set_config_param */
    ath_get_noisefloor,			/* get_noisefloor */
    ath_update_sm_pwrsave,      /* ath_sm_pwrsave_update */
    athop_rate_newassoc,        /* ath_rate_newassoc */
    athop_rate_newstate,        /* ath_rate_newstate */
#ifdef DBG
    ath_register_read,          /* Read register value  */
    ath_register_write,         /* Write register value */
#endif
    ath_setTxPwrLimit,          /* ath_set_txPwrLimit */
    ath_get_common_power,       /* get_common_power */
#ifdef ATH_CCX
    ath_update_mib_macstats,    /* ath_update_mibMacstats */
    ath_get_mib_macstats,       /* ath_get_mibMacstats */
    ath_rcRateValueToPer,       /* rcRateValueToPer */
    ath_get_mib_cyclecounts,    /* ath_get_mibCycleCounts */
    ath_clear_mib_counters,     /* ath_clear_mibCounters */
    ath_gettsf32,               /* ath_get_tsf32 */
    ath_gettsf64,               /* ath_get_tsf64 */
    ath_setrxfilter,            /* ath_set_rxfilter */
    ath_getserialnumber,        /* ath_get_sernum */
    ath_getchandata,            /* ath_get_chandata */
    ath_getcurrssi,             /* ath_get_curRSSI */
#endif
    ath_get_amsdusupported,     /* get_amsdusupported */

#ifdef ATH_SWRETRY
    ath_set_swretrystate,       /* set_swretrystate */
#endif
#ifdef ATH_SUPPORT_UAPSD
    ath_process_uapsd_trigger,  /* process_uapsd_trigger */
    ath_tx_uapsd_depth,         /* uapsd_depth */
#endif
#ifndef REMOVE_PKT_LOG
    ath_start_pktlog,           /* pktlog_start */
    ath_read_pktlog_hdr,        /* pktlog_read_hdr */
    ath_read_pktlog_buf,        /* pktlog_read_buf */
#endif
#ifdef ATH_SUPPORT_IQUE
    ath_setacparams,            /* ath_set_acparams */
    ath_setrtparams,            /* ath_set_rtparams */
    ath_getiqueconfig,          /* ath_get_iqueconfig */
	ath_sethbrparams,			/* ath_set_hbrparams */
#endif
    ath_printreg,
    ath_getmfpsupport,          /* ath_get_mfpsupport */
#ifdef ATH_SUPPORT_DFS
    ath_check_dfs_wait,		/* check_dfs_wait */
#endif
};

#ifdef debug_wds_endurance
void
inact_get_noise_floor(struct ath_hal *ah, int16_t *nfarray)
{
    int16_t nf;

#define AR_PHY_CCA              0x9864
#define AR_PHY_MINCCA_PWR       0x0FF80000
#define AR_PHY_MINCCA_PWR_S     19

#define AR_PHY_CH1_CCA          0xa864
#define AR_PHY_CH1_MINCCA_PWR   0x0FF80000
#define AR_PHY_CH1_MINCCA_PWR_S 19

#define AR_PHY_CH2_CCA          0xb864
#define AR_PHY_CH2_MINCCA_PWR   0x0FF80000
#define AR_PHY_CH2_MINCCA_PWR_S 19

#define AR_PHY_EXT_CCA          0x99bc
#define AR_PHY_EXT_MINCCA_PWR   0xFF800000
#define AR_PHY_EXT_MINCCA_PWR_S 23

#define AR_PHY_CH1_EXT_CCA          0xa9bc
#define AR_PHY_CH1_EXT_MINCCA_PWR   0xFF800000
#define AR_PHY_CH1_EXT_MINCCA_PWR_S 23

#define AR_PHY_CH2_EXT_CCA          0xb9bc
#define AR_PHY_CH2_EXT_MINCCA_PWR   0xFF800000
#define AR_PHY_CH2_EXT_MINCCA_PWR_S 23

#define MS(_v, _f)  (((_v) & _f) >> _f##_S)

    nf = MS(OS_REG_READ(ah, AR_PHY_CCA), AR_PHY_MINCCA_PWR);
    if (nf & 0x100)
        nf = 0 - ((nf ^ 0x1ff) + 1);
    nfarray[0] = nf;

    nf = MS(OS_REG_READ(ah, AR_PHY_CH1_CCA), AR_PHY_CH1_MINCCA_PWR);
    if (nf & 0x100)
        nf = 0 - ((nf ^ 0x1ff) + 1);
    nfarray[1] = nf;

    nf = MS(OS_REG_READ(ah, AR_PHY_CH2_CCA), AR_PHY_CH2_MINCCA_PWR);
    if (nf & 0x100)
        nf = 0 - ((nf ^ 0x1ff) + 1);
    nfarray[2] = nf;

    nf = MS(OS_REG_READ(ah, AR_PHY_EXT_CCA), AR_PHY_EXT_MINCCA_PWR);
    if (nf & 0x100)
        nf = 0 - ((nf ^ 0x1ff) + 1);
    nfarray[3] = nf;

    nf = MS(OS_REG_READ(ah, AR_PHY_CH1_EXT_CCA), AR_PHY_CH1_EXT_MINCCA_PWR);
    if (nf & 0x100)
        nf = 0 - ((nf ^ 0x1ff) + 1);
    nfarray[4] = nf;

    nf = MS(OS_REG_READ(ah, AR_PHY_CH2_EXT_CCA), AR_PHY_CH2_EXT_MINCCA_PWR);
    if (nf & 0x100)
        nf = 0 - ((nf ^ 0x1ff) + 1);
    nfarray[5] = nf;
}

u_int32_t
bbstate(struct ath_softc *sc, u_int32_t n)
{
    u_int32_t r = OS_REG_READ(sc->sc_ah, 0x9808);
    r = (r & (~(0xf << 10))) | (n << 10);
    OS_REG_WRITE(sc->sc_ah, 0x9808, r);
    r = OS_REG_READ(sc->sc_ah, 0x9c24);
    return r;
}



void
print_state(struct ath_softc *sc)
{
    u_int32_t *p, d;
    int16_t nf[6];

    printk("--%d,%p,%lu:0x%x 0x%x 0x%p 0x%x 0x%x 0x%x 0x%x\n",
        sc->sc_tx_inact, sc->sc_txq[1].axq_link, jiffies,
        OS_REG_READ(sc->sc_ah, 0x806c), // obs
        OS_REG_READ(sc->sc_ah, 0x0840), // AR_Q_TXE
        (void *)(d = OS_REG_READ(sc->sc_ah, 0x0804)), // AR_Q1_TXDP
        OS_REG_READ(sc->sc_ah, 0x0008), // AR_CR
        OS_REG_READ(sc->sc_ah, 0x000c), // AR_RXDP
        OS_REG_READ(sc->sc_ah, 0x00e0), // AR_DMADBG_0
        OS_REG_READ(sc->sc_ah, 0x00f0)); // AR_DMADBG_4

    printk("bb state: 0x%08x 0x%08x\n", bbstate(sc, 4ul), bbstate(sc, 5ul));

    ath_hal_dmaRegDump(sc->sc_ah);

    p = (void *)KSEG1ADDR(d);
    printk("%08x %08x %08x %08x %08x %08x %08x %08x\n%08x %08x %08x %08x\n",
        p[ 0], p[ 1], p[ 2], p[ 3], p[ 4], p[ 5], p[ 6], p[ 7],
        p[ 8], p[ 9], p[10], p[11]);

    inact_get_noise_floor(sc->sc_ah, nf);

    printk("noise floor: (%d, %d) (%d, %d) (%d, %d)\n",
        nf[ 0], nf[ 3], nf[ 1], nf[ 4], nf[ 2], nf[ 5]);

}

void
ath_dump_buf_data(struct ath_buf *bf)
{
    unsigned int *p = (unsigned int *)wbuf_header(bf->bf_mpdu);
    printk("%p: %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x\n",
        p, p[ 0], p[ 1], p[ 2], p[ 3], p[ 4], p[ 5], p[ 6], p[ 7],
        p[ 8], p[ 9], p[10], p[11]);
}

void
print_state2(struct ath_softc *sc)
{
    u_int32_t *p, d;

    printk("--%d,%p,%lu:0x%x 0x%x 0x%p 0x%x 0x%x 0x%x 0x%x\n",
        sc->sc_tx_inact, sc->sc_txq[1].axq_link, jiffies,
        OS_REG_READ(sc->sc_ah, 0x806c), // obs
        OS_REG_READ(sc->sc_ah, 0x0840), // AR_Q_TXE
        (void *)(d = OS_REG_READ(sc->sc_ah, 0x0804)), // AR_Q1_TXDP
        OS_REG_READ(sc->sc_ah, 0x0008), // AR_CR
        OS_REG_READ(sc->sc_ah, 0x000c), // AR_RXDP
        OS_REG_READ(sc->sc_ah, 0x00e0), // AR_DMADBG_0
        OS_REG_READ(sc->sc_ah, 0x00f0)); // AR_DMADBG_4

    p = (void *)KSEG1ADDR(d);
    printk("%08x %08x %08x %08x %08x %08x %08x %08x\n%08x %08x %08x %08x\n",
        p[ 0], p[ 1], p[ 2], p[ 3], p[ 4], p[ 5], p[ 6], p[ 7],
        p[ 8], p[ 9], p[10], p[11]);

}
#endif

#ifdef AH_WAR_52640
static
OS_TIMER_FUNC(ath_inact)
{
    struct ath_softc    *sc;

    OS_GET_TIMER_ARG(sc, struct ath_softc *);

    if (!sc->sc_nvaps || sc->sc_invalid ) {
        OS_SET_TIMER(&sc->sc_inact, 2000 /* ms */);
        return;
    }
#ifdef ATH_SUPPORT_DFS
    if (sc->sc_dfs && sc->sc_dfs->sc_dfswait) {
        OS_SET_TIMER(&sc->sc_inact, 60000 /* ms */);
        return;
    }
#endif
    if ((sc->sc_tx_inact & 0xf) != 0xf) {
        /*
         * No Tx Complete interrupt for about 2 seconds for one of the queues. 
         * Check for TXE and if it is not set (for data/mgmt  queues ),
         * process that queue. Refer to ev#52640
         */ 
        sc->sc_qcumask = ~OS_REG_READ(sc->sc_ah, 0x0840 /* AR_Q_TXE */);
        sc->sc_qcumask &= ~sc->sc_tx_inact & 0xf; 
        sc->sc_stats.ast_donebit_hang ++;
        ath_tx_tasklet(sc);
    }
    sc->sc_tx_inact = 0;
    OS_SET_TIMER(&sc->sc_inact, 2000 /* ms */);
}
#endif

int ath_dev_attach(u_int16_t            devid,
                   void                 *bus_context,
                   ieee80211_handle_t   ieee,
                   struct ieee80211_ops *ieee_ops,
                   osdev_t              osdev,
                   ath_dev_t            *dev,
                   struct ath_ops       **ops,
                   struct ath_reg_parm  *ath_conf_parm,
                   struct hal_reg_parm  *hal_conf_parm)
{
#define	N(a)	(sizeof(a)/sizeof(a[0]))
    struct ath_softc *sc = NULL;
    struct ath_hal *ah = NULL;
    HAL_STATUS status;
    int error = 0, i;
    int csz = 0;
    u_int32_t chainmask;
    u_int32_t rd;
    u_int32_t stbcsupport;

    sc = (struct ath_softc *)OS_MALLOC(osdev, sizeof(struct ath_softc), GFP_KERNEL);
    if (sc == NULL) {
        printk("%s: unable to allocate device object.\n", __func__);
        error = -ENOMEM;
        goto bad;
    }
    OS_MEMZERO(sc, sizeof(struct ath_softc));
    
    *dev = (ath_dev_t)sc;
    *ops = &ath_ar_ops;

    sc->sc_osdev = osdev;
    sc->sc_ieee = ieee;
    sc->sc_ieee_ops = ieee_ops;
    sc->sc_reg_parm = *ath_conf_parm;
    
    /* XXX: hardware will not be ready until ath_open() being called */
    sc->sc_invalid = 1;

    sc->sc_debug = DBG_DEFAULT;

    sc->sc_limit_legacy_frames = 25;
    DPRINTF(sc, ATH_DEBUG_ANY, "%s: devid 0x%x\n", __func__, devid);

    /*
     * Cache line size is used to size and align various
     * structures used to communicate with the hardware.
     */
    bus_read_cachesize(sc->sc_osdev, &csz);
    /* XXX assert csz is non-zero */
    sc->sc_cachelsz = csz << 2;		/* convert to bytes */

    //ATH_LOCK_INIT(sc);
    ATH_PS_LOCK_INIT(sc);
    ATH_RESET_LOCK_INIT(sc);

#if defined(ATH_SWRETRY) || defined(AP_SLOW_ANT_DIV)
    ATH_NODETABLE_LOCK_INIT(sc);
    LIST_INIT(&sc->sc_nt);
#endif

    /*
     * Attach the hal and verify ABI compatibility by checking
     * the hal's ABI signature against the one the driver was
     * compiled with.  A mismatch indicates the driver was
     * built with an ah.h that does not correspond to the hal
     * module loaded in the kernel.
     */
    ah = _ath_hal_attach(devid, osdev, sc, bus_context, hal_conf_parm, 
                        &halCallbackTable, &status);
    if (ah == NULL) {
        printk("%s: unable to attach hardware; HAL status %u\n",
               __func__, status);
        error = -ENXIO;
        goto bad;
    }
    if (ah->ah_abi != HAL_ABI_VERSION) {
        printk("%s: HAL ABI msmatch; "
               "driver expects 0x%x, HAL reports 0x%x\n",
               __func__, HAL_ABI_VERSION, ah->ah_abi);
        error = -ENXIO;		/* XXX */
        goto bad;
    }
    sc->sc_ah = ah;

    /*
     * Get the chipset-specific aggr limit.
     */
    ath_hal_getrtsaggrlimit(sc->sc_ah, &sc->sc_rtsaggrlimit);

    /*
     * Check if the MAC has multi-rate retry support.
     * We do this by trying to setup a fake extended
     * descriptor.  MAC's that don't have support will
     * return false w/o doing anything.  MAC's that do
     * support it will return true w/o doing anything.
     * 
     *  XXX This is lame.  Just query a hal property, Luke!
     */
    sc->sc_mrretry = ath_hal_setupxtxdesc(ah, NULL, 0,0, 0,0, 0,0);

    /*
     * Check if the device has hardware counters for PHY
     * errors.  If so we need to enable the MIB interrupt
     * so we can act on stat triggers.
     */
    if (ath_hal_hwphycounters(ah)) {
        sc->sc_needmib = 1;
    }

    /*
     * Get the hardware key cache size.
     */
    sc->sc_keymax = ath_hal_keycachesize(ah);
    if (sc->sc_keymax > ATH_KEYMAX) {
        printk("%s: Warning, using only %u entries in %u key cache\n",
               __func__, ATH_KEYMAX, sc->sc_keymax);
        sc->sc_keymax = ATH_KEYMAX;
    }

    // single key cache writes
    // Someday Soon we should probe the chipset id to see if it is
    // one of the broken ones, then turn on the singleWriteKC, which 
    // should be just a hal property, IMNSHO.
    // ath_hal_set_singleWriteKC(ah, TRUE);

    /*
     * Reset the key cache since some parts do not
     * reset the contents on initial power up.
     */
    for (i = 0; i < sc->sc_keymax; i++) {
        ath_hal_keyreset(ah, (u_int16_t)i);
    }
    /*
     * Mark key cache slots associated with global keys
     * as in use.  If we knew TKIP was not to be used we
     * could leave the +32, +64, and +32+64 slots free.
     * XXX only for splitmic.
     */
    for (i = 0; i < IEEE80211_WEP_NKID; i++) {
        setbit(sc->sc_keymap, i);
        setbit(sc->sc_keymap, i+32);
        setbit(sc->sc_keymap, i+64);
        setbit(sc->sc_keymap, i+32+64);
    }

    /*
     * Initialize timer module
     */
    ath_initialize_timer_module(osdev);

    /*
     * Collect the channel list using the default country
     * code and including outdoor channels.  The 802.11 layer
     * is resposible for filtering this list based on settings
     * like the phy mode.
     */
    ath_hal_getregdomain(ah, &rd);

    if (sc->sc_reg_parm.regdmn) {
        ath_hal_setregdomain(ah, sc->sc_reg_parm.regdmn, &status);
    }

    sc->sc_config.ath_countrycode = CTRY_DEFAULT;
    if (countrycode != -1) {
        sc->sc_config.ath_countrycode = countrycode;
    }

    /*
     * Only allow set country code for debug domain.
     */
    if ((rd == 0) && sc->sc_reg_parm.countryName[0]) {
        sc->sc_config.ath_countrycode = findCountryCode((u_int8_t *)sc->sc_reg_parm.countryName);
    }

    sc->sc_config.ath_xchanmode = AH_TRUE;
    if (xchanmode != -1) {
        sc->sc_config.ath_xchanmode = xchanmode;
    } else {
        sc->sc_config.ath_xchanmode = sc->sc_reg_parm.extendedChanMode;
    }
    error = ath_getchannels(sc, sc->sc_config.ath_countrycode,
                            ath_outdoor, sc->sc_config.ath_xchanmode);
    if (error != 0) {
        goto bad;
    }

    /* default to STA mode */
    sc->sc_opmode = HAL_M_STA;

    /*
     * Initialize rate tables.
     */
    ath_rate_table_init();

    /*
     * Setup rate tables for all potential media types.
     */
#ifndef ATH_NO_5G_SUPPORT
    ath_rate_setup(sc, WIRELESS_MODE_11a);
#endif
    ath_rate_setup(sc, WIRELESS_MODE_11b);
    ath_rate_setup(sc, WIRELESS_MODE_11g);
#ifndef ATH_NO_5G_SUPPORT
    ath_rate_setup(sc, WIRELESS_MODE_108a);
#endif
    ath_rate_setup(sc, WIRELESS_MODE_108g);
    ath_rate_setup(sc, WIRELESS_MODE_11NA_HT20);
    ath_rate_setup(sc, WIRELESS_MODE_11NG_HT20);
#ifndef ATH_NO_5G_SUPPORT
    ath_rate_setup(sc, WIRELESS_MODE_11NA_HT40PLUS);
#endif
    ath_rate_setup(sc, WIRELESS_MODE_11NA_HT40MINUS);
    ath_rate_setup(sc, WIRELESS_MODE_11NG_HT40PLUS);
    ath_rate_setup(sc, WIRELESS_MODE_11NG_HT40MINUS);

    /* Setup for half/quarter rates */
    ath_setup_subrates(sc);

    /* NB: setup here so ath_rate_update is happy */
#ifndef ATH_NO_5G_SUPPORT
    ath_setcurmode(sc, WIRELESS_MODE_11a);
#else
    ath_setcurmode(sc, WIRELESS_MODE_11g);
#endif
    /*
     * Allocate hardware transmit queues: one queue for
     * beacon frames and one data queue for each QoS
     * priority.  Note that the hal handles reseting
     * these queues at the needed time.
     *
     * XXX PS-Poll
     */
    sc->sc_bhalq = ath_beaconq_setup(ah);
    if (sc->sc_bhalq == (u_int) -1) {
        printk("unable to setup a beacon xmit queue!\n");
        error = -EIO;
        goto bad2;
    }
    sc->sc_cabq = ath_txq_setup(sc, HAL_TX_QUEUE_CAB, 0);
    if (sc->sc_cabq == NULL) {
        printk("unable to setup CAB xmit queue!\n");
        error = -EIO;
        goto bad2;
    }

    sc->sc_config.cabqReadytime = ATH_CABQ_READY_TIME;

    ath_cabq_update(sc);

    for (i = 0; i < N(sc->sc_haltype2q); i++)
        sc->sc_haltype2q[i] = -1;

    /* NB: insure BK queue is the lowest priority h/w queue */
    if (!ath_tx_setup(sc, HAL_WME_AC_BK)) {
        printk("unable to setup xmit queue for BE traffic!\n");
        error = -EIO;
        goto bad2;
    }

    if (!ath_tx_setup(sc, HAL_WME_AC_BE) ||
        !ath_tx_setup(sc, HAL_WME_AC_VI) ||
        !ath_tx_setup(sc, HAL_WME_AC_VO)) {
        /*
         * Not enough hardware tx queues to properly do WME;
         * just punt and assign them all to the same h/w queue.
         * We could do a better job of this if, for example,ath_hal_getmcastkeysearch(ah)
         * we allocate queues when we switch from station to
         * AP mode.
         */
        if (sc->sc_haltype2q[HAL_WME_AC_VI] != -1)
            ath_tx_cleanupq(sc, &sc->sc_txq[sc->sc_haltype2q[HAL_WME_AC_VI]]);
        if (sc->sc_haltype2q[HAL_WME_AC_BE] != -1)
            ath_tx_cleanupq(sc, &sc->sc_txq[sc->sc_haltype2q[HAL_WME_AC_BE]]);
        sc->sc_haltype2q[HAL_WME_AC_BE] = sc->sc_haltype2q[HAL_WME_AC_BK];
        sc->sc_haltype2q[HAL_WME_AC_VI] = sc->sc_haltype2q[HAL_WME_AC_BK];
        sc->sc_haltype2q[HAL_WME_AC_VO] = sc->sc_haltype2q[HAL_WME_AC_BK];
    }
    else {
		/*
		 * Mark WME capability since we have sufficient
		 * hardware queues to do proper priority scheduling.
		 */
        sc->sc_haswme = 1;
#ifdef ATH_SUPPORT_UAPSD
        sc->sc_uapsdq = ath_txq_setup(sc, HAL_TX_QUEUE_UAPSD, 0);
        if (sc->sc_uapsdq == NULL) {
            DPRINTF(sc, ATH_DEBUG_UAPSD, "%s: unable to setup UAPSD xmit queue!\n",
                    __func__);
        }
        else {
            /*
             * default UAPSD on if HW capable
             */
            sc->sc_uapsdsupported = 1;
        }
#endif
    }

#ifdef ATH_SUPERG_COMP
    if (ath_hal_compressionsupported(ah))
        sc->sc_hascompression = 1;
#endif

#ifdef ATH_SUPERG_XR
    ath_xr_rate_setup(sc);
    sc->sc_xrpollint =  XR_DEFAULT_POLL_INTERVAL;
    sc->sc_xrpollcount = XR_DEFAULT_POLL_COUNT;
    strcpy(sc->sc_grppoll_str,XR_DEFAULT_GRPPOLL_RATE_STR);
    sc->sc_grpplq.axq_qnum=-1;
    sc->sc_xrtxq = ath_txq_setup(sc, HAL_TX_QUEUE_DATA,HAL_XR_DATA);
#endif

#ifdef ATH_WOW
    /*
     * If hardware supports wow, allocate memory for wow information.
     */
    sc->sc_hasWow = ath_hal_hasWow(ah);
    if(sc->sc_hasWow)
    {
        sc->sc_wowInfo = (struct wow_info *)OS_MALLOC(sc->sc_osdev, sizeof(struct wow_info), GFP_KERNEL);
        if (sc->sc_wowInfo != NULL) {
            OS_MEMZERO(sc->sc_wowInfo, sizeof(struct wow_info));
        } else {
            goto bad;
        }
    }
    sc->sc_wow_sleep = 0;
#endif

    if (ath_hal_htsupported(ah))
        sc->sc_hashtsupport=1;

    if(sc->sc_reg_parm.stbcEnable) {
        if (ath_hal_txstbcsupport(ah, &stbcsupport))
            sc->sc_txstbcsupport = stbcsupport;

        if (ath_hal_rxstbcsupport(ah, &stbcsupport))
            sc->sc_rxstbcsupport = stbcsupport;
    }

    sc->sc_setdefantenna = ath_setdefantenna;
    sc->sc_rc = ath_rate_attach(ah);
    if (sc->sc_rc == NULL) {
        error = EIO;
        goto bad2;
    }

    OS_INIT_TIMER(sc->sc_osdev, &sc->sc_cal_ch, ath_calibrate, sc);

#ifdef AH_WAR_52640
    OS_INIT_TIMER(sc->sc_osdev, &sc->sc_inact, ath_inact, sc);
    sc->sc_tx_inact = 0;
    OS_SET_TIMER(&sc->sc_inact, 60000 /* ms */);
#endif

#ifdef ATH_SUPPORT_DFS
    sc->sc_dfs = NULL;
    if (dfs_attach(sc)) {
        error = EIO;
        printk("%s DFS attach failed\n", __func__);
        goto bad2;
    }
    /*Initialize DFS related timers*/
    OS_INIT_TIMER(sc->sc_osdev, &sc->sc_dfs->sc_dfswaittimer, ath_check_dfs_clear, sc);
    OS_INIT_TIMER(sc->sc_osdev, &sc->sc_dfs->sc_dfstesttimer, ath_dfs_test_return, sc);
    OS_INIT_TIMER(sc->sc_osdev, &sc->sc_dfs->sc_dfs_task_timer, ath_radar_task, sc);
    /* Initialize the Sowl TX hang WAR state */
    OS_INIT_TIMER(sc->sc_osdev, &sc->sc_dfs_hang.hang_war_timer, ath_dfs_hang_war, sc);
    sc->sc_dfs_hang.hang_war_timeout = (ATH_DFS_HANG_WAR_MIN_TIMEOUT_MS/2);
    sc->sc_dfs_hang.total_stuck=0;
    sc->sc_dfs_hang.last_dfs_hang_war_tstamp = 0;
    sc->sc_dfs_hang.hang_war_ht40count = 0;
    sc->sc_dfs_hang.hang_war_ht20count = 0;
    sc->sc_dfs_hang.hang_war_activated = 0;

#endif

#ifdef ATH_SUPERG_DYNTURBO
#ifdef notyet
    OS_INIT_TIMER(sc->sc_osdev, &sc->sc_dturbo_switch_mode, ath_turbo_switch_mode, sc);
#endif    
#endif


    // initialize LED module
    ath_initialize_led_control(sc, sc->sc_osdev, sc->sc_ah,
                               &sc->sc_led_control, &sc->sc_reg_parm);

    sc->sc_sw_phystate = sc->sc_hw_phystate = 1;    /* default hw/sw phy state to be on */

#ifdef ATH_SUPERG_FF
    sc->sc_fftxqmin = ATH_FF_TXQMIN;
#endif

    if (ath_hal_ciphersupported(ah, HAL_CIPHER_TKIP)) {
        /*
         * Whether we should enable h/w TKIP MIC.
         * XXX: if we don't support WME TKIP MIC, then we wouldn't report WMM capable,
         * so it's always safe to turn on TKIP MIC in this case.
         */
        ath_hal_setcapability(sc->sc_ah, HAL_CAP_TKIP_MIC, 0, 1, NULL);
    }
    sc->sc_hasclrkey = ath_hal_ciphersupported(ah, HAL_CIPHER_CLR);

    /* turn on mcast key search if possible */
    if (ath_hal_hasmcastkeysearch(ah)) {
         (void) ath_hal_setmcastkeysearch(ah, 1);
    }

    /*
     * TPC support can be done either with a global cap or
     * per-packet support.  The latter is not available on
     * all parts.  We're a bit pedantic here as all parts
     * support a global cap.
     */
    sc->sc_hastpc = ath_hal_hastpc(ah);

    sc->sc_config.txpowlimit = ATH_TXPOWER_MAX;
    sc->sc_config.txpowlimit_override = sc->sc_reg_parm.overRideTxPower;

    /* 11n Capabilities */
    if (sc->sc_hashtsupport) {
        sc->sc_config.ampdu_limit = sc->sc_reg_parm.aggrLimit;
        sc->sc_config.ampdu_subframes = sc->sc_reg_parm.aggrSubframes;

        sc->sc_txaggr = (sc->sc_reg_parm.txAggrEnable) ? 1 : 0;
        sc->sc_rxaggr = (sc->sc_reg_parm.rxAggrEnable) ? 1 : 0;

        sc->sc_txamsdu = (sc->sc_reg_parm.txAmsduEnable) ? 1 : 0;

        if (ath_hal_hasrifstx(ah)) {
            sc->sc_txrifs = (sc->sc_reg_parm.txRifsEnable) ? 1 : 0;
            sc->sc_config.rifs_ampdu_div =
                sc->sc_reg_parm.rifsAggrDiv;
        }
#ifdef ATH_RB
        if (!ath_hal_hasrifsrx(ah)) {
            sc->sc_rxrifs_timeout = sc->sc_reg_parm.rxRifsTimeout;
            sc->sc_rxrifs_skipthresh =
                                 sc->sc_reg_parm.rxRifsSkipThresh;
            sc->sc_rxrifs = sc->sc_reg_parm.rxRifsEnable;
            sc->sc_do_rb_war = 1;
            ath_rb_init(sc);
        }
#endif
    }

    /*
     * Check for misc other capabilities.
     */
    sc->sc_hasbmask = ath_hal_hasbssidmask(ah);
    sc->sc_hastsfadd = ath_hal_hastsfadjust(ah);

    if (sc->sc_reg_parm.txChainMask) {
        chainmask = sc->sc_reg_parm.txChainMask;
    } else {
        if (!ath_hal_gettxchainmask(ah, &chainmask))
            chainmask = 0;
    }

#ifdef ATH_CHAINMASK_SELECT
    /*
     * If we cannot transmit on three chains, prevent chain mask
     * selection logic from switching between 2x3 and 3x3 chain
     * masks based on RSSI.
     * Even when we can transmit on three chains, our default
     * behavior is to use two transmit chains only.
     */
    if (chainmask == ATH_CHAINMASK_SEL_3X3) {
        sc->sc_no_tx_3_chains = AH_TRUE;
        chainmask = ATH_CHAINMASK_SEL_2X3;
    } else {
        sc->sc_no_tx_3_chains = AH_FALSE;
    }
    sc->sc_config.chainmask_sel = sc->sc_no_tx_3_chains;
#endif

    sc->sc_config.txchainmask = chainmask;

    if (sc->sc_reg_parm.rxChainMask) {
        chainmask = sc->sc_reg_parm.rxChainMask;
    } else {
        if (!ath_hal_getrxchainmask(ah, &chainmask))
            chainmask = 0;
    }

    sc->sc_config.rxchainmask = chainmask;

    /* legacy chain mask */
    if (sc->sc_reg_parm.txChainMaskLegacy) {
        chainmask = sc->sc_reg_parm.txChainMaskLegacy;
    } else {
        if (!ath_hal_gettxchainmask(ah, &chainmask))
            chainmask = 0;
    }
    sc->sc_config.txchainmasklegacy = chainmask;

    if (sc->sc_reg_parm.rxChainMaskLegacy) {
        chainmask = sc->sc_reg_parm.rxChainMaskLegacy;
    } else {
        if (!ath_hal_getrxchainmask(ah, &chainmask))
            chainmask = 0;
    }
    sc->sc_config.rxchainmasklegacy = chainmask;

    /*
     * Configuration for rx chain detection
     */
    sc->sc_rxchaindetect_ref = sc->sc_reg_parm.rxChainDetectRef;
    sc->sc_rxchaindetect_thresh5GHz = sc->sc_reg_parm.rxChainDetectThreshA;
    sc->sc_rxchaindetect_thresh2GHz = sc->sc_reg_parm.rxChainDetectThreshG;
    sc->sc_rxchaindetect_delta5GHz = sc->sc_reg_parm.rxChainDetectDeltaA;
    sc->sc_rxchaindetect_delta2GHz = sc->sc_reg_parm.rxChainDetectDeltaG;

    /* Initialize ForcePPM module */
    ath_initialize_force_ppm_module(&sc->sc_ppm_info,
                                    sc,
                                    sc->sc_osdev,
                                    ah,
                                    &sc->sc_reg_parm);

    /*
     * Query the hal about antenna support
     * Enable rx fast diversity if hal has support
     */
    if (ath_hal_hasdiversity(ah)) {
        sc->sc_hasdiversity = 1;
        ath_hal_setdiversity(ah, AH_TRUE);
        sc->sc_diversity = 1;
    } else {
        sc->sc_hasdiversity = 0;
        sc->sc_diversity=0;
        ath_hal_setdiversity(ah, AH_FALSE);
    }
    sc->sc_defant = ath_hal_getdefantenna(ah);

    /*
     * Not all chips have the VEOL support we want to
     * use with IBSS beacons; check here for it.
     */
    sc->sc_hasveol = ath_hal_hasveol(ah);

    ath_hal_getmac(ah, sc->sc_myaddr);
    if (sc->sc_hasbmask) {
        u_int32_t isULBitsSupported; 
        ath_hal_getbssidmask(ah, sc->sc_bssidmask);
        
        ath_hal_hasMbssidAggrSupport(ah,&isULBitsSupported);
        if(isULBitsSupported & UL_BIT_MASK ){
            ATH_SET_VAP_BSSID_MASK_ULB(sc->sc_bssidmask);
        }   
        else { 
            ATH_SET_VAP_BSSID_MASK(sc->sc_bssidmask);
        }   
        ath_hal_setbssidmask(ah, sc->sc_bssidmask);
    }

#ifdef ATH_RFKILL
    if (ath_hal_hasrfkill(ah) && !sc->sc_reg_parm.disableRFKill) {
        sc->sc_hasrfkill = 1;
        ath_rfkill_attach(sc);
    }
#endif

    sc->sc_wpsgpiointr = 0;
    sc->sc_wpsbuttonpushed = 0;
    if (sc->sc_reg_parm.wpsButtonGpio && ath_hal_haswpsbutton(ah)) {
        /* Set corresponding line as input as older versions may have had it as output (bug 37281) */
        ath_hal_gpioCfgInput(ah, sc->sc_reg_parm.wpsButtonGpio);
        ath_hal_gpioSetIntr(ah, sc->sc_reg_parm.wpsButtonGpio, HAL_GPIO_INTR_LOW);
        sc->sc_wpsgpiointr = 1;
    }

    sc->sc_hasautosleep = ath_hal_hasautosleep(ah);
    sc->sc_waitbeacon   = 0;
    
#ifdef ATH_BT_COEX
    if (sc->sc_reg_parm.btCoexEnable) {
        ath_bt_coex_attach(sc, sc->sc_reg_parm.btCoexEnable);
    }
#endif

    sc->sc_slottime = HAL_SLOT_TIME_9; /* default to short slot time */
    
    /* initialize beacon slots */
    for (i = 0; i < N(sc->sc_bslot); i++)
        sc->sc_bslot[i] = ATH_IF_ID_ANY;
    
#ifndef REMOVE_PKT_LOG
    ath_pktlog_attach(sc);
#endif

    /* save MISC configurations */
#ifdef ATH_SWRETRY
    if (sc->sc_reg_parm.numSwRetries &&
         (!(ath_hal_htsupported(ah)) || 
         (ath_hal_htsupported(ah) && !(sc->sc_txaggr)))) {
        sc->sc_swRetryEnabled = AH_TRUE;
        sc->sc_num_swretries = sc->sc_reg_parm.numSwRetries;
    }
#endif
    sc->sc_config.pcieDisableAspmOnRfWake = sc->sc_reg_parm.pcieDisableAspmOnRfWake;
    sc->sc_config.pcieAspm = sc->sc_reg_parm.pcieAspm;
#ifdef ATH_WOW    
    sc->sc_wowenable = sc->sc_reg_parm.wowEnable;
#endif
    sc->sc_config.swBeaconProcess = sc->sc_reg_parm.swBeaconProcess;
    ath_hal_get_hang_types(ah, &sc->sc_hang_war);
    sc->sc_slowAntDiv = (sc->sc_reg_parm.slowAntDivEnable) ? 1 : 0;
    sc->sc_toggle_immunity = 0;

    /* Added a iwpriv to enable using the lowest data rate for EAP packets
       sc->sc_eap_lowest_rate reflects whether lowest rate should be used or not 
       By default turn OFF sending EAP packets at the lowest data rate */
    sc->sc_eap_lowest_rate = 0;

#ifdef SLOW_ANT_DIV
    if (sc->sc_slowAntDiv) {
        ath_slow_ant_div_init(&sc->sc_antdiv, sc, sc->sc_reg_parm.slowAntDivThreshold,
                              sc->sc_reg_parm.slowAntDivMinDwellTime,
                              sc->sc_reg_parm.slowAntDivSettleTime);
    }
#endif

sc->sc_antDivComb = ath_hal_AntDivCombSupport(ah);
    if (sc->sc_antDivComb) {
        ath_ant_div_comb_init(&sc->sc_antcomb, sc);
    }

#ifdef ATH_SUPPORT_IQUE
    /* Initialize Rate control and AC parameters with default values */
    sc->sc_rc_params[0].per_threshold = 55;
    sc->sc_rc_params[1].per_threshold = 35;
    sc->sc_rc_params[0].probe_interval = 50;
    sc->sc_rc_params[1].probe_interval = 50;

    sc->sc_hbr_params[WME_AC_VI].hbr_per_low = HBR_TRIGGER_PER_LOW;
    sc->sc_hbr_params[WME_AC_VO].hbr_per_low = HBR_TRIGGER_PER_LOW;
    sc->sc_hbr_per_low = HBR_TRIGGER_PER_LOW;
    sc->sc_hbr_per_high = HBR_TRIGGER_PER_HIGH;
    sc->sc_retry_duration = ATH_RC_DURATION_WITH_RETRIES;
#endif

#ifdef ATH_MIB_INTR_FILTER
    sc->sc_intr_filter.state            = INTR_FILTER_OFF;
    sc->sc_intr_filter.activation_count = 0;
    sc->sc_intr_filter.intr_count       = 0;
#endif
    return 0;

bad2:
    /* cleanup tx queues */
    for (i = 0; i < HAL_NUM_TX_QUEUES; i++)
        if (ATH_TXQ_SETUP(sc, i))
            ath_tx_cleanupq(sc, &sc->sc_txq[i]);
bad:
#ifdef ATH_RFKILL
    if (sc->sc_hasrfkill)
        ath_rfkill_detach(sc);
#endif    
#ifdef ATH_BT_COEX
    if (ah && ath_hal_hasbtcoex(ah)) {
        ath_bt_coex_detach(sc);
    }
#endif
    if (ah) {
        ath_hal_detach(ah);
    }
    //ATH_LOCK_DESTROY(sc);

    if (sc)
        OS_FREE(sc);
    
    /* null "out" parameters */
    *dev = NULL;
    *ops = NULL;

    return error;
#undef N
}

void
ath_dev_free(ath_dev_t dev)
{
    struct ath_softc *sc = (struct ath_softc *)dev;
    struct ath_hal *ah = sc->sc_ah;
    int i;

    DPRINTF(sc, ATH_DEBUG_ANY, "%s\n", __func__);

    ath_stop(sc);

    if (!sc->sc_invalid)
        ath_hal_setpower(sc->sc_ah, HAL_PM_AWAKE);

#ifdef AH_WAR_52640
    OS_CANCEL_TIMER(&sc->sc_inact);
#endif

#ifdef ATH_SUPPORT_DFS
    dfs_detach(sc);
     if (sc->sc_dfs_hang.hang_war_activated) {
        OS_CANCEL_TIMER(&sc->sc_dfs_hang.hang_war_timer);
        sc->sc_dfs_hang.hang_war_activated = 0;
     }
#endif

    ath_rate_detach(sc->sc_rc);

    /* cleanup tx queues */
    for (i = 0; i < HAL_NUM_TX_QUEUES; i++)
        if (ATH_TXQ_SETUP(sc, i))
            ath_tx_cleanupq(sc, &sc->sc_txq[i]);

#ifdef ATH_RFKILL
    if (sc->sc_hasrfkill)
        ath_rfkill_detach(sc);
#endif    

#ifdef ATH_BT_COEX
    if (ath_hal_hasbtcoex(ah)) {
        ath_bt_coex_detach(sc);
    }
#endif    
    ath_hal_detach(ah);

#ifndef REMOVE_PKT_LOG
    if (sc->pl_info)
        ath_pktlog_detach(sc);
#endif

    //ATH_LOCK_DESTROY(sc);
    ATH_RESET_LOCK_DESTROY(sc);
#ifdef ATH_SWRETRY
    ATH_NODETABLE_LOCK_DESTROY(sc);
#endif
    OS_FREE(sc);
}

static int
ath_vap_attach(ath_dev_t dev, int if_id, ieee80211_if_t if_data, HAL_OPMODE opmode, HAL_OPMODE iv_opmode, int nostabeacons)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_vap *avp;

    if (if_id >= ATH_BCBUF || sc->sc_vaps[if_id] != NULL) {
        printk("%s: Invalid interface id = %u\n", __func__, if_id);
        return -EINVAL;
    }

    switch (opmode) {
    case HAL_M_STA:
        sc->sc_nostabeacons = nostabeacons;
        break;
    case HAL_M_IBSS:
	case HAL_M_MONITOR:
        break;
    case HAL_M_HOSTAP:
        /* copy nostabeacons - for WDS client */
        sc->sc_nostabeacons = nostabeacons;
        /* XXX not right, beacon buffer is allocated on RUN trans */
        if (TAILQ_EMPTY(&sc->sc_bbuf)) {
            return -ENOMEM;
        }
        break;
    default:
        return -EINVAL;
    }

    /* create ath_vap */
    avp = (struct ath_vap *)OS_MALLOC(sc->sc_osdev, sizeof(struct ath_vap), GFP_KERNEL);
    if (avp == NULL)
        return -ENOMEM;

    OS_MEMZERO(avp, sizeof(struct ath_vap));
    avp->av_if_data = if_data;
    /* Set the VAP opmode */
    avp->av_opmode = iv_opmode;
    avp->av_atvp = ath_rate_create_vap(sc->sc_rc, avp);
    if (avp->av_atvp == NULL) {
        OS_FREE(avp);
        return -ENOMEM;
    }

    avp->av_bslot = -1;
    TAILQ_INIT(&avp->av_mcastq.axq_q);
    ATH_TXQ_LOCK_INIT(&avp->av_mcastq);
    if (opmode == HAL_M_HOSTAP || opmode == HAL_M_IBSS) {
        if (sc->sc_hastsfadd) {
            /*
             * Multiple vaps are to transmit beacons and we
             * have h/w support for TSF adjusting; enable use
             * of staggered beacons.
             */
            /* XXX check for beacon interval too small */
            sc->sc_stagbeacons = 1;
        }
    }
    if (sc->sc_hastsfadd)
        ath_hal_settsfadjust(sc->sc_ah, sc->sc_stagbeacons);

    sc->sc_vaps[if_id] = avp;
    sc->sc_nvaps++;
    sc->sc_bsthresh = (sc->sc_nvaps < 5) ? 36 :
                         (sc->sc_nvaps * (BSTUCK_THRESH / ATH_BCBUF));

    /* Set the device opmode */
    sc->sc_opmode = opmode;

#ifdef ATH_SUPERG_XR
    if (ieee80211vap_has_flag(vap, IEEE80211_F_XR)) {
        if (ath_descdma_setup(sc, &sc->sc_grppolldma, &sc->sc_grppollbuf,
                              "grppoll",(sc->sc_xrpollcount+1)*HAL_ANTENNA_MAX_MODE,1) != 0) {
            printk("%s:grppoll Buf allocation failed \n",__func__);
        }
        if(!sc->sc_xrtxq)
            sc->sc_xrtxq = ath_txq_setup(sc, HAL_TX_QUEUE_DATA,HAL_XR_DATA);
        if (sc->sc_hasdiversity) {
            /* Save current diversity state if user destroys XR vap */
            sc->sc_olddiversity = sc->sc_diversity;
            ath_hal_setdiversity(sc->sc_ah, 0);
            sc->sc_diversity=0;
        }
    }
#endif

    /* default VAP configuration */
    avp->av_config.av_fixed_rateset = IEEE80211_FIXED_RATE_NONE;
    avp->av_config.av_fixed_retryset = 0x03030303;
    return 0;
}

static int
ath_vap_detach(ath_dev_t dev, int if_id)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;
    struct ath_vap *avp;
    int flags;

    ATH_PS_WAKEUP(sc);

    avp = sc->sc_vaps[if_id];
    if (avp == NULL) {
        DPRINTF(sc, ATH_DEBUG_STATE, "%s: invalid interface id %u\n",
                __func__, if_id);
        return -EINVAL;
    }

    flags = sc->sc_ieee_ops->get_netif_settings(sc->sc_ieee);
    if (flags & ATH_NETIF_RUNNING) {
        /*
         * Quiesce the hardware while we remove the vap.  In
         * particular we need to reclaim all references to the
         * vap state by any frames pending on the tx queues.
         *
         * XXX can we do this w/o affecting other vap's?
         */
        ath_hal_intrset(ah, 0);     /* disable interrupts */
        ath_draintxq(sc, AH_FALSE); /* stop xmit side */
        ath_stoprecv(sc);           /* stop recv side */
        ath_flushrecv(sc);          /* flush recv queue */
    }
    
    /*
     * Reclaim any pending mcast bufs on the vap.
     */
    ath_tx_draintxq(sc, &avp->av_mcastq, AH_FALSE);
    ATH_TXQ_LOCK_DESTROY(&avp->av_mcastq);

    if (sc->sc_opmode == HAL_M_HOSTAP && sc->sc_nostabeacons)
        sc->sc_nostabeacons = 0;

    ath_rate_free_vap(avp->av_atvp);

    OS_FREE(avp);
    sc->sc_vaps[if_id] = NULL;
    sc->sc_nvaps--;
    sc->sc_bsthresh = (sc->sc_nvaps < 5) ? 36 :
                         (sc->sc_nvaps * (BSTUCK_THRESH / ATH_BCBUF));


#ifdef ATH_SUPERG_XR 
    /*
     * if its an XR vap ,free the memory allocated explicitly.
     * since the XR vap is not registered , OS can not free the memory.
     */
    if(ieee80211vap_has_flag(vap, IEEE80211_F_XR)) {
        ath_grppoll_stop(vap);
        ath_descdma_cleanup(sc,&sc->sc_grppolldma,&sc->sc_grppollbuf, BUS_DMA_FROMDEVICE);
        OS_MEMZERO(&sc->sc_grppollbuf, sizeof(sc->sc_grppollbuf));
        OS_MEMZERO(&sc->sc_grppolldma, sizeof(sc->sc_grppolldma));
#ifndef ATHR_RNWF
        if(vap->iv_xrvap)
            vap->iv_xrvap->iv_xrvap=NULL;
        kfree(vap->iv_dev);
#endif
        ath_tx_cleanupq(sc,sc->sc_xrtxq);
        sc->sc_xrtxq=NULL;
        if (sc->sc_hasdiversity) {
            /* Restore diversity setting to old diversity setting */
            ath_hal_setdiversity(ah, sc->sc_olddiversity);
            sc->sc_diversity = sc->sc_olddiversity;
        }
    }
#endif

    /* restart H/W in case there are other VAPs */
    if ((flags & ATH_NETIF_RUNNING) && sc->sc_nvaps) {
        /*
         * Restart rx+tx machines if device is still running.
         */
        if (ath_startrecv(sc) != 0)	/* restart recv */
            printk("%s: unable to start recv logic\n",
                   __func__);
        if (sc->sc_beacons) {
            ath_beacon_config(sc, ATH_IF_ID_ANY);    /* restart beacons */
        }
        ath_hal_intrset(ah, sc->sc_imask);
    }
    ATH_PS_SLEEP(sc);
    return 0;
}

static int
ath_vap_config(ath_dev_t dev, int if_id, struct ath_vap_config *if_config)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_vap *avp;

    if (if_id >= ATH_BCBUF) {
        printk("%s: Invalid interface id = %u\n", __func__, if_id);
        return -EINVAL;
    }

    avp = sc->sc_vaps[if_id];
    ASSERT(avp != NULL);

    if (avp) {
        OS_MEMCPY(&avp->av_config, if_config, sizeof(avp->av_config));
    }

    return 0;
}

#if AR_DEBUG
ath_sm_update_announce(ath_dev_t dev, ath_node_t node)
{
	char smpstate[32] = "";
	struct ath_node *an = ATH_NODE(node);
	struct ieee80211_node *ni = (struct ieee80211_node *) an->an_node;
	u_int8_t *mac = ni->ni_macaddr;
	int capflag = an->an_cap;
	if (OS_MEMCPY(ni->ni_macaddr, ni->ni_bssid, IEEE80211_ADDR_LEN) == 0) {
		return;
	}
	OS_MEMCPY(smpstate,"", 1);
	if (capflag & ATH_RC_HT_FLAG) {
		if (smenable) {
			OS_MEMCPY(smpstate, "SM Enabled", strlen("SM Enabled"));
		} else {
			if (dynamic) {
				OS_MEMCPY(smpstate, "SM Dynamic pwrsav",
				    strlen("SM Dynamic pwrsav"));
			} else {
				OS_MEMCPY(smpstate,"SM Static pwrsav",
				    strlen("SM Static pwrsav"));
			}
		}
	}
	DPRINTF(((struct ath_softc *)sc), ATH_DEBUG_PWR_SAVE,
		   "mac=%02x:%02x:%02x:"
		   "%02x:%02x:%02x : %s capflag=0x%u Flags:%s%s%s%s\n",
		   mac[0], mac[1], mac[2],
		   mac[3], mac[4], mac[5], smpstate, capflag,
		   (capflag & ATH_RC_HT_FLAG) ? " HT" : "",
		   (capflag & ATH_RC_CW40_FLAG) ? " HT40" : "",
		   (capflag & ATH_RC_SGI_FLAG) ? " SGI" : "",
		   (capflag & ATH_RC_DS_FLAG) ? " DS" : "");
}
#endif /* AR_DEBUG */

static void
ath_update_sm_pwrsave(ath_dev_t dev, ath_node_t node, ATH_SM_PWRSAV mode,
	int ratechg)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_node *an = (struct ath_node *) node;

    switch (sc->sc_opmode) {
    case HAL_M_HOSTAP:
        /*
         * Update spatial multiplexing mode of client node.
         */
        an->an_smmode = mode;
        DPRINTF(sc,ATH_DEBUG_PWR_SAVE,"%s: ancaps %#x\n", __func__, an->an_smmode);
        if (ratechg) {
            ath_rate_node_update(an);
#if AR_DEBUG
            ath_sm_update_announce(dev, node);
#endif
        }
        break;

    case HAL_M_STA:
        switch (mode) {
        case ATH_SM_ENABLE:
            /* Spatial Multiplexing enabled, revert back to default SMPS register settings */
            ath_hal_setsmpsmode(sc->sc_ah, HAL_SMPS_DEFAULT);
            break;
        case ATH_SM_PWRSAV_STATIC:
            /*
             * Update current chainmask from configuration parameter.
             * A subsequent reset is needed for new chainmask to take effect.
             */
            sc->sc_rx_chainmask = sc->sc_config.rxchainmask;
            break;
        case ATH_SM_PWRSAV_DYNAMIC:
            /* Enable hardware control of SM Power Save */
            ath_hal_setsmpsmode(sc->sc_ah, HAL_SMPS_HW_CTRL);
            break;
        default:
            break;
        }
        an->an_smmode = mode;
        if (ratechg)
            ath_rate_node_update(an);
        break;

    case HAL_M_IBSS:
    case HAL_M_MONITOR:
    default:
        /* Not supported */
        break;
    }
}

static int
athop_rate_newassoc(ath_dev_t dev, ath_node_t node, int isnew, unsigned int capflag,
                                  struct ieee80211_rateset *negotiated_rates,
                                  struct ieee80211_rateset *negotiated_htrates)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
	struct ath_node *an = (struct ath_node *) node;

    return ath_rate_newassoc(sc, an, isnew, capflag, negotiated_rates, negotiated_htrates);
}

static void
athop_rate_newstate(ath_dev_t dev, int if_id, int up)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_vap *avp = sc->sc_vaps[if_id];

    return ath_rate_newstate(sc, avp, up);
}


static ath_node_t
ath_node_attach(ath_dev_t dev, int if_id, ieee80211_node_t ni)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_vap *avp;
    struct ath_node *an;

    avp = sc->sc_vaps[if_id];
    ASSERT(avp != NULL);
    
    an = (struct ath_node *)OS_MALLOC(sc->sc_osdev, sizeof(struct ath_node), GFP_ATOMIC);
    if (an == NULL)
        return NULL;
    OS_MEMZERO(an, sizeof(*an));

    an->an_node = ni;
    an->an_sc = sc;
    an->an_decomp_index = INVALID_DECOMP_INDEX;

    an->an_rc_node = ath_rate_node_alloc(avp->av_atvp);
    if (an->an_rc_node == NULL)
        return NULL;

    /* XXX no need to setup vap pointer in ni here.
     * It should already be taken care of by caller. */
    ath_rate_node_init(an->an_rc_node);

    /* set up per-node tx/rx state */
    ath_tx_node_init(sc, an);
    ath_rx_node_init(sc, an);

#ifdef ATH_SUPPORT_UAPSD
    /* U-APSD init */
    TAILQ_INIT(&an->an_uapsd_q);
    an->an_uapsd_qdepth = 0;
#endif

#ifdef ATH_SWRETRY
    /* As of now, SW Retry mechanism will be enabled only when 
     * STA enters into RUN state. Need to revisit this part 
     * if Sw retries are to be enabled right from JOIN state
     */
    ATH_NODE_SWRETRY_TXBUF_LOCK_INIT(an);
    TAILQ_INIT(&an->an_softxmit_q);
    an->an_swrenabled = AH_FALSE;
    an->an_softxmit_qdepth = 0;
    an->an_total_swrtx_successfrms = 0;
    an->an_total_swrtx_flushfrms = 0;
    an->an_total_swrtx_pendfrms = 0;
#endif
#ifdef ATH_CHAINMASK_SELECT
    ath_chainmask_sel_init(sc, an);
    ath_chainmask_sel_timerstart(&an->an_chainmask_sel);
#endif

#if defined(ATH_SWRETRY) || defined(AP_SLOW_ANT_DIV)
    ATH_NODETABLE_LOCK(sc);
    LIST_INSERT_HEAD(&sc->sc_nt, an, an_le);
    ATH_NODETABLE_UNLOCK(sc);
#endif

    DPRINTF(sc, ATH_DEBUG_NODE, "%s: an %p\n", __func__, an);
    return an;
}

static void
ath_node_detach(ath_dev_t dev, ath_node_t node)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_node *an = ATH_NODE(node);

    ath_tx_node_free(sc, an);
    ath_rx_node_free(sc, an);

#ifdef ATH_SUPERG_XR
    ath_grppoll_period_update(sc);
#endif

    ath_rate_node_free(an->an_rc_node);

#if defined(ATH_SWRETRY) || defined(AP_SLOW_ANT_DIV)
    ATH_NODETABLE_LOCK(sc);
    LIST_REMOVE(an, an_le);
    ATH_NODETABLE_UNLOCK(sc);
#endif

    OS_FREE(an);
}

static void
ath_node_cleanup(ath_dev_t dev, ath_node_t node)
{
    struct ath_node *an = ATH_NODE(node);
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);

#ifdef ATH_CHAINMASK_SELECT
    ath_chainmask_sel_timerstop(&an->an_chainmask_sel);
#endif

#ifdef ATH_SWRETRY
    if (sc->sc_swRetryEnabled)
        ath_tx_flush_sxmitq(sc);
#endif

    an->an_flags |= ATH_NODE_CLEAN;
#ifdef ATH_SUPPORT_UAPSD
    if (an->an_flags & ATH_NODE_UAPSD) {
        an->an_flags &= ~ATH_NODE_UAPSD;
        ath_tx_uapsd_node_cleanup(sc, an);
    }
#endif
    ath_tx_node_cleanup(sc, an);
    ath_rx_node_cleanup(sc, an);
    ath_rate_node_cleanup(an->an_rc_node);
}

static void
ath_node_update_pwrsave(ath_dev_t dev, ath_node_t node, int pwrsave)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
	struct ath_node *an = ATH_NODE(node);
    if (pwrsave) {
        an->an_flags |= ATH_NODE_PWRSAVE;
        ath_tx_node_pause(sc,an);
    } else {
        an->an_flags &= ~ATH_NODE_PWRSAVE;
        ath_tx_node_resume(sc,an);
    }
}
/*
 * Allocate data  key slots for TKIP.  We allocate two slots for
 * one for decrypt/encrypt and the other for the MIC.
 */
static u_int16_t
ath_key_alloc_pair(ath_dev_t dev)
{
#define	N(a)	(sizeof(a)/sizeof(a[0]))
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    u_int i, keyix;

    /* XXX could optimize */
    for (i = 0; i < N(sc->sc_keymap)/2; i++) {
        u_int8_t b = sc->sc_keymap[i];
        if (b != 0xff) {
            /*
             * One or more slots in this byte are free.
             */
            keyix = i*NBBY;
            while (b & 1) {
        again:
                keyix++;
                b >>= 1;
            }
            /* XXX IEEE80211_KEY_XMIT | IEEE80211_KEY_RECV */
            if ( isset(sc->sc_keymap, keyix+64) ) {
                /* pair unavailable */
                /* XXX statistic */
                if (keyix == (i+1)*NBBY) {
                    /* no slots were appropriate, advance */
                    continue;
                }
                goto again;
            }
            setbit(sc->sc_keymap, keyix);
            setbit(sc->sc_keymap, keyix+64);
            DPRINTF(sc, ATH_DEBUG_KEYCACHE,
                "%s: key pair %u ,%u\n",
                __func__, keyix, keyix+64 );
            return keyix;
        }
    }
    DPRINTF(sc, ATH_DEBUG_KEYCACHE, "%s: out of pair space\n", __func__);
    return -1;
#undef N
}

/*
 * Allocate tx/rx key slots for TKIP.  We allocate two slots for
 * each key, one for decrypt/encrypt and the other for the MIC.
 */
static u_int16_t
ath_key_alloc_2pair(ath_dev_t dev)
{
#define	N(a)	(sizeof(a)/sizeof(a[0]))
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    u_int i, keyix;

    KASSERT(ath_hal_tkipsplit(sc->sc_ah), ("key cache !split"));
    /* XXX could optimize */
    for (i = 0; i < N(sc->sc_keymap)/4; i++) {
        u_int8_t b = sc->sc_keymap[i];
        if (b != 0xff) {
            /*
             * One or more slots in this byte are free.
             */
            keyix = i*NBBY;
            while (b & 1) {
again:
                keyix++;
                b >>= 1;
            }
            /* XXX IEEE80211_KEY_XMIT | IEEE80211_KEY_RECV */
            if (isset(sc->sc_keymap, keyix+32) ||
                isset(sc->sc_keymap, keyix+64) ||
                isset(sc->sc_keymap, keyix+32+64)) {
                /* full pair unavailable */
                /* XXX statistic */
                if (keyix == (i+1)*NBBY) {
                    /* no slots were appropriate, advance */
                    continue;
                }
                goto again;
            }
            setbit(sc->sc_keymap, keyix);
            setbit(sc->sc_keymap, keyix+64);
            setbit(sc->sc_keymap, keyix+32);
            setbit(sc->sc_keymap, keyix+32+64);
#if 0
            DPRINTF(sc, ATH_DEBUG_KEYCACHE,
                    "%s: key pair %u,%u %u,%u\n",
                    __func__, keyix, keyix+64,
                    keyix+32, keyix+32+64);
#endif
            return keyix;
        }
    }
    DPRINTF(sc, ATH_DEBUG_KEYCACHE, "%s: out of pair space\n", __func__);
    return -1;
#undef N
}

/*
 * Allocate a single key cache slot.
 */
static u_int16_t
ath_key_alloc_single(ath_dev_t dev)
{
#define	N(a)	(sizeof(a)/sizeof(a[0]))
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    u_int i, keyix;

    /* XXX try i,i+32,i+64,i+32+64 to minimize key pair conflicts */
    for (i = 0; i < N(sc->sc_keymap); i++) {
        u_int8_t b = sc->sc_keymap[i];
        if (b != 0xff) {
            /*
             * One or more slots are free.
             */
            keyix = i*NBBY;
            while (b & 1) {
                keyix++, b >>= 1;
            }
            setbit(sc->sc_keymap, keyix);
            DPRINTF(sc, ATH_DEBUG_KEYCACHE, "%s: key %u\n",
                    __func__, keyix);
            return keyix;
        }
    }
    DPRINTF(sc, ATH_DEBUG_KEYCACHE, "%s: out of space\n", __func__);
    return -1;
#undef N
}

static void
ath_key_reset(ath_dev_t dev, u_int16_t keyix, int freeslot)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);

    ATH_PS_WAKEUP(sc);

    ath_hal_keyreset(sc->sc_ah, keyix);
    if (freeslot)
        clrbit(sc->sc_keymap, keyix);

    ATH_PS_SLEEP(sc);
}

static int
ath_keyset(ath_dev_t dev, u_int16_t keyix, HAL_KEYVAL *hk,
           const u_int8_t mac[IEEE80211_ADDR_LEN])
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    HAL_BOOL status;
    
    ATH_FUNC_ENTRY_CHECK(sc, 0);
    ATH_PS_WAKEUP(sc);

    status = ath_hal_keyset(sc->sc_ah, keyix, hk, mac);

    ATH_PS_SLEEP(sc);

    return (status != AH_FALSE);
}

static u_int
ath_keycache_size(ath_dev_t dev)
{
    return ATH_DEV_TO_SC(dev)->sc_keymax;
}

static void
ath_set_divant(ath_dev_t dev, int divant_value)

{
  struct ath_softc *sc = ATH_DEV_TO_SC(dev);
  struct ath_hal *ah = sc->sc_ah;
  if (divant_value == 0)
  {
      sc->sc_antDivComb = 0;
      ath_hal_divant(ah, 0);
  }
  else
  {
      sc->sc_antDivComb = 1;
      ath_hal_divant(ah, 1);
      ath_ant_div_comb_init(&sc->sc_antcomb, sc);
  }    
}



/*
 * Set the 802.11D country
 */
static int
ath_set_country(ath_dev_t dev, char *isoName, u_int16_t cc)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;
    HAL_CHANNEL *chans;
    int nchan;
    u_int8_t regclassids[ATH_REGCLASSIDS_MAX];
    u_int nregclass = 0;
    u_int wMode;
    u_int netBand;
    int outdoor = ath_outdoor;

    if(!isoName || !isoName[0] || !isoName[1]) {
        if (cc) 
            sc->sc_config.ath_countrycode = cc;
        else
            sc->sc_config.ath_countrycode = CTRY_DEFAULT;
    } else {
        sc->sc_config.ath_countrycode = findCountryCode((u_int8_t *)isoName);
 
        /* Map the ISO name ' ', 'I', 'O' */
        if((isoName[2] == ' ') || (isoName[2] == 'O'))
            outdoor = AH_TRUE;
        else
            outdoor = AH_FALSE;
    }

    wMode = sc->sc_reg_parm.wModeSelect;
    if (!(wMode & HAL_MODE_11A)) {
        wMode &= ~(HAL_MODE_TURBO|HAL_MODE_108A|HAL_MODE_11A_HALF_RATE);
    }
    if (!(wMode & HAL_MODE_11G)) {
        wMode &= ~(HAL_MODE_108G);
    }
    netBand = sc->sc_reg_parm.NetBand;
    if (!(netBand & HAL_MODE_11A)) {
        netBand &= ~(HAL_MODE_TURBO|HAL_MODE_108A|HAL_MODE_11A_HALF_RATE);
    }
    if (!(netBand & HAL_MODE_11G)) {
        netBand &= ~(HAL_MODE_108G);
    }
    wMode &= netBand;

    chans = (HAL_CHANNEL *)OS_MALLOC(sc->sc_osdev, 
                      IEEE80211_CHAN_MAX * sizeof(HAL_CHANNEL), 
                      GFP_ATOMIC);

    if (chans == NULL) {
        printk("%s: unable to allocate channel table\n", __func__);
        return -ENOMEM;
    }
    
    ATH_PS_WAKEUP(sc);
    if (!ath_hal_init_channels(ah, chans, IEEE80211_CHAN_MAX, (u_int *)&nchan,
                               regclassids, ATH_REGCLASSIDS_MAX, &nregclass,
                               sc->sc_config.ath_countrycode, wMode, outdoor, 
                               sc->sc_config.ath_xchanmode)) {
        OS_FREE(chans);
        ATH_PS_SLEEP(sc);
        return -EINVAL;
    }

#ifdef ATH_SUPPORT_DFS
    /* Setting country code might change the DFS domain
     * so  initialize the DFS Radar filters */
    dfs_init_radar_filters(sc);
#endif
    ATH_PS_SLEEP(sc);

    if (sc->sc_ieee_ops->setup_channel_list) {
        sc->sc_ieee_ops->setup_channel_list(sc->sc_ieee, CLIST_UPDATE,
                                            chans, nchan, regclassids, nregclass,
                                            sc->sc_config.ath_countrycode);
    }

    OS_FREE(chans);     
    return 0;
}

/*
 * Return the current country and domain information
 */
static void
ath_get_currentCountry(ath_dev_t dev, HAL_COUNTRY_ENTRY *ctry)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);

    ath_hal_getCurrentCountry(sc->sc_ah, ctry);

    /* If HAL not specific yet, since it is band dependent, use the one we passed in.*/
    if (ctry->countryCode == CTRY_DEFAULT) {
        ctry->iso[0] = 0;
        ctry->iso[1] = 0;
    }
    else if (ctry->iso[0] && ctry->iso[1]) {
        if (!ctry->iso[2]) {
            if (ath_outdoor)
                ctry->iso[2] = 'O';
            else
                ctry->iso[2] = ' ';	/* Per Mark, for XBOX interop don't use "I" for indoor */
        }
    }
}

static int
ath_set_regdomain(ath_dev_t dev, int regdomain)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    HAL_STATUS status;
  
    if (ath_hal_setregdomain(sc->sc_ah, regdomain, &status) == AH_TRUE)
        return 0;
    else
        return -EIO;
}

static int
ath_get_regdomain(ath_dev_t dev)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    u_int32_t regdomain;
    HAL_STATUS status;

    status = ath_hal_getregdomain(sc->sc_ah, &regdomain);
    KASSERT(status == HAL_OK, ("get_regdomain failed"));

    return (regdomain);
}

static int
ath_set_quiet(ath_dev_t dev, u_int16_t period, u_int16_t duration,
              u_int16_t nextStart, u_int16_t enabled)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    return ath_hal_setQuiet(sc->sc_ah, period, period, nextStart, enabled);
}

static u_int16_t
ath_find_countrycode(ath_dev_t dev, char* isoName)
{
    UNREFERENCED_PARAMETER(dev);

    return findCountryCode((u_int8_t*)isoName);
}

static int
ath_set_tx_antenna_select(ath_dev_t dev, u_int32_t antenna_select)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    u_int8_t antenna_cfgd = 0;

    if (!sc->sc_cfg_txchainmask) {
        sc->sc_cfg_txchainmask = sc->sc_config.txchainmask;
        sc->sc_cfg_rxchainmask = sc->sc_config.rxchainmask;
        sc->sc_cfg_txchainmask_leg = sc->sc_config.txchainmasklegacy;
        sc->sc_cfg_rxchainmask_leg = sc->sc_config.rxchainmasklegacy;
    }

    if (ath_hal_setAntennaSwitch(sc->sc_ah, antenna_select, &sc->sc_curchan,
                                 &sc->sc_tx_chainmask, &sc->sc_rx_chainmask,
                                 &antenna_cfgd)) {
        if (antenna_cfgd) {
            if (antenna_select) {
                /* Overwrite chainmask configuration so that antenna selection is 
                 * retained during join.
                 */
                sc->sc_config.txchainmask = sc->sc_tx_chainmask;
                sc->sc_config.rxchainmask = sc->sc_rx_chainmask;
                sc->sc_config.txchainmasklegacy = sc->sc_tx_chainmask;
                sc->sc_config.rxchainmasklegacy = sc->sc_rx_chainmask;
            } else {
                /* Restore original chainmask config */
                sc->sc_config.txchainmask = sc->sc_cfg_txchainmask;
                sc->sc_config.rxchainmask = sc->sc_cfg_rxchainmask;
                sc->sc_config.txchainmasklegacy = sc->sc_cfg_txchainmask_leg;
                sc->sc_config.rxchainmasklegacy = sc->sc_cfg_rxchainmask_leg;
                sc->sc_cfg_txchainmask = 0;
            }
        }
        printk("%s: Tx Antenna Switch. Do internal reset.\n", __func__);
        ath_internal_reset(sc);
        return 0;
    }
    return -EIO;
}

static u_int32_t
ath_get_current_tx_antenna(ath_dev_t dev)
{
    return ATH_DEV_TO_SC(dev)->sc_cur_txant;
}

static u_int32_t
ath_get_default_antenna(ath_dev_t dev)
{
    return ATH_DEV_TO_SC(dev)->sc_defant;
}

#ifdef DBG
static u_int32_t
ath_register_read(ath_dev_t dev, u_int32_t offset)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    return(ath_hal_readRegister(sc->sc_ah, offset));
}

static void
ath_register_write(ath_dev_t dev, u_int32_t offset, u_int32_t value)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    ath_hal_writeRegister(sc->sc_ah, offset, value);
}
#endif

void
ath_setTxPwrLimit(ath_dev_t dev, u_int32_t limit, u_int16_t tpcInDb)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);

    sc->sc_config.txpowlimit = (u_int16_t)limit;

    ath_update_txpow(sc, tpcInDb);
}

u_int8_t
ath_get_common_power(u_int16_t freq)
{
    return getCommonPower(freq);
}

#ifdef ATH_CHAINMASK_SELECT

static int
ath_chainmask_sel_timertimeout(void *timerArg)
{
    struct ath_chainmask_sel *cm = (struct ath_chainmask_sel *)timerArg;
    cm->switch_allowed = 1;
    return 1; /* no re-arm  */
}

/*
 * Start chainmask select timer
 */
static void
ath_chainmask_sel_timerstart(struct ath_chainmask_sel *cm)
{
    cm->switch_allowed = 0;
    ath_start_timer(&cm->timer);
}

/*
 * Stop chainmask select timer
 */
static void
ath_chainmask_sel_timerstop(struct ath_chainmask_sel *cm)
{
    cm->switch_allowed = 0;
    ath_cancel_timer(&cm->timer, CANCEL_NO_SLEEP);
}

int
ath_chainmask_sel_logic(struct ath_softc *sc, struct ath_node *an)
{
    struct ath_chainmask_sel *cm  = &an->an_chainmask_sel;

    /*
     * Disable auto-swtiching in one of the following if conditions.
     * sc_chainmask_auto_sel is used for internal global auto-switching
     * enabled/disabled setting
     */
    if ((sc->sc_no_tx_3_chains == AH_FALSE) ||
        (sc->sc_config.chainmask_sel == AH_FALSE)) {
        cm->cur_tx_mask = sc->sc_config.txchainmask;
        return cm->cur_tx_mask;
    }

    if (cm->tx_avgrssi == ATH_RSSI_DUMMY_MARKER) {
        return cm->cur_tx_mask;
    }

    if (cm->switch_allowed) {
        /* Switch down from tx 3 to tx 2. */
    	if (cm->cur_tx_mask == ATH_CHAINMASK_SEL_3X3  &&
            ATH_RSSI_OUT(cm->tx_avgrssi) >= ath_chainmask_sel_down_rssi_thres) {
            cm->cur_tx_mask = sc->sc_config.txchainmask;

            /* Don't let another switch happen until this timer expires */
            ath_chainmask_sel_timerstart(cm);
        }
        /* Switch up from tx 2 to 3. */
    	else if (cm->cur_tx_mask == sc->sc_config.txchainmask  &&
                 ATH_RSSI_OUT(cm->tx_avgrssi) <= ath_chainmask_sel_up_rssi_thres) {
            cm->cur_tx_mask =  ATH_CHAINMASK_SEL_3X3;

            /* Don't let another switch happen until this timer expires */
            ath_chainmask_sel_timerstart(cm);
    	}
    }

    return cm->cur_tx_mask;
}

static void
ath_chainmask_sel_init(struct ath_softc *sc, struct ath_node *an)
{
    struct ath_chainmask_sel *cm  = &an->an_chainmask_sel;

    OS_MEMZERO(cm, sizeof(struct ath_chainmask_sel));

    cm->cur_tx_mask = sc->sc_config.txchainmask;
    cm->cur_rx_mask = sc->sc_config.rxchainmask;
    cm->tx_avgrssi = ATH_RSSI_DUMMY_MARKER;

    ath_initialize_timer(sc->sc_osdev, &cm->timer, ath_chainmask_sel_period,
                         ath_chainmask_sel_timertimeout, cm);
}

#endif

static int16_t
ath_get_noisefloor(ath_dev_t dev, u_int16_t	freq,  u_int chan_flags)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
	struct ath_hal *ah = sc->sc_ah;
	HAL_CHANNEL hchan;
	
	hchan.channel = freq;
	hchan.channelFlags = chan_flags;

	return ah->ah_getChanNoise(ah,&hchan);
}

#ifdef ATH_SUPPORT_DFS
static
OS_TIMER_FUNC(ath_radar_task)
{
        struct ath_softc *sc;
        struct ath_hal *ah;
        struct ieee80211_channel ichan;
        HAL_CHANNEL hchan;
        int i=0;
        struct ath_vap *avp=NULL;
	struct ieee80211vap *vap = NULL;

        OS_GET_TIMER_ARG(sc, struct ath_softc *);
        ah = sc->sc_ah;

        sc->sc_dfs->sc_rtasksched = 0;
        OS_CANCEL_TIMER(&sc->sc_dfs->sc_dfs_task_timer);
        if (dfs_process_radarevent(sc,&hchan)) {
                /*
                 * radar was detected on this channel, initiate channel change
                 */
                ichan.ic_ieee = ath_hal_mhz2ieee(ah, hchan.channel, hchan.channelFlags);
                ichan.ic_freq = hchan.channel;
                ichan.ic_flags = hchan.channelFlags;

                /* do channel switch only when usenol is set to true -
                 * otherwise, for test mode
                 * disable channel switch - as per owl_sw_dev
                 */
                if ( sc->sc_dfs->dfs_rinfo.rn_use_nol ) {
                    for (i=0; i < sc->sc_nvaps; i++) {
                        avp = sc->sc_vaps[i];
                        if (avp) {
			    vap = (struct ieee80211vap *)avp->av_if_data;
                            /* Bug fix for 52582 and 54279 - set VAP to RUN state before calling mark_dfs
                               this is required for channel switch to happen */
                            if (vap->iv_state != 5 && (!sc->sc_dfs->sc_dfswait))
                                sc->sc_ieee_ops->ath_net80211_set_vap_state(sc->sc_ieee,i,5);
                            /* Bug fix for 58032, if in DFS WAIT, make sure VAP state is INIT
                               so that channel switch happens */
                            if (sc->sc_dfs->sc_dfswait)
                                sc->sc_ieee_ops->ath_net80211_set_vap_state(sc->sc_ieee,i,0);
                        }
                    }
                    sc->sc_ieee_ops->ath_net80211_mark_dfs(sc->sc_ieee, &ichan);
                }

		if ((!sc->sc_dfs->dfs_rinfo.rn_use_nol) &&
                    (sc->sc_opmode == HAL_M_HOSTAP)) {
                /* TEST : To hold to the same channel, though detected RADAR */
                /* The actual channel change will be done in ieee80211_mark_dfs,
                   but schedule the timer that will return us to the current channel
                   here */
                       printk("Radar found on channel %d (%d MHz)\n",
                               ichan.ic_ieee,ichan.ic_freq);
                       sc->sc_dfs->sc_dfstest_ieeechan = ichan.ic_ieee;
                       sc->sc_dfs->sc_dfstest=1;
                       OS_CANCEL_TIMER(&sc->sc_dfs->sc_dfstesttimer);
                       OS_SET_TIMER(&sc->sc_dfs->sc_dfstesttimer, sc->sc_dfs->sc_dfstesttime);
                }
        }
}

static
OS_TIMER_FUNC(ath_dfs_test_return)
{
        struct ath_softc *sc;
        OS_GET_TIMER_ARG(sc, struct ath_softc *);
        sc->sc_dfs->sc_dfstest=0;
        sc->sc_ieee_ops->ath_net80211_dfs_test_return(sc->sc_ieee, sc->sc_dfs->sc_dfstest_ieeechan);
}

int 
ath_dfs_control(ath_dev_t dev, u_int id,
                void *indata, u_int32_t insize,
                void *outdata, u_int32_t *outsize)
{
        int error=0;

        error = dfs_control(dev, id, indata, insize, outdata, outsize);
        return error;
}

int 
ath_check_dfs_wait(ath_dev_t dev)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    if (sc->sc_dfs && sc->sc_dfs->sc_dfswait) 
	return 1;
    else
	return 0;
}

/*
 * periodically checks for the hal to set
 * CHANNEL_DFS_CLEAR flag on current channel.
 * if the flag is set and a vap is waiting for it ,push
 * transition the vap to RUN state.
 */
static
OS_TIMER_FUNC(ath_check_dfs_clear)
{
        struct ath_softc *sc;
        HAL_CHANNEL hchan;
        int i;

        OS_GET_TIMER_ARG(sc, struct ath_softc *);

        if(!sc->sc_dfs->sc_dfswait) return;

        /* Check with HAL, HAL clears the relevant channel flags if 
           channel is good to go. */
        ath_hal_radar_wait(sc->sc_ah, &hchan);

        if(hchan.privFlags & CHANNEL_INTERFERENCE) return;

        if ((hchan.privFlags & CHANNEL_DFS_CLEAR) ||
                (!(hchan.privFlags & CHANNEL_DFS))) {

                /* Bug 32403 - Once the CAC is over, clear the DFS_CLEAR flag,
                   so that every single time a DFS channel is set, CAC will 
                   happen, irrespective of whether channel has been used before*/
                sc->sc_curchan.privFlags &= ~CHANNEL_DFS_CLEAR;
               
                sc->sc_dfs->sc_dfswait=0;
                printk("End of DFS wait period\n");
                for (i=0; i < sc->sc_nvaps; i++) {

                        /* 
                        ** This will change later, an event will be sent to all
                        ** interested VAPs saying DFS wait is now clear and the
                        ** VAP will set its state to RUN after that.
                        */

                        struct ath_vap *avp = sc->sc_vaps[i];
                        if(avp->av_dfswait_run) {
                            /* re alloc beacons to update new channel info */
                            int error;
                            error = ath_beacon_alloc(sc, i);
                            if(error) {
                                       printk("%s error allocating beacon\n", __func__);
                                       return;
                            }

                            /*
                            ** Pass the VAP ID number, gets resolved inside call
                            */

                            sc->sc_ieee_ops->ath_net80211_set_vap_state(sc->sc_ieee,i,5);
#ifdef ATH_SUPERG_XR
                            if(vap->iv_flags & IEEE80211_F_XR ) {
                                       u_int32_t rfilt=0;
                                       rfilt = ath_calcrxfilter(sc);
                                       ATH_SETUP_XR_VAP(sc,vap,rfilt);
                            }
#endif
                        avp->av_dfswait_run=0;
                        } 
                }
        /* Make sure beaconing resumes after DFS wait is over. See bugs 54279 and 52582*/
        if (sc->sc_beacons) {
            ath_beacon_config(sc, ATH_IF_ID_ANY);
        }
    } else {
                /* fire the timer again */
                    sc->sc_dfs->sc_dfswait=1;
                    OS_CANCEL_TIMER(&sc->sc_dfs->sc_dfswaittimer);
                    OS_SET_TIMER(&sc->sc_dfs->sc_dfswaittimer, ATH_DFS_WAIT_POLL_PERIOD_MS);
        }
}
#endif

#ifdef ATH_CCX
int
ath_update_mib_macstats(ath_dev_t dev)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;

    ATH_PS_WAKEUP(sc);
    ath_hal_updateMibMacStats(ah);
    ATH_PS_SLEEP(sc);

    return 0;
}

int
ath_get_mib_macstats(ath_dev_t dev, struct ath_mib_mac_stats *pStats)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;

    if (pStats == NULL) {
        return -EINVAL;
    }

    ath_hal_getMibMacStats(ah, (HAL_MIB_STATS*)pStats);
    return 0;
}

int
ath_get_mib_cyclecounts(ath_dev_t dev, struct ath_mib_cycle_cnts *pCnts)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;

    if (pCnts == NULL) {
        return -EINVAL;
    }

    ath_hal_getMibCycleCounts(ah, (HAL_COUNTERS*)pCnts);
    return 0;
}

void
ath_clear_mib_counters(ath_dev_t dev)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;

    ath_hal_clearMibCounters(ah);
}

u_int32_t
ath_gettsf32(ath_dev_t dev)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;

    return ath_hal_gettsf32(ah);
}

u_int64_t
ath_gettsf64(ath_dev_t dev)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;

    return ath_hal_gettsf64(ah);
}

void
ath_setrxfilter(ath_dev_t dev)
{
    struct ath_softc    *sc = ATH_DEV_TO_SC(dev);
    u_int32_t           rxfilt;

    rxfilt = ath_calcrxfilter(sc);
    ath_hal_setrxfilter(sc->sc_ah, rxfilt);
}

int
ath_getserialnumber(ath_dev_t dev, u_int8_t *pSerNum)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;

    if (pSerNum == NULL) {
        return -EINVAL;
    }

    ath_hal_get_sernum(ah, pSerNum);
    return 0;
}

int
ath_getchandata(ath_dev_t dev, struct ieee80211_channel *pChan, struct ath_chan_data *pData)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;
    if (ath_hal_get_chandata(ah, (HAL_CHANNEL*)pChan, (HAL_CHANNEL_DATA*)pData) == AH_TRUE){
        return 0;
    } else {
        return -EINVAL;
    }
}

u_int32_t
ath_getcurrssi(ath_dev_t dev)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;
    return ath_hal_getCurRSSI(ah);
}
#endif /* ATH_CCX */

static void
ath_printreg(ath_dev_t dev, u_int32_t printctrl)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;

    ATH_PS_WAKEUP(sc);

    ath_hal_getdiagstate(ah, HAL_DIAG_PRINT_REG, &printctrl, sizeof(printctrl), NULL, 0);

    ATH_PS_SLEEP(sc);
}

u_int32_t
ath_getmfpsupport(ath_dev_t dev)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;
    u_int32_t        mfpsupport;
    ath_hal_getmfpsupport(ah, &mfpsupport);
    switch(mfpsupport) {
    case HAL_MFP_QOSDATA:
        return ATH_MFP_QOSDATA;
    case HAL_MFP_PASSTHRU:
        return ATH_MFP_PASSTHRU;
    case HAL_MFP_HW_CRYPTO:
        return ATH_MFP_HW_CRYPTO;
    default:
        return ATH_MFP_QOSDATA;
    };
}

