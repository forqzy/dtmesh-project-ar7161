/*****************************************************************************/
/* \file ath_config.c
** \brief Configuration code for controlling the ATH layer.
**
**  This file contains the routines used to configure and control the ATH
**  object once instantiated.
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

#include "ath_internal.h"


/******************************************************************************/
/*!
**  \brief Set ATH configuration parameter
**
**  This routine will set ATH parameters to the provided values.  The incoming
**  values are always encoded as integers -- conversions are done here.
**
**  \param dev Handle for the ATH device
**  \param ID Parameter ID value, as defined in the ath_param_ID_t type
**  \param buff Buffer containing the value, currently always an integer
**  \return 0 on success
**  \return -1 for unsupported values
*/

int
ath_set_config(ath_dev_t dev, ath_param_ID_t ID, void *buff)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
#ifdef ATH_RB
    int value = *(int *)buff;
#endif

    switch(ID)
    {
        case ATH_PARAM_TXCHAINMASK:
            sc->sc_config.txchainmask = *(int *)buff;
        break;

        case ATH_PARAM_RXCHAINMASK:
            sc->sc_config.rxchainmask = *(int *)buff;
        break;

        case ATH_PARAM_TXCHAINMASKLEGACY:
            sc->sc_config.txchainmasklegacy = *(int *)buff;
        break;

        case ATH_PARAM_RXCHAINMASKLEGACY:
            sc->sc_config.rxchainmasklegacy = *(int *)buff;
        break;

        case ATH_PARAM_CHAINMASK_SEL:
            sc->sc_config.chainmask_sel = *(int *)buff;
        break;

        case ATH_PARAM_AMPDU:
            sc->sc_txaggr = *(int *)buff;
            sc->sc_rxaggr = *(int *)buff;
        break;

        case ATH_PARAM_AMPDU_LIMIT:
            sc->sc_config.ampdu_limit = *(int *)buff;
        break;

        case ATH_PARAM_AMPDU_SUBFRAMES:
            sc->sc_config.ampdu_subframes = *(int *)buff;
        break;

        case ATH_PARAM_AGGR_PROT:
            sc->sc_config.ath_aggr_prot = *(int *)buff;
        break;

        case ATH_PARAM_AGGR_PROT_DUR:
            sc->sc_config.ath_aggr_prot_duration = *(int *)buff;
        break;

        case ATH_PARAM_AGGR_PROT_MAX:
            sc->sc_config.ath_aggr_prot_max = *(int *)buff;
        break;

        case ATH_PARAM_TXPOWER_LIMIT:
            sc->sc_config.txpowlimit = *(int *)buff;
        break;

        case ATH_PARAM_TXPOWER_OVERRIDE:
            sc->sc_config.txpowlimit_override = *(int *)buff;
        break;

        case ATH_PARAM_PCIE_DISABLE_ASPM_WK:
            sc->sc_config.pcieDisableAspmOnRfWake = *(int *)buff;
        break;

        case ATH_PARAM_PCID_ASPM:
            sc->sc_config.pcieAspm = *(int *)buff;
        break;

        case ATH_PARAM_BEACON_NORESET:
            sc->sc_noreset = *(int *)buff;
        break;

        case ATH_PARAM_CAB_CONFIG:
            sc->sc_config.cabqReadytime = *(int *)buff;
            ath_cabq_update(sc);
        break;

        case ATH_PARAM_ATH_DEBUG:
            sc->sc_debug = *(int *)buff;
        break;

        case ATH_PARAM_ATH_TPSCALE:
           sc->sc_config.tpscale = *(int *)buff;
           ath_update_tpscale(sc); 
        break;

        case ATH_PARAM_ACKTIMEOUT:
           /* input is in usec */
           ath_hal_setacktimeout(sc->sc_ah, (*(int *)buff));
        break;

#ifdef ATH_RB
        case ATH_PARAM_RX_RB:
            if (!sc->sc_do_rb_war ||
                ATH_RB_MODE_DETECT == sc->sc_rxrifs ||
                !(value ^ sc->sc_rxrifs))
                return (-1);
            if (value)
                sc->sc_rxrifs = ATH_RB_MODE_FORCE;
            else
                sc->sc_rxrifs = ATH_RB_MODE_OFF;
            ath_rb_set(sc, value);
        break;

        case ATH_PARAM_RX_RB_DETECT:
            if (!sc->sc_do_rb_war ||
                ATH_RB_MODE_FORCE == sc->sc_rxrifs ||
                !(value ^ sc->sc_rxrifs))
                return (-1);
            if (value) {
                ath_rb_reset(sc);
                sc->sc_rxrifs = ATH_RB_MODE_DETECT;
            } else {
                /* Not really off if detection has already triggered.
                 * Settings will be reverted when the timeout routine runs.
                 */
                sc->sc_rxrifs = ATH_RB_MODE_OFF;
            }
        break;

        case ATH_PARAM_RX_RB_TIMEOUT:
            sc->sc_rxrifs_timeout = *(int *)buff;
        break;

        case ATH_PARAM_RX_RB_SKIPTHRESH:
            sc->sc_rxrifs_skipthresh = *(int *)buff;
        break;
#endif
        case ATH_PARAM_AMSDU_ENABLE:
            sc->sc_txamsdu = *(int *)buff;
        break;
#ifdef ATH_SUPPORT_IQUE
		case ATH_PARAM_RETRY_DURATION:
			sc->sc_retry_duration = *(int *)buff;
		break;

		case ATH_PARAM_HBR_HIGHPER:
			sc->sc_hbr_per_high = *(int *)buff;
		break;
		
		case ATH_PARAM_HBR_LOWPER:
			sc->sc_hbr_per_low = *(int *)buff;
		break;
#endif
        case ATH_PARAM_RX_STBC:
            sc->sc_rxstbcsupport = *(int *)buff;
        break;

        case ATH_PARAM_TX_STBC:
            sc->sc_txstbcsupport = *(int *)buff;
        break;

        case ATH_PARAM_LIMIT_LEGACY_FRM:
            sc->sc_limit_legacy_frames = *(int *)buff;
        break;

        case ATH_PARAM_TOGGLE_IMMUNITY:
            sc->sc_toggle_immunity = *(int *)buff;
        break;
	
	case ATH_PARAM_GPIO_LED_CUSTOM:
	    sc->sc_reg_parm.gpioLedCustom = *(int *)buff;
	    ath_reinit_led(sc);
	break;

	case ATH_PARAM_SWAP_DEFAULT_LED:
	    sc->sc_reg_parm.swapDefaultLED =  *(int *)buff;
	    ath_reinit_led(sc);
	break;

        case ATH_PARAM_USE_EAP_LOWEST_RATE:
            sc->sc_eap_lowest_rate = *(int *)buff;
        break;

#ifdef AP_SLOW_ANT_DIV
        case ATH_PARAM_SLOWANTDIV_RSSITHRHIGH:
            sc->sc_antdiv.rssiThrHigh = *(int *)buff;
        break;

        case ATH_PARAM_SLOWANTDIV_RSSITHRLOW:
            sc->sc_antdiv.rssiThrLow = *(int *)buff;
        break;

        case ATH_PARAM_SLOWANTDIV_BYTESTHRRX:
            sc->sc_antdiv.bytesThrRx = *(int *)buff;
        break;

        case ATH_PARAM_SLOWANTDIV_DWELL_TIME:
            sc->sc_antdiv.antdiv_min_dwell_time = *(int *)buff * 1000;
        break;

        case ATH_PARAM_SLOWANTDIV_SETTLE_TIME:
            sc->sc_antdiv.antdiv_settle_time = *(int *)buff * 1000;
        break;

        case ATH_PARAM_SLOWANTDIV_ENABLE:
            (void)ath_slow_ant_div_setenable(sc, (*(int *)buff));
        break;
#endif

        default:
            return (-1);
    }

    return (0);
}

/******************************************************************************/
/*!
**  \brief Get ATH configuration parameter
**
**  This returns the current value of the indicated parameter.  Conversion
**  to integer done here.
**
**  \param dev Handle for the ATH device
**  \param ID Parameter ID value, as defined in the ath_param_ID_t type
**  \param buff Buffer to place the integer value into.
**  \return 0 on success
**  \return -1 for unsupported values
*/

int
ath_get_config(ath_dev_t dev, ath_param_ID_t ID, void *buff)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);

    switch(ID)
    {
        case ATH_PARAM_TXCHAINMASK:
            *(int *)buff = sc->sc_config.txchainmask;
        break;

        case ATH_PARAM_RXCHAINMASK:
            *(int *)buff = sc->sc_config.rxchainmask;
        break;

        case ATH_PARAM_TXCHAINMASKLEGACY:
            *(int *)buff = sc->sc_config.txchainmasklegacy;
        break;

        case ATH_PARAM_RXCHAINMASKLEGACY:
            *(int *)buff = sc->sc_config.rxchainmasklegacy;
        break;
        case ATH_PARAM_CHAINMASK_SEL:
            *(int *)buff = sc->sc_config.chainmask_sel;
        break;

        case ATH_PARAM_AMPDU:
            *(int *)buff = sc->sc_txaggr;
        break;

        case ATH_PARAM_AMPDU_LIMIT:
            *(int *)buff = sc->sc_config.ampdu_limit;
        break;

        case ATH_PARAM_AMPDU_SUBFRAMES:
            *(int *)buff = sc->sc_config.ampdu_subframes;
        break;

        case ATH_PARAM_AGGR_PROT:
            *(int *)buff = sc->sc_config.ath_aggr_prot;
        break;

        case ATH_PARAM_AGGR_PROT_DUR:
            *(int *)buff = sc->sc_config.ath_aggr_prot_duration;
        break;

        case ATH_PARAM_AGGR_PROT_MAX:
            *(int *)buff = sc->sc_config.ath_aggr_prot_max;
        break;

        case ATH_PARAM_TXPOWER_LIMIT:
            *(int *)buff = sc->sc_config.txpowlimit;
        break;

        case ATH_PARAM_TXPOWER_OVERRIDE:
            *(int *)buff = sc->sc_config.txpowlimit_override;
        break;

        case ATH_PARAM_PCIE_DISABLE_ASPM_WK:
            *(int *)buff = sc->sc_config.pcieDisableAspmOnRfWake;
        break;

        case ATH_PARAM_PCID_ASPM:
            *(int *)buff = sc->sc_config.pcieAspm;
        break;

        case ATH_PARAM_BEACON_NORESET:
            *(int *)buff = sc->sc_noreset;
        break;

        case ATH_PARAM_CAB_CONFIG:
            *(int *)buff = sc->sc_config.cabqReadytime;
        break;

        case ATH_PARAM_ATH_DEBUG:
            *(int *)buff = sc->sc_debug;
        break;

        case ATH_PARAM_ATH_TPSCALE:
           *(int *) buff = sc->sc_config.tpscale;
        break;

        case ATH_PARAM_ACKTIMEOUT:
           *(int *) buff = ath_hal_getacktimeout(sc->sc_ah) /* usec */;
        break;

#ifdef ATH_RB
        case ATH_PARAM_RX_RB:
            *(int *)buff = (sc->sc_rxrifs & ATH_RB_MODE_FORCE) ? 1 : 0;
        break;

        case ATH_PARAM_RX_RB_DETECT:
            *(int *)buff = (sc->sc_rxrifs & ATH_RB_MODE_DETECT) ? 1 : 0;
        break;

        case ATH_PARAM_RX_RB_TIMEOUT:
            *(int *)buff = sc->sc_rxrifs_timeout;
        break;

        case ATH_PARAM_RX_RB_SKIPTHRESH:
            *(int *)buff = sc->sc_rxrifs_skipthresh;
        break;
#endif
#ifdef ATH_SUPPORT_IQUE
		case ATH_PARAM_RETRY_DURATION:
			*(int *)buff = sc->sc_retry_duration;
		break;

		case ATH_PARAM_HBR_HIGHPER:
			*(int *)buff = sc->sc_hbr_per_high;
		break;
		
		case ATH_PARAM_HBR_LOWPER:
			*(int *)buff = sc->sc_hbr_per_low;
		break;
#endif
        case ATH_PARAM_RX_STBC:
            *(int *)buff = sc->sc_rxstbcsupport;
        break;

        case ATH_PARAM_TX_STBC:
            *(int *)buff = sc->sc_txstbcsupport;
        break;

        case ATH_PARAM_LIMIT_LEGACY_FRM:
            *(int *)buff = sc->sc_limit_legacy_frames;
        break;

        case ATH_PARAM_TOGGLE_IMMUNITY:
            *(int *)buff = sc->sc_toggle_immunity;
        break;

        case ATH_PARAM_WEP_TKIP_AGGR_TX_DELIM:
            (void)ath_hal_gettxdelimweptkipaggr(sc->sc_ah, (u_int32_t *)buff);
            break;

        case ATH_PARAM_WEP_TKIP_AGGR_RX_DELIM:
            (void)ath_hal_getrxdelimweptkipaggr(sc->sc_ah, (u_int32_t *)buff);
            break;

	case ATH_PARAM_GPIO_LED_CUSTOM:
            *(int *)buff = sc->sc_reg_parm.gpioLedCustom;
            break;
 
        case ATH_PARAM_SWAP_DEFAULT_LED:
            *(int *)buff = sc->sc_reg_parm.swapDefaultLED;
            break;

        case ATH_PARAM_USE_EAP_LOWEST_RATE:
            *(int *)buff = sc->sc_eap_lowest_rate;
        break;

#ifdef AP_SLOW_ANT_DIV
        case ATH_PARAM_SLOWANTDIV_RSSITHRHIGH:
            *(int *)buff = sc->sc_antdiv.rssiThrHigh;
        break;

        case ATH_PARAM_SLOWANTDIV_RSSITHRLOW:
            *(int *)buff = sc->sc_antdiv.rssiThrLow;
        break;

        case ATH_PARAM_SLOWANTDIV_BYTESTHRRX:
            *(int *)buff = sc->sc_antdiv.bytesThrRx;
        break;

        case ATH_PARAM_SLOWANTDIV_DWELL_TIME:
            *(int *)buff = sc->sc_antdiv.antdiv_min_dwell_time / 1000;
        break;

        case ATH_PARAM_SLOWANTDIV_SETTLE_TIME:
            *(int *)buff = sc->sc_antdiv.antdiv_settle_time / 1000;
        break;

        case ATH_PARAM_SLOWANTDIV_ENABLE:
            *(int *)buff = sc->sc_antdiv.enabled;
        break;
#endif

        default:
            return (-1);
    }

    return (0);
}
