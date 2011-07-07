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
 */

#include "ath_internal.h"
#include "ath_bt.h"

#ifdef ATH_BT_COEX

/*
 * static settings for different BT modules
 */
static const HAL_BT_COEX_CONFIG ath_bt_configs[HAL_MAX_BT_MODULES] = {
/* TIME_EXT     TXSTATE_EXT        TXFRM_EXT   MODE
 * QUIET        RXCLEAR_POLARITY   PRI_TIME    FST_SLOTTIME   HLD_RXCLEAR */

    /* HAL_BT_MODULE_CSR_V4 */
    {0,         AH_TRUE,           AH_TRUE,    HAL_BT_COEX_MODE_LEGACY,
     AH_FALSE,  AH_TRUE,           5,          29,            AH_TRUE},
};

static void
ath_bt_coex_enable_prot_mode(struct ath_softc *sc, u_int8_t chainmask)
{
    struct ath_bt_info *btinfo = &sc->sc_btinfo;
    struct ath_hal *ah = sc->sc_ah;

    /* Disable GPIO interrupt */
    if (btinfo->bt_gpioIntEnabled) {
        ath_hal_gpioSetIntr(ah, btinfo->bt_gpioSelect, HAL_GPIO_INTR_DISABLE);
        btinfo->bt_gpioIntEnabled = AH_FALSE;
    }

    /* Update chain mask */
    sc->sc_config.txchainmasklegacy = sc->sc_config.txchainmask = chainmask;
    sc->sc_config.rxchainmasklegacy = sc->sc_config.rxchainmask = chainmask;
    sc->sc_tx_chainmask = sc->sc_config.txchainmask;
    sc->sc_rx_chainmask = sc->sc_config.rxchainmask;
    ath_internal_reset(sc);
    sc->sc_ieee_ops->rate_newstate(sc->sc_ieee, sc->sc_vaps[0]->av_if_data);
}

static int
ath_bt_coex_active_check(struct ath_softc *sc)
{
    struct ath_bt_info *btinfo = &sc->sc_btinfo;
    struct ath_hal *ah = sc->sc_ah;

    if (!btinfo->bt_initStateDone) {
        btinfo->bt_initStateTime++;
        if (btinfo->bt_protectMode == ATH_BT_PROT_MODE_NONE) {
            if (btinfo->bt_initStateTime >= BT_COEX_INIT_STATE_TIME) {
                btinfo->bt_initStateDone = AH_TRUE;
            }
            if (btinfo->bt_activeCount >= BT_COEX_BT_ACTIVE_THRESHOLD) {
                btinfo->bt_timeOverThre++;
            }
            else {
                btinfo->bt_timeOverThre = 0;
            }
            if (btinfo->bt_activeCount >= BT_COEX_BT_ACTIVE_THRESHOLD2) {
                btinfo->bt_timeOverThre2++;
            }
            else {
                btinfo->bt_timeOverThre2 = 0;
            }
            if (btinfo->bt_timeOverThre >= BT_COEX_PROT_MODE_ON_TIME) {
                btinfo->bt_protectMode = ATH_BT_PROT_MODE_DEFER;
            }
        }
        else if (btinfo->bt_protectMode == ATH_BT_PROT_MODE_DEFER) {
            if (btinfo->bt_activeCount >= BT_COEX_BT_ACTIVE_THRESHOLD2) {
                btinfo->bt_timeOverThre2++;
            }
            else {
                if (btinfo->bt_timeOverThre2 >= Bt_COEX_INIT_STATE_SCAN_TIME) {
                    /* 
                     * BT_ACTIVE count is over threshold2 for too long. No BT device 
                     * is found. Go back to no protection mode.
                     */
                    btinfo->bt_protectMode = ATH_BT_PROT_MODE_NONE;
                }
                else {
                    /* BT device pairing. Turn on protection mode. */
                    btinfo->bt_protectMode = ATH_BT_PROT_MODE_ON;

                    ath_bt_coex_enable_prot_mode(sc, btinfo->bt_activeChainMask);
                }

                btinfo->bt_initStateDone = AH_TRUE;
                btinfo->bt_timeOverThre = 0;
                btinfo->bt_timeOverThre2 = 0;
            }
        }
    }
    else if ((btinfo->bt_protectMode == ATH_BT_PROT_MODE_NONE) && 
        (btinfo->bt_activeCount > BT_COEX_BT_ACTIVE_THRESHOLD)) {
        btinfo->bt_timeOverThre++;
        if (btinfo->bt_timeOverThre >= BT_COEX_PROT_MODE_ON_TIME) {
            btinfo->bt_timeOverThre = 0;
            btinfo->bt_protectMode = ATH_BT_PROT_MODE_ON;

            ath_bt_coex_enable_prot_mode(sc, btinfo->bt_activeChainMask);
        }
    }
    else {
        btinfo->bt_timeOverThre = 0;
    }

    btinfo->bt_activeCount = 0;
    return 0;
}

/*
 * Primitives to disable BT COEX in hardware.
 * BT and WLAN traffic will go out independently and would be prune to interference.
 */
static INLINE void
ath_bt_disable_coex(struct ath_softc *sc)
{
    ath_hal_bt_disable_coex(sc->sc_ah);
}

/*
 * Primitives to enable BT COEX in hardware.
 * Hardware will prioritize BT and WLAN traffic based on the programmed weights.
 */
static INLINE void
ath_bt_enable_coex(struct ath_softc *sc, struct ath_bt_info *btinfo)
{
    struct ath_hal *ah = sc->sc_ah;
    
    /* set BT/WLAN weights */
    ath_hal_bt_setweights(ah, btinfo->bt_weightBT, btinfo->bt_weightWlan);
    ath_hal_bt_enable_coex(ah);
}

int
ath_bt_coex_attach(struct ath_softc *sc, u_int8_t cfg)
{
    struct ath_hal *ah = sc->sc_ah;
    struct ath_bt_info *btinfo = &sc->sc_btinfo;
    HAL_BT_COEX_INFO hwinfo;

    OS_MEMZERO(btinfo, sizeof(struct ath_bt_info));   

    if ((cfg == ATH_BT_COEX_CFG_2WIRE_CH0)  || (cfg == ATH_BT_COEX_CFG_2WIRE_CH1) ||
        (cfg == ATH_BT_COEX_CFG_2WIRE_2CH)) {
        btinfo->bt_coex_enabled = AH_TRUE;
        btinfo->bt_on = AH_FALSE;
        btinfo->bt_state = ATH_BT_COEX_STATE_OFF;
        btinfo->bt_bmissThresh = IEEE80211_BMISS_LIMIT/2; /* default */
        ATH_BT_LOCK_INIT(btinfo);

        /* Get board information */
        ath_hal_bt_getinfo(ah, &hwinfo);
        btinfo->bt_gpioSelect = hwinfo.bt_gpioBTActive;
        btinfo->bt_activePolarity = hwinfo.bt_activePolarity;

        if (cfg == ATH_BT_COEX_CFG_2WIRE_CH0) {
            /* Use chain 0 when BT_ACTIVE is detected */
            btinfo->bt_activeChainMask = 1;
        }
        else if (cfg == ATH_BT_COEX_CFG_2WIRE_CH1) {
            /* Use chain 1 when BT_ACTIVE is detected */
            btinfo->bt_activeChainMask = 2;
        }
        else if (cfg == ATH_BT_COEX_CFG_2WIRE_2CH) {
            /* Use two chains when BT_ACTIVE is detected */
            btinfo->bt_activeChainMask = 3;
        }
        /* find/program the correct BT config based on the BT module on board */
        /*
         * For 3-wire
         * ASSERT(hwinfo.bt_module < HAL_MAX_BT_MODULES);
         * ath_hal_bt_config(ah, &ath_bt_configs[hwinfo.bt_module]);
         */

        ath_hal_bt_enable_coex(ah);

        /* Start BT_ACTIVE monitoring */
        btinfo->bt_gpioIntEnabled = AH_TRUE;
        btinfo->bt_preGpioState = btinfo->bt_activePolarity;
        ath_hal_gpioSetIntr(ah, btinfo->bt_gpioSelect, btinfo->bt_activePolarity);
        ath_initialize_timer(sc->sc_osdev, &btinfo->bt_activeTimer, 
                             BT_COEX_BT_ACTIVE_MEASURE, 
                             (timer_handler_function)ath_bt_coex_active_check,
                             sc);
        ath_start_timer(&btinfo->bt_activeTimer);
    }

    return 0;
}

void ath_bt_coex_gpio_intr(struct ath_softc *sc)
{
    struct ath_hal *ah = sc->sc_ah;
    struct ath_bt_info *btinfo = &sc->sc_btinfo;
    u_int32_t   value;

    value = ath_hal_gpioget(ah, btinfo->bt_gpioSelect);
    if (value == btinfo->bt_preGpioState) {
        /* It's not our GPIO int */
        return;
    }

    if (value != btinfo->bt_activePolarity) {
        btinfo->bt_activeCount++;
    }
   
    btinfo->bt_preGpioState = value;
    ath_hal_gpioSetIntr(ah, btinfo->bt_gpioSelect, !value);
}

void
ath_bt_coex_detach(struct ath_softc *sc)
{
    struct ath_bt_info *btinfo = &sc->sc_btinfo;
    struct ath_hal *ah = sc->sc_ah;

    if (!btinfo->bt_coex_enabled) {
        return;
    }

    if (ath_timer_is_initialized(&btinfo->bt_activeTimer)) {
        ath_cancel_timer(&btinfo->bt_activeTimer, CANCEL_NO_SLEEP);
    }

    if (btinfo->bt_gpioIntEnabled) {
        ath_hal_gpioSetIntr(ah, btinfo->bt_gpioSelect, HAL_GPIO_INTR_DISABLE);
        btinfo->bt_gpioIntEnabled = AH_FALSE;
    }

    ATH_BT_LOCK_DESTROY(btinfo);
}

#endif /* ATH_BT_COEX */




