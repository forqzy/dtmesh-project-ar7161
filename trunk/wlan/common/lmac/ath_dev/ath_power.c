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
#include "ath_power.h"

/*
 * Notification of device suprise removal event
 */
void
ath_notify_device_removal(ath_dev_t dev)
{
    ATH_DEV_TO_SC(dev)->sc_invalid = 1;
    ATH_DEV_TO_SC(dev)->sc_removed = 1;
}

/*
 * Detect card present
 */
int
ath_detect_card_present(ath_dev_t dev)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;

    return ath_hal_detectcardpresent(ah);
}


/*
 * To query software PHY state
 */
int
ath_get_sw_phystate(ath_dev_t dev)
{
    return ATH_DEV_TO_SC(dev)->sc_sw_phystate;
}

/*
 * To query hardware PHY state
 */
int
ath_get_hw_phystate(ath_dev_t dev)
{
    return ATH_DEV_TO_SC(dev)->sc_hw_phystate;
}

/*
 * To set software PHY state
 * NB: this is just to set the software state. To turn on/off
 * radio, ath_radio_enable()/ath_radio_disable() has to be
 * explicitly called.
 */
void
ath_set_sw_phystate(ath_dev_t dev, int swstate)
{
    ATH_DEV_TO_SC(dev)->sc_sw_phystate = swstate;
}

/*
 * To disable PHY (radio off)
 */
int
ath_radio_disable(ath_dev_t dev)
{
	struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;
    HAL_STATUS status;
    HAL_HT_MACMODE ht_macmode = sc->sc_ieee_ops->cwm_macmode(sc->sc_ieee);

    if (sc->sc_invalid)
        return -EIO;

    ATH_PS_WAKEUP(sc);
    
    /*
     * notify LED module radio has been turned off
     * This function will access the hw, so we must call it before 
     * the power save function.
     */
    ath_led_disable(&sc->sc_led_control);

    ath_hal_intrset(ah, 0);     /* disable interrupts */

    ath_draintxq(sc, AH_FALSE); /* stop xmit side */
    ath_stoprecv(sc);           /* stop recv side */
    ath_flushrecv(sc);          /* flush recv queue */

    ATH_RESET_LOCK(sc);
    if (!ath_hal_reset(ah, sc->sc_opmode, &sc->sc_curchan,
                       ht_macmode,
                       sc->sc_tx_chainmask, sc->sc_rx_chainmask,
                       sc->sc_ht_extprotspacing, AH_FALSE, &status)) {
        printk("%s: unable to reset hardware; hal status %u\n",
               __func__, status);
    }
    ATH_RESET_UNLOCK(sc);

    ath_hal_phydisable(ah);

#ifdef ATH_RFKILL
    if (sc->sc_hasrfkill) {
        ath_hal_intrset(ah, HAL_INT_GLOBAL | HAL_INT_GPIO);    /* Enable interrupt to capture GPIO event */
    }
#endif
    
    /* Turn on PCIE ASPM during RF Silence */
    ath_pcie_pwrsave_enable_on_phystate_change(sc, 1);
    /*
     * XXX TODO: We should put chip to forced sleep when radio is disabled. 
     */

    ath_pwrsave_fullsleep(sc);
    ATH_PS_SLEEP(sc);

    return 0;
}

/*
 * To enable PHY (radio on)
 */
int
ath_radio_enable(ath_dev_t dev)
{
	struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_hal *ah = sc->sc_ah;
    HAL_STATUS status;
    HAL_HT_MACMODE ht_macmode = sc->sc_ieee_ops->cwm_macmode(sc->sc_ieee);

    if (sc->sc_invalid)
        return -EIO;
    
    ATH_PS_WAKEUP(sc);

    ath_pwrsave_awake(sc);

    /* Turn off PCIE ASPM when card is active */
    ath_pcie_pwrsave_enable_on_phystate_change(sc, 0);

    ATH_RESET_LOCK(sc);
    if (!ath_hal_reset(ah, sc->sc_opmode, &sc->sc_curchan,
                       ht_macmode,
                       sc->sc_tx_chainmask, sc->sc_rx_chainmask,
                       sc->sc_ht_extprotspacing, AH_FALSE, &status)) {
        printk("%s: unable to reset hardware; hal status %u\n",
               __func__, status);
    }
    ATH_RESET_UNLOCK(sc);

    ath_update_txpow(sc, 0);		/* update tx power state */
    if (ath_startrecv(sc) != 0)	{ /* restart recv */
        printk("%s: unable to start recv logic\n",
               __func__);
    }

    if (sc->sc_beacons) {
        ath_beacon_config(sc, ATH_IF_ID_ANY);   /* restart beacons */
    }
    ath_hal_intrset(ah, sc->sc_imask);

    /*
     * notify LED module radio has been turned on
     * This function will access the hw, so we must call it after 
     * the power save function.
     */
    ath_led_enable(&sc->sc_led_control);

    ATH_PS_SLEEP(sc);

    return 0;
}

/*
 * handles power save state transitions.
 */

int
ath_pwrsave_get_state(struct ath_softc *sc)
{
    return sc->sc_pwrsave.ps_pwrsave_state;
}

static void
ath_pwrsave_set_state_sync(struct ath_softc *sc)
{
    struct ath_hal *ah = sc->sc_ah;
    ATH_PWRSAVE_STATE newstate = (sc)->sc_pwrsave.ps_set_state;

    if(sc->sc_pwrsave.ps_pwrsave_state == newstate) 
        return;
    if (sc->sc_removed) 
        return;

    switch (sc->sc_pwrsave.ps_pwrsave_state) {

    case ATH_PWRSAVE_NETWORK_SLEEP:
        switch (newstate) {
        case ATH_PWRSAVE_AWAKE:
            ath_hal_setpower(ah, HAL_PM_AWAKE);

            /* 
             * Must clear RxAbort bit manually if hardware does not support 
             * automatic sleep after waking up for TIM.
             */
            if (! sc->sc_hasautosleep) {
                u_int32_t    imask;
            
                ath_hal_setrxabort(ah, 0);
                
                /* Disable TIM_TIMER interrupt */
                imask = ath_hal_intrget(ah);
                if (imask & HAL_INT_TIM_TIMER) {
                    sc->sc_imask &= ~HAL_INT_TIM_TIMER;
                    ath_hal_intrset(ah, imask & ~HAL_INT_TIM_TIMER);
                }
            }
            break;
        case ATH_PWRSAVE_FULL_SLEEP:
            ath_hal_setpower(ah, HAL_PM_FULL_SLEEP);
            break;

        default:
            break;
        }
        break;

    case ATH_PWRSAVE_AWAKE:
        switch (newstate) {
        case ATH_PWRSAVE_NETWORK_SLEEP:
            /* 
             * Chips that do not support automatic sleep after waking up to 
             * receive TIM must make sure at least one beacon is received 
             * before reentering network sleep.
             */
            if (! sc->sc_hasautosleep) {
                /* 
                 * Do not enter network sleep if no beacon received
                 */
                if (! sc->sc_waitbeacon) {
                    u_int32_t    imask;
                    
                    /* Enable TIM_TIMER interrupt */
                    imask = ath_hal_intrget(ah);
                    if ((imask & HAL_INT_TIM_TIMER) == 0) {
                        sc->sc_imask |= HAL_INT_TIM_TIMER;
                        ath_hal_intrset(ah, imask | HAL_INT_TIM_TIMER);
                    }
                    
                    /* Stop RX state machine */
                    if (ath_hal_setrxabort(ah, 1)) {
                        ath_hal_setpower(ah, HAL_PM_NETWORK_SLEEP);
                    }
                }
            }
            else {
                ath_hal_setpower(ah, HAL_PM_NETWORK_SLEEP);
            }
            break;
        case ATH_PWRSAVE_FULL_SLEEP:
            /* 
             * Must set RxAbort bit manually if hardware does not support 
             * automatic sleep after waking up for TIM.
             */
            if (! sc->sc_hasautosleep) {
                ath_hal_setrxabort(ah, 1);
            }
            ath_hal_setpower(ah, HAL_PM_FULL_SLEEP);
            break;

        default:
            break;
        }
        break;

    case ATH_PWRSAVE_FULL_SLEEP:
        switch (newstate) {
        case ATH_PWRSAVE_AWAKE:
            ath_hal_setpower(ah, HAL_PM_AWAKE);

            /* 
             * Must clear RxAbort bit manually if hardware does not support 
             * automatic sleep after waking up for TIM.
             */
            if (! sc->sc_hasautosleep) {
                u_int32_t    imask;
            
                ath_hal_setrxabort(ah, 0);
                
                /* Disable TIM_TIMER interrupt */
                imask = ath_hal_intrget(ah);
                if (imask & HAL_INT_TIM_TIMER) {
                    sc->sc_imask &= ~HAL_INT_TIM_TIMER;
                    ath_hal_intrset(ah, imask & ~HAL_INT_TIM_TIMER);
                }
            }
            break;
        default:
            break;
        }
    default:
        break;

    }
    sc->sc_pwrsave.ps_pwrsave_state = newstate;

    /* If chip has been put in full sleep, make sure full reset is called */
    if (sc->sc_pwrsave.ps_pwrsave_state == ATH_PWRSAVE_FULL_SLEEP)
        sc->sc_full_reset = 1;
}

void
ath_pwrsave_set_state(struct ath_softc *sc, ATH_PWRSAVE_STATE newstate)
{
    sc->sc_pwrsave.ps_set_state = newstate;
    OS_EXEC_INTSAFE(sc->sc_osdev, ath_pwrsave_set_state_sync, sc);
}

void    
ath_pwrsave_awake(ath_dev_t dev)
{
	struct ath_softc *sc = ATH_DEV_TO_SC(dev);

    ATH_PS_LOCK(sc);
    sc->sc_pwrsave.ps_restore_state = ATH_PWRSAVE_AWAKE;
    if (sc->sc_pwrsave.ps_pwrsave_state != ATH_PWRSAVE_AWAKE) { 
        ath_pwrsave_set_state(sc, ATH_PWRSAVE_AWAKE); 
    }
    ATH_PS_UNLOCK(sc);
}

void
ath_pwrsave_fullsleep(ath_dev_t dev)
{
	struct ath_softc *sc = ATH_DEV_TO_SC(dev);

    ATH_PS_LOCK(sc);
    /*
     * if nobody is using hal, then put it back to
     * sleep. 
     */
    if (sc->sc_pwrsave.ps_hal_usecount) 
        sc->sc_pwrsave.ps_restore_state = ATH_PWRSAVE_FULL_SLEEP;
    else {
        ath_pwrsave_set_state(sc, ATH_PWRSAVE_FULL_SLEEP);
    }
    ATH_PS_UNLOCK(sc);
}

void
ath_pwrsave_netsleep(ath_dev_t dev)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);

    ATH_PS_LOCK(sc);
    /*
     * if nobody is using hal, then put it back to
     * sleep. 
     */
    if (sc->sc_pwrsave.ps_hal_usecount) {
        if (sc->sc_pwrsave.ps_restore_state != ATH_PWRSAVE_FULL_SLEEP)
            sc->sc_pwrsave.ps_restore_state = ATH_PWRSAVE_NETWORK_SLEEP;
    } else {
        if (sc->sc_pwrsave.ps_pwrsave_state != ATH_PWRSAVE_FULL_SLEEP) {
            ath_pwrsave_set_state(sc, ATH_PWRSAVE_NETWORK_SLEEP);
        }
    }
    ATH_PS_UNLOCK(sc);
}

void
ath_pwrsave_proc_intdone(struct ath_softc *sc, u_int32_t intrStatus)
{
    ATH_PWRSAVE_STATE new_state;
    ATH_PS_LOCK(sc);

    new_state = sc->sc_pwrsave.ps_pwrsave_state;
    /* 
     * any interrupt atomatically wakens the HW. 
     * so push the state machine to AWAKE state
     * to keep the HW and SW in sync.
     */
    if (sc->sc_pwrsave.ps_hal_usecount) {
        sc->sc_pwrsave.ps_restore_state = new_state;
    } else {
        sc->sc_pwrsave.ps_pwrsave_state = ATH_PWRSAVE_AWAKE;
        ath_pwrsave_set_state(sc, new_state);
    }
            
    ATH_PS_UNLOCK(sc);
}

void
ath_pwrsave_init(struct ath_softc *sc)
{
    /* init the HW and SW state to awake */
    sc->sc_pwrsave.ps_hal_usecount  = 0;
    sc->sc_pwrsave.ps_restore_state = ATH_PWRSAVE_AWAKE;
    sc->sc_pwrsave.ps_pwrsave_state = ATH_PWRSAVE_AWAKE;
    ath_hal_setpower(sc->sc_ah, HAL_PM_AWAKE);
    /* 
     * Must clear RxAbort bit manually if hardware does not support 
     * automatic sleep after waking up for TIM.
     */
    if (! sc->sc_hasautosleep) {
        ath_hal_setrxabort(sc->sc_ah, 0);
    }
}

void
ath_pcie_pwrsave_enable_on_phystate_change(struct ath_softc *sc, int enable)
{
	if (!sc->sc_config.pcieDisableAspmOnRfWake)
		return;

	ath_pcie_pwrsave_enable(sc, enable);
}

void
ath_pcie_pwrsave_enable(struct ath_softc *sc, int enable)
{
    u_int8_t     byte   = 0;

    if (!ath_hal_isPciePwrsaveEnabled(sc->sc_ah) || !ath_hal_hasPciePwrsave(sc->sc_ah))
        return;
  
    OS_PCI_READ_CONFIG(sc->sc_osdev, PCIE_CAP_LINK_CTRL, &byte, 1);

    /* Clear the bits if they are set, then or in the desired bits */
    byte &= ~(PCIE_CAP_LINK_L0S | PCIE_CAP_LINK_L1);
    if (enable) {
        byte |= sc->sc_config.pcieAspm;
    }

    OS_PCI_WRITE_CONFIG(sc->sc_osdev, PCIE_CAP_LINK_CTRL, &byte, 1);
}
