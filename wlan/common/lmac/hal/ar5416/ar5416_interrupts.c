/*
 * Copyright (c) 2002-2005 Sam Leffler, Errno Consulting
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

#ifdef AH_SUPPORT_AR5416

#include "ah.h"
#include "ah_internal.h"

#include "ar5416/ar5416.h"
#include "ar5416/ar5416reg.h"
#include "ar5416/ar5416phy.h"


/*
 * Checks to see if an interrupt is pending on our NIC
 *
 * Returns: TRUE    if an interrupt is pending
 *          FALSE   if not
 */
HAL_BOOL
ar5416IsInterruptPending(struct ath_hal *ah)
{
#ifndef AR9100
    /*
     * Some platforms trigger our ISR before applying power to
     * the card, so make sure.
     */
    u_int32_t host_isr = OS_REG_READ(ah, AR_INTR_ASYNC_CAUSE);
    if ((host_isr & AR_INTR_ASYNC_USED) && (host_isr != AR_INTR_SPURIOUS))
        return AH_TRUE;

    host_isr = OS_REG_READ(ah, AR_INTR_SYNC_CAUSE);
    if ((host_isr & (AR_INTR_SYNC_DEFAULT | AR_INTR_SYNC_MASK_GPIO)) && (host_isr != AR_INTR_SPURIOUS))
        return AH_TRUE;

    return AH_FALSE;
#else
    return AH_TRUE;
#endif
}

/*
 * Reads the Interrupt Status Register value from the NIC, thus deasserting
 * the interrupt line, and returns both the masked and unmasked mapped ISR
 * values.  The value returned is mapped to abstract the hw-specific bit
 * locations in the Interrupt Status Register.
 *
 * Returns: A hardware-abstracted bitmap of all non-masked-out
 *          interrupts pending, as well as an unmasked value
 */
HAL_BOOL
ar5416GetPendingInterrupts(struct ath_hal *ah, HAL_INT *masked)
{
    u_int32_t isr = 0;
    u_int32_t mask2=0;
    HAL_CAPABILITIES *pCap = &AH_PRIVATE(ah)->ah_caps;
#ifndef AR9100
    HAL_BOOL fatal_int = AH_FALSE;
    u_int32_t sync_cause = 0;
    u_int32_t async_cause;

    /* Make sure mac interrupt is pending in async interrupt cause register */
    async_cause = OS_REG_READ(ah, AR_INTR_ASYNC_CAUSE);
    if (async_cause & AR_INTR_ASYNC_USED) {
        /* RTC may not be on since it runs on a slow 32khz clock so check its status to be sure */
        if ((OS_REG_READ(ah, AR_RTC_STATUS) & AR_RTC_STATUS_M) == AR_RTC_STATUS_ON) {
            isr = OS_REG_READ(ah, AR_ISR);
        }
    }
    
    sync_cause = OS_REG_READ(ah, AR_INTR_SYNC_CAUSE) & (AR_INTR_SYNC_DEFAULT | AR_INTR_SYNC_MASK_GPIO);

    *masked = 0;

    if (!isr && !sync_cause)
        return AH_FALSE;

#else
    *masked = 0;
    isr = OS_REG_READ(ah, AR_ISR);
#endif

    if (isr) {
        struct ath_hal_5416 *ahp = AH5416(ah);

        if (isr & AR_ISR_BCNMISC) {
            u_int32_t isr2;
            isr2 = OS_REG_READ(ah, AR_ISR_S2);
            if (isr2 & AR_ISR_S2_TIM)
                mask2 |= HAL_INT_TIM;
            if (isr2 & AR_ISR_S2_DTIM)
                mask2 |= HAL_INT_DTIM;
            if (isr2 & AR_ISR_S2_DTIMSYNC)
                mask2 |= HAL_INT_DTIMSYNC;
            if (isr2 & (AR_ISR_S2_CABEND ))
                mask2 |= HAL_INT_CABEND;
            if (isr2 & AR_ISR_S2_GTT)
                mask2 |= HAL_INT_GTT;
            if (isr2 & AR_ISR_S2_CST)
                mask2 |= HAL_INT_CST;
            if (isr2 & AR_ISR_S2_TSFOOR)
                mask2 |= HAL_INT_TSFOOR;

            if (!pCap->halIsrRacSupport) {
                /*
                 * EV61133 (missing interrupts due to ISR_RAC):
                 * If not using ISR_RAC, clear interrupts by writing to ISR_S2.
                 * This avoids a race condition where a new BCNMISC interrupt
                 * could come in between reading the ISR and clearing the interrupt
                 * via the primary ISR.  We therefore clear the interrupt via
                 * the secondary, which avoids this race.
                 */ 
                OS_REG_WRITE(ah, AR_ISR_S2, isr2);
                isr &= ~AR_ISR_BCNMISC;
            }
        }

        /* Use AR_ISR_RAC only if chip supports it. 
         * See EV61133 (missing interrupts due to ISR_RAC) 
         */
        if (pCap->halIsrRacSupport) {
            isr = OS_REG_READ(ah, AR_ISR_RAC);
        }
        if (isr == 0xffffffff) {
            *masked = 0;
            return AH_FALSE;
        }

        *masked = isr & HAL_INT_COMMON;

        /*
         * When interrupt mitigation is switched on, we fake a normal RX or TX
         * interrupt when we received a mitigated interrupt. This way, the upper
         * layer do not need to know about feature.
         */
        if (ahp->ah_intrMitigationRx) {
            if (isr & (AR_ISR_RXMINTR | AR_ISR_RXINTM)) {
                *masked |= HAL_INT_RX;
            }
        }
        if (ahp->ah_intrMitigationTx) {
            if (isr & (AR_ISR_TXMINTR | AR_ISR_TXINTM)) {
                *masked |= HAL_INT_TX;
            }
        }

        if (!ahp->ah_intrMitigationRx) {
            if (isr & (AR_ISR_RXOK | AR_ISR_RXERR))
                *masked |= HAL_INT_RX;
        }
        if (isr & (AR_ISR_TXOK | AR_ISR_TXDESC | AR_ISR_TXERR | AR_ISR_TXEOL)) {
            u_int32_t           s0, s1;

            if (ahp->ah_intrMitigationTx) {
                if (isr & (AR_ISR_TXERR | AR_ISR_TXEOL))
                    *masked |= HAL_INT_TX;
            } else {
                *masked |= HAL_INT_TX;
            }
            if (pCap->halIsrRacSupport) {
                /* Use secondary shadow registers if using ISR_RAC */
                s0 = OS_REG_READ(ah, AR_ISR_S0_S);
                s1 = OS_REG_READ(ah, AR_ISR_S1_S);
            } else {
                /*
                 * EV61133 (missing interrupts due to ISR_RAC):
                 * If not using ISR_RAC, clear interrupts by writing to ISR_S0/S1.
                 * This avoids a race condition where a new interrupt
                 * could come in between reading the ISR and clearing the interrupt
                 * via the primary ISR.  We therefore clear the interrupt via
                 * the secondary, which avoids this race.
                 */ 
                s0 = OS_REG_READ(ah, AR_ISR_S0);
                OS_REG_WRITE(ah, AR_ISR_S0, s0);
                s1 = OS_REG_READ(ah, AR_ISR_S1);
                OS_REG_WRITE(ah, AR_ISR_S1, s1);

                isr &= ~(AR_ISR_TXOK | AR_ISR_TXDESC | AR_ISR_TXERR | AR_ISR_TXEOL);
            }

            ahp->ah_intrTxqs |= MS(s0, AR_ISR_S0_QCU_TXOK);
            ahp->ah_intrTxqs |= MS(s0, AR_ISR_S0_QCU_TXDESC);
            ahp->ah_intrTxqs |= MS(s1, AR_ISR_S1_QCU_TXERR);
            ahp->ah_intrTxqs |= MS(s1, AR_ISR_S1_QCU_TXEOL);
        }

        /*
         * Do not treat receive overflows as fatal for owl.
         */
        if (isr & AR_ISR_RXORN) {
            HDPRINTF(ah, HAL_DBG_INTERRUPT, "%s: receive FIFO overrun interrupt\n", __func__);
            // *masked |= HAL_INT_FATAL;
        }

#ifndef AR9100
        if ((isr & AR_ISR_GENTMR) || (!pCap->halAutoSleepSupport)) {
            u_int32_t s5;

            if (pCap->halIsrRacSupport) {
                /* Use secondary shadow registers if using ISR_RAC */
                s5 = OS_REG_READ(ah, AR_ISR_S5_S);
            } else {
                s5 = OS_REG_READ(ah, AR_ISR_S5);
            }
            if (isr & AR_ISR_GENTMR) {

                HDPRINTF(ah, HAL_DBG_INTERRUPT,
                    "%s: GENTIMER, ISR_RAC=0x%x ISR_S2_S=0x%x\n", __func__,
                    isr, s5);
                ahp->ah_intrGenTimerTrigger = MS(s5, AR_ISR_S5_GENTIMER_TRIG);
                ahp->ah_intrGenTimerThresh = MS(s5, AR_ISR_S5_GENTIMER_THRESH);
                if (ahp->ah_intrGenTimerTrigger) {
                    *masked |= HAL_INT_GENTIMER;
                }
            }
            if (! pCap->halAutoSleepSupport) {
                if (s5 & AR_ISR_S5_TIM_TIMER) {
                    *masked |= HAL_INT_TIM_TIMER;
                }
            }
            if (!pCap->halIsrRacSupport) {
                /*
                 * EV61133 (missing interrupts due to ISR_RAC):
                 * If not using ISR_RAC, clear interrupts by writing to ISR_S5.
                 * This avoids a race condition where a new interrupt
                 * could come in between reading the ISR and clearing the interrupt
                 * via the primary ISR.  We therefore clear the interrupt via
                 * the secondary, which avoids this race.
                 */ 
                OS_REG_WRITE(ah, AR_ISR_S5, s5);
                isr &= ~AR_ISR_GENTMR;
            }

        }
#endif

        *masked |= mask2;

        if (!pCap->halIsrRacSupport) {
            /*
             * EV61133 (missing interrupts due to ISR_RAC):
             * If not using ISR_RAC, clear the interrupts we've read by writing back ones 
             * in these locations to the primary ISR (except for interrupts that
             * have a secondary isr register - see above).
             */ 
            OS_REG_WRITE(ah, AR_ISR, isr);

            /* Flush prior write */
            (void) OS_REG_READ(ah, AR_ISR);
        }
    }

#ifndef AR9100
    if (async_cause & AR_INTR_ASYNC_CAUSE_GPIO)
        *masked |= HAL_INT_GPIO;

    if (sync_cause) {
        fatal_int = (sync_cause & (AR_INTR_SYNC_HOST1_FATAL | AR_INTR_SYNC_HOST1_PERR)) ?
                    AH_TRUE : AH_FALSE;

        if (AH_TRUE == fatal_int) {
            if (sync_cause & AR_INTR_SYNC_HOST1_FATAL) {
                HDPRINTF(ah, HAL_DBG_UNMASKABLE, "%s: received PCI FATAL interrupt\n", __func__);
            }
            if (sync_cause & AR_INTR_SYNC_HOST1_PERR) {
                HDPRINTF(ah, HAL_DBG_UNMASKABLE, "%s: received PCI PERR interrupt\n", __func__);
            }

            /* Can mark this as fatal when upper layer does resets for
             * fatal interrupts. Otherwise, this can be uncommented to aid
             * in debugging PCI errors.
             */
            // *masked |= HAL_INT_FATAL;
        }
        if (sync_cause & AR_INTR_SYNC_RADM_CPL_TIMEOUT) {
            HDPRINTF(ah, HAL_DBG_INTERRUPT, 
                "%s: AR_INTR_SYNC_RADM_CPL_TIMEOUT\n",
                __func__);
            
            OS_REG_WRITE(ah, AR_RC, AR_RC_HOSTIF);
            OS_REG_WRITE(ah, AR_RC, 0);
            *masked |= HAL_INT_FATAL;
        }
        if (sync_cause & AR_INTR_SYNC_LOCAL_TIMEOUT) {
            HDPRINTF(ah, HAL_DBG_INTERRUPT, 
                "%s: AR_INTR_SYNC_LOCAL_TIMEOUT\n",
                __func__);
        }

        if (sync_cause & AR_INTR_SYNC_MASK_GPIO) {
            *masked |= HAL_INT_GPIO;
            HDPRINTF(ah, HAL_DBG_INTERRUPT, "%s: AR_INTR_SYNC_GPIO\n", __func__);
        }

        OS_REG_WRITE(ah, AR_INTR_SYNC_CAUSE_CLR, sync_cause);
        // Flush prior write
        (void) OS_REG_READ(ah, AR_INTR_SYNC_CAUSE_CLR);
    }
#endif

    return AH_TRUE;
}

HAL_INT
ar5416GetInterrupts(struct ath_hal *ah)
{
    return AH5416(ah)->ah_maskReg;
}

/*
 * Atomically enables NIC interrupts.  Interrupts are passed in
 * via the enumerated bitmask in ints.
 */
HAL_INT
ar5416SetInterrupts(struct ath_hal *ah, HAL_INT ints)
{
    struct ath_hal_5416 *ahp = AH5416(ah);
    u_int32_t omask = ahp->ah_maskReg;
    u_int32_t mask, mask2;
    HAL_CAPABILITIES *pCap = &AH_PRIVATE(ah)->ah_caps;

    HDPRINTF(ah, HAL_DBG_INTERRUPT, "%s: 0x%x => 0x%x\n", __func__, omask, ints);

    if (omask & HAL_INT_GLOBAL) {
        HDPRINTF(ah, HAL_DBG_INTERRUPT, "%s: disable IER\n", __func__);
        OS_REG_WRITE(ah, AR_IER, AR_IER_DISABLE);
        (void) OS_REG_READ(ah, AR_IER);   /* flush write to HW */      
#ifndef AR9100
        OS_REG_WRITE(ah, AR_INTR_ASYNC_ENABLE, 0);
        (void) OS_REG_READ(ah, AR_INTR_ASYNC_ENABLE); /* flush write to HW */

        OS_REG_WRITE(ah, AR_INTR_SYNC_ENABLE, 0);
        (void) OS_REG_READ(ah, AR_INTR_SYNC_ENABLE); /* flush write to HW */

#endif
    }

    mask = (ints & HAL_INT_COMMON) & ~HAL_INT_GPIO; /* bit 24 is reserved */;
    mask2 = 0;
    
    if (ints & HAL_INT_TX) {
        if (ahp->ah_intrMitigationTx) {
            mask |= AR_IMR_TXMINTR | AR_IMR_TXINTM;
        } else {
            if (ahp->ah_txOkInterruptMask)
                mask |= AR_IMR_TXOK;
            if (ahp->ah_txDescInterruptMask)
                mask |= AR_IMR_TXDESC;
        }
        if (ahp->ah_txErrInterruptMask)
            mask |= AR_IMR_TXERR;
        if (ahp->ah_txEolInterruptMask)
            mask |= AR_IMR_TXEOL;
    }
    if (ints & HAL_INT_RX) {
        mask |= AR_IMR_RXERR;
        if (ahp->ah_intrMitigationRx) {
            /* Only Rx interrupt mitigation. No Tx intr. mitigation. */
            mask |=  AR_IMR_RXMINTR | AR_IMR_RXINTM;
        } else {
            mask |= AR_IMR_RXOK | AR_IMR_RXDESC;
        }
        if (!pCap->halAutoSleepSupport) {
            mask |= AR_IMR_GENTMR;
        }
    }

    if (ints & (HAL_INT_BMISC)) {
        mask |= AR_IMR_BCNMISC;
        if (ints & HAL_INT_TIM)
            mask2 |= AR_IMR_S2_TIM;
        if (ints & HAL_INT_DTIM)
            mask2 |= AR_IMR_S2_DTIM;
        if (ints & HAL_INT_DTIMSYNC)
            mask2 |= AR_IMR_S2_DTIMSYNC;
        if (ints & HAL_INT_CABEND)
            mask2 |= (AR_IMR_S2_CABEND );
        if (ints & HAL_INT_TSFOOR)
            mask2 |= AR_IMR_S2_TSFOOR;
    }
    
    if (ints & (HAL_INT_GTT | HAL_INT_CST)) {
        mask |= AR_IMR_BCNMISC;
        if (ints & HAL_INT_GTT)
            mask2 |= AR_IMR_S2_GTT;
        if (ints & HAL_INT_CST)
            mask2 |= AR_IMR_S2_CST;
    }
    mask2 |= AR_IMR_S2_TSFOOR;

    /* Write the new IMR and store off our SW copy. */
    HDPRINTF(ah, HAL_DBG_INTERRUPT, "%s: new IMR 0x%x\n", __func__, mask);
    OS_REG_WRITE(ah, AR_IMR, mask);
    mask = OS_REG_READ(ah, AR_IMR_S2) & ~(AR_IMR_S2_TIM |
                    AR_IMR_S2_DTIM |
                    AR_IMR_S2_DTIMSYNC |
                    AR_IMR_S2_CABEND |
                    AR_IMR_S2_CABTO  |
                    AR_IMR_S2_TSFOOR |
                    AR_IMR_S2_GTT |
                    AR_IMR_S2_CST );
    OS_REG_WRITE(ah, AR_IMR_S2, mask | mask2 );
    ahp->ah_maskReg = ints;

    if (! pCap->halAutoSleepSupport) {
        if (ints & HAL_INT_TIM_TIMER) {
            OS_REG_SET_BIT(ah, AR_IMR_S5, AR_IMR_S5_TIM_TIMER);
        }
        else {
            OS_REG_CLR_BIT(ah, AR_IMR_S5, AR_IMR_S5_TIM_TIMER);
        }
    }

    /* Re-enable interrupts if they were enabled before. */
    if (ints & HAL_INT_GLOBAL) {
        HDPRINTF(ah, HAL_DBG_INTERRUPT, "%s: enable IER\n", __func__);
        OS_REG_WRITE(ah, AR_IER, AR_IER_ENABLE);
#ifndef AR9100
        mask = AR_INTR_MAC_IRQ;
        if (ints & HAL_INT_GPIO)
            mask |= SM(ahp->ah_gpioMask, AR_INTR_ASYNC_MASK_GPIO);
        OS_REG_WRITE(ah, AR_INTR_ASYNC_ENABLE, mask);
        OS_REG_WRITE(ah, AR_INTR_ASYNC_MASK, mask);

        /*
         * debug - enable to see all synchronous interrupts status
         * Enable synchronous GPIO interrupts as well, since some async GPIO interrupts
         * don't wake the chip up.
         */
        mask = 0;
        if (ints & HAL_INT_GPIO)
            mask |= SM(ahp->ah_gpioMask, AR_INTR_SYNC_MASK_GPIO);
        OS_REG_WRITE(ah, AR_INTR_SYNC_ENABLE, (AR_INTR_SYNC_DEFAULT | mask));
        OS_REG_WRITE(ah, AR_INTR_SYNC_MASK, (AR_INTR_SYNC_DEFAULT | mask));

#endif
        HDPRINTF(ah,  HAL_DBG_INTERRUPT, "AR_IMR 0x%x IER 0x%x\n", OS_REG_READ(ah, AR_IMR), OS_REG_READ(ah, AR_IER));
    }

    return omask;
}

void ar5416SetIntrMitigationTimer(
    struct ath_hal* ah, HAL_INT_MITIGATION reg, u_int32_t value)
{
#ifdef AR5416_INT_MITIGATION
    struct ath_hal_private *ahpriv = AH_PRIVATE(ah);
    HAL_CAPABILITIES *pCap = &ahpriv->ah_caps; 

    if (pCap->halintr_mitigation == AH_FALSE)
        return;

    switch(reg) {
        case HAL_INT_THRESHOLD:
            OS_REG_WRITE(ah, AR_MIRT, 0);
            break;
        case HAL_INT_RX_LASTPKT:
            OS_REG_RMW_FIELD(ah, AR_RIMT, AR_RIMT_LAST, value);
            break;
        case HAL_INT_RX_FIRSTPKT:
            OS_REG_RMW_FIELD(ah, AR_RIMT, AR_RIMT_FIRST, value);
            break;
        case HAL_INT_TX_LASTPKT:
            OS_REG_RMW_FIELD(ah, AR_TIMT, AR_TIMT_LAST, value);
            break;
        case HAL_INT_TX_FIRSTPKT:
            OS_REG_RMW_FIELD(ah, AR_TIMT, AR_TIMT_FIRST, value);
            break;
        default:
            break;
    }
#endif
}

u_int32_t ar5416GetIntrMitigationTimer(
    struct ath_hal* ah, HAL_INT_MITIGATION reg)
{
    u_int32_t val = 0;
#ifdef AR5416_INT_MITIGATION
    switch(reg) {
        case HAL_INT_THRESHOLD:
            val = OS_REG_READ(ah, AR_MIRT);
            break;
        case HAL_INT_RX_LASTPKT:
            val = OS_REG_READ(ah, AR_RIMT) & 0xFFFF;
            break;
        case HAL_INT_RX_FIRSTPKT:
            val = OS_REG_READ(ah, AR_RIMT) >> 16;
            break;
        case HAL_INT_TX_LASTPKT:
            val = OS_REG_READ(ah, AR_TIMT) & 0xFFFF;
            break;
        case HAL_INT_TX_FIRSTPKT:
            val = OS_REG_READ(ah, AR_TIMT) >> 16;
            break;
        default:
            break;
    }
#endif
    return val;
}

#endif /* AH_SUPPORT_AR5416 */

