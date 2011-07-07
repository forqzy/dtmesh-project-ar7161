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
#include "ah_desc.h"
#include "ah_internal.h"

#include "ar5416/ar5416.h"
#include "ar5416/ar5416reg.h"
#include "ar5416/ar5416desc.h"

/*
 * Get the RXDP.
 */
u_int32_t
ar5416GetRxDP(struct ath_hal *ath)
{
    return OS_REG_READ(ath, AR_RXDP);
}

/*
 * Set the RxDP.
 */
void
ar5416SetRxDP(struct ath_hal *ah, u_int32_t rxdp)
{
    OS_REG_WRITE(ah, AR_RXDP, rxdp);
    HALASSERT(OS_REG_READ(ah, AR_RXDP) == rxdp);
}

/*
 * Set Receive Enable bits.
 */
void
ar5416EnableReceive(struct ath_hal *ah)
{
    OS_REG_WRITE(ah, AR_CR, AR_CR_RXE);
}

/*
 * Set the RX abort bit.
 */
HAL_BOOL
ar5416SetRxAbort(struct ath_hal *ah, HAL_BOOL set)
{
    if (set) {
        /* Set the ForceRXAbort bit */
        OS_REG_SET_BIT(ah, AR_DIAG_SW, (AR_DIAG_RX_DIS | AR_DIAG_RX_ABORT));

        /* Wait for Rx state to return to 0 */
        if (!ath_hal_wait(ah, AR_OBS_BUS_1, AR_OBS_BUS_1_RX_STATE, 0, AH_WAIT_TIMEOUT)) {
            u_int32_t    reg;

            /* abort: chip rx failed to go idle in 10 ms */
            OS_REG_CLR_BIT(ah, AR_DIAG_SW, (AR_DIAG_RX_DIS | AR_DIAG_RX_ABORT));

            reg = OS_REG_READ(ah, AR_OBS_BUS_1);
            HDPRINTF(ah, HAL_DBG_RX, "%s: rx failed to go idle in 10 ms RXSM=0x%x\n",
                    __func__, reg);

            return AH_FALSE;    // Indicate failure
        }
    }
    else {
        OS_REG_CLR_BIT(ah, AR_DIAG_SW, (AR_DIAG_RX_DIS | AR_DIAG_RX_ABORT));
    }

    return AH_TRUE;    // Function completed successfully
}

/*
 * Stop Receive at the DMA engine
 */
HAL_BOOL
ar5416StopDmaReceive(struct ath_hal *ah)
{
    int wait;
    HAL_BOOL status;  

#define AH_RX_STOP_DMA_TIMEOUT 10000   /* usec */  
#define AH_TIME_QUANTUM        100     /* usec */

    /* Set receive disable bit */
    OS_REG_WRITE(ah, AR_CR, AR_CR_RXD); 

    /* Wait for rx enable bit to go low */
    for (wait = AH_RX_STOP_DMA_TIMEOUT / AH_TIME_QUANTUM; wait != 0; wait--) {

        if ((OS_REG_READ(ah, AR_CR) & AR_CR_RXE) == 0) {
            break;
        }
        OS_DELAY(AH_TIME_QUANTUM);        
    }

    if (wait == 0) {
        HDPRINTF(ah, HAL_DBG_RX, "%s: dma failed to stop in %d ms\n"
                "AR_CR=0x%08x\nAR_DIAG_SW=0x%08x\n",
                __func__,
                AH_RX_STOP_DMA_TIMEOUT / 1000,
                OS_REG_READ(ah, AR_CR),
                OS_REG_READ(ah, AR_DIAG_SW));
        status=AH_FALSE;
    } else {
        status=AH_TRUE;
    }

#ifdef AR9100
    OS_DELAY(3000);
#endif
    return status;
#undef AH_RX_STOP_DMA_TIMEOUT
#undef AH_TIME_QUANTUM 
}

/*
 * Start Transmit at the PCU engine (unpause receive)
 */
void
ar5416StartPcuReceive(struct ath_hal *ah)
{

    ar5416EnableMIBCounters(ah);

    ar5416AniReset(ah);

    /* Clear RX_DIS and RX_ABORT after enabling phy errors in aniReset */
    OS_REG_CLR_BIT(ah, AR_DIAG_SW, (AR_DIAG_RX_DIS | AR_DIAG_RX_ABORT));
}

/*
 * Stop Transmit at the PCU engine (pause receive)
 */
void
ar5416StopPcuReceive(struct ath_hal *ah)
{
    OS_REG_SET_BIT(ah, AR_DIAG_SW, AR_DIAG_RX_DIS);

    ar5416DisableMIBCounters(ah);

#if 0
    ar5416RadarDisable(ah);
#endif
}

/*
 * Abort current ingress frame at the PCU
 */
void
ar5416AbortPcuReceive(struct ath_hal *ah)
{
    OS_REG_SET_BIT(ah, AR_DIAG_SW, AR_DIAG_RX_ABORT | AR_DIAG_RX_DIS);

    ar5416DisableMIBCounters(ah);

#if 0
    ar5416RadarDisable(ah);
#endif
}

/*
 * Set multicast filter 0 (lower 32-bits)
 *               filter 1 (upper 32-bits)
 */
void
ar5416SetMulticastFilter(struct ath_hal *ah, u_int32_t filter0, u_int32_t filter1)
{
    OS_REG_WRITE(ah, AR_MCAST_FIL0, filter0);
    OS_REG_WRITE(ah, AR_MCAST_FIL1, filter1);
}

/*
 * Clear multicast filter by index
 */
HAL_BOOL
ar5416ClrMulticastFilterIndex(struct ath_hal *ah, u_int32_t ix)
{
    u_int32_t val;

    if (ix >= 64)
        return AH_FALSE;
    if (ix >= 32) {
        val = OS_REG_READ(ah, AR_MCAST_FIL1);
        OS_REG_WRITE(ah, AR_MCAST_FIL1, (val &~ (1<<(ix-32))));
    } else {
        val = OS_REG_READ(ah, AR_MCAST_FIL0);
        OS_REG_WRITE(ah, AR_MCAST_FIL0, (val &~ (1<<ix)));
    }
    return AH_TRUE;
}

/*
 * Set multicast filter by index
 */
HAL_BOOL
ar5416SetMulticastFilterIndex(struct ath_hal *ah, u_int32_t ix)
{
    u_int32_t val;

    if (ix >= 64)
        return AH_FALSE;
    if (ix >= 32) {
        val = OS_REG_READ(ah, AR_MCAST_FIL1);
        OS_REG_WRITE(ah, AR_MCAST_FIL1, (val | (1<<(ix-32))));
    } else {
        val = OS_REG_READ(ah, AR_MCAST_FIL0);
        OS_REG_WRITE(ah, AR_MCAST_FIL0, (val | (1<<ix)));
    }
    return AH_TRUE;
}

/*
 * Get the receive filter.
 */
u_int32_t
ar5416GetRxFilter(struct ath_hal *ah)
{
    u_int32_t bits = OS_REG_READ(ah, AR_RX_FILTER);
    u_int32_t phybits = OS_REG_READ(ah, AR_PHY_ERR);
    if (phybits & AR_PHY_ERR_RADAR)
        bits |= HAL_RX_FILTER_PHYRADAR;
    if (phybits & (AR_PHY_ERR_OFDM_TIMING|AR_PHY_ERR_CCK_TIMING))
        bits |= HAL_RX_FILTER_PHYERR;
    return bits;
}

/*
 * Set the receive filter.
 */
void
ar5416SetRxFilter(struct ath_hal *ah, u_int32_t bits)
{
    u_int32_t phybits;

    OS_REG_WRITE(ah, AR_RX_FILTER, (bits & 0xffff) | AR_RX_COMPR_BAR);
    phybits = 0;
    if (bits & HAL_RX_FILTER_PHYRADAR)
        phybits |= AR_PHY_ERR_RADAR;
    if (bits & HAL_RX_FILTER_PHYERR)
        phybits |= AR_PHY_ERR_OFDM_TIMING | AR_PHY_ERR_CCK_TIMING;
    OS_REG_WRITE(ah, AR_PHY_ERR, phybits);
    if (phybits) {
        OS_REG_WRITE(ah, AR_RXCFG,
            OS_REG_READ(ah, AR_RXCFG) | AR_RXCFG_ZLFDMA);
    } else {
        OS_REG_WRITE(ah, AR_RXCFG,
            OS_REG_READ(ah, AR_RXCFG) &~ AR_RXCFG_ZLFDMA);
    }
}

#endif /* AH_SUPPORT_AR5416 */
