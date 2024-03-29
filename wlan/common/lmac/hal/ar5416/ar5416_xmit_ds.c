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
#include "ah_xr.h"
#include "ah_internal.h"

#include "ar5416/ar5416.h"
#include "ar5416/ar5416reg.h"
#include "ar5416/ar5416desc.h"
#include "ar5416/ar5416phy.h"

#ifdef AH_NEED_DESC_SWAP
static void ar5416SwapTxDesc(struct ath_desc *ds);
#endif


/* Set the Interrupt Req and VEOL for the frame */
/* Note: caller must ensure (_ds) is not NULL   */
#define AR5416_SET_INT_VEOL_IN_TXDESC(_ds, _value) do {                                 \
                u_int8_t moreData = ((_ds)->ds_ctl1 & AR_TxMore);                               \
                struct ar5416_desc *currds = (_ds);                                     \
                while (currds && moreData) {                                            \
                        currds->ds_ctl0 = (_value) ?                                    \
                                                (currds->ds_ctl0 | (AR_TxIntrReq))      \
                                              : (currds->ds_ctl0 & ~(AR_TxIntrReq));    \
                        currds++;                                                       \
                        if (currds) {                                                   \
                                moreData = (currds->ds_ctl1 & AR_TxMore);                       \
                        }                                                               \
                }                                                                       \
                HALASSERT(currds);                                                      \
                /* Set the Last Desc interrupt Req and VEOL bit */                      \
                currds->ds_ctl0 = (_value) ?                                            \
                                        (currds->ds_ctl0 | (AR_TxIntrReq | AR_VEOL))    \
                                      : (currds->ds_ctl0 & ~(AR_TxIntrReq | AR_VEOL));  \
} while(0)

HAL_BOOL
ar5416UpdateCTSForBursting(struct ath_hal *ah, struct ath_desc *ds,
        struct ath_desc *prevds,
        struct ath_desc *prevdsWithCTS,
        struct ath_desc *gatingds,
        u_int32_t txOpLimit /* in us */,
        u_int32_t ctsDuration)
{
        struct ath_hal_5416 *ahp = AH5416(ah);
        struct ar5416_desc *ads = AR5416DESC(ds);
        struct ar5416_desc *aprevds = AR5416DESC(prevds);
        struct ar5416_desc *aprevdsWithCTS = AR5416DESC(prevdsWithCTS);
        struct ar5416_desc *agatingds = AR5416DESC(gatingds);
        u_int32_t       aprevRTSCTSDur = 0;
        u_int32_t       decCTSdur = 0;
        u_int8_t        doneBit = 1;
        u_int8_t        disableRTSCTS = 0;
        u_int8_t        aprevRTSEnable = 0;
        u_int32_t       adsRTSCTSDur = 0;
        u_int32_t       adsRTSCTSRate = 0;
        u_int32_t       adsRTSCTSEnable = 0;

        HALASSERT(ads);

        if (txOpLimit) {
                /* Check if previous RTS/CTS has been sent yet.   */
                /* If not, record its RTS/CTS duration            */
                HAL_INT omask;
                /*
                 * Disable interrupts while futzing with the live queue.
                 */
                omask = ar5416SetInterrupts(ah, ahp->ah_maskReg &~ HAL_INT_GLOBAL);

                if (agatingds) {
                        doneBit = (agatingds->ds_txstatus9 & AR_TxDone);
                        aprevRTSCTSDur = MS(aprevdsWithCTS->ds_ctl2, AR_BurstDur);
                        aprevRTSEnable = (aprevdsWithCTS->ds_ctl0 & AR_RTSEnable);
                }

                /* Record current frame's RTS/CTS properties:        */
                /* duration, rate, enable                            */
                adsRTSCTSRate = MS(ads->ds_ctl7, AR_RTSCTSRate);
                adsRTSCTSDur = MS(ads->ds_ctl2, AR_BurstDur);
                adsRTSCTSEnable = (ads->ds_ctl0 & (AR_RTSEnable | AR_CTSEnable));

                /*  If the previous CTS packet has been sent, exit early */
                if (!doneBit && adsRTSCTSEnable) {
                        /* Account for extra RTS duration that covers the CTS packet */
                        if (aprevRTSEnable) {
                                decCTSdur = ctsDuration;
                        }

                        /*
                         *  If the previous CTS packet's CTS duration can be extended to cover
                         *  the current packet while not exceeding the TxOpLimit,
                         *  extend this CTS duration and disable CTS
                         *  on the current packet ---> burst extention
                         */
                        if ((aprevRTSCTSDur + adsRTSCTSDur - decCTSdur) <= txOpLimit) {
                                aprevRTSCTSDur += (adsRTSCTSDur - decCTSdur);
                                if (aprevdsWithCTS) {
                                        aprevdsWithCTS->ds_ctl2 =
                                                (aprevdsWithCTS->ds_ctl2 & ~AR_BurstDur) |
                                                SM(aprevRTSCTSDur, AR_BurstDur);
                                        if (!(agatingds->ds_txstatus9 & AR_TxDone)) {
                                                disableRTSCTS = 1;
                                        }
                                }

                                if (disableRTSCTS) {
                                        ads->ds_ctl0 = (ads->ds_ctl0 & ~(AR_RTSEnable | AR_CTSEnable));
                                        ads->ds_ctl2 = (ads->ds_ctl2 & ~AR_BurstDur);
                                }
                        }
                }

                AR5416_SET_INT_VEOL_IN_TXDESC(ads, 1);
                if (disableRTSCTS && aprevds) {
                        AR5416_SET_INT_VEOL_IN_TXDESC(aprevds, 0);
                }

                /* re-enable chip interrupts */
                ar5416SetInterrupts(ah, omask);
        }

        return disableRTSCTS;
}

void
ar5416TxReqIntrDesc(struct ath_hal *ah, struct ath_desc *ds)
{
    struct ar5416_desc *ads = AR5416DESC(ds);
    ads->ds_ctl0 |= AR_TxIntrReq;
}

HAL_BOOL
ar5416SetupTxDesc(struct ath_hal *ah, struct ath_desc *ds,
        u_int pktLen,
        u_int hdrLen,
        HAL_PKT_TYPE type,
        u_int txPower,
        u_int txRate0, u_int txTries0,
        u_int keyIx,
        u_int antMode,
        u_int flags,
        u_int rtsctsRate,
        u_int rtsctsDuration,
        u_int compicvLen,
        u_int compivLen,
        u_int comp)
{
#define RTSCTS  (HAL_TXDESC_RTSENA|HAL_TXDESC_CTSENA)
        struct ar5416_desc *ads = AR5416DESC(ds);
        struct ath_hal_5416 *ahp = AH5416(ah);

        (void) hdrLen;

        ads->ds_txstatus9 &= ~AR_TxDone;

        HALASSERT(txTries0 != 0);
        HALASSERT(isValidPktType(type));
        HALASSERT(isValidTxRate(txRate0));
        HALASSERT((flags & RTSCTS) != RTSCTS);
        /* XXX validate antMode */

        txPower = (txPower + ahp->ah_txPowerIndexOffset );
        if(txPower > 63)  txPower=63;

        ads->ds_ctl0 = (pktLen & AR_FrameLen)
                     | (txPower << AR_XmitPower_S)
                     | (flags & HAL_TXDESC_VEOL ? AR_VEOL : 0)
                     | (flags & HAL_TXDESC_CLRDMASK ? AR_ClrDestMask : 0)
                     | (flags & HAL_TXDESC_INTREQ ? AR_TxIntrReq : 0)
                     ;
        ads->ds_ctl1 = (type << AR_FrameType_S)
                     | (flags & HAL_TXDESC_NOACK ? AR_NoAck : 0);
        ads->ds_ctl2 = SM(txTries0, AR_XmitDataTries0)
                     ;
        ads->ds_ctl3 = (txRate0 << AR_XmitRate0_S)
                     ;

        ads->ds_ctl7 = SM(AR5416_LEGACY_CHAINMASK, AR_ChainSel0)
                     | SM(AR5416_LEGACY_CHAINMASK, AR_ChainSel1)
                     | SM(AR5416_LEGACY_CHAINMASK, AR_ChainSel2)
                     | SM(AR5416_LEGACY_CHAINMASK, AR_ChainSel3)
                     ;

        if (keyIx != HAL_TXKEYIX_INVALID) {
                /* XXX validate key index */
                ads->ds_ctl1 |= SM(keyIx, AR_DestIdx);
                ads->ds_ctl0 |= AR_DestIdxValid;
        }
        if (flags & RTSCTS) {
                if (!isValidTxRate(rtsctsRate)) {
                        HDPRINTF(ah, HAL_DBG_TXDESC, "%s: invalid rts/cts rate 0x%x\n",
                                __func__, rtsctsRate);
                        return AH_FALSE;
                }
                /* XXX validate rtsctsDuration */
                ads->ds_ctl0 |= (flags & HAL_TXDESC_CTSENA ? AR_CTSEnable : 0)
                             | (flags & HAL_TXDESC_RTSENA ? AR_RTSEnable : 0)
                             ;
                ads->ds_ctl2 |= SM(rtsctsDuration, AR_BurstDur);
                ads->ds_ctl3 |= (rtsctsRate << AR_RTSCTSRate_S);
        }
        return AH_TRUE;
#undef RTSCTS
}

HAL_BOOL
ar5416SetupXTxDesc(struct ath_hal *ah, struct ath_desc *ds,
        u_int txRate1, u_int txTries1,
        u_int txRate2, u_int txTries2,
        u_int txRate3, u_int txTries3)
{
        struct ar5416_desc *ads = AR5416DESC(ds);

        if (txTries1) {
                HALASSERT(isValidTxRate(txRate1));
                ads->ds_ctl2 |= AR_DurUpdateEna;
                ads->ds_ctl2 |= SM(txTries1, AR_XmitDataTries1);
                ads->ds_ctl3 |= (txRate1 << AR_XmitRate1_S);
        }
        if (txTries2) {
                HALASSERT(isValidTxRate(txRate2));
                ads->ds_ctl2 |= SM(txTries2, AR_XmitDataTries2);
                ads->ds_ctl3 |= (txRate2 << AR_XmitRate2_S);
        }
        if (txTries3) {
                HALASSERT(isValidTxRate(txRate3));
                ads->ds_ctl2 |= SM(txTries3, AR_XmitDataTries3);
                ads->ds_ctl3 |= (txRate3 << AR_XmitRate3_S);
        }
        return AH_TRUE;
}

HAL_BOOL
ar5416FillTxDesc(struct ath_hal *ah, struct ath_desc *ds,
        u_int segLen, HAL_BOOL firstSeg, HAL_BOOL lastSeg,
        const struct ath_desc *ds0)
{
        struct ar5416_desc *ads = AR5416DESC(ds);

        HALASSERT((segLen &~ AR_BufLen) == 0);

        if (firstSeg) {
                /*
                 * First descriptor, don't clobber xmit control data
                 * setup by ar5416SetupTxDesc.
                 */
                ads->ds_ctl1 |= segLen | (lastSeg ? 0 : AR_TxMore);
        } else if (lastSeg) {           /* !firstSeg && lastSeg */
                /*
                 * Last descriptor in a multi-descriptor frame,
                 * copy the multi-rate transmit parameters from
                 * the first frame for processing on completion.
                 */
                ads->ds_ctl0 = 0;
                ads->ds_ctl1 = segLen;
#ifdef AH_NEED_DESC_SWAP
                ads->ds_ctl2 = __bswap32(AR5416DESC_CONST(ds0)->ds_ctl2);
                ads->ds_ctl3 = __bswap32(AR5416DESC_CONST(ds0)->ds_ctl3);
#else
                ads->ds_ctl2 = AR5416DESC_CONST(ds0)->ds_ctl2;
                ads->ds_ctl3 = AR5416DESC_CONST(ds0)->ds_ctl3;
#endif
        } else {                        /* !firstSeg && !lastSeg */
                /*
                 * Intermediate descriptor in a multi-descriptor frame.
                 */
                ads->ds_ctl0 = 0;
                ads->ds_ctl1 = segLen | AR_TxMore;
                ads->ds_ctl2 = 0;
                ads->ds_ctl3 = 0;
        }
        ads->ds_txstatus0 = ads->ds_txstatus1 = 0;
        ads->ds_txstatus2 = ads->ds_txstatus3 = 0;
        ads->ds_txstatus4 = ads->ds_txstatus5 = 0;
        ads->ds_txstatus6 = ads->ds_txstatus7 = 0;
        ads->ds_txstatus8 = ads->ds_txstatus9 = 0;
        return AH_TRUE;
}

void
ar5416ClearTxDescStatus(struct ath_hal *ah, struct ath_desc *ds)
{
    struct ar5416_desc *ads = AR5416DESC(ds);
    ads->ds_txstatus0 = ads->ds_txstatus1 = 0;
    ads->ds_txstatus2 = ads->ds_txstatus3 = 0;
    ads->ds_txstatus4 = ads->ds_txstatus5 = 0;
    ads->ds_txstatus6 = ads->ds_txstatus7 = 0;
    ads->ds_txstatus8 = ads->ds_txstatus9 = 0;
}

#ifdef ATH_SWRETRY
void 
ar5416ClearDestMask(struct ath_hal *ah, struct ath_desc *ds)
{
    struct ar5416_desc *ads = AR5416DESC(ds);
    ads->ds_ctl0 |= AR_ClrDestMask;
}
#endif

#ifdef AH_NEED_DESC_SWAP
/* Swap transmit descriptor */
static __inline void
ar5416SwapTxDesc(struct ath_desc *ds)
{
        ds->ds_data = __bswap32(ds->ds_data);
        ds->ds_ctl0 = __bswap32(ds->ds_ctl0);
        ds->ds_ctl1 = __bswap32(ds->ds_ctl1);
        ds->ds_hw[0] = __bswap32(ds->ds_hw[0]);
        ds->ds_hw[1] = __bswap32(ds->ds_hw[1]);
        ds->ds_hw[2] = __bswap32(ds->ds_hw[2]);
        ds->ds_hw[3] = __bswap32(ds->ds_hw[3]);
}
#endif

/*
 * Processing of HW TX descriptor.
 */
HAL_STATUS
ar5416ProcTxDesc(struct ath_hal *ah, struct ath_desc *ds)
{
        struct ar5416_desc *ads = AR5416DESC(ds);

#ifdef AH_NEED_DESC_SWAP
        if ((ads->ds_txstatus9 & __bswap32(AR_TxDone)) == 0)
                return HAL_EINPROGRESS;

        ar5416SwapTxDesc(ds);
#else
        if ((ads->ds_txstatus9 & AR_TxDone) == 0)
                return HAL_EINPROGRESS;
#endif

        /* Update software copies of the HW status */
        ds->ds_txstat.ts_seqnum = MS(ads->ds_txstatus9, AR_SeqNum);
        ds->ds_txstat.ts_tstamp = ads->AR_SendTimestamp;
        ds->ds_txstat.ts_status = 0;
        ds->ds_txstat.ts_flags  = 0;

        if (ads->ds_txstatus1 & AR_ExcessiveRetries)
            ds->ds_txstat.ts_status |= HAL_TXERR_XRETRY;
        if (ads->ds_txstatus1 & AR_Filtered)
            ds->ds_txstat.ts_status |= HAL_TXERR_FILT;
        if (ads->ds_txstatus1 & AR_FIFOUnderrun) {
            ds->ds_txstat.ts_status |= HAL_TXERR_FIFO;
	        ar5416UpdateTxTrigLevel(ah, AH_TRUE);
        }
        if (ads->ds_txstatus9 & AR_TxOpExceeded)
            ds->ds_txstat.ts_status |= HAL_TXERR_XTXOP;
        if (ads->ds_txstatus1 & AR_TxTimerExpired)
            ds->ds_txstat.ts_status |= HAL_TXERR_TIMER_EXPIRED;

        if (ads->ds_txstatus1 & AR_DescCfgErr)
            ds->ds_txstat.ts_flags |= HAL_TX_DESC_CFG_ERR;
        if (ads->ds_txstatus1 & AR_TxDataUnderrun) {
            ds->ds_txstat.ts_flags |= HAL_TX_DATA_UNDERRUN;
        ar5416UpdateTxTrigLevel(ah, AH_TRUE);
    }
        if (ads->ds_txstatus1 & AR_TxDelimUnderrun) {
            ds->ds_txstat.ts_flags |= HAL_TX_DELIM_UNDERRUN;
        ar5416UpdateTxTrigLevel(ah, AH_TRUE);
    }
        if (ads->ds_txstatus0 & AR_TxBaStatus) {
            ds->ds_txstat.ts_flags |= HAL_TX_BA;
            ds->ds_txstat.ba_low = ads->AR_BaBitmapLow;
            ds->ds_txstat.ba_high = ads->AR_BaBitmapHigh;
        }

        /*
         * Extract the transmit rate.
         */
        ds->ds_txstat.ts_rateindex = MS(ads->ds_txstatus9, AR_FinalTxIdx);
        switch (ds->ds_txstat.ts_rateindex) {
        case 0:
            ds->ds_txstat.ts_ratecode = MS(ads->ds_ctl3, AR_XmitRate0);
            break;
        case 1:
            ds->ds_txstat.ts_ratecode = MS(ads->ds_ctl3, AR_XmitRate1);
            break;
        case 2:
            ds->ds_txstat.ts_ratecode = MS(ads->ds_ctl3, AR_XmitRate2);
            break;
        case 3:
            ds->ds_txstat.ts_ratecode = MS(ads->ds_ctl3, AR_XmitRate3);
            break;
        }

        ds->ds_txstat.ts_rssi =
                                    MS(ads->ds_txstatus5, AR_TxRSSICombined);
        ds->ds_txstat.ts_rssi_ctl0 = MS(ads->ds_txstatus0, AR_TxRSSIAnt00);
        ds->ds_txstat.ts_rssi_ctl1 = MS(ads->ds_txstatus0, AR_TxRSSIAnt01);
        ds->ds_txstat.ts_rssi_ctl2 = MS(ads->ds_txstatus0, AR_TxRSSIAnt02);
        ds->ds_txstat.ts_rssi_ext0 = MS(ads->ds_txstatus5, AR_TxRSSIAnt10);
        ds->ds_txstat.ts_rssi_ext1 = MS(ads->ds_txstatus5, AR_TxRSSIAnt11);
        ds->ds_txstat.ts_rssi_ext2 = MS(ads->ds_txstatus5, AR_TxRSSIAnt12);
        ds->ds_txstat.evm0 = ads->AR_TxEVM0;
        ds->ds_txstat.evm1 = ads->AR_TxEVM1;
        ds->ds_txstat.evm2 = ads->AR_TxEVM2;
        ds->ds_txstat.ts_shortretry = MS(ads->ds_txstatus1, AR_RTSFailCnt);
        ds->ds_txstat.ts_longretry = MS(ads->ds_txstatus1, AR_DataFailCnt);
        ds->ds_txstat.ts_virtcol = MS(ads->ds_txstatus1, AR_VirtRetryCnt);
        ds->ds_txstat.ts_antenna = 1;

        return HAL_OK;
}

/*
 * Calculate air time of a transmit packet
 */
u_int32_t
ar5416CalcTxAirtime(struct ath_hal *ah, struct ath_desc *ds)
{
    struct ar5416_desc *ads = AR5416DESC(ds);

    /*
     * Calculate time of transmit on air for packet including retries
     * at different rates.
     */
    switch (MS(ads->ds_txstatus9, AR_FinalTxIdx)) {
    case 0:
        return  MS(ads->ds_ctl3, AR_PacketDur0);
    case 1:
        return  MS(ads->ds_ctl4, AR_PacketDur1) +
               (MS(ads->ds_ctl2, AR_XmitDataTries0) * MS(ads->ds_ctl4, AR_PacketDur0));
    case 2:
        return  MS(ads->ds_ctl5, AR_PacketDur2) +
               (MS(ads->ds_ctl2, AR_XmitDataTries1) * MS(ads->ds_ctl4, AR_PacketDur1)) +
               (MS(ads->ds_ctl2, AR_XmitDataTries0) * MS(ads->ds_ctl4, AR_PacketDur0));
    case 3:
        return  MS(ads->ds_ctl5, AR_PacketDur3) +
               (MS(ads->ds_ctl2, AR_XmitDataTries2) * MS(ads->ds_ctl5, AR_PacketDur2)) +
               (MS(ads->ds_ctl2, AR_XmitDataTries1) * MS(ads->ds_ctl4, AR_PacketDur1)) +
               (MS(ads->ds_ctl2, AR_XmitDataTries0) * MS(ads->ds_ctl4, AR_PacketDur0));
    default:
        HALASSERT(0);
        return 0;
    }
}

#ifdef AH_PRIVATE_DIAG
void
ar5416_ContTxMode(struct ath_hal *ah, struct ath_desc *ds, int mode)
{
        static int qnum =0;
        int i;
        unsigned int qbits, val, val1, val2;
        int prefetch;
        struct ar5416_desc *ads = AR5416DESC(ds);

        if(mode == 10) return;

    if (mode==7) {                      // print status from the cont tx desc
                if (ads) {
                        val1 = ads->ds_txstatus0;
                        val2 = ads->ds_txstatus1;
                        HDPRINTF(ah, HAL_DBG_TXDESC, "s0(%x) s1(%x)\n",
                                                   (unsigned)val1, (unsigned)val2);
                }
                HDPRINTF(ah, HAL_DBG_TXDESC, "txe(%x) txd(%x)\n",
                                           OS_REG_READ(ah, AR_Q_TXE),
                                           OS_REG_READ(ah, AR_Q_TXD)
                        );
                for(i=0;i<HAL_NUM_TX_QUEUES; i++) {
                        val = OS_REG_READ(ah, AR_QTXDP(i));
                        val2 = OS_REG_READ(ah, AR_QSTS(i)) & AR_Q_STS_PEND_FR_CNT;
                        HDPRINTF(ah, HAL_DBG_TXDESC, "[%d] %x %d\n", i, val, val2);
                }
                return;
    }
    if (mode==8) {                      // set TXE for qnum
                OS_REG_WRITE(ah, AR_Q_TXE, 1<<qnum);
                return;
    }
    if (mode==9) {
                prefetch = (int)ds;
                return;
    }

    if (mode >= 1) {                    // initiate cont tx operation
                /* Disable AGC to A2 */
                qnum = (int) ds;

                OS_REG_WRITE(ah, AR_PHY_TEST,
                                         (OS_REG_READ(ah, AR_PHY_TEST) | PHY_AGC_CLR) );

                OS_REG_WRITE(ah, 0x9864, OS_REG_READ(ah, 0x9864) | 0x7f000);
                OS_REG_WRITE(ah, 0x9924, OS_REG_READ(ah, 0x9924) | 0x7f00fe);
                OS_REG_WRITE(ah, AR_DIAG_SW,
                                         (OS_REG_READ(ah, AR_DIAG_SW) | (AR_DIAG_FORCE_RX_CLEAR+AR_DIAG_IGNORE_VIRT_CS)) );


                OS_REG_WRITE(ah, AR_CR, AR_CR_RXD);     // set receive disable

                if (mode == 3 || mode == 4) {
                        int txcfg;

                        if (mode == 3) {
                                OS_REG_WRITE(ah, AR_DLCL_IFS(qnum), 0);
                                OS_REG_WRITE(ah, AR_DRETRY_LIMIT(qnum), 0xffffffff);
                                OS_REG_WRITE(ah, AR_D_GBL_IFS_SIFS, 100);
                                OS_REG_WRITE(ah, AR_D_GBL_IFS_EIFS, 100);
                                OS_REG_WRITE(ah, AR_TIME_OUT, 2);
                                OS_REG_WRITE(ah, AR_D_GBL_IFS_SLOT, 100);
                        }
                        OS_REG_WRITE(ah, AR_DRETRY_LIMIT(qnum), 0xffffffff);
                        OS_REG_WRITE(ah, AR_D_FPCTL, 0x10|qnum);        // enable prefetch on qnum
                        txcfg = 5 | (6<<AR_FTRIG_S);
                        OS_REG_WRITE(ah, AR_TXCFG, txcfg);

                        OS_REG_WRITE(ah, AR_QMISC(qnum),        // set QCU modes
                                                 AR_Q_MISC_DCU_EARLY_TERM_REQ
                                                 +AR_Q_MISC_FSP_ASAP
                                                 +AR_Q_MISC_CBR_INCR_DIS1
                                                 +AR_Q_MISC_CBR_INCR_DIS0
                                );

                        /* stop tx dma all all except qnum */
                        qbits = 0x3ff;
                        qbits &= ~(1<<qnum);
                        for (i=0; i<10; i++) {
                                if (i==qnum) continue;
                                OS_REG_WRITE(ah, AR_Q_TXD, 1<<i);
                        }
                        OS_REG_WRITE(ah, AR_Q_TXD, qbits);

                        /* clear and freeze MIB counters */
                        OS_REG_WRITE(ah, AR_MIBC, AR_MIBC_CMC);
                        OS_REG_WRITE(ah, AR_MIBC, AR_MIBC_FMC);

                        OS_REG_WRITE(ah, AR_DMISC(qnum),
                                                 (AR_D_MISC_ARB_LOCKOUT_CNTRL_GLOBAL << AR_D_MISC_ARB_LOCKOUT_CNTRL_S)
                                                 +(AR_D_MISC_ARB_LOCKOUT_IGNORE)
                                                 +(AR_D_MISC_POST_FR_BKOFF_DIS)
                                                 +(AR_D_MISC_VIR_COL_HANDLING_IGNORE << AR_D_MISC_VIR_COL_HANDLING_S)
                                );

                        for(i=0; i<HAL_NUM_TX_QUEUES+2; i++) {  // disconnect QCUs
                                if (i==qnum) continue;
                                OS_REG_WRITE(ah, AR_DQCUMASK(i), 0);
                        }
                }
    }
    if (mode == 0) {

                OS_REG_WRITE(ah, AR_PHY_TEST,
                                         (OS_REG_READ(ah, AR_PHY_TEST) & ~PHY_AGC_CLR) );
                OS_REG_WRITE(ah, AR_DIAG_SW,
                                         (OS_REG_READ(ah, AR_DIAG_SW) & ~(AR_DIAG_FORCE_RX_CLEAR+AR_DIAG_IGNORE_VIRT_CS)) );
    }
}
#endif

void
ar5416Set11nTxDesc(struct ath_hal *ah, struct ath_desc *ds,
                                   u_int pktLen, HAL_PKT_TYPE type, u_int txPower,
                                   u_int keyIx, HAL_KEY_TYPE keyType,
                                   u_int flags)
{
        struct ar5416_desc *ads = AR5416DESC(ds);
        struct ath_hal_5416 *ahp = AH5416(ah);

        HALASSERT(isValidPktType(type));
        HALASSERT(isValidKeyType(keyType));

        txPower += ahp->ah_txPowerIndexOffset;
        if (txPower > 63)
                txPower = 63;

        ads->ds_ctl0 = (pktLen & AR_FrameLen)
                                 | (flags & HAL_TXDESC_VMF ? AR_VirtMoreFrag : 0)
                                 | SM(txPower, AR_XmitPower)
                                 | (flags & HAL_TXDESC_VEOL ? AR_VEOL : 0)
                                 | (flags & HAL_TXDESC_CLRDMASK ? AR_ClrDestMask : 0)
                                 | (flags & HAL_TXDESC_INTREQ ? AR_TxIntrReq : 0)
                                 | (keyIx != HAL_TXKEYIX_INVALID ? AR_DestIdxValid : 0)
                                 | (flags & HAL_TXDESC_LOWRXCHAIN ? AR_LowRxChain : 0);

        ads->ds_ctl1 = (keyIx != HAL_TXKEYIX_INVALID ? SM(keyIx, AR_DestIdx) : 0)
                                 | SM(type, AR_FrameType)
                                 | (flags & HAL_TXDESC_NOACK ? AR_NoAck : 0)
                                 | (flags & HAL_TXDESC_EXT_ONLY ? AR_ExtOnly : 0)
                                 | (flags & HAL_TXDESC_EXT_AND_CTL ? AR_ExtAndCtl : 0);

        ads->ds_ctl6 = SM(keyType, AR_EncrType);
		
		/* For Kite set the tx antenna selection */
		if (AR_SREV_KITE(ah)) {
			/* FIXME: For now set the tx antenna to always 0,
			 * till tx selection diverisity is implementated
			 */
		    ads->ds_ctl8 = 0;
		    ads->ds_ctl9 = 0;
		    ads->ds_ctl10 = 0;
		    ads->ds_ctl11 = 0;
		}
}

#define HT_RC_2_MCS(_rc)        ((_rc) & 0x0f)
static const u_int8_t baDurationDelta[] = {
    24,     //  0: BPSK
    12,     //  1: QPSK 1/2
    12,     //  2: QPSK 3/4
     4,     //  3: 16-QAM 1/2
     4,     //  4: 16-QAM 3/4
     4,     //  5: 64-QAM 2/3
     4,     //  6: 64-QAM 3/4
     4,     //  7: 64-QAM 5/6
    24,     //  8: BPSK
    12,     //  9: QPSK 1/2
    12,     // 10: QPSK 3/4
     4,     // 11: 16-QAM 1/2
     4,     // 12: 16-QAM 3/4
     4,     // 13: 64-QAM 2/3
     4,     // 14: 64-QAM 3/4
     4,     // 15: 64-QAM 5/6
};

void
ar5416Set11nRateScenario(struct ath_hal *ah, struct ath_desc *ds,
                             struct ath_desc *lastds,
                             u_int durUpdateEn, u_int rtsctsRate, u_int rtsctsDuration,
                             HAL_11N_RATE_SERIES series[], u_int nseries,
                             u_int flags)
{
        struct ar5416_desc *ads = AR5416DESC(ds);
        struct ar5416_desc *last_ads = AR5416DESC(lastds);
        u_int32_t ds_ctl0;
        u_int32_t burstDur = 0;

        HALASSERT(nseries == 4);
        (void)nseries;
        (void)rtsctsDuration;   /* use H/W to calculate RTSCTSDuration */

        /*
         * Rate control settings override
         */
        if (flags & (HAL_TXDESC_RTSENA | HAL_TXDESC_CTSENA)) {
            ds_ctl0 = ads->ds_ctl0;

            if (flags & HAL_TXDESC_RTSENA) {
                ds_ctl0 &= ~AR_CTSEnable;
                ds_ctl0 |= AR_RTSEnable;
            } else {
                ds_ctl0 &= ~AR_RTSEnable;
                ds_ctl0 |= AR_CTSEnable;
            }

            ads->ds_ctl0 = ds_ctl0;
        } else {
            ads->ds_ctl0 = (ads->ds_ctl0 & ~(AR_RTSEnable | AR_CTSEnable));
        }

        if (!AR_SREV_MERLIN_10_OR_LATER(ah)) {
            /*
             * WAR for bug#27504: - MAC assumes normal ACK time instead of 
             * block ACK while computing packet duration. Add this delta to 
             * the burst duration in the descriptor.
             */
            if (ads->ds_ctl1 & AR_IsAggr) {
                burstDur = baDurationDelta[HT_RC_2_MCS(series[0].Rate)]; 
            }
        }

        ads->ds_ctl2 = set11nTries(series, 0)
                                 |  set11nTries(series, 1)
                                 |  set11nTries(series, 2)
                                 |  set11nTries(series, 3)
                                 |  (durUpdateEn ? AR_DurUpdateEna : 0)
                                 |  SM(0, AR_BurstDur);

        ads->ds_ctl3 = set11nRate(series, 0)
                                 |  set11nRate(series, 1)
                                 |  set11nRate(series, 2)
                                 |  set11nRate(series, 3);

        ads->ds_ctl4 = set11nPktDurRTSCTS(series, 0)
                                 |  set11nPktDurRTSCTS(series, 1);

        ads->ds_ctl5 = set11nPktDurRTSCTS(series, 2)
                                 |  set11nPktDurRTSCTS(series, 3);

        ads->ds_ctl7 = set11nRateFlags(series, 0)
                                 |  set11nRateFlags(series, 1)
                                 |  set11nRateFlags(series, 2)
                                 |  set11nRateFlags(series, 3)
                                 | SM(rtsctsRate, AR_RTSCTSRate);
#ifdef AH_NEED_DESC_SWAP
        last_ads->ds_ctl2 = __bswap32(ads->ds_ctl2);
        last_ads->ds_ctl3 = __bswap32(ads->ds_ctl3);
#else
        last_ads->ds_ctl2 = ads->ds_ctl2;
        last_ads->ds_ctl3 = ads->ds_ctl3;
#endif
}

void
ar5416Set11nAggrFirst(struct ath_hal *ah, struct ath_desc *ds, u_int aggrLen)
{
    struct ar5416_desc *ads = AR5416DESC(ds);

    ads->ds_ctl1 |= (AR_IsAggr | AR_MoreAggr);

    ads->ds_ctl6 &= ~AR_AggrLen;
    ads->ds_ctl6 |= SM(aggrLen, AR_AggrLen);
}

void
ar5416Set11nAggrMiddle(struct ath_hal *ah, struct ath_desc *ds, u_int numDelims)
{
    struct ar5416_desc *ads = AR5416DESC(ds);
    unsigned int ctl6;

    ads->ds_ctl1 |= (AR_IsAggr | AR_MoreAggr);

    /*
     * We use a stack variable to manipulate ctl6 to reduce uncached
     * read modify, modfiy, write.
     */
    ctl6 = ads->ds_ctl6;
    ctl6 &= ~AR_PadDelim;
    ctl6 |= SM(numDelims, AR_PadDelim);
    ads->ds_ctl6 = ctl6;
}

void
ar5416Set11nAggrLast(struct ath_hal *ah, struct ath_desc *ds)
{
    struct ar5416_desc *ads = AR5416DESC(ds);

    ads->ds_ctl1 |= AR_IsAggr;
    ads->ds_ctl1 &= ~AR_MoreAggr;
    ads->ds_ctl6 &= ~AR_PadDelim;
}

void
ar5416Clr11nAggr(struct ath_hal *ah, struct ath_desc *ds)
{
    struct ar5416_desc *ads = AR5416DESC(ds);

    ads->ds_ctl1 &= (~AR_IsAggr & ~AR_MoreAggr);
}

void
ar5416Set11nBurstDuration(struct ath_hal *ah, struct ath_desc *ds,
                                                  u_int burstDuration)
{
    struct ar5416_desc *ads = AR5416DESC(ds);

    ads->ds_ctl2 &= ~AR_BurstDur;
    ads->ds_ctl2 |= SM(burstDuration, AR_BurstDur);
}

void
ar5416Set11nRifsBurstMiddle(struct ath_hal *ah, struct ath_desc *ds)
{
    struct ar5416_desc *ads = AR5416DESC(ds);

    ads->ds_ctl1 |= AR_MoreRifs | AR_NoAck;
    ads->ds_ctl0 &= ~AR_TxIntrReq;
}

void
ar5416Set11nRifsBurstLast(struct ath_hal *ah, struct ath_desc *ds)
{
    struct ar5416_desc *ads = AR5416DESC(ds);

    ads->ds_ctl1 &= (~AR_MoreAggr & ~AR_MoreRifs);
}

void
ar5416Clr11nRifsBurst(struct ath_hal *ah, struct ath_desc *ds)
{
    struct ar5416_desc *ads = AR5416DESC(ds);

    ads->ds_ctl1 &= (~AR_MoreRifs & ~AR_NoAck);
    ads->ds_ctl0 |= AR_TxIntrReq;
}

void
ar5416Set11nAggrRifsBurst(struct ath_hal *ah, struct ath_desc *ds)
{
    struct ar5416_desc *ads = AR5416DESC(ds);

    ads->ds_ctl1 |= AR_NoAck;
    ads->ds_ctl1 &= ~AR_MoreRifs;
    ads->ds_ctl0 &= ~AR_TxIntrReq;
}

void
ar5416Set11nVirtualMoreFrag(struct ath_hal *ah, struct ath_desc *ds,
                                                  u_int vmf)
{
    struct ar5416_desc *ads = AR5416DESC(ds);

    if (vmf)
    {
        ads->ds_ctl0 |=  AR_VirtMoreFrag;
    }
    else
    {
        ads->ds_ctl0 &= ~AR_VirtMoreFrag;
    }
}

#endif /* AH_SUPPORT_AR5416 */
