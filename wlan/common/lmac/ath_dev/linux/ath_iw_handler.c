/*! \file
**  \brief 
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
** This module is the Atheros specific ioctl/iwconfig/iwpriv interface
** to the ATH object, normally instantiated as wifiX, where X is the
** instance number (e.g. wifi0, wifi1).
**
** This provides a mechanism to configure the ATH object within the
** Linux OS enviornment.  This file is OS specific.
**
*/


/*
 * Wireless extensions support for 802.11 common code.
 */
#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif
#include <linux/version.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/utsname.h>
#include <linux/if_arp.h>       /* XXX for ARPHRD_ETHER */
#include <net/iw_handler.h>

#include "if_athvar.h"

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)) && !defined(UINTPTR_T_IN_TYPES)
#define UINTPTR_T_IN_TYPES
#endif
#include "ah.h"


/*
** Defines
*/

#define TABLE_SIZE(a)    (sizeof (a) / sizeof (a[0]))

#define ATH_HAL_IOCTL_SETPARAM              (SIOCIWFIRSTPRIV+0)
#define ATH_HAL_IOCTL_GETPARAM              (SIOCIWFIRSTPRIV+1)
#define ATH_IOCTL_SETCOUNTRY                (SIOCIWFIRSTPRIV+2)
#define ATH_IOCTL_GETCOUNTRY                (SIOCIWFIRSTPRIV+3)

#define ATH_GET_COUNTRY                     1
#define ATH_SET_COUNTRY                     2

/*
** We have to do a "split" of ID values, since they are all combined
** into the same table.  This value is a "shift" value for ATH parameters
*/

#define ATH_PARAM_SHIFT     0x1000
#define SPECIAL_PARAM_SHIFT 0x2000


static const struct iw_priv_args ath_iw_priv_args[] = {
    /*
    ** HAL interface routines and parameters
    */

    { ATH_HAL_IOCTL_SETPARAM,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2,
       0, "setHALparam" },
    { ATH_HAL_IOCTL_GETPARAM,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,
       IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,    "getHALparam" },

    /* sub-ioctl handlers */
    { ATH_HAL_IOCTL_SETPARAM,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "" },
    { ATH_HAL_IOCTL_GETPARAM,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "" },

    { HAL_CONFIG_DMA_BEACON_RESPONSE_TIME,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "DMABcnRespT" },
    { HAL_CONFIG_DMA_BEACON_RESPONSE_TIME,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetDMABcnRespT" },

    { HAL_CONFIG_SW_BEACON_RESPONSE_TIME,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "SWBcnRespT" },
    { HAL_CONFIG_SW_BEACON_RESPONSE_TIME,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetSWBcnRespT" },

    { HAL_CONFIG_ADDITIONAL_SWBA_BACKOFF,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "AddSWBbo" },
    { HAL_CONFIG_ADDITIONAL_SWBA_BACKOFF,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetAddSWBAbo" },

    { HAL_CONFIG_6MB_ACK,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "6MBAck" },
    { HAL_CONFIG_6MB_ACK,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "Get5MBAck" },

    { HAL_CONFIG_CWMIGNOREEXTCCA,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "CWMIgnExCCA" },
    { HAL_CONFIG_CWMIGNOREEXTCCA,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetCWMIgnExCCA" },

    { HAL_CONFIG_FORCEBIAS,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "ForceBias" },
    { HAL_CONFIG_FORCEBIAS,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetForceBias" },

    { HAL_CONFIG_FORCEBIASAUTO,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "ForBiasAuto" },
    { HAL_CONFIG_FORCEBIASAUTO,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetForBiasAuto" },

    { HAL_CONFIG_PCIEPOWERSAVEENABLE,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "PCIEPwrSvEn" },
    { HAL_CONFIG_PCIEPOWERSAVEENABLE,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetPCIEPwrSvEn" },

    { HAL_CONFIG_PCIEL1SKPENABLE,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "PCIEL1SKPEn" },
    { HAL_CONFIG_PCIEL1SKPENABLE,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetPCIEL1SKPEn" },

    { HAL_CONFIG_PCIECLOCKREQ,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "PCIEClkReq" },
    { HAL_CONFIG_PCIECLOCKREQ,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetPCIEClkReq" },

    { HAL_CONFIG_PCIEWAEN,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "PCIEWAEN" },
    { HAL_CONFIG_PCIEWAEN,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetPCIEWAEN" },

    { HAL_CONFIG_PCIEPOWERRESET,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "PCIEPwRset" },
    { HAL_CONFIG_PCIEPOWERRESET,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetPCIEPwRset" },

    { HAL_CONFIG_PCIERESTORE,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "PCIERestore" },
    { HAL_CONFIG_PCIERESTORE,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetPCIERestore" },

    { HAL_CONFIG_HTENABLE,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "HTEna" },
    { HAL_CONFIG_HTENABLE,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetHTEna" },

    { HAL_CONFIG_DISABLETURBOG,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "DisTurboG" },
    { HAL_CONFIG_DISABLETURBOG,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetDisTurboG" },

    { HAL_CONFIG_OFDMTRIGLOW,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "OFDMTrgLow" },
    { HAL_CONFIG_OFDMTRIGLOW,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetOFDMTrgLow" },

    { HAL_CONFIG_OFDMTRIGHIGH,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "OFDMTrgHi" },
    { HAL_CONFIG_OFDMTRIGHIGH,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetOFDMTrgHi" },

    { HAL_CONFIG_CCKTRIGHIGH,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "CCKTrgHi" },
    { HAL_CONFIG_CCKTRIGHIGH,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetCCKTrgHi" },

    { HAL_CONFIG_CCKTRIGLOW,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "CCKTrgLow" },
    { HAL_CONFIG_CCKTRIGLOW,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetCCKTrgLow" },

    { HAL_CONFIG_ENABLEANI,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "ANIEna" },
    { HAL_CONFIG_ENABLEANI,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetANIEna" },

    { HAL_CONFIG_NOISEIMMUNITYLVL,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "NoiseImmLvl" },
    { HAL_CONFIG_NOISEIMMUNITYLVL,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetNoiseImmLvl" },

    { HAL_CONFIG_OFDMWEAKSIGDET,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "OFDMWeakDet" },
    { HAL_CONFIG_OFDMWEAKSIGDET,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetOFDMWeakDet" },

    { HAL_CONFIG_CCKWEAKSIGTHR,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "CCKWeakThr" },
    { HAL_CONFIG_CCKWEAKSIGTHR,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetCCKWeakThr" },

    { HAL_CONFIG_SPURIMMUNITYLVL,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "SpurImmLvl" },
    { HAL_CONFIG_SPURIMMUNITYLVL,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetSpurImmLvl" },

    { HAL_CONFIG_FIRSTEPLVL,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "FIRStepLvl" },
    { HAL_CONFIG_FIRSTEPLVL,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetFIRStepLvl" },

    { HAL_CONFIG_RSSITHRHIGH,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "RSSIThrHi" },
    { HAL_CONFIG_RSSITHRHIGH,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetRSSIThrHi" },

    { HAL_CONFIG_RSSITHRLOW,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "RSSIThrLow" },
    { HAL_CONFIG_RSSITHRLOW,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetRSSIThrLow" },

    { HAL_CONFIG_DIVERSITYCONTROL,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "DivtyCtl" },
    { HAL_CONFIG_DIVERSITYCONTROL,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetDivtyCtl" },

    { HAL_CONFIG_ANTENNASWITCHSWAP,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "AntSwap" },
    { HAL_CONFIG_ANTENNASWITCHSWAP,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetAntSwap" },

    { HAL_CONFIG_DEBUG,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "HALDbg" },
    { HAL_CONFIG_DEBUG,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "GetHALDbg" },
    { HAL_CONFIG_RXMIT_LASTPKT,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "rxmit_last" },
    { HAL_CONFIG_RXMIT_LASTPKT,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_rxmit_last" },
    { HAL_CONFIG_RXMIT_FIRSTPKT,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "rxmit_first" },
    { HAL_CONFIG_RXMIT_FIRSTPKT,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_rxmit_first" },
    { HAL_CONFIG_TXMIT_LASTPKT,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "txmit_last" },
    { HAL_CONFIG_TXMIT_LASTPKT,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_txmit_last" },
    { HAL_CONFIG_TXMIT_FIRSTPKT,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "txmit_first" },
    { HAL_CONFIG_TXMIT_FIRSTPKT,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_txmit_first" },

    /*
    ** ATH interface routines and parameters
    ** This is for integer parameters
    */

    { ATH_PARAM_TXCHAINMASK | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "txchainmask" },
    { ATH_PARAM_TXCHAINMASK | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "get_txchainmask" },

    { ATH_PARAM_RXCHAINMASK | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "rxchainmask" },
    { ATH_PARAM_RXCHAINMASK | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "get_rxchainmask" },

    { ATH_PARAM_CHAINMASK_SEL | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "chainmasksel" },
    { ATH_PARAM_CHAINMASK_SEL | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "get_chainmasksel" },

    { ATH_PARAM_AMPDU | ATH_PARAM_SHIFT,
      IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,         "AMPDU" },
    { ATH_PARAM_AMPDU | ATH_PARAM_SHIFT,
      0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,         "getAMPDU" },

    { ATH_PARAM_AMPDU_LIMIT | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "AMPDULim" },
    { ATH_PARAM_AMPDU_LIMIT | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "getAMPDULim" },

    { ATH_PARAM_AMPDU_SUBFRAMES | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "AMPDUFrames" },
    { ATH_PARAM_AMPDU_SUBFRAMES | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "getAMPDUFrames" },
#ifdef ATH_RB
    { ATH_PARAM_RX_RB | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "rb" },
    { ATH_PARAM_RX_RB | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "get_rb" },

    { ATH_PARAM_RX_RB_DETECT | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "rbdetect" },
    { ATH_PARAM_RX_RB_DETECT | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "get_rbdetect" },

    { ATH_PARAM_RX_RB_TIMEOUT | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "rbto" },
    { ATH_PARAM_RX_RB_TIMEOUT | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "get_rbto" },

    { ATH_PARAM_RX_RB_SKIPTHRESH | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,      "rbskipthresh" },
    { ATH_PARAM_RX_RB_SKIPTHRESH | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,      "get_rbskipthresh" },
#endif
    { ATH_PARAM_AGGR_PROT | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "AggrProt" },
    { ATH_PARAM_AGGR_PROT | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "getAggrProt" },

    { ATH_PARAM_AGGR_PROT_DUR | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "AggrProtDur" },
    { ATH_PARAM_AGGR_PROT_DUR | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "getAggrProtDur" },

    { ATH_PARAM_AGGR_PROT_MAX | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "AggrProtMax" },
    { ATH_PARAM_AGGR_PROT_MAX | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "getAddrProtMax" },

    { ATH_PARAM_TXPOWER_LIMIT | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "TXPowLim" },
    { ATH_PARAM_TXPOWER_LIMIT | ATH_PARAM_SHIFT
        ,0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,      "getTxPowLim" },

    { ATH_PARAM_TXPOWER_OVERRIDE | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "TXPwrOvr" },
    { ATH_PARAM_TXPOWER_OVERRIDE | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "getTXPwrOvr" },

    { ATH_PARAM_PCIE_DISABLE_ASPM_WK | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "DisASPMWk" },
    { ATH_PARAM_PCIE_DISABLE_ASPM_WK | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "getDisASPMWk" },

    { ATH_PARAM_PCID_ASPM | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "EnaASPM" },
    { ATH_PARAM_PCID_ASPM | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "getEnaASPM" },

    { ATH_PARAM_BEACON_NORESET | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "BcnNoReset" },
    { ATH_PARAM_BEACON_NORESET | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "getBcnNoReset" },
    { ATH_PARAM_CAB_CONFIG | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "CABlevel" },
    { ATH_PARAM_CAB_CONFIG | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "getCABlevel" },
    { ATH_PARAM_ATH_DEBUG | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "ATHDebug" },
    { ATH_PARAM_ATH_DEBUG | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "getATHDebug" },

    { ATH_PARAM_ATH_TPSCALE | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "tpscale" },
    { ATH_PARAM_ATH_TPSCALE | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "get_tpscale" },

    { ATH_PARAM_ACKTIMEOUT | ATH_PARAM_SHIFT,
      IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,         "acktimeout" },
    { ATH_PARAM_ACKTIMEOUT | ATH_PARAM_SHIFT,
      0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,         "get_acktimeout" },

#ifdef ATH_SUPPORT_IQUE
    { ATH_PARAM_RETRY_DURATION | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "retrydur" },
    { ATH_PARAM_RETRY_DURATION | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "get_retrydur" },
    { ATH_PARAM_HBR_HIGHPER | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "hbrPER_high" },
    { ATH_PARAM_HBR_HIGHPER | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "get_hbrPER_high" },
    { ATH_PARAM_HBR_LOWPER | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "hbrPER_low" },
    { ATH_PARAM_HBR_LOWPER | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "get_hbrPER_low" },
#endif

    { ATH_PARAM_TX_STBC | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "txstbc" },
    { ATH_PARAM_TX_STBC | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "get_txstbc" },

    { ATH_PARAM_RX_STBC | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "rxstbc" },
    { ATH_PARAM_RX_STBC | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "get_rxstbc" },
    { ATH_PARAM_TOGGLE_IMMUNITY | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "immunity" },
    { ATH_PARAM_TOGGLE_IMMUNITY | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "get_immunity" },

    { ATH_PARAM_LIMIT_LEGACY_FRM | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "limit_legacy" },
    { ATH_PARAM_LIMIT_LEGACY_FRM | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "get_limit_legacy" },

    { ATH_PARAM_GPIO_LED_CUSTOM | ATH_PARAM_SHIFT,
	 0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,      "get_ledcustom" },
    { ATH_PARAM_GPIO_LED_CUSTOM | ATH_PARAM_SHIFT,
	 IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,      "set_ledcustom" },

    { ATH_PARAM_SWAP_DEFAULT_LED | ATH_PARAM_SHIFT,
	 0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,      "get_swapled" },
    { ATH_PARAM_SWAP_DEFAULT_LED | ATH_PARAM_SHIFT,
	IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "set_swapled" },

    { ATH_PARAM_USE_EAP_LOWEST_RATE | ATH_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "eap_lowest_rate" },
    { ATH_PARAM_USE_EAP_LOWEST_RATE | ATH_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "get_eap_lowest" },

#ifdef AP_SLOW_ANT_DIV
    { ATH_PARAM_SLOWANTDIV_RSSITHRHIGH | ATH_PARAM_SHIFT,
      IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,         "antRssiHigh" },
    { ATH_PARAM_SLOWANTDIV_RSSITHRHIGH | ATH_PARAM_SHIFT,
      0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,         "get_antRssiHigh" },

    { ATH_PARAM_SLOWANTDIV_RSSITHRLOW | ATH_PARAM_SHIFT,
      IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,         "antRssiThrLow" },
    { ATH_PARAM_SLOWANTDIV_RSSITHRLOW | ATH_PARAM_SHIFT,
      0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,         "get_antRssiLow" },

    { ATH_PARAM_SLOWANTDIV_BYTESTHRRX | ATH_PARAM_SHIFT,
      IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,         "antBytesThrRx" },
    { ATH_PARAM_SLOWANTDIV_BYTESTHRRX | ATH_PARAM_SHIFT,
      0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,         "get_antBytesRx" },

    { ATH_PARAM_SLOWANTDIV_DWELL_TIME | ATH_PARAM_SHIFT,
      IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,         "antDwellTime" },
    { ATH_PARAM_SLOWANTDIV_DWELL_TIME | ATH_PARAM_SHIFT,
      0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,         "get_antDwellTime" },

    { ATH_PARAM_SLOWANTDIV_SETTLE_TIME | ATH_PARAM_SHIFT,
      IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,         "antSetleTime" },
    { ATH_PARAM_SLOWANTDIV_SETTLE_TIME | ATH_PARAM_SHIFT,
      0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,         "get_antSetleTime" },

    { ATH_PARAM_SLOWANTDIV_ENABLE | ATH_PARAM_SHIFT,
      IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,         "antEnable" },
    { ATH_PARAM_SLOWANTDIV_ENABLE | ATH_PARAM_SHIFT,
      0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,         "get_antEnable" },
#endif

/*
** "special" parameters
** processed in this funcion
*/

    { SPECIAL_PARAM_SHIFT,
        IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0,       "setCountryID" },
    { SPECIAL_PARAM_SHIFT,
        0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,       "getCountryID" },

    /*
    ** The Get Country/Set Country interface
    */

    { ATH_IOCTL_SETCOUNTRY, IW_PRIV_TYPE_CHAR | 3, 0,       "setCountry" },
    { ATH_IOCTL_GETCOUNTRY, 0, IW_PRIV_TYPE_CHAR | 3,       "getCountry" },
};


/******************************************************************************/
/*!
**  \brief Force Channel Set
**
**  This routine will force update the channel set in the IC when the country
**  code is changed.  This overrides the "world mode" processing required for
**  the station side.
**
**  \param dev Net Device pointer, Linux driver handle
**  \param info Request Info Stucture containing type info, etc.
**  \param w Pointer to a word value, not used here
**  \param extra Pointer to the actual data buffer.
**  \return 0 for success
**  \return Non Zero on error
*/

static void
forceChannelSet(struct ath_softc *sc,int countryCode)
{
    struct ath_hal *ah = sc->sc_ah;
    HAL_CHANNEL *chans;
    int nchan;
    u_int8_t regclassids[ATH_REGCLASSIDS_MAX];
    u_int nregclass = 0;
    u_int wMode;
    u_int netBand;


    wMode = MODE_SELECT_ALL;
    netBand = MODE_SELECT_ALL;

    chans = (HAL_CHANNEL *)OS_MALLOC(sc->sc_osdev, 
                      IEEE80211_CHAN_MAX * sizeof(HAL_CHANNEL), 
                      GFP_KERNEL);

    if (chans == NULL) {
        printk("%s: unable to allocate channel table\n", __func__);
        return;
    }
    
    /*
     * Need to contiue setup_channel_list, even if ath_hal_init_channels returns fail.
     * If the new country doesn't allow any channel, we need to update the channel list.
     */
    if (!ath_hal_init_channels(ah, chans, IEEE80211_CHAN_MAX, (u_int *)&nchan,
                           regclassids, ATH_REGCLASSIDS_MAX, &nregclass,
                           countryCode, wMode, AH_FALSE, 1)) {
        nchan = 0;
    }

    if (sc->sc_ieee_ops->setup_channel_list) {
        sc->sc_ieee_ops->setup_channel_list(sc->sc_ieee, CLIST_NEW_COUNTRY,
                                            chans, nchan, regclassids, nregclass,
                                            countryCode);
    }

    OS_FREE(chans);
    return;
}

/******************************************************************************/
/*!
**  \brief Set ATH/HAL parameter value
**
**  Interface routine called by the iwpriv handler to directly set specific
**  HAL parameters.  Parameter ID is stored in the above iw_priv_args structure
**
**  \param dev Net Device pointer, Linux driver handle
**  \param info Request Info Stucture containing type info, etc.
**  \param w Pointer to a word value, not used here
**  \param extra Pointer to the actual data buffer.
**  \return 0 for success
**  \return Non Zero on error
*/

static int ath_iw_setparam(struct net_device *dev,
                           struct iw_request_info *info,
                           void *w,
                           char *extra)
{

    struct ath_softc_net80211 *scn =  dev->priv;
    struct ath_softc          *sc  =  ATH_DEV_TO_SC(scn->sc_dev);
    struct ath_hal            *ah =   sc->sc_ah;
    int *i = (int *) extra;
    int param = i[0];       /* parameter id is 1st */
    int value = i[1];       /* NB: most values are TYPE_INT */
    int retval = 0;
    
    /*
    ** Code Begins
    ** Since the parameter passed is the value of the parameter ID, we can call directly
    */

    if ( param & ATH_PARAM_SHIFT )
    {
        /*
        ** It's an ATH value.  Call the  ATH configuration interface
        */
        
        param -= ATH_PARAM_SHIFT;
        retval = scn->sc_ops->ath_set_config_param(scn->sc_dev,
                                                   (ath_param_ID_t)param,
                                                   &value);
    }
    else if ( param & SPECIAL_PARAM_SHIFT )
    {
        /*
        ** We will only allow the country code to be set if no VAPs have been
        ** created.  This forces the operation to occur only at initial bring
        ** up using the ATH_countrycode variable and the apup script
        */
        
        if( !sc->sc_nvaps )
        {
            retval = scn->sc_ops->set_country(scn->sc_dev,NULL,value);
            forceChannelSet(sc,value);
        }
        else
        {
            retval = -EOPNOTSUPP;
        }
    }
    else
    {
        retval = (int) ath_hal_set_config_param(ah,
                                                (HAL_CONFIG_OPS_PARAMS_TYPE)param,
                                                &value);
    }

    return (retval);    
}

/******************************************************************************/
/*!
**  \brief Returns value of HAL parameter
**
**  This returns the current value of the indicated HAL parameter, used by
**  iwpriv interface.
**
**  \param dev Net Device pointer, Linux driver handle
**  \param info Request Info Stucture containing type info, etc.
**  \param w Pointer to a word value, not used here
**  \param extra Pointer to the actual data buffer.
**  \return 0 for success
**  \return -EOPNOTSUPP for invalue request
*/

static int ath_iw_getparam(struct net_device *dev, struct iw_request_info *info, void *w, char *extra)
{
    struct ath_softc_net80211 *scn  = dev->priv;
    struct ath_softc          *sc   = ATH_DEV_TO_SC(scn->sc_dev);
    struct ath_hal            *ah   = sc->sc_ah;
    int                     param   = *(int *)extra;
    int                      *val   = (int *)extra; 
    int                    retval   = 0;
    
    /*
    ** Code Begins
    ** Since the parameter passed is the value of the parameter ID, we can call directly
    */
                                       

    if ( param & ATH_PARAM_SHIFT )
    {
        /*
        ** It's an ATH value.  Call the  ATH configuration interface
        */
        
        param -= ATH_PARAM_SHIFT;
        if ( scn->sc_ops->ath_get_config_param(scn->sc_dev,(ath_param_ID_t)param,extra) )
        {
            retval = -EOPNOTSUPP;
        }
    }
    else if ( param & SPECIAL_PARAM_SHIFT )
    {
        HAL_COUNTRY_ENTRY         cval;
    
        scn->sc_ops->get_current_country(scn->sc_dev, &cval);
        val[0] = cval.countryCode;
    }
    else
    {
        if ( !ath_hal_get_config_param(ah, (HAL_CONFIG_OPS_PARAMS_TYPE)param, extra) )
        {
            retval = -EOPNOTSUPP;
        }
    }

    return (retval);
}



/******************************************************************************/
/*!
**  \brief Set country
**
**  Interface routine called by the iwpriv handler to directly set specific
**  HAL parameters.  Parameter ID is stored in the above iw_priv_args structure
**
**  \param dev Net Device pointer, Linux driver handle
**  \param info Request Info Stucture containing type info, etc.
**  \param w Pointer to a word value, not used here
**  \param extra Pointer to the actual data buffer.
**  \return 0 for success
**  \return Non Zero on error
*/

static int ath_iw_setcountry(struct net_device *dev,
                           struct iw_request_info *info,
                           void *w,
                           char *extra)
{

    struct ath_softc_net80211 *scn =  dev->priv;
    struct ath_softc          *sc  = ATH_DEV_TO_SC(scn->sc_dev);
    char *p = (char *) extra;
    int retval;
    HAL_COUNTRY_ENTRY         cval;

    retval = scn->sc_ops->set_country(scn->sc_dev,p,0);
    scn->sc_ops->get_current_country(scn->sc_dev, &cval);
    forceChannelSet(sc,cval.countryCode);

    return retval;
}

/******************************************************************************/
/*!
**  \brief Returns country string
**
**  This returns the current value of the indicated HAL parameter, used by
**  iwpriv interface.
**
**  \param dev Net Device pointer, Linux driver handle
**  \param info Request Info Stucture containing type info, etc.
**  \param w Pointer to a word value, not used here
**  \param extra Pointer to the actual data buffer.
**  \return 0 for success
*/

static int ath_iw_getcountry(struct net_device *dev, struct iw_request_info *info, void *w, char *extra)
{
    struct ath_softc_net80211 *scn  = dev->priv;
    struct iw_point           *wri  = (struct iw_point *)w;
    HAL_COUNTRY_ENTRY         cval;
    char                      *str  = (char *)extra;
    
    /*
    ** Code Begins
    */

    scn->sc_ops->get_current_country(scn->sc_dev, &cval);
    str[0] = cval.iso[0];
    str[1] = cval.iso[1];
    str[2] = cval.iso[2];
    str[3] = 0;
    wri->length = 3;
    
    return (0);
}


/*
** iwpriv Handlers
** This table contains the references to the routines that actually get/set
** the parameters in the args table.
*/

static const iw_handler ath_iw_priv_handlers[] = {
    (iw_handler) ath_iw_setparam,          /* SIOCWFIRSTPRIV+0 */
    (iw_handler) ath_iw_getparam,          /* SIOCWFIRSTPRIV+1 */
    (iw_handler) ath_iw_setcountry,        /* SIOCWFIRSTPRIV+2 */
    (iw_handler) ath_iw_getcountry,        /* SIOCWFIRSTPRIV+3 */
};

/*
** Wireless Handler Structure
** This table provides the Wireless tools the interface required to access
** parameters in the HAL.  Each sub table contains the definition of various
** control tables that reference functions this module
*/

static struct iw_handler_def ath_iw_handler_def = {
    .standard           = (iw_handler *) NULL,
    .num_standard       = 0,
    .private            = (iw_handler *) ath_iw_priv_handlers,
    .num_private        = TABLE_SIZE(ath_iw_priv_handlers),
    .private_args       = (struct iw_priv_args *) ath_iw_priv_args,
    .num_private_args   = TABLE_SIZE(ath_iw_priv_args),
    .get_wireless_stats	= NULL,
};

void ath_iw_attach(struct net_device *dev)
{
    dev->wireless_handlers = &ath_iw_handler_def;
}
EXPORT_SYMBOL(ath_iw_attach);


