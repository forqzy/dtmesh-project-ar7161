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
#ifndef _ATH_AR5212_H_
#define _ATH_AR5212_H_

#include "ah_eeprom.h"
#include "ar5212/ar5212radar.h"

#define	AR5212_MAGIC	0x19541014

/* DCU Transmit Filter macros */
#define CALC_MMR(dcu, idx) \
	( (4 * dcu) + (idx < 32 ? 0 : (idx < 64 ? 1 : (idx < 96 ? 2 : 3))) )
#define TXBLK_FROM_MMR(mmr) \
	(AR_D_TXBLK_BASE + ((mmr & 0x1f) << 6) + ((mmr & 0x20) >> 3))
#define CALC_TXBLK_ADDR(dcu, idx)	(TXBLK_FROM_MMR(CALC_MMR(dcu, idx)))
#define CALC_TXBLK_VALUE(idx)		(1 << (idx & 0x1f))

/* MAC register values */

#define INIT_BEACON_CONTROL \
	((INIT_RESET_TSF << 24)  | (INIT_BEACON_EN << 23) | \
	  (INIT_TIM_OFFSET << 16) | INIT_BEACON_PERIOD)

#define INIT_CONFIG_STATUS	0x00000000
#define INIT_RSSI_THR		0x00000781	/* Missed beacon counter initialized to 0x7 (max is 0xff) */
#define INIT_IQCAL_LOG_COUNT_MAX	0xF
#define INIT_BCON_CNTRL_REG	0x00000000

#define INIT_USEC		40
#define HALF_RATE_USEC		19 /* ((40 / 2) - 1 ) */
#define QUARTER_RATE_USEC	9  /* ((40 / 4) - 1 ) */

#define RX_NON_FULL_RATE_LATENCY	63
#define TX_HALF_RATE_LATENCY		108
#define TX_QUARTER_RATE_LATENCY		216

#define IFS_SLOT_FULL_RATE	0x168 /* 9 us half, 40 MHz core clock (9*40) */
#define IFS_SLOT_HALF_RATE	0x104 /* 13 us half, 20 MHz core clock (13*20) */
#define IFS_SLOT_QUARTER_RATE	0xD2 /* 21 us quarter, 10 MHz core clock (21*10) */
#define IFS_EIFS_FULL_RATE	0xE60 /* (74 + (2 * 9)) * 40MHz core clock */
#define IFS_EIFS_HALF_RATE	0xDAC /* (149 + (2 * 13)) * 20MHz core clock */
#define IFS_EIFS_QUARTER_RATE	0xD48 /* (298 + (2 * 21)) * 10MHz core clock */

#define ACK_CTS_TIMEOUT_11A	0x3E8 /* ACK timeout in 11a core clocks */

/* Tx frame start to tx data start delay */
#define TX_FRAME_D_START_HALF_RATE 	0xc
#define TX_FRAME_D_START_QUARTER_RATE 	0xd

/*
 * Various fifo fill before Tx start, in 64-byte units
 * i.e. put the frame in the air while still DMAing
 */
#define MIN_TX_FIFO_THRESHOLD	0x1
#define MAX_TX_FIFO_THRESHOLD	((IEEE80211_MAX_LEN / 64) + 1)
#define INIT_TX_FIFO_THRESHOLD	MIN_TX_FIFO_THRESHOLD

/*
 * Gain support.
 */
#define	NUM_CORNER_FIX_BITS		4
#define	NUM_CORNER_FIX_BITS_5112	7
#define	DYN_ADJ_UP_MARGIN		15
#define	DYN_ADJ_LO_MARGIN		20
#define	PHY_PROBE_CCK_CORRECTION	5
#define	CCK_OFDM_GAIN_DELTA		15

enum GAIN_PARAMS {
	GP_TXCLIP,
	GP_PD90,
	GP_PD84,
	GP_GSEL,
};

enum GAIN_PARAMS_5112 {
	GP_MIXGAIN_OVR,
	GP_PWD_138,
	GP_PWD_137,
	GP_PWD_136,
	GP_PWD_132,
	GP_PWD_131,
	GP_PWD_130,
};

typedef struct _gainOptStep {
	int16_t	paramVal[NUM_CORNER_FIX_BITS_5112];
	int32_t	stepGain;
	int8_t	stepName[16];
} GAIN_OPTIMIZATION_STEP;

typedef struct {
	u_int32_t	numStepsInLadder;
	u_int32_t	defaultStepNum;
	GAIN_OPTIMIZATION_STEP optStep[10];
} GAIN_OPTIMIZATION_LADDER;

typedef struct {
	u_int32_t	currStepNum;
	u_int32_t	currGain;
	u_int32_t	targetGain;
	u_int32_t	loTrig;
	u_int32_t	hiTrig;
	u_int32_t	gainFCorrection;
	u_int32_t	active;
	const GAIN_OPTIMIZATION_STEP *currStep;
} GAIN_VALUES;

/* RF HAL structures */
typedef struct RfHalFuncs {
	void	  (*rfDetach)(struct ath_hal *ah);
	void	  (*writeRegs)(struct ath_hal *,
			u_int modeIndex, u_int freqIndex, int regWrites);
	u_int32_t *(*getRfBank)(struct ath_hal *ah, int bank);
	HAL_BOOL  (*setChannel)(struct ath_hal *, HAL_CHANNEL_INTERNAL *);
	HAL_BOOL  (*setRfRegs)(struct ath_hal *,
		      HAL_CHANNEL_INTERNAL *, u_int16_t modesIndex,
		      u_int16_t *rfXpdGain);
	HAL_BOOL  (*setPowerTable)(struct ath_hal *ah,
		      int16_t *minPower, int16_t *maxPower,
		      HAL_CHANNEL_INTERNAL *, u_int16_t *rfXpdGain);
	HAL_BOOL  (*getChipPowerLim)(struct ath_hal *ah, HAL_CHANNEL *chans,
				       u_int32_t nchancs);
	int16_t	  (*getNfAdjust)(struct ath_hal *, const HAL_CHANNEL_INTERNAL*);
} RF_HAL_FUNCS;

/*
 * Per-channel ANI state private to the driver.
 */
struct ar5212AniState {
	HAL_CHANNEL	c;

	/*
	 * Max spur immunity level needs to be set to different levels
	 * depending on whether we have a venice chip or something later
	 * like Griffin or Cobra, Spider, etc... Not sure if Eagle should
	 * follow venice values or Griffin values....
	 *
	 */

	u_int8_t	maxSpurImmunity;
	u_int8_t	noiseImmunityLevel;
	u_int8_t	spurImmunityLevel;
	u_int8_t	firstepLevel;
	u_int8_t	ofdmWeakSigDetectOff;
	u_int8_t	cckWeakSigThreshold;

	/* Thresholds */
	u_int32_t	listenTime;
	u_int32_t	ofdmTrigHigh;
	u_int32_t	ofdmTrigLow;
	int32_t		cckTrigHigh;
	int32_t		cckTrigLow;
	int32_t		rssiThrLow;
	int32_t		rssiThrHigh;

    /* Current RSSI */
	int32_t		rssi;

	u_int32_t	noiseFloor;	/* The current noise floor */
	u_int32_t	txFrameCount;	/* Last txFrameCount */
	u_int32_t	rxFrameCount;	/* Last rx Frame count */
	u_int32_t	cycleCount;	/* Last cycleCount (can detect wrap-around) */
	u_int32_t	ofdmPhyErrCount;/* OFDM err count since last reset */
	u_int32_t	cckPhyErrCount;	/* CCK err count since last reset */
	u_int32_t	ofdmPhyErrBase;	/* Base value for ofdm err counter */
	u_int32_t	cckPhyErrBase;	/* Base value for cck err counters */
	int16_t		pktRssi[2];	/* Average rssi of pkts for 2 antennas */
	int16_t		ofdmErrRssi[2];	/* Average rssi of ofdm phy errs for 2 ant */
	int16_t		cckErrRssi[2];	/* Average rssi of cck phy errs for 2 ant */
};

#define	HAL_PROCESS_ANI		0x00000001	/* ANI state setup */
#define HAL_RADAR_EN		0x80000000	/* Radar detect is capable */
#define HAL_AR_EN		0x40000000	/* AR detect is capable */

#define DO_ANI(ah) \
	((AH5212(ah)->ah_procPhyErr & HAL_PROCESS_ANI))

struct ar5212Stats {
	u_int32_t	ast_ani_niup;	/* ANI increased noise immunity */
	u_int32_t	ast_ani_nidown;	/* ANI decreased noise immunity */
	u_int32_t	ast_ani_spurup;	/* ANI increased spur immunity */
	u_int32_t	ast_ani_spurdown;/* ANI descreased spur immunity */
	u_int32_t	ast_ani_ofdmon;	/* ANI OFDM weak signal detect on */
	u_int32_t	ast_ani_ofdmoff;/* ANI OFDM weak signal detect off */
	u_int32_t	ast_ani_cckhigh;/* ANI CCK weak signal threshold high */
	u_int32_t	ast_ani_ccklow;	/* ANI CCK weak signal threshold low */
	u_int32_t	ast_ani_stepup;	/* ANI increased first step level */
	u_int32_t	ast_ani_stepdown;/* ANI decreased first step level */
	u_int32_t	ast_ani_ofdmerrs;/* ANI cumulative ofdm phy err count */
	u_int32_t	ast_ani_cckerrs;/* ANI cumulative cck phy err count */
	u_int32_t	ast_ani_reset;	/* ANI parameters zero'd for non-STA */
	u_int32_t	ast_ani_lzero;	/* ANI listen time forced to zero */
	u_int32_t	ast_ani_lneg;	/* ANI listen time calculated < 0 */
	HAL_MIB_STATS	ast_mibstats;	/* MIB counter stats */
	HAL_NODE_STATS	ast_nodestats;	/* Latest rssi stats from driver */
};

struct ath_hal_5212 {
	struct ath_hal_private	ah_priv;	/* base class */

	/*
	 * Information retrieved from EEPROM.
	 */
	HAL_EEPROM	ah_eeprom;

	GAIN_VALUES	ah_gainValues;

	u_int8_t	ah_macaddr[IEEE80211_ADDR_LEN];
	u_int8_t	ah_bssid[IEEE80211_ADDR_LEN];
	u_int8_t	ah_bssidmask[IEEE80211_ADDR_LEN];
        u_int16_t       ah_assocId;
	
	int16_t		ah_curchanRadIndex;	/* cur. channel radar index */

	/*
	 * Runtime state.
	 */
	u_int32_t	ah_maskReg;		/* copy of AR_IMR */
	struct ar5212Stats ah_stats;		/* various statistics */
	RF_HAL_FUNCS	ah_rfHal;
	u_int32_t	ah_txDescMask;		/* mask for TXDESC */
	u_int32_t	ah_txOkInterruptMask;
	u_int32_t	ah_txErrInterruptMask;
	u_int32_t	ah_txDescInterruptMask;
	u_int32_t	ah_txEolInterruptMask;
	u_int32_t	ah_txUrnInterruptMask;
	HAL_TX_QUEUE_INFO ah_txq[HAL_NUM_TX_QUEUES];
	HAL_POWER_MODE	ah_powerMode;
	HAL_BOOL	ah_chipFullSleep;
	u_int32_t	ah_atimWindow;
	HAL_ANT_SETTING ah_diversityControl;	/* antenna setting */
	enum {
		IQ_CAL_INACTIVE,
		IQ_CAL_RUNNING,
		IQ_CAL_DONE
	} ah_bIQCalibration;			/* IQ calibrate state */
	HAL_RFGAIN	ah_rfgainState;		/* RF gain calibrartion state */
	u_int32_t	ah_tx6PowerInHalfDbm;	/* power output for 6Mb tx */
	u_int32_t	ah_staId1Defaults;	/* STA_ID1 default settings */
	u_int32_t	ah_miscMode;		/* MISC_MODE settings */
	HAL_BOOL	ah_tpcEnabled;		/* per-packet tpc enabled */
	u_int32_t	ah_macTPC;		/* tpc register */
	u_int32_t	ah_beaconInterval;	/* XXX */
	enum {
		AUTO_32KHZ,		/* use it if 32kHz crystal present */
		USE_32KHZ,		/* do it regardless */
		DONT_USE_32KHZ,		/* don't use it regardless */
	} ah_enable32kHzClock;			/* whether to sleep at 32kHz */
	void		*ah_analogBanks;	/* RF register banks */
	u_int32_t	ah_ofdmTxPower;
	int16_t		ah_txPowerIndexOffset;

	u_int		ah_slottime;		/* user-specified slot time */
	u_int		ah_acktimeout;		/* user-specified ack timeout */
	u_int		ah_ctstimeout;		/* user-specified cts timeout */
	/*
	 * XXX
	 * 11g-specific stuff; belongs in the driver.
	 */
	u_int8_t	ah_gBeaconRate;		/* fixed rate for G beacons */
	/*
	 * RF Silent handling; setup according to the EEPROM.
	 */
	u_int32_t	ah_gpioSelect;		/* GPIO pin to use */
	u_int32_t	ah_polarity;		/* polarity to disable RF */
	u_int32_t	ah_gpioBit;		/* after init, prev value */
	HAL_BOOL	ah_eepEnabled;		/* EEPROM bit for capability */
	/*
	 * ANI & Radar support.
	 */
	u_int32_t	ah_procPhyErr;		/* Process Phy errs */
	HAL_BOOL	ah_hasHwPhyCounters;	/* Hardware has phy counters */
	u_int32_t	ah_aniPeriod;		/* ani update list period */
	struct ar5212AniState	*ah_curani;	/* cached last reference */
	struct ar5212AniState	ah_ani[64];	/* per-channel state */
	struct ar5212RadarState	ah_radar[HAL_NUM_RADAR_STATES];	/* Per-Channel Radar detector state */
	struct ar5212ArState	ah_ar;		/* AR detector state */
	struct ar5212RadarQElem	*ah_arq;	/* AR event queue */
	struct ar5212RadarQInfo	ah_arqInfo;	/* AR event q read/write state */
#ifdef AH_SUPPORT_DFS
	struct ar5212RadarQElem	*ah_radarq;	/* radar event queue */
	struct ar5212RadarQInfo	ah_radarqInfo;	/* radar event q read/write state */
	struct ar5212RadarFilterType *ah_radarf[HAL_MAX_RADAR_TYPES]; /* One filter for each radar pulse type */
	struct ar5212RadarInfo ah_rinfo;	/* State variables for radar processing */
	struct ar5212Bin5Radars *ah_b5radars;   /* array of bin5 radars events*/
	int8_t **ah_radarTable;                 /* Mapping of radar durations to filter types*/	
	struct ar5212RadarNolElem *ah_radarNol;	/* Non occupancy list for radar */
	u_int8_t	ah_dfsDisabled;		/* Is the HAL disabled b/c of lack of dfs processing */
#define	HAL_DFS_DISABLE		0x01		/* Disable HAL */
#define	HAL_DFS_TIMEMARK	0x02		/* Time of first dfs event already marked */

	u_int64_t	ah_dfsMarkTime;
#endif

	/*
	 * Ani tables that change between the 5212 and 5312.
	 * These get set at attach time.
	 * XXX don't belong here
	 * XXX need better explanation
	 */
        int		ah_totalSizeDesired[5];
        int		ah_coarseHigh[5];
        int		ah_coarseLow[5];
        int		ah_firpwr[5];

	/*
	 * Transmit power state.  Note these are maintained
	 * here so they can be retrieved by diagnostic tools.
	 */
	u_int16_t	*ah_pcdacTable;
	u_int		ah_pcdacTableSize;
	u_int16_t	ah_ratesArray[16];
    void        *ah_vpdTable;

	/*
	 * Tx queue interrupt state.
	 */
	u_int32_t	ah_intrTxqs;
	HAL_BOOL	ah_xrEnable;
};

#define	AH5212(_ah)	((struct ath_hal_5212 *)(_ah))

#define	IS_5112(ah) \
	((AH_PRIVATE(ah)->ah_analog5GhzRev&0xf0) >= AR_RAD5112_SREV_MAJOR \
	 && (AH_PRIVATE(ah)->ah_analog5GhzRev&0xf0) < AR_RAD2316_SREV_MAJOR )
#define IS_5212(ah) \
    ((ah)->ah_macVersion == AR_SREV_VERSION_VENICE)
/* Hainan card and up */
/* XXX: This macro, ported from release 5.2, is comparing ah_macRev against */
/*      version constant AR_SREV_VERSION_VENICE. Is this correct?? */
#define IS_5112_REV5_UP(ah) \
    (((AH_PRIVATE(ah)->ah_macVersion) > AR_SREV_VERSION_VENICE) || \
    (IS_5212(ah) && ((AH_PRIVATE(ah)->ah_macRev) >= AR_SREV_VERSION_VENICE)))
#define	IS_RAD5112_REV1(ah) \
	(((AH_PRIVATE(ah)->ah_analog5GhzRev&0xf0) == AR_RAD5112_SREV_MAJOR) && \
	((AH_PRIVATE(ah)->ah_analog5GhzRev&0x0f) < (AR_RAD5112_SREV_2_0&0x0f)))
#define IS_RADX112_REV2(ah) \
        (IS_5112(ah) && \
	  ((AH_PRIVATE(ah)->ah_analog5GhzRev == AR_RAD5112_SREV_2_0) || \
	   (AH_PRIVATE(ah)->ah_analog5GhzRev == AR_RAD2112_SREV_2_0) || \
	   (AH_PRIVATE(ah)->ah_analog5GhzRev == AR_RAD2112_SREV_2_1) || \
	   (AH_PRIVATE(ah)->ah_analog5GhzRev == AR_RAD5112_SREV_2_1)))
#define	IS_5312_2_X(ah) \
	(((AH_PRIVATE(ah)->ah_macVersion) == AR_SREV_VERSION_VENICE) && \
	 (((AH_PRIVATE(ah)->ah_macRev) == 2) || ((AH_PRIVATE(ah)->ah_macRev) == 7)))
#define	IS_2316(ah) \
	((AH_PRIVATE(ah)->ah_macVersion) == AR_SREV_2415)
#define	IS_2413(ah) \
	((AH_PRIVATE(ah)->ah_macVersion) == AR_SREV_2413 || IS_2316(ah))
#define IS_5424(ah) \
 	( (AH_PRIVATE(ah)->ah_macVersion) == AR_SREV_5424 || \
 	( (AH_PRIVATE(ah)->ah_macVersion) == AR_SREV_5413 && \
 	  (AH_PRIVATE(ah)->ah_macRev) <=3 && \
 	  (AH_PRIVATE(ah)->ah_isPciExpress) == AH_TRUE) )
#define IS_5413(ah) \
	(((AH_PRIVATE(ah)->ah_macVersion) == AR_SREV_5413) || IS_5424(ah))
#define IS_2425(ah) \
	((AH_PRIVATE(ah)->ah_macVersion) == AR_SREV_2425)
#define IS_HB63(ah)	\
    ((AH_PRIVATE(ah)->ah_flags) & AH_IS_HB63)
#define IS_2417(ah) \
        ((AH_PRIVATE(ah)->ah_macVersion) == AR_SREV_2417)

#define IS_PCIE(ah) (IS_5424((ah)) || IS_2425((ah)))

#define	ar5212RfDetach(ah) do {				\
	if (AH5212(ah)->ah_rfHal.rfDetach != AH_NULL)	\
		AH5212(ah)->ah_rfHal.rfDetach(ah);	\
} while (0)
#define	ar5212GetRfBank(ah, b) \
	AH5212(ah)->ah_rfHal.getRfBank(ah, b)

#define CHECK_CCK(_ah, _chan, _flag) do {                       \
    if ((AH_PRIVATE(_ah)->ah_macVersion) == AR_SREV_2425 ||     \
        (AH_PRIVATE(_ah)->ah_macVersion) == AR_SREV_2417) {     \
            if (((_chan)->channelFlags) & CHANNEL_CCK)  {       \
                    ((_chan)->channelFlags) &= (~CHANNEL_CCK);  \
                    ((_chan)->channelFlags) |= CHANNEL_OFDM;    \
                    (_flag) = AH_TRUE;                          \
            }                                                   \
   }                                                            \
} while (0)

#define UNCHECK_CCK(_ah, _chan, _flag) do {                     \
    if ((AH_PRIVATE(_ah)->ah_macVersion) == AR_SREV_2425 ||     \
        (AH_PRIVATE(_ah)->ah_macVersion) == AR_SREV_2417) {     \
            if ((_flag) == AH_TRUE) {                           \
                    ((_chan)->channelFlags) &= (~CHANNEL_OFDM); \
                    ((_chan)->channelFlags) |= CHANNEL_CCK;     \
                    (_flag) = AH_FALSE;                         \
            }                                                   \
   }                                                            \
} while (0)

extern	HAL_BOOL ar5111RfAttach(struct ath_hal *, HAL_STATUS *);
extern	HAL_BOOL ar5112RfAttach(struct ath_hal *, HAL_STATUS *);
extern	HAL_BOOL ar2413RfAttach(struct ath_hal *, HAL_STATUS *);
extern  HAL_BOOL ar5413RfAttach(struct ath_hal *, HAL_STATUS *);
extern	HAL_BOOL ar2316RfAttach(struct ath_hal *, HAL_STATUS *);
extern	HAL_BOOL ar2317RfAttach(struct ath_hal *, HAL_STATUS *);
extern	HAL_BOOL ar2425RfAttach(struct ath_hal *, HAL_STATUS *);

struct ath_hal;

extern	u_int32_t ar5212GetRadioRev(struct ath_hal *ah);
extern	struct ath_hal_5212 * ar5212NewState(u_int16_t devid,
        HAL_ADAPTER_HANDLE osdev, HAL_SOFTC sc,
		HAL_BUS_TAG st, HAL_BUS_HANDLE sh, HAL_BUS_TYPE bustype,
		struct hal_reg_parm *hal_conf_parm, HAL_STATUS *status);
extern struct ath_hal * ar5212Attach(u_int16_t devid,
        HAL_ADAPTER_HANDLE osdev, HAL_SOFTC sc,
		HAL_BUS_TAG st, HAL_BUS_HANDLE sh, HAL_BUS_TYPE bustype, 
		struct hal_reg_parm *hal_conf_parm, HAL_STATUS *status);
extern	void ar5212Detach(struct ath_hal *ah);
extern  HAL_BOOL ar5212ChipTest(struct ath_hal *ah);
extern  HAL_BOOL ar5212GetChannelEdges(struct ath_hal *ah,
                u_int16_t flags, u_int16_t *low, u_int16_t *high);
extern	HAL_BOOL ar5212FillCapabilityInfo(struct ath_hal *ah);

extern	void ar5212BeaconInit(struct ath_hal *ah,
		u_int32_t next_beacon, u_int32_t beacon_period);
extern	HAL_BOOL ar5212WaitForBeaconDone(struct ath_hal *, HAL_BUS_ADDR baddr);
extern	void ar5212ResetStaBeaconTimers(struct ath_hal *ah);
extern	void ar5212SetStaBeaconTimers(struct ath_hal *ah,
		const HAL_BEACON_STATE *);

extern	HAL_BOOL ar5212IsInterruptPending(struct ath_hal *ah);
extern	HAL_BOOL ar5212GetPendingInterrupts(struct ath_hal *ah, HAL_INT *);
extern	HAL_INT ar5212GetInterrupts(struct ath_hal *ah);
extern	HAL_INT ar5212SetInterrupts(struct ath_hal *ah, HAL_INT ints);

extern	u_int32_t ar5212GetKeyCacheSize(struct ath_hal *);
extern  u_int32_t  ar5212GetMacVersion(struct ath_hal *ah);
extern	HAL_BOOL ar5212IsKeyCacheEntryValid(struct ath_hal *, u_int16_t entry);
extern	HAL_BOOL ar5212ResetKeyCacheEntry(struct ath_hal *ah, u_int16_t entry);
extern	HAL_BOOL ar5212SetKeyCacheEntryMac(struct ath_hal *,
			u_int16_t entry, const u_int8_t *mac);
extern	HAL_BOOL ar5212SetKeyCacheEntry(struct ath_hal *ah, u_int16_t entry,
                       const HAL_KEYVAL *k, const u_int8_t *mac, int xorKey);

extern	void ar5212GetMacAddress(struct ath_hal *ah, u_int8_t *mac);
extern	HAL_BOOL ar5212SetMacAddress(struct ath_hal *ah, const u_int8_t *);
extern	void ar5212GetBssIdMask(struct ath_hal *ah, u_int8_t *mac);
extern	HAL_BOOL ar5212SetBssIdMask(struct ath_hal *, const u_int8_t *);
extern	HAL_BOOL ar5212EepromRead(struct ath_hal *, u_int off, u_int16_t *data);
extern	HAL_BOOL ar5212EepromWrite(struct ath_hal *, u_int off, u_int16_t data);
extern	HAL_BOOL ar5212SetRegulatoryDomain(struct ath_hal *ah,
		u_int16_t regDomain, HAL_STATUS *stats);
extern	u_int ar5212GetWirelessModes(struct ath_hal *ah);
extern	void ar5212EnableRfKill(struct ath_hal *);
extern	HAL_BOOL ar5212GpioCfgOutput(struct ath_hal *, u_int32_t gpio, HAL_GPIO_OUTPUT_MUX_TYPE signalType);
extern	HAL_BOOL ar5212GpioCfgInput(struct ath_hal *, u_int32_t gpio);
extern	HAL_BOOL ar5212GpioSet(struct ath_hal *, u_int32_t gpio, u_int32_t val);
extern	u_int32_t ar5212GpioGet(struct ath_hal *ah, u_int32_t gpio);
extern	void ar5212GpioSetIntr(struct ath_hal *ah, u_int, u_int32_t ilevel);
extern	void ar5212SetLedState(struct ath_hal *ah, HAL_LED_STATE state);
extern	void ar5212SetPowerLedState(struct ath_hal *ah, u_int8_t enable);
extern	void ar5212SetNetworkLedState(struct ath_hal *ah, u_int8_t enable);
extern	void ar5212WriteAssocid(struct ath_hal *ah, const u_int8_t *bssid,
		u_int16_t assocId);
extern	u_int32_t ar5212GetTsf32(struct ath_hal *ah);
extern	u_int64_t ar5212GetTsf64(struct ath_hal *ah);
extern	void ar5212ResetTsf(struct ath_hal *ah);
extern	void ar5212SetBasicRate(struct ath_hal *ah, HAL_RATE_SET *pSet);
extern	u_int32_t ar5212GetRandomSeed(struct ath_hal *ah);
extern	HAL_BOOL ar5212DetectCardPresent(struct ath_hal *ah);
extern	void ar5212UpdateMibMacStats(struct ath_hal *ah);
extern  void ar5212GetMibMacStats(struct ath_hal *ah, HAL_MIB_STATS* stats);
extern	HAL_BOOL ar5212IsJapanChannelSpreadSupported(struct ath_hal *ah);
extern	u_int32_t ar5212GetCurRssi(struct ath_hal *ah);
extern	u_int ar5212GetDefAntenna(struct ath_hal *ah);
extern	void ar5212SetDefAntenna(struct ath_hal *ah, u_int antenna);
extern	HAL_BOOL ar5212SetAntennaSwitch(struct ath_hal *ah,
		HAL_ANT_SETTING settings, HAL_CHANNEL *chan, u_int8_t *, u_int8_t *, u_int8_t *);
extern	HAL_BOOL ar5212IsSleepAfterBeaconBroken(struct ath_hal *ah);
extern	HAL_BOOL ar5212SetSlotTime(struct ath_hal *, u_int);
extern	u_int ar5212GetSlotTime(struct ath_hal *);
extern	HAL_BOOL ar5212SetAckTimeout(struct ath_hal *, u_int);
extern	u_int ar5212GetAckTimeout(struct ath_hal *);
extern	HAL_BOOL ar5212SetCTSTimeout(struct ath_hal *, u_int);
extern	u_int ar5212GetCTSTimeout(struct ath_hal *);
extern  HAL_BOOL ar5212SetDecompMask(struct ath_hal *, u_int16_t, int);
void 	ar5212SetCoverageClass(struct ath_hal *, u_int8_t, int);
extern  HAL_STATUS ar5212SetQuiet(struct ath_hal *ah,u_int16_t period, u_int16_t duration, u_int16_t nextStart, u_int16_t enabled);
extern	void ar5212SetPCUConfig(struct ath_hal *);
extern	HAL_BOOL ar5212Use32KHzclock(struct ath_hal *ah, HAL_OPMODE opmode);
extern	void ar5212SetupClock(struct ath_hal *ah, HAL_OPMODE opmode);
extern	void ar5212RestoreClock(struct ath_hal *ah, HAL_OPMODE opmode);
extern	int16_t ar5212GetNfAdjust(struct ath_hal *,
		const HAL_CHANNEL_INTERNAL *);
extern	HAL_STATUS ar5212GetCapability(struct ath_hal *, HAL_CAPABILITY_TYPE,
		u_int32_t, u_int32_t *);
extern	HAL_BOOL ar5212SetCapability(struct ath_hal *, HAL_CAPABILITY_TYPE,
		u_int32_t, u_int32_t, HAL_STATUS *);
extern	HAL_BOOL ar5212GetDiagState(struct ath_hal *ah, int request,
		const void *args, u_int32_t argsize,
		void **result, u_int32_t *resultsize);
extern void ar5212GetDescInfo(struct ath_hal *ah, HAL_DESC_INFO *desc_info);
extern  HAL_STATUS ar5212SelectAntConfig(struct ath_hal *ah, u_int32_t cfg);

extern	HAL_BOOL ar5212SetPowerMode(struct ath_hal *ah, HAL_POWER_MODE mode,
		int setChip);
extern	HAL_POWER_MODE ar5212GetPowerMode(struct ath_hal *ah);
extern	void ar5212SetSmPowerMode(struct ath_hal *ah, HAL_SMPS_MODE mode);
#if ATH_WOW
extern  void ar5212WowApplyPattern(struct ath_hal *ah, u_int8_t *pAthPattern, u_int8_t *pAthMask, int32_t pattern_count, u_int32_t athPatternLen);
extern  u_int32_t ar5212WowWakeUp(struct ath_hal *ah);
extern  HAL_BOOL ar5212WowEnable(struct ath_hal *ah, u_int32_t patternEnable);
#endif
extern	HAL_BOOL ar5212GetPowerStatus(struct ath_hal *ah);

extern	u_int32_t ar5212GetRxDP(struct ath_hal *ath);
extern	void ar5212SetRxDP(struct ath_hal *ah, u_int32_t rxdp);
extern	void ar5212EnableReceive(struct ath_hal *ah);
extern	HAL_BOOL ar5212StopDmaReceive(struct ath_hal *ah);
extern	void ar5212StartPcuReceive(struct ath_hal *ah);
extern	void ar5212StopPcuReceive(struct ath_hal *ah);
extern	void ar5212SetMulticastFilter(struct ath_hal *ah,
		u_int32_t filter0, u_int32_t filter1);
extern	HAL_BOOL ar5212ClrMulticastFilterIndex(struct ath_hal *, u_int32_t ix);
extern	HAL_BOOL ar5212SetMulticastFilterIndex(struct ath_hal *, u_int32_t ix);
extern	u_int32_t ar5212GetRxFilter(struct ath_hal *ah);
extern	void ar5212SetRxFilter(struct ath_hal *ah, u_int32_t bits);
extern	HAL_BOOL ar5212SetRxAbort(struct ath_hal *ah, HAL_BOOL);
extern	HAL_BOOL ar5212SetupRxDesc(struct ath_hal *,
		struct ath_desc *, u_int32_t size, u_int flags);
extern	HAL_STATUS ar5212ProcRxDesc(struct ath_hal *ah, struct ath_desc *,
		u_int32_t, struct ath_desc *, u_int64_t);

extern HAL_BOOL ar5212Reset(struct ath_hal *ah, HAL_OPMODE opmode, HAL_CHANNEL *chan, HAL_HT_MACMODE macmode, u_int8_t txchainmask, u_int8_t rxchainmask, HAL_HT_EXTPROTSPACING extprotspacing, HAL_BOOL bChannelChange, HAL_STATUS *status);

extern	void ar5212SetOperatingMode(struct ath_hal *ah, int opmode);
extern	HAL_BOOL ar5212PhyDisable(struct ath_hal *ah);
extern	HAL_BOOL ar5212Disable(struct ath_hal *ah);
extern	HAL_BOOL ar5212ChipReset(struct ath_hal *ah, HAL_CHANNEL *);
extern  HAL_BOOL ar5212PerCalibration(struct ath_hal *ah,  HAL_CHANNEL *chan,
        u_int8_t rxchainmask, HAL_BOOL longcal, HAL_BOOL *isIQdone);
extern  void ar5212ResetCalValid(struct ath_hal *ah,  HAL_CHANNEL *chan,
                                 HAL_BOOL *isIQdone);
extern	int16_t ar5212GetNoiseFloor(struct ath_hal *ah);
extern	HAL_BOOL ar5212SetTxPowerLimit(struct ath_hal *ah, u_int32_t limit, u_int16_t tpcInDb);
extern	void ar5212InitializeGainValues(struct ath_hal *);
extern	HAL_RFGAIN ar5212GetRfgain(struct ath_hal *ah);

extern	HAL_BOOL ar5212UpdateTxTrigLevel(struct ath_hal *,
		HAL_BOOL IncTrigLevel);
extern  u_int16_t ar5212GetTxTrigLevel(struct ath_hal *);
extern  HAL_BOOL ar5212SetTxQueueProps(struct ath_hal *ah, int q,
		const HAL_TXQ_INFO *qInfo);
extern	HAL_BOOL ar5212GetTxQueueProps(struct ath_hal *ah, int q,
		HAL_TXQ_INFO *qInfo);
extern	int ar5212SetupTxQueue(struct ath_hal *ah, HAL_TX_QUEUE type,
		const HAL_TXQ_INFO *qInfo);
extern	HAL_BOOL ar5212ReleaseTxQueue(struct ath_hal *ah, u_int q);
extern	HAL_BOOL ar5212ResetTxQueue(struct ath_hal *ah, u_int q);
extern	u_int32_t ar5212GetTxDP(struct ath_hal *ah, u_int q);
extern	HAL_BOOL ar5212SetTxDP(struct ath_hal *ah, u_int q, u_int32_t txdp);
extern	HAL_BOOL ar5212StartTxDma(struct ath_hal *ah, u_int q);
extern	u_int32_t ar5212NumTxPending(struct ath_hal *ah, u_int q);
extern	HAL_BOOL ar5212StopTxDma(struct ath_hal *ah, u_int q);
extern  HAL_BOOL ar5212UpdateCTSForBursting(struct ath_hal *, struct ath_desc *,
		 struct ath_desc *,struct ath_desc *, struct ath_desc *,
		 u_int32_t, u_int32_t);
extern	HAL_BOOL ar5212SetupTxDesc(struct ath_hal *ah, struct ath_desc *ds,
		u_int pktLen, u_int hdrLen, HAL_PKT_TYPE type, u_int txPower,
		u_int txRate0, u_int txTries0,
		u_int keyIx, u_int antMode, u_int flags,
		u_int rtsctsRate, u_int rtsctsDuration,
		u_int compicvLen, u_int compivLen, u_int comp);
extern	HAL_BOOL ar5212SetupXTxDesc(struct ath_hal *, struct ath_desc *,
		u_int txRate1, u_int txRetries1,
		u_int txRate2, u_int txRetries2,
		u_int txRate3, u_int txRetries3);
extern	HAL_BOOL ar5212FillTxDesc(struct ath_hal *ah, struct ath_desc *ds,
		u_int segLen, HAL_BOOL firstSeg, HAL_BOOL lastSeg,
		const struct ath_desc *ds0);
extern	void ar5212ClearTxDescStatus(struct ath_hal *ah, struct ath_desc *ds);
#ifdef ATH_SWRETRY
extern	void ar5212ClearDestMask(struct ath_hal *ah, struct ath_desc *ds);
#endif
extern	HAL_STATUS ar5212ProcTxDesc(struct ath_hal *ah, struct ath_desc *);
extern  u_int32_t ar5212CalcTxAirtime(struct ath_hal *ah, struct ath_desc *);
extern  void ar5212GetTxIntrQueue(struct ath_hal *ah, u_int32_t *);
extern  void ar5212IntrReqTxDesc(struct ath_hal *ah, struct ath_desc *);
#ifdef AH_PRIVATE_DIAG
extern	void ar5212_ContTxMode(struct ath_hal *, struct ath_desc *, int mode);
#endif

extern	const HAL_RATE_TABLE *ar5212GetRateTable(struct ath_hal *, u_int mode);

extern	void ar5212EnableMIBCounters(struct ath_hal *);
extern	void ar5212DisableMIBCounters(struct ath_hal *);
extern	void ar5212AniAttach(struct ath_hal *);
extern	void ar5212AniDetach(struct ath_hal *);
extern	struct ar5212AniState *ar5212AniGetCurrentState(struct ath_hal *);
extern	struct ar5212Stats *ar5212AniGetCurrentStats(struct ath_hal *);
extern	HAL_BOOL ar5212AniControl(struct ath_hal *, HAL_ANI_CMD cmd, int param);
struct ath_rx_status;
extern	void ar5212AniPhyErrReport(struct ath_hal *ah,
		const struct ath_rx_status *rs);
extern	void ar5212ProcessMibIntr(struct ath_hal *, const HAL_NODE_STATS *);
extern	void ar5212AniArPoll(struct ath_hal *, const HAL_NODE_STATS *,
			     HAL_CHANNEL *);
extern	void ar5212AniReset(struct ath_hal *, int isRestore);

/* DFS declarations*/

extern  void ar5212CheckDfs(struct ath_hal *ah, HAL_CHANNEL *chan);
extern  void ar5212DfsFound(struct ath_hal *ah, HAL_CHANNEL *chan, u_int64_t nolTime);
extern  void ar5212EnableDfs(struct ath_hal *ah, HAL_PHYERR_PARAM *pe);
extern  void ar5212GetDfsThresh(struct ath_hal *ah, HAL_PHYERR_PARAM *pe);
extern  HAL_BOOL ar5212RadarWait(struct ath_hal *ah, HAL_CHANNEL *chan);
extern struct dfs_pulse * ar5212GetDfsRadars(struct ath_hal *ah, u_int32_t dfsdomain, int *numradars, struct dfs_bin5pulse **bin5pulses, int *numb5radars, HAL_PHYERR_PARAM *pe);
extern HAL_CHANNEL *ar5212GetExtensionChannel(struct ath_hal *ah);
extern void ar5212MarkPhyInactive(struct ath_hal *ah);
extern HAL_BOOL ar5212IsFastClockEnabled(struct ath_hal *ah);

extern  void ar5212TxEnable(struct ath_hal *ah,HAL_BOOL enable);

#if defined(AH_PRIVATE_DIAG) && defined(AH_SUPPORT_DFS)
extern	HAL_BOOL ar5212SetRadarThresholds(struct ath_hal *ah, const u_int32_t threshType,
					  const u_int32_t value);
extern	HAL_BOOL ar5212GetRadarThresholds(struct ath_hal *ah, struct ar5212RadarState *thresh);
#endif

extern	void ar5212XrEnable(struct ath_hal *ah);
extern	void ar5212XrDisable(struct ath_hal *ah);


/* 11n specific declarations. Unused in ar5212 */
extern void ar5212Set11nTxDesc(struct ath_hal *ah, struct ath_desc *ds, u_int pktLen, HAL_PKT_TYPE type, u_int txPower, u_int keyIx, HAL_KEY_TYPE keyType, u_int flags);
extern void ar5212Set11nRateScenario(struct ath_hal *ah, struct ath_desc *ds, struct ath_desc *lastds, u_int durUpdateEn, u_int rtsctsRate, u_int rtsctsDuration, HAL_11N_RATE_SERIES series[], u_int nseries, u_int flags);
extern void ar5212Set11nAggrFirst(struct ath_hal *ah, struct ath_desc *ds, u_int aggrLen);
extern void ar5212Set11nAggrMiddle(struct ath_hal *ah, struct ath_desc *ds, u_int numDelims);
extern void ar5212Set11nAggrLast(struct ath_hal *ah, struct ath_desc *ds);
extern void ar5212Clear11nAggr(struct ath_hal *ah, struct ath_desc *ds);
extern void ar5212Set11nRifsBurstMiddle(struct ath_hal *ah, struct ath_desc *ds);
extern void ar5212Set11nRifsBurstLast(struct ath_hal *ah, struct ath_desc *ds);
extern void ar5212Clr11nRifsBurst(struct ath_hal *ah, struct ath_desc *ds);
extern void ar5212Set11nAggrRifsBurst(struct ath_hal *ah, struct ath_desc *ds);
extern HAL_BOOL ar5212Set11nRxRifs(struct ath_hal *ah, HAL_BOOL enable);
extern HAL_BOOL ar5212Get11nRxRifs(struct ath_hal *ah);
extern HAL_BOOL ar5212DetectBbHang(struct ath_hal *ah);
extern HAL_BOOL ar5212DetectMacHang(struct ath_hal *ah);
extern void ar5212Set11nBurstDuration(struct ath_hal *ah, struct ath_desc *ds, u_int burstDuration);
extern void ar5212Set11nVirtMoreFrag(struct ath_hal *ah, struct ath_desc *ds, u_int vmf);
extern u_int32_t ar5212Get11nExtBusy(struct ath_hal *ah);
extern void ar5212Set11nMac2040(struct ath_hal *ah, HAL_HT_MACMODE);
extern HAL_HT_RXCLEAR ar5212Get11nRxClear(struct ath_hal *ah);
extern void ar5212Set11nRxClear(struct ath_hal *ah, HAL_HT_RXCLEAR rxclear);
extern int ar5212Get11nHwPlatform(struct ath_hal *ah);
extern u_int32_t ar5212GetMibCycleCountsPct(struct ath_hal *, u_int32_t*, u_int32_t*, u_int32_t*);
extern void ar5212DmaRegDump(struct ath_hal *);
extern u_int32_t ar5212PpmGetRssiDump(struct ath_hal *);
extern u_int32_t ar5212PpmArmTrigger(struct ath_hal *);
extern int ar5212PpmGetTrigger(struct ath_hal *);
extern u_int32_t ar5212PpmForce(struct ath_hal *);
extern void ar5212PpmUnForce(struct ath_hal *);
extern u_int32_t ar5212PpmGetForceState(struct ath_hal *);
extern int ar5212GetSpurInfo(struct ath_hal * ah, int *enable, int len, u_int16_t *freq);
extern int ar5212SetSpurInfo(struct ath_hal * ah, int enable, int len, u_int16_t *freq);
extern void ar5212SetIntrMitigationTimer(struct ath_hal* ah,
       HAL_INT_MITIGATION reg, u_int32_t value);
extern u_int32_t ar5212GetIntrMitigationTimer(struct ath_hal* ah,
       HAL_INT_MITIGATION reg);
#ifdef ATH_CCX
extern void ar5212GetMibCycleCounts(struct ath_hal *, HAL_COUNTERS*);
extern void ar5212ClearMibCounters(struct ath_hal *ah);
extern u_int8_t ar5212GetCcaThreshold(struct ath_hal *ah);
#endif
#ifdef ATH_BT_COEX
extern void ar5212GetBTCoexInfo(struct ath_hal *ah, HAL_BT_COEX_INFO *btinfo);
extern void ar5212BTCoexConfig(struct ath_hal *ah, HAL_BT_COEX_CONFIG *btconf);
extern void ar5212BTCoexSetQcuThresh(struct ath_hal *ah, int qnum);
extern void ar5212BTCoexSetWeights(struct ath_hal *ah, u_int32_t bt_weight, u_int32_t wlan_weight);
extern void ar5212BTCoexSetupBmissThresh(struct ath_hal *ah, u_int32_t thresh);
extern void ar5212BTCoexDisable(struct ath_hal *ah);
extern int ar5212BTCoexEnable(struct ath_hal *ah);
#endif
extern void ar5212SetImmunity(struct ath_hal *ah, HAL_BOOL enable);
extern void ar5212GetHwHangs(struct ath_hal *ah, hal_hw_hangs_t *hangs);
#endif	/* _ATH_AR5212_H_ */
