/*
 * Copyright (c) 2002-2006 Sam Leffler, Errno Consulting
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
 * $Id: //depot/sw/releases/7.3_AP/wlan/linux/tools/radartool.c#2 $
 */

#include <sys/ioctl.h>
#include <string.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>

#include "if_athioctl.h"
#define _LINUX_TYPES_H
typedef void *va_list;
#include "ah.h"
#include "dfs_ioctl.h"
#include "ah_devid.h"
#include "ah_internal.h"
#include "ar5212/ar5212.h"
#include "ar5212/ar5212reg.h"

#ifndef ATH_DEFAULT
#define	ATH_DEFAULT	"wifi0"
#endif

struct radarhandler {
	int	s;
	struct ath_diag atd;
};

static int
radarShowNol(struct radarhandler *radar)
{
	u_int32_t result;
    struct ifreq ifr;

	radar->atd.ad_id = DFS_SHOW_NOL | ATH_DIAG_DYN;
	radar->atd.ad_out_data = NULL;
	radar->atd.ad_out_size = 0;
	radar->atd.ad_in_data = (void *) &result;
	radar->atd.ad_in_size = sizeof(u_int32_t);
    strcpy(ifr.ifr_name, radar->atd.ad_name);
    ifr.ifr_data = (caddr_t)&radar->atd;
	if (ioctl(radar->s, SIOCGATHPHYERR, &ifr) < 0)
		err(1, radar->atd.ad_name);
	return 0;
}
static int
radarSetDebugLevel(struct radarhandler *radar, u_int32_t level)
{
	u_int32_t result;
    struct ifreq ifr;

	radar->atd.ad_id = DFS_SET_DEBUG_LEVEL | ATH_DIAG_IN;
	radar->atd.ad_out_data = NULL;
	radar->atd.ad_out_size = 0;
	radar->atd.ad_in_data = (void *) &level;
	radar->atd.ad_in_size = sizeof(u_int32_t);
    strcpy(ifr.ifr_name, radar->atd.ad_name);
    ifr.ifr_data = (caddr_t)&radar->atd;
	if (ioctl(radar->s, SIOCGATHPHYERR, &ifr) < 0)
		err(1, radar->atd.ad_name);
	return 0;
}
static int
radarDisableFFT(struct radarhandler *radar)
{
	u_int32_t result;
    struct ifreq ifr;

	radar->atd.ad_id = DFS_DISABLE_FFT | ATH_DIAG_DYN;
	radar->atd.ad_out_data = NULL;
	radar->atd.ad_out_size = 0;
	radar->atd.ad_in_data = (void *) &result;
	radar->atd.ad_in_size = sizeof(u_int32_t);
    strcpy(ifr.ifr_name, radar->atd.ad_name);
    ifr.ifr_data = (caddr_t)&radar->atd;
	if (ioctl(radar->s, SIOCGATHPHYERR, &ifr) < 0)
		err(1, radar->atd.ad_name);
	return 0;
}

static int
radarEnableFFT(struct radarhandler *radar)
{
	u_int32_t result;
    struct ifreq ifr;

	radar->atd.ad_id = DFS_ENABLE_FFT | ATH_DIAG_DYN;
	radar->atd.ad_out_data = NULL;
	radar->atd.ad_out_size = 0;
	radar->atd.ad_in_data = (void *) &result;
	radar->atd.ad_in_size = sizeof(u_int32_t);
    strcpy(ifr.ifr_name, radar->atd.ad_name);
    ifr.ifr_data = (caddr_t)&radar->atd;
	if (ioctl(radar->s, SIOCGATHPHYERR, &ifr) < 0)
		err(1, radar->atd.ad_name);
	return 0;
}
static int
radarDisableDetect(struct radarhandler *radar)
{
	u_int32_t result;
    struct ifreq ifr;

	radar->atd.ad_id = DFS_DISABLE_DETECT | ATH_DIAG_DYN;
	radar->atd.ad_out_data = NULL;
	radar->atd.ad_out_size = 0;
	radar->atd.ad_in_data = (void *) &result;
	radar->atd.ad_in_size = sizeof(u_int32_t);
    strcpy(ifr.ifr_name, radar->atd.ad_name);
    ifr.ifr_data = (caddr_t)&radar->atd;
	if (ioctl(radar->s, SIOCGATHPHYERR, &ifr) < 0)
		err(1, radar->atd.ad_name);
	return 0;
}

static int
radarEnableDetect(struct radarhandler *radar)
{
	u_int32_t result;
    struct ifreq ifr;

	radar->atd.ad_id = DFS_ENABLE_DETECT | ATH_DIAG_DYN;
	radar->atd.ad_out_data = NULL;
	radar->atd.ad_out_size = 0;
	radar->atd.ad_in_data = (void *) &result;
	radar->atd.ad_in_size = sizeof(u_int32_t);
    strcpy(ifr.ifr_name, radar->atd.ad_name);
    ifr.ifr_data = (caddr_t)&radar->atd;
	if (ioctl(radar->s, SIOCGATHPHYERR, &ifr) < 0)
		err(1, radar->atd.ad_name);
	return 0;
}

static int
radarBangRadar(struct radarhandler *radar)
{
	u_int32_t result;
    struct ifreq ifr;

	radar->atd.ad_id = DFS_BANGRADAR | ATH_DIAG_DYN;
	radar->atd.ad_out_data = NULL;
	radar->atd.ad_out_size = 0;
	radar->atd.ad_in_data = (void *) &result;
	radar->atd.ad_in_size = sizeof(u_int32_t);
    strcpy(ifr.ifr_name, radar->atd.ad_name);
    ifr.ifr_data = (caddr_t)&radar->atd;
	if (ioctl(radar->s, SIOCGATHPHYERR, &ifr) < 0)
		err(1, radar->atd.ad_name);
	return 0;
}

static void
radarGetThresholds(struct radarhandler *radar, HAL_PHYERR_PARAM *pe)
{
    struct ifreq ifr;
	radar->atd.ad_id = DFS_GET_THRESH | ATH_DIAG_DYN;
	radar->atd.ad_out_data = (void *) pe;
	radar->atd.ad_out_size = sizeof(HAL_PHYERR_PARAM);
    strcpy(ifr.ifr_name, radar->atd.ad_name);
    ifr.ifr_data = (caddr_t)&radar->atd;
	if (ioctl(radar->s, SIOCGATHPHYERR, &ifr) < 0)
		err(1, radar->atd.ad_name);
}

static int
radarGetNol(struct radarhandler *radar)
{
	u_int32_t result;
    struct ifreq ifr;

	radar->atd.ad_id = DFS_GET_USENOL | ATH_DIAG_DYN;
	radar->atd.ad_in_data = NULL;
	radar->atd.ad_in_size = 0;
	radar->atd.ad_out_data = (void *) &result;
	radar->atd.ad_out_size = sizeof(u_int32_t);
    strcpy(ifr.ifr_name, radar->atd.ad_name);
    ifr.ifr_data = (caddr_t)&radar->atd;
	if (ioctl(radar->s, SIOCGATHPHYERR, &ifr) < 0)
		err(1, radar->atd.ad_name);
	return(result);
}

static int
radarSetUsenol(struct radarhandler *radar, u_int32_t usenol )
{
    struct ifreq ifr;
    radar->atd.ad_id = DFS_SET_USENOL | ATH_DIAG_IN;
    radar->atd.ad_out_data = NULL;
    radar->atd.ad_out_size = 0;
    radar->atd.ad_in_data = (void *) &usenol;
    radar->atd.ad_in_size = sizeof(u_int32_t);
    strcpy(ifr.ifr_name, radar->atd.ad_name);
    ifr.ifr_data = (caddr_t) &radar->atd;
    if (ioctl(radar->s, SIOCGATHPHYERR, &ifr) < 0)
              err(1, radar->atd.ad_name);
        return 0;
}

static int
radarSetMuteTime(struct radarhandler *radar, u_int32_t dur)
{
    struct ifreq ifr;

	radar->atd.ad_id = DFS_MUTE_TIME | ATH_DIAG_IN;
	radar->atd.ad_out_data = NULL;
	radar->atd.ad_out_size = 0;
	radar->atd.ad_in_data = (void *) &dur;
	radar->atd.ad_in_size = sizeof(u_int32_t);
    strcpy(ifr.ifr_name, radar->atd.ad_name);
    ifr.ifr_data = (caddr_t) &radar->atd;
	if (ioctl(radar->s, SIOCGATHPHYERR, &ifr) < 0)
		err(1, radar->atd.ad_name);
	return 0;
}

static int
radarGetRadarDetects(struct radarhandler *radar)
{
    u_int32_t result;
    struct ifreq ifr;

    radar->atd.ad_id = DFS_RADARDETECTS | ATH_DIAG_DYN;
    radar->atd.ad_in_data = NULL;
    radar->atd.ad_in_size = 0;
    radar->atd.ad_out_data = (void *) &result;
    radar->atd.ad_out_size = sizeof(u_int32_t);
    strcpy(ifr.ifr_name, radar->atd.ad_name);
    ifr.ifr_data = (caddr_t) &radar->atd;
    if (ioctl(radar->s, SIOCGATHPHYERR, &ifr) < 0)
          err(1, radar->atd.ad_name);
    return(result);
}

void
radarset(struct radarhandler *radar, int op, u_int32_t param)
{
	HAL_PHYERR_PARAM pe;
    struct ifreq ifr;

	pe.pe_firpwr = HAL_PHYERR_PARAM_NOVAL;
	pe.pe_rrssi = HAL_PHYERR_PARAM_NOVAL;
	pe.pe_height = HAL_PHYERR_PARAM_NOVAL;
	pe.pe_prssi = HAL_PHYERR_PARAM_NOVAL;
	pe.pe_inband = HAL_PHYERR_PARAM_NOVAL;

	/* 5413 specific */
	pe.pe_relpwr = HAL_PHYERR_PARAM_NOVAL;
	pe.pe_relstep = HAL_PHYERR_PARAM_NOVAL;
	pe.pe_maxlen = HAL_PHYERR_PARAM_NOVAL;

	switch(op) {
	case DFS_PARAM_FIRPWR:
		pe.pe_firpwr = param;
		break;
	case DFS_PARAM_RRSSI:
		pe.pe_rrssi = param;
		break;
	case DFS_PARAM_HEIGHT:
		pe.pe_height = param;
		break;
	case DFS_PARAM_PRSSI:
		pe.pe_prssi = param;
		break;
	case DFS_PARAM_INBAND:
		pe.pe_inband = param;
		break;
	/* following are valid for 5413 only */
	case DFS_PARAM_RELPWR:
		pe.pe_relpwr = param;
		break;
	case DFS_PARAM_RELSTEP:
		pe.pe_relstep = param;
		break;
	case DFS_PARAM_MAXLEN:
		pe.pe_maxlen = param;
		break;
	}
	radar->atd.ad_id = DFS_SET_THRESH | ATH_DIAG_IN;
	radar->atd.ad_out_data = NULL;
	radar->atd.ad_out_size = 0;
	radar->atd.ad_in_data = (void *) &pe;
	radar->atd.ad_in_size = sizeof(HAL_PHYERR_PARAM);
    strcpy(ifr.ifr_name, radar->atd.ad_name);
    ifr.ifr_data = (caddr_t) &radar->atd;

	if (ioctl(radar->s, SIOCGATHPHYERR, &ifr) < 0)
		err(1, radar->atd.ad_name);
}

static void
usage(void)
{
	const char *msg = "\
Usage: radartool [cmd]\n\
firpwr X            set firpwr (thresh to check radar sig is gone) to X (int32)\n\
rrssi X             set radar rssi (start det) to X dB (u_int32)\n\
height X            set threshold for pulse height to X dB (u_int32)\n\
prssi               set threshold to checkif pulse is gone to X dB (u_int32)\n\
inband X            set threshold to check if pulse is inband to X (0.5 dB) (u_int32)\n\
dfstime X           set dfs test time to X secs\n\
en_relpwr_check X   enable/disable radar relative power check (AR5413 only)\n\
relpwr X            set threshold to check the relative power of radar (AR5413 only)\n\
usefir128 X         en/dis using in-band pwr measurement over 128 cycles(AR5413 only)\n\
en_block_check X    en/dis to block OFDM weak sig as radar det(AR5413 only)\n\
en_max_rrssi X      en/dis to use max rssi instead of last rssi (AR5413 only)\n\
en_relstep X        en/dis to check pulse relative step (AR5413 only)\n\
relstep X           set threshold to check relative step for pulse det(AR5413 only)\n\
maxlen X            set max length of radar signal(in 0.8us step) (AR5413 only)\n\
numdetects          get number of radar detects\n";
	fprintf(stderr, "%s", msg);
}

int
main(int argc, char *argv[])
{
#define	streq(a,b)	(strcasecmp(a,b) == 0)
	struct radarhandler radar;
	HAL_REVS revs;
    struct ifreq ifr;

	memset(&radar, 0, sizeof(radar));
	radar.s = socket(AF_INET, SOCK_DGRAM, 0);
	if (radar.s < 0)
		err(1, "socket");
	if (argc > 1 && strcmp(argv[1], "-i") == 0) {
		if (argc < 2) {
			fprintf(stderr, "%s: missing interface name for -i\n",
				argv[0]);
			exit(-1);
		}
		strncpy(radar.atd.ad_name, argv[2], sizeof (radar.atd.ad_name));
		argc -= 2, argv += 2;
	} else
		strncpy(radar.atd.ad_name, ATH_DEFAULT, sizeof (radar.atd.ad_name));

#if 0
	radar.atd.ad_id = HAL_DIAG_REVS;
	radar.atd.ad_out_data = (void *) &revs;
	radar.atd.ad_out_size = sizeof(revs);
    strcpy(ifr.ifr_name, radar.atd.ad_name);
    ifr.ifr_data = (caddr_t) &radar.atd;
	if (ioctl(radar.s, SIOCGATHDIAG, &ifr) < 0)
		err(1, radar.atd.ad_name);

	switch (revs.ah_devid) {
	case AR5210_PROD:
	case AR5210_DEFAULT:
		printf("No radar detection yet for a 5210\n");
		exit(0);
	case AR5211_DEVID:
	case AR5311_DEVID:
	case AR5211_DEFAULT:
	case AR5211_FPGA11B:
		printf("No radar detecton yet for a 5211\n");
		exit(0);
	case AR5212_FPGA:
	case AR5212_DEVID:
	case AR5212_DEVID_IBM:
	case AR5212_DEFAULT:
	case AR5212_AR5312_REV2:
	case AR5212_AR5312_REV7:
		break;
	default:
		printf("No radar detection for device 0x%x\n", revs.ah_devid);
		exit(0);
	}
#endif
	if (argc >= 2) {
		if(streq(argv[1], "firpwr")) {
			radarset(&radar, DFS_PARAM_FIRPWR, (u_int32_t) atoi(argv[2]));
		} else if (streq(argv[1], "rrssi")) {
			radarset(&radar, DFS_PARAM_RRSSI, strtoul(argv[2], NULL, 0));
		} else if (streq(argv[1], "height")) {
			radarset(&radar, DFS_PARAM_HEIGHT, strtoul(argv[2], NULL, 0));
		} else if (streq(argv[1], "prssi")) {
			radarset(&radar, DFS_PARAM_PRSSI, strtoul(argv[2], NULL, 0));
		} else if (streq(argv[1], "inband")) {
			radarset(&radar, DFS_PARAM_INBAND, strtoul(argv[2], NULL, 0));
		} else if (streq(argv[1], "dfstime")) {
			radarSetMuteTime(&radar, strtoul(argv[2], NULL, 0));
		} else if (streq(argv[1], "usenol")) {
			radarSetUsenol(&radar, atoi(argv[2]));
		} else if (streq(argv[1], "dfsdebug")) {
                        radarSetDebugLevel(&radar, (u_int32_t)atoi(argv[2]));
		} else if (streq(argv[1], "fft")) {
			radarEnableFFT(&radar);
		} else if (streq(argv[1], "nofft")) {
			radarDisableFFT(&radar);
		} else if (streq(argv[1], "bangradar")) {
			radarBangRadar(&radar);
		} else if (streq(argv[1], "shownol")) {
			radarShowNol(&radar);
		} else if (streq(argv[1], "disable")) {
			radarDisableDetect(&radar);
		} else if (streq(argv[1], "enable")) {
			radarEnableDetect(&radar);
		} else if (streq(argv[1], "numdetects")) {
			printf("Radar: detected %d radars\n", radarGetRadarDetects(&radar));
		} else if (streq(argv[1],"-h")) {
			usage();
		/* Following are valid for 5413 only */
		} else if (streq(argv[1], "relpwr")) {
			radarset(&radar, DFS_PARAM_RELPWR, strtoul(argv[2], NULL, 0));
		} else if (streq(argv[1], "relstep")) {
			radarset(&radar, DFS_PARAM_RELSTEP, strtoul(argv[2], NULL, 0));
		} else if (streq(argv[1], "maxlen")) {
			radarset(&radar, DFS_PARAM_MAXLEN, strtoul(argv[2], NULL, 0));
		}
	} else if (argc == 1) {
		HAL_PHYERR_PARAM pe;
		u_int32_t nol;
		nol = radarGetNol(&radar);
		printf ("Radar;\nUse NOL: %s\n",nol ? "yes" : "no");
		radarGetThresholds(&radar, &pe);
		printf ("Firpwr (thresh to see if radar sig is gone):  %d\n",pe.pe_firpwr);
		printf ("Radar Rssi (thresh to start radar det in dB): %u\n",pe.pe_rrssi);
		printf ("Height (thresh for pulse height (dB):         %u\n",pe.pe_height);
		printf ("Pulse rssi (thresh if pulse is gone in dB):   %u\n",pe.pe_prssi);
		printf ("Inband (thresh if pulse is inband (in 0.5dB): %u\n",pe.pe_inband);
		/* Following are valid for 5413 only */
                if (pe.pe_relpwr & HAL_PHYERR_PARAM_ENABLE)
                        printf ("Relative power check, thresh in 0.5dB steps: %u\n", pe.pe_relpwr & ~HAL_PHYERR_PARAM_ENABLE);
                else
                        printf ("Relative power check disabled\n");
                if (pe.pe_relstep & HAL_PHYERR_PARAM_ENABLE)
                        printf ("Relative step thresh in 0.5dB steps: %u\n", pe.pe_relstep & ~HAL_PHYERR_PARAM_ENABLE);
                else
                        printf ("Relative step for pulse detection disabled\n");                printf ("Max length of radar sig in 0.8us units: %u\n",pe.pe_maxlen);
	} else {
		usage ();
	}
	return 0;
}
