/*-
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

#include <osdep.h>

/*
 * Packet Logging for Atheros driver
 */

#include <sys/param.h>
#include <sys/systm.h> 
#include <sys/mbuf.h>   
#include <sys/malloc.h>
#include <sys/lock.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/ioccom.h>
#include <sys/errno.h>
 
#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_arp.h>
#include <net/if_llc.h>
#include <net/bpf.h>

#include <net/if_ether.h>
#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>

#ifdef UMAC
#include <ieee80211_var.h>
#else
#include <net80211/ieee80211_var.h>
#endif

#include "pktlog_i.h"
#include "pktlog_rc.h"
#include "pktlog_hal.h"
#include "if_llc.h"

#include "if_athvar.h"

#ifndef REMOVE_PKT_LOG

#ifdef TIMER_TEST			/* GLC */
static void timer_test(void *);
#endif

#define	AR_DEBUG

#ifdef DEBUG
#define PKTLOG_CHATTY_DEBUG(x)	printf x
#define PKTLOG_TAG "ATH_PKTLOG" 
#else
#define PKTLOG_CHATTY_DEBUG(x)
#define PKTLOG_TAG
#endif

static int pktlog_attach(struct ath_softc *sc);
static void pktlog_detach(struct ath_softc *sc);

struct ath_pktlog_funcs g_exported_pktlog_funcs = {
    .pktlog_attach = pktlog_attach,
    .pktlog_detach = pktlog_detach,
    .pktlog_txctl = pktlog_txctl,
    .pktlog_txstatus = pktlog_txstatus,
    .pktlog_rx = pktlog_rx,
    .pktlog_text = pktlog_text
};

struct ath_pktlog_halfuncs g_exported_pktlog_halfuncs = {
    .pktlog_ani = pktlog_ani
};

struct ath_pktlog_rcfuncs g_exported_pktlog_rcfuncs = {
    .pktlog_rcupdate = pktlog_rcupdate,
    .pktlog_rcfind = pktlog_rcfind
};

extern struct ath_pktlog_info *g_pktlog_info;
extern int g_pktlog_mode;

static const struct ath_pktlog_funcs pktlogfn = {
	.pktlog_attach	= pktlog_attach,
	.pktlog_detach	= pktlog_detach,
	.pktlog_txctl	= pktlog_txctl,
	.pktlog_txstatus = pktlog_txstatus,
	.pktlog_rx	= pktlog_rx,
        .pktlog_text    = pktlog_text,
};

/*
 * Helper functions.
 */
int
pktlog_alloc_buf(struct ath_softc *sc, struct ath_pktlog_info *pl_info)
{
	PKTLOG_CHATTY_DEBUG((PKTLOG_TAG "%s, sc (%p)\n", __func__, sc));

	pl_info->buf = (struct ath_pktlog_buf *)malloc(pl_info->buf_size + sizeof(struct ath_pktlog_buf),
	    M_TEMP,M_NOWAIT | M_ZERO);
	if(pl_info->buf == NULL)
		return ENOMEM;
	memset(pl_info->buf,0,pl_info->buf_size + sizeof(struct ath_pktlog_buf));
	printf("log state %x enabled buffer size %d\n",pl_info->log_state,
	    pl_info->buf_size);

	return 0;
}

void
pktlog_release_buf(struct ath_pktlog_info *pl_info)
{
	if(pl_info->buf)
		free(pl_info->buf, M_TEMP);
	pl_info->buf = NULL;
}

/*
 * Disable a specific adapter logging.
 */
void
pktlog_disable_adapter_logging(void)
{
	printf("disabling the adapter not allowed\n");
}

/*
 * Initialize logging for system or adapter
 */

int pktlog_attach(struct ath_softc *sc)
{
	struct ath_pktlog_info *pl_info;

	PKTLOG_CHATTY_DEBUG((PKTLOG_TAG "%s, sc (%p)\n", __func__, sc));

	/* Zero out the contents of pktlog info */
	pl_info = (struct ath_pktlog_info *) malloc(sizeof(struct 
	    ath_pktlog_info), M_TEMP,M_NOWAIT | M_ZERO);
	if (pl_info == NULL) {
		printf(PKTLOG_TAG "%s: allocation failed for pl_info\n",
		    __FUNCTION__);
		return (ENOMEM);
	}
	memset(pl_info, 0, sizeof(struct ath_pktlog_info));
	sc->pl_info = pl_info;
	pktlog_init(pl_info);
	if (g_pktlog_mode == PKTLOG_MODE_SYSTEM)
		g_pktlog_info = pl_info;

	return 0;
}

static
void pktlog_detach(struct ath_softc *sc)
{
	struct ath_pktlog_info *pl_info = sc->pl_info;

	PKTLOG_CHATTY_DEBUG((PKTLOG_TAG " %s\n", __func__));

	pktlog_release_buf(pl_info);
	free(pl_info, M_TEMP);
	sc->pl_info = NULL;
}

static
int pktlog_reset(struct ath_softc *sc)
{
	struct ath_pktlog_info *pl_info = sc->pl_info;
	int savestate;

	PKTLOG_CHATTY_DEBUG((PKTLOG_TAG "%s\n", __func__));

	savestate = pl_info->log_state;
	pl_info->log_state = 0;
	if(pl_info->buf){
		free(pl_info->buf,M_TEMP);
		pl_info->buf = NULL;
	}

	return 0;
}

int
ath_ioctl_pktlog(struct ieee80211com *ic, u_long cmd, caddr_t data)
{
	#define	IS_RUNNING(ifp) \
	((ifnet_flags((ifnet_t) ifp) & (IFF_RUNNING|IFF_UP)) == (IFF_RUNNING|IFF_UP))
	struct ath_softc *sc = NET80211_2_ATH_SOFTC(ic);
	struct ath_pktlog_info *pl_info = sc->pl_info;
	int error = 0;
	struct ath_pktlog_ioctl *atpkt = (struct ath_pktlog_ioctl *) data;

	if (cmd != SIOCGATHPKT) {
		return ENXIO;
	}
	ATH_LOCK(sc->sc_osdev);
	switch (atpkt->ap_cmd) {
	case ATH_PKT_ENABLE:
		error = pktlog_enable(sc, atpkt->ap_val);
		if (error == 0) {
			printf("log state %x enabled, buffer size is %d\n",
			    pl_info->log_state, pl_info->buf_size);
		}
		break;
	case ATH_PKT_SETSIZE:
		error = pktlog_setsize(sc, atpkt->ap_val);
		break;
	case ATH_PKT_GETSIZE:
		atpkt->ap_val = pl_info->buf_size;
		break;
	case ATH_PKT_GETBUF:
		if (atpkt->ap_data == NULL) {
			error = ENOMEM;
		} else {
			*(u_int32_t **)atpkt->ap_data =
				(u_int32_t *)pl_info->buf;
			atpkt->ap_data_size = sizeof(pl_info->buf);
		}
		break;
	case ATH_PKT_GET_LOGSTATE:
		atpkt->ap_val = pl_info->log_state;
		break;
	case ATH_PKT_RESET:
		error = pktlog_reset(sc);
		break;
	}
	ATH_UNLOCK(sc->sc_osdev);
	return error;
}

int
pktlog_tcpip(struct ath_softc *sc, struct llc *llc, u_int32_t *proto_log, int *proto_len)
{
    struct ip *ip;
    struct tcphdr *tcp;
    u_int32_t *proto_hdr;
    int i;

    ip = (struct ip *)((u_int8_t *)llc + LLC_SNAPFRAMELEN);

    switch (ip->ip_p) {
    case IPPROTO_TCP:
        /* 
         * tcp + ip hdr len are in units of 32-bit words
         */ 
        tcp = (struct tcphdr *)((u_int32_t *)ip + ip->ip_hl);
        proto_hdr = (u_int32_t *)tcp;

        for (i = 0; i < tcp->th_off; i++) {
            *proto_log = ntohl(*proto_hdr);
            proto_log++, proto_hdr++;
        }
        *proto_len = tcp->th_off * sizeof(u_int32_t);
        return PKTLOG_PROTO_TCP;

    case IPPROTO_UDP:
    case IPPROTO_ICMP:
    default:
        *proto_len = 0;
        return PKTLOG_PROTO_NONE;
    }
}

#endif /* REMOVE_PKT_LOG */
