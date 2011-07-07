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
 *
 * $Id: //depot/sw/releases/7.3_AP/wlan/common/lmac/ath_pktlog/winvista.c#2 $
 */

#ifndef REMOVE_PKT_LOG

/*
 * Atheros packet log module (Vista-specific code)
 */
#include <osdep.h>

#include "pktlog_i.h"
#include "pktlog_rc.h"
#include "pktlog_hal.h"

extern int g_pktlog_mode;

static int pktlog_attach(struct ath_softc *sc);
static void pktlog_detach(struct ath_softc *sc);

static struct ath_pktlog_funcs g_exported_pktlog_funcs = {
    pktlog_attach,
    pktlog_detach,
    pktlog_txctl,
    pktlog_txstatus,
    pktlog_rx,
    pktlog_text,
    pktlog_start,
    pktlog_read_hdr,
    pktlog_read_buf
};

static struct ath_pktlog_halfuncs g_exported_pktlog_halfuncs = {
    pktlog_ani
};

static struct ath_pktlog_rcfuncs g_exported_pktlog_rcfuncs = {
    pktlog_rcfind,
    pktlog_rcupdate
};

int
module_init_pktlog(void)
{
    g_pktlog_funcs = &g_exported_pktlog_funcs;
    g_pktlog_halfuncs = &g_exported_pktlog_halfuncs;
    g_pktlog_rcfuncs = &g_exported_pktlog_rcfuncs;

    return 0;
}

static int
pktlog_attach(struct ath_softc *sc)
{
    struct ath_pktlog_info *pl_info;
    
    /* We don't support system-wide logging in Vista */
    ASSERT(sc);

    g_pktlog_mode = PKTLOG_MODE_ADAPTER;
    
    pl_info = (struct ath_pktlog_info *)OS_MALLOC(sc->sc_osdev, sizeof(*pl_info), GFP_KERNEL);
    if (pl_info == NULL) {
        printk("%s:allocation failed for pl_info\n", __func__);
        return -ENOMEM;
    }
    
    pktlog_init(pl_info);
    sc->pl_info = pl_info;
    return 0;
}

static void
pktlog_detach(struct ath_softc *sc)
{
    struct ath_pktlog_info *pl_info = sc->pl_info;

    pktlog_cleanup(pl_info);
    
    if (pl_info->buf)
        pktlog_release_buf(pl_info);

    OS_FREE(pl_info);
    sc->pl_info = NULL;
}

void
pktlog_disable_adapter_logging(void)
{
    /* We don't support disabling adapter logging in Vista */
    ASSERT(0);
}

int
pktlog_alloc_buf(struct ath_softc *sc, struct ath_pktlog_info *pl_info)
{
    pl_info->buf =
        (struct ath_pktlog_buf *)OS_MALLOC(sc->sc_osdev,
                                           sizeof(*(pl_info->buf)) + pl_info->buf_size,
                                           GFP_KERNEL);
    if (pl_info->buf == NULL) {
        printk("%s: Unable to allocate buffer\n", __func__);
        return -ENOMEM;
    }
    return 0;
}

void
pktlog_release_buf(struct ath_pktlog_info *pl_info)
{
    OS_FREE(pl_info->buf);
    pl_info->buf = NULL;
}

int
pktlog_tcpip(struct ath_softc *sc, struct llc *llc, u_int32_t *proto_log, int *proto_len)
{
    return PKTLOG_PROTO_NONE;
}

#endif /* REMOVE_PKT_LOG */
