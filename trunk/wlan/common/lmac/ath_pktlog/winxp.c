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
 * $Id: //depot/sw/releases/7.3_AP/wlan/common/lmac/ath_pktlog/winxp.c#2 $
 */

#ifndef REMOVE_PKT_LOG

/*
 * Atheros packet log module (XP-specific code)
 */
#include <osdep.h>

#include "pktlog_i.h"
#include "pktlog_rc.h"
#include "pktlog_hal.h"

#include "if_athvar.h"

extern int g_pktlog_mode;

static int pktlog_attach(struct ath_softc *sc);
static void pktlog_detach(struct ath_softc *sc);

static struct ath_pktlog_funcs g_exported_pktlog_funcs = {
    pktlog_attach,
    pktlog_detach,
    pktlog_txctl,
    pktlog_txstatus,
    pktlog_rx,
    pktlog_text
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
    /* We don't support disabling adapter logging in XP */
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

int
pktlogEventGet(struct ath_pktlog_info *pl_info, u_int32_t *len, void *data)
{
    struct ath_pktlog_hdr *logHdr;
    struct ath_pktlog_buf *log_buf = pl_info->buf;

    if ((log_buf->rd_offset < 0)) { 
        uiPrintf("wr..%d  rd..%d\n", log_buf->wr_offset, log_buf->rd_offset);
        *len     = 0;
        return 1;
    }

    logHdr   = (struct ath_pktlog_hdr *)(log_buf->log_data + log_buf->rd_offset);
    *len     = sizeof(*logHdr) + logHdr->size;

    OS_MEMCPY(data, logHdr, sizeof(*logHdr));
    if(pl_info->buf_size - log_buf->rd_offset >= *len) {
        OS_MEMCPY((char *)data + sizeof(*logHdr), logHdr + 1, logHdr->size);
    } else {
        OS_MEMCPY((char *)data + sizeof(*logHdr), log_buf->log_data, logHdr->size);
    }

    PKTLOG_MOV_RD_IDX(log_buf->rd_offset, log_buf, pl_info->buf_size);

    /* We've got all the data. Notify rgtest to stop. */
    if (log_buf->rd_offset == log_buf->wr_offset)
        return 1;

    return 0;
}

NDIS_STATUS
pktlogGetBuffer(
    OS_DEV_INFO *pOsInfo,
    PVOID InformationBuffer,
    ULONG InformationBufferLength,
    PULONG BytesWritten,
    PULONG BytesNeeded
    )
{
    struct ath_softc *sc = (struct ath_softc *)pOsInfo->sc_dev;
    struct ath_pktlog_info *pl_info = sc->pl_info;
    struct ath_pktlog_buf *log_buf=pl_info->buf;
    int done;
   
    if (log_buf == NULL) {
        return NDIS_STATUS_FAILURE;
    }

    if ((pOsInfo->rdCnt == 0) && pl_info->log_state) {
        pl_info->saved_state = pl_info->log_state;
        pl_info->log_state = 0;
    }

    /*
     * Here is what we should return to rgtest.
     * InformationBuffer + 1 (Output): Current Index.
     * InformationBuffer + 2 (Output): Length.
     * InformationBuffer + 3 (Output): Max Count.
     * InformationBuffer + 4 (Input) : Buffer pointer.
     */

    done = pktlogEventGet(pl_info,
                         ((u_int32_t *)InformationBuffer) + 2,
                         ((u_int32_t *)InformationBuffer) + 4);
    *BytesWritten = 4 * sizeof(u_int32_t) + ((u_int32_t *)InformationBuffer)[2];

    *((u_int32_t *)InformationBuffer + 1) = pOsInfo->rdCnt++;

    /* Since we don't know the exact maxCnt, we just keep maxCnt greater than curIdx. */
    *((u_int32_t *)InformationBuffer + 3) = pOsInfo->rdCnt + 1;

    /* When rd_offset == wr_offset, modify maxCnt == curIdx to force rgtest to stop requesting for data */
    if (done)
        *((u_int32_t *)InformationBuffer + 3) = *((u_int32_t *)InformationBuffer + 1);

    if (done) {
        pOsInfo->rdCnt = 0;
        log_buf->rd_offset = -1;
        pl_info->log_state = pl_info->saved_state;
    }
    
    return NDIS_STATUS_SUCCESS;
}

NDIS_STATUS
pktlogGetBufferHeader(
    OS_DEV_INFO *pDevInfo,
    PVOID InformationBuffer,
    ULONG InformationBufferLength,
    PULONG BytesWritten,
    PULONG BytesNeeded
    )
{
    struct ath_softc *sc = (struct ath_softc *)pDevInfo->sc_dev;
    struct ath_pktlog_info *pl_info = sc->pl_info;
    struct ath_pktlog_buf *log_buf=pl_info->buf;
    u_int32_t *pu32data, hdrsize;

    if (log_buf == NULL)
        return NDIS_STATUS_FAILURE;
    
    pu32data = (u_int32_t *)InformationBuffer;

    /* Return header as expected by current rgtest application */
    *pu32data = log_buf->bufhdr.version;
    *(pu32data+1) = 0;
    hdrsize = 2 * sizeof(u_int32_t);

    if (InformationBufferLength < hdrsize) {
        *BytesNeeded = hdrsize;
        return NDIS_STATUS_INVALID_LENGTH;
    }

    *BytesWritten = hdrsize;
    
    return NDIS_STATUS_SUCCESS;
}

int
pktlogStart(
    OS_DEV_INFO *pDevInfo,
    A_UINT32 sizekb, 
    A_UINT32 log_state
    )
{
    struct ath_softc *sc = (struct ath_softc *)pDevInfo->sc_dev;
    struct ath_pktlog_info *pl_info = sc->pl_info;
    int status;

    pl_info->buf_size = sizekb * 1024;

    status = pktlog_enable(sc, log_state);
    pl_info->saved_state = 0;

    return status;
}

#endif /* REMOVE_PKT_LOG */
