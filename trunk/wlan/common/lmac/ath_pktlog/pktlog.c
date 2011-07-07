/*************************************************************************
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
 *
 * Packet logging routines are defined here.
 *
 */


#include <osdep.h>
#include "pktlog_i.h"
#include "pktlog_rc.h"
#include "pktlog_hal.h"
#include "if_llc.h"

#ifndef REMOVE_PKT_LOG

struct ath_pktlog_info *g_pktlog_info = NULL;
int g_pktlog_mode = PKTLOG_MODE_SYSTEM;

void
pktlog_init(struct ath_pktlog_info *pl_info)
{
    OS_MEMZERO(pl_info, sizeof(*pl_info));
    
    PKTLOG_LOCK_INIT(pl_info);

    pl_info->buf_size = PKTLOG_DEFAULT_BUFSIZE;
    pl_info->buf = NULL;
    pl_info->log_state = 0;
}

void
pktlog_cleanup(struct ath_pktlog_info *pl_info)
{
    pl_info->log_state = 0;
    PKTLOG_LOCK_DESTROY(pl_info);
}

int
pktlog_enable(struct ath_softc *sc, int32_t log_state)
{
    struct ath_pktlog_info *pl_info = (sc) ? sc->pl_info : g_pktlog_info;
    int error;

    pl_info->log_state = 0;

    if (log_state != 0) {
        if (!sc) {
            if (g_pktlog_mode == PKTLOG_MODE_ADAPTER) {
                pktlog_disable_adapter_logging();
                g_pktlog_mode = PKTLOG_MODE_SYSTEM;
            }
        } else {
            if (g_pktlog_mode == PKTLOG_MODE_SYSTEM) {
                g_pktlog_info->log_state = 0;
                g_pktlog_mode = PKTLOG_MODE_ADAPTER;
            }
        }

        if (pl_info->buf == NULL) {
            error = pktlog_alloc_buf(sc, pl_info);
            if (error != 0)
                return error;
                
            pl_info->buf->bufhdr.version = CUR_PKTLOG_VER;
            pl_info->buf->bufhdr.magic_num = PKTLOG_MAGIC_NUM;
            pl_info->buf->wr_offset = 0;
            pl_info->buf->rd_offset = -1;
        }
    }
    pl_info->log_state = log_state;
    return 0;
}

int
pktlog_setsize(struct ath_softc *sc, int32_t size)
{
    struct ath_pktlog_info *pl_info = (sc) ? sc->pl_info : g_pktlog_info;

    if (size < 0)
        return -EINVAL;

    if (size == pl_info->buf_size)
        return 0;

    if (pl_info->log_state) {
        printk("Logging should be disabled before changing bufer size\n");
        return -EINVAL;
    }

    if (pl_info->buf != NULL)
        pktlog_release_buf(pl_info);

    if (size != 0)
        pl_info->buf_size = size;

    return 0;
}

static char *
pktlog_getbuf(struct ath_pktlog_info *pl_info,
              u_int16_t log_type, size_t log_size, 
              u_int32_t flags)
{
    struct ath_pktlog_buf *log_buf;
    int32_t buf_size;
    struct ath_pktlog_hdr *log_hdr;
    int32_t cur_wr_offset;
    char *log_ptr;

    log_buf = pl_info->buf;
    buf_size = pl_info->buf_size;

    PKTLOG_LOCK(pl_info);
    cur_wr_offset = log_buf->wr_offset;
    /* Move read offset to the next entry if there is a buffer overlap */
    if (log_buf->rd_offset >= 0) {
        if ((cur_wr_offset <= log_buf->rd_offset)
            && (cur_wr_offset + sizeof(struct ath_pktlog_hdr)) >
            log_buf->rd_offset)
            PKTLOG_MOV_RD_IDX(log_buf->rd_offset, log_buf, buf_size);
    } else {
        log_buf->rd_offset = cur_wr_offset;
    }

    log_hdr =
        (struct ath_pktlog_hdr *) (log_buf->log_data + cur_wr_offset);
    log_hdr->log_type = log_type;
    log_hdr->flags = flags;
    log_hdr->timestamp = OS_GET_TIMESTAMP();
    log_hdr->size = (u_int16_t)log_size;

    cur_wr_offset += sizeof(*log_hdr);

    if ((buf_size - cur_wr_offset) < log_size) {
        while ((cur_wr_offset <= log_buf->rd_offset)
               && (log_buf->rd_offset < buf_size))
            PKTLOG_MOV_RD_IDX(log_buf->rd_offset, log_buf, buf_size);
        cur_wr_offset = 0;
    }

    while ((cur_wr_offset <= log_buf->rd_offset)
           && (cur_wr_offset + log_size) > log_buf->rd_offset)
        PKTLOG_MOV_RD_IDX(log_buf->rd_offset, log_buf, buf_size);

    log_ptr = &(log_buf->log_data[cur_wr_offset]);

    cur_wr_offset += log_hdr->size;

    log_buf->wr_offset =
        ((buf_size - cur_wr_offset) >=
         sizeof(struct ath_pktlog_hdr)) ? cur_wr_offset : 0;
    PKTLOG_UNLOCK(pl_info);
   
    return log_ptr;
}



#ifndef REMOVE_PKTLOG_PROTO
/*
 * Searches for the presence of a protocol header and returns protocol type.
 * proto_len and proto_log are modified to return length (in bytes) and
 * contents of header (in host byte order), if one is found.
 */ 
static int 
pktlog_proto(struct ath_softc *sc, u_int32_t proto_log[PKTLOG_MAX_PROTO_WORDS],
             void *log_data, pktlog_proto_desc_t ds_type, int *proto_len)
{
#define IPHDRLEN     20
    struct ieee80211_frame *wh = NULL;
    struct llc *llc;
    int icv_len;
    struct log_tx *tx_log;
    struct log_rx *rx_log;
    struct ieee80211_node *ni = NULL;
    static const int pktlog_proto_min_hlen = sizeof(struct ieee80211_frame) + 
                                             sizeof(struct llc) + IPHDRLEN;

    switch (ds_type) {
        case PKTLOG_PROTO_TX_DESC:
            tx_log = (struct log_tx *)log_data;
            if (wbuf_get_pktlen((wbuf_t)(tx_log->bf->bf_mpdu)) < pktlog_proto_min_hlen)
                return PKTLOG_PROTO_NONE;

            wh = (struct ieee80211_frame *)(tx_log->firstds->ds_vdata);
            ni = wbuf_get_node((wbuf_t)(tx_log->bf->bf_mpdu));
            break;
        case PKTLOG_PROTO_RX_DESC:
            rx_log = (struct log_rx *)log_data;
            if (rx_log->status->rs_datalen < pktlog_proto_min_hlen)
                return PKTLOG_PROTO_NONE;
            /*
             * Positively need to find ni for cipher
             */
            if (rx_log->status->rs_keyix == HAL_RXKEYIX_INVALID) {
                struct ieee80211com *ic = (struct ieee80211com *)sc->sc_ieee;
                ni = ieee80211_find_rxnode(ic, (struct ieee80211_frame_min *)
                                           (rx_log->ds->ds_vdata));
                if (ni == NULL)
                    return PKTLOG_PROTO_NONE;
            } else {
                /*
                 * XXX: this violates the module abstraction!
                 * need a better way to do this.
                 */
                struct ieee80211com *ic = (struct ieee80211com *)sc->sc_ieee;
                struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);

                ni = scn->sc_keyixmap[rx_log->status->rs_keyix];
                if (ni == NULL)
                    return PKTLOG_PROTO_NONE;
            }

            wh = (struct ieee80211_frame *)(rx_log->ds->ds_vdata);
            break;
    }

    if ((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) != IEEE80211_FC0_TYPE_DATA)
        return PKTLOG_PROTO_NONE;

    if (ni->ni_ucastkey.wk_cipher == &ieee80211_cipher_none) {
        icv_len = 0;
    } else {
        struct ieee80211_key *k = &ni->ni_ucastkey;
        const struct ieee80211_cipher *cip = k->wk_cipher;

        icv_len = cip->ic_header;
    }

    llc = (struct llc *)((u_int8_t *)wh + ieee80211_anyhdrspace(ni->ni_ic, wh)
           + icv_len);

    return pktlog_tcpip(sc, llc, proto_log, proto_len);
#undef IPHDRLEN
}
#endif

/* 
 * Log Tx data - logs into adapter's buffer if sc is not NULL; 
 *               logs into system-wide buffer if sc is NULL.
 */
void
pktlog_txctl(struct ath_softc *sc, struct log_tx *log_data, u_int16_t iflags)
{
    struct ath_pktlog_txctl *tx_log;
    int i, proto = PKTLOG_PROTO_NONE, proto_len = 0;
    u_int8_t misc_cnt;
    struct ath_pktlog_info *pl_info;
    struct ieee80211_frame *wh;
    u_int8_t dir;
    u_int32_t *ds_words, proto_hdr[PKTLOG_MAX_PROTO_WORDS];
    HAL_DESC_INFO desc_info;
	u_int32_t	flags = iflags;

    if (g_pktlog_mode == PKTLOG_MODE_ADAPTER)
        pl_info = sc->pl_info;
    else
        pl_info = g_pktlog_info;

    if ((pl_info->log_state & ATH_PKTLOG_TX)== 0 || 
        log_data->firstds->ds_vdata == 0) {
        return;
    }

    misc_cnt = flags & PHFLAGS_MISCCNT_MASK;
    flags |= (((sc->sc_ah->ah_macRev << PHFLAGS_MACREV_SFT) & PHFLAGS_MACREV_MASK) |
		((sc->sc_ah->ah_macVersion << PHFLAGS_MACVERSION_SFT) & PHFLAGS_MACVERSION_MASK));


#ifndef REMOVE_PKTLOG_PROTO
    if (pl_info->options & ATH_PKTLOG_PROTO) {
        proto = pktlog_proto(sc, proto_hdr, log_data, PKTLOG_PROTO_TX_DESC,
                             &proto_len);
        flags |= (proto << PHFLAGS_PROTO_SFT) & PHFLAGS_PROTO_MASK;
    }
#endif

    tx_log = (struct ath_pktlog_txctl *)pktlog_getbuf(pl_info, 
              PKTLOG_TYPE_TXCTL, sizeof(*tx_log) + 
              misc_cnt * sizeof(tx_log->misc[0]) + proto_len, flags);

    wh = (struct ieee80211_frame *) (log_data->firstds->ds_vdata);
    tx_log->framectrl = *(u_int16_t *)(wh->i_fc);
    tx_log->seqctrl   = *(u_int16_t *)(wh->i_seq);

    dir = (wh->i_fc[1] & IEEE80211_FC1_DIR_MASK);
    if(dir == IEEE80211_FC1_DIR_TODS) {
        tx_log->bssid_tail = (wh->i_addr1[IEEE80211_ADDR_LEN-2] << 8) | 
                             (wh->i_addr1[IEEE80211_ADDR_LEN-1]);
        tx_log->sa_tail    = (wh->i_addr2[IEEE80211_ADDR_LEN-2] << 8) | 
                             (wh->i_addr2[IEEE80211_ADDR_LEN-1]);
        tx_log->da_tail    = (wh->i_addr3[IEEE80211_ADDR_LEN-2] << 8) | 
                             (wh->i_addr3[IEEE80211_ADDR_LEN-1]);
    }
    else if(dir == IEEE80211_FC1_DIR_FROMDS) {
        tx_log->bssid_tail = (wh->i_addr2[IEEE80211_ADDR_LEN-2] << 8) | 
                             (wh->i_addr2[IEEE80211_ADDR_LEN-1]);
        tx_log->sa_tail    = (wh->i_addr3[IEEE80211_ADDR_LEN-2] << 8) | 
                             (wh->i_addr3[IEEE80211_ADDR_LEN-1]);
        tx_log->da_tail    = (wh->i_addr1[IEEE80211_ADDR_LEN-2] << 8) | 
                             (wh->i_addr1[IEEE80211_ADDR_LEN-1]);
    }
    else {
        tx_log->bssid_tail = (wh->i_addr3[IEEE80211_ADDR_LEN-2] << 8) | 
                             (wh->i_addr3[IEEE80211_ADDR_LEN-1]);
        tx_log->sa_tail    = (wh->i_addr2[IEEE80211_ADDR_LEN-2] << 8) | 
                             (wh->i_addr2[IEEE80211_ADDR_LEN-1]);
        tx_log->da_tail    = (wh->i_addr1[IEEE80211_ADDR_LEN-2] << 8) | 
                             (wh->i_addr1[IEEE80211_ADDR_LEN-1]);
    }       

    ath_hal_getdescinfo(sc->sc_ah, &desc_info);

    ds_words = (u_int32_t *)(log_data->firstds) + desc_info.txctl_offset;
    for(i = 0; i < desc_info.txctl_numwords; i++)
        tx_log->txdesc_ctl[i] = ds_words[i];

    for (i = 0; i < misc_cnt; i++)
        tx_log->misc[i] = log_data->misc[i];

    if (proto != PKTLOG_PROTO_NONE)
        OS_MEMCPY(tx_log->proto_hdr, proto_hdr, proto_len);
}

void
pktlog_txstatus(struct ath_softc *sc, struct log_tx *log_data, u_int16_t iflags)
{
    struct ath_pktlog_txstatus *tx_log;
    int i;
    u_int8_t misc_cnt;
    struct ath_pktlog_info *pl_info;
    u_int32_t *ds_words;
    HAL_DESC_INFO desc_info;
	u_int32_t flags = iflags;

    if (g_pktlog_mode == PKTLOG_MODE_ADAPTER)
        pl_info = sc->pl_info;
    else
        pl_info = g_pktlog_info;

    if ((pl_info->log_state & ATH_PKTLOG_TX)== 0) 
        return;

    misc_cnt = flags & PHFLAGS_MISCCNT_MASK;
    flags |= (((sc->sc_ah->ah_macRev << PHFLAGS_MACREV_SFT) & PHFLAGS_MACREV_MASK) |
		((sc->sc_ah->ah_macVersion << PHFLAGS_MACVERSION_SFT) & PHFLAGS_MACVERSION_MASK));
    tx_log = (struct ath_pktlog_txstatus *)pktlog_getbuf(pl_info, PKTLOG_TYPE_TXSTATUS, 
                  sizeof(*tx_log) + misc_cnt * sizeof(tx_log->misc[0]), flags);

    ath_hal_getdescinfo(sc->sc_ah, &desc_info);

    ds_words = (u_int32_t *)(log_data->lastds) + desc_info.txstatus_offset;

    for(i = 0; i < desc_info.txstatus_numwords; i++)
        tx_log->txdesc_status[i] = ds_words[i];

    for (i = 0; i < misc_cnt; i++)
        tx_log->misc[i] = log_data->misc[i];
}

void
pktlog_rx(struct ath_softc *sc, struct log_rx *log_data, u_int16_t iflags)
{
    struct ath_pktlog_rx *rx_log;
    int i, proto = PKTLOG_PROTO_NONE, proto_len = 0;
    u_int8_t misc_cnt;
    struct ath_pktlog_info *pl_info;
    struct ieee80211_frame *wh;
    u_int32_t *ds_words, proto_hdr[PKTLOG_MAX_PROTO_WORDS];
    HAL_DESC_INFO desc_info;
    u_int8_t dir;
	u_int32_t flags = iflags;

    if (g_pktlog_mode == PKTLOG_MODE_ADAPTER)
        pl_info = sc->pl_info;
    else
        pl_info = g_pktlog_info;

    if ((pl_info->log_state & ATH_PKTLOG_RX) == 0)
        return;

    misc_cnt = flags & PHFLAGS_MISCCNT_MASK;
    flags |= (((sc->sc_ah->ah_macRev << PHFLAGS_MACREV_SFT) & PHFLAGS_MACREV_MASK) |
		((sc->sc_ah->ah_macVersion << PHFLAGS_MACVERSION_SFT) & PHFLAGS_MACVERSION_MASK));

#ifndef REMOVE_PKTLOG_PROTO
    if (pl_info->options & ATH_PKTLOG_PROTO) {
        proto = pktlog_proto(sc, proto_hdr, log_data, PKTLOG_PROTO_RX_DESC,
                             &proto_len);
        flags |= (proto << PHFLAGS_PROTO_SFT) & PHFLAGS_PROTO_MASK;
    }
#endif

    rx_log = (struct ath_pktlog_rx *)pktlog_getbuf(pl_info, PKTLOG_TYPE_RX,
             sizeof(*rx_log) + misc_cnt * sizeof(rx_log->misc[0]) + proto_len,
             flags);

    if(log_data->status->rs_datalen > sizeof(struct ieee80211_frame)) {
        wh = (struct ieee80211_frame *) (log_data->ds->ds_vdata);
        rx_log->framectrl = *(u_int16_t *)(wh->i_fc);
        rx_log->seqctrl   = *(u_int16_t *)(wh->i_seq);

        dir = (wh->i_fc[1] & IEEE80211_FC1_DIR_MASK);
        if(dir == IEEE80211_FC1_DIR_TODS) {
            rx_log->bssid_tail = (wh->i_addr1[IEEE80211_ADDR_LEN-2] << 8) |
                                 (wh->i_addr1[IEEE80211_ADDR_LEN-1]);
            rx_log->sa_tail    = (wh->i_addr2[IEEE80211_ADDR_LEN-2] << 8) |
                                 (wh->i_addr2[IEEE80211_ADDR_LEN-1]);
            rx_log->da_tail    = (wh->i_addr3[IEEE80211_ADDR_LEN-2] << 8) |
                                 (wh->i_addr3[IEEE80211_ADDR_LEN-1]);
        } else if(dir == IEEE80211_FC1_DIR_FROMDS) {
            rx_log->bssid_tail = (wh->i_addr2[IEEE80211_ADDR_LEN-2] << 8) |
                                 (wh->i_addr2[IEEE80211_ADDR_LEN-1]);
            rx_log->sa_tail    = (wh->i_addr3[IEEE80211_ADDR_LEN-2] << 8) |
                                 (wh->i_addr3[IEEE80211_ADDR_LEN-1]);
            rx_log->da_tail    = (wh->i_addr1[IEEE80211_ADDR_LEN-2] << 8) |
                                 (wh->i_addr1[IEEE80211_ADDR_LEN-1]);
        } else {
            rx_log->bssid_tail = (wh->i_addr3[IEEE80211_ADDR_LEN-2] << 8) |
                                 (wh->i_addr3[IEEE80211_ADDR_LEN-1]);
            rx_log->sa_tail    = (wh->i_addr2[IEEE80211_ADDR_LEN-2] << 8) |
                                 (wh->i_addr2[IEEE80211_ADDR_LEN-1]);
            rx_log->da_tail    = (wh->i_addr1[IEEE80211_ADDR_LEN-2] << 8) |
                                 (wh->i_addr1[IEEE80211_ADDR_LEN-1]);
        }
    } else {
        wh = (struct ieee80211_frame *) (log_data->ds->ds_vdata);

        if ((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) == IEEE80211_FC0_TYPE_CTL) {
            rx_log->framectrl = *(u_int16_t *)(wh->i_fc);
            rx_log->da_tail    = (wh->i_addr1[IEEE80211_ADDR_LEN-2] << 8) |
                                 (wh->i_addr1[IEEE80211_ADDR_LEN-1]);
            if(log_data->status->rs_datalen < sizeof(struct ieee80211_ctlframe_addr2)) {
                rx_log->sa_tail = 0;
            } else {
                rx_log->sa_tail    = (wh->i_addr2[IEEE80211_ADDR_LEN-2] << 8) |
                                     (wh->i_addr2[IEEE80211_ADDR_LEN-1]);
            }
        } else {
            rx_log->framectrl = 0xFFFF;
            rx_log->da_tail = 0;
            rx_log->sa_tail = 0;
        }

        rx_log->seqctrl   = 0;
        rx_log->bssid_tail = 0;
    }

    ath_hal_getdescinfo(sc->sc_ah, &desc_info);

    ds_words = (u_int32_t *)(log_data->ds) + desc_info.rxstatus_offset;

    for(i = 0; i < desc_info.rxstatus_numwords; i++)
        rx_log->rxdesc_status[i] = ds_words[i];

    for (i = 0; i < misc_cnt; i++)
        rx_log->misc[i] = log_data->misc[i];

    if (proto != PKTLOG_PROTO_NONE)
        OS_MEMCPY(rx_log->proto_hdr, proto_hdr, proto_len);
}

void
pktlog_ani(HAL_SOFTC hal_sc, struct log_ani *log_data, u_int16_t iflags)
{
    struct ath_softc *sc = (struct ath_softc *)hal_sc;
    struct ath_pktlog_ani *ani_log;
    int i;
    u_int8_t misc_cnt;
    struct ath_pktlog_info *pl_info;
	u_int32_t	flags = iflags;

    if (g_pktlog_mode == PKTLOG_MODE_ADAPTER)
        pl_info = sc->pl_info;
    else
        pl_info = g_pktlog_info;

    if ((pl_info->log_state & ATH_PKTLOG_ANI) == 0)
        return;

    misc_cnt = flags & PHFLAGS_MISCCNT_MASK;
    flags |= (((sc->sc_ah->ah_macRev << PHFLAGS_MACREV_SFT) & PHFLAGS_MACREV_MASK) |
		((sc->sc_ah->ah_macVersion << PHFLAGS_MACVERSION_SFT) & PHFLAGS_MACVERSION_MASK));
    ani_log = (struct ath_pktlog_ani *)pktlog_getbuf(pl_info, PKTLOG_TYPE_ANI, 
                  sizeof(*ani_log) + misc_cnt * sizeof(ani_log->misc[0]), flags);

    ani_log->phyStatsDisable = log_data->phyStatsDisable;
    ani_log->noiseImmunLvl = log_data->noiseImmunLvl;
    ani_log->spurImmunLvl = log_data->spurImmunLvl;
    ani_log->ofdmWeakDet = log_data->ofdmWeakDet;
    ani_log->cckWeakThr = log_data->cckWeakThr;
    ani_log->firLvl = log_data->firLvl;
    ani_log->listenTime = (u_int16_t) (log_data->listenTime);
    ani_log->cycleCount = log_data->cycleCount;
    ani_log->ofdmPhyErrCount = log_data->ofdmPhyErrCount;
    ani_log->cckPhyErrCount = log_data->cckPhyErrCount;
    ani_log->rssi = log_data->rssi;

    for (i = 0; i < misc_cnt; i++)
        ani_log->misc[i] = log_data->misc[i];
}

void
pktlog_rcfind(struct ath_softc *sc, struct log_rcfind *log_data, u_int16_t iflags)
{
    struct ath_pktlog_rcfind *rcf_log;
    struct TxRateCtrl_s *pRc = log_data->rc;
    int i;
    u_int8_t misc_cnt;
    struct ath_pktlog_info *pl_info;
	u_int32_t	flags = iflags;

    if (g_pktlog_mode == PKTLOG_MODE_ADAPTER)
        pl_info = sc->pl_info;
    else
        pl_info = g_pktlog_info;

    if ((pl_info->log_state & ATH_PKTLOG_RCFIND) == 0)
        return;

    misc_cnt = flags & PHFLAGS_MISCCNT_MASK;
    flags |= (((sc->sc_ah->ah_macRev << PHFLAGS_MACREV_SFT) & PHFLAGS_MACREV_MASK) |
		((sc->sc_ah->ah_macVersion << PHFLAGS_MACVERSION_SFT) & PHFLAGS_MACVERSION_MASK));
    rcf_log = (struct ath_pktlog_rcfind *)pktlog_getbuf(pl_info, PKTLOG_TYPE_RCFIND, 
                  sizeof(*rcf_log) + misc_cnt * sizeof(rcf_log->misc[0]), flags);

    rcf_log->rate = log_data->rate;
    rcf_log->rateCode = log_data->rateCode;
    rcf_log->rcRssiLast = pRc->rssiLast;
    rcf_log->rcRssiLastPrev = pRc->rssiLastPrev;
    rcf_log->rcRssiLastPrev2 = pRc->rssiLastPrev2;
    rcf_log->rssiReduce = log_data->rssiReduce;
    rcf_log->rcProbeRate = log_data->isProbing? pRc->probeRate:0;
    rcf_log->isProbing = log_data->isProbing;
    rcf_log->primeInUse = log_data->primeInUse;
    rcf_log->currentPrimeState = log_data->currentPrimeState;
    rcf_log->ac = log_data->ac;
    rcf_log->rcRateMax = pRc->rateMaxPhy;
    rcf_log->rcRateTableSize = pRc->rateTableSize;

    for (i = 0; i < misc_cnt; i++)
        rcf_log->misc[i] = log_data->misc[i];
}

void
pktlog_rcupdate(struct ath_softc *sc, struct log_rcupdate *log_data, u_int16_t iflags)
{
    struct ath_pktlog_rcupdate *rcu_log;
    struct TxRateCtrl_s *pRc = log_data->rc;
    int i;
    u_int8_t misc_cnt;
    struct ath_pktlog_info *pl_info;
	u_int32_t	flags = iflags;

    if (g_pktlog_mode == PKTLOG_MODE_ADAPTER)
        pl_info = sc->pl_info;
    else
        pl_info = g_pktlog_info;

    if ((pl_info->log_state & ATH_PKTLOG_RCUPDATE) == 0)
        return;

    misc_cnt = flags & PHFLAGS_MISCCNT_MASK;
    flags |= (((sc->sc_ah->ah_macRev << PHFLAGS_MACREV_SFT) & PHFLAGS_MACREV_MASK) |
		((sc->sc_ah->ah_macVersion << PHFLAGS_MACVERSION_SFT) & PHFLAGS_MACVERSION_MASK));
    rcu_log = (struct ath_pktlog_rcupdate *)pktlog_getbuf(pl_info, PKTLOG_TYPE_RCUPDATE, 
                  sizeof(*rcu_log) + misc_cnt * sizeof(rcu_log->misc[0]), flags);

    rcu_log->txRate = log_data->txRate;
    rcu_log->rateCode = log_data->rateCode;
    rcu_log->Xretries = log_data->Xretries;
    rcu_log->retries = log_data->retries;
    rcu_log->rssiAck = log_data->rssiAck;
    rcu_log->ac = log_data->ac;
    rcu_log->rcRssiLast = pRc->rssiLast;
    rcu_log->rcRssiLastLkup = pRc->rssiLastLkup;
    rcu_log->rcRssiLastPrev = pRc->rssiLastPrev;
    rcu_log->rcRssiLastPrev2 = pRc->rssiLastPrev2;
    rcu_log->rcProbeRate = pRc->probeRate;
    rcu_log->rcRateMax = pRc->rateMaxPhy;
    rcu_log->useTurboPrime = log_data->useTurboPrime;
    rcu_log->currentBoostState = log_data->currentBoostState;
    rcu_log->rcHwMaxRetryRate = pRc->hwMaxRetryRate;

    for (i = 0; i < MAX_TX_RATE_TBL; i++) {
        rcu_log->rcRssiThres[i] = pRc->state[i].rssiThres;
        rcu_log->rcPer[i] = pRc->state[i].per;
    }

    for (i = 0; i < misc_cnt; i++)
        rcu_log->misc[i] = log_data->misc[i];
}

int
pktlog_text(struct ath_softc *sc, char *tbuf, u_int16_t iflags)
{
    size_t len;
    struct ath_pktlog_info *pl_info;
    u_int8_t *data;
	u_int32_t	flags = iflags;

    if (!tbuf)
        return 0;

    if (g_pktlog_mode == PKTLOG_MODE_ADAPTER)
        pl_info = sc->pl_info;
    else
        pl_info = g_pktlog_info;

    if ((pl_info->log_state & ATH_PKTLOG_TEXT) == 0)
        return 0;

    flags |= (((sc->sc_ah->ah_macRev << PHFLAGS_MACREV_SFT) & PHFLAGS_MACREV_MASK) |
		((sc->sc_ah->ah_macVersion << PHFLAGS_MACVERSION_SFT) & PHFLAGS_MACVERSION_MASK));
    len = strlen(tbuf);

    data = (u_int8_t *)pktlog_getbuf(pl_info, PKTLOG_TYPE_TEXT, len, flags);
    OS_MEMCPY(data, tbuf, len);
    return 1;
}

#ifndef MIN
#define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

static u_int
pktlog_read(struct ath_pktlog_info *pl_info, u_int8_t *buf, int nbytes, u_int *ppos)
{
    u_int bufhdr_size;
    u_int count = 0, ret_val = 0;
    int rem_len;
    int start_offset, end_offset;
    int fold_offset, ppos_data, cur_rd_offset;
    struct ath_pktlog_buf *log_buf = pl_info->buf;
   
    if (log_buf == NULL)
        return 0;

    if (*ppos == 0 && pl_info->log_state) {
        pl_info->saved_state = pl_info->log_state;
        pl_info->log_state = 0;
    }

    bufhdr_size = sizeof(log_buf->bufhdr);

    /* copy valid log entries from circular buffer into user buffer */
    rem_len = nbytes;

    count = 0;

    if (*ppos < bufhdr_size) {
        count = MIN((bufhdr_size - *ppos), rem_len);
        OS_MEMCPY(buf, ((u_int8_t *) &log_buf->bufhdr) + *ppos, count);
        rem_len -= count;
        ret_val += count;
    }

    start_offset = log_buf->rd_offset;

    if ((rem_len == 0) || (start_offset < 0))
        goto rd_done;

    fold_offset = -1;
    cur_rd_offset = start_offset;

    /* Find the last offset and fold-offset if the buffer is folded */
    do {
        struct ath_pktlog_hdr *log_hdr;
        int log_data_offset;

        log_hdr =
            (struct ath_pktlog_hdr *) (log_buf->log_data + cur_rd_offset);

        log_data_offset = cur_rd_offset + sizeof(struct ath_pktlog_hdr);

        if ((fold_offset == -1)
            && ((pl_info->buf_size - log_data_offset) <= log_hdr->size))
            fold_offset = log_data_offset - 1;

        PKTLOG_MOV_RD_IDX(cur_rd_offset, log_buf, pl_info->buf_size);

        if ((fold_offset == -1) && (cur_rd_offset == 0)
            && (cur_rd_offset != log_buf->wr_offset))
            fold_offset = log_data_offset + log_hdr->size - 1;

        end_offset = log_data_offset + log_hdr->size - 1;
    } while (cur_rd_offset != log_buf->wr_offset);

    ppos_data = *ppos + ret_val - bufhdr_size + start_offset;

    if (fold_offset == -1) {
        if (ppos_data > end_offset)
            goto rd_done;

        count = MIN(rem_len, (end_offset - ppos_data + 1));
        OS_MEMCPY(buf + ret_val, log_buf->log_data + ppos_data, count);
        ret_val += count;
        rem_len -= count;
    } else {
        if (ppos_data <= fold_offset) {
            count = MIN(rem_len, (fold_offset - ppos_data + 1));
            OS_MEMCPY(buf + ret_val, log_buf->log_data + ppos_data,
                      count);
            ret_val += count;
            rem_len -= count;
        }

        if (rem_len == 0)
            goto rd_done;

        ppos_data =
            *ppos + ret_val - (bufhdr_size +
                               (fold_offset - start_offset + 1));

        if (ppos_data <= end_offset) {
            count = MIN(rem_len, (end_offset - ppos_data + 1));
            OS_MEMCPY(buf + ret_val, log_buf->log_data + ppos_data,
                      count);
            ret_val += count;
            rem_len -= count;
        }
    }

rd_done:
    *ppos += ret_val;
    return ret_val;
}

int
pktlog_start(struct ath_softc *sc, int log_state)
{
    struct ath_pktlog_info *pl_info = sc->pl_info;
    int error = 0;

    if (log_state == 0) {
        /* use default log_state */
        log_state = ATH_PKTLOG_TX | ATH_PKTLOG_RX | ATH_PKTLOG_ANI |
            ATH_PKTLOG_RCFIND | ATH_PKTLOG_RCUPDATE | ATH_PKTLOG_TEXT;
    }
    
    if (pl_info->log_state)     /* already started, do nothing */
        return 0;
    
    if (pl_info->saved_state) {
        /* restore previous log state and log buffer */
        pl_info->log_state = pl_info->saved_state;
        pl_info->saved_state = 0;
    } else {
        error = pktlog_enable(sc, log_state);
    }

    return error;
}

int
pktlog_read_hdr(struct ath_softc *sc, void *buf, u_int32_t buf_len,
                u_int32_t *required_len, u_int32_t *actual_len)
{
    struct ath_pktlog_info *pl_info = sc->pl_info;
    struct ath_pktlog_buf *log_buf = pl_info->buf;
    u_int pos;
   
    if (log_buf == NULL)
        return 0;

    if (buf_len < sizeof(log_buf->bufhdr)) {
        *required_len = sizeof (struct ath_pktlog_bufhdr);
        return -EINVAL;
        
    }
    
    pos = 0;
    *actual_len = pktlog_read(pl_info, buf, sizeof(log_buf->bufhdr), &pos);
    
    return 0;
}

int
pktlog_read_buf(struct ath_softc *sc, void *buf, u_int32_t buf_len,
                u_int32_t *required_len, u_int32_t *actual_len)
{
    struct ath_pktlog_info *pl_info = sc->pl_info;
    struct ath_pktlog_buf *log_buf = pl_info->buf;
    u_int pos;
   
    if (log_buf == NULL)
        return 0;

    if (buf_len < pl_info->buf_size) {
        *required_len = pl_info->buf_size;
        return -EINVAL;
    }

    pos = sizeof(log_buf->bufhdr);
    *actual_len = pktlog_read(pl_info, buf, pl_info->buf_size, &pos);
    
    return 0;
}

#endif /* REMOVE_PKT_LOG */
