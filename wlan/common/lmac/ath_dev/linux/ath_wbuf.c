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
 */

#include <osdep.h>
#include <wbuf.h>

#include "ath_internal.h"

static struct sk_buff *
ath_alloc_skb(u_int size, u_int align)
{
    struct sk_buff *skb;
    u_int off;

    skb = dev_alloc_skb(size + align - 1);
    if (skb != NULL) {
        off = ((unsigned long) skb->data) % align;
        if (off != 0)
            skb_reserve(skb, align - off);
    }
    return skb;
}

struct sk_buff *
ath_rxbuf_alloc(struct ath_softc *sc, u_int32_t len)
{
    struct sk_buff *skb;
    struct net_device *dev = sc->sc_osdev->netdev;

    if (sc->sc_opmode == HAL_M_MONITOR) {
        u_int off;
        /*
         * Allocate buffer for monitor mode with space for the
         * wlan-ng style physical layer header at the start.
         */
        skb = dev_alloc_skb(len +
                            sizeof(wlan_ng_prism2_header) +
                            sc->sc_cachelsz - 1);
        if (skb == NULL) {
            DPRINTF(sc, ATH_DEBUG_ANY,
                    "%s: skbuff alloc of size %zu failed\n",
                    __func__,
                    len
                    + sizeof(wlan_ng_prism2_header)
                    + sc->sc_cachelsz -1);
            return NULL;
        }
        /*
         * Reserve space for the Prism header.
         */
        skb_reserve(skb, sizeof(wlan_ng_prism2_header));
        /*
         * Align to cache line.
         */
        off = ((unsigned long) skb->data) % sc->sc_cachelsz;
        if (off != 0)
            skb_reserve(skb, sc->sc_cachelsz - off);
	} else {
        ASSERT(sizeof(struct ieee80211_cb) + sizeof(void *) <= sizeof(skb->cb));
        /*
         * Cache-line-align.  This is important (for the
         * 5210 at least) as not doing so causes bogus data
         * in rx'd frames.
         */
        skb = ath_alloc_skb(len, sc->sc_cachelsz);
        if (skb == NULL) {
            DPRINTF(sc, ATH_DEBUG_ANY,
                    "%s: skbuff alloc of size %u failed\n",
                    __func__, len);
			return NULL;
        }
    }

    skb->dev = dev;

    /*
     * setup rx context in skb
     * XXX: this violates the rule of ath_dev,
     * which is supposed to be protocol independent.
     */
    ((struct ieee80211_cb *)skb->cb)->context =
        &(((struct ieee80211_cb *)skb->cb)[1]);
    
    return skb;
}
EXPORT_SYMBOL(ath_rxbuf_alloc);

struct sk_buff *
wbuf_alloc(osdev_t os_handle, enum wbuf_type type, u_int32_t len)
{
    return dev_alloc_skb(len);
}
EXPORT_SYMBOL(wbuf_alloc);

void
wbuf_release(osdev_t os_handle, enum wbuf_type type, struct sk_buff *skb)
{
    dev_kfree_skb(skb);
}

int wbuf_start_dma(wbuf_t wbuf, sg_t *sg, u_int32_t n_sg, void *arg)
{
    return ath_tx_start_dma(wbuf, sg, n_sg, arg);
}

/* decrement with wrap-around */
#define DECR(_l,  _sz)  (_l)--; (_l) &= ((_sz) - 1)
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22))
  #define  UNI_SKB_END_POINTER(skb)   (skb)->end
#else
  #define  UNI_SKB_END_POINTER(skb)    skb_end_pointer(skb)
#endif

void
__wbuf_map_sg(osdev_t osdev, struct sk_buff *skb, dma_addr_t *pa, void *arg)
{
    struct scatterlist sg;

    *pa = bus_map_single(osdev->bdev, skb->data, UNI_SKB_END_POINTER(skb) - skb->data, BUS_DMA_TODEVICE);

    /* setup S/G list */
    memset(&sg, 0, sizeof(struct scatterlist));
    sg_dma_address(&sg) = *pa;
    sg_dma_len(&sg) = skb->len;

    if (wbuf_start_dma(skb, &sg, 1, arg) != 0) {
        ieee80211_tx_control_t *txctl = (ieee80211_tx_control_t *)arg; /* XXX */
        struct ath_softc *sc = (struct ath_softc *)txctl->dev;
        ieee80211_tx_status_t tx_status;
        struct ath_atx_tid *tid;
        struct ath_txq *txq = &sc->sc_txq[txctl->qnum];

        /*
         * NB: common code doesn't tail drop frame
         * because it's not allowed in NDIS 6.0.
         * For Linux, we have to do it here.
         */
        bus_unmap_single(osdev->bdev, *pa, UNI_SKB_END_POINTER(skb) - skb->data, BUS_DMA_TODEVICE);

        tx_status.retries = 0;
        tx_status.flags = ATH_TX_ERROR;

#ifdef ATH_RIFS
        if (txctl->ht && (sc->sc_txaggr || sc->sc_txrifs)) {
#else
        if (txctl->ht && sc->sc_txaggr) {
#endif
            // Reclaim the seqno.
            ATH_TXQ_LOCK(txq);
            tid = ATH_AN_2_TID((struct ath_node *)txctl->an, txctl->tidno);
            DECR(tid->seq_next, IEEE80211_SEQ_MAX);
            ATH_TXQ_UNLOCK(txq);
        }

        sc->sc_ieee_ops->tx_complete(skb, &tx_status);
    }
}

void
__wbuf_unmap_sg(osdev_t osdev, struct sk_buff *skb, dma_addr_t *pa)
{
    bus_unmap_single(osdev->bdev, *pa, UNI_SKB_END_POINTER(skb) - skb->data, BUS_DMA_TODEVICE);
}

dma_addr_t
__wbuf_map_single(osdev_t osdev, struct sk_buff *skb, int direction, dma_addr_t *pa)
{
    /*
     * NB: do NOT use skb->len, which is 0 on initialization.
     * Use skb's entire data area instead.
     */
    *pa = bus_map_single(osdev->bdev, skb->data, UNI_SKB_END_POINTER(skb) - skb->data, direction);

    return *pa;
}

void
__wbuf_unmap_single(osdev_t osdev, struct sk_buff *skb, int direction, dma_addr_t *pa)
{
    /*
     * Unmap skb's entire data area.
     */
    bus_unmap_single(osdev->bdev, *pa, UNI_SKB_END_POINTER(skb) - skb->data, direction);
}
