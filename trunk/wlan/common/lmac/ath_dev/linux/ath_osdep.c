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
 *  This module contains an interface/shim layer between the HW layer and our 
 *  ath layer.  
 */

#include <osdep.h>
#include <wbuf.h>

#include "ath_internal.h"

#include <net80211/_ieee80211.h>

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#endif

/*
 * Indication of hardware PHY state change
 */
void
ath_hw_phystate_change(struct ath_softc *sc, int newstate)
{
    if (sc->sc_hw_phystate == newstate)
        return;

    sc->sc_hw_phystate = newstate;  /* save the hardware switch state */

    /* TODO: rfkill support in Linux*/
	if (newstate)
		ath_radio_enable(sc);
    else
    	ath_radio_disable(sc);
}

void ath_internal_reset(struct ath_softc *sc)
{
    ath_reset_start(sc, 0);
    ath_reset(sc);
    ath_reset_end(sc, 0);
}

void ath_handle_tx_intr(struct ath_softc *sc)
{
    ath_tx_tasklet(sc);
}

void ath_handle_rx_intr(struct ath_softc *sc)
{
    ath_rx_tasklet(sc, 0);
}

/*
 * Linux's specific rx indicator
 * 
 * In Linux, the wbuf indicated to upper stack won't be returned to us.
 * So we have to allocate a new one and queue it by ourselves.
 */
int ath_rx_indicate(struct ath_softc *sc, wbuf_t wbuf, ieee80211_rx_status_t *status, u_int16_t keyix)
{
    struct ath_buf *bf = ATH_RX_CONTEXT(wbuf)->ctx_rxbuf;
    wbuf_t nwbuf;
    int type;

    type=-1;    
    /* allocate a new wbuf and queue it to for H/W processing */
    nwbuf = ath_rxbuf_alloc(sc, sc->sc_rxbufsize);
    if (nwbuf != NULL) {

        /* indicate frame to the stack, which will free the old wbuf. */
        type = sc->sc_ieee_ops->rx_indicate(sc->sc_ieee, wbuf, status, keyix);

        bf->bf_mpdu = nwbuf;
        bf->bf_buf_addr = wbuf_map_single(sc->sc_osdev, nwbuf, BUS_DMA_FROMDEVICE,
                                          OS_GET_DMA_MEM_CONTEXT(bf, bf_dmacontext));
        ATH_RX_CONTEXT(nwbuf)->ctx_rxbuf = bf;

        /* queue the new wbuf to H/W */
        ath_rx_requeue(sc, nwbuf);
    } else {
        
        /* Could not allocate the buffer
         * give the wbuf back 
         */
        bf->bf_buf_addr = wbuf_map_single(sc->sc_osdev, wbuf, BUS_DMA_FROMDEVICE,
                                          OS_GET_DMA_MEM_CONTEXT(bf, bf_dmacontext));

        /* queue back the old wbuf to H/W */
        ath_rx_requeue(sc, wbuf);

    }

    return type;
}

#ifndef ATH_WLAN_COMBINE
/*
 * Linux module glue
 */
static char *dev_info = "ath_dev";

MODULE_AUTHOR("Atheros Communications, Inc.");
MODULE_DESCRIPTION("Atheros Device Module");
#ifdef MODULE_LICENSE
MODULE_LICENSE("Proprietary");
#endif

static int __init
init_ath_dev(void)
{
    /* XXX version is a guess; where should it come from? */
    printk(KERN_INFO "%s: "
           "Copyright (c) 2001-2007 Atheros Communications, Inc, "
           "All Rights Reserved\n", dev_info);

    return 0;
}
module_init(init_ath_dev);

static void __exit
exit_ath_dev(void)
{
	printk(KERN_INFO "%s: driver unloaded\n", dev_info);
}
module_exit(exit_ath_dev);

EXPORT_SYMBOL(ath_dev_attach);
EXPORT_SYMBOL(ath_dev_free);
EXPORT_SYMBOL(ath_initialize_timer);
EXPORT_SYMBOL(ath_cancel_timer);
EXPORT_SYMBOL(ath_start_timer);
EXPORT_SYMBOL(ath_timer_is_active);
#endif /* #ifndef ATH_WLAN_COMBINE */

#ifndef REMOVE_PKT_LOG
#include "pktlog.h"
extern struct ath_pktlog_funcs *g_pktlog_funcs;
EXPORT_SYMBOL(g_pktlog_funcs);
#endif
