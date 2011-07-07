/*****************************************************************************/
/*! \file ath_desc.c
**  \brief ATH Descriptor Processing
**    
**  This file contains the functionality for management of descriptor queues
**  in the ATH object.
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
*/

#include "ath_internal.h"

/******************************************************************************/
/*!
**  \brief Set up DMA descriptors
**
**  This function will allocate both the DMA descriptor structure, and the
**  buffers it contains.  These are used to contain the descriptors used
**  by the system.
**
**  \param sc Pointer to ATH object ("this" pointer)
**  \param dd Pointer to DMA descriptor structure being allocated
**  \param head Pointer to queue head that manages the DMA descriptors
**  \param name String that names the dma descriptor object
**  \param nbuf Number of buffers to allocate for this object
**  \param ndesc Number of descriptors used for each buffer.  Used for fragmentation.
**
**  \return -ENOMEM if unable to allocate memory
**  \return   0 upon success
*/

int
ath_descdma_setup(
    struct ath_softc *sc,
    struct ath_descdma *dd, ath_bufhead *head,
    const char *name, int nbuf, int ndesc)
{
#define	DS2PHYS(_dd, _ds) \
    ((_dd)->dd_desc_paddr + ((caddr_t)(_ds) - (caddr_t)(_dd)->dd_desc))
#define ATH_DESC_4KB_BOUND_CHECK(_daddr) ((((_daddr) & 0xFFF) > 0xF7F) ? 1 : 0)
#define ATH_DESC_4KB_BOUND_NUM_SKIPPED(_len) ((_len) / 4096)

    struct ath_desc *ds;
    struct ath_buf *bf;
    int i, bsize, error;

    DPRINTF(sc, ATH_DEBUG_RESET, "%s: %s DMA: %u buffers %u desc/buf\n",
             __func__, name, nbuf, ndesc);

    /* ath_desc must be a multiple of DWORDs */
    if ((sizeof(struct ath_desc) % 4) != 0) {
        DPRINTF(sc, ATH_DEBUG_FATAL, "%s: ath_desc not DWORD aligned\n",
                __func__ );
        ASSERT((sizeof(struct ath_desc) % 4) == 0);
        error = -ENOMEM;
        goto fail;
    }

    dd->dd_name = name;
    dd->dd_desc_len = sizeof(struct ath_desc) * nbuf * ndesc;

    /* 
     * WAR for bug 30982 (Merlin)
     * Need additional DMA memory because we can't use descriptors that cross the 
     * 4K page boundary. Assume one skipped descriptor per 4K page.
     */
    if (!ath_hal_has4kbsplittrans(sc->sc_ah)) {
        u_int32_t ndesc_skipped = ATH_DESC_4KB_BOUND_NUM_SKIPPED(dd->dd_desc_len);
        u_int32_t dma_len;

        while (ndesc_skipped) {
            dma_len = ndesc_skipped * sizeof(struct ath_desc);
            dd->dd_desc_len += dma_len;
            
            ndesc_skipped = ATH_DESC_4KB_BOUND_NUM_SKIPPED(dma_len);
        };
    }

    /* allocate descriptors */
    dd->dd_desc = (struct ath_desc *)OS_MALLOC_CONSISTENT(sc->sc_osdev,
                                       dd->dd_desc_len, &dd->dd_desc_paddr,
                                       OS_GET_DMA_MEM_CONTEXT(dd, dd_dmacontext),
                                       sc->sc_reg_parm.shMemAllocRetry);
    if (dd->dd_desc == NULL) {
        error = -ENOMEM;
        goto fail;
    }
    ds = dd->dd_desc;
    DPRINTF(sc, ATH_DEBUG_RESET, "%s: %s DMA map: %p (%u) -> %llx (%u)\n",
            __func__, dd->dd_name, ds, (u_int32_t) dd->dd_desc_len,
            ito64(dd->dd_desc_paddr), /*XXX*/ (u_int32_t) dd->dd_desc_len);

    /* allocate buffers */
    bsize = sizeof(struct ath_buf) * nbuf;
    bf = (struct ath_buf *)OS_MALLOC(sc->sc_osdev, bsize, GFP_KERNEL);
    if (bf == NULL) {
        error = -ENOMEM;
        goto fail2;
    }
    OS_MEMZERO(bf, bsize);
    dd->dd_bufptr = bf;

    TAILQ_INIT(head);
    for (i = 0; i < nbuf; i++, bf++, ds += ndesc) {
        bf->bf_desc = ds;
        bf->bf_daddr = DS2PHYS(dd, ds);

        if (!ath_hal_has4kbsplittrans(sc->sc_ah)) {
            /* 
             * WAR for bug 30982.
             * Skip descriptor addresses which can cause 4KB boundary crossing (addr + length)
             * with a 32 dword descriptor fetch. 
             */
            while (ATH_DESC_4KB_BOUND_CHECK(bf->bf_daddr)) {
                ASSERT((caddr_t)bf->bf_desc < ((caddr_t)dd->dd_desc + dd->dd_desc_len));

                ds += ndesc;
                bf->bf_desc = ds;
                bf->bf_daddr = DS2PHYS(dd, ds);
            }
        }
        TAILQ_INSERT_TAIL(head, bf, bf_list);
    }

	/* 
	 * For OS's that need to allocate dma context to be used to 
	 * send down to hw, do that here. (BSD is the only one that needs
	 * it currently.)
	 */ 
	ALLOC_DMA_CONTEXT_POOL(sc->sc_osdev, name, nbuf);

    return 0;
fail2:
    OS_FREE_CONSISTENT(sc->sc_osdev, dd->dd_desc_len,
                       dd->dd_desc, dd->dd_desc_paddr,
                       OS_GET_DMA_MEM_CONTEXT(dd, dd_dmacontext));
fail:
	OS_MEMZERO(dd, sizeof(*dd));
	return error;
#undef ATH_DESC_4KB_BOUND_CHECK
#undef ATH_DESC_4KB_BOUND_NUM_SKIPPED
#undef DS2PHYS
}

/******************************************************************************/
/*!
**  \brief Cleanup DMA descriptors
**
**  This function will free the DMA block that was allocated for the descriptor
**  pool.  Since this was allocated as one "chunk", it is freed in the same
**  manner.
**
**  \param sc Pointer to ATH object ("this" pointer)
**  \param dd Pointer to dma descriptor object that is being freed
**  \param head Pointer to queue head containing the pointer to the queue.
**
**  \return N/A
*/

void
ath_descdma_cleanup(
    struct ath_softc *sc,
    struct ath_descdma *dd,
    ath_bufhead *head)
{
	/* Free memory associated with descriptors */
	OS_FREE_CONSISTENT(sc->sc_osdev, dd->dd_desc_len,
                       dd->dd_desc, dd->dd_desc_paddr,
                       OS_GET_DMA_MEM_CONTEXT(dd, dd_dmacontext));

	TAILQ_INIT(head);
	OS_FREE(dd->dd_bufptr);
	OS_MEMZERO(dd, sizeof(*dd));
}

/******************************************************************************/
/*!
**  \brief Endian Swap for transmit descriptor
**
**  Since the hardware expects all 32 bit values and pointers to be in little
**  endian mode, this function is called in case the cpu is in big endian mode.
**  The AH_NEED_DESC_SWAP define is used to flag this condition.
**
**  \param ds Pointer to descriptor to be "fixed"
**
**  \return N/A
*/

void
ath_desc_swap(struct ath_desc *ds)
{
#ifdef AH_NEED_DESC_SWAP
    ds->ds_link = cpu_to_le32(ds->ds_link);
    ds->ds_data = cpu_to_le32(ds->ds_data);
    ds->ds_ctl0 = cpu_to_le32(ds->ds_ctl0);
    ds->ds_ctl1 = cpu_to_le32(ds->ds_ctl1);
    ds->ds_hw[0] = cpu_to_le32(ds->ds_hw[0]);
    ds->ds_hw[1] = cpu_to_le32(ds->ds_hw[1]);
#endif
}
