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

#include "if_athvar.h"
#include <osdep.h>

int
ath_limit_legacy_create_freeq(struct ath_softc_net80211 *scn)
{
    int i;
    struct ath_limit_legacy_buf *q_buf=NULL;

    TAILQ_INIT(&scn->sc_limit_legacy_freeq);
    ATH_LIMIT_LEGACY_FREEQ_LOCK(scn); 

    for (i=0; i< 25; i++) {
        q_buf = (struct ath_limit_legacy_buf*)OS_MALLOC(scn->sc_osdev, sizeof(struct ath_limit_legacy_buf), GFP_KERNEL);
        if (q_buf) {
            OS_MEMZERO(q_buf, sizeof(struct ath_limit_legacy_buf));
            q_buf->wb = NULL;
            TAILQ_INSERT_TAIL(&scn->sc_limit_legacy_freeq, q_buf, qelem);
        } else
            goto bad;
    }
    ATH_LIMIT_LEGACY_FREEQ_UNLOCK(scn); 
    return 0;
bad:
    ATH_LIMIT_LEGACY_FREEQ_UNLOCK(scn); 
    (void) ath_limit_legacy_destroy_freeq(scn); 
    return 1;
}

int
ath_limit_legacy_destroy_freeq(struct ath_softc_net80211 *scn)
{
    struct ath_limit_legacy_buf *q_buf=NULL;

    ATH_LIMIT_LEGACY_FREEQ_LOCK(scn); 

     TAILQ_FOREACH(q_buf, &scn->sc_limit_legacy_freeq, qelem) {
        if(q_buf) {
                TAILQ_REMOVE(&scn->sc_limit_legacy_freeq, q_buf, qelem);
                OS_FREE(q_buf);
        }
    }
    ATH_LIMIT_LEGACY_FREEQ_UNLOCK(scn); 
    return 0;
}

int
ath_limit_legacy_attach(struct ath_softc_net80211 *scn)
{
    TAILQ_INIT(&scn->sc_limit_legacy_txq);
    ATH_LIMIT_LEGACY_TXQ_LOCK_INIT(scn);
    ATH_LIMIT_LEGACY_FREEQ_LOCK_INIT(scn);
    scn->sc_limit_legacy_txq_count=0;
    (void) ath_limit_legacy_create_freeq(scn);
    OS_INIT_TIMER(scn->sc_osdev, &scn->sc_limit_legacy_flush_timer, limit_legacy_flush_timeout, scn);
    return 0;
}

void
ath_limit_legacy_detach(struct ath_softc_net80211 *scn)
{
    OS_CANCEL_TIMER(&scn->sc_limit_legacy_flush_timer);
    (void) ath_limit_legacy_destroy_freeq(scn);
    ATH_LIMIT_LEGACY_TXQ_LOCK_DESTROY(scn);
    ATH_LIMIT_LEGACY_FREEQ_LOCK_DESTROY(scn);
}

OS_TIMER_FUNC(limit_legacy_flush_timeout)
{
        struct ath_softc_net80211 *scn = NULL;
        struct ath_limit_legacy_buf *q_buf=NULL;

	OS_GET_TIMER_ARG(scn, struct ath_softc_net80211*);
        
	if (scn) {
            TAILQ_FOREACH(q_buf, &scn->sc_limit_legacy_txq, qelem) {
                deq_legacy_wbuf(scn);
            }
	}
	return;
}

void enq_legacy_wbuf(struct ath_softc_net80211 *scn, wbuf_t wb)
{
    struct ath_limit_legacy_buf *q_buf=NULL;

    ATH_LIMIT_LEGACY_FREEQ_LOCK(scn); 

    q_buf=TAILQ_FIRST(&scn->sc_limit_legacy_freeq);

    if(q_buf) {
        TAILQ_REMOVE(&scn->sc_limit_legacy_freeq, q_buf, qelem);
        ATH_LIMIT_LEGACY_FREEQ_UNLOCK(scn); 
        q_buf->wb = wb;

        ATH_LIMIT_LEGACY_TXQ_LOCK(scn); 

        TAILQ_INSERT_TAIL(&scn->sc_limit_legacy_txq, q_buf, qelem);
        scn->sc_limit_legacy_txq_count++;

        ATH_LIMIT_LEGACY_TXQ_UNLOCK(scn); 
        OS_SET_TIMER(&scn->sc_limit_legacy_flush_timer, LIMIT_LEGACY_TIMEOUT_MS);

    } else {
        ATH_LIMIT_LEGACY_FREEQ_UNLOCK(scn); 
    }
}

int deq_legacy_wbuf(struct ath_softc_net80211 *scn)
{
    int error=0;
    struct ath_limit_legacy_buf *q_buf=NULL;

    OS_CANCEL_TIMER(&scn->sc_limit_legacy_flush_timer);

    ATH_LIMIT_LEGACY_TXQ_LOCK(scn); 

    q_buf=TAILQ_FIRST(&scn->sc_limit_legacy_txq);
    if(q_buf) {
       TAILQ_REMOVE(&scn->sc_limit_legacy_txq, q_buf, qelem);
        scn->sc_limit_legacy_txq_count--;
    }
    ATH_LIMIT_LEGACY_TXQ_UNLOCK(scn); 

    if(q_buf) {
        error = ath_tx_send(q_buf->wb);
        ATH_LIMIT_LEGACY_FREEQ_LOCK(scn); 
        TAILQ_INSERT_TAIL(&scn->sc_limit_legacy_freeq, q_buf, qelem);
        ATH_LIMIT_LEGACY_FREEQ_UNLOCK(scn); 
    }
    return error;

}

