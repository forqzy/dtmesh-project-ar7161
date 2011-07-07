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
 *  Implementation of Timer module.
 */

#include "ath_internal.h"
#include "ath_timer.h"

#define TIMER_SIGNATURE    0xABCD9876

static
OS_TIMER_FUNC(_ath_internal_timer_handler)
{
    struct ath_timer    *timer_object;

    OS_GET_TIMER_ARG(timer_object, struct ath_timer *);

    /* 
     * Acquire the synchronization object. 
     * This is used to synchronize with routine ath_cancelTimer.
     */
    if (cmpxchg(&(timer_object->timer_lock), 0, 1) == 1) {
        return;
    }

    // if timer is being cancelled, do not run handler, and do not rearm timer.
    if (timer_object->cancel_flag) {
        // release the synchronization object
        (void) cmpxchg(&(timer_object->timer_lock), 1, 0);
        return;
    }

    // run timer handler
    if (timer_object->timer_handler(timer_object->context) == 0) {
        // rearm timer only if handler function returned 0
        OS_SET_TIMER(&timer_object->os_timer, timer_object->timer_period);
    }
    else {
        // timer not rearmed - no longer active
        timer_object->active_flag = 0;
    }

    // release the synchronization object
    (void) cmpxchg(&(timer_object->timer_lock), 1, 0); 
}

u_int8_t ath_initialize_timer_module (osdev_t sc_osdev)
{
    UNREFERENCED_PARAMETER(sc_osdev);

    return 1;
}

u_int8_t ath_initialize_timer (osdev_t                osdev,
                               struct ath_timer*      timer_object, 
                               u_int32_t              timer_period, 
                               timer_handler_function timer_handler, 
                               void*                  context)
{
    ASSERT(timer_object != NULL);
    ASSERT(osdev != NULL);

    if (timer_object != NULL) {
        OS_INIT_TIMER(osdev, &timer_object->os_timer, _ath_internal_timer_handler, timer_object);

        timer_object->timer_lock    = 0;
        timer_object->active_flag   = 0;
        timer_object->cancel_flag   = 0;
        timer_object->timer_period  = timer_period;
        timer_object->context       = context;
        timer_object->timer_handler = timer_handler;
        timer_object->signature     = TIMER_SIGNATURE;

        return 1;
    }

    return 0;
}

void ath_set_timer_period (struct ath_timer* timer_object, u_int32_t timer_period)
{
    timer_object->timer_period  = timer_period;
}

u_int8_t ath_start_timer (struct ath_timer* timer_object)
{
    // mark timer as active
    timer_object->active_flag = 1;
    timer_object->cancel_flag = 0;

    // arm timer for the first time
    OS_SET_TIMER(&timer_object->os_timer, timer_object->timer_period);

    return 1;
}

/*
 * ath_cancelTimer: Argument busywait_flag indicates whether the 
 * delay function used by _ath_internal_cancelTimer will perform a 
 * "busy wait" or relinquish access to the CPU. The former can
 * be called at IRQL <= DISPATCH_LEVEL, while the latter is only
 * valid at IRQL < DISPATCH_LEVEL.
 */
u_int8_t ath_cancel_timer (struct ath_timer* timer_object, enum timer_flags flags)
{
    int         tick_counter = 0;
    u_int8_t    canceled     = 1;

    // indicate timer is being cancelled
    timer_object->cancel_flag = 1;

    // nothing to do if timer not active.
    if (! timer_object->active_flag) {
        return 1;
    }

    // try to acquire LED synchronization object
    while (cmpxchg(&(timer_object->timer_lock), 0, 1) == 1) {
        if (tick_counter++ > 1000) {    // no more than 10ms
            break;
        }

        if (flags == CANCEL_NO_SLEEP) {
            OS_DELAY(10);   // busy wait; can be executed at IRQL <= DISPATCH_LEVEL
        }
        else {
            OS_SLEEP(10);   // sleep; can only be executed at IRQL < DISPATCH_LEVEL
        }
    }

    if (! OS_CANCEL_TIMER(&timer_object->os_timer)) {
        canceled = 0;
        printk("OS_CANCEL_TIMER failed!!\n");
    }

    timer_object->active_flag = 0;

    // release synchronization object
    (void) cmpxchg(&(timer_object->timer_lock), 1, 0); 

    return canceled;
}

u_int8_t ath_timer_is_active (struct ath_timer* timer_object)
{
    return (timer_object->active_flag);
}

u_int8_t ath_timer_is_initialized (struct ath_timer* timer_object)
{
    return (timer_object->signature == TIMER_SIGNATURE);
}
