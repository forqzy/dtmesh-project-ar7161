/*! \file
**  \brief 
**
** Copyright (c) 2001 Atsushi Onoe
** Copyright (c) 2002-2005 Sam Leffler, Errno Consulting
** Copyright (c) 2004-2007 Atheros Communications, Inc.
**
** All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef EXPORT_SYMTAB
#define	EXPORT_SYMTAB
#endif

__FBSDID("$FreeBSD: src/sys/net80211/ieee80211_node.c,v 1.31 2005/01/01 17:48:27 sam Exp $");

/*
 * IEEE 802.11 power save support.
 */
#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
#include <linux/config.h>
#endif
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>

#include "if_media.h"

#include <net80211/ieee80211_var.h>

static void ieee80211_set_tim(struct ieee80211_node *ni, int set);

void
ieee80211_power_attach(struct ieee80211com *ic)
{
}

void
ieee80211_power_detach(struct ieee80211com *ic)
{
}

void
ieee80211_power_vattach(struct ieee80211vap *vap)
{
    if (vap->iv_opmode == IEEE80211_M_HOSTAP ||
        vap->iv_opmode == IEEE80211_M_IBSS)
    {
        /* NB: driver should override */
        vap->iv_set_tim = ieee80211_set_tim;
    }

	vap->iv_pwrsave.ips_inactivitytime = (IEEE80211_PS_INACTIVITYTIME * HZ)/1000;
	vap->iv_pwrsave.ips_sta_psmode = IEEE80211_PWRSAVE_NONE;
	vap->iv_pwrsave.ips_state = IEEE80211_PWRSAVE_INIT;
	vap->iv_pwrsave.ips_connected = 0;
	vap->iv_pwrsave.ips_presleep = 0;
	vap->iv_pwrsave.ips_fullsleep_enable = 1;
	vap->iv_pwrsave.ips_fakesleep_pend = 0;

	vap->iv_pwrsave.ips_max_sleeptime = 1000;
	vap->iv_pwrsave.ips_normal_sleeptime = 500;
	vap->iv_pwrsave.ips_low_sleeptime = 100;
	vap->iv_pwrsave.ips_max_inactivitytime = (IEEE80211_PS_INACTIVITYTIME * HZ)/1000;
	vap->iv_pwrsave.ips_normal_inactivitytime = (IEEE80211_PS_INACTIVITYTIME * HZ)/1000;
	vap->iv_pwrsave.ips_low_inactivitytime = (IEEE80211_PS_INACTIVITYTIME * HZ)/1000;
    
	/*
	 * Initialize pwrsave timer 
	 */
	init_timer(&vap->iv_pwrsave.ips_timer);
	vap->iv_pwrsave.ips_timer.function = ieee80211_pwrsave_timer;
	vap->iv_pwrsave.ips_timer.data = (unsigned long) vap;

	IEEE80211_PWRSAVE_LOCK_INIT(vap);
}

void
ieee80211_power_latevattach(struct ieee80211vap *vap)
{
    /*
    * Allocate these only if needed.  Beware that we
    * know adhoc mode doesn't support ATIM yet...
    */
    if (vap->iv_opmode == IEEE80211_M_HOSTAP)
    {
        vap->iv_tim_len = howmany(vap->iv_max_aid,8) * sizeof(u_int8_t);
        MALLOC(vap->iv_tim_bitmap, u_int8_t *, vap->iv_tim_len,
            M_DEVBUF, M_NOWAIT | M_ZERO);
        if (vap->iv_tim_bitmap == NULL)
        {
            printf("%s: no memory for TIM bitmap!\n", __func__);
            /* XXX good enough to keep from crashing? */
            vap->iv_tim_len = 0;
        }
    }
}

void
ieee80211_power_vdetach(struct ieee80211vap *vap)
{
    if (vap->iv_tim_bitmap != NULL)
    {
        FREE(vap->iv_tim_bitmap, M_DEVBUF);
        vap->iv_tim_bitmap = NULL;
    }

	IEEE80211_PWRSAVE_LOCK(vap);
	vap->iv_pwrsave.ips_connected = 0;
	IEEE80211_PWRSAVE_UNLOCK(vap);

	del_timer(&vap->iv_pwrsave.ips_timer);

	IEEE80211_NODE_LOCK_DESTROY(&vap->iv_pwrsave);
}

/*
 * Clear any frames queued on a node's power save queue.
 * The number of frames that were present is returned.
 */
int
ieee80211_node_saveq_drain(struct ieee80211_node *ni)
{
    struct sk_buff *skb;
    int qlen;

    IEEE80211_NODE_SAVEQ_LOCK(ni);
    qlen = skb_queue_len(&ni->ni_savedq);
    while ((skb = __skb_dequeue(&ni->ni_savedq)) != NULL)
        dev_kfree_skb(skb);
    IEEE80211_NODE_SAVEQ_UNLOCK(ni);

    return qlen;
}

/*
 * Age frames on the power save queue. The aging interval is
 * 4 times the listen interval specified by the station.  This
 * number is factored into the age calculations when the frame
 * is placed on the queue.  We store ages as time differences
 * so we can check and/or adjust only the head of the list.
 * If a frame's age exceeds the threshold then discard it.
 * The number of frames discarded is returned so the caller
 * can check if it needs to adjust the tim.
 */
int
ieee80211_node_saveq_age(struct ieee80211_node *ni,
                         struct sk_buff_head *skb_freeqp)
{
    int discard = 0;

    /* XXX racey but good 'nuf? */
    if (IEEE80211_NODE_SAVEQ_QLEN(ni) != 0)
    {
#ifdef IEEE80211_DEBUG
        struct ieee80211vap *vap = ni->ni_vap;
#endif
        struct sk_buff *skb, *tail;

        IEEE80211_NODE_SAVEQ_LOCK(ni);
        while ((skb = skb_peek(&ni->ni_savedq)) != NULL &&
               M_AGE_GET(skb) < IEEE80211_INACT_WAIT)
        {
            IEEE80211_NOTE(vap, IEEE80211_MSG_POWER, ni,
                "discard frame, age %u", M_AGE_GET(skb));

            skb = __skb_dequeue(&ni->ni_savedq);
            tail = skb_peek_tail(skb_freeqp);
            if (tail != NULL)
            {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,25)
                __skb_queue_after(skb_freeqp, tail, skb);
#elif defined(__LINUX_MIPS32_ARCH__) || (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,14))
                __skb_append(tail, skb, skb_freeqp);
#else
                __skb_append(tail, skb);
#endif
            } else {
                __skb_queue_head(skb_freeqp, skb);
            }
            discard++;
        }
        if (skb != NULL)
            M_AGE_SUB(skb, IEEE80211_INACT_WAIT);
        IEEE80211_NODE_SAVEQ_UNLOCK(ni);

        IEEE80211_NOTE(vap, IEEE80211_MSG_POWER, ni,
            "discard %u frames for age", discard);
        IEEE80211_NODE_STAT_ADD(ni, ps_discard, discard);
    }
    return discard;
}

/*
 * Indicate whether there are frames queued for a station in power-save mode.
 */
static void
ieee80211_set_tim(struct ieee80211_node *ni, int set)
{
    struct ieee80211vap *vap = ni->ni_vap;
    u_int16_t aid;

    KASSERT(vap->iv_opmode == IEEE80211_M_HOSTAP ||
        vap->iv_opmode == IEEE80211_M_IBSS,
        ("operating mode %u", vap->iv_opmode));

    aid = IEEE80211_AID(ni->ni_associd);
    KASSERT(aid < vap->iv_max_aid,
        ("bogus aid %u, max %u", aid, vap->iv_max_aid));

    IEEE80211_BEACON_LOCK(ni->ni_ic);
    if (set != (isset(vap->iv_tim_bitmap, aid) != 0))
    {
        if (set)
        {
            setbit(vap->iv_tim_bitmap, aid);
            vap->iv_ps_pending++;
        }
        else
        {
            clrbit(vap->iv_tim_bitmap, aid);
            vap->iv_ps_pending--;
        }
        vap->iv_flags |= IEEE80211_F_TIMUPDATE;
    }
    IEEE80211_BEACON_UNLOCK(ni->ni_ic);
}

/*
 * Save an outbound packet for a node in power-save sleep state.
 * The new packet is placed on the node's saved queue, and the TIM
 * is changed, if necessary.
 */
void
ieee80211_pwrsave(struct ieee80211_node *ni, struct sk_buff *skb)
{
    struct ieee80211vap *vap = ni->ni_vap;
    struct ieee80211com *ic = ni->ni_ic;
    unsigned long flags;
    struct sk_buff *tail;
    int qlen, age;

    spin_lock_irqsave(&ni->ni_savedq.lock, flags);
    if (skb_queue_len(&ni->ni_savedq) >= IEEE80211_PS_MAX_QUEUE)
    {
        IEEE80211_NODE_STAT(ni,psq_drops);
        spin_unlock_irqrestore(&ni->ni_savedq.lock, flags);
		IEEE80211_NOTE(vap, IEEE80211_MSG_ANY, ni,
            "pwr save q overflow, drops %d (size %d)",
            ni->ni_stats.ns_psq_drops, IEEE80211_PS_MAX_QUEUE);
#ifdef IEEE80211_DEBUG
        if (ieee80211_msg_dumppkts(vap))
            ieee80211_dump_pkt(ni->ni_ic, skb->data, skb->len, -1, -1);
#endif
        dev_kfree_skb(skb);
        return;
    }
    /*
     * Tag the frame with it's expiry time and insert
     * it in the queue.  The aging interval is 4 times
     * the listen interval specified by the station.
     * Frames that sit around too long are reclaimed
     * using this information.
     */
    /* XXX handle overflow? */
    age = ((ni->ni_intval * ic->ic_lintval) << 2) / 1024; /* TU -> secs */
    tail = skb_peek_tail(&ni->ni_savedq);
    if (tail != NULL)
    {
        age -= M_AGE_GET(tail);
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,25)
	__skb_queue_after(&ni->ni_savedq, tail, skb);
#elif defined(__LINUX_MIPS32_ARCH__) || (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,14))
        __skb_append(tail, skb, &ni->ni_savedq);
#else
        __skb_append(tail, skb);
#endif
    }
    else
    {
        __skb_queue_head(&ni->ni_savedq, skb);
    }
    M_AGE_SET(skb, age);
    qlen = skb_queue_len(&ni->ni_savedq);
    spin_unlock_irqrestore(&ni->ni_savedq.lock, flags);

    IEEE80211_NOTE(vap, IEEE80211_MSG_POWER, ni,
        "save frame, %u now queued", qlen);

    if (qlen == 1 && vap->iv_set_tim != NULL)
        vap->iv_set_tim(ni, 1);
}

/*
 * Handle power-save state change in ap/ibss mode.
 */
void
ieee80211_node_pwrsave(struct ieee80211_node *ni, int enable)
{
    struct ieee80211vap *vap = ni->ni_vap;
#ifdef ATH_PSUPDATE
    struct ieee80211com *ic = vap->iv_ic;
#endif

    KASSERT(vap->iv_opmode == IEEE80211_M_HOSTAP ||
        vap->iv_opmode == IEEE80211_M_IBSS,
        ("unexpected operating mode %u", vap->iv_opmode));

    if (enable)
    {
        if ((ni->ni_flags & IEEE80211_NODE_PWR_MGT) == 0) {
            vap->iv_ps_sta++;
#ifdef ATH_PSUPDATE
            ic->ic_node_psupdate(ni, 1);
#endif
        }
        ni->ni_flags |= IEEE80211_NODE_PWR_MGT;

        IEEE80211_NOTE(vap, IEEE80211_MSG_POWER, ni,
            "power save mode on, %u sta's in ps mode", vap->iv_ps_sta);
        return;
    }

    if ((ni->ni_flags & IEEE80211_NODE_PWR_MGT)) {
#ifdef ATH_PSUPDATE
        ic->ic_node_psupdate(ni, 0);
#endif
        vap->iv_ps_sta--;
    }
    ni->ni_flags &= ~IEEE80211_NODE_PWR_MGT;

    IEEE80211_NOTE(vap, IEEE80211_MSG_POWER, ni,
        "power save mode off, %u sta's in ps mode", vap->iv_ps_sta);
    /* XXX if no stations in ps mode, flush mc frames */

    /*
    * Flush queued unicast frames.
    */
    if (IEEE80211_NODE_SAVEQ_QLEN(ni) == 0)
    {
        if (vap->iv_set_tim != NULL)
            vap->iv_set_tim(ni, 0);		/* just in case */
        return;
    }
    IEEE80211_NOTE(vap, IEEE80211_MSG_POWER, ni,
        "flush ps queue, %u packets queued", IEEE80211_NODE_SAVEQ_QLEN(ni));
    for (;;)
    {
        struct sk_buff *skb;
        int qlen;

        IEEE80211_NODE_SAVEQ_DEQUEUE(ni, skb, qlen);
        if (skb == NULL)
            break;
        /* 
         * If this is the last packet, turn off the TIM bit.
         *
         * Set the M_PWR_SAV bit on skb to allow encap to test for
         * adding MORE_DATA bit to wh.
         */
        M_PWR_SAV_SET(skb);

#ifdef ATH_SUPERG_XR
        /*
        * if it is a XR vap, send the data to associated normal net
        * device. XR vap has a net device which is not registered with
        * OS. 
        */
        if (vap->iv_flags & IEEE80211_F_XR)
        {
            skb->dev = vap->iv_xrvap->iv_dev;
        }
        else
        {
            skb->dev = vap->iv_dev;		/* XXX? unnecessary */
        }
#else
        skb->dev = vap->iv_dev;		/* XXX? unnecessary */
#endif
        dev_queue_xmit(skb);
    }
    vap->iv_set_tim(ni, 0);
}

/*
 * Handle power-save state change in station mode.
 */
void
ieee80211_sta_pwrsave(struct ieee80211vap *vap, int enable)
{
    struct ieee80211_node *ni = vap->iv_bss;
    int qlen;

    if (!((enable != 0) ^ ((ni->ni_flags & IEEE80211_NODE_PWR_MGT) != 0)))
        return;

    IEEE80211_NOTE(vap, IEEE80211_MSG_POWER, ni,
        "sta power save mode %s", enable ? "on" : "off");
    if (!enable)
    {
        ni->ni_flags &= ~IEEE80211_NODE_PWR_MGT;
#ifdef NODE_FREE_DEBUG
        ieee80211_send_nulldata(ieee80211_ref_node(ni, __func__));
#else
        ieee80211_send_nulldata(ieee80211_ref_node(ni));
#endif
        /*
         * Flush any queued frames; we can do this immediately
         * because we know they'll be queued behind the null
         * data frame we send the ap.
         * XXX can we use a data frame to take us out of ps?
         */
        qlen = IEEE80211_NODE_SAVEQ_QLEN(ni);
        if (qlen != 0)
        {
            IEEE80211_NOTE(vap, IEEE80211_MSG_POWER, ni,
                "flush ps queue, %u packets queued", qlen);
            for (;;)
            {
                struct sk_buff *skb;

                IEEE80211_NODE_SAVEQ_LOCK(ni);
                skb = __skb_dequeue(&ni->ni_savedq);
                IEEE80211_NODE_SAVEQ_UNLOCK(ni);
                if (skb == NULL)
                    break;
                dev_queue_xmit(skb);
            }
        }
    }
    else
    {
        ni->ni_flags |= IEEE80211_NODE_PWR_MGT;
#ifdef NODE_FREE_DEBUG
        ieee80211_send_nulldata(ieee80211_ref_node(ni, __func__));
#else
        ieee80211_send_nulldata(ieee80211_ref_node(ni));
#endif
    }
}

static int
ieee80211_pwrsave_set_state(struct ieee80211vap *vap, IEEE80211_PWRSAVE_STATE newstate)
{
	int status = IEEE80211_PWRSAVE_SUCCESS;
	struct ieee80211com *ic = vap->iv_ic;

	if(vap->iv_pwrsave.ips_state == newstate) 
		return status;

	switch (vap->iv_pwrsave.ips_state) {

	case IEEE80211_PWRSAVE_INIT:
		switch (newstate) {
		case IEEE80211_PWRSAVE_AWAKE:
			ic->ic_pwrsave_set_state(ic,IEEE80211_PWRSAVE_AWAKE);
			break;
		case IEEE80211_PWRSAVE_FULL_SLEEP:
			if (vap->iv_pwrsave.ips_fullsleep_enable) {
				ic->ic_pwrsave_set_state(ic, IEEE80211_PWRSAVE_FULL_SLEEP);
			}
			break;
		default:
			printk("%s: wrong state transition from %d to %d\n", 
			       __func__,vap->iv_pwrsave.ips_state,newstate);
			status = IEEE80211_PWRSAVE_FAIL;
			break;
		}
		break;

	case IEEE80211_PWRSAVE_NETWORK_SLEEP:
		switch (newstate) {
		case IEEE80211_PWRSAVE_AWAKE:
			ic->ic_pwrsave_set_state(ic,IEEE80211_PWRSAVE_AWAKE);
			break;
		case IEEE80211_PWRSAVE_FULL_SLEEP:
			if (vap->iv_pwrsave.ips_fullsleep_enable) {
				ic->ic_pwrsave_set_state(ic, IEEE80211_PWRSAVE_FULL_SLEEP);
			}
			break;
		case IEEE80211_PWRSAVE_FAKE_SLEEP:
			ic->ic_pwrsave_set_state(ic,IEEE80211_PWRSAVE_AWAKE);
			break;

		default:
			printk("%s: wrong state transition from %d to %d\n", 
			       __func__,vap->iv_pwrsave.ips_state,newstate);
			status = IEEE80211_PWRSAVE_FAIL;
			break;
		}
		break;

	case IEEE80211_PWRSAVE_AWAKE:
		switch (newstate) {
		case IEEE80211_PWRSAVE_NETWORK_SLEEP:
			ic->ic_pwrsave_set_state(ic, IEEE80211_PWRSAVE_NETWORK_SLEEP);
			break;
		case IEEE80211_PWRSAVE_FULL_SLEEP:
			if (!vap->iv_pwrsave.ips_connected &&
			    vap->iv_pwrsave.ips_fullsleep_enable) {
				ic->ic_pwrsave_set_state(ic, IEEE80211_PWRSAVE_FULL_SLEEP);
			}
			break;
		case IEEE80211_PWRSAVE_FAKE_SLEEP:
			break;

		default:
			printk("%s: wrong state transition from %d to %d\n", 
			       __func__,vap->iv_pwrsave.ips_state,newstate);
			status = IEEE80211_PWRSAVE_FAIL;
			break;
		}
		break;

	case IEEE80211_PWRSAVE_FULL_SLEEP:
		switch (newstate) {
		case IEEE80211_PWRSAVE_AWAKE:
			ic->ic_pwrsave_set_state(ic,IEEE80211_PWRSAVE_AWAKE);
			break;
		default:
			printk("%s: wrong state transition from %d to %d\n", 
			       __func__,vap->iv_pwrsave.ips_state,newstate);
			status = IEEE80211_PWRSAVE_FAIL;
			break;
		}
		break;

	case IEEE80211_PWRSAVE_FAKE_SLEEP:
		switch (newstate) {
		case IEEE80211_PWRSAVE_AWAKE:
			ic->ic_pwrsave_set_state(ic,IEEE80211_PWRSAVE_AWAKE);		
			break;
		case IEEE80211_PWRSAVE_FULL_SLEEP:
			if (vap->iv_pwrsave.ips_fullsleep_enable) {
				ic->ic_pwrsave_set_state(ic, IEEE80211_PWRSAVE_FULL_SLEEP);
			}
			break;
		default:
			printk("%s: wrong state transition from %d to %d\n", 
			       __func__,vap->iv_pwrsave.ips_state,newstate);
			status = IEEE80211_PWRSAVE_FAIL;
			break;
		}
		break;

	default:
		printk("%s: wrong state transition from %d to %d\n", 
                   __func__,vap->iv_pwrsave.ips_state,newstate);
		status = IEEE80211_PWRSAVE_FAIL;
		break;

	}
	if (status == IEEE80211_PWRSAVE_SUCCESS)
		vap->iv_pwrsave.ips_state = newstate;

	return status;
}

IEEE80211_PWRSAVE_STATE
ieee80211_pwrsave_get_state(struct ieee80211vap *vap)
{
	return vap->iv_pwrsave.ips_state;
}

void ieee80211_pwrsave_init(struct ieee80211vap *vap)
{
    
	IEEE80211_PWRSAVE_LOCK(vap);
	//ieee80211_pwrsave_set_state(vap, IEEE80211_PWRSAVE_FULL_SLEEP);
	ieee80211_pwrsave_set_state(vap, IEEE80211_PWRSAVE_AWAKE);
	IEEE80211_PWRSAVE_UNLOCK(vap);
}
/*
 * Check if there is a quiet period in network activaty, if yes start the sleep
 * process.
 */
static void
ieee80211_pwrsave_presleep(struct ieee80211vap *vap)
{
	struct ieee80211com *ic = vap->iv_ic;
	int send = 0;

	IEEE80211_PWRSAVE_LOCK(vap);

	if (!vap->iv_pwrsave.ips_connected ||
	    vap->iv_state != IEEE80211_S_RUN || 
	    vap->iv_opmode != IEEE80211_M_STA ||
	    vap->iv_pwrsave.ips_sta_psmode == IEEE80211_PWRSAVE_NONE) {
		IEEE80211_PWRSAVE_UNLOCK(vap);
		return;
	}
//	if (time_after(jiffies, vap->iv_lastdata + vap->iv_pwrsave.ips_inactivitytime))
	if ( jiffies > (vap->iv_lastdata + vap->iv_pwrsave.ips_inactivitytime))
    {
        if (vap->iv_pwrsave.ips_state == IEEE80211_PWRSAVE_AWAKE) {

		    vap->iv_pwrsave.ips_presleep = 1;
		    if (ic->ic_txq_depth(ic) == 0) {
			    IEEE80211_DPRINTF(vap, IEEE80211_MSG_POWER, "%s: send nulldata\n", __func__);
			    send = 1;
            }
		}
	}
	IEEE80211_PWRSAVE_UNLOCK(vap);
	if (send) {
#ifdef NODE_FREE_DEBUG
		if (ieee80211_send_nulldata_pwrsave(ieee80211_ref_node(vap->iv_bss, __func__), 1))
#else
		if (ieee80211_send_nulldata_pwrsave(ieee80211_ref_node(vap->iv_bss), 1))
#endif
                {
			/* send fail */
			IEEE80211_PWRSAVE_LOCK(vap);
			vap->iv_pwrsave.ips_presleep = 0;
			IEEE80211_PWRSAVE_UNLOCK(vap);
		}
	}

	mod_timer(&vap->iv_pwrsave.ips_timer, jiffies + ((IEEE80211_PWRSAVE_TIMER_INTERVAL * HZ)/1000));
}

void
ieee80211_pwrsave_timer(unsigned long arg)
{
	struct ieee80211vap *vap = (struct ieee80211vap *) arg;
    
	ieee80211_pwrsave_presleep(vap);

}

void ieee80211_pwrsave_connect(struct ieee80211vap *vap)
{

	IEEE80211_DPRINTF(vap, IEEE80211_MSG_POWER, "%s:\n", __func__);

	IEEE80211_PWRSAVE_LOCK(vap);

	vap->iv_pwrsave.ips_connected = 1;
	ieee80211_pwrsave_set_state(vap, IEEE80211_PWRSAVE_AWAKE);

	if (vap->iv_opmode == IEEE80211_M_STA) {
		vap->iv_pwrsave.ips_presleep = 0;
		vap->iv_pwrsave.ips_fakesleep_pend = 0;
		if (vap->iv_pwrsave.ips_sta_psmode != IEEE80211_PWRSAVE_NONE) {
			mod_timer(&vap->iv_pwrsave.ips_timer, jiffies);
		}
	}
	IEEE80211_PWRSAVE_UNLOCK(vap);


}

void ieee80211_pwrsave_disconnect(struct ieee80211vap *vap)
{

	IEEE80211_DPRINTF(vap, IEEE80211_MSG_POWER, "%s:\n", __func__);

	IEEE80211_PWRSAVE_LOCK(vap);
	if (vap->iv_pwrsave.ips_connected) {
		if (vap->iv_opmode == IEEE80211_M_STA) {           
			del_timer(&vap->iv_pwrsave.ips_timer);
		}
		vap->iv_pwrsave.ips_connected = 0;
	}

	/* keep awake for re-connect */
	//ieee80211_pwrsave_set_state(vap, IEEE80211_PWRSAVE_FULL_SLEEP);
	ieee80211_pwrsave_set_state(vap, IEEE80211_PWRSAVE_AWAKE);
	IEEE80211_PWRSAVE_UNLOCK(vap);

}

int ieee80211_pwrsave_cannot_send(struct ieee80211vap *vap)
{
	if (vap->iv_pwrsave.ips_connected && 
	    (vap->iv_pwrsave.ips_presleep ||
	     vap->iv_pwrsave.ips_state == IEEE80211_PWRSAVE_FAKE_SLEEP)) {
		return 1;
	} else {
		return 0;
	}
}

/*
 * Called by scan code to do the fake sleep.
 */
int
ieee80211_pwrsave_fakesleep(struct ieee80211vap *vap)
{
	struct ieee80211_node *ni = vap->iv_bss;
	struct ieee80211com *ic = vap->iv_ic;

	if (vap->iv_opmode == IEEE80211_M_IBSS) {
		IEEE80211_PWRSAVE_LOCK(vap);
		if (vap->iv_pwrsave.ips_connected) {
			ieee80211_pwrsave_set_state(vap, IEEE80211_PWRSAVE_FAKE_SLEEP);
		} else {
			IEEE80211_PWRSAVE_UNLOCK(vap);
			return IEEE80211_PWRSAVE_FAIL;
		}
		IEEE80211_PWRSAVE_UNLOCK(vap);
		/* TODO: Implement fake sleep for IBSS mode */
		//ieee80211_scan_fakesleep_complete(vap, 0x0);
		return IEEE80211_PWRSAVE_SUCCESS;
	}

	if (vap->iv_opmode != IEEE80211_M_STA)
		return IEEE80211_PWRSAVE_FAIL;

	IEEE80211_PWRSAVE_LOCK(vap);
	if (vap->iv_pwrsave.ips_connected) {
		ieee80211_pwrsave_set_state(vap, IEEE80211_PWRSAVE_FAKE_SLEEP);
		vap->iv_pwrsave.ips_fakesleep_pend = 1;
		vap->iv_pwrsave.ips_presleep = 0;
	} else {
		IEEE80211_PWRSAVE_UNLOCK(vap);
		return IEEE80211_PWRSAVE_FAIL;
	}

	IEEE80211_PWRSAVE_UNLOCK(vap);
	if (ic->ic_txq_depth(ic) == 0) {
		IEEE80211_DPRINTF(vap, IEEE80211_MSG_POWER, "%s: send nulldata\n", __func__);
#ifdef NODE_FREE_DEBUG
		if (ieee80211_send_nulldata_pwrsave(ieee80211_ref_node(ni, __func__), 1))
#else
		if (ieee80211_send_nulldata_pwrsave(ieee80211_ref_node(ni), 1))
#endif
                {
			/* send fail */
			IEEE80211_PWRSAVE_LOCK(vap);
			ieee80211_pwrsave_set_state(vap, IEEE80211_PWRSAVE_AWAKE);
			vap->iv_pwrsave.ips_fakesleep_pend = 0;
			IEEE80211_PWRSAVE_UNLOCK(vap);
			return IEEE80211_PWRSAVE_FAIL;
		}
	}
	return IEEE80211_PWRSAVE_SUCCESS;
}

/*
 * Send a null data to AP with the power mgmt bit set.
 */
void
ieee80211_pwrsave_txq_empty(struct ieee80211com *ic)
{
	struct ieee80211vap *vap = NULL;
	struct ieee80211_node *ni = NULL;
	u_int32_t fakesleep_complete = 0, send = 0;
	struct ieee80211vap *tmpvap;

	/* Only for single STA mode */
	TAILQ_FOREACH(tmpvap, &ic->ic_vaps, iv_next) {
		if (tmpvap->iv_opmode == IEEE80211_M_STA) {
			vap = tmpvap;
			break;
		}
	}
	if (vap != NULL) {
		ni = vap->iv_bss;
	}
	else {
		return;
	}

	IEEE80211_PWRSAVE_LOCK(vap);
	if (vap->iv_pwrsave.ips_connected) {
		if (vap->iv_pwrsave.ips_presleep ||
		    vap->iv_pwrsave.ips_fakesleep_pend) {
			IEEE80211_DPRINTF(vap, IEEE80211_MSG_POWER, "%s\n", __func__);
			send = 1;
		}
	} else { /* disconnect */
		if (vap->iv_pwrsave.ips_fakesleep_pend) {
			vap->iv_pwrsave.ips_fakesleep_pend =  0;
			fakesleep_complete = 1;
		}
	}
	IEEE80211_PWRSAVE_UNLOCK(vap);

	if (send) {
#ifdef NODE_FREE_DEBUG
		if (ieee80211_send_nulldata_pwrsave(ieee80211_ref_node(ni, __func__), 1))
#else
		if (ieee80211_send_nulldata_pwrsave(ieee80211_ref_node(ni), 1))
#endif
                {
			/* send fail */
			IEEE80211_PWRSAVE_LOCK(vap);
			if (vap->iv_pwrsave.ips_presleep) {
				vap->iv_pwrsave.ips_presleep = 0;
			} else {
				ieee80211_pwrsave_set_state(vap, IEEE80211_PWRSAVE_AWAKE);
				vap->iv_pwrsave.ips_fakesleep_pend =  0;
				fakesleep_complete = 1;
			}
			IEEE80211_PWRSAVE_UNLOCK(vap);

		}
	}

	/* TODO: Implement fake sleep for IBSS mode */
	//if (fakesleep_complete)
	//    ieee80211_scan_fakesleep_complete(vap, 0x1);
}

/*
 * Wake up and/or send a null data to AP without the power mgmt bit set.
 */
void
ieee80211_pwrsave_wakeup(struct ieee80211vap *vap, int reason)
{
	struct ieee80211_node *ni = vap->iv_bss;
	int send = 0;

	IEEE80211_PWRSAVE_LOCK(vap);
	vap->iv_pwrsave.ips_presleep = 0;

	/*
	 * Don't let other reasons except FAKE_WAKEUP change state from IEEE80211_PWRSAVE_FAKE_SLEEP
	 * to IEEE80211_PWRSAVE_AWAKE. Otherwise, when probe requests are sent or beacons are received,
	 * ieee80211_pwrsave_cannot_send would not be able to hold TX traffic and cause data loss.
	 */
	if (ieee80211_pwrsave_get_state(vap) == IEEE80211_PWRSAVE_FAKE_SLEEP && reason != FAKE_WAKEUP) {
		IEEE80211_PWRSAVE_UNLOCK(vap);
		return;
	}

	ieee80211_pwrsave_set_state(vap, IEEE80211_PWRSAVE_AWAKE);

	switch (reason) {
	case RECEIVE:
	case FAKE_WAKEUP:
		if (vap->iv_pwrsave.ips_connected && vap->iv_opmode == IEEE80211_M_STA) {
			send = 1;
		}
		break;
	case START_SCAN:
	case START_CONNECT:
	case TRANSMIT:
	default:
		break;

	}
	IEEE80211_PWRSAVE_UNLOCK(vap);
    
	if (send) {
#ifdef NODE_FREE_DEBUG
		ieee80211_send_nulldata_pwrsave(ieee80211_ref_node(ni, __func__), 0);
#else
		ieee80211_send_nulldata_pwrsave(ieee80211_ref_node(ni), 0);
#endif
	}

}

/*
 * Go to sleep or send a null data to AP with the power mgmt bit set.
 */
int
ieee80211_pwrsave_sleep(struct ieee80211vap *vap, int reason)
{
	int status = IEEE80211_PWRSAVE_SUCCESS;

	switch (reason) {
	case SCAN_COMPLETE:
		IEEE80211_PWRSAVE_LOCK(vap);
		if (!vap->iv_pwrsave.ips_connected) {
			ieee80211_pwrsave_set_state(vap, IEEE80211_PWRSAVE_FULL_SLEEP);
		} else {
			if (vap->iv_opmode != IEEE80211_M_STA) {
				if (vap->iv_pwrsave.ips_state == IEEE80211_PWRSAVE_FAKE_SLEEP) {
					/* error: the scan is completed, it shouldn't stay in fake sleep state.
					 * correct it.
					 */
					ieee80211_pwrsave_set_state(vap, IEEE80211_PWRSAVE_AWAKE);
				}
			}
		}
		IEEE80211_PWRSAVE_UNLOCK(vap);
		break;
	case FAKE_SLEEP:
		status = ieee80211_pwrsave_fakesleep(vap);
		break;
	default:
		break;

	}
	return status;
}

/*
 * Call back from ath_tx_processq. If we'v recived the ACK for the pwrsave frame
 * put the chip to sleep.
 */
void
ieee80211_pwrsave_complete_wbuf(struct ieee80211vap *vap, u_int32_t status)
{
	u_int32_t fakesleep_complete = 0;

	IEEE80211_PWRSAVE_LOCK(vap);
	if (vap->iv_pwrsave.ips_presleep) {
		vap->iv_pwrsave.ips_presleep = 0;

		if (status == 0x0) {
			IEEE80211_DPRINTF(vap, IEEE80211_MSG_POWER, "%s: sleep \n", __func__);
			ieee80211_pwrsave_set_state(vap, IEEE80211_PWRSAVE_NETWORK_SLEEP);
		} else {
			ieee80211_pwrsave_set_state(vap, IEEE80211_PWRSAVE_AWAKE);
			IEEE80211_DPRINTF(vap, IEEE80211_MSG_POWER, "%s: awake \n", __func__);
		}
	}
    
	if (vap->iv_pwrsave.ips_fakesleep_pend) {
		vap->iv_pwrsave.ips_fakesleep_pend = 0;
		if (status != 0x0) {
			ieee80211_pwrsave_set_state(vap, IEEE80211_PWRSAVE_AWAKE);
		}
		fakesleep_complete = 1;
	}
	IEEE80211_PWRSAVE_UNLOCK(vap);

	/* TODO: Implement fake sleep for IBSS mode */
	//if (fakesleep_complete)
	//    ieee80211_scan_fakesleep_complete(vap, status);

}
EXPORT_SYMBOL_C(ieee80211_pwrsave_complete_wbuf);

void ieee80211_pwrsave_set_mode(struct ieee80211vap *vap, u_int32_t mode)
{

	IEEE80211_DPRINTF(vap, IEEE80211_MSG_POWER, "%s:\n", __func__);

	if (vap->iv_opmode != IEEE80211_M_STA ||
	    mode > IEEE80211_PWRSAVE_MAXIMUM ) 
		return;

	IEEE80211_PWRSAVE_LOCK(vap);
	if (mode != vap->iv_pwrsave.ips_sta_psmode) {
        
		if (mode == IEEE80211_PWRSAVE_NONE) {
			IEEE80211_DPRINTF(vap, IEEE80211_MSG_POWER, "%s: power save off \n", __func__);
		} else if (mode == IEEE80211_PWRSAVE_LOW) {
			IEEE80211_DPRINTF(vap, IEEE80211_MSG_POWER, "%s: low power save on\n", __func__);
			vap->iv_pwrsave.ips_inactivitytime = vap->iv_pwrsave.ips_low_inactivitytime;
			vap->iv_update_ps_mode(vap);
			if (vap->iv_pwrsave.ips_connected && (vap->iv_pwrsave.ips_sta_psmode == IEEE80211_PWRSAVE_NONE)) {
				mod_timer(&vap->iv_pwrsave.ips_timer, jiffies);
			}
		} else if (mode == IEEE80211_PWRSAVE_NORMAL) {
			IEEE80211_DPRINTF(vap, IEEE80211_MSG_POWER, "%s: normal power save on\n", __func__);
			vap->iv_pwrsave.ips_inactivitytime = vap->iv_pwrsave.ips_normal_inactivitytime;
			vap->iv_update_ps_mode(vap);
			if (vap->iv_pwrsave.ips_connected && (vap->iv_pwrsave.ips_sta_psmode == IEEE80211_PWRSAVE_NONE)) {
				mod_timer(&vap->iv_pwrsave.ips_timer, jiffies);
			}
		} else if (mode == IEEE80211_PWRSAVE_MAXIMUM) {
			IEEE80211_DPRINTF(vap, IEEE80211_MSG_POWER, "%s: max power save on\n", __func__);
			vap->iv_pwrsave.ips_inactivitytime = vap->iv_pwrsave.ips_max_inactivitytime;
			vap->iv_update_ps_mode(vap);
			if (vap->iv_pwrsave.ips_connected && (vap->iv_pwrsave.ips_sta_psmode == IEEE80211_PWRSAVE_NONE)) {
				mod_timer(&vap->iv_pwrsave.ips_timer, jiffies);
			}
		}
		vap->iv_pwrsave.ips_sta_psmode = mode;
	}
	IEEE80211_PWRSAVE_UNLOCK(vap);
    
}
EXPORT_SYMBOL_C(ieee80211_pwrsave_set_mode);

/*
 * When we get TIM (from int or check beacon), wake up the chip, send a null data to AP to
 * tell it the sta is wakeup.
 */
void
ieee80211_pwrsave_proc_tim(struct ieee80211com *ic)
{
	struct ieee80211vap *vap = NULL;
	struct ieee80211vap *tmpvap;

	/* Only for single STA mode */
	TAILQ_FOREACH(tmpvap, &ic->ic_vaps, iv_next) {
		if (tmpvap->iv_opmode == IEEE80211_M_STA) {
			vap = tmpvap;
			break;
		}
	}
	if (vap == NULL) {
		return;
	}

	IEEE80211_DPRINTF(vap, IEEE80211_MSG_POWER, "%s:\n", __func__);
	vap->iv_lastdata = jiffies; /* msec */
	ieee80211_pwrsave_wakeup(vap, RECEIVE);
}
EXPORT_SYMBOL_C(ieee80211_pwrsave_proc_tim);

int ieee80211_pwrsave_get_mode(struct ieee80211vap *vap)
{
	return vap->iv_pwrsave.ips_sta_psmode;
}

void	
ieee80211_pwrsave_proc_dtim(struct ieee80211com *ic)
{
	ic->ic_pwrsave_set_state(ic,IEEE80211_PWRSAVE_AWAKE);
}

/* 
 * Received the last broadcast/multicast packet that the AP has sent within
 * the beacon period, return to sleep.
 */
void
ieee80211_pwrsave_rcv_lastbmcast(struct ieee80211vap *vap)
{
	struct ieee80211com *ic = vap->iv_ic;

	IEEE80211_PWRSAVE_LOCK(vap);
	if (vap->iv_pwrsave.ips_state == IEEE80211_PWRSAVE_NETWORK_SLEEP) {
		ic->ic_pwrsave_set_state(ic, IEEE80211_PWRSAVE_NETWORK_SLEEP);
	}
	IEEE80211_PWRSAVE_UNLOCK(vap);
}

void
ieee80211_pwrsave_disable_fullsleep(struct ieee80211vap *vap)
{
	IEEE80211_PWRSAVE_LOCK(vap);
	vap->iv_pwrsave.ips_fullsleep_enable = 0;
	IEEE80211_PWRSAVE_UNLOCK(vap);

}

u_int32_t
ieee80211_pwrsave_get_max_sleeptime(struct ieee80211vap *vap)
{
	return vap->iv_pwrsave.ips_max_sleeptime;
}

u_int32_t
ieee80211_pwrsave_get_normal_sleeptime(struct ieee80211vap *vap)
{
	return vap->iv_pwrsave.ips_normal_sleeptime;
}

u_int32_t
ieee80211_pwrsave_get_low_sleeptime(struct ieee80211vap *vap)
{
	return vap->iv_pwrsave.ips_low_sleeptime;
}

u_int32_t
ieee80211_pwrsave_get_max_inactivitytime(struct ieee80211vap *vap)
{
	return vap->iv_pwrsave.ips_max_inactivitytime;
}

u_int32_t
ieee80211_pwrsave_get_normal_inactivitytime(struct ieee80211vap *vap)
{
	return vap->iv_pwrsave.ips_normal_inactivitytime;
}

u_int32_t
ieee80211_pwrsave_get_low_inactivitytime(struct ieee80211vap *vap)
{
	return vap->iv_pwrsave.ips_low_inactivitytime;
}
