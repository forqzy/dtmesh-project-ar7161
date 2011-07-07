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
 * $Id: //depot/sw/releases/7.3_AP/wlan/linux/net80211/ieee80211_hbr.c#4 $
 */
/********************************************************************/
/* \file ieee80211_hbr.c
 * \brief Headline block removal algorithm, as part of 
 * Atheros iQUE features.
 *
 * This algorithm tries to block the outgoing UDP frames in the VI/VO queues 
 * if the connection of this node is unreliable, thus they will not block 
 * other VI/VO data frames to be transmitted to the nodes with good channel 
 * condition. While transmission being blocked, Qos Null framses will be 
 * sent out periodically as probing frames to check the channel condition, 
 * untill the channel condition is good again, and the normal transmissions
 * are restored thereafter. For more details, please refer to the design document.
 */

#ifdef ATH_SUPPORT_IQUE
#ifndef EXPORT_SYMTAB
#define	EXPORT_SYMTAB
#endif


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
#include <linux/config.h>
#endif
#include <linux/version.h>
#include <osdep.h>
#include <sys/queue.h>
#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_sm.h>
#include <net80211/ieee80211_hbr.h>
#include <net80211/ieee80211_node.h>

static void ieee80211_hbr_probe_allnodes(struct ieee80211vap *vap, u_int8_t *addr);
static void	ieee80211_hbr_setblock_allnodes(struct ieee80211vap *vap, u_int8_t *addr, u_int8_t block);
void ieee80211_hbr_probe(void *, struct ieee80211_node *);
void ieee80211_hbr_setblock(void *, struct ieee80211_node *);
int ieee80211_hbr_step(struct ieee80211_hbr_sm *sm, void *arg1, void *arg2);
int ieee80211_hbr_setstate(struct ieee80211_hbr_sm *sm, void *arg1, void *arg2);
int ieee80211_hbr_getstate(struct ieee80211_hbr_sm *sm, void *arg1, void *arg2);
static void ieee80211_hbr_nodestate(void *arg, struct ieee80211_node *ni);

/* Iterate the state machine list with running the function *f. If *f returns 1, break out of the loop;
 * If *f returns 0, continue to loop 
 */
void ieee80211_hbr_iterate(struct ieee80211vap *vap, ieee80211_hbr_iterate_func *f, void *arg1, void *arg2)
{
	struct ieee80211_hbr_sm *smt;
	struct ieee80211_hbr_list *ht;
	
	ht = &(vap->iv_hbr_list);
	IEEE80211_HBR_LOCK_BH(ht);
	TAILQ_FOREACH(smt, &ht->hbr_node, hbr_list) {
		if (smt) {	
			if ((*f)(smt, arg1, arg2))
				break;
			else
				continue;
		}	
	}
	IEEE80211_HBR_UNLOCK_BH(ht);		
}

/*Three states of the state machine: ACTIVE, BLOCKING, and PROBING*/
SM_STATE(HBR_SM, ACTIVE)
{	
	SM_ENTRY(HBR_SM, ACTIVE, hbr_sm);
	sm->hbr_block = 0;	/* Allow data frames to be xmitted */
	sm->hbr_trigger = HBR_TRIGGER_STALL;
	ieee80211_hbr_setblock_allnodes(sm->hbr_vap, sm->hbr_addr, 0);
}

SM_STATE(HBR_SM, BLOCKING)
{
	SM_ENTRY(HBR_SM, BLOCKING, hbr_sm);
	/* Block data frames to be xmitted. only send out QOS NULL frames for probing*/
	sm->hbr_block = 1;	
	sm->hbr_trigger = HBR_TRIGGER_STALL;
	ieee80211_hbr_setblock_allnodes(sm->hbr_vap, sm->hbr_addr, 1);
}

SM_STATE(HBR_SM, PROBING)
{
	SM_ENTRY(HBR_SM, PROBING, hbr_sm);
	ieee80211_hbr_probe_allnodes(sm->hbr_vap, sm->hbr_addr);
}

SM_STEP(SM_HBR)
{
	struct ieee80211vap *vap;
	vap = sm->hbr_vap;
	if(vap) {
		switch(sm->hbr_sm_state) {
			case HBR_SM_ACTIVE:
				if(sm->hbr_trigger == HBR_TRIGGER_BACK) {
					SM_ENTER(HBR_SM, BLOCKING);
				}
				break;
			case HBR_SM_BLOCKING:
				SM_ENTER(HBR_SM, PROBING);
				break;
			case HBR_SM_PROBING:
				if (sm->hbr_trigger == HBR_TRIGGER_FORWARD) {
					SM_ENTER(HBR_SM, ACTIVE);
				} else {
					SM_ENTER(HBR_SM, PROBING);
				}			
				break;
		}
	}
}
/* Timer function to run the state machines */
OS_TIMER_FUNC(ieee80211_hbr_timer)
{
	struct ieee80211vap *vap;

	OS_GET_TIMER_ARG(vap, struct ieee80211vap *);
	if (vap) {
		ieee80211_hbr_iterate(vap, ieee80211_hbr_step, NULL, NULL);	
	}
	OS_SET_TIMER(&vap->iv_hbr_sm_timer, vap->iv_hbr_timeout);
	return;
}
EXPORT_SYMBOL_C(ieee80211_hbr_timer);

/* Run each state machine */
int ieee80211_hbr_step(struct ieee80211_hbr_sm *sm, void *arg1, void *arg2)
{
	SM_STEP_RUN(SM_HBR);
	return 0;
}

/* Iterate through the list to find the state machine by the node's address */
struct ieee80211_hbr_sm * ieee80211_hbr_find_byaddr(struct ieee80211vap *vap, u_int8_t *addr)
{
	struct ieee80211_hbr_sm *sm, *smt;
	struct ieee80211_hbr_list *ht;
	
	if (!addr)
		return NULL;	
	sm = NULL;
	ht = &(vap->iv_hbr_list);
	IEEE80211_HBR_LOCK_BH(ht);
	TAILQ_FOREACH(smt, &ht->hbr_node, hbr_list) {
		if (smt) {
			if(IEEE80211_ADDR_EQ(addr, smt->hbr_addr)) {	
				sm = smt;
				break;
			}
		}	
	}
	IEEE80211_HBR_UNLOCK_BH(ht);		
	return sm;
}

/* Add one entry to the state machine list if no one state machine with 
 * this address is found in the list 
 */
void ieee80211_hbr_addentry(struct ieee80211vap *vap, u_int8_t *addr)
{
	struct ieee80211_hbr_sm *sm;
	struct ieee80211_hbr_list *ht;
	ht = &(vap->iv_hbr_list);
	sm = ieee80211_hbr_find_byaddr(vap, addr);
	if (sm == NULL) {
		MALLOC(sm, struct ieee80211_hbr_sm *, sizeof(struct ieee80211_hbr_sm), 
			M_80211_NODE, M_NOWAIT | M_ZERO);
		sm->hbr_vap = vap;
		IEEE80211_ADDR_COPY(sm->hbr_addr, addr);
		IEEE80211_HBR_LOCK_BH(ht);
		ht->hbr_count ++;
		TAILQ_INSERT_TAIL(&ht->hbr_node, sm, hbr_list);
		IEEE80211_HBR_UNLOCK_BH(ht);
	}
	/* We always reset the state of the state machine to ACTIVE, regardless it is
	 * a new entry or one already exists in the list, because this function is
	 * called by ieee80211_node_join which imply that we should reset the state
	 * even if the entry already exists 
	 */
	SM_ENTER(HBR_SM, ACTIVE);	
}
EXPORT_SYMBOL_C(ieee80211_hbr_addentry);

/* Delete an entry from the list indicated by the address */
void ieee80211_hbr_delentry(struct ieee80211vap *vap, u_int8_t *addr)
{
	struct ieee80211_hbr_sm *sm;
	struct ieee80211_hbr_list *ht;
	ht = &(vap->iv_hbr_list);
	sm = ieee80211_hbr_find_byaddr(vap, addr);
	if (sm) {
		IEEE80211_HBR_LOCK_BH(ht);
		TAILQ_REMOVE(&ht->hbr_node, sm, hbr_list);
		ht->hbr_count --;
		IEEE80211_HBR_UNLOCK_BH(ht);
		/* Before we release this entry, we need to reset the state of this
		 * entry to ACTIVE, thus the block flags of all the nodes with its address
		 * will be cleared. 
		 */
		SM_ENTER(HBR_SM, ACTIVE);
		FREE(sm, M_80211_NODE);
	}
}
EXPORT_SYMBOL_C(ieee80211_hbr_delentry);

/* Iterate all associated node to send out probing frames */
void ieee80211_hbr_probe_allnodes(struct ieee80211vap *vap, u_int8_t *addr)
{
	ieee80211_iterate_vap_nodes(vap, ieee80211_hbr_probe, (void *)addr);
}
void ieee80211_hbr_probe(void *arg, struct ieee80211_node *ni)
{
	u_int8_t *addr;
	addr = (u_int8_t *)arg;
	if (ni) {
	    if(IEEE80211_ADDR_EQ(addr, ni->ni_macaddr) && 
			ni->ni_associd && ni != ni->ni_vap->iv_bss) 
		{
			ni->ni_ique_flag = M_PROBING;
			ieee80211_send_qosnulldata(ni, WME_AC_VI);
            ni->ni_ique_flag = 0;
		}
	}
}

void ieee80211_hbr_setblock_allnodes(struct ieee80211vap *vap, u_int8_t *addr, u_int8_t block)
{
	u_int8_t arg[IEEE80211_ADDR_LEN+1];
	IEEE80211_ADDR_COPY(arg, addr);
	arg[IEEE80211_ADDR_LEN] = block;	
	ieee80211_iterate_vap_nodes(vap, ieee80211_hbr_setblock, (void *)arg);	
}

void ieee80211_hbr_setblock(void *arg, struct ieee80211_node *ni)
{
	u_int8_t *addr;
	addr = (u_int8_t *)arg;
	if (ni) {
	    if(IEEE80211_ADDR_EQ(addr, ni->ni_macaddr) && 
			ni->ni_associd && ni != ni->ni_vap->iv_bss) 
		{
			ni->ni_hbr_block = addr[IEEE80211_ADDR_LEN];
            if (!ni->ni_hbr_block)
                ni->ni_ique_flag = 0;                    
		} 
	}
}

/* Based on the signal from the rate control module (which is in the ath_dev layer),
 * find out the state machine associated with the address and then setup the tri-state 
 * trigger based upon the signal and current state
 */
void ieee80211_hbr_settrigger_byaddr(struct ieee80211vap *vap, u_int8_t *addr, int signal)
{
	struct ieee80211_hbr_sm *sm;
	sm = ieee80211_hbr_find_byaddr(vap, addr);
	if (sm) {
		if (sm->hbr_sm_state == HBR_SM_ACTIVE && 
			signal == HBR_SIGNAL_PROBING) 
		{
			sm->hbr_trigger = HBR_TRIGGER_BACK;
		} else if (sm->hbr_sm_state != HBR_SM_ACTIVE &&
			signal == HBR_SIGNAL_ACTIVE) 
		{
			sm->hbr_trigger = HBR_TRIGGER_FORWARD;
		} else {
			sm->hbr_trigger = HBR_TRIGGER_STALL;	
		}
	}
	/* TODO if sm == NULL, it means there's no entry in the list with this address, 
	 * this looks like will never happern. However, shoud we add an entry to the list 
	 * IF sm is NULL? Would it be a little bit too aggressive? 
	 */
}
EXPORT_SYMBOL_C(ieee80211_hbr_settrigger_byaddr);

int ieee80211_hbr_reportstate_byaddr(struct ieee80211vap *vap, u_int8_t *addr)
{
	struct ieee80211_hbr_sm *sm;
	sm = ieee80211_hbr_find_byaddr(vap, addr);
	if (sm) {
		switch (sm->hbr_sm_state) {
			case HBR_SM_ACTIVE:
				return HBR_SIGNAL_ACTIVE;
			case HBR_SM_PROBING:
				return HBR_SIGNAL_PROBING;
			case HBR_SM_BLOCKING:
				return HBR_SIGNAL_BLOCKING;	
			default:
				return -1;	
		}
	}
	return -1;
}
EXPORT_SYMBOL_C(ieee80211_hbr_reportstate_byaddr);

void ieee80211_hbr_setstate_all(struct ieee80211vap * vap, int state)
{
	ieee80211_hbr_iterate(vap, ieee80211_hbr_setstate, (void *)(&state), NULL);
}
EXPORT_SYMBOL_C(ieee80211_hbr_setstate_all);

int ieee80211_hbr_setstate(struct ieee80211_hbr_sm *sm, void *arg1, void *arg2)
{
	int state;
	state = *(int *)arg1;
	switch(state) {
		case HBR_SM_ACTIVE:
			SM_ENTER(HBR_SM, ACTIVE);
			break;
		case HBR_SM_BLOCKING:
			SM_ENTER(HBR_SM, BLOCKING);
			break;
		case HBR_SM_PROBING:
			SM_ENTER(HBR_SM, PROBING);
			break;
		default:
			break;	
	}
	return 0;
}

void ieee80211_hbr_getstate_all(struct ieee80211vap * vap)
{
	int i;
	i = 0;
	printk("HBR list dump\nNode\tAddress\t\t\tState\tTrigger\tBlock\n");
	ieee80211_hbr_iterate(vap, ieee80211_hbr_getstate, (void *)(&i), NULL);
	printk("Nodes information\nAddress\t\t\tBlock\t\tDroped VI frames\n");
	ieee80211_iterate_vap_nodes(vap, ieee80211_hbr_nodestate, NULL);
}
EXPORT_SYMBOL_C(ieee80211_hbr_getstate_all);

int ieee80211_hbr_getstate(struct ieee80211_hbr_sm *sm, void *arg1, void *arg2)
{
		int state;
		u_int8_t block;
		int8_t trigger;
		state = sm->hbr_sm_state;
		trigger = sm->hbr_trigger;
		block = sm->hbr_block;
		printk("%d\t %2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x\t%s\t%s\t%s\n", 
			*(int *)arg1, sm->hbr_addr[0], sm->hbr_addr[1], sm->hbr_addr[2], sm->hbr_addr[3],
			sm->hbr_addr[4], sm->hbr_addr[5], 
			(state==HBR_SM_ACTIVE)?"ACTIVE":((state==HBR_SM_BLOCKING)?"BLOCKING":"PROBING"),
			(trigger==HBR_TRIGGER_BACK)?"BACK":((trigger==HBR_TRIGGER_STALL)?"STALL":"FORWARD"),
			(block==0)?"No":"Yes");
		(*(int *)arg1) ++;
		return 0;
}

void ieee80211_hbr_nodestate(void *arg, struct ieee80211_node *ni)
{
	if (ni) {
		printk("%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x\t%s\t\t%d\n",
			ni->ni_macaddr[0], ni->ni_macaddr[1], ni->ni_macaddr[2], ni->ni_macaddr[3], 
			ni->ni_macaddr[4], ni->ni_macaddr[5], ni->ni_hbr_block?"Yes":"No",
            ni->ni_stats.ns_tx_dropblock);
	}
}

void ieee80211_hbr_init(struct ieee80211vap *vap)
{
	vap->iv_hbr_list.hbrlist_vap = vap;	
	vap->iv_hbr_list.hbr_count = 0;
	IEEE80211_HBR_LOCK_INIT(&(vap->iv_hbr_list),"HBR List");
	TAILQ_INIT(&vap->iv_hbr_list.hbr_node);
}
EXPORT_SYMBOL_C(ieee80211_hbr_init);
#endif

