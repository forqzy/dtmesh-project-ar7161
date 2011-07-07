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
 * $Id: //depot/sw/releases/7.3_AP/wlan/linux/net80211/ieee80211_me.c#5 $
 */
/********************************************************************/
/* \file ieee80211_me.c
 * \brief Atheros multicast enhancement algorithm, as part of 
 * Atheros iQUE modules.
 *
 * This file contains the main implementation of the Atheros multicast 
 * enhancement functionality, as a module namely ath_me.
 *
 * The main purpose of this module is to convert (by translating or 
 * tunneling) the multicast stream into duplicated unicast streams for 
 * performance enhancement of home wireless applications. For more 
 * details, please refer to the documentation in this folder.
 */


#ifdef IEEE80211_MCAST_ENHANCEMENT 


/*
 * IEEE 802.11 multicast specific code (multicast tunnel and snoop)
 */

#include <linux/version.h>
#include <linux/igmp.h> // RSP-OK gebraucht für igmphdr

#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif

#include <linux/module.h>
#include <linux/kmod.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/sysctl.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/ip.h>
#include <osdep.h>
#include <net/iw_handler.h>
#include <linux/wireless.h>

#include "if_media.h"
#include "if_ethersubr.h"
#include "if_athproto.h"

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_me.h>
#include <net80211/ieee80211_linux.h>

extern struct ieee80211_ique_me_ops vap_me_ops;

static struct MC_GROUP_LIST_ENTRY *
ieee80211_me_SnoopListFindMember(struct ieee80211vap *vap, u_int8_t *gaL2, u_int8_t *saL2);
static struct MC_GROUP_LIST_ENTRY *
ieee80211_me_SnoopListFindEntry(struct ieee80211vap *vap, struct ieee80211_node *ni);
static void ieee80211_me_SnoopListDelEntry(struct ieee80211vap *vap, 
	struct MC_GROUP_LIST_ENTRY *pEntry);
static void ieee80211_me_SnoopListInit(struct ieee80211vap *vap);
static int ieee80211_me_SnoopListUpdate(struct ieee80211vap *vap, int cmd,
	u_int8_t *gaL2, u_int8_t *saL2, struct ieee80211_node *ni);
static int ieee80211_me_SnoopListGetGroup(struct ieee80211vap *vap, u_int8_t *gaL2, u_int8_t *table);
OS_DECLARE_TIMER(ieee80211_me_SnoopListTimer);
/**********************************************************************
 * !
 * \brief Find the member of the snoop list with matching group address
 * and MAC address of STA
 *
 * \param   vap
 * 			gaL2 Group address
 * 			saL2 MAC address of STA
 *
 * 	\return entry for the matching item in the snoop list
 */

static struct MC_GROUP_LIST_ENTRY *
ieee80211_me_SnoopListFindMember(struct ieee80211vap *vap, u_int8_t *gaL2,
	u_int8_t *saL2)
{
	struct MC_SNOOP_LIST *ps = &(vap->iv_me->ieee80211_me_snooplist);	
	struct MC_GROUP_LIST_ENTRY *tmpEntry;

	IEEE80211_SNOOP_LOCK_BH(ps);
	TAILQ_FOREACH(tmpEntry, &ps->msl_node, mgl_list) {
	
		if (IEEE80211_ADDR_EQ(gaL2, tmpEntry->mgl_group_addr) &&
			IEEE80211_ADDR_EQ(saL2, tmpEntry->mgl_group_member))
		{
			/* found match */
			IEEE80211_SNOOP_UNLOCK_BH(ps);
			return tmpEntry;
		}
	}

	IEEE80211_SNOOP_UNLOCK_BH(ps);
	return NULL;
}


/***************************************************************************
 * !
 * \brief Find the entry of the item in the snoop list matching the node pointer
 *
 * \param   vap Pointer to the vap
 * 			ni Pointer to the node
 *
 * 	\return Pointer to the matching item in the snoop list
 */
static struct MC_GROUP_LIST_ENTRY *
ieee80211_me_SnoopListFindEntry(struct ieee80211vap *vap, struct ieee80211_node *ni)
{
	struct MC_SNOOP_LIST *ps = &(vap->iv_me->ieee80211_me_snooplist);
	struct MC_GROUP_LIST_ENTRY *tmpEntry;

	IEEE80211_SNOOP_LOCK_BH(ps);
	TAILQ_FOREACH(tmpEntry, &ps->msl_node, mgl_list)
	{
		if(IEEE80211_ADDR_EQ(ni->ni_macaddr, tmpEntry->mgl_ni->ni_macaddr))
		{
			IEEE80211_SNOOP_UNLOCK_BH(ps);

			return(tmpEntry);
		}
	}
	IEEE80211_SNOOP_UNLOCK_BH(ps);

	return(NULL);
}


/*****************************************************************************
 * !
 * \brief Delete one entry in the snoop list
 *
 * \param vap Pointer to the VAP 
 * 		  pEntry Pointer to the item to be deleted in the snoop list
 *
 * 	\return N/A
 */
static void 
ieee80211_me_SnoopListDelEntry(struct ieee80211vap *vap, 
	struct MC_GROUP_LIST_ENTRY *pEntry)
{
	struct MC_SNOOP_LIST *ps = &(vap->iv_me->ieee80211_me_snooplist);	
	if (vap->iv_me->ieee80211_mc_snoop_debug & 2)
	{
		IEEE80211_DPRINTF(vap, IEEE80211_MSG_IQUE, "Snoop Entry Del [%2.2x] (%2.2x) <%10lu>\n",
			pEntry->mgl_group_addr[5], pEntry->mgl_group_member[5], jiffies);
	}

	IEEE80211_SNOOP_LOCK_BH(ps);
	TAILQ_REMOVE(&ps->msl_node, pEntry, mgl_list);
	ps->msl_group_list_count--;
	IEEE80211_SNOOP_UNLOCK_BH(ps);
	FREE(pEntry, M_80211_NODE);

	return;
}

/*
 * DEBUG
 * Dump the mc snoop table.
 */
void
ieee80211_me_SnoopListDump(struct ieee80211vap *vap)
{
	struct MC_SNOOP_LIST *ps = &(vap->iv_me->ieee80211_me_snooplist);	
	struct MC_GROUP_LIST_ENTRY *tmpEntry;

	printf("Snoop Dump:\n");
    printf("  count  %d\n", ps->msl_group_list_count);
    printf("  misc   %d\n", ps->msl_misc);
    printf("  headf  %p\n", ps->msl_node.tqh_first);
    printf("  headl  %p\n", ps->msl_node.tqh_last);
	if (ps->msl_node.tqh_last) {
	    printf("  headl* %p\n", *ps->msl_node.tqh_last);
	} else {
		printf("  headl null\n");
	}
    if (TAILQ_EMPTY(&ps->msl_node)) {
		printf("  ---- Empty ----\n");
	} else {
		printf("  ---------------\n");

		TAILQ_FOREACH(tmpEntry, &ps->msl_node, mgl_list) {

			printf("    group     %2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x\n",
					tmpEntry->mgl_group_addr[0],
					tmpEntry->mgl_group_addr[1],
					tmpEntry->mgl_group_addr[2],
					tmpEntry->mgl_group_addr[3],
					tmpEntry->mgl_group_addr[4],
					tmpEntry->mgl_group_addr[5]);
			printf("    member    %2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x\n",
					tmpEntry->mgl_group_member[0],
					tmpEntry->mgl_group_member[1],
					tmpEntry->mgl_group_member[2],
					tmpEntry->mgl_group_member[3],
					tmpEntry->mgl_group_member[4],
					tmpEntry->mgl_group_member[5]);
			printf("    node      %2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x\n",
					tmpEntry->mgl_ni->ni_macaddr[0],
					tmpEntry->mgl_ni->ni_macaddr[1],
					tmpEntry->mgl_ni->ni_macaddr[2],
					tmpEntry->mgl_ni->ni_macaddr[3],
					tmpEntry->mgl_ni->ni_macaddr[4],
					tmpEntry->mgl_ni->ni_macaddr[5]);
			printf("    timestamp %d\n", tmpEntry->mgl_timestamp);
			printf("	xmited: %d frames\n", tmpEntry->mgl_xmited);
		}
	}
}

/**************************************************************************
 * !
 * \brief Initialize the mc snooping and encapsulation feature.
 *
 * \param vap
 *
 * \return N/A
 */
static void ieee80211_me_SnoopListInit(struct ieee80211vap *vap)
{
	vap->iv_me->ieee80211_me_snooplist.msl_group_list_count = 0;
	vap->iv_me->ieee80211_me_snooplist.msl_misc = 0;
	IEEE80211_SNOOP_LOCK_INIT(&vap->iv_me->ieee80211_me_snooplist, "Snoop Table");
	TAILQ_INIT(&vap->iv_me->ieee80211_me_snooplist.msl_node);
}

/**************************************************************************
 * !
 * \brief Add or remove entries in the mc snoop table based on
 * IGMP JOIN or LEAVE messages.
 *
 * \param vap
 * 		  cmd	IGMP Command
 * 		  gaL2	Group address
 * 		  saL2	MAC address of the STA
 * 		  ni	Pointer to the node
 *
 * \return		  
 */
static int ieee80211_me_SnoopListUpdate(struct ieee80211vap *vap, int cmd,
	u_int8_t *gaL2, u_int8_t *saL2, struct ieee80211_node *ni)
{
	struct MC_SNOOP_LIST *ps = &(vap->iv_me->ieee80211_me_snooplist);	
	struct MC_GROUP_LIST_ENTRY *pNew, *pOld;
	systime_t timestamp; 
	
	if (cmd == IGMP_SNOOP_CMD_JOIN) {

		pNew = ieee80211_me_SnoopListFindMember(vap, gaL2, saL2);

		if (pNew) {
			if (vap->iv_me->ieee80211_mc_snoop_debug & 2) {
				IEEE80211_DPRINTF(vap, IEEE80211_MSG_IQUE, "Snoop Entry Found [%2.2x] (%2.2x)\n", gaL2[5], saL2[5]);
			}
		}
		else {
			if (vap->iv_me->ieee80211_mc_snoop_debug & 2) {
				IEEE80211_DPRINTF(vap, IEEE80211_MSG_IQUE, "Snoop Entry Add [%2.2x] (%2.2x) <%10lu>\n",
					gaL2[5], saL2[5], jiffies);
			}

			if (ps->msl_group_list_count < ps->msl_max_length) {
				IEEE80211_SNOOP_LOCK_BH(ps);
				ps->msl_group_list_count++;
				IEEE80211_SNOOP_UNLOCK_BH(ps);

				MALLOC(pNew, struct MC_GROUP_LIST_ENTRY *,
						sizeof(struct MC_GROUP_LIST_ENTRY),
						M_80211_NODE, M_NOWAIT | M_ZERO);
		
				if (pNew != NULL) {
					IEEE80211_SNOOP_LOCK_BH(ps);
					TAILQ_INSERT_TAIL(&ps->msl_node, pNew, mgl_list);
					IEEE80211_ADDR_COPY(pNew->mgl_group_addr, gaL2);
					IEEE80211_ADDR_COPY(pNew->mgl_group_member, saL2);
					timestamp  = OS_GET_TIMESTAMP();
					pNew->mgl_timestamp = (u_int32_t) CONVERT_SYSTEM_TIME_TO_MS(timestamp);
					pNew->mgl_ni = ni;
					pNew->mgl_xmited = 0;
					IEEE80211_SNOOP_UNLOCK_BH(ps);
				}
			} 	
		}
	}
	else if (cmd == IGMP_SNOOP_CMD_LEAVE) {

		pOld = ieee80211_me_SnoopListFindMember(vap, gaL2, saL2);
		if (pOld) {
			ieee80211_me_SnoopListDelEntry(vap, pOld);
		}
		else {
			if (vap->iv_me->ieee80211_mc_snoop_debug & 2) {
				IEEE80211_DPRINTF(vap, IEEE80211_MSG_IQUE, "Snoop Entry Not Found [%2.2x] (%2.2x)\n", gaL2[5], saL2[5]);
			}
		}
	}

	if (vap->iv_me->ieee80211_mc_snoop_debug & 4) {
		ieee80211_me_SnoopListDump(vap);
	}

	return 0;
}

/**********************************************************************************
 * !
 * \brief Fill in tmp table of all unicast destinations for the given group.
 *
 * \param vap Pointer to VAP
 * 			gaL2 Group address
 * 			table Temporary table containing the matching items
 *
 * \return Count of table rows filled in.
 */
static int ieee80211_me_SnoopListGetGroup(struct ieee80211vap *vap, u_int8_t *gaL2,
	u_int8_t *table)
{
	struct MC_SNOOP_LIST *ps = &(vap->iv_me->ieee80211_me_snooplist);	
	struct MC_GROUP_LIST_ENTRY *tmpEntry;
	int idx = 0;
	int count = 0;

	IEEE80211_SNOOP_LOCK_BH(ps);
	TAILQ_FOREACH(tmpEntry, &ps->msl_node, mgl_list) {
		if (IEEE80211_ADDR_EQ(gaL2, tmpEntry->mgl_group_addr)) {
			IEEE80211_ADDR_COPY(&table[idx], tmpEntry->mgl_group_member);
			idx += IEEE80211_ADDR_LEN;
			count++;
		}
	}
	IEEE80211_SNOOP_UNLOCK_BH(ps);

	return count;
}

/*************************************************************************
 * !
 * \brief Function of the timer to update the snoop list 
 *
 * \param unsigned long arg been casted to pointer to vap
 *
 * \return N/A
 */
OS_TIMER_FUNC(ieee80211_me_SnoopListTimer)
{
	struct ieee80211vap *vap;
	struct MC_SNOOP_LIST *ps;	
	struct MC_GROUP_LIST_ENTRY *tmpEntry, *tmpEntry2;
	systime_t timestamp  = OS_GET_TIMESTAMP();
	u_int32_t now        = (u_int32_t) CONVERT_SYSTEM_TIME_TO_MS(timestamp);
	u_int32_t oldest_timestamp =  0 ;
	u_int32_t nxt_timer;

	OS_GET_TIMER_ARG(vap, struct ieee80211vap *);
	ps = (struct MC_SNOOP_LIST *)&(vap->iv_me->ieee80211_me_snooplist);	


	IEEE80211_SNOOP_LOCK(ps);
	tmpEntry = TAILQ_FIRST(&ps->msl_node);

	TAILQ_FOREACH_SAFE(tmpEntry, &ps->msl_node, mgl_list, tmpEntry2) {
		if (now - tmpEntry->mgl_timestamp > vap->iv_me_timeout) {
			ieee80211_me_SnoopListDelEntry(vap, tmpEntry);
		}
		else {
			if((tmpEntry->mgl_timestamp < oldest_timestamp) || (!oldest_timestamp))
				oldest_timestamp = tmpEntry->mgl_timestamp ;	
		}		
	}
	IEEE80211_SNOOP_UNLOCK(ps);

	if(!oldest_timestamp) {
		nxt_timer = vap->iv_me_timer;
	}
	else {
		nxt_timer =  (vap->iv_me_timeout + 1) - (now - oldest_timestamp);
	}

	OS_SET_TIMER(&vap->iv_me->snooplist_timer, nxt_timer);
	return;
}

/*************************************************************************
 * !
 * \brief Build up the snoop list by monitoring the IGMP packets
 *
 * \param ni Pointer to the node
 * 		  skb Pointer to the skb buffer
 *
 * \return N/A
 */
void
ieee80211_me_SnoopInspecting(struct ieee80211_node *ni, struct sk_buff *skb)
{
	struct ieee80211vap *vap = ni->ni_vap;
	struct net_device *dev = vap->iv_dev;
	struct ether_header *eh = (struct ether_header *) skb->data;
	int i;

	if (vap->iv_opmode == IEEE80211_M_HOSTAP ) {
        if (IEEE80211_IS_MULTICAST(eh->ether_dhost) &&
		    !IEEE80211_ADDR_EQ(eh->ether_dhost, dev->broadcast))
		{
			if (eh->ether_type == __constant_htons(ETHERTYPE_IP)) {
				const struct iphdr *ip = (struct iphdr *)
					(skb->data + sizeof (struct ether_header));

				if (ip->protocol == 2) {
					/* ver1, ver2 */
	 				const struct igmphdr *igmp = (struct igmphdr *)
						(skb->data + sizeof (struct ether_header) + (4 * ip->ihl));
					/* ver 3*/
	 				const struct igmpv3_report *igmpr3 = (struct igmpv3_report *) igmp;
					u_int32_t	groupAddr = 0;
					int			cmd = IGMP_SNOOP_CMD_OTHER;
					u_int8_t	*srcAddr = eh->ether_shost;
					u_int8_t	groupAddrL2[IEEE80211_ADDR_LEN];

					switch (igmp->type) {
					case 0x11:
						/* query */
						groupAddr = igmp->group;
						break;
					case 0x12:
						/* V1 report */
						groupAddr = igmp->group;
						cmd = IGMP_SNOOP_CMD_JOIN;
						break;
					case 0x16:
						/* V2 report */
						groupAddr = igmp->group;
						cmd = IGMP_SNOOP_CMD_JOIN;
						break;
					case 0x17:
						/* V2 leave */
						groupAddr = igmp->group;
						cmd = IGMP_SNOOP_CMD_LEAVE;
						break;
					case 0x22:
						/* V3 report */
						groupAddr = igmpr3->grec[0].grec_mca;
						if (igmpr3->grec[0].grec_type == IGMPV3_CHANGE_TO_EXCLUDE ||
							igmpr3->grec[0].grec_type == IGMPV3_MODE_IS_EXCLUDE)
						{
							cmd = IGMP_SNOOP_CMD_JOIN;
						}
						else if (igmpr3->grec[0].grec_type == IGMPV3_CHANGE_TO_INCLUDE ||
							igmpr3->grec[0].grec_type == IGMPV3_MODE_IS_INCLUDE)
						{
							cmd = IGMP_SNOOP_CMD_LEAVE;
						}
						break;
					default:
						break;
					}

#ifdef IEEE80211_DEBUG
					if (vap->iv_me->ieee80211_mc_snoop_debug & 1) {
						ieee80211_note(vap, "IGMP %s %2.2x [%2.2x] %8.8x - %02x:%02x:%02x:%02x:%02x:%02x\n",
							cmd == 2 ? "Leave" : (cmd == 1 ? "Join " : "other"),
							igmp->type,
							igmp->type == 0x22 ? igmpr3->grec[0].grec_type : 0,
							groupAddr,
							srcAddr[0],srcAddr[1],srcAddr[2],
							srcAddr[3],srcAddr[4],srcAddr[5]);
					}
#endif
					groupAddrL2[0] = 0x01;
					groupAddrL2[1] = 0x00;
					groupAddrL2[2] = 0x5e;
					groupAddrL2[3] = (groupAddr >> 16) & 0x7f;
					groupAddrL2[4] = (groupAddr >>  8) & 0xff;
					groupAddrL2[5] = (groupAddr >>  0) & 0xff;
					for (i = 0; i < GRPADDR_FILTEROUT_N; i ++) {
						if(groupAddrL2[3] == grpaddr_filterout[i][3] &&
							groupAddrL2[4] == grpaddr_filterout[i][4] && 
							groupAddrL2[5] == grpaddr_filterout[i][5] )
								return;
					}
					ieee80211_me_SnoopListUpdate(vap, cmd, &groupAddrL2[0], srcAddr, ni);
				}
			}
		}
	}

	/* multicast tunnel egress */
	if (eh->ether_type == htons(ATH_ETH_TYPE) &&
		vap->iv_opmode == IEEE80211_M_STA &&
		vap->iv_mc_snoop_enable)
	{
		struct athl2p_tunnel_hdr *tunHdr;

		tunHdr = (struct athl2p_tunnel_hdr *) skb_pull(skb, sizeof(struct ether_header));
		/* check protocol subtype */
		eh = (struct ether_header *) skb_pull(skb, sizeof(struct athl2p_tunnel_hdr));
	}

#ifdef PRINT_MCENCAP_DBG
    ieee80211_note(vap,"ieee80211_deliver_data() %d eh-%02x:%02x:%02x:%02x:%02x:%02x-%02x:%02x:%02x:%02x:%02x:%02x-%04x-%02x:%02x:%02x:%02x:%02x:%02x\n",
		skb_headroom(skb),
		eh->ether_dhost[0],eh->ether_dhost[1],eh->ether_dhost[2],
        eh->ether_dhost[3],eh->ether_dhost[4],eh->ether_dhost[5],
		eh->ether_shost[0],eh->ether_shost[1],eh->ether_shost[2],
        eh->ether_shost[3],eh->ether_shost[4],eh->ether_shost[5],
		eh->ether_type,
		*((unsigned char *)(skb->data) + 14),
		*((unsigned char *)(skb->data) + 15),
		*((unsigned char *)(skb->data) + 16),
		*((unsigned char *)(skb->data) + 17),
		*((unsigned char *)(skb->data) + 18),
		*((unsigned char *)(skb->data) + 19)
		);
#endif /* PRINT_MCENCAP_DBG */

	return;
}


/***************************************************************************
 * !
 * \brief Cleanup the snoop list for the specific node
 *
 * \param ni Pointer to the node
 *
 * \return N/A
 */
void ieee80211_me_SnoopWDSNodeCleanup(struct ieee80211_node *ni)
{
	struct ieee80211vap *vap = ni->ni_vap;
	struct MC_SNOOP_LIST *ps = &(vap->iv_me->ieee80211_me_snooplist);

	if(vap->iv_opmode == IEEE80211_M_HOSTAP &&
		vap->iv_mc_snoop_enable &&
		ps != NULL)
	{
		struct MC_GROUP_LIST_ENTRY *pEntry;

		while((pEntry = ieee80211_me_SnoopListFindEntry(vap, ni)) != NULL)
		{
#ifdef IEEE80211_DEBUG
			if(vap->iv_me->ieee80211_mc_snoop_debug & 1)
			{
				ieee80211_note(vap,
					"%s: WDS node %02x:%02x:%02x:%02x:%02x:%02x\n", __func__,
					ni->ni_macaddr[0],ni->ni_macaddr[1],ni->ni_macaddr[2],
					ni->ni_macaddr[3],ni->ni_macaddr[4],ni->ni_macaddr[5]);
			}
#endif
	
			ieee80211_me_SnoopListDelEntry(vap, pEntry);
		}
	}

	return;
}

/*******************************************************************
 * !
 * \brief Mcast enhancement option 1: Tunneling, or option 2: Translate
 *
 * Add an IEEE802.3 header to the mcast packet using node's MAC address
 * as the destination address
 *
 * \param 
 * 			vap Pointer to the virtual AP
 * 			skb Pointer to the sk_buffer
 *
 * \return number of packets converted and transmitted, or 0 if failed
 */

int
ieee80211_me_SnoopConvert(struct ieee80211vap *vap, struct sk_buff *skb)
{
//	struct ieee80211com *ic = vap->iv_ic;
//	struct net_device *parent = ic->ic_dev;
	struct ieee80211_node *ni = NULL;
	struct ieee80211_cb *cb;
	struct ether_header *eh;
	u_int8_t *dstmac;						/* reference to frame dst addr */
	u_int8_t srcmac[IEEE80211_ADDR_LEN];	/* copy of original frame src addr */
	u_int8_t grpmac[IEEE80211_ADDR_LEN];	/* copy of original frame group addr */
											/* table of tunnel group dest mac addrs */
	u_int8_t newmac[MAX_SNOOP_ENTRIES][IEEE80211_ADDR_LEN];
	int newmaccnt = 0;						/* count of entries in newmac */
	int newmacidx = 0;						/* local index into newmac */
	int xmited = 0;							/* Number of packets converted and xmited */
	struct ether_header *eh2;				/* eth hdr of tunnelled frame */
	struct athl2p_tunnel_hdr *tunHdr;		/* tunnel header */
	struct sk_buff *skb1 = NULL;			/* cloned skb if necessary */
	struct MC_GROUP_LIST_ENTRY *mgle;
    systime_t timestamp;

	eh = (struct ether_header *)skb->data;

	/* Get members of this group */
	/* save original frame's src addr once */
	IEEE80211_ADDR_COPY(srcmac, eh->ether_shost);
	IEEE80211_ADDR_COPY(grpmac, eh->ether_dhost);
	dstmac = eh->ether_dhost;
	newmaccnt = ieee80211_me_SnoopListGetGroup(vap, eh->ether_dhost, &newmac[0][0]);
	/* save original frame's src addr once */
	/*
	 * If newmaccnt is 0: not multicast or multicast not in snoop table,
	 *                    no tunnel
	 *                 1: mc in table, only one dest, skb cloning not needed
	 *                >1: multiple mc in table, skb cloned for each entry past
	 *                    first one.
	 */


	/* loop start */
	do {	

		if (newmaccnt > 0) {	
			/* reference new dst mac addr */
			dstmac = &newmac[newmacidx][0];
		

		/*
		 * Note - cloned here pre-tunnel due to the way ieee80211_classify()
		 * currently works. This is not efficient.  Better to split 
		 * ieee80211_classify() functionality into per node and per frame,
		 * then clone the post-tunnelled copy.
		 * Currently, the priority must be determined based on original frame,
		 * before tunnelling.
		 */
			if (newmaccnt > 1) {
				skb1 = skb_copy(skb,GFP_ATOMIC);
				if(skb1 != NULL) {
					cb = (struct ieee80211_cb *) skb1->cb;
					cb->ni = ni;
					cb->flags = 0;
				}
			}
		}
		
		/* In case of loop */
		if(IEEE80211_ADDR_EQ(dstmac, srcmac)) {
			goto bad;
		}

		/* Look up destination */
		ni = ieee80211_find_txnode(vap, dstmac);

		/* Drop frame if dest not found in tx table */
		if (ni == NULL) {
			/* NB: ieee80211_find_txnode does stat+msg */
			if (newmaccnt > 0) {
				mgle = ieee80211_me_SnoopListFindMember(vap, grpmac, dstmac);
				if(mgle != NULL) {
					ieee80211_me_SnoopListDelEntry(vap, mgle);
				}
			}
			goto bad2;
		}

		/* Drop frame if dest sta not associated */
		if (ni->ni_associd == 0 && ni != vap->iv_bss) {
			/* the node hasn't been associated */

			if(ni != NULL) {
#ifdef NODE_FREE_DEBUG
                                ieee80211_add_trace(ni, (char *)__func__, "DecRef", ieee80211_node_refcnt(ni));
#endif
				ieee80211_node_decref(ni);
			}
			
			if (newmaccnt > 0) {
				ieee80211_me_SnoopWDSNodeCleanup(ni);
			}
			goto bad;
		}

		/* calculate priority so drivers can find the tx queue */
		if (ieee80211_classify(ni, skb)) {
			IEEE80211_NOTE(vap, IEEE80211_MSG_OUTPUT, ni,
				"%s: discard, classification failure", __func__);

			/*XXX Should we del this entry for this case???*/
#if 0			
			mgle = ath_me_SnoopListFindMember(sc, grpmac, dstmac);
			if(mgle != NULL) {
				ath_me_SnoopListDelEntry(sc, mgle);
			}
#endif			
			goto bad2;
		}


		/* Insert tunnel header
		 * eh is the pointer to the ethernet header of the original frame.
		 * eh2 is the pointer to the ethernet header of the encapsulated frame.
		 *
		 */
		if (newmaccnt > 0 && vap->iv_mc_snoop_enable) {
			/*Option 1: Tunneling*/
			if (vap->iv_mc_snoop_enable & 1) {
	
				/* Encapsulation */
				tunHdr = (struct athl2p_tunnel_hdr *) skb_push(skb, sizeof(struct athl2p_tunnel_hdr));
				eh2 = (struct ether_header *) skb_push(skb, sizeof(struct ether_header));
		
				/* ATH_ETH_TYPE protocol subtype */
				tunHdr->proto = 17;
			
				/* copy original src addr */
				IEEE80211_ADDR_COPY(&eh2->ether_shost[0], srcmac);
	
				/* copy new ethertype */
				eh2->ether_type = htons(ATH_ETH_TYPE);

			}
			/*Option 2: Translating*/
			else if (vap->iv_mc_snoop_enable & 2) {
				eh2 = (struct ether_header *)skb->data;
			}
			
			else {
				eh2 = (struct ether_header *)skb->data;
			}

			/* copy new dest addr */
			IEEE80211_ADDR_COPY(&eh2->ether_dhost[0], &newmac[newmacidx][0]);
			mgle = ieee80211_me_SnoopListFindMember(vap, grpmac, dstmac);
			mgle->mgl_xmited ++;
			timestamp  = OS_GET_TIMESTAMP();
			mgle->mgl_timestamp = (u_int32_t) CONVERT_SYSTEM_TIME_TO_MS(timestamp);

			if(vap->iv_me_debug & 8) {
				IEEE80211_DPRINTF(vap, IEEE80211_MSG_IQUE, "Package from %2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x\n",
					srcmac[0], srcmac[1], srcmac[2], srcmac[3], srcmac[4], srcmac[5]);
			}
		}
		cb = (struct ieee80211_cb *) skb->cb;
		cb->ni = ni;


		ieee80211_hardstart_send(vap, ni, skb);
		goto loop_end;

	bad2:
		if (ni != NULL) {
			ieee80211_free_node(ni);
		} 
	bad:
		if (skb != NULL) {
			dev_kfree_skb(skb);
		}
	
	loop_end:
		/* loop end */
		if (skb1 != NULL) {
			skb = skb1;
		}
		skb1 = NULL;
		newmacidx++;
		xmited ++;
	} while (--newmaccnt > 0 && vap->iv_mc_snoop_enable); 
	return xmited;
}


/*******************************************************************
 * !
 * \brief Attach the ath_me module for multicast enhancement. 
 *
 * This function allocates the ath_me structure and attachs function 
 * entry points to the function table of ath_softc.
 *
 * \param  vap
 *
 * \return ath_ique_me pointer allocated.
 */
int
ieee80211_me_attach(struct ieee80211vap * vap)
{
	struct ieee80211_ique_me *ame;
	OS_MALLOC_WITH_TAG(&ame, sizeof(struct ieee80211_ique_me), IQUE_ME_MEMTAG);
	if (ame == NULL)
			return 0;

	OS_MEMZERO(ame, sizeof(struct ieee80211_ique_me));


	/*Attach function entry points*/
	vap->iv_me = ame;
	ame->me_iv = vap;
	ame->ieee80211_me_snooplist.msl_max_length = vap->iv_ic->ic_me_max_length;
	vap->iv_me_ops = &vap_me_ops;
	OS_INIT_TIMER(vap->iv_dev, &ame->snooplist_timer, ieee80211_me_SnoopListTimer, vap);
	OS_SET_TIMER(&ame->snooplist_timer, vap->iv_me_timer);
	IEEE80211_ME_LOCK_INIT(vap);
	ieee80211_me_SnoopListInit(vap);
	return 1;
}


/********************************************************************
 * !
 * \brief Detach the resources for multicast enhancement
 *
 * \param sc Pointer to ATH object (this)
 *
 * \return N/A
 */
void
ieee80211_me_detach(struct ieee80211vap *vap)
{
	IEEE80211_ME_LOCK(vap);
	if(vap->iv_me) {
		OS_CANCEL_TIMER(&vap->iv_me->snooplist_timer);	
		OS_FREE_WITH_TAG(vap->iv_me, sizeof(struct ieee80211_ique_me));
		vap->iv_me = NULL;
	}
	vap->iv_me_ops = NULL;
	IEEE80211_ME_UNLOCK(vap);
	IEEE80211_ME_LOCK_DESTROY(vap);
}

#ifdef __linux__
/*
 * Linux module glue
 */

static char *dev_info = "wlan_me";

MODULE_AUTHOR("Atheros Communications, Inc.");
MODULE_DESCRIPTION("Multicast enhancement support for Atheros devices in AP mode");
#ifdef MODULE_LICENSE
MODULE_LICENSE("Proprietary");
#endif

static int __init
init_ieee80211_me(void)
{
	vap_me_ops.me_attach = ieee80211_me_attach;
	vap_me_ops.me_detach = ieee80211_me_detach;
	vap_me_ops.ieee80211_me_inspect = ieee80211_me_SnoopInspecting;
	vap_me_ops.ieee80211_me_convert = ieee80211_me_SnoopConvert;
	vap_me_ops.ieee80211_me_dump = ieee80211_me_SnoopListDump;
	vap_me_ops.ieee80211_me_cleanup = ieee80211_me_SnoopWDSNodeCleanup;

	printk(KERN_INFO "%s: Version 0.1\n"
					"Copyright (c) 2008 Atheros Communications, Inc. "
					"All Rights Reserved\n", dev_info);

	return 0;
}
module_init(init_ieee80211_me);

static void __exit
exit_ieee80211_me(void)
{
	OS_MEMZERO(&vap_me_ops, sizeof(struct ieee80211_ique_me_ops));
	printk(KERN_INFO "%s: driver unloaded\n", dev_info);
}
module_exit(exit_ieee80211_me);

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#endif

EXPORT_SYMBOL_C(ieee80211_me_attach);
EXPORT_SYMBOL_C(ieee80211_me_detach);
EXPORT_SYMBOL_C(ieee80211_me_SnoopInspecting);
EXPORT_SYMBOL_C(ieee80211_me_SnoopConvert);
EXPORT_SYMBOL_C(ieee80211_me_SnoopListDump);
EXPORT_SYMBOL_C(ieee80211_me_SnoopWDSNodeCleanup);


#endif /*__linux__*/

#endif /*IEEE80211_MCAST_ENHANCEMENT*/
