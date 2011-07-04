/*
 *  ebt_masq
 *
 *	Authors:
 *	David Claffey <dclaffey@jjplus.com>
 *	Bart De Schuymer <bdschuym@pandora.be>
 *
 *  February, 2008
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/times.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/rcupdate.h>
#include <linux/jhash.h>
#include <asm/atomic.h>
#include <linux/netfilter_bridge/ebtables.h>
#include <linux/netfilter_bridge/ebt_nat.h>
#include <linux/module.h>
#include <net/sock.h>
#include <linux/if_ether.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <linux/if_arp.h>
#include <net/arp.h>

#define NMACHEX_FMT "%02x:%02x:%02x:%02x:%02x:%02x"
#define NMACHEX(x) (x)[0],(x)[1],(x)[2],(x)[3],(x)[4],(x)[5]

struct masq_entry
{
	struct hlist_node	hlist;
	struct rcu_head		rcu;
	atomic_t		use_count;
	uint32_t                ip;
	unsigned char	        mac[ETH_ALEN];
	unsigned long           ageing_timer;
};

static struct kmem_cache *masq_cache __read_mostly;

#define MASQ_HASH_BITS 8
#define MASQ_HASH_SIZE (1 << MASQ_HASH_BITS)

static spinlock_t		masq_hash_lock;
static uint32_t                masq_hash_rnd;
static struct timer_list       masq_timer;
static struct hlist_head	masq_hash_table[MASQ_HASH_SIZE];

#define AGEING_TIME_DFT 300 /* 5 minutes, matches default bridge timing */
#define MIN_AGEING_TIME(x) (((x) ? x : AGEING_TIME_DFT ) * HZ)

static uint32_t masq_hash(const uint32_t ip, const struct net_device * dev)
{
	return jhash_2words(ip, dev->ifindex, masq_hash_rnd) 
		& (MASQ_HASH_SIZE - 1); 
}

/* No locking or refcounting, assumes caller has no preempt (rcu_read_lock) */
static inline struct masq_entry *masq_entry_find(struct hlist_head *head, 
						 const uint32_t ip)
{
	struct hlist_node *h;
	struct masq_entry *m_entry;
	
	hlist_for_each_entry_rcu(m_entry, h, head, hlist) {
		if (atomic_read(&m_entry->use_count) && m_entry->ip == ip)
			return m_entry;
	}
	
	return NULL;
}

int masq_copy_mac( struct net_device * dev, const uint32_t ip, unsigned char *mac,
					uint32_t time)
{
	struct hlist_head *head = &masq_hash_table[masq_hash(ip, dev)];
	struct masq_entry *m_entry;
	
	rcu_read_lock();
	m_entry = masq_entry_find(head, ip);
	if (m_entry && !atomic_read(&m_entry->use_count))
		m_entry = NULL;

	if(m_entry)
		memcpy(mac, m_entry->mac, ETH_ALEN);
	
	rcu_read_unlock();

	if(m_entry) {
		spin_lock_bh(&masq_hash_lock);
		m_entry->ageing_timer = jiffies + MIN_AGEING_TIME(time);
		spin_unlock_bh(&masq_hash_lock);
		return 0;
	}

	return -ENODATA;
}

static void masq_entry_rcu_free(struct rcu_head *head)
{
	struct masq_entry *m_entry = container_of(head, struct masq_entry, rcu);
	kmem_cache_free(masq_cache, m_entry);
}

static __inline__ void masq_entry_delete(struct masq_entry *m_entry)
{
	hlist_del_rcu(&m_entry->hlist);
	if (atomic_dec_and_test(&m_entry->use_count))
		call_rcu(&m_entry->rcu, masq_entry_rcu_free);
}

static void masq_cleanup(unsigned long force)
{
	int i;

	spin_lock_bh(&masq_hash_lock);
	for (i = 0; i < MASQ_HASH_SIZE; i++) {
		struct masq_entry *m_entry;
		struct hlist_node *h, *n;
		
		hlist_for_each_entry_safe(m_entry, h, n, 
					  &masq_hash_table[i], hlist) {
			if (force || 
			    time_after(jiffies,m_entry->ageing_timer))
				masq_entry_delete(m_entry);
		}
	}
	spin_unlock_bh(&masq_hash_lock);
	
	if(!force)
		mod_timer(&masq_timer, jiffies + HZ/10);
}

/* assume lock acquired */
static struct masq_entry * masq_entry_create(struct hlist_head *head, const uint32_t ip, 
			     const unsigned char *mac, uint32_t time)
{
	struct masq_entry *m_entry;

	m_entry = kmem_cache_alloc(masq_cache, GFP_ATOMIC);
	if (m_entry) {
		atomic_set(&m_entry->use_count, 1);
		hlist_add_head_rcu(&m_entry->hlist, head);
		m_entry->ip = ip;
		memcpy(m_entry->mac, mac, ETH_ALEN);
	}

	return  m_entry;
}

static __inline__ int __masq_update( struct net_device * dev, const uint32_t ip, 
			  const unsigned char * mac, uint32_t time)
{
	struct hlist_head *head = &masq_hash_table[masq_hash(ip, dev)];
	struct masq_entry *m_entry;

	m_entry = masq_entry_find(head, ip);

	if (unlikely(!m_entry)) {
		m_entry = masq_entry_create(head, ip, mac, time);
		if(!m_entry)
			return -ENOMEM;
	}
	else {
		if(memcmp(m_entry->mac, mac, ETH_ALEN)) {
			printk(KERN_INFO "MASQ %u.%u.%u.%u changed from "
			       NMACHEX_FMT " to " NMACHEX_FMT "\n",
			       NIPQUAD(ip), NMACHEX(m_entry->mac), NMACHEX(mac));
			memcpy(m_entry->mac, mac, ETH_ALEN);
		}
	}
	
	m_entry->ageing_timer = jiffies + MIN_AGEING_TIME(time);
	return 0;
}

static int masq_update ( struct net_device * dev, const uint32_t ip, 
		  const unsigned char * mac, uint32_t time)
{
	int ret;

	spin_lock_bh(&masq_hash_lock);
	ret = __masq_update(dev, ip, mac, time);
	spin_unlock_bh(&masq_hash_lock);

	return ret;
}

#ifdef CONFIG_PROC_FS
static int masq_cache_procinfo(char *buffer, char **start, off_t offset, int length)
{
        int len = 0, i;
        int expires;
	
	rcu_read_lock();

	len += sprintf(buffer, "       MAC                    IP           Expires       Use\n");
	for (i = 0; i < MASQ_HASH_SIZE; i++) {
		struct hlist_node *h;
		struct masq_entry *m_entry;
		unsigned long tmp = jiffies;
		
		hlist_for_each_entry_rcu(m_entry, h, &masq_hash_table[i], hlist) {
			if (!atomic_read(&m_entry->use_count))
				continue;
			if(time_before_eq(jiffies, m_entry->ageing_timer))
				expires = (m_entry->ageing_timer - tmp) / HZ;
			else
				expires = 0;
			len += sprintf(buffer + len, 
				       NMACHEX_FMT"\t%u.%u.%u.%u\t%8u\t%3u\n",
				       NMACHEX(m_entry->mac), 
				       NIPQUAD(m_entry->ip),
				       expires,
				       (unsigned  int) atomic_read(&m_entry->use_count));
		}
	}
	rcu_read_unlock();

        len -= offset;
        if (len > length)
                len = length;
        if (len < 0)
                len = 0;
        *start = buffer + offset;
        return len;
}
#endif

struct bootp_pkt {		/* BOOTP packet format */
	struct iphdr iph;	/* IP header */
	struct udphdr udph;	/* UDP header */
	u8 op;			/* 1=request, 2=reply */
#define BOOTP_REQUEST	1
#define BOOTP_REPLY	2
	u8 htype;		/* HW address type */
	u8 hlen;		/* HW address length */
	u8 hops;		/* Used only by gateways */
	__be32 xid;		/* Transaction ID */
	__be16 secs;		/* Seconds since we started */
	__be16 flags;		/* Just what it says */
	__be32 client_ip;	/* Client's IP address if known */
	__be32 your_ip;		/* Assigned IP address */
	__be32 server_ip;	/* (Next, e.g. NFS) Server's IP address */
	__be32 relay_ip;	/* IP address of BOOTP relay */
	u8 hw_addr[ETH_ALEN];	/* Client's HW address */
};

static int ebt_target_masq(struct sk_buff **pskb, unsigned int hooknr,
   const struct net_device *in, const struct net_device *out,
   const void *data, unsigned int datalen)
{
	struct ebt_masq_info *info = (struct ebt_masq_info *) data;
	struct arphdr _ah, *ap;
	struct iphdr _iph, *ih;
	struct sk_buff *skb = *pskb;
	unsigned char mac[ETH_ALEN];
	uint32_t ip;
	struct bootp_pkt *b = (struct bootp_pkt *) skb->nh.iph;
	struct udphdr * uh = &b->udph;
	unsigned short cs;

	/* inspect ARPs packet and record sender IP and MAC into hash tables */
	if(out) {
		if (eth_hdr(skb)->h_proto == __constant_htons(ETH_P_ARP)) {
			ap = skb_header_pointer(skb, 0, sizeof(_ah), &_ah);
			if ((ap->ar_op != htons(ARPOP_REQUEST) &&
			     ap->ar_op != htons(ARPOP_REPLY)) ||
			    ap->ar_hln != ETH_ALEN ||
			    ap->ar_pro != htons(ETH_P_IP) ||
			    ap->ar_pln != 4)
			    return EBT_DROP;
			skb_copy_bits(skb, sizeof(_ah), mac, ETH_ALEN);
			skb_copy_bits(skb, sizeof(_ah) + ETH_ALEN, &ip, 4);
			masq_update((struct net_device *) out, ip, mac, info->ageing_time);
		}
		/* if dhcp request, set broadcast flag (if not already set) */
		if((eth_hdr(skb)->h_proto == __constant_htons(ETH_P_IP)) &&
		   (skb->nh.iph->protocol == IPPROTO_UDP) &&
		   (uh->source == __constant_htons(68)) &&
		   (uh->dest == __constant_htons(67)) &&
		   (b->htype == ARPHRD_ETHER) &&
		   (b->op == BOOTP_REQUEST) &&
		   is_broadcast_ether_addr(eth_hdr(skb)->h_dest) &&
		   !(b->flags & __constant_htons(0x8000))) {
			/* set the broadcast bit in the flags */
			b->flags |= __constant_htons(0x8000);
			/* fixup UDP checksum */
			cs = -(__constant_ntohs(uh->check));
			cs += 0x8000;
			if(cs < 0x8000) /* add carry */
				cs++;
			uh->check = __constant_htons(-cs);
		}
		
		return info->target;
	}

	if(!in || is_multicast_ether_addr(eth_hdr(skb)->h_dest))
		return info->target;
	
	/* IP and ARP have IP addresses we can use */
	if(!(eth_hdr(skb)->h_proto == __constant_htons(ETH_P_IP) || 
	     eth_hdr(skb)->h_proto == __constant_htons(ETH_P_ARP)))
		return info->target;

	/* get IP from packet and restore mac if lookup successful */
	if(eth_hdr(skb)->h_proto == __constant_htons(ETH_P_IP)) {
		ih = skb_header_pointer(skb, 0, sizeof(_iph), &_iph);
		ip = ih->daddr;
	}
	
	if(eth_hdr(skb)->h_proto == __constant_htons(ETH_P_ARP)) {
		ap = skb_header_pointer(skb, 0, sizeof(_ah), &_ah);
		skb_copy_bits(skb, sizeof(_ah) + (2*ETH_ALEN) + sizeof(uint32_t), &ip, 4);
	}
	
	masq_copy_mac((struct net_device *) in, ip, eth_hdr(skb)->h_dest,
                  info->ageing_time);
	
	/* fixup the ARP messsage TARGET HA */
	if(eth_hdr(skb)->h_proto == __constant_htons(ETH_P_ARP))
		skb_store_bits(skb, sizeof(_ah) + ETH_ALEN + sizeof(uint32_t), 
			       eth_hdr(skb)->h_dest, ETH_ALEN);
	
	return info->target;
}

static int ebt_target_masq_check(const char *tablename, unsigned int hookmask,
				 const struct ebt_entry *e, void *data, unsigned int datalen)
{
	struct ebt_masq_info *info = (struct ebt_masq_info *) data;
	
	if (BASE_CHAIN && info->target == EBT_RETURN)
		return -EINVAL;
	CLEAR_BASE_CHAIN_BIT;
	if (strcmp(tablename, "nat"))
		return -EINVAL;
	if (hookmask & ~((1 << NF_BR_PRE_ROUTING) | (1 << NF_BR_POST_ROUTING)))
		return -EINVAL;
	if (datalen != EBT_ALIGN(sizeof(struct ebt_masq_info)))
		return -EINVAL;
	if (INVALID_TARGET)
		return -EINVAL;
	
	return 0;
}

static struct ebt_target masq =
{
	.name		= EBT_MASQ_TARGET,
	.target		= ebt_target_masq,
	.check		= ebt_target_masq_check,
	.me		= THIS_MODULE,
};

static int __init ebt_masq_init(void)
{
	masq_cache = kmem_cache_create("ebt_masq_cache", 
					  sizeof(struct masq_entry),
					  0, SLAB_HWCACHE_ALIGN, NULL, NULL);
	memset(masq_hash_table, 0, sizeof(masq_hash_table));
	spin_lock_init(&masq_hash_lock);
	get_random_bytes(&masq_hash_rnd, 4);
	setup_timer(&masq_timer, masq_cleanup, 0);
#ifdef CONFIG_PROC_FS
        proc_net_create("ebt_masq_cache", 0, masq_cache_procinfo);
#endif
	mod_timer(&masq_timer, jiffies + HZ/10);
	return ebt_register_target(&masq);
}

static void __exit ebt_masq_fini(void)
{
	ebt_unregister_target(&masq);
	del_timer_sync(&masq_timer);
#ifdef CONFIG_PROC_FS
	proc_net_remove("ebt_masq_cache");
#endif
	masq_cleanup(1);
	rcu_barrier();
	kmem_cache_destroy(masq_cache);
}

module_init(ebt_masq_init);
module_exit(ebt_masq_fini);
MODULE_LICENSE("GPL");
