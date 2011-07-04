/*
 *	Forwarding database
 *	Linux ethernet bridge
 *
 *	Authors:
 *	Lennert Buytenhek		<buytenh@gnu.org>
 *
 *	$Id: //depot/sw/releases/7.3_AP/linux/kernels/mips-linux-2.6.15/net/bridge/br_fdb.c#7 $
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/times.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/jhash.h>
#include <asm/atomic.h>
#include "br_private.h"

static kmem_cache_t *br_fdb_cache __read_mostly;
static int fdb_insert(struct net_bridge *br, struct net_bridge_port *source,
		      const unsigned char *addr);

void __init br_fdb_init(void)
{
	br_fdb_cache = kmem_cache_create("bridge_fdb_cache",
					 sizeof(struct net_bridge_fdb_entry),
					 0,
					 SLAB_HWCACHE_ALIGN, NULL, NULL);
#ifdef CONFIG_ATHR_ETHERNET_PORT_MAC_LIMIT
        athr_port_mac_limit_init();
#endif
}

void __exit br_fdb_fini(void)
{
	kmem_cache_destroy(br_fdb_cache);
}


/* if topology_changing then use forward_delay (default 15 sec)
 * otherwise keep longer (default 5 minutes)
 */
static __inline__ unsigned long hold_time(const struct net_bridge *br)
{
	return br->topology_change ? br->forward_delay : br->ageing_time;
}

static __inline__ int has_expired(const struct net_bridge *br,
				  const struct net_bridge_fdb_entry *fdb)
{
	return !fdb->is_static 
		&& time_before_eq(fdb->ageing_timer + hold_time(br), jiffies);
}

static __inline__ int br_mac_hash(const unsigned char *mac)
{
	return jhash(mac, ETH_ALEN, 0) & (BR_HASH_SIZE - 1);
}

static __inline__ void fdb_delete(struct net_bridge_fdb_entry *f)
{
	hlist_del_rcu(&f->hlist);
	br_fdb_put(f);
}

static void fdb_rcu_free(struct rcu_head *head);

#ifdef CONFIG_ATHR_ETHERNET_PORT_MAC_LIMIT

/*
 * Add a virtual phy port for wifi/wan interface
 */
static struct athr_eth_physical_port athr_phy_ports[ATHR_MAX_PHY_PORTS + 1] ;


static void athr_print_mac(struct mac_addr *macaddr)
{
        unsigned char mac[ETH_ALEN];
        unsigned char mac1[40], *ptr;
        int i, cnt;

        memcpy(mac, macaddr->addr, ETH_ALEN);

        ptr = mac1;

        for (i = 0; i < ETH_ALEN; i++)  {
                cnt = sprintf(ptr, "%02x%c", mac[i], i == ETH_ALEN - 1 ? ' ':':');
                ptr += cnt;
                //printk("%02x%c", mac[i], i == ETH_ALEN - 1 ? ' ':':');
        }

        *(ptr + 1)='\0';

        printk("%s\n", mac1);
}

static void __init athr_port_mac_limit_init(void)
{

        int i=0;

        for (i=0; i < ATHR_MAX_PHY_PORTS; i++) {
            athr_phy_ports[i].total_mac_allowed = ATHR_DEFAULT_MAX_MAC_ADDRS_PER_PORT;
            athr_phy_ports[i].mac_addr_count = 0;
        }

        printk("%s: athr_port_mac_limit_init init complete ...\n", __func__);
}

struct athr_eth_physical_port * athr_get_phy_port_from_num(int phy_port_num)
{
        if (phy_port_num < 0 || phy_port_num > (ATHR_MAX_PHY_PORTS + 1)) {
                printk("%s: invalid phy port no %d specified\n", __func__, phy_port_num);
                return NULL;
        }
        else  {
                /*
                 * For wan/wifi interface, port no can be zero, so
                 * assign the max+1 phy port to them
                 */
                if (phy_port_num == 0) {
                        return &athr_phy_ports[ATHR_MAX_PHY_PORTS];
                }
                else
                {
                        return &athr_phy_ports[phy_port_num - 1];
                }
        }

}

void athr_fdb_delete_by_phy_port(struct net_bridge *br, struct athr_eth_physical_port *phy_port)
{
	int i;
	struct net_bridge_fdb_entry *f;

	for (i = 0; i < BR_HASH_SIZE; i++) {
		struct hlist_node *h, *g;

		hlist_for_each_safe(h, g, &br->hash[i]) {
			f = hlist_entry(h, struct net_bridge_fdb_entry, hlist);
			if (!f->is_local && f->phy_port == phy_port) {
                                fdb_delete(f);
                        }
		}
	}
}

int athr_set_phy_port_mac_limit(struct net_bridge *br, int phy_port_num, int new_total_mac_addrs)
{

        struct athr_eth_physical_port *phy_port;

        phy_port = athr_get_phy_port_from_num(phy_port_num);

        if (phy_port == NULL)
                return -EINVAL;

	spin_lock_bh(&br->hash_lock);

        if (new_total_mac_addrs < phy_port->total_mac_allowed) {
            /* clean the mac table */
            printk("%s: phy port %d cleaning the mac table entries\n", __func__, phy_port_num);
            athr_fdb_delete_by_phy_port(br, phy_port);
        }

        phy_port->total_mac_allowed = new_total_mac_addrs;

	spin_unlock_bh(&br->hash_lock);

        return 0;
}


static int athr_phy_port_mac_ageing_timer_op(struct net_bridge *br, unsigned char *mac_addr,
                                      uint16_t new_ageing_interval, int op)
{
	int i, age = -ENODATA;
	struct net_bridge_fdb_entry *f;

	spin_lock_bh(&br->hash_lock);

	for (i = 0; i < BR_HASH_SIZE; i++) {
		struct hlist_node *h, *g;

		hlist_for_each_safe(h, g, &br->hash[i]) {
			f = hlist_entry(h, struct net_bridge_fdb_entry, hlist);
			if (!f->is_local && !compare_ether_addr(mac_addr, f->addr.addr)) {
                                if (op == 1) {
                                        f->max_age = (new_ageing_interval * HZ);
                                        if (f->max_age == 0) {
                                                f->ageing_timer = 0;
                                        }
                                        else
                                        {
                                                f->ageing_timer = jiffies;
                                        }

                                        age = 0;
                                }
                                else if (op == 0) {
                                        age = (f->max_age > 0) ? (f->max_age / HZ) : f->max_age;
                                }
                                break;
                        }
		}
	}


	spin_unlock_bh(&br->hash_lock);

        return age;
}

int athr_set_phy_port_mac_ageing_interval(struct net_bridge *br, unsigned char *mac_addr, int16_t new_ageing_interval)
{
	int ret = 0;

        if (new_ageing_interval < 0) {
                printk("%s: invalid ageing interval %d specified\n", 
                       __func__, new_ageing_interval);
                return -EINVAL;
        }

        ret = athr_phy_port_mac_ageing_timer_op(br, mac_addr, new_ageing_interval, 1);

        return ret;

}

int athr_get_phy_port_mac_ageing_interval(struct net_bridge *br, unsigned char *mac_addr)
{

        int ret = 0;

        ret = athr_phy_port_mac_ageing_timer_op(br, mac_addr, 0, 0);

        return ret;

}

int athr_get_phy_port_mac_addr_limit(int phy_port_num)
{
        struct athr_eth_physical_port *phy_port;

        phy_port = athr_get_phy_port_from_num(phy_port_num);

        if (phy_port == NULL)
                return -EINVAL;

        return phy_port->total_mac_allowed;

}

int athr_get_mac_table_from_phy_port(struct net_bridge *br, int phy_port_num, void __user *userbuf)
{
	int i, cnt = 0, total_mac_count;
	struct net_bridge_fdb_entry *f;
        struct athr_eth_physical_port *phy_port;
        struct athr_phy_port_mac_table *mac_table;
	struct hlist_node *h;
        void *buf;
        int ret=0;
        size_t size =0;
        unsigned long maxnum = 0;

        phy_port = athr_get_phy_port_from_num(phy_port_num);

        if (phy_port == NULL)
                return -EINVAL;

	rcu_read_lock();

        total_mac_count = phy_port->mac_addr_count;

        if (total_mac_count < 1) {
                printk("%s: mac table is empty for port %d\n", __func__, phy_port_num);
                rcu_read_unlock();
                return 0;
        }

        size = phy_port->mac_addr_count * sizeof(struct athr_phy_port_mac_table);

        if (size > PAGE_SIZE) {
                size = PAGE_SIZE;
                maxnum = PAGE_SIZE/sizeof(struct athr_phy_port_mac_table);
        }

        buf = kmalloc(size, GFP_USER);

        if (!buf) {
                printk("%s: kmalloc failed\n", __func__);
                rcu_read_unlock();
                return -ENOMEM;
        }

        mac_table = buf;

	for (i = 0; i < BR_HASH_SIZE; i++) {
		hlist_for_each_entry_rcu(f, h, &br->hash[i], hlist) {
			if (!f->is_local && f->phy_port == phy_port) {

                                memcpy(mac_table->addr, f->addr.addr, ETH_ALEN);
                                mac_table->addr[ETH_ALEN + 1] = '\0';

                                mac_table->max_ageing_interval = (f->max_age > 0) ? \
                                                (f->max_age / HZ) : f->max_age;

                                mac_table->cur_ageing_interval = (f->ageing_timer > 0) ? \
                                                ((jiffies - f->ageing_timer) / HZ) : 
                                                f->ageing_timer;
                                ++mac_table;
                                ++cnt;

                                if (maxnum != 0 && cnt == maxnum) {
                                        break;
                                }
                        }
		}
	}

        if (cnt > 0) {
            if (copy_to_user(userbuf, buf, (cnt * sizeof(struct athr_phy_port_mac_table))))
                     ret = -EFAULT;
            else
                     ret = cnt;
        }
        else
                ret = 0;

        kfree(buf);

        rcu_read_unlock();

        return ret;
}

#endif


void br_fdb_changeaddr(struct net_bridge_port *p, const unsigned char *newaddr)
{
	struct net_bridge *br = p->br;
	int i;

	spin_lock_bh(&br->hash_lock);

	/* Search all chains since old address/hash is unknown */
	for (i = 0; i < BR_HASH_SIZE; i++) {
		struct hlist_node *h;
		hlist_for_each(h, &br->hash[i]) {
			struct net_bridge_fdb_entry *f;

			f = hlist_entry(h, struct net_bridge_fdb_entry, hlist);
			if (f->dst == p && f->is_local) {
				/* maybe another port has same hw addr? */
				struct net_bridge_port *op;
				list_for_each_entry(op, &br->port_list, list) {
					if (op != p && 
					    !compare_ether_addr(op->dev->dev_addr,
								f->addr.addr)) {
						f->dst = op;
						goto insert;
					}
				}

				/* delete old one */
				fdb_delete(f);
				goto insert;
			}
		}
	}
 insert:
	/* insert new address,  may fail if invalid address or dup. */
	fdb_insert(br, p, newaddr);

	spin_unlock_bh(&br->hash_lock);
}

void br_fdb_cleanup(unsigned long _data)
{
	struct net_bridge *br = (struct net_bridge *)_data;
	unsigned long delay = hold_time(br);
#ifdef CONFIG_ATHR_ETHERNET_PORT_MAC_LIMIT
	struct net_bridge_fdb_entry *f;
#endif
	int i;

	spin_lock_bh(&br->hash_lock);
	for (i = 0; i < BR_HASH_SIZE; i++) {
#ifndef CONFIG_ATHR_ETHERNET_PORT_MAC_LIMIT
		struct net_bridge_fdb_entry *f;
#endif
		struct hlist_node *h, *n;

		hlist_for_each_entry_safe(f, h, n, &br->hash[i], hlist) {
#ifndef CONFIG_ATHR_ETHERNET_PORT_MAC_LIMIT
			if (!f->is_static && 
			    time_before_eq(f->ageing_timer + delay, jiffies)) 
				fdb_delete(f);
#else
            if (!f->is_static && (f->phy_port != NULL)) {
                                /*
                                 * Delete the entry only if the mac ageing
                                 * interval is set to non-zero
                                 */
                                delay = f->max_age;
                                if (delay > 0) {
                                        if (time_before_eq(f->ageing_timer +
                                                           delay,
                                                           jiffies)) {
                                                fdb_delete(f);
                                        }
                                }
                        }
#endif
		}
	}
	spin_unlock_bh(&br->hash_lock);

	mod_timer(&br->gc_timer, jiffies + HZ/10);
}

void br_fdb_delete_by_port(struct net_bridge *br, struct net_bridge_port *p)
{
	int i;

	spin_lock_bh(&br->hash_lock);
	for (i = 0; i < BR_HASH_SIZE; i++) {
		struct hlist_node *h, *g;

		hlist_for_each_safe(h, g, &br->hash[i]) {
			struct net_bridge_fdb_entry *f
				= hlist_entry(h, struct net_bridge_fdb_entry, hlist);
			if (f->dst != p) 
				continue;

			/*
			 * if multiple ports all have the same device address
			 * then when one port is deleted, assign
			 * the local entry to other port
			 */
			if (f->is_local) {
				struct net_bridge_port *op;
				list_for_each_entry(op, &br->port_list, list) {
					if (op != p && 
					    !compare_ether_addr(op->dev->dev_addr,
								f->addr.addr)) {
						f->dst = op;
						goto skip_delete;
					}
				}
			}

			fdb_delete(f);
		skip_delete: ;
		}
	}
	spin_unlock_bh(&br->hash_lock);
}

/* No locking or refcounting, assumes caller has no preempt (rcu_read_lock) */
struct net_bridge_fdb_entry *__br_fdb_get(struct net_bridge *br,
					  const unsigned char *addr)
{
	struct hlist_node *h;
	struct net_bridge_fdb_entry *fdb;

	hlist_for_each_entry_rcu(fdb, h, &br->hash[br_mac_hash(addr)], hlist) {
		if (!compare_ether_addr(fdb->addr.addr, addr)) {
			if (unlikely(has_expired(br, fdb)))
				break;
			return fdb;
		}
	}

	return NULL;
}

/* Interface used by ATM hook that keeps a ref count */
struct net_bridge_fdb_entry *br_fdb_get(struct net_bridge *br, 
					unsigned char *addr)
{
	struct net_bridge_fdb_entry *fdb;

	rcu_read_lock();
	fdb = __br_fdb_get(br, addr);
	if (fdb) 
		atomic_inc(&fdb->use_count);
	rcu_read_unlock();
	return fdb;
}

static void fdb_rcu_free(struct rcu_head *head)
{
	struct net_bridge_fdb_entry *ent
		= container_of(head, struct net_bridge_fdb_entry, rcu);
#ifdef CONFIG_ATHR_ETHERNET_PORT_MAC_LIMIT
        if (ent->phy_port != NULL) {
                if (ent->phy_port->mac_addr_count > 0) {
                        --(ent->phy_port->mac_addr_count);
                        ent->phy_port = NULL;
                        printk("%s: Deleting entry for mac addr : ", __func__);
                        athr_print_mac(&ent->addr);
                }
        }
        else
                printk("%s: Danger will robinson, fdb entry with no phy_port...\n", __func__);
#endif
	kmem_cache_free(br_fdb_cache, ent);
}

/* Set entry up for deletion with RCU  */
void br_fdb_put(struct net_bridge_fdb_entry *ent)
{
	if (atomic_dec_and_test(&ent->use_count))
		call_rcu(&ent->rcu, fdb_rcu_free);
}

/*
 * Fill buffer with forwarding table records in 
 * the API format.
 */
int br_fdb_fillbuf(struct net_bridge *br, void *buf,
		   unsigned long maxnum, unsigned long skip)
{
	struct __fdb_entry *fe = buf;
	int i, num = 0;
	struct hlist_node *h;
	struct net_bridge_fdb_entry *f;

	memset(buf, 0, maxnum*sizeof(struct __fdb_entry));

	rcu_read_lock();
	for (i = 0; i < BR_HASH_SIZE; i++) {
		hlist_for_each_entry_rcu(f, h, &br->hash[i], hlist) {
			if (num >= maxnum)
				goto out;

			if (has_expired(br, f)) 
				continue;

			if (skip) {
				--skip;
				continue;
			}

			/* convert from internal format to API */
			memcpy(fe->mac_addr, f->addr.addr, ETH_ALEN);
			fe->port_no = f->dst->port_no;
			fe->is_local = f->is_local;
			if (!f->is_static)
				fe->ageing_timer_value = jiffies_to_clock_t(jiffies - f->ageing_timer);
			++fe;
			++num;
		}
	}

 out:
	rcu_read_unlock();

	return num;
}

static inline struct net_bridge_fdb_entry *fdb_find(struct hlist_head *head,
						    const unsigned char *addr)
{
	struct hlist_node *h;
	struct net_bridge_fdb_entry *fdb;

	hlist_for_each_entry_rcu(fdb, h, head, hlist) {
		if (!compare_ether_addr(fdb->addr.addr, addr))
			return fdb;
	}
	return NULL;
}

static struct net_bridge_fdb_entry *fdb_create(struct hlist_head *head,
					       struct net_bridge_port *source,
					       const unsigned char *addr, 
#ifdef CONFIG_ATHR_ETHERNET_PORT_MAC_LIMIT
					       int is_local,
                                               int phy_port_num)
#else
					       int is_local)

#endif
{
	struct net_bridge_fdb_entry *fdb;
#ifdef CONFIG_ATHR_ETHERNET_PORT_MAC_LIMIT
        struct athr_eth_physical_port *phy_port;
        struct mac_addr mac;

        phy_port =  athr_get_phy_port_from_num(phy_port_num);

        if (phy_port == NULL) {
            printk("%s: packet from mac addr %s arrived on invalid phy port no %d\n",
                   __func__, addr, phy_port_num);
            return NULL;
        }

        /*
         * check the mac limit only if the total mac allowed
         */
        if (phy_port->total_mac_allowed > 0) {
                if (phy_port->mac_addr_count == phy_port->total_mac_allowed) {
                    printk("%s: phy port %d mac limit %d reached, dropping mac addr: ",
                           __func__, phy_port_num, phy_port->total_mac_allowed);
                    memcpy(mac.addr, addr, ETH_ALEN);
                    athr_print_mac(&mac);
                    return NULL;
                }
        }
#endif
	fdb = kmem_cache_alloc(br_fdb_cache, GFP_ATOMIC);
	if (fdb) {
		memcpy(fdb->addr.addr, addr, ETH_ALEN);
		atomic_set(&fdb->use_count, 1);
		hlist_add_head_rcu(&fdb->hlist, head);

		fdb->dst = source;
		fdb->is_local = is_local;
		fdb->is_static = is_local;
                fdb->ageing_timer = jiffies;
#ifdef CONFIG_ATHR_ETHERNET_PORT_MAC_LIMIT
                fdb->max_age = (ATHR_DEFAULT_PORT_MAC_AGEING_INTERVAL * HZ);
                fdb->phy_port = phy_port;

                /* Don't count the physical port lan address */

                if (!is_local) {
                        ++(phy_port->mac_addr_count);
                }
#endif
	}
	return fdb;
}

static int fdb_insert(struct net_bridge *br, struct net_bridge_port *source,
		  const unsigned char *addr)
{
	struct hlist_head *head = &br->hash[br_mac_hash(addr)];
	struct net_bridge_fdb_entry *fdb;

	if (!is_valid_ether_addr(addr))
		return -EINVAL;

	fdb = fdb_find(head, addr);
	if (fdb) {
		/* it is okay to have multiple ports with same 
		 * address, just use the first one.
		 */
		if (fdb->is_local) 
			return 0;

		printk(KERN_WARNING "%s adding interface with same address "
		       "as a received packet\n",
		       source->dev->name);
		fdb_delete(fdb);
 	}

#ifdef CONFIG_ATHR_ETHERNET_PORT_MAC_LIMIT
        /* Pass port no 1 by default for the local device address */
	if (!fdb_create(head, source, addr, 1, 1))
#else
	if (!fdb_create(head, source, addr, 1))
#endif
		return -ENOMEM;

	return 0;
}

int br_fdb_insert(struct net_bridge *br, struct net_bridge_port *source,
		  const unsigned char *addr)
{
	int ret;

	spin_lock_bh(&br->hash_lock);
	ret = fdb_insert(br, source, addr);
	spin_unlock_bh(&br->hash_lock);
	return ret;
}

#ifndef CONFIG_ATHR_ETHERNET_PORT_MAC_LIMIT
void br_fdb_update(struct net_bridge *br, struct net_bridge_port *source,
		   const unsigned char *addr)
#else
int br_fdb_update(struct net_bridge *br, struct net_bridge_port *source,
		   const unsigned char *addr, int phy_port)
#endif
{
	struct hlist_head *head = &br->hash[br_mac_hash(addr)];
	struct net_bridge_fdb_entry *fdb;
#ifdef CONFIG_ATHR_ETHERNET_PORT_MAC_LIMIT
        int ret=0;
#endif

	/* some users want to always flood. */
#ifndef CONFIG_ATHR_ETHERNET_PORT_MAC_LIMIT
	if (hold_time(br) == 0)
		return;
#else
	if (hold_time(br) == 0) {
		return -1;
        }
#endif

	rcu_read_lock();

	fdb = fdb_find(head, addr);

	if (likely(fdb)) {
		/* attempt to update an entry for a local interface */
		if (unlikely(fdb->is_local)) {
			if (net_ratelimit())
				printk(KERN_WARNING "%s: received packet with "
				       " own address as source address\n",
				       source->dev->name);
		} else {
			/* fastpath: update of existing entry */
			fdb->dst = source;
#ifdef CONFIG_ATHR_ETHERNET_PORT_MAC_LIMIT
           if (fdb->max_age > 0) {
               fdb->ageing_timer = jiffies;
           }
#else
                        fdb->ageing_timer = jiffies;
#endif
		}
	} else {
		spin_lock_bh(&br->hash_lock);
		if (!fdb_find(head, addr))
#ifdef CONFIG_ATHR_ETHERNET_PORT_MAC_LIMIT
                        if (fdb_create(head, source, addr, 0, phy_port) == NULL) {
                                ret = -1;
                        }
#else
			fdb_create(head, source, addr, 0);
#endif
		/* else  we lose race and someone else inserts
		 * it first, don't bother updating
		 */
		spin_unlock_bh(&br->hash_lock);
	}
	rcu_read_unlock();

#ifdef CONFIG_ATHR_ETHERNET_PORT_MAC_LIMIT
   return ret;
#endif
}

