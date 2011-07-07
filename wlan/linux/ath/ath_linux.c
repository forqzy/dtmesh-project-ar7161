#include <osdep.h>
#include <wbuf.h>

#include "if_media.h"
#include <ieee80211_var.h>

#include "if_athvar.h"
#include "if_ath_cwm.h"
#include "if_ethersubr.h"

#define ATH_TXBUF   512/ATH_FRAG_PER_MSDU

/*
 * Maximum acceptable MTU
 * MAXFRAMEBODY - WEP - QOS - RSN/WPA:
 * 2312 - 8 - 2 - 12 = 2290
 */
#define ATH_MAX_MTU     2290
#define ATH_MIN_MTU     32

#define AP_PUSH_BUTTON  1
#define STA_PUSH_BUTTON 2

static struct ath_reg_parm ath_params = {
    .wModeSelect = MODE_SELECT_ALL,
    .NetBand = MODE_SELECT_ALL,
    .txAggrEnable = 1,
    .rxAggrEnable = 1,
    .txAmsduEnable = 1,
    .aggrLimit = IEEE80211_AMPDU_LIMIT_DEFAULT,
    .aggrSubframes = IEEE80211_AMPDU_SUBFRAME_DEFAULT,
    .aggrProtDuration = 8192,
    .aggrProtMax = 8192,
    .txRifsEnable = 0,
    .rifsAggrDiv = IEEE80211_RIFS_AGGR_DIV,
#ifdef ATH_RB
    .rxRifsEnable = ATH_RB_MODE_DETECT,
    .rxRifsTimeout = ATH_RB_DEF_TIMEOUT,
    .rxRifsSkipThresh = ATH_RB_DEF_SKIP_THRESH,
#endif
    .txChainMaskLegacy = 1,
    .rxChainMaskLegacy = 1,
    .rxChainDetectThreshA = 35,
    .rxChainDetectThreshG = 35,
    .rxChainDetectDeltaA = 30,
    .rxChainDetectDeltaG = 30,
    .calibrationTime = 30,
#ifdef ATH_WLAN_LED
	.gpioPinFuncs = {GPIO_PIN_FUNC_0,GPIO_PIN_FUNC_1,GPIO_PIN_FUNC_2},
#else
	.gpioPinFuncs = {1, 7, 7},
#endif
    .hwTxRetries = 4,
    .extendedChanMode = 1,
    .DmaStopWaitTime = 4,
    .swBeaconProcess = 1,
    .stbcEnable = 1,
    .cwmEnable = 1,
    .wpsButtonGpio = 0,
};

/*
** Prototype for iw attach
*/

void ath_iw_attach(struct net_device *dev);
#ifndef NO_SIMPLE_CONFIG
extern void unregister_simple_config_callback(char *name);
extern void register_simple_config_callback (char *name, void *callback, void *arg);
static irqreturn_t jumpstart_intr(int cpl, void *dev_id, struct pt_regs *regs);
#endif
void *
OS_ALLOC_VAP(osdev_t osdev, u_int32_t len)
{
    struct net_device *dev;
    struct ath_vap_net80211 *avp;

    dev = alloc_etherdev(len);
    if (dev == NULL)
        return NULL;

    avp = dev->priv;
    avp->av_vap.iv_dev = dev;

    return avp;
}

void
OS_FREE_VAP(void *netif)
{
    /* NB: netdev memeory will be reclaimed through dev->destructor callback */
}

int
ath_get_netif_settings(ieee80211_handle_t ieee)
{
    struct ieee80211com *ic = NET80211_HANDLE(ieee);
    struct net_device *dev = ic->ic_dev;
    int flags = 0;

    if (dev->flags & IFF_RUNNING)
        flags |= ATH_NETIF_RUNNING;
    if (dev->flags & IFF_PROMISC)
        flags |= ATH_NETIF_PROMISCUOUS;
    if (dev->flags & IFF_ALLMULTI)
        flags |= ATH_NETIF_ALLMULTI;

    return flags;
}

/*
 * Merge multicast addresses from all vap's to form the
 * hardware filter.  Ideally we should only inspect our
 * own list and the 802.11 layer would merge for us but
 * that's a bit difficult so for now we put the onus on
 * the driver.
 */
void
ath_mcast_merge(ieee80211_handle_t ieee, u_int32_t mfilt[2])
{
    struct ieee80211com *ic = NET80211_HANDLE(ieee);
    struct ieee80211vap *vap;
    struct dev_mc_list *mc;
    u_int32_t val;
    u_int8_t pos;

    mfilt[0] = mfilt[1] = 0;
    /* XXX locking */
    TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) {
        struct net_device *dev = vap->iv_dev;
        for (mc = dev->mc_list; mc; mc = mc->next) {
            /* calculate XOR of eight 6bit values */
            val = LE_READ_4(mc->dmi_addr + 0);
            pos = (val >> 18) ^ (val >> 12) ^ (val >> 6) ^ val;
            val = LE_READ_4(mc->dmi_addr + 3);
            pos ^= (val >> 18) ^ (val >> 12) ^ (val >> 6) ^ val;
            pos &= 0x3f;
            mfilt[pos / 32] |= (1 << (pos % 32));
        }
    }
}

static int
ath_netdev_open(struct net_device *dev)
{
    struct ath_softc_net80211 *scn = dev->priv;
    dev->flags |= IFF_RUNNING;      /* we are ready to go */
    return ath_resume(scn);
    }

static int
ath_netdev_stop(struct net_device *dev)
{
    struct ath_softc_net80211 *scn = dev->priv;
    int ret;

    ieee80211_stop_running(&scn->sc_ic);
    ath_cwm_stop(scn);
    dev->flags &= ~IFF_RUNNING;
    ret = scn->sc_ops->stop(scn->sc_dev);
    dev->flags &= (~IFF_RUNNING);
    return ret;
}

static int
ath_netdev_hardstart(struct sk_buff *skb, struct net_device *dev)
{
#ifdef DEBUG_DEPTH
#	define gotobad()	do { line = __LINE__; goto bad; } while (0)
#else
#	define gotobad()	do { goto bad; } while (0)
#endif
    struct ath_softc_net80211 *scn = dev->priv;
    struct ath_softc *sc = ATH_DEV_TO_SC(scn->sc_dev);
    struct ieee80211_cb *cb;
    struct ieee80211_node *ni;
    struct ieee80211com *ic = &scn->sc_ic;
    int error = 0;
    struct ether_header *eh = (struct ether_header *)skb->data;
    int ismulti = IEEE80211_IS_MULTICAST(eh->ether_dhost) ? 1 : 0;
    u_int16_t addba_status;
    int txq_depth, txq_aggr_nbuf, legacy_depth, txq_tot_depth;
#ifdef DEBUG_DEPTH
    int line = 0;
#endif

    cb = (struct ieee80211_cb *)skb->cb;
    ni = cb->ni;

#ifdef DEBUG_DEPTH
    /* to avoid compiler warnings */
    txq_depth = txq_aggr_nbuf = legacy_depth = txq_tot_depth = 0;
#endif
    /*
     * device must be up and running
     */
    if ((dev->flags & (IFF_RUNNING|IFF_UP)) != (IFF_RUNNING|IFF_UP)) {
        error = -ENETDOWN;
        gotobad();
    }

    /*
     * NB: check for valid node in case kernel directly sends packets
     * on wifiX interface (such as broadcast packets generated by ipv6)
     */
    if (ni == NULL)
        gotobad();

#ifdef ATH_SUPPORT_UAPSD
    /* Limit UAPSD node queue depth to WME_UAPSD_NODE_MAXQDEPTH */
    if ((ni->ni_flags & IEEE80211_NODE_UAPSD) &&
        scn->sc_ops->uapsd_depth(ATH_NODE_NET80211(ni)->an_sta) >= WME_UAPSD_NODE_MAXQDEPTH)
    {
        gotobad();
    }
#endif

    txq_depth = scn->sc_ops->txq_depth(scn->sc_dev, scn->sc_ac2q[skb->priority]);
    legacy_depth = txq_depth - scn->sc_ops->txq_aggr_depth(scn->sc_dev, scn->sc_ac2q[skb->priority]);
    txq_aggr_nbuf = scn->sc_ops->txq_aggr_nbuf(scn->sc_dev, scn->sc_ac2q[skb->priority]);
    if (legacy_depth == txq_depth && txq_aggr_nbuf) {
        /*
         * This means, only singles are being sent for 11n traffic.
         * So calculate, legacy depth appropriately.
         */
        legacy_depth = txq_depth - txq_aggr_nbuf;
    }
    txq_tot_depth = legacy_depth + txq_aggr_nbuf;

    ic->ic_addba_status(ni, cb->u_tid, &addba_status);

    /*
     * This logic throttles legacy and unaggregated HT frames if they share the hardware
     * queue with aggregates. This improves the transmit throughput performance to
     * aggregation enabled nodes when they coexist with legacy nodes.
     */
    /* Do not throttle EAPOL packets - this causes the REKEY packets
     * to be dropped and station disconnects.
     */
    DPRINTF(scn, ATH_DEBUG_RESET, "skb->priority=%d cb->u_tid=%d addba_status=%d txq_aggr_nbuf=%d txq_depth=%d\n",skb->priority, cb->u_tid, addba_status, txq_aggr_nbuf, txq_depth);

    if ((addba_status != IEEE80211_STATUS_SUCCESS) &&
        (txq_aggr_nbuf > 0) &&
        (legacy_depth >= sc->sc_limit_legacy_frames) &&
        !(eh->ether_type == ETHERTYPE_PAE)) {
        gotobad();
    } else if (txq_aggr_nbuf > (ATH_TXBUF - sc->sc_limit_legacy_frames)) {
        gotobad();
    }


    /*
     * Try to avoid running out of descriptors
     */
    if (ismulti && txq_tot_depth >= (ATH_TXBUF - 40)) {
        gotobad();
    }

    if (txq_tot_depth >= (ATH_TXBUF - scn->sc_ops->txq_lim(scn->sc_dev, scn->sc_ac2q[skb->priority]))) {
        gotobad();
    }

    error = ath_tx_send(skb);

    return error;

bad:
#ifdef DEBUG_DEPTH
    if (sc->sc_wifi) {
    printk("---:%d pri:%d qd:%u ad:%u sd:%u tot:%u amp:%d %02x:%02x:%02x\n",
		line, skb->priority,
		txq_depth, txq_aggr_nbuf, legacy_depth, txq_tot_depth,
		!(ni->ni_flags & IEEE80211_NODE_NOAMPDU), ni->ni_macaddr[3],
		ni->ni_macaddr[4], ni->ni_macaddr[5]);
    }
#endif

    IEEE80211_TX_COMPLETE_WITH_ERROR(skb, error);
    return error;
}

static void
ath_netdev_tx_timeout(struct net_device *dev)
{
    struct ath_softc_net80211 *scn = dev->priv;

    DPRINTF(scn, ATH_DEBUG_WATCHDOG, "%s: %sRUNNING\n",
            __func__, (dev->flags & IFF_RUNNING) ? "" : "!");

	if (dev->flags & IFF_RUNNING) {
        scn->sc_ops->reset_start(scn->sc_dev, 0);
        scn->sc_ops->reset(scn->sc_dev);
        scn->sc_ops->reset_end(scn->sc_dev, 0);
	}
}

static int
ath_netdev_set_macaddr(struct net_device *dev, void *addr)
{
    struct ath_softc_net80211 *scn = dev->priv;
    struct ieee80211com *ic = &scn->sc_ic;
    struct sockaddr *mac = addr;

    if (netif_running(dev)) {
        DPRINTF(scn, ATH_DEBUG_ANY,
            "%s: cannot set address; device running\n", __func__);
        return -EBUSY;
    }
    DPRINTF(scn, ATH_DEBUG_ANY, "%s: %.2x:%.2x:%.2x:%.2x:%.2x:%.2x\n",
        __func__,
        mac->sa_data[0], mac->sa_data[1], mac->sa_data[2],
        mac->sa_data[3], mac->sa_data[4], mac->sa_data[5]);

    /* XXX not right for multiple vap's */
    IEEE80211_ADDR_COPY(ic->ic_myaddr, mac->sa_data);
    IEEE80211_ADDR_COPY(dev->dev_addr, mac->sa_data);
    scn->sc_ops->set_macaddr(scn->sc_dev, dev->dev_addr);
    return 0;
}

static void
ath_netdev_set_mcast_list(struct net_device *dev)
{
    struct ath_softc_net80211 *scn = dev->priv;
    scn->sc_ops->mc_upload(scn->sc_dev);
}

static int
ath_change_mtu(struct net_device *dev, int mtu)
{
    struct ath_softc_net80211 *scn = dev->priv;

    if (!(ATH_MIN_MTU < mtu && mtu <= ATH_MAX_MTU)) {
        DPRINTF(scn, ATH_DEBUG_ANY, "%s: invalid %d, min %u, max %u\n",
                __func__, mtu, ATH_MIN_MTU, ATH_MAX_MTU);
        return -EINVAL;
    }
    DPRINTF(scn, ATH_DEBUG_ANY, "%s: %d\n", __func__, mtu);

    dev->mtu = mtu;
    return 0;
}

/*
 * Diagnostic interface to the HAL.  This is used by various
 * tools to do things like retrieve register contents for
 * debugging.  The mechanism is intentionally opaque so that
 * it can change frequently w/o concern for compatiblity.
 */
static int
ath_ioctl_diag(struct ath_softc_net80211 *scn, struct ath_diag *ad)
{
    struct ath_hal *ah = (ATH_DEV_TO_SC(scn->sc_dev))->sc_ah;
    u_int id = ad->ad_id & ATH_DIAG_ID;
    void *indata = NULL;
    void *outdata = NULL;
    u_int32_t insize = ad->ad_in_size;
    u_int32_t outsize = ad->ad_out_size;
    int error = 0;
    if (ad->ad_id & ATH_DIAG_IN) {
        /*
         * Copy in data.
         */
        indata = kmalloc(insize, GFP_KERNEL);
        if (indata == NULL) {
            error = -ENOMEM;
            goto bad;
        }
        if (copy_from_user(indata, ad->ad_in_data, insize)) {
            error = -EFAULT;
            goto bad;
        }
    }
    if (ad->ad_id & ATH_DIAG_DYN) {
        /*
         * Allocate a buffer for the results (otherwise the HAL
         * returns a pointer to a buffer where we can read the
         * results).  Note that we depend on the HAL leaving this
         * pointer for us to use below in reclaiming the buffer;
         * may want to be more defensive.
         */
        outdata = kmalloc(outsize, GFP_KERNEL);
        if (outdata == NULL) {
            error = -ENOMEM;
            goto bad;
        }
    }
    if (ath_hal_getdiagstate(ah, id, indata, insize, &outdata, &outsize)) {
        if (outsize < ad->ad_out_size)
            ad->ad_out_size = outsize;
        if (outdata &&
             copy_to_user(ad->ad_out_data, outdata, ad->ad_out_size))
            error = -EFAULT;
    } else {
        error = -EINVAL;
    }
bad:
    if ((ad->ad_id & ATH_DIAG_IN) && indata != NULL)
        kfree(indata);
    if ((ad->ad_id & ATH_DIAG_DYN) && outdata != NULL)
        kfree(outdata);
    return error;

}

extern int ath_ioctl_ethtool(struct ath_softc_net80211 *scn, int cmd, void *addr);

#ifdef ATH_SUPPORT_DFS
static int
ath_ioctl_phyerr(struct ath_softc_net80211 *scn, struct ath_diag *ad)
{
     void *indata=NULL;
     void *outdata=NULL;
     int error = 0;
     u_int32_t insize = ad->ad_in_size;
     u_int32_t outsize = ad->ad_out_size;
     u_int id= ad->ad_id & ATH_DIAG_ID;

    if (ad->ad_id & ATH_DIAG_IN) {
                /*
                 * Copy in data.
                 */
                indata = OS_MALLOC(sc->sc_osdev,insize, GFP_KERNEL);
                if (indata == NULL) {
                        error = -ENOMEM;
                        goto bad;
                }
                if (copy_from_user(indata, ad->ad_in_data, insize)) {
                        error = -EFAULT;
                        goto bad;
                }
                id = id & ~ATH_DIAG_IN;
        }
        if (ad->ad_id & ATH_DIAG_DYN) {
                /*
                 * Allocate a buffer for the results (otherwise the HAL
                 * returns a pointer to a buffer where we can read the
                 * results).  Note that we depend on the HAL leaving this
                 * pointer for us to use below in reclaiming the buffer;
                 * may want to be more defensive.
                 */
                outdata = OS_MALLOC(sc->sc_osdev, outsize, GFP_KERNEL);
                if (outdata == NULL) {
                        error = -ENOMEM;
                        goto bad;
                }
                id = id & ~ATH_DIAG_DYN;
        }

        error = scn->sc_ops->ath_dfs_control(scn->sc_dev, id, indata, insize, outdata, &outsize);

         if (outsize < ad->ad_out_size)
                ad->ad_out_size = outsize;

        if (outdata &&
            copy_to_user(ad->ad_out_data, outdata, ad->ad_out_size))
                error = -EFAULT;
bad:
        if ((ad->ad_id & ATH_DIAG_IN) && indata != NULL)
                OS_FREE(indata);
        if ((ad->ad_id & ATH_DIAG_DYN) && outdata != NULL)
                OS_FREE(outdata);

        return error;

}
#endif

static int
ath_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
    struct ath_softc_net80211 *scn = dev->priv;
    struct ieee80211com *ic = &scn->sc_ic;
    struct ath_stats *as;
    int error;

    switch (cmd) {
    case SIOCGATHSTATS:
        as = scn->sc_ops->get_ath_stats(scn->sc_dev);
        if (copy_to_user(ifr->ifr_data, as,
                         sizeof(struct ath_stats)))
            error = -EFAULT;
        else
            error = 0;
        break;
    case SIOCGATHSTATSCLR:
        as = scn->sc_ops->get_ath_stats(scn->sc_dev);
        memset(as, 0, sizeof(struct ath_stats));
        error = 0;
        break;
    case SIOCGATHDIAG:
        if (!capable(CAP_NET_ADMIN))
            error = -EPERM;
        else
            error = ath_ioctl_diag(scn, (struct ath_diag *) ifr);
        break;
#ifdef ATH_SUPPORT_DFS
    case SIOCGATHPHYERR:
        if (!capable(CAP_NET_ADMIN)) {
            error = -EPERM;
        } else {
            error = ath_ioctl_phyerr(scn, ifr->ifr_data);
        }
        break;
#endif
    case SIOCETHTOOL:
        if (copy_from_user(&cmd, ifr->ifr_data, sizeof(cmd)))
            error = -EFAULT;
        else
            error = ath_ioctl_ethtool(scn, cmd, ifr->ifr_data);
        break;
    case SIOC80211IFCREATE:
        error = ieee80211_ioctl_create_vap(ic, ifr); 
        break;
    default:
        error = -EINVAL;
        break;
    }
	return error;
}

/*
 * Return netdevice statistics.
 */
static struct net_device_stats *
ath_getstats(struct net_device *dev)
{
    struct ath_softc_net80211 *scn = dev->priv;
    struct net_device_stats *stats = &scn->sc_osdev->devstats;
    struct ath_stats *as = scn->sc_ops->get_ath_stats(scn->sc_dev);
    struct ath_phy_stats *ps;
    WIRELESS_MODE wmode;

    /* update according to private statistics */
    stats->tx_errors = as->ast_tx_xretries
             + as->ast_tx_fifoerr
             + as->ast_tx_filtered
             ;
    stats->tx_dropped = as->ast_tx_nobuf
            + as->ast_tx_encap
            + as->ast_tx_nonode
            + as->ast_tx_nobufmgt;

    for (wmode = 0; wmode < WIRELESS_MODE_MAX; wmode++) {
        ps = scn->sc_ops->get_phy_stats(scn->sc_dev, wmode);
        
        stats->rx_errors = ps->ast_rx_fifoerr;
        stats->rx_dropped = ps->ast_rx_tooshort;
        stats->rx_crc_errors = ps->ast_rx_crcerr;
    }
    
    return stats;
}

static void
ath_tasklet(TQUEUE_ARG data)
{
    struct net_device *dev = (struct net_device *)data;
    struct ath_softc_net80211 *scn = dev->priv;

    scn->sc_ops->handle_intr(scn->sc_dev);
}

irqreturn_t
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
ath_isr(int irq, void *dev_id)
#else
ath_isr(int irq, void *dev_id, struct pt_regs *regs)
#endif
{
    struct net_device *dev = dev_id;
    struct ath_softc_net80211 *scn = dev->priv;
    int sched, needmark = 0;

    /* always acknowledge the interrupt */
    sched = scn->sc_ops->isr(scn->sc_dev);
    
    switch(sched)
    {
    case ATH_ISR_NOSCHED:
        return  IRQ_HANDLED;
        
    case ATH_ISR_NOTMINE:
        return IRQ_NONE;
        
    default:
        if ((dev->flags & (IFF_RUNNING|IFF_UP)) != (IFF_RUNNING|IFF_UP))
        {
            DPRINTF(scn, ATH_DEBUG_INTR, "%s: flags 0x%x\n", __func__, dev->flags);
            scn->sc_ops->disable_interrupt(scn->sc_dev);     /* disable further intr's */
            return IRQ_HANDLED;
        }
    }
    
    /*
    ** See if the transmit queue processing needs to be scheduled
    */
    
    ATH_SCHEDULE_TQUEUE(&scn->sc_osdev->intr_tq, &needmark);
    if (needmark)
        mark_bh(IMMEDIATE_BH);

    return IRQ_HANDLED;
}

int
__ath_attach(u_int16_t devid, struct net_device *dev, HAL_BUS_CONTEXT *bus_context, osdev_t osdev)
{
    struct ath_softc_net80211 *scn = dev->priv;
    struct ieee80211com *ic = &scn->sc_ic;
    int error = 0;

    ic->ic_dev = dev;

    /*
     * create and initialize ath layer
     */
    error = ath_attach(devid, bus_context, scn, osdev, &ath_params, NULL);
    if (error != 0)
        goto bad;

    /* For STA Mode default CWM mode is Auto */	
    if ( ic->ic_opmode == IEEE80211_M_STA)
        ic->ic_cwm.cw_mode = IEEE80211_CWM_MODE2040;     

    /*
     * This will be used in linux client while processing 
     * country ie in 11d beacons
     */
    ic->ic_ignore_11dbeacon = 0;

    /*
     * commommode is used while determining the channel power
     * in standard client mode
     */ 
    ic->ic_commonmode = ic->ic_country.isMultidomain; 

    /*
     * initialize tx/rx engine
     */
    error = scn->sc_ops->tx_init(scn->sc_dev, ATH_TXBUF);
    if (error != 0)
        goto bad1;

    error = scn->sc_ops->rx_init(scn->sc_dev, ATH_RXBUF);
    if (error != 0)
        goto bad2;

    /*
     * setup net device
     */
    dev->open = ath_netdev_open;
    dev->stop = ath_netdev_stop;
    dev->hard_start_xmit = ath_netdev_hardstart;
    dev->set_mac_address = ath_netdev_set_macaddr;
    dev->tx_timeout = ath_netdev_tx_timeout;
    dev->set_multicast_list = ath_netdev_set_mcast_list;
    dev->do_ioctl = ath_ioctl;
    dev->get_stats = ath_getstats;
    dev->change_mtu = ath_change_mtu;
    dev->watchdog_timeo = 5 * HZ;           /* XXX */
    dev->tx_queue_len = ATH_TXBUF-1;        /* 1 for mgmt frame */
#ifdef USE_HEADERLEN_RESV
    dev->hard_header_len += sizeof (struct ieee80211_qosframe) + sizeof(struct llc) + IEEE80211_ADDR_LEN + IEEE80211_WEP_IVLEN + IEEE80211_WEP_KIDLEN;
#ifdef ATH_SUPERG_FF
    dev->hard_header_len += ATH_FF_MAX_HDR;
#endif
#endif
    /*
    ** Attach the iwpriv handlers
    */
    
    ath_iw_attach(dev);

    /*
     * setup interrupt serivce routine
     */
     
    ATH_INIT_TQUEUE(&osdev->intr_tq, ath_tasklet, dev);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
    if (request_irq(dev->irq, ath_isr, SA_SHIRQ, dev->name, dev)) {
#else
    if (request_irq(dev->irq, ath_isr, IRQF_SHARED, dev->name, dev)) {
#endif
        printk(KERN_WARNING "%s: request_irq failed\n", dev->name);
        error = -EIO;
        goto bad3;
    }

    /* Kernel 2.6.25 needs valid dev_addr before  register_netdev */
    IEEE80211_ADDR_COPY(dev->dev_addr,ic->ic_myaddr);

    /*
     * finally register netdev and ready to go
     */
    if ((error = register_netdev(dev)) != 0) {
        printk(KERN_ERR "%s: unable to register device\n", dev->name);
        goto bad4;
    }
#ifndef NO_SIMPLE_CONFIG
    /* Request Simple Config intr handler */
    register_simple_config_callback (dev->name, (void *) jumpstart_intr, (void *) dev);
#endif
    return 0;
    
bad4:
    free_irq(dev->irq, dev);
bad3:
    scn->sc_ops->rx_cleanup(scn->sc_dev);
bad2:
    scn->sc_ops->tx_cleanup(scn->sc_dev);
bad1:
    ath_detach(scn);
bad:
    return error;
}

int
__ath_detach(struct net_device *dev)
{
    struct ath_softc_net80211 *scn = dev->priv;

    if (dev->irq)
        free_irq(dev->irq, dev);
    
#ifndef NO_SIMPLE_CONFIG
    unregister_simple_config_callback(dev->name);
#endif
    unregister_netdev(dev);

    scn->sc_ops->rx_cleanup(scn->sc_dev);
    scn->sc_ops->tx_cleanup(scn->sc_dev);

    return ath_detach(scn);
}

int
__ath_suspend(struct net_device *dev)
{
    return ath_netdev_stop(dev);
}

int
__ath_resume(struct net_device *dev)
{
    return ath_netdev_open(dev);
}
#ifndef NO_SIMPLE_CONFIG
/*
 * Handler for front panel SW jumpstart switch
 */
static irqreturn_t
jumpstart_intr (int cpl, void *dev_id, struct pt_regs *regs)
{
    struct net_device *dev = dev_id;
    struct ath_softc_net80211 *scn = dev->priv;
    struct ieee80211com *ic = &scn->sc_ic;
    struct ieee80211vap *vap;

	/*
	** Iterate through all VAPs, since any of them may have WPS enabled
	*/

    vap = TAILQ_FIRST(&ic->ic_vaps);
	while ( vap != NULL )
    {
		printk("SC Pushbutton Notify on %s::%s\n",dev->name,vap->iv_dev->name);
	if (cpl == AP_PUSH_BUTTON) {
           ieee80211_notify_push_button (vap);
	} else if (cpl == STA_PUSH_BUTTON) {
           ieee80211_notify_sta_push_button (vap);
	}
        vap = TAILQ_NEXT(vap, iv_next);
    }
    return IRQ_HANDLED;
}
#endif
