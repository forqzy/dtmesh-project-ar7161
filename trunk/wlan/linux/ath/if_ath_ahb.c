/*-
 * Copyright (c) 2004 Atheros Communications, Inc.
 * All rights reserved
 */

/*
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*/

#ifndef EXPORT_SYMTAB
#define	EXPORT_SYMTAB
#endif

#include <osdep.h>

#include "if_athvar.h"
#include "ah_devid.h"
#include "if_ath_ahb.h"

struct ath_ahb_softc {
	struct ath_softc_net80211   aps_sc;
	struct _NIC_DEV             aps_osdev;
#ifdef CONFIG_PM
	u32			aps_pmstate[16];
#endif
};

static struct ath_ahb_softc *sclist[2] = {NULL, NULL};
static u_int8_t num_activesc=0;

static struct ar531x_boarddata *ar5312_boardConfig=NULL;
static char *radioConfig=NULL;

extern int __ath_attach(u_int16_t devid, struct net_device *dev, HAL_BUS_CONTEXT *bus_context, osdev_t osdev);
extern int __ath_detach(struct net_device *dev);

static int
ar5312_get_board_config(void)
{
	int dataFound;
	char *bd_config;
	
	/*
	 * Find start of Board Configuration data, using heuristics:
	 * Search back from the (aliased) end of flash by 0x1000 bytes
	 * at a time until we find the string "5311", which marks the
	 * start of Board Configuration.  Give up if we've searched
	 * more than 500KB.
	 */
	dataFound = 0;
	for (bd_config = (char *)0xbffff000;
	     bd_config > (char *)0xbff80000;
	     bd_config -= 0x1000) {
		if ( *(int *)bd_config == AR531X_BD_MAGIC) {
			dataFound = 1;
			break;
		}
	}
	
	if (!dataFound) {
		printk("Could not find Board Configuration Data\n");
		bd_config = NULL;
	}
	ar5312_boardConfig = (struct ar531x_boarddata *) bd_config;
	return(dataFound);
}

static int
ar5312_get_radio_config(void)
{
	int dataFound;
	char *radio_config;
	
	/* 
	 * Now find the start of Radio Configuration data, using heuristics:
	 * Search forward from Board Configuration data by 0x1000 bytes
	 * at a time until we find non-0xffffffff.
	 */
	dataFound = 0;
	for (radio_config = ((char *) ar5312_boardConfig) + 0x1000;
	     radio_config < (char *)0xbffff000;
	     radio_config += 0x1000) {
		if (*(int *)radio_config != 0xffffffff) {
			dataFound = 1;
			break;
		}
	}

	if (!dataFound) { /* AR2316 relocates radio config to new location */
	    dataFound = 0;
	    for (radio_config = ((char *) ar5312_boardConfig) + 0xf8;
		 radio_config < (char *)0xbffff0f8;
		 radio_config += 0x1000) {
		if (*(int *)radio_config != 0xffffffff) {
		    dataFound = 1;
		    break;
		}
	    }
	}

	if (!dataFound) {
		printk("Could not find Radio Configuration data\n");
		radio_config = NULL;
	}
	radioConfig = radio_config;
	return(dataFound);
}

static int
ar5312SetupFlash(void)
{
	if (ar5312_get_board_config()) {
		if (ar5312_get_radio_config()) {
			return(1);
		}
	}
	return(0);
}	

/*
 * Read 16 bits of data from offset into *data
 */

static void
ar5312BspEepromRead(u_int32_t off, u_int32_t nbytes, u_int8_t *data)
{
	int i;
	char *eepromAddr = radioConfig;
	
	for (i=0; i<nbytes; i++,off++) {
		data[i] = eepromAddr[off];
	}
}

int ahb_enable_wmac(u_int16_t devid,u_int16_t wlanNum)
{

	if((devid & AR5315_REV_MAJ_M) == AR5315_REV_MAJ) {
		u_int32_t reg;
		KASSERT(wlanNum == 0,("invalid wlan # %d",wlanNum) ); 
		/* Enable Arbitration for WLAN */
		u_int32_t *en = (u_int32_t *) AR5315_AHB_ARB_CTL;
		*en  |= AR5315_ARB_WLAN;

		/* Enable global swapping so this looks like a normal BE system */
		reg = REG_READ(AR5315_ENDIAN_CTL);
		reg |= AR5315_CONFIG_WLAN;
		REG_WRITE(AR5315_ENDIAN_CTL, reg);

		/* wake up the MAC */
		/* NOTE: for the following write to succeed the
		   RST_AHB_ARB_CTL should be set to 0. This driver
		   asuumes that the register has been set to 0 by boot loader */ 
		reg = REG_READ(AR5315_PCI_MAC_SCR);
		reg= (reg & ~AR5315_PCI_MAC_SCR_SLMODE_M) | 
			(AR5315_PCI_MAC_SCR_SLM_FWAKE << AR5315_PCI_MAC_SCR_SLMODE_S);
		REG_WRITE(AR5315_PCI_MAC_SCR, reg);

		/* wait for the MAC to wakeup */
		while (REG_READ(AR5315_PCI_MAC_PCICFG) & AR5315_PCI_MAC_PCICFG_SPWR_DN);

  } else {

        u_int32_t *en = (u_int32_t *)AR531X_ENABLE;
        switch (wlanNum) {
        case AR531X_WLAN0_NUM:
                *en |= AR531X_ENABLE_WLAN0;
                break;
        case AR531X_WLAN1_NUM:
                *en |= AR531X_ENABLE_WLAN1;
                break;
        default:
                return -ENODEV;
        }
  }
        return 0;
}

#ifdef when_ar531x_is_implemented
int ahb_disable_wmac(u_int16_t devid, u_int16_t wlanNum)
{
	if((devid & AR5315_REV_MAJ_M) == AR5315_REV_MAJ) {
		KASSERT(wlanNum == 0,("invalid wlan # %d",wlanNum) ); 
		/* Enable Arbitration for WLAN */
		u_int32_t *en = (u_int32_t *) AR5315_AHB_ARB_CTL;
		*en  &= ~AR5315_ARB_WLAN;
	}
	else { 
		u_int32_t *en = (u_int32_t *)AR531X_ENABLE;
		switch (wlanNum) {
		case AR531X_WLAN0_NUM:
			*en &= ~AR531X_ENABLE_WLAN0;
			break;
		case AR531X_WLAN1_NUM:
			*en &= ~AR531X_ENABLE_WLAN1;
			break;
		default:
			return -ENODEV;
		}
	}
	return 0;
}


int exit_ath_wmac(u_int16_t wlanNum)
{
	struct ath_ahb_softc *sc = sclist[wlanNum];
	struct net_device *dev;
	const char *sysType;
	u_int16_t devid;

        if(sc == NULL)
                return -ENODEV; /* XXX: correct return value? */

        dev = sc->aps_osdev.sc_dev;
	__ath_detach(dev);
	sysType = get_system_type();
	if(strcmp(sysType,"Atheros AR5315") == 0) {
		devid = (u_int16_t) (sysRegRead(AR5315_SREV) & (AR5315_REV_MAJ_M | AR5315_REV_MIN_M));
	} else {
		devid = ((u_int16_t) ((sysRegRead(AR531X_REV) >>8) &
							  (AR531X_REV_MAJ | AR531X_REV_MIN)));
	}

	ahb_disable_wmac(devid,wlanNum);
	free_netdev(dev);
        sclist[wlanNum]=NULL;
        return 0;
}

int init_ath_wmac(u_int16_t devid, u_int16_t wlanNum)
{
        const char *athname;
        struct net_device *dev;
        struct ath_ahb_softc *sc;
        HAL_BUS_CONTEXT bus_context;

        if (((wlanNum != 0) && (wlanNum != 1)) ||
            (sclist[wlanNum] != NULL)) {
                goto bad;
        }

	ahb_enable_wmac(devid,wlanNum);

	dev = alloc_netdev(sizeof(struct ath_ahb_softc), "wifi%d", ether_setup);
	if (dev == NULL) {
		printk(KERN_ERR "ath_dev_probe: no memory for device state\n");
		goto bad2;
	}
	sc = dev->priv;
	sc->aps_osdev.netdev = dev;

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,5,41)
	dev->owner = THIS_MODULE;
#else
	SET_MODULE_OWNER(dev);
#endif
        sclist[wlanNum] = sc;

        switch (wlanNum) {
        case AR531X_WLAN0_NUM:
	        if((devid & AR5315_REV_MAJ_M) == AR5315_REV_MAJ) {
                    dev->irq = AR5315_IRQ_WLAN0_INTRS;
                    dev->mem_start = AR5315_WLAN0;
                } else {
		    dev->irq = AR531X_IRQ_WLAN0_INTRS;
                    dev->mem_start = AR531X_WLAN0;
		}
                break;
        case AR531X_WLAN1_NUM:
                dev->irq = AR531X_IRQ_WLAN1_INTRS;
                dev->mem_start = KSEG1ADDR(AR531X_WLAN1);
                break;
        default:
                goto bad3;
        }
        dev->mem_end = dev->mem_start + AR531X_WLANX_LEN;
        sc->aps_osdev.bdev = (void *) NULL;

        bus_context.bc_tag = NULL;
        bus_context.bc_handle = (void *)dev->mem_start;
        bus_context.bc_bustype = HAL_BUS_TYPE_AHB;

        if (__ath_attach(devid, dev, &bus_context, &sc->aps_osdev) != 0)
                goto bad3;
        athname = ath_hal_probe(ATHEROS_VENDOR_ID, devid);
        printk(KERN_INFO "%s: %s: mem=0x%lx, irq=%d\n",
               dev->name, athname ? athname : "Atheros ???", dev->mem_start, dev->irq);
        num_activesc++;
        return 0;

 bad3:
        free_netdev(dev);
        sclist[wlanNum]=NULL;
 bad2:
        ahb_disable_wmac(devid,wlanNum);
 bad:
        return -ENODEV;
}
#endif /* when_ar531x_is_implemented */

static int
init_ath_wmac(u_int16_t devid, u_int16_t wlanNum)
{
	const char		*athname;
	struct net_device	*dev;
	struct ath_ahb_softc	*sc;
        HAL_BUS_CONTEXT		bc;

	if (!valid_wmac_num(wlanNum) || (sclist[wlanNum] != NULL)) {
		return -ENODEV;
	}

	dev = alloc_netdev(sizeof(struct ath_ahb_softc), "wifi%d", ether_setup);
	if (dev == NULL) {
		printk(KERN_ERR "%s: no memory for device state\n", __func__);
		goto bad1;
	}
	sc = dev->priv;
	sc->aps_osdev.netdev = dev;

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,5,41)
	dev->owner = THIS_MODULE;
#else
	SET_MODULE_OWNER(dev);
#endif
	sclist[wlanNum] = sc;

	dev->irq = get_wmac_irq(wlanNum);
	dev->mem_start = get_wmac_base(wlanNum);
	dev->mem_end = dev->mem_start + get_wmac_mem_len(wlanNum);
        sc->aps_osdev.bdev = NULL;

        bc.bc_tag = NULL;
        bc.bc_handle = (HAL_BUS_HANDLE)dev->mem_start;
        bc.bc_bustype = HAL_BUS_TYPE_AHB;

        if (__ath_attach(devid, dev, &bc, &sc->aps_osdev) != 0) {
		goto bad2;
	}

	athname = ath_hal_probe(ATHEROS_VENDOR_ID, devid);

	enable_wmac_led();

	printk(KERN_INFO "%s: %s: mem=0x%lx, irq=%d\n",
		dev->name, athname ? athname : "Atheros ???",
		dev->mem_start, dev->irq);

	num_activesc++;

	return 0;
bad2:
	free_netdev(dev);
bad1:
	sclist[wlanNum]=NULL;
	return -ENODEV;
}

static int
exit_ath_wmac(u_int16_t wlanNum)
{
	struct ath_ahb_softc *sc = sclist[wlanNum];
	struct net_device *dev;

	if(sc == NULL) {
		return -ENODEV; /* XXX: correct return value? */
	}

        dev = sc->aps_osdev.netdev;
	if(dev == NULL) {
		printk("%s: No device\n", __func__);
		return -ENODEV;
	}
	__ath_detach(dev);
	free_netdev(dev);
	disable_wmac_led();
	sclist[wlanNum]=NULL;
	return 0;
}

int init_ahb(void)
{
	int ret;
	u_int16_t devid,radioMask;
	const char *sysType;
	sysType = get_system_type();
#ifdef AR9100
	if(strcmp(sysType,"Atheros AR9100") == 0) {
		devid = (ar7100_reg_rd(AR7100_CHIP_REV) &
				AR7100_CHIP_REV_MAJOR_M) >>
				AR7100_CHIP_REV_MAJOR_S;
		return init_ath_wmac(devid, 0);
	} else
#endif
	if(strcmp(sysType,"Atheros AR5315") == 0) {
		devid = (u_int16_t) (sysRegRead(AR5315_SREV) & (AR5315_REV_MAJ_M | AR5315_REV_MIN_M));
		if((devid & AR5315_REV_MAJ_M) == AR5315_REV_MAJ)
			return init_ath_wmac(devid,0);
	}

	/* Probe to find out the silicon revision and enable the
	   correct number of macs */
	if (!ar5312SetupFlash())
	  return -ENODEV;
        devid = ((u_int16_t) ((sysRegRead(AR531X_REV) >>8) & 
                            (AR531X_REV_MAJ | AR531X_REV_MIN)));
        switch (devid) {
        case AR5212_AR5312_REV2:
        case AR5212_AR5312_REV7:
	  /* Need to determine if we have a 5312 or a 2312 since they
	     have the same Silicon Rev ID*/
		ar5312BspEepromRead(2*AR531X_RADIO_MASK_OFF,2,
				    (char *) &radioMask);
		if ((radioMask & AR531X_RADIO0_MASK) != 0) {
		  if((ret=init_ath_wmac(devid,0))!=0)
		    return ret;
		}
        case AR5212_AR2313_REV8:
                if((ret=init_ath_wmac(devid,1))!=0)
                        return ret;
                break;
        default:
                return -ENODEV;
        }
        return 0;
}


/*
 * Module glue.
 */
#include "version.h"
static char *version = ATH_PCI_VERSION " (Atheros/multi-bss)";
static char *dev_info = "ath_ahb";

#include <linux/ethtool.h>

int
ath_ioctl_ethtool(struct ath_softc *sc, int cmd, void *addr)
{
	struct ethtool_drvinfo info;

	if (cmd != ETHTOOL_GDRVINFO)
		return -EOPNOTSUPP;
	memset(&info, 0, sizeof(info));
	info.cmd = cmd;
	strncpy(info.driver, dev_info, sizeof(info.driver)-1);
	strncpy(info.version, version, sizeof(info.version)-1);
	return copy_to_user(addr, &info, sizeof(info)) ? -EFAULT : 0;
}

MODULE_AUTHOR("Atheros Communications, Inc.");
MODULE_DESCRIPTION("Support for Atheros 802.11 wireless LAN cards.");
MODULE_SUPPORTED_DEVICE("Atheros WLAN cards");
#ifdef MODULE_LICENSE
MODULE_LICENSE("BSD");
#endif

static int __init
init_ath_ahb(void)
{
	printk(KERN_INFO "%s: %s\n", dev_info, version);

	if (init_ahb() != 0) {
		printk("ath_ahb: No devices found, driver not installed.\n");
		return (-ENODEV);
	}
#if defined(CONFIG_SYSCTL) && !defined(AR9100)
	ath_sysctl_register();
#endif
	return (0);
}
module_init(init_ath_ahb);

static void __exit
exit_ath_ahb(void)
{
#if defined(CONFIG_SYSCTL) && !defined(AR9100)
	ath_sysctl_unregister();
#endif

#ifdef AR9100
	exit_ath_wmac(0);
#else
	exit_ath_wmac(AR531X_WLAN0_NUM);
	exit_ath_wmac(AR531X_WLAN1_NUM);
#endif

	printk(KERN_INFO "%s: driver unloaded\n", dev_info);
}
module_exit(exit_ath_ahb);
