/*! \file ah_osdep.c
**  \brief Linux support for HAL
**    
**  This file contains the supporting functions to operate the OS independant
**  HAL implementation in the Linux environment, the so-called "OS Shim" for the
**  HAL.
**
**  Copyright (c) 2002-2004 Sam Leffler, Errno Consulting.
**  Copyright (c) 2005-2007 Atheros Communications Inc.
**
**  All rights reserved.
**
** Redistribution and use in source and binary forms are permitted
** provided that the following conditions are met:
** 1. The materials contained herein are unmodified and are used
**    unmodified.
** 2. Redistributions of source code must retain the above copyright
**    notice, this list of conditions and the following NO
**    ''WARRANTY'' disclaimer below (''Disclaimer''), without
**    modification.
** 3. Redistributions in binary form must reproduce at minimum a
**    disclaimer similar to the Disclaimer below and any redistribution
**    must be conditioned upon including a substantially similar
**    Disclaimer requirement for further binary redistribution.
** 4. Neither the names of the above-listed copyright holders nor the
**    names of any contributors may be used to endorse or promote
**    product derived from this software without specific prior written
**    permission.
**
** NO WARRANTY
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT,
** MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
** IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE
** FOR SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
** USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
** OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
** OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
** SUCH DAMAGES.
**
*/

#include "opt_ah.h"

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#endif

/*
** Linux Includes
*/

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,25)
#include <linux/jiffies.h>
#endif
#include <asm/io.h>
#ifdef AR9100
#include <ar9100.h>
#endif

/*
** HAL Includes
*/
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)) && !defined(UINTPTR_T_IN_TYPES)
#define UINTPTR_T_IN_TYPES
#endif

#include "ah.h"
#include "ah_internal.h"
#include "ath_pktlog/pktlog_hal.h"

#ifndef __MOD_INC_USE_COUNT
#define AH_MOD_INC_USE_COUNT(_m)                    \
    if (!try_module_get(_m)) {                  \
        printk(KERN_WARNING "try_module_get failed\n");     \
        return NULL;                        \
    }
#define AH_MOD_DEC_USE_COUNT(_m)    module_put(_m)
#else
#define AH_MOD_INC_USE_COUNT(_m)    MOD_INC_USE_COUNT
#define AH_MOD_DEC_USE_COUNT(_m)    MOD_DEC_USE_COUNT
#endif


/*
** Internal static variables
** These are used to set GLOBAL HAL operating modes.  Any
** configuration variable that is instance specific SHOULD NOT
** be defined here
*/

static struct ath_hal_callback    ath_hal_callback_table;

#ifdef DEBUG_PKTLOG
struct ath_pktlog_halfuncs *g_pktlog_halfuncs;
EXPORT_SYMBOL(g_pktlog_halfuncs);
#endif

#ifdef AR9100
void
ath_hal_ahb_mac_reset(void)
{
    ar9100_ahb2wmac_reset();
}

void ath_hal_get_chip_revisionid(u_int32_t *val)
{
    ar9100_get_rst_revisionid(*val);
}
#endif

/******************************************************************************/
/*!
**  \brief Create a HAL instance
**
**  This function creates an instance of a HAL object for a specific device
**  type.  This is in essence the constructor function for the HAL.
**
**  \param devid 16 bit PCI Device ID value
**  \param sc Pointer to the ATH Soft C Object
**  \param bus_context Pointer to the PCI bus information structure.
**  \param callbackTable Pointer to ATH object callback table.
**  \param s Void pointer to area to pass Hal Status back to caller
**
**  \return Pointer to ath_hal object, HAL public object reference
**  \return NULL if unsuccessful
*/


struct ath_hal *
_ath_hal_attach(u_int16_t               devid,
                HAL_ADAPTER_HANDLE      osdev,
                HAL_SOFTC               sc,
                HAL_BUS_CONTEXT         *bus_context,
                struct hal_reg_parm     *hal_conf_parm,
                const struct ath_hal_callback *callbackTable,
                void                    *s)
{
    HAL_STATUS status;
    struct ath_hal *ah;

	/* 
	 * save callback table locally; 
         * can be made dynamically;
         * other handles are passed to the chip-specific routine.
	 */
	ath_hal_callback_table = *callbackTable;
	ah = ath_hal_attach(devid, osdev, sc, 
                        bus_context->bc_tag, bus_context->bc_handle, bus_context->bc_bustype,
                        hal_conf_parm, &status);

    *(HAL_STATUS *)s = status;
    if (ah)
        AH_MOD_INC_USE_COUNT(THIS_MODULE);
    return ah;
}

/******************************************************************************/
/*!
**  \brief Decrement the use count for this module
**
**  This decrements the module use count for each HAL instance that is detached.
**  Note that the ah reference is freed in the detach routine, so the ah
**  pointer is not valid on return.
**
**  \param ah Pointer to HAL instance structure
**
**  \return N/A
*/

void
ath_hal_detach(struct ath_hal *ah)
{
    (*ah->ah_detach)(ah);
    AH_MOD_DEC_USE_COUNT(THIS_MODULE);
}

/******************************************************************************/
/*!
**  \brief b
**
**  XXXX
**
**  \param P1 d
**
**  \return RV
*/

void __ahdecl
ath_hal_printf(struct ath_hal *ah, const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    ath_hal_vprintf(ah, fmt, ap);
    va_end(ap);
}
//EXPORT_SYMBOL(ath_hal_printf);


/******************************************************************************/
/*!
**  \brief b
**
**  XXXX
**
**  \param P1 d
**
**  \return RV
*/

/*
 * Print/log message support.
 */

void __ahdecl
ath_hal_vprintf(struct ath_hal *ah, const char* fmt, va_list ap)
{
    char buf[1024];                 /* XXX */
    vsnprintf(buf, sizeof(buf), fmt, ap);
    printk(KERN_INFO "%s", buf);
}

/******************************************************************************/
/*!
**  \brief b
**
**  XXXX
**
**  \param P1 d
**
**  \return RV
*/

/*
 * Format an Ethernet MAC for printing.
 */
const char* __ahdecl
ath_hal_ether_sprintf(const u_int8_t *mac)
{
    static char etherbuf[18];
    snprintf(etherbuf, sizeof(etherbuf), "%02x:%02x:%02x:%02x:%02x:%02x",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return etherbuf;
}

#ifdef AH_ASSERT
/******************************************************************************/
/*!
**  \brief b
**
**  XXXX
**
**  \param P1 d
**
**  \return RV
*/

void __ahdecl
ath_hal_assert_failed(const char* filename, int lineno, const char *msg)
{
    printk("Atheros HAL assertion failure: %s: line %u: %s\n",
        filename, lineno, msg);
    panic("ath_hal_assert");
}
#endif /* AH_ASSERT */

#ifdef AH_DEBUG_ALQ
/******************************************************************************/
/*!
**  \brief b
**
**  XXXX
**
**  \param P1 d
**
**  \return RV
*/

/*
 * ALQ register tracing support.
 *
 * Setting hw.ath.hal.alq=1 enables tracing of all register reads and
 * writes to the file /tmp/ath_hal.log.  The file format is a simple
 * fixed-size array of records.  When done logging set hw.ath.hal.alq=0
 * and then decode the file with the ardecode program (that is part of the
 * HAL).  If you start+stop tracing the data will be appended to an
 * existing file.
 *
 * NB: doesn't handle multiple devices properly; only one DEVICE record
 *     is emitted and the different devices are not identified.
 */
#include "alq/alq.h"
#include "ah_decode.h"

static  struct alq *ath_hal_alq;
static  int ath_hal_alq_emitdev;    /* need to emit DEVICE record */
static  u_int ath_hal_alq_lost;     /* count of lost records */
static  const char *ath_hal_logfile = "/tmp/ath_hal.log";
static  u_int ath_hal_alq_qsize = 8*1024;

static int
ath_hal_setlogging(int enable)
{
    int error;

    if (enable) {
        if (!capable(CAP_NET_ADMIN))
            return -EPERM;
        error = alq_open(&ath_hal_alq, ath_hal_logfile,
                sizeof (struct athregrec), ath_hal_alq_qsize);
        ath_hal_alq_lost = 0;
        ath_hal_alq_emitdev = 1;
        printk("ath_hal: logging to %s %s\n", ath_hal_logfile,
            error == 0 ? "enabled" : "could not be setup");
    } else {
        if (ath_hal_alq)
            alq_close(ath_hal_alq);
        ath_hal_alq = NULL;
        printk("ath_hal: logging disabled\n");
        error = 0;
    }
    return error;
}


/******************************************************************************/
/*!
**  \brief b
**
**  XXXX
**
**  \param P1 d
**
**  \return RV
*/

/*
 * Deal with the sysctl handler api changing.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
#define AH_SYSCTL_ARGS_DECL \
    ctl_table *ctl, int write, struct file *filp, void *buffer, \
        size_t *lenp
#define AH_SYSCTL_ARGS      ctl, write, filp, buffer, lenp
#else /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,8) */
#define AH_SYSCTL_ARGS_DECL \
    ctl_table *ctl, int write, struct file *filp, void *buffer,\
        size_t *lenp, loff_t *ppos
#define AH_SYSCTL_ARGS      ctl, write, filp, buffer, lenp, ppos
#endif

static int
sysctl_hw_ath_hal_log(AH_SYSCTL_ARGS_DECL)
{
    int error, enable;

    ctl->data = &enable;
    ctl->maxlen = sizeof(enable);
    enable = (ath_hal_alq != NULL);
        error = proc_dointvec(AH_SYSCTL_ARGS);
        if (error || !write)
                return error;
    else
        return ath_hal_setlogging(enable);
}

/******************************************************************************/
/*!
**  \brief b
**
**  XXXX
**
**  \param P1 d
**
**  \return RV
*/

static struct ale *
ath_hal_alq_get(struct ath_hal *ah)
{
    struct ale *ale;

    if (ath_hal_alq_emitdev) {
        ale = alq_get(ath_hal_alq, ALQ_NOWAIT);
        if (ale) {
            struct athregrec *r =
                (struct athregrec *) ale->ae_data;
            r->op = OP_DEVICE;
            r->reg = 0;
            r->val = ah->ah_devid;
            alq_post(ath_hal_alq, ale);
            ath_hal_alq_emitdev = 0;
        } else
            ath_hal_alq_lost++;
    }
    ale = alq_get(ath_hal_alq, ALQ_NOWAIT);
    if (!ale)
        ath_hal_alq_lost++;
    return ale;
}

/******************************************************************************/
/*!
**  \brief b
**
**  XXXX
**
**  \param P1 d
**
**  \return RV
*/

void __ahdecl
ath_hal_reg_write(struct ath_hal *ah, u_int32_t reg, u_int32_t val)
{
    if (ath_hal_alq) {
        unsigned long flags;
        struct ale *ale;

        local_irq_save(flags);
        ale = ath_hal_alq_get(ah);
        if (ale) {
            struct athregrec *r = (struct athregrec *) ale->ae_data;
            r->op = OP_WRITE;
            r->reg = reg;
            r->val = val;
            alq_post(ath_hal_alq, ale);
        }
        local_irq_restore(flags);
    }
    _OS_REG_WRITE(ah, reg, val);
}
EXPORT_SYMBOL(ath_hal_reg_write);

/******************************************************************************/
/*!
**  \brief b
**
**  XXXX
**
**  \param P1 d
**
**  \return RV
*/

u_int32_t __ahdecl
ath_hal_reg_read(struct ath_hal *ah, u_int32_t reg)
{
    u_int32_t val;

    val = _OS_REG_READ(ah, reg);
    if (ath_hal_alq) {
        unsigned long flags;
        struct ale *ale;

        local_irq_save(flags);
        ale = ath_hal_alq_get(ah);
        if (ale) {
            struct athregrec *r = (struct athregrec *) ale->ae_data;
            r->op = OP_READ;
            r->reg = reg;
            r->val = val;
            alq_post(ath_hal_alq, ale);
        }
        local_irq_restore(flags);
    }
    return val;
}
EXPORT_SYMBOL(ath_hal_reg_read);

/******************************************************************************/
/*!
**  \brief b
**
**  XXXX
**
**  \param P1 d
**
**  \return RV
*/

void __ahdecl
OS_MARK(struct ath_hal *ah, u_int id, u_int32_t v)
{
    if (ath_hal_alq) {
        unsigned long flags;
        struct ale *ale;

        local_irq_save(flags);
        ale = ath_hal_alq_get(ah);
        if (ale) {
            struct athregrec *r = (struct athregrec *) ale->ae_data;
            r->op = OP_MARK;
            r->reg = id;
            r->val = v;
            alq_post(ath_hal_alq, ale);
        }
        local_irq_restore(flags);
    }
}
EXPORT_SYMBOL(OS_MARK);

#elif defined(AH_DEBUG) || defined(AH_REGOPS_FUNC)
/******************************************************************************/
/*!
**  \brief b
**
**  XXXX
**
**  \param P1 d
**
**  \return RV
*/

/*
 * Memory-mapped device register read/write.  These are here
 * as routines when debugging support is enabled and/or when
 * explicitly configured to use function calls.  The latter is
 * for architectures that might need to do something before
 * referencing memory (e.g. remap an i/o window).
 *
 * NB: see the comments in ah_osdep.h about byte-swapping register
 *     reads and writes to understand what's going on below.
 */
void __ahdecl
ath_hal_reg_write(struct ath_hal *ah, u_int reg, u_int32_t val)
{
    HDPRINTF(ah, HAL_DBG_REG_IO, "WRITE 0x%x <= 0x%x\n", reg, val);
    _OS_REG_WRITE(ah, reg, val);
}
EXPORT_SYMBOL(ath_hal_reg_write);

/******************************************************************************/
/*!
**  \brief b
**
**  XXXX
**
**  \param P1 d
**
**  \return RV
*/

u_int32_t __ahdecl
ath_hal_reg_read(struct ath_hal *ah, u_int reg)
{
    u_int32_t val;
    
    val = _OS_REG_READ(ah, reg);
    HDPRINTF(ah, HAL_DBG_REG_IO, "READ 0x%x => 0x%x\n", reg, val);
    return val;
}
EXPORT_SYMBOL(ath_hal_reg_read);
#endif /* AH_DEBUG || AH_REGOPS_FUNC */

#ifdef AH_DEBUG

/******************************************************************************/
/*!
**  \brief HAL Debug Print interface
**
**  This function will selectively print debug information to the console as
**  enabled by the user.  This function is completely compiled out if AH_DEBUG
**  is not defined.  The parameter ah_config.ah_hal_debug is used to select
**  specific debug information as required.
**
**  \param ah Pointer to AH object containing the debug flag.  This is instance
**            specific.
**  \param dmask Debug mask of the print.  Defines selection level
**  \param fmt Standard "C" format string defining the output format
**  \param ... Variable list of arguments that are used by the format.
**
**  \return N/A
*/

void __ahdecl
HDPRINTF(struct ath_hal *ah, u_int dmask, const char* fmt, ...)
{
    if ( ( ah == AH_NULL && dmask == HAL_DBG_UNMASKABLE) ||
         ( AH_PRIVATE(ah)->ah_config.ath_hal_debug & dmask) )
    {
       __va_list ap;
       va_start(ap, fmt);
       ath_hal_vprintf(ah, fmt, ap);
       va_end(ap);
    }
}
#endif /* AH_DEBUG */

/******************************************************************************/
/*!
**  \brief b
**
**  XXXX
**
**  \param P1 d
**
**  \return RV
*/

/*
 * Delay n microseconds.
 */
void __ahdecl
ath_hal_delay(int n)
{
    udelay(n);
}

/******************************************************************************/
/*!
**  \brief b
**
**  XXXX
**
**  \param P1 d
**
**  \return RV
*/

void* __ahdecl
ath_hal_ioremap(uintptr_t addr, u_int32_t len)
{
    return (ioremap(addr, len));
}

/******************************************************************************/
/*!
**  \brief b
**
**  XXXX
**
**  \param P1 d
**
**  \return RV
*/

u_int32_t __ahdecl
ath_hal_getuptime(struct ath_hal *ah)
{
    return ((jiffies / HZ) * 1000) + (jiffies % HZ) * (1000 / HZ);
}
EXPORT_SYMBOL(ath_hal_getuptime);

/******************************************************************************/
/*!
**  \brief b
**
**  XXXX
**
**  \param P1 d
**
**  \return RV
*/

/*
 * Allocate/free memory.
 */

void * __ahdecl
ath_hal_malloc(HAL_ADAPTER_HANDLE osdev, size_t size)
{
    void *p;
    p = kmalloc(size, GFP_KERNEL);
    if (p)
        OS_MEMZERO(p, size);
    return p;

}

/******************************************************************************/
/*!
**  \brief b
**
**  XXXX
**
**  \param P1 d
**
**  \return RV
*/

void __ahdecl
ath_hal_free(void* p)
{
    kfree(p);
}

/******************************************************************************/
/*!
**  \brief b
**
**  XXXX
**
**  \param P1 d
**
**  \return RV
*/

u_int32_t
ath_hal_read_pci_config_space(struct ath_hal *ah, u_int32_t offset, void *pBuffer, u_int32_t length)
{
    u_int32_t bytesRead = 0;

    if (ath_hal_callback_table.read_pci_config_space != NULL) {
        bytesRead = ath_hal_callback_table.read_pci_config_space(ah->ah_sc, offset, pBuffer, length);
    }

    return (bytesRead);
}

/******************************************************************************/
/*!
**  \brief b
**
**  XXXX
**
**  \param P1 d
**
**  \return RV
*/

void __ahdecl
ath_hal_memzero(void *dst, size_t n)
{
    memset(dst, 0, n);
}
EXPORT_SYMBOL(ath_hal_memzero);

/******************************************************************************/
/*!
**  \brief b
**
**  XXXX
**
**  \param P1 d
**
**  \return RV
*/

void * __ahdecl
ath_hal_memcpy(void *dst, const void *src, size_t n)
{
    return memcpy(dst, src, n);
}
EXPORT_SYMBOL(ath_hal_memcpy);

/******************************************************************************/
/*!
**  \brief Set Factory Defaults for the HAL instance
**
**  This function will set factory default values for the HAL operations
**  configuration structure, used in the operation of each HAL instance.
**  Specific values can be overriden using IOCTL interfaces, as required.
**
**  \param ahp  Pointer to HAL private data structure
**
**  \return N/A
*/

void __ahdecl
ath_hal_factory_defaults(struct ath_hal_private *ap, struct hal_reg_parm *hal_conf_parm)
{
    int i;
    ap->ah_config.ath_hal_dma_beacon_response_time  = 2;
    ap->ah_config.ath_hal_sw_beacon_response_time   = 10;
    ap->ah_config.ath_hal_additional_swba_backoff   = 0;
    ap->ah_config.ath_hal_6mb_ack                   = 0x0;
    ap->ah_config.ath_hal_cwmIgnoreExtCCA           = 0;
#ifdef ATH_FORCE_BIAS
    ap->ah_config.ath_hal_forceBias                 = 0;
    ap->ah_config.ath_hal_forceBiasAuto             = 0;
#endif
    ap->ah_config.ath_hal_pciePowerSaveEnable       = 0;
    ap->ah_config.ath_hal_pcieL1SKPEnable           = 0;
    ap->ah_config.ath_hal_pcieClockReq              = 0;
    ap->ah_config.ath_hal_pciePowerReset            = 0x100;
    ap->ah_config.ath_hal_pcieRestore               = 0;
    ap->ah_config.ath_hal_pcieWaen                  = 0;
    ap->ah_config.ath_hal_analogShiftRegWAR         = 1;  /* enable analog shift reg WAR by default */
    ap->ah_config.ath_hal_htEnable                  = 1;
#ifdef ATH_SUPERG_DYNTURBO
    ap->ah_config.ath_hal_disableTurboG             = 2;
#endif
    ap->ah_config.ath_hal_ofdmTrigLow               = 200;
    ap->ah_config.ath_hal_ofdmTrigHigh              = 500;
    ap->ah_config.ath_hal_cckTrigHigh               = 200;
    ap->ah_config.ath_hal_cckTrigLow                = 100;
    ap->ah_config.ath_hal_enableANI                 = 1;
    ap->ah_config.ath_hal_noiseImmunityLvl          = 4;
    ap->ah_config.ath_hal_ofdmWeakSigDet            = 1;
    ap->ah_config.ath_hal_cckWeakSigThr             = 0;
    ap->ah_config.ath_hal_spurImmunityLvl           = 2;
    ap->ah_config.ath_hal_firStepLvl                = 0;
    ap->ah_config.ath_hal_rssiThrHigh               = 40;
    ap->ah_config.ath_hal_rssiThrLow                = 7;
    ap->ah_config.ath_hal_diversityControl          = 0;
    ap->ah_config.ath_hal_antennaSwitchSwap         = 0;
    ap->ah_config.ath_hal_rifs_enable         	    = 0;
    
    for (i=0; i< AR_EEPROM_MODAL_SPURS; i++) {
    	ap->ah_config.ath_hal_spurChans[i][0] = AR_NO_SPUR;
	ap->ah_config.ath_hal_spurChans[i][1] = AR_NO_SPUR;
    }

#ifdef AR5416_INT_MITIGATION
    /* 
     * For Linux OS, if this compile flag is set, then we enable the
     * interrupt mitigation hardware feature.
     */
#ifdef ATH_SUPPORT_UAPSD
    ap->ah_config.ath_hal_intrMitigationRx            = 0;
#else
    ap->ah_config.ath_hal_intrMitigationRx            = 1;
#endif
    ap->ah_config.ath_hal_intrMitigationTx            = 0;
#endif

    ap->ah_config.ath_hal_fastClockEnable           = 1;

#ifdef AH_DEBUG
    ap->ah_config.ath_hal_debug                     = 0;  /* off by default */
#endif
    ap->ah_config.ath_hal_mfpSupport                = HAL_MFP_QOSDATA;
}
EXPORT_SYMBOL(ath_hal_factory_defaults);

/*
 * Module glue.
 */
static char *dev_info = "ath_hal";

MODULE_AUTHOR("Errno Consulting, Sam Leffler");
MODULE_DESCRIPTION("Atheros Hardware Access Layer (HAL)");
MODULE_SUPPORTED_DEVICE("Atheros WLAN devices");
#ifdef MODULE_LICENSE
MODULE_LICENSE("Proprietary");
#endif

EXPORT_SYMBOL(ath_hal_probe);
EXPORT_SYMBOL(_ath_hal_attach);
EXPORT_SYMBOL(ath_hal_detach);
EXPORT_SYMBOL(ath_hal_init_channels);
EXPORT_SYMBOL(ath_hal_getwirelessmodes);
EXPORT_SYMBOL(ath_hal_computetxtime);
EXPORT_SYMBOL(ath_hal_mhz2ieee);
EXPORT_SYMBOL(ath_hal_enabledANI);
EXPORT_SYMBOL(ath_hal_getChanNoise);
EXPORT_SYMBOL(ath_hal_getCurrentCountry);
EXPORT_SYMBOL(findCountryCode);
EXPORT_SYMBOL(ath_hal_get_config_param);
EXPORT_SYMBOL(ath_hal_set_config_param);
#ifdef ATH_CCX
EXPORT_SYMBOL(ath_hal_get_sernum);
EXPORT_SYMBOL(ath_hal_get_chandata);
#endif
EXPORT_SYMBOL(getCommonPower);

/******************************************************************************/
/*!
**  \brief Load and Initialize HAL "layer"
**
**  This function installs the HAL module into memory, and performs minimal
**  initialization.  The main purpose of this initialization is to make all
**  of the HAL symbols accessible by other modules, as required.  This init
**  function DOES NOT allocate any HAL instances.
**
**  \return N/A, This is a module initialization function
*/

static int __init
init_ath_hal(void)
{
    const char *sep;
    int i;

    printk(KERN_INFO "%s: %s (", dev_info, ath_hal_version);
    sep = "";
    for (i = 0; ath_hal_buildopts[i] != NULL; i++) {
        printk("%s%s", sep, ath_hal_buildopts[i]);
        sep = ", ";
    }
    printk(")\n");
    
    return (0);
}
module_init(init_ath_hal);

/******************************************************************************/
/*!
**  \brief b
**
**  XXXX
**
**  \param P1 d
**
**  \return RV
*/

static void __exit
exit_ath_hal(void)
{
    printk(KERN_INFO "%s: driver unloaded\n", dev_info);
}
module_exit(exit_ath_hal);
