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
 * $Id: //depot/sw/releases/7.3_AP/wlan/common/lmac/ath_pktlog/linux.c#4 $
 */
#ifndef REMOVE_PKT_LOG

#ifndef EXPORT_SYMTAB
#define	EXPORT_SYMTAB
#endif

/*
 * Atheros packet log module (Linux-specific code)
 */
#include <osdep.h>

#include <linux/proc_fs.h>
#include <linux/vmalloc.h>
#include <linux/highmem.h>


#include "pktlog_i.h"
#include "pktlog_rc.h"
#include "pktlog_hal.h"
#include "if_llc.h"

#include "if_athvar.h"

#ifndef NULL
#define NULL 0
#endif

#ifndef __MOD_INC_USE_COUNT
#define	PKTLOG_MOD_INC_USE_COUNT					\
	if (!try_module_get(THIS_MODULE)) {					\
		printk(KERN_WARNING "try_module_get failed\n");		\
	}

#define	PKTLOG_MOD_DEC_USE_COUNT	module_put(THIS_MODULE)
#else
#define	PKTLOG_MOD_INC_USE_COUNT	MOD_INC_USE_COUNT
#define	PKTLOG_MOD_DEC_USE_COUNT	MOD_DEC_USE_COUNT
#endif

#define PKTLOG_SYSCTL_SIZE      8

/* Permissions for creating proc entries */
#define PKTLOG_PROC_PERM        0444
#define PKTLOG_PROCSYS_DIR_PERM 0555
#define PKTLOG_PROCSYS_PERM     0644

#define PKTLOG_DEVNAME_SIZE 32

#define PKTLOG_PROC_DIR "ath_pktlog"
#define PKTLOG_PROC_SYSTEM "system"

#define WLANDEV_BASENAME "wifi"
#define MAX_WLANDEV 10

#define PKTLOG_TAG "ATH_PKTLOG" 

/*
 * Linux specific pktlog state information
 */
struct ath_pktlog_info_lnx {
    struct ath_pktlog_info info;
    struct ctl_table sysctls[PKTLOG_SYSCTL_SIZE];
    struct proc_dir_entry *proc_entry;
    struct ctl_table_header *sysctl_header;
};

#define PL_INFO_LNX(_pl_info)   ((struct ath_pktlog_info_lnx *)(_pl_info))

static struct proc_dir_entry *g_pktlog_pde;

static int pktlog_attach(struct ath_softc *sc);
static void pktlog_detach(struct ath_softc *sc);
static int pktlog_open(struct inode *i, struct file *f);
static int pktlog_release(struct inode *i, struct file *f);
static int pktlog_mmap(struct file *f, struct vm_area_struct *vma);
static ssize_t pktlog_read(struct file *file, char *buf, size_t nbytes,
                           loff_t * ppos);

static struct ath_pktlog_funcs g_exported_pktlog_funcs = {
    .pktlog_attach = pktlog_attach,
    .pktlog_detach = pktlog_detach,
    .pktlog_txctl = pktlog_txctl,
    .pktlog_txstatus = pktlog_txstatus,
    .pktlog_rx = pktlog_rx,
    .pktlog_text = pktlog_text,
    .pktlog_start = pktlog_start,
    .pktlog_readhdr = pktlog_read_hdr,
    .pktlog_readbuf = pktlog_read_buf,
};

static struct ath_pktlog_halfuncs g_exported_pktlog_halfuncs = {
    .pktlog_ani = pktlog_ani
};

static struct ath_pktlog_rcfuncs g_exported_pktlog_rcfuncs = {
    .pktlog_rcupdate = pktlog_rcupdate,
    .pktlog_rcfind = pktlog_rcfind
};

static struct file_operations pktlog_fops = {
    open:pktlog_open,
    release:pktlog_release,
    mmap:pktlog_mmap,
    read:pktlog_read,
};

extern struct ath_pktlog_info *g_pktlog_info;

static int
ATH_SYSCTL_DECL(ath_sysctl_pktlog_enable, ctl, write, filp, buffer, lenp,
                ppos)
{
    int ret, enable;
    struct ath_softc *sc = (struct ath_softc *) ctl->extra1;

    ctl->data = &enable;
    ctl->maxlen = sizeof(enable);

    if (write) {
        ret = ATH_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
                                       lenp, ppos);
        if (ret == 0)
            return pktlog_enable(sc, enable);
        else
            printk(PKTLOG_TAG "%s:proc_dointvec failed\n", __FUNCTION__);
    } else {
        enable = get_pktlog_state(sc);
        ret = ATH_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
                                       lenp, ppos);
        if (ret)
            printk(PKTLOG_TAG "%s:proc_dointvec failed\n", __FUNCTION__);
    }
    return ret;
}

static int
ATH_SYSCTL_DECL(ath_sysctl_pktlog_size, ctl, write, filp, buffer, lenp,
                ppos)
{
    int ret, size;
    struct ath_softc *sc = (struct ath_softc *) ctl->extra1;

    ctl->data = &size;
    ctl->maxlen = sizeof(size);

    if (write) {
        ret = ATH_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
                                       lenp, ppos);
        if (ret == 0)
            return pktlog_setsize(sc, size);
    } else {
        size = get_pktlog_bufsize(sc);
        ret = ATH_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
                                       lenp, ppos);
    }
    return ret;
}


static int
pktlog_sysctl_register(struct ath_softc *sc)
{
    struct ath_pktlog_info_lnx *pl_info_lnx;
    char *proc_name;

    if (sc) {
        pl_info_lnx = PL_INFO_LNX(sc->pl_info);
        proc_name = sc->sc_osdev->netdev->name;
    } else {
        pl_info_lnx = PL_INFO_LNX(g_pktlog_info);
        proc_name = PKTLOG_PROC_SYSTEM;
    }

    /*
     * Setup the sysctl table for creating the following sysctl entries:
     * /proc/sys/PKTLOG_PROC_DIR/<adapter>/enable for enabling/disabling pktlog
     * /proc/sys/PKTLOG_PROC_DIR/<adapter>/size for changing the buffer size
     */
    memset(pl_info_lnx->sysctls, 0, sizeof(pl_info_lnx->sysctls));
    pl_info_lnx->sysctls[0].ctl_name = CTL_AUTO;
    pl_info_lnx->sysctls[0].procname = PKTLOG_PROC_DIR;
    pl_info_lnx->sysctls[0].mode = PKTLOG_PROCSYS_DIR_PERM;
    pl_info_lnx->sysctls[0].child = &pl_info_lnx->sysctls[2];
    /* [1] is NULL terminator */
    pl_info_lnx->sysctls[2].ctl_name = CTL_AUTO;
    pl_info_lnx->sysctls[2].procname = proc_name;
    pl_info_lnx->sysctls[2].mode = PKTLOG_PROCSYS_DIR_PERM;
    pl_info_lnx->sysctls[2].child = &pl_info_lnx->sysctls[4];
    /* [3] is NULL terminator */
    pl_info_lnx->sysctls[4].ctl_name = CTL_AUTO;
    pl_info_lnx->sysctls[4].procname = "enable";
    pl_info_lnx->sysctls[4].mode = PKTLOG_PROCSYS_PERM;
    pl_info_lnx->sysctls[4].proc_handler = ath_sysctl_pktlog_enable;
    pl_info_lnx->sysctls[4].extra1 = sc;

    pl_info_lnx->sysctls[5].ctl_name = CTL_AUTO;
    pl_info_lnx->sysctls[5].procname = "size";
    pl_info_lnx->sysctls[5].mode = PKTLOG_PROCSYS_PERM;
    pl_info_lnx->sysctls[5].proc_handler = ath_sysctl_pktlog_size;
    pl_info_lnx->sysctls[5].extra1 = sc;

    pl_info_lnx->sysctls[6].ctl_name = CTL_AUTO;
    pl_info_lnx->sysctls[6].procname = "options";
    pl_info_lnx->sysctls[6].mode = PKTLOG_PROCSYS_PERM;
    pl_info_lnx->sysctls[6].proc_handler = proc_dointvec;
    pl_info_lnx->sysctls[6].data = &pl_info_lnx->info.options;
    pl_info_lnx->sysctls[6].maxlen = sizeof(pl_info_lnx->info.options);
    /* [7] is NULL terminator */

    /* and register everything */
    /* register_sysctl_table changed from 2.6.21 onwards */
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20))
    pl_info_lnx->sysctl_header = register_sysctl_table(pl_info_lnx->sysctls);
#else
    pl_info_lnx->sysctl_header = register_sysctl_table(pl_info_lnx->sysctls, 1);
#endif
    if (!pl_info_lnx->sysctl_header) {
        printk("%s: failed to register sysctls!\n", proc_name);
        return -1;
    }

    return 0;
}

static void
pktlog_sysctl_unregister(struct ath_softc *sc)
{
    struct ath_pktlog_info_lnx *pl_info_lnx = (sc) ?
        PL_INFO_LNX(sc->pl_info) : PL_INFO_LNX(g_pktlog_info);

    if (pl_info_lnx->sysctl_header) {
        unregister_sysctl_table(pl_info_lnx->sysctl_header);
        pl_info_lnx->sysctl_header = NULL;
    }
}

static int __init
pktlogmod_init(void)
{
    int i;
    char dev_name[PKTLOG_DEVNAME_SIZE];
    struct net_device *dev;
    int ret;

    /* Init system-wide logging */
    g_pktlog_pde = proc_mkdir(PKTLOG_PROC_DIR, NULL);
    if (g_pktlog_pde == NULL) {
        printk(PKTLOG_TAG "%s: proc_mkdir failed\n", __FUNCTION__);
        return -1;
    }
    if ((ret = pktlog_attach(NULL)))
        goto init_fail1;

    /* Init per-adapter logging */
    for (i = 0; i < MAX_WLANDEV; i++) {
        snprintf(dev_name, sizeof(dev_name), WLANDEV_BASENAME "%d", i);
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,23))
        if ((dev = dev_get_by_name(&init_net,dev_name)) != NULL) {
#else
        if ((dev = dev_get_by_name(dev_name)) != NULL) {
#endif
            struct ath_softc_net80211 *scn = dev->priv;
            if (pktlog_attach((struct ath_softc *) (scn->sc_dev)))
                printk(PKTLOG_TAG "%s: pktlog_attach failed for %s\n",
                       __FUNCTION__, dev_name);
            dev_put(dev);
        }
    }

    g_pktlog_funcs = &g_exported_pktlog_funcs;
    g_pktlog_halfuncs = &g_exported_pktlog_halfuncs;
    g_pktlog_rcfuncs = &g_exported_pktlog_rcfuncs;

    return 0;

init_fail1:
    remove_proc_entry(PKTLOG_PROC_DIR, NULL);
    return ret;
}
module_init(pktlogmod_init);

static void __exit
pktlogmod_exit(void)
{
    int i;
    char dev_name[PKTLOG_DEVNAME_SIZE];
    struct net_device *dev;

    g_pktlog_funcs = NULL;
    g_pktlog_halfuncs = NULL;
    g_pktlog_rcfuncs = NULL;

    pktlog_detach(NULL);

    for (i = 0; i < MAX_WLANDEV; i++) {
        snprintf(dev_name, sizeof(dev_name), WLANDEV_BASENAME "%d", i);
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,23))
        if ((dev = dev_get_by_name(&init_net,dev_name)) != NULL) {
#else
        if ((dev = dev_get_by_name(dev_name)) != NULL) {
#endif
            struct ath_softc_net80211 *scn = dev->priv;
            pktlog_detach((struct ath_softc *) (scn->sc_dev));
            dev_put(dev);
        }
    }

    remove_proc_entry(PKTLOG_PROC_DIR, NULL);
}
module_exit(pktlogmod_exit);

/* 
 * Initialize logging for system or adapter
 * Parameter sc should be NULL for system wide logging 
 */
static int
pktlog_attach(struct ath_softc *sc)
{
    struct ath_pktlog_info_lnx *pl_info_lnx;
    char *proc_name;
    struct proc_dir_entry *proc_entry;

    if (sc) {
        pl_info_lnx = kmalloc(sizeof(*pl_info_lnx), GFP_KERNEL);
        if (pl_info_lnx == NULL) {
            printk(PKTLOG_TAG "%s:allocation failed for pl_info\n", __FUNCTION__);
            return -ENOMEM;
        }
        sc->pl_info = &pl_info_lnx->info;
        proc_name = sc->sc_osdev->netdev->name;
    } else {
        if (g_pktlog_info == NULL) {
            /* allocate the global log structure */
            pl_info_lnx = kmalloc(sizeof(*pl_info_lnx), GFP_KERNEL);
            if (pl_info_lnx == NULL) {
                printk(PKTLOG_TAG "%s:allocation failed for pl_info\n", __FUNCTION__);
                return -ENOMEM;
            }
            g_pktlog_info = &pl_info_lnx->info;
        } else {
            pl_info_lnx = PL_INFO_LNX(g_pktlog_info);
        }
        proc_name = PKTLOG_PROC_SYSTEM;
    }

    /* initialize log info */
    pktlog_init(&pl_info_lnx->info);
    pl_info_lnx->proc_entry = NULL;
    pl_info_lnx->sysctl_header = NULL;

    proc_entry = create_proc_entry(proc_name, PKTLOG_PROC_PERM, g_pktlog_pde);
    if (proc_entry == NULL) {
        printk(PKTLOG_TAG "%s: create_proc_entry failed for %s\n",
               __FUNCTION__, proc_name);
        goto attach_fail1;
    }

    proc_entry->data = &pl_info_lnx->info;
    proc_entry->owner = THIS_MODULE;
    proc_entry->proc_fops = &pktlog_fops;
    pl_info_lnx->proc_entry = proc_entry;

    if (pktlog_sysctl_register(sc)) {
        printk(PKTLOG_TAG "%s: sysctl register failed for %s\n",
               __FUNCTION__, proc_name);
        goto attach_fail2;
    }
    return 0;

attach_fail2:
    remove_proc_entry(proc_name, g_pktlog_pde);
attach_fail1:
    if(sc)
        kfree(sc->pl_info);
    return -1;
}

static void
pktlog_detach(struct ath_softc *sc)
{
    struct ath_pktlog_info *pl_info = sc ? sc->pl_info : g_pktlog_info;
    struct ath_pktlog_info_lnx *pl_info_lnx = PL_INFO_LNX(pl_info);

    remove_proc_entry(pl_info_lnx->proc_entry->name, g_pktlog_pde);

    pktlog_sysctl_unregister(sc);

    pktlog_cleanup(pl_info);
    
    if (pl_info->buf)
        pktlog_release_buf(pl_info);

    if (sc) {
        kfree(pl_info);
        sc->pl_info = NULL;
    } else {
        kfree(g_pktlog_info);
        g_pktlog_info = NULL;
    }

    return;
}

static int pktlog_open(struct inode *i, struct file *f)
{
    PKTLOG_MOD_INC_USE_COUNT;
    return 0;
}


static int pktlog_release(struct inode *i, struct file *f)
{
    PKTLOG_MOD_DEC_USE_COUNT;
    return 0;
}

#ifndef MIN
#define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

static ssize_t
pktlog_read(struct file *file, char *buf, size_t nbytes, loff_t *ppos)
{
    size_t bufhdr_size;
    size_t count = 0, ret_val = 0;
    int rem_len;
    int start_offset, end_offset;
    int fold_offset, ppos_data, cur_rd_offset;
    struct proc_dir_entry *proc_entry = PDE(file->f_dentry->d_inode);
    struct ath_pktlog_info *pl_info =
        (struct ath_pktlog_info *) proc_entry->data;
    struct ath_pktlog_buf *log_buf = pl_info->buf;
   
    if (log_buf == NULL)
        return 0;

    if(*ppos == 0 && pl_info->log_state) {
        pl_info->saved_state = pl_info->log_state;
        pl_info->log_state = 0;
    }

    bufhdr_size = sizeof(log_buf->bufhdr);

    /* copy valid log entries from circular buffer into user space */
    rem_len = nbytes;

    count = 0;

    if (*ppos < bufhdr_size) {
        count = MIN((bufhdr_size - *ppos), rem_len);
        if (copy_to_user(buf, ((char *) &log_buf->bufhdr) + *ppos, count))
	    return -EFAULT;
        rem_len -= count;
        ret_val += count;
    }

    start_offset = log_buf->rd_offset;

    if ((rem_len == 0) || (start_offset < 0))
        goto rd_done;

    fold_offset = -1;
    cur_rd_offset = start_offset;

    /* Find the last offset and fold-offset if the buffer is folded */
    do {
        struct ath_pktlog_hdr *log_hdr;
        int log_data_offset;

        log_hdr =
            (struct ath_pktlog_hdr *) (log_buf->log_data + cur_rd_offset);

        log_data_offset = cur_rd_offset + sizeof(struct ath_pktlog_hdr);

        if ((fold_offset == -1)
            && ((pl_info->buf_size - log_data_offset) <= log_hdr->size))
            fold_offset = log_data_offset - 1;

        PKTLOG_MOV_RD_IDX(cur_rd_offset, log_buf, pl_info->buf_size);

        if ((fold_offset == -1) && (cur_rd_offset == 0)
            && (cur_rd_offset != log_buf->wr_offset))
            fold_offset = log_data_offset + log_hdr->size - 1;

        end_offset = log_data_offset + log_hdr->size - 1;
    } while (cur_rd_offset != log_buf->wr_offset);

    ppos_data = *ppos + ret_val - bufhdr_size + start_offset;

    if (fold_offset == -1) {
        if (ppos_data > end_offset)
            goto rd_done;

        count = MIN(rem_len, (end_offset - ppos_data + 1));
        if (copy_to_user(buf + ret_val, log_buf->log_data + ppos_data, count))
	    return -EFAULT;
        ret_val += count;
        rem_len -= count;
    } else {
        if (ppos_data <= fold_offset) {
            count = MIN(rem_len, (fold_offset - ppos_data + 1));
            if (copy_to_user(buf + ret_val, log_buf->log_data + ppos_data,
                         count)) return -EFAULT;
            ret_val += count;
            rem_len -= count;
        }

        if (rem_len == 0)
            goto rd_done;

        ppos_data =
            *ppos + ret_val - (bufhdr_size +
                               (fold_offset - start_offset + 1));

        if (ppos_data <= end_offset) {
            count = MIN(rem_len, (end_offset - ppos_data + 1));
            if (copy_to_user(buf + ret_val, log_buf->log_data + ppos_data,
                         count)) return -EFAULT;
            ret_val += count;
            rem_len -= count;
        }
    }

rd_done:
    if((ret_val < nbytes) && pl_info->saved_state) {
        pl_info->log_state = pl_info->saved_state;
        pl_info->saved_state = 0;
    }
    *ppos += ret_val;

    return ret_val;
}

#ifndef VMALLOC_VMADDR
#define VMALLOC_VMADDR(x) ((unsigned long)(x))
#endif

/* Convert a kernel virtual address to a kernel logical address */
static volatile void *pktlog_virt_to_logical(volatile void *addr)
{
    pgd_t *pgd;
    pmd_t *pmd;
    pte_t *ptep, pte;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15) || (defined (__i386__) && LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11)))
    pud_t *pud;
#endif
    unsigned long vaddr, ret = 0UL;

    vaddr = VMALLOC_VMADDR((unsigned long) addr);

    pgd = pgd_offset_k(vaddr);

    if (!pgd_none(*pgd)) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15) || (defined (__i386__) && LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11)))
		pud = pud_offset(pgd, vaddr);
        pmd = pmd_offset(pud, vaddr);
#else
        pmd = pmd_offset(pgd, vaddr);
#endif

        if (!pmd_none(*pmd)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
            ptep = pte_offset_map(pmd, vaddr);
#else
            ptep = pte_offset(pmd, vaddr);
#endif
            pte = *ptep;

            if (pte_present(pte)) {
                ret = (unsigned long) page_address(pte_page(pte));
                ret |= (vaddr & (PAGE_SIZE - 1));
            }
        }
    }
    return ((volatile void *) ret);
}

/* vma operations for mapping vmalloced area to user space */
static void pktlog_vopen(struct vm_area_struct *vma)
{
    PKTLOG_MOD_INC_USE_COUNT;
}

static void pktlog_vclose(struct vm_area_struct *vma)
{
    PKTLOG_MOD_DEC_USE_COUNT;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,25)
int pktlog_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	unsigned long address = (unsigned long)vmf->virtual_address;

	if (address == 0UL) {
        	printk(PKTLOG_TAG "%s: page fault out of range\n", __FUNCTION__);
        	return VM_FAULT_NOPAGE;
    	}
	if (vmf->pgoff > vma->vm_end)
		return VM_FAULT_SIGBUS;

	/*    
	page = alloc_page (GFP_HIGHUSER);
 	if (!page) 
	{
		return VM_FAULT_OOM;
	}
	*/

	get_page(virt_to_page((void *)address));
	vmf->page = virt_to_page((void *)address);
	return VM_FAULT_MINOR;
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
struct page *pktlog_vmmap(struct vm_area_struct *vma, unsigned long addr,
                          int *type)
#else
struct page *pktlog_vmmap(struct vm_area_struct *vma, unsigned long addr,
                          int write_access)
#endif
{
    unsigned long offset, vaddr;
    struct proc_dir_entry *proc_entry = PDE(vma->vm_file->f_dentry->d_inode);

    struct ath_pktlog_info *pl_info =
        (struct ath_pktlog_info *) proc_entry->data;

    offset = addr - vma->vm_start + (vma->vm_pgoff << PAGE_SHIFT);

    vaddr = (unsigned long) pktlog_virt_to_logical((void *) (pl_info->buf) + offset);

    if (vaddr == 0UL) {
        printk(PKTLOG_TAG "%s: page fault out of range\n", __FUNCTION__);
        return ((struct page *) 0UL);
    }

    /* increment the usage count of the page */
    get_page(virt_to_page((void*)vaddr));
    
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    if(type) {
        *type = VM_FAULT_MINOR;
    }
#endif

    return (virt_to_page((void*)vaddr));
}

static struct vm_operations_struct pktlog_vmops = {
    open:pktlog_vopen,
    close:pktlog_vclose,
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,25)
    fault:pktlog_fault,
#else 
    nopage:pktlog_vmmap,		
#endif
};

static int pktlog_mmap(struct file *file, struct vm_area_struct *vma)
{
    struct proc_dir_entry *proc_entry = PDE(file->f_dentry->d_inode);
    struct ath_pktlog_info *pl_info =
        (struct ath_pktlog_info *) proc_entry->data;

    if (vma->vm_pgoff != 0) {
        /* Entire buffer should be mapped */
        return -EINVAL;
    }

    if (!pl_info->buf) {
        printk(PKTLOG_TAG "%s: Log buffer unavailable\n", __FUNCTION__);
        return -ENOMEM;
    }

    vma->vm_flags |= VM_LOCKED;
    vma->vm_ops = &pktlog_vmops;
    pktlog_vopen(vma);
    return 0;
}

/*
 * Linux implementation of helper functions
 */
void
pktlog_disable_adapter_logging(void)
{
    struct net_device *dev;
    int i;
    char dev_name[PKTLOG_DEVNAME_SIZE];

    /* Disable any active per adapter logging */
    for (i = 0; i < MAX_WLANDEV; i++) {
        snprintf(dev_name, sizeof(dev_name),
                 WLANDEV_BASENAME "%d", i);


#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,23))
        if ((dev = dev_get_by_name(&init_net,dev_name)) != NULL) {
#else
        if ((dev = dev_get_by_name(dev_name)) != NULL) {
#endif
            struct ath_softc_net80211 *scn = dev->priv;
            struct ath_softc *dev_sc = (struct ath_softc *)(scn->sc_dev);
            dev_sc->pl_info->log_state = 0;
            dev_put(dev);
        }
    }
}

int
pktlog_alloc_buf(struct ath_softc *sc, struct ath_pktlog_info *pl_info)
{
    u_int32_t page_cnt;
    unsigned long vaddr;
    struct page *vpg;

    page_cnt = (sizeof(*(pl_info->buf)) +
                pl_info->buf_size) / PAGE_SIZE;

    if ((pl_info->buf = vmalloc((page_cnt + 2) * PAGE_SIZE))
        == NULL) {
        printk(PKTLOG_TAG
               "%s: Unable to allocate buffer"
               "(%d pages)\n", __FUNCTION__, page_cnt);
        return -ENOMEM;
    }

    pl_info->buf = (struct ath_pktlog_buf *)
        (((unsigned long) (pl_info->buf) + PAGE_SIZE - 1)
         & PAGE_MASK);

    for (vaddr = (unsigned long) (pl_info->buf);
         vaddr < ((unsigned long) (pl_info->buf)
                  + (page_cnt * PAGE_SIZE)); vaddr += PAGE_SIZE) {
        vpg = virt_to_page(pktlog_virt_to_logical((void *) vaddr));
        SetPageReserved(vpg);
    }

    return 0;
}

void
pktlog_release_buf(struct ath_pktlog_info *pl_info)
{
    unsigned long page_cnt;
    unsigned long vaddr;
    struct page *vpg;

    page_cnt =
        ((sizeof(*(pl_info->buf)) + pl_info->buf_size) / PAGE_SIZE) + 1;

    for (vaddr = (unsigned long) (pl_info->buf); vaddr <
         (unsigned long) (pl_info->buf) + (page_cnt * PAGE_SIZE);
         vaddr += PAGE_SIZE) {
        vpg = virt_to_page(pktlog_virt_to_logical((void *) vaddr));
        ClearPageReserved(vpg);
    }

    vfree(pl_info->buf);
    pl_info->buf = NULL;
}

int
pktlog_tcpip(struct ath_softc *sc, struct llc *llc, u_int32_t *proto_log, int *proto_len)
{
    struct iphdr *ip;
    struct tcphdr *tcp;
    u_int32_t *proto_hdr;
    int i;

    ip = (struct iphdr *)((u_int8_t *)llc + LLC_SNAPFRAMELEN);

    switch (ip->protocol) {
    case IPPROTO_TCP:
        /* 
         * tcp + ip hdr len are in units of 32-bit words
         */ 
        tcp = (struct tcphdr *)((u_int32_t *)ip + ip->ihl);
        proto_hdr = (u_int32_t *)tcp;

        for (i = 0; i < tcp->doff; i++) {
            *proto_log = ntohl(*proto_hdr);
            proto_log++, proto_hdr++;
        }
        *proto_len = tcp->doff * sizeof(u_int32_t);
        return PKTLOG_PROTO_TCP;

    case IPPROTO_UDP:
    case IPPROTO_ICMP:
    default:
        return PKTLOG_PROTO_NONE;
    }
}

#endif /* REMOVE_PKT_LOG */
