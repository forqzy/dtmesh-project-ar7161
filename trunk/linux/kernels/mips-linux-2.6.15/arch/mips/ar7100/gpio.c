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
 */

#ifndef EXPORT_SYMTAB
#define EXPORT_SYMTAB
#endif

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/signal.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/resource.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#include <linux/timer.h>
#include <asm/types.h>
#include <asm/irq.h>
#include <asm/delay.h>

#include "ar7100.h"
#include "board_type.h"

#include <linux/moduleparam.h>
static u_int8_t wps_led, wps_gpio, dft_gpio;

#define AR7100_FACTORY_RESET		0x89ABCDEF
#define AP_PUSH_BUTTON 1
static atomic_t ar7100_fr_status = ATOMIC_INIT(0);
static volatile int ar7100_fr_opened=0;
static wait_queue_head_t ar7100_fr_wq;

/*
 * GPIO interrupt stuff
 */
typedef enum {
    INT_TYPE_EDGE,
    INT_TYPE_LEVEL,
}ar7100_gpio_int_type_t;

typedef enum {
    INT_POL_ACTIVE_LOW,
    INT_POL_ACTIVE_HIGH,
}ar7100_gpio_int_pol_t;


/* 
** Simple Config stuff
*/

#if !defined(IRQ_NONE)
#define IRQ_NONE
#define IRQ_HANDLED
#endif /* !defined(IRQ_NONE) */

typedef irqreturn_t(*sc_callback_t)(int, void *, struct pt_regs *);

/*
** Multiple Simple Config callback support
** For multiple radio scenarios, we need to post the button push to
** all radios at the same time.  However, there is only 1 button, so
** we only have one set of GPIO callback pointers.
**
** Creating a structure that contains each callback, tagged with the
** name of the device registering the callback.  The unregister routine
** will need to determine which element to "unregister", so the device
** name will have to be passed to unregister also
*/

typedef struct {
char	*name;
sc_callback_t	registered_cb;
void			*cb_arg;
} Multi_Callback_t;

/*
** Specific instance of the callback structure
*/

static Multi_Callback_t	SCcallback[2] = {{NULL,NULL,NULL},{NULL,NULL,NULL}};

/*
** These are for the GPIO, so we don't create multiple instances
*/

static int ignore_pushbutton = 0;
static int ignore_resetbutton = 0;
static struct proc_dir_entry *simple_config_entry = NULL;
static struct proc_dir_entry *simulate_push_button_entry = NULL;
static struct proc_dir_entry *gpio_led_entry = NULL;


void ar7100_gpio_config_int(int gpio, 
                       ar7100_gpio_int_type_t type,
                       ar7100_gpio_int_pol_t polarity)
{
    u32 val;

    /*
     * allow edge sensitive/rising edge too
     */
    if (type == INT_TYPE_LEVEL) {
        /* level sensitive */
        ar7100_reg_rmw_set(AR7100_GPIO_INT_TYPE, (1 << gpio));
    }
    else {
       /* edge triggered */
       val = ar7100_reg_rd(AR7100_GPIO_INT_TYPE);
       val &= ~(1 << gpio);
       ar7100_reg_wr(AR7100_GPIO_INT_TYPE, val);
    }

    if (polarity == INT_POL_ACTIVE_HIGH) {
        ar7100_reg_rmw_set (AR7100_GPIO_INT_POLARITY, (1 << gpio));
    }
    else {
       val = ar7100_reg_rd(AR7100_GPIO_INT_POLARITY);
       val &= ~(1 << gpio);
       ar7100_reg_wr(AR7100_GPIO_INT_POLARITY, val);
    }

    ar7100_reg_rmw_set(AR7100_GPIO_INT_ENABLE, (1 << gpio));
}

void
ar7100_gpio_config_output(int gpio)
{
    ar7100_reg_rmw_set(AR7100_GPIO_OE, (1 << gpio));
}

void
ar7100_gpio_config_input(int gpio)
{
    ar7100_reg_rmw_clear(AR7100_GPIO_OE, (1 << gpio));
}

void
ar7100_gpio_out_val(int gpio, int val)
{
    if (val & 0x1) {
        ar7100_reg_rmw_set(AR7100_GPIO_OUT, (1 << gpio));
    }
    else {
        ar7100_reg_rmw_clear(AR7100_GPIO_OUT, (1 << gpio));
    }
}

int
ar7100_gpio_in_val(int gpio)
{
    return((1 << gpio) & (ar7100_reg_rd(AR7100_GPIO_IN)));
}

static void
ar7100_gpio_intr_enable(unsigned int irq)
{
    ar7100_reg_rmw_set(AR7100_GPIO_INT_MASK, 
                      (1 << (irq - AR7100_GPIO_IRQ_BASE)));
}

static void
ar7100_gpio_intr_disable(unsigned int irq)
{
    ar7100_reg_rmw_clear(AR7100_GPIO_INT_MASK, 
                        (1 << (irq - AR7100_GPIO_IRQ_BASE)));
}

static unsigned int
ar7100_gpio_intr_startup(unsigned int irq)
{
	ar7100_gpio_intr_enable(irq);
	return 0;
}

static void
ar7100_gpio_intr_shutdown(unsigned int irq)
{
	ar7100_gpio_intr_disable(irq);
}

static void
ar7100_gpio_intr_ack(unsigned int irq)
{
	ar7100_gpio_intr_disable(irq);
}

static void
ar7100_gpio_intr_end(unsigned int irq)
{
	if (!(irq_desc[irq].status & (IRQ_DISABLED | IRQ_INPROGRESS)))
		ar7100_gpio_intr_enable(irq);
}

static void
ar7100_gpio_intr_set_affinity(unsigned int irq, unsigned long mask)
{
	/* 
     * Only 1 CPU; ignore affinity request
     */
}

struct hw_interrupt_type ar7100_gpio_intr_controller = {
	"AR7100 GPIO",
	ar7100_gpio_intr_startup,
	ar7100_gpio_intr_shutdown,
	ar7100_gpio_intr_enable,
	ar7100_gpio_intr_disable,
	ar7100_gpio_intr_ack,
	ar7100_gpio_intr_end,
	ar7100_gpio_intr_set_affinity,
};

void
ar7100_gpio_irq_init(int irq_base)
{
	int i;

	for (i = irq_base; i < irq_base + AR7100_GPIO_IRQ_COUNT; i++) {
		irq_desc[i].status  = IRQ_DISABLED;
		irq_desc[i].action  = NULL;
		irq_desc[i].depth   = 1;
		irq_desc[i].handler = &ar7100_gpio_intr_controller;
	}
}


void register_simple_config_callback (char *cbname, void *callback, void *arg)
{
	printk("!!!!! SC Callback Registration for %s\n",cbname);
	if(!SCcallback[0].name)
	{
		SCcallback[0].name = cbname;
		SCcallback[0].registered_cb = (sc_callback_t) callback;
		SCcallback[0].cb_arg = arg;
	}
	else
	{
		SCcallback[1].name = cbname;
		SCcallback[1].registered_cb = (sc_callback_t) callback;
		SCcallback[1].cb_arg = arg;
	}
}
EXPORT_SYMBOL(register_simple_config_callback);

void unregister_simple_config_callback (char *cbname)
{
	if(SCcallback[1].name == cbname)
	{
		SCcallback[1].name = NULL;
		SCcallback[1].registered_cb = NULL;
		SCcallback[1].cb_arg = NULL;
	}
	else if(SCcallback[0].name == cbname)
	{
		SCcallback[0].name = NULL;
		SCcallback[0].registered_cb = NULL;
		SCcallback[0].cb_arg = NULL;
	}
	else
	{
		printk("!&!&!&!& ERROR: Unknown callback name %s\n",cbname);
	}
}
EXPORT_SYMBOL(unregister_simple_config_callback);

/*
 * Irq for front panel SW wps switch
 * Connected to XSCALE through GPIO4
 */
irqreturn_t wps_irq(int cpl, void *dev_id, struct pt_regs *regs)
{
    if (ignore_pushbutton) {
        ar7100_gpio_config_int (wps_gpio,INT_TYPE_LEVEL,
                                INT_POL_ACTIVE_HIGH);
        ignore_pushbutton = 0;
        return IRQ_HANDLED;
    }

    ar7100_gpio_config_int (wps_gpio,INT_TYPE_LEVEL,INT_POL_ACTIVE_LOW);
    cpl = AP_PUSH_BUTTON;
    ignore_pushbutton = 1;

    if (SCcallback[0].registered_cb) {
        SCcallback[0].registered_cb (cpl, SCcallback[0].cb_arg, regs);
    }
    if (SCcallback[1].registered_cb) {
        SCcallback[1].registered_cb (cpl, SCcallback[1].cb_arg, regs);
    }

    return IRQ_HANDLED;
}

static int push_button_read (char *page, char **start, off_t off,
                               int count, int *eof, void *data)
{
    return 0;
}

static int push_button_write (struct file *file, const char *buf,
                                        unsigned long count, void *data)
{
    if (SCcallback[0].registered_cb) {
        SCcallback[0].registered_cb (0, SCcallback[0].cb_arg, 0);
    }
    if (SCcallback[1].registered_cb) {
        SCcallback[1].registered_cb (0, SCcallback[1].cb_arg, 0);
    }
    return count;
}

#define WPS_LED_MASK    (1<<(wps_led))
#define WPS_LED_OFF     1
#define WPS_LED_ON      2
#define WPS_LED_BLINK   3
#define WPS_HALF_PERIOD (HZ/2)  /* set to 500ms per half period */

static int gpio_leds = WPS_LED_OFF;
static struct timer_list wps_blink_timer;

int get_simple_config_lock(void)
{
    if(gpio_leds == WPS_LED_BLINK)
	return -1;
    
    return 0;
}
EXPORT_SYMBOL(get_simple_config_lock);

static void wps_led_blink(unsigned long data)
{
    if(gpio_leds != WPS_LED_BLINK) {
        if(ar7100_board_type == BT_JA76PF) {
            ar7100_reg_wr(AR7100_GPIO_SET, WPS_LED_MASK);
        }
        else {
            ar7100_reg_wr(AR7100_GPIO_CLEAR, WPS_LED_MASK);
        }
        return;
    }

    if(ar7100_reg_rd(AR7100_GPIO_IN) & WPS_LED_MASK) {
        ar7100_reg_wr(AR7100_GPIO_CLEAR, WPS_LED_MASK);
    }
    else {
        ar7100_reg_wr(AR7100_GPIO_SET, WPS_LED_MASK);
    }
    mod_timer(&wps_blink_timer, jiffies + WPS_HALF_PERIOD);
}

static int gpio_led_read (char *page, char **start, off_t off,
               int count, int *eof, void *data)
{
    return sprintf (page, "%d\n", gpio_leds);
}

static int gpio_led_write (struct file *file, const char *buf,
                                        unsigned long count, void *data)
{
    u_int32_t val;

    if (sscanf(buf, "%d", &val) != 1)
      return -EINVAL;
    
    gpio_leds = val;
	
    switch (gpio_leds) {
    case WPS_LED_OFF :
        if(ar7100_board_type == BT_JA76PF) {
  	         ar7100_reg_wr(AR7100_GPIO_SET, WPS_LED_MASK);
        } 
        else {
            ar7100_reg_wr(AR7100_GPIO_CLEAR, WPS_LED_MASK);
        }
	 break;
    case WPS_LED_ON :
        if(ar7100_board_type == BT_JA76PF) {
            ar7100_reg_wr(AR7100_GPIO_CLEAR, WPS_LED_MASK);
        }
        else {
            ar7100_reg_wr(AR7100_GPIO_SET, WPS_LED_MASK);
        }
	 break;
    case WPS_LED_BLINK:
	 wps_led_blink(0);
	 break;
    }
    
    return count;
}


static int create_simple_config_led_proc_entry (void)
{
    int ii;

    if (simple_config_entry != NULL) {
        printk ("Already have a proc entry for /proc/simple_config!\n");
        return -ENOENT;
    }

    simple_config_entry = proc_mkdir("simple_config", NULL);
    if (!simple_config_entry)
        return -ENOENT;

    simulate_push_button_entry = create_proc_entry ("push_button", 0644,
                                                      simple_config_entry);
    if (!simulate_push_button_entry)
        return -ENOENT;

    simulate_push_button_entry->write_proc = push_button_write;
    simulate_push_button_entry->read_proc = push_button_read;

    gpio_led_entry = create_proc_entry ("simple_config_led", 0644,
                                            simple_config_entry);
    if (!gpio_led_entry)
        return -ENOENT;

    gpio_led_entry->write_proc = gpio_led_write;
    gpio_led_entry->read_proc = gpio_led_read;

    for (ii=0; ii < AR7100_GPIO_COUNT; ii++) {
        if ((1<<ii) & WPS_LED_MASK) {
            /* configure gpio as outputs */
            ar7100_gpio_config_output(ii); 

            /* switch off the led */
            if(ar7100_board_type == BT_JA76PF)
                ar7100_gpio_out_val(ii, 1);
            else
                ar7100_gpio_out_val(ii, 0);
        }
    }

    init_timer(&wps_blink_timer);
    wps_blink_timer.function = wps_led_blink;
    wps_blink_timer.expires = jiffies + WPS_HALF_PERIOD;

    return 0;
}

int __init assign_gpio_pins(void)
{
#ifdef CONFIG_INTELLUS
    if(ar7100_board_type == BT_JA76PF) {
        dft_gpio = 11;
        wps_gpio = 6;
        wps_led = 4;
    }
    else if(ar7100_board_type == BT_JA73PF) {
        dft_gpio = 11;
        wps_gpio = 11;
        wps_led = 4;
    }
    else if(ar7100_board_type == BT_JA76HX || ar7100_board_type == BT_JA76PF2) {
        dft_gpio = 8;
        wps_gpio = 7;
        wps_led = 5;
    }
    else {
        wps_led = 0; //none
#elif defined(CONFIG_AR9100)
        dft_gpio = 21;
        wps_gpio = 12;
#else
        dft_gpio =  8;
#if defined(CONFIG_CUS100)
        wps_gpio  = 7;
#else
        wps_gpio = 3;
#endif
#endif
    }
    printk("%s: %d %d %d\n", __func__, dft_gpio, wps_gpio, wps_led);
    return 1;
}
arch_initcall(assign_gpio_pins);

int __init pbXX_simple_config_init(void)
{
#ifdef CONFIG_CUS100
	u32 mask = 0;
#endif
    int req;

    if( wps_gpio == dft_gpio)
        return 0;

#ifdef CONFIG_CUS100
	mask = ar7100_reg_rd(AR7100_MISC_INT_MASK);
	ar7100_reg_wr(AR7100_MISC_INT_MASK, mask | (1 << 2)); /* Enable GPIO interrupt mask */
    ar7100_gpio_config_int (wps_gpio, INT_TYPE_LEVEL,INT_POL_ACTIVE_HIGH);
	ar7100_gpio_intr_enable(wps_gpio);
	ar7100_gpio_config_input(wps_gpio);
#else
    /* configure GPIO 3 as level triggered interrupt */ 
    ar7100_gpio_config_int (wps_gpio, INT_TYPE_LEVEL,INT_POL_ACTIVE_HIGH);
#endif

    req = request_irq (AR7100_GPIO_IRQn(wps_gpio), wps_irq, 0,
                       "SW WPS", NULL);
    if (req != 0) {
        printk (KERN_ERR "unable to request IRQ for SWWPS GPIO (error %d)\n", req);
    }

    create_simple_config_led_proc_entry ();
    return 0;
}

subsys_initcall(pbXX_simple_config_init);
static int
ar7100fr_open(struct inode *inode, struct file *file)
{
	if (MINOR(inode->i_rdev) != FACTORY_RESET_MINOR) {
		return -ENODEV;
	}

	if (ar7100_fr_opened) {
		return -EBUSY;
	}

        ar7100_fr_opened = 1;
	return nonseekable_open(inode, file);
}

static int
ar7100fr_close(struct inode *inode, struct file *file)
{
	if (MINOR(inode->i_rdev) != FACTORY_RESET_MINOR) {
		return -ENODEV;
	}

	ar7100_fr_opened = 0;
	return 0;
}

static ssize_t
ar7100fr_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	return -ENOTSUPP;
}

static ssize_t
ar7100fr_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	return -ENOTSUPP;
}

static int
ar7100fr_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int ret = 0;

	switch(cmd) {
		case AR7100_FACTORY_RESET:
            
			/* Userspace process "factoryreset" is doing IOCTL
 			 * Hope AP is successfully UP
 			 * Turn the power LED on
 			 */
#ifdef CONFIG_AR9100
#define AP8x_GPIO_POWER_LED  (0x01 << 14) 
 			ar7100_reg_wr(AR7100_GPIO_OE, AP8x_GPIO_POWER_LED);
			ar7100_reg_wr(AR7100_GPIO_OUT,
	          ((ar7100_reg_rd(AR7100_GPIO_OUT)) | ((AP8x_GPIO_POWER_LED))));
#endif

            atomic_inc(&ar7100_fr_status);
			sleep_on(&ar7100_fr_wq);
			break;

		default: ret = -EINVAL;
	}

	return ret;
}

/* 
   if wps_gpio == dft_gpio, WSC and FR share one pushbutton:
   use time to select. >5 seconds is FR, less is WSC
 */
irqreturn_t factory_reset_irq(int cpl, void *dev_id, struct pt_regs *regs)
{
	unsigned int delay;
	
	if (ignore_resetbutton) {
        ar7100_gpio_config_int (dft_gpio,INT_TYPE_LEVEL,
                                INT_POL_ACTIVE_LOW);
        ignore_resetbutton = 0;
        return IRQ_HANDLED;
    }

    ar7100_gpio_config_int (dft_gpio,INT_TYPE_LEVEL,INT_POL_ACTIVE_HIGH);
    ignore_resetbutton = 1;

    local_irq_disable();
    
#define UDELAY_COUNT 4000

    for (delay = UDELAY_COUNT; delay; delay--) {
        if (ar7100_gpio_in_val(dft_gpio)) {
            break;
        }
        udelay(1000);
    }
    
    if (!delay) {
        if(atomic_read(&ar7100_fr_status))	{
            /* 
             * since we are going to reboot the board, we
             * don't need the interrupt handler anymore,
             * so disable it. 
             */
            disable_irq(AR7100_GPIO_IRQn(dft_gpio));
            wake_up(&ar7100_fr_wq);
            printk("\nar7100: factory configuration restored..\n");
        }
    }
    else if((wps_gpio == dft_gpio) &&
            ((UDELAY_COUNT - delay) > 500)) { 
        cpl = AP_PUSH_BUTTON;
        if (SCcallback[0].registered_cb) {
            SCcallback[0].registered_cb (cpl, SCcallback[0].cb_arg, regs);
        }
        if (SCcallback[1].registered_cb) {
            SCcallback[1].registered_cb (cpl, SCcallback[1].cb_arg, regs);
        }
    }

    local_irq_enable();
    
    return IRQ_HANDLED;
}

static struct file_operations ar7100fr_fops = {
	read:	ar7100fr_read,
	write:	ar7100fr_write,
	ioctl:	ar7100fr_ioctl,
	open:	ar7100fr_open,
	release:ar7100fr_close
};

static struct miscdevice ar7100fr_miscdev = 
{ FACTORY_RESET_MINOR, "Factory reset", &ar7100fr_fops };


int __init ar7100_factory_reset_init(void)
{
#ifdef CONFIG_CUS100
	u32 mask = 0;
#endif
    int req, ret;
    ret = misc_register(&ar7100fr_miscdev);

    if (ret < 0) {
            printk("*** ar7100 misc_register failed %d *** \n", ret);
            return -1;
    }
#ifdef CONFIG_CUS100
	mask = ar7100_reg_rd(AR7100_MISC_INT_MASK);
	ar7100_reg_wr(AR7100_MISC_INT_MASK, mask | (1 << 2)); /* Enable GPIO interrupt mask */
	ar7100_gpio_config_int (dft_gpio, INT_TYPE_LEVEL,INT_POL_ACTIVE_HIGH);
	ar7100_gpio_intr_enable(dft_gpio);
	ar7100_gpio_config_input(dft_gpio);
#else
	ar7100_gpio_config_int (dft_gpio, INT_TYPE_LEVEL,INT_POL_ACTIVE_HIGH);
#endif
	req = request_irq (AR7100_GPIO_IRQn(dft_gpio), factory_reset_irq, 0,
                       "FACTORY RESET", NULL);
    if (req != 0) {
        printk (KERN_ERR "unable to request IRQ for FACTORY_RESET GPIO (error %d)\n", req);
        misc_deregister(&ar7100fr_miscdev);
        ar7100_gpio_intr_shutdown(AR7100_GPIO_IRQn(dft_gpio));
        return -1;
    }

	init_waitqueue_head(&ar7100_fr_wq);
    return 0;
}
/* 
 * used late_initcall so that misc_register will succeed 
 * otherwise, misc driver won't be in a initializated state
 * thereby resulting in misc_register api to fail.
 */
late_initcall(ar7100_factory_reset_init);

struct gpio_proc {
    int bit;
    int val;
    int dir;
};

static struct gpio_proc gpio_procs [] = {
    {
	.bit = 2,
    },
    {
	.bit = 7,
    },
    {
	.bit = 8,
    },
};

static struct gpio_proc ja76pf2_gpio_procs [] = {
    {
        .bit = 2,
    },
    {
        .bit = 3,  //D4
    },
    {
        .bit = 4,  //D3
    },
    {
        .bit = 5,  //D2
    },
    {
        .bit = 6,
    },
    {
        .bit = 11,
    },
};


#define N_GPROCS (sizeof(gpio_procs)/sizeof(struct gpio_proc))
#define N_JA76PF2_GPROCS (sizeof(ja76pf2_gpio_procs)/sizeof(struct gpio_proc))

static int gpio_out_read (char *page, char **start, off_t off,
               int count, int *eof, void *data)
{
    struct gpio_proc * g = (struct gpio_proc *) data;

    return sprintf(page, "%d\n", g->val);
}

static int gpio_out_write (struct file *file, const char *buf,
                                        unsigned long count, void *data)
{
    struct gpio_proc * g = (struct gpio_proc *) data;
    u_int32_t val;

    if (sscanf(buf, "%d", &val) != 1)
	return -EINVAL;
    
    if(val == 1) { 
	ar7100_reg_wr(AR7100_GPIO_SET, 1 << g->bit);
    }
    else if (val == 0) {
	ar7100_reg_wr(AR7100_GPIO_CLEAR, 1 << g->bit);
    }
    else
	return -EINVAL;
    
    g->val = val;
	
    return count;
}

static int gpio_in_read (char *page, char **start, off_t off,
               int count, int *eof, void *data)
{
    struct gpio_proc * g = (struct gpio_proc *) data;
    u_int32_t val;

    if(ar7100_gpio_in_val(g->bit))
	val = 1;
    else
	val = 0;

    return sprintf(page, "%d\n", val);
}

static int gpio_dir_read (char *page, char **start, off_t off,
               int count, int *eof, void *data)
{
    struct gpio_proc * g = (struct gpio_proc *) data;

    if(g->dir)
	return sprintf(page, "out\n");
    else
	return sprintf(page, "in\n");
}

static int gpio_dir_write (struct file *file, const char *buf,
                                        unsigned long count, void *data)
{
    struct gpio_proc * g = (struct gpio_proc *) data;

    if (!strncmp(buf, "in", 2) || !strncmp(buf, "IN", 2)) {
	ar7100_gpio_config_input(g->bit);
	g->dir = 0;
    }
    else if (!strncmp(buf, "out", 3) || !strncmp(buf, "OUT", 3)) {
	ar7100_gpio_config_output(g->bit);
	g->dir = 1;
    }
    else
	return -EINVAL; 

    return count;
}

static int __init create_gpio_entry(struct gpio_proc * g, struct proc_dir_entry * parent)
{
    struct proc_dir_entry *directory;
    struct proc_dir_entry *out_entry;
    struct proc_dir_entry *in_entry;
    struct proc_dir_entry *oe_entry;
    char name[10];

    sprintf(name, "%d", g->bit);

    directory = proc_mkdir(name, parent);
    if (!directory)
	goto err;

    out_entry = create_proc_entry ("out", 0644, directory);
    if (!out_entry)
	goto err;

    in_entry = create_proc_entry ("in", 0444, directory);
    if (!in_entry)
	goto err;

    oe_entry = create_proc_entry ("dir", 0644, directory);
    if (!oe_entry)
	goto err;

    out_entry->write_proc = gpio_out_write; 
    out_entry->read_proc = gpio_out_read; 
    out_entry->data = g; 

    in_entry->read_proc = gpio_in_read; 
    in_entry->data = g; 

    oe_entry->write_proc = gpio_dir_write; 
    oe_entry->read_proc = gpio_dir_read; 
    oe_entry->data = g; 

    g->val = 0;
    g->dir = 1;

    ar7100_reg_wr(AR7100_GPIO_CLEAR, 1 << g->bit);
    ar7100_gpio_config_output(g->bit);

    printk("creating %s/%s proc entry\n", parent->name, name);
    return 0;

err:
	printk("failed creating %s/%s proc entry\n", parent->name, name);
        return -ENOENT;
}

int __init gpio_proc_init(void)
{
    struct proc_dir_entry *gpio_proc_entry;
    
    int ii;

    gpio_proc_entry = proc_mkdir("gpio", NULL);

    if (!gpio_proc_entry) {
	printk("failed creating gpio proc entry\n");
        return -ENOENT;
    }
    
    if(ar7100_board_type == BT_JA76HX || ar7100_board_type == BT_JA76PF2) {
        for (ii = 0; ii < N_JA76PF2_GPROCS; ii++) {
            if(create_gpio_entry(&ja76pf2_gpio_procs[ii], gpio_proc_entry))
                break;
        }
    }
    else {
        for (ii = 0; ii < N_GPROCS; ii++) {
            if(create_gpio_entry(&gpio_procs[ii], gpio_proc_entry))
	        break;
        }
    }

    if (ii == 0)
	remove_proc_entry("gpio", NULL);
	
    return 0;
}

device_initcall(gpio_proc_init);

