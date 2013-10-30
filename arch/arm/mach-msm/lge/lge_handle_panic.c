/* 
 * arch/arm/mach-msm/lge/lge_handle_panic.c
 *
 * Copyright (C) 2010 LGE, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <asm/setup.h>
#include <mach/board_lge.h>

#define PANIC_HANDLER_NAME "panic-handler"
#define PANIC_DUMP_CONSOLE 0
#define PANIC_MAGIC_KEY	0x12345678
#define CRASH_ARM9		0x87654321
#define CRASH_REBOOT	0x618E1000

struct crash_log_dump {
	unsigned int magic_key;
	unsigned int size;
	unsigned char buffer[0];
};

static struct crash_log_dump *crash_dump_log;
static unsigned int crash_buf_size = 0;
static int crash_store_flag = 0;

/* LGE_UPDATE_S kideok.kim@lge.com 20110717 hidden_reset_porting from Victor. */
#ifndef CONFIG_LGE_HIDDEN_RESET_PATCH
static DEFINE_SPINLOCK(lge_panic_lock);
#endif
/* LGE_UPDATE_E kideok.kim@lge.com 20110717 hidden_reset_porting from Victor. */

static int dummy_arg;
static int gen_bug(const char *val, struct kernel_param *kp)
{
	BUG();

	return 0;
}
module_param_call(gen_bug, gen_bug, param_get_bool, &dummy_arg, S_IWUSR | S_IRUGO);

static int gen_panic(const char *val, struct kernel_param *kp)
{
	panic("generate test-panic");

	return 0;
}
module_param_call(gen_panic, gen_panic, param_get_bool, &dummy_arg, S_IWUSR | S_IRUGO);

static int crash_handle_enable = 1;
module_param_named(crash_handle_enable, crash_handle_enable,
				   int, S_IRUGO | S_IWUSR | S_IWGRP);

/* LGE_UPDATE_S kideok.kim@lge.com 20110717 hidden_reset_porting from Victor. */
#ifdef CONFIG_LGE_HIDDEN_RESET_PATCH
static int hidden_reset_complete(const char *val, struct kernel_param *kp)
{
	unsigned int *copy_addr_rgb888;
	copy_addr_rgb888 = lge_get_fb_copy_virt_rgb888_addr();
	kfree(copy_addr_rgb888);

	printk(KERN_INFO "hidden reset complete %d\n", on_hidden_reset);
	
	return 0;
}
module_param_call(hidden_reset_complete, hidden_reset_complete, param_get_bool, &dummy_arg, S_IWUSR | S_IRUGO);

int hidden_reset_enable = 0;
module_param_named(
		hidden_reset_enable, hidden_reset_enable,
		int, S_IRUGO | S_IWUSR | S_IWGRP);

int on_hidden_reset = 0;
module_param_named(
		on_hidden_reset, on_hidden_reset,
		int, S_IRUGO | S_IWUSR | S_IWGRP);

static int __init check_hidden_reset(char *reset_mode)
{
	if (!strncmp(reset_mode, "on", 2)) {
		on_hidden_reset = 1;
		printk(KERN_INFO"reboot mode: hidden reset %s\n", "on");
	}

	return 1;
}
__setup("lge.hreset=", check_hidden_reset);
#endif
/* LGE_UPDATE_E kideok.kim@lge.com 20110717 hidden_reset_porting from Victor. */

void set_crash_store_enable(void)
{
	crash_store_flag = 1;

	return;
}

void set_crash_store_disable(void)
{
	crash_store_flag = 0;

	return;
}

void store_crash_log(char *p)
{
	if (!crash_store_flag)
		return;

	if (crash_dump_log->size == crash_buf_size)
		return;

	for ( ; *p; p++) {
		if (*p == '[') {
			for ( ; *p != ']'; p++)
				;
			p++;
			if (*p == ' ')
				p++;
		}

		if (*p == '<') {
			for ( ; *p != '>'; p++)
				;
			p++;
		}

		crash_dump_log->buffer[crash_dump_log->size] = *p;
		crash_dump_log->size++;
	}
	crash_dump_log->buffer[crash_dump_log->size] = 0;

	return;
}

/* LGE_UPDATE_S kideok.kim@lge.com 20110717 hidden_reset_porting from Victor. */
#ifdef CONFIG_LGE_HIDDEN_RESET_PATCH
static int copy_fb_phys_addr(void)
{
	void *copy_addr;
	unsigned int fb_phys_addr;

	copy_addr = lge_get_fb_copy_virt_addr();
	fb_phys_addr = (unsigned int)lge_get_fb_phys_addr();
	*((unsigned int *)copy_addr) = fb_phys_addr;
	printk(KERN_INFO"[hidden_reset] fb phisical address %x\n", fb_phys_addr);
	
	return 0;
}

static int hidden_reset_call(struct notifier_block *this, unsigned long event,
		void *ptr)
{
	printk(KERN_INFO"[hidden_reset] start hidden reset..\n");
	copy_fb_phys_addr();
	lge_set_reboot_reason(CRASH_REBOOT);

	return NOTIFY_DONE;
}

static void rgb565_to_rgb888(void)
{
	unsigned short *f;
	unsigned int *copy_addr_rgb888;
	void *copy_addr;
	int i = 0;

	copy_addr = lge_get_fb_copy_virt_addr();
	f = (unsigned short*)copy_addr;
	copy_addr_rgb888 = lge_get_fb_copy_virt_rgb888_addr();

	printk(KERN_INFO"[hidden_reset] convert frame buffer to rgb888\n");

	for (i = 0; i < 800 * 480; i++) {
		*copy_addr_rgb888 = (unsigned int)( ((*f & 0x0000001f) << 19) | ((*f & 0x000007e0) << 5) | ((*f & 0x0000f800) >> 8) | 0xff000000 );
		copy_addr_rgb888++;
		f++;
	}

    memcpy(copy_addr_rgb888, lge_get_fb_copy_virt_rgb888_addr(), sizeof(unsigned int) * 480 * 800);
}
#else
static int restore_crash_log(struct notifier_block *this, unsigned long event,
		void *ptr)
{
	unsigned long flags;

	crash_store_flag = 0;

	spin_lock_irqsave(&lge_panic_lock, flags);

#if 0
	printk(KERN_EMERG"%s", crash_dump_log->buffer);
	printk(KERN_EMERG"%s: buffer size %d\n",__func__, crash_dump_log->size);
	printk(KERN_EMERG"%s: %d\n",__func__, crash_buf_size);
#endif

	lge_set_reboot_reason(CRASH_REBOOT);
	crash_dump_log->magic_key = PANIC_MAGIC_KEY;

	spin_unlock_irqrestore(&lge_panic_lock, flags);

	return NOTIFY_DONE;
}
#endif
/* LGE_UPDATE_E kideok.kim@lge.com 20110717 hidden_reset_porting from Victor. */

static struct notifier_block panic_handler_block = {
/* LGE_UPDATE_S kideok.kim@lge.com 20110717 hidden_reset_porting from Victor. */
#ifdef	CONFIG_LGE_HIDDEN_RESET_PATCH
	.notifier_call  = hidden_reset_call,
#else
	.notifier_call  = restore_crash_log,
#endif
/* LGE_UPDATE_E kideok.kim@lge.com 20110717 hidden_reset_porting from Victor. */
};

static int __init lge_panic_handler_probe(struct platform_device *pdev)
{
	struct resource *res = pdev->resource;
	size_t start;
	size_t buffer_size;
	void *buffer;
	int ret = 0;

	if (res == NULL || pdev->num_resources != 1 ||
			!(res->flags & IORESOURCE_MEM)) {
		printk(KERN_ERR "lge_panic_handler: invalid resource, %p %d flags "
				"%lx\n", res, pdev->num_resources, res ? res->flags : 0);
		return -ENXIO;
	}

	buffer_size = res->end - res->start + 1;
	start = res->start;
	printk(KERN_INFO "lge_panic_handler: got buffer at %zx, size %zx\n",
			start, buffer_size);
	buffer = ioremap(res->start, buffer_size);
	if (buffer == NULL) {
		printk(KERN_ERR "lge_panic_handler: failed to map memory\n");
		return -ENOMEM;
	}

	crash_dump_log = (struct crash_log_dump *)buffer;
	memset(crash_dump_log, 0, buffer_size);
	crash_dump_log->magic_key = 0;
	crash_dump_log->size = 0;
	crash_buf_size = buffer_size - offsetof(struct crash_log_dump, buffer);

	/* Setup panic notifier */
	atomic_notifier_chain_register(&panic_notifier_list, &panic_handler_block);

/* LGE_UPDATE_S kideok.kim@lge.com 20110717 hidden_reset_porting from Victor. */
#ifdef CONFIG_LGE_HIDDEN_RESET_PATCH
	if (on_hidden_reset)
		rgb565_to_rgb888();
#endif
/* LGE_UPDATE_E kideok.kim@lge.com 20110717 hidden_reset_porting from Victor. */

	return ret;
}

static int __devexit lge_panic_handler_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver panic_handler_driver __refdata = {
	.probe = lge_panic_handler_probe,
	.remove = __devexit_p(lge_panic_handler_remove),
	.driver = {
		.name = PANIC_HANDLER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init lge_panic_handler_init(void)
{
	return platform_driver_register(&panic_handler_driver);
}

static void __exit lge_panic_handler_exit(void)
{
	platform_driver_unregister(&panic_handler_driver);
}

module_init(lge_panic_handler_init);
module_exit(lge_panic_handler_exit);

MODULE_DESCRIPTION("LGE panic handler driver");
MODULE_AUTHOR("SungEun Kim <cleaneye.kim@lge.com>");
MODULE_LICENSE("GPL");
