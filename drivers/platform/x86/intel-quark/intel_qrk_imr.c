/*
 * Copyright(c) 2014 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
/*
 * Intel Quark X1000 IMR driver
 *
 * IMR stands for Isolated Memory Region.
 *
 * A total number of 8 IMRs are supported Intel Quark X1000 and are used
 * as part of the secure boot flow to ensure software that has been
 * authenticated cannot be modified by any system agent other than the
 * core.
 *
 * Some of these IMRs are occupied by BIOS and Linux run-time code section.
 *
 * To view current IMR status, user performs:
 *   $ cat /sys/devices/platform/intel-qrk-imr/stat
 *
 * To allocate an IMR, the low & high addresses are alinged to 1k.
 *
 * The IMR alloc API will locate the next available IMR slot set up
 * with input memory egion, then apply the input access right masks
 *
 * The IMR can be freed with the pre-allocated memory addresses.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <asm-generic/uaccess.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/kallsyms.h>
#include <asm/iosf_mbi.h>
#include <asm/cpu_device_id.h>
#include "intel_qrk_imr.h"


// ToDo: to remove this platform driver model
#define DRIVER_NAME	"intel-qrk-imr"

/* PCI ID for Intel Quark Host Bridge */
#define PCI_DEVICE_ID_X1000_HOST_BRIDGE  0x0958

/* Host Bridge - Memory Manager MBI interface */
#define IMR_MBI_ID		QRK_MBI_UNIT_MM
#define IMR_READ_OPCODE		QRK_MBI_MM_READ
#define IMR_WRITE_OPCODE	QRK_MBI_MM_WRITE

/*
 * Each IMR is described by a set of 4 registers
 * with following offset
 */
#define IMR0L			0x40
#define IMR0H			0x41
#define IMR0RM			0x42
#define IMR0WM			0x43
#define IMR1L			0x44
#define IMR1H			0x45
#define IMR1RM			0x46
#define IMR1WM			0x47
#define IMR2L			0x48
#define IMR2H			0x49
#define IMR2RM			0x4A
#define IMR2WM			0x4B
#define IMR3L			0x4C
#define IMR3H			0x4D
#define IMR3RM			0x4E
#define IMR3WM			0x4F
#define IMR4L			0x50
#define IMR4H			0x51
#define IMR4RM			0x52
#define IMR4WM			0x53
#define IMR5L			0x54
#define IMR5H			0x55
#define IMR5RM			0x56
#define IMR5WM			0x57
#define IMR6L			0x58
#define IMR6H			0x59
#define IMR6RM			0x5A
#define IMR6WM			0x5B
#define IMR7L			0x5C
#define IMR7H			0x5D
#define IMR7RM			0x5E
#define IMR7WM			0x5F

/* To lock an IMR through IMRxL register */
#define IMRXL_LOCK_BIT		0x80000000

/* IMR low & high address */
#define IMR_ADDR_MASK		0xFFFFFC
#define IMR_ADDR_SHIFT		0x8
#define IMR_DEFAULT_ADDR	0x000000
/* IMR is 1-k alignment */
#define IMR_MEM_ALIGN		0x400

/* IMR write mask */
#define IMR_WM_ESRAM_FLUSH_INIT_EN	0x80000000
#define IMR_WM_CORE_SNOOP_EN		0x40000000
#define IMR_WM_PUNIT_EN			0x20000000
#define IMR_WM_VC1_EN			0x0000F000
#define IMR_WM_VC0_EN			0x00000F00
#define IMR_WM_CPU0_EN			0x00000002
#define IMR_WM_CPU_0_EN			0x00000001
/* IMR write mask default value */
#define IMR_WM_ALL_EN			0xFFFFFFFF

/* IMR write default mask */
#define IMR_WM_DEFAULT_MASK	(IMR_WM_ESRAM_FLUSH_INIT_EN | \
				 IMR_WM_CORE_SNOOP_EN | \
				 IMR_WM_CPU0_EN | \
				 IMR_WM_CPU_0_EN)

/* IMR read mask */
#define IMR_RM_ESRAM_FLUSH_INIT_EN	0x80000000
#define IMR_RM_PUNIT_EN			0x20000000
#define IMR_RM_VC1_EN			0x0000F000
#define IMR_RM_VC0_EN			0x00000F00
#define IMR_RM_CPU0_EN			0x00000002
#define IMR_RM_CPU_0_EN			0x00000001
/* IMR read mask default value */
#define IMR_RM_ALL_EN			0xBFFFFFFF

/* IMR read default mask */
#define IMR_RM_DEFAULT_MASK	(IMR_RM_ESRAM_FLUSH_INIT_EN | \
				 IMR_RM_CPU0_EN | \
				 IMR_RM_CPU_0_EN)

/* IMR description max char length */
#define IMR_INFO_MAX_LEN	64

#define TRUE	1
#define FALSE	0

/*
 * IMR Register-set Address Offset
 */
struct imr_reg_addr_t {
	u8 imr_xl;	/* high address offset */
	u8 imr_xh;	/* low address offset */
	u8 imr_rm;	/* read mask offset */
	u8 imr_wm;	/* write mask offset */
};

/*
 * IMR Register-set Value
 */
struct imr_reg_t {
	u32 addr_low;	/* low address register */
	u32 addr_high;	/* high address register */
	u32 read_mask;	/* read mask register */
	u32 write_mask;	/* write mask register */
};

/*
 * Per-IMR locally cached context
 */
struct imr_context {
	bool occupied;			/* IMR occupied */
	bool locked;			/* IMR lock */
	struct imr_reg_t imr_reg;	/* IMR register set value */
	char info[IMR_INFO_MAX_LEN]; 	/* IMR info */
};

/* IMR Register-set offset */
static struct imr_reg_addr_t imr_reg_addr[] = {
	{ IMR0L, IMR0H, IMR0RM, IMR0WM },
	{ IMR1L, IMR1H, IMR1RM, IMR1WM },
	{ IMR2L, IMR2H, IMR2RM, IMR2WM },
	{ IMR3L, IMR3H, IMR3RM, IMR3WM },
	{ IMR4L, IMR4H, IMR4RM, IMR4WM },
	{ IMR5L, IMR5H, IMR5RM, IMR5WM },
	{ IMR6L, IMR6H, IMR6RM, IMR6WM },
	{ IMR7L, IMR7H, IMR7RM, IMR7WM }
};

/* ToDo: remove platform driver model */
static struct platform_device *pdev;

/* local IMR context */
static struct imr_context local_imr[IMR_MAX_ID];

static bool imr_init = FALSE;

static int imr_lock = 1;
module_param(imr_lock, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(imr_lock, "Lock unpopulated IMR: 1 for Yes (default) or 0 for No");


/****************************************************************************
 * Helper Functions
 ****************************************************************************/

/*
 * IOSF MBI helper read/write function for IMR register access
 */
static inline u32 imr_reg_read(u8 reg)
{
	u32 val = 0;

	iosf_mbi_read(IMR_MBI_ID, IMR_READ_OPCODE, reg, &val);
	return val;
}

static inline void imr_reg_write(u8 reg, u32 val)
{
	iosf_mbi_write(IMR_MBI_ID, IMR_WRITE_OPCODE, reg, val);
}

/*
 * Prepare IMR low & high address to be 1-KB aligned.
 */
static inline uint32_t imr_addr_prepare(uint32_t addr)
{
	return ((addr >> IMR_ADDR_SHIFT) & IMR_ADDR_MASK);
}

/*
 * Convert register format to physical address format.
 */
static inline uint32_t imr_addr_get(uint32_t reg)
{
	return ((reg & IMR_ADDR_MASK) << IMR_ADDR_SHIFT);
}

/*
 * Latch a IMR HW context by ID
 */
static void imr_hwctxt_latch_by_id(int id)
{
	struct imr_context *lo = &local_imr[id];

	lo->imr_reg.addr_low =
		imr_reg_read(imr_reg_addr[id].imr_xl);
	lo->imr_reg.addr_high =
		imr_reg_read(imr_reg_addr[id].imr_xh);
	lo->imr_reg.read_mask =
		imr_reg_read(imr_reg_addr[id].imr_rm);
	lo->imr_reg.write_mask =
		imr_reg_read(imr_reg_addr[id].imr_wm);

	if (lo->imr_reg.addr_low & IMRXL_LOCK_BIT)
		lo->locked = true;

	/*
	 * IMR is not occupied if low & high addresses
	 * are both zero (default value after power-cycle)
	 */
	if (!lo->imr_reg.addr_low &&
	    !lo->imr_reg.addr_high)
		lo->occupied = false;
	else
		lo->occupied = true;
}

/*
 * Latch All IMRs HW context locally
 */
static void imr_hwctxt_latch_all(void)
{
	int id;

	for (id = 0; id < IMR_MAX_ID; id++)
		imr_hwctxt_latch_by_id(id);
}

/*
 * Initialize IMR HW context structures
 */
static void imr_hwctxt_init(void)
{
	int id;
	char *info = "System Reserved Region\0";

	imr_hwctxt_latch_all();

	for (id = 0; id < IMR_MAX_ID; id++)
		memcpy(local_imr[id].info, info, IMR_INFO_MAX_LEN);
}

/*
 * Find the next free IMR entry
 */
static int imr_free_entry_find(void)
{
	int id;

	imr_hwctxt_latch_all();

	for (id = 0; id < IMR_MAX_ID; id++) {
		struct imr_context *lo = &local_imr[id];
		if ((!lo->occupied) && (!lo->locked))
			return id;
	}

	pr_err("No more free IMR available.\n");
	return -ENOMEM;
}

/*
 * Setup a un-locked IMR and lock it if needed.
 * Caller should ensure the following:
 *   @id is within valid range.
 *   @hi & @lo are parsed by imr_addr_prepare()
 */
static void imr_entry_add(int id, uint32_t hi, uint32_t lo,
			  uint32_t read_mask, uint32_t write_mask, bool lock)
{
	imr_reg_write(imr_reg_addr[id].imr_xh, hi);
	imr_reg_write(imr_reg_addr[id].imr_rm, (read_mask & IMR_RM_ALL_EN));
	imr_reg_write(imr_reg_addr[id].imr_wm, (write_mask & IMR_WM_ALL_EN));

	if (lock) {
		/*
		 * IMR once locked is good until next power cycle.
		 * So, imr_xl is last to be written.
		 */
		lo |= IMRXL_LOCK_BIT;
	}
	imr_reg_write(imr_reg_addr[id].imr_xl, lo);
}

/*
 * Remove an IMR entry which is not locked.
 * Caller should ensure @id is within valid range and IMT is unlocked.
 */
static void imr_entry_remove(int id)
{
	imr_reg_write(imr_reg_addr[id].imr_rm, IMR_RM_ALL_EN);
	imr_reg_write(imr_reg_addr[id].imr_wm, IMR_WM_ALL_EN);
	imr_reg_write(imr_reg_addr[id].imr_xl, IMR_DEFAULT_ADDR);
	imr_reg_write(imr_reg_addr[id].imr_xh, IMR_DEFAULT_ADDR);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*
 * Setup the next available IMR with customized read and write masks
 * Note:
 *   IMR protection is 1-KB aligned, if input range (high,low) are not
 *   1-KB aligned, IMR takes effect beyond the specified non-aligned
 *   address range.
 *   If input range (high, low) are both consecutively 1-KB aligned, IMR
 *   protection covers 2-KB between low and high+0x3FF inclusive.
 */
int intel_qrk_imr_alloc(uint32_t high, uint32_t low,
			uint32_t read_mask, uint32_t write_mask,
			char *info, bool lock)
{
	int low_addr, high_addr, id = 0;

	if (!imr_init)
		return -ENODEV;

	/* Ensure input arguments are sane */
	if ( (info == NULL) || !read_mask || !write_mask )
		return -EINVAL;

	/*
	 * Ensure low-address is not having lock-bit set and cause
	 * conflicts with 'lock' input
	 */
	if (low & IMRXL_LOCK_BIT) {
		pr_err("IMR low address has lock-bit set\n");
		return -EINVAL;
	}

	/* Calculate aligned addresses and validate range */
	high_addr = imr_addr_prepare(high);
	low_addr = imr_addr_prepare(low);

	/* Ensure IMR addresses are sane */
	if (high_addr < low_addr) {
		pr_err("IMR low(0x%x) & high(0x%x) addr are invalid \n",
			low, high);
		return -EINVAL;
	}

	/* Find a free entry */
	id = imr_free_entry_find();
	if (id < 0)
		return -ENOMEM;

	/* Add entry - locking as necessary */
	imr_entry_add(id, high_addr, low_addr, read_mask, write_mask, lock);

	/* Update IMR context info */
	memcpy(local_imr[id].info, info, IMR_INFO_MAX_LEN);
	imr_hwctxt_latch_by_id(id);

	return 0;
}
EXPORT_SYMBOL(intel_qrk_imr_alloc);

/*
 * Remove IMR based on ID
 */
int intel_qrk_imr_free_by_id(int id)
{
	if (!imr_init)
		return -ENODEV;

	if (id >= IMR_MAX_ID)
		return -EINVAL;

	if (local_imr[id].locked) {
		pr_err("IMR(id=%d) is locked & cannot be freed.\n", id);
		return -EPERM;
	}

	imr_entry_remove(id);

	imr_hwctxt_latch_by_id(id);

	return 0;
}
EXPORT_SYMBOL(intel_qrk_imr_free_by_id);

/*
 * Remove IMR based on input memory region
 */
int intel_qrk_imr_free_by_addr(uint32_t high, uint32_t low)
{
	int low_addr, high_addr, id;

	if (!imr_init)
		return -ENODEV;

	if (!high && !low)
		return -EINVAL;

	high_addr = imr_addr_prepare(high);
	low_addr = imr_addr_prepare(low);

	/* Ensure IMR addresses are sane */
	if (!high_addr || high_addr < low_addr) {
		pr_err("IMR low(0x%x) & high(0x%x) addr are invalid \n",
			low, high);
		return -EINVAL;
	}

	for (id = 0; id < IMR_MAX_ID; id++) {
		struct imr_context *lo = &local_imr[id];
		if (lo->occupied &&
		    !lo->locked &&
		    (lo->imr_reg.addr_low == low_addr) &&
		    (lo->imr_reg.addr_high == high_addr)) {
			intel_qrk_imr_free_by_id(id);
			return 0;
		}
	}

	pr_err("IMR failed to be freed. \n");
	return -EINVAL;
}
EXPORT_SYMBOL(intel_qrk_imr_free_by_addr);

/*
 * Lock up all un-locked IMRs
 */
void intel_qrk_imr_lockall(void)
{
	int low_addr, id;

	imr_hwctxt_latch_all();

	/* Cycle through IMRs locking whichever are unlocked */
	for (id = 0; id < IMR_MAX_ID; id++) {
		low_addr = local_imr[id].imr_reg.addr_low;
		if (!(low_addr & IMRXL_LOCK_BIT)) {
			low_addr |= IMRXL_LOCK_BIT;
			imr_reg_write(imr_reg_addr[id].imr_xl, low_addr);
		}
	}
}
EXPORT_SYMBOL(intel_qrk_imr_lockall);

/****************************************************************************
 * SYS-FS interface
 ****************************************************************************/

/*
 * Populates IMR state via /sys/device/intel-qrk-imr/stat
 */
static ssize_t intel_qrk_imr_stat_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	uint32_t hi_phy_addr, lo_phy_addr;
	int id,  size, len = 0, count = PAGE_SIZE;

	for (id = 0; id < IMR_MAX_ID; id++) {
		struct imr_context *lo = &local_imr[id];

		/* Extract IMR low & high addresses */
		hi_phy_addr = imr_addr_get(lo->imr_reg.addr_high);
		lo_phy_addr = imr_addr_get(lo->imr_reg.addr_low);

		/*
		 * IMR upper boundary is effectively 1-KB above the specified
		 * IMR high address, so we need to add 1-KB here.
		 */
		size = ((hi_phy_addr - lo_phy_addr) / IMR_MEM_ALIGN) + 1;

		size = snprintf(buf+len, count,
				"imr - id : %d\n"
				"info     : %s\n"
				"occupied : %s\n"
				"locked   : %s\n"
				"size     : %d kB\n"
				"hi addr (phy): 0x%08x\n"
				"lo addr (phy): 0x%08x\n"
				"hi addr (vir): 0x%08x\n"
				"lo addr (vir): 0x%08x\n"
				"read mask  : 0x%08x\n"
				"write mask : 0x%08x\n\n",
				id,
				lo->info,
				lo->occupied ? "yes" : "no",
				lo->locked ? "yes" : "no",
				size,
				hi_phy_addr,
				lo_phy_addr,
				(uint32_t)phys_to_virt(hi_phy_addr),
				(uint32_t)phys_to_virt(lo_phy_addr),
				lo->imr_reg.read_mask,
				lo->imr_reg.write_mask);
		len += size;
		count -= size;
	}
	return len;
}

static struct device_attribute dev_attr_stats = {
	.attr = {
		.name = "stat",
		.mode = 0444, },
	.show = intel_qrk_imr_stat_show,
};

static struct attribute *platform_attributes[] = {
	&dev_attr_stats.attr,
	NULL,
};

static struct attribute_group imr_attrib_group = {
	.attrs = platform_attributes
};

/****************************************************************************
 * Module initialization
 ****************************************************************************/

static __init void intel_qrk_imr_runt_kerndata_setup(void)
{
	uint32_t hi;
	uint32_t lo;

	hi = (uint32_t) virt_to_phys(kallsyms_lookup_name("__init_begin"));
	lo = (uint32_t) virt_to_phys(kallsyms_lookup_name("_text"));

	/*
	 * IMR protected zone from _text upto __init_begin. As IMRxH sets
	 * the base address of the upper boundary, therefore minus 1-K here.
	 */
	if (!intel_qrk_imr_alloc((hi - IMR_MEM_ALIGN), lo,
				IMR_RM_DEFAULT_MASK, IMR_WM_DEFAULT_MASK,
				"KERNEL RUNTIME DATA", 1))
		pr_err("IMR: Fail to lock run-time kernel data IMR !\n");
}

static const struct x86_cpu_id soc_imr_ids[] = {
	{ X86_VENDOR_INTEL, 5, 9}, /* Intel Quark SoC X1000 */
	{}
};
MODULE_DEVICE_TABLE(x86cpu, soc_imr_ids);

static int __init intel_qrk_imr_init(void)
{
	int ret, id;

	if (!x86_match_cpu(soc_imr_ids) || !iosf_mbi_available())
	{
		pr_info("IMR init failed due to IOSF_MBI not available. \n");
		return -ENODEV;
	}

	// ToDo: to check if platform driver model is acceptable
	pdev = platform_device_alloc(DRIVER_NAME, -1);
	if (!pdev)
		return -ENOMEM;

	ret = platform_device_add(pdev);
	if (ret)
		goto fail_platform;

	ret = sysfs_create_group(&pdev->dev.kobj, &imr_attrib_group);
	if (ret)
		goto fail_platform;

	/* initialise local imr data structure */
	imr_hwctxt_init();
	imr_init = TRUE;

	/* Lock-down kernel run-time memory access */
	intel_qrk_imr_runt_kerndata_setup();

	/* Make sure unlocked IMR is not-populated  */
	for (id = 0; id < IMR_MAX_ID; id++)
		intel_qrk_imr_free_by_id(id);

	/* User may (un)lock all unpopulated IMR by default */
	if (imr_lock)
		intel_qrk_imr_lockall();

	return 0;

fail_platform:
	platform_device_del(pdev);
	return ret;
}

static void __exit intel_qrk_imr_exit(void)
{
	if (pdev)
		platform_device_del(pdev);
	pdev = NULL;
}

module_init(intel_qrk_imr_init)
module_exit(intel_qrk_imr_exit)

MODULE_DESCRIPTION("Intel Quark IMR Driver");
MODULE_LICENSE("GPL v2");
