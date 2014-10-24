/*
 * Copyright(c) 2013 Intel Corporation.
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
 * Intel Quark IMR driver
 *
 * IMR stands for Isolated Memory Region.
 *
 * A total of 8 IMRs are implemented by Intel Quark SoC X1000 where
 * some are pre-occupied by BIOS or Linux run-time kernel.
 *
 * The IMR alloc API will locate the next available IMR slot set up
 * with input memory region, and apply with the default access right
 * (CPU & CPU_snoop enable).
 *
 * The alloc_mask API takes input read & write masks values to set up
 * IMR with customized access right.
 *
 * User can free IMR with pre-alloc specified addresses.
 */

#ifndef __INTEL_QRK_IMR_H__
#define __INTEL_QRK_IMR_H__

/* pre-defined imr id for uncompressed kernel */
#define IMR_KERNEL_ID	3

/* Intel Quark supports maximum 8 IMRs */
#define IMR_MAX_ID	8

/*
 * intel_qrk_imr_alloc
 *
 * @param high: the end of physical memory address
 * @param low: the start of physical memory address
 * @param read: IMR read mask value
 * @param write: IMR write maks value
 * @param info: imr information
 * @param lock: imr lock
 *
 * Setup imr with customised read/ write masks
 */
int intel_qrk_imr_alloc(uint32_t high, uint32_t low,
			uint32_t read_mask, uint32_t write_mask,
			char *info, bool lock);

/**
 * intel_qrk_imr_free_by_addr
 *
 * @param high: high boundary of memory address
 * @param low: low boundary of memorry address
 *
 * remove the imr based on input memory region
 */
int intel_qrk_imr_free_by_addr(uint32_t high, uint32_t low);

/**
 * intel_qrk_imr_free_by_id
 *
 * @param id: IMR ID returned from intel_qrk_imr_lloc()
 *
 * Remove imr based on input imr data structure id
 */
int intel_qrk_imr_free_by_id(int id);

/**
 * intel_qrk_imr_lockall
 *
 * Lock all IMRs that are not locked yet.
 *
 * Note:
 *  - Caller can also lock single IMR by using intel_qrk_imr_alloc().
 *  - Caller should use intel_qrk_imr_free_by_addr|id() to remove invalid
 *    IMR before calling intel_qrk_imr_lockall function.
 */
void intel_qrk_imr_lockall(void);

#endif
