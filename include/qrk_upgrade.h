/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright (c) 2026 ninozhang001@gmail.com */

#ifndef __QRK_UPGRADE_H
#define __QRK_UPGRADE_H

#include <linux/types.h>

#define QRK_UPGRADE_HEADER_SIZE 512
#define QRK_UPGRADE_COMPONENT_MAX 4
#define QRK_UPGRADE_SHA256_HEX_LEN 64
#define QRK_UPGRADE_BUFFER_SIZE (288UL * 1024 * 1024)

/* Fixed byte offsets in the eMMC user area; never supplied by the image. */
#define QRK_UPGRADE_SECTOR_SIZE 512
#define QRK_UPGRADE_FIP_OFFSET 0x00107200ULL
#define QRK_UPGRADE_FIP_SIZE (20ULL * 1024 * 1024)
#define QRK_UPGRADE_KERNEL_OFFSET 0x02000000ULL
#define QRK_UPGRADE_KERNEL_SIZE (120ULL * 1024 * 1024)
#define QRK_UPGRADE_DTB_OFFSET 0x09800000ULL
#define QRK_UPGRADE_DTB_SIZE (2ULL * 1024 * 1024)
#define QRK_UPGRADE_ROOTFS_OFFSET 0x0a000000ULL
#define QRK_UPGRADE_ROOTFS_SIZE (128ULL * 1024 * 1024)

enum qrk_upgrade_component_id {
	QRK_UPGRADE_ROOTFS,
	QRK_UPGRADE_KERNEL,
	QRK_UPGRADE_FIP,
	QRK_UPGRADE_DTB,
	QRK_UPGRADE_COMPONENT_COUNT,
};

struct qrk_upgrade_component {
	enum qrk_upgrade_component_id id;
	u64 offset;
	u64 size;
	char sha256[QRK_UPGRADE_SHA256_HEX_LEN + 1];
};

struct qrk_upgrade_image {
	struct qrk_upgrade_component components[QRK_UPGRADE_COMPONENT_MAX];
	u32 component_count;
	char plan_id[QRK_UPGRADE_SHA256_HEX_LEN + 1];
};

int qrk_upgrade_image_parse(const void *buffer, size_t size, bool xmodem,
			    struct qrk_upgrade_image *image);
const char *qrk_upgrade_component_name(enum qrk_upgrade_component_id id);
int qrk_upgrade_target_range(enum qrk_upgrade_component_id id, u64 sectors,
			     u64 *start, u64 *capacity);

#endif
