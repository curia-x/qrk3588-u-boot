// SPDX-License-Identifier: GPL-2.0-or-later
/* Copyright (c) 2026 ninozhang001@gmail.com */

#include <qrk_upgrade.h>
#include <asm/unaligned.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <u-boot/sha256.h>

static void digest_hex(const u8 digest[32], char output[65])
{
	unsigned int i;

	for (i = 0; i < 32; i++) {
		output[2 * i] = "0123456789abcdef"[digest[i] >> 4];
		output[2 * i + 1] = "0123456789abcdef"[digest[i] & 15];
	}
	output[64] = '\0';
}

const char *qrk_upgrade_component_name(enum qrk_upgrade_component_id id)
{
	static const char *const names[] = { "rootfs", "kernel", "fip", "dtb" };

	return id < ARRAY_SIZE(names) ? names[id] : "invalid";
}

int qrk_upgrade_target_range(enum qrk_upgrade_component_id id, u64 sectors,
			     u64 *start, u64 *capacity)
{
	switch (id) {
	case QRK_UPGRADE_ROOTFS:
		*start = QRK_UPGRADE_ROOTFS_OFFSET / QRK_UPGRADE_SECTOR_SIZE;
		*capacity = QRK_UPGRADE_ROOTFS_SIZE / QRK_UPGRADE_SECTOR_SIZE;
		break;
	case QRK_UPGRADE_KERNEL:
		*start = QRK_UPGRADE_KERNEL_OFFSET / QRK_UPGRADE_SECTOR_SIZE;
		*capacity = QRK_UPGRADE_KERNEL_SIZE / QRK_UPGRADE_SECTOR_SIZE;
		break;
	case QRK_UPGRADE_DTB:
		*start = QRK_UPGRADE_DTB_OFFSET / QRK_UPGRADE_SECTOR_SIZE;
		*capacity = QRK_UPGRADE_DTB_SIZE / QRK_UPGRADE_SECTOR_SIZE;
		break;
	case QRK_UPGRADE_FIP:
		*start = QRK_UPGRADE_FIP_OFFSET / QRK_UPGRADE_SECTOR_SIZE;
		*capacity = QRK_UPGRADE_FIP_SIZE / QRK_UPGRADE_SECTOR_SIZE;
		break;
	default:
		return -EINVAL;
	}
	/* Subtraction avoids overflow for an untrusted device capacity. */
	if (*start > sectors || *capacity > sectors - *start)
		return -EINVAL;
	return 0;
}

int qrk_upgrade_image_parse(const void *buffer, size_t size, bool xmodem,
			    struct qrk_upgrade_image *image)
{
	const u8 *data = buffer;
	bool selected[QRK_UPGRADE_COMPONENT_COUNT] = {};
	u8 digest[32];
	u64 total, offset = QRK_UPGRADE_HEADER_SIZE;
	unsigned int i, j;

	memset(image, 0, sizeof(*image));
	if (size < QRK_UPGRADE_HEADER_SIZE ||
	    memcmp(data, "QRKUPG01", 8) ||
	    get_unaligned_le32(data + 8) != QRK_UPGRADE_HEADER_SIZE)
		return -EINVAL;
	image->component_count = get_unaligned_le32(data + 12);
	total = get_unaligned_le64(data + 16);
	if (!image->component_count || image->component_count > QRK_UPGRADE_COMPONENT_MAX ||
	    total < QRK_UPGRADE_HEADER_SIZE ||
	    total > QRK_UPGRADE_BUFFER_SIZE || total > size)
		return -EINVAL;
	if ((!xmodem && size != total) ||
	    (xmodem && size != total && size != ALIGN(total, 128) &&
	     size != ALIGN(total, 1024)))
		return -EINVAL;
	for (i = total; i < size; i++)
		if (data[i] != 0x1a)
			return -EINVAL;
	/* Fixed product/layout tag; the unused header bytes must be zero. */
	if (memcmp(data + 24, "qrk3588:raw-v1", 14))
		return -EINVAL;
	for (i = 38; i < 64; i++)
		if (data[i])
			return -EINVAL;
	for (i = 64 + image->component_count * 56; i < 512; i++)
		if (data[i])
			return -EINVAL;
	for (i = 0; i < image->component_count; i++) {
		const u8 *entry = data + 64 + i * 56;
		struct qrk_upgrade_component *component = &image->components[i];
		u32 id = get_unaligned_le32(entry);
		u64 rounded;

		if (id >= QRK_UPGRADE_COMPONENT_COUNT || selected[id] ||
		    get_unaligned_le32(entry + 4))
			return -EINVAL;
		selected[id] = true;
		component->id = id;
		component->offset = get_unaligned_le64(entry + 8);
		component->size = get_unaligned_le64(entry + 16);
		if (!component->size || component->offset != offset ||
		    (id == QRK_UPGRADE_KERNEL && component->size > QRK_UPGRADE_KERNEL_SIZE) ||
		    (id == QRK_UPGRADE_DTB && component->size > QRK_UPGRADE_DTB_SIZE) ||
		    (id == QRK_UPGRADE_ROOTFS &&
		     (component->size > QRK_UPGRADE_ROOTFS_SIZE || (component->size & 511))) ||
		    (id == QRK_UPGRADE_FIP && component->size > QRK_UPGRADE_FIP_SIZE))
			return -EINVAL;
		rounded = ALIGN(component->size, 512);
		if (offset > total || rounded > total - offset)
			return -EINVAL;
		sha256_csum_wd(data + offset, component->size, digest, 1024 * 1024);
		if (memcmp(digest, entry + 24, 32))
			return -EBADMSG;
		digest_hex(digest, component->sha256);
		for (j = component->size; j < rounded; j++)
			if (data[offset + j])
				return -EINVAL;
		offset += rounded;
	}
	if (offset != total)
		return -EINVAL;
	sha256_csum_wd(data, total, digest, 1024 * 1024);
	digest_hex(digest, image->plan_id);
	return 0;
}
