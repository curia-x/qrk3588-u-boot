// SPDX-License-Identifier: GPL-2.0-or-later
/* Copyright (c) 2026 ninozhang001@gmail.com */

#include <blk.h>
#include <command.h>
#include <dm/read.h>
#include <env.h>
#include <lmb.h>
#include <malloc.h>
#include <mapmem.h>
#include <mmc.h>
#include <net-common.h>
#include <xyzModem.h>
#include <stdio.h>
#include <qrk_upgrade.h>
#include <asm/cache.h>
#include <asm/global_data.h>
#include <linux/ctype.h>
#include <linux/errno.h>
#include <linux/sizes.h>
#include <u-boot/sha256.h>

DECLARE_GLOBAL_DATA_PTR;

#define QRK_EMMC_DEVICE		0
#define QRK_EMMC_USER_PART	0
#define QRK_EMMC_BASE		0xfe2e0000ULL
#define QRK_BLOCK_SIZE		QRK_UPGRADE_SECTOR_SIZE

#define QRK_STAGE_BASE		0x40000000UL
#define QRK_STAGE_SIZE		QRK_UPGRADE_BUFFER_SIZE
#define QRK_READBACK_CHUNK	SZ_1M

/* JEDEC JESD84 EXT_CSD byte indices. */
#define QRK_EXT_CSD_FLUSH_CACHE	32
#define QRK_EXT_CSD_CACHE_CTRL	33

struct qrk_target {
	struct mmc *mmc;
	struct blk_desc *desc;
};

enum qrk_mode {
	QRK_MODE_CHECK,
	QRK_MODE_APPLY,
	QRK_MODE_RECOVER,
};

extern int mmc_switch(struct mmc *mmc, u8 set, u8 index, u8 value);

static bool qrk_env_valid(void)
{
	return gd->env_valid == ENV_VALID || gd->env_valid == ENV_REDUND;
}

static bool qrk_safe_locator(const char *value)
{
	const char *p;

	if (!value || !value[0] || value[0] == '/' || strstr(value, ".."))
		return false;
	for (p = value; *p; p++)
		if (!isalnum(*p) && *p != '-' && *p != '_' &&
		    *p != '.' && *p != '/')
			return false;
	return true;
}

static int qrk_persist_barrier(struct qrk_target *target)
{
	if (!target->mmc->ext_csd)
		return -EIO;
	if (!(target->mmc->ext_csd[QRK_EXT_CSD_CACHE_CTRL] & 1))
		return 0;
	return mmc_switch(target->mmc, EXT_CSD_CMD_SET_NORMAL,
			  QRK_EXT_CSD_FLUSH_CACHE, 1);
}

static int qrk_set_state(struct qrk_target *target, const char *state,
			 const struct qrk_upgrade_image *image,
			 const char *component, const char *error)
{
	int ret;

	ret = env_set("upgrade_state", state);
	ret |= env_set("upgrade_plan_id", image->plan_id);
	ret |= env_set("upgrade_component", component);
	ret |= env_set("upgrade_error", error);
	if (ret || env_save())
		return -ENOSPC;
	return qrk_persist_barrier(target) ? -EUCLEAN : 0;
}

static void qrk_mark_failed(struct qrk_target *target,
			    const struct qrk_upgrade_image *image,
			    const char *component, const char *error)
{
	if (qrk_set_state(target, "failed", image, component, error))
		puts("qrk_upgrade: warning: failed-state metadata could not be persisted\n");
}

static int qrk_open_target(struct qrk_target *target)
{
	fdt_addr_t address;
	int ret;

	memset(target, 0, sizeof(*target));
	target->mmc = find_mmc_device(QRK_EMMC_DEVICE);
	if (!target->mmc)
		return -ENODEV;
	ret = mmc_init(target->mmc);
	if (ret)
		return ret;
	address = dev_read_addr(target->mmc->dev);
	if (address != QRK_EMMC_BASE) {
		printf("qrk_upgrade: mmc%d is controller @%llx, expected @%llx\n",
		       QRK_EMMC_DEVICE, (unsigned long long)address,
		       (unsigned long long)QRK_EMMC_BASE);
		return -ENODEV;
	}
	ret = mmc_switch_part(target->mmc, QRK_EMMC_USER_PART);
	if (ret)
		return ret;
	target->desc = mmc_get_blk_desc(target->mmc);
	if (!target->desc || target->desc->blksz != QRK_BLOCK_SIZE ||
	    !target->desc->lba || target->desc->hwpart != QRK_EMMC_USER_PART)
		return -EINVAL;
	return 0;
}

static int qrk_hash_media(struct blk_desc *desc, lbaint_t start, u64 size,
			  char output[65])
{
	sha256_context context;
	u8 digest[32];
	u8 *buffer;
	u64 offset = 0;
	unsigned int i;
	int ret = 0;

	buffer = memalign(ARCH_DMA_MINALIGN, QRK_READBACK_CHUNK);
	if (!buffer)
		return -ENOMEM;
	sha256_starts(&context);
	while (offset < size) {
		size_t bytes = min_t(u64, size - offset, QRK_READBACK_CHUNK);
		lbaint_t blocks = DIV_ROUND_UP(bytes, desc->blksz);

		if (blk_dread(desc, start + offset / desc->blksz,
			      blocks, buffer) != blocks) {
			ret = -EIO;
			goto out;
		}
		sha256_update(&context, buffer, bytes);
		offset += bytes;
	}
	sha256_finish(&context, digest);
	for (i = 0; i < sizeof(digest); i++) {
		output[i * 2] = "0123456789abcdef"[digest[i] >> 4];
		output[i * 2 + 1] = "0123456789abcdef"[digest[i] & 15];
	}
	output[64] = '\0';
out:
	free(buffer);
	return ret;
}

static int qrk_admit(struct qrk_target *target,
		     const struct qrk_upgrade_image *image,
		     enum qrk_mode mode)
{
	const char *state = env_get("upgrade_state");
	const char *plan = env_get("upgrade_plan_id");
	bool retry = state && (!strcmp(state, "applying") ||
			       !strcmp(state, "failed"));
	bool selected[QRK_UPGRADE_COMPONENT_COUNT] = {};
	unsigned int i;

	(void)target;

	if (!qrk_env_valid())
		return -EPERM;
	for (i = 0; i < image->component_count; i++)
		selected[image->components[i].id] = true;
	if (mode == QRK_MODE_RECOVER &&
	    (!selected[QRK_UPGRADE_ROOTFS] || !selected[QRK_UPGRADE_KERNEL] ||
	     !selected[QRK_UPGRADE_DTB] || !selected[QRK_UPGRADE_FIP]))
		return -EINVAL;
	if (retry) {
		if (mode != QRK_MODE_RECOVER &&
		    (!plan || strcmp(plan, image->plan_id)))
			return -EPERM;
	} else if (state && strcmp(state, "idle") && strcmp(state, "complete")) {
		return -EPERM;
	}
	return 0;
}

static int qrk_tftp(const char *path, void *buffer, size_t capacity,
		    size_t *received)
{
	const char *filesize;
	char *end;
	ulong length;
	int ret;

	tftp_set_receive_range(map_to_sysmem(buffer), capacity);
	ret = run_commandf("tftpboot %lx %s", (ulong)map_to_sysmem(buffer), path);
	tftp_set_receive_range(0, 0);
	if (ret)
		return -EIO;
	filesize = env_get("filesize");
	if (!filesize)
		return -EIO;
	length = hextoul(filesize, &end);
	if (*end || !length || length > capacity)
		return -EFBIG;
	*received = length;
	return 0;
}

static int qrk_getchar(void)
{
	return tstc() ? getchar() : -1;
}

static int qrk_xmodem(void *buffer, size_t capacity, size_t *received)
{
	connection_info_t info = { .mode = xyzModem_xmodem };
	char block[1024];
	size_t length = 0;
	int err = 0, result, close_error;
	bool abort = false;

	puts("Send upgrade image via XMODEM...\n");
	xyzModem_set_binary(true);
	if (xyzModem_stream_open(&info, &err)) {
		xyzModem_set_binary(false);
		return -EIO;
	}
	while ((result = xyzModem_stream_read(block, sizeof(block), &err)) > 0) {
		if (result > capacity - length) {
			abort = true;
			break;
		}
		memcpy((u8 *)buffer + length, block, result);
		length += result;
	}
	if (err)
		abort = true;
	xyzModem_stream_terminate(abort, qrk_getchar);
	xyzModem_stream_close(&close_error);
	xyzModem_set_binary(false);
	if (abort)
		return -EIO;
	*received = length;
	return length ? 0 : -EINVAL;
}

static int qrk_fetch_image(const char *source, const char *path,
			    void *buffer, size_t *size)
{
	if (!strcmp(source, "tftp")) {
		if (!qrk_safe_locator(path) || strlen(path) > 255)
			return -ENAMETOOLONG;
		return qrk_tftp(path, buffer, QRK_STAGE_SIZE, size);
	}
	return qrk_xmodem(buffer, QRK_STAGE_SIZE, size);
}

static int qrk_write_component(struct qrk_target *target,
			       const struct qrk_upgrade_component *component,
			       const void *buffer)
{
	u64 start, capacity;
	lbaint_t blocks;
	char digest[65];
	int ret;

	ret = qrk_upgrade_target_range(component->id, target->desc->lba,
				       &start, &capacity);
	if (ret)
		return ret;
	blocks = DIV_ROUND_UP(component->size, target->desc->blksz);
	if (!blocks || blocks > capacity)
		return -EFBIG;
	if (component->size & (target->desc->blksz - 1))
		memset((u8 *)buffer + component->size, 0,
		       ALIGN(component->size, target->desc->blksz) - component->size);
	if (blk_dwrite(target->desc, start, blocks, buffer) != blocks)
		return -EIO;
	blkcache_invalidate(target->desc->uclass_id, target->desc->devnum);
	ret = qrk_persist_barrier(target);
	if (ret)
		return -EUCLEAN;
	ret = qrk_hash_media(target->desc, start, component->size, digest);
	if (ret)
		return -EREMOTEIO;
	if (strcmp(digest, component->sha256))
		return -EBADMSG;
	return 0;
}

static const struct qrk_upgrade_component *
qrk_component(const struct qrk_upgrade_image *image,
	      enum qrk_upgrade_component_id id)
{
	unsigned int i;

	for (i = 0; i < image->component_count; i++)
		if (image->components[i].id == id)
			return &image->components[i];
	return NULL;
}

static const char *qrk_error_code(int ret)
{
	if (ret == -EBADMSG)
		return "HASH_FAIL";
	if (ret == -EREMOTEIO)
		return "READBACK_FAIL";
	if (ret == -EUCLEAN)
		return "PERSIST_FAIL";
	if (ret == -ENODATA)
		return "MARKER_FAIL";
	if (ret == -ENOSPC)
		return "ENV_SAVE_FAIL";
	if (ret == -EIO)
		return "WRITE_FAIL";
	if (ret == -EINVAL)
		return "TARGET_INVALID";
	return "UPGRADE_FAIL";
}

static int qrk_run(enum qrk_mode mode, const char *source, const char *path)
{
	static const enum qrk_upgrade_component_id order[] = {
		QRK_UPGRADE_ROOTFS, QRK_UPGRADE_KERNEL, QRK_UPGRADE_DTB, QRK_UPGRADE_FIP,
	};
	struct qrk_upgrade_image image;
	struct qrk_target target;
	phys_addr_t address = QRK_STAGE_BASE;
	void *buffer;
	size_t size;
	unsigned int i;
	bool destructive = mode != QRK_MODE_CHECK;
	bool applying = false;
	const char *failure_error = "UPGRADE_FAIL";
	int admission, apply_admission, recover_admission, ret;

	ret = lmb_alloc_mem(LMB_MEM_ALLOC_ADDR, 0, &address,
			    QRK_STAGE_SIZE, LMB_NONE);
	if (ret) {
		puts("qrk_upgrade: staging window is unavailable\n");
		return ret;
	}
	buffer = map_sysmem(QRK_STAGE_BASE, QRK_STAGE_SIZE);
	ret = qrk_fetch_image(source, path, buffer, &size);
	if (ret)
		goto out;
	ret = qrk_upgrade_image_parse(buffer, size, !strcmp(source, "xmodem"), &image);
	if (ret) {
		printf("qrk_upgrade: image rejected: %d\n", ret);
		goto out;
	}
	ret = qrk_open_target(&target);
	if (ret) {
		printf("qrk_upgrade: eMMC target validation failed: %d\n", ret);
		goto out;
	}
	for (i = 0; i < image.component_count; i++) {
		const struct qrk_upgrade_component *component = &image.components[i];
		u64 start, capacity;

		ret = qrk_upgrade_target_range(component->id, target.desc->lba,
					       &start, &capacity);
		if (ret || component->size > capacity * QRK_BLOCK_SIZE) {
			printf("qrk_upgrade: %s fixed range exceeds eMMC/image limits\n",
			       qrk_upgrade_component_name(component->id));
			ret = ret ?: -EFBIG;
			goto out;
		}
		printf("qrk_upgrade: %s offset=0x%llx size=0x%llx\n",
		       qrk_upgrade_component_name(component->id),
		       (unsigned long long)(start * QRK_BLOCK_SIZE),
		       (unsigned long long)component->size);
	}
	if (!qrk_env_valid() && destructive) {
		puts("qrk_upgrade: persistent environment is invalid; destructive operation blocked\n");
		ret = -EPERM;
		goto out;
	}
	apply_admission = qrk_admit(&target, &image, QRK_MODE_APPLY);
	recover_admission = qrk_admit(&target, &image, QRK_MODE_RECOVER);
	admission = mode == QRK_MODE_RECOVER ? recover_admission : apply_admission;
	if (admission && destructive) {
		printf("qrk_upgrade: admission rejected: %d\n", admission);
		ret = admission;
		goto out;
	}

	for (i = 0; i < ARRAY_SIZE(order); i++) {
		const struct qrk_upgrade_component *component;

		component = qrk_component(&image, order[i]);
		if (!component)
			continue;
		if (!destructive)
			continue;
		if (!applying) {
			ret = qrk_set_state(&target, "applying", &image,
					    "none", "");
			if (ret)
				goto fail;
			applying = true;
		}
		ret = qrk_set_state(&target, "applying", &image,
				    qrk_upgrade_component_name(order[i]), "");
		if (ret) {
			failure_error = qrk_error_code(ret);
			goto fail;
		}
		ret = qrk_write_component(&target, component,
					  (u8 *)buffer + component->offset);
		if (ret) {
			failure_error = qrk_error_code(ret);
			printf("qrk_upgrade: %s write/readback failed: %d\n",
			       qrk_upgrade_component_name(order[i]), ret);
			goto fail;
		}
		printf("qrk_upgrade: %s durable and verified\n",
		       qrk_upgrade_component_name(order[i]));
	}
	if (!destructive) {
		printf("qrk_upgrade: package_validation=PASS plan=%s\n",
		       image.plan_id);
		printf("qrk_upgrade: apply_admission=%s (%d)\n",
		       apply_admission ? "BLOCKED" : "PASS", apply_admission);
		printf("qrk_upgrade: recover_admission=%s (%d)\n",
		       recover_admission ? "BLOCKED" : "PASS", recover_admission);
		ret = 0;
		goto out;
	}
	ret = qrk_set_state(&target, "complete", &image, "none", "");
	if (ret) {
		failure_error = qrk_error_code(ret);
		goto fail;
	}
	printf("qrk_upgrade: COMPLETE plan=%s\n", image.plan_id);
	goto out;

fail:
	if (applying)
		qrk_mark_failed(&target, &image, env_get("upgrade_component") ?: "none",
				failure_error);
out:
	unmap_sysmem(buffer);
	lmb_free(QRK_STAGE_BASE, QRK_STAGE_SIZE, LMB_NONE);
	return ret;
}

static int qrk_status(void)
{
	static const char *const names[] = {
		"upgrade_state", "upgrade_plan_id", "upgrade_component",
		"upgrade_error",
	};
	unsigned int i;

	printf("persistent_env     = %s\n", qrk_env_valid() ? "VALID" : "UNAVAILABLE");
	for (i = 0; i < ARRAY_SIZE(names); i++) {
		const char *unavailable = !strcmp(names[i], "upgrade_error") ?
			"ENV_INVALID" : "UNAVAILABLE";

		printf("%-22s = %s\n", names[i], qrk_env_valid() ?
		       (env_get(names[i]) ?: "<unset>") : unavailable);
	}
	printf("autoboot_interlock = %s\n", !qrk_env_valid() ||
	       (env_get("upgrade_state") &&
		strcmp(env_get("upgrade_state"), "idle") &&
		strcmp(env_get("upgrade_state"), "complete")) ? "ACTIVE" : "clear");
	return 0;
}

static int do_qrk_upgrade(struct cmd_tbl *cmdtp, int flag, int argc,
			  char *const argv[])
{
	enum qrk_mode mode;
	const char *path = NULL;
	int ret;

	if (argc == 2 && !strcmp(argv[1], "status"))
		return qrk_status() ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
	if (argc < 3 || argc > 4)
		return CMD_RET_USAGE;
	if (!strcmp(argv[1], "check"))
		mode = QRK_MODE_CHECK;
	else if (!strcmp(argv[1], "apply"))
		mode = QRK_MODE_APPLY;
	else if (!strcmp(argv[1], "recover"))
		mode = QRK_MODE_RECOVER;
	else
		return CMD_RET_USAGE;
	if (!strcmp(argv[2], "tftp")) {
		if (argc != 4)
			return CMD_RET_USAGE;
		path = argv[3];
	} else if (!strcmp(argv[2], "xmodem")) {
		if (argc != 3)
			return CMD_RET_USAGE;
	} else {
		return CMD_RET_USAGE;
	}
	ret = qrk_run(mode, argv[2], path);
	return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
}

U_BOOT_CMD(qrk_upgrade, 4, 0, do_qrk_upgrade,
	   "validate and install QRK3588 component images",
	   "status\n"
	   "qrk_upgrade check tftp <image-path>\n"
	   "qrk_upgrade apply tftp <image-path>\n"
	   "qrk_upgrade recover tftp <image-path>\n"
	   "qrk_upgrade check xmodem\n"
	   "qrk_upgrade apply xmodem\n"
	   "qrk_upgrade recover xmodem");
