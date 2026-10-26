// SPDX-License-Identifier: GPL-2.0-or-later
/* Copyright (c) 2026 ninozhang001@gmail.com */

#include <bootm.h>
#include <console.h>
#include <dm/read.h>
#include <env.h>
#include <event.h>
#include <hang.h>
#include <image.h>
#include <mmc.h>
#include <asm/global_data.h>
#include <linux/libfdt.h>

#if !defined(CONFIG_ENV_IS_NOWHERE)
#error "QRK3588 secure OS requires ENV_IS_NOWHERE"
#endif

DECLARE_GLOBAL_DATA_PTR;

#define QRK_FIT_ADDR 0x10000000UL
#define QRK_FIT_LBA 0x10000
#define QRK_FIT_SIZE 0x07800000

/* This event never returns to main_loop(), preboot, autoboot or the CLI. */
static int qrk3588_secure_boot(void)
{
	struct mmc *mmc;
	struct blk_desc *desc;
	void *fit = (void *)QRK_FIT_ADDR;
	const void *fdt;
	const char *args, *required;
	size_t size;
	int conf, kernel, dtb, chosen, key, len, ret;
	struct bootm_info bmi;

	disable_ctrlc(1);
	puts("QRK3588: authenticated OS boot\n");
	key = fdt_path_offset(gd->fdt_blob, "/signature/key-os");
	required = fdt_getprop(gd->fdt_blob, key, "required", &len);
	if (!required || len != 5 || memcmp(required, "conf", 5))
		goto fail;

	mmc = find_mmc_device(0);
	if (!mmc || mmc_init(mmc) ||
	    dev_read_addr(mmc->dev) != 0xfe2e0000 ||
	    mmc_switch_part(mmc, 0))
		goto fail;
	desc = mmc_get_blk_desc(mmc);
	if (!desc || desc->blksz != 512 ||
	    blk_dread(desc, QRK_FIT_LBA, 1, fit) != 1 ||
	    fdt_check_header(fit))
		goto fail;
	size = fdt_totalsize(fit);
	if (size < sizeof(struct fdt_header) || size > QRK_FIT_SIZE ||
	    blk_dread(desc, QRK_FIT_LBA, DIV_ROUND_UP(size, 512), fit) !=
	    DIV_ROUND_UP(size, 512) || fdt_check_full(fit, size))
		goto fail;

	conf = fit_conf_get_node(fit, "conf-secure");
	if (conf < 0 || fit_config_verify(fit, conf))
		goto fail;
	kernel = fit_conf_get_prop_node(fit, conf, "kernel", IH_PHASE_NONE);
	dtb = fit_conf_get_prop_node(fit, conf, "fdt", IH_PHASE_NONE);
	if (kernel < 0 || dtb < 0 || !fit_image_verify(fit, kernel) ||
	    !fit_image_verify(fit, dtb) ||
	    fit_image_get_data(fit, dtb, &fdt, &size) ||
	    fdt_check_full(fdt, size))
		goto fail;
	chosen = fdt_path_offset(fdt, "/chosen");
	args = fdt_getprop(fdt, chosen, "bootargs", &len);
	if (!args || len < 2 || args[len - 1] ||
	    !strstr(args, "root=/dev/dm-0") ||
	    !strstr(args, "dm-mod.create="))
		goto fail;
	ret = env_set("bootargs", args);
	if (ret || env_set("verify", "yes"))
		goto fail;
	bootm_init(&bmi);
	bmi.addr_img = "10000000#conf-secure";
	bmi.cmd_name = "bootm";
	bootm_run(&bmi);
fail:
	puts("QRK3588: secure OS verification/boot failed; stopped\n");
	hang();
	return -EPERM;
}
EVENT_SPY_SIMPLE(EVT_MAIN_LOOP, qrk3588_secure_boot);
