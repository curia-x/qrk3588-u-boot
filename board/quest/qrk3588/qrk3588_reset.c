// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2026 ninozhang001@gmail.com
 *
 * Quest QRK3588 reset provider.
 */

#include <dm.h>
#include <dm/read.h>
#include <reset-uclass.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <board/quest/qrk3588_cru.h>
#include <dt-bindings/reset/quest,qrk3588-cru-reset.h>

struct qrk3588_reset_plat {
	void __iomem *cru_base;
	void __iomem *phptop_cru_base;
};

enum qrk3588_reset_domain {
	QRK3588_RESET_DOMAIN_CRU,
	QRK3588_RESET_DOMAIN_PHPTOP_CRU,
};

struct qrk3588_reset_desc {
	enum qrk3588_reset_domain domain;
	u16 offset;
	u16 mask;
};

static const struct qrk3588_reset_desc qrk3588_resets[] = {
	[QRK3588_RESET_EMMC_HOST] = {
		.domain = QRK3588_RESET_DOMAIN_CRU,
		.offset = CRU_SOFTRST_CON31,
		.mask = EMMC_HOST_RESET_MASK,
	},
	[QRK3588_RESET_PCIE_FABRIC_MAIN] = {
		.domain = QRK3588_RESET_DOMAIN_CRU,
		.offset = QRK3588_CRU_SOFTRST_CON32,
		.mask = BIT(12) | BIT(9) | BIT(8) | BIT(5),
	},
	[QRK3588_RESET_PCIE_FABRIC_AUX] = {
		.domain = QRK3588_RESET_DOMAIN_CRU,
		.offset = QRK3588_CRU_SOFTRST_CON34,
		.mask = BIT(9),
	},
	[QRK3588_RESET_PCIE1L0_PIPE] = {
		.domain = QRK3588_RESET_DOMAIN_CRU,
		.offset = QRK3588_CRU_SOFTRST_CON32,
		.mask = BIT(15),
	},
	[QRK3588_RESET_PCIE1L0_POWER_UP] = {
		.domain = QRK3588_RESET_DOMAIN_CRU,
		.offset = QRK3588_CRU_SOFTRST_CON33,
		.mask = BIT(14),
	},
	[QRK3588_RESET_PCIE1L2_PIPE] = {
		.domain = QRK3588_RESET_DOMAIN_CRU,
		.offset = QRK3588_CRU_SOFTRST_CON33,
		.mask = BIT(1),
	},
	[QRK3588_RESET_PCIE1L2_POWER_UP] = {
		.domain = QRK3588_RESET_DOMAIN_CRU,
		.offset = QRK3588_CRU_SOFTRST_CON34,
		.mask = BIT(0),
	},
	[QRK3588_RESET_PIPE_PHY0_GRF] = {
		.domain = QRK3588_RESET_DOMAIN_PHPTOP_CRU,
		.offset = QRK3588_PHPTOP_SOFTRST_CON00,
		.mask = BIT(2),
	},
	[QRK3588_RESET_PIPE_PHY0_APB] = {
		.domain = QRK3588_RESET_DOMAIN_PHPTOP_CRU,
		.offset = QRK3588_PHPTOP_SOFTRST_CON00,
		.mask = BIT(5),
	},
	[QRK3588_RESET_PIPE_PHY0_CORE] = {
		.domain = QRK3588_RESET_DOMAIN_CRU,
		.offset = QRK3588_CRU_SOFTRST_CON77,
		.mask = BIT(6),
	},
	[QRK3588_RESET_PIPE_PHY1_GRF] = {
		.domain = QRK3588_RESET_DOMAIN_PHPTOP_CRU,
		.offset = QRK3588_PHPTOP_SOFTRST_CON00,
		.mask = BIT(3),
	},
	[QRK3588_RESET_PIPE_PHY1_APB] = {
		.domain = QRK3588_RESET_DOMAIN_PHPTOP_CRU,
		.offset = QRK3588_PHPTOP_SOFTRST_CON00,
		.mask = BIT(6),
	},
	[QRK3588_RESET_PIPE_PHY1_CORE] = {
		.domain = QRK3588_RESET_DOMAIN_CRU,
		.offset = QRK3588_CRU_SOFTRST_CON77,
		.mask = BIT(7),
	},
};

static int qrk3588_reset_request(struct reset_ctl *reset_ctl)
{
	if (reset_ctl->id >= ARRAY_SIZE(qrk3588_resets))
		return -EINVAL;

	if (!qrk3588_resets[reset_ctl->id].mask)
		return -EINVAL;

	return 0;
}

static int qrk3588_reset_set(struct reset_ctl *reset_ctl, bool assert)
{
	struct qrk3588_reset_plat *plat = dev_get_plat(reset_ctl->dev);
	const struct qrk3588_reset_desc *reset;
	void __iomem *base;
	u32 value;

	if (reset_ctl->id >= ARRAY_SIZE(qrk3588_resets))
		return -EINVAL;

	reset = &qrk3588_resets[reset_ctl->id];

	if (!reset->mask)
		return -EINVAL;

	if (reset->domain == QRK3588_RESET_DOMAIN_PHPTOP_CRU)
		base = plat->phptop_cru_base;
	else
		base = plat->cru_base;

	/*
	 * CRU SOFTRST registers use hiword write-enable semantics.
	 *
	 * For these reset bits:
	 *   1 = assert reset
	 *   0 = deassert reset
	 */
	value = (u32)reset->mask << 16;

	if (assert)
		value |= reset->mask;

	writel(value,
	       (u8 __iomem *)base + reset->offset);

	return 0;
}

static int qrk3588_reset_assert(struct reset_ctl *reset_ctl)
{
	return qrk3588_reset_set(reset_ctl, true);
}

static int qrk3588_reset_deassert(struct reset_ctl *reset_ctl)
{
	return qrk3588_reset_set(reset_ctl, false);
}

static const struct reset_ops qrk3588_reset_ops = {
	.request	= qrk3588_reset_request,
	.rst_assert	= qrk3588_reset_assert,
	.rst_deassert	= qrk3588_reset_deassert,
};

static int qrk3588_reset_of_to_plat(struct udevice *dev)
{
	struct qrk3588_reset_plat *plat = dev_get_plat(dev);
	struct udevice *parent = dev_get_parent(dev);

	plat->cru_base = dev_read_addr_name_ptr(parent, "cru");
	if (!plat->cru_base)
		return -EINVAL;

	plat->phptop_cru_base = dev_read_addr_name_ptr(parent, "phptop-cru");
	if (!plat->phptop_cru_base)
		return -EINVAL;

	return 0;
}

static const struct udevice_id qrk3588_reset_ids[] = {
	{ .compatible = "quest,qrk3588-reset" },
	{ }
};

U_BOOT_DRIVER(qrk3588_reset) = {
	.name		= "qrk3588_reset",
	.id		= UCLASS_RESET,
	.of_match	= qrk3588_reset_ids,
	.of_to_plat	= qrk3588_reset_of_to_plat,
	.ops		= &qrk3588_reset_ops,
	.plat_auto	= sizeof(struct qrk3588_reset_plat),
};
