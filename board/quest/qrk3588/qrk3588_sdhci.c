// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2026 ninozhang001@gmail.com
 *
 * Quest QRK3588 eMMC host glue for U-Boot's generic SDHCI core.
 */

#include <clk.h>
#include <dm.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <mmc.h>
#include <reset.h>
#include <sdhci.h>

#define QRK3588_EMMC_CTRL		0x052c
#define QRK3588_EMMC_CTRL_CARD_IS_EMMC	BIT(0)

enum qrk3588_sdhci_clk_id {
	QRK3588_SDMMC_CLK_CORE,
	QRK3588_SDMMC_CLK_BUS,
	QRK3588_SDMMC_CLK_AXI,
	QRK3588_SDMMC_CLK_BLOCK,
	QRK3588_SDMMC_CLK_TIMER,
	QRK3588_SDMMC_CLK_COUNT,
};

static const char *const qrk3588_sdhci_clk_names[QRK3588_SDMMC_CLK_COUNT] = {
	[QRK3588_SDMMC_CLK_CORE]	= "core",
	[QRK3588_SDMMC_CLK_BUS]		= "bus",
	[QRK3588_SDMMC_CLK_AXI]		= "axi",
	[QRK3588_SDMMC_CLK_BLOCK]	= "block",
	[QRK3588_SDMMC_CLK_TIMER]	= "timer",
};

/*
 * Platform data is the persistent MMC configuration parsed from DT and the
 * struct mmc instance bound to this device.
 */
struct qrk3588_sdhci_plat {
	struct mmc_config cfg;
	struct mmc mmc;
};

/*
 * Private data is runtime state owned by the driver.
 *
 * host is kept inside the runtime state. Generic SDHCI accesses it through
 * mmc->priv; the QRK3588 driver keeps its clock/reset handles alongside it.
 */
struct qrk3588_sdhci_priv {
	struct sdhci_host host;

	struct clk clocks[QRK3588_SDMMC_CLK_COUNT];
	unsigned int num_clocks;

	struct reset_ctl_bulk resets;
	bool resets_acquired;
	bool resets_deasserted;
};

static struct sdhci_host *qrk3588_sdhci_host(struct udevice *dev)
{
	struct qrk3588_sdhci_priv *priv = dev_get_priv(dev);

	return &priv->host;
}

static int qrk3588_sdhci_bind(struct udevice *dev)
{
	struct qrk3588_sdhci_plat *plat = dev_get_plat(dev);

	return sdhci_bind(dev, &plat->mmc, &plat->cfg);
}

static int qrk3588_sdhci_of_to_plat(struct udevice *dev)
{
	struct qrk3588_sdhci_plat *plat = dev_get_plat(dev);
	struct sdhci_host *host = qrk3588_sdhci_host(dev);

	host->name = dev->name;
	host->ioaddr = dev_read_addr_ptr(dev);
	if (!host->ioaddr)
		return -EINVAL;

	return mmc_of_parse(dev, &plat->cfg);
}

static int qrk3588_sdhci_get_clocks(struct udevice *dev,
				    struct qrk3588_sdhci_priv *priv)
{
	unsigned int i;
	int ret;

	priv->num_clocks = 0;

	for (i = 0; i < QRK3588_SDMMC_CLK_COUNT; i++) {
		ret = clk_get_by_name(dev, qrk3588_sdhci_clk_names[i],
				      &priv->clocks[i]);
		if (ret) {
			printf("emmc: clk %s get failed, %d\n", qrk3588_sdhci_clk_names[i], ret);
			return ret;
		}

		priv->num_clocks++;
	}

	return 0;
}

static void qrk3588_sdhci_release_clocks(struct qrk3588_sdhci_priv *priv)
{
	if (!priv->num_clocks)
		return;

	/* Disable/free all successfully requested handles. */
	clk_release_all(priv->clocks, priv->num_clocks);
	priv->num_clocks = 0;
}

static int qrk3588_sdhci_enable_clocks(struct qrk3588_sdhci_priv *priv)
{
	unsigned int i;
	int ret;

	for (i = 0; i < priv->num_clocks; i++) {
		ret = clk_enable(&priv->clocks[i]);
		if (ret) {
			printf("emmc: fail to enable \"%s\" clock, %d\n",
			       qrk3588_sdhci_clk_names[i], ret);
			return ret;
		}
	}

	return 0;
}

static int qrk3588_sdhci_prepare_core_clock(struct qrk3588_sdhci_priv *priv,
					    struct mmc_config *cfg,
					    ulong *actual_rate)
{
	struct clk *core = &priv->clocks[QRK3588_SDMMC_CLK_CORE];
	ulong rate;

	/*
	 * max-frequency is optional in generic MMC parsing. If present, request
	 * that controller-source ceiling. Otherwise preserve the inherited clock
	 * state and discover its current rate.
	 */
	if (cfg->f_max) {
		rate = clk_set_rate(core, cfg->f_max);
		if (IS_ERR_VALUE(rate))
			return (int)(long)rate;
	} else {
		rate = clk_get_rate(core);
		if (IS_ERR_VALUE(rate))
			return (int)(long)rate;
	}

	if (!rate)
		return -EINVAL;

	*actual_rate = rate;
	return 0;
}

static void qrk3588_sdhci_cleanup(struct qrk3588_sdhci_priv *priv)
{
	if (priv->resets_acquired) {
		if (priv->resets_deasserted)
			reset_assert_bulk(&priv->resets);

		reset_release_bulk(&priv->resets);
		priv->resets_acquired = false;
		priv->resets_deasserted = false;
	}

	qrk3588_sdhci_release_clocks(priv);
}

static void qrk3588_sdhci_set_clock(struct sdhci_host *host, u32 div)
{
	ulong rate;
	struct qrk3588_sdhci_priv *priv = dev_get_priv(host->mmc->dev);
	struct clk *core = &priv->clocks[QRK3588_SDMMC_CLK_CORE];

	(void)div;

	rate = host->mmc->clock;

	clk_set_rate(core, rate);
}

static int qrk3588_sdhci_set_ios_post(struct sdhci_host *host)
{
	u16 ctrl;

	/* SDHCI_RESET_ALL restores this implementation register to its reset. */
	ctrl = sdhci_readw(host, QRK3588_EMMC_CTRL);
	ctrl |= QRK3588_EMMC_CTRL_CARD_IS_EMMC;
	sdhci_writew(host, ctrl, QRK3588_EMMC_CTRL);

	return 0;
}

const struct sdhci_ops qrk3588_sdhci_ops = {
	.set_clock	= qrk3588_sdhci_set_clock,
	.set_ios_post	= qrk3588_sdhci_set_ios_post,
};

static int qrk3588_sdhci_probe(struct udevice *dev)
{
	struct mmc_uclass_priv *upriv = dev_get_uclass_priv(dev);
	struct qrk3588_sdhci_plat *plat = dev_get_plat(dev);
	struct qrk3588_sdhci_priv *priv = dev_get_priv(dev);
	struct sdhci_host *host = &priv->host;
	ulong core_rate;
	int ret;

	ret = qrk3588_sdhci_get_clocks(dev, priv);
	if (ret) {
		printf("emmc: fail to get clocks, %d\n", ret);
		goto err;
	}

	ret = qrk3588_sdhci_prepare_core_clock(priv, &plat->cfg, &core_rate);
	if (ret) {
		printf("emmc: core clock prepare failed, %d\n", ret);
		goto err;
	}

	ret = qrk3588_sdhci_enable_clocks(priv);
	if (ret) {
		printf("emmc: fail to enable clocks, %d\n", ret);
		goto err;
	}

	ret = reset_get_bulk(dev, &priv->resets);
	if (ret) {
		printf("emmc: fail to get reset pin, %d\n", ret);
		goto err;
	}
	priv->resets_acquired = true;

	ret = reset_deassert_bulk(&priv->resets);
	if (ret) {
		printf("emmc: reset deassert failed, %d\n", ret);
		goto err;
	}
	priv->resets_deasserted = true;

	/*
	 * Establish MMC/SDHCI relationships before sdhci_setup_cfg(), since
	 * generic SDHCI capability parsing may access host->mmc->dev.
	 */
	host->mmc = &plat->mmc;
	host->mmc->dev = dev;
	host->mmc->priv = host;
	upriv->mmc = host->mmc;

	/*
	 * This is the actual SDHCI controller input/base clock. Generic SDHCI
	 * derives identification and transfer card clocks with its own divider.
	 */
	host->max_clk = core_rate;

	host->ops = &qrk3588_sdhci_ops;

	ret = sdhci_setup_cfg(&plat->cfg, host, core_rate, 0);
	if (ret) {
		printf("emmc: sdhci_setup_cfg failed, %d\n", ret);
		goto err;
	}

	ret = sdhci_probe(dev);
	if (ret) {
		printf("emmc: sdhci_probe failed, %d\n", ret);
		goto err;
	}

	return 0;

err:
	qrk3588_sdhci_cleanup(priv);
	return ret;
}

static int qrk3588_sdhci_remove(struct udevice *dev)
{
	struct qrk3588_sdhci_priv *priv = dev_get_priv(dev);

	qrk3588_sdhci_cleanup(priv);
	return 0;
}

static const struct udevice_id qrk3588_sdhci_ids[] = {
	{ .compatible = "quest,qrk3588-emmc-sdhci" },
	{ }
};

U_BOOT_DRIVER(qrk3588_sdhci) = {
	.name		= "qrk3588_sdhci",
	.id		= UCLASS_MMC,
	.of_match	= qrk3588_sdhci_ids,
	.of_to_plat	= qrk3588_sdhci_of_to_plat,
	.ops		= &sdhci_ops,
	.bind		= qrk3588_sdhci_bind,
	.probe		= qrk3588_sdhci_probe,
	.remove		= qrk3588_sdhci_remove,
	.priv_auto	= sizeof(struct qrk3588_sdhci_priv),
	.plat_auto	= sizeof(struct qrk3588_sdhci_plat),
};
