// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2026 ninozhang001@gmail.com
 *
 * Quest QRK3588 DesignWare PCIe host controller glue.
 *
 * The register programming in this file is independently implemented from
 * the RK3588 technical reference manual and the board schematic.
 */

#include <clk.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <dm/read.h>
#include <pci.h>
#include <reset.h>
#include <asm/io.h>
#include <asm-generic/gpio.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/iopoll.h>
#include "pcie_dw_common.h"

#define QRK3588_PMU_BIU_IDLE_SFTCON1    0x8110
#define QRK3588_PMU_BIU_IDLE_ACK_STS0   0x8118
#define QRK3588_PMU_BIU_IDLE_STS0   0x8120
#define QRK3588_PMU_PWR_GATE_SFTCON1    0x8150
#define QRK3588_PMU_PWR_GATE_STS0   0x8180
#define QRK3588_PMU_PHP_SFTCON_BIT  BIT(5)
#define QRK3588_PMU_PHP_STATUS_BIT  BIT(21)
#define QRK3588_PHP_PCIESEL_CON     0x0100
#define QRK3588_PCIE1L0_PHY_SEL     BIT(0)
#define QRK3588_PIPE_PHY_CON0       0x0000
#define QRK3588_PIPE_PHY_CON1       0x0004
#define QRK3588_PIPE_PHY_CON2       0x0008
#define QRK3588_PIPE_PHY_CON3       0x000c
#define QRK3588_PIPE_PHY_STATUS1        0x0034
#define QRK3588_PIPE_PHY_MODE       GENMASK(3, 2)
#define QRK3588_PIPE_PHY_RATE       GENMASK(5, 4)
#define QRK3588_PIPE_PHY_RATE_GEN2  FIELD_PREP(QRK3588_PIPE_PHY_RATE, 1)
#define QRK3588_PIPE_PHY_MAC_CLKREQ_N   BIT(8)
#define QRK3588_PIPE_PHY_SEL_CLKREQ_N   BIT(3)
#define QRK3588_PIPE_PHY_CLK_SEL    GENMASK(14, 13)
#define QRK3588_PIPE_PHY_CLK_100M   FIELD_PREP(QRK3588_PIPE_PHY_CLK_SEL, 2)
#define QRK3588_PIPE_PHY_REF_SRC    GENMASK(9, 8)
#define QRK3588_PIPE_PHY_REF_OUTPUT FIELD_PREP(QRK3588_PIPE_PHY_REF_SRC, 2)
#define QRK3588_PIPE_PHY_SEL_TXELECIDLE BIT(12)
#define QRK3588_PIPE_PHY_SEL_TXDETECTRX BIT(11)
#define QRK3588_PIPE_PHY_SEL_POWERDOWN  BIT(4)
#define QRK3588_PIPE_PHY_SEL_RATE       BIT(2)
#define QRK3588_PIPE_PHY_SEL_CONTROLLER_CTRL \
    (QRK3588_PIPE_PHY_SEL_TXELECIDLE | \
     QRK3588_PIPE_PHY_SEL_TXDETECTRX | \
     QRK3588_PIPE_PHY_SEL_POWERDOWN | \
     QRK3588_PIPE_PHY_SEL_RATE)
#define QRK3588_COMBO_PHY_REG005        0x0014
#define QRK3588_COMBO_PHY_MODE      GENMASK(2, 0)
#define QRK3588_COMBO_PHY_REG031        0x007c
#define QRK3588_COMBO_PHY_CKDRV_REF_SRC BIT(0)
#define QRK3588_COMBO_PHY_STATUS39  0x009c
#define QRK3588_COMBO_PHY_PLL_LOCK  BIT(0)
#define QRK3588_PCIE_CLIENT_CON     0x0000
#define QRK3588_PCIE_HOT_RESET_CTRL 0x0180
#define QRK3588_PCIE_LTSSM_STATUS   0x0300
#define QRK3588_PCIE_LTSSM_STATE    GENMASK(5, 0)
#define QRK3588_PCIE_DEVICE_TYPE        GENMASK(7, 4)
#define QRK3588_PCIE_ROOT_PORT      FIELD_PREP(QRK3588_PCIE_DEVICE_TYPE, 4)
#define QRK3588_PCIE_LTSSM_ENABLE   BIT(2)
#define QRK3588_PCIE_LTSSM_ENHANCE  BIT(4)
#define QRK3588_PCIE_LINK_UP        (BIT(17) | BIT(16))
#define QRK3588_PCIE1L0_DBI     0xf5800000UL
#define QRK3588_PCIE1L2_DBI     0xf6000000UL
#define QRK3588_LINK_TIMEOUT_US     1000000
#define QRK3588_POWER_TIMEOUT_US        100000
#define QRK3588_PERST_DELAY_MS      100
#define QRK3588_PCIE_REFCLK_HZ       100000000UL

enum qrk3588_pcie_port {
    QRK3588_PCIE1L0,
    QRK3588_PCIE1L2,
};

struct qrk3588_pcie {
    /* The common PCIe state must be the first member. */
    struct pcie_dw dw;
    void __iomem *apb;
    void __iomem *pipe_grf;
    void __iomem *phy;
    void __iomem *php_grf;
    void __iomem *pmu;
    struct gpio_desc perst;
    struct clk core_clk;
    struct clk phy_clk;
    struct clk refclk;
    struct reset_ctl fabric_main_reset;
    struct reset_ctl fabric_aux_reset;
    struct reset_ctl phy_grf_reset;
    struct reset_ctl phy_apb_reset;
    struct reset_ctl phy_core_reset;
    struct reset_ctl pipe_reset;
    struct reset_ctl power_up_reset;
    unsigned int num_resets;
    enum qrk3588_pcie_port port;
};

struct qrk3588_pcie_reset_ref {
    const char *name;
    size_t offset;
};

static const struct qrk3588_pcie_reset_ref qrk3588_pcie_resets[] = {
    { "fabric-main", offsetof(struct qrk3588_pcie, fabric_main_reset) },
    { "fabric-aux", offsetof(struct qrk3588_pcie, fabric_aux_reset) },
    { "phy-grf", offsetof(struct qrk3588_pcie, phy_grf_reset) },
    { "phy-apb", offsetof(struct qrk3588_pcie, phy_apb_reset) },
    { "phy-core", offsetof(struct qrk3588_pcie, phy_core_reset) },
    { "pipe", offsetof(struct qrk3588_pcie, pipe_reset) },
    { "power-up", offsetof(struct qrk3588_pcie, power_up_reset) },
};

static void qrk3588_hiword_update(void __iomem *base, u32 offset,
				  u32 mask, u32 value)
{
    writel((mask << 16) | (value & mask), base + offset);
}

static int qrk3588_pcie_enable_clocks(struct qrk3588_pcie *priv)
{
    int ret;

    ret = clk_enable(&priv->core_clk);
    if (ret) {
	dev_err(priv->dw.dev, "failed to enable PCIe core clocks: %d\n", ret);
	return ret;
    }

    ret = clk_enable(&priv->phy_clk);
    if (ret) {
	dev_err(priv->dw.dev, "failed to enable Combo PHY clocks: %d\n", ret);
	clk_disable(&priv->core_clk);
	return ret;
    }

    return 0;
}

static void qrk3588_pcie_disable_clocks(struct qrk3588_pcie *priv)
{
    clk_disable(&priv->phy_clk);
    clk_disable(&priv->core_clk);
}

static int qrk3588_pcie_release_fabric_resets(struct qrk3588_pcie *priv)
{
    int ret;

    ret = reset_deassert(&priv->fabric_main_reset);
    if (ret)
	return ret;

    return reset_deassert(&priv->fabric_aux_reset);
}

static int qrk3588_pcie_power_on(struct qrk3588_pcie *priv)
{
    u32 value;
    int ret;

    /* PD_PHP contains the three x1 controllers, PHP_GRF, and BIU_PHP. */
    qrk3588_hiword_update(priv->pmu, QRK3588_PMU_PWR_GATE_SFTCON1,
			  QRK3588_PMU_PHP_SFTCON_BIT, 0);
    ret = readl_poll_timeout(priv->pmu + QRK3588_PMU_PWR_GATE_STS0,
			     value,
		 !(value & QRK3588_PMU_PHP_STATUS_BIT),
		 QRK3588_POWER_TIMEOUT_US);
    if (ret) {
	dev_err(priv->dw.dev, "PD_PHP power-up timed out (status %#x)\n",
		value);
	return ret;
    }

    /* Resume transactions through BIU_PHP after the domain is powered. */
    qrk3588_hiword_update(priv->pmu, QRK3588_PMU_BIU_IDLE_SFTCON1,
			  QRK3588_PMU_PHP_SFTCON_BIT, 0);
    ret = readl_poll_timeout(priv->pmu + QRK3588_PMU_BIU_IDLE_ACK_STS0,
			     value,
		 !(value & QRK3588_PMU_PHP_STATUS_BIT),
		 QRK3588_POWER_TIMEOUT_US);
    if (ret) {
	dev_err(priv->dw.dev, "BIU_PHP resume ACK timed out (status %#x)\n",
		value);
	return ret;
    }

    ret = readl_poll_timeout(priv->pmu + QRK3588_PMU_BIU_IDLE_STS0,
			     value,
		 !(value & QRK3588_PMU_PHP_STATUS_BIT),
		 QRK3588_POWER_TIMEOUT_US);
    if (ret)
	dev_err(priv->dw.dev, "BIU_PHP resume timed out (status %#x)\n",
		value);

    return ret;
}

static int qrk3588_pcie_set_phy_grf_reset(struct qrk3588_pcie *priv,
					  bool assert)
{
    return assert ? reset_assert(&priv->phy_grf_reset) :
	    reset_deassert(&priv->phy_grf_reset);
}

static int qrk3588_pcie_set_phy_apb_reset(struct qrk3588_pcie *priv,
					  bool assert)
{
    return assert ? reset_assert(&priv->phy_apb_reset) :
	    reset_deassert(&priv->phy_apb_reset);
}

static int qrk3588_pcie_set_phy_core_reset(struct qrk3588_pcie *priv,
					   bool assert)
{
    return assert ? reset_assert(&priv->phy_core_reset) :
	    reset_deassert(&priv->phy_core_reset);
}

static int qrk3588_pcie_set_controller_resets(struct qrk3588_pcie *priv,
					      bool assert)
{
    int ret;

    ret = assert ? reset_assert(&priv->pipe_reset) :
	    reset_deassert(&priv->pipe_reset);
    if (ret)
	return ret;

    return assert ? reset_assert(&priv->power_up_reset) :
	    reset_deassert(&priv->power_up_reset);
}

static int qrk3588_pcie_configure_refclk(struct qrk3588_pcie *priv)
{
    ulong rate;
    int ret;

    rate = clk_set_rate(&priv->refclk, QRK3588_PCIE_REFCLK_HZ);
    if (IS_ERR_VALUE(rate)) {
	ret = (long)rate;
	dev_err(priv->dw.dev, "failed to set PCIe REFCLK: %d\n", ret);
	return ret;
    }

    if (rate != QRK3588_PCIE_REFCLK_HZ) {
	dev_err(priv->dw.dev,
		"PCIe REFCLK requested %lu Hz, provider selected %lu Hz\n",
	    QRK3588_PCIE_REFCLK_HZ, rate);
	return -ERANGE;
    }

    ret = clk_enable(&priv->refclk);
    if (ret) {
	dev_err(priv->dw.dev, "failed to enable PCIe REFCLK: %d\n", ret);
	return ret;
    }

    rate = clk_get_rate(&priv->refclk);
    if (IS_ERR_VALUE(rate)) {
	ret = (long)rate;
	dev_err(priv->dw.dev, "failed to read PCIe REFCLK rate: %d\n", ret);
	goto err_disable_refclk;
    }

    if (rate != QRK3588_PCIE_REFCLK_HZ) {
	dev_err(priv->dw.dev,
		"PCIe REFCLK readback is %lu Hz, expected %lu Hz\n",
	    rate, QRK3588_PCIE_REFCLK_HZ);
	ret = -ERANGE;
	goto err_disable_refclk;
    }

    return 0;

err_disable_refclk:
    clk_disable(&priv->refclk);
    return ret;
}

static void qrk3588_pcie_configure_phy(struct qrk3588_pcie *priv)
{
    /*
     * Program PCIe mode and the Gen2 fallback rate value.  RATE itself stays
     * owned by the controller so link training starts at the protocol-defined
     * rate and can change it dynamically.
     */
    qrk3588_hiword_update(priv->pipe_grf, QRK3588_PIPE_PHY_CON0,
			  QRK3588_PIPE_PHY_MODE | QRK3588_PIPE_PHY_RATE,
		  QRK3588_PIPE_PHY_RATE_GEN2);
    /*
     * PIPE_CON1[14:13] is phy_clk_sel:
     *   00 = 24 MHz
     *   01 = 25 MHz
     *   10 = 100 MHz
     *
     * The clock provider now supplies PLL_CKREF_INNER at 100 MHz, so the PHY
     * must be told that its selected reference clock is 100 MHz before the
     * PHY core reset is released.
     */
    qrk3588_hiword_update(priv->pipe_grf, QRK3588_PIPE_PHY_CON1,
			  QRK3588_PIPE_PHY_CLK_SEL,
		  QRK3588_PIPE_PHY_CLK_100M);
    /*
     * A common-clock endpoint needs REFCLK before PERST# is released.
     * Force the active-low PIPE clock request asserted during bring-up.
     */
    qrk3588_hiword_update(priv->pipe_grf, QRK3588_PIPE_PHY_CON0,
			  QRK3588_PIPE_PHY_MAC_CLKREQ_N, 0);
    qrk3588_hiword_update(priv->pipe_grf, QRK3588_PIPE_PHY_CON2,
			  QRK3588_PIPE_PHY_SEL_CLKREQ_N,
		  QRK3588_PIPE_PHY_SEL_CLKREQ_N);

    /*
     * Receiver detection is driven by the controller LTSSM. Do not leave
     * TxElecIdle, TxDetectRx/Loopback, or PowerDown on their PHY-GRF
     * overrides. In particular, the reset value selects the PHY-GRF value
     * for TxElecIdle; CON1 leaves that value low, which prevents the MAC from
     * asserting electrical idle during Detect.
     */
    qrk3588_hiword_update(priv->pipe_grf, QRK3588_PIPE_PHY_CON2,
			  QRK3588_PIPE_PHY_SEL_CONTROLLER_CTRL, 0);

    /*
     * PIPE_CON3[9:8] = 2 selects PLL_CKREF_INNER as the PHY reference
     * and enables CKREFP/N as a differential 100 MHz output in PCIe
     * mode while PIPE PHY CLKREQ is asserted.
     */
    qrk3588_hiword_update(priv->pipe_grf, QRK3588_PIPE_PHY_CON3,
			  QRK3588_PIPE_PHY_REF_SRC,
		  QRK3588_PIPE_PHY_REF_OUTPUT);
}

static int qrk3588_pcie_wait_phy_pll_lock(struct qrk3588_pcie *priv)
{
    u32 status;
    int ret;

    ret = readl_poll_timeout(priv->phy + QRK3588_COMBO_PHY_STATUS39,
			     status, status & QRK3588_COMBO_PHY_PLL_LOCK,
		 QRK3588_POWER_TIMEOUT_US);
    if (ret) {
	ulong ref_rate = clk_get_rate(&priv->refclk);

	dev_err(priv->dw.dev,
		"Combo PHY PLL lock timed out: status=%#x con1=%#x con3=%#x refclk=%ld\n",
	    status,
	    readl(priv->pipe_grf + QRK3588_PIPE_PHY_CON1),
	    readl(priv->pipe_grf + QRK3588_PIPE_PHY_CON3),
	    IS_ERR_VALUE(ref_rate) ? (long)ref_rate : (long)ref_rate);
    }

    return ret;
}

static void qrk3588_pcie_configure_phy_internal(struct qrk3588_pcie *priv)
{
    /* PMA protocol mode follows the PIPE interface; do not use APB override. */
    clrsetbits_le32(priv->phy + QRK3588_COMBO_PHY_REG005,
		    QRK3588_COMBO_PHY_MODE, 0);

    /* Drive CKREF output selection from PIPE_CON3.CKREF_SRC. */
    setbits_le32(priv->phy + QRK3588_COMBO_PHY_REG031,
		 QRK3588_COMBO_PHY_CKDRV_REF_SRC);
}

static int qrk3588_pcie_get_resets(struct udevice *dev,
				   struct qrk3588_pcie *priv)
{
    unsigned int i;
    int ret;

    for (i = 0; i < ARRAY_SIZE(qrk3588_pcie_resets); i++) {
	const struct qrk3588_pcie_reset_ref *ref = &qrk3588_pcie_resets[i];
	struct reset_ctl *reset = (void *)priv + ref->offset;

	ret = reset_get_by_name(dev, ref->name, reset);
	if (ret) {
	    dev_err(dev, "failed to get %s reset: %d\n",
		    ref->name, ret);
	    return ret;
	}
	priv->num_resets++;
    }

    return 0;
}

static void qrk3588_pcie_free_resets(struct qrk3588_pcie *priv)
{
    while (priv->num_resets) {
	const struct qrk3588_pcie_reset_ref *ref;
	struct reset_ctl *reset;

	ref = &qrk3588_pcie_resets[--priv->num_resets];
	reset = (void *)priv + ref->offset;
	reset_free(reset);
    }
}

static int qrk3588_pcie_parse_dt(struct udevice *dev,
				 struct qrk3588_pcie *priv)
{
    fdt_addr_t dbi_addr;
    fdt_size_t cfg_size;
    int ret;

    priv->dw.dbi_base = dev_read_addr_name_ptr(dev, "dbi");
    priv->dw.cfg_base = dev_read_addr_size_name_ptr(dev, "config", &cfg_size);
    priv->apb = dev_read_addr_name_ptr(dev, "apb");
    priv->pipe_grf = dev_read_addr_name_ptr(dev, "pipe-grf");
    priv->phy = dev_read_addr_name_ptr(dev, "phy");
    priv->php_grf = dev_read_addr_name_ptr(dev, "php-grf");
    priv->pmu = dev_read_addr_name_ptr(dev, "pmu");

    if (!priv->dw.dbi_base || !priv->dw.cfg_base || !priv->apb ||
	!priv->pipe_grf || !priv->phy || !priv->php_grf || !priv->pmu)
	return -EINVAL;

    priv->dw.cfg_size = cfg_size;
    dbi_addr = dev_read_addr_name(dev, "dbi");
    if (dbi_addr == QRK3588_PCIE1L0_DBI)
	priv->port = QRK3588_PCIE1L0;
    else if (dbi_addr == QRK3588_PCIE1L2_DBI)
	priv->port = QRK3588_PCIE1L2;
    else
	return -EINVAL;

    ret = clk_get_by_name(dev, "core", &priv->core_clk);
    if (ret) {
	dev_err(dev, "failed to get PCIe core clock: %d\n", ret);
	return ret;
    }

    ret = clk_get_by_name(dev, "phy", &priv->phy_clk);
    if (ret) {
	dev_err(dev, "failed to get Combo PHY clock: %d\n", ret);
	return ret;
    }

    ret = clk_get_by_name(dev, "ref", &priv->refclk);
    if (ret) {
	dev_err(dev, "failed to get PCIe reference clock: %d\n", ret);
	return ret;
    }

    ret = qrk3588_pcie_get_resets(dev, priv);
    if (ret) {
	qrk3588_pcie_free_resets(priv);
	return ret;
    }

    ret = gpio_request_by_name(dev, "reset-gpios", 0, &priv->perst,
			       GPIOD_IS_OUT | GPIOD_ACTIVE_LOW);
    if (ret)
	dev_err(dev, "failed to request endpoint reset GPIO: %d\n", ret);

    if (ret)
	qrk3588_pcie_free_resets(priv);

    return ret;
}

static int qrk3588_pcie_wait_link(struct qrk3588_pcie *priv)
{
    u32 status;
    int ret;

    ret = readl_poll_timeout(priv->apb + QRK3588_PCIE_LTSSM_STATUS,
			     status,
		 (status & QRK3588_PCIE_LINK_UP) == QRK3588_PCIE_LINK_UP,
		 QRK3588_LINK_TIMEOUT_US);
    if (ret) {
	dev_err(priv->dw.dev,
		"link timeout: ltssm=%#x state=%#x pipe=%#x phy=%#x\n",
	    status,
	    (u32)FIELD_GET(QRK3588_PCIE_LTSSM_STATE, status),
	    readl(priv->pipe_grf + QRK3588_PIPE_PHY_STATUS1),
	    readl(priv->phy + QRK3588_COMBO_PHY_STATUS39));
    }

    return ret;
}

static int qrk3588_pcie_assert_local_resets(struct qrk3588_pcie *priv)
{
    int ret;

    ret = qrk3588_pcie_set_controller_resets(priv, true);
    if (ret)
	return ret;
    ret = qrk3588_pcie_set_phy_core_reset(priv, true);
    if (ret)
	return ret;
    ret = qrk3588_pcie_set_phy_apb_reset(priv, true);
    if (ret)
	return ret;

    return qrk3588_pcie_set_phy_grf_reset(priv, true);
}

static void qrk3588_pcie_disable_ltssm(struct qrk3588_pcie *priv)
{
    qrk3588_hiword_update(priv->apb, QRK3588_PCIE_CLIENT_CON,
			  QRK3588_PCIE_LTSSM_ENABLE, 0);
}

static int qrk3588_pcie_probe(struct udevice *dev)
{
    struct qrk3588_pcie *priv = dev_get_priv(dev);
    int ret;

    priv->dw.dev = dev;
    priv->dw.first_busno = dev_seq(dev);

    ret = qrk3588_pcie_parse_dt(dev, priv);
    if (ret)
	return ret;

    /* Keep the endpoint in reset until clocks, PHY and RC are ready. */
    dm_gpio_set_value(&priv->perst, 1);

    ret = qrk3588_pcie_enable_clocks(priv);
    if (ret)
	goto err_free_gpio;

    ret = qrk3588_pcie_power_on(priv);
    if (ret)
	goto err_disable_clocks;

    ret = qrk3588_pcie_release_fabric_resets(priv);
    if (ret) {
	dev_err(dev, "failed to release PCIe fabric resets: %d\n", ret);
	goto err_disable_clocks;
    }

    ret = qrk3588_pcie_assert_local_resets(priv);
    if (ret) {
	dev_err(dev, "failed to assert PCIe resets: %d\n", ret);
	goto err_disable_clocks;
    }
    udelay(10);

    ret = qrk3588_pcie_set_phy_grf_reset(priv, false);
    if (ret)
	goto err_assert_resets;
    ret = qrk3588_pcie_set_phy_apb_reset(priv, false);
    if (ret)
	goto err_assert_resets;
    udelay(10);

    if (priv->port == QRK3588_PCIE1L0)
	qrk3588_hiword_update(priv->php_grf, QRK3588_PHP_PCIESEL_CON,
			      QRK3588_PCIE1L0_PHY_SEL, 0);

    ret = qrk3588_pcie_configure_refclk(priv);
    if (ret)
	goto err_assert_resets;

    qrk3588_pcie_configure_phy(priv);
    qrk3588_pcie_configure_phy_internal(priv);

    ret = qrk3588_pcie_set_phy_core_reset(priv, false);
    if (ret)
	goto err_disable_refclk;
    udelay(10);
    ret = qrk3588_pcie_set_controller_resets(priv, false);
    if (ret)
	goto err_disable_refclk;

    ret = qrk3588_pcie_wait_phy_pll_lock(priv);
    if (ret)
	goto err_disable_refclk;

    udelay(10);

    qrk3588_hiword_update(priv->apb, QRK3588_PCIE_CLIENT_CON,
			  QRK3588_PCIE_DEVICE_TYPE,
		  QRK3588_PCIE_ROOT_PORT);
    qrk3588_hiword_update(priv->apb, QRK3588_PCIE_HOT_RESET_CTRL,
			  QRK3588_PCIE_LTSSM_ENHANCE,
		  QRK3588_PCIE_LTSSM_ENHANCE);

    dw_pcie_dbi_write_enable(&priv->dw, true);
    dw_pcie_link_set_max_link_width(&priv->dw, 1);
    dw_pcie_dbi_write_enable(&priv->dw, false);

    pcie_dw_setup_host(&priv->dw);

    mdelay(QRK3588_PERST_DELAY_MS);
    dm_gpio_set_value(&priv->perst, 0);
    mdelay(QRK3588_PERST_DELAY_MS);

    qrk3588_hiword_update(priv->apb, QRK3588_PCIE_CLIENT_CON,
			  QRK3588_PCIE_LTSSM_ENABLE,
		  QRK3588_PCIE_LTSSM_ENABLE);

    ret = qrk3588_pcie_wait_link(priv);
    if (ret)
	goto err_disable_link;

    ret = pcie_dw_prog_outbound_atu_unroll(&priv->dw,
					   PCIE_ATU_REGION_INDEX0,
			   PCIE_ATU_TYPE_MEM,
			   priv->dw.mem.phys_start,
			   priv->dw.mem.bus_start,
			   priv->dw.mem.size);
    if (ret)
	goto err_disable_link;

    dev_info(dev, "link up: Gen%d x%d\n",
	     pcie_dw_get_link_speed(&priv->dw),
	 pcie_dw_get_link_width(&priv->dw));

    return 0;

err_disable_link:
    qrk3588_pcie_disable_ltssm(priv);
err_disable_refclk:
    dm_gpio_set_value(&priv->perst, 1);
    qrk3588_pcie_assert_local_resets(priv);
    clk_disable(&priv->refclk);
    goto err_disable_clocks;

err_assert_resets:
    qrk3588_pcie_assert_local_resets(priv);
err_disable_clocks:
    qrk3588_pcie_disable_clocks(priv);
err_free_gpio:
    dm_gpio_free(dev, &priv->perst);
    qrk3588_pcie_free_resets(priv);
    return ret;
}

static int qrk3588_pcie_remove(struct udevice *dev)
{
    struct qrk3588_pcie *priv = dev_get_priv(dev);

    qrk3588_pcie_disable_ltssm(priv);
    dm_gpio_set_value(&priv->perst, 1);
    qrk3588_pcie_assert_local_resets(priv);
    clk_disable(&priv->refclk);
    qrk3588_pcie_disable_clocks(priv);
    dm_gpio_free(dev, &priv->perst);
    qrk3588_pcie_free_resets(priv);

    return 0;
}

static const struct dm_pci_ops qrk3588_pcie_ops = {
    .read_config = pcie_dw_read_config,
    .write_config = pcie_dw_write_config,
};

static const struct udevice_id qrk3588_pcie_ids[] = {
    { .compatible = "quest,qrk3588-pcie" },
    { }
};

U_BOOT_DRIVER(qrk3588_pcie) = {
    .name = "qrk3588_pcie",
    .id = UCLASS_PCI,
    .of_match = qrk3588_pcie_ids,
    .ops = &qrk3588_pcie_ops,
    .probe = qrk3588_pcie_probe,
    .remove = qrk3588_pcie_remove,
    .priv_auto = sizeof(struct qrk3588_pcie),
};
