// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2026 ninozhang001@gmail.com
 *
 * QRK3588 pin controller
 */

#include <dm.h>
#include <dm/pinctrl.h>
#include <dm/read.h>
#include <errno.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <board/quest/qrk3588_ioc.h>

enum qrk3588_gpio_group {
	QRK3588_GPIO_A = 0,
	QRK3588_GPIO_B,
	QRK3588_GPIO_C,
	QRK3588_GPIO_D,
};

/*
 * The pinctrl core needs a scalar selector, but the hardware model is
 * kept explicit as (bank, A/B/C/D, pin).
 *
 * The resulting selector is only an encoding of that hardware coordinate:
 *
 *   selector = bank * 32 + group * 8 + pin
 */
#define QRK3588_PIN(bank, group, pin) \
	((unsigned int)(bank) * QRK3588_PINS_PER_BANK + \
	 (unsigned int)(group) * QRK3588_PINS_PER_GROUP + \
	 (unsigned int)(pin))

#define QRK3588_PIN_BANK(selector) \
	((unsigned int)(selector) / QRK3588_PINS_PER_BANK)

#define QRK3588_PIN_IN_BANK(selector) \
	((unsigned int)(selector) % QRK3588_PINS_PER_BANK)

#define QRK3588_PIN_GROUP(selector) \
	(QRK3588_PIN_IN_BANK(selector) / QRK3588_PINS_PER_GROUP)

#define QRK3588_PIN_INDEX(selector) \
	(QRK3588_PIN_IN_BANK(selector) % QRK3588_PINS_PER_GROUP)

enum qrk3588_bias {
	QRK3588_BIAS_DISABLE = 0,
	QRK3588_BIAS_PULL_DOWN,
	QRK3588_BIAS_PULL_UP,
};

enum qrk3588_pinctrl_group {
	QRK3588_GROUP_EMMC,
	QRK3588_GROUP_COUNT,
};

enum qrk3588_pinctrl_function {
	QRK3588_FUNCTION_EMMC,
	QRK3588_FUNCTION_COUNT,
};

struct qrk3588_emmc_pad {
	unsigned int selector;
	enum qrk3588_bias bias;
};

/*
 * eMMC pins described directly in hardware coordinates.
 *
 * A0: CMD
 * A1: CLKOUT
 * A2: DATA_STROBE
 * A3: RSTN
 * D0..D7: DAT0..DAT7
 *
 * RSTN has an external 10K pull-up on the board, so its internal bias
 * is disabled. Other bias values preserve the documented RK3588 eMMC
 * reset/default electrical state.
 */
static const struct qrk3588_emmc_pad qrk3588_emmc_pads[] = {
	{ QRK3588_PIN(2, QRK3588_GPIO_A, 0), QRK3588_BIAS_PULL_UP   },
	{ QRK3588_PIN(2, QRK3588_GPIO_A, 1), QRK3588_BIAS_PULL_DOWN },
	{ QRK3588_PIN(2, QRK3588_GPIO_A, 2), QRK3588_BIAS_PULL_DOWN },
	{ QRK3588_PIN(2, QRK3588_GPIO_A, 3), QRK3588_BIAS_DISABLE   },

	{ QRK3588_PIN(2, QRK3588_GPIO_D, 0), QRK3588_BIAS_PULL_UP },
	{ QRK3588_PIN(2, QRK3588_GPIO_D, 1), QRK3588_BIAS_PULL_UP },
	{ QRK3588_PIN(2, QRK3588_GPIO_D, 2), QRK3588_BIAS_PULL_UP },
	{ QRK3588_PIN(2, QRK3588_GPIO_D, 3), QRK3588_BIAS_PULL_UP },
	{ QRK3588_PIN(2, QRK3588_GPIO_D, 4), QRK3588_BIAS_PULL_UP },
	{ QRK3588_PIN(2, QRK3588_GPIO_D, 5), QRK3588_BIAS_PULL_UP },
	{ QRK3588_PIN(2, QRK3588_GPIO_D, 6), QRK3588_BIAS_PULL_UP },
	{ QRK3588_PIN(2, QRK3588_GPIO_D, 7), QRK3588_BIAS_PULL_UP },
};

struct qrk3588_pinctrl_priv {
	void __iomem *pmu1_ioc;
	void __iomem *pmu2_ioc;
	void __iomem *bus_ioc;
	void __iomem *emmc_ioc;

	/*
	 * GPIO request/free temporarily changes IOMUX to GPIO and restores the
	 * previous selector on free.
	 */
	u8 saved_mux[QRK3588_PIN_COUNT];
	bool saved_mux_valid[QRK3588_PIN_COUNT];
};

static const char *const qrk3588_group_names[] = {
	[QRK3588_GROUP_EMMC] = "emmc",
};

static const char *const qrk3588_function_names[] = {
	[QRK3588_FUNCTION_EMMC] = "emmc",
};

static bool qrk3588_pin_is_valid(unsigned int selector)
{
	unsigned int bank;
	unsigned int group;
	unsigned int pin;

	if (selector >= QRK3588_PIN_COUNT)
		return false;

	bank = QRK3588_PIN_BANK(selector);
	group = QRK3588_PIN_GROUP(selector);
	pin = QRK3588_PIN_INDEX(selector);

	switch (bank) {
	case 0:
		/* GPIO0_B4 and GPIO0_D7 are not exposed as GPIO pads. */
		if (group == QRK3588_GPIO_B && pin == 4)
			return false;
		if (group == QRK3588_GPIO_D && pin == 7)
			return false;
		return true;

	case 1:
		return true;

	case 2:
		/*
		 * GPIO2_A4/A5 and GPIO2_C6/C7 are reserved in the IOMUX
		 * register map.
		 */
		if (group == QRK3588_GPIO_A && (pin == 4 || pin == 5))
			return false;
		if (group == QRK3588_GPIO_C && (pin == 6 || pin == 7))
			return false;
		return true;

	case 3:
		/* GPIO3_D6 and GPIO3_D7 are not exposed as GPIO pads. */
		if (group == QRK3588_GPIO_D && pin >= 6)
			return false;
		return true;

	case 4:
		/*
		 * GPIO4_C7, GPIO4_D6 and GPIO4_D7 are not exposed as
		 * GPIO pads.
		 */
		if (group == QRK3588_GPIO_C && pin == 7)
			return false;
		if (group == QRK3588_GPIO_D && pin >= 6)
			return false;
		return true;

	default:
		return false;
	}
}


/*
 * GPIO controller port numbering is not identical to the physical
 * A/B/C/D pad numbering on every bank.
 *
 * In particular, GPIO2 port 22 is wired to GPIO2_D0 while ports 23 and
 * 24 are unused. Keep this translation separate from the physical pin
 * encoding used by the pinctrl register layer.
 *
 * These ranges mirror the RK3588 GPIO interface table and the
 * gpio-ranges entries in the board DTS.
 */
struct qrk3588_gpio_pin_range {
	u8 bank;
	u8 gpio_start;
	u8 count;
	u16 selector_start;
};

static const struct qrk3588_gpio_pin_range qrk3588_gpio_pin_ranges[] = {
	/* GPIO0: port 12 and port 31 are unused. */
	{ 0,  0, 12, QRK3588_PIN(0, QRK3588_GPIO_A, 0) },
	{ 0, 13, 18, QRK3588_PIN(0, QRK3588_GPIO_B, 5) },

	/* GPIO1: all 32 controller ports map linearly. */
	{ 1,  0, 32, QRK3588_PIN(1, QRK3588_GPIO_A, 0) },

	/*
	 * GPIO2:
	 *   port 0..3   -> A0..A3
	 *   port 6..21  -> A6..C5
	 *   port 22     -> D0
	 *   port 25..31 -> D1..D7
	 */
	{ 2,  0,  4, QRK3588_PIN(2, QRK3588_GPIO_A, 0) },
	{ 2,  6, 16, QRK3588_PIN(2, QRK3588_GPIO_A, 6) },
	{ 2, 22,  1, QRK3588_PIN(2, QRK3588_GPIO_D, 0) },
	{ 2, 25,  7, QRK3588_PIN(2, QRK3588_GPIO_D, 1) },

	/* GPIO3: D6/D7 are unused. */
	{ 3,  0, 30, QRK3588_PIN(3, QRK3588_GPIO_A, 0) },

	/* GPIO4: C7 and D6/D7 are unused. */
	{ 4,  0, 23, QRK3588_PIN(4, QRK3588_GPIO_A, 0) },
	{ 4, 24,  6, QRK3588_PIN(4, QRK3588_GPIO_D, 0) },
};

static int qrk3588_gpio_port_to_selector(unsigned int bank,
					 unsigned int port,
					 unsigned int *selector)
{
	unsigned int i;

	if (bank >= QRK3588_GPIO_BANK_COUNT ||
	    port >= QRK3588_PINS_PER_BANK)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(qrk3588_gpio_pin_ranges); i++) {
		const struct qrk3588_gpio_pin_range *range =
			&qrk3588_gpio_pin_ranges[i];

		if (range->bank != bank)
			continue;

		if (port < range->gpio_start ||
		    port >= range->gpio_start + range->count)
			continue;

		*selector = range->selector_start +
			    (port - range->gpio_start);

		if (!qrk3588_pin_is_valid(*selector))
			return -ENODEV;

		return 0;
	}

	return -ENODEV;
}

static void qrk3588_hiword_update(void __iomem *base, u32 offset,
				  u16 mask, u16 value)
{
	writel(((u32)mask << 16) | (value & mask),
	       (u8 __iomem *)base + offset);
}

static void qrk3588_hiword_field_update(void __iomem *base, u32 offset,
					unsigned int shift,
					unsigned int width,
					u16 value)
{
	u16 mask = ((1U << width) - 1U) << shift;

	qrk3588_hiword_update(base, offset, mask, value << shift);
}

static int qrk3588_pin_mux_reg(struct qrk3588_pinctrl_priv *priv,
			       unsigned int selector,
			       void __iomem **reg,
			       unsigned int *shift)
{
	unsigned int bank;
	unsigned int group;
	unsigned int pin;
	u32 offset;

	if (!qrk3588_pin_is_valid(selector))
		return -ENOTSUPP;

	bank = QRK3588_PIN_BANK(selector);
	group = QRK3588_PIN_GROUP(selector);
	pin = QRK3588_PIN_INDEX(selector);

	/*
	 * GPIO1..GPIO4 use the regular BUS_IOC A/B/C/D layout.
	 */
	if (bank >= 1 && bank <= 4) {
		offset = bank * BUS_IOC_BANK_STRIDE +
			 group * BUS_IOC_GROUP_STRIDE +
			 (pin >= 4 ? BUS_IOC_HALF_STRIDE : 0);

		*reg = (u8 __iomem *)priv->bus_ioc + offset;
		*shift = (pin & 0x3) * 4;
		return 0;
	}

	/*
	 * GPIO0 is physically split across PMU1_IOC and PMU2_IOC.
	 */
	switch (group) {
	case QRK3588_GPIO_A:
		*reg = (u8 __iomem *)priv->pmu1_ioc +
		       (pin < 4 ? PMU1_IOC_GPIO0A_IOMUX_SEL_L :
				  PMU1_IOC_GPIO0A_IOMUX_SEL_H);
		*shift = (pin & 0x3) * 4;
		return 0;

	case QRK3588_GPIO_B:
		if (pin < 4) {
			*reg = (u8 __iomem *)priv->pmu1_ioc +
			       PMU1_IOC_GPIO0B_IOMUX_SEL_L;
			*shift = pin * 4;
			return 0;
		}

		/* B4 is rejected by qrk3588_pin_is_valid(). */
		*reg = (u8 __iomem *)priv->pmu2_ioc +
		       PMU2_IOC_GPIO0B_IOMUX_SEL_H;
		*shift = (pin - 4) * 4;
		return 0;

	case QRK3588_GPIO_C:
		*reg = (u8 __iomem *)priv->pmu2_ioc +
		       (pin < 4 ? PMU2_IOC_GPIO0C_IOMUX_SEL_L :
				  PMU2_IOC_GPIO0C_IOMUX_SEL_H);
		*shift = (pin & 0x3) * 4;
		return 0;

	case QRK3588_GPIO_D:
		*reg = (u8 __iomem *)priv->pmu2_ioc +
		       (pin < 4 ? PMU2_IOC_GPIO0D_IOMUX_SEL_L :
				  PMU2_IOC_GPIO0D_IOMUX_SEL_H);
		*shift = (pin & 0x3) * 4;
		return 0;

	default:
		return -EINVAL;
	}
}

static int qrk3588_mux_get(struct qrk3588_pinctrl_priv *priv,
			   unsigned int selector,
			   u8 *mux)
{
	void __iomem *reg;
	unsigned int shift;
	int ret;

	ret = qrk3588_pin_mux_reg(priv, selector, &reg, &shift);
	if (ret)
		return ret;

	*mux = (readl(reg) >> shift) & 0xf;

	return 0;
}

static int qrk3588_mux_set(struct qrk3588_pinctrl_priv *priv,
			   unsigned int selector,
			   u8 mux)
{
	void __iomem *reg;
	unsigned int shift;
	u16 mask;
	u16 value;
	int ret;

	ret = qrk3588_pin_mux_reg(priv, selector, &reg, &shift);
	if (ret)
		return ret;

	mask = (u16)(0xfU << shift);
	value = (u16)((mux & 0xfU) << shift);

	writel(((u32)mask << 16) | value, reg);

	return 0;
}

static int qrk3588_emmc_pad_regs(unsigned int selector,
				 u32 *ds_reg,
				 unsigned int *ds_shift,
				 u32 *pull_reg,
				 unsigned int *pull_shift,
				 u32 *ie_reg,
				 unsigned int *ie_shift,
				 u32 *smt_reg,
				 unsigned int *smt_shift)
{
	unsigned int bank = QRK3588_PIN_BANK(selector);
	unsigned int group = QRK3588_PIN_GROUP(selector);
	unsigned int pin = QRK3588_PIN_INDEX(selector);

	if (bank != 2)
		return -EINVAL;

	if (group == QRK3588_GPIO_A && pin <= 3) {
		*ds_reg = EMMC_IOC_GPIO2A_DS_L;
		*ds_shift = pin * 4;
		*pull_reg = EMMC_IOC_GPIO2A_P;
		*pull_shift = pin * 2;
		*ie_reg = EMMC_IOC_GPIO2A_IE;
		*ie_shift = pin;
		*smt_reg = EMMC_IOC_GPIO2A_SMT;
		*smt_shift = pin;
		return 0;
	}

	if (group == QRK3588_GPIO_D) {
		*ds_reg = pin < 4 ? EMMC_IOC_GPIO2D_DS_L :
				   EMMC_IOC_GPIO2D_DS_H;
		*ds_shift = (pin & 0x3) * 4;
		*pull_reg = EMMC_IOC_GPIO2D_P;
		*pull_shift = pin * 2;
		*ie_reg = EMMC_IOC_GPIO2D_IE;
		*ie_shift = pin;
		*smt_reg = EMMC_IOC_GPIO2D_SMT;
		*smt_shift = pin;
		return 0;
	}

	return -ENOTSUPP;
}

static int qrk3588_emmc_set_bias(struct qrk3588_pinctrl_priv *priv,
				 unsigned int selector,
				 enum qrk3588_bias bias)
{
	u32 ds_reg, pull_reg, ie_reg, smt_reg;
	unsigned int ds_shift, pull_shift, ie_shift, smt_shift;
	u16 value;
	int ret;

	ret = qrk3588_emmc_pad_regs(selector,
				    &ds_reg, &ds_shift,
				    &pull_reg, &pull_shift,
				    &ie_reg, &ie_shift,
				    &smt_reg, &smt_shift);
	if (ret)
		return ret;

	switch (bias) {
	case QRK3588_BIAS_DISABLE:
		value = 0x0; /* PE=0, PS is don't-care */
		break;
	case QRK3588_BIAS_PULL_DOWN:
		value = 0x1; /* PE=1, PS=0 */
		break;
	case QRK3588_BIAS_PULL_UP:
		value = 0x3; /* PE=1, PS=1 */
		break;
	default:
		return -EINVAL;
	}

	qrk3588_hiword_field_update(priv->emmc_ioc, pull_reg,
				    pull_shift, 2, value);

	return 0;
}

static int qrk3588_configure_emmc_pad(struct qrk3588_pinctrl_priv *priv,
				      const struct qrk3588_emmc_pad *pad)
{
	u32 ds_reg, pull_reg, ie_reg, smt_reg;
	unsigned int ds_shift, pull_shift, ie_shift, smt_shift;
	int ret;

	ret = qrk3588_emmc_pad_regs(pad->selector,
				    &ds_reg, &ds_shift,
				    &pull_reg, &pull_shift,
				    &ie_reg, &ie_shift,
				    &smt_reg, &smt_shift);
	if (ret)
		return ret;

	/* Deterministically restore the documented 40-ohm drive setting. */
	qrk3588_hiword_field_update(priv->emmc_ioc, ds_reg,
				    ds_shift, 3, EMMC_DS_40OHM);

	ret = qrk3588_emmc_set_bias(priv, pad->selector, pad->bias);
	if (ret)
		return ret;

	/* Input buffer enabled, matching the documented reset/default state. */
	qrk3588_hiword_field_update(priv->emmc_ioc, ie_reg,
				    ie_shift, 1, 1);

	/*
	 * CMOS input mode. The supplied RK3588 documentation does not require
	 * Schmitt mode for these eMMC pins.
	 */
	qrk3588_hiword_field_update(priv->emmc_ioc, smt_reg,
				    smt_shift, 1, 0);

	return 0;
}

static int qrk3588_configure_emmc(struct qrk3588_pinctrl_priv *priv)
{
	unsigned int i;
	int ret;

	/*
	 * Establish electrical state before switching the pads to the eMMC
	 * peripheral function.
	 */
	for (i = 0; i < ARRAY_SIZE(qrk3588_emmc_pads); i++) {
		ret = qrk3588_configure_emmc_pad(priv,
						&qrk3588_emmc_pads[i]);
		if (ret)
			return ret;
	}

	for (i = 0; i < ARRAY_SIZE(qrk3588_emmc_pads); i++) {
		ret = qrk3588_mux_set(priv,
				      qrk3588_emmc_pads[i].selector,
				      QRK3588_IOMUX_EMMC);
		if (ret)
			return ret;
	}

	return 0;
}

static int qrk3588_get_groups_count(struct udevice *dev)
{
	return QRK3588_GROUP_COUNT;
}

static const char *qrk3588_get_group_name(struct udevice *dev,
					   unsigned int selector)
{
	if (selector >= QRK3588_GROUP_COUNT)
		return NULL;

	return qrk3588_group_names[selector];
}

static int qrk3588_get_functions_count(struct udevice *dev)
{
	return QRK3588_FUNCTION_COUNT;
}

static const char *qrk3588_get_function_name(struct udevice *dev,
					      unsigned int selector)
{
	if (selector >= QRK3588_FUNCTION_COUNT)
		return NULL;

	return qrk3588_function_names[selector];
}

static int qrk3588_pinmux_group_set(struct udevice *dev,
				    unsigned int group_selector,
				    unsigned int func_selector)
{
	struct qrk3588_pinctrl_priv *priv = dev_get_priv(dev);

	if (group_selector != QRK3588_GROUP_EMMC ||
	    func_selector != QRK3588_FUNCTION_EMMC)
		return -EINVAL;

	return qrk3588_configure_emmc(priv);
}

static int qrk3588_gpio_request_enable(struct udevice *dev,
				       unsigned int selector)
{
	struct qrk3588_pinctrl_priv *priv = dev_get_priv(dev);
	u8 mux;
	int ret;

	if (!qrk3588_pin_is_valid(selector))
		return -ENOTSUPP;

	if (priv->saved_mux_valid[selector])
		return -EBUSY;

	ret = qrk3588_mux_get(priv, selector, &mux);
	if (ret)
		return ret;

	ret = qrk3588_mux_set(priv, selector, QRK3588_IOMUX_GPIO);
	if (ret)
		return ret;

	priv->saved_mux[selector] = mux;
	priv->saved_mux_valid[selector] = true;

	return 0;
}

static int qrk3588_gpio_disable_free(struct udevice *dev,
				     unsigned int selector)
{
	struct qrk3588_pinctrl_priv *priv = dev_get_priv(dev);
	int ret;

	if (!qrk3588_pin_is_valid(selector))
		return -ENOTSUPP;

	if (!priv->saved_mux_valid[selector])
		return 0;

	ret = qrk3588_mux_set(priv, selector, priv->saved_mux[selector]);
	if (ret)
		return ret;

	priv->saved_mux_valid[selector] = false;

	return 0;
}

static int qrk3588_get_gpio_mux(struct udevice *dev, int banknum, int index)
{
	struct qrk3588_pinctrl_priv *priv = dev_get_priv(dev);
	unsigned int selector;
	u8 mux;
	int ret;

	if (banknum < 0 || index < 0)
		return -EINVAL;

	ret = qrk3588_gpio_port_to_selector(banknum, index, &selector);
	if (ret)
		return ret;

	ret = qrk3588_mux_get(priv, selector, &mux);
	if (ret)
		return ret;

	return mux;
}

static const struct pinctrl_ops qrk3588_pinctrl_ops = {
	.get_groups_count	= qrk3588_get_groups_count,
	.get_group_name		= qrk3588_get_group_name,
	.get_functions_count	= qrk3588_get_functions_count,
	.get_function_name	= qrk3588_get_function_name,
	.pinmux_group_set	= qrk3588_pinmux_group_set,
	.gpio_request_enable	= qrk3588_gpio_request_enable,
	.gpio_disable_free	= qrk3588_gpio_disable_free,
	.get_gpio_mux		= qrk3588_get_gpio_mux,
	.set_state		= pinctrl_generic_set_state,
};

static int qrk3588_pinctrl_probe(struct udevice *dev)
{
	struct qrk3588_pinctrl_priv *priv = dev_get_priv(dev);

	priv->pmu1_ioc = dev_remap_addr_name(dev, "pmu1-ioc");
	priv->pmu2_ioc = dev_remap_addr_name(dev, "pmu2-ioc");
	priv->bus_ioc = dev_remap_addr_name(dev, "bus-ioc");
	priv->emmc_ioc = dev_remap_addr_name(dev, "emmc-ioc");

	if (!priv->pmu1_ioc || !priv->pmu2_ioc ||
	    !priv->bus_ioc || !priv->emmc_ioc)
		return -EINVAL;

	return 0;
}

static const struct udevice_id qrk3588_pinctrl_ids[] = {
	{ .compatible = "quest,qrk3588-pinctrl" },
	{ }
};

U_BOOT_DRIVER(qrk3588_pinctrl) = {
	.name		= "qrk3588_pinctrl",
	.id		= UCLASS_PINCTRL,
	.of_match	= qrk3588_pinctrl_ids,
	.ops		= &qrk3588_pinctrl_ops,
	.probe		= qrk3588_pinctrl_probe,
	.priv_auto	= sizeof(struct qrk3588_pinctrl_priv),
};
