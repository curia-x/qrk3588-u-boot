// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2026 ninozhang001@gmail.com
 *
 * QRK3588 GPIO controller
 */

#include <clk.h>
#include <dm.h>
#include <dm/pinctrl.h>
#include <dm/read.h>
#include <dm/uclass.h>
#include <errno.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/libfdt.h>
#include <linux/types.h>
#include <stdio.h>
#include <board/quest/qrk3588_gpio.h>

struct qrk3588_gpio_priv {
	void __iomem *base;
	struct clk pclk;
	struct udevice *pinctrl;
	u32 bank_id;
	u32 valid_mask;
	char bank_name[12];
};

/*
 * Valid pads per GPIO controller, derived from the RK3588 GPIO interface
 * table and IOMUX register map.
 *
 * The GPIO controller is 32 bits wide, but not every logical bit has a
 * corresponding external GPIO pad.
 */
static u32 qrk3588_gpio_hw_valid_mask(unsigned int bank)
{
	u32 mask = 0xffffffffU;

	switch (bank) {
	case 0:
		/* GPIO0 ports 12 and 31 are unused. */
		mask &= ~BIT(12);
		mask &= ~BIT(31);
		return mask;

	case 1:
		return mask;

	case 2:
		/*
		 * GPIO2 controller port numbering has holes at 4, 5, 23 and 24.
		 * Port 22 is valid and is physically connected to GPIO2_D0.
		 */
		mask &= ~BIT(4);
		mask &= ~BIT(5);
		mask &= ~BIT(23);
		mask &= ~BIT(24);
		return mask;

	case 3:
		/* GPIO3 ports 30 and 31 are unused. */
		mask &= ~BIT(30);
		mask &= ~BIT(31);
		return mask;

	case 4:
		/* GPIO4 ports 23, 30 and 31 are unused. */
		mask &= ~BIT(23);
		mask &= ~BIT(30);
		mask &= ~BIT(31);
		return mask;

	default:
		return 0;
	}
}

static int qrk3588_gpio_validate(struct qrk3588_gpio_priv *priv,
				 unsigned int offset)
{
	if (offset >= QRK3588_GPIO_PORT_COUNT)
		return -EINVAL;

	if (!(priv->valid_mask & BIT(offset)))
		return -ENODEV;

	return 0;
}

static void __iomem *qrk3588_gpio_half_reg(struct qrk3588_gpio_priv *priv,
					   u32 low_offset,
					   unsigned int offset)
{
	return (u8 __iomem *)priv->base +
	       low_offset + (offset >= 16 ? 4 : 0);
}

static u32 qrk3588_gpio_half_bit(unsigned int offset)
{
	return BIT(offset & 0xf);
}

static void qrk3588_gpio_write_bit(struct qrk3588_gpio_priv *priv,
				   u32 low_offset,
				   unsigned int offset,
				   bool value)
{
	void __iomem *reg = qrk3588_gpio_half_reg(priv, low_offset, offset);
	u32 bit = qrk3588_gpio_half_bit(offset);
	u32 data = bit << 16;

	if (value)
		data |= bit;

	/*
	 * GPIO DR/DDR registers use per-bit hiword write masks:
	 * upper 16 bits enable writes to the corresponding lower 16 bits.
	 */
	writel(data, reg);
}

static int qrk3588_gpio_request(struct udevice *dev,
				unsigned int offset,
				const char *label)
{
	struct qrk3588_gpio_priv *priv = dev_get_priv(dev);
	int ret;

	ret = qrk3588_gpio_validate(priv, offset);
	if (ret)
		return ret;

	/*
	 * gpio-ranges translates this controller-local offset to the pinctrl
	 * scalar selector. The pinctrl driver decodes that selector back into
	 * the RK3588 hardware coordinate (bank, A/B/C/D, pin).
	 */
	return pinctrl_gpio_request(dev, offset, label);
}

static int qrk3588_gpio_free(struct udevice *dev, unsigned int offset)
{
	struct qrk3588_gpio_priv *priv = dev_get_priv(dev);
	int ret;

	ret = qrk3588_gpio_validate(priv, offset);
	if (ret)
		return ret;

	return pinctrl_gpio_free(dev, offset);
}

static int qrk3588_gpio_direction_input(struct udevice *dev,
					unsigned int offset)
{
	struct qrk3588_gpio_priv *priv = dev_get_priv(dev);
	int ret;

	ret = qrk3588_gpio_validate(priv, offset);
	if (ret)
		return ret;

	qrk3588_gpio_write_bit(priv, GPIO_SWPORT_DDR_L, offset, false);

	return 0;
}

static int qrk3588_gpio_direction_output(struct udevice *dev,
					 unsigned int offset,
					 int value)
{
	struct qrk3588_gpio_priv *priv = dev_get_priv(dev);
	int ret;

	ret = qrk3588_gpio_validate(priv, offset);
	if (ret)
		return ret;

	/*
	 * Program the output latch before enabling output mode so the pin does
	 * not momentarily drive the opposite level during the direction switch.
	 */
	qrk3588_gpio_write_bit(priv, GPIO_SWPORT_DR_L, offset, !!value);
	qrk3588_gpio_write_bit(priv, GPIO_SWPORT_DDR_L, offset, true);

	return 0;
}

static int qrk3588_gpio_get_value(struct udevice *dev, unsigned int offset)
{
	struct qrk3588_gpio_priv *priv = dev_get_priv(dev);
	u32 value;
	int ret;

	ret = qrk3588_gpio_validate(priv, offset);
	if (ret)
		return ret;

	/*
	 * GPIO_EXT_PORT always reflects the value on the external I/O port,
	 * regardless of GPIO direction.
	 */
	value = readl((u8 __iomem *)priv->base + GPIO_EXT_PORT);

	return !!(value & BIT(offset));
}

static int qrk3588_gpio_set_value(struct udevice *dev,
				  unsigned int offset,
				  int value)
{
	struct qrk3588_gpio_priv *priv = dev_get_priv(dev);
	int ret;

	ret = qrk3588_gpio_validate(priv, offset);
	if (ret)
		return ret;

	qrk3588_gpio_write_bit(priv, GPIO_SWPORT_DR_L, offset, !!value);

	return 0;
}

static int qrk3588_gpio_get_mux(struct qrk3588_gpio_priv *priv,
				unsigned int offset)
{
	/*
	 * offset is the GPIO controller's local port number. The pinctrl
	 * driver owns the SoC-specific port -> physical PAD translation.
	 */
	return pinctrl_get_gpio_mux(priv->pinctrl, priv->bank_id, offset);
}

static int qrk3588_gpio_get_function(struct udevice *dev,
				     unsigned int offset)
{
	struct qrk3588_gpio_priv *priv = dev_get_priv(dev);
	void __iomem *reg;
	u32 bit;
	int mux;
	int ret;

	ret = qrk3588_gpio_validate(priv, offset);
	if (ret)
		return ret;

	mux = qrk3588_gpio_get_mux(priv, offset);
	if (mux < 0)
		return mux;

	if (mux != 0)
		return GPIOF_FUNC;

	reg = qrk3588_gpio_half_reg(priv, GPIO_SWPORT_DDR_L, offset);
	bit = qrk3588_gpio_half_bit(offset);

	return (readl(reg) & bit) ? GPIOF_OUTPUT : GPIOF_INPUT;
}

static int qrk3588_gpio_set_flags(struct udevice *dev,
				  unsigned int offset,
				  ulong flags)
{
	struct qrk3588_gpio_priv *priv = dev_get_priv(dev);
	bool is_input = !!(flags & GPIOD_IS_IN);
	bool is_output = !!(flags & GPIOD_IS_OUT);
	ulong unsupported;
	int ret;

	ret = qrk3588_gpio_validate(priv, offset);
	if (ret)
		return ret;

	unsupported = GPIOD_OPEN_DRAIN |
		      GPIOD_OPEN_SOURCE |
		      GPIOD_PULL_UP |
		      GPIOD_PULL_DOWN |
		      GPIOD_IS_AF;

	if (flags & unsupported)
		return -ENOTSUPP;

	if (is_input && is_output)
		return -EINVAL;

	if ((flags & GPIOD_IS_OUT_ACTIVE) && !is_output)
		return -EINVAL;

	if (is_output)
		return qrk3588_gpio_direction_output(
			dev, offset, !!(flags & GPIOD_IS_OUT_ACTIVE));

	if (is_input)
		return qrk3588_gpio_direction_input(dev, offset);

	return 0;
}

static int qrk3588_gpio_get_flags(struct udevice *dev,
				  unsigned int offset,
				  ulong *flags)
{
	struct qrk3588_gpio_priv *priv = dev_get_priv(dev);
	void __iomem *reg;
	u32 bit;
	int mux;
	int ret;

	ret = qrk3588_gpio_validate(priv, offset);
	if (ret)
		return ret;

	*flags = 0;

	mux = qrk3588_gpio_get_mux(priv, offset);
	if (mux < 0)
		return mux;

	if (mux != 0) {
		*flags = GPIOD_IS_AF;
		return 0;
	}

	reg = qrk3588_gpio_half_reg(priv, GPIO_SWPORT_DDR_L, offset);
	bit = qrk3588_gpio_half_bit(offset);

	if (readl(reg) & bit) {
		*flags |= GPIOD_IS_OUT;

		reg = qrk3588_gpio_half_reg(priv, GPIO_SWPORT_DR_L, offset);
		if (readl(reg) & bit)
			*flags |= GPIOD_IS_OUT_ACTIVE;
	} else {
		*flags |= GPIOD_IS_IN;
	}

	return 0;
}

static int qrk3588_gpio_parse_reserved_ranges(struct udevice *dev,
					      struct qrk3588_gpio_priv *priv)
{
	const fdt32_t *ranges;
	int len;
	int cells;
	int i;

	ranges = dev_read_prop(dev, "gpio-reserved-ranges", &len);
	if (!ranges)
		return 0;

	if (len <= 0 || len % (2 * sizeof(*ranges)))
		return -EINVAL;

	cells = len / sizeof(*ranges);

	for (i = 0; i < cells; i += 2) {
		u32 start = fdt32_to_cpu(ranges[i]);
		u32 count = fdt32_to_cpu(ranges[i + 1]);
		u32 pin;

		if (!count || start >= QRK3588_GPIO_PORT_COUNT ||
		    count > QRK3588_GPIO_PORT_COUNT - start)
			return -EINVAL;

		for (pin = start; pin < start + count; pin++)
			priv->valid_mask &= ~BIT(pin);
	}

	return 0;
}

static int qrk3588_gpio_get_pinctrl(struct udevice *dev,
				    struct udevice **pctldev)
{
	struct ofnode_phandle_args args;
	int ret;

	/*
	 * This binding uses one QRK3588 pinctrl controller for all ranges.
	 * Resolve it from the first gpio-ranges entry.
	 */
	ret = dev_read_phandle_with_args(dev, "gpio-ranges", NULL, 3,
					 0, &args);
	if (ret)
		return ret;

	return uclass_get_device_by_ofnode(UCLASS_PINCTRL,
					   args.node, pctldev);
}

static const struct dm_gpio_ops qrk3588_gpio_ops = {
	.request		= qrk3588_gpio_request,
	.rfree			= qrk3588_gpio_free,
	.direction_input	= qrk3588_gpio_direction_input,
	.direction_output	= qrk3588_gpio_direction_output,
	.get_value		= qrk3588_gpio_get_value,
	.set_value		= qrk3588_gpio_set_value,
	.get_function		= qrk3588_gpio_get_function,
	.set_flags		= qrk3588_gpio_set_flags,
	.get_flags		= qrk3588_gpio_get_flags,
};

static int qrk3588_gpio_probe(struct udevice *dev)
{
	struct gpio_dev_priv *uc_priv = dev_get_uclass_priv(dev);
	struct qrk3588_gpio_priv *priv = dev_get_priv(dev);
	u32 bank_id;
	int ret;

	priv->base = dev_remap_addr(dev);
	if (!priv->base)
		return -EINVAL;

	ret = dev_read_u32(dev, "quest,bank-id", &bank_id);
	if (ret)
		return ret;

	if (bank_id >= QRK3588_GPIO_BANK_COUNT)
		return -EINVAL;

	priv->bank_id = bank_id;
	priv->valid_mask = qrk3588_gpio_hw_valid_mask(bank_id);
	if (!priv->valid_mask)
		return -EINVAL;

	ret = qrk3588_gpio_parse_reserved_ranges(dev, priv);
	if (ret)
		return ret;

	ret = qrk3588_gpio_get_pinctrl(dev, &priv->pinctrl);
	if (ret)
		return ret;

	ret = clk_get_by_name(dev, "pclk", &priv->pclk);
	if (ret)
		return ret;

	ret = clk_enable(&priv->pclk);
	if (ret) {
		/*
		 * clk_get_by_name() has already requested the clock. Release
		 * that request even though enabling the clock failed.
		 */
		clk_release_all(&priv->pclk, 1);
		return ret;
	}

	snprintf(priv->bank_name, sizeof(priv->bank_name),
		 "GPIO%u_", priv->bank_id);

	uc_priv->bank_name = priv->bank_name;
	uc_priv->gpio_count = QRK3588_GPIO_PORT_COUNT;

	return 0;
}

static int qrk3588_gpio_remove(struct udevice *dev)
{
	struct qrk3588_gpio_priv *priv = dev_get_priv(dev);

	/*
	 * Release the clock request acquired by clk_get_by_name().
	 * clk_release_all() also disables the clock before releasing it.
	 */
	return clk_release_all(&priv->pclk, 1);
}

static const struct udevice_id qrk3588_gpio_ids[] = {
	{ .compatible = "quest,qrk3588-gpio" },
	{ }
};

U_BOOT_DRIVER(qrk3588_gpio) = {
	.name		= "qrk3588_gpio",
	.id		= UCLASS_GPIO,
	.of_match	= qrk3588_gpio_ids,
	.ops		= &qrk3588_gpio_ops,
	.probe		= qrk3588_gpio_probe,
	.remove		= qrk3588_gpio_remove,
	.priv_auto	= sizeof(struct qrk3588_gpio_priv),
};
