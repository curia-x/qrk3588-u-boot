// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2026 ninozhang001@gmail.com
 *
 * QRK3588 clock driver.
 */

#include <clk-uclass.h>
#include <dm.h>
#include <dm/read.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <board/quest/qrk3588_cru.h>
#include <dt-bindings/clock/quest,qrk3588-cru-clk.h>

/*
 * Generic capabilities of an exported U-Boot clock.
 * These describe the provider API, not cross-stage ownership.
 */
#define QRK_CLK_CAP_GET_RATE		BIT(0)
#define QRK_CLK_CAP_SET_RATE		BIT(1)
#define QRK_CLK_CAP_ROUND_RATE		BIT(2)
#define QRK_CLK_CAP_GATE			BIT(3)

/*
 * A register field is described by offset/shift/width. width == 0 means that
 * the stage does not exist for this node.
 */
struct qrk3588_clk_field {
	u16 reg;
	u8 shift;
	u8 width;
};

enum qrk3588_gate_domain {
	QRK_GATE_NONE,
	QRK_GATE_CRU,
	QRK_GATE_PMU1_CRU,
	QRK_GATE_PHPTOP_CRU,
};

struct qrk3588_gate_desc {
	enum qrk3588_gate_domain domain;
	u16 reg;
	u8 bit;
};

/* A node may require additional shared gates besides its own output gate. */
struct qrk3588_gate_ref {
	enum qrk3588_gate_domain domain;
	u16 reg;
	u8 bit;
};

struct qrk3588_pll_desc {
	u16 con0;
	u16 con1;
	u16 con2;
	u16 con6;
	u8 mode_shift;
};

enum qrk3588_clk_node_id {
	QRK_NODE_XIN24M,

	QRK_NODE_CPLL,
	QRK_NODE_GPLL,
	QRK_NODE_PPLL,

	QRK_NODE_MATRIX_50,
	QRK_NODE_MATRIX_100,
	QRK_NODE_MATRIX_200,

	QRK_NODE_HCLK_NVM_ROOT,
	QRK_NODE_ACLK_NVM_ROOT,

	QRK_NODE_REF_PIPE_PHY0_OSC_SRC,
	QRK_NODE_REF_PIPE_PHY0_PLL_SRC,
	QRK_NODE_REF_PIPE_PHY0,
	QRK_NODE_REF_PIPE_PHY1_OSC_SRC,
	QRK_NODE_REF_PIPE_PHY1_PLL_SRC,
	QRK_NODE_REF_PIPE_PHY1,

	/* PCIe consumer clocks with all required gate dependencies. */
	QRK_NODE_PCLK_PCIE_1L0,
	QRK_NODE_PCLK_PCIE_1L2,
	QRK_NODE_PCLK_PCIE_COMBO_PIPE_PHY0,
	QRK_NODE_PCLK_PCIE_COMBO_PIPE_PHY1,

	QRK_NODE_CCLK_EMMC,
	QRK_NODE_HCLK_EMMC,
	QRK_NODE_ACLK_EMMC,
	QRK_NODE_BCLK_EMMC,
	QRK_NODE_TMCLK_EMMC,

	QRK_NODE_PCLK_GPIO0,
	QRK_NODE_PCLK_GPIO1,
	QRK_NODE_PCLK_GPIO2,
	QRK_NODE_PCLK_GPIO3,
	QRK_NODE_PCLK_GPIO4,

	QRK_NODE_COUNT,
};

struct qrk3588_clk_priv;
struct qrk3588_clk_node;

struct qrk3588_node_ops {
	int (*get_rate)(struct qrk3588_clk_priv *priv,
			const struct qrk3588_clk_node *node,
			unsigned int depth, ulong *rate);
	int (*round_rate)(struct qrk3588_clk_priv *priv,
			  const struct qrk3588_clk_node *node,
			  ulong requested, ulong *rate);
	int (*set_rate)(struct qrk3588_clk_priv *priv,
			const struct qrk3588_clk_node *node,
			ulong requested, ulong *rate);
};

struct qrk3588_clk_node {
	const char *name;
	const struct qrk3588_node_ops *ops;

	const enum qrk3588_clk_node_id *parents;
	u8 num_parents;

	struct qrk3588_clk_field mux;
	struct qrk3588_clk_field div;

	struct qrk3588_gate_desc gate;

	/*
	 * Shared dependencies are enabled on demand but intentionally left on
	 * when a consumer is disabled. They may be used by another block whose
	 * lifetime is not represented by this exported clock.
	 */
	const struct qrk3588_gate_ref *shared_gates;
	u8 num_shared_gates;

	/*
	 * Owned dependencies have the same lifetime as this exported clock and
	 * are disabled again with the consumer.
	 */
	const struct qrk3588_gate_ref *owned_gates;
	u8 num_owned_gates;

	const struct qrk3588_pll_desc *pll;
	ulong fixed_rate;
	/*
	 * Most exported clocks own a leaf gate.  A mux-only exported clock can
	 * instead request that disable() gate only its currently selected parent.
	 * This is used for the PIPE PHY REF clocks, whose source gates live on the
	 * OSC/PLL branches rather than on the final mux.
	 */
	bool disable_selected_parent;
};

struct qrk3588_export_clk {
	ulong id;
	enum qrk3588_clk_node_id node;
	u32 caps;
};

struct qrk3588_clk_priv {
	void __iomem *base;
	void __iomem *pmu1_base;
	void __iomem *phptop_base;
};

struct qrk3588_rate_candidate {
	bool valid;
	u8 sel;
	u8 div;
	ulong rate;
};

/* Forward declaration used by the specialized PIPE REF mux operations. */
static const struct qrk3588_clk_node qrk3588_nodes[];

#define QRK_FIELD(_reg, _shift, _width)	\
	{ .reg = (_reg), .shift = (_shift), .width = (_width) }

#define QRK_GATE(_domain, _reg, _bit)		\
	{ .domain = (_domain), .reg = (_reg), .bit = (_bit) }

#define QRK_NO_GATE				\
	{ .domain = QRK_GATE_NONE, .reg = 0, .bit = 0 }

static inline void __iomem *qrk3588_reg(struct qrk3588_clk_priv *priv,
					u32 offset)
{
	return (void __iomem *)((u8 __iomem *)priv->base + offset);
}

static u32 qrk3588_read(struct qrk3588_clk_priv *priv, u32 offset)
{
	return readl(qrk3588_reg(priv, offset));
}

/* CRU mux/div/gate registers use a lower-16-bit per-bit write mask. */
static void qrk3588_hiword_update(struct qrk3588_clk_priv *priv, u32 offset,
				  u32 mask, u32 value)
{
	writel((mask << 16) | (value & mask), qrk3588_reg(priv, offset));
}

static u32 qrk3588_field_mask(const struct qrk3588_clk_field *field)
{
	if (!field->width || field->width > 16)
		return 0;

	return GENMASK(field->shift + field->width - 1, field->shift);
}

static u32 qrk3588_field_read(struct qrk3588_clk_priv *priv,
			      const struct qrk3588_clk_field *field)
{
	u32 mask = qrk3588_field_mask(field);

	if (!mask)
		return 0;

	return (qrk3588_read(priv, field->reg) & mask) >> field->shift;
}

static int qrk3588_field_write(struct qrk3588_clk_priv *priv,
			       const struct qrk3588_clk_field *field,
			       u32 raw)
{
	u32 mask = qrk3588_field_mask(field);
	u32 max;

	if (!mask)
		return -EINVAL;

	max = (1U << field->width) - 1;
	if (raw > max)
		return -ERANGE;

	qrk3588_hiword_update(priv, field->reg, mask, raw << field->shift);
	return 0;
}

static void __iomem *qrk3588_gate_addr(struct qrk3588_clk_priv *priv,
				       const struct qrk3588_gate_desc *gate)
{
	switch (gate->domain) {
	case QRK_GATE_CRU:
		return (u8 __iomem *)priv->base + gate->reg;
	case QRK_GATE_PMU1_CRU:
		return (u8 __iomem *)priv->pmu1_base + gate->reg;
	case QRK_GATE_PHPTOP_CRU:
		if (!priv->phptop_base)
			return NULL;
		return (u8 __iomem *)priv->phptop_base + gate->reg;
	default:
		return NULL;
	}
}

static bool qrk3588_gate_is_enabled(struct qrk3588_clk_priv *priv,
				    const struct qrk3588_gate_desc *gate)
{
	void __iomem *addr;

	if (gate->domain == QRK_GATE_NONE)
		return true;

	addr = qrk3588_gate_addr(priv, gate);
	if (!addr)
		return false;

	/* Gate bit high means disabled. */
	return !(readl(addr) & BIT(gate->bit));
}

static int qrk3588_gate_set(struct qrk3588_clk_priv *priv,
			    const struct qrk3588_gate_desc *gate,
			    bool enable)
{
	void __iomem *addr;
	u32 mask;

	if (gate->domain == QRK_GATE_NONE)
		return 0;

	addr = qrk3588_gate_addr(priv, gate);
	if (!addr)
		return -EINVAL;

	mask = BIT(gate->bit);
	writel((mask << 16) | (enable ? 0 : mask), addr);
	return 0;
}

static int qrk3588_gate_ref_set(struct qrk3588_clk_priv *priv,
				const struct qrk3588_gate_ref *ref,
				bool enable)
{
	struct qrk3588_gate_desc gate = {
		.domain = ref->domain,
		.reg = ref->reg,
		.bit = ref->bit,
	};

	return qrk3588_gate_set(priv, &gate, enable);
}

static int qrk3588_node_get_rate_depth(struct qrk3588_clk_priv *priv,
				       enum qrk3588_clk_node_id id,
				       unsigned int depth, ulong *rate);

static int qrk3588_node_current_parent(struct qrk3588_clk_priv *priv,
				       const struct qrk3588_clk_node *node,
				       enum qrk3588_clk_node_id *parent,
				       u32 *sel)
{
	u32 raw;

	if (!node->num_parents)
		return -ENOENT;

	if (node->num_parents == 1 && !node->mux.width) {
		*parent = node->parents[0];
		if (sel)
			*sel = 0;
		return 0;
	}

	if (!node->mux.width)
		return -EINVAL;

	raw = qrk3588_field_read(priv, &node->mux);
	if (raw >= node->num_parents)
		return -EINVAL;

	*parent = node->parents[raw];
	if (sel)
		*sel = raw;

	return 0;
}

static int qrk3588_fixed_get_rate(struct qrk3588_clk_priv *priv,
				  const struct qrk3588_clk_node *node,
				  unsigned int depth, ulong *rate)
{
	(void)priv;
	(void)depth;

	if (!node->fixed_rate)
		return -ENODATA;

	*rate = node->fixed_rate;
	return 0;
}

static int qrk3588_ppll_get_rate(struct qrk3588_clk_priv *priv,
				 const struct qrk3588_clk_node *node,
				 unsigned int depth, ulong *rate)
{
	u32 con0, con1, con2, con6;
	u32 m, p, s;
	s16 k;
	s64 scaled_m;
	s64 numerator;
	s64 denominator;

	(void)node;
	(void)depth;

	if (!priv->phptop_base)
		return -ENODEV;

	con0 = readl((u8 __iomem *)priv->phptop_base + QRK3588_PHPTOP_PPLL_CON0);

	if (con0 & PLL_BYPASS) {
		*rate = QRK3588_XIN24M_RATE;
		return 0;
	}

	con1 = readl((u8 __iomem *)priv->phptop_base + QRK3588_PHPTOP_PPLL_CON1);
	con2 = readl((u8 __iomem *)priv->phptop_base + QRK3588_PHPTOP_PPLL_CON2);
	con6 = readl((u8 __iomem *)priv->phptop_base + QRK3588_PHPTOP_PPLL_CON6);

	if (!(con6 & PLL_LOCK))
		return -EAGAIN;

	m = con0 & PLL_M_MASK;
	p = con1 & PLL_P_MASK;
	s = (con1 & PLL_S_MASK) >> PLL_S_SHIFT;
	k = (s16)(con2 & 0xffff);

	if (m < 64 || m > 1023 || !p || p > 63 || s > 6)
		return -EINVAL;

	/*
	 * RK3588 FRACPLL:
	 *   FOUT = FIN * (M + K / 65536) / P / 2^S
	 *
	 * K is signed two's-complement and FIN is 24 MHz. Keep M+K/65536
	 * scaled by 2^16 so the calculation stays entirely integer.
	 */
	scaled_m = ((s64)m << 16) + k;
	if (scaled_m <= 0)
		return -EINVAL;

	numerator = (s64)QRK3588_XIN24M_RATE * scaled_m;
	denominator = ((s64)p << s) << 16;
	if (denominator <= 0)
		return -EINVAL;

	*rate = (ulong)(numerator / denominator);
	return *rate ? 0 : -EINVAL;
}

static int qrk3588_pll_get_rate(struct qrk3588_clk_priv *priv,
				const struct qrk3588_clk_node *node,
				unsigned int depth, ulong *rate)
{
	const struct qrk3588_pll_desc *pll = node->pll;
	u32 mode_con, mode;
	u32 con0, con1, con2;
	u32 m, p, s;
	s16 k;
	s64 scaled_m, numerator, denominator;

	(void)depth;

	if (!pll)
		return -EINVAL;

	mode_con = qrk3588_read(priv, CRU_MODE_CON00);
	mode = (mode_con >> pll->mode_shift) & PLL_MODE_MASK;

	if (mode == PLL_MODE_SLOW) {
		*rate = QRK3588_XIN24M_RATE;
		return 0;
	}

	/*
	 * clk_deepslow is outside the U-Boot clock closure currently modeled
	 * here. Do not infer its rate from another implementation.
	 */
	if (mode == PLL_MODE_DEEP_SLOW)
		return -ENOTSUPP;

	if (mode != PLL_MODE_NORMAL)
		return -EINVAL;

	con0 = qrk3588_read(priv, pll->con0);
	con1 = qrk3588_read(priv, pll->con1);
	con2 = qrk3588_read(priv, pll->con2);

	if (con0 & PLL_BYPASS) {
		*rate = QRK3588_XIN24M_RATE;
		return 0;
	}

	if (!(qrk3588_read(priv, pll->con6) & PLL_LOCK))
		return -EAGAIN;

	m = con0 & PLL_M_MASK;
	p = con1 & PLL_P_MASK;
	s = (con1 & PLL_S_MASK) >> PLL_S_SHIFT;
	k = (s16)(con2 & 0xffff);

	if (m < 64 || !p || p > 63 || s > 6)
		return -EINVAL;

	/*
	 * FRACPLL:
	 *   Fout = Fin * (m + k / 65536) / (p * 2^s)
	 * K is a signed 16-bit two's-complement value.
	 */
	scaled_m = ((s64)m << 16) + k;
	if (scaled_m <= 0)
		return -EINVAL;

	numerator = (s64)QRK3588_XIN24M_RATE * scaled_m;
	denominator = ((s64)p << s) << 16;
	*rate = (ulong)(numerator / denominator);

	return *rate ? 0 : -EINVAL;
}

static int qrk3588_passthrough_get_rate(struct qrk3588_clk_priv *priv,
					const struct qrk3588_clk_node *node,
					unsigned int depth, ulong *rate)
{
	enum qrk3588_clk_node_id parent;
	int ret;

	ret = qrk3588_node_current_parent(priv, node, &parent, NULL);
	if (ret)
		return ret;

	return qrk3588_node_get_rate_depth(priv, parent, depth + 1, rate);
}

static int qrk3588_muxdiv_get_rate(struct qrk3588_clk_priv *priv,
				   const struct qrk3588_clk_node *node,
				   unsigned int depth, ulong *rate)
{
	enum qrk3588_clk_node_id parent_id;
	ulong parent_rate;
	u32 div = 1;
	int ret;

	ret = qrk3588_node_current_parent(priv, node, &parent_id, NULL);
	if (ret)
		return ret;

	ret = qrk3588_node_get_rate_depth(priv, parent_id, depth + 1,
					  &parent_rate);
	if (ret)
		return ret;

	if (node->div.width)
		div = qrk3588_field_read(priv, &node->div) + 1;

	if (!div)
		return -EINVAL;

	*rate = parent_rate / div;
	return *rate ? 0 : -EINVAL;
}

static void qrk3588_consider_candidate(struct qrk3588_rate_candidate *best,
				       ulong requested, ulong parent,
				       u8 sel, unsigned int max_div)
{
	unsigned int div;

	if (!parent || !requested)
		return;

	for (div = 1; div <= max_div; div++) {
		ulong candidate = parent / div;

		if (!candidate)
			continue;

		if (!best->valid) {
			best->valid = true;
			best->sel = sel;
			best->div = div;
			best->rate = candidate;
			continue;
		}

		/* Peripheral clock requests are treated as a frequency ceiling. */
		if (candidate <= requested) {
			if (best->rate > requested || candidate > best->rate) {
				best->sel = sel;
				best->div = div;
				best->rate = candidate;
			}
		} else if (best->rate > requested && candidate < best->rate) {
			/* If none are <= request, keep the smallest achievable rate. */
			best->sel = sel;
			best->div = div;
			best->rate = candidate;
		}
	}
}

static int qrk3588_muxdiv_choose_rate(struct qrk3588_clk_priv *priv,
				      const struct qrk3588_clk_node *node,
				      ulong requested,
				      struct qrk3588_rate_candidate *best)
{
	int parent_error = -ENODATA;
	bool have_parent = false;
	unsigned int sel;
	unsigned int max_div;

	if (!requested || !node->num_parents)
		return -EINVAL;

	*best = (struct qrk3588_rate_candidate){ 0 };
	max_div = node->div.width ? (1U << node->div.width) : 1;

	for (sel = 0; sel < node->num_parents; sel++) {
		ulong parent_rate;
		int ret;

		ret = qrk3588_node_get_rate_depth(priv, node->parents[sel], 0,
						  &parent_rate);
		if (ret) {
			/*
			 * Preserve the real upstream failure. In particular, an unavailable
			 * upstream parent rate must keep its original error instead of being
			 * collapsed into an unrelated -ERANGE at the end of the search.
			 */
			parent_error = ret;
			continue;
		}

		have_parent = true;
		qrk3588_consider_candidate(best, requested, parent_rate,
					   sel, max_div);
	}

	if (!have_parent)
		return parent_error;

	return best->valid ? 0 : -ERANGE;
}

static int qrk3588_muxdiv_round_rate(struct qrk3588_clk_priv *priv,
				     const struct qrk3588_clk_node *node,
				     ulong requested, ulong *rate)
{
	struct qrk3588_rate_candidate best;
	int ret;

	ret = qrk3588_muxdiv_choose_rate(priv, node, requested, &best);
	if (ret)
		return ret;

	*rate = best.rate;
	return 0;
}

static int qrk3588_muxdiv_set_rate(struct qrk3588_clk_priv *priv,
				   const struct qrk3588_clk_node *node,
				   ulong requested, ulong *rate)
{
	struct qrk3588_rate_candidate best;
	bool was_enabled;
	int ret;

	ret = qrk3588_muxdiv_choose_rate(priv, node, requested, &best);
	if (ret)
		return ret;

	was_enabled = qrk3588_gate_is_enabled(priv, &node->gate);
	if (was_enabled) {
		ret = qrk3588_gate_set(priv, &node->gate, false);
		if (ret)
			return ret;
	}

	if (node->mux.width) {
		ret = qrk3588_field_write(priv, &node->mux, best.sel);
		if (ret)
			goto restore_gate;
	}

	if (node->div.width) {
		ret = qrk3588_field_write(priv, &node->div, best.div - 1);
		if (ret)
			goto restore_gate;
	}

	*rate = best.rate;
	ret = 0;

restore_gate:
	if (was_enabled) {
		int gate_ret = qrk3588_gate_set(priv, &node->gate, true);

		if (!ret)
			ret = gate_ret;
	}

	return ret;
}

/*
 * clk_ref_pipe_phyN is an asymmetric mux:
 *
 *   sel=0: xin24m -> OSC source gate -> final mux
 *   sel=1: PPLL -> 6-bit divider -> PLL source gate -> final mux
 *
 * The generic mux/div node cannot describe a divider that exists on only one
 * parent branch, so the final REF mux has a small specialized rate operation.
 */
static int qrk3588_pipe_ref_exact_pll_rate(
					struct qrk3588_clk_priv *priv,
					const struct qrk3588_clk_node *pll_branch,
					ulong requested, u32 *divisor)
{
	ulong parent_rate;
	unsigned int max_div;
	int ret;

	if (!requested || !pll_branch->div.width ||
		pll_branch->num_parents != 1)
		return -EINVAL;

	ret = qrk3588_node_get_rate_depth(priv, pll_branch->parents[0], 0,
					  &parent_rate);
	if (ret)
		return ret;

	if (parent_rate % requested)
		return -ERANGE;

	*divisor = parent_rate / requested;
	max_div = 1U << pll_branch->div.width;

	if (!*divisor || *divisor > max_div)
		return -ERANGE;

	return 0;
}

static int qrk3588_pipe_ref_round_rate(struct qrk3588_clk_priv *priv,
					   const struct qrk3588_clk_node *node,
					   ulong requested, ulong *rate)
{
	const struct qrk3588_clk_node *pll_branch;
	u32 divisor;
	int ret;

	if (!requested || node->num_parents != 2 || !node->mux.width)
		return -EINVAL;

	if (requested == QRK3588_XIN24M_RATE) {
		*rate = QRK3588_XIN24M_RATE;
		return 0;
	}

	pll_branch = &qrk3588_nodes[node->parents[1]];
	ret = qrk3588_pipe_ref_exact_pll_rate(priv, pll_branch, requested,
						  &divisor);
	if (ret)
		return ret;

	*rate = requested;
	return 0;
}

static int qrk3588_pipe_ref_set_rate(struct qrk3588_clk_priv *priv,
					 const struct qrk3588_clk_node *node,
					 ulong requested, ulong *rate)
{
	const struct qrk3588_clk_node *pll_branch;
	bool was_enabled;
	u32 divisor;
	int ret;

	if (!requested || node->num_parents != 2 || !node->mux.width)
		return -EINVAL;

	/*
	 * The final mux encoding is documented by the TRM:
	 *   0 = oscillator source
	 *   1 = PLL-derived source.
	 */
	if (requested == QRK3588_XIN24M_RATE) {
		ret = qrk3588_field_write(priv, &node->mux, 0);
		if (ret)
			return ret;

		*rate = QRK3588_XIN24M_RATE;
		return 0;
	}

	pll_branch = &qrk3588_nodes[node->parents[1]];

	/*
	 * PCIe REFCLK is an exact protocol clock.  Resolve the single PPLL parent
	 * directly instead of using the generic "best <= requested" policy.
	 * This also preserves an upstream PPLL register/mapping error.
	 */
	ret = qrk3588_pipe_ref_exact_pll_rate(priv, pll_branch, requested,
						  &divisor);
	if (ret)
		return ret;

	was_enabled = qrk3588_gate_is_enabled(priv, &pll_branch->gate);
	if (was_enabled) {
		ret = qrk3588_gate_set(priv, &pll_branch->gate, false);
		if (ret)
			return ret;
	}

	ret = qrk3588_field_write(priv, &pll_branch->div, divisor - 1);
	if (ret)
		goto restore_gate;

	/*
	 * Only select the PLL branch after its divider has been programmed.
	 * clk_enable() will subsequently enable the selected source gate.
	 */
	ret = qrk3588_field_write(priv, &node->mux, 1);
	if (!ret)
		*rate = requested;

restore_gate:
	if (was_enabled) {
		int gate_ret = qrk3588_gate_set(priv, &pll_branch->gate, true);

		if (!ret)
			ret = gate_ret;
	}

	return ret;
}

static const struct qrk3588_node_ops qrk_fixed_ops = {
	.get_rate = qrk3588_fixed_get_rate,
};

static const struct qrk3588_node_ops qrk_pll_ops = {
	.get_rate = qrk3588_pll_get_rate,
};

static const struct qrk3588_node_ops qrk_ppll_ops = {
	.get_rate = qrk3588_ppll_get_rate,
};

static const struct qrk3588_node_ops qrk_passthrough_ops = {
	.get_rate = qrk3588_passthrough_get_rate,
};

static const struct qrk3588_node_ops qrk_muxdiv_ro_ops = {
	.get_rate = qrk3588_muxdiv_get_rate,
};

static const struct qrk3588_node_ops qrk_muxdiv_rw_ops = {
	.get_rate = qrk3588_muxdiv_get_rate,
	.round_rate = qrk3588_muxdiv_round_rate,
	.set_rate = qrk3588_muxdiv_set_rate,
};

static const struct qrk3588_node_ops qrk_pipe_ref_ops = {
	.get_rate = qrk3588_passthrough_get_rate,
	.round_rate = qrk3588_pipe_ref_round_rate,
	.set_rate = qrk3588_pipe_ref_set_rate,
};

static const struct qrk3588_pll_desc qrk3588_cpll = {
	.con0 = CRU_CPLL_CON0,
	.con1 = CRU_CPLL_CON1,
	.con2 = CRU_CPLL_CON2,
	.con6 = CRU_CPLL_CON6,
	.mode_shift = CPLL_MODE_SHIFT,
};

static const struct qrk3588_pll_desc qrk3588_gpll = {
	.con0 = CRU_GPLL_CON0,
	.con1 = CRU_GPLL_CON1,
	.con2 = CRU_GPLL_CON2,
	.con6 = CRU_GPLL_CON6,
	.mode_shift = GPLL_MODE_SHIFT,
};

static const enum qrk3588_clk_node_id parents_pll[] = {
	QRK_NODE_GPLL,
	QRK_NODE_CPLL,
};

static const enum qrk3588_clk_node_id parent_ppll[] = {
	QRK_NODE_PPLL,
};

static const enum qrk3588_clk_node_id parent_xin24m[] = {
	QRK_NODE_XIN24M,
};

/*
 * Final PIPE REF mux parent order mirrors the documented mux encoding:
 * parent[0] = oscillator source, parent[1] = PLL-derived source.
 */
static const enum qrk3588_clk_node_id parents_ref_pipe_phy0[] = {
	QRK_NODE_REF_PIPE_PHY0_OSC_SRC,
	QRK_NODE_REF_PIPE_PHY0_PLL_SRC,
};

static const enum qrk3588_clk_node_id parents_ref_pipe_phy1[] = {
	QRK_NODE_REF_PIPE_PHY1_OSC_SRC,
	QRK_NODE_REF_PIPE_PHY1_PLL_SRC,
};

static const enum qrk3588_clk_node_id parents_cclk_emmc[] = {
	QRK_NODE_GPLL,
	QRK_NODE_CPLL,
	QRK_NODE_XIN24M,
};

static const enum qrk3588_clk_node_id parents_hclk_nvm[] = {
	QRK_NODE_MATRIX_200,
	QRK_NODE_MATRIX_100,
	QRK_NODE_MATRIX_50,
	QRK_NODE_XIN24M,
};

static const enum qrk3588_clk_node_id parent_hclk_emmc[] = {
	QRK_NODE_HCLK_NVM_ROOT,
};

static const enum qrk3588_clk_node_id parent_aclk_emmc[] = {
	QRK_NODE_ACLK_NVM_ROOT,
};

static const enum qrk3588_clk_node_id parent_tmclk_emmc[] = {
	QRK_NODE_XIN24M,
};

static const struct qrk3588_gate_ref hclk_emmc_shared_gates[] = {
	QRK_GATE(QRK_GATE_CRU, CRU_GATE_CON31, 2), /* HCLK_NVM_BIU */
};

static const struct qrk3588_gate_ref aclk_emmc_shared_gates[] = {
	QRK_GATE(QRK_GATE_CRU, CRU_GATE_CON31, 3), /* ACLK_NVM_BIU */
};

/* Shared PHP / PCIe fabric. Keep these enabled once PCIe is in use. */
static const struct qrk3588_gate_ref pcie_shared_gates[] = {
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON32, 12), /* ACLK_PCIE_BIU */
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON32, 9),  /* ACLK_PHP_BIU */
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON32, 8),  /* ACLK_PCIE_BRIDGE */
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON32, 7),  /* ACLK_PHP_ROOT */
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON32, 6),  /* ACLK_PCIE_ROOT */
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON32, 5),  /* PCLK_PHP_BIU */
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON32, 0),  /* PCLK_PHP_ROOT */
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON34, 9),  /* ACLK_MMU_BIU */
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON34, 7),  /* ACLK_MMU_PCIE */
};

/* Clocks private to PCIe1L0 and therefore owned by its consumer clock. */
static const struct qrk3588_gate_ref pcie_1l0_owned_gates[] = {
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON32, 15), /* ACLK_PCIE_1L0_DBI */
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON33, 9),  /* ACLK_PCIE_1L0_SLV */
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON33, 4),  /* ACLK_PCIE_1L0_MSTR */
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON34, 3),  /* CLK_PCIE_1L0_AUX */
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON38, 14), /* CLK_PCIE_1L0_PIPE */
};

/* Clocks private to PCIe1L2 and therefore owned by its consumer clock. */
static const struct qrk3588_gate_ref pcie_1l2_owned_gates[] = {
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON33, 11), /* ACLK_PCIE_1L2_SLV */
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON33, 6),  /* ACLK_PCIE_1L2_MSTR */
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON33, 1),  /* ACLK_PCIE_1L2_DBI */
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON34, 5),  /* CLK_PCIE_1L2_AUX */
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON38, 13), /* CLK_PCIE_1L2_PIPE */
};

static const struct qrk3588_gate_ref pipe_phy0_owned_gates[] = {
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON37, 0), /* CLK_PIPEPHY0_REF */
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON37, 4), /* CLK_PMALIVE0 */
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON38, 3), /* CLK_PIPEPHY0_PIPE_G */
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON38, 6), /* CLK_PIPEPHY0_PIPE_ASIC_G */
	QRK_GATE(QRK_GATE_PHPTOP_CRU, QRK3588_PHPTOP_GATE_CON00, 2),
		/* PCLK_PCIE_COMBO_PIPE_GRF0 */
};

static const struct qrk3588_gate_ref pipe_phy1_owned_gates[] = {
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON37, 1), /* CLK_PIPEPHY1_REF */
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON37, 5), /* CLK_PMALIVE1 */
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON38, 4), /* CLK_PIPEPHY1_PIPE_G */
	QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON38, 7), /* CLK_PIPEPHY1_PIPE_ASIC_G */
	QRK_GATE(QRK_GATE_PHPTOP_CRU, QRK3588_PHPTOP_GATE_CON00, 3),
		/* PCLK_PCIE_COMBO_PIPE_GRF1 */
};

/*
 * Clock graph.
 *
 * For ordinary mux/div/gate nodes, topology and register fields are data.
 * No clock-ID-specific rate, enable, or set-rate code is required.
 *
 * Hardware mux encoding is represented by parent-array order:
 * parent[0] is selector value 0, parent[1] selector value 1, etc.
 */
static const struct qrk3588_clk_node qrk3588_nodes[QRK_NODE_COUNT] = {
	[QRK_NODE_XIN24M] = {
		.name = "xin24m",
		.ops = &qrk_fixed_ops,
		.fixed_rate = QRK3588_XIN24M_RATE,
		.gate = QRK_NO_GATE,
	},

	[QRK_NODE_CPLL] = {
		.name = "cpll",
		.ops = &qrk_pll_ops,
		.pll = &qrk3588_cpll,
		.gate = QRK_NO_GATE,
	},
	[QRK_NODE_GPLL] = {
		.name = "gpll",
		.ops = &qrk_pll_ops,
		.pll = &qrk3588_gpll,
		.gate = QRK_NO_GATE,
	},
	[QRK_NODE_PPLL] = {
		.name = "ppll",
		.ops = &qrk_ppll_ops,
		.gate = QRK_NO_GATE,
	},

	[QRK_NODE_MATRIX_50] = {
		.name = "matrix50",
		.ops = &qrk_muxdiv_ro_ops,
		.parents = parents_pll,
		.num_parents = ARRAY_SIZE(parents_pll),
		.mux = QRK_FIELD(CRU_CLKSEL_CON00, 5, 1),
		.div = QRK_FIELD(CRU_CLKSEL_CON00, 0, 5),
		.gate = QRK_GATE(QRK_GATE_CRU, CRU_GATE_CON00, 0),
	},
	[QRK_NODE_MATRIX_100] = {
		.name = "matrix100",
		.ops = &qrk_muxdiv_ro_ops,
		.parents = parents_pll,
		.num_parents = ARRAY_SIZE(parents_pll),
		.mux = QRK_FIELD(CRU_CLKSEL_CON00, 11, 1),
		.div = QRK_FIELD(CRU_CLKSEL_CON00, 6, 5),
		.gate = QRK_GATE(QRK_GATE_CRU, CRU_GATE_CON00, 1),
	},
	[QRK_NODE_MATRIX_200] = {
		.name = "matrix200",
		.ops = &qrk_muxdiv_ro_ops,
		.parents = parents_pll,
		.num_parents = ARRAY_SIZE(parents_pll),
		.mux = QRK_FIELD(CRU_CLKSEL_CON01, 11, 1),
		.div = QRK_FIELD(CRU_CLKSEL_CON01, 6, 5),
		.gate = QRK_GATE(QRK_GATE_CRU, CRU_GATE_CON00, 3),
	},

	[QRK_NODE_HCLK_NVM_ROOT] = {
		.name = "hclk_nvm_root",
		.ops = &qrk_muxdiv_ro_ops,
		.parents = parents_hclk_nvm,
		.num_parents = ARRAY_SIZE(parents_hclk_nvm),
		.mux = QRK_FIELD(CRU_CLKSEL_CON77, 0, 2),
		.gate = QRK_GATE(QRK_GATE_CRU, CRU_GATE_CON31, 0),
	},
	[QRK_NODE_ACLK_NVM_ROOT] = {
		.name = "aclk_nvm_root",
		.ops = &qrk_muxdiv_ro_ops,
		.parents = parents_pll,
		.num_parents = ARRAY_SIZE(parents_pll),
		.mux = QRK_FIELD(CRU_CLKSEL_CON77, 7, 1),
		.div = QRK_FIELD(CRU_CLKSEL_CON77, 2, 5),
		.gate = QRK_GATE(QRK_GATE_CRU, CRU_GATE_CON31, 1),
	},

	[QRK_NODE_REF_PIPE_PHY0_OSC_SRC] = {
		.name = "ref_pipe_phy0_osc_src",
		.ops = &qrk_passthrough_ops,
		.parents = parent_xin24m,
		.num_parents = ARRAY_SIZE(parent_xin24m),
		.gate = QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON77, 0),
	},
	[QRK_NODE_REF_PIPE_PHY0_PLL_SRC] = {
		.name = "ref_pipe_phy0_pll_src",
		.ops = &qrk_muxdiv_rw_ops,
		.parents = parent_ppll,
		.num_parents = ARRAY_SIZE(parent_ppll),
		.div = QRK_FIELD(QRK3588_CRU_CLKSEL_CON176, 0, 6),
		.gate = QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON77, 3),
	},
	[QRK_NODE_REF_PIPE_PHY0] = {
		.name = "ref_pipe_phy0",
		.ops = &qrk_pipe_ref_ops,
		.parents = parents_ref_pipe_phy0,
		.num_parents = ARRAY_SIZE(parents_ref_pipe_phy0),
		.mux = QRK_FIELD(QRK3588_CRU_CLKSEL_CON177, 6, 1),
		.gate = QRK_NO_GATE,
		.disable_selected_parent = true,
	},

	[QRK_NODE_REF_PIPE_PHY1_OSC_SRC] = {
		.name = "ref_pipe_phy1_osc_src",
		.ops = &qrk_passthrough_ops,
		.parents = parent_xin24m,
		.num_parents = ARRAY_SIZE(parent_xin24m),
		.gate = QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON77, 1),
	},
	[QRK_NODE_REF_PIPE_PHY1_PLL_SRC] = {
		.name = "ref_pipe_phy1_pll_src",
		.ops = &qrk_muxdiv_rw_ops,
		.parents = parent_ppll,
		.num_parents = ARRAY_SIZE(parent_ppll),
		.div = QRK_FIELD(QRK3588_CRU_CLKSEL_CON176, 6, 6),
		.gate = QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON77, 4),
	},
	[QRK_NODE_REF_PIPE_PHY1] = {
		.name = "ref_pipe_phy1",
		.ops = &qrk_pipe_ref_ops,
		.parents = parents_ref_pipe_phy1,
		.num_parents = ARRAY_SIZE(parents_ref_pipe_phy1),
		.mux = QRK_FIELD(QRK3588_CRU_CLKSEL_CON177, 7, 1),
		.gate = QRK_NO_GATE,
		.disable_selected_parent = true,
	},

	/*
	 * PCIe gate-only consumer clocks. Shared PHP/MMU fabric stays enabled;
	 * controller- and PHY-private dependencies follow the consumer lifetime.
	 * No rate is invented for bus/PIPE clocks whose parent rates are outside
	 * the current QuestSDK clock graph.
	 */
	[QRK_NODE_PCLK_PCIE_1L0] = {
		.name = "pclk_pcie_1l0",
		.gate = QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON33, 14),
		.shared_gates = pcie_shared_gates,
		.num_shared_gates = ARRAY_SIZE(pcie_shared_gates),
		.owned_gates = pcie_1l0_owned_gates,
		.num_owned_gates = ARRAY_SIZE(pcie_1l0_owned_gates),
	},
	[QRK_NODE_PCLK_PCIE_1L2] = {
		.name = "pclk_pcie_1l2",
		.gate = QRK_GATE(QRK_GATE_CRU, QRK3588_CRU_GATE_CON34, 0),
		.shared_gates = pcie_shared_gates,
		.num_shared_gates = ARRAY_SIZE(pcie_shared_gates),
		.owned_gates = pcie_1l2_owned_gates,
		.num_owned_gates = ARRAY_SIZE(pcie_1l2_owned_gates),
	},
	[QRK_NODE_PCLK_PCIE_COMBO_PIPE_PHY0] = {
		.name = "pclk_pcie_combo_pipe_phy0",
		.gate = QRK_GATE(QRK_GATE_PHPTOP_CRU, QRK3588_PHPTOP_GATE_CON00, 5),
		.owned_gates = pipe_phy0_owned_gates,
		.num_owned_gates = ARRAY_SIZE(pipe_phy0_owned_gates),
	},
	[QRK_NODE_PCLK_PCIE_COMBO_PIPE_PHY1] = {
		.name = "pclk_pcie_combo_pipe_phy1",
		.gate = QRK_GATE(QRK_GATE_PHPTOP_CRU, QRK3588_PHPTOP_GATE_CON00, 6),
		.owned_gates = pipe_phy1_owned_gates,
		.num_owned_gates = ARRAY_SIZE(pipe_phy1_owned_gates),
	},

	[QRK_NODE_CCLK_EMMC] = {
		.name = "cclk_emmc",
		.ops = &qrk_muxdiv_rw_ops,
		.parents = parents_cclk_emmc,
		.num_parents = ARRAY_SIZE(parents_cclk_emmc),
		.mux = QRK_FIELD(CRU_CLKSEL_CON77, 14, 2),
		.div = QRK_FIELD(CRU_CLKSEL_CON77, 8, 6),
		.gate = QRK_GATE(QRK_GATE_CRU, CRU_GATE_CON31, 6),
	},
	[QRK_NODE_HCLK_EMMC] = {
		.name = "hclk_emmc",
		.ops = &qrk_passthrough_ops,
		.parents = parent_hclk_emmc,
		.num_parents = ARRAY_SIZE(parent_hclk_emmc),
		.gate = QRK_GATE(QRK_GATE_CRU, CRU_GATE_CON31, 4),
		.shared_gates = hclk_emmc_shared_gates,
		.num_shared_gates = ARRAY_SIZE(hclk_emmc_shared_gates),
	},
	[QRK_NODE_ACLK_EMMC] = {
		.name = "aclk_emmc",
		.ops = &qrk_passthrough_ops,
		.parents = parent_aclk_emmc,
		.num_parents = ARRAY_SIZE(parent_aclk_emmc),
		.gate = QRK_GATE(QRK_GATE_CRU, CRU_GATE_CON31, 5),
		.shared_gates = aclk_emmc_shared_gates,
		.num_shared_gates = ARRAY_SIZE(aclk_emmc_shared_gates),
	},
	[QRK_NODE_BCLK_EMMC] = {
		.name = "bclk_emmc",
		.ops = &qrk_muxdiv_rw_ops,
		.parents = parents_pll,
		.num_parents = ARRAY_SIZE(parents_pll),
		.mux = QRK_FIELD(CRU_CLKSEL_CON78, 5, 1),
		.div = QRK_FIELD(CRU_CLKSEL_CON78, 0, 5),
		.gate = QRK_GATE(QRK_GATE_CRU, CRU_GATE_CON31, 7),
	},
	[QRK_NODE_TMCLK_EMMC] = {
		.name = "tmclk_emmc",
		.ops = &qrk_passthrough_ops,
		.parents = parent_tmclk_emmc,
		.num_parents = ARRAY_SIZE(parent_tmclk_emmc),
		.gate = QRK_GATE(QRK_GATE_CRU, CRU_GATE_CON31, 8),
	},

	[QRK_NODE_PCLK_GPIO0] = {
		.name = "pclk_gpio0",
		.gate = QRK_GATE(QRK_GATE_PMU1_CRU, PMU1_CRU_GATE_CON05, 5),
	},
	[QRK_NODE_PCLK_GPIO1] = {
		.name = "pclk_gpio1",
		.gate = QRK_GATE(QRK_GATE_CRU, CRU_GATE_CON16, 14),
	},
	[QRK_NODE_PCLK_GPIO2] = {
		.name = "pclk_gpio2",
		.gate = QRK_GATE(QRK_GATE_CRU, CRU_GATE_CON17, 0),
	},
	[QRK_NODE_PCLK_GPIO3] = {
		.name = "pclk_gpio3",
		.gate = QRK_GATE(QRK_GATE_CRU, CRU_GATE_CON17, 2),
	},
	[QRK_NODE_PCLK_GPIO4] = {
		.name = "pclk_gpio4",
		.gate = QRK_GATE(QRK_GATE_CRU, CRU_GATE_CON17, 4),
	},
};

/*
 * Public U-Boot clock IDs are intentionally separate from internal graph IDs.
 * Internal upstream nodes therefore remain implementation details and cannot
 * accidentally become consumer-visible clocks.
 */
static const struct qrk3588_export_clk qrk3588_exports[] = {
	{
		.id = QRK3588_CLK_REF_PIPE_PHY0,
		.node = QRK_NODE_REF_PIPE_PHY0,
		.caps = QRK_CLK_CAP_GET_RATE | QRK_CLK_CAP_SET_RATE |
			QRK_CLK_CAP_ROUND_RATE | QRK_CLK_CAP_GATE,
	},
	{
		.id = QRK3588_CLK_REF_PIPE_PHY1,
		.node = QRK_NODE_REF_PIPE_PHY1,
		.caps = QRK_CLK_CAP_GET_RATE | QRK_CLK_CAP_SET_RATE |
			QRK_CLK_CAP_ROUND_RATE | QRK_CLK_CAP_GATE,
	},
	{
		.id = QRK3588_CLK_PCLK_PCIE_1L0,
		.node = QRK_NODE_PCLK_PCIE_1L0,
		.caps = QRK_CLK_CAP_GATE,
	},
	{
		.id = QRK3588_CLK_PCLK_PCIE_1L2,
		.node = QRK_NODE_PCLK_PCIE_1L2,
		.caps = QRK_CLK_CAP_GATE,
	},
	{
		.id = QRK3588_CLK_PCLK_PCIE_COMBO_PIPE_PHY0,
		.node = QRK_NODE_PCLK_PCIE_COMBO_PIPE_PHY0,
		.caps = QRK_CLK_CAP_GATE,
	},
	{
		.id = QRK3588_CLK_PCLK_PCIE_COMBO_PIPE_PHY1,
		.node = QRK_NODE_PCLK_PCIE_COMBO_PIPE_PHY1,
		.caps = QRK_CLK_CAP_GATE,
	},
	{
		.id = QRK3588_CLK_CCLK_EMMC,
		.node = QRK_NODE_CCLK_EMMC,
		.caps = QRK_CLK_CAP_GET_RATE | QRK_CLK_CAP_SET_RATE |
			QRK_CLK_CAP_ROUND_RATE | QRK_CLK_CAP_GATE,
	},
	{
		.id = QRK3588_CLK_HCLK_EMMC,
		.node = QRK_NODE_HCLK_EMMC,
		.caps = QRK_CLK_CAP_GET_RATE | QRK_CLK_CAP_GATE,
	},
	{
		.id = QRK3588_CLK_ACLK_EMMC,
		.node = QRK_NODE_ACLK_EMMC,
		.caps = QRK_CLK_CAP_GET_RATE | QRK_CLK_CAP_GATE,
	},
	{
		.id = QRK3588_CLK_BCLK_EMMC,
		.node = QRK_NODE_BCLK_EMMC,
		.caps = QRK_CLK_CAP_GET_RATE | QRK_CLK_CAP_SET_RATE |
			QRK_CLK_CAP_ROUND_RATE | QRK_CLK_CAP_GATE,
	},
	{
		.id = QRK3588_CLK_TMCLK_EMMC,
		.node = QRK_NODE_TMCLK_EMMC,
		.caps = QRK_CLK_CAP_GET_RATE | QRK_CLK_CAP_GATE,
	},
	{
		.id = QRK3588_CLK_PCLK_GPIO0,
		.node = QRK_NODE_PCLK_GPIO0,
		.caps = QRK_CLK_CAP_GATE,
	},
	{
		.id = QRK3588_CLK_PCLK_GPIO1,
		.node = QRK_NODE_PCLK_GPIO1,
		.caps = QRK_CLK_CAP_GATE,
	},
	{
		.id = QRK3588_CLK_PCLK_GPIO2,
		.node = QRK_NODE_PCLK_GPIO2,
		.caps = QRK_CLK_CAP_GATE,
	},
	{
		.id = QRK3588_CLK_PCLK_GPIO3,
		.node = QRK_NODE_PCLK_GPIO3,
		.caps = QRK_CLK_CAP_GATE,
	},
	{
		.id = QRK3588_CLK_PCLK_GPIO4,
		.node = QRK_NODE_PCLK_GPIO4,
		.caps = QRK_CLK_CAP_GATE,
	},
};

static const struct qrk3588_export_clk *qrk3588_find_export(ulong id)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(qrk3588_exports); i++) {
		if (qrk3588_exports[i].id == id)
			return &qrk3588_exports[i];
	}

	return NULL;
}

static int qrk3588_node_get_rate_depth(struct qrk3588_clk_priv *priv,
				       enum qrk3588_clk_node_id id,
				       unsigned int depth, ulong *rate)
{
	const struct qrk3588_clk_node *node;

	if (id >= QRK_NODE_COUNT)
		return -ENOENT;

	if (depth >= QRK_NODE_COUNT)
		return -ELOOP;

	node = &qrk3588_nodes[id];
	if (!node->ops || !node->ops->get_rate)
		return -ENOTSUPP;

	return node->ops->get_rate(priv, node, depth, rate);
}

static int qrk3588_node_get_rate(struct qrk3588_clk_priv *priv,
				 enum qrk3588_clk_node_id id, ulong *rate)
{
	return qrk3588_node_get_rate_depth(priv, id, 0, rate);
}

static int qrk3588_node_enable_depth(struct qrk3588_clk_priv *priv,
				     enum qrk3588_clk_node_id id,
				     unsigned int depth)
{
	const struct qrk3588_clk_node *node;
	enum qrk3588_clk_node_id parent;
	unsigned int i;
	int ret;

	if (id >= QRK_NODE_COUNT)
		return -ENOENT;

	if (depth >= QRK_NODE_COUNT)
		return -ELOOP;

	node = &qrk3588_nodes[id];

	/* Enable the selected upstream path before this consumer. */
	if (node->num_parents) {
		ret = qrk3588_node_current_parent(priv, node, &parent, NULL);
		if (ret)
			return ret;

		ret = qrk3588_node_enable_depth(priv, parent, depth + 1);
		if (ret)
			return ret;
	}

	/* Shared infrastructure is idempotently enabled and intentionally kept. */
	for (i = 0; i < node->num_shared_gates; i++) {
		ret = qrk3588_gate_ref_set(priv, &node->shared_gates[i], true);
		if (ret)
			return ret;
	}

	/* Owned dependencies are rolled back if this consumer fails to enable. */
	for (i = 0; i < node->num_owned_gates; i++) {
		ret = qrk3588_gate_ref_set(priv, &node->owned_gates[i], true);
		if (ret)
			goto rollback_owned;
	}

	ret = qrk3588_gate_set(priv, &node->gate, true);
	if (!ret)
		return 0;

rollback_owned:
	while (i)
		qrk3588_gate_ref_set(priv, &node->owned_gates[--i], false);

	return ret;
}

static int qrk3588_node_enable(struct qrk3588_clk_priv *priv,
			       enum qrk3588_clk_node_id id)
{
	return qrk3588_node_enable_depth(priv, id, 0);
}

static int qrk3588_node_disable(struct qrk3588_clk_priv *priv,
				enum qrk3588_clk_node_id id)
{
	const struct qrk3588_clk_node *node;
	enum qrk3588_clk_node_id parent;
	unsigned int i;
	int first_err = 0;
	int ret;

	if (id >= QRK_NODE_COUNT)
		return -ENOENT;

	node = &qrk3588_nodes[id];

	ret = qrk3588_gate_set(priv, &node->gate, false);
	if (ret && !first_err)
		first_err = ret;

	/* Disable private dependencies in reverse enable order. */
	for (i = node->num_owned_gates; i > 0; i--) {
		ret = qrk3588_gate_ref_set(priv, &node->owned_gates[i - 1], false);
		if (ret && !first_err)
			first_err = ret;
	}

	/*
	 * Shared gates and recursively enabled parent infrastructure deliberately
	 * remain on.  Their lifetime is broader than a single exported consumer.
	 */

	/*
	 * A mux-only exported REF clock has no leaf gate. Disable only the source
	 * branch selected by the final mux.
	 */
	if (node->disable_selected_parent) {
		ret = qrk3588_node_current_parent(priv, node, &parent, NULL);
		if (ret) {
			if (!first_err)
				first_err = ret;
		} else {
			ret = qrk3588_gate_set(priv, &qrk3588_nodes[parent].gate, false);
			if (ret && !first_err)
				first_err = ret;
		}
	}

	return first_err;
}

static int qrk3588_clk_request(struct clk *clk)
{
	return qrk3588_find_export(clk->id) ? 0 : -ENOENT;
}

static ulong qrk3588_clk_get_rate(struct clk *clk)
{
	const struct qrk3588_export_clk *export;
	struct qrk3588_clk_priv *priv = dev_get_priv(clk->dev);
	ulong rate;
	int ret;

	export = qrk3588_find_export(clk->id);
	if (!export)
		return (ulong)-ENOENT;

	if (!(export->caps & QRK_CLK_CAP_GET_RATE))
		return (ulong)-ENOTSUPP;

	ret = qrk3588_node_get_rate(priv, export->node, &rate);
	return ret ? (ulong)ret : rate;
}

static ulong qrk3588_clk_round_rate(struct clk *clk, ulong requested)
{
	const struct qrk3588_export_clk *export;
	const struct qrk3588_clk_node *node;
	struct qrk3588_clk_priv *priv = dev_get_priv(clk->dev);
	ulong rate;
	int ret;

	export = qrk3588_find_export(clk->id);
	if (!export)
		return (ulong)-ENOENT;

	if (!(export->caps & QRK_CLK_CAP_ROUND_RATE))
		return (ulong)-EPERM;

	node = &qrk3588_nodes[export->node];
	if (!node->ops || !node->ops->round_rate)
		return (ulong)-ENOTSUPP;

	ret = node->ops->round_rate(priv, node, requested, &rate);
	return ret ? (ulong)ret : rate;
}

static ulong qrk3588_clk_set_rate(struct clk *clk, ulong requested)
{
	const struct qrk3588_export_clk *export;
	const struct qrk3588_clk_node *node;
	struct qrk3588_clk_priv *priv = dev_get_priv(clk->dev);
	ulong rate;
	int ret;

	export = qrk3588_find_export(clk->id);
	if (!export)
		return (ulong)-ENOENT;

	if (!(export->caps & QRK_CLK_CAP_SET_RATE))
		return (ulong)-EPERM;

	node = &qrk3588_nodes[export->node];
	if (!node->ops || !node->ops->set_rate)
		return (ulong)-ENOTSUPP;

	ret = node->ops->set_rate(priv, node, requested, &rate);
	return ret ? (ulong)ret : rate;
}

static int qrk3588_clk_enable(struct clk *clk)
{
	const struct qrk3588_export_clk *export;
	struct qrk3588_clk_priv *priv = dev_get_priv(clk->dev);

	export = qrk3588_find_export(clk->id);
	if (!export)
		return -ENOENT;

	if (!(export->caps & QRK_CLK_CAP_GATE))
		return -EPERM;

	return qrk3588_node_enable(priv, export->node);
}

static int qrk3588_clk_disable(struct clk *clk)
{
	const struct qrk3588_export_clk *export;
	struct qrk3588_clk_priv *priv = dev_get_priv(clk->dev);

	export = qrk3588_find_export(clk->id);
	if (!export)
		return -ENOENT;

	if (!(export->caps & QRK_CLK_CAP_GATE))
		return -EPERM;

	return qrk3588_node_disable(priv, export->node);
}

#if IS_ENABLED(CONFIG_CMD_CLK)
static void qrk3588_clk_dump_ppll(struct qrk3588_clk_priv *priv)
{
	u32 con0, con1, con2, con6;
	u32 m, p, s;
	s16 k;
	ulong rate;
	int ret;

	con0 = readl((u8 __iomem *)priv->phptop_base + QRK3588_PHPTOP_PPLL_CON0);
	con1 = readl((u8 __iomem *)priv->phptop_base + QRK3588_PHPTOP_PPLL_CON1);
	con2 = readl((u8 __iomem *)priv->phptop_base + QRK3588_PHPTOP_PPLL_CON2);
	con6 = readl((u8 __iomem *)priv->phptop_base + QRK3588_PHPTOP_PPLL_CON6);

	m = con0 & PLL_M_MASK;
	p = con1 & PLL_P_MASK;
	s = (con1 & PLL_S_MASK) >> PLL_S_SHIFT;
	k = (s16)(con2 & 0xffff);

	ret = qrk3588_node_get_rate(priv, QRK_NODE_PPLL, &rate);
	if (ret) {
		printf("%-12s %-16s %s (M=%u P=%u S=%u K=%d, error %d)\n",
			   "-", "ppll",
			   con6 & PLL_LOCK ? "locked" : "unlocked",
			   m, p, s, k, ret);
		return;
	}

	if (con0 & PLL_BYPASS) {
		printf("%-12lu %-16s bypass (xin24m)\n", rate, "ppll");
		return;
	}

	printf("%-12lu %-16s %s (M=%u P=%u S=%u K=%d)\n",
		   rate, "ppll",
		   con6 & PLL_LOCK ? "locked" : "unlocked",
		   m, p, s, k);
}

static void qrk3588_clk_dump(struct udevice *dev)
{
	struct qrk3588_clk_priv *priv = dev_get_priv(dev);
	unsigned int i;

	qrk3588_clk_dump_ppll(priv);

	for (i = 0; i < ARRAY_SIZE(qrk3588_exports); i++) {
		const struct qrk3588_export_clk *export;
		const struct qrk3588_clk_node *node;
		ulong rate;
		int ret;

		export = &qrk3588_exports[i];
		node = &qrk3588_nodes[export->node];

		if (!(export->caps & QRK_CLK_CAP_GET_RATE))
			continue;

		ret = qrk3588_node_get_rate(priv, export->node, &rate);
		if (ret) {
			printf("%-12s %-16s error %d\n", "-", node->name, ret);
			continue;
		}

		printf("%-12lu %s\n", rate, node->name);
	}
}
#endif

static const struct clk_ops qrk3588_clk_ops = {
	.request = qrk3588_clk_request,
	.round_rate = qrk3588_clk_round_rate,
	.get_rate = qrk3588_clk_get_rate,
	.set_rate = qrk3588_clk_set_rate,
	.enable = qrk3588_clk_enable,
	.disable = qrk3588_clk_disable,
#if IS_ENABLED(CONFIG_CMD_CLK)
	.dump = qrk3588_clk_dump,
#endif
};

static int qrk3588_clk_probe(struct udevice *dev)
{
	struct qrk3588_clk_priv *priv = dev_get_priv(dev);

	priv->base = dev_read_addr_name_ptr(dev, "cru");
	if (!priv->base)
		return -EINVAL;

	priv->pmu1_base = dev_read_addr_name_ptr(dev, "pmu1-cru");
	if (!priv->pmu1_base)
		return -EINVAL;

	/* PPLL and PCIe Combo-PHY gates live in PHPTOPCRU. */
	priv->phptop_base = dev_read_addr_name_ptr(dev, "phptop-cru");
	if (!priv->phptop_base)
		return -EINVAL;

	return 0;
}

static const struct udevice_id qrk3588_clk_ids[] = {
	{ .compatible = "quest,qrk3588-cru" },
	{ }
};

U_BOOT_DRIVER(qrk3588_clk) = {
	.name = "qrk3588_clk",
	.id = UCLASS_CLK,
	.of_match = qrk3588_clk_ids,
	.bind = dm_scan_fdt_dev,
	.probe = qrk3588_clk_probe,
	.ops = &qrk3588_clk_ops,
	.priv_auto = sizeof(struct qrk3588_clk_priv),
};
