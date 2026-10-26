// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2026 ninozhang001@gmail.com
 *
 * Quest QRK3588 board support.
 */

#include <config.h>
#include <bootm.h>
#include <dm.h>
#include <dm/uclass.h>
#include <env.h>
#include <asm/armv8/mmu.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <board/quest/qrk3588_memory.h>
#include <cpu_func.h>
#include <fdtdec.h>
#include <init.h>
#include <net-common.h>
#include <serial.h>

DECLARE_GLOBAL_DATA_PTR;

#if IS_ENABLED(CONFIG_QRK3588_ITS_DIAGNOSTICS)
#define QRK3588_CRU_GATE_CON32		((void __iomem *)0xfd7c0880)
#define QRK3588_CRU_GATE_CON34		((void __iomem *)0xfd7c0888)
#define QRK3588_CRU_SOFTRST_CON34	((void __iomem *)0xfd7c0a88)
#define QRK3588_CRU_QCHANNEL_CON01	((void __iomem *)0xfd7c0ca4)
#define QRK3588_PMU_BIU_IDLE_ACK_STS0	((void __iomem *)0xfd8d8118)
#define QRK3588_PMU_BIU_IDLE_STS0	((void __iomem *)0xfd8d8120)
#define QRK3588_PMU_PWR_GATE_STS0	((void __iomem *)0xfd8d8180)
#define QRK3588_PMU_QCHANNEL_PWR_CON	((void __iomem *)0xfd8d81d0)
#define QRK3588_PMU_QCHANNEL_PWR_SFTCON	((void __iomem *)0xfd8d81d4)
#define QRK3588_PMU_QCHANNEL_PWR_STS	((void __iomem *)0xfd8d81d8)
#define QRK3588_GITS0_BASE		((void __iomem *)0xfe640000)
#define QRK3588_GITS1_BASE		((void __iomem *)0xfe660000)
#define GITS_CTLR			0x0000
#define GITS_FCTLR			0x0020
#define GITS_FCTLR_CEE			BIT(3)
#endif

static struct mm_region quest_mem_map[] = {
	{
		.virt = QRK3588_DRAM_BANK0_BASE,
		.phys = QRK3588_DRAM_BANK0_BASE,
		.size = QRK3588_DRAM_BANK0_SIZE,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			PTE_BLOCK_INNER_SHARE,
	}, {
		.virt = QRK3588_DRAM_BANK1_BASE,
		.phys = QRK3588_DRAM_BANK1_BASE,
		.size = QRK3588_DRAM_BANK1_SIZE,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			PTE_BLOCK_INNER_SHARE,
	}, {
		.virt = QRK3588_DEVICE_BASE,
		.phys = QRK3588_DEVICE_BASE,
		.size = QRK3588_DEVICE_SIZE,
		.attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			PTE_BLOCK_NON_SHARE,
	}, {
		0,
	},
};

/* Will be used in mmu_setup. */
struct mm_region *mem_map = quest_mem_map;

int dram_init(void)
{
	/*
	 * gd->ram_base/ram_size describe the lowest contiguous DRAM bank
	 * used during early U-Boot initialization and relocation setup.
	 * They do not represent the total size of all DRAM banks.
	 *
	 * The complete DRAM topology is populated later in
	 * dram_init_banksize() via gd->bd->bi_dram[].
	 */
	gd->ram_base = QRK3588_DRAM_BANK0_BASE;
	gd->ram_size = QRK3588_DRAM_BANK0_SIZE;

	return 0;
}

int dram_init_banksize(void)
{
	return fdtdec_setup_memory_banksize();
}

int board_init(void)
{
	return 0;
}

#if IS_ENABLED(CONFIG_QRK3588_ITS_DIAGNOSTICS)
void board_prep_linux(struct bootm_headers *images)
{
	(void)images;

	/* Record command errors without changing the ITS stall behavior. */
	setbits_le32(QRK3588_GITS0_BASE + GITS_FCTLR, GITS_FCTLR_CEE);
	setbits_le32(QRK3588_GITS1_BASE + GITS_FCTLR, GITS_FCTLR_CEE);

	printf("QRK3588: Linux ITS handoff: pwr=%#x idle_ack=%#x idle=%#x",
	       readl(QRK3588_PMU_PWR_GATE_STS0),
	       readl(QRK3588_PMU_BIU_IDLE_ACK_STS0),
	       readl(QRK3588_PMU_BIU_IDLE_STS0));
	printf(" gate32=%#x gate34=%#x reset34=%#x\n",
	       readl(QRK3588_CRU_GATE_CON32),
	       readl(QRK3588_CRU_GATE_CON34),
	       readl(QRK3588_CRU_SOFTRST_CON34));
	printf("QRK3588: ITS observation: qclk=%#x qpwr=%#x qsft=%#x qsts=%#x\n",
	       readl(QRK3588_CRU_QCHANNEL_CON01),
	       readl(QRK3588_PMU_QCHANNEL_PWR_CON),
	       readl(QRK3588_PMU_QCHANNEL_PWR_SFTCON),
	       readl(QRK3588_PMU_QCHANNEL_PWR_STS));
	printf("QRK3588: ITS registers: its0 ctlr=%#x fctlr=%#x",
	       readl(QRK3588_GITS0_BASE + GITS_CTLR),
	       readl(QRK3588_GITS0_BASE + GITS_FCTLR));
	printf(" its1 ctlr=%#x fctlr=%#x\n",
	       readl(QRK3588_GITS1_BASE + GITS_CTLR),
	       readl(QRK3588_GITS1_BASE + GITS_FCTLR));
}
#endif

static int qrk3588_name_eth_devices(void)
{
	struct udevice *dev;

	uclass_foreach_dev_probe(UCLASS_ETH, dev) {
		char name[32];
		int ret;

		snprintf(name, sizeof(name), "%s#%d", dev->driver->name,
			 dev_seq(dev));
		ret = device_set_name(dev, name);
		if (ret)
			return ret;
	}

	return 0;
}

int board_late_init(void)
{
	const char *state;
	int ret;

	ret = qrk3588_name_eth_devices();
	if (ret)
		return ret;

	/* Secure OS uses the fixed, non-returning main-loop event policy. */
	if (IS_ENABLED(CONFIG_QRK3588_SECURE_OS))
		return 0;

	/* Default environment is not evidence that an interrupted update is safe. */
	if (gd->env_valid != ENV_VALID && gd->env_valid != ENV_REDUND) {
		puts("QRK3588: persistent ENV unavailable; autoboot inhibited\n");
		env_set("bootdelay", "-1");
		env_set("bootcmd", NULL);
		return 0;
	}

	state = env_get("upgrade_state");
	if (state && strcmp(state, "idle") && strcmp(state, "complete")) {
		printf("QRK3588: upgrade_state=%s; autoboot inhibited\n", state);
		env_set("bootdelay", "-1");
		env_set("bootcmd", NULL);
	}

	return 0;
}

int print_cpuinfo(void)
{
	return 0;
}

struct serial_device *default_serial_console(void)
{
	return NULL;
}
