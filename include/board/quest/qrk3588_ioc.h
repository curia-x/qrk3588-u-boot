/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright (c) 2026 ninozhang001@gmail.com */

#ifndef __BOARD_QUEST_QRK3588_IOC_H__
#define __BOARD_QUEST_QRK3588_IOC_H__

#include <board/quest/qrk3588_pins.h>

/* BUS_IOC IOMUX layout for GPIO1..GPIO4. */
#define BUS_IOC_BANK_STRIDE			0x20
#define BUS_IOC_GROUP_STRIDE			0x08
#define BUS_IOC_HALF_STRIDE			0x04

/* GPIO0 IOMUX is split between PMU1_IOC and PMU2_IOC. */
#define PMU1_IOC_GPIO0A_IOMUX_SEL_L		0x0000
#define PMU1_IOC_GPIO0A_IOMUX_SEL_H		0x0004
#define PMU1_IOC_GPIO0B_IOMUX_SEL_L		0x0008

#define PMU2_IOC_GPIO0B_IOMUX_SEL_H		0x0000
#define PMU2_IOC_GPIO0C_IOMUX_SEL_L		0x0004
#define PMU2_IOC_GPIO0C_IOMUX_SEL_H		0x0008
#define PMU2_IOC_GPIO0D_IOMUX_SEL_L		0x000c
#define PMU2_IOC_GPIO0D_IOMUX_SEL_H		0x0010

/* EMMC_IOC pad-control registers. */
#define EMMC_IOC_GPIO2A_DS_L			0x0040
#define EMMC_IOC_GPIO2D_DS_L			0x0058
#define EMMC_IOC_GPIO2D_DS_H			0x005c

#define EMMC_IOC_GPIO2A_P			0x0120
#define EMMC_IOC_GPIO2D_P			0x012c

#define EMMC_IOC_GPIO2A_IE			0x0190
#define EMMC_IOC_GPIO2D_IE			0x019c

#define EMMC_IOC_GPIO2A_SMT			0x0220
#define EMMC_IOC_GPIO2D_SMT			0x022c

#define QRK3588_IOMUX_GPIO			0x0
#define QRK3588_IOMUX_EMMC			0x1

/* EMMC_IOC documented drive-strength encoding. */
#define EMMC_DS_40OHM				0x6

#endif
