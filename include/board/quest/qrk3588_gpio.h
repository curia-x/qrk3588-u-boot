/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright (c) 2026 ninozhang001@gmail.com */

#ifndef __BOARD_QUEST_QRK3588_GPIO_H__
#define __BOARD_QUEST_QRK3588_GPIO_H__

#include <board/quest/qrk3588_pins.h>

#define QRK3588_GPIO_PORT_COUNT			32

/* GPIO register offsets. */
#define GPIO_SWPORT_DR_L			0x0000
#define GPIO_SWPORT_DR_H			0x0004
#define GPIO_SWPORT_DDR_L			0x0008
#define GPIO_SWPORT_DDR_H			0x000c
#define GPIO_EXT_PORT				0x0070

#endif
