/* SPDX-License-Identifier: GPL-2.0-only */
/*
* es9080q.h  --  ES9080Q ALSA SoC Audio driver header
*
* Copyright 2025 BeagleBoard.org
*
* Author: Jian De <jiande2020@gmail.com>,
*         Giulio Moro
*/

#ifndef _ES9080Q_H
#define _ES9080Q_H

/* Register definitions */
#define ES9080_REG_AMP_CONTROL          0x00
#define ES9080_REG_CLK_EN               0x01
#define ES9080_REG_TDM_EN               0x02
#define ES9080_REG_DAC_CONFIG           0x03
#define ES9080_REG_MASTER_CLOCK_CONFIG  0x04
#define ES9080_REG_CP_CLOCK_DIV         0x06

#define ES9080_REG_VOLUME1              94
#define ES9080_REG_VOLUME2              95
#define ES9080_REG_VOLUME3              96
#define ES9080_REG_VOLUME4              97
#define ES9080_REG_VOLUME5              98
#define ES9080_REG_VOLUME6              99
#define ES9080_REG_VOLUME7              100
#define ES9080_REG_VOLUME8              101

#define ES9080_REG_GAIN_18DB            154
#define ES9080_PLL_REG                  202

#endif /* _ES9080Q_H */