/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * es9039q2m.h  -- es9039q2m DAC driver
 *
 * Copyright 2024 Blue Rock Software Inc.
 *
 * Author: Yash Gandhi <yash@bluerocksoft.com>
 */

#ifndef _ES9039Q2M_H
#define _ES9039Q2M_H

#include <linux/regmap.h>

/*
 * Register values.
 */

 #define ES9039Q2M_SYS_CFG                  0x00
 #define ES9039Q2M_SYS_MODE                 0x01
 #define ES9039Q2M_DAC_CLK_CFG              0x03
 #define ES9039Q2M_CLK_CFG                  0x04
 #define ES9039Q2M_CLK_GEAR_SEL             0x05
 #define ES9039Q2M_INTERUPT_MASKP_1         0x0A
 #define ES9039Q2M_INTERUPT_MASKP_2         0x0B
 #define ES9039Q2M_INTERUPT_MASKN_1         0x0F
 #define ES9039Q2M_INTERUPT_MASKN_2         0x10
 #define ES9039Q2M_INTERRUPT_CLEAR_1        0x14
 #define ES9039Q2M_INTERRUPT_CLEAR_2        0x15
 #define ES9039Q2M_DPLL_BW                  0x1D
 #define ES9039Q2M_DATA_PATH_CFG            0x22
 #define ES9039Q2M_PCM_4X_GAIN              0x23
 #define ES9039Q2M_GPIO12_CFG               0x25
 #define ES9039Q2M_GPIO34_CFG               0x26
 #define ES9039Q2M_GPIO56_CFG               0x27
 #define ES9039Q2M_GPIO78_CFG               0x28
 #define ES9039Q2M_GPIO_OP_EN               0x29
 #define ES9039Q2M_GPIO_INPUT               0x2A
 #define ES9039Q2M_GPIO_WK_EN               0x2B
 #define ES9039Q2M_GPIO_INVERT              0x2C
 #define ES9039Q2M_GPIO_READ                0x2D
 #define ES9039Q2M_GPIO_OP_LOGIC_1          0x2E
 #define ES9039Q2M_GPIO_OP_LOGIC_2          0x2F
 #define ES9039Q2M_PWM1_COUNT               0x30
 #define ES9039Q2M_PWM1_FREQ_1              0x31
 #define ES9039Q2M_PWM1_FREQ_2              0x32
 #define ES9039Q2M_PWM2_COUNT               0x33
 #define ES9039Q2M_PWM2_FREQ_1              0x34
 #define ES9039Q2M_PWM2_FREQ_2              0x35
 #define ES9039Q2M_PWM3_COUNT               0x36
 #define ES9039Q2M_PWM3_FREQ_1              0x37
 #define ES9039Q2M_PWM3_FREQ_2              0x38
 #define ES9039Q2M_INPUT_SEL                0x39
 #define ES9039Q2M_MASTER_ENCODER_CFG       0x3A
 #define ES9039Q2M_TDM_CFG                  0x3B
 #define ES9039Q2M_TDM_CFG1                 0x3C
 #define ES9039Q2M_TDM_CFG2                 0x3D
 #define ES9039Q2M_BCKWS_MONITOR_CFG        0x3E
 #define ES9039Q2M_CH1_SLOT_CFG             0x40
 #define ES9039Q2M_CH2_SLOT_CFG             0x41
 #define ES9039Q2M_CH1_VOLUME               0x4A
 #define ES9039Q2M_CH2_VOLUME               0x4B
 #define ES9039Q2M_DAC_VOL_UP_RATE          0x52
 #define ES9039Q2M_DAC_VOL_DOWN_RATE        0x53
 #define ES9039Q2M_DAC_VOL_DOWN_RATE_FAST   0x54
 #define ES9039Q2M_DAC_MUTE                 0x56
 #define ES9039Q2M_DAC_INVERT               0X57
 #define ES9039Q2M_FILTER_SHAPE             0X58
 #define ES9039Q2M_IIR_BW_SPDIF_SEL         0x59
 #define ES9039Q2M_DAC_PATH_CFG             0x5A
 #define ES9039Q2M_THD_C2_CH1_1             0x5B
 #define ES9039Q2M_THD_C2_CH1_2             0x5C
 #define ES9039Q2M_THD_C2_CH2_1             0x5D
 #define ES9039Q2M_THD_C2_CH2_2             0x5E
 #define ES9039Q2M_THD_C3_CH1_1             0x6B
 #define ES9039Q2M_THD_C3_CH1_2             0x6C
 #define ES9039Q2M_THD_C3_CH2_1             0x6D
 #define ES9039Q2M_THD_C3_CH2_2             0x6E
 #define ES9039Q2M_AUTOMUTE_EN              0x7B
 #define ES9039Q2M_AUTOMUTE_TIME_1          0x7C
 #define ES9039Q2M_AUTOMUTE_TIME_2          0x7D
 #define ES9039Q2M_AUTOMUTE_LEVEL_1         0x7E
 #define ES9039Q2M_AUTOMUTE_LEVEL_2         0x7F
 #define ES9039Q2M_AUTOMUTE_OFF_LEVEL_1     0x80
 #define ES9039Q2M_AUTOMUTE_OFF_LEVEL_2     0x81
 #define ES9039Q2M_SOFT_RAMP_CFG            0x82
 #define ES9039Q2M_PROGRAM_RAM_CTL          0x87
 #define ES9039Q2M_SPDIF_READ_CTL           0x88
 #define ES9039Q2M_PROGRAM_RAM_ADDR         0x89
 #define ES9039Q2M_PROGRAM_RAM_DATA_1       0x8A
 #define ES9039Q2M_PROGRAM_RAM_DATA_2       0x8B
 #define ES9039Q2M_PROGRAM_RAM_DATA_3       0x8C
 #define ES9039Q2M_CHIP_ID_READ             0xE1
 #define ES9039Q2M_INTERRUPT_STATES_1       0xE5
 #define ES9039Q2M_INTERRUPT_STATES_2       0xE6
 #define ES9039Q2M_INTERRUPT_SOURCES_1      0xEA
 #define ES9039Q2M_INTERRUPT_SOURCES_2      0xEB
 #define ES9039Q2M_RATIO_VALID_READ         0xEF
 #define ES9039Q2M_GPIO_READ_ONLY           0xF0
 #define ES9039Q2M_VOL_MIN_READ             0xF1
 #define ES9039Q2M_AUTOMUTE_READ            0xF2
 #define ES9039Q2M_SOFT_RAMP_UP_READ        0xF3
 #define ES9039Q2M_SOFT_RAMP_DOWN_READ      0xF4
 #define ES9039Q2M_INPUT_STREAM_READBACK    0xF5
 #define ES9039Q2M_PROG_COEFF_OUT_READ_1    0xF6
 #define ES9039Q2M_PROG_COEFF_OUT_READ_2    0xF7
 #define ES9039Q2M_PROG_COEFF_OUT_READ_3    0xF8
 #define ES9039Q2M_SPDIF_DATA_READ          0xFB

#define ES9039Q2M_MAX_REGISTER 0xFB

 extern const struct regmap_config es9039q2m_regmap_config;
 extern const struct dev_pm_ops es9039q2m_pm;

 int es9039q2m_probe(struct device *dev, struct regmap *regmap);
 void es9039q2m_remove(struct device *dev);

 #endif /* _es9039q2m_H */

