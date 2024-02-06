/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * es9039q2m-i2c.c  -- es9039q2m DAC driver - I2C
 *
 * Copyright 2024 Blue Rock Software Inc.
 *
 * Author: Yash Gandhi <yash@bluerocksoft.com>
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/soc-dapm.h>

#include "es9039q2m.h"

#define ES9039Q2M_NUM_SUPPLIES 3
static const char *es9039q2m_supply_names[ES9039Q2M_NUM_SUPPLIES] = {
    "VCCA",
    "DVDD",
    "AVDD",
};

static const struct reg_default es9039q2m_reg_defaults[] = {
    {ES9039Q2M_DAC_CLK_CFG, 0x80},
    {ES9039Q2M_CLK_CFG, 0x07},
    {ES9039Q2M_GPIO12_CFG, 0x7D},
    {ES9039Q2M_GPIO34_CFG, 0x00},
    {ES9039Q2M_GPIO56_CFG, 0x00},
    {ES9039Q2M_GPIO78_CFG, 0x00},
    {ES9039Q2M_GPIO_OP_EN, 0x03},
    {ES9039Q2M_GPIO_INPUT, 0x00},
    {ES9039Q2M_GPIO_WK_EN, 0x00},
    {ES9039Q2M_GPIO_INVERT,0x00},
    {ES9039Q2M_GPIO_READ, 0x00},
    {ES9039Q2M_PWM1_COUNT, 0x00},
    {ES9039Q2M_PWM1_FREQ_1, 0x00},
    {ES9039Q2M_PWM1_FREQ_2, 0x00},
    {ES9039Q2M_PWM2_COUNT, 0x00},
    {ES9039Q2M_PWM2_FREQ_1, 0x00},
    {ES9039Q2M_PWM2_FREQ_2, 0x00},
    {ES9039Q2M_PWM3_COUNT, 0x00},
    {ES9039Q2M_PWM3_FREQ_1, 0x00},
    {ES9039Q2M_PWM3_FREQ_2, 0x00},
    {ES9039Q2M_CH1_SLOT_CFG, 0x00},
    {ES9039Q2M_CH2_SLOT_CFG, 0x61},
    {ES9039Q2M_CH1_VOLUME, 0x00},
    {ES9039Q2M_CH2_VOLUME, 0x00},
    {ES9039Q2M_DAC_VOL_UP_RATE, 0x04},
    {ES9039Q2M_DAC_VOL_DOWN_RATE, 0x04},
    {ES9039Q2M_DAC_VOL_DOWN_RATE_FAST, 0xFF},
    {ES9039Q2M_IIR_BW_SPDIF_SEL, 0x04},
    {ES9039Q2M_THD_C2_CH1_1, 0x00},
    {ES9039Q2M_THD_C2_CH1_2, 0x00},
    {ES9039Q2M_THD_C2_CH2_1, 0x00},
    {ES9039Q2M_THD_C2_CH2_2, 0x00},
    {ES9039Q2M_THD_C3_CH1_1, 0x00},
    {ES9039Q2M_THD_C3_CH1_2, 0x00},
    {ES9039Q2M_THD_C3_CH2_1, 0x00},
    {ES9039Q2M_THD_C3_CH2_2, 0x00},
    {ES9039Q2M_AUTOMUTE_LEVEL_1, 0x08},
    {ES9039Q2M_AUTOMUTE_LEVEL_2, 0x00},
    {ES9039Q2M_AUTOMUTE_OFF_LEVEL_1, 0x0A},
    {ES9039Q2M_AUTOMUTE_OFF_LEVEL_2, 0x00},
    {ES9039Q2M_PROGRAM_RAM_ADDR, 0x00},
    {ES9039Q2M_PROGRAM_RAM_DATA_1, 0x00},
    {ES9039Q2M_PROGRAM_RAM_DATA_2, 0x00},
    {ES9039Q2M_PROGRAM_RAM_DATA_3, 0x00},
    };

    struct es9039q2m_priv {
        struct device *dev;
        struct regmap *regmap;
        struct regulator_bulk_data supplies[ES9039Q2M_NUM_SUPPLIES];
        struct notifier_block disable_nb[ES9039Q2M_NUM_SUPPLIES];
        int mclk_div;
        struct gpio_desc *reset;
    }

    static bool es9039q2m_volatile(struct device *dev, unsigned int reg)
    {
        switch(reg) {
            case ES9039Q2M_CHIP_ID_READ:
            case ES9039Q2M_INTERRUPT_STATES_1:
            case ES9039Q2M_INTERRUPT_STATES_2:
            case ES9039Q2M_INTERRUPT_SOURCES_1:
            case ES9039Q2M_INTERRUPT_SOURCES_2:
            case ES9039Q2M_RATIO_VALID_READ:
            case ES9039Q2M_GPIO_READ:
            case ES9039Q2M_VOL_MIN_READ:
            case ES9039Q2M_AUTOMUTE_READ:
            case ES9039Q2M_SOFT_RAMP_UP_READ:
            case ES9039Q2M_SOFT_RAMP_DOWN_READ:
            case ES9039Q2M_INPUT_STREAM_READBACK:
            case ES9039Q2M_PROG_COEFF_OUT_READ_1:
            case ES9039Q2M_PROG_COEFF_OUT_READ_2:
            case ES9039Q2M_PROG_COEFF_OUT_READ_3:
            case ES9039Q2M_SPDIF_DATA_READ:
                return true;
            default: 
                return false;
        }
    }

    static const DECLARE_TLV_DB_SCALE(volume_tlv, -12750, 50, 1);

    static const struct snd_kcontrol_new es9039q2m_dac_controls[] = {
        SOC_DOUBLE_R_TLV("Master Playback Volume", ES9039Q2M_CH1_VOLUME, ES9039Q2M_CH2_VOLUME,
		     0, 255, 1, volume_tlv),
        SOC_DOUBLE_R_TLV("Digital Playback Volume", ES9039Q2M_CH1_VOLUME, ES9039Q2M_CH2_VOLUME,
		     0, 255, 1, volume_tlv),

    };

    static int es9039q2m_software_reset(struct es9039q2m_priv *es9039q2m)
    {
        return regmap_update_bits(es9039q2m->regmap, ES9039Q2M_SYS_CFG, 0x8, 0x1);
    }

    static int es9039q2m_set_format(struct snd_soc_dai *dai, unsigned int fmt)
    {
        struct snd_soc_component *component;
        u8 master, blck_inv, lrclk_inv;
        component = dai->conponent;

        switch(fmt & SND_SOC_DAIFMT_FORMAT_MASK){
            case SND_SOC_DAIFMT_I2S:
            break;
        default:
            dev_err(dai->dev,"unsupported dai format \n");
            return -EINVAL;
        }

        switch(fmt & SND_SOC_DAIFMT_MASTER_MASK) {
            case SND_SOC_DAIFMT_CBM_CFM:
                master = 1;
                break;
            case SND_SOC_DAIFMT_CBS_CFS:
                master = 0;
                break;
            default:
                dev_err(dai->dev,"unkown master/slave configuration");
                return -EINVAL;    
        }
        snd_soc_component_update_bits(component, ES9039Q2M_INPUT_SEL, 0x06, master << 4);

        blck_inv = lrclk_inv = 0;
        switch (fmt & SND_SOC_DAIFMT_INV_MASK){
            case SND_SOC_DAIFMT_NB_NF:
                break;
            case SND_SOC_DAIFMT_IB_IF:
                blck_inv = lrclk_inv = 1;
                break;
            case SND_SOC_DAIFMT_NB_IF:
                lrclk_inv = 1;
                break;
            case SND_SOC_DAIFMT_IB_NF:
                blck_inv = 1;
                break;
            default:
                dev_err("unkown polarity \n")
                return -EINVAL;
        }

        /* set BCLK and LRCLK/Frame Clk inversion  and invert slave bclk if needed */
        if(master == 1)
        {
            snd_soc_component_update_bits(component, ES9039Q2M_MASTER_ENCODER_CFG, 0x01 | 0x02, (blck_inv) | (lrclk_inv << 1) );
        }
        else 
        {
            snd_soc_component_update_bits(component, ES9039Q2M_MASTER_ENCODER_CFG, 0x40, blck_inv << 6);

        }

        return 0;
    }

    static int es9039q2m_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
    {
        struct snd_soc_component *component;
        u8 word_length;
        
        component = dai->component;

        switch(params_width(params)) {
        
        case 16:
            word_length = 0x02;
            break;
        case 24:
            word_length = 0x01;
            break;
        case 32:
            word_length = 0x00;
            break;
        default:
            dev_err(dai->dev,"Unsupported word length %u/n",params_width(params));
            return -EINVAL;
        } 

        snd_soc_component_update_bits(component, ES9039Q2M_MASTER_ENCODER_CFG, 0x18, word_length << 4)

        return 0;

    }


    static const struct snd_soc_dai_ops es9039q2m_ops = {
        .hw_prams = es9039q2m_hw_params,
        .set_fmt = es9039q2m_set_format,

    };

    #define ES9039Q2M_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE )

    static struct snd_soc_dai_driver ES9039Q2M_dai = {
        .name = "ES9039Q2M-dai"
        .playback = {
            .stream_name = "PLayback",
            .channels_min = 2,
            .channels_max = 2,
            .rate_min = 8000,
            .rate_max = 768000,
            .formats = ES9039Q2M_FORMATS,
        },
        .ops = es9039q2m_ops,
        };

    static const struct snd_soc_dapm_widget es9039q2m_dapm_widgets[] = {
        SND_SOC_DAPM_DAC ("DACL",NULL, SND_SOC_NOPM, 0 ,0),
        SND_SOC_DAPM_DAC ("DACR",NULL, SND_SOC_NOPM, 0 ,0),

        SND_SOC_DAPM_OUTPUT("OUTL"),
        SND_SOC_DAPM_OUTPUT("OUTR"),
    };

    static const struct snd_soc_dapm_route es9039q2m_dapm_routes[] = {
        {"DACL",NULL,"Playback"},
        {"DACR",NULL,"Playback"},
        {"OUTL",NULL,"DACL"},
        {"OUTR",NULL,"DACR"},
    };

    static struct snd_soc_component_driver es9039q2m_dac_codec_driver = {
        .controls = es9039q2m_dac_controls,
        .num_controls = ARRAY_SIZE(es9039q2m_dac_controls)
        .dapm_widgets = es9039q2m_dapm_widgets,
        .num_dapm_widgets = ARRAY_SIZE(es9039q2m_dapm_widgets),
        .dapm_routes = es9039q2m_dapm_routes,
        .num_dapm_widgets = ARRAY_SIZE(es9039q2m_dapm_widgets),


        };
    
    static const struct regmap_config es9039q2m_regmap_config = {
        .reg_bits = 8,
        .val_bits = 8,

        .max_register = ES9039Q2M_MAX_REGISTER,
        .volatile_reg = es9039q2m_volatile,
        .reg_defaults = es9039q2m_reg_defaults,
        .num_reg_defaults = ARRAY_SIZE(es9039q2m_reg_defaults),

        .cache_type = REGCACHE_RBTREE,

    }

    int es9039q2m_probe(struct device *dev, struct regmap *regmap)
    {
        struct es9039q2m_priv *es9039q2m;
        unsigned int id1,id2,id3,id4;
        int i, ret;

        es9039q2m = devm_kzalloc(dev,sizeof(*es9039q2m), GPP_KERNEL);
        if(!es9039q2m)
            return -ENOMEM;

        dev_set_drvdata(dev,es9039q2m);

        es9039q2m -> dev;
        es9039q2m->regmap = regmap;

        es9039q2m -> reset = devm_gpiod_get_optional(dev,"wlf,reset",GPIOD_OUT_LOW);
        if(IS_ERR(es9039q2m->reset)){
            ret = PIR_ERR(es9039q2m->reset);
            dev_err(dev,"failed to get reset line from : %d\n",ret);
            return ret;
        }

        for(i=0; i < ARRAY_SIZE(es9039q2m->supplies); i++)
            es9039q2m->supplies[i].supply = es9039q2m_supply_names[i];

        ret = devm_regulator_bulk_get(dev,ARRAY_SIZE(es9039q2m->supplies), es9039q2m->supplies);

        if(ret){
            dev_err(dev,"failed to get supplies: %d\n",ret);
            return ret;
        }
        
        
    }