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
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
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

#define ES9039Q2M_PCM_RATES SNDRV_PCM_RATE_8000_192000
#define ES9039Q2M_NUM_SUPPLIES 3
#define clk22 22579200UL
#define clk24 24576000UL
//int sysclk_freq = 0;

static const char *es9039q2m_supply_names[ES9039Q2M_NUM_SUPPLIES] = {
    "VCCA",
    "DVDD",
    "AVDD",
};

static const struct reg_default es9039q2m_reg_defaults[] = {
    {ES9039Q2M_DAC_CLK_CFG, 0x80},
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
        int gpio_reset;
        int gpio_clock22;
        int gpio_clock24;
        int rate;
        int sysclk_freq;
    };

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
    };

    static const DECLARE_TLV_DB_SCALE(volume_tlv, -12750, 50, 1);

    static const struct snd_kcontrol_new es9039q2m_dac_controls[] = {
        SOC_DOUBLE_R_TLV("Channel 1 Playback Volume", ES9039Q2M_CH1_VOLUME, ES9039Q2M_CH2_VOLUME,
		     0, 255, 1, volume_tlv),
        SOC_DOUBLE_R_TLV("Channel 2 Playback Volume", ES9039Q2M_CH1_VOLUME, ES9039Q2M_CH2_VOLUME,
		     0, 255, 1, volume_tlv),

    };


    static int es9039q2m_software_reset(struct es9039q2m_priv *es9039q2m)
    {
        return regmap_update_bits(es9039q2m->regmap, ES9039Q2M_SYS_CFG, 0x8, 1);
    }
    
    static int es9039q2m_start_dac(struct es9039q2m_priv *es9039q2m)
    {
        printk(KERN_EMERG "starting DAC");
        return regmap_update_bits(es9039q2m->regmap, ES9039Q2M_SYS_CFG, 0xFF, 0x02);
    }

    static int es9039q2m_set_format(struct snd_soc_dai *dai, unsigned int fmt)
    {
        struct snd_soc_component *component;
        u8 master, blck_inv, lrclk_inv;
        component = dai->component;

        switch(fmt & SND_SOC_DAIFMT_FORMAT_MASK){
            case SND_SOC_DAIFMT_I2S:
            break;
        default:
            dev_err(dai->dev,"unsupported dai format \n");
            return -EINVAL;
        }

        switch(fmt & SND_SOC_DAIFMT_MASTER_MASK) {
            case SND_SOC_DAIFMT_CBM_CFM:
                printk("setting ess9039q2m to master mode");
                master = 1;
                break;
            case SND_SOC_DAIFMT_CBS_CFS:
                master = 0;
                break;
            default:
                dev_err(dai->dev,"unkown master/slave configuration");
                return -EINVAL;    
        }
        snd_soc_component_update_bits(component, ES9039Q2M_INPUT_SEL, 0x10, master << 4);

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
                dev_err(dai->dev,"unkown polarity \n");
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

    static unsigned int snd_es9039q2m_clock_switch(unsigned int samplerate, struct es9039q2m_priv *drvdata){
        switch (samplerate){
        case 11025:
        case 22050:
        case 44100:
        case 88200:
        case 176400:
            if(drvdata->sysclk_freq == clk22){
                return drvdata->sysclk_freq;
            }
            else{
                gpio_set_value(drvdata->gpio_clock22,1);
                gpio_set_value(drvdata->gpio_clock24,0);
                drvdata->sysclk_freq = clk22;
                return clk22;
            }
        default:
            if(drvdata->sysclk_freq == clk24){
                return drvdata->sysclk_freq;
            }
            else{
                gpio_set_value(drvdata->gpio_clock24,1);
                gpio_set_value(drvdata->gpio_clock22,0);
                return clk24;
            }
        }
    }

    static int es9039q2m_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
    {
        struct snd_soc_component *component = dai->component;
        struct es9039q2m_priv *drvdata = snd_soc_component_get_drvdata(component);
        u8 word_length;
        int sample_rate;
        int bclk_div;

        sample_rate = params_rate(params);

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

        snd_soc_component_update_bits(component, ES9039Q2M_MASTER_ENCODER_CFG, 0x18, word_length << 4);
        if(gpio_is_valid(drvdata->gpio_clock22)&& gpio_is_valid(drvdata->gpio_clock24))
        {
            drvdata->sysclk_freq = snd_es9039q2m_clock_switch(sample_rate, drvdata);
            dev_info(dai->dev,"dual clock mode active");
        }
        
        if(!drvdata->sysclk_freq){
            return -EINVAL;
            dev_err(dai->dev,"sysclk_freq not set\n");
        }
        bclk_div = ((drvdata->sysclk_freq / ((sample_rate * 2) * 32) )- 1);
        printk(KERN_EMERG "sample-rate: %d, bclk_div: %d, sysclk_frq: %d\n",sample_rate,bclk_div,drvdata->sysclk_freq);
        regmap_update_bits(drvdata->regmap, ES9039Q2M_CLK_CFG, 0xFF, bclk_div);

        return 0;
    }


    static const struct snd_soc_dai_ops es9039q2m_ops = {
        .hw_params = es9039q2m_hw_params,
        .set_fmt = es9039q2m_set_format,

    };

    #define ES9039Q2M_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE )

    static struct snd_soc_dai_driver ES9039Q2M_dai = {
        .name = "es9039q2m-dac",
        .playback = {
            .stream_name = "PLayback",
            .channels_min = 2,
            .channels_max = 2,
            .rates = ES9039Q2M_PCM_RATES,
            .formats = ES9039Q2M_FORMATS,
        },
        .ops = &es9039q2m_ops,
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
        .num_controls = ARRAY_SIZE(es9039q2m_dac_controls),
        .dapm_widgets = es9039q2m_dapm_widgets,
        .num_dapm_widgets = ARRAY_SIZE(es9039q2m_dapm_widgets),
        .dapm_routes = es9039q2m_dapm_routes,
        .num_dapm_widgets = ARRAY_SIZE(es9039q2m_dapm_widgets),


        };
    
    const struct regmap_config es9039q2m_regmap_config = {
        .reg_bits = 8,
        .val_bits = 8,
        .max_register = ES9039Q2M_MAX_REGISTER,
        .volatile_reg = es9039q2m_volatile,
        .reg_defaults = es9039q2m_reg_defaults,
        .num_reg_defaults = ARRAY_SIZE(es9039q2m_reg_defaults),
        .cache_type = REGCACHE_RBTREE,

    };
    EXPORT_SYMBOL_GPL(es9039q2m_regmap_config);

     void es9039q2m_remove(struct device *dev)
    {
        struct es9039q2m_priv *es9039q2m = dev_get_drvdata(dev);

        pm_runtime_disable(dev);
        pm_runtime_set_suspended(dev);

        regulator_bulk_disable(ARRAY_SIZE(es9039q2m->supplies), es9039q2m->supplies);

        if(es9039q2m->gpio_reset)
            gpio_set_value(es9039q2m->gpio_reset, 0);

        dev_set_drvdata(dev,NULL);
    }
    EXPORT_SYMBOL_GPL(es9039q2m_remove);

const struct of_device_id es9039q2m_dt_ids[] = {
    { .compatible = "ess,es9039q2m",},
    {}
};
MODULE_DEVICE_TABLE(of, es9039q2m_dt_ids);
EXPORT_SYMBOL_GPL(es9039q2m_dt_ids);

int es9039q2m_probe(struct device *dev, struct regmap *regmap)
    {
        struct es9039q2m_priv *es9039q2m;
        unsigned int id1;
        int i, ret;
        printk(KERN_EMERG "hello from es9039q2m probe");
        es9039q2m = devm_kzalloc(dev,sizeof(*es9039q2m), GFP_KERNEL);
        if(!es9039q2m)
            return -ENOMEM;

        dev_set_drvdata(dev,es9039q2m);
        es9039q2m->regmap = regmap;
        if(of_property_read_u32(dev->of_node,"sys-clk",&es9039q2m->sysclk_freq))
        {
            printk(KERN_EMERG "es9039q2m: no sys-clk property found in device tree\n");
            es9039q2m -> gpio_clock22 = of_get_named_gpio(dev->of_node,"clock22",0);
            if(gpio_is_valid(es9039q2m->gpio_clock22))
            {   gpio_direction_output(es9039q2m->gpio_clock22,0);
                ret = devm_gpio_request(dev, es9039q2m->gpio_clock22,"CLOCK 22");
                if(ret < 0)
                    dev_info(dev,"no gpio line for clock 22");
            }
            es9039q2m -> gpio_clock24 = of_get_named_gpio(dev->of_node,"clock24",0);
            if(gpio_is_valid(es9039q2m->gpio_clock24))
            {   gpio_direction_output(es9039q2m->gpio_clock24,0);
                ret = devm_gpio_request(dev, es9039q2m->gpio_clock24,"CLOCK 24");
                if(ret < 0)
                    dev_info(dev,"no gpio line for clock 24");
            } 
        }

        if(es9039q2m->sysclk_freq <= 0){
            if(gpio_is_valid(es9039q2m->gpio_clock24)){
                    gpio_set_value(es9039q2m->gpio_clock24,1);
                    printk(KERN_EMERG "es9039q2m: using 24Mhz clock\n");
                    es9039q2m->sysclk_freq = clk24;
            }
            else {
            dev_err(dev,"error getting clock gpios and systen clock frequency defaultinh to 27Mhz");
            es9039q2m->sysclk_freq = 27000000;
            }
        }

        printk(KERN_EMERG "es9039q2m: sysclk_freq: %d\n",es9039q2m->sysclk_freq);

        es9039q2m->gpio_reset = of_get_named_gpio(dev->of_node,"reset",0);
        if(gpio_is_valid(es9039q2m->gpio_reset)){
            ret = devm_gpio_request(dev, es9039q2m->gpio_reset,"ES9039Q2M reset");
            if(ret < 0)
                dev_err(dev,"error getting reset line");

        }

        for(i=0; i < ARRAY_SIZE(es9039q2m->supplies); i++)
            es9039q2m->supplies[i].supply = es9039q2m_supply_names[i];

        ret = devm_regulator_bulk_get(dev,ARRAY_SIZE(es9039q2m->supplies), es9039q2m->supplies);

        if(ret){
            dev_err(dev,"failed to get supplies: %d\n",ret);
            return ret;
        }
        
        gpio_set_value(es9039q2m->gpio_reset, 1);

        ret = regmap_read(regmap, ES9039Q2M_CHIP_ID_READ, &id1 );
        printk(KERN_EMERG "es9039q2m: device ID: %d\n",id1);
       if(id1 != 0x63) {
            dev_err(dev,"failed to read device ID: %d\n",ret);
            return ret;
        }

        if(!es9039q2m->gpio_reset){
            ret = es9039q2m_software_reset(es9039q2m);
            if(ret < 0 ){
                dev_err(dev,"Software reset failed: %d\n",ret);
                return ret;
            }
        }

        ret = es9039q2m_start_dac(es9039q2m);
        if(ret){
            dev_err(dev,"failed to start DAC: %d\n",ret);
            return ret;
        }

        ret = devm_snd_soc_register_component(dev, &es9039q2m_dac_codec_driver, &ES9039Q2M_dai, 1);
        if(ret){
            printk(KERN_EMERG "es9039q2m: failed to register component: %d\n",ret);
            dev_err(dev,"failed to register component: %d\n",ret);
            return ret;
        }

        pm_runtime_set_active(dev);
        pm_runtime_enable(dev);
        pm_runtime_idle(dev);

        return 0;


    }
    EXPORT_SYMBOL_GPL(es9039q2m_probe);

    #if IS_ENABLED(CONFIG_PM)
    static int es9039q2m_runtime_resune(struct device *dev)
    {
        struct es9039q2m_priv *es9039q2m = dev_get_drvdata(dev);
        int ret;

        ret = regulator_bulk_enable(ARRAY_SIZE(es9039q2m->supplies), es9039q2m->supplies);

        if(ret){
            dev_err(es9039q2m->dev,"failed to enable supplies: %d\n",ret);
            return ret;

        }

        regcache_sync(es9039q2m->regmap);

        return 0;
    }

    static int es9039q2m_runtime_suspend(struct device *dev)
    {
        struct es9039q2m_priv *es9039q2m = dev_get_drvdata(dev);

        regulator_bulk_disable(ARRAY_SIZE(es9039q2m->supplies),es9039q2m->supplies);

        return 0;

    }

    #endif
    
    const struct dev_pm_ops es9039q2m_pm = {
        SET_RUNTIME_PM_OPS(es9039q2m_runtime_suspend, es9039q2m_runtime_resune, NULL)
    };
    EXPORT_SYMBOL_GPL(es9039q2m_pm);

    MODULE_DESCRIPTION("ES9039Q2M DaC driver");
    MODULE_AUTHOR("Yash Gandhi yash@bluerocksoft.com");
    MODULE_LICENSE("GPL");
    