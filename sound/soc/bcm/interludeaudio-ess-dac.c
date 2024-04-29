/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * es9039q2m.h  -- es9039q2m DAC driver
 *
 * Copyright 2024 Blue Rock Software Inc.
 *
 * Author: Yash Gandhi <yash@bluerocksoft.com>
 *
 *Interlude audio ESS DAC hat driver 
 *
 */

 #include <linux/gpio/consumer.h>
 #include <linux/platform_device.h>
 #include <linux/module.h>
 #include <linux/delay.h>
 #include <sound/core.h>
 #include <sound/pcm.h>
 #include <sound/pcm_params.h>
 #include <sound/soc.h>

 #include ".../codecs/es9039q2m.h"

 struct es9039q2m_clock_config {
    unsigned int sysclk_freq;
    unsigned int clk_idac;
    unsigned int clk_gear_sel;
    unsigned int master_bck_div;
 }

 struct snd_es9039q2m_drvdata {
    struct snd_soc_dai_link *dai;
    
    const char *card_name;
    const char *card_name_dt;
    const char *dai_name_dt;
    const char *dai_stream_name_dt;

    int(*probe)(struct platform_device *pdev);


 }

static struct gpio_desc *clock24;
static struct gpio_desc *clock22;
static int es9039q2m_samplerate;    
static struct gpio_desc *reset;
#define ACTIVE_CLOCK_RATE 0
#define CLOCK_22EN 22579200UL
#define CLOCK_24EN 24576000UL

static unsigned int snd_es9039q2m_clock_switch(unsigned int samplerate)
{

    switch(samplerate) {
    case 11025:
	case 22050:
	case 44100:
	case 88200:
	case 176400:
        if(ACTIVE_CLOCK_RATE == CLOCK_22EN)
            return ACTIVE_CLOCK_RATE;
        else{    
            gpiod_set_value_cansleep(clock22,1);
            gpiod_set_value_cansleep(clock24,0);
            ACTIVE_CLOCK_RATE = CLOCK_22EN;
        }
    default:
        if(ACTIVE_CLOCK_RATE == CLOCK_24EN)
            return ACTIVE_CLOCK_RATE;
        else{
            gpiod_set_value_cansleep(clock24,1);
            gpiod_set_value_cansleep(clock22,0);
            ACTIVE_CLOCK_RATE = CLOCK_24EN;
        }

    }
    return ACTIVE_CLOCK_RATE;
}
static struct snd_soc_ops interludeaudio_es9039q2m_ops = {
    .hw_params = snd_rpi_es9039q2m_hw_params,
};

SND_SOC_DAILINK_DEFS(interludeaudio_es9039q2m,
    DAILINK_COMP_ARRAY(COMP_EMPTY()),
    DAILINK_COMP_ARRAY(COMP_EMPTY()),
    DAILINK_COMP_ARRAY(COMP_EMPTY()),);

static struct snd_soc_dai_link interludeaudio_es9039q2m_dai = {
    .name = "Interludeaudio ESS DAC",
    .stream_name = "Interludeaudio ESS HI-FI DAC",
    SND_SOC_DAILINK_REG(interludeaudio_es9039q2m),
};

static struct snd_es9039q2m_drvdata interludeaudio_es9039q2m_drvdata = {
    .dai = &interludeaudio_es9039q2m_dai,
    .card_name = "Interludeaudio ESS DAC",
};

static const struct of_device_id es9039q2m_of_match[] = {
    { .compatible = "interludeaudio,es9039q2m", .data =(void *) &interludeaudio_es9039q2m_drvdata}, },
    { },
};

static struct snd_soc_card es9039q2m_card = {
    .drive_name = "Rpi-es9039q2m",
    .owner = THIS_MODULE,
    .dai_link = 1,
    .num_links = 1,
};

static void snd_es9039q2m_probe(struct snd_pcm_substream *substream)
{
    int ret = 0;
    const struct of_device_id *of_id

    es9039q2m_card.dev = &pdev->dev;
    of_id = of_match_node(es9039q2m_of_match, pdev->dev.of_node);

    clock22 = devm_gpiod_get(&pdev->dev, "clock22", GPIOD_OUT_LOW);
    clock24 = devm_gpiod_get(&pdev->dev, "clock24", GPIOD_OUT_LOW);
    gpiod_set_value_cansleep(clock24,1);
    ACTIVE_CLOCK_RATE = CLOCK_24EN;

    if(pdev->dev.of_node && of_id->data) {
        struct device_node *i2s_node;
        struct snd_es9039q2m_drvdata *drvdata = 
            (struct snd_es9039q2m_drvdata *) of_id->data;
        struct snd_soc_dai_link *dai = drvdata->dai;

        snd_soc_card_set_drvdata(&es9039q2m_card, drvdata);

        if(!dai->ops) {
            dai->ops = &snd_es9039q2m_ops;
        }
        if(!dai->codes->dai_name) {
            dai->codecs->dai_name = "es9039q2m-dac";
        }
        if(!dai->codecs->name) {
            dai->codecs->name = "es9039q2m.1-004b";
        }
        if(!dai->dai_fmt) {
            dai->dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM;
        }

        es9039q2m_card.dai_link = dai;
        i2s_node = of_parse_phandle(pdev->dev.of_node, "i2s-controller", 0);
        if(!i2s_node) {
            dev_err(&pdev->dev, "i2s-controller property not found\n");
            return -ENODEV;
        }

        es9039q2m_card.name = drvdata->card_name;

        if (drvdata->card_name_dt){
            of_property_read_string(i2s_node, drvdata->card_name_dt, &es9039q2m_card.name);
        }

        if(drvdata->dai_name_dt) {
            of_property_read_string(i2s_node, drvdata->dai_name_dt, &dai->name);
        }

        if(drvdata->dai_stream_name_dt) {
            of_property_read_string(i2s_node, drvdata->dai_stream_name_dt, &dai->stream_name);
        }
        
        dai->cpus->of_node = i2s_node;
        dai->platforms->of_node = i2s_node;

}