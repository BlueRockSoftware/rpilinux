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
        gpiod_set_value_cansleep(clock22,1);
        gpiod_set_value_cansleep(clock24,0);
        return CLOCK_22EN;
    default:
        gpiod_set_value_cansleep(clock24,1);
        gpiod_set_value_cansleep(clock22,0);
        return CLOCK_24EN;

    }
}

static void snd_es9039q2m_clock_config(unsigned int samplerate, struct es9039q2m_clock_config *clock_config)
{
    clock_config->sysclk_freq = sysclk_freq;

    switch(samplerate) {
        case 
    }

    
}


