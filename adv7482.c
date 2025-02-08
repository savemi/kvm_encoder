/*
 * drivers/media/i2c/adv7482.c
 *     This file is Analog Devices ADV7482 HDMI receiver driver.
 *
 * Copyright (C) 2015-2016 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <linux/v4l2-dv-timings.h>
#include <linux/hdmi.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <dt-bindings/gpio/tegra-gpio.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/i2c-dev.h>
#include <linux/fs.h>
#include <uapi/linux/media.h>
#include <linux/clk-provider.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/pcm_params.h>
#include "adv7482.h"
#include <linux/delay.h>  /* for msleep() */
#define DRIVER_NAME "adv7482"
/****************************************/
/* ADV7482 I2C slave address definition */
/****************************************/
#define ADV7482_I2C_IO			0x70	/* IO Map */
#define ADV7482_I2C_DPLL		0x26	/* DPLL Map */
#define ADV7482_I2C_CP			0x22	/* CP Map */
#define ADV7482_I2C_HDMI		0x34	/* HDMI Map */
#define ADV7482_I2C_EDID		0x36	/* EDID Map */
#define ADV7482_I2C_REPEATER	0x32	/* HDMI RX Repeater Map */
#define ADV7482_I2C_INFOFRAME	0x31	/* HDMI RX InfoFrame Map */
#define ADV7482_I2C_CEC			0x41	/* CEC Map */
#define ADV7482_I2C_SDP			0x79	/* SDP Map */
#define ADV7482_I2C_TXB			0x48	/* CSI-TXB Map */
#define ADV7482_I2C_TXA			0x4A	/* CSI-TXA Map */
#define ADV7482_I2C_WAIT		0xFE	/* Wait x mesec */
#define ADV7482_I2C_EOR			0xFF	/* End Mark */
/****************************************/
/* ADV7482 IO register definition       */
/****************************************/
/* Revision */
#define ADV7482_IO_RD_INFO1_REG	0xDF	/* chip version register */
#define ADV7482_IO_RD_INFO2_REG	0xE0	/* chip version register */
#define ADV7482_IO_CP_DATAPATH_REG	0x03	/* datapath ctrl */
#define ADV7482_IO_CP_COLORSPACE_REG	0x04
#define ADV7482_IO_CP_VID_STD_REG	0x05	/* Video Standard */
/* Power Management */
#define ADV7482_IO_PWR_MAN_REG	0x0C	/* Power management register */
#define ADV7482_IO_PWR_ON		0xE0	/* Power on */
#define ADV7482_IO_PWR_OFF		0x00	/* Power down */
#define ADV7482_HDMI_DDC_PWRDN	0x73	/* Power DDC pads control register */
#define ADV7482_HDMI_DDC_PWR_ON		0x00	/* Power on */
#define ADV7482_HDMI_DDC_PWR_OFF	0x01	/* Power down */
#define ADV7482_IO_CP_VID_STD_480I	0x40
#define ADV7482_IO_CP_VID_STD_576I	0x41
#define ADV7482_IO_CP_VID_STD_480P	0x4A
#define ADV7482_IO_CP_VID_STD_576P	0x4B
#define ADV7482_IO_CP_VID_STD_720P	0x53
#define ADV7482_IO_CP_VID_STD_1080I	0x54
#define ADV7482_IO_CP_VID_STD_1080P	0x5E
#define ADV7482_IO_CP_VID_STD_SVGA56	0x80
#define ADV7482_IO_CP_VID_STD_SVGA60	0x81
#define ADV7482_IO_CP_VID_STD_SVGA72	0x82
#define ADV7482_IO_CP_VID_STD_SVGA75	0x83
#define ADV7482_IO_CP_VID_STD_SVGA85	0x84
#define ADV7482_IO_CP_VID_STD_SXGA60	0x85
#define ADV7482_IO_CP_VID_STD_SXGA75	0x86
#define ADV7482_IO_CP_VID_STD_VGA60	0x88
#define ADV7482_IO_CP_VID_STD_VGA72	0x89
#define ADV7482_IO_CP_VID_STD_VGA75	0x8A
#define ADV7482_IO_CP_VID_STD_VGA85	0x8B
#define ADV7482_IO_CP_VID_STD_XGA60	0x8C
#define ADV7482_IO_CP_VID_STD_XGA70	0x8D
#define ADV7482_IO_CP_VID_STD_XGA75	0x8E
#define ADV7482_IO_CP_VID_STD_XGA85	0x8F
#define ADV7482_IO_CP_VID_STD_UXGA60	0x96
#define ADV7482_IO_CSI4_EN_ENABLE       0x80
#define ADV7482_IO_CSI4_EN_DISABLE      0x00
#define ADV7482_IO_CSI2_EN_ENABLE       0x40
#define ADV7482_IO_CSI2_EN_DISABLE      0x00
/****************************************/
/* ADV7482 CP register definition       */
/****************************************/
/* Contrast Control */
#define ADV7482_CP_CON_REG	0x3a	/* Contrast (unsigned) */
#define ADV7482_CP_CON_MIN	0	/* Minimum contrast */
#define ADV7482_CP_CON_DEF	128	/* Default */
#define ADV7482_CP_CON_MAX	255	/* Maximum contrast */
/* Saturation Control */
#define ADV7482_CP_SAT_REG	0x3b	/* Saturation (unsigned) */
#define ADV7482_CP_SAT_MIN	0	/* Minimum saturation */
#define ADV7482_CP_SAT_DEF	128	/* Default */
#define ADV7482_CP_SAT_MAX	255	/* Maximum saturation */
/* Brightness Control */
#define ADV7482_CP_BRI_REG	0x3c	/* Brightness (signed) */
#define ADV7482_CP_BRI_MIN	-128	/* Luma is -512d */
#define ADV7482_CP_BRI_DEF	0	/* Luma is 0 */
#define ADV7482_CP_BRI_MAX	127	/* Luma is 508d */
/* Hue Control */
#define ADV7482_CP_HUE_REG	0x3d	/* Hue (unsigned) */
#define ADV7482_CP_HUE_MIN	0	/* -90 degree */
#define ADV7482_CP_HUE_DEF	0	/* -90 degree */
#define ADV7482_CP_HUE_MAX	255	/* +90 degree */
/* Video adjustment register */
#define ADV7482_CP_VID_ADJ_REG		0x3e
/* Video adjustment mask */
#define ADV7482_CP_VID_ADJ_MASK		0x7F
/* Enable color controls */
#define ADV7482_CP_VID_ADJ_ENABLE	0x80
/****************************************/
/* ADV7482 HDMI register definition     */
/****************************************/
/* HDMI status register */
#define ADV7482_HDMI_STATUS1_REG		0x07
/* VERT_FILTER_LOCKED flag */
#define ADV7482_HDMI_VF_LOCKED_FLG		0x80
/* DE_REGEN_FILTER_LOCKED flag */
#define ADV7482_HDMI_DERF_LOCKED_FLG		0x20
/* LINE_WIDTH[12:8] mask */
#define ADV7482_HDMI_LWIDTH_MSBS_MASK		0x1F
/* LINE_WIDTH[7:0] register */
#define ADV7482_HDMI_LWIDTH_REG			0x08
/* FIELD0_HEIGHT[12:8] register */
#define ADV7482_HDMI_F0HEIGHT_MSBS_REG		0x09
/* FIELD0_HEIGHT[12:8] mask */
#define ADV7482_HDMI_F0HEIGHT_MSBS_MASK		0x1F
/* FIELD0_HEIGHT[7:0] register */
#define ADV7482_HDMI_F0HEIGHT_LSBS_REG		0x0A
/* HDMI status register */
#define ADV7482_HDMI_STATUS2_REG		0x0B
/* DEEP_COLOR_MODE[1:0] mask */
#define ADV7482_HDMI_DCM_MASK			0xC0
/* HDMI_INTERLACED flag */
#define ADV7482_HDMI_IP_FLAG			0x20
/* FIELD1_HEIGHT[12:8] mask */
#define ADV7482_HDMI_F1HEIGHT_MSBS_MASK		0x1F
/* FIELD1_HEIGHT[7:0] register */
#define ADV7482_HDMI_F1HEIGHT_REG		0x0C

#define ADV7482_HDMI_I2S		0x03

struct adv7482_sdp_main_info {
	u8			status_reg_10;
};
/****************************************/
/* ADV7482 SDP register definition      */
/****************************************/
#define ADV7482_SDP_MAIN_MAP	0x00
#define ADV7482_SDP_SUB_MAP1	0x20
#define ADV7482_SDP_SUB_MAP2	0x40
#define ADV7482_SDP_NO_RO_MAIN_MAP	0x00
#define ADV7482_SDP_RO_MAIN_MAP		0x01
#define ADV7482_SDP_RO_SUB_MAP1		0x02
#define ADV7482_SDP_RO_SUB_MAP2		0x03
#define ADV7482_SDP_MAIN_MAP_RW \
			(ADV7482_SDP_MAIN_MAP | ADV7482_SDP_NO_RO_MAIN_MAP)
#define ADV7482_SDP_STD_AD_PAL_BG_NTSC_J_SECAM		0x0
#define ADV7482_SDP_STD_AD_PAL_BG_NTSC_J_SECAM_PED	0x1
#define ADV7482_SDP_STD_AD_PAL_N_NTSC_J_SECAM		0x2
#define ADV7482_SDP_STD_AD_PAL_N_NTSC_M_SECAM		0x3
#define ADV7482_SDP_STD_NTSC_J				0x4
#define ADV7482_SDP_STD_NTSC_M				0x5
#define ADV7482_SDP_STD_PAL60				0x6
#define ADV7482_SDP_STD_NTSC_443			0x7
#define ADV7482_SDP_STD_PAL_BG				0x8
#define ADV7482_SDP_STD_PAL_N				0x9
#define ADV7482_SDP_STD_PAL_M				0xa
#define ADV7482_SDP_STD_PAL_M_PED			0xb
#define ADV7482_SDP_STD_PAL_COMB_N			0xc
#define ADV7482_SDP_STD_PAL_COMB_N_PED			0xd
#define ADV7482_SDP_STD_PAL_SECAM			0xe
#define ADV7482_SDP_STD_PAL_SECAM_PED			0xf
#define ADV7482_SDP_REG_INPUT_CONTROL		0x00
#define ADV7482_SDP_INPUT_CONTROL_INSEL_MASK	0x0f
#define ADV7482_SDP_REG_INPUT_VIDSEL		0x02
#define ADV7482_SDP_REG_CTRL			0x0e
#define ADV7482_SDP_REG_PWR_MAN		0x0f
#define ADV7482_SDP_PWR_MAN_ON		0x00
#define ADV7482_SDP_PWR_MAN_OFF		0x20
#define ADV7482_SDP_PWR_MAN_RES		0x80
/* Contrast */
#define ADV7482_SDP_REG_CON		0x08	/*Unsigned */
#define ADV7482_SDP_CON_MIN		0
#define ADV7482_SDP_CON_DEF		128
#define ADV7482_SDP_CON_MAX		255
/* Brightness*/
#define ADV7482_SDP_REG_BRI		0x0a	/*Signed */
#define ADV7482_SDP_BRI_MIN		-128
#define ADV7482_SDP_BRI_DEF		0
#define ADV7482_SDP_BRI_MAX		127
/* Hue */
#define ADV7482_SDP_REG_HUE		0x0b	/*Signed, inverted */
#define ADV7482_SDP_HUE_MIN		-127
#define ADV7482_SDP_HUE_DEF		0
#define ADV7482_SDP_HUE_MAX		128
/* Saturation */
#define ADV7482_SDP_REG_SD_SAT_CB	0xe3
#define ADV7482_SDP_REG_SD_SAT_CR	0xe4
#define ADV7482_SDP_SAT_MIN		0
#define ADV7482_SDP_SAT_DEF		128
#define ADV7482_SDP_SAT_MAX		255
#define ADV7482_SDP_INPUT_CVBS_AIN1		0x00
#define ADV7482_SDP_INPUT_CVBS_AIN2		0x01
#define ADV7482_SDP_INPUT_CVBS_AIN3		0x02
#define ADV7482_SDP_INPUT_CVBS_AIN4		0x03
#define ADV7482_SDP_INPUT_CVBS_AIN5		0x04
#define ADV7482_SDP_INPUT_CVBS_AIN6		0x05
#define ADV7482_SDP_INPUT_CVBS_AIN7		0x06
#define ADV7482_SDP_INPUT_CVBS_AIN8		0x07
#define ADV7482_SDP_INPUT_SVIDEO_AIN1_AIN2	0x08
#define ADV7482_SDP_INPUT_SVIDEO_AIN3_AIN4	0x09
#define ADV7482_SDP_INPUT_SVIDEO_AIN5_AIN6	0x0a
#define ADV7482_SDP_INPUT_SVIDEO_AIN7_AIN8	0x0b
#define ADV7482_SDP_INPUT_YPRPB_AIN1_AIN2_AIN3	0x0c
#define ADV7482_SDP_INPUT_YPRPB_AIN4_AIN5_AIN6	0x0d
#define ADV7482_SDP_INPUT_DIFF_CVBS_AIN1_AIN2	0x0e
#define ADV7482_SDP_INPUT_DIFF_CVBS_AIN3_AIN4	0x0f
#define ADV7482_SDP_INPUT_DIFF_CVBS_AIN5_AIN6	0x10
#define ADV7482_SDP_INPUT_DIFF_CVBS_AIN7_AIN8	0x11
#define ADV7482_SDP_R_REG_10			0x10
#define ADV7482_SDP_R_REG_10_IN_LOCK		0x01
#define ADV7482_SDP_R_REG_10_FSC_LOCK		0x04
#define ADV7482_SDP_R_REG_10_AUTOD_MASK		0x70
#define ADV7482_SDP_R_REG_10_AUTOD_NTSM_M_J	0x00
#define ADV7482_SDP_R_REG_10_AUTOD_NTSC_4_43	0x10
#define ADV7482_SDP_R_REG_10_AUTOD_PAL_M	0x20
#define ADV7482_SDP_R_REG_10_AUTOD_PAL_60	0x30
#define ADV7482_SDP_R_REG_10_AUTOD_PAL_B_G	0x40
#define ADV7482_SDP_R_REG_10_AUTOD_SECAM	0x50
#define ADV7482_SDP_R_REG_10_AUTOD_PAL_COMB	0x60
#define ADV7482_SDP_R_REG_10_AUTOD_SECAM_525	0x70
#define ADV7482_MAX_WIDTH		1920
#define ADV7482_MAX_HEIGHT		1080
/****************************************/
/* ADV7482 structure definition         */
/****************************************/
/* Structure for register values */
struct adv7482_reg_value {
	u8 addr;				/* i2c slave address */
	u8 reg;					/* sub (register) address */
	u8 value;				/* register value */
};
#define END_REGISTER_TABLE \
{ADV7482_I2C_EOR, 0xFF, 0xFF} /* End of register table */


static int debug=3;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-3)");

struct adv748x_hdmi_video_standards {
	struct v4l2_dv_timings timings;
	u8 vid_std;
	u8 v_freq;
};

static const struct adv748x_hdmi_video_standards
adv748x_hdmi_video_standards[] = {
	{ V4L2_DV_BT_CEA_720X480P59_94, 0x4a, 0x00 },
	{ V4L2_DV_BT_CEA_720X576P50, 0x4b, 0x00 },
	{ V4L2_DV_BT_CEA_1280X720P60, 0x53, 0x00 },
	{ V4L2_DV_BT_CEA_1280X720P50, 0x53, 0x01 },
	{ V4L2_DV_BT_CEA_1280X720P30, 0x53, 0x02 },
	{ V4L2_DV_BT_CEA_1280X720P25, 0x53, 0x03 },
	{ V4L2_DV_BT_CEA_1280X720P24, 0x53, 0x04 },
	{ V4L2_DV_BT_CEA_1920X1080P60, 0x5e, 0x00 },
	{ V4L2_DV_BT_CEA_1920X1080P50, 0x5e, 0x01 },
	{ V4L2_DV_BT_CEA_1920X1080P30, 0x5e, 0x02 },
	{ V4L2_DV_BT_CEA_1920X1080P25, 0x5e, 0x03 },
	{ V4L2_DV_BT_CEA_1920X1080P24, 0x5e, 0x04 },
	/* SVGA */
	{ V4L2_DV_BT_DMT_800X600P56, 0x80, 0x00 },
	{ V4L2_DV_BT_DMT_800X600P60, 0x81, 0x00 },
	{ V4L2_DV_BT_DMT_800X600P72, 0x82, 0x00 },
	{ V4L2_DV_BT_DMT_800X600P75, 0x83, 0x00 },
	{ V4L2_DV_BT_DMT_800X600P85, 0x84, 0x00 },
	/* SXGA */
	{ V4L2_DV_BT_DMT_1280X1024P60, 0x85, 0x00 },
	{ V4L2_DV_BT_DMT_1280X1024P75, 0x86, 0x00 },
	/* VGA */
	{ V4L2_DV_BT_DMT_640X480P60, 0x88, 0x00 },
	{ V4L2_DV_BT_DMT_640X480P72, 0x89, 0x00 },
	{ V4L2_DV_BT_DMT_640X480P75, 0x8a, 0x00 },
	{ V4L2_DV_BT_DMT_640X480P85, 0x8b, 0x00 },
	/* XGA */
	{ V4L2_DV_BT_DMT_1024X768P60, 0x8c, 0x00 },
	{ V4L2_DV_BT_DMT_1024X768P70, 0x8d, 0x00 },
	{ V4L2_DV_BT_DMT_1024X768P75, 0x8e, 0x00 },
	{ V4L2_DV_BT_DMT_1024X768P85, 0x8f, 0x00 },
	/* UXGA */
	{ V4L2_DV_BT_DMT_1600X1200P60, 0x96, 0x00 },
};

/* Register default values */
static const struct adv7482_reg_value adv7482_sw_reset[] = {
	{ADV7482_I2C_IO, 0xFF, 0xFF},	/* SW reset */
	{ADV7482_I2C_WAIT, 0x00, 0x05},	/* delay 5 */
	{ADV7482_I2C_IO, 0x01, 0x76},	/* ADI Required Write */
	{ADV7482_I2C_IO, 0xF2, 0x01},	/* Enable I2C Read Auto-Increment */
	{ADV7482_I2C_EOR, 0xFF, 0xFF}	/* End of register table */
};
static const struct adv7482_reg_value adv7482_set_slave_address[] = {
	/* I2C Slave Address settings */
	{ADV7482_I2C_IO, 0xF3, ADV7482_I2C_DPLL * 2},	/* DPLL Map */
	{ADV7482_I2C_IO, 0xF4, ADV7482_I2C_CP * 2},	/* CP Map */
	{ADV7482_I2C_IO, 0xF5, ADV7482_I2C_HDMI * 2},	/* HDMI Map */
	{ADV7482_I2C_IO, 0xF6, ADV7482_I2C_EDID * 2},	/* EDID Map */
	{ADV7482_I2C_IO, 0xF7, ADV7482_I2C_REPEATER * 2},
						/* HDMI RX Repeater Map */
	{ADV7482_I2C_IO, 0xF8, ADV7482_I2C_INFOFRAME * 2},
						/* HDMI RX InfoFrame Map */
	{ADV7482_I2C_IO, 0xFA, ADV7482_I2C_CEC * 2},	/* CEC Map */
	{ADV7482_I2C_IO, 0xFB, ADV7482_I2C_SDP * 2},	/* SDP Map */
	{ADV7482_I2C_IO, 0xFC, ADV7482_I2C_TXB * 2},	/* CSI-TXB Map */
	{ADV7482_I2C_IO, 0xFD, ADV7482_I2C_TXA * 2},	/* CSI-TXA Map */
	{ADV7482_I2C_EOR, 0xFF, 0xFF}	/* End of register table */
};
/* Supported Formats For Script Below */
/* - 01-29 HDMI to MIPI TxA CSI 4-Lane - RGB888: */
static const struct adv7482_reg_value adv7482_init_txa_4lane[] = {
	/* I2C Slave Address settings */
	{ADV7482_I2C_IO, 0xF3, ADV7482_I2C_DPLL * 2},	/* DPLL Map */
	{ADV7482_I2C_IO, 0xF4, ADV7482_I2C_CP * 2},	/* CP Map */
	{ADV7482_I2C_IO, 0xF5, ADV7482_I2C_HDMI * 2},	/* HDMI Map */
	{ADV7482_I2C_IO, 0xF6, ADV7482_I2C_EDID * 2},	/* EDID Map */
	{ADV7482_I2C_IO, 0xF7, ADV7482_I2C_REPEATER * 2},
						/* HDMI RX Repeater Map */
	{ADV7482_I2C_IO, 0xF8, ADV7482_I2C_INFOFRAME * 2},
						/* HDMI RX InfoFrame Map */
	{ADV7482_I2C_IO, 0xFA, ADV7482_I2C_CEC * 2},	/* CEC Map */
	{ADV7482_I2C_IO, 0xFB, ADV7482_I2C_SDP * 2},	/* SDP Map */
	{ADV7482_I2C_IO, 0xFC, ADV7482_I2C_TXB * 2},	/* CSI-TXB Map */
	{ADV7482_I2C_IO, 0xFD, ADV7482_I2C_TXA * 2},	/* CSI-TXA Map */
	/* Disable chip powerdown & Enable HDMI Rx block */
	{ADV7482_I2C_IO, 0x00, 0x40},
	{ADV7482_I2C_REPEATER, 0x40, 0x83},	/* Enable HDCP 1.1 */
	{ADV7482_I2C_HDMI, 0x00, 0x08},	/* Foreground Channel = A */
	{ADV7482_I2C_HDMI, 0x98, 0xFF},	/* ADI Required Write */
	{ADV7482_I2C_HDMI, 0x99, 0xA3},	/* ADI Required Write */
	{ADV7482_I2C_HDMI, 0x9A, 0x00},	/* ADI Required Write */
	{ADV7482_I2C_HDMI, 0x9B, 0x0A},	/* ADI Required Write */
	{ADV7482_I2C_HDMI, 0x9D, 0x40},	/* ADI Required Write */
	{ADV7482_I2C_HDMI, 0xCB, 0x09},	/* ADI Required Write */
	{ADV7482_I2C_HDMI, 0x3D, 0x10},	/* ADI Required Write */
	{ADV7482_I2C_HDMI, 0x3E, 0x7B},	/* ADI Required Write */
	{ADV7482_I2C_HDMI, 0x3F, 0x5E},	/* ADI Required Write */
	{ADV7482_I2C_HDMI, 0x4E, 0xFE},	/* ADI Required Write */
	{ADV7482_I2C_HDMI, 0x4F, 0x18},	/* ADI Required Write */
	{ADV7482_I2C_HDMI, 0x57, 0xA3},	/* ADI Required Write */
	{ADV7482_I2C_HDMI, 0x58, 0x04},	/* ADI Required Write */
	{ADV7482_I2C_HDMI, 0x85, 0x10},	/* ADI Required Write */
	{ADV7482_I2C_HDMI, 0x83, 0x00},	/* Enable All Terminations */
	{ADV7482_I2C_HDMI, 0xA3, 0x01},	/* ADI Required Write */
	{ADV7482_I2C_HDMI, 0xBE, 0x00},	/* ADI Required Write */
	{ADV7482_I2C_HDMI, 0x6C, 0x00},	/* HPA Manual Enable */
	{ADV7482_I2C_HDMI, 0xF8, 0x00},	/* HPA Asserted */
	{ADV7482_I2C_HDMI, 0x0F, 0x00},	/* Audio Mute Speed Set to Fastest */
					/* (Smallest Step Size) */
	/* EDID */
	/* Header information(0-19th byte) */
{ADV7482_I2C_EDID,0x00,0x00},
{ADV7482_I2C_EDID,0x01,0xFF},
{ADV7482_I2C_EDID,0x02,0xFF},
{ADV7482_I2C_EDID,0x03,0xFF},
{ADV7482_I2C_EDID,0x04,0xFF},
{ADV7482_I2C_EDID,0x05,0xFF},
{ADV7482_I2C_EDID,0x06,0xFF},
{ADV7482_I2C_EDID,0x07,0x00},
{ADV7482_I2C_EDID,0x08,0x4C},
{ADV7482_I2C_EDID,0x09,0x2D},
{ADV7482_I2C_EDID,0x0A,0x9E},
{ADV7482_I2C_EDID,0x0B,0x07},
{ADV7482_I2C_EDID,0x0C,0x00},
{ADV7482_I2C_EDID,0x0D,0x00},
{ADV7482_I2C_EDID,0x0E,0x00},
{ADV7482_I2C_EDID,0x0F,0x00},
{ADV7482_I2C_EDID,0x10,0x25},
{ADV7482_I2C_EDID,0x11,0x15},
{ADV7482_I2C_EDID,0x12,0x01},
{ADV7482_I2C_EDID,0x13,0x03},
{ADV7482_I2C_EDID,0x14,0x80},
{ADV7482_I2C_EDID,0x15,0x3C},
{ADV7482_I2C_EDID,0x16,0x22},
{ADV7482_I2C_EDID,0x17,0x78},
{ADV7482_I2C_EDID,0x18,0x2A},
{ADV7482_I2C_EDID,0x19,0xEE},
{ADV7482_I2C_EDID,0x1A,0x91},
{ADV7482_I2C_EDID,0x1B,0xA3},
{ADV7482_I2C_EDID,0x1C,0x54},
{ADV7482_I2C_EDID,0x1D,0x4C},
{ADV7482_I2C_EDID,0x1E,0x99},
{ADV7482_I2C_EDID,0x1F,0x26},
{ADV7482_I2C_EDID,0x20,0x0F},
{ADV7482_I2C_EDID,0x21,0x50},
{ADV7482_I2C_EDID,0x22,0x54},
{ADV7482_I2C_EDID,0x23,0x23},
{ADV7482_I2C_EDID,0x24,0x08},
{ADV7482_I2C_EDID,0x25,0x00},
{ADV7482_I2C_EDID,0x26,0x81},
{ADV7482_I2C_EDID,0x27,0x80},
{ADV7482_I2C_EDID,0x28,0x81},
{ADV7482_I2C_EDID,0x29,0x40},
{ADV7482_I2C_EDID,0x2A,0x81},
{ADV7482_I2C_EDID,0x2B,0x00},
{ADV7482_I2C_EDID,0x2C,0x95},
{ADV7482_I2C_EDID,0x2D,0x00},
{ADV7482_I2C_EDID,0x2E,0xB3},
{ADV7482_I2C_EDID,0x2F,0x00},
{ADV7482_I2C_EDID,0x30,0x01},
{ADV7482_I2C_EDID,0x31,0x01},
{ADV7482_I2C_EDID,0x32,0x01},
{ADV7482_I2C_EDID,0x33,0x01},
{ADV7482_I2C_EDID,0x34,0x01},
{ADV7482_I2C_EDID,0x35,0x01},
{ADV7482_I2C_EDID,0x36,0x02},
{ADV7482_I2C_EDID,0x37,0x3A},
{ADV7482_I2C_EDID,0x38,0x80},
{ADV7482_I2C_EDID,0x39,0x18},
{ADV7482_I2C_EDID,0x3A,0x71},
{ADV7482_I2C_EDID,0x3B,0x38},
{ADV7482_I2C_EDID,0x3C,0x2D},
{ADV7482_I2C_EDID,0x3D,0x40},
{ADV7482_I2C_EDID,0x3E,0x58},
{ADV7482_I2C_EDID,0x3F,0x2C},
{ADV7482_I2C_EDID,0x40,0x45},
{ADV7482_I2C_EDID,0x41,0x00},
{ADV7482_I2C_EDID,0x42,0x56},
{ADV7482_I2C_EDID,0x43,0x50},
{ADV7482_I2C_EDID,0x44,0x21},
{ADV7482_I2C_EDID,0x45,0x00},
{ADV7482_I2C_EDID,0x46,0x00},
{ADV7482_I2C_EDID,0x47,0x1E},
{ADV7482_I2C_EDID,0x48,0x02},
{ADV7482_I2C_EDID,0x49,0x3A},
{ADV7482_I2C_EDID,0x4A,0x80},
{ADV7482_I2C_EDID,0x4B,0xD0},
{ADV7482_I2C_EDID,0x4C,0x72},
{ADV7482_I2C_EDID,0x4D,0x38},
{ADV7482_I2C_EDID,0x4E,0x2D},
{ADV7482_I2C_EDID,0x4F,0x40},
{ADV7482_I2C_EDID,0x50,0x10},
{ADV7482_I2C_EDID,0x51,0x2C},
{ADV7482_I2C_EDID,0x52,0x45},
{ADV7482_I2C_EDID,0x53,0x80},
{ADV7482_I2C_EDID,0x54,0x56},
{ADV7482_I2C_EDID,0x55,0x50},
{ADV7482_I2C_EDID,0x56,0x21},
{ADV7482_I2C_EDID,0x57,0x00},
{ADV7482_I2C_EDID,0x58,0x00},
{ADV7482_I2C_EDID,0x59,0x1E},
{ADV7482_I2C_EDID,0x5A,0x00},
{ADV7482_I2C_EDID,0x5B,0x00},
{ADV7482_I2C_EDID,0x5C,0x00},
{ADV7482_I2C_EDID,0x5D,0xFD},
{ADV7482_I2C_EDID,0x5E,0x00},
{ADV7482_I2C_EDID,0x5F,0x18},
{ADV7482_I2C_EDID,0x60,0x4B},
{ADV7482_I2C_EDID,0x61,0x1A},
{ADV7482_I2C_EDID,0x62,0x51},
{ADV7482_I2C_EDID,0x63,0x17},
{ADV7482_I2C_EDID,0x64,0x00},
{ADV7482_I2C_EDID,0x65,0x0A},
{ADV7482_I2C_EDID,0x66,0x20},
{ADV7482_I2C_EDID,0x67,0x20},
{ADV7482_I2C_EDID,0x68,0x20},
{ADV7482_I2C_EDID,0x69,0x20},
{ADV7482_I2C_EDID,0x6A,0x20},
{ADV7482_I2C_EDID,0x6B,0x20},
{ADV7482_I2C_EDID,0x6C,0x00},
{ADV7482_I2C_EDID,0x6D,0x00},
{ADV7482_I2C_EDID,0x6E,0x00},
{ADV7482_I2C_EDID,0x6F,0xFC},
{ADV7482_I2C_EDID,0x70,0x00},
{ADV7482_I2C_EDID,0x71,0x53},
{ADV7482_I2C_EDID,0x72,0x32},
{ADV7482_I2C_EDID,0x73,0x37},
{ADV7482_I2C_EDID,0x74,0x41},
{ADV7482_I2C_EDID,0x75,0x39},
{ADV7482_I2C_EDID,0x76,0x35},
{ADV7482_I2C_EDID,0x77,0x30},
{ADV7482_I2C_EDID,0x78,0x44},
{ADV7482_I2C_EDID,0x79,0x0A},
{ADV7482_I2C_EDID,0x7A,0x20},
{ADV7482_I2C_EDID,0x7B,0x20},
{ADV7482_I2C_EDID,0x7C,0x20},
{ADV7482_I2C_EDID,0x7D,0x20},
{ADV7482_I2C_EDID,0x7E,0x01},
{ADV7482_I2C_EDID,0x7F,0x07},
{ADV7482_I2C_EDID,0x80,0x02},
{ADV7482_I2C_EDID,0x81,0x03},
{ADV7482_I2C_EDID,0x82,0x28},
{ADV7482_I2C_EDID,0x83,0xF1},
{ADV7482_I2C_EDID,0x84,0x49},
{ADV7482_I2C_EDID,0x85,0x90},
{ADV7482_I2C_EDID,0x86,0x1F},
{ADV7482_I2C_EDID,0x87,0x04},
{ADV7482_I2C_EDID,0x88,0x13},
{ADV7482_I2C_EDID,0x89,0x05},
{ADV7482_I2C_EDID,0x8A,0x14},
{ADV7482_I2C_EDID,0x8B,0x03},
{ADV7482_I2C_EDID,0x8C,0x12},
{ADV7482_I2C_EDID,0x8D,0x20},
{ADV7482_I2C_EDID,0x8E,0x23},
{ADV7482_I2C_EDID,0x8F,0x09},
{ADV7482_I2C_EDID,0x90,0x07},
{ADV7482_I2C_EDID,0x91,0x07},
{ADV7482_I2C_EDID,0x92,0x83},
{ADV7482_I2C_EDID,0x93,0x01},
{ADV7482_I2C_EDID,0x94,0x00},
{ADV7482_I2C_EDID,0x95,0x00},
{ADV7482_I2C_EDID,0x96,0xE2},
{ADV7482_I2C_EDID,0x97,0x00},
{ADV7482_I2C_EDID,0x98,0x0F},
{ADV7482_I2C_EDID,0x99,0x6E},
{ADV7482_I2C_EDID,0x9A,0x03},
{ADV7482_I2C_EDID,0x9B,0x0C},
{ADV7482_I2C_EDID,0x9C,0x00},
{ADV7482_I2C_EDID,0x9D,0x10},
{ADV7482_I2C_EDID,0x9E,0x00},
{ADV7482_I2C_EDID,0x9F,0x00},
{ADV7482_I2C_EDID,0xA0,0x2D},
{ADV7482_I2C_EDID,0xA1,0x20},
{ADV7482_I2C_EDID,0xA2,0x90},
{ADV7482_I2C_EDID,0xA3,0x04},
{ADV7482_I2C_EDID,0xA4,0x08},
{ADV7482_I2C_EDID,0xA5,0x10},
{ADV7482_I2C_EDID,0xA6,0x18},
{ADV7482_I2C_EDID,0xA7,0x10},
{ADV7482_I2C_EDID,0xA8,0x01},
{ADV7482_I2C_EDID,0xA9,0x1D},
{ADV7482_I2C_EDID,0xAA,0x00},
{ADV7482_I2C_EDID,0xAB,0x72},
{ADV7482_I2C_EDID,0xAC,0x51},
{ADV7482_I2C_EDID,0xAD,0xD0},
{ADV7482_I2C_EDID,0xAE,0x1E},
{ADV7482_I2C_EDID,0xAF,0x20},
{ADV7482_I2C_EDID,0xB0,0x6E},
{ADV7482_I2C_EDID,0xB1,0x28},
{ADV7482_I2C_EDID,0xB2,0x55},
{ADV7482_I2C_EDID,0xB3,0x00},
{ADV7482_I2C_EDID,0xB4,0x56},
{ADV7482_I2C_EDID,0xB5,0x50},
{ADV7482_I2C_EDID,0xB6,0x21},
{ADV7482_I2C_EDID,0xB7,0x00},
{ADV7482_I2C_EDID,0xB8,0x00},
{ADV7482_I2C_EDID,0xB9,0x1E},
{ADV7482_I2C_EDID,0xBA,0x01},
{ADV7482_I2C_EDID,0xBB,0x1D},
{ADV7482_I2C_EDID,0xBC,0x00},
{ADV7482_I2C_EDID,0xBD,0xBC},
{ADV7482_I2C_EDID,0xBE,0x52},
{ADV7482_I2C_EDID,0xBF,0xD0},
{ADV7482_I2C_EDID,0xC0,0x1E},
{ADV7482_I2C_EDID,0xC1,0x20},
{ADV7482_I2C_EDID,0xC2,0xB8},
{ADV7482_I2C_EDID,0xC3,0x28},
{ADV7482_I2C_EDID,0xC4,0x55},
{ADV7482_I2C_EDID,0xC5,0x40},
{ADV7482_I2C_EDID,0xC6,0x56},
{ADV7482_I2C_EDID,0xC7,0x50},
{ADV7482_I2C_EDID,0xC8,0x21},
{ADV7482_I2C_EDID,0xC9,0x00},
{ADV7482_I2C_EDID,0xCA,0x00},
{ADV7482_I2C_EDID,0xCB,0x1E},
{ADV7482_I2C_EDID,0xCC,0x01},
{ADV7482_I2C_EDID,0xCD,0x1D},
{ADV7482_I2C_EDID,0xCE,0x80},
{ADV7482_I2C_EDID,0xCF,0x18},
{ADV7482_I2C_EDID,0xD0,0x71},
{ADV7482_I2C_EDID,0xD1,0x1C},
{ADV7482_I2C_EDID,0xD2,0x16},
{ADV7482_I2C_EDID,0xD3,0x20},
{ADV7482_I2C_EDID,0xD4,0x58},
{ADV7482_I2C_EDID,0xD5,0x2C},
{ADV7482_I2C_EDID,0xD6,0x25},
{ADV7482_I2C_EDID,0xD7,0x00},
{ADV7482_I2C_EDID,0xD8,0x56},
{ADV7482_I2C_EDID,0xD9,0x50},
{ADV7482_I2C_EDID,0xDA,0x21},
{ADV7482_I2C_EDID,0xDB,0x00},
{ADV7482_I2C_EDID,0xDC,0x00},
{ADV7482_I2C_EDID,0xDD,0x9E},
{ADV7482_I2C_EDID,0xDE,0x01},
{ADV7482_I2C_EDID,0xDF,0x1D},
{ADV7482_I2C_EDID,0xE0,0x80},
{ADV7482_I2C_EDID,0xE1,0xD0},
{ADV7482_I2C_EDID,0xE2,0x72},
{ADV7482_I2C_EDID,0xE3,0x1C},
{ADV7482_I2C_EDID,0xE4,0x16},
{ADV7482_I2C_EDID,0xE5,0x20},
{ADV7482_I2C_EDID,0xE6,0x10},
{ADV7482_I2C_EDID,0xE7,0x2C},
{ADV7482_I2C_EDID,0xE8,0x25},
{ADV7482_I2C_EDID,0xE9,0x80},
{ADV7482_I2C_EDID,0xEA,0x56},
{ADV7482_I2C_EDID,0xEB,0x50},
{ADV7482_I2C_EDID,0xEC,0x21},
{ADV7482_I2C_EDID,0xED,0x00},
{ADV7482_I2C_EDID,0xEE,0x00},
{ADV7482_I2C_EDID,0xEF,0x9E},
{ADV7482_I2C_EDID,0xF0,0x00},
{ADV7482_I2C_EDID,0xF1,0x00},
{ADV7482_I2C_EDID,0xF2,0x00},
{ADV7482_I2C_EDID,0xF3,0x00},
{ADV7482_I2C_EDID,0xF4,0x00},
{ADV7482_I2C_EDID,0xF5,0x00},
{ADV7482_I2C_EDID,0xF6,0x00},
{ADV7482_I2C_EDID,0xF7,0x00},
{ADV7482_I2C_EDID,0xF8,0x00},
{ADV7482_I2C_EDID,0xF9,0x00},
{ADV7482_I2C_EDID,0xFA,0x00},
{ADV7482_I2C_EDID,0xFB,0x00},
{ADV7482_I2C_EDID,0xFC,0x00},
{ADV7482_I2C_EDID,0xFD,0x00},
{ADV7482_I2C_EDID,0xFE,0x00},
{ADV7482_I2C_EDID,0xFF,0xD6},
{ADV7482_I2C_REPEATER, 0x70, 0x10},
					/* EDID image, in 128-byte blocks. */

    {ADV7482_I2C_HDMI, 0x6C, 0x01},	/* HPA Manual Enable */
	{ADV7482_I2C_HDMI, 0xF8, 0x01},	/* HPA Asserted */

	{ADV7482_I2C_REPEATER, 0x74, 0x01},
				/* Manual enable active for E-EDID on Port A */
	{ADV7482_I2C_IO, 0x04, 0x00},	/* RGB Out of CP */ 
	{ADV7482_I2C_IO, 0x12, 0xF2},
		/* CSC Depends on ip Packets - SDR 444 */
	{ADV7482_I2C_IO, 0x17, 0x80},
		/* Luma & Chroma Values Can Reach 254d */
	{ADV7482_I2C_IO, 0x03, 0x86},	/* CP-Insert_AV_Code */
	{ADV7482_I2C_CP, 0x7C, 0x00},	/* ADI Required Write */
	{ADV7482_I2C_IO, 0x0C, 0xE0},	/* Enable LLC_DLL & Double LLC Timing */
	{ADV7482_I2C_IO, 0x0E, 0xDD},	/* LLC/PIX/SPI PINS TRISTATED AUD */
					/* Outputs Enabled */
	{ADV7482_I2C_IO, 0x10, 0xA0},	/* Enable 4-lane CSI Tx & Pixel Port */
	{ADV7482_I2C_TXA, 0x00, 0x84},	/* Enable 4-lane MIPI */
	{ADV7482_I2C_TXA, 0x00, 0xA4},	/* Set Auto DPHY Timing */
	{ADV7482_I2C_TXA, 0xDB, 0x10},	/* ADI Required Write */
	{ADV7482_I2C_TXA, 0xD6, 0x07},	/* ADI Required Write */
	{ADV7482_I2C_TXA, 0xC4, 0x0A},	/* ADI Required Write */
	{ADV7482_I2C_TXA, 0x71, 0x33},	/* ADI Required Write */
	{ADV7482_I2C_TXA, 0x72, 0x11},	/* ADI Required Write */
	{ADV7482_I2C_TXA, 0xF0, 0x00},	/* i2c_dphy_pwdn - 1'b0 */
	{ADV7482_I2C_TXA, 0x31, 0x82},	/* ADI Required Write */
	{ADV7482_I2C_TXA, 0x1E, 0x40},	/* ADI Required Write */
	{ADV7482_I2C_TXA, 0xDA, 0x01},	/* i2c_mipi_pll_en - 1'b1 */
	{ADV7482_I2C_WAIT, 0x00, 0x02},	/* delay 2 */
	{ADV7482_I2C_TXA, 0x00, 0x24 },	/* Power-up CSI-TX */
	{ADV7482_I2C_WAIT, 0x00, 0x01},	/* delay 1 */
	{ADV7482_I2C_TXA, 0xC1, 0x2B},	/* ADI Required Write */
	{ADV7482_I2C_WAIT, 0x00, 0x01},	/* delay 1 */
	{ADV7482_I2C_TXA, 0x31, 0x80},	/* ADI Required Write */
	/* for debug */
#ifdef REL_DGB_FORCE_TO_SEND_COLORBAR
	{ADV7482_I2C_CP, 0x37, 0x81},	/* Output Colorbars Pattern */
#endif
	{ADV7482_I2C_EOR, 0xFF, 0xFF}	/* End of register table */
};


static const struct adv7482_reg_value adv7482_init_txa_rgb[] = {
	{ADV7482_I2C_IO, 0x04, 0x02},	/* RGB Out of CP */
	{ADV7482_I2C_IO, 0x12, 0xF0},
};

static const struct adv7482_reg_value adv7482_init_txa_yuv[] = {
	{ADV7482_I2C_IO, 0x04, 0x00},	/* RGB Out of CP */
	{ADV7482_I2C_IO, 0x12, 0xF2}, 
};

/* 02-01 Analog CVBS to MIPI TX-B CSI 1-Lane - */
/* Autodetect CVBS Single Ended In Ain 1 - MIPI Out */
static const struct adv7482_reg_value adv7482_init_txb_1lane[] = {
	{ADV7482_I2C_IO, 0x00, 0x30},
		/* Disable chip powerdown - powerdown Rx */
	{ADV7482_I2C_IO, 0xF2, 0x01},	/* Enable I2C Read Auto-Increment */
	/* I2C Slave Address settings */
	{ADV7482_I2C_IO, 0xF3, ADV7482_I2C_DPLL * 2},	/* DPLL Map */
	{ADV7482_I2C_IO, 0xF4, ADV7482_I2C_CP * 2},	/* CP Map */
	{ADV7482_I2C_IO, 0xF5, ADV7482_I2C_HDMI * 2},	/* HDMI Map */
	{ADV7482_I2C_IO, 0xF6, ADV7482_I2C_EDID * 2},	/* EDID Map */
	{ADV7482_I2C_IO, 0xF7, ADV7482_I2C_REPEATER * 2},
						/* HDMI RX Repeater Map */
	{ADV7482_I2C_IO, 0xF8, ADV7482_I2C_INFOFRAME * 2},
						/* HDMI RX InfoFrame Map */
	{ADV7482_I2C_IO, 0xFA, ADV7482_I2C_CEC * 2},	/* CEC Map */
	{ADV7482_I2C_IO, 0xFB, ADV7482_I2C_SDP * 2},	/* SDP Map */
	{ADV7482_I2C_IO, 0xFC, ADV7482_I2C_TXB * 2},	/* CSI-TXB Map */
	{ADV7482_I2C_IO, 0xFD, ADV7482_I2C_TXA * 2},	/* CSI-TXA Map */
	{ADV7482_I2C_IO, 0x0E, 0xFF},	/* LLC/PIX/AUD/SPI PINS TRISTATED */
	{ADV7482_I2C_SDP, ADV7482_SDP_REG_PWR_MAN, ADV7482_SDP_PWR_MAN_ON}, /* Exit Power Down Mode */
	{ADV7482_I2C_SDP, 0x52, 0xCD},	/* ADI Required Write */
	
	{ADV7482_I2C_SDP, ADV7482_SDP_REG_INPUT_CONTROL,
		ADV7482_SDP_INPUT_CVBS_AIN1},	/* INSEL = CVBS in on Ain 1 */

	{ADV7482_I2C_SDP, ADV7482_SDP_REG_CTRL, 0x80},
		/* ADI Required Write */
	{ADV7482_I2C_SDP, 0x9C, 0x00},	/* ADI Required Write */
	{ADV7482_I2C_SDP, 0x9C, 0xFF},	/* ADI Required Write */
	{ADV7482_I2C_SDP, ADV7482_SDP_REG_CTRL, ADV7482_SDP_MAIN_MAP_RW},
		/* ADI Required Write */
	/* ADI recommended writes for improved video quality */
	{ADV7482_I2C_SDP, 0x80, 0x51},	/* ADI Required Write */
	{ADV7482_I2C_SDP, 0x81, 0x51},	/* ADI Required Write */
	{ADV7482_I2C_SDP, 0x82, 0x68},	/* ADI Required Write */
	{ADV7482_I2C_SDP, 0x03, 0x42},
		/* Tri-S Output Drivers, PwrDwn 656 pads */
	{ADV7482_I2C_SDP, 0x04, 0xB5},	/* ITU-R BT.656-4 compatible */
	{ADV7482_I2C_SDP, 0x13, 0x00},	/* ADI Required Write */
	{ADV7482_I2C_SDP, 0x17, 0x41},	/* Select SH1 */
	{ADV7482_I2C_SDP, 0x31, 0x12},	/* ADI Required Write */
	{ADV7482_I2C_SDP, 0xE6, 0x4F},
		/* Set V bit end position manually in NTSC mode */
#ifdef REL_DGB_FORCE_TO_SEND_COLORBAR
	{ADV7482_I2C_SDP, 0x0C, 0x01},	/* ColorBar */
	{ADV7482_I2C_SDP, 0x14, 0x01},	/* ColorBar */
#endif
	/* Enable 1-Lane MIPI Tx, */
	/* enable pixel output and route SD through Pixel port */
	{ADV7482_I2C_IO, 0x10, 0x70},
	{ADV7482_I2C_TXB, 0x00, 0x81},	/* Enable 1-lane MIPI */
	{ADV7482_I2C_TXB, 0x00, 0xA1},	/* Set Auto DPHY Timing */
#if 0 /* If CVBS input only is used, please enable this code. */
	{ADV7482_I2C_TXA, 0xF0, 0x00},	/* ADI Required Write */
	{ADV7482_I2C_TXA, 0xD6, 0x07},	/* ADI Required Write */
	{ADV7482_I2C_TXA, 0xC0, 0x3C},	/* ADI Required Write */
	{ADV7482_I2C_TXA, 0xC3, 0x3C},	/* ADI Required Write */
	{ADV7482_I2C_TXA, 0xC6, 0x3C},	/* ADI Required Write */
	{ADV7482_I2C_TXA, 0xC9, 0x3C},	/* ADI Required Write */
	{ADV7482_I2C_TXA, 0xCC, 0x3C},	/* ADI Required Write */
	{ADV7482_I2C_TXA, 0xD5, 0x03},	/* ADI Required Write */
#endif
	{ADV7482_I2C_TXB, 0xD2, 0x40},	/* ADI Required Write */
	{ADV7482_I2C_TXB, 0xC4, 0x0A},	/* ADI Required Write */
	{ADV7482_I2C_TXB, 0x71, 0x33},	/* ADI Required Write */
	{ADV7482_I2C_TXB, 0x72, 0x11},	/* ADI Required Write */
	{ADV7482_I2C_TXB, 0xF0, 0x00},	/* i2c_dphy_pwdn - 1'b0 */
	{ADV7482_I2C_TXB, 0x31, 0x82},	/* ADI Required Write */
	{ADV7482_I2C_TXB, 0x1E, 0x40},	/* ADI Required Write */
	{ADV7482_I2C_TXB, 0xDA, 0x01},	/* i2c_mipi_pll_en - 1'b1 */
	{ADV7482_I2C_WAIT, 0x00, 0x02},	/* delay 2 */
	{ADV7482_I2C_TXB, 0x00, 0x21 },	/* Power-up CSI-TX */
	{ADV7482_I2C_WAIT, 0x00, 0x01},	/* delay 1 */
	{ADV7482_I2C_TXB, 0xC1, 0x2B},	/* ADI Required Write */
	{ADV7482_I2C_WAIT, 0x00, 0x01},	/* delay 1 */
	{ADV7482_I2C_TXB, 0x31, 0x80},	/* ADI Required Write */
	{ADV7482_I2C_TXA, 0x7E, 0xA8},
	{ADV7482_I2C_EOR, 0xFF, 0xFF}	/* End of register table */
};
static const struct adv7482_reg_value adv7482_power_up_txa_4lane[] = {
	{ADV7482_I2C_TXA, 0x00, 0x84},	/* Enable 4-lane MIPI */
	{ADV7482_I2C_TXA, 0x00, 0xA4},	/* Set Auto DPHY Timing */
	{ADV7482_I2C_TXA, 0x31, 0x82},	/* ADI Required Write */
	{ADV7482_I2C_TXA, 0x1E, 0x40},	/* ADI Required Write */
	{ADV7482_I2C_TXA, 0xDA, 0x01},	/* i2c_mipi_pll_en - 1'b1 */
	{ADV7482_I2C_WAIT, 0x00, 0x02},	/* delay 2 */
	{ADV7482_I2C_TXA, 0x00, 0x24 },	/* Power-up CSI-TX */
	{ADV7482_I2C_WAIT, 0x00, 0x01},	/* delay 1 */
	{ADV7482_I2C_TXA, 0xC1, 0x2B},	/* ADI Required Write */
	{ADV7482_I2C_WAIT, 0x00, 0x01},	/* delay 1 */
	{ADV7482_I2C_TXA, 0x31, 0x80},	/* ADI Required Write */
	{ADV7482_I2C_EOR, 0xFF, 0xFF}	/* End of register table */
};
static const struct adv7482_reg_value adv7482_power_down_txa_4lane[] = {
	{ADV7482_I2C_TXA, 0x31, 0x82},	/* ADI Required Write */
	{ADV7482_I2C_TXA, 0x1E, 0x00},	/* ADI Required Write */
	{ADV7482_I2C_TXA, 0x00, 0x84},	/* Enable 4-lane MIPI */
	{ADV7482_I2C_TXA, 0xDA, 0x01},	/* i2c_mipi_pll_en - 1'b1 */
	{ADV7482_I2C_TXA, 0xC1, 0x3B},	/* ADI Required Write */
	{ADV7482_I2C_EOR, 0xFF, 0xFF}	/* End of register table */
};
static const struct adv7482_reg_value adv7482_power_up_txb_1lane[] = {
	{ADV7482_I2C_TXB, 0x00, 0x81},	/* Enable 1-lane MIPI */
	{ADV7482_I2C_TXB, 0x00, 0xA1},	/* Set Auto DPHY Timing */
	{ADV7482_I2C_TXB, 0x31, 0x82},	/* ADI Required Write */
	{ADV7482_I2C_TXB, 0x1E, 0x40},	/* ADI Required Write */
	{ADV7482_I2C_TXB, 0xDA, 0x01},	/* i2c_mipi_pll_en - 1'b1 */
	{ADV7482_I2C_WAIT, 0x00, 0x02},	/* delay 2 */
	{ADV7482_I2C_TXB, 0x00, 0x21 },	/* Power-up CSI-TX */
	{ADV7482_I2C_WAIT, 0x00, 0x01},	/* delay 1 */
	{ADV7482_I2C_TXB, 0xC1, 0x2B},	/* ADI Required Write */
	{ADV7482_I2C_WAIT, 0x00, 0x01},	/* delay 1 */
	{ADV7482_I2C_TXB, 0x31, 0x80},	/* ADI Required Write */
	{ADV7482_I2C_EOR, 0xFF, 0xFF}	/* End of register table */
};
static const struct adv7482_reg_value adv7482_power_down_txb_1lane[] = {
	{ADV7482_I2C_TXB, 0x31, 0x82},	/* ADI Required Write */
	{ADV7482_I2C_TXB, 0x1E, 0x00},	/* ADI Required Write */
	{ADV7482_I2C_TXB, 0x00, 0x81},	/* Enable 4-lane MIPI */
	{ADV7482_I2C_TXB, 0xDA, 0x01},	/* i2c_mipi_pll_en - 1'b1 */
	{ADV7482_I2C_TXB, 0xC1, 0x3B},	/* ADI Required Write */
	{ADV7482_I2C_EOR, 0xFF, 0xFF}	/* End of register table */
};
static const struct adv7482_reg_value adv7482_power_up_hdmi_rx[] = {
	/* Disable chip powerdown & Enable HDMI Rx block */
	{ADV7482_I2C_IO, 0x00, 0x40},
	{ADV7482_I2C_EOR, 0xFF, 0xFF}	/* End of register table */
};
static const struct adv7482_reg_value adv7482_power_down_hdmi_rx[] = {
	{ADV7482_I2C_IO, 0x00, 0x30},	/* Disable chip powerdown */
	{ADV7482_I2C_EOR, 0xFF, 0xFF}	/* End of register table */
};
static const struct adv7482_reg_value adv7482_enable_csi4_csi1[] = {
	{ADV7482_I2C_IO, 0x10, 0xE0},	/* Enable 4-lane CSI Tx & Pixel Port */
	{ADV7482_I2C_EOR, 0xFF, 0xFF}	/* End of register table */
};
static const struct adv7482_reg_value adv7482_set_virtual_channel0[] = {
	{ADV7482_I2C_TXB, 0x0D, 0x00},	/* Set virtual channel 0 */
	{ADV7482_I2C_TXA, 0x0D, 0x00},	/* Set virtual channel 0 */
	{ADV7482_I2C_EOR, 0xFF, 0xFF}	/* End of register table */
};
static const struct adv7482_reg_value adv7482_set_virtual_channel1[] = {
	{ADV7482_I2C_TXB, 0x0D, 0x40},	/* Set virtual channel 1 */
	{ADV7482_I2C_TXA, 0x0D, 0x40},	/* Set virtual channel 1 */
	{ADV7482_I2C_EOR, 0xFF, 0xFF}	/* End of register table */
};
static const struct adv7482_reg_value adv7482_set_virtual_channel2[] = {
	{ADV7482_I2C_TXB, 0x0D, 0x80},	/* Set virtual channel 2 */
	{ADV7482_I2C_TXA, 0x0D, 0x80},	/* Set virtual channel 2 */
	{ADV7482_I2C_EOR, 0xFF, 0xFF}	/* End of register table */
};
static const struct adv7482_reg_value adv7482_set_virtual_channel3[] = {
	{ADV7482_I2C_TXB, 0x0D, 0xC0},	/* Set virtual channel 3 */
	{ADV7482_I2C_TXA, 0x0D, 0xC0},	/* Set virtual channel 3 */
	{ADV7482_I2C_EOR, 0xFF, 0xFF}	/* End of register table */
};
enum decoder_input_interface {
	DECODER_INPUT_INTERFACE_RGB888,
	DECODER_INPUT_INTERFACE_YCBCR422,
};
enum decoder_input {
	DECODER_INPUT_COMPOSITE,
	DECODER_INPUT_HDMI
};
/**
 * struct adv7482_link_config - Describes adv7482 hardware configuration
 * @input_colorspace:		The input colorspace (RGB, YUV444, YUV422)
 */
struct adv7482_link_config {
	enum decoder_input_interface	input_interface;
	enum decoder_input			input;
	struct adv7482_reg_value	*regs;
	struct adv7482_reg_value	*power_up;
	struct adv7482_reg_value	*power_down;
	int (*init_device)(void *);
	int (*init_controls)(void *);
	int (*s_power)(void *);
	int (*s_ctrl)(void *);
	int (*enum_mbus_code)(void *);
	int (*set_pad_format)(void *);
	int (*get_pad_format)(void *);
	int (*s_std)(void *);
	int (*querystd)(void *);
	int (*g_input_status)(void *);
	int (*s_routing)(void *);
	int (*g_mbus_config)(void *);
	struct device	*dev;
	bool		sw_reset;
	bool		hdmi_in;
	bool		sdp_in;
	int		vc_ch;
};
struct adv7482_state {
	struct v4l2_ctrl_handler		ctrl_hdl;
	struct v4l2_subdev			sd;
	struct media_pad			pad;
	 /* mutual excl. when accessing chip */
	struct mutex				mutex;
	int					irq;
	v4l2_std_id				curr_norm;
	bool					autodetect;
	bool					powered;
	const struct adv7482_color_format	*cfmt;
	u32					width;
	u32					height;
	struct i2c_client			*client;
	unsigned int				register_page;
	struct i2c_client			*csi_client;
	enum v4l2_field				field;
	struct device				*dev;
	struct adv7482_link_config		mipi_csi2_link[2];
	struct v4l2_dv_timings timings;

	struct snd_soc_dai dai;
	struct snd_soc_dai_driver dai_drv;

	char *mclk_name;
	struct clk_hw *mclk_hw;
	unsigned int freq;
	unsigned int fmt, tdm;
};

u32 media_bus_formats[] = {

	MEDIA_BUS_FMT_RGB888_1X24
};

#define ADV748X_HDMI_MIN_WIDTH		640
#define ADV748X_HDMI_MAX_WIDTH		1920
#define ADV748X_HDMI_MIN_HEIGHT		480
#define ADV748X_HDMI_MAX_HEIGHT		1200

/* V4L2_DV_BT_CEA_720X480I59_94 - 0.5 MHz */
#define ADV748X_HDMI_MIN_PIXELCLOCK	13000000
/* V4L2_DV_BT_DMT_1600X1200P60 */
#define ADV748X_HDMI_MAX_PIXELCLOCK	162000000

static const struct v4l2_dv_timings_cap adv7482_timings_cap = {
	.type = V4L2_DV_BT_656_1120,
	/* keep this initialization for compatibility with GCC < 4.4.6 */
	.reserved = { 0 },

	V4L2_INIT_BT_TIMINGS(ADV748X_HDMI_MIN_WIDTH, ADV748X_HDMI_MAX_WIDTH,
			     ADV748X_HDMI_MIN_HEIGHT, ADV748X_HDMI_MAX_HEIGHT,
			     ADV748X_HDMI_MIN_PIXELCLOCK,
			     ADV748X_HDMI_MAX_PIXELCLOCK,
			     V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT,
			     V4L2_DV_BT_CAP_PROGRESSIVE)
};

/*****************************************************************************/
/*  Private functions                                                        */
/*****************************************************************************/
#define to_adv7482_sd(_ctrl) (&container_of(_ctrl->handler,		\
					    struct adv7482_state,	\
					    ctrl_hdl)->sd)
static inline struct adv7482_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct adv7482_state, sd);
}

static inline struct adv7482_state *dai_to_state(struct snd_soc_dai *dai)
{
	return container_of(dai, struct adv7482_state, dai);
}

/*
 * adv7482_write_registers() - Write adv7482 device registers
 * @client: pointer to i2c_client structure
 * @regs: pointer to adv7482_reg_value structure
 *
 * Write the specified adv7482 register's values.
 */
static int adv7482_write_registers(struct i2c_client *client,
					const struct adv7482_reg_value *regs)
{
	struct i2c_msg msg;
	u8 data_buf[2];
	int ret = -EINVAL;
	if (!client->adapter)
		return -ENODEV;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = &data_buf[0];
	
	while (regs->addr != ADV7482_I2C_EOR) {
		if (regs->addr == ADV7482_I2C_WAIT)
		{
		
			msleep(regs->value);
		}
		else {
			msg.addr = regs->addr;
			data_buf[0] = regs->reg;
			data_buf[1] = regs->value;
			
			ret = i2c_transfer(client->adapter, &msg, 1);
			if (ret < 0)
				break;
		}
		regs++;
	}
	return (ret < 0) ? ret : 0;
}
/*
 * adv7482_write_register() - Write adv7482 device register
 * @client: pointer to i2c_client structure
 * @addr: i2c slave address of adv7482 device
 * @reg: adv7482 device register address
 * @value: the value to be written
 *
 * Write the specified adv7482 register's value.
 */
static int adv7482_write_register(struct i2c_client *client,
		u8 addr, u8 reg, u8 value)
{
	struct adv7482_reg_value regs[2];
	int ret;
	regs[0].addr = addr;
	regs[0].reg = reg;
	regs[0].value = value;
	regs[1].addr = ADV7482_I2C_EOR;
	regs[1].reg = 0xFF;
	regs[1].value = 0xFF;
	ret = adv7482_write_registers(client, regs);
	return ret;
}
/*
 * adv7482_read_register() - Read adv7482 device register
 * @client: pointer to i2c_client structure
 * @addr: i2c slave address of adv7482 device
 * @reg: adv7482 device register address
 * @value: pointer to the value
 *
 * Read the specified adv7482 register's value.
 */
static int adv7482_read_register(struct i2c_client *client,
		u8 addr, u8 reg, u8 *value)
{
	struct i2c_msg msg[2];
	u8 reg_buf, data_buf;
	int ret;
	if (!client->adapter)
		return -ENODEV;
	msg[0].addr = addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg_buf;
	msg[1].addr = addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = &data_buf;
	reg_buf = reg;
	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0)
		return ret;
	*value = data_buf;
	return (ret < 0) ? ret : 0;
}
static int adv7482_read_sdp_main_info(struct i2c_client *client,
		struct adv7482_sdp_main_info *info)
{
	int ret;
	u8 value;
	ret = adv7482_write_register(client, ADV7482_I2C_SDP,
				ADV7482_SDP_REG_CTRL, ADV7482_SDP_RO_MAIN_MAP);
	if (ret < 0)
		return ret;
	/* status_reg_10 */
	ret = adv7482_read_register(client, ADV7482_I2C_SDP,
			ADV7482_SDP_R_REG_10, &value);
	if (ret < 0)
		return ret;
	info->status_reg_10 = value;

	return ret;
}
static v4l2_std_id adv7482_std_to_v4l2(u8 status_reg_10)
{
	/* in case V4L2_IN_ST_NO_SIGNAL */
	if (!(status_reg_10 & ADV7482_SDP_R_REG_10_IN_LOCK))
		return V4L2_STD_UNKNOWN;
	switch (status_reg_10 & ADV7482_SDP_R_REG_10_AUTOD_MASK) {
	case ADV7482_SDP_R_REG_10_AUTOD_NTSM_M_J:
		return V4L2_STD_NTSC;
	case ADV7482_SDP_R_REG_10_AUTOD_NTSC_4_43:
		return V4L2_STD_NTSC_443;
	case ADV7482_SDP_R_REG_10_AUTOD_PAL_M:
		return V4L2_STD_PAL_M;
	case ADV7482_SDP_R_REG_10_AUTOD_PAL_60:
		return V4L2_STD_PAL_60;
	case ADV7482_SDP_R_REG_10_AUTOD_PAL_B_G:
		return V4L2_STD_PAL;
	case ADV7482_SDP_R_REG_10_AUTOD_SECAM:
		return V4L2_STD_SECAM;
	case ADV7482_SDP_R_REG_10_AUTOD_PAL_COMB:
		return V4L2_STD_PAL_Nc | V4L2_STD_PAL_N;
	case ADV7482_SDP_R_REG_10_AUTOD_SECAM_525:
		return V4L2_STD_SECAM;
	default:
		return V4L2_STD_UNKNOWN;
	}
}
static u32 adv7482_status_to_v4l2(u8 status_reg_10)
{
	if (!(status_reg_10 & ADV7482_SDP_R_REG_10_IN_LOCK))
		return V4L2_IN_ST_NO_SIGNAL;
	return 0;
}
static int __adv7482_status(struct adv7482_state *state, u32 *status,
			    v4l2_std_id *std)
{
	int ret;
	u8 status_reg_10;
	struct adv7482_sdp_main_info sdp_info;
	ret = adv7482_read_sdp_main_info(state->client, &sdp_info);
	if (ret < 0)
		return ret;
	status_reg_10 = sdp_info.status_reg_10;
	if (status_reg_10 < 0)
		return (int)status_reg_10;
	if (status)
		*status = adv7482_status_to_v4l2(status_reg_10);
	if (std)
		*std = adv7482_std_to_v4l2(status_reg_10);
	return 0;
}
/*
 * adv7482_get_vid_info() - Get video information
 * @sd: pointer to standard V4L2 sub-device structure
 * @progressive: progressive or interlace
 * @width: line width
 * @height: lines per frame
 *
 * Get video information.
 */
static int adv7482_get_vid_info(struct v4l2_subdev *sd, u8 *progressive,
				u32 *width, u32 *height, u8 *signal)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 hdmi_int;
	u8 msb;
	u8 lsb;
	int ret;
	printk("adv7482_get_vid_info++\n");
	if (signal)
		*signal = 0;
	/* decide line width */
	ret = adv7482_read_register(client, ADV7482_I2C_HDMI,
			ADV7482_HDMI_STATUS1_REG, &msb);
	if (ret < 0)
		return ret;
	if (!(msb & ADV7482_HDMI_VF_LOCKED_FLG) ||
	    !(msb & ADV7482_HDMI_DERF_LOCKED_FLG))
		return -EIO;
	if (signal)
		*signal = 1;
	/* decide interlaced or progressive */
	ret = adv7482_read_register(client, ADV7482_I2C_HDMI,
			ADV7482_HDMI_STATUS2_REG, &hdmi_int);
	if (ret < 0)
		return ret;
	*progressive =  1;
	if ((hdmi_int & ADV7482_HDMI_IP_FLAG) != 0)
		*progressive =  0;
	ret = adv7482_read_register(client, ADV7482_I2C_HDMI,
			ADV7482_HDMI_LWIDTH_REG, &lsb);
	if (ret < 0)
		return ret;
	*width = (u32)(ADV7482_HDMI_LWIDTH_MSBS_MASK & msb);
	*width = (lsb | (*width << 8));
	/* decide lines per frame */
	ret = adv7482_read_register(client, ADV7482_I2C_HDMI,
			ADV7482_HDMI_F0HEIGHT_MSBS_REG, &msb);
	if (ret < 0)
		return ret;
	ret = adv7482_read_register(client, ADV7482_I2C_HDMI,
			ADV7482_HDMI_F0HEIGHT_LSBS_REG, &lsb);
	if (ret < 0)
		return ret;
	*height = (u32)(ADV7482_HDMI_F0HEIGHT_MSBS_MASK & msb);
	*height = (lsb | (*height << 8));
	if (!(*progressive))
		*height = *height * 2;
	if (*width == 0 || *height == 0)
		return -EIO;
	printk("adv7482_get_vid_info(%d,%d,%d)--\n",*progressive,*width,*height);
	return 0;
}
static int adv7482_set_vid_info(struct v4l2_subdev *sd)
{
	struct adv7482_state *state = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 progressive;
	u8 signal;
	u8 val = ADV7482_IO_CP_VID_STD_480P;
	u32 width;
	u32 height;
	int ret;
	/* Get video information */
	ret = adv7482_get_vid_info(sd, &progressive, &width, &height, &signal);
	if (ret < 0) {
		width		= ADV7482_MAX_WIDTH;
		height		= ADV7482_MAX_HEIGHT;
		progressive	= 1;
	} else {
		if ((width == 640) &&
			(height == 480) && (progressive == 1)) {
			val = ADV7482_IO_CP_VID_STD_VGA60;
			dev_info(state->dev,
				 "Changed active resolution to 640x480p\n");
		} else if ((width == 720) &&
			(height == 480) && (progressive == 1)) {
			val = ADV7482_IO_CP_VID_STD_480P;
			dev_info(state->dev,
				 "Changed active resolution to 720x480p\n");
		} else if ((width == 720) &&
			(height == 576) && (progressive == 1)) {
			val = ADV7482_IO_CP_VID_STD_576P;
			dev_info(state->dev,
				 "Changed active resolution to 720x576p\n");
		} else if ((width == 1280) &&
			(height == 720) && (progressive == 1)) {
			val = ADV7482_IO_CP_VID_STD_720P;
			dev_info(state->dev,
				 "Changed active resolution to 1280x720p\n");
		} else if ((width == 1920) &&
			(height == 1080) && (progressive == 1)) {
			val = ADV7482_IO_CP_VID_STD_1080P;
		
		} else if ((width == 1920) &&
			(height == 1080) && (progressive == 0)) {
			val = ADV7482_IO_CP_VID_STD_1080I;

		} else {
			dev_err(state->dev,
				 "Not support resolution %dx%d%c\n",
				  width, height, (progressive) ? 'p' : 'i');
			return -EINVAL;
		}
	}
	/*
	 * The resolution of 720p, 1080i and 1080p is Hsync width of 40 pixel
	 * clock cycles. These resolutions must be shifted horizontally to
	 * the left in active video mode.
	 */
	if ((val == ADV7482_IO_CP_VID_STD_1080I) ||
		(val == ADV7482_IO_CP_VID_STD_1080P)) {
		ret = adv7482_write_register(client,
					ADV7482_I2C_CP, 0x8B, 0x43);
		ret = adv7482_write_register(client,
					ADV7482_I2C_CP, 0x8C, 0xD4);
		ret = adv7482_write_register(client,
					ADV7482_I2C_CP, 0x8B, 0x4F);
		ret = adv7482_write_register(client,
					ADV7482_I2C_CP, 0x8D, 0xD4);
	} else if (val == ADV7482_IO_CP_VID_STD_720P) {
		ret = adv7482_write_register(client,
					ADV7482_I2C_CP, 0x8B, 0x43);
		ret = adv7482_write_register(client,
					ADV7482_I2C_CP, 0x8C, 0xD8);
		ret = adv7482_write_register(client,
					ADV7482_I2C_CP, 0x8B, 0x4F);
		ret = adv7482_write_register(client,
					ADV7482_I2C_CP, 0x8D, 0xD8);
	} else {
		ret = adv7482_write_register(client,
					ADV7482_I2C_CP, 0x8B, 0x40);
		ret = adv7482_write_register(client,
					ADV7482_I2C_CP, 0x8C, 0x00);
		ret = adv7482_write_register(client,
					ADV7482_I2C_CP, 0x8B, 0x40);
		ret = adv7482_write_register(client,
					ADV7482_I2C_CP, 0x8D, 0x00);
	}
	ret = adv7482_write_register(client, ADV7482_I2C_IO,
				ADV7482_IO_CP_VID_STD_REG, val);
	return ret;
}
/*****************************************************************************/
/*  V4L2 decoder i/f handler for v4l2_subdev_core_ops                        */
/*****************************************************************************/
/*
 * adv7482_querystd() - V4L2 decoder i/f handler for querystd
 * @sd: ptr to v4l2_subdev struct
 * @std: standard input video id
 *
 * Obtains the video standard input id
 */
static int adv7482_querystd(struct v4l2_subdev *sd, v4l2_std_id *std)
{
	struct adv7482_state *state = to_state(sd);
	struct adv7482_link_config *config = &state->mipi_csi2_link[0];
	int err = mutex_lock_interruptible(&state->mutex);
	if (err)
		return err;
	if (config->input == DECODER_INPUT_COMPOSITE)
		/* when we are interrupt driven we know the state */
		if (!state->autodetect)
			*std = state->curr_norm;
		else
			err = __adv7482_status(state, NULL, std);
	else
		*std = V4L2_STD_ATSC;
	mutex_unlock(&state->mutex);
	return err;
}

static int adv7482_s_dv_timings(struct v4l2_subdev *sd,
				     struct v4l2_dv_timings *timings)
{
	int i;
	printk("adv7482_s_dv_timings\n");
	struct adv7482_state *state = to_state(sd);

	if (!timings)
		return -EINVAL;
	v4l2_print_dv_timings(sd->name, "adv7482_s_dv_timings: ",
			timings, false);

	if (v4l2_match_dv_timings(&state->timings, timings, 0, false)) {
		v4l2_dbg(1, debug, sd, "%s: no change\n", __func__);
		return 0;
	}


	if (!v4l2_valid_dv_timings(timings, &adv7482_timings_cap, NULL, NULL)) {
		v4l2_dbg(1, debug, sd, "%s: timings out of range\n", __func__);
		return -ERANGE;
	}

	for (i = 0; i < ARRAY_SIZE(adv748x_hdmi_video_standards); i++) {
		if (!v4l2_match_dv_timings(timings, &adv748x_hdmi_video_standards[i].timings, 250000,
					   false))
			continue;
	}

	if (i >= ARRAY_SIZE(adv748x_hdmi_video_standards))
		return -EINVAL;

	state->timings = *timings;

	return 0;
}

static int adv7482_g_dv_timings(struct v4l2_subdev *sd,
				     struct v4l2_dv_timings *timings)
{
	printk("adv7482_g_dv_timings\n");
	struct adv7482_state *state = to_state(sd);
	if (!timings)
		return -EINVAL;
	*timings = state->timings;
	return 0;
}

static int adv7482_query_dv_timings(struct v4l2_subdev *sd,
				     struct v4l2_dv_timings *timings)
{
	printk("adv7482_query_dv_timings\n");
	u8 progressive;
	u8 signal;
	u32 width;
	u32 height;
	struct adv7482_state *state = to_state(sd);
	if (!timings)
		return -EINVAL;
	if (debug)
		v4l2_print_dv_timings(sd->name, "adv7482_query_dv_timings: ",
			timings, false);
	if (!v4l2_valid_dv_timings(timings, &adv7482_timings_cap, NULL, NULL)) {
		v4l2_dbg(1, debug, sd, "%s: timings out of range\n", __func__);
		return -ERANGE;
	}

	return 0;
}


static bool adv7482_check_dv_timings(const struct v4l2_dv_timings *timings,
					  void *hdl)
{
	unsigned int i;

	for (i = 0; adv748x_hdmi_video_standards[i].timings.bt.width; i++)
		if (v4l2_match_dv_timings(timings, &adv748x_hdmi_video_standards[i].timings, 0, false))
			return true;

	return false;
}

static int adv7482_enum_dv_timings(struct v4l2_subdev *sd,
					struct v4l2_enum_dv_timings *timings)
{
	printk("adv7482_enum_dv_timings\n");
	return v4l2_enum_dv_timings_cap(timings, &adv7482_timings_cap,
					adv7482_check_dv_timings, NULL);
}

static int adv7482_dv_timings_cap(struct v4l2_subdev *sd,
				       struct v4l2_dv_timings_cap *cap)
{
	printk("adv7482_dv_timings_cap\n");
	*cap = adv7482_timings_cap;
	return 0;
}
/*
 * adv7482_g_input_status() - V4L2 decoder i/f handler for g_input_status
 * @sd: ptr to v4l2_subdev struct
 * @status: video input status flag
 *
 * Obtains the video input status flags.
 */
static int adv7482_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct adv7482_state *state = to_state(sd);
	struct adv7482_link_config *config = &state->mipi_csi2_link[0];
	printk("adv7482_g_input_status(%x)\n",config->input_interface);
	u8 status1 = 0;
	int ret = mutex_lock_interruptible(&state->mutex);
	if (ret)
		return ret;
	if (config->input == DECODER_INPUT_COMPOSITE)
		ret = __adv7482_status(state, status, NULL);
	else {
		ret = adv7482_read_register(client, ADV7482_I2C_HDMI,
				ADV7482_HDMI_STATUS1_REG, &status1);
		if (ret < 0)
			goto out;
		if (!(status1 & ADV7482_HDMI_VF_LOCKED_FLG))
			*status = V4L2_IN_ST_NO_SIGNAL;
		else if (!(status1 & ADV7482_HDMI_DERF_LOCKED_FLG))
			*status = V4L2_IN_ST_NO_SIGNAL;
		else
			*status = 0;
	}
	ret = 0;
out:
	mutex_unlock(&state->mutex);
	return ret;
}
static int adv7482_enum_mbus_code(struct v4l2_subdev *sd,
				 
				   struct v4l2_subdev_state *,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct adv7482_state *state = to_state(sd);
	struct adv7482_link_config *config = &state->mipi_csi2_link[0];

	if (code->index != 0)
		return -EINVAL;
	if (config->input == DECODER_INPUT_COMPOSITE)
		code->code = MEDIA_BUS_FMT_YUYV8_2X8;
	else
	{
		if (code->index >= ARRAY_SIZE(media_bus_formats))
			return -EINVAL;
	   code->code = media_bus_formats[code->index];


	}

	return 0;
}
static int adv7482_mbus_fmt(struct v4l2_subdev *sd,
			    struct v4l2_mbus_framefmt *fmt)
{
	printk("adv7482_mbus_fmt++");
	struct adv7482_state *state = to_state(sd);
	printk("code=%x, fmt=%x\n",fmt->code, fmt->colorspace);
	u8 progressive;
	u8 signal;
	u32 width;
	u32 height;
	int ret;
	struct adv7482_link_config *config = &state->mipi_csi2_link[0];
	u8 status_reg_10;
	struct adv7482_sdp_main_info sdp_info;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	if (config->input == DECODER_INPUT_COMPOSITE) {
		ret = __adv7482_status(state, NULL, &state->curr_norm);
		if (ret < 0)
			return ret;
		fmt->code = MEDIA_BUS_FMT_YUYV8_2X8;
		fmt->colorspace = V4L2_COLORSPACE_SMPTE170M;
		fmt->width = 720;
		fmt->height = state->curr_norm & V4L2_STD_525_60 ? 480 : 576;
		/* Get video information */
		ret = adv7482_read_sdp_main_info(client, &sdp_info);
		if (ret < 0)
			return ret;
		status_reg_10 = sdp_info.status_reg_10;
		
		pr_info("%s info->status_reg_10 = 0x%02X = %d\n",__FUNCTION__,status_reg_10,(int)status_reg_10);
		if ((int)status_reg_10 < 0){
		 	dev_info(state->dev,
		 		"Not detect any video input signal\n");
		}
		
		else {
			if ((status_reg_10 & ADV7482_SDP_R_REG_10_IN_LOCK) &&
				(status_reg_10 & ADV7482_SDP_R_REG_10_FSC_LOCK)
				&& (((status_reg_10 &
				ADV7482_SDP_R_REG_10_AUTOD_PAL_M) ==
				ADV7482_SDP_R_REG_10_AUTOD_PAL_M) ||
				((status_reg_10 &
				ADV7482_SDP_R_REG_10_AUTOD_PAL_60) ==
				ADV7482_SDP_R_REG_10_AUTOD_PAL_60) ||
				((status_reg_10 &
				ADV7482_SDP_R_REG_10_AUTOD_PAL_B_G) ==
				ADV7482_SDP_R_REG_10_AUTOD_PAL_B_G) ||
				((status_reg_10 &
				ADV7482_SDP_R_REG_10_AUTOD_PAL_COMB) ==
				ADV7482_SDP_R_REG_10_AUTOD_PAL_COMB)))
				dev_info(state->dev,
				   "Detected the PAL video input signal\n");
			else if ((status_reg_10 & ADV7482_SDP_R_REG_10_IN_LOCK)
				&& (status_reg_10 &
				ADV7482_SDP_R_REG_10_FSC_LOCK) &&
				(((status_reg_10 &
				ADV7482_SDP_R_REG_10_AUTOD_NTSC_4_43) ==
				ADV7482_SDP_R_REG_10_AUTOD_NTSC_4_43) ||
				((status_reg_10 &
				ADV7482_SDP_R_REG_10_AUTOD_MASK) ==
				ADV7482_SDP_R_REG_10_AUTOD_NTSM_M_J)))
				dev_info(state->dev,
				   "Detected the NTSC video input signal\n");
			else{
				dev_info(state->dev,
				   "%d Not detect any video input signal\n",__LINE__);
			}
		}
		state->width = fmt->width;
		state->height = fmt->height;
		state->field = V4L2_FIELD_INTERLACED;
	} else {
		if(fmt->code)
		{
			printk("Setting colour space\n");
			switch(fmt->code)
			{
				case MEDIA_BUS_FMT_YUYV8_2X8:
				case MEDIA_BUS_FMT_VYUY8_2X8:
				case MEDIA_BUS_FMT_UYVY8_2X8:
					printk("Using UYVY colour space\n");
					fmt->colorspace = V4L2_COLORSPACE_SRGB;
					ret = adv7482_write_registers(state->client,adv7482_init_txa_rgb);
					break;
				case MEDIA_BUS_FMT_YVYU8_2X8:
				case MEDIA_BUS_FMT_YUYV8_1X16:
				case MEDIA_BUS_FMT_VYUY8_1X16:
				case MEDIA_BUS_FMT_UYVY8_1X16:
				case MEDIA_BUS_FMT_YVYU8_1X16:
					printk("Using YUV colour space\n");
					fmt->colorspace = V4L2_COLORSPACE_REC709;
					ret = adv7482_write_registers(state->client,adv7482_init_txa_yuv);
					break;

				case MEDIA_BUS_FMT_RGB888_1X32_PADHI:
				case MEDIA_BUS_FMT_RGB888_1X24:
					printk("Using RGB colour space\n");
					fmt->colorspace = V4L2_COLORSPACE_SRGB;
					ret = adv7482_write_registers(state->client,adv7482_init_txa_rgb);
					break;

				default:
					printk("Unrecognised input: %x\n",fmt->code);
					if (config->input_interface == DECODER_INPUT_INTERFACE_YCBCR422) {
					fmt->code = MEDIA_BUS_FMT_UYVY8_2X8;
					fmt->colorspace = V4L2_COLORSPACE_REC709 ;
				} else {
					fmt->code = MEDIA_BUS_FMT_RGB888_1X24;
					fmt->colorspace = V4L2_COLORSPACE_SRGB;
				}
			}
		}
		else
		{
			if (config->input_interface == DECODER_INPUT_INTERFACE_YCBCR422) {
				fmt->code = MEDIA_BUS_FMT_UYVY8_2X8;
				fmt->colorspace = V4L2_COLORSPACE_REC709 ;
			} else {
				fmt->code = MEDIA_BUS_FMT_RGB888_1X24;
				fmt->colorspace = V4L2_COLORSPACE_SRGB;
			}
		}
		/* Get video information */
		ret = adv7482_get_vid_info(sd, &progressive,
			&width, &height, &signal);
		if (ret < 0) {
			printk("adv7482_get_vid_info ret %d\n",ret);
			width		= ADV7482_MAX_WIDTH;
			height		= ADV7482_MAX_HEIGHT;
			progressive	= 1;
		}
		if (signal)
			dev_info(state->dev,
			 "Detected the HDMI video input signal (%dx%d%c)\n",
			  width, height, (progressive) ? 'p' : 'i');
		else
			dev_info(state->dev,
				"%d Not detect any video input signal\n",__LINE__);
		state->width = width;
		state->height = height;
		state->field =
		      (progressive) ? V4L2_FIELD_NONE : V4L2_FIELD_INTERLACED;
		fmt->width = state->width;
		fmt->height = state->height;
	}
	printk("adv7482_mbus_fmt(%d,%d)--",fmt->width,fmt->height);
	return 0;
}
/*
 * adv7482_cropcap() - V4L2 decoder i/f handler for cropcap
 * @sd: pointer to standard V4L2 sub-device structure
 * @a: pointer to standard V4L2 cropcap structure
 *
 * Gets cropping limits, default cropping rectangle and pixel aspect.
 */
static int adv7482_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	struct adv7482_state *state = to_state(sd);
	u8 progressive;
	u32 width;
	u32 height;
	int ret;
	struct adv7482_link_config *config = &state->mipi_csi2_link[0];
	/* cropping limits */
	a->bounds.left = 0;
	a->bounds.top  = 0;
	if (config->input == DECODER_INPUT_COMPOSITE) {
		ret = __adv7482_status(state, NULL, &state->curr_norm);
		if (ret < 0)
			return ret;
		a->bounds.width  = 720;
		a->bounds.height = state->curr_norm & V4L2_STD_525_60
							 ? 480 : 576;
	} else {
		/* Get video information */
		ret = adv7482_get_vid_info(sd, &progressive,
						 &width, &height, NULL);
		if (ret < 0) {
			a->bounds.width  = ADV7482_MAX_WIDTH;
			a->bounds.height = ADV7482_MAX_HEIGHT;
		} else {
			a->bounds.width  = width;
			a->bounds.height = height;
		}
	}
	/* default cropping rectangle */
	a->defrect = a->bounds;
	/* does not support scaling */
	a->pixelaspect.numerator   = 1;
	a->pixelaspect.denominator = 1;
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	return 0;
}
/*
 * adv7482_g_crop() - V4L2 decoder i/f handler for g_crop
 * @sd: pointer to standard V4L2 sub-device structure
 * @a: pointer to standard V4L2 cropcap structure
 *
 * Gets current cropping rectangle.
 */
static int adv7482_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct adv7482_state *state = to_state(sd);
	u8 progressive;
	u32 width;
	u32 height;
	int ret;
	struct adv7482_link_config *config = &state->mipi_csi2_link[0];
	a->c.left = 0;
	a->c.top  = 0;
	if (config->input == DECODER_INPUT_COMPOSITE) {
		ret = __adv7482_status(state, NULL, &state->curr_norm);
		if (ret < 0)
			return ret;
		a->c.width  = 720;
		a->c.height = state->curr_norm & V4L2_STD_525_60
							 ? 480 : 576;
	} else {
		/* Get video information */
		ret = adv7482_get_vid_info(sd, &progressive,
						 &width, &height, NULL);
		if (ret < 0) {
			a->c.width  = ADV7482_MAX_WIDTH;
			a->c.height = ADV7482_MAX_HEIGHT;
		} else {
			a->c.width  = width;
			a->c.height = height;
		}
	}
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	return 0;
}

static int adv7482_set_power(struct adv7482_state *state, bool on)
{
	u8 val;
	int ret;
	struct adv7482_link_config *config = &state->mipi_csi2_link[0];
	
	if (config->input == DECODER_INPUT_COMPOSITE) {
		ret = adv7482_read_register(state->client, ADV7482_I2C_TXB,
				0x1E, &val);
		if (ret < 0)
			return ret;
		if (on && ((val & 0x40) == 0)) {
			/* Power up */
			ret = adv7482_write_registers(state->client,
					adv7482_power_up_txb_1lane);
			if (ret < 0)
				goto fail;
		} else {
			/* Power down */
			ret = adv7482_write_registers(state->client,
					adv7482_power_down_txb_1lane);
			if (ret < 0)
				goto fail;
		}
	} else {
		/* set active resolution */
		ret = adv7482_set_vid_info(&state->sd);
		ret = adv7482_read_register(state->client, ADV7482_I2C_TXA,
				0x1E, &val);
		if (ret < 0)
			return ret;
		if (1) {
			/* Power up */
			ret = adv7482_write_registers(state->client,
					adv7482_power_up_txa_4lane);
			if (ret < 0)
				goto fail;
		} else {
			/* Power down */
			ret = adv7482_write_registers(state->client,
					adv7482_power_down_txa_4lane);
			if (ret < 0)
				goto fail;
		}
	}
	return 0;
fail:
	pr_info("%s: Failed set power operation, ret = %d\n", __func__, ret);
	return ret;
}
static int adv7482_s_power(struct v4l2_subdev *sd, int on)
{
	struct adv7482_state *state = to_state(sd);
	int ret;
	ret = mutex_lock_interruptible(&state->mutex);
	if (ret)
		return ret;
	ret = adv7482_set_power(state, on);
	if (ret == 0)
		state->powered = on;
	mutex_unlock(&state->mutex);
	return ret;
}
static int adv7482_enum_framesizes(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_frame_size_enum *fse)
{
	
	struct adv7482_state *state = to_state(sd);
	fse->min_width=640;
	fse->max_width=1920;
	fse->min_height=480;
	fse->max_height=1080;
	return 0;
}
static int adv7482_get_pad_format(struct v4l2_subdev *sd,
				 
				   struct v4l2_subdev_state *cfg,
				
				  struct v4l2_subdev_format *format)
{
	struct adv7482_state *state = to_state(sd);

	printk("which=%x, code=%x, fmt=%x\n",format->which, format->format.code, format->format.colorspace);
	if ((format->which == V4L2_SUBDEV_FORMAT_TRY)&&cfg)
	{
	
		format->format = *v4l2_subdev_get_try_format(sd, cfg, 0);
		return 0;
		
	}
	else {
		adv7482_mbus_fmt(sd, &format->format);
		format->format.field = state->field;
	}
	printk("adv7482_get_pad_format--\n");
	return 0;
}
static int adv7482_set_pad_format(struct v4l2_subdev *sd,
				 
				   struct v4l2_subdev_state *cfg,
				
				  struct v4l2_subdev_format *format)
{
	struct adv7482_state *state = to_state(sd);
	struct v4l2_mbus_framefmt *framefmt;
	printk("adv7482_set_pad_format(%p,%p,%p)++\n",sd,cfg,format);
	printk("which=%x, code=%x, fmt=%x\n",format->which, format->format.code, format->format.colorspace);

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		framefmt = &format->format;
		if (state->field != format->format.field) {
			state->field = format->format.field;
			adv7482_set_power(state, false);
			adv7482_set_power(state, true);
		}
	} else {
		adv7482_mbus_fmt(sd, &format->format);
		if (format->format.field == V4L2_FIELD_ANY)
			format->format.field = state->field;
		return 0;
	}
	return adv7482_mbus_fmt(sd, framefmt);
}

static int adv7482_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct adv7482_state *state = to_state(sd);
	adv7482_set_power(state, enable);
	return 0;
}

/*
 * adv7482_g_mbus_config() - V4L2 decoder i/f handler for g_mbus_config
 * @sd: pointer to standard V4L2 sub-device structure
 * @cfg: pointer to V4L2 mbus_config structure
 *
 * Get mbus configuration.
 */
static int adv7482_g_mbus_config(struct v4l2_subdev *sd,
					struct v4l2_mbus_config *cfg)
{
	struct adv7482_state *state = to_state(sd);
	struct adv7482_link_config *config = &state->mipi_csi2_link[0];
	if (config->input == DECODER_INPUT_COMPOSITE) {
		cfg->bus.mipi_csi2.flags = V4L2_MBUS_CSI2_1_LANE |
				V4L2_MBUS_CSI2_CHANNEL_0 |
				V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	}
	else
	{
		cfg->bus.mipi_csi2.flags = V4L2_MBUS_CSI2_LANES |
				V4L2_MBUS_CSI2_CHANNELS |
				V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	}
	cfg->type = V4L2_MBUS_CSI1;
	return 0;
}
/*****************************************************************************/
/*  V4L2 decoder i/f handler for v4l2_ctrl_ops                               */
/*****************************************************************************/
static int adv7482_cp_s_ctrl(struct v4l2_ctrl *ctrl, struct i2c_client *client)
{
	u8 val;
	int ret;
	/* Enable video adjustment first */
	ret = adv7482_read_register(client, ADV7482_I2C_CP,
			ADV7482_CP_VID_ADJ_REG, &val);
	if (ret < 0)
		return ret;
	val |= ADV7482_CP_VID_ADJ_ENABLE;
	ret = adv7482_write_register(client, ADV7482_I2C_CP,
			ADV7482_CP_VID_ADJ_REG, val);
	if (ret < 0)
		return ret;
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		if ((ctrl->val < ADV7482_CP_BRI_MIN) ||
					(ctrl->val > ADV7482_CP_BRI_MAX))
			ret = -ERANGE;
		else
			ret = adv7482_write_register(client, ADV7482_I2C_CP,
					ADV7482_CP_BRI_REG, ctrl->val);
		break;
	case V4L2_CID_HUE:
		if ((ctrl->val < ADV7482_CP_HUE_MIN) ||
					(ctrl->val > ADV7482_CP_HUE_MAX))
			ret = -ERANGE;
		else
			ret = adv7482_write_register(client, ADV7482_I2C_CP,
					ADV7482_CP_HUE_REG, ctrl->val);
		break;
	case V4L2_CID_CONTRAST:
		if ((ctrl->val < ADV7482_CP_CON_MIN) ||
					(ctrl->val > ADV7482_CP_CON_MAX))
			ret = -ERANGE;
		else
			ret = adv7482_write_register(client, ADV7482_I2C_CP,
					ADV7482_CP_CON_REG, ctrl->val);
		break;
	case V4L2_CID_SATURATION:
		if ((ctrl->val < ADV7482_CP_SAT_MIN) ||
					(ctrl->val > ADV7482_CP_SAT_MAX))
			ret = -ERANGE;
		else
			ret = adv7482_write_register(client, ADV7482_I2C_CP,
					ADV7482_CP_SAT_REG, ctrl->val);
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}
static int adv7482_sdp_s_ctrl(struct v4l2_ctrl *ctrl, struct i2c_client *client)
{
	int ret;
	ret = adv7482_write_register(client, ADV7482_I2C_SDP,
			ADV7482_SDP_REG_CTRL, ADV7482_SDP_MAIN_MAP_RW);
	if (ret < 0)
		return ret;
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		if ((ctrl->val < ADV7482_SDP_BRI_MIN) ||
					(ctrl->val > ADV7482_SDP_BRI_MAX))
			ret = -ERANGE;
		else
			ret = adv7482_write_register(client, ADV7482_I2C_SDP,
					ADV7482_SDP_REG_BRI, ctrl->val);
		break;
	case V4L2_CID_HUE:
		if ((ctrl->val < ADV7482_SDP_HUE_MIN) ||
					(ctrl->val > ADV7482_SDP_HUE_MAX))
			ret = -ERANGE;
		else
			/*Hue is inverted according to HSL chart */
			ret = adv7482_write_register(client, ADV7482_I2C_SDP,
					ADV7482_SDP_REG_HUE, -ctrl->val);
		break;
	case V4L2_CID_CONTRAST:
		if ((ctrl->val < ADV7482_SDP_CON_MIN) ||
					(ctrl->val > ADV7482_SDP_CON_MAX))
			ret = -ERANGE;
		else
			ret = adv7482_write_register(client, ADV7482_I2C_SDP,
					ADV7482_SDP_REG_CON, ctrl->val);
		break;
	case V4L2_CID_SATURATION:
		/*
		 *This could be V4L2_CID_BLUE_BALANCE/V4L2_CID_RED_BALANCE
		 *Let's not confuse the user, everybody understands saturation
		 */
		if ((ctrl->val < ADV7482_SDP_SAT_MIN) ||
					(ctrl->val > ADV7482_SDP_SAT_MAX))
			ret = -ERANGE;
		else {
			ret = adv7482_write_register(client, ADV7482_I2C_SDP,
					ADV7482_SDP_REG_SD_SAT_CB, ctrl->val);
			if (ret < 0)
				break;
			ret = adv7482_write_register(client, ADV7482_I2C_SDP,
					ADV7482_SDP_REG_SD_SAT_CR, ctrl->val);
		}
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}
/*
 * adv7482_s_ctrl() - V4L2 decoder i/f handler for s_ctrl
 * @ctrl: pointer to standard V4L2 control structure
 *
 * Set a control in ADV7482 decoder device.
 */
static int adv7482_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_adv7482_sd(ctrl);
	struct adv7482_state *state = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = mutex_lock_interruptible(&state->mutex);
	struct adv7482_link_config *config = &state->mipi_csi2_link[0];
	if (ret)
		return ret;
	if (config->input == DECODER_INPUT_COMPOSITE)
		ret = adv7482_sdp_s_ctrl(ctrl, client);
	else
		ret = adv7482_cp_s_ctrl(ctrl, client);
	mutex_unlock(&state->mutex);
	return ret;
}

static int adv7482_isr(struct v4l2_subdev *sd, u32 status, bool *handled)
{
	struct adv7482_state *state = to_state(sd);
	int ret;
	u8 val;
	printk("adv7482_isr++\n");
	ret = adv7482_read_register(state->client, ADV7482_I2C_IO,
			0x8A, &val);
	if(val&2)
	{
		printk("New frequency\n");
	}
	if (handled)
		*handled = true;
	return ret;
}

static const struct v4l2_subdev_core_ops adv7482_core_ops = {
#ifdef FIXME
	.queryctrl = v4l2_subdev_queryctrl,
	.g_ctrl = v4l2_subdev_g_ctrl,
	.s_ctrl = v4l2_subdev_s_ctrl,
	.g_ext_ctrls = v4l2_subdev_g_ext_ctrls,
	.s_ext_ctrls = v4l2_subdev_s_ext_ctrls,
	.try_ext_ctrls = v4l2_subdev_try_ext_ctrls,
	.querymenu = v4l2_subdev_querymenu,
#endif
	.interrupt_service_routine = adv7482_isr,
	.s_power = adv7482_s_power,
};
static const struct v4l2_subdev_video_ops adv7482_video_ops = {
	.querystd	= adv7482_querystd,
	.s_dv_timings = adv7482_s_dv_timings,
	.g_dv_timings = adv7482_g_dv_timings,
	.query_dv_timings = adv7482_query_dv_timings,
	.g_input_status = adv7482_g_input_status,
	//.cropcap	= adv7482_cropcap,
	//.g_crop		= adv7482_g_crop,
	//.g_mbus_config	= adv7482_g_mbus_config, 
	.s_stream = adv7482_s_stream,
};
static const struct v4l2_subdev_pad_ops adv7482_pad_ops = {
	.enum_mbus_code = adv7482_enum_mbus_code,
	.set_fmt = adv7482_set_pad_format,
	.get_fmt = adv7482_get_pad_format,
	//.enum_frame_size = adv7482_enum_framesizes,
	//.dv_timings_cap = adv7482_dv_timings_cap,
	//.enum_dv_timings = adv7482_enum_dv_timings,
};
static const struct v4l2_subdev_ops adv7482_ops = {
	.core = &adv7482_core_ops,
	.video = &adv7482_video_ops,
	.pad = &adv7482_pad_ops,
};
static const struct v4l2_ctrl_ops adv7482_ctrl_ops = {
	.s_ctrl = adv7482_s_ctrl,
};
/*
 * adv7482_init_controls() - Init controls
 * @state: pointer to private state structure
 *
 * Init ADV7482 supported control handler.
 */
static int adv7482_cp_init_controls(struct adv7482_state *state)
{
	v4l2_ctrl_handler_init(&state->ctrl_hdl, 4);
	v4l2_ctrl_new_std(&state->ctrl_hdl, &adv7482_ctrl_ops,
			  V4L2_CID_BRIGHTNESS, ADV7482_CP_BRI_MIN,
			  ADV7482_CP_BRI_MAX, 1, ADV7482_CP_BRI_DEF);
	v4l2_ctrl_new_std(&state->ctrl_hdl, &adv7482_ctrl_ops,
			  V4L2_CID_CONTRAST, ADV7482_CP_CON_MIN,
			  ADV7482_CP_CON_MAX, 1, ADV7482_CP_CON_DEF);
	v4l2_ctrl_new_std(&state->ctrl_hdl, &adv7482_ctrl_ops,
			  V4L2_CID_SATURATION, ADV7482_CP_SAT_MIN,
			  ADV7482_CP_SAT_MAX, 1, ADV7482_CP_SAT_DEF);
	v4l2_ctrl_new_std(&state->ctrl_hdl, &adv7482_ctrl_ops,
			  V4L2_CID_HUE, ADV7482_CP_HUE_MIN,
			  ADV7482_CP_HUE_MAX, 1, ADV7482_CP_HUE_DEF);
	state->sd.ctrl_handler = &state->ctrl_hdl;
	if (state->ctrl_hdl.error) {
		int err = state->ctrl_hdl.error;
		v4l2_ctrl_handler_free(&state->ctrl_hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&state->ctrl_hdl);
	return 0;
}
static int adv7482_sdp_init_controls(struct adv7482_state *state)
{
	int err;
	v4l2_ctrl_handler_init(&state->ctrl_hdl, 4);
	v4l2_ctrl_new_std(&state->ctrl_hdl, &adv7482_ctrl_ops,
			  V4L2_CID_BRIGHTNESS, ADV7482_SDP_BRI_MIN,
			  ADV7482_SDP_BRI_MAX, 1, ADV7482_SDP_BRI_DEF);
	v4l2_ctrl_new_std(&state->ctrl_hdl, &adv7482_ctrl_ops,
			  V4L2_CID_CONTRAST, ADV7482_SDP_CON_MIN,
			  ADV7482_SDP_CON_MAX, 1, ADV7482_SDP_CON_DEF);
	v4l2_ctrl_new_std(&state->ctrl_hdl, &adv7482_ctrl_ops,
			  V4L2_CID_SATURATION, ADV7482_SDP_SAT_MIN,
			  ADV7482_SDP_SAT_MAX, 1, ADV7482_SDP_SAT_DEF);
	v4l2_ctrl_new_std(&state->ctrl_hdl, &adv7482_ctrl_ops,
			  V4L2_CID_HUE, ADV7482_SDP_HUE_MIN,
			  ADV7482_SDP_HUE_MAX, 1, ADV7482_SDP_HUE_DEF);
	state->sd.ctrl_handler = &state->ctrl_hdl;
	if (state->ctrl_hdl.error) {
		int err = state->ctrl_hdl.error;
		v4l2_ctrl_handler_free(&state->ctrl_hdl);
		return err;
	}
	err = v4l2_ctrl_handler_setup(&state->ctrl_hdl);
	pr_info("%s return %d\n",__FUNCTION__,err);
	return 0;
}
static void adv7482_exit_controls(struct adv7482_state *state)
{
	v4l2_ctrl_handler_free(&state->ctrl_hdl);
}
/*****************************************************************************/
/*  i2c driver interface handlers                                            */
/*****************************************************************************/
static int adv7482_parse_dt(struct device_node *np,
			    struct adv7482_link_config *config)
{


	 config->input_interface = DECODER_INPUT_INTERFACE_YCBCR422;
	 config->regs =
	 (struct adv7482_reg_value *)adv7482_init_txa_4lane;
	 config->power_up =
	 (struct adv7482_reg_value *)adv7482_power_up_txa_4lane;
	 config->power_down =
	 (struct adv7482_reg_value *)adv7482_power_down_txa_4lane;
	 config->init_controls =
	 (int (*)(void *))adv7482_cp_init_controls;

	config->hdmi_in = 1;
	config->sdp_in = 0;
	config->sw_reset = 1;
	config->vc_ch = 0;
	
	config->init_device    = NULL;
	config->s_power        = NULL;
	config->s_ctrl         = NULL;
	config->enum_mbus_code = NULL;
	config->set_pad_format = NULL;
	config->get_pad_format = NULL;
	config->s_std          = NULL;
	config->querystd       = NULL;
	config->g_input_status = NULL;
	config->s_routing      = NULL;
	config->g_mbus_config  = NULL;
	return 0;
}
static int adv7482_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return 0;
}

static const struct v4l2_subdev_internal_ops adv7482_subdev_internal_ops = {
	.open = adv7482_open,
};

static const struct media_entity_operations adv7482_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int adv7482_dai_set_sysclk(struct snd_soc_dai *dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct adv7482_state *state = dai_to_state(dai);
	if (clk_id != 0 || freq != clk_get_rate(state->mclk_hw->clk)) {
		dev_err(state->dev, "invalid clock (%d) or frequency (%u, dir %d)\n",
			clk_id, freq, dir);
		return -EINVAL;
	}
	state->freq = freq;
	return 0;
}

#define ADV748X_I2SOUTMODE_I2S 0
#define ADV748X_I2SOUTMODE_RIGHT_J 1
#define ADV748X_I2SOUTMODE_LEFT_J 2
#define ADV748X_I2SOUTMODE_SPDIF 3

static int adv7482_dai_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	int ret;
	struct adv7482_state *state = dai_to_state(dai);
	if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBM_CFM) {
		dev_err(dai->dev, "only I2S master clock mode supported\n");
		ret = -EINVAL;
		goto done;
	}
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		state->tdm = 0;
		state->fmt = ADV748X_I2SOUTMODE_I2S;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		state->tdm = 1;
		state->fmt = ADV748X_I2SOUTMODE_RIGHT_J;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		state->tdm = 1;
		state->fmt = ADV748X_I2SOUTMODE_LEFT_J;
		break;
	default:
		dev_err(dai->dev, "only i2s, left_j and right_j supported\n");
		ret = -EINVAL;
		goto done;
	}
	if ((fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF) {
		dev_err(dai->dev, "only normal bit clock + frame supported\n");
		ret = -EINVAL;
	}
done:
	return ret;
}

static int adv7482_dai_startup(struct snd_pcm_substream *sub, struct snd_soc_dai *dai)
{
	u8 val;
	int ret;
	struct adv7482_state *state = dai_to_state(dai);

	ret = adv7482_read_register(state->client, ADV7482_I2C_IO,
			0x0E, &val);
	if (ret < 0)
		return ret;
	val &= ~0x22;
	ret = adv7482_write_register(state->client, ADV7482_I2C_IO,
			0x0E, val);
	return 0;
}

static int adv7482_dai_hw_params(struct snd_pcm_substream *sub,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	int ret;
	struct adv7482_state *state = dai_to_state(dai);
	uint fs = state->freq / params_rate(params);
	u8 val;

	dev_dbg(dai->dev, "dai %s substream %s rate=%u (fs=%u), channels=%u sample width=%u(%u)\n",
		dai->name, sub->name,
		params_rate(params), fs,
		params_channels(params),
		params_width(params),
		params_physical_width(params));
	switch (fs) {
	case 128:
	case 256:
	case 384:
	case 512:
	case 640:
	case 768:
		break;
	default:
		ret = -EINVAL;
		dev_err(dai->dev, "invalid clock frequency (%u) or rate (%u)\n",
			state->freq, params_rate(params));
		goto done;
	}
	ret=adv7482_write_register(state->client, ADV7482_I2C_DPLL,0xB5,(fs / 128) - 1);
	ret=adv7482_write_register(state->client, ADV7482_I2C_HDMI, 0x0f, state->tdm?0xc0:0x00);
	ret=adv7482_write_register(state->client, ADV7482_I2C_HDMI, 0x6D, state->tdm?0x80:0x00);
	ret=adv7482_write_register(state->client, ADV7482_I2C_HDMI, 0x03, (state->fmt<<5)|params_width(params));
	done:
	return ret;
}

static int adv7482_dai_mute_stream(struct snd_soc_dai *dai, int mute, int dir)
{
	u8 val;
	int ret;
	struct adv7482_state *state = dai_to_state(dai);

	ret = adv7482_read_register(state->client, ADV7482_I2C_HDMI,
			0x1A, &val);
	if (ret < 0)
		return ret;
	if(mute)
		val |= 0x10;
	else
		val &= ~0x10;
	ret = adv7482_write_register(state->client, ADV7482_I2C_HDMI,
			0x1A, val);
	return ret;
}

static void adv7482_dai_shutdown(struct snd_pcm_substream *sub, struct snd_soc_dai *dai)
{
	u8 val;
	int ret;
	struct adv7482_state *state = dai_to_state(dai);

	ret = adv7482_read_register(state->client, ADV7482_I2C_IO,
			0x0E, &val);
	if (ret < 0)
		return ;
	val |= 0x22;
	ret = adv7482_write_register(state->client, ADV7482_I2C_IO,
			0x0E, val);

}


static const struct snd_soc_dai_ops adv7482_dai_ops = {
	.set_sysclk = adv7482_dai_set_sysclk,
	.set_fmt = adv7482_dai_set_fmt,
	.startup = adv7482_dai_startup,
	.hw_params = adv7482_dai_hw_params,
	.mute_stream = adv7482_dai_mute_stream,
	.shutdown = adv7482_dai_shutdown,
};

static const char ADV748X_DAI_NAME[] = "adv7482-i2s";

static	int adv748x_of_xlate_dai_name(struct snd_soc_component *component,
				      const struct of_phandle_args *args,
				      const char **dai_name)
{
	if (dai_name)
		*dai_name = ADV748X_DAI_NAME;
	return 0;
}

static const struct snd_soc_component_driver adv7482_codec = {
	.of_xlate_dai_name = adv748x_of_xlate_dai_name,
};

int adv7482_dai_init(struct adv7482_state *state)
{
	int ret;
	state->mclk_name = kasprintf(GFP_KERNEL, "%s.%s-i2s-mclk",
				   state->dev->driver->name,
				   dev_name(state->dev));
	if (!state->mclk_name) {
		ret = -ENOMEM;
		printk("No memory for MCLK\n");
		goto fail;
	}
	state->mclk_hw = clk_hw_register_fixed_rate(state->dev, state->mclk_name,
						  NULL, 0, 12288000);
	if (IS_ERR(state->mclk_hw)) {
		ret = PTR_ERR(state->mclk_hw);
		printk("Failed to register MCLK (%d)\n", ret);
		goto fail;
	}
	ret = of_clk_add_hw_provider(state->dev->of_node, of_clk_hw_simple_get,
				     state->mclk_hw->clk);
	if (ret < 0) {
		printk("Failed to add MCLK provider (%d)\n", ret);
		goto unreg_mclk;
	}
	state->dai_drv.name = ADV748X_DAI_NAME;
	state->dai_drv.ops = &adv7482_dai_ops;
	state->dai_drv.capture = (const struct snd_soc_pcm_stream){
		.stream_name	= "Capture",
		.channels_min	= 8,
		.channels_max	= 8,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_U24_LE,
	};

	ret = devm_snd_soc_register_component(state->dev, &adv7482_codec,
					      &state->dai_drv, 1);
	if (ret < 0) {
		printk("Failed to register the codec (%d)\n", ret);
		goto cleanup_mclk;
	}
	return 0;
cleanup_mclk:
	of_clk_del_provider(state->dev->of_node);
unreg_mclk:
	clk_hw_unregister_fixed_rate(state->mclk_hw);
fail:
	return ret;
}

void adv7482_dai_cleanup(struct adv7482_state *state)
{
	of_clk_del_provider(state->dev->of_node);
	clk_hw_unregister_fixed_rate(state->mclk_hw);
	kfree(state->mclk_name);
}

/*
 * adv7482_probe - Probe a ADV7482 device
 * @client: pointer to i2c_client structure
 * @id: pointer to i2c_device_id structure
 *
 * Initialize the ADV7482 device
 */
static int adv7482_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct adv7482_state *state;
	struct adv7482_link_config link_config;
	struct device *dev = &client->dev;
	struct v4l2_subdev *sd;
	int ret;
	/* Check if the adapter supports the needed features */
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;
	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);
	state = kzalloc(sizeof(struct adv7482_state), GFP_KERNEL);
	if (state == NULL) {
		ret = -ENOMEM;
		goto err;
	}
	state->client = client;
	state->irq = client->irq;
	ret = adv7482_parse_dt(dev->of_node, &link_config);
	if (ret) {
		dev_err(&client->dev, "adv7482 parse error\n");
		return ret;
	}else{
		dev_err(&client->dev, "adv7482 parse success\n");
	}
	state->mipi_csi2_link[0].input_interface = link_config.input_interface;
	state->mipi_csi2_link[0].input = DECODER_INPUT_HDMI;
	printk("input_interface=%x\n",link_config.input_interface);
	
	mutex_init(&state->mutex);
	state->autodetect = true;
	state->powered = true;
	sd = &state->sd;
	state->width = ADV7482_MAX_WIDTH;
	state->height = ADV7482_MAX_HEIGHT;
	state->field = V4L2_FIELD_NONE;
	state->field = V4L2_FIELD_NONE;
	if(sd->name)
		printk("sd->name=%s\n",sd->name);
	else
	{
		printk("No name!");
		strcpy(sd->name,"adv748[12]");
	}

	state->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	state->sd.grp_id = 678;
	state->dev		= dev;


	v4l2_i2c_subdev_init(&state->sd, client, &adv7482_ops);
	i2c_set_clientdata(client,state);

	state->mipi_csi2_link[0].dev = dev;
	/* SW reset ADV7482 to its default values */
	if (link_config.sw_reset) {
		ret = adv7482_write_registers(client, adv7482_sw_reset);
		if (ret < 0){
			dev_err(&client->dev, "adv7482_write_registers failed\n");
			goto err_unreg_subdev;
		}else{
			dev_err(&client->dev, "adv7482_write_registers success\n");
		}
		/* check rd_info */
		{
			u8 msb;
			u8 lsb;
			ret = adv7482_read_register(client, ADV7482_I2C_IO,
					ADV7482_IO_RD_INFO1_REG, &lsb);
			if (ret < 0){
				dev_err(&client->dev, "adv7482_read_register ADV7482_I2C_IO failed\n");
				return ret;
			}else{
				dev_err(&client->dev, "adv7482_read_register ADV7482_I2C_IO success\n");
			}
			ret = adv7482_read_register(client, ADV7482_I2C_IO,
					ADV7482_IO_RD_INFO2_REG, &msb);
			if (ret < 0)
				return ret;
			v4l_info(client, "adv7482 revision is %02x%02x\n",
					lsb, msb);
		}
	}
	if (link_config.hdmi_in) {
		ret = adv7482_write_registers(client,
				adv7482_init_txa_4lane);
		if (ret < 0)
			goto err_unreg_subdev;
		/* Power down */
		ret = adv7482_write_registers(client,
				adv7482_power_down_txa_4lane);
		if (ret < 0)
			goto err_unreg_subdev;
	} else
		v4l_info(client, "adv7482 hdmi_in is disabled.\n");
	/* Initializes ADV7482 to its default values */
	if (link_config.sdp_in) {
		ret = adv7482_write_registers(client,
						adv7482_init_txb_1lane);
		if (ret < 0)
			goto err_unreg_subdev;
		/* Power down */
		ret = adv7482_write_registers(client,
						adv7482_power_down_txb_1lane);
		if (ret < 0)
			goto err_unreg_subdev;
	} else
		v4l_info(client, "adv7482 sdp_in is disabled.\n");
	if (link_config.sdp_in && link_config.hdmi_in) { 
		/* Power up hdmi rx */
		ret = adv7482_write_registers(client,
						adv7482_power_up_hdmi_rx);
		if (ret < 0)
			goto err_unreg_subdev;
		/* Enable csi4 and sci1 */
		ret = adv7482_write_registers(client,
						adv7482_enable_csi4_csi1);
		if (ret < 0)
			goto err_unreg_subdev;
		v4l_info(client, "adv7482 enable csi1 and csi4\n");
	}

	/* Setting virtual channel for ADV7482 */
	if (link_config.vc_ch == 0)
		ret = adv7482_write_registers(client,
					adv7482_set_virtual_channel0);
	else if (link_config.vc_ch == 1)
		ret = adv7482_write_registers(client,
					adv7482_set_virtual_channel1);
	else if (link_config.vc_ch == 2)
		ret = adv7482_write_registers(client,
					adv7482_set_virtual_channel2);
	else if (link_config.vc_ch == 3)
		ret = adv7482_write_registers(client,
					adv7482_set_virtual_channel3);
	dev_err(&client->dev, "adv7482_write_registers %d chanels return %d\n",
		link_config.vc_ch,ret);
	if (ret < 0)
		goto err_unreg_subdev;
	state->pad.flags = MEDIA_PAD_FL_SOURCE;
	
	state->sd.entity.function  = MEDIA_ENT_F_VID_IF_BRIDGE;
	
	ret = media_entity_pads_init(&state->sd.entity, 1, &state->pad);
	
	state->sd.dev = &client->dev;
	dev_err(&client->dev, "media_entity_init return %d\n",ret);
	if (ret)
		goto err_free_ctrl;
	
	sd->dev = &client->dev;

	ret = v4l2_async_register_subdev(&state->sd);
	dev_err(&client->dev, "v4l2_async_register_subdev return %d\n",ret);
	if (ret)
		goto err_free_ctrl;
	dev_err(&client->dev, "Drivers added success\n");
	ret=adv7482_dai_init(state);
	if (ret)
		goto err_free_ctrl;


	return 0;
err_free_ctrl:
	adv7482_exit_controls(state);
err_unreg_subdev:
	mutex_destroy(&state->mutex);
	v4l2_device_unregister_subdev(&state->sd);
	kfree(state);
err:
	dev_err(&client->dev, ": Failed to probe: %d\n", ret);
	return ret;
}
/*
 * adv7482_remove - Remove ADV7482 device support
 * @client: pointer to i2c_client structure
 *
 * Reset the ADV7482 device
 */
static void adv7482_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct adv7482_state *state = to_state(sd);
	adv7482_dai_cleanup(state);
	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	adv7482_exit_controls(state);
	mutex_destroy(&state->mutex);
	kfree(to_state(sd));
	
}
static const struct i2c_device_id adv7482_id[] = {
	{DRIVER_NAME, 0},
	{},
};
#ifdef CONFIG_PM_SLEEP
/*
 * adv7482_suspend - Suspend ADV7482 device
 * @client: pointer to i2c_client structure
 * @state: power management state
 *
 * Power down the ADV7482 device
 */
static int adv7482_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret;
	ret = adv7482_write_register(client, ADV7482_I2C_IO,
				ADV7482_IO_PWR_MAN_REG, ADV7482_IO_PWR_OFF);
	return ret;
}
/*
 * adv7482_resume - Resume ADV7482 device
 * @client: pointer to i2c_client structure
 *
 * Power on and initialize the ADV7482 device
 */
static int adv7482_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct adv7482_link_config link_config;
	int ret;
	ret = adv7482_write_register(client, ADV7482_I2C_IO,
				ADV7482_IO_PWR_MAN_REG, ADV7482_IO_PWR_ON);
	if (ret < 0)
		return ret;
	ret = adv7482_parse_dt(dev->of_node, &link_config);
	if (ret)
		return ret;
	/* Initializes ADV7482 to its default values */
	ret = adv7482_write_registers(client, link_config.regs);
	/* Power down */
	ret = adv7482_write_registers(client, link_config.power_down);
	if (link_config.sdp_in && link_config.hdmi_in) {
		/* Power up hdmi rx */
		ret = adv7482_write_registers(client,
						adv7482_power_up_hdmi_rx);
		if (ret < 0)
			return ret;
		/* Enable csi4 and sci1 */
		ret = adv7482_write_registers(client,
						adv7482_enable_csi4_csi1);
		if (ret < 0)
			return ret;
	}
	return ret;
}
static SIMPLE_DEV_PM_OPS(adv7482_pm_ops, adv7482_suspend, adv7482_resume);
#define ADV7482_PM_OPS (&adv7482_pm_ops)
#else
#define ADV7482_PM_OPS NULL
#endif
MODULE_DEVICE_TABLE(i2c, adv7482_id);
static const struct of_device_id adv7482_of_ids[] = {
	{ .compatible = "adi,adv7481", },
	{}
};
MODULE_DEVICE_TABLE(of, adv7482_of_ids);
static struct i2c_driver adv7482_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		// .pm = ADV7482_PM_OPS,
		.of_match_table = adv7482_of_ids,
	},
	.probe		= adv7482_probe,
	.remove		= adv7482_remove,
	.id_table	= adv7482_id,
};
module_i2c_driver(adv7482_driver);
MODULE_DESCRIPTION("HDMI Receiver ADV7482 video decoder driver");
MODULE_ALIAS("platform:adv7482");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("rajeshwar.shinde@acclivistechnologies.com");