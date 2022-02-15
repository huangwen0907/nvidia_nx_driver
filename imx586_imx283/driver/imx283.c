/*
 * imx283.c - imx283 sensor driver
 *
 * Copyright (c) 2016-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>
#include "imx283_mode_tbls.h"




#define IMX283_MIN_FRAME_LENGTH	(1125)
#define IMX283_MAX_FRAME_LENGTH	(0x1FFFF)

#define IMX283_FRAME_LENGTH_ADDR_MSB		0x303A
#define IMX283_FRAME_LENGTH_ADDR_MID		0x3039
#define IMX283_FRAME_LENGTH_ADDR_LSB		0x3038

#define IMX283_COARSE_TIME_SHS1_ADDR_MSB	0x3022
#define IMX283_COARSE_TIME_SHS1_ADDR_MID	0x3021
#define IMX283_COARSE_TIME_SHS1_ADDR_LSB		0x3020

#define IMX283_SVR_ADDR_MSB		0x300A
#define IMX283_SVR_ADDR_LSB		0x3009

#define IMX283_SHR_ADDR_MSB			0x303c
#define IMX283_SHR_ADDR_LSB			0x303b



#define IMX283_SMD_ADDR		0x3008

#define IMX283_SHUTTER_ROLLING_VALUE		0
#define IMX283_SHUTTER_GLOBAL_VALUE			1


#define IMX283_FUSE_ID_ADDR	0x3382
#define IMX283_FUSE_ID_SIZE		6
#define IMX283_FUSE_ID_STR_SIZE		(IMX283_FUSE_ID_SIZE * 2)




// Analog gain from 0db to 27db
//(0d to 1957d) 0h to 7A5h
//PGC [7:0] 3042h [7:0] 
//PGC [10:8] 3043h [2:0]
//Gain [dB] = -20log{(2048 - PGC [10:0]) /2048}
#define IMX283_ANA_GAIN_GLOBAL_ADDR_MSB		0x3043 //bit[2:0]
#define IMX283_ANA_GAIN_GLOBAL_ADDR_LSB		0x3042 //bit[7:0]

#define IMX283_DIGITAL_GAIN_ADDR							0x3044


#define IMX283_ANA_GAIN_MAX_VALUE			1957
#define IMX283_ANA_GAIN_MAX_DB				27


#define IMX283_GAIN_FACTOR		1000000
#define IMX283_MIN_GAIN			(1 * IMX283_GAIN_FACTOR)
#define IMX283_MAX_ANALOG_GAIN	(IMX283_MIN_GAIN * 111 / 5)
#define IMX283_MAX_DIGITAL_GAIN	64
#define IMX283_MAX_GAIN	(IMX283_MAX_ANALOG_GAIN * IMX283_MAX_DIGITAL_GAIN)



//REGHOLD
#define IMX283_GROUP_HOLD_ADDR				0x303f

//Vertical Direction Readout Inversion
#define IMX283_VERTICAL_DIRECTION_INVERSION_ADDR		0x300b


//Start position of vertical arbitrary cropping
#define IMX283_VERTICAL_CROP_WINPOS_ADDR_MSB		0x3010 //bit[3:0]
#define IMX283_VERTICAL_CROP_WINPOS_ADDR_LSB		0x300F //bit[7:0]

//Width of vertical arbitrary cropping
#define IMX283_VERTICAL_CROP_WIDTH_ADDR_MSB	0x3012 //bit[2:0]
#define IMX283_VERTICAL_CROP_WIDTH_ADDR_LSB		0x3011 //bit[7:0]


//Enable of Horizontal Arbitrary Cropping
#define IMX283_HORIZONTAL_CROP_EN_ADDR		0x300b //bit[4]

//Start position of Horizontal arbitrary cropping
#define IMX283_HORIZONTAL_CROP_START_ADDR_MSB	0x3059 //bit[4:0]
#define IMX283_HORIZONTAL_CROP_START_ADDR_LSB		0x3058 //bit[7:0]

//End of vertical Horizontal cropping
#define IMX283_VERTICAL_CROP_END_ADDR_MSB		0x305b //bit[2:0]
#define IMX283_VERTICAL_CROP_END_ADDR_LSB		0x305a //bit[7:0]

//horizontal drive period length
#define IMX283_HMAX_ADDR_MSB		0x3037 //bit[7:0]
#define IMX283_HMAX_ADDR_LSB			0x3036 //bit[7:0]

//vertical drive period length
#define IMX283_VMAX_ADDR_MSB		0x303a //bit[3:0]
#define IMX283_VMAX_ADDR_MID		0x3039 //bit[7:0]
#define IMX283_VMAX_ADDR_LSB			0x3038 //bit[7:0]


#define IMX283_SENSOR_INTERNAL_CLK_FREQ	72000000

#define IMX283_5472X3648_10BIT_FULL_HMAX				745
#define IMX283_5472X3648_10BIT_FULL_MIN_VMAX	3793
#define IMX283_5472X3648_10BIT_FULL_OFFSET			157


#define IMX283_5472X3078_10BIT_FULL_HMAX				745
#define IMX283_5472X3078_10BIT_FULL_MIN_VMAX	3203
#define IMX283_5472X3078_10BIT_FULL_OFFSET			157


#define IMX283_3000X3000_10BIT_FULL_HMAX				544
#define IMX283_3000X3000_10BIT_FULL_MIN_VMAX	3081
#define IMX283_3000X3000_10BIT_FULL_OFFSET			157


#define IMX283_2376X1538_10BIT_FULL_HMAX				364
#define IMX283_2376X1538_10BIT_FULL_MIN_VMAX	3296
#define IMX283_2376X1538_10BIT_FULL_OFFSET			157


#define IMX283_3840X2160_10BIT_CROP_HMAX				544
#define IMX283_3840X2160_10BIT_CROP_MIN_VMAX	2200
#define IMX283_3840X2160_10BIT_CROP_OFFSET			157



static const struct of_device_id imx283_of_match[] = {
	{ .compatible = "nvidia,imx283",},
	{ },
};
MODULE_DEVICE_TABLE(of, imx283_of_match);

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_FUSE_ID,
	TEGRA_CAMERA_CID_HDR_EN,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};

struct imx283 {
	struct i2c_client	*i2c_client;
	struct v4l2_subdev	*subdev;
	u8 fuse_id[IMX283_FUSE_ID_SIZE];
	u32 frame_length;
	u32 frame_rate;
	u32 vmax;
	s64 last_wdr_et_val;
	struct camera_common_data	*s_data;
	struct tegracam_device		*tc_dev;
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.use_single_rw = true,
};


static inline void imx283_get_frame_length_regs(imx283_reg *regs,
				u32 frame_length)
{
	regs->addr = IMX283_FRAME_LENGTH_ADDR_MSB;
	regs->val = (frame_length >> 16) & 0x0f;

	(regs + 1)->addr = IMX283_FRAME_LENGTH_ADDR_MID;
	(regs + 1)->val = (frame_length >> 8) & 0xff;

	(regs + 2)->addr = IMX283_FRAME_LENGTH_ADDR_LSB;
	(regs + 2)->val = (frame_length) & 0xff;
}

static inline void imx283_get_coarse_time_regs_shs1(imx283_reg *regs,
				u32 coarse_time)
{
	regs->addr = IMX283_COARSE_TIME_SHS1_ADDR_MSB;
	regs->val = (coarse_time >> 16) & 0x01;

	(regs + 1)->addr = IMX283_COARSE_TIME_SHS1_ADDR_MID;
	(regs + 1)->val = (coarse_time >> 8) & 0xff;

	(regs + 2)->addr = IMX283_COARSE_TIME_SHS1_ADDR_LSB;
	(regs + 2)->val = (coarse_time) & 0xff;

}

static inline void imx283_get_gain_reg(imx283_reg *regs,
				u8 gain)
{
#if 0
	regs->addr = IMX283_GAIN_ADDR;
	regs->val = (gain) & 0xff;
#endif

	//printk("%s: anlog gain:%d\n",__func__, gain);

	regs->addr = IMX283_ANA_GAIN_GLOBAL_ADDR_MSB;
	regs->val = ((gain>>8) & 0x03);	//regs->val = (gain>>8) & 0xff;

	(regs + 1)->addr = IMX283_ANA_GAIN_GLOBAL_ADDR_LSB;
	(regs + 1)->val = (gain & 0xff);
}

static int test_mode;
module_param(test_mode, int, 0644);

static inline int imx283_read_reg(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xFF;

	return err;
}

static int imx283_write_reg(struct camera_common_data *s_data,
				u16 addr, u8 val)
{
	int err;
	struct device *dev = s_data->dev;

	err = regmap_write(s_data->regmap, addr, val);
	if (err)
		dev_err(dev, "%s: i2c write failed, 0x%x = %x\n",
			__func__, addr, val);

	return err;
}

static int imx283_write_table(struct imx283 *priv,
				const imx283_reg table[])
{
	struct camera_common_data *s_data = priv->s_data;

	return regmap_util_write_table_8(s_data->regmap,
					 table,
					 NULL, 0,
					 IMX283_TABLE_WAIT_MS,
					 IMX283_TABLE_END);
}

static inline u32 imx283_get_svr_regs(struct camera_common_data *s_data, u16 *val)
{
	int err = 0;
	u32 reg_val1 = 0, reg_val2 = 0;
	u16 addr = 0;

	addr = IMX283_SVR_ADDR_LSB;
	err = regmap_read(s_data->regmap, addr, &reg_val1);

	addr = IMX283_SVR_ADDR_MSB;
	err = regmap_read(s_data->regmap, addr, &reg_val2);
	
	*val = (((reg_val2 & 0xFF) << 8) | (reg_val1 & 0xFF));

	return err;
}

static inline void imx283_get_shr_regs(imx283_reg *regs, u16 shr)
{
	regs->addr = IMX283_SHR_ADDR_MSB;
	regs->val = (shr >> 8) & 0xff;
	(regs + 1)->addr = IMX283_SHR_ADDR_LSB;
	(regs + 1)->val = (shr) & 0xff;
}

static inline void imx283_get_vmax_regs(imx283_reg *regs,
				u32 vmax)
{
	regs->addr = IMX283_VMAX_ADDR_MSB;
	regs->val = (vmax >> 16) & 0x0f;
	
	(regs + 1)->addr = IMX283_VMAX_ADDR_MID;
	(regs + 1)->val = (vmax >> 8) & 0xff;
	
	(regs + 2)->addr = IMX283_VMAX_ADDR_LSB;
	(regs + 2)->val = (vmax) & 0xff;
}

static inline u32 imx283_set_smd_regs(struct camera_common_data *s_data, u16 val)
{
	int err = 0;

	err = imx283_write_reg(s_data, IMX283_SMD_ADDR, val);

	return err;
}
//4550 - 
static int imx283_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	int err;

	err = imx283_write_reg(s_data,
				IMX283_GROUP_HOLD_ADDR, val);
	if (err) {
		dev_dbg(dev,
			"%s: Group hold control error\n", __func__);
		return err;
	}

	return 0;
}

static int imx283_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx283 *priv = (struct imx283 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];
	imx283_reg reg_list[2];
	int err;
	int i = 0;
	u32 again;
	u8 dgain;
	u16 reg_again;
	u8 reg_dgain;

	dev_dbg(dev, "%s: val: %lld\n", __func__, val);

	if (val < IMX283_MIN_GAIN)
		val = IMX283_MIN_GAIN;
	else if (val > IMX283_MAX_GAIN)
		val = IMX283_MAX_GAIN;

	if  (val > (IMX283_MAX_ANALOG_GAIN * 32)) {
		dgain = 64;
		reg_dgain = 0x03;
	} else if  (val > (IMX283_MAX_ANALOG_GAIN * 16)) {
		dgain = 32;
		reg_dgain = 0x03;
	} else if  (val > (IMX283_MAX_ANALOG_GAIN * 8)) {
		dgain = 16;
		reg_dgain = 0x03;
	} else if  (val > (IMX283_MAX_ANALOG_GAIN * 4)) {
		dgain = 8;
		reg_dgain = 0x03;
	} else if  (val > (IMX283_MAX_ANALOG_GAIN * 2)) {
		dgain = 4;
		reg_dgain = 0x02;
	} else if (val > (IMX283_MAX_ANALOG_GAIN)) {
		dgain = 2;
		reg_dgain = 0x01;
	} else  {
		dgain = 1;
		reg_dgain = 0x00;
	}

	reg_again = 2048 - (2048 * dgain * mode->control_properties.gain_factor / val);
	if (reg_again > IMX283_ANA_GAIN_MAX_VALUE)
		reg_again = IMX283_ANA_GAIN_MAX_VALUE;
	
	again = val / (dgain * mode->control_properties.gain_factor);

	imx283_get_gain_reg(reg_list, reg_again);

	dev_dbg(dev, "%s: val:%lld, gain:%lld, again:(%d, %d), dgain:(%d, %d)\n",
			__func__,
			val,
			val / IMX283_MIN_GAIN,
			again,
			reg_again,
			dgain,
			reg_dgain);

	/* writing analog gain */
	for (i = 0; i < 2; i++) {
		err = imx283_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	/* writing digital gain */
	err = imx283_write_reg(priv->s_data, IMX283_DIGITAL_GAIN_ADDR,
				reg_dgain);
	if (err)
		goto fail;

	return 0;

fail:
	dev_err(dev, "%s: GAIN control error\n", __func__);
	return err;
}

static int imx283_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx283 *priv = (struct imx283 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	imx283_reg reg_list[3];
	int err;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	int i = 0;
	u16 svr = 0, hmax = 0, vmax_min = 0;
	u64 freq = IMX283_SENSOR_INTERNAL_CLK_FREQ;

	dev_dbg(dev, "%s: val: %lld\n", __func__, val);

	imx283_get_svr_regs(s_data, &svr);
	priv->frame_rate = 72000000/(priv->frame_length * mode->image_properties.line_length * (svr + 1));

	if (s_data->mode == IMX283_MODE_5472X3648_10BIT_FULL_25FPS)
	{
		hmax = IMX283_5472X3648_10BIT_FULL_HMAX;
		vmax_min = IMX283_5472X3648_10BIT_FULL_MIN_VMAX;
	}
	else if (s_data->mode == IMX283_MODE_5472X3078_10BIT_FULL_30FPS)
	{
		hmax = IMX283_5472X3078_10BIT_FULL_HMAX;
		vmax_min = IMX283_5472X3078_10BIT_FULL_MIN_VMAX;
	}
	else if (s_data->mode == IMX283_MODE_3000X3000_10BIT_FULL_42FPS)
	{
		hmax = IMX283_3000X3000_10BIT_FULL_HMAX;
		vmax_min = IMX283_3000X3000_10BIT_FULL_MIN_VMAX;
	}
	else if (s_data->mode == IMX283_MODE_2376X1538_10BIT_FULL_60FPS)
	{
		hmax = IMX283_2376X1538_10BIT_FULL_HMAX;
		vmax_min = IMX283_2376X1538_10BIT_FULL_MIN_VMAX;
	}
	else if (s_data->mode == IMX283_MODE_3840X2160_10BIT_CROP_60FPS)
	{
		hmax = IMX283_3840X2160_10BIT_CROP_HMAX;
		vmax_min = IMX283_3840X2160_10BIT_CROP_MIN_VMAX;
	}

	priv->vmax = (u32)(freq *mode->control_properties.framerate_factor /(val *	hmax));
	
	if (priv->vmax < vmax_min)
		priv->vmax = vmax_min;

	imx283_get_vmax_regs(reg_list, priv->vmax);
	
	for (i = 0; i < 3; i++) {
		err = imx283_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	dev_dbg(dev, "%s: PCLK:%lld, LL:%d, fps:%lld, VMAX:%d\n", __func__,
			mode->signal_properties.pixel_clock.val,
			mode->image_properties.line_length,
			val / mode->control_properties.framerate_factor,
			priv->vmax);
	
	return 0;

fail:
	dev_dbg(dev, "%s: FRAME_LENGTH control error\n", __func__);
	return err;
}


static u16 imx283_calculate_exposure_shr(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx283 *priv = (struct imx283 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];
	u16 svr;
	u16 shr;
	u64 freq = IMX283_SENSOR_INTERNAL_CLK_FREQ;
	u16 offset = 0, hmax = 0, shr_min = 0;

	dev_dbg(dev, "%s: val: %lld\n", __func__, val);

	imx283_get_svr_regs(s_data, &svr);

	if (s_data->mode == IMX283_MODE_5472X3648_10BIT_FULL_25FPS)
	{
		offset = IMX283_5472X3648_10BIT_FULL_OFFSET;
		hmax = IMX283_5472X3648_10BIT_FULL_HMAX;

		shr_min = 10;

	}
	else if (s_data->mode == IMX283_MODE_5472X3078_10BIT_FULL_30FPS)
	{
		offset = IMX283_5472X3078_10BIT_FULL_OFFSET;
		hmax = IMX283_5472X3078_10BIT_FULL_HMAX;

		shr_min = 10;
	}
	else if (s_data->mode == IMX283_MODE_3000X3000_10BIT_FULL_42FPS)
	{
		offset = IMX283_3000X3000_10BIT_FULL_OFFSET;
		hmax = IMX283_3000X3000_10BIT_FULL_HMAX;

		shr_min = 10;
	}
	else if (s_data->mode == IMX283_MODE_2376X1538_10BIT_FULL_60FPS)
	{
		offset = IMX283_2376X1538_10BIT_FULL_OFFSET;
		hmax = IMX283_2376X1538_10BIT_FULL_HMAX;

		shr_min = 10;
	}
	else if (s_data->mode == IMX283_MODE_3840X2160_10BIT_CROP_60FPS)
	{
		offset = IMX283_3840X2160_10BIT_CROP_OFFSET;
		hmax = IMX283_3840X2160_10BIT_CROP_HMAX;

		shr_min = 12;
	}
	
	shr = priv->vmax - (u32) (val  * freq /mode->control_properties.exposure_factor - offset) /hmax;

	if (shr > priv->vmax - 4)
		shr = priv->vmax - 4;
	
	if (shr < shr_min)
		shr = shr_min;

	dev_dbg(dev, "%s: shr: %u vmax: %d\n", __func__, shr, priv->vmax);
	
	return shr;
}

static int imx283_set_exposure_shr(struct tegracam_device *tc_dev, s64 val)
{
	struct imx283 *priv = (struct imx283 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	imx283_reg reg_list[2];
	int err;
	u16 shr;
	int i = 0;

	dev_dbg(dev, "%s: val: %lld\n", __func__, val);

	shr = imx283_calculate_exposure_shr(tc_dev, val);

	imx283_get_shr_regs(reg_list, shr);

	for (i = 0; i < 2; i++) {
		err = imx283_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_err(dev, "%s: Exposure control error\n", __func__);
	return err;
}

static int imx283_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	struct device *dev = tc_dev->dev;
	int err;

	dev_dbg(dev, "%s: val: %lld\n", __func__, val);

	err = imx283_set_exposure_shr(tc_dev, val);
	if (err)
		dev_dbg(dev,"%s: error coarse time SHS1 override\n", __func__);

	return err;
}

static int imx283_fill_string_ctrl(struct tegracam_device *tc_dev,
				struct v4l2_ctrl *ctrl)
{
	struct imx283 *priv = tc_dev->priv;
	int i;

	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_FUSE_ID:
		for (i = 0; i < IMX283_FUSE_ID_SIZE; i++)
			sprintf(&ctrl->p_new.p_char[i*2], "%02x",
				priv->fuse_id[i]);
		break;
	default:
		return -EINVAL;
	}

	ctrl->p_cur.p_char = ctrl->p_new.p_char;

	return 0;
}

static struct tegracam_ctrl_ops imx283_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.string_ctrl_size = {0, IMX283_FUSE_ID_STR_SIZE},
	.set_gain = imx283_set_gain,
	.set_exposure = imx283_set_exposure,
	.set_frame_rate = imx283_set_frame_rate,
	.set_group_hold = imx283_set_group_hold,
	.fill_string_ctrl = imx283_fill_string_ctrl,
};

static int imx283_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power on\n", __func__);
	if (pdata && pdata->power_on) {
		err = pdata->power_on(pw);
		if (err)
			dev_err(dev, "%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	/*exit reset mode: XCLR */
	if (pw->reset_gpio) {
		gpio_set_value(pw->reset_gpio, 0);
		usleep_range(30, 50);
		gpio_set_value(pw->reset_gpio, 1);
		usleep_range(30, 50);
	}

	pw->state = SWITCH_ON;
	return 0;

}

static int imx283_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power off\n", __func__);

	if (pdata && pdata->power_off) {
		err = pdata->power_off(pw);
		if (!err)
			goto power_off_done;
		else
			dev_err(dev, "%s failed.\n", __func__);
		return err;
	}
	/* enter reset mode: XCLR */
	usleep_range(1, 2);
	if (pw->reset_gpio)
		gpio_set_value(pw->reset_gpio, 0);

power_off_done:
	pw->state = SWITCH_OFF;

	return 0;
}

static int imx283_power_get(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	const char *mclk_name;
	struct clk *parent;
	int err = 0;

	mclk_name = pdata->mclk_name ?
		    pdata->mclk_name : "extperiph1";
	pw->mclk = devm_clk_get(dev, mclk_name);
	if (IS_ERR(pw->mclk)) {
		dev_err(dev, "unable to get clock %s\n", mclk_name);
		return PTR_ERR(pw->mclk);
	}

	parent = devm_clk_get(dev, "pllp_grtba");
	if (IS_ERR(parent))
		dev_err(dev, "devm_clk_get failed for pllp_grtba");
	else
		clk_set_parent(pw->mclk, parent);

	pw->reset_gpio = pdata->reset_gpio;

	pw->state = SWITCH_OFF;
	return err;
}

static int imx283_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;

	if (unlikely(!pw))
		return -EFAULT;

	return 0;
}

static struct camera_common_pdata *imx283_parse_dt(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	struct camera_common_pdata *ret = NULL;
	int err;
	int gpio;

	if (!np)
		return NULL;

	match = of_match_device(imx283_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(dev,
					sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	err = of_property_read_string(np, "mclk",
				      &board_priv_pdata->mclk_name);
	if (err)
		dev_err(dev, "mclk not in DT\n");

	gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (gpio < 0) {
		if (gpio == -EPROBE_DEFER)
			ret = ERR_PTR(-EPROBE_DEFER);
		dev_err(dev, "reset-gpios not found %d\n", err);
		goto error;
	}
	board_priv_pdata->reset_gpio = (unsigned int)gpio;

	return board_priv_pdata;

error:
	devm_kfree(dev, board_priv_pdata);
	return ret;
}

static int imx283_set_mode(struct tegracam_device *tc_dev)
{
	struct imx283 *priv = (struct imx283 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	const struct of_device_id *match;
	int err;

	match = of_match_device(imx283_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return -EINVAL;
	}

	err = imx283_write_table(priv, mode_table[s_data->mode_prop_idx]);
	if (err)
		return err;

	return 0;
}

static int imx283_start_streaming(struct tegracam_device *tc_dev)
{
	struct imx283 *priv = (struct imx283 *)tegracam_get_privdata(tc_dev);
	int err;

	if (test_mode) {
		err = imx283_write_table(priv,
			mode_table[IMX283_MODE_TEST_PATTERN]);
		if (err)
			return err;
	}

	err = imx283_write_table(priv,
		mode_table[IMX283_MODE_START_STREAM]);
	if (err)
		return err;

	return 0;
}

static int imx283_stop_streaming(struct tegracam_device *tc_dev)
{
	struct imx283 *priv = (struct imx283 *)tegracam_get_privdata(tc_dev);
	int err;

	err = imx283_write_table(priv, mode_table[IMX283_MODE_STOP_STREAM]);
	if (err)
		return err;

	/*
	 * Wait for one frame to make sure sensor is set to
	 * software standby in V-blank
	 *
	 * delay = frame length rows * Tline (10 us)
	 */
	usleep_range(priv->frame_length * 10, priv->frame_length * 10 + 1000);

	return 0;
}


static struct camera_common_sensor_ops imx283_common_ops = {
	.numfrmfmts = ARRAY_SIZE(imx283_frmfmt),
	.frmfmt_table = imx283_frmfmt,
	.power_on = imx283_power_on,
	.power_off = imx283_power_off,
	.write_reg = imx283_write_reg,
	.read_reg = imx283_read_reg,
	.parse_dt = imx283_parse_dt,
	.power_get = imx283_power_get,
	.power_put = imx283_power_put,
	.set_mode = imx283_set_mode,
	.start_streaming = imx283_start_streaming,
	.stop_streaming = imx283_stop_streaming,
};

static int imx283_board_setup(struct imx283 *priv)
{
	struct camera_common_data *s_data = priv->s_data;
	struct device *dev = s_data->dev;
	int err = 0;

	dev_dbg(dev, "%s++\n", __func__);

	err = camera_common_mclk_enable(s_data);
	if (err) {
		dev_err(dev,
			"Error %d turning on mclk\n", err);
		return err;
	}

	err = imx283_power_on(s_data);
	if (err) {
		dev_err(dev,
			"Error %d during power on sensor\n", err);
		return err;
	}

//error:
	imx283_power_off(s_data);
	camera_common_mclk_disable(s_data);
	
	return err;
}

static int imx283_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops imx283_subdev_internal_ops = {
	.open = imx283_open,
};

static int imx283_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct tegracam_device *tc_dev;
	struct imx283 *priv;
	int err;

	dev_info(dev, "probing v4l2 sensor\n");

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	priv = devm_kzalloc(dev,
			sizeof(struct imx283), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	tc_dev = devm_kzalloc(dev,
			sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	priv->i2c_client = tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "imx283", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &sensor_regmap_config;
	tc_dev->sensor_ops = &imx283_common_ops;
	tc_dev->v4l2sd_internal_ops = &imx283_subdev_internal_ops;
	tc_dev->tcctrl_ops = &imx283_ctrl_ops;

	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}
	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);

	err = imx283_board_setup(priv);
	if (err) {
		tegracam_device_unregister(tc_dev);
		dev_err(dev, "board setup failed\n");
		return err;
	}

	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}

	dev_info(dev, "Detected IMX283 sensor\n");

	return 0;
}

static int
imx283_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx283 *priv = (struct imx283 *)s_data->priv;

	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);

	return 0;
}

static const struct i2c_device_id imx283_id[] = {
	{ "imx283", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, imx283_id);

static struct i2c_driver imx283_i2c_driver = {
	.driver = {
		.name = "imx283",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(imx283_of_match),
	},
	.probe = imx283_probe,
	.remove = imx283_remove,
	.id_table = imx283_id,
};

module_i2c_driver(imx283_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for Sony IMX283");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL v2");
