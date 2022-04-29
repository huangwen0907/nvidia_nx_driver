/*
 * imx586.c - imx586 sensor driver
 *
 * Copyright (c) 2018-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * Copyright (c) 2016-2019, Guoxin Wu <guoxinw@leopardimaging.com>.
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


//#define _ERROR_TEST_

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <linux/i2c.h>
#include <linux/kthread.h>

#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>
#include <media/tegra-v4l2-camera.h>
#include <media/camera_common.h>


#include "imx586_8_mode_tbls.h"

#include "pdaflibrary.h"



#define ASSET_TRACKING_ID_ADDR_START	   		0x4A
#define ASSET_TRACKING_ID_ADDR_END		   		0x58
#define ASSET_TRACKING_ID_LEN  			  				 (0x58 - 0x4A + 1)
#define ASSET_TRACKING_ID_BASEENCODE_LEN   ((ASSET_TRACKING_ID_LEN/3+1)*4 + 1)


// default mode is 
#define IMX586_DEFAULT_MODE	IMX586_MODE_8000X6000_FULL_RAW
#define IMX586_DEFAULT_DATAFMT	MEDIA_BUS_FMT_SRGGB12_1X12

#define IMX586_MIN_FRAME_LENGTH	(1125)
#define IMX586_MAX_FRAME_LENGTH	(0xFFFF)//(0x1FFFF)

#define IMX586_MIN_FRAME_LINES (3000)
#define IMX586_DEFAULT_FRAME_LINES (6000)
#define IMX586_MAX_FRAME_LINES (6060)

#define IMX586_FRAME_LINES_ADDR_MSB	0x0340
#define IMX586_FRAME_LINES_ADDR_LSB	0x0341

#define IMX586_MIN_SHS1_1080P_HDR	(5)
#define IMX586_MIN_SHS2_1080P_HDR	(82)
#define IMX586_MAX_SHS2_1080P_HDR	(IMX586_MAX_FRAME_LENGTH - 5)
#define IMX586_MAX_SHS1_1080P_HDR	(IMX586_MAX_SHS2_1080P_HDR / 16)

#define IMX586_FRAME_LENGTH_ADDR_MSB		IMX586_FRAME_LINES_ADDR_MSB
#define IMX586_FRAME_LENGTH_ADDR_LSB		IMX586_FRAME_LINES_ADDR_LSB

//#define IMX586_COARSE_TIME_SHS1_ADDR_MSB	0x3022
//#define IMX586_COARSE_TIME_SHS1_ADDR_MID	0x3021
//#define IMX586_COARSE_TIME_SHS1_ADDR_LSB	0x3020
//#define IMX586_COARSE_TIME_SHS2_ADDR_MSB	0x3025
//#define IMX586_COARSE_TIME_SHS2_ADDR_MID	0x3024
//#define IMX586_COARSE_TIME_SHS2_ADDR_LSB	0x3023

//#define IMX586_GAIN_ADDR					0x3014
#define IMX586_GROUP_HOLD_ADDR				0x0104
#define IMX586_SW_RESET_ADDR				0x0103
// 1: 2 lane, 3: 4 lanes
#define IMX586_CSI_LANE_MODE_ADDR			0x0114

// Analog gain from 0 to 448 uint16
#define IMX586_ANA_GAIN_GLOBAL_ADDR_MSB		0x0204
#define IMX586_ANA_GAIN_GLOBAL_ADDR_LSB		0x0205

#define IMX586_COARSE_TIME_MIX				0x05
#define IMX586_COARSE_MARGIN_MAX			48
#define IMX586_COARSE_TIME_MAX				(IMX586_DEFAULT_FRAME_LINES-IMX586_COARSE_MARGIN_MAX)

// coarse intergraation time, unit lines
#define IMX586_COARSE_INTEG_TIME_MSB		0x0202
#define IMX586_COARSE_INTEG_TIME_LSB		0x0203

// Fine storage time
#define IMX586_FINE_INTEG_TIME_MSB				0x0200
#define IMX586_FINE_INTEG_TIME_LSB				0x0201


#define IMX586_FRM_LENGTH_CTL						0x0350


#define IMX586_COARSE_INTEG_TIME_MIN_QBC		5
#define IMX586_COARSE_INTEG_TIME_MIN_NOR		6


// flip or mirror
#define IMX586_FLIP_MIRROR_ADDR			0x0101


// hdr mode
#define IMX586_ST_COARSE_TIME_MSB		0x0224
#define IMX586_ST_COARSE_TIME_LSB		0x0225

//limit analog gain
#define IMX586_ANALOG_GAIN_LIMIT_ADDR			0x3012
#define IMX586_ANALOG_GAIN_LIMIT_VALUE			0x0f

//analog gain
#define IMX586_ANALOG_GAIN_VALUE_MIN			112
#define IMX586_ANALOG_GAIN_VALUE_MAX			1008

#define IMX586_ANALOG_GAIN_DB_MIN					1
#define IMX586_ANALOG_GAIN_DB_MAX_4000	36
#define IMX586_ANALOG_GAIN_DB_MAX_8000	24


#define IMX586_COARSE_TIME_MIN_2X2_4000X3000_PDAFX				6
#define IMX586_COARSE_TIME_STEP_2X2_4000X3000_PDAFX			2
#define IMX586_COARSE_MARGINS_MAX			48


#define IMX586_FUSE_ID_ADDR	0x3382
#define IMX586_FUSE_ID_SIZE	6
#define IMX586_FUSE_ID_STR_SIZE	(IMX586_FUSE_ID_SIZE * 2)
#define IMX586_DEFAULT_WIDTH	8000
#define IMX586_DEFAULT_HEIGHT	6000
#define IMX586_DEFAULT_CLK_FREQ	37125000


// enable
#define IMX586_VAL_DISABLE		0
#define IMX586_VAL_ENABLE			1


// motor
#define IMX586_MOTOR_ADDRESS		0x0C//0x18


#define IMX586_MOTOR_STEPS_INFINNITY	0
#define IMX586_MOTOR_STEPS_MICRO		1023

//eeprom
#define IMX586_EEPROM_ADDRESS	0x50//0x18
#define IMX586_EEPROM_ZIZE			8192

//Module information Flag   0x01: Valid, other: Invalid
#define IMX586_EEPROM_REGADDR_MODINFO		0x00

//Module id   0X07��Ofilm
#define IMX586_EEPROM_REGADDR_MODID			0x01

//Sensor ID  0x01: IMX586
#define IMX586_EEPROM_REGADDR_SENSORID	0x05

//Lens ID  0x01: OF-4801A
#define IMX586_EEPROM_REGADDR_LENSID			0x06

//AF Data  
// 0x01: Valid, other: Invalid
#define IMX586_EEPROM_REGADDR_AF_FLAG			0x50

//AF Calibration Direction
#define IMX586_EEPROM_REGADDR_CALI_DIR			0x51

//AF_Infinity (High Byte)  10m
#define IMX586_EEPROM_REGADDR_10M_H				0x52

//AF_Infinity (Low Byte)  10m
#define IMX586_EEPROM_REGADDR_10M_L				0x53

//AF_ Macro (High Byte)  10cm
#define IMX586_EEPROM_REGADDR_10CM_H				0x54

//AF_ Macro (Low Byte)  10cm
#define IMX586_EEPROM_REGADDR_10CM_L				0x55



// PDAF:  EEPROM
// LRC DATA: 384
#define IMX586_EEPROM_LRC_NUMBER				384

#define IMX586_EEPROM_REGADDR_LRC_FLAG				0x07AE

#define IMX586_EEPROM_REGADDR_LRC_LEFT_START				0x07AF
#define IMX586_EEPROM_REGADDR_LRC_LEFT_END					0x086E

#define IMX586_EEPROM_REGADDR_LRC_RIGHT_START				0x086F
#define IMX586_EEPROM_REGADDR_LRC_RIGHT_END					0x092E

// DCC:  96
#define IMX586_EEPROM_DCC_NUMBER				96

#define IMX586_EEPROM_REGADDR_DCC_START				0x092F
#define IMX586_EEPROM_REGADDR_DCC_END					0x098E


// PDAF:  SENSOR
#define IMX586_PDAF_LRC_NUMBER				384

#define IMX586_PDAF_REGADDR_LRC_LEFT_START				0x7510
#define IMX586_PDAF_REGADDR_LRC_LEFT_END					0x75CF

#define IMX586_PDAF_REGADDR_LRC_RIGHT_START				0x7600
#define IMX586_PDAF_REGADDR_LRC_RIGHT_END					0x76BF


// PDAF_CTRL1
 #define IMX586_PDAF_CTRL1		0x3E37


/* area mode
Type of AF detection area
0: Fixed area (16x12) 
1: Fixed area (8x6) 
2: Free area switch
3: Reserved
*/
#define IMX586_PDAF_AREA_MODE		0x38A3

#define IMX586_PDAF_AREA_MODE0		0	//Fixed area (16x12)
#define IMX586_PDAF_AREA_MODE1		1	//Fixed area (8x6)
#define IMX586_PDAF_AREA_MODE2		2	//Free area switch


//Flexible area 0 enable
#define IMX586_PDAF_AREA_EN_0		0x38AC	

// Area setting
#define IMX586_PD_AREA_XSTA_0_H5		0x38B4	
#define IMX586_PD_AREA_XSTA_0_L8			0x38B5	
#define IMX586_PD_AREA_YSTA_0_H5		0x38B6	
#define IMX586_PD_AREA_YSTA_0_L8			0x38B7	
#define IMX586_PD_AREA_XEND_0_H5		0x38B8	
#define IMX586_PD_AREA_XEND_0_L8		0x38B9	
#define IMX586_PD_AREA_YEND_0_H5		0x38BA	
#define IMX586_PD_AREA_YEND_0_L8			0x38BB	


// area center window ratio
#define IMX586_PDAF_AREA_WINDOW_RATI0		6


// Confidence Level
#define IMX586_PD_CONFIDENCE_LEVEL_H		0xC808
#define IMX586_PD_CONFIDENCE_LEVEL_L			0xC809

#define IMX586_PD_PHASE_DIFFERENCE_L		0xC80A


#define IMX586_PD_CONFIDENCE_LEVEL_VALID_MIN		150


// Phase Difference Data hold control
#define IMX586_TABLE_HOLD									0xE24F


//exposure hdr
#define IMX586_PIXEL_CLK_4000X3000_30FPS_HDR			1718400000

#define IMX586_CIT_LSHIFT			0x3100


// limit frame rate
#define LIMIT_FRAMERATE_EXPOSURE			1

#define FLIP_MIRROR_ENABLE			0

// pdaf
#define PDAF_SOLUTION_NO		0
#define PDAF_SOLUTION_1			1 //fixeanble window
#define PDAF_SOLUTION_2			2	// 8 windows(TEST)
#define PDAF_SOLUTION_3			3	// no thread


#define PDAF_SOLUTION				PDAF_SOLUTION_NO//PDAF_SOLUTION_NO//PDAF_SOLUTION_1//PDAF_SOLUTION_2	//


#if (PDAF_SOLUTION_NO != PDAF_SOLUTION)
#define QSC_EANBLE		0
#else
#define QSC_EANBLE		0
#endif


// trackid
#define IMX586_CHECK_TRACKID			0



#define _I2C_DEB_



#if (1 == QSC_EANBLE)
#define IMX586_EEPROM_QSC_NUM				2304

#define IMX586_EEPROM_QSC_ADDR_START			0x0990
#define IMX586_EEPROM_QSC_ADDR_END				0x128F


#define IMX586_SENSOR_QSC_EN			0x3621

#define IMX586_SENSOR_QSC_ADDR_START		0x7F00
#define IMX586_SENSOR_QSC_ADDR_END			0x87FF

#endif



const static struct of_device_id imx586_of_match[] = {
	{ .compatible = "nvidia,imx586_8",},
	{ },
};

MODULE_DEVICE_TABLE(of, imx586_of_match);

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_FUSE_ID,
	TEGRA_CAMERA_CID_HDR_EN,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};

struct imx586_extdevice {
	struct i2c_client	*i2c_client;
	struct i2c_adapter *adap;
	struct i2c_board_info brd;
	struct regmap	*regmap;
};

struct imx586 {
	struct camera_common_power_rail	power;
	int	numctrls;
	struct v4l2_ctrl_handler	ctrl_handler;
	struct i2c_client	*i2c_client;

	struct v4l2_subdev	*subdev;
    u8 fuse_id[IMX586_FUSE_ID_SIZE];
	struct media_pad	pad;
	u32				frame_length;
	s32	group_hold_prev;
	bool	group_hold_en;
	s64 last_wdr_et_val;
	struct regmap	*regmap;
	struct camera_common_data	*s_data;
	struct camera_common_pdata	*pdata;
	struct tegracam_device		*tc_dev;
	//struct v4l2_ctrl		*ctrls[];

	struct imx586_extdevice trackid;
	u8 check_encrypt;
	
#if (PDAF_SOLUTION_NO != PDAF_SOLUTION)
	struct imx586_extdevice focusmotor;
	struct imx586_extdevice eeprom;
	
	struct task_struct *kthread_focus;
	struct mutex kthread_lock;

	u16 cur_pos;
	u32 frame_rate;
	u8 streamflag;
	bool readingpd;
	u16 dis_infinity;
	u16 dis_micro;
#endif	
};


static const struct regmap_config sensor_regmap_config = {
         .reg_bits = 16,
         .val_bits = 8,
         .cache_type = REGCACHE_RBTREE,	//REGCACHE_NONE,	//
         .use_single_rw = true,
};

static u16 gain_analog[36] = {
	112,// 1db
	211,
	300,
	378,
	449,
	
	511,// 6db
	567,
	617,
	661,
	701,

	736,// 11db
	767,
	795,
	820,
	842,

	862,// 16db
	880, 
	896,
	910,
	922,

	933,// 21db
	943,
	952,
	960,
	967,

	973,//26db
	979,
	984,
	988,
	992,

	996,// 31db
	999,
	1002,
	1004,
	1006,// 35db
	1008,
};


#if (PDAF_SOLUTION_1 == PDAF_SOLUTION)

static signed long eeprom_dcc[IMX586_EEPROM_DCC_NUMBER/2] = {0};

static imx586_reg center_area[9] = {
		{IMX586_PD_AREA_XSTA_0_H5,	0},
		{IMX586_PD_AREA_XSTA_0_L8,	0},
		{IMX586_PD_AREA_YSTA_0_H5,	0},
		{IMX586_PD_AREA_YSTA_0_L8,	0},
		
		{IMX586_PD_AREA_XEND_0_H5, 0},
		{IMX586_PD_AREA_XEND_0_L8,	0},
		{IMX586_PD_AREA_YEND_0_H5,	0},
		{IMX586_PD_AREA_YEND_0_L8,	0},
		
		{IMX586_TABLE_END, 0x00}
};
#elif (PDAF_SOLUTION_2 == PDAF_SOLUTION || PDAF_SOLUTION_3 == PDAF_SOLUTION)
#if (PDAF_SOLUTION_2 == PDAF_SOLUTION)
static imx586_reg fixedarea_dcc[10] = {
		{0x3E20,0x01},
		{0x3E37,0x01},
		{0x0101,0x00},
		{0x0B00,0x00},
		{0x3606,0x01},
		{0x3E36,0x01},
		{0x3E35,0x01},
		{0x3D0D,0x01},
		{0x3D10,0x00},

		{IMX586_TABLE_END, 0x00}
};
#endif
static imx586_reg center_area[9] = {
		{0x38A4,	0x00},
		{0x38A5,	0x10},
		{0x38A6,	0x00},
		{0x38A7, 0x0C},
		
		{0x38A8,	0x01},
		{0x38A9,	0xF0},
		{0x38AA,	0x01},
		{0x38AB,	0xF0},
		
		{IMX586_TABLE_END, 0x00}
};
#endif

#if (PDAF_SOLUTION_3 == PDAF_SOLUTION)
static unsigned short lrc_table0[12*16] = {
	0x00ce, 0x00c3, 0x00b6, 0x00aa, 0x009e, 0x0092, 0x0087, 0x007e, 0x0075, 0x006d, 0x0066, 0x0060, 0x005b, 0x0056, 0x0053, 0x0050,
	0x00d0, 0x00c5, 0x00b7, 0x00ab, 0x009e, 0x0093, 0x0088, 0x007e, 0x0075, 0x006d, 0x0066, 0x0060, 0x005b, 0x0056, 0x0053, 0x0050,
	0x00d1, 0x00c6, 0x00b8, 0x00ab, 0x009f, 0x0093, 0x0088, 0x007e, 0x0074, 0x006d, 0x0066, 0x0060, 0x005b, 0x0056, 0x0052, 0x0050,
	0x00d3, 0x00c7, 0x00b9, 0x00ac, 0x00a0, 0x0094, 0x0088, 0x007e, 0x0074, 0x006c, 0x0066, 0x0060, 0x005a, 0x0056, 0x0052, 0x004f,
	0x00d3, 0x00c7, 0x00b9, 0x00ac, 0x00a0, 0x0094, 0x0088, 0x007e, 0x0074, 0x006c, 0x0065, 0x005f, 0x005a, 0x0056, 0x0052, 0x004f,
	0x00d3, 0x00c7, 0x00ba, 0x00ac, 0x00a0, 0x0094, 0x0088, 0x007d, 0x0074, 0x006c, 0x0065, 0x005f, 0x005a, 0x0055, 0x0052, 0x004f,
	0x00d2, 0x00c6, 0x00b9, 0x00ab, 0x009f, 0x0093, 0x0087, 0x007d, 0x0073, 0x006b, 0x0065, 0x005f, 0x005a, 0x0055, 0x0051, 0x004f,
	0x00d1, 0x00c5, 0x00b8, 0x00ab, 0x009e, 0x0092, 0x0086, 0x007c, 0x0073, 0x006b, 0x0064, 0x005e, 0x0059, 0x0055, 0x0051, 0x004e,
	0x00cf, 0x00c4, 0x00b6, 0x00a9, 0x009d, 0x0091, 0x0086, 0x007c, 0x0073, 0x006b, 0x0064, 0x005e, 0x0059, 0x0054, 0x0051, 0x004e,
	0x00cc, 0x00c1, 0x00b5, 0x00a8, 0x009c, 0x0090, 0x0085, 0x007b, 0x0072, 0x006a, 0x0064, 0x005e, 0x0059, 0x0054, 0x0051, 0x004e,
	0x00c9, 0x00bf, 0x00b3, 0x00a6, 0x009a, 0x008f, 0x0085, 0x007b, 0x0072, 0x006a, 0x0064, 0x005e, 0x0059, 0x0054, 0x0050, 0x004e,
	0x00c6, 0x00bd, 0x00b0, 0x00a5, 0x0099, 0x008e, 0x0084, 0x007a, 0x0072, 0x006a, 0x0064, 0x005e, 0x0059, 0x0054, 0x0050, 0x004e,
};

static unsigned short lrc_table1[12*16] = {
	0x004d, 0x0050, 0x0053, 0x0057, 0x005c, 0x0062, 0x0068, 0x0070, 0x0078, 0x0081, 0x008b, 0x0096, 0x00a1, 0x00ab, 0x00b6, 0x00bd,
	0x004d, 0x0050, 0x0053, 0x0057, 0x005c, 0x0062, 0x0069, 0x0071, 0x0079, 0x0083, 0x008d, 0x0097, 0x00a3, 0x00ad, 0x00b8, 0x00c0,
	0x004d, 0x0050, 0x0053, 0x0058, 0x005d, 0x0063, 0x006a, 0x0071, 0x007a, 0x0084, 0x008e, 0x0099, 0x00a4, 0x00b0, 0x00bb, 0x00c3,
	0x004d, 0x0050, 0x0053, 0x0058, 0x005d, 0x0063, 0x006a, 0x0072, 0x007b, 0x0085, 0x008f, 0x009a, 0x00a6, 0x00b1, 0x00bd, 0x00c6,
	0x004d, 0x0050, 0x0053, 0x0058, 0x005d, 0x0063, 0x006a, 0x0072, 0x007c, 0x0086, 0x0090, 0x009b, 0x00a7, 0x00b2, 0x00be, 0x00c7,
	0x004d, 0x0050, 0x0053, 0x0058, 0x005d, 0x0063, 0x006a, 0x0072, 0x007c, 0x0086, 0x0090, 0x009c, 0x00a8, 0x00b3, 0x00bf, 0x00c9,
	0x004d, 0x0050, 0x0053, 0x0058, 0x005d, 0x0063, 0x006b, 0x0073, 0x007c, 0x0086, 0x0091, 0x009d, 0x00a9, 0x00b5, 0x00c0, 0x00ca,
	0x004d, 0x0050, 0x0053, 0x0058, 0x005d, 0x0063, 0x006b, 0x0073, 0x007c, 0x0086, 0x0091, 0x009d, 0x00a9, 0x00b5, 0x00c1, 0x00ca,
	0x004d, 0x0050, 0x0053, 0x0058, 0x005d, 0x0063, 0x006b, 0x0073, 0x007c, 0x0086, 0x0091, 0x009d, 0x00a9, 0x00b5, 0x00c1, 0x00ca,
	0x004d, 0x0050, 0x0053, 0x0058, 0x005d, 0x0063, 0x006a, 0x0072, 0x007c, 0x0086, 0x0090, 0x009c, 0x00a8, 0x00b4, 0x00c0, 0x00c9,
	0x004d, 0x0050, 0x0053, 0x0058, 0x005d, 0x0063, 0x006a, 0x0072, 0x007b, 0x0085, 0x008f, 0x009b, 0x00a7, 0x00b2, 0x00bf, 0x00c8,
	0x004d, 0x0050, 0x0054, 0x0058, 0x005d, 0x0063, 0x006a, 0x0072, 0x007a, 0x0084, 0x008e, 0x0099, 0x00a6, 0x00b1, 0x00bd, 0x00c7,
};
#endif

#if IMX586_CHECK_TRACKID
static u8 trackid[16] = {
	1, 4, 2, 3, 1, 2, 1, 0, 3, 4, 7, 1, 0, 0, 0, 0
};
#endif

#if (PDAF_SOLUTION_1 == PDAF_SOLUTION)

/******   pdaflibrary.c  **********************************************************************/


/****************************************************************/
/*                          version                             */
/****************************************************************/

#define D_MAJOR_VERSION (1)                         /* Integer part of PDAF Library version. */
#define D_MINOR_VERSION (00)                        /* Decimal part of PDAF Library version. */

/****************************************************************/
/*                 local function declaration                   */
/****************************************************************/

void job_init_output_data ( PdLibOutputData_t *pfa_OutputData );
signed long job_check_input ( PdLibInputData_t *pfa_InputData );
void job_calc_defocus ( PdLibInputData_t *pfa_InputData, signed long *pfa_Defocus );
void job_calc_defocus_confidence_level ( PdLibInputData_t *pfa_InputData, unsigned long *pfa_DefocusConfidenceLevel );
void job_calc_defocus_confidence ( unsigned long fa_DefocusConfidenceLevel, signed char *pfa_DefocusConfidence );
void job_calc_phase_difference ( PdLibInputData_t *pfa_InputData, signed long *pfa_PhaseDifference );

signed long calc_defocus_formula ( PdLibInputData_t *pfa_InputData, unsigned short fa_Index );
//signed long limit_defocus_formula ( double fa_Defocus );
signed long limit_defocus_formula (signed long fa_Defocus );
signed long calc_defocus_ok_ng_thr ( PdLibInputData_t *pfa_InputData, unsigned short fa_Index );
//unsigned long limit_defocus_confidence_level ( double fa_DefocusConfidenceLevel );
unsigned long limit_defocus_confidence_level (signed long fa_DefocusConfidenceLevel );

/****************************************************************/
/*                      external function                       */
/****************************************************************/
/* API : Get version information of PDAF Library. */
void PdLibGetVersion
( 
    PdLibVersion_t  *pfa_PdLibVersion                       /* Output : PDAF Library version */
)
{
    (*pfa_PdLibVersion).MajorVersion = D_MAJOR_VERSION;     /* Set integer part of PDAF Library version. */
    (*pfa_PdLibVersion).MinorVersion = D_MINOR_VERSION;     /* Set decimal part of PDAF Library version. */

    return ;
}

//EXPORT_SYMBOL_GPL(PdLibGetVersion);

/* API : Get defocus data according to a PDAF window. */
signed long PdLibGetDefocus 
(
    PdLibInputData_t    *pfa_PdLibInputData,                /* Input  : Input data structure */
    PdLibOutputData_t   *pfa_PdLibOutputData                /* Output : Output data structure */
)
{
    signed long ret;
    signed long RetCheckInput;
    PdLibOutputData_t PdLibOutputData;

    job_init_output_data ( pfa_PdLibOutputData );           /* Initialization of  output data structure */

    RetCheckInput = job_check_input ( pfa_PdLibInputData ); /* Check value of input data structure */

    if ( RetCheckInput != D_PD_LIB_E_OK ) {                 /* Check the value of input */
        ret = RetCheckInput;
        return ret;                                         /* Return error value */
    } else {
        ret = D_PD_LIB_E_OK;                                /* Set return value as OK */
    }

    job_calc_defocus ( pfa_PdLibInputData, &(PdLibOutputData.Defocus) );    /* Calculate defocus */

    /* Check XKnotNumDefocusOKNG and YKnotNumDefocusOKNG */ 
    if ((pfa_PdLibInputData->XKnotNumDefocusOKNG != 0) && (pfa_PdLibInputData->YKnotNumDefocusOKNG != 0)) {
        /* Check the value of input */
        if ( (*pfa_PdLibInputData).PhaseDifference != ( D_PD_ERROR_VALUE << 4 ) ) {
            /* Calculate defocus confidence level */
            job_calc_defocus_confidence_level(pfa_PdLibInputData, &(PdLibOutputData.DefocusConfidenceLevel));
            /* Calculate defocus confidence */
            job_calc_defocus_confidence ( PdLibOutputData.DefocusConfidenceLevel, &(PdLibOutputData.DefocusConfidence) );
        } else {                                            /* Error of phase difference */
            PdLibOutputData.DefocusConfidenceLevel = 0;     /* Set defocus confidence level as Zero */
            PdLibOutputData.DefocusConfidence = -EPDVALERR; /* Set defocus confidence as input error */
        }

    } else {                                                /* Error of XKnotNumDefocusOKNG or YKnotNumDefocusOKNG */
        PdLibOutputData.DefocusConfidenceLevel = 0;         /* Set defocus confidence level as Zero */
        PdLibOutputData.DefocusConfidence = -ENCWDDON;      /* Set defocus confidence as NCW */
    }

    /* Calculate phase difference */
    job_calc_phase_difference ( pfa_PdLibInputData, &(PdLibOutputData.PhaseDifference) );

    (*pfa_PdLibOutputData) = PdLibOutputData;               /* Set result of job_calc_phase_difference() */

    return ret;                                             /* Return OK */
}

/**** PdafMathFunc. **********************************************************************************/


/* Function for calculating coordination at the point of the line */
void CalcAddressOnLine_slXslY
(
    /* Input */
    signed long *pf_x,
    signed long *pf_y,
    signed long f_xx,
    /* Output */
    signed long *fp_yy
)
{
    signed long y;

     if ( pf_x[0] == pf_x[1] && pf_y[0] == pf_y[1] ) { y = pf_y[0]; }
    else if ( pf_x[0] != pf_x[1] && pf_y[0] == pf_y[1] ) { y = pf_y[0]; }
    else if ( pf_x[0] == pf_x[1] ) { y = (pf_y[0]+pf_y[1])/2; }
    else {
        /* Equation of the line passing through (x0, y0) and (x1, y1). */ 
        /* y = y0 + (y1 - y0) * (x - x0) / (x1 - x0) */
        
        signed long y0;
        signed long y1;
        signed long x0;
        signed long x1;
        signed long x;

        x = f_xx;

        if ( pf_x[0] <= pf_x[1] ) {
            x0 = pf_x[0];
            x1 = pf_x[1];
            y0 = pf_y[0];
            y1 = pf_y[1];
        } else {
            x0 = pf_x[1];
            x1 = pf_x[0];
            y0 = pf_y[1];
            y1 = pf_y[0];
        }

        if ( x < x0 ) {
            y = y0;
        } else if ( x1 < x ) {
            y = y1;
        } else {
        	#if 0
            double yy;
            /* y = y0 + (y1 - y0) * (x - x0) / (x1 - x0) */
            yy  = (double)y0 
                + ((double)y1 - (double)y0)
                * ((double) x - (double)x0) 
                / ((double)x1 - (double)x0);
            y = (long)yy;
			#endif

			#if 1
			signed long yy;
            /* y = y0 + (y1 - y0) * (x - x0) / (x1 - x0) */
            yy  = (signed long )y0 
                + ((signed long)y1 - (signed long)y0)
                * ((signed long) x - (signed long)x0) 
                / ((signed long)x1 - (signed long)x0);
            y = (signed long)yy;		
			#endif
        }
        
    }

    (*fp_yy) = y;

    return ;
}

/* Function for calculating coordination at the point of the broken line */
signed char CalcAddressOnBrokenLine_ulXulY
(
    /* Input */
    unsigned long *pf_x,
    unsigned long *pf_y,
    unsigned long f_PointNum,
    unsigned long f_xx,
    /* Output */
    unsigned long *pf_yy
)
{
    unsigned long i;
    unsigned long y;

    if ( 2 <= f_PointNum ) {
    } else {
        return D_MATH_FUNC_NG;
    }

    /*
        *pf_x
        *pf_y
        f_xx
        The range needs equal or less than 0x7FFFFFFF.
        In the case of out of bounds, return D_MATH_FUNC_NG.
    */
    
    if( pf_x[f_PointNum-1] <= 0x7FFFFFFF ) {
    } else {
        return D_MATH_FUNC_NG;
    }
    
    if( f_xx <= 0x7FFFFFFF ) {
    } else {
        return D_MATH_FUNC_NG;
    }
    
    for( i=0; i < f_PointNum; i++ ) {
        if( pf_y[i] <= 0x7FFFFFFF ) {
            
        } else {
            return D_MATH_FUNC_NG;
        }
    }
    
    for ( i = 0; i < f_PointNum-1; i++ ) {
        if( pf_x[i] <= pf_x[i+1] ) {
        } else {
            return D_MATH_FUNC_NG;
        }
    }

    if ( f_xx < pf_x[0] ) {
        y = pf_y[0];
    } else if ( pf_x[f_PointNum-1] < f_xx ) {
        y = pf_y[f_PointNum-1];
    } else {
        y = 0;
        
        for ( i = 0; i < f_PointNum-1; i++ ) {
            if( pf_x[i] <= f_xx && f_xx <= pf_x[i+1] ) {
                
                signed long LineX[2];
                signed long LineY[2];
                signed long PointX;
                signed long PointY;

                LineX[0] = (signed long)(pf_x[i]);
                LineX[1] = (signed long)(pf_x[i+1]);
                LineY[0] = (signed long)(pf_y[i]);
                LineY[1] = (signed long)(pf_y[i+1]);
                PointX   = (signed long)(f_xx);

                CalcAddressOnLine_slXslY ( LineX, LineY, PointX, &PointY );

                if ( PointY <= 0 ) {
                    PointY = 0;
                }

                y = (unsigned long)PointY;

                break ;
                
            }
        }
    }

    (*pf_yy) = y;

    return D_MATH_FUNC_OK;
}

/* Function for calculating coordination at the point of the plane */
signed char CalcAddressOnPlane_slXslYslZ
(
    /* Input */
    signed long *pf_x,
    signed long *pf_y,
    signed long *pf_z,
    signed long f_xx,
    signed long f_yy,
    /* Output */
    signed long *pf_zz
)
{
    if ( pf_x[0] <= f_xx && f_xx <= pf_x[1] &&  
         pf_y[0] <= f_yy && f_yy <= pf_y[1] &&  
         pf_x[0] < pf_x[1] &&  pf_y[0] < pf_y[1] ) {
    } else {
        return D_MATH_FUNC_NG;
    }

    {
        signed long z1;
        signed long z2;
        signed long z;

        signed long LineX[2];
        signed long LineY[2];
        signed long PointX;
        signed long PointY;

        LineX[0] = pf_x[0];
        LineX[1] = pf_x[1];
        LineY[0] = pf_z[0];
        LineY[1] = pf_z[1];
        PointX   = f_xx;

        CalcAddressOnLine_slXslY ( LineX, LineY, PointX, &PointY );

        z1 = PointY;

        LineY[0] = pf_z[2];
        LineY[1] = pf_z[3];

        CalcAddressOnLine_slXslY ( LineX, LineY, PointX, &PointY );

        z2 = PointY;

        LineX[0] = pf_y[0];
        LineX[1] = pf_y[1];
        LineY[0] = z1;
        LineY[1] = z2;
        PointX   = f_yy;

        CalcAddressOnLine_slXslY ( LineX, LineY, PointX, &PointY );

        z = PointY;

        (*pf_zz) = z;
    }

    return D_MATH_FUNC_OK;
}

/************************************************************************************************/




/****************************************************************/
/*                       local function                         */
/****************************************************************/
/* Function for initializing output data structure */
void job_init_output_data 
( 
    PdLibOutputData_t *pfa_OutputData                       /* Output : Output data structure */
)
{
    (*pfa_OutputData).Defocus                = 0;
    (*pfa_OutputData).DefocusConfidence      = D_PD_LIB_E_NG;
    (*pfa_OutputData).DefocusConfidenceLevel = 0;
    (*pfa_OutputData).PhaseDifference        = 0;

    return ;
}

/* Function for checking value of input data structure */
signed long job_check_input 
( 
    PdLibInputData_t *pfa_InputData                         /* Input : Input data structure */
)
{
    signed long ret;

    ret = D_PD_LIB_E_OK;                                    /* Set return value as OK */

    /* Check the value of input */
    /* Check XSizeOfImage */
    if ( 2 <= (*pfa_InputData).XSizeOfImage ) {             /* Check XSizeOfImage */
    } else {
        ret = -EINXSOI;                                     /* Out of range of XSizeOfImage */
    }
    /* Check YSizeOfImage */
    if ( ret == D_PD_LIB_E_OK ) {                           /* Check return value */
        if ( 2 <= (*pfa_InputData).YSizeOfImage ) {         /* Check YSizeOfImage */
        } else {
            ret = -EINYSOI;                                 /* Out of range of YSizeOfImage */
        }
    }
    /* Check PDAFWindowsX */
    if ( ret == D_PD_LIB_E_OK ) {                           /* Check return value */
        if ( ( (*pfa_InputData).XAddressOfWindowStart <= ( (*pfa_InputData).XAddressOfWindowEnd - 1 ) ) &&
             ( (*pfa_InputData).XAddressOfWindowEnd <= ( (*pfa_InputData).XSizeOfImage - 1 ) ) ) {
        } else {
            ret = -EINPDAFWX;                               /* Out of range of PDAFWindowsX */
        }
    }
    /* Check PDAFWindowsY */
    if ( ret == D_PD_LIB_E_OK ) {                           /* Check return value */
        if ( ( (*pfa_InputData).YAddressOfWindowStart <= ( (*pfa_InputData).YAddressOfWindowEnd - 1 ) ) &&
             ( (*pfa_InputData).YAddressOfWindowEnd <= ( (*pfa_InputData).YSizeOfImage - 1 ) ) ) {
        } else {
            ret = -EINPDAFWY;                               /* Out of range of PDAFWindowsY */
        }
    }
    /* Check Slope and Offset (defocus vs phase difference) */
    if ( ret == D_PD_LIB_E_OK ) {                           /* Check return value */
        if ( 2 <= (*pfa_InputData).XKnotNumSlopeOffset &&
             2 <= (*pfa_InputData).YKnotNumSlopeOffset ) {
        } else {
            ret = -EINSO;                                   /* Out of range of SlopeOffset */
        }
    }
    /* Check AdjCoeffSlope */
    if ( ret == D_PD_LIB_E_OK ) {                           /* Check return value */
        if ( ( (*pfa_InputData).AdjCoeffSlope == D_PD_LIB_SLOPE_ADJ_COEFF_SENS_MODE0 ) ||
             ( (*pfa_InputData).AdjCoeffSlope == D_PD_LIB_SLOPE_ADJ_COEFF_SENS_MODE1 ) ||
             ( (*pfa_InputData).AdjCoeffSlope == D_PD_LIB_SLOPE_ADJ_COEFF_SENS_MODE2 ) ||
             ( (*pfa_InputData).AdjCoeffSlope == D_PD_LIB_SLOPE_ADJ_COEFF_SENS_MODE3 ) ||
             ( (*pfa_InputData).AdjCoeffSlope == D_PD_LIB_SLOPE_ADJ_COEFF_SENS_MODE4 ) ) {
        } else {
            ret = -EINACS;                                  /* Out of range of AdjCoeffSlope */
        }
    }
    /* Check SlopeOffsetXAddressKnot */
    if ( ret == D_PD_LIB_E_OK ) {                           /* Check return value */
        unsigned short i;
        unsigned short XKnotNum;
        unsigned short *p_XAddressKnot;
        
        XKnotNum = (*pfa_InputData).XKnotNumSlopeOffset;
        p_XAddressKnot = (*pfa_InputData).p_XAddressKnotSlopeOffset;
        for ( i = 0; i < ( XKnotNum - 1 ); i++ ) {
            if ( p_XAddressKnot[i] < p_XAddressKnot[i + 1] ){
            } else {
                ret = -EINSOXAK;                            /* Out of range of SlopeOffsetXAddressKnot */
                break ;
            }
        }
    }
    /* Check SlopeOffsetYAddressKnot */
    if ( ret == D_PD_LIB_E_OK ) {                           /* Check return value */
        unsigned short i;
        unsigned short YKnotNum;
        unsigned short *p_YAddressKnot;
        
        YKnotNum = (*pfa_InputData).YKnotNumSlopeOffset;
        p_YAddressKnot = (*pfa_InputData).p_YAddressKnotSlopeOffset;
        for ( i = 0; i < ( YKnotNum - 1 ); i++ ) {
            if ( p_YAddressKnot[i] < p_YAddressKnot[i + 1] ) {
            } else {
                ret = -EINSOYAK;                            /* Out of range of SlopeOffsetYAddressKnot */
                break ;
            }
        }
    }
    /* Check Defocus OK/NG */
    if ( ret == D_PD_LIB_E_OK ) {                           /* Check return value */
        /* Check disable of the judge function of confidence */
        if ( 0 == (*pfa_InputData).XKnotNumDefocusOKNG ||
             0 == (*pfa_InputData).YKnotNumDefocusOKNG ) {
            if ( 0 == (*pfa_InputData).XKnotNumDefocusOKNG &&
                 0 == (*pfa_InputData).YKnotNumDefocusOKNG ) {
            }
            else {
                ret = -EINVALDISCONFJ;                      /* Invalid of Disable Confidence Judgement */
            }
        }
        /* Check disable of the judge function of confidence in each image area */
        else if ( 1 == (*pfa_InputData).XKnotNumDefocusOKNG ||
                  1 == (*pfa_InputData).YKnotNumDefocusOKNG ) {
            if ( 1 == (*pfa_InputData).XKnotNumDefocusOKNG &&
                 1 == (*pfa_InputData).YKnotNumDefocusOKNG ) {
            } else {
                ret = -EINVALDISIHC;                        /* Invalid of Disable compensation relation with image height */
            }
        }
    }
    /* Check DefocusOKNGThrPointNum */
    if ( ret == D_PD_LIB_E_OK ) {                           /* Check return value */
        if ( 2 <= (*((*pfa_InputData).p_DefocusOKNGThrLine)).PointNum ) {
        } else {
            ret = -EINDONTPN;                               /* Out of range of DefocusOKNGThrPointNum */
        }
    }
    /* Check DefocusOKNGXAddressKnot */
    if ( ret == D_PD_LIB_E_OK ) {                           /* Check return value */
        unsigned short i;
        unsigned short XKnotNum;
        unsigned short *p_XAddressKnot;
        
        XKnotNum = (*pfa_InputData).XKnotNumDefocusOKNG;
        p_XAddressKnot = (*pfa_InputData).p_XAddressKnotDefocusOKNG;
        for ( i = 0; i < ( XKnotNum - 1 ); i++ ) {
            if ( p_XAddressKnot[i] < p_XAddressKnot[i + 1] ) {
            } else {
                ret = -EINDONXAK;                           /* Out of range of DefocusOKNGXAddressKnot */
                break ;
            }
        }
    }
    /*  Check DefocusOKNGYAddressKnot */
    if ( ret == D_PD_LIB_E_OK ) {                           /* Check return value */
        unsigned short i;
        unsigned short YKnotNum;
        unsigned short *p_YAddressKnot;
        
        YKnotNum = (*pfa_InputData).YKnotNumDefocusOKNG;
        p_YAddressKnot = (*pfa_InputData).p_YAddressKnotDefocusOKNG;
        for ( i = 0; i < ( YKnotNum - 1 ); i++ ) {
            if ( p_YAddressKnot[i] < p_YAddressKnot[i + 1] ) {
            } else {
                ret = -EINDONYAK;                           /* Out of range of DefocusOKNGYAddressKnot */
                break ;
            }
        }
    }
    /* Check Phase Detection Pixel Density */
    if ( ret == D_PD_LIB_E_OK ) {               /* Check return value */
        if ( ( (*pfa_InputData).DensityOfPhasePix == D_PD_LIB_DENSITY_SENS_MODE0 ) ||
             ( (*pfa_InputData).DensityOfPhasePix == D_PD_LIB_DENSITY_SENS_MODE1 ) ||
             ( (*pfa_InputData).DensityOfPhasePix == D_PD_LIB_DENSITY_SENS_MODE2 ) ||
             ( (*pfa_InputData).DensityOfPhasePix == D_PD_LIB_DENSITY_SENS_MODE3 ) ||
             ( (*pfa_InputData).DensityOfPhasePix == D_PD_LIB_DENSITY_SENS_MODE4 ) ) {
        } else {
            ret = -EINDOP;                                  /* Out of range of DensityOfPhasePix */
        }
    }

    return ret;                                             /* Return result */
}

/* Function for calculating defocus */
void job_calc_defocus 
( 
    PdLibInputData_t *pfa_InputData,                        /* Input  : Input data structure */
    signed long *pfa_Defocus                                /* Output : Defocus */
)
{
    unsigned short  i;
    unsigned short  XKnotNum;
    unsigned short  YKnotNum;
    unsigned short  *p_XAddressKnot;
    unsigned short  *p_YAddressKnot;
    unsigned short  XKnotStart;
    unsigned short  YKnotStart;
    unsigned char   AreaIndex;
    signed long     Defocus;
    signed long     XAddressPDAFWindowCenter;
    signed long     YAddressPDAFWindowCenter;

    XKnotNum = (*pfa_InputData).XKnotNumSlopeOffset;
    YKnotNum = (*pfa_InputData).YKnotNumSlopeOffset;

    p_XAddressKnot = (*pfa_InputData).p_XAddressKnotSlopeOffset;
    p_YAddressKnot = (*pfa_InputData).p_YAddressKnotSlopeOffset;

    XAddressPDAFWindowCenter = ( (*pfa_InputData).XAddressOfWindowStart + 
                                 (*pfa_InputData).XAddressOfWindowEnd ) / 2;
    YAddressPDAFWindowCenter = ( (*pfa_InputData).YAddressOfWindowStart + 
                                 (*pfa_InputData).YAddressOfWindowEnd ) / 2;
    
    XKnotStart = 0;
    for ( i = 0; i < XKnotNum-1; i++ ) {                    /* Check XKnotStart */
        if ( p_XAddressKnot[i] <= XAddressPDAFWindowCenter && 
             XAddressPDAFWindowCenter <= p_XAddressKnot[i+1] ) {
            XKnotStart = i;
            break ;
        }
    }

    YKnotStart = 0;
    for ( i = 0; i < YKnotNum-1; i++ ) {                    /* Check YKnotStar */
        if ( p_YAddressKnot[i] <= YAddressPDAFWindowCenter && 
             YAddressPDAFWindowCenter <= p_YAddressKnot[i+1] ) {
            YKnotStart = i;
            break ;
        }
    }

/*

    Divided by area. AreaIndex

    +---+---+---+
    | 0 | 1 | 2 |
    +---+---+---+
    | 3 | 4 | 5 |
    +---+---+---+
    | 6 | 7 | 8 |
    +---+---+---+

    Area 4 is surrounded by the knot points.
    Other area does not have knot points.

*/

    if ( YAddressPDAFWindowCenter < p_YAddressKnot[0] ) {
        /* Top */
             if ( XAddressPDAFWindowCenter   < p_XAddressKnot[0]        ) {/* Left   */ AreaIndex = 0;}
        else if ( p_XAddressKnot[XKnotNum-1] < XAddressPDAFWindowCenter ) {/* Right  */ AreaIndex = 2;}
        else                                                              {/* Center */ AreaIndex = 1;}       
    }
    else if ( p_YAddressKnot[YKnotNum-1] < YAddressPDAFWindowCenter ) {
        /* Bottom */
             if ( XAddressPDAFWindowCenter   < p_XAddressKnot[0]        ) {/* Left   */ AreaIndex = 6;}
        else if ( p_XAddressKnot[XKnotNum-1] < XAddressPDAFWindowCenter ) {/* Right  */ AreaIndex = 8;}
        else                                                              {/* Center */ AreaIndex = 7;}       
    } else {
        /* Center */
             if ( XAddressPDAFWindowCenter   < p_XAddressKnot[0]        ) {/* Left   */ AreaIndex = 3;}
        else if ( p_XAddressKnot[XKnotNum-1] < XAddressPDAFWindowCenter ) {/* Right  */ AreaIndex = 5;}
        else                                                              {/* Center */ AreaIndex = 4;}
    }

    if ( AreaIndex == 4 ) {                                 /* Center */
        unsigned short  Index;
        signed long     LineX[2];
        signed long     LineY[2];
        signed long     PlaneZ[4];
        signed long     PointX;
        signed long     PointY;
        signed long     PointZ = 0;
        
        Index = YKnotStart*XKnotNum+XKnotStart;

        LineX[0]  = p_XAddressKnot[XKnotStart  ];
        LineX[1]  = p_XAddressKnot[XKnotStart+1];           /* Next to LineX[0] */
        LineY[0]  = p_YAddressKnot[YKnotStart  ];
        LineY[1]  = p_YAddressKnot[YKnotStart+1];           /* Next to LineY[0] */

        /* Calculate defocus value of each knot point */
        PlaneZ[0] = calc_defocus_formula ( pfa_InputData, Index            );
        PlaneZ[1] = calc_defocus_formula ( pfa_InputData, Index+1          );
        PlaneZ[2] = calc_defocus_formula ( pfa_InputData, Index+XKnotNum   );
        PlaneZ[3] = calc_defocus_formula ( pfa_InputData, Index+XKnotNum+1 );

        PointX    = XAddressPDAFWindowCenter;
        PointY    = YAddressPDAFWindowCenter;

        /* Calculate coordination at the point of the plane */
        CalcAddressOnPlane_slXslYslZ (LineX, LineY, PlaneZ, PointX, PointY, &PointZ);

        Defocus = PointZ;
    } else if ( AreaIndex == 0 || AreaIndex == 2 || AreaIndex == 6 || AreaIndex == 8 ) {    /* Cornar of area */
        unsigned short Index;
        
             if ( AreaIndex == 2 ) { Index = XKnotNum-1; }
        else if ( AreaIndex == 6 ) { Index = (YKnotNum-1)*XKnotNum; }
        else if ( AreaIndex == 8 ) { Index = YKnotNum*XKnotNum-1; }
        else                       { Index = 0; }

        /* Calculate defocus value which uses slope and offset of index point */
        Defocus = calc_defocus_formula ( pfa_InputData, Index ); 

    } else if ( AreaIndex == 1 ) {                          /* Top Center */
        unsigned short  Index;
        signed long     LineX[2];
        signed long     LineY[2];
        signed long     PointX;
        signed long     PointY;

        Index = XKnotStart;

        LineX[0] = p_XAddressKnot[XKnotStart  ];
        LineX[1] = p_XAddressKnot[XKnotStart+1];            /* Next to LineX[0] */

        /* Calculate defocus value of each knot point */
        LineY[0] = calc_defocus_formula ( pfa_InputData, Index   );
        LineY[1] = calc_defocus_formula ( pfa_InputData, Index+1 ); /* Next to LineY[0] */

        PointX   = XAddressPDAFWindowCenter;
        /* Calculate coordination at the point of the line */
        CalcAddressOnLine_slXslY ( LineX, LineY, PointX, &PointY );

        Defocus = PointY;
    } else if ( AreaIndex == 7 ) {                          /* Bottom Center */
        unsigned short  Index;
        signed long     LineX[2];
        signed long     LineY[2];
        signed long     PointX;
        signed long     PointY;

        Index = (YKnotNum-1)*XKnotNum + XKnotStart;

        LineX[0] = p_XAddressKnot[XKnotStart  ];
        LineX[1] = p_XAddressKnot[XKnotStart+1];            /* Next to LineX[0] */

        /* Calculate defocus value of each knot point */
        LineY[0] = calc_defocus_formula ( pfa_InputData, Index   );
        LineY[1] = calc_defocus_formula ( pfa_InputData, Index+1 ); /* Next to LineY[0] */

        PointX   = XAddressPDAFWindowCenter;

        /* Calculate coordination at the point of the line */
        CalcAddressOnLine_slXslY ( LineX, LineY, PointX, &PointY );

        Defocus = PointY;
    } else if ( AreaIndex == 3 ) {                          /* Center Left */
        unsigned short  Index;
        signed long     LineX[2];
        signed long     LineY[2];
        signed long     PointX;
        signed long     PointY;

        Index = YKnotStart*XKnotNum;

        LineX[0] = p_YAddressKnot[YKnotStart  ];
        LineX[1] = p_YAddressKnot[YKnotStart+1];            /* Next to LineX[0] */

        /* Calculate defocus value of each knot point */
        LineY[0] = calc_defocus_formula ( pfa_InputData, Index   );
        LineY[1] = calc_defocus_formula ( pfa_InputData, Index+XKnotNum );  /* Next to LineY[0] */

        PointX   = YAddressPDAFWindowCenter;

        /* Calculate coordination at the point of the line */
        CalcAddressOnLine_slXslY ( LineX, LineY, PointX, &PointY );

        Defocus = PointY;
    } else {                                                /* Center Right(5) */
        unsigned short  Index;
        signed long     LineX[2];
        signed long     LineY[2];
        signed long     PointX;
        signed long     PointY;

        Index = (YKnotStart+1)*XKnotNum-1;

        LineX[0] = p_YAddressKnot[YKnotStart  ];
        LineX[1] = p_YAddressKnot[YKnotStart+1];            /* Next to LineX[0] */

        /* Calculate defocus value of each knot point */
        LineY[0] = calc_defocus_formula ( pfa_InputData, Index   );
        LineY[1] = calc_defocus_formula ( pfa_InputData, Index+XKnotNum );  /* Next to LineY[0] */

        PointX   = YAddressPDAFWindowCenter;

        /* Calculate coordination at the point of the line */
        CalcAddressOnLine_slXslY ( LineX, LineY, PointX, &PointY );

        Defocus = PointY;
    }

    (*pfa_Defocus) = Defocus;

    return ;
}

/* Function for calculating defocus confidence level */
void job_calc_defocus_confidence_level 
( 
    PdLibInputData_t *pfa_InputData,                        /* Input  : Input data structure */
    unsigned long *pfa_DefocusConfidenceLevel               /* Output : Defocus confidence level */
)
{
    signed long     DefocusOkNgThr;
    unsigned short  XKnotNum;
    unsigned short  YKnotNum;

    XKnotNum = (*pfa_InputData).XKnotNumDefocusOKNG;
    YKnotNum = (*pfa_InputData).YKnotNumDefocusOKNG;
    
    /* Disable of the judge function of confidence in each image area */
    if ( ( XKnotNum == 1 ) && ( YKnotNum == 1 ) ) {
        /* Disable compensation relation with image height. */
        DefocusOkNgThr = calc_defocus_ok_ng_thr ( pfa_InputData, 0);
    } else {
        unsigned short i;    
        unsigned short  *p_XAddressKnot;
        unsigned short  *p_YAddressKnot;
        unsigned short  XKnotStart;
        unsigned short  YKnotStart;
        unsigned char   AreaIndex;
        signed long     XAddressPDAFWindowCenter;
        signed long     YAddressPDAFWindowCenter;

        p_XAddressKnot = (*pfa_InputData).p_XAddressKnotDefocusOKNG;
        p_YAddressKnot = (*pfa_InputData).p_YAddressKnotDefocusOKNG;

        XAddressPDAFWindowCenter = ( (*pfa_InputData).XAddressOfWindowStart + 
                                     (*pfa_InputData).XAddressOfWindowEnd ) / 2;
        YAddressPDAFWindowCenter = ( (*pfa_InputData).YAddressOfWindowStart + 
                                     (*pfa_InputData).YAddressOfWindowEnd ) / 2;
        
        XKnotStart = 0;
        for ( i = 0; i < XKnotNum-1; i++ ) {                /* Check XKnotStart */
            if ( p_XAddressKnot[i] <= XAddressPDAFWindowCenter && 
                 XAddressPDAFWindowCenter <= p_XAddressKnot[i+1] ) {
                XKnotStart = i;
                break ;
            }
        }

        YKnotStart = 0;
        for ( i = 0; i < YKnotNum-1; i++ ) {                /* Check YKnotStar */
            if ( p_YAddressKnot[i] <= YAddressPDAFWindowCenter && 
                 YAddressPDAFWindowCenter <= p_YAddressKnot[i+1] ) {
                YKnotStart = i;
                break ;
            }
        }

    /*

        Divided by area. AreaIndex

        +---+---+---+
        | 0 | 1 | 2 |
        +---+---+---+
        | 3 | 4 | 5 |
        +---+---+---+
        | 6 | 7 | 8 |
        +---+---+---+

        Area 4 is surrounded by the knot points.
        Other area does not have knot points.

    */

        if ( YAddressPDAFWindowCenter < p_YAddressKnot[0] ) {
            /* Top */
                 if ( XAddressPDAFWindowCenter   < p_XAddressKnot[0]        ) {/* Left */   AreaIndex = 0;}
            else if ( p_XAddressKnot[XKnotNum-1] < XAddressPDAFWindowCenter ) {/* Right */  AreaIndex = 2;}
            else                                                              {/* Center */ AreaIndex = 1;}       
        } else if ( p_YAddressKnot[YKnotNum-1] < YAddressPDAFWindowCenter ) {
            /* Bottom */
                 if ( XAddressPDAFWindowCenter   < p_XAddressKnot[0]        ) {/* Left */   AreaIndex = 6;}
            else if ( p_XAddressKnot[XKnotNum-1] < XAddressPDAFWindowCenter ) {/* Right */  AreaIndex = 8;}
            else                                                              {/* Center */ AreaIndex = 7;}       
        } else {
            /* Center */
                 if ( XAddressPDAFWindowCenter   < p_XAddressKnot[0]        ) {/* Left */   AreaIndex = 3;}
            else if ( p_XAddressKnot[XKnotNum-1] < XAddressPDAFWindowCenter ) {/* Right */  AreaIndex = 5;}
            else                                                              {/* Center */ AreaIndex = 4;}
        }

        if ( AreaIndex == 4 ) {                                             /* Center */
            unsigned short  Index;
            signed long     LineX[2];
            signed long     LineY[2];
            signed long     PlaneZ[4];
            signed long     PointX;
            signed long     PointY;
            signed long     PointZ = 0;
            
            Index = YKnotStart*XKnotNum+XKnotStart;         /* Obtain index of knot point */

            LineX[0]  = p_XAddressKnot[XKnotStart  ];
            LineX[1]  = p_XAddressKnot[XKnotStart+1];       /* Next to LineX[0] */
            LineY[0]  = p_YAddressKnot[YKnotStart  ];
            LineY[1]  = p_YAddressKnot[YKnotStart+1];       /* Next to LineY[0] */
            
            /* Calculate threshold of confidence of each knot point */
            PlaneZ[0] = calc_defocus_ok_ng_thr ( pfa_InputData, Index            );
            PlaneZ[1] = calc_defocus_ok_ng_thr ( pfa_InputData, Index+1          );
            PlaneZ[2] = calc_defocus_ok_ng_thr ( pfa_InputData, Index+XKnotNum   );
            PlaneZ[3] = calc_defocus_ok_ng_thr ( pfa_InputData, Index+XKnotNum+1 );

            PointX    = XAddressPDAFWindowCenter;
            PointY    = YAddressPDAFWindowCenter;

            /* Calculate coordination at the point of the plane */
            CalcAddressOnPlane_slXslYslZ (LineX, LineY, PlaneZ, PointX, PointY, &PointZ);

            DefocusOkNgThr = PointZ;
        } else if ( AreaIndex == 0 || AreaIndex == 2 || AreaIndex == 6 || AreaIndex == 8 ) {    /* Cornar of area */
            unsigned short Index;
            
                 if ( AreaIndex == 2 ) { Index = XKnotNum-1; }
            else if ( AreaIndex == 6 ) { Index = (YKnotNum-1)*XKnotNum; }
            else if ( AreaIndex == 8 ) { Index = YKnotNum*XKnotNum-1; }
            else                       { Index = 0; }
            DefocusOkNgThr = calc_defocus_ok_ng_thr ( pfa_InputData, Index );   /* Calculate threshold of confidence */               
        } else if ( AreaIndex == 1 ) {                      /* Top Center */
            unsigned short  Index;
            signed long     LineX[2];
            signed long     LineY[2];
            signed long     PointX;
            signed long     PointY;

            Index = XKnotStart;

            LineX[0] = p_XAddressKnot[XKnotStart  ];
            LineX[1] = p_XAddressKnot[XKnotStart+1];        /* Next to LineX[0] */

            /* Calculate threshold of confidence of each knot point */            
            LineY[0] = calc_defocus_ok_ng_thr ( pfa_InputData, Index   );
            LineY[1] = calc_defocus_ok_ng_thr ( pfa_InputData, Index+1 );   /* Next to LineY[0] */

            PointX   = XAddressPDAFWindowCenter;

            /* Calculate coordination at the point of the line */
            CalcAddressOnLine_slXslY ( LineX, LineY, PointX, &PointY );

            DefocusOkNgThr = PointY;
        } else if ( AreaIndex == 7 ) {                      /* Bottom Center */
            unsigned short  Index;
            signed long     LineX[2];
            signed long     LineY[2];
            signed long     PointX;
            signed long     PointY;

            Index = (YKnotNum-1)*XKnotNum + XKnotStart;

            LineX[0] = p_XAddressKnot[XKnotStart  ];
            LineX[1] = p_XAddressKnot[XKnotStart+1];        /* Next to LineX[0] */

            /* Calculate threshold of confidence of each knot point */
            LineY[0] = calc_defocus_ok_ng_thr ( pfa_InputData, Index   );
            LineY[1] = calc_defocus_ok_ng_thr ( pfa_InputData, Index+1 );   /* Next to LineY[0] */

            PointX   = XAddressPDAFWindowCenter;

            /* Calculate coordination at the point of the line */
            CalcAddressOnLine_slXslY ( LineX, LineY, PointX, &PointY );

            DefocusOkNgThr = PointY;
        } else if ( AreaIndex == 3 ) {                      /* Center Left */
            unsigned short  Index;
            signed long     LineX[2];
            signed long     LineY[2];
            signed long     PointX;
            signed long     PointY;

            Index = YKnotStart*XKnotNum;

            LineX[0] = p_YAddressKnot[YKnotStart  ];
            LineX[1] = p_YAddressKnot[YKnotStart+1];        /* Next to LineX[0] */

            /* Calculate threshold of confidence of each knot point */
            LineY[0] = calc_defocus_ok_ng_thr ( pfa_InputData, Index   );
            LineY[1] = calc_defocus_ok_ng_thr ( pfa_InputData, Index+XKnotNum );    /* Next to LineY[0] */

            PointX   = YAddressPDAFWindowCenter;

            /* Calculate coordination at the point of the line */
            CalcAddressOnLine_slXslY ( LineX, LineY, PointX, &PointY );

            DefocusOkNgThr = PointY;
        } else {                                            /* Center Right(5) */
            unsigned short  Index;
            signed long     LineX[2];
            signed long     LineY[2];
            signed long     PointX;
            signed long     PointY;

            Index = (YKnotStart+1)*XKnotNum-1;

            LineX[0] = p_YAddressKnot[YKnotStart  ];
            LineX[1] = p_YAddressKnot[YKnotStart+1];        /* Next to LineX[0] */

            /* Calculate threshold of confidence of each knot point */
            LineY[0] = calc_defocus_ok_ng_thr ( pfa_InputData, Index   );
            LineY[1] = calc_defocus_ok_ng_thr ( pfa_InputData, Index+XKnotNum );

            PointX   = YAddressPDAFWindowCenter;

            /* Calculate coordination at the point of the line */
            CalcAddressOnLine_slXslY ( LineX, LineY, PointX, &PointY );

            DefocusOkNgThr = PointY;
        }
    }

    if ( DefocusOkNgThr <= 0 ) DefocusOkNgThr = 0;          /* Check DefocusOkNgThr */

    
    if ( DefocusOkNgThr == 0 ) {                            /* If DefocusOkNgThr is Zero */
        (*pfa_DefocusConfidenceLevel) = 1024;               /* Set max value to ConfidenceLevel */
    } else {
        //double ConfidenceLevel;
        //double DensityOfPhasePix;
        //double DefocusConfidenceLevel;
				
        signed long ConfidenceLevel;
        signed long DensityOfPhasePix;
        signed long DefocusConfidenceLevel;

        //ConfidenceLevel   = (double)((*pfa_InputData).ConfidenceLevel);
        ConfidenceLevel   = (signed long)((*pfa_InputData).ConfidenceLevel);
      
        if ( (*pfa_InputData).DensityOfPhasePix == 0 ) {    /* If DensityOfPhasePix is not set */
            DensityOfPhasePix = 2304;      //DensityOfPhasePix = 2304.0;               /* Set default value to DensityOfPhasePix */
        } else {
            //DensityOfPhasePix = (double)((*pfa_InputData).DensityOfPhasePix);
			 DensityOfPhasePix = (signed long)((*pfa_InputData).DensityOfPhasePix);
        }

        /* Calculate defocus confidence level */
        //DefocusConfidenceLevel = 1024.0 * ConfidenceLevel * 2304.0 / DensityOfPhasePix / (double)DefocusOkNgThr;
        DefocusConfidenceLevel = 1024 * ConfidenceLevel * 2304 / DensityOfPhasePix / (signed long)DefocusOkNgThr;

        (*pfa_DefocusConfidenceLevel) = limit_defocus_confidence_level(DefocusConfidenceLevel);
    }

    return ;
}

/* Function for calculating defocus confidence */
void job_calc_defocus_confidence 
( 
    unsigned long fa_DefocusConfidenceLevel,                /* Input  : Defocus confidence level */
    signed char *pfa_DefocusConfidence                      /* Output : Defocus confidence */
)
{
    if ( 1024 <= fa_DefocusConfidenceLevel ) {
        (*pfa_DefocusConfidence) = D_PD_LIB_E_OK;
    } else {
        /* Check the value of input */
        (*pfa_DefocusConfidence) = -ELDCL;                  /* Low DefocusConfidenceLevel */
    }

    return ;
}

/* Function for calculating phase difference */
void job_calc_phase_difference 
( 
    PdLibInputData_t *pfa_InputData,                        /* Input : Input data structure */
    signed long *pfa_PhaseDifference                        /* Output : Phase difference */
)
{
    (*pfa_PhaseDifference) = (*pfa_InputData).PhaseDifference;
    return ;
}

/* Sub function of job_calc_defocus() */
/* Function for calculating defocus value which uses slope and offset of index point */
signed long calc_defocus_formula 
( 
    PdLibInputData_t *pfa_InputData,                        /* Input : Input data structure */
    unsigned short fa_Index                                 /* Input : Index of knot point  */
)
{
    signed long PhaseDifference;
    signed long AdjCoeffSlope;
    long zed;	//double zed;

    PhaseDifference = (*pfa_InputData).PhaseDifference;
    AdjCoeffSlope   = (*pfa_InputData).AdjCoeffSlope;

    //zed = (double)AdjCoeffSlope * (double)((*pfa_InputData).p_SlopeData[fa_Index]) * (double)PhaseDifference / 2304.0 + (double)((*pfa_InputData).p_OffsetData[fa_Index]);
    zed = (signed long)AdjCoeffSlope * (signed long)((*pfa_InputData).p_SlopeData[fa_Index]) * (signed long)PhaseDifference / 2304 + (signed long)((*pfa_InputData).p_OffsetData[fa_Index]);

    return limit_defocus_formula(zed);                        /* Return defocus value with limitation */
}

/* Sub function of calc_defocus_formula() */
/* Function for Limiting defocus value as  0x80000000 - 0x7FFFFFFF */
signed long limit_defocus_formula 
( 
    signed long fa_Defocus         //double fa_Defocus                               /* Input : Defocus */
)
{
    signed long ret;

    /*  0x80000000 - 0x7FFFFFFF */
    /* -2147483648 - +2147483647 */
    /* -2147483647 - +2147483646 */

    if ( fa_Defocus <= -2147483647 ) {
        ret = -2147483647;                                  /* Limit min */
    } else  if ( 2147483646 <= fa_Defocus ) {
        ret = 2147483646;                                   /* Limit max */
    } else {
        ret = (signed long)fa_Defocus;
    }

    return ret;                                             /* Return limited value */
}

/* Sub function of job_calc_defocus_confidence_level() */
/* Function for calculating threshold of confidence */
signed long calc_defocus_ok_ng_thr 
( 
    PdLibInputData_t *pfa_InputData,                        /* Input : Input data structure */
    unsigned short fa_Index                                 /* Input : Index of knot point  */
)
{

    unsigned long *LineX;
    unsigned long *LineY;
    unsigned long PointNum;
    unsigned long PointX;
    unsigned long PointY = 0;

    LineX    = (*pfa_InputData).p_DefocusOKNGThrLine[fa_Index].p_AnalogGain;
    LineY    = (*pfa_InputData).p_DefocusOKNGThrLine[fa_Index].p_Confidence;
    PointNum = (*pfa_InputData).p_DefocusOKNGThrLine[fa_Index].PointNum;
    PointX   = (*pfa_InputData).ImagerAnalogGain;

    /* Calculate coordination at the point of the broken line */
    CalcAddressOnBrokenLine_ulXulY ( LineX, LineY, PointNum, PointX, &PointY );

    return (signed long)PointY;                             /* return threshold of confidence */
}

/* Sub function of job_calc_defocus_confidence_level() */
/* Function for Limiting defocus confidence level */
unsigned long limit_defocus_confidence_level 
( 
    signed long fa_DefocusConfidenceLevel//	double fa_DefocusConfidenceLevel                        /* Input : Defocus confidence level */
)
{
    unsigned long ret;

    /*  0x00000000 - 0xFFFFFFFF */
    /*           0 - 4294967295 */
    /*           0 - 4294967294 */

    if ( fa_DefocusConfidenceLevel <= 0 ) {               /* limit min */
        ret = 0;
    } else if ( 4294967294 <= fa_DefocusConfidenceLevel ) {  /* limit max */
        /* ret = 4294967294(0xFFFFFFFE) */
        ret = 0xFFFFFFFE;
    } else {
        ret = (unsigned long)fa_DefocusConfidenceLevel;
    }

    return ret;                                             /* Return defocus confidence level */
}

/******************************************************************************************/

#endif

static inline void imx586_get_frame_length_regs(imx586_reg *regs,
				u32 frame_length)
{
	//regs->addr = IMX586_FRAME_LENGTH_ADDR_MSB;
	//regs->val = (frame_length >> 16) & 0x01;

	//(regs + 1)->addr = IMX586_FRAME_LENGTH_ADDR_LSB;
	//(regs + 1)->val = (frame_length) & 0xff;

	regs->addr = IMX586_FRAME_LENGTH_ADDR_MSB;
	regs->val = ((frame_length >> 8) & 0xff);
	(regs + 1)->addr = IMX586_FRAME_LENGTH_ADDR_LSB;
	(regs + 1)->val = ((frame_length) & 0xff);

	//printk("%s frame_length:%d \n",__func__, frame_length);
}

static inline void imx586_get_integ_coarse_time_regs(imx586_reg *regs,
				u32 coarse_time)
{
	printk("%s, %d\n",__func__,coarse_time);

	regs->addr = IMX586_COARSE_INTEG_TIME_MSB;
	regs->val = ((coarse_time >> 8) & 0xff);

	(regs + 1)->addr = IMX586_COARSE_INTEG_TIME_LSB;
	(regs + 1)->val = (coarse_time & 0xff);
}

static inline void imx586_get_coarse_time_regs_hdr(imx586_reg *regs,
				u32 coarse_time)
{
	printk("%s, %d\n",__func__,coarse_time);

	regs->addr = IMX586_ST_COARSE_TIME_MSB;
	regs->val = ((coarse_time >> 8) & 0xff);
	//printk("%s\n",__func__);

	(regs + 1)->addr = IMX586_ST_COARSE_TIME_LSB;
	(regs + 1)->val = (coarse_time & 0xff);
}

static inline void imx586_get_gain_reg(imx586_reg *regs,
				u16 gain)
{
#if 0
	regs->addr = IMX586_GAIN_ADDR;
	regs->val = (gain) & 0xff;
#endif
	//printk("%s: anlog gain:%d\n",__func__, gain);

	regs->addr = IMX586_ANA_GAIN_GLOBAL_ADDR_MSB;
	regs->val = ((gain>>8) & 0x03);	//regs->val = (gain>>8) & 0xff;

	(regs + 1)->addr = IMX586_ANA_GAIN_GLOBAL_ADDR_LSB;
	(regs + 1)->val = (gain & 0xff);
}


static int test_mode;
module_param(test_mode, int, 0644);

static inline int imx586_read_reg(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	int err = 0;
	u32 reg_val = 0;
	
	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xFF;

	//printk("%s: i2c read reg, addr: 0x%04x, reg_val: %02x, err: %d\n",
	//		__func__,  addr,  *val, err);

	return err;
}

static inline int imx586_read_reg2(struct camera_common_data *s_data,
				u16 addr, s16 *val)
{
	int err = 0, reg_val = 0;
	
	err = regmap_read(s_data->regmap, addr, &reg_val);
	//err = volatile_reg(s_data->regmap, addr, &reg_val);
	
	*val = (reg_val & 0xFF);

	printk("%s: i2c read reg2, addr: 0x%04x, reg_val: 0x%02x, src_val: 0x%02x, err: %d\n",
			__func__,  addr,  *val, reg_val, err);

	return err;
}

#if (PDAF_SOLUTION_1 == PDAF_SOLUTION || PDAF_SOLUTION_2 == PDAF_SOLUTION)
static int imx586_read_reg3(struct imx586 *priv, u16 reg, u8 *values)
{
	struct i2c_client *client = priv->i2c_client;
	u8 addr[2] = { reg >> 8, reg & 0xff };
	int err;
	
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = addr,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = values,
		},
	};

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err != ARRAY_SIZE(msgs)) {
		printk("%s: reading register 0x%x from 0x%x failed\n",
			__func__, reg, client->addr);

		return -1;
	}

	//printk("%s: i2c read reg3, addr: 0x%04x, reg_val: 0x%02x, err: %d\n",
	//		__func__,  reg,  values[0], (err == ARRAY_SIZE(msgs))?0: -1);

	return 0;
}
#endif

static int imx586_write_reg(struct camera_common_data *s_data,
				u16 addr, u8 val)
{
	int err;
	struct device *dev = s_data->dev;
	
	//printk("%s: i2c read , addr: 0x%04x, reg_val: %02x\n",__func__,  addr,  val);

	err = regmap_write(s_data->regmap, addr, val);
	if (err)
		dev_err(dev, "%s: i2c write reg failed, addr: 0x%04x, val: %02x, err: %d\n",
			__func__, addr, val, err);

	return err;
}

static int imx586_write_table(struct imx586 *priv,
				const imx586_reg table[])
{
	struct camera_common_data *s_data = priv->s_data;
	// print table
	int i, err; 
	u8 val;
	
	i = 0;
	val = 0;

#if 0
	printk("\n");
	while( table[i].addr != IMX586_TABLE_END ){
		printk("%s: write table, addr:0x%04x, value:0x%02x \n",__func__, table[i].addr, table[i].val);
	}
	printk("\n");

#endif

	err =  regmap_util_write_table_8(s_data->regmap,
					 table,
					 NULL, 0,
					 IMX586_TABLE_WAIT_MS,
					 IMX586_TABLE_END);
	if(err != 0)
	{
		printk("%s, regmap_util_write_table_8 error: %d\n",__func__, err);
		return err;
	}

#if 0
	printk("\n");
	while( table[i].addr != IMX586_TABLE_END ){
		err = imx586_read_reg(s_data, table[i].addr, &val);
		printk("%s: read table, addr: 0x%0x, write: 0x%02x, read: 0x%02x, ret: %d\n",__func__, table[i].addr, table[i].val, val, err);
		if(table[i].val != val)
		{
			printk("%s : different val ,addr= 0x%x, table.val = 0x%x, val = 0x%x\n", __func__, table[i].addr, table[i].val, val);
		}
		i++;
	}
	printk("\n");
#endif

	return 0;
}

void imx586_pdaf_read_tablereg(struct imx586 *priv, const imx586_reg table[])
{
	int i, err; 
	u8 val;
	
	i = 0;
	printk("\n\nread sensor reg:\n");
	while( table[i].addr != IMX586_TABLE_END ){
		err = imx586_read_reg(priv->s_data, table[i].addr, &val);
		printk("%s: read table, addr: 0x%04x, write: 0x%02x, read: 0x%02x, ret: %d\n",__func__, table[i].addr, table[i].val, val, err);
		if(table[i].val != val)
		{
			printk("%s : different val, addr: 0x%04x, table.val: 0x%02x, val: 0x%02x\n", __func__, table[i].addr, table[i].val, val);
		}
		i++;
	}
	printk("\n\n");	
}

#ifdef _I2C_DEB_
static int check_reg(struct camera_common_data *s_data, u16 addr)
{
        u8 reg_val;
        int err = 0;
        err = imx586_read_reg(s_data, addr, &reg_val);
        //printk("%s: check reg, addr: 0x%04x, val: 0x%02x, err = %d.\n", __func__, addr, reg_val, err);
        return err;
}
#endif

#if (PDAF_SOLUTION_NO != PDAF_SOLUTION)	
static int imx586_read_devreg(struct regmap *map,
				u16 addr, s16 *val)
{
	int err = 0, reg_val = 0;
	
	err = regmap_read(map, addr, &reg_val);
	*val = reg_val & 0xFF;

	//printk("%s: i2c read devreg, addr: 0x%04x, reg_val: %02x, err: %d\n",
	//		__func__,  addr,  *val, err);

	return err;
}

static int imx586_write_devreg(struct regmap *map,
				u16 addr, s16 val)
{
	int err;

	err = regmap_write(map, addr, val);
	if (err)
		printk("%s: i2c write devreg failed, addr: 0x%04x, val: %02x, err: %d\n",
			__func__, addr, val, err);

	return err;
}
#endif

#if (PDAF_SOLUTION_NO != PDAF_SOLUTION)
static int imx586_focus_device_release(struct imx586 *priv)
{	
	//printk("%s: motor_release\n", __func__);

	if (priv->focusmotor.i2c_client != NULL) {
		i2c_unregister_device(priv->focusmotor.i2c_client);
		priv->focusmotor.i2c_client = NULL;
	}

	return 0;
}

static int imx586_focus_device_init(struct imx586 *priv)
{
	int err;
	char *dev_name = "focus_imx586";
	static struct regmap_config motor_regmap_config = {
		.reg_bits = 8,
		.val_bits = 8,
        //.cache_type = REGCACHE_NONE,
	};
		
	printk("%s: focus_init start !!------------------\n", __func__);
			
	priv->focusmotor.adap = i2c_get_adapter(priv->i2c_client->adapter->nr);
	
	memset(&priv->focusmotor.brd, 0, sizeof(priv->focusmotor.brd));
	strncpy(priv->focusmotor.brd.type, dev_name, sizeof(priv->focusmotor.brd.type));

	priv->focusmotor.brd.addr = IMX586_MOTOR_ADDRESS;
	
	priv->focusmotor.i2c_client = i2c_new_device(priv->focusmotor.adap, &priv->focusmotor.brd);

	priv->focusmotor.regmap = devm_regmap_init_i2c(priv->focusmotor.i2c_client, &motor_regmap_config);

	if (IS_ERR(priv->focusmotor.regmap)) {
		err = PTR_ERR(priv->focusmotor.regmap);
		imx586_focus_device_release(priv);

		printk("%s: focus_init error: %d\n", __func__, err);

		return err;
	}

	printk("%s: focus_init end, i2c addr: 0x%x !!------------------\n", __func__, priv->focusmotor.i2c_client->addr);	
	
	return 0;
}

static void imx586_focus_init(struct imx586 *priv)
{
	u16 regaddr = 0x02;
	s16  value = 0;
	int err = 0;
	
	err = imx586_write_devreg(priv->focusmotor.regmap, regaddr, value);
	printk("%s: write_devreg(regaddr: 0x%04x, value: %d, err:%d)\n", __func__, regaddr, value, err);
}

static void imx586_focus_set_position(struct imx586 *priv, u16 pos)
{
	u16 addr1 = 0x00, addr2 = 0x01;
	u16  value = pos;

	if (IMX586_MOTOR_STEPS_MICRO < pos)
		value = IMX586_MOTOR_STEPS_MICRO;

	if ((priv->cur_pos != value) && (1 < abs(priv->cur_pos - value)))
	{
		imx586_write_devreg(priv->focusmotor.regmap, addr1, ((value >> 2) & 0xff));	// high
		imx586_write_devreg(priv->focusmotor.regmap, addr2, ((value << 6) & 0xff));	// low

		printk("%s: setpos:%d, set cur value:%d, old cur pos: %d)\n", __func__, pos, value, priv->cur_pos);

		priv->cur_pos = value;
	}
}

static int imx586_eeprom_device_release(struct imx586 *priv)
{	
	//printk("%s: eeprom_release\n", __func__);

	if (priv->eeprom.i2c_client != NULL) {
		i2c_unregister_device(priv->eeprom.i2c_client);
		priv->eeprom.i2c_client = NULL;
	}

	return 0;
}

static int imx586_eeprom_device_init(struct imx586 *priv)
{
	int err;
	char *dev_name = "eeprom_imx586";
	static struct regmap_config eeprom_regmap_config = {
		.reg_bits = 16,
		.val_bits = 8,
       //.cache_type = REGCACHE_NONE,
	};
	
	u16 regaddr = 0;
	s16 val1 = 0, val2 = 0;
	
	printk("%s: eeprom_init start !!------------------\n", __func__);
			
	priv->eeprom.adap = i2c_get_adapter(priv->i2c_client->adapter->nr);
	
	memset(&priv->eeprom.brd, 0, sizeof(priv->eeprom.brd));
	strncpy(priv->eeprom.brd.type, dev_name,sizeof(priv->eeprom.brd.type));

	priv->eeprom.brd.addr = IMX586_EEPROM_ADDRESS;
	
	priv->eeprom.i2c_client = i2c_new_device(priv->eeprom.adap, &priv->eeprom.brd);

	priv->eeprom.regmap = devm_regmap_init_i2c(priv->eeprom.i2c_client, &eeprom_regmap_config);

	if (IS_ERR(priv->eeprom.regmap)) {
		err = PTR_ERR(priv->eeprom.regmap);
		imx586_eeprom_device_release(priv);

		printk("%s: eeprom_init error: %d\n", __func__, err);

		return err;
	}

	printk("%s: eeprom_init end, i2c addr: 0x%x !!------------------\n", __func__, priv->eeprom.i2c_client->addr);

	regaddr = IMX586_EEPROM_REGADDR_MODINFO;
	err = imx586_read_devreg(priv->eeprom.regmap, regaddr, &val1);	
	printk("%s (Module information Flag, regaddr: 0x%04x, val: %d, ret: %d)\n", __func__, 
				IMX586_EEPROM_REGADDR_MODINFO, val1, err);

	regaddr = IMX586_EEPROM_REGADDR_MODID;
	err = imx586_read_devreg(priv->eeprom.regmap, regaddr, &val1);	
	printk("%s (Module ID, regaddr: 0x%04x, val: %d, ret: %d)\n", __func__, 
				IMX586_EEPROM_REGADDR_MODID, val1, err);

	regaddr = IMX586_EEPROM_REGADDR_SENSORID;
	err = imx586_read_devreg(priv->eeprom.regmap, regaddr, &val1);	
	printk("%s (Sensor ID, regaddr: 0x%04x, val: %d, ret: %d)\n", __func__, 
				IMX586_EEPROM_REGADDR_SENSORID, val1, err);

	regaddr = IMX586_EEPROM_REGADDR_AF_FLAG;
	err = imx586_read_devreg(priv->eeprom.regmap, regaddr, &val1);	
	printk("%s (Flag of AF, regaddr: 0x%04x, val: %d, ret: %d)\n", __func__, 
				IMX586_EEPROM_REGADDR_AF_FLAG, val1, err);

	regaddr = IMX586_EEPROM_REGADDR_CALI_DIR;
	err = imx586_read_devreg(priv->eeprom.regmap, regaddr, &val1);	
	printk("%s (AF Calibration Direction, regaddr: 0x%04x, val: %d, ret: %d)\n", __func__, 
				IMX586_EEPROM_REGADDR_CALI_DIR, val1, err);
	
	regaddr = IMX586_EEPROM_REGADDR_10M_H;
	err = imx586_read_devreg(priv->eeprom.regmap, regaddr, &val1);	
	printk("%s (AF Calibration Direction, regaddr: 0x%04x, val: %d, ret: %d)\n", __func__, 
				IMX586_EEPROM_REGADDR_10M_H, val1, err);

	regaddr = IMX586_EEPROM_REGADDR_10M_L;
	err = imx586_read_devreg(priv->eeprom.regmap, regaddr, &val2);
	printk("%s (AF Calibration Direction, regaddr: 0x%04x, val: %d, ret: %d)\n", __func__, 
				IMX586_EEPROM_REGADDR_10M_L, val1, err);

	priv->dis_infinity = (val1 <<8) |val2;
	printk("%s: 10m (regaddr: 0x%04x 0x%04x, dis_infinity: %d, val1: %d, val2: %d)\n", __func__, 
				IMX586_EEPROM_REGADDR_10M_H, IMX586_EEPROM_REGADDR_10M_L, priv->dis_infinity, val1, val2);

	regaddr = IMX586_EEPROM_REGADDR_10CM_H;
	err = imx586_read_devreg(priv->eeprom.regmap, regaddr, &val1);
	printk("%s (AF Calibration Direction, regaddr: 0x%04x, val: %d, ret: %d)\n", __func__, 
				IMX586_EEPROM_REGADDR_10CM_H, val1, err);
	
	regaddr = IMX586_EEPROM_REGADDR_10CM_L;
	err = imx586_read_devreg(priv->eeprom.regmap, regaddr, &val2);
	printk("%s (AF Calibration Direction, regaddr: 0x%04x, val: %d, ret: %d)\n", __func__, 
				IMX586_EEPROM_REGADDR_10CM_L, val1, err);

	priv->dis_micro = (val1 <<8) |val2;
	printk("%s: 10cm (regaddr: 0x%04x 0x%04x, dis_micro: %d, val1: %d, val2: %d)\n", __func__, 
				IMX586_EEPROM_REGADDR_10CM_H, IMX586_EEPROM_REGADDR_10CM_L, priv->dis_micro, val1, val2);
	
	return 0;
}
#endif

#if (1 == IMX586_CHECK_TRACKID)
static int imx586_read_trackid(struct imx586 *priv, u16 reg, u8 *values)
{
	struct i2c_client *client = priv->trackid.i2c_client;
	u8 addr = reg;
	int err;
	
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = values,
		},
	};

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err != ARRAY_SIZE(msgs)) {
		printk("%s: reading register 0x%x from 0x%x failed\n",
			__func__, reg, client->addr);

		return -1;
	}

	//printk("%s: i2c read reg3, addr: 0x%04x, reg_val: 0x%02x, err: %d\n",
	//		__func__,  reg,  values[0], (err == ARRAY_SIZE(msgs))?0: -1);

	return 0;
}

static int imx586_trackid_release(struct imx586 *priv)
{	
	//printk("%s: trackid_release\n", __func__);

	if (priv->trackid.i2c_client != NULL) {
		i2c_unregister_device(priv->trackid.i2c_client);
		priv->trackid.i2c_client = NULL;
	}

	return 0;
}

static int imx586_trackid_init(struct imx586 *priv)
{
	int err;
	char *dev_name = "trackid_imx586";
	static struct regmap_config trackid_regmap_config = {
		.reg_bits = 8,
		.val_bits = 8,
        //.cache_type = REGCACHE_NONE,
	};
		
	printk("%s: trackid_init start !!------------------\n", __func__);
			
	priv->trackid.adap = i2c_get_adapter(0);
	
	memset(&priv->trackid.brd, 0, sizeof(priv->trackid.brd));
	strncpy(priv->trackid.brd.type, dev_name, sizeof(priv->trackid.brd.type));

	priv->trackid.brd.addr = 0x50;
	
	priv->trackid.i2c_client = i2c_new_device(priv->trackid.adap, &priv->trackid.brd);

	priv->trackid.regmap = devm_regmap_init_i2c(priv->trackid.i2c_client, &trackid_regmap_config);

	if (IS_ERR(priv->trackid.regmap)) {
		err = PTR_ERR(priv->trackid.regmap);
		imx586_trackid_release(priv);

		printk("%s: ftrackid_init error: %d\n", __func__, err);

		return err;
	}

	printk("%s: trackid_init end, i2c addr: 0x%x !!------------------\n", __func__, priv->trackid.i2c_client->addr);	
	
	return 0;
}

static int check_trackid(struct imx586 *priv)
{
	u16 tempindex = 0, reg_addr = 0;
	char tempbuf[2] = {0};
	char idbuffer[16] = {0};

	for (reg_addr = ASSET_TRACKING_ID_ADDR_START; reg_addr < ASSET_TRACKING_ID_ADDR_END + 1; reg_addr++)
	{
		imx586_read_trackid(priv, reg_addr, tempbuf);
		if (tempbuf[0] != 0)
		{
			idbuffer[tempindex] = (tempbuf[0] - 48);

			if (trackid[tempindex] != (tempbuf[0] - 48))
				return -1;
		}

		//printk("%s, data[0x%02x]=%d\n",__func__, reg_addr, idbuffer[tempindex]);

		tempindex++;
	}

	imx586_trackid_release(priv);
	
	return 0;
}
#endif

#if (PDAF_SOLUTION_1 == PDAF_SOLUTION)
static void imx586_eeprom_get_dcc(struct imx586 *priv)
{
	u8 i = 0;
	s16 val1 = 0, val2 = 0;
	u16 addr = 0;
	int err;

	printk("%s get dcc from eeprom\n", __func__);

	for (i = 0; i < IMX586_EEPROM_DCC_NUMBER/2; i++)
	{
		addr = IMX586_EEPROM_REGADDR_DCC_START + 2 * i;
		
		err = imx586_read_devreg(priv->eeprom.regmap, addr, &val1);
		err = imx586_read_devreg(priv->eeprom.regmap, addr + 1, &val2);

		eeprom_dcc[i] = ((val2 << 8) | val1);
			
		//printk("%s: read_devreg %d (regaddr: 0x%04x, value: %ld, val1: %d, val2:%d)\n", __func__, i, addr, eeprom_dcc[i], val1, val2);
	}
}
#endif

#if (1 == QSC_EANBLE)
static void imx586_eeprom_to_sensor_qsc(struct imx586 *priv)
{
	s16 val = 0;
	u16 i = 0, addr1 = 0, addr2 = 0;
	int err;

	err = imx586_write_reg(priv->s_data, IMX586_SENSOR_QSC_EN, 1);	
	printk("%s: read reg from eeprom, Flag of qsc, addr: 0x%04x, val:%d, ret: %d\n", __func__,IMX586_SENSOR_QSC_EN, 1, err);
	
	for (i = 0; i < IMX586_EEPROM_QSC_NUM; i++)
	{
		addr1 = IMX586_EEPROM_QSC_ADDR_START + i;
		err = imx586_read_devreg(priv->eeprom.regmap, addr1, &val);
		//printk("%s: %d read reg from eeprom (regaddr: 0x%04x, value: %d, err: %d)\n", __func__, i, addr1, val, err);

		addr2 = IMX586_SENSOR_QSC_ADDR_START + i;
		err = imx586_write_reg(priv->s_data, addr2, val & 0xff);
		//printk("%s: %d write reg to sonsor (regaddr: 0x%04x, value: %d, err: %d)\n", __func__, i, addr2, val, err);
	}
}
#endif

#if (PDAF_SOLUTION_1 == PDAF_SOLUTION || PDAF_SOLUTION_3 == PDAF_SOLUTION)
static void imx586_eeprom_to_sensor_lrc(struct imx586 *priv)
{
	s16 val = 0;
	u16 i = 0, addr1 = 0, addr2 = 0;
	int err;

	addr1 = IMX586_EEPROM_REGADDR_LRC_FLAG;
	
	#if (PDAF_SOLUTION_1 == PDAF_SOLUTION)
	err = imx586_read_devreg(priv->eeprom.regmap, addr1, &val);
	printk("%s: read reg from eeprom, Flag of SONY PDAF and SPC Calibration, addr: 0x%04x, val:%d, ret: %d\n", __func__,addr1, val, err);
	#endif
	
	for (i = 0; i < IMX586_PDAF_LRC_NUMBER; i++)
	{
		if (IMX586_PDAF_LRC_NUMBER/2 > i)
		{
			addr1 = IMX586_EEPROM_REGADDR_LRC_LEFT_START + i;
			addr2 = IMX586_PDAF_REGADDR_LRC_LEFT_START + i;

			#if (PDAF_SOLUTION_3 == PDAF_SOLUTION)
			val = (lrc_table0[i] & 0xff);
			#endif
		}
		else
		{
			addr1 = IMX586_EEPROM_REGADDR_LRC_RIGHT_START + i - IMX586_PDAF_LRC_NUMBER/2;
			addr2 = IMX586_PDAF_REGADDR_LRC_RIGHT_START + i - IMX586_PDAF_LRC_NUMBER/2;

			#if (PDAF_SOLUTION_3 == PDAF_SOLUTION)
			val = (lrc_table1[i - IMX586_PDAF_LRC_NUMBER/2] & 0xff);
			#endif
		}			

		#if (PDAF_SOLUTION_1 == PDAF_SOLUTION)
		err = imx586_read_devreg(priv->eeprom.regmap, addr1, &val);
		//printk("%s: %d read reg from eeprom (regaddr: 0x%04x, value: %d, err: %d)\n", __func__, i, addr1, val, err);
		#endif
		
		err = imx586_write_reg(priv->s_data, addr2, val);
		//printk("%s: %d write reg to sonsor (regaddr: 0x%04x, value: %d, err: %d)\n", __func__, i, addr2, val, err);
	}

	err = imx586_write_reg(priv->s_data, IMX586_PDAF_CTRL1, IMX586_VAL_ENABLE);	
	printk("%s: write reg to sonsor (regaddr: 0x%04x, value: %d, err: %d)\n", __func__, IMX586_PDAF_CTRL1, IMX586_VAL_ENABLE, err);
}
#endif


#if (PDAF_SOLUTION_2 == PDAF_SOLUTION)
static void imx586_pdaf_set_dcc(struct imx586 *priv, bool isstreaming)
{
	int err = 0;

	printk("%s: isstreaming: %d\n", __func__, isstreaming);

	// hold
	if (true == isstreaming)
		err = imx586_write_reg(priv->s_data, IMX586_GROUP_HOLD_ADDR, IMX586_VAL_ENABLE);

	// set dcc
	err = imx586_write_table(priv, fixedarea_dcc);
	
	// release hold
	if (true == isstreaming)
		err = imx586_write_reg(priv->s_data, IMX586_GROUP_HOLD_ADDR, IMX586_VAL_DISABLE);
}
#endif

#if (PDAF_SOLUTION_NO != PDAF_SOLUTION)
static void imx586_pdaf_set_area(struct imx586 *priv, bool isstreaming)
{
	int err = 0;
	u8  value = IMX586_PDAF_AREA_MODE2;

#if (PDAF_SOLUTION_2 == PDAF_SOLUTION || PDAF_SOLUTION_3 == PDAF_SOLUTION)
	value = IMX586_PDAF_AREA_MODE1;
#elif (PDAF_SOLUTION_1 == PDAF_SOLUTION)
	int xstart = 0, ystart = 0;
	int width = 0, height = 0;

	width = priv->s_data->fmt_width;
	height = priv->s_data->fmt_height;

	xstart = (width - width/IMX586_PDAF_AREA_WINDOW_RATI0)/2;
	ystart = (height - height/IMX586_PDAF_AREA_WINDOW_RATI0)/2;
	
	center_area[0].val = ((xstart >> 8) & 0x1f);
	center_area[1].val = (xstart & 0xff);
	center_area[2].val = ((ystart >> 8) & 0x1f);
	center_area[3].val = (ystart & 0xff);

	center_area[4].val = (((xstart + width/IMX586_PDAF_AREA_WINDOW_RATI0) >> 8) & 0x1f);
	center_area[5].val = ((xstart +width/IMX586_PDAF_AREA_WINDOW_RATI0) & 0xff);
	center_area[6].val = (((ystart + height/IMX586_PDAF_AREA_WINDOW_RATI0) >> 8) & 0x1f);
	center_area[7].val = ((ystart + height/IMX586_PDAF_AREA_WINDOW_RATI0) & 0xff);

	printk("%s: area (mode_prop_idx:%d, width: %d, height: %d, xstart: %d, ystart: %d, xend: %d, yend: %d)\n", __func__, 
				priv->s_data->mode_prop_idx,width, height, xstart, ystart, xstart + width/IMX586_PDAF_AREA_WINDOW_RATI0,
				ystart + height/IMX586_PDAF_AREA_WINDOW_RATI0);
#endif

	err = imx586_write_reg(priv->s_data, IMX586_PDAF_AREA_MODE, value);	
	printk("%s: imx586_write_reg (regaddr: 0x%04x, value: %d, err: %d)\n", __func__, 
				IMX586_PDAF_AREA_MODE, value, err);

	#if (PDAF_SOLUTION_1 == PDAF_SOLUTION)
	value = 01;
	err = imx586_write_reg(priv->s_data, IMX586_PDAF_AREA_EN_0, value);	
	printk("%s: imx586_write_reg (regaddr: 0x%04x, value: %d, err: %d)\n", __func__, 
				IMX586_PDAF_AREA_EN_0, value, err);
	#endif
	
	// hold
	if (true == isstreaming)
		err = imx586_write_reg(priv->s_data, IMX586_GROUP_HOLD_ADDR, IMX586_VAL_ENABLE);

	// set area
	err = imx586_write_table(priv, center_area);
	
	// release hold
	if (true == isstreaming)
		err = imx586_write_reg(priv->s_data, IMX586_GROUP_HOLD_ADDR, IMX586_VAL_DISABLE);
	
}
#endif

#if (PDAF_SOLUTION_1 == PDAF_SOLUTION || PDAF_SOLUTION_2 == PDAF_SOLUTION)
static void imx586_pdaf_get_pd_cl(struct imx586 *priv,unsigned long *cl, signed long *pd)
{
	u8 val1, val2, val3;
	int err = 0;
	signed long temp = 0, temp2 = 0;

	priv->readingpd = true;

	// hold on
    err = imx586_write_reg(priv->s_data, IMX586_TABLE_HOLD, 1);

    err = imx586_read_reg3(priv, IMX586_PD_CONFIDENCE_LEVEL_H, &val1);
    err = imx586_read_reg3(priv, IMX586_PD_CONFIDENCE_LEVEL_L, &val2);
    err = imx586_read_reg3(priv, IMX586_PD_PHASE_DIFFERENCE_L, &val3);

	// hold release
    err = imx586_write_reg(priv->s_data, IMX586_TABLE_HOLD, 0);

	priv->readingpd = false;

	//*cl = (val1 << 3) | (val2 >> 5);
	//temp = (((val2 & 0x1f ) << 6) | (val3 >> 2)) - 1;

	temp2 = (val1 << 3);
	temp2 |=  (val2 >> 5);
	*cl = temp2;
		
	temp = ((val2 & 0x1f ) << 6);
	temp |= (val3 >> 2);
	temp -= 1;

	//s6.4
	if ((temp >> 4) > 63)
		*pd = 0 - (0x7ff - temp);
	else
		*pd = temp;
	
	//printk("%s: addr start: 0x%04X, val1: %d, val2: %d, val3: %d, src pd:%ld, chg pd:%ld, cl: %ld, cl_valid_min: %d\n",__func__, 
	//			IMX586_PD_CONFIDENCE_LEVEL_H, val1, val2, val3, temp,
	//			*pd, *cl, IMX586_PD_CONFIDENCE_LEVEL_VALID_MIN);	
}

static int imx586_focus_thread(void *data)
{
	struct imx586 *priv = (struct imx586 *)data;
	u16 fps = 0;
	unsigned long cl = 0;
	signed long pd = 0;
	
#if (PDAF_SOLUTION_1 == PDAF_SOLUTION)
	u8 i = 0, val = 0;
	s16 ret = 0;
	u16 gain = 0;

	signed long offsetData[IMX586_EEPROM_DCC_NUMBER/2] = {0, 0};
	unsigned short XAddressKnot[8] = {264,760,1256,1752,2248,2744,3240,3736};
	unsigned short YAddressKnot[6] = {260,756,1252,1748,2244,2740};
	int err;

	DefocusOKNGThrLine_t  p_DefocusOKNGThrLine[48] ={0};

	unsigned long temp1[2] = {0, 960};
	unsigned long temp2[2] = {150, 150};
	
	PdLibInputData_t indata;
	PdLibOutputData_t outdata;

	signed long offsetpos = 0, temp = 0;
	bool changepos = false;
	
	for (i = 0; i < 48; i++)
	{
		p_DefocusOKNGThrLine[i].PointNum = 2;
		p_DefocusOKNGThrLine[i].p_AnalogGain= &temp1[0];
		p_DefocusOKNGThrLine[i].p_Confidence = &temp2[0];
	}

	indata.XAddressOfWindowStart = ((center_area[0].val << 8) | center_area[1].val);
	indata.YAddressOfWindowStart = ((center_area[2].val << 8) | center_area[3].val);		
	indata.XAddressOfWindowEnd = ((center_area[4].val << 8) | center_area[5].val);;
	indata.YAddressOfWindowEnd = ((center_area[6].val << 8) | center_area[7].val);;		

	indata.XKnotNumSlopeOffset = 8;
	indata.YKnotNumSlopeOffset = 6;		

	indata.p_SlopeData = eeprom_dcc;
	indata.p_OffsetData = offsetData;
	
	indata.p_XAddressKnotSlopeOffset = XAddressKnot;
	indata.p_YAddressKnotSlopeOffset = YAddressKnot;

	indata.AdjCoeffSlope = D_PD_LIB_SLOPE_ADJ_COEFF_SENS_MODE2;

    indata.XKnotNumDefocusOKNG = 8;
    indata.YKnotNumDefocusOKNG = 6;

    indata.p_XAddressKnotDefocusOKNG = XAddressKnot;
    indata.p_YAddressKnotDefocusOKNG = YAddressKnot;

	indata.p_DefocusOKNGThrLine = p_DefocusOKNGThrLine;

	indata.DensityOfPhasePix = D_PD_LIB_DENSITY_SENS_MODE0;          /* Density of phase detection pixel. */
#endif

	printk("%s frame_rate: %d fps, mode_prop_id: %d, frame_rate: %d, width: %d, height: %d\n",__func__, 
			priv->frame_rate/1000000, priv->s_data->mode_prop_idx, priv->frame_rate, priv->s_data->fmt_width, priv->s_data->fmt_height);

	imx586_pdaf_set_area(priv, true);
	usleep_range(10 * 1000, 11 * 1000);

#if (PDAF_SOLUTION_2 == PDAF_SOLUTION)
	imx586_pdaf_set_dcc(priv, true);
	usleep_range(5 * 1000, 5 * 1000);
#endif
	
	while (1)
	{
		if (2 == priv->streamflag)
		{
			priv->streamflag = 0;
			priv->readingpd = false;
			
			printk("%s: focus thread exit\n",__func__);

			break;			
		}

		if (kthread_should_stop())
			break;

		// read cl and pdd
		//imx586_pdaf_get_pd_cl(priv, &indata.ConfidenceLevel, &indata.PhaseDifference);
		imx586_pdaf_get_pd_cl(priv, &cl, &pd);

		#if (PDAF_SOLUTION_1 == PDAF_SOLUTION)
		indata.ConfidenceLevel = cl;
		indata.PhaseDifference = pd;
		//printk("%s: get pd_cl:  ConfidenceLevel: %ld, PhaseDifference:%ld\n",__func__, indata.ConfidenceLevel, indata.PhaseDifference);

		if (IMX586_PD_CONFIDENCE_LEVEL_VALID_MIN + 1 > indata.ConfidenceLevel)
		{
			//printk("%s: warn, get pd_cl:  ConfidenceLevel: %ld, PhaseDifference:%ld, ConfidenceLevel: %ld < %d\n",__func__, 
			//			indata.ConfidenceLevel, indata.PhaseDifference, indata.ConfidenceLevel, IMX586_PD_CONFIDENCE_LEVEL_VALID_MIN + 1);
		}
		else
		{
			err = imx586_read_reg(priv->s_data, IMX586_ANA_GAIN_GLOBAL_ADDR_MSB, &val);
			gain = ((val & 0x03) << 8);
			
			err = imx586_read_reg(priv->s_data, IMX586_ANA_GAIN_GLOBAL_ADDR_LSB, &val);
			gain |= val;

			gain = (1024/(1024-gain));

			indata.XSizeOfImage = priv->s_data->fmt_width;
			indata.YSizeOfImage = priv->s_data->fmt_height;
			
			indata.ImagerAnalogGain	= gain;
			
			ret = PdLibGetDefocus(&indata, &outdata);

			//printk("%s: PdLibGetDefocus:  Defocus: 0x%lx, dac: %ld, DefocusConfidence:%d, DefocusConfidenceLevel: %ld, PhaseDifference: %ld, cur_pos: %d, cl: %ld, pd: %ld, XSizeOfImage:%d, YSizeOfImage:%d, gain: %d, ret: %d\n",__func__, 
			//			outdata.Defocus, ((outdata.Defocus * (-1)) >> 14), outdata.DefocusConfidence, outdata.DefocusConfidenceLevel, outdata.PhaseDifference, priv->cur_pos, indata.ConfidenceLevel, indata.PhaseDifference, indata.XSizeOfImage, indata.YSizeOfImage, gain, ret);

			if (D_PD_LIB_E_OK == ret)
			{
				changepos = false;
				offsetpos = ((outdata.Defocus * (-1)) >> 14);

				if ( (0 == temp) 
					|| (3 < (abs(offsetpos) - abs(temp))))
				{
					changepos = true;
				}
				
				temp = offsetpos;

				//printk("%s: cl: %ld, cur_pos:%d, offsetpos: %ld, is change pos: %d,\n",__func__, indata.ConfidenceLevel, priv->cur_pos, offsetpos, changepos);
				
				if (IMX586_PD_CONFIDENCE_LEVEL_VALID_MIN < indata.ConfidenceLevel && changepos)	//Defocus: s17.14
					imx586_focus_set_position(priv, offsetpos + priv->cur_pos);
			}
		}
		#endif
		
		fps = priv->frame_rate/1000000;
		if (0 != priv->frame_rate/1000000)
			fps++;

		if (8 > fps)
			usleep_range(600 * 1000, 600 * 1000 + 100);
		else
			usleep_range((1000/fps) * 10 * 1000, (1000/fps) * 10 * 1000 + 100);
			
	}

	return 0;
}

#endif



#if (PDAF_SOLUTION_1 == PDAF_SOLUTION || PDAF_SOLUTION_2 == PDAF_SOLUTION)
static void imx586_focus_start_kthreads(struct imx586 *priv)
{
	int rc;

	priv->streamflag = 1;
	priv->readingpd = false;

	if (NULL == priv->kthread_focus)
	{
		printk("%s: imx586_focus_start_kthreads !!! \n",__func__);

		priv->kthread_focus = kthread_run(imx586_focus_thread, priv, "imx586_focus");

		if (IS_ERR(priv->kthread_focus)) {
			rc = PTR_ERR(priv->kthread_focus);
			priv->kthread_focus = NULL;

			printk("%s: imx586_focus_start_kthreads  error\n",__func__);
			
			return;
		}
	}
}

static void imx586_focus_stop_kthreads(struct imx586 *priv)
{
	mutex_lock(&priv->kthread_lock);
	
	priv->streamflag = 2;
	priv->readingpd = false;

	if (priv->kthread_focus) {
		printk("%s: imx586_focus_stop_kthreads !!! \n",__func__);

		kthread_stop(priv->kthread_focus);
		priv->kthread_focus = NULL;
	}
		
	mutex_unlock(&priv->kthread_lock);
}
#endif


static inline u16 imx586_get_fine_store_time(struct camera_common_data *s_data)
{
    u8 reg_val1, reg_val2, ret = 0;
    int err = 0;

    err = imx586_read_reg(s_data, IMX586_FINE_INTEG_TIME_MSB, &reg_val1);
    err = imx586_read_reg(s_data, IMX586_FINE_INTEG_TIME_LSB, &reg_val2);

	 ret = ((reg_val1<< 8)|reg_val2);

	return ret;
}

static inline bool imx586_get_hdrmode(const struct imx586 *priv)
{
	const struct camera_common_data *s_data = priv->s_data;

	switch (s_data->mode_prop_idx) {
	case IMX586_MODE_4000X3000_10BIT_HDR_30FPS:
		return true;
	default:
		return false;
	}
}

static inline u32 imx586_get_cit_lshift(const struct imx586 *priv)
{
	struct camera_common_data *s_data = priv->s_data;
    int err = 0;
	u8 i = 0, reg_val;
	u32 value = 1;
	
	if (IMX586_MODE_4000X3000_10BIT_HDR_30FPS == s_data->mode_prop_idx)
		return 1;

    err = imx586_read_reg(s_data, IMX586_CIT_LSHIFT, &reg_val);
	if (0 > reg_val)
		return 1;
	
	for (i = 0; i < reg_val; i++)
		value *= 2;

	return value;
}

static inline u8 imx586_get_coarsetime_min(const struct imx586 *priv)
{
	const struct camera_common_data *s_data = priv->s_data;
	u8 ret = IMX586_COARSE_INTEG_TIME_MIN_NOR;
	
	if (IMX586_MODE_4000X3000_10BIT_HDR_30FPS == s_data->mode_prop_idx)
		ret = IMX586_COARSE_INTEG_TIME_MIN_QBC;

	return ret;
}

static int imx586_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
    struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	int err;

#if (PDAF_SOLUTION_NO != PDAF_SOLUTION)
	struct imx586 *priv = (struct imx586 *)tc_dev->priv;

	if (true == priv->readingpd ||1 == priv->streamflag)
	{
		//printk("%s: reading pd or streaming, set val: %d, return\n",__func__, val);
		
		return 0;
	}
#endif

	dev_dbg(dev, "%s: imx586_set_group_hold start, val :%d !!------------------\n", __func__, val);

	err = imx586_write_reg(s_data,
				IMX586_GROUP_HOLD_ADDR, val);

#ifdef _I2C_DEB_
	//dev_dbg(dev, "%s: check_reg!!----\n", __func__);
	check_reg(s_data, IMX586_GROUP_HOLD_ADDR);
#endif

#ifdef _ERROR_TEST_
		err = 0;
#endif
	if (err) {
		dev_dbg(dev,
			"%s: Group hold control error\n", __func__);
		return err;
	}

	return 0;
}

static int imx586_set_gain(struct tegracam_device *tc_dev, s64 val)
{
    struct camera_common_data *s_data = tc_dev->s_data;
	struct imx586 *priv = (struct imx586 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	
	const struct sensor_control_properties *ctrlprops = 
		&s_data->sensor_props.sensor_modes[0].control_properties;
	imx586_reg reg_list[2];
#ifdef _ERROR_TEST_
	imx586_reg reg_list_test[2];
#endif
	int err;
	int i;
	//u16 gain;
	u16 temp = 0;

#if (PDAF_SOLUTION_NO != PDAF_SOLUTION)	
	if (true == priv->readingpd)
	{		
		return 0;
	}
#endif

	dev_dbg(dev, "%s, val:%lld \n", __func__, val);

	if (0 == priv->check_encrypt)
	{
		dev_info(dev, "%s : imx586_set_mode encrypt check faild, return!!!--------\n", __func__);
		return -1;
	}
	
	temp = (u16)(val/ctrlprops->gain_factor);
	if (IMX586_ANALOG_GAIN_DB_MIN > temp)
		temp = IMX586_ANALOG_GAIN_DB_MIN;
	else if (IMX586_ANALOG_GAIN_DB_MAX_4000 < temp && 360 == ctrlprops->max_gain_val)
		temp = IMX586_ANALOG_GAIN_DB_MAX_4000;
	else if (IMX586_ANALOG_GAIN_DB_MAX_8000 < temp && 240 == ctrlprops->max_gain_val)
		temp = IMX586_ANALOG_GAIN_DB_MAX_8000;

	dev_dbg(dev, "%s, min_gain_val:%d, max_gain_val:%d, step_gain_val:%d, gain_factor:%d\n", __func__, 
				ctrlprops->min_gain_val, ctrlprops->max_gain_val, ctrlprops->step_gain_val, ctrlprops->gain_factor);
	
	dev_dbg(dev, "%s: set value: %lld, db gain: %d, anagle gani: %d\n",  __func__, val, temp, gain_analog[temp - 1]);
	
	imx586_get_gain_reg(reg_list, gain_analog[temp - 1]);

	for( i = 0; i < 2; i++){
		dev_dbg(dev, "%s:  set gain,  address: 0x%04x, val: 0x%02x\n",  __func__, reg_list[i].addr, reg_list[i].val );
		err = imx586_write_reg(priv->s_data, reg_list[i].addr,reg_list[i].val);

#ifdef _ERROR_TEST_
		err = 0;
#endif
		if (err)
			goto fail;
	}

#if 0
	for( i = 0; i < 2; i++){
        err = imx586_read_reg(priv->s_data, reg_list[i].addr, &reg_list[i].val);
		//dev_dbg(dev,"%s:  set gain, read address: 0x%04x, read val : 0x%02x, \n",  __func__, reg_list[i].addr, reg_list[i].val);
	}
#endif

#ifdef _ERROR_TEST_
	imx586_get_gain_reg(reg_list_test, 666);
	for( i = 0; i < 2; i++){
        err = imx586_read_reg(priv->s_data, reg_list_test[i].addr, &reg_list_test[i].val);
		//dev_dbg(dev,"%s: gain reg test address: 0x%04x, read val : 0x%02x, \n",  __func__, reg_list_test[i].addr, reg_list_test[i].val);
	}
#endif

	return 0;

fail:
	dev_dbg(dev, "%s: GAIN control error\n", __func__);
	return err;
}

static int imx586_set_coarse_time(struct imx586 *priv, s64 val)
{
	struct camera_common_data *s_data = priv->s_data;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	
	imx586_reg reg_list[2];
	int err;
	u32 coarse_time, def_coarse_time, max_coarse_time, min_coarse_time, fine_time;
	int i = 0;

	if (0 == priv->frame_length)
		priv->frame_length = IMX586_MIN_FRAME_LENGTH;

	max_coarse_time = priv->frame_length - IMX586_COARSE_MARGINS_MAX;
	fine_time = imx586_get_fine_store_time(s_data);
	//max_coarse_time -= fine_time;
	
	/* coarse time in lines */
	def_coarse_time = (u32) (val * s_data->frmfmt[s_data->mode].framerates[0] *
		priv->frame_length / mode->control_properties.exposure_factor);

	//Long Exposure time = (65487 * 8 * 9440) / (216[MHz] *8) = 2.86 [sec]
	//Long Exposure time = COARSE_INTEG_TIME * 2 ^ CIT_LSHIFT * LINE_LENGTH_PCK /( VTPXCK * 8)
	//0.02 = COARSE_INTEG_TIME * 2 * 1 * 9440 * (214.8M * 8)
	//0.02 = COARSE_INTEG_TIME * 2 * 1 * 9440 / (1718400000)
	//COARSE_INTEG_TIME = (0.02 * 1718400000)/(2 * 1 * 9440) = 3,640.677966101695

	coarse_time = def_coarse_time;
    if (coarse_time > max_coarse_time)
         coarse_time = max_coarse_time;

	/* 0 - 5 are prohibited */
	min_coarse_time = imx586_get_coarsetime_min(priv);
	if (priv->frame_length - coarse_time < min_coarse_time)
		coarse_time = min_coarse_time;

	//4000x3000 30fps(hdr), 29708, 	frame_length:6067
	//4000x3000 35fps, 28035, 	frame_length:3060
	//5600x4200 9fps, 29708, 	frame_length:4300
	//5600x4200 27fps, 29708, 	frame_length:4300
	//8000x6000 raw 6fps, 28035, 	frame_length:6060
	//8000x6000 6fps, 28035, 	frame_length:6060

	//max_coarse_time : 6067 - 48 = 6019
	//max_coarse_time : 6067 - 52 = 6015
	
	//max_coarse_time : 3060 - 48 = 3012
	//max_coarse_time : 3060 - 52 = 3008
	
	//max_coarse_time : 4300 - 48 = 4252
	//max_coarse_time : 4300 - 52 = 4248

	//max_coarse_time : 6060 - 48 = 6012
	//max_coarse_time : 6060 - 52 = 6008


	//4000x3000 35fps
	//def_coarse_time : 28035(val) * 35 * 3060 /1000000 = 3031
	//maxval = 3012 * 1000000 / 35 / 3060 = 28123
	//maxval = 3008 * 1000000 / 35 / 3060 = 28085
	//maxval = 2964 * 1000000 / 35 / 3060 = 28085
	//minxval = 6 * 1000000 / 35 / 3060 = 56

	//4000x3000 30fps(hdr)
	//maxval = 6019 * 1000000 / 30 / 6067 = 33069
	//maxval = 6015 * 1000000 / 30 / 6067 = 33047
	//minxval = 5 * 1000000 / 30 / 6067 = 27

	//5600x4200 9fps
	//maxval = 4252 * 1000000 / 9 / 4300 = 109870
	//maxval = 4248 * 1000000 / 9 / 4300 = 109767
	//minxval = 6 * 1000000 / 9 / 4300 = 155
	
	//5600x4200 27fps
	//maxval = 4252 * 1000000 / 27 / 4300 = 36623
	//maxval = 4248 * 1000000 / 27 / 4300 = 36589
	//minxval = 6 * 1000000 / 27 / 4300 = 51

	//8000x6000 6fps
	//maxval = 6012 * 1000000 / 6 / 6060 = 165346
	//maxval = 6008 * 1000000 / 6 / 6060 = 165236
	//minxval = 6 * 1000000 / 6 / 6060 = 165

	// 20000 = 
	printk("%s: val: %lld, coarse_integ_time:%d, def coarse time:%d, max_coarse_time: %d, fine_time: %d, set: %d, mode: %d, num_framerates: %d, framerates: %d, frame_length: %d, exposure_factor:%d\n",__func__, 
			val, coarse_time, def_coarse_time, max_coarse_time, max_coarse_time - coarse_time, fine_time, s_data->mode_prop_idx,
			s_data->frmfmt[s_data->mode_prop_idx].num_framerates, s_data->frmfmt[s_data->mode_prop_idx].framerates[0], 
			priv->frame_length, mode->control_properties.exposure_factor);

	imx586_get_integ_coarse_time_regs(reg_list, coarse_time);
	
	for (i = 0; i < 2; i++) {
		err = imx586_write_reg(priv->s_data, reg_list[i].addr, reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		"%s: set coarse time error\n", __func__);
	return err;
}

static int imx586_set_coarse_time_hdr(struct imx586 *priv, s64 val)
{
	struct camera_common_data *s_data = priv->s_data;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	
	imx586_reg reg_list[2];
	int err;
	u32 coarse_time, def_coarse_time, max_coarse_time, min_coarse_time, fine_time;
	int i = 0;

	if (0 == priv->frame_length)
		priv->frame_length = IMX586_MIN_FRAME_LENGTH;

	max_coarse_time = priv->frame_length - IMX586_COARSE_MARGINS_MAX;
	fine_time = imx586_get_fine_store_time(s_data);
	//max_coarse_time -= fine_time;

	// long exposure
	if (mode->control_properties.exposure_factor/30 < val/mode->control_properties.exposure_factor)	
	{
		if (IMX586_MODE_4000X3000_10BIT_HDR_30FPS == s_data->mode_prop_idx)
		{
			def_coarse_time= (IMX586_PIXEL_CLK_4000X3000_30FPS_HDR * val /
						mode->control_properties.exposure_factor)/(mode->image_properties.line_length  * imx586_get_cit_lshift(priv));
		}
	}
	else
	{
		/* coarse time in lines */
		def_coarse_time = (u32) (val * s_data->frmfmt[s_data->mode].framerates[0] *
			priv->frame_length / mode->control_properties.exposure_factor);
	}
	
	//Long Exposure time = (65487 * 8 * 9440) / (216[MHz] *8) = 2.86 [sec]
	//Long Exposure time = COARSE_INTEG_TIME * 2 ^ CIT_LSHIFT * LINE_LENGTH_PCK /( VTPXCK * 8)
	//0.02 = COARSE_INTEG_TIME * 2 * 1 * 9440 * (214.8M * 8)
	//0.02 = COARSE_INTEG_TIME * 2 * 1 * 9440 / (1718400000)
	//COARSE_INTEG_TIME = (0.02 * 1718400000)/(2 * 1 * 9440) = 3,640.677966101695

	coarse_time = def_coarse_time;
    if (coarse_time > max_coarse_time)
         coarse_time = max_coarse_time;

	/* 0 - 5 are prohibited */
	min_coarse_time = imx586_get_coarsetime_min(priv);
	if (priv->frame_length - coarse_time < min_coarse_time)
		coarse_time = min_coarse_time;

	//4000x3000 30fps(hdr), 29708, 	frame_length:6067
	//4000x3000 35fps, 28035, 	frame_length:3060
	//5600x4200 9fps, 29708, 	frame_length:4300
	//5600x4200 27fps, 29708, 	frame_length:4300
	//8000x6000 raw 6fps, 28035, 	frame_length:6060
	//8000x6000 6fps, 28035, 	frame_length:6060

	//max_coarse_time : 6067 - 48 = 6019
	//max_coarse_time : 6067 - 52 = 6015
	
	//max_coarse_time : 3060 - 48 = 3012
	//max_coarse_time : 3060 - 52 = 3008
	
	//max_coarse_time : 4300 - 48 = 4252
	//max_coarse_time : 4300 - 52 = 4248

	//max_coarse_time : 6060 - 48 = 6012
	//max_coarse_time : 6060 - 52 = 6008


	//4000x3000 35fps
	//def_coarse_time : 28035(val) * 35 * 3060 /1000000 = 3031
	//maxval = 3012 * 1000000 / 35 / 3060 = 28123
	//maxval = 3008 * 1000000 / 35 / 3060 = 28085
	//maxval = 2964 * 1000000 / 35 / 3060 = 28085
	//minxval = 6 * 1000000 / 35 / 3060 = 56

	//4000x3000 30fps(hdr)
	//maxval = 6019 * 1000000 / 30 / 6067 = 33069
	//maxval = 6015 * 1000000 / 30 / 6067 = 33047
	//minxval = 5 * 1000000 / 30 / 6067 = 27

	//5600x4200 9fps
	//maxval = 4252 * 1000000 / 9 / 4300 = 109870
	//maxval = 4248 * 1000000 / 9 / 4300 = 109767
	//minxval = 6 * 1000000 / 9 / 4300 = 155
	
	//5600x4200 27fps
	//maxval = 4252 * 1000000 / 27 / 4300 = 36623
	//maxval = 4248 * 1000000 / 27 / 4300 = 36589
	//minxval = 6 * 1000000 / 27 / 4300 = 51

	//8000x6000 6fps
	//maxval = 6012 * 1000000 / 6 / 6060 = 165346
	//maxval = 6008 * 1000000 / 6 / 6060 = 165236
	//minxval = 6 * 1000000 / 6 / 6060 = 165

	// 20000 = 
	printk("%s: val: %lld, coarse_integ_time:%d, def coarse time:%d, max_coarse_time: %d, fine_time: %d, set: %d, mode: %d, num_framerates: %d, framerates: %d, frame_length: %d, exposure_factor:%d\n",__func__, 
			val, coarse_time, def_coarse_time, max_coarse_time, max_coarse_time - coarse_time, fine_time, s_data->mode_prop_idx,
			s_data->frmfmt[s_data->mode_prop_idx].num_framerates, s_data->frmfmt[s_data->mode_prop_idx].framerates[0], 
			priv->frame_length, mode->control_properties.exposure_factor);

	imx586_get_coarse_time_regs_hdr(reg_list, coarse_time);
	
	for (i = 0; i < 2; i++) {
		err = imx586_write_reg(priv->s_data, reg_list[i].addr, reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		"%s: set coarse time error\n", __func__);
	return err;
}


/*
	For imx586 frame rate is calculated by pixel rate, itvp clock and frame size
	according to manual, the frame rate is IVTPXCK[MHz] * 8 / (FRM_LENGTH_LINE * LINE_LENGTH_PCK)
*/
static int imx586_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx586 *priv = (struct imx586 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	imx586_reg reg_list[2];
	int err;
	u32 frame_length;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	int i = 0;

#if (PDAF_SOLUTION_NO != PDAF_SOLUTION)
	if (true == priv->readingpd)
	{
		return 0;
	}
#endif

	if (0 == priv->check_encrypt)
	{
		dev_info(dev, "%s : imx586_set_mode encrypt check faild, return!!!--------\n", __func__);
		return -1;
	}

	frame_length = mode->signal_properties.pixel_clock.val *
		mode->control_properties.framerate_factor /
		mode->image_properties.line_length / val;

	dev_info(dev, "%s: val: %lld, calc frame_length:%d, mode: %d, pixel_clock: %llu, framerate_factor:%d, line_length:%d\n", __func__,
		val, frame_length, s_data->mode_prop_idx, mode->signal_properties.pixel_clock.val, mode->control_properties.framerate_factor, mode->image_properties.line_length);
	
	if (frame_length > IMX586_MAX_FRAME_LENGTH)
		frame_length = IMX586_MAX_FRAME_LENGTH;

	priv->frame_length = frame_length;

#if (PDAF_SOLUTION_NO != PDAF_SOLUTION)
	priv->frame_rate = mode->signal_properties.pixel_clock.val *
										mode->control_properties.framerate_factor /
										mode->image_properties.line_length / priv->frame_length;

	/*
	dev_info(dev, "%s: val: %lld, frame_length: %d, frame_rate:%d, times: %d, numctrls:%d, csi_port:%d, numlanes:%d, mode_prop_idx: %d, mode:%d\n", __func__,
		val, priv->frame_length, priv->frame_rate, times, s_data->numctrls, s_data->csi_port, s_data->numlanes, s_data->mode_prop_idx, s_data->mode);

	dev_info(dev, "%s: numfmts: %d, def_mode:%d, def_width:%d, def_height:%d, def_clk_freq: %d, fmt_width: %d, fmt_height:%d\n", __func__,
		s_data->numfmts, s_data->def_mode, s_data->def_width, s_data->def_height, s_data->def_clk_freq, s_data->fmt_width, s_data->fmt_height);

	dev_info(dev, "%s: sensor_mode_id: %d, use_sensor_mode_id:%d, override_enable:%d, version:%u, last_wdr_et_val: %lld\n", __func__,
		s_data->sensor_mode_id, s_data->use_sensor_mode_id, s_data->override_enable, s_data->version, priv->last_wdr_et_val);
	*/
	
	dev_info(dev, "%s: val: %lld, frame_length: %d, frame_rate:%d fps\n", __func__,
					val, priv->frame_length, priv->frame_rate/1000000);
#endif

	imx586_get_frame_length_regs(reg_list, priv->frame_length);

	// Disable write reg when set frame rate, this must be rewritten in future.
#if 1
	for (i = 0; i < 2; i++) {
		//dev_info(dev, "%s:  set frame reg, address = 0x%04x, val: 0x%02x, \n",  __func__, reg_list[i].addr, reg_list[i].val);
		err = imx586_write_reg(s_data, reg_list[i].addr, reg_list[i].val);
#endif 

#ifdef _I2C_DEB_
		//dev_info(dev, "%s: --------------check_reg!!----\n", __func__);
       check_reg(s_data, reg_list[i].addr);
#endif

#if 0
		if (err)
			goto fail;
#endif
	}

	return 0;

//fail:
//	dev_dbg(dev, "%s: FRAME_LENGTH control error\n", __func__);
//	return err;
}

static int imx586_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx586 *priv = (struct imx586 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	int err;
	struct v4l2_control control;
	int hdr_en;

	//dev_dbg(dev, "%s: val: %lld\n", __func__, val);

	/* check hdr enable ctrl */
	control.id = TEGRA_CAMERA_CID_HDR_EN;
	err = camera_common_g_ctrl(s_data, &control);
#if 0
	if (err < 0) {
		dev_err(dev, "could not find device ctrl.\n");
		return err;
	}
#endif

#if (PDAF_SOLUTION_NO != PDAF_SOLUTION)
	if (true == priv->readingpd)
	{		
		return 0;
	}
#endif

	if (0 == priv->check_encrypt)
	{
		dev_info(dev, "%s : imx586_set_mode encrypt check faild, return!!!--------\n", __func__);
		return -1;
	}

	hdr_en = switch_ctrl_qmenu[control.value];
	printk("%s: val: %lld, hdr_en: %d, %d, mode: %d\n", __func__, val, hdr_en, control.value, s_data->mode_prop_idx);
	if (hdr_en == SWITCH_ON || imx586_get_hdrmode(priv)) {
		dev_dbg(dev, "%s: imx586_set_coarse_time(hdr) \n", __func__);
		err = imx586_set_coarse_time_hdr(priv, val);
		if (err)
			dev_dbg(dev,
			"%s: error coarse time hdr override\n", __func__);
	} else {
		dev_dbg(dev, "%s: imx586_set_coarse_time(nor) \n", __func__);
		err = imx586_set_coarse_time(priv, val);
		if (err)
			dev_dbg(dev,
			"%s: error coarse time nor override\n", __func__);
	}

	return err;
}

static int imx586_fill_string_ctrl(struct tegracam_device *tc_dev, struct v4l2_ctrl *ctrl)
{
	struct imx586 *priv = tc_dev->priv;
	struct device *dev = tc_dev->dev;
	int i;

	dev_dbg(dev,"%s: imx586_fill_string_ctrl start ! ----------------\n", __func__);

	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_FUSE_ID:
		for (i = 0; i < IMX586_FUSE_ID_SIZE; i++)
			sprintf(&ctrl->p_new.p_char[i*2], "%02x", priv->fuse_id[i]);
		break;
	default:
		return -EINVAL;
	}

	ctrl->p_cur.p_char = ctrl->p_new.p_char;

	return 0;
}

static const struct tegracam_ctrl_ops imx586_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.string_ctrl_size = {0, IMX586_FUSE_ID_STR_SIZE},
	.set_gain = imx586_set_gain,
	.set_exposure = imx586_set_exposure,
	.set_frame_rate = imx586_set_frame_rate,
	.set_group_hold = imx586_set_group_hold,
	.fill_string_ctrl = imx586_fill_string_ctrl,
};

static int imx586_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	printk("%s: power_on\n",__func__);

	//dev_dbg(dev, "%s: power on, pdata->power_on = %d !!------------\n", __func__, pdata->power_on);
	if (pdata && pdata->power_on) {
		err = pdata->power_on(pw);
		if (err)
			dev_err(dev, "%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}


	/*exit reset mode: XCLR */
    dev_dbg(dev, "%s: set reset gpio = %d.\n", __func__, pw->reset_gpio);
	if (pw->reset_gpio) {
		gpio_set_value(pw->reset_gpio, 0);
		usleep_range(30, 50);
		gpio_set_value(pw->reset_gpio, 1);
		usleep_range(30, 50);
	}

	pw->state = SWITCH_ON;
	return 0;

}

static int imx586_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	//dev_dbg(dev, "%s: power off, pdata->power_off = %d-----------\n", __func__, pdata->power_off);
	printk("%s: power_off\n",__func__);

	//dump_stack();

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

static int imx586_power_get(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	const char *mclk_name;
	struct clk *parent;
	int err = 0;

	dev_dbg(dev, "%s: imx586_power_get-----------\n", __func__);

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

static int imx586_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;

	printk("%s: power_put\n",__func__);

	if (unlikely(!pw))
		return -EFAULT;

	return 0;
}

static struct camera_common_pdata *imx586_parse_dt(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	struct camera_common_pdata *ret = NULL;
	int err;
	int gpio;

	dev_dbg(dev, "%s: imx586_parse_dt-----------\n", __func__);

	if (!np)
		return NULL;

	match = of_match_device(imx586_of_match, dev);
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

	of_property_read_u32(np, "motor_addr",
		&board_priv_pdata->pwdn_gpio);

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

static int imx586_stand_by(struct tegracam_device *tc_dev )
{
	struct imx586 *priv = (struct imx586 *)tegracam_get_privdata(tc_dev);
	printk("%s: write table id: %d\n", __func__, IMX586_MODE_STAND_BY);
	return imx586_write_table(priv,mode_table[IMX586_MODE_STAND_BY]);
}

#if (1== FLIP_MIRROR_ENABLE)
static int imx586_set_flip(struct camera_common_data *s_data, u8 flip)
{
	u8 val = 0x00;
	int err;
	
	switch (flip)
	{
		case 0:
			val = 0;
			break;
		case 1:	// Image orientation for Vertical direction
			val |= 0x01;
			break;
		case 2:	// Image orientation for Horizontal direction
			val |= 0x02;
			break;			
		case 3:	// Image orientation for Vertical and Horizontal direction
			val |= 0x03;
			break;			
	}

	err = imx586_write_reg(s_data, IMX586_FLIP_MIRROR_ADDR, val);
	
	return err;
}
#endif

static int imx586_set_mode(struct tegracam_device *tc_dev)
{
	struct imx586 *priv = (struct imx586 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	bool limit_analog_gain = false;
	const struct of_device_id *match;
	int err;
	
	dev_dbg(dev, "%s: imx586_set_mode-----------\n", __func__);

	if (0 == priv->check_encrypt)
	{
		dev_info(dev, "%s : imx586_set_mode encrypt check faild, return!!!--------\n", __func__);
		return -1;
	}
	
	match = of_match_device(imx586_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return -EINVAL;
	}

	limit_analog_gain = of_property_read_bool(np, "limit_analog_gain");

	dev_dbg(dev, "%s: imx586_set_mode, mode_prop_idx:%d, limit_analog_gain:%d, csi_port:%d, numlanes:%d, mode:%d,fmt_width: %d, fmt_height:%d, def_clk_freq:%d\n", 
				__func__, s_data->mode_prop_idx, limit_analog_gain, s_data->csi_port, s_data->numlanes, s_data->mode, s_data->fmt_width, s_data->fmt_height, s_data->def_clk_freq);


	#if 1
	printk("%s: imx586_stand_by-----------\n", __func__);
	err = imx586_stand_by(tc_dev);
	if(err){
		printk("%s: imx586_stand_by error %d\n",__func__, err);
		return err;
	}
	#endif 
	usleep_range(5 * 1000, 6 * 1000);

	printk("%s: imx586_write_table, mode: %d-----------\n", __func__, s_data->mode_prop_idx);
	err = imx586_write_table(priv, mode_table[s_data->mode_prop_idx]);
	if (err) {
		dev_err(dev, "%s: imx586_write_table error, table id:%d\n", __func__, s_data->mode_prop_idx);
		return err;
	}
	usleep_range(5 * 1000, 6 * 1000);

	//set hdr
	if (imx586_get_hdrmode(priv))
	{
		printk("%s: imx586_write_table, mode: %d, 4000X3000_10BIT_HDR_30FPS\n", __func__, s_data->mode_prop_idx);
		
		err = imx586_write_table(priv, mode_table_hdr[s_data->mode_prop_idx]);
		if (err) {
			dev_err(dev, "%s: imx586_write_table hdr reg error, table id:%d\n", __func__, s_data->mode_prop_idx);
			return err;
		}
		usleep_range(5 * 1000, 6 * 1000);	

		err = imx586_write_table(priv, mode_table_hdr_exposure[s_data->mode_prop_idx]);
		if (err) {
			dev_err(dev, "%s: imx586_write_table hdr exposure reg error, table id:%d\n", __func__, s_data->mode_prop_idx);
			return err;
		}
		usleep_range(5 * 1000, 6 * 1000);	

		
	}

	// set mirror
	#if (1== FLIP_MIRROR_ENABLE)
	imx586_set_flip(s_data, 1);
	usleep_range(50 * 1000, 51 * 1000);
	#endif
	
	#if (PDAF_SOLUTION_1 == PDAF_SOLUTION)
	imx586_eeprom_get_dcc(priv);
	usleep_range(10 * 1000, 11 * 1000);
	#endif

	#if (1 == QSC_EANBLE)
	imx586_eeprom_to_sensor_qsc(priv);
	usleep_range(10 * 1000, 11 * 1000);
	#endif
	
	#if (PDAF_SOLUTION_1 == PDAF_SOLUTION || PDAF_SOLUTION_3 == PDAF_SOLUTION)
	imx586_eeprom_to_sensor_lrc(priv);
	usleep_range(10 * 1000, 11 * 1000);
	#endif

	//  FRM_LENGTH_CTL:  
	//	0:  if (FRM_LENGTH_LINES < COARSE_INTEG_TIME + 48),  coarse_integ_time = val
	//    1:  if (FRM_LENGTH_LINES < COARSE_INTEG_TIME + 48),  FRM_LENGTH_LINES = COARSE_INTEG_TIME + 48

	#if (1 == LIMIT_FRAMERATE_EXPOSURE)
	err = imx586_write_reg(s_data, IMX586_FRM_LENGTH_CTL, 0);
	#else
	err = imx586_write_reg(s_data, IMX586_FRM_LENGTH_CTL, 1);
	#endif
	
	/*
	dev_dbg(dev, "%s: finish write mode table %d, limit_analog_gain = %d\n", __func__, s_data->mode_prop_idx, limit_analog_gain);

	if (limit_analog_gain) {
		err = imx586_write_reg(priv->s_data,
			IMX586_ANALOG_GAIN_LIMIT_ADDR,
			IMX586_ANALOG_GAIN_LIMIT_VALUE);
#ifdef _ERROR_TEST_
			err = 0;
#endif
		if (err){
			dev_err(dev, "%s: imx586_write_reg error, addr: 0x%x, value: 0x%x\n",
						__func__,IMX586_ANALOG_GAIN_LIMIT_ADDR, IMX586_ANALOG_GAIN_LIMIT_VALUE);
			return err;
		}
	}
	*/
	
	return 0;
}

static int imx586_start_streaming(struct tegracam_device *tc_dev)
{
	struct imx586 *priv = (struct imx586 *)tegracam_get_privdata(tc_dev);
	int err;

	printk("%s\n", __func__);

	#if 0
	err = imx586_stand_by(tc_dev);
	if(err){
		printk("%s: imx586_sstand_by error %d\n",__func__, err);
		return err;
	}
	#endif 
	
	printk("%s: imx586_start_streaming-------------\n",__func__);

	if (test_mode) {
		printk("%s: IMX586_MODE_TEST_PATTERN, %d!!! \n",__func__, IMX586_MODE_TEST_PATTERN);
		err = imx586_write_table(priv,
			mode_table[IMX586_MODE_TEST_PATTERN]);
		if (err)
			return err;
	}

	printk("%s: IMX586_MODE_START_STREAM, %d!!! \n",__func__, IMX586_MODE_START_STREAM);
	err = imx586_write_table(priv,
		mode_table[IMX586_MODE_START_STREAM]);
	if (err) {
		printk("%s: imx586_write_table, table id: %d\n",__func__, IMX586_MODE_START_STREAM);
		return err;
	}

#if (PDAF_SOLUTION_1 == PDAF_SOLUTION || PDAF_SOLUTION_2 == PDAF_SOLUTION)
	imx586_focus_start_kthreads(priv);
#elif (PDAF_SOLUTION_3 == PDAF_SOLUTION)
	imx586_pdaf_set_area(priv, true);
	usleep_range(5 * 1000, 6 * 1000);
#endif

	//usleep_range(priv->frame_length * 10, priv->frame_length * 10 + 1000);
	
	return 0;
}

static int imx586_stop_streaming(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx586 *priv = (struct imx586 *)tegracam_get_privdata(tc_dev);
	int err;

	printk("%s: imx586_stop_streaming-------------\n",__func__);

	err = imx586_write_table(priv, mode_table[IMX586_MODE_STOP_STREAM]);
	if (err)
		return err;

	/* SW_RESET will have no ACK */
	imx586_write_reg(s_data, IMX586_SW_RESET_ADDR, 0x01);

	/*
	 * Wait for one frame to make sure sensor is set to
	 * software standby in V-blank
	 *
	 * delay = frame length rows * Tline (10 us)
	 */
	usleep_range(priv->frame_length * 10, priv->frame_length * 10 + 1000);

#if (PDAF_SOLUTION_1 == PDAF_SOLUTION || PDAF_SOLUTION_2 == PDAF_SOLUTION)
	imx586_focus_stop_kthreads(priv);
#endif

	return 0;
}


static struct camera_common_sensor_ops imx586_common_ops = {
	.numfrmfmts = ARRAY_SIZE(imx586_frmfmt),
	.frmfmt_table = imx586_frmfmt,
	.power_on = imx586_power_on,
	.power_off = imx586_power_off,
	.write_reg = imx586_write_reg,
	.read_reg = imx586_read_reg,
	.parse_dt = imx586_parse_dt,
	.power_get = imx586_power_get,
	.power_put = imx586_power_put,
	.set_mode = imx586_set_mode,
	.start_streaming = imx586_start_streaming,
	.stop_streaming = imx586_stop_streaming,
};

#if 0
static int imx586_fuse_id_setup(struct imx586 *priv)
{
	int err;
	int i;
	struct camera_common_data *s_data = priv->s_data;
	struct device *dev = s_data->dev;
	u8 bak = 0;

	dev_dbg(dev, "%s start !!!------------\n", __func__);

	for (i = 0; i < IMX586_FUSE_ID_SIZE; i++) {
		err |= imx586_read_reg(s_data, IMX586_FUSE_ID_ADDR + i, &bak);
		if (!err)
			priv->fuse_id[i] = bak;
		else {
			dev_err(dev, "%s: can not read fuse id\n", __func__);
			return -EINVAL;
		}
	}

	return 0;
}
#endif

static int imx586_board_setup(struct imx586 *priv)
{
	struct camera_common_data *s_data = priv->s_data;
	struct device *dev = s_data->dev;
	int err = 0;

	dev_dbg(dev, "%s start !!!------------\n", __func__);

	err = camera_common_mclk_enable(s_data);
	if (err) {
		dev_err(dev,
			"Error %d turning on mclk\n", err);
		return err;
	}
	dev_dbg(dev, "%s camera_common_mclk_enable end !!!------------\n", __func__);

	err = imx586_power_on(s_data);
	if (err) {
		dev_err(dev,
			"Error %d during power on sensor\n", err);
		return err;
	}
	dev_dbg(dev, "%s imx586_power_on end !!!------------\n", __func__);

#if 0
	err = imx586_fuse_id_setup(priv);
	if (err) {
		dev_err(dev,
			"Error %d reading fuse id data\n", err);
		goto error;
	}
#endif
	
//error:
	imx586_power_off(s_data);
	camera_common_mclk_disable(s_data);
	return err;
}

// Done
static int imx586_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:-----------imx586_open!!----------\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops imx586_subdev_internal_ops = {
	.open = imx586_open,
};

static int imx586_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct tegracam_device *tc_dev;
	struct imx586 *priv;
	
	struct camera_common_data *cam_data;
	struct sensor_control_properties *ctrlprops = NULL;	
	
	int err;
	__u64 temp = 0;

	dev_info(dev, "%s : probing imx586_8 sensor, 3 mode\n", __func__);

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	priv = devm_kzalloc(dev,
			sizeof(struct imx586), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	tc_dev = devm_kzalloc(dev,
			sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	priv->i2c_client = tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "imx586", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &sensor_regmap_config;
	tc_dev->sensor_ops = &imx586_common_ops;
	tc_dev->v4l2sd_internal_ops = &imx586_subdev_internal_ops;
	tc_dev->tcctrl_ops = &imx586_ctrl_ops;

	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}
	else
	{
		dev_info(dev, "%s : tegracam_device_register end!!!--------\n", __func__);
	}

	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	priv->check_encrypt = 0;
	tegracam_set_privdata(tc_dev, (void *)priv);

#if (1 != IMX586_CHECK_TRACKID)
	priv->check_encrypt = 1;
#endif

#if (PDAF_SOLUTION_NO != PDAF_SOLUTION)
	priv->kthread_focus = NULL;
	priv->frame_rate = 0;
	priv->streamflag = 0;
	priv->readingpd = false;
	priv->dis_infinity = IMX586_MOTOR_STEPS_INFINNITY;
	priv->dis_micro = IMX586_MOTOR_STEPS_MICRO;
	priv->cur_pos = priv->dis_micro;
#endif

#if (1 == IMX586_CHECK_TRACKID)
	imx586_trackid_init(priv);

	if ( 0 != check_trackid(priv))
	{
		dev_info(dev, "%s : imx586_board_setup encrypt check faild, return!!!--------\n", __func__);
		return -1;
	}

	priv->check_encrypt = 1;
#endif

	err = imx586_board_setup(priv);
	if (err) {
		dev_err(dev, "board setup failed\n");
		return err;
	}
	else
	{
		dev_info(dev, "%s : imx586_board_setup end!!!--------\n", __func__);
	}
	
	cam_data = tc_dev->s_data;

	ctrlprops =&cam_data->sensor_props.sensor_modes[0].control_properties;
	dev_info(dev, "%s : ctrlprops->gain_factor = %d, ctrlprops->framerate_factor = %d,ctrlprops->inherent_gain = %d!--------\n", __func__, 
		ctrlprops->gain_factor, ctrlprops->framerate_factor, ctrlprops->inherent_gain);

	dev_info(dev, "%s : ctrlprops->min_gain_val = %d, ctrlprops->max_gain_val = %d,ctrlprops->min_hdr_ratio = %d, ctrlprops->max_hdr_ratio = %d !--------\n", __func__, 
		ctrlprops->min_gain_val, ctrlprops->max_gain_val, ctrlprops->min_hdr_ratio, ctrlprops->max_hdr_ratio);

	dev_info(dev, "%s : ctrlprops->min_framerate = %d, ctrlprops->max_framerate = %d,ctrlprops->step_gain_val = %d, ctrlprops->step_framerate = %d !--------\n", __func__, 
		ctrlprops->min_framerate, ctrlprops->max_framerate, ctrlprops->step_gain_val, ctrlprops->step_framerate);

	dev_info(dev, "%s : ctrlprops->exposure_factor = %d, ctrlprops->default_gain = %d,ctrlprops->default_framerate = %d, ctrlprops->is_interlaced = %d, ctrlprops->interlace_type = %d !--------\n", __func__, 
		ctrlprops->exposure_factor, ctrlprops->default_gain, ctrlprops->default_framerate, ctrlprops->is_interlaced, ctrlprops->interlace_type);

	temp = (__u64)(ctrlprops->min_exp_time.val);
	dev_info(dev, "%s : ctrlprops->min_exp_time = %llu !--------\n", __func__, temp);

	temp = (__u64)(ctrlprops->max_exp_time.val);
	dev_info(dev, "%s : ctrlprops->max_exp_time = %llu !--------\n", __func__, temp);

	temp = (__u64)(ctrlprops->step_exp_time.val);
	dev_info(dev, "%s : ctrlprops->step_exp_time = %llu !--------\n", __func__, temp);

	temp = (__u64)(ctrlprops->default_exp_time.val);
	dev_info(dev, "%s : ctrlprops->default_exp_time = %llu !--------\n", __func__, temp);

	dev_info(dev, "%s: frame_length:%d, numctrls:%d, mode_prop_idx:%d, mode:%d, csi_port:%d, numlanes:%d, numfmts: %d, def_mode: %d, def_width: %d, def_height:%d, def_clk_freq:%d, fmt_width: %d, fmt_height:%d\n", 
				__func__, priv->frame_length, priv->s_data->numctrls, priv->s_data->mode_prop_idx, priv->s_data->mode, priv->s_data->csi_port, priv->s_data->numlanes, 
				priv->s_data->numfmts, priv->s_data->def_mode, priv->s_data->def_width, priv->s_data->def_height, priv->s_data->def_clk_freq, priv->s_data->fmt_width, priv->s_data->fmt_height);

	dev_info(dev, "%s: framerates:%d, num_framerates:%d, mode:%d, hdr_en:%d\n", 
				__func__, *(priv->s_data->frmfmt->framerates), priv->s_data->frmfmt->num_framerates, priv->s_data->frmfmt->mode, priv->s_data->frmfmt->hdr_en);

	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}
	else
	{
		dev_info(dev, "%s : tegracam_v4l2subdev_register end!!!--------\n", __func__);
	}

	dev_info(dev, "%s : Detected IMX586 sensor\n", __func__);

#if (PDAF_SOLUTION_NO != PDAF_SOLUTION)
	// focus motor
	err = imx586_focus_device_init(priv);
	dev_info(dev, "%s imx586_focus_device_init: %d\n", __func__, err);

	imx586_focus_init(priv);

	imx586_focus_set_position(priv, priv->dis_infinity);
	dev_info(dev, "%s imx586_focus_set_position: %d\n", __func__, priv->dis_infinity);

	// eeprom */
	err = imx586_eeprom_device_init(priv);
	dev_info(dev, "%s imx586_eeprom_device_init: %d\n", __func__, err);
#endif

#if (PDAF_SOLUTION_1 == PDAF_SOLUTION || PDAF_SOLUTION_2 == PDAF_SOLUTION)
	mutex_init(&priv->kthread_lock);
#endif

	return 0;
}

// Done
static int
imx586_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx586 *priv = (struct imx586 *)s_data->priv;
	
	dev_dbg(&client->dev,"%s: IMX586 Removed\n", __func__);

#if (PDAF_SOLUTION_NO != PDAF_SOLUTION)
	priv->streamflag = 2;
#endif

	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);

#if (PDAF_SOLUTION_1 == PDAF_SOLUTION)
	imx586_focus_device_release(priv);
	
	imx586_eeprom_device_release(priv);
#endif

	return 0;
}

static const struct i2c_device_id imx586_id[] = {
	{ "imx586_8", 0 },
	{ }
};

// Done
static struct i2c_driver imx586_i2c_driver = {
	.driver = {
		.name = "imx586_8",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(imx586_of_match),
	},
	.probe = imx586_probe,
	.remove = imx586_remove,
	.id_table = imx586_id,
};

module_i2c_driver(imx586_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for Sony imx586");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL v2");
