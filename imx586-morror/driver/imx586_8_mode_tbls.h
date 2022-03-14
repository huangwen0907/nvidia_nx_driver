/*
 * imx185_mode_tbls.h - imx185 sensor mode tables
 *
 * Copyright (c) 2016-2019, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef __IMX586_I2C_TABLES__
#define __IMX586_I2C_TABLES__

#include <media/camera_common.h>
#include <linux/miscdevice.h>

#define IMX586_TABLE_WAIT_MS	0
#define IMX586_TABLE_END	1
#define IMX586_MAX_RETRIES	3
#define IMX586_WAIT_MS_STOP	1
#define IMX586_WAIT_MS_START	30
#define IMX586_WAIT_MS_STREAM	210
#define IMX586_GAIN_TABLE_SIZE 255


/* #define INIT_ET_INSETTING 1 */
#define imx586_reg struct reg_8

//start steam mode
static imx586_reg imx586_start[] = {
	{0x0100, 0x00 },
	{IMX586_TABLE_WAIT_MS, IMX586_WAIT_MS_START},
	{0x0100, 0x01},
	{IMX586_TABLE_WAIT_MS, IMX586_WAIT_MS_STREAM},
	{IMX586_TABLE_WAIT_MS, IMX586_WAIT_MS_STREAM},
	{ IMX586_TABLE_END, 0x00 }
};

//stop steam
static imx586_reg imx586_stop[] = {
	{0x0100, 0x00 },
	{IMX586_TABLE_WAIT_MS, IMX586_WAIT_MS_STOP},
	{IMX586_TABLE_END, 0x00 }
};

static imx586_reg tp_colorbars[] = {
	//{0x300A, 0x00},/*BLC for PG*/
	//{0x300E, 0x00},
	//{0x3089, 0x00},
	//{0x308C, 0x13},
	/*
	* bit 0: PG mode enable
	* bit 1: Back Ground Transient:
	* bit [4-7]: PG mode setting, Set at 0h to Fh, suggest 1 or 5
	* raw12 max output FFEh
	*/
	{IMX586_TABLE_WAIT_MS, IMX586_WAIT_MS_STOP},
	{IMX586_TABLE_END, 0x00}
};

static imx586_reg imx586_Stand_by[] = {
	//External Clock Setting: 24M		
	{0x0136,	0x18},
	{0x0137,	0x00},
		
	//Register version		
	{0x3C7E,	0x01},		// old {0x3C7E,	0x02},
	{0x3C7F,	0x08},		// old {0x3C7F,	0x04},

	//Signaling mode setting		, CSI_SIG_MODE 2: CSI-2 Signaling D-PHY, 3: CSI-2 Signaling C-PHY
	{0x0111,	0x02},		//D-PHY	
		
	//Global Setting	
	{0x380C,	0x00},
	{0x3C00,	0x10},
	{0x3C01,	0x10},
	{0x3C02,	0x10},
	{0x3C03,	0x10},
	{0x3C04,	0x10},
	{0x3C05,	0x01},
	{0x3C06,	0x00},
	{0x3C07,	0x00},
	{0x3C08,	0x03},
	{0x3C09,	0xFF},
	{0x3C0A,	0x01},
	{0x3C0B,	0x00},
	{0x3C0C,	0x00},
	{0x3C0D,	0x03},
	{0x3C0E,	0xFF},
	{0x3C0F,	0x20},

	{0x3F88,	0x00},
	{0x3F8E,	0x00},

	{0x5282, 0x01},
	{0x9004, 0x14},
	{0x9200, 0xF4},
	{0x9201, 0xA7},
	{0x9202, 0xF4},
	{0x9203, 0xAA},
	{0x9204, 0xF4},
	{0x9205, 0xAD},
	{0x9206, 0xF4},
	{0x9207, 0xB0},
	{0x9208, 0xF4},
	{0x9209, 0xB3},
	{0x920A, 0xB7},
	{0x920B, 0x34},
	{0x920C, 0xB7},
	{0x920D, 0x36},
	{0x920E, 0xB7},
	{0x920F, 0x37},
	{0x9210, 0xB7},
	{0x9211, 0x38},
	{0x9212, 0xB7},
	{0x9213, 0x39},
	{0x9214, 0xB7},
	{0x9215, 0x3A},
	{0x9216, 0xB7},
	{0x9217, 0x3C},
	{0x9218, 0xB7},
	{0x9219, 0x3D},
	{0x921A, 0xB7},
	{0x921B, 0x3E},
	{0x921C, 0xB7},
	{0x921D, 0x3F},
	{0x921E, 0x77},
	{0x921F, 0x77},
	{0x9222, 0xC4},
	{0x9223, 0x4B},
	{0x9224, 0xC4},
	{0x9225, 0x4C},
	{0x9226, 0xC4},
	{0x9227, 0x4D},
	{0x9810, 0x14},
	{0x9814, 0x14},
	{0x99B2, 0x20},
	{0x99B3, 0x0F},
	{0x99B4, 0x0F},
	{0x99B5, 0x0F},
	{0x99B6, 0x0F},
	{0x99E4, 0x0F},
	{0x99E5, 0x0F},
	{0x99E6, 0x0F},
	{0x99E7, 0x0F},
	{0x99E8, 0x0F},
	{0x99E9, 0x0F},
	{0x99EA, 0x0F},
	{0x99EB, 0x0F},
	{0x99EC, 0x0F},
	{0x99ED, 0x0F},
	{0xA569, 0x06},
	{0xA679, 0x20},
	{0xC020, 0x01},
	{0xC61D, 0x00},
	{0xC625, 0x00},
	{0xC638, 0x03},
	{0xC63B, 0x01},
	{0xE286, 0x31},
	{0xE2A6, 0x32},
	{0xE2C6, 0x33},
	{0xBCF1, 0x00},

	{IMX586_TABLE_END, 0x00}

} ;

static  imx586_reg imx586_4000x3000_HDR_30FPS[] = {
	//CSI_SIG_MODE 2: CSI-2 Signaling D-PHY 3: CSI-2 Signaling C-PHY
	//{0x0111,	0x02},

	//MIPI output setting		
	{0x0112,	0x0A},	// CSI data format for D-PHY: 0x0a: RAW10(top 10bit of internam data)
	{0x0113,	0x0A},
	
	{0x0114,	0x03},//csi lane mode : 1: 2 lane, 3: 4 lane
	
	//Line Length PCK Setting	, length of line: 9440

	{0x0342,	0x24},
	{0x0343,	0xE0},

	//Frame Length Lines Setting, The length of frame	:6067

	{0x0340,	0x17},
	{0x0341,	0xB3},

	//ROI Setting		

	{0x0344,	0x00},
	{0x0345,	0x00},
	{0x0346,	0x00},
	{0x0347,	0x00},
	{0x0348,	0x1F},
	{0x0349,	0x3F},
	{0x034A,	0x17},
	{0x034B,	0x6F},

	//Mode Setting		

	{0x0220,	0x63},
	{0x0222,	0x01},	
	
	{0x0900,	0x00},
	{0x0901,	0x11},
	{0x0902,	0x0A},
	{0x3140,	0x04},
	{0x3246,	0x01},
	{0x3247,	0x01},
	{0x3F15,	0x00},

	//Digital Crop & Scaling		

	{0x0401,	0x00},
	{0x0404,	0x00},
	{0x0405,	0x10},
	{0x0408,	0x00},
	{0x0409,	0x00},
	{0x040A,	0x00},
	{0x040B,	0x00},
	{0x040C,	0x0F},
	{0x040D,	0xA0},
	{0x040E,	0x0B},
	{0x040F,	0xB8},

	//Output Size Setting		

	{0x034C,	0x0F},
	{0x034D,	0xA0},
	{0x034E,	0x0B},
	{0x034F,	0xB8},

	//Clock Setting		

	{0x0301,	0x05},		// IVT_SYCK_DIV
	{0x0303,	0x02},		// IVT_PXCK_DIV
	
	{0x0305,	0x04},		// IVT_PREPLLCK_DIV:  24M/4 = 6M

	//30fps
	{0x0306,	0x01},		//IVT_PLL_MPY      		IVTCT: 358 * 6 = 2148M
	{0x0307,	0x66},		//IVT_PLL_MPY
	
	{0x030B,	0x02},		// IOP_SYCK_DIV
	
	{0x030D,	0x0C},		// IOP_PREPLLCK_DIV:   24m/12=2M

	{0x030E,	0x04},		// IOP_PLL_MPY	 IOPCK: 1042 * 2 = 2084M
	{0x030F,	0x12},		// IOP_PLL_MPY
	
	{0x0310,	0x01},		// PLL mode select,   0 : Single PLL mode 1 : Dual PLL mode   

	/*
	IVTCK = 24M * 1/4 * 358 = 2148M
	Predivider setting =   1/IVT_PREPLLCK_DIV				// 1/4
	Pll multiple setting = 	 IVT_PLL_MPY								// 358

	IOPCK = 24M * 1/12 * 1042 = 2084M
	Predivider setting = 	1/IOP_PREPLLCK_DIV		1/12
	Pll multiple setting = IOP_PLL_MPY						1042

	IVTPXCK clock frequency = IVTCK x IVTPXCK clock division ratio
				IVTPXCK clock division ratio = 1 / (IVT_SYCK_DIV * IVT_PXCK_DIV)

	IOPSYCK clock frequency = IOPCK x IOPSYCK clock division ratio
   				IOPSYCK clock division ratio = 1 / (IOP_SYCK_DIV)
	 
	IVTPXCK: 2148M * 1/(0x0301: 05 * 0x0303: 02) = 2148M / 10 = 214.8M
	IOPSYCK = 2084M *  1/(IOP_SYCK_DIV) = 2084M * 1/2 = 1042M

	Frame Rate [frame/s] = Pixel_rate [pixels/s] / Total number of pixels [pixels/frame]
				Pixel rate [pixels/s] = IVTPXCK [MHz] * 8 (Total number of IVTPX channel)
				Total number of pixels [pixels/frame] = FRM_LENGTH_LINES [lines/frame] * LINE_LENGTH_PCK [pixels/line]

	Framerate = 1718.4M /57272480  = 1718400000 / 57272480 = 30.003939
		Pixel_rate = IVTPXCK * 8 = 214.8M * 8 = 1718.4M = 1718400000
		pixel_clk = Pixel_rate
		Total number of pixels = 6067 * 9440 = 57272480	
	
	*/
	
	//Other Setting

	{0x3620,	0x00},
	{0x3621,	0x00},
	{0x3C11,	0x08},
	{0x3C12,	0x08},
	{0x3C13,	0x2A},
	
	{0x3F0C,	0x00},	// BINNING_PDAF_EN:  Adjacent Pixel Binning, PDAF enable,  0:  disable, 1:  enable
	
	{0x3F14,	0x01},//  FULL_QHDR_EN:  Full Pixel and QBC HDR mode enable, 0:  disable, 1:  enable
	{0x3F80,	0x01},
	{0x3F81,	0x7C},
	{0x3F8C,	0x03},
	{0x3F8D,	0x84},
	{0x3FF8,	0x00},
	{0x3FF9,	0x00},
	{0x3FFE,	0x00},
	{0x3FFF,	0xA2},

	//Integration Setting		

	{0x0202,	0x17},
	{0x0203,	0x83},

	//{0x0350, 0x01},
	
	{0x0224,	0x17},
	{0x0225,	0x83},
	{0x3FE0,	0x17},
	{0x3FE1,	0x83},

	//Gain Setting		

	{0x0204,	0x00},
	{0x0205,	0x70},
	
	{0x0216,	0x00},
	{0x0217,	0x70},
	{0x0218,	0x01},
	{0x0219,	0x00},
	{0x020E,	0x01},
	{0x020F,	0x00},
	{0x0210,	0x01},
	{0x0211,	0x00},
	{0x0212,	0x01},
	{0x0213,	0x00},
	{0x0214,	0x01},
	{0x0215,	0x00},
	{0x3FE2,	0x00},
	{0x3FE3,	0x70},
	{0x3FE4,	0x01},
	{0x3FE5,	0x00},

	//PDAF TYPE2 Setting		
	
	{0x3E20,	0x01},
	{0x3E37,	0x01},

	//AE-Hist Setting

	{0x323B,	0x01},

	//Flicker Setting
	{0x323C,	0x01},	
	
	{IMX586_TABLE_END, 0x00}
};

static  imx586_reg imx586_4000x3000_HDR_30FPS_HDR_EXPOSURE[] = {
	/* 0x0220
	Bit[0] QBC HDR mode enable
		0 : QBC HDR disable
		1 : QBC HDR enable 
	Bit[1] Analog Gain mode select during QBC HDR
		0 : combined gain used 
		1 : separate gain used
	Bit[4:2] Reserved
	Bit[5] exposure mode select during QBC HDR
		0 : short exposure determined by ratio
		 (controlled by EXPO_RATIO(0x0222))
		1 : short exposure controlled by direct control
	Bit[6] Digital Gain mode select during QBC HDR
		0 : combined gain used 
		1 : separate gain used
	*/
	{0x0220,	0x43},

	/* 0x0222: 短和长期曝光之间的曝光比率
	QBC HDR:
		Middle exposure value = coarse_integration_time / Exposure_ratio
		Short exposure value = coarse_integration_time / Exposure_ratio^2
			1, 2, 4, 8(unsigned integer) can be set.
	*/
	{0x0222,	0x08},	

	/*
	QBC HDR function select
		4 : QBC HDR
	*/
	{0x3140,	0x04},	

	/* Long exposure mode related registers
	0x3100: CIT_LSHIFT
	Long exposure mode setting. 
		0 : Long exposure mode OFF
		 	Exposure time = COARSE_INTEG_TIME
		Other : Long exposure mode ON
			Exposure time = COARSE_INTEG_TIME << LSHIFT

		ex.
			1=2times
			2=4times
			3=8times
			 …
			7=128times (max setting)
	*/
	{0x3100,	0x03},	

	// Frame Rate [frame/s] = Pixel_rate [pixels/s] / Total number of pixels [pixels/frame]
	// Pixel_rate [pixels/s] = 214.8M * 8 = 1718.4M = 1718400000
	// Total number of pixels [pixels/frame]
	//Exposure time
	//(xx * 8 * 9440)/(214.8M * 8) = 

	{IMX586_TABLE_END, 0x00}	
};

static  imx586_reg imx586_4000x3000_HDR_30FPS_HDRSETTING[] = {
	{0x0220, 0x63},
	{0x3140, 0x04},
	{0x3620, 0x00},
	{0x0900, 0x00},
	{0x0901, 0x11},
	{0x323A, 0x00},
	{0x323B, 0x01},
	{0x323C, 0x00},
	{0x37E0, 0x00},
	{0x37E1, 0x00},
	{0x37E2, 0x00},
	{0x37E3, 0x00},
	{0x37E4, 0x0F},
	{0x37E5, 0xA0},
	{0x37E6, 0x0B},
	{0x37E7, 0xB8},
	{0x37F0, 0x00},
	{0x37F1, 0x00},
	{0x37F2, 0x00},
	{0x37F3, 0x00},
	{0x37F4, 0x0F},
	{0x37F5, 0xA0},
	{0x37F6, 0x0B},
	{0x37F7, 0xB8},
	{0x37F8, 0x03},
	{0xAE27, 0x05},
	{0xAE28, 0x05},
	{0xAE29, 0x05},
	{0x3C00, 0x10},
	{0x3C01, 0x10},
	{0x3C02, 0x10},
	{0x3C03, 0x10},
	{0x3C04, 0x10},
	{0x8943, 0x00},
	{0x3C05, 0x00},
	{0x3C0A, 0x01},
	{0x3C06, 0x00},
	{0x3C07, 0x00},
	{0x3C08, 0x03},
	{0x3C09, 0xFF},
	{0x3C0B, 0x00},
	{0x3C0C, 0x00},
	{0x3C0D, 0x03},
	{0x3C0E, 0xFF},
	{0x0B00, 0x01},
	{0x380C, 0x00},
	{0x380D, 0x80},
	{0xF501, 0x01},
	{0xF503, 0x01},
	{0xF505, 0x01},
	{0xF507, 0x01},
	{0xF509, 0x01},
	{0xF50B, 0x01},
	{0xF50D, 0x01},
	{0xF50F, 0x01},
	{0xF511, 0x01},
	{0xF513, 0x01},
	{0xF515, 0x01},
	{0xF517, 0x01},
	{0xF519, 0x01},
	{0xF51B, 0x01},
	{0xF51D, 0x01},
	{0xF006, 0x00},
	{0xF008, 0x00},
	{0xF00A, 0x00},
	{0xF012, 0x00},
	{0xF014, 0x00},
	{0xF016, 0x00},
	{0xF007, 0x08},
	{0xF009, 0x08},
	{0xF00B, 0x08},
	{0xF013, 0x10},
	{0xF015, 0x10},
	{0xF017, 0x10},
	{0xF53C, 0x00},
	{0xF53D, 0x20},
	{0xF53E, 0x00},
	{0xF53F, 0x20},
	{0xF540, 0x00},
	{0xF541, 0x20},
	{0xF542, 0x00},
	{0xF543, 0x20},
	{0xF544, 0x00},
	{0xF545, 0x20},
	{0xF546, 0x00},
	{0xF547, 0x20},
	{0xF548, 0x00},
	{0xF549, 0x20},
	{0xF54A, 0x00},
	{0xF54B, 0x20},
	{0xF54C, 0x00},
	{0xF54D, 0x20},
	{0xF54E, 0x00},
	{0xF54F, 0x20},
	{0xF550, 0x00},
	{0xF551, 0x20},
	{0xF552, 0x00},
	{0xF553, 0x20},
	{0xF554, 0x00},
	{0xF555, 0x20},
	{0xF556, 0x00},
	{0xF557, 0x20},
	{0xF558, 0x00},
	{0xF559, 0x20},
	{0xBCB9, 0x01},

	{IMX586_TABLE_END, 0x00}
};


static  imx586_reg imx586_4000x3000_Nor_H2V2_BIN_35FPS[] = {
	//CSI_SIG_MODE 2: CSI-2 Signaling D-PHY 3: CSI-2 Signaling C-PHY
	//{0x0111,	0x02},

	//MIPI output setting		
	{0x0112,	0x0A},	// CSI data format for D-PHY: 0x0a: RAW10(top 10bit of internam data)
	{0x0113,	0x0A},
	
	{0x0114,	0x03},//csi lane mode : 1: 2 lane, 3: 4 lane
	
	//Line Length PCK Setting		, length of line: 7872

	{0x0342,	0x1E},	//LINE_LENGTH_PCK
	{0x0343,	0xC0},

	//Frame Length Lines Setting	, The length of frame	:3060

	{0x0340,	0x0B},	//FRM_LENGTH_LINES
	{0x0341,	0xF4},

	//ROI Setting		

	{0x0344,	0x00},
	{0x0345,	0x00},
	{0x0346,	0x00},
	{0x0347,	0x00},
	{0x0348,	0x1F},
	{0x0349,	0x3F},
	{0x034A,	0x17},
	{0x034B,	0x6F},

	//Mode Setting		

	{0x0220,	0x62},//{0x0220,	0x00},	//
	{0x0222,	0x01},
	{0x0900,	0x01},
	{0x0901,	0x22},
	{0x0902,	0x08},
	{0x3140,	0x00},
	{0x3246,	0x81},
	{0x3247,	0x81},
	{0x3F15,	0x00},

	//Digital Crop & Scaling		

	{0x0401,	0x00},
	{0x0404,	0x00},
	{0x0405,	0x10},
	{0x0408,	0x00},
	{0x0409,	0x00},
	{0x040A,	0x00},
	{0x040B,	0x00},
	{0x040C,	0x0F},
	{0x040D,	0xA0},
	{0x040E,	0x0B},
	{0x040F,	0xB8},

	//Output Size Setting		

	{0x034C,	0x0F},
	{0x034D,	0xA0},
	{0x034E,	0x0B},
	{0x034F,	0xB8},

	//Clock Setting		

	{0x0301,	0x05},		// IVT_SYCK_DIV
	{0x0303,	0x04},		// IVT_PXCK_DIV
	
	{0x0305,	0x04},		// IVT_PREPLLCK_DIV:  24M/4 = 6M

	// 24fps
	//{0x0306,	0x00},		//IVT_PLL_MPY      IVTCT: 249 * 6 = 1494M
	//{0x0307,	0xF9},		//IVT_PLL_MPY

	//30fps
	//{0x0306,	0x01},		//IVT_PLL_MPY      IVTCT: 309 * 6 = 1854M
	//{0x0307,	0x35},		//IVT_PLL_MPY

	//35fps
	//{0x0306,	0x01},		//IVT_PLL_MPY      		IVTCT: 358 * 6 = 2148M
	//{0x0307,	0x66},		//IVT_PLL_MPY

	{0x0306,	0x01},		//IVT_PLL_MPY      		IVTCT: 360 * 6 = 2160M
	{0x0307,	0x68},		//IVT_PLL_MPY
	
	{0x030B,	0x02},		// IOP_SYCK_DIV
	//{0x030B,	0x01},		// IOP_SYCK_DIV
	
	{0x030D,	0x06},		// IOP_PREPLLCK_DIV:   24m/6=4M

	//{0x030E,	0x01},		// IOP_PLL_MPY	 IOPCK: 400 * 4 = 1600M
	//{0x030F,	0x90},		// IOP_PLL_MPY

	//{0x030E,	0x01},		// IOP_PLL_MPY	 IOPCK: 432 * 4 = 1728M
	//{0x030F,	0xB0},		// IOP_PLL_MPY
	
	//{0x030E,	0x02},		// IOP_PLL_MPY	 IOPCK: 537 * 4 = 2148M
	//{0x030F,	0x19},		// IOP_PLL_MPY

	{0x030E,	0x02},		// IOP_PLL_MPY	 IOPCK: 625 * 4 = 2500M
	{0x030F,	0x71},		// IOP_PLL_MPY
	
	{0x0310,	0x01},		// PLL mode select,   0 : Single PLL mode 1 : Dual PLL mode   

	/*
	IVTCK = 24M * 1/4 * 249 = 1494M
	Predivider setting =   1/IVT_PREPLLCK_DIV				// 1/4
	Pll multiple setting = 	 IVT_PLL_MPY								// 249

	IOPCK = 24M * 1/6 * 400 = 1600M
	Predivider setting = 	1/IOP_PREPLLCK_DIV		1/6
	Pll multiple setting = IOP_PLL_MPY						400

	IVTPXCK clock frequency = IVTCK x IVTPXCK clock division ratio
				IVTPXCK clock division ratio = 1 / (IVT_SYCK_DIV * IVT_PXCK_DIV)

	IOPSYCK clock frequency = IOPCK x IOPSYCK clock division ratio
   				IOPSYCK clock division ratio = 1 / (IOP_SYCK_DIV)
	 
	IVTPXCK: 1494M * 1/(0x0301: 05 * 0x0303: 04) = 1494 / 20 = 74.7M
	IOPSYCK = 1600M *  1/(IOP_SYCK_DIV) = 1600M * 1/2 = 800M

	Frame Rate [frame/s] = Pixel_rate [pixels/s] / Total number of pixels [pixels/frame]
				Pixel rate [pixels/s] = IVTPXCK [MHz] * 8 (Total number of IVTPX channel)
				Total number of pixels [pixels/frame] = FRM_LENGTH_LINES [lines/frame] * LINE_LENGTH_PCK [pixels/line]

	Framerate = 597.6M /24088320  = 597600000 / 24088320 = 24.808
		Pixel_rate = IVTPXCK * 8 = 74.7M * 8 = 597.6M = 597600000
		pixel_clk = Pixel_rate
		Total number of pixels = 3060 * 7872 = 24088320	



	// 30fps
	IVTCK = 24M * 1/4 * 309 = 1854M
	Predivider setting =   1/IVT_PREPLLCK_DIV		// 1/4
	Pll multiple setting = 	 IVT_PLL_MPY						// 309

	IOPCK = 24M * 1/6 * 400 = 1600M
	Predivider setting = 	1/IOP_PREPLLCK_DIV		//1/6
	Pll multiple setting = IOP_PLL_MPY						//400

	IVTPXCK: 1854M * 1/(0x0301: 05 * 0x0303: 04) = 1854M / 20 = 92.7M
	IOPSYCK = 1600M *  1/(IOP_SYCK_DIV) = 1600M * 1/2 = 800M

	Framerate = 741.6M /24088320  = 741600000 / 24088320 = 30.7867
		Pixel_rate = IVTPXCK * 8 = 92.7M * 8 = 741.6M
		pixel_clk = Pixel_rate
		Total number of pixels = 3060 * 7872 = 24088320


	//35 fps
	IVTCK = 24M * 1/4 * 360 = 2160M
	Predivider setting =   1/IVT_PREPLLCK_DIV		// 1/4
	Pll multiple setting = 	 IVT_PLL_MPY						// 360

	IOPCK = 24M * 1/6 * 625 = 2500M
	Predivider setting = 	1/IOP_PREPLLCK_DIV		//1/6
	Pll multiple setting = IOP_PLL_MPY						//625
	
	IVTPXCK: 2160M * 1/(0x0301: 05 * 0x0303: 04) = 2160M / 20 = 108M
	IOPSYCK = 2500M *  1/(IOP_SYCK_DIV) = 2500M * 1/2 = 1250M

	Framerate = 864M /24088320  = 864000000 / 24088320 = 35.86800573888092
		Pixel_rate = IVTPXCK * 8 = 108M * 8 = 864M
		pixel_clk = Pixel_rate
		Total number of pixels = 3060 * 7872 = 24088320		


	// 71fps
	IVTCK = 24M * 1/4 * 360 = 2160M
	Predivider setting =   1/IVT_PREPLLCK_DIV		// 1/4
	Pll multiple setting = 	 IVT_PLL_MPY						// 360

	IOPCK = 24M * 1/6 * 625 = 2500M
	Predivider setting = 	1/IOP_PREPLLCK_DIV		//1/6
	Pll multiple setting = IOP_PLL_MPY						//625

	
	IVTPXCK: 2160M * 1/(0x0301: 05 * 0x0303: 02) = 2160M / 10 = 216M
	IOPSYCK = 2500M *  1/(IOP_SYCK_DIV) = 2500M * 1/1 = 2500M

	Framerate = 1728M /24088320  = 1728000000 / 24088320 = 71.73601147776184
		Pixel_rate = IVTPXCK * 8 = 216M * 8 = 1728M
		pixel_clk = Pixel_rate
		Total number of pixels = 3060 * 7872 = 24088320		
	*/
	
	//Other Setting

	{0x3620,	0x00},
	{0x3621,	0x00},
	{0x3C11,	0x04},
	{0x3C12,	0x03},
	{0x3C13,	0x2D},
	
	{0x3F0C,	0x01},	// BINNING_PDAF_EN:  Adjacent Pixel Binning, PDAF enable,  0:  disable, 1:  enable
	
	{0x3F14,	0x00},//  FULL_QHDR_EN:  Full Pixel and QBC HDR mode enable, 0:  disable, 1:  enable
	{0x3F80,	0x01},
	{0x3F81,	0x90},
	{0x3F8C,	0x00},
	{0x3F8D,	0x14},
	{0x3FF8,	0x01},
	{0x3FF9,	0x2A},
	{0x3FFE,	0x00},
	{0x3FFF,	0x6C},

	//Integration Setting		

	{0x0202,	0x0B},
	{0x0203,	0xC4},

	//{0x0350, 0x01},
	
	{0x0224,	0x01},
	{0x0225,	0xF4},
	{0x3FE0,	0x01},
	{0x3FE1,	0xF4},

	//Gain Setting		

	{0x0204,	0x00},
	{0x0205,	0x70},
	
	{0x0216,	0x00},
	{0x0217,	0x70},
	{0x0218,	0x01},
	{0x0219,	0x00},
	{0x020E,	0x01},
	{0x020F,	0x00},
	{0x0210,	0x01},
	{0x0211,	0x00},
	{0x0212,	0x01},
	{0x0213,	0x00},
	{0x0214,	0x01},
	{0x0215,	0x00},
	{0x3FE2,	0x00},
	{0x3FE3,	0x70},
	{0x3FE4,	0x01},
	{0x3FE5,	0x00},

	//PDAF TYPE1 Setting		
	/*
	{0x3E20,	0x01},
	{0x3E37,	0x01},
	*/
	
	{IMX586_TABLE_END, 0x00}
};

static  imx586_reg imx586_4000x3000_HDR_30FPS_mirror[] = {
	//CSI_SIG_MODE 2: CSI-2 Signaling D-PHY 3: CSI-2 Signaling C-PHY
	//{0x0111,	0x02},

	//MIPI output setting		
	{0x0112,	0x0A},	// CSI data format for D-PHY: 0x0a: RAW10(top 10bit of internam data)
	{0x0113,	0x0A},
	
	{0x0114,	0x03},//csi lane mode : 1: 2 lane, 3: 4 lane
	
	//Line Length PCK Setting	, length of line: 9440

	{0x0342,	0x24},
	{0x0343,	0xE0},

	//Frame Length Lines Setting, The length of frame	:6067

	{0x0340,	0x17},
	{0x0341,	0xB3},

	//ROI Setting		

	{0x0344,	0x00},
	{0x0345,	0x00},
	{0x0346,	0x00},
	{0x0347,	0x00},
	{0x0348,	0x1F},
	{0x0349,	0x3F},
	{0x034A,	0x17},
	{0x034B,	0x6F},

	//Mode Setting		

	{0x0220,	0x63},
	{0x0222,	0x01},	
	
	{0x0900,	0x00},
	{0x0901,	0x11},
	{0x0902,	0x0A},
	{0x3140,	0x04},
	{0x3246,	0x01},
	{0x3247,	0x01},
	{0x3F15,	0x00},

	//Digital Crop & Scaling		

	{0x0401,	0x00},
	{0x0404,	0x00},
	{0x0405,	0x10},
	{0x0408,	0x00},
	{0x0409,	0x00},
	{0x040A,	0x00},
	{0x040B,	0x00},
	{0x040C,	0x0F},
	{0x040D,	0xA0},
	{0x040E,	0x0B},
	{0x040F,	0xB8},

	//Output Size Setting		

	{0x034C,	0x0F},
	{0x034D,	0xA0},
	{0x034E,	0x0B},
	{0x034F,	0xB8},

	//Clock Setting		

	{0x0301,	0x05},		// IVT_SYCK_DIV
	{0x0303,	0x02},		// IVT_PXCK_DIV
	
	{0x0305,	0x04},		// IVT_PREPLLCK_DIV:  24M/4 = 6M

	//30fps
	{0x0306,	0x01},		//IVT_PLL_MPY      		IVTCT: 358 * 6 = 2148M
	{0x0307,	0x66},		//IVT_PLL_MPY
	
	{0x030B,	0x02},		// IOP_SYCK_DIV
	
	{0x030D,	0x0C},		// IOP_PREPLLCK_DIV:   24m/12=2M

	{0x030E,	0x04},		// IOP_PLL_MPY	 IOPCK: 1042 * 2 = 2084M
	{0x030F,	0x12},		// IOP_PLL_MPY
	
	{0x0310,	0x01},		// PLL mode select,   0 : Single PLL mode 1 : Dual PLL mode   

	/*
	IVTCK = 24M * 1/4 * 358 = 2148M
	Predivider setting =   1/IVT_PREPLLCK_DIV				// 1/4
	Pll multiple setting = 	 IVT_PLL_MPY								// 358

	IOPCK = 24M * 1/12 * 1042 = 2084M
	Predivider setting = 	1/IOP_PREPLLCK_DIV		1/12
	Pll multiple setting = IOP_PLL_MPY						1042

	IVTPXCK clock frequency = IVTCK x IVTPXCK clock division ratio
				IVTPXCK clock division ratio = 1 / (IVT_SYCK_DIV * IVT_PXCK_DIV)

	IOPSYCK clock frequency = IOPCK x IOPSYCK clock division ratio
   				IOPSYCK clock division ratio = 1 / (IOP_SYCK_DIV)
	 
	IVTPXCK: 2148M * 1/(0x0301: 05 * 0x0303: 02) = 2148M / 10 = 214.8M
	IOPSYCK = 2084M *  1/(IOP_SYCK_DIV) = 2084M * 1/2 = 1042M

	Frame Rate [frame/s] = Pixel_rate [pixels/s] / Total number of pixels [pixels/frame]
				Pixel rate [pixels/s] = IVTPXCK [MHz] * 8 (Total number of IVTPX channel)
				Total number of pixels [pixels/frame] = FRM_LENGTH_LINES [lines/frame] * LINE_LENGTH_PCK [pixels/line]

	Framerate = 1718.4M /57272480  = 1718400000 / 57272480 = 30.003939
		Pixel_rate = IVTPXCK * 8 = 214.8M * 8 = 1718.4M = 1718400000
		pixel_clk = Pixel_rate
		Total number of pixels = 6067 * 9440 = 57272480	
	
	*/
	
	//Other Setting

	{0x3620,	0x00},
	{0x3621,	0x00},
	{0x3C11,	0x08},
	{0x3C12,	0x08},
	{0x3C13,	0x2A},
	
	{0x3F0C,	0x00},	// BINNING_PDAF_EN:  Adjacent Pixel Binning, PDAF enable,  0:  disable, 1:  enable
	
	{0x3F14,	0x01},//  FULL_QHDR_EN:  Full Pixel and QBC HDR mode enable, 0:  disable, 1:  enable
	{0x3F80,	0x01},
	{0x3F81,	0x7C},
	{0x3F8C,	0x03},
	{0x3F8D,	0x84},
	{0x3FF8,	0x00},
	{0x3FF9,	0x00},
	{0x3FFE,	0x00},
	{0x3FFF,	0xA2},

	//Integration Setting		

	{0x0202,	0x17},
	{0x0203,	0x83},

	//{0x0350, 0x01},
	
	{0x0224,	0x17},
	{0x0225,	0x83},
	{0x3FE0,	0x17},
	{0x3FE1,	0x83},

	//Gain Setting		

	{0x0204,	0x00},
	{0x0205,	0x70},
	
	{0x0216,	0x00},
	{0x0217,	0x70},
	{0x0218,	0x01},
	{0x0219,	0x00},
	{0x020E,	0x01},
	{0x020F,	0x00},
	{0x0210,	0x01},
	{0x0211,	0x00},
	{0x0212,	0x01},
	{0x0213,	0x00},
	{0x0214,	0x01},
	{0x0215,	0x00},
	{0x3FE2,	0x00},
	{0x3FE3,	0x70},
	{0x3FE4,	0x01},
	{0x3FE5,	0x00},

	//PDAF TYPE2 Setting		
	
	{0x3E20,	0x01},
	{0x3E37,	0x01},

	//AE-Hist Setting

	{0x323B,	0x01},

	//Flicker Setting
	{0x323C,	0x01},	
	
	{IMX586_TABLE_END, 0x00}
};

static  imx586_reg imx586_4000x3000_Nor_H2V2_BIN_35FPS_mirror[] = {
	//CSI_SIG_MODE 2: CSI-2 Signaling D-PHY 3: CSI-2 Signaling C-PHY
	//{0x0111,	0x02},

	//MIPI output setting		
	{0x0112,	0x0A},	// CSI data format for D-PHY: 0x0a: RAW10(top 10bit of internam data)
	{0x0113,	0x0A},
	
	{0x0114,	0x03},//csi lane mode : 1: 2 lane, 3: 4 lane
	
	//Line Length PCK Setting		, length of line: 7872

	{0x0342,	0x1E},	//LINE_LENGTH_PCK
	{0x0343,	0xC0},

	//Frame Length Lines Setting	, The length of frame	:3060

	{0x0340,	0x0B},	//FRM_LENGTH_LINES
	{0x0341,	0xF4},

	//ROI Setting		

	{0x0344,	0x00},
	{0x0345,	0x00},
	{0x0346,	0x00},
	{0x0347,	0x00},
	{0x0348,	0x1F},
	{0x0349,	0x3F},
	{0x034A,	0x17},
	{0x034B,	0x6F},

	//Mode Setting		

	{0x0220,	0x62},//{0x0220,	0x00},	//
	{0x0222,	0x01},
	{0x0900,	0x01},
	{0x0901,	0x22},
	{0x0902,	0x08},
	{0x3140,	0x00},
	{0x3246,	0x81},
	{0x3247,	0x81},
	{0x3F15,	0x00},

	//Digital Crop & Scaling		

	{0x0401,	0x00},
	{0x0404,	0x00},
	{0x0405,	0x10},
	{0x0408,	0x00},
	{0x0409,	0x00},
	{0x040A,	0x00},
	{0x040B,	0x00},
	{0x040C,	0x0F},
	{0x040D,	0xA0},
	{0x040E,	0x0B},
	{0x040F,	0xB8},

	//Output Size Setting		

	{0x034C,	0x0F},
	{0x034D,	0xA0},
	{0x034E,	0x0B},
	{0x034F,	0xB8},

	//Clock Setting		

	{0x0301,	0x05},		// IVT_SYCK_DIV
	{0x0303,	0x04},		// IVT_PXCK_DIV
	
	{0x0305,	0x04},		// IVT_PREPLLCK_DIV:  24M/4 = 6M

	// 24fps
	//{0x0306,	0x00},		//IVT_PLL_MPY      IVTCT: 249 * 6 = 1494M
	//{0x0307,	0xF9},		//IVT_PLL_MPY

	//30fps
	//{0x0306,	0x01},		//IVT_PLL_MPY      IVTCT: 309 * 6 = 1854M
	//{0x0307,	0x35},		//IVT_PLL_MPY

	//35fps
	//{0x0306,	0x01},		//IVT_PLL_MPY      		IVTCT: 358 * 6 = 2148M
	//{0x0307,	0x66},		//IVT_PLL_MPY

	{0x0306,	0x01},		//IVT_PLL_MPY      		IVTCT: 360 * 6 = 2160M
	{0x0307,	0x68},		//IVT_PLL_MPY
	
	{0x030B,	0x02},		// IOP_SYCK_DIV
	//{0x030B,	0x01},		// IOP_SYCK_DIV
	
	{0x030D,	0x06},		// IOP_PREPLLCK_DIV:   24m/6=4M

	//{0x030E,	0x01},		// IOP_PLL_MPY	 IOPCK: 400 * 4 = 1600M
	//{0x030F,	0x90},		// IOP_PLL_MPY

	//{0x030E,	0x01},		// IOP_PLL_MPY	 IOPCK: 432 * 4 = 1728M
	//{0x030F,	0xB0},		// IOP_PLL_MPY
	
	//{0x030E,	0x02},		// IOP_PLL_MPY	 IOPCK: 537 * 4 = 2148M
	//{0x030F,	0x19},		// IOP_PLL_MPY

	{0x030E,	0x02},		// IOP_PLL_MPY	 IOPCK: 625 * 4 = 2500M
	{0x030F,	0x71},		// IOP_PLL_MPY
	
	{0x0310,	0x01},		// PLL mode select,   0 : Single PLL mode 1 : Dual PLL mode   

	/*
	IVTCK = 24M * 1/4 * 249 = 1494M
	Predivider setting =   1/IVT_PREPLLCK_DIV				// 1/4
	Pll multiple setting = 	 IVT_PLL_MPY								// 249

	IOPCK = 24M * 1/6 * 400 = 1600M
	Predivider setting = 	1/IOP_PREPLLCK_DIV		1/6
	Pll multiple setting = IOP_PLL_MPY						400

	IVTPXCK clock frequency = IVTCK x IVTPXCK clock division ratio
				IVTPXCK clock division ratio = 1 / (IVT_SYCK_DIV * IVT_PXCK_DIV)

	IOPSYCK clock frequency = IOPCK x IOPSYCK clock division ratio
   				IOPSYCK clock division ratio = 1 / (IOP_SYCK_DIV)
	 
	IVTPXCK: 1494M * 1/(0x0301: 05 * 0x0303: 04) = 1494 / 20 = 74.7M
	IOPSYCK = 1600M *  1/(IOP_SYCK_DIV) = 1600M * 1/2 = 800M

	Frame Rate [frame/s] = Pixel_rate [pixels/s] / Total number of pixels [pixels/frame]
				Pixel rate [pixels/s] = IVTPXCK [MHz] * 8 (Total number of IVTPX channel)
				Total number of pixels [pixels/frame] = FRM_LENGTH_LINES [lines/frame] * LINE_LENGTH_PCK [pixels/line]

	Framerate = 597.6M /24088320  = 597600000 / 24088320 = 24.808
		Pixel_rate = IVTPXCK * 8 = 74.7M * 8 = 597.6M = 597600000
		pixel_clk = Pixel_rate
		Total number of pixels = 3060 * 7872 = 24088320	



	// 30fps
	IVTCK = 24M * 1/4 * 309 = 1854M
	Predivider setting =   1/IVT_PREPLLCK_DIV		// 1/4
	Pll multiple setting = 	 IVT_PLL_MPY						// 309

	IOPCK = 24M * 1/6 * 400 = 1600M
	Predivider setting = 	1/IOP_PREPLLCK_DIV		//1/6
	Pll multiple setting = IOP_PLL_MPY						//400

	IVTPXCK: 1854M * 1/(0x0301: 05 * 0x0303: 04) = 1854M / 20 = 92.7M
	IOPSYCK = 1600M *  1/(IOP_SYCK_DIV) = 1600M * 1/2 = 800M

	Framerate = 741.6M /24088320  = 741600000 / 24088320 = 30.7867
		Pixel_rate = IVTPXCK * 8 = 92.7M * 8 = 741.6M
		pixel_clk = Pixel_rate
		Total number of pixels = 3060 * 7872 = 24088320


	//35 fps
	IVTCK = 24M * 1/4 * 360 = 2160M
	Predivider setting =   1/IVT_PREPLLCK_DIV		// 1/4
	Pll multiple setting = 	 IVT_PLL_MPY						// 360

	IOPCK = 24M * 1/6 * 625 = 2500M
	Predivider setting = 	1/IOP_PREPLLCK_DIV		//1/6
	Pll multiple setting = IOP_PLL_MPY						//625
	
	IVTPXCK: 2160M * 1/(0x0301: 05 * 0x0303: 04) = 2160M / 20 = 108M
	IOPSYCK = 2500M *  1/(IOP_SYCK_DIV) = 2500M * 1/2 = 1250M

	Framerate = 864M /24088320  = 864000000 / 24088320 = 35.86800573888092
		Pixel_rate = IVTPXCK * 8 = 108M * 8 = 864M
		pixel_clk = Pixel_rate
		Total number of pixels = 3060 * 7872 = 24088320		


	// 71fps
	IVTCK = 24M * 1/4 * 360 = 2160M
	Predivider setting =   1/IVT_PREPLLCK_DIV		// 1/4
	Pll multiple setting = 	 IVT_PLL_MPY						// 360

	IOPCK = 24M * 1/6 * 625 = 2500M
	Predivider setting = 	1/IOP_PREPLLCK_DIV		//1/6
	Pll multiple setting = IOP_PLL_MPY						//625

	
	IVTPXCK: 2160M * 1/(0x0301: 05 * 0x0303: 02) = 2160M / 10 = 216M
	IOPSYCK = 2500M *  1/(IOP_SYCK_DIV) = 2500M * 1/1 = 2500M

	Framerate = 1728M /24088320  = 1728000000 / 24088320 = 71.73601147776184
		Pixel_rate = IVTPXCK * 8 = 216M * 8 = 1728M
		pixel_clk = Pixel_rate
		Total number of pixels = 3060 * 7872 = 24088320		
	*/
	
	//Other Setting

	{0x3620,	0x00},
	{0x3621,	0x00},
	{0x3C11,	0x04},
	{0x3C12,	0x03},
	{0x3C13,	0x2D},
	
	{0x3F0C,	0x01},	// BINNING_PDAF_EN:  Adjacent Pixel Binning, PDAF enable,  0:  disable, 1:  enable
	
	{0x3F14,	0x00},//  FULL_QHDR_EN:  Full Pixel and QBC HDR mode enable, 0:  disable, 1:  enable
	{0x3F80,	0x01},
	{0x3F81,	0x90},
	{0x3F8C,	0x00},
	{0x3F8D,	0x14},
	{0x3FF8,	0x01},
	{0x3FF9,	0x2A},
	{0x3FFE,	0x00},
	{0x3FFF,	0x6C},

	//Integration Setting		

	{0x0202,	0x0B},
	{0x0203,	0xC4},

	//{0x0350, 0x01},
	
	{0x0224,	0x01},
	{0x0225,	0xF4},
	{0x3FE0,	0x01},
	{0x3FE1,	0xF4},

	//Gain Setting		

	{0x0204,	0x00},
	{0x0205,	0x70},
	
	{0x0216,	0x00},
	{0x0217,	0x70},
	{0x0218,	0x01},
	{0x0219,	0x00},
	{0x020E,	0x01},
	{0x020F,	0x00},
	{0x0210,	0x01},
	{0x0211,	0x00},
	{0x0212,	0x01},
	{0x0213,	0x00},
	{0x0214,	0x01},
	{0x0215,	0x00},
	{0x3FE2,	0x00},
	{0x3FE3,	0x70},
	{0x3FE4,	0x01},
	{0x3FE5,	0x00},

	//PDAF TYPE1 Setting		
	/*
	{0x3E20,	0x01},
	{0x3E37,	0x01},
	*/
	
	{IMX586_TABLE_END, 0x00}
};

static  imx586_reg imx586_4000x3000_HDR_30FPS_flip[] = {
	//CSI_SIG_MODE 2: CSI-2 Signaling D-PHY 3: CSI-2 Signaling C-PHY
	//{0x0111,	0x02},

	//MIPI output setting		
	{0x0112,	0x0A},	// CSI data format for D-PHY: 0x0a: RAW10(top 10bit of internam data)
	{0x0113,	0x0A},
	
	{0x0114,	0x03},//csi lane mode : 1: 2 lane, 3: 4 lane
	
	//Line Length PCK Setting	, length of line: 9440

	{0x0342,	0x24},
	{0x0343,	0xE0},

	//Frame Length Lines Setting, The length of frame	:6067

	{0x0340,	0x17},
	{0x0341,	0xB3},

	//ROI Setting		

	{0x0344,	0x00},
	{0x0345,	0x00},
	{0x0346,	0x00},
	{0x0347,	0x00},
	{0x0348,	0x1F},
	{0x0349,	0x3F},
	{0x034A,	0x17},
	{0x034B,	0x6F},

	//Mode Setting		

	{0x0220,	0x63},
	{0x0222,	0x01},	
	
	{0x0900,	0x00},
	{0x0901,	0x11},
	{0x0902,	0x0A},
	{0x3140,	0x04},
	{0x3246,	0x01},
	{0x3247,	0x01},
	{0x3F15,	0x00},

	//Digital Crop & Scaling		

	{0x0401,	0x00},
	{0x0404,	0x00},
	{0x0405,	0x10},
	{0x0408,	0x00},
	{0x0409,	0x00},
	{0x040A,	0x00},
	{0x040B,	0x00},
	{0x040C,	0x0F},
	{0x040D,	0xA0},
	{0x040E,	0x0B},
	{0x040F,	0xB8},

	//Output Size Setting		

	{0x034C,	0x0F},
	{0x034D,	0xA0},
	{0x034E,	0x0B},
	{0x034F,	0xB8},

	//Clock Setting		

	{0x0301,	0x05},		// IVT_SYCK_DIV
	{0x0303,	0x02},		// IVT_PXCK_DIV
	
	{0x0305,	0x04},		// IVT_PREPLLCK_DIV:  24M/4 = 6M

	//30fps
	{0x0306,	0x01},		//IVT_PLL_MPY      		IVTCT: 358 * 6 = 2148M
	{0x0307,	0x66},		//IVT_PLL_MPY
	
	{0x030B,	0x02},		// IOP_SYCK_DIV
	
	{0x030D,	0x0C},		// IOP_PREPLLCK_DIV:   24m/12=2M

	{0x030E,	0x04},		// IOP_PLL_MPY	 IOPCK: 1042 * 2 = 2084M
	{0x030F,	0x12},		// IOP_PLL_MPY
	
	{0x0310,	0x01},		// PLL mode select,   0 : Single PLL mode 1 : Dual PLL mode   

	/*
	IVTCK = 24M * 1/4 * 358 = 2148M
	Predivider setting =   1/IVT_PREPLLCK_DIV				// 1/4
	Pll multiple setting = 	 IVT_PLL_MPY								// 358

	IOPCK = 24M * 1/12 * 1042 = 2084M
	Predivider setting = 	1/IOP_PREPLLCK_DIV		1/12
	Pll multiple setting = IOP_PLL_MPY						1042

	IVTPXCK clock frequency = IVTCK x IVTPXCK clock division ratio
				IVTPXCK clock division ratio = 1 / (IVT_SYCK_DIV * IVT_PXCK_DIV)

	IOPSYCK clock frequency = IOPCK x IOPSYCK clock division ratio
   				IOPSYCK clock division ratio = 1 / (IOP_SYCK_DIV)
	 
	IVTPXCK: 2148M * 1/(0x0301: 05 * 0x0303: 02) = 2148M / 10 = 214.8M
	IOPSYCK = 2084M *  1/(IOP_SYCK_DIV) = 2084M * 1/2 = 1042M

	Frame Rate [frame/s] = Pixel_rate [pixels/s] / Total number of pixels [pixels/frame]
				Pixel rate [pixels/s] = IVTPXCK [MHz] * 8 (Total number of IVTPX channel)
				Total number of pixels [pixels/frame] = FRM_LENGTH_LINES [lines/frame] * LINE_LENGTH_PCK [pixels/line]

	Framerate = 1718.4M /57272480  = 1718400000 / 57272480 = 30.003939
		Pixel_rate = IVTPXCK * 8 = 214.8M * 8 = 1718.4M = 1718400000
		pixel_clk = Pixel_rate
		Total number of pixels = 6067 * 9440 = 57272480	
	
	*/
	
	//Other Setting

	{0x3620,	0x00},
	{0x3621,	0x00},
	{0x3C11,	0x08},
	{0x3C12,	0x08},
	{0x3C13,	0x2A},
	
	{0x3F0C,	0x00},	// BINNING_PDAF_EN:  Adjacent Pixel Binning, PDAF enable,  0:  disable, 1:  enable
	
	{0x3F14,	0x01},//  FULL_QHDR_EN:  Full Pixel and QBC HDR mode enable, 0:  disable, 1:  enable
	{0x3F80,	0x01},
	{0x3F81,	0x7C},
	{0x3F8C,	0x03},
	{0x3F8D,	0x84},
	{0x3FF8,	0x00},
	{0x3FF9,	0x00},
	{0x3FFE,	0x00},
	{0x3FFF,	0xA2},

	//Integration Setting		

	{0x0202,	0x17},
	{0x0203,	0x83},

	//{0x0350, 0x01},
	
	{0x0224,	0x17},
	{0x0225,	0x83},
	{0x3FE0,	0x17},
	{0x3FE1,	0x83},

	//Gain Setting		

	{0x0204,	0x00},
	{0x0205,	0x70},
	
	{0x0216,	0x00},
	{0x0217,	0x70},
	{0x0218,	0x01},
	{0x0219,	0x00},
	{0x020E,	0x01},
	{0x020F,	0x00},
	{0x0210,	0x01},
	{0x0211,	0x00},
	{0x0212,	0x01},
	{0x0213,	0x00},
	{0x0214,	0x01},
	{0x0215,	0x00},
	{0x3FE2,	0x00},
	{0x3FE3,	0x70},
	{0x3FE4,	0x01},
	{0x3FE5,	0x00},

	//PDAF TYPE2 Setting		
	
	{0x3E20,	0x01},
	{0x3E37,	0x01},

	//AE-Hist Setting

	{0x323B,	0x01},

	//Flicker Setting
	{0x323C,	0x01},	
	
	{IMX586_TABLE_END, 0x00}
};

static  imx586_reg imx586_4000x3000_Nor_H2V2_BIN_35FPS_flip[] = {
	//CSI_SIG_MODE 2: CSI-2 Signaling D-PHY 3: CSI-2 Signaling C-PHY
	//{0x0111,	0x02},

	//MIPI output setting		
	{0x0112,	0x0A},	// CSI data format for D-PHY: 0x0a: RAW10(top 10bit of internam data)
	{0x0113,	0x0A},
	
	{0x0114,	0x03},//csi lane mode : 1: 2 lane, 3: 4 lane
	
	//Line Length PCK Setting		, length of line: 7872

	{0x0342,	0x1E},	//LINE_LENGTH_PCK
	{0x0343,	0xC0},

	//Frame Length Lines Setting	, The length of frame	:3060

	{0x0340,	0x0B},	//FRM_LENGTH_LINES
	{0x0341,	0xF4},

	//ROI Setting		

	{0x0344,	0x00},
	{0x0345,	0x00},
	{0x0346,	0x00},
	{0x0347,	0x00},
	{0x0348,	0x1F},
	{0x0349,	0x3F},
	{0x034A,	0x17},
	{0x034B,	0x6F},

	//Mode Setting		

	{0x0220,	0x62},//{0x0220,	0x00},	//
	{0x0222,	0x01},
	{0x0900,	0x01},
	{0x0901,	0x22},
	{0x0902,	0x08},
	{0x3140,	0x00},
	{0x3246,	0x81},
	{0x3247,	0x81},
	{0x3F15,	0x00},

	//Digital Crop & Scaling		

	{0x0401,	0x00},
	{0x0404,	0x00},
	{0x0405,	0x10},
	{0x0408,	0x00},
	{0x0409,	0x00},
	{0x040A,	0x00},
	{0x040B,	0x00},
	{0x040C,	0x0F},
	{0x040D,	0xA0},
	{0x040E,	0x0B},
	{0x040F,	0xB8},

	//Output Size Setting		

	{0x034C,	0x0F},
	{0x034D,	0xA0},
	{0x034E,	0x0B},
	{0x034F,	0xB8},

	//Clock Setting		

	{0x0301,	0x05},		// IVT_SYCK_DIV
	{0x0303,	0x04},		// IVT_PXCK_DIV
	
	{0x0305,	0x04},		// IVT_PREPLLCK_DIV:  24M/4 = 6M

	// 24fps
	//{0x0306,	0x00},		//IVT_PLL_MPY      IVTCT: 249 * 6 = 1494M
	//{0x0307,	0xF9},		//IVT_PLL_MPY

	//30fps
	//{0x0306,	0x01},		//IVT_PLL_MPY      IVTCT: 309 * 6 = 1854M
	//{0x0307,	0x35},		//IVT_PLL_MPY

	//35fps
	//{0x0306,	0x01},		//IVT_PLL_MPY      		IVTCT: 358 * 6 = 2148M
	//{0x0307,	0x66},		//IVT_PLL_MPY

	{0x0306,	0x01},		//IVT_PLL_MPY      		IVTCT: 360 * 6 = 2160M
	{0x0307,	0x68},		//IVT_PLL_MPY
	
	{0x030B,	0x02},		// IOP_SYCK_DIV
	//{0x030B,	0x01},		// IOP_SYCK_DIV
	
	{0x030D,	0x06},		// IOP_PREPLLCK_DIV:   24m/6=4M

	//{0x030E,	0x01},		// IOP_PLL_MPY	 IOPCK: 400 * 4 = 1600M
	//{0x030F,	0x90},		// IOP_PLL_MPY

	//{0x030E,	0x01},		// IOP_PLL_MPY	 IOPCK: 432 * 4 = 1728M
	//{0x030F,	0xB0},		// IOP_PLL_MPY
	
	//{0x030E,	0x02},		// IOP_PLL_MPY	 IOPCK: 537 * 4 = 2148M
	//{0x030F,	0x19},		// IOP_PLL_MPY

	{0x030E,	0x02},		// IOP_PLL_MPY	 IOPCK: 625 * 4 = 2500M
	{0x030F,	0x71},		// IOP_PLL_MPY
	
	{0x0310,	0x01},		// PLL mode select,   0 : Single PLL mode 1 : Dual PLL mode   

	/*
	IVTCK = 24M * 1/4 * 249 = 1494M
	Predivider setting =   1/IVT_PREPLLCK_DIV				// 1/4
	Pll multiple setting = 	 IVT_PLL_MPY								// 249

	IOPCK = 24M * 1/6 * 400 = 1600M
	Predivider setting = 	1/IOP_PREPLLCK_DIV		1/6
	Pll multiple setting = IOP_PLL_MPY						400

	IVTPXCK clock frequency = IVTCK x IVTPXCK clock division ratio
				IVTPXCK clock division ratio = 1 / (IVT_SYCK_DIV * IVT_PXCK_DIV)

	IOPSYCK clock frequency = IOPCK x IOPSYCK clock division ratio
   				IOPSYCK clock division ratio = 1 / (IOP_SYCK_DIV)
	 
	IVTPXCK: 1494M * 1/(0x0301: 05 * 0x0303: 04) = 1494 / 20 = 74.7M
	IOPSYCK = 1600M *  1/(IOP_SYCK_DIV) = 1600M * 1/2 = 800M

	Frame Rate [frame/s] = Pixel_rate [pixels/s] / Total number of pixels [pixels/frame]
				Pixel rate [pixels/s] = IVTPXCK [MHz] * 8 (Total number of IVTPX channel)
				Total number of pixels [pixels/frame] = FRM_LENGTH_LINES [lines/frame] * LINE_LENGTH_PCK [pixels/line]

	Framerate = 597.6M /24088320  = 597600000 / 24088320 = 24.808
		Pixel_rate = IVTPXCK * 8 = 74.7M * 8 = 597.6M = 597600000
		pixel_clk = Pixel_rate
		Total number of pixels = 3060 * 7872 = 24088320	



	// 30fps
	IVTCK = 24M * 1/4 * 309 = 1854M
	Predivider setting =   1/IVT_PREPLLCK_DIV		// 1/4
	Pll multiple setting = 	 IVT_PLL_MPY						// 309

	IOPCK = 24M * 1/6 * 400 = 1600M
	Predivider setting = 	1/IOP_PREPLLCK_DIV		//1/6
	Pll multiple setting = IOP_PLL_MPY						//400

	IVTPXCK: 1854M * 1/(0x0301: 05 * 0x0303: 04) = 1854M / 20 = 92.7M
	IOPSYCK = 1600M *  1/(IOP_SYCK_DIV) = 1600M * 1/2 = 800M

	Framerate = 741.6M /24088320  = 741600000 / 24088320 = 30.7867
		Pixel_rate = IVTPXCK * 8 = 92.7M * 8 = 741.6M
		pixel_clk = Pixel_rate
		Total number of pixels = 3060 * 7872 = 24088320


	//35 fps
	IVTCK = 24M * 1/4 * 360 = 2160M
	Predivider setting =   1/IVT_PREPLLCK_DIV		// 1/4
	Pll multiple setting = 	 IVT_PLL_MPY						// 360

	IOPCK = 24M * 1/6 * 625 = 2500M
	Predivider setting = 	1/IOP_PREPLLCK_DIV		//1/6
	Pll multiple setting = IOP_PLL_MPY						//625
	
	IVTPXCK: 2160M * 1/(0x0301: 05 * 0x0303: 04) = 2160M / 20 = 108M
	IOPSYCK = 2500M *  1/(IOP_SYCK_DIV) = 2500M * 1/2 = 1250M

	Framerate = 864M /24088320  = 864000000 / 24088320 = 35.86800573888092
		Pixel_rate = IVTPXCK * 8 = 108M * 8 = 864M
		pixel_clk = Pixel_rate
		Total number of pixels = 3060 * 7872 = 24088320		


	// 71fps
	IVTCK = 24M * 1/4 * 360 = 2160M
	Predivider setting =   1/IVT_PREPLLCK_DIV		// 1/4
	Pll multiple setting = 	 IVT_PLL_MPY						// 360

	IOPCK = 24M * 1/6 * 625 = 2500M
	Predivider setting = 	1/IOP_PREPLLCK_DIV		//1/6
	Pll multiple setting = IOP_PLL_MPY						//625

	
	IVTPXCK: 2160M * 1/(0x0301: 05 * 0x0303: 02) = 2160M / 10 = 216M
	IOPSYCK = 2500M *  1/(IOP_SYCK_DIV) = 2500M * 1/1 = 2500M

	Framerate = 1728M /24088320  = 1728000000 / 24088320 = 71.73601147776184
		Pixel_rate = IVTPXCK * 8 = 216M * 8 = 1728M
		pixel_clk = Pixel_rate
		Total number of pixels = 3060 * 7872 = 24088320		
	*/
	
	//Other Setting

	{0x3620,	0x00},
	{0x3621,	0x00},
	{0x3C11,	0x04},
	{0x3C12,	0x03},
	{0x3C13,	0x2D},
	
	{0x3F0C,	0x01},	// BINNING_PDAF_EN:  Adjacent Pixel Binning, PDAF enable,  0:  disable, 1:  enable
	
	{0x3F14,	0x00},//  FULL_QHDR_EN:  Full Pixel and QBC HDR mode enable, 0:  disable, 1:  enable
	{0x3F80,	0x01},
	{0x3F81,	0x90},
	{0x3F8C,	0x00},
	{0x3F8D,	0x14},
	{0x3FF8,	0x01},
	{0x3FF9,	0x2A},
	{0x3FFE,	0x00},
	{0x3FFF,	0x6C},

	//Integration Setting		

	{0x0202,	0x0B},
	{0x0203,	0xC4},

	//{0x0350, 0x01},
	
	{0x0224,	0x01},
	{0x0225,	0xF4},
	{0x3FE0,	0x01},
	{0x3FE1,	0xF4},

	//Gain Setting		

	{0x0204,	0x00},
	{0x0205,	0x70},
	
	{0x0216,	0x00},
	{0x0217,	0x70},
	{0x0218,	0x01},
	{0x0219,	0x00},
	{0x020E,	0x01},
	{0x020F,	0x00},
	{0x0210,	0x01},
	{0x0211,	0x00},
	{0x0212,	0x01},
	{0x0213,	0x00},
	{0x0214,	0x01},
	{0x0215,	0x00},
	{0x3FE2,	0x00},
	{0x3FE3,	0x70},
	{0x3FE4,	0x01},
	{0x3FE5,	0x00},

	//PDAF TYPE1 Setting		
	/*
	{0x3E20,	0x01},
	{0x3E37,	0x01},
	*/
	
	{IMX586_TABLE_END, 0x00}
};

static  imx586_reg imx586_4000x3000_HDR_30FPS_mirror_flip[] = {
	//CSI_SIG_MODE 2: CSI-2 Signaling D-PHY 3: CSI-2 Signaling C-PHY
	//{0x0111,	0x02},

	//MIPI output setting		
	{0x0112,	0x0A},	// CSI data format for D-PHY: 0x0a: RAW10(top 10bit of internam data)
	{0x0113,	0x0A},
	
	{0x0114,	0x03},//csi lane mode : 1: 2 lane, 3: 4 lane
	
	//Line Length PCK Setting	, length of line: 9440

	{0x0342,	0x24},
	{0x0343,	0xE0},

	//Frame Length Lines Setting, The length of frame	:6067

	{0x0340,	0x17},
	{0x0341,	0xB3},

	//ROI Setting		

	{0x0344,	0x00},
	{0x0345,	0x00},
	{0x0346,	0x00},
	{0x0347,	0x00},
	{0x0348,	0x1F},
	{0x0349,	0x3F},
	{0x034A,	0x17},
	{0x034B,	0x6F},

	//Mode Setting		

	{0x0220,	0x63},
	{0x0222,	0x01},	
	
	{0x0900,	0x00},
	{0x0901,	0x11},
	{0x0902,	0x0A},
	{0x3140,	0x04},
	{0x3246,	0x01},
	{0x3247,	0x01},
	{0x3F15,	0x00},

	//Digital Crop & Scaling		

	{0x0401,	0x00},
	{0x0404,	0x00},
	{0x0405,	0x10},
	{0x0408,	0x00},
	{0x0409,	0x00},
	{0x040A,	0x00},
	{0x040B,	0x00},
	{0x040C,	0x0F},
	{0x040D,	0xA0},
	{0x040E,	0x0B},
	{0x040F,	0xB8},

	//Output Size Setting		

	{0x034C,	0x0F},
	{0x034D,	0xA0},
	{0x034E,	0x0B},
	{0x034F,	0xB8},

	//Clock Setting		

	{0x0301,	0x05},		// IVT_SYCK_DIV
	{0x0303,	0x02},		// IVT_PXCK_DIV
	
	{0x0305,	0x04},		// IVT_PREPLLCK_DIV:  24M/4 = 6M

	//30fps
	{0x0306,	0x01},		//IVT_PLL_MPY      		IVTCT: 358 * 6 = 2148M
	{0x0307,	0x66},		//IVT_PLL_MPY
	
	{0x030B,	0x02},		// IOP_SYCK_DIV
	
	{0x030D,	0x0C},		// IOP_PREPLLCK_DIV:   24m/12=2M

	{0x030E,	0x04},		// IOP_PLL_MPY	 IOPCK: 1042 * 2 = 2084M
	{0x030F,	0x12},		// IOP_PLL_MPY
	
	{0x0310,	0x01},		// PLL mode select,   0 : Single PLL mode 1 : Dual PLL mode   

	/*
	IVTCK = 24M * 1/4 * 358 = 2148M
	Predivider setting =   1/IVT_PREPLLCK_DIV				// 1/4
	Pll multiple setting = 	 IVT_PLL_MPY								// 358

	IOPCK = 24M * 1/12 * 1042 = 2084M
	Predivider setting = 	1/IOP_PREPLLCK_DIV		1/12
	Pll multiple setting = IOP_PLL_MPY						1042

	IVTPXCK clock frequency = IVTCK x IVTPXCK clock division ratio
				IVTPXCK clock division ratio = 1 / (IVT_SYCK_DIV * IVT_PXCK_DIV)

	IOPSYCK clock frequency = IOPCK x IOPSYCK clock division ratio
   				IOPSYCK clock division ratio = 1 / (IOP_SYCK_DIV)
	 
	IVTPXCK: 2148M * 1/(0x0301: 05 * 0x0303: 02) = 2148M / 10 = 214.8M
	IOPSYCK = 2084M *  1/(IOP_SYCK_DIV) = 2084M * 1/2 = 1042M

	Frame Rate [frame/s] = Pixel_rate [pixels/s] / Total number of pixels [pixels/frame]
				Pixel rate [pixels/s] = IVTPXCK [MHz] * 8 (Total number of IVTPX channel)
				Total number of pixels [pixels/frame] = FRM_LENGTH_LINES [lines/frame] * LINE_LENGTH_PCK [pixels/line]

	Framerate = 1718.4M /57272480  = 1718400000 / 57272480 = 30.003939
		Pixel_rate = IVTPXCK * 8 = 214.8M * 8 = 1718.4M = 1718400000
		pixel_clk = Pixel_rate
		Total number of pixels = 6067 * 9440 = 57272480	
	
	*/
	
	//Other Setting

	{0x3620,	0x00},
	{0x3621,	0x00},
	{0x3C11,	0x08},
	{0x3C12,	0x08},
	{0x3C13,	0x2A},
	
	{0x3F0C,	0x00},	// BINNING_PDAF_EN:  Adjacent Pixel Binning, PDAF enable,  0:  disable, 1:  enable
	
	{0x3F14,	0x01},//  FULL_QHDR_EN:  Full Pixel and QBC HDR mode enable, 0:  disable, 1:  enable
	{0x3F80,	0x01},
	{0x3F81,	0x7C},
	{0x3F8C,	0x03},
	{0x3F8D,	0x84},
	{0x3FF8,	0x00},
	{0x3FF9,	0x00},
	{0x3FFE,	0x00},
	{0x3FFF,	0xA2},

	//Integration Setting		

	{0x0202,	0x17},
	{0x0203,	0x83},

	//{0x0350, 0x01},
	
	{0x0224,	0x17},
	{0x0225,	0x83},
	{0x3FE0,	0x17},
	{0x3FE1,	0x83},

	//Gain Setting		

	{0x0204,	0x00},
	{0x0205,	0x70},
	
	{0x0216,	0x00},
	{0x0217,	0x70},
	{0x0218,	0x01},
	{0x0219,	0x00},
	{0x020E,	0x01},
	{0x020F,	0x00},
	{0x0210,	0x01},
	{0x0211,	0x00},
	{0x0212,	0x01},
	{0x0213,	0x00},
	{0x0214,	0x01},
	{0x0215,	0x00},
	{0x3FE2,	0x00},
	{0x3FE3,	0x70},
	{0x3FE4,	0x01},
	{0x3FE5,	0x00},

	//PDAF TYPE2 Setting		
	
	{0x3E20,	0x01},
	{0x3E37,	0x01},

	//AE-Hist Setting

	{0x323B,	0x01},

	//Flicker Setting
	{0x323C,	0x01},	
	
	{IMX586_TABLE_END, 0x00}
};

static imx586_reg imx586_4000x3000_Nor_H2V2_BIN_35FPS_mirror_flip[] = {
		//CSI_SIG_MODE 2: CSI-2 Signaling D-PHY 3: CSI-2 Signaling C-PHY
	//{0x0111,	0x02},

	//MIPI output setting		
	{0x0112,	0x0A},	// CSI data format for D-PHY: 0x0a: RAW10(top 10bit of internam data)
	{0x0113,	0x0A},
	
	{0x0114,	0x03},//csi lane mode : 1: 2 lane, 3: 4 lane
	
	//Line Length PCK Setting		, length of line: 7872

	{0x0342,	0x1E},	//LINE_LENGTH_PCK
	{0x0343,	0xC0},

	//Frame Length Lines Setting	, The length of frame	:3060

	{0x0340,	0x0B},	//FRM_LENGTH_LINES
	{0x0341,	0xF4},

	//ROI Setting		

	{0x0344,	0x00},
	{0x0345,	0x00},
	{0x0346,	0x00},
	{0x0347,	0x00},
	{0x0348,	0x1F},
	{0x0349,	0x3F},
	{0x034A,	0x17},
	{0x034B,	0x6F},

	//Mode Setting		

	{0x0220,	0x62},//{0x0220,	0x00},	//
	{0x0222,	0x01},
	{0x0900,	0x01},
	{0x0901,	0x22},
	{0x0902,	0x08},
	{0x3140,	0x00},
	{0x3246,	0x81},
	{0x3247,	0x81},
	{0x3F15,	0x00},

	//Digital Crop & Scaling		

	{0x0401,	0x00},
	{0x0404,	0x00},
	{0x0405,	0x10},
	{0x0408,	0x00},
	{0x0409,	0x00},
	{0x040A,	0x00},
	{0x040B,	0x00},
	{0x040C,	0x0F},
	{0x040D,	0xA0},
	{0x040E,	0x0B},
	{0x040F,	0xB8},

	//Output Size Setting		

	{0x034C,	0x0F},
	{0x034D,	0xA0},
	{0x034E,	0x0B},
	{0x034F,	0xB8},

	//Clock Setting		

	{0x0301,	0x05},		// IVT_SYCK_DIV
	{0x0303,	0x04},		// IVT_PXCK_DIV
	
	{0x0305,	0x04},		// IVT_PREPLLCK_DIV:  24M/4 = 6M

	// 24fps
	//{0x0306,	0x00},		//IVT_PLL_MPY      IVTCT: 249 * 6 = 1494M
	//{0x0307,	0xF9},		//IVT_PLL_MPY

	//30fps
	//{0x0306,	0x01},		//IVT_PLL_MPY      IVTCT: 309 * 6 = 1854M
	//{0x0307,	0x35},		//IVT_PLL_MPY

	//35fps
	//{0x0306,	0x01},		//IVT_PLL_MPY      		IVTCT: 358 * 6 = 2148M
	//{0x0307,	0x66},		//IVT_PLL_MPY

	{0x0306,	0x01},		//IVT_PLL_MPY      		IVTCT: 360 * 6 = 2160M
	{0x0307,	0x68},		//IVT_PLL_MPY
	
	{0x030B,	0x02},		// IOP_SYCK_DIV
	//{0x030B,	0x01},		// IOP_SYCK_DIV
	
	{0x030D,	0x06},		// IOP_PREPLLCK_DIV:   24m/6=4M

	//{0x030E,	0x01},		// IOP_PLL_MPY	 IOPCK: 400 * 4 = 1600M
	//{0x030F,	0x90},		// IOP_PLL_MPY

	//{0x030E,	0x01},		// IOP_PLL_MPY	 IOPCK: 432 * 4 = 1728M
	//{0x030F,	0xB0},		// IOP_PLL_MPY
	
	//{0x030E,	0x02},		// IOP_PLL_MPY	 IOPCK: 537 * 4 = 2148M
	//{0x030F,	0x19},		// IOP_PLL_MPY

	{0x030E,	0x02},		// IOP_PLL_MPY	 IOPCK: 625 * 4 = 2500M
	{0x030F,	0x71},		// IOP_PLL_MPY
	
	{0x0310,	0x01},		// PLL mode select,   0 : Single PLL mode 1 : Dual PLL mode   

	/*
	IVTCK = 24M * 1/4 * 249 = 1494M
	Predivider setting =   1/IVT_PREPLLCK_DIV				// 1/4
	Pll multiple setting = 	 IVT_PLL_MPY								// 249

	IOPCK = 24M * 1/6 * 400 = 1600M
	Predivider setting = 	1/IOP_PREPLLCK_DIV		1/6
	Pll multiple setting = IOP_PLL_MPY						400

	IVTPXCK clock frequency = IVTCK x IVTPXCK clock division ratio
				IVTPXCK clock division ratio = 1 / (IVT_SYCK_DIV * IVT_PXCK_DIV)

	IOPSYCK clock frequency = IOPCK x IOPSYCK clock division ratio
   				IOPSYCK clock division ratio = 1 / (IOP_SYCK_DIV)
	 
	IVTPXCK: 1494M * 1/(0x0301: 05 * 0x0303: 04) = 1494 / 20 = 74.7M
	IOPSYCK = 1600M *  1/(IOP_SYCK_DIV) = 1600M * 1/2 = 800M

	Frame Rate [frame/s] = Pixel_rate [pixels/s] / Total number of pixels [pixels/frame]
				Pixel rate [pixels/s] = IVTPXCK [MHz] * 8 (Total number of IVTPX channel)
				Total number of pixels [pixels/frame] = FRM_LENGTH_LINES [lines/frame] * LINE_LENGTH_PCK [pixels/line]

	Framerate = 597.6M /24088320  = 597600000 / 24088320 = 24.808
		Pixel_rate = IVTPXCK * 8 = 74.7M * 8 = 597.6M = 597600000
		pixel_clk = Pixel_rate
		Total number of pixels = 3060 * 7872 = 24088320	



	// 30fps
	IVTCK = 24M * 1/4 * 309 = 1854M
	Predivider setting =   1/IVT_PREPLLCK_DIV		// 1/4
	Pll multiple setting = 	 IVT_PLL_MPY						// 309

	IOPCK = 24M * 1/6 * 400 = 1600M
	Predivider setting = 	1/IOP_PREPLLCK_DIV		//1/6
	Pll multiple setting = IOP_PLL_MPY						//400

	IVTPXCK: 1854M * 1/(0x0301: 05 * 0x0303: 04) = 1854M / 20 = 92.7M
	IOPSYCK = 1600M *  1/(IOP_SYCK_DIV) = 1600M * 1/2 = 800M

	Framerate = 741.6M /24088320  = 741600000 / 24088320 = 30.7867
		Pixel_rate = IVTPXCK * 8 = 92.7M * 8 = 741.6M
		pixel_clk = Pixel_rate
		Total number of pixels = 3060 * 7872 = 24088320


	//35 fps
	IVTCK = 24M * 1/4 * 360 = 2160M
	Predivider setting =   1/IVT_PREPLLCK_DIV		// 1/4
	Pll multiple setting = 	 IVT_PLL_MPY						// 360

	IOPCK = 24M * 1/6 * 625 = 2500M
	Predivider setting = 	1/IOP_PREPLLCK_DIV		//1/6
	Pll multiple setting = IOP_PLL_MPY						//625
	
	IVTPXCK: 2160M * 1/(0x0301: 05 * 0x0303: 04) = 2160M / 20 = 108M
	IOPSYCK = 2500M *  1/(IOP_SYCK_DIV) = 2500M * 1/2 = 1250M

	Framerate = 864M /24088320  = 864000000 / 24088320 = 35.86800573888092
		Pixel_rate = IVTPXCK * 8 = 108M * 8 = 864M
		pixel_clk = Pixel_rate
		Total number of pixels = 3060 * 7872 = 24088320		


	// 71fps
	IVTCK = 24M * 1/4 * 360 = 2160M
	Predivider setting =   1/IVT_PREPLLCK_DIV		// 1/4
	Pll multiple setting = 	 IVT_PLL_MPY						// 360

	IOPCK = 24M * 1/6 * 625 = 2500M
	Predivider setting = 	1/IOP_PREPLLCK_DIV		//1/6
	Pll multiple setting = IOP_PLL_MPY						//625

	
	IVTPXCK: 2160M * 1/(0x0301: 05 * 0x0303: 02) = 2160M / 10 = 216M
	IOPSYCK = 2500M *  1/(IOP_SYCK_DIV) = 2500M * 1/1 = 2500M

	Framerate = 1728M /24088320  = 1728000000 / 24088320 = 71.73601147776184
		Pixel_rate = IVTPXCK * 8 = 216M * 8 = 1728M
		pixel_clk = Pixel_rate
		Total number of pixels = 3060 * 7872 = 24088320		
	*/
	
	//Other Setting

	{0x3620,	0x00},
	{0x3621,	0x00},
	{0x3C11,	0x04},
	{0x3C12,	0x03},
	{0x3C13,	0x2D},
	
	{0x3F0C,	0x01},	// BINNING_PDAF_EN:  Adjacent Pixel Binning, PDAF enable,  0:  disable, 1:  enable
	
	{0x3F14,	0x00},//  FULL_QHDR_EN:  Full Pixel and QBC HDR mode enable, 0:  disable, 1:  enable
	{0x3F80,	0x01},
	{0x3F81,	0x90},
	{0x3F8C,	0x00},
	{0x3F8D,	0x14},
	{0x3FF8,	0x01},
	{0x3FF9,	0x2A},
	{0x3FFE,	0x00},
	{0x3FFF,	0x6C},

	//Integration Setting		

	{0x0202,	0x0B},
	{0x0203,	0xC4},

	//{0x0350, 0x01},
	
	{0x0224,	0x01},
	{0x0225,	0xF4},
	{0x3FE0,	0x01},
	{0x3FE1,	0xF4},

	//Gain Setting		

	{0x0204,	0x00},
	{0x0205,	0x70},
	
	{0x0216,	0x00},
	{0x0217,	0x70},
	{0x0218,	0x01},
	{0x0219,	0x00},
	{0x020E,	0x01},
	{0x020F,	0x00},
	{0x0210,	0x01},
	{0x0211,	0x00},
	{0x0212,	0x01},
	{0x0213,	0x00},
	{0x0214,	0x01},
	{0x0215,	0x00},
	{0x3FE2,	0x00},
	{0x3FE3,	0x70},
	{0x3FE4,	0x01},
	{0x3FE5,	0x00},

	//PDAF TYPE1 Setting		
	/*
	{0x3E20,	0x01},
	{0x3E37,	0x01},
	*/
	
	{IMX586_TABLE_END, 0x00}
};

enum {	
	IMX586_MODE_4000X3000_10BIT_HDR_30FPS,	
	IMX586_MODE_4000X3000_10BIT_NOR_H2V2_BIN_35FPS,	
	IMX586_MODE_4000X3000_10BIT_HDR_30FPS_MIRROR,	
	IMX586_MODE_4000X3000_10BIT_NOR_H2V2_BIN_35FPS_MIRROR,
	IMX586_MODE_4000X3000_10BIT_HDR_30FPS_FLIP,
	IMX586_MODE_4000X3000_10BIT_NOR_H2V2_BIN_35FPS_FLIP,
	IMX586_MODE_4000X3000_10BIT_HDR_30FPS_MIRROR_FLIP,
	IMX586_MODE_4000X3000_10BIT_NOR_H2V2_BIN_35FPS_MIRROR_FLIP,
	IMX586_MODE_START_STREAM,
	IMX586_MODE_STOP_STREAM,
	IMX586_MODE_TEST_PATTERN,
	IMX586_MODE_STAND_BY
};

static imx586_reg *mode_table[] = {
	[IMX586_MODE_4000X3000_10BIT_HDR_30FPS] = imx586_4000x3000_HDR_30FPS,
	[IMX586_MODE_4000X3000_10BIT_NOR_H2V2_BIN_35FPS] = imx586_4000x3000_Nor_H2V2_BIN_35FPS,
	[IMX586_MODE_4000X3000_10BIT_HDR_30FPS_MIRROR] = imx586_4000x3000_HDR_30FPS_mirror,
	[IMX586_MODE_4000X3000_10BIT_NOR_H2V2_BIN_35FPS_MIRROR] = imx586_4000x3000_Nor_H2V2_BIN_35FPS_mirror,
	[IMX586_MODE_4000X3000_10BIT_HDR_30FPS_FLIP] = imx586_4000x3000_HDR_30FPS_flip,
	[IMX586_MODE_4000X3000_10BIT_NOR_H2V2_BIN_35FPS_FLIP] = imx586_4000x3000_Nor_H2V2_BIN_35FPS_flip,
	[IMX586_MODE_4000X3000_10BIT_HDR_30FPS_MIRROR_FLIP] = imx586_4000x3000_HDR_30FPS_mirror_flip,
	[IMX586_MODE_4000X3000_10BIT_NOR_H2V2_BIN_35FPS_MIRROR_FLIP] = imx586_4000x3000_Nor_H2V2_BIN_35FPS_mirror_flip,
	[IMX586_MODE_START_STREAM] = imx586_start,
	[IMX586_MODE_STOP_STREAM] = imx586_stop,
	[IMX586_MODE_TEST_PATTERN] = tp_colorbars,
	[IMX586_MODE_STAND_BY] = imx586_Stand_by,
};

static imx586_reg *mode_table_hdr[] = {
	[IMX586_MODE_4000X3000_10BIT_HDR_30FPS] = imx586_4000x3000_HDR_30FPS_HDRSETTING,
};

static imx586_reg *mode_table_hdr_exposure[] = {
	[IMX586_MODE_4000X3000_10BIT_HDR_30FPS] = imx586_4000x3000_HDR_30FPS_HDR_EXPOSURE,
};

static const int imx586_6fps[] = {
	6,
};

static const int imx586_9fps[] = {
	9,
};

static const int imx586_35fps[] = {
	35,
};

static const int imx586_30fps[] = {
	30,
};

static const int imx586_24fps[] = {
	24,
};

static const int imx586_27fps[] = {
	27,
};

static const struct camera_common_frmfmt imx586_frmfmt[] = {
	{{4000, 3000}, imx586_30fps, 1, 1,
			IMX586_MODE_4000X3000_10BIT_HDR_30FPS},	
	{{4000, 3000}, imx586_35fps, 1, 0,
			IMX586_MODE_4000X3000_10BIT_NOR_H2V2_BIN_35FPS},
	{{4000, 3000}, imx586_30fps, 1, 1,
			IMX586_MODE_4000X3000_10BIT_HDR_30FPS_MIRROR},
	{{4000, 3000}, imx586_35fps, 1, 0,
			IMX586_MODE_4000X3000_10BIT_NOR_H2V2_BIN_35FPS_MIRROR},
	{{4000, 3000}, imx586_30fps, 1, 1,
			IMX586_MODE_4000X3000_10BIT_HDR_30FPS_FLIP},			
	{{4000, 3000}, imx586_35fps, 1, 0,
			IMX586_MODE_4000X3000_10BIT_NOR_H2V2_BIN_35FPS_FLIP},
	{{4000, 3000}, imx586_30fps, 1, 1,
			IMX586_MODE_4000X3000_10BIT_HDR_30FPS_MIRROR_FLIP},
	{{4000, 3000}, imx586_35fps, 1, 0,
			IMX586_MODE_4000X3000_10BIT_NOR_H2V2_BIN_35FPS_MIRROR_FLIP},
};

#endif /* __IMX185_I2C_TABLES__ */
