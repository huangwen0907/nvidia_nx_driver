/*
 * imx283_mode_tbls.h - imx283 sensor mode tables
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef __IMX283_I2C_TABLES__
#define __IMX283_I2C_TABLES__

#include <media/camera_common.h>
#include <linux/miscdevice.h>

#define IMX283_TABLE_WAIT_MS	0
#define IMX283_TABLE_END	1
#define IMX283_MAX_RETRIES	3
#define IMX283_WAIT_MS_STOP	1
#define IMX283_WAIT_MS_START	30
#define IMX283_WAIT_MS_STREAM	210
#define IMX283_GAIN_TABLE_SIZE 255

#define IMX283_WAIT_MS_1ST		2
#define IMX283_WAIT_MS_2ND		20



/* #define INIT_ET_INSETTING 1 */

#define imx283_reg struct reg_8

static imx283_reg imx283_start[] = {
	// 1st
	{0x3000, 0x0a},
		
	//24MHz PLRD1=0x02 PLRD2=0x00f0 PLRD3=0x02 PLRD4=0xc0     
	{0x36c1, 0x02},	//PLRD1=0x02    
	{0x36c2, 0xf0},	//PLRD2=0x00f0
	{0x36c3, 0x00},	//PLRD2=0x00f0
	
	{0x36f7, 0x02},	//PLRD3=0x02    
	{0x36f8, 0xc0},	//PLRD4=0xc0 

	{0x3003, 0x77},	//PLSTMG084=0x77 
	{0x36aa, 0x00},//PLSTMG02=0x00

	{0x320B, 0x00},	//STBPL=0x00 
	
	{IMX283_TABLE_WAIT_MS, IMX283_WAIT_MS_1ST},

	// 2nd
	{0x3000, 0x00},
	{IMX283_TABLE_WAIT_MS, IMX283_WAIT_MS_2ND},

	// 3rd
	{0x3001, 0x01},//CQPSQRST=1   
	{0x3105, 0x00},//XMSTA=0    
	{0x3107, 0x02},//SYNCDRV=2
	
	{ IMX283_TABLE_END, 0x00 }
};

static imx283_reg imx283_stop[] = {
	{IMX283_TABLE_WAIT_MS, IMX283_WAIT_MS_STOP},
	{0x3000, 0x01 },
	{IMX283_TABLE_END, 0x00 }
};

static imx283_reg tp_colorbars[] = {
	{0x300A, 0x00},/*BLC for PG*/
	{0x300E, 0x00},
	{0x3089, 0x00},
	{0x308C, 0x13},
	/*
	* bit 0: PG mode enable
	* bit 1: Back Ground Transient:
	* bit [4-7]: PG mode setting, Set at 0h to Fh, suggest 1 or 5
	* raw12 max output FFEh
	*/
	{IMX283_TABLE_WAIT_MS, IMX283_WAIT_MS_STOP},
	{IMX283_TABLE_END, 0x00}
};

// 3:2 MODE 1
static  imx283_reg imx283_5472x3648_10bit_full_25fps[] = {
    {0x3004, 0x04},//MDSEL1    
    {0x3005, 0x01},//MDSEL2    
    {0x3006, 0x00},//MDSEL3    
    {0x3007, 0x00},//MDSEL4

    {0x3009, 0x00},//SVR[7:0]    
    {0x300a, 0x00},//SVR[15:8]

    {0x300b, 0x30},//bit[0]: MDVREV: 0h: vertical direction normal /1h: inverted
								//bit[4]: HTRIMMING_EN: 1h, bit[7:5]: 1h

	{0x300f, 0x00},//VWINPOS[7:0]    
	{0x3010, 0x00},//VWINPOS[15:8]

	{0x3011, 0x00},//VWIDCUT[7:0]    
	{0x3012, 0x00},//VWIDCUT[15:8]

	{0x3013, 0x00},//MDSEL7[7:0]    
	{0x3014, 0x00},//MDSEL7[15:8]

	{0x302f, 0x6e},//Y_OUT_SIZE[7:0]    
	{0x3030, 0x0e},//Y_OUT_SIZE[15:8]

	{0x3031, 0x7e},//WRITE_VSIZE[7:0]    
	{0x3032, 0x0e},//WRITE_VSIZE[15:8]

	{0x3033, 0x10},//OB_SIZE_V

	// HMAX VMAX SHR
	{0x3036, 0xe9},	//HMAX 745    
	{0x3037, 0x02},	//HMAX

	{0x3038, 0xd1},	//VMAX 3793 
	{0x3039, 0x0e},	//VMAX    
	{0x303a, 0x00},	//VMAX    

		//shr: 10 to {(SVR value + 1) x VMAX value - 4} 
		//		10 -- 3789
	{0x303b, 0x00},	//SHR
	{0x303c, 0x01},	//SHR

	{0x3058, 0x78},//HTRIMMING_START    
	{0x3059, 0x00},//HTRIMMING_START

	{0x305a, 0xf0},//HTRIMMING_END   
	{0x305b, 0x15},//HTRIMMING_END

	{IMX283_TABLE_END, 0x00}	
};

// 3:2 MODE 1A
static  imx283_reg imx283_5472x3078_10bit_full_30fps[] = {
    {0x3004, 0x04},//MDSEL1    
    {0x3005, 0x01},//MDSEL2    
    {0x3006, 0x20},//MDSEL3    
    {0x3007, 0x50},//MDSEL4

    {0x3009, 0x00},//SVR[7:0]    
    {0x300a, 0x00},//SVR[15:8]

    {0x300b, 0x30},//bit[0]: MDVREV: 0h: vertical direction normal /1h: inverted
								//bit[4]: HTRIMMING_EN: 1h, bit[7:5]: 1h

	{0x300f, 0x92},//VWINPOS[7:0]    
	{0x3010, 0x00},//VWINPOS[15:8]

	{0x3011, 0x23},//VWIDCUT[7:0]    
	{0x3012, 0x01},//VWIDCUT[15:8]

	{0x3013, 0x00},//MDSEL7[7:0]    
	{0x3014, 0x00},//MDSEL7[15:8]

	{0x302f, 0x28},//Y_OUT_SIZE[7:0]    
	{0x3030, 0x0c},//Y_OUT_SIZE[15:8]

	{0x3031, 0x38},//WRITE_VSIZE[7:0]    
	{0x3032, 0x0c},//WRITE_VSIZE[15:8]

	{0x3033, 0x10},//OB_SIZE_V

	// HMAX VMAX SHR
	{0x3036, 0xe9},	//HMAX 745    
	{0x3037, 0x02},	//HMAX

	{0x3038, 0x83},	//VMAX 3203 
	{0x3039, 0x0c},	//VMAX    
	{0x303a, 0x00},	//VMAX    

		//shr: 10 to {(SVR value + 1) x VMAX value - 4} 
		//		10 -- 3789
	{0x303b, 0x00},	//SHR
	{0x303c, 0x01},	//SHR

	{0x3058, 0x78},//HTRIMMING_START    
	{0x3059, 0x00},//HTRIMMING_START

	{0x305a, 0xf0},//HTRIMMING_END   
	{0x305b, 0x15},//HTRIMMING_END
	
	{IMX283_TABLE_END, 0x00}	
};

// 3:2 MODE 1S
static  imx283_reg imx283_3000x3000_10bit_full_42fps[] = {
    {0x3004, 0x04},//MDSEL1    
    {0x3005, 0x41},//MDSEL2    
    {0x3006, 0x20},//MDSEL3    
    {0x3007, 0x50},//MDSEL4

    {0x3009, 0x00},//SVR[7:0]    
    {0x300a, 0x00},//SVR[15:8]

    {0x300b, 0x30},//bit[0]: MDVREV: 0h: vertical direction normal /1h: inverted
								//bit[4]: HTRIMMING_EN: 1h, bit[7:5]: 1h

	{0x300f, 0xa2},//VWINPOS[7:0]    
	{0x3010, 0x00},//VWINPOS[15:8]

	{0x3011, 0x44},//VWIDCUT[7:0]    
	{0x3012, 0x01},//VWIDCUT[15:8]

	{0x3013, 0x00},//MDSEL7[7:0]    
	{0x3014, 0x10},//MDSEL7[15:8]

	{0x302f, 0xe6},//Y_OUT_SIZE[7:0]    
	{0x3030, 0x0b},//Y_OUT_SIZE[15:8]

	{0x3031, 0xf6},//WRITE_VSIZE[7:0]    
	{0x3032, 0x0b},//WRITE_VSIZE[15:8]

	{0x3033, 0x10},//OB_SIZE_V

	// HMAX VMAX SHR
	{0x3036, 0x20},	//HMAX 544    
	{0x3037, 0x02},	//HMAX

	{0x3038, 0x09},	//VMAX 3081 
	{0x3039, 0x0c},	//VMAX    
	{0x303a, 0x00},	//VMAX    

		//shr: 10 to {(SVR value + 1) x VMAX value - 4} 
		//		10 -- 3789
	{0x303b, 0x00},	//SHR
	{0x303c, 0x01},	//SHR

	{0x3058, 0x90},//HTRIMMING_START    
	{0x3059, 0x02},//HTRIMMING_START

	{0x305a, 0x68},//HTRIMMING_END   
	{0x305b, 0x0e},//HTRIMMING_END

	{0x30f6, 0x98},//MDSEL18  
	{0x30f7, 0x10},//MDSEL18		
	
	{IMX283_TABLE_END, 0x00}	
};

// 3:2 mode 6
static  imx283_reg imx283_2376x1538_10bit_full_60fps[] = {
    {0x3004, 0x18},//MDSEL1    
    {0x3005, 0x21},//MDSEL2    
    {0x3006, 0x00},//MDSEL3    
    {0x3007, 0x09},//MDSEL4

    {0x3009, 0x00},//SVR[7:0]    
    {0x300a, 0x00},//SVR[15:8]

    {0x300b, 0x30},//bit[0]: MDVREV: 0h: vertical direction normal /1h: inverted
								//bit[4]: HTRIMMING_EN: 1h, bit[7:5]: 1h

	{0x300f, 0x00},//VWINPOS[7:0]    
	{0x3010, 0x00},//VWINPOS[15:8]

	{0x3011, 0x00},//VWIDCUT[7:0]    
	{0x3012, 0x00},//VWIDCUT[15:8]

	{0x3013, 0x00},//MDSEL7[7:0]    
	{0x3014, 0x00},//MDSEL7[15:8]

	{0x302f, 0x14},//Y_OUT_SIZE[7:0]    
	{0x3030, 0x06},//Y_OUT_SIZE[15:8]

	{0x3031, 0x18},//WRITE_VSIZE[7:0]    
	{0x3032, 0x06},//WRITE_VSIZE[15:8]

	{0x3033, 0x04},//OB_SIZE_V

	// HMAX VMAX SHR
	{0x3036, 0x6c},	//HMAX 364    
	{0x3037, 0x01},	//HMAX

	{0x3038, 0xe0},	//VMAX 3296 
	{0x3039, 0x0c},	//VMAX    
	{0x303a, 0x00},	//VMAX    

		//shr: 10 to {(SVR value + 1) x VMAX value - 4} 
		//		10 -- 3789
	{0x303b, 0x00},	//SHR
	{0x303c, 0x01},	//SHR

	{0x3058, 0x78},//HTRIMMING_START    
	{0x3059, 0x00},//HTRIMMING_START

	{0x305a, 0xf0},//HTRIMMING_END   
	{0x305b, 0x15},//HTRIMMING_END
	
	{IMX283_TABLE_END, 0x00}	
};

// 16:9 mode1
static  imx283_reg imx283_3840x2160_10bit_crop_60fps[] = {
    {0x3004, 0x30},//MDSEL1    
    {0x3005, 0x41},//MDSEL2    
    {0x3006, 0x00},//MDSEL3    
    {0x3007, 0x00},//MDSEL4

    {0x3009, 0x00},//SVR[7:0]    
    {0x300a, 0x00},//SVR[15:8]

    {0x300b, 0x30},//bit[0]: MDVREV: 0h: vertical direction normal /1h: inverted
								//bit[4]: HTRIMMING_EN: 1h, bit[7:5]: 1h

	{0x300f, 0x00},//VWINPOS[7:0]    
	{0x3010, 0x00},//VWINPOS[15:8]

	{0x3011, 0x00},//VWIDCUT[7:0]    
	{0x3012, 0x00},//VWIDCUT[15:8]

	{0x3013, 0x00},//MDSEL7[7:0]    
	{0x3014, 0x00},//MDSEL7[15:8]

	{0x302f, 0x7e},//Y_OUT_SIZE[7:0]    
	{0x3030, 0x08},//Y_OUT_SIZE[15:8]

	{0x3031, 0x86},//WRITE_VSIZE[7:0]    
	{0x3032, 0x08},//WRITE_VSIZE[15:8]

	{0x3033, 0x08},//OB_SIZE_V

	// HMAX VMAX SHR
	{0x3036, 0x20},	//HMAX 544    
	{0x3037, 0x02},	//HMAX

	{0x3038, 0x98},	//VMAX 2200 
	{0x3039, 0x08},	//VMAX    
	{0x303a, 0x00},	//VMAX    

		//shr: 10 to {(SVR value + 1) x VMAX value - 4} 
		//		10 -- 3789
	{0x303b, 0x00},	//SHR
	{0x303c, 0x01},	//SHR

	{0x3058, 0xec},//HTRIMMING_START    
	{0x3059, 0x00},//HTRIMMING_START

	{0x305a, 0x0c},//HTRIMMING_END   
	{0x305b, 0x10},//HTRIMMING_END
	
	{IMX283_TABLE_END, 0x00}	
};

enum {
	IMX283_MODE_5472X3648_10BIT_FULL_25FPS,
	IMX283_MODE_5472X3078_10BIT_FULL_30FPS,
	IMX283_MODE_3000X3000_10BIT_FULL_42FPS,
	IMX283_MODE_2376X1538_10BIT_FULL_60FPS,
	IMX283_MODE_3840X2160_10BIT_CROP_60FPS,
	IMX283_MODE_START_STREAM,
	IMX283_MODE_STOP_STREAM,
	IMX283_MODE_TEST_PATTERN,
};

static imx283_reg *mode_table[] = {
	[IMX283_MODE_5472X3648_10BIT_FULL_25FPS] = imx283_5472x3648_10bit_full_25fps,
	[IMX283_MODE_5472X3078_10BIT_FULL_30FPS] = imx283_5472x3078_10bit_full_30fps,
	[IMX283_MODE_3000X3000_10BIT_FULL_42FPS] = imx283_3000x3000_10bit_full_42fps,
	[IMX283_MODE_2376X1538_10BIT_FULL_60FPS] = imx283_2376x1538_10bit_full_60fps,
	[IMX283_MODE_3840X2160_10BIT_CROP_60FPS] = imx283_3840x2160_10bit_crop_60fps,
	[IMX283_MODE_START_STREAM] = imx283_start,
	[IMX283_MODE_STOP_STREAM] = imx283_stop,
	[IMX283_MODE_TEST_PATTERN] = tp_colorbars,
};

static const int imx283_25fps[] = {
	25,
};

static const int imx283_30fps[] = {
	30,
};

static const int imx283_42fps[] = {
	42,
};

static const int imx283_60fps[] = {
	60,
};

/*
 * WARNING: frmfmt ordering need to match mode definition in
 * device tree!
 */
static const struct camera_common_frmfmt imx283_frmfmt[] = {
	{{5472, 3648}, imx283_25fps, 1, 0, IMX283_MODE_5472X3648_10BIT_FULL_25FPS},
	{{5472, 3078}, imx283_30fps, 1, 0, IMX283_MODE_5472X3078_10BIT_FULL_30FPS},
	{{3000, 3000}, imx283_42fps, 1, 0, IMX283_MODE_3000X3000_10BIT_FULL_42FPS},
	{{2376, 1538}, imx283_60fps, 1, 0, IMX283_MODE_2376X1538_10BIT_FULL_60FPS},
	{{3840, 2160}, imx283_60fps, 1, 0, IMX283_MODE_3840X2160_10BIT_CROP_60FPS},
};
#endif /* __IMX283_I2C_TABLES__ */
