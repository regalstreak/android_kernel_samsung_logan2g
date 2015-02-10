/******************************************************************************
 ** Copyright (c)
 ** File Name:		sensor_SR300PC20.c 										  *
 ** Author: 		dhee79.lee@samsung.com											  *
 ** DATE:			2012.05.03												  *
 ** Description:   This file contains driver for sensor SR300PC20.
 **
 ******************************************************************************

 ******************************************************************************
 ** 					   Edit History 									  *
 ** ------------------------------------------------------------------------- *
 ** DATE		   NAME 			DESCRIPTION 							  *
 **
 ******************************************************************************/

/**---------------------------------------------------------------------------*
 ** 						Dependencies									  *
 **---------------------------------------------------------------------------*/
#include "common/sensor.h"
#include "common/sensor_drv.h"
#include "common/jpeg_exif_header_k.h"
//#include <mach/ldo.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/err.h>


#include "sensor_sr300pc20_regs.h"

struct class *camera_class_sr300pc20;
/**---------------------------------------------------------------------------*
 ** 						Compiler Flag									  *
 **---------------------------------------------------------------------------*/
#ifdef	 __cplusplus
	extern	 "C"
	{
#endif
/**---------------------------------------------------------------------------*
 ** 					Extern Function Declaration 						  *
 **---------------------------------------------------------------------------*/
//extern uint32_t OS_TickDelay(uint32_t ticks);

/**---------------------------------------------------------------------------*
 ** 						Const variables 								  *
 **---------------------------------------------------------------------------*/

/**---------------------------------------------------------------------------*
 ** 						   Macro Define
 **---------------------------------------------------------------------------*/
#define SR300PC20_I2C_ADDR_W		0x20
#define SR300PC20_I2C_ADDR_R		0x20

/*camera sensor Tuning*/
//#define CONFIG_LOAD_FILE

#define SR300PC20_SIGNAL_PCLK_PHASE               0x00
#ifdef CONFIG_LOAD_FILE
#define CAM_DEBUG   SENSOR_PRINT
#else
#define CAM_DEBUG  printk //(...) // bst chris.shen add for debug sensor driver code.
#endif

#undef SENSOR_PRINT
#define SENSOR_PRINT printk



// Address can change by pin  0(AF mode) -> 78,79  1(FF mode)-> 5a,5b
// In Amazing_TD,  Set 1 by IO level

/**---------------------------------------------------------------------------*
 ** 					Local Function Prototypes							  *
 **---------------------------------------------------------------------------*/
LOCAL uint32_t SR300PC20_set_ae_enable(uint32_t enable);
LOCAL uint32_t SR300PC20_set_hmirror_enable(uint32_t enable);
LOCAL uint32_t SR300PC20_set_vmirror_enable(uint32_t enable);
LOCAL uint32_t SR300PC20_set_scene_mode(uint32_t scene_mode);
LOCAL uint32_t SR300PC20_set_preview_mode(uint32_t preview_mode);
LOCAL uint32_t SR300PC20_set_capture_mode(uint32_t capture_mode);
LOCAL uint32_t SR300PC20_Power_Ctrl(uint32_t param);
LOCAL uint32_t SR300PC20_Identify(uint32_t param);
LOCAL uint32_t SR300PC20_BeforeSnapshot(uint32_t param);

LOCAL uint32_t SR300PC20_set_brightness(uint32_t level);
LOCAL uint32_t SR300PC20_set_contrast(uint32_t level);
LOCAL uint32_t SR300PC20_set_image_effect(uint32_t effect_type);
LOCAL uint32_t SR300PC20_chang_image_format(uint32_t param);
LOCAL uint32_t SR300PC20_check_image_format_support(uint32_t param);
LOCAL uint32_t SR300PC20_after_snapshot(uint32_t param);

LOCAL uint32_t SR300PC20_set_awb(uint32_t mode);
LOCAL uint32_t SR300PC20_GetExifInfo(uint32_t param);
LOCAL uint32_t SR300PC20_set_focus(uint32_t effect_type);
LOCAL uint32_t SR300PC20_InitExifInfo(void);

LOCAL uint32_t SR300PC20_set_quality(uint32_t quality_type);
LOCAL uint32_t SR300PC20_set_DTP(uint32_t dtp_mode);
LOCAL uint32_t SR300PC20_set_Metering(uint32_t metering_mode);
LOCAL uint32_t SR300PC20_set_FPS(uint32_t fps_mode);
LOCAL uint32_t SR300PC20_I2C_write(SENSOR_REG_T* sensor_reg_ptr, char* reg_name );

LOCAL uint32_t SR300PC20_LightCheck(void);
LOCAL uint32_t SR300PC20_set_lightcapture(uint32_t level);
LOCAL uint32_t SR300PC20_Get_Exif_Exporsure(uint32_t level);
LOCAL uint32_t SR300PC20_Get_Exif_ISO(uint32_t level);
LOCAL uint32_t SR300PC20_Get_Exif_Flash(uint32_t level);
LOCAL uint32_t SR300PC20_GetExifInfo(uint32_t level);
LOCAL uint32_t SR300PC20_InitExt(uint32_t param);
LOCAL uint32_t SR300PC20_init_by_burst_write(uint32_t param);
LOCAL uint32_t SR300PC20_StreamOn(uint32_t param);
LOCAL uint32_t SR300PC20_StreamOff(uint32_t param);



/**---------------------------------------------------------------------------*
 ** 						Local Variables 								 *
 **---------------------------------------------------------------------------*/

LOCAL uint32_t s_image_effect = 0;

LOCAL uint32_t sr_scenemode = 0;
LOCAL uint32_t s_caputuresize = 0;
LOCAL uint32_t s_firstInit = 0;

/*++ FOR CHECKING THE LIGHT BEFORE CAPTURE :dhee79.lee@samsung.com++*/
typedef enum
{
	HIGH_LIGHT_CAPTURE,
	NORMAL_LIGHT_CAPTURE,
	LOW_LIGHT_CAPTURE
}SensorLightCaptureType;
/*-- FOR CHECKING THE LIGHT BEFORE CAPTURE :dhee79.lee@samsung.com--*/


/*lint -save -e533 */



LOCAL SENSOR_REG_TAB_INFO_T s_SR300PC20_resolution_Tab_YUV[]=
{
	// COMMON INIT
	{ADDR_AND_LEN_OF_ARRAY(reg_main_init),640, 480, 24, SENSOR_IMAGE_FORMAT_YUV422},

	{PNULL, 0,   640,  480,   24, SENSOR_IMAGE_FORMAT_YUV422},
	{PNULL, 0,   720,  480,   24, SENSOR_IMAGE_FORMAT_YUV422},//Songww 20130320 add for 720*480 record
	{PNULL,  0, 1280,  960,   24, SENSOR_IMAGE_FORMAT_YUV422},
	{PNULL,  0, 1600,  1200,   24, SENSOR_IMAGE_FORMAT_YUV422},
	{PNULL,  0, 2048,  1536,   24, SENSOR_IMAGE_FORMAT_YUV422},

	// YUV422 PREVIEW 1
	/*
	{ADDR_AND_LEN_OF_ARRAY(sr300pc20_640x480_preview_set),   640,  480,   24, SENSOR_IMAGE_FORMAT_YUV422},
	{ADDR_AND_LEN_OF_ARRAY(sr300pc20_1280x960_snapshot),   1280,  960,   24, SENSOR_IMAGE_FORMAT_YUV422},
	{ADDR_AND_LEN_OF_ARRAY(sr300pc20_1600x1200_snapshot),   1600,  1200,   24, SENSOR_IMAGE_FORMAT_YUV422},
	{ADDR_AND_LEN_OF_ARRAY(sr300pc20_snapshot),   2048,  1536,   24, SENSOR_IMAGE_FORMAT_YUV422},
	*/
	// YUV422 PREVIEW 2
	{PNULL, 0, 0, 0, 0, 0},
	{PNULL, 0, 0, 0, 0, 0},
	{PNULL, 0, 0, 0, 0, 0},
	{PNULL, 0, 0, 0, 0, 0}

};

LOCAL SENSOR_IOCTL_FUNC_TAB_T s_SR300PC20_ioctl_func_tab =
{
    // Internal
    PNULL, /*0*/
    SR300PC20_Power_Ctrl, /*1*/
    PNULL,/*2*/
    SR300PC20_Identify,/*3*/
    PNULL,/*4*/			// write register
    PNULL,/*5*/// read  register
    SR300PC20_init_by_burst_write,/*6*/
    PNULL,/*7*/

    // External
    PNULL,/*8*///SR300PC20_set_ae_enable,
    PNULL,/*9*///SR300PC20_set_hmirror_enable,
    PNULL,/*10*///SR300PC20_set_vmirror_enable,
    SR300PC20_set_brightness,/*11*/
    SR300PC20_set_contrast,/*12*/
    PNULL,/*13*///SR300PC20_set_sharpness,
    PNULL,/*14*///SR300PC20_set_saturation,
    SR300PC20_set_scene_mode ,/*15*///SR300PC20_set_preview_mode,
    SR300PC20_set_image_effect,/*16*/
    SR300PC20_BeforeSnapshot,/*17*/
    SR300PC20_after_snapshot,/*18*/
    PNULL,/*19*/
    PNULL,/*20*/
    PNULL,/*21*/
    PNULL,/*22*/
    PNULL,/*23*/
    PNULL,/*24*/
    PNULL,/*25*/
    PNULL,/*26*/
    PNULL,/*27*/
    SR300PC20_set_awb,/*28*/
    PNULL,/*29*/
    PNULL,/*30*///iso
    PNULL,/*31*///exposure
    PNULL,/*32*/
    PNULL,/*33*/
    PNULL,/*34*/ //wxz:???
    PNULL,/*35*/// set FPS
    PNULL,/*36*///focus,
    PNULL,/*37*///anti flicker
    PNULL,/*38*/// video mode
    PNULL,/*39*///pick j peg
    SR300PC20_set_Metering,/*40*/// set mertering
//    PNULL, /*41*///get_status
//    SR300PC20_StreamOn, /*42*///stream_on
//    SR300PC20_StreamOff, /*43*/ // stream_off
    SR300PC20_set_FPS,
    SR300PC20_Get_Exif_ISO,
    SR300PC20_Get_Exif_Exporsure,
    SR300PC20_Get_Exif_Flash,
    SR300PC20_set_DTP
};


/**---------------------------------------------------------------------------*
 ** 						Global Variables								  *
 **---------------------------------------------------------------------------*/
SENSOR_INFO_T g_sr300pc20_yuv_info =
{
	SR300PC20_I2C_ADDR_W,				// salve i2c write address
	SR300PC20_I2C_ADDR_R, 				// salve i2c read address

	SENSOR_I2C_VAL_8BIT|\
	SENSOR_I2C_REG_8BIT|\
	SENSOR_I2C_FREQ_400,
						//SENSOR_I2C_VAL_8BIT|SENSOR_I2C_REG_8BIT,
									// bit0: 0: i2c register value is 8 bit, 1: i2c register value is 16 bit
									// bit1: 0: i2c register addr  is 8 bit, 1: i2c register addr  is 16 bit
									// other bit: reseved

	SENSOR_HW_SIGNAL_PCLK_P|\
	SENSOR_HW_SIGNAL_VSYNC_N|\
	SENSOR_HW_SIGNAL_HSYNC_P|\
	SR300PC20_SIGNAL_PCLK_PHASE,		// bit0: 0:negative; 1:positive -> polarily of pixel clock
									// bit2: 0:negative; 1:positive -> polarily of horizontal synchronization signal
									// bit4: 0:negative; 1:positive -> polarily of vertical synchronization signal
									// other bit: reseved
									// bit5~7: ccir delay sel


	// preview mode
	SENSOR_ENVIROMENT_NORMAL|\
	SENSOR_ENVIROMENT_NIGHT|\
	SENSOR_ENVIROMENT_SUNNY,

	// image effect
	0,

	// while balance mode
	 SENSOR_WB_MODE_AUTO|\
        SENSOR_WB_MODE_INCANDESCENCE|\
        SENSOR_WB_MODE_U30|\
        SENSOR_WB_MODE_CWF|\
        SENSOR_WB_MODE_FLUORESCENT|\
        SENSOR_WB_MODE_SUN|\
        SENSOR_WB_MODE_CLOUD,

	0x0005,								// bit[0:7]: count of step in brightness, contrast, sharpness, saturation
									// bit[8:31] reseved

	SENSOR_LOW_PULSE_RESET,		// reset pulse level
	20,								// reset pulse width(ms)

	SENSOR_LOW_LEVEL_PWDN,		// 1: high level valid; 0: low level valid

	1,								// count of identify code
	{0x04, 0xA4},						// supply two code to identify sensor.
									// for Example: index = 0-> Device id, index = 1 -> version id
									// supply two code to identify sensor.
	                       					// for Example: index = 0-> Device id, index = 1 -> version id

	SENSOR_AVDD_2800MV,			// voltage of avdd

	2048,							// max width of source image
	1536,							// max height of source image
	"SR300PC20",						// name of sensor

	SENSOR_IMAGE_FORMAT_YUV422,		// define in SENSOR_IMAGE_FORMAT_E enum,SENSOR_IMAGE_FORMAT_MAX
									// if set to SENSOR_IMAGE_FORMAT_MAX here, image format depent on SENSOR_REG_TAB_INFO_T

	SENSOR_IMAGE_PATTERN_YUV422_VYUY,	// pattern of input image form sensor;

	&s_SR300PC20_resolution_Tab_YUV,	// point to resolution table information structure
	&s_SR300PC20_ioctl_func_tab,		// point to ioctl function table

	PNULL,							// information and table about Rawrgb sensor
	PNULL,				// extend information about sensor
	SENSOR_AVDD_1800MV,                     // iovdd
	SENSOR_AVDD_1200MV,                      // dvdd
	0,                     // skip frame num before preview
	0,                     // skip frame num before capture
	0,                     // deci frame num during preview;
    0,                     // deci frame num during video preview;

	0,                     // threshold enable
    0,                     // threshold mode
    0,                     // threshold start postion
    0,                     // threshold end postion
    0
//	-1,	// i2c_dev_handler
//	{SENSOR_INTERFACE_TYPE_CSI2, 1, 8, 1} // SENSOR_INF_T
};

/**---------------------------------------------------------------------------*
 ** 							Function  Definitions
 **---------------------------------------------------------------------------*/
/******************************************************************************/
// Description:
// Global resource dependence:
// Author:
// Note:
//
/******************************************************************************/
LOCAL uint32_t SR300PC20_set_ae_enable(uint32_t enable)
{

	return 0;
}

/******************************************************************************/
// Description:
// Global resource dependence:
// Author:
// Note:
//
/******************************************************************************/
LOCAL uint32_t SR300PC20_set_hmirror_enable(uint32_t enable)
{
	return 0;
}

/******************************************************************************/
// Description:
// Global resource dependence:
// Author:
// Note:
//
/******************************************************************************/
LOCAL uint32_t SR300PC20_set_vmirror_enable(uint32_t enable)
{
	return 0;
}


/*
LOCAL uint32_t SR300PC20_InitExt(uint32_t param)	//wujinyou, 2012.11.14
{
//	SENSOR_REG_TAB_INFO_T * sensor_reg_tab_info_ptr;
	uint32_t i;
//	SENSOR_IOCTL_FUNC_PTR write_reg_func;
	uint16_t subaddr;
	uint16_t data;
	int32_t ret = -1;
//	nsecs_t 			timestamp_old;
//	nsecs_t				timestamp_new;

	SENSOR_PRINT("SR300PC20_InitExt start \n");
	SR300PC20_I2C_write((SENSOR_REG_T*) reg_main_init);
	timestamp_old = systemTime(CLOCK_MONOTONIC);

	sensor_reg_tab_info_ptr = &s_SR300PC20_resolution_Tab_YUV[param];


	{
		SENSOR_REG_TAB_T regTab;
		regTab.reg_count			= sensor_reg_tab_info_ptr->reg_count;
		regTab.reg_bits				= SENSOR_I2C_REG_8BIT | SENSOR_I2C_VAL_8BIT;	//s_sensor_info_ptr->reg_addr_value_bits;
		regTab.burst_mode			= 7;
		regTab.sensor_reg_tab_ptr 	= sensor_reg_tab_info_ptr->sensor_reg_tab_ptr;

		ret = _Sensor_Device_WriteRegTab(&regTab);
	}

	//SENSOR_PRINT("SENSOR: Sensor_SendRegValueToSensor -> reg_count = %d, g_is_main_sensor: %d.\n",
	     			//sensor_reg_tab_info_ptr->reg_count, g_is_main_sensor);

	timestamp_new = systemTime(CLOCK_MONOTONIC);
	SENSOR_PRINT("SENSOR: SR300PC20_InitExt end, ret=%d, time=%d us\n", ret, (timestamp_new-timestamp_old)/1000);

	return SENSOR_SUCCESS;
}

*/ //comment for bringup

/******************************************************************************/
// Description: SR300PC20_Power_Ctrl
// Global resource dependence:
// Author:
// Note:
//
/******************************************************************************/
LOCAL uint32_t SR300PC20_Power_Ctrl(uint32_t power_on)
{
	SENSOR_AVDD_VAL_E dvdd_val = g_sr300pc20_yuv_info.dvdd_val;
	SENSOR_AVDD_VAL_E avdd_val = g_sr300pc20_yuv_info.avdd_val;
	SENSOR_AVDD_VAL_E iovdd_val = g_sr300pc20_yuv_info.iovdd_val;
	BOOLEAN power_down = g_sr300pc20_yuv_info.power_down_level;
	BOOLEAN reset_level = g_sr300pc20_yuv_info.reset_pulse_level;
	//uint32_t reset_width=g_ov5640_yuv_info.reset_pulse_width;
		SENSOR_PRINT("SENSOR:SR300PC20_Power_Ctrl_E (1:on, 0:off): %d \n", power_on);
	if (SENSOR_TRUE == power_on) {
		Sensor_PowerDown(power_down);
		// Open power
		Sensor_SetVoltage(dvdd_val, avdd_val, iovdd_val);
		SENSOR_Sleep(10);
		Sensor_SetMCLK(SENSOR_DEFALUT_MCLK);
		SENSOR_Sleep(2);
		Sensor_PowerDown(!power_down);
		// Reset sensor
		SENSOR_Sleep(10);

		Sensor_Reset(reset_level);
		SENSOR_Sleep(16);
	} else {
		Sensor_Reset(!reset_level);
		SENSOR_Sleep(15);
		Sensor_SetMCLK(SENSOR_DISABLE_MCLK);
		SENSOR_Sleep(1);
		Sensor_PowerDown(power_down);
		Sensor_SetVoltage(SENSOR_AVDD_CLOSED, SENSOR_AVDD_CLOSED,
				  SENSOR_AVDD_CLOSED);
	}
	SENSOR_PRINT("SENSOR:SR300PC20_Power_Ctrl_X (1:on, 0:off): %d \n", power_on);
	return SENSOR_SUCCESS;
}

/******************************************************************************/
// Description: SR300PC20_Identify
// Global resource dependence:
// Author:
// Note:
//
/******************************************************************************/
LOCAL uint32_t SR300PC20_Identify(uint32_t param)
{
	uint8_t ret       = 0;
	uint32_t i;

    SENSOR_PRINT("SR300PC20_Identify:E:\n");

    //return SENSOR_OP_SUCCESS;

    for(i = 0; i<2; i++ )
    {
       ret = Sensor_ReadReg(0x04);
	SENSOR_PRINT("anrry: SR300PC20 Read reg0x04 = %x\n",ret);

        if( ret ==  0xa4)
        {
                SENSOR_PRINT("The SR300PC20 sensor is Connected..!!\n");
                return SENSOR_OP_SUCCESS;
        }
     }
                SENSOR_PRINT("The SR300PC20 sensor is not Connected..!!\n");
     	        return SENSOR_OP_ERR;
}


LOCAL uint32_t SR300PC20_StreamOn(uint32_t param)
{
	SENSOR_PRINT("SENSOR: SR300PC201_StreamOn");

	//Sensor_WriteReg(0x01, 0xf0);

	return 0;
}

LOCAL uint32_t SR300PC20_StreamOff(uint32_t param)
{
	SENSOR_PRINT("SENSOR: SR300PC20_StreamOff");

//	Sensor_WriteReg(0x01, 0xf1);

	return 0;
}


/******************************************************************************/
// Description: SR300PC20_set_lightcapture
// Global resource dependence:
// Author:
// Note:
//
/******************************************************************************/
LOCAL uint32_t SR300PC20_set_lightcapture(uint32_t level)
{
#if 0
#ifdef CONFIG_LOAD_FILE
	CAM_DEBUG("SENSOR Tuning: SR300PC20_set_lightcapture ");

	switch(level)
		{
			case 0:
				Sensor_regs_table_write("SR300PC20_highlight_snapshot");
				break;
			case 1:
				Sensor_regs_table_write("SR300PC20_normal_snapshot");
				break;
			case 2:
				Sensor_regs_table_write("SR300PC20_lowlight_snapshot");
				break;
		}
#else
	if(level >=3)
		return SENSOR_OP_PARAM_ERR;

	CAM_DEBUG("NORMAL_LIGHT_CAPTURE..");
	//SR300PC20_I2C_write((SENSOR_REG_T*) sr300pc20_snapshot, "sr300pc20_snapshot");

	CAM_DEBUG("SR300PC20_set_lightcapture: level = %d", level);

#endif
#endif
	return 0;
}
/******************************************************************************/
// Description: SR300PC20_set_brightness
// Global resource dependence:
// Author:
// Note:
//
/******************************************************************************/
LOCAL uint32_t SR300PC20_set_brightness(uint32_t level)
{

	CAM_DEBUG("SENSOR : SR300PC20_set_brightness\n ");
	if(level >= 9)
		return SENSOR_OP_PARAM_ERR;

	switch(level)
		{
			case 0:
				SR300PC20_I2C_write(sr300pc20_brightness_tab_0,"sr300pc20_brightness_tab_0");
				break;
			case 1:
				SR300PC20_I2C_write(sr300pc20_brightness_tab_1,"sr300pc20_brightness_tab_1");
				break;
			case 2:
				SR300PC20_I2C_write(sr300pc20_brightness_tab_2,"sr300pc20_brightness_tab_2");
				break;
			case 3:
				SR300PC20_I2C_write(sr300pc20_brightness_tab_3,"sr300pc20_brightness_tab_3");
				break;
			case 4:
				SR300PC20_I2C_write(sr300pc20_brightness_tab_4,"sr300pc20_brightness_tab_4");
				break;
			case 5:
				SR300PC20_I2C_write(sr300pc20_brightness_tab_5,"sr300pc20_brightness_tab_5");
				break;
			case 6:
				SR300PC20_I2C_write(sr300pc20_brightness_tab_6,"sr300pc20_brightness_tab_6");
				break;
			case 7:
				SR300PC20_I2C_write(sr300pc20_brightness_tab_7,"sr300pc20_brightness_tab_7");
				break;
			case 8:
				SR300PC20_I2C_write(sr300pc20_brightness_tab_8,"sr300pc20_brightness_tab_8");
				break;

			default:
				CAM_DEBUG("[AWB]Invalid value is ordered!!!\n");
				break;
		}

	 //SR300PC20_I2C_write((SENSOR_REG_T*) sr300pc20_brightness_tab[level], "sr300pc20_brightness_tab");

	CAM_DEBUG("SR300PC20_set_brightness: level = %d", level);
    SENSOR_Sleep(10);
	return 0;
}

/******************************************************************************/
// Description: SR300PC20_set_contrast
// Global resource dependence:
// Author:
// Note:
//
/******************************************************************************/
LOCAL uint32_t SR300PC20_set_contrast(uint32_t level)
{

	CAM_DEBUG("SENSOR: SR300PC20_set_brightness ");
	if(level >= 7)
		return SENSOR_OP_PARAM_ERR;

	switch(level)
		{
			case 0:
				SR300PC20_I2C_write(sr300pc20_contrast_m_3,"sr300pc20_contrast_m_3");
				break;
			case 1:
				SR300PC20_I2C_write(sr300pc20_contrast_m_2,"sr300pc20_contrast_m_2");
				break;
			case 2:
				SR300PC20_I2C_write(sr300pc20_contrast_m_1,"sr300pc20_contrast_m_1");
				break;
			case 3:
				SR300PC20_I2C_write(sr300pc20_contrast_0,"sr300pc20_contrast_0");
				break;
			case 4:
				SR300PC20_I2C_write(sr300pc20_contrast_p_1,"sr300pc20_contrast_p_1");
				break;
			case 5:
				SR300PC20_I2C_write(sr300pc20_contrast_p_2,"sr300pc20_contrast_p_2");
				break;
			case 6:
				SR300PC20_I2C_write(sr300pc20_contrast_p_3,"sr300pc20_contrast_p_3");
				break;
			default:
				CAM_DEBUG("[AWB]Invalid value is ordered!!!\n");
				break;
		}

  //	SR300PC20_I2C_write((SENSOR_REG_T*) sr300pc20_contrast_tab[level], "sr300pc20_contrast_tab");
	SENSOR_PRINT("SR300PC20_set_contrast: level = %d", level);
	return 0;
}

/******************************************************************************/
// Description: SR300PC20_set_image_effect
// Global resource dependence:
// Author:
// Note:
//
/******************************************************************************/
LOCAL uint32_t SR300PC20_set_image_effect(uint32_t level)
{

	CAM_DEBUG("SENSOR : SR300PC20_set_image_effect ");
	if(level >=4)
		return SENSOR_OP_PARAM_ERR;

	switch(level)
		{
			case 0:
				SR300PC20_I2C_write(sr300pc20_effect_off,"sr300pc20_effect_off");
				break;
			case 1:
				SR300PC20_I2C_write(sr300pc20_effect_mono,"sr300pc20_effect_mono");
				break;
			case 2:
				SR300PC20_I2C_write(sr300pc20_effect_sepia,"sr300pc20_effect_sepia");
				break;
			case 3:
				SR300PC20_I2C_write(sr300pc20_effect_negative,"sr300pc20_effect_negative");
				break;

			default:
				SENSOR_TRACE("[Effect]Invalid value is ordered!!!\n");
				break;
		}

	//SR300PC20_I2C_write((SENSOR_REG_T*) sr300pc20_image_effect_tab[level], "sr300pc20_image_effect_tab");

	SENSOR_PRINT("SR300PC20_set_image_effect: level = %d", level);
	return 0;
}


/******************************************************************************/
// Description: set_SR300PC20_awb
// Global resource dependence:
// Author:
// Note:
//
/******************************************************************************/
LOCAL uint32_t SR300PC20_set_awb(uint32_t mode)
{
	CAM_DEBUG("SENSOR : set_SR300PC20_awb ");
	if(mode >= DCAMERA_WB_MODE_MAX)
		return SENSOR_OP_PARAM_ERR;

	switch(mode)
		{
			case 0:
				SR300PC20_I2C_write(sr300pc20_wb_auto,"sr300pc20_wb_auto");
				break;
			case 1:
				SR300PC20_I2C_write(sr300pc20_wb_daylight,"sr300pc20_wb_daylight");
				break;
			case 2:
				SR300PC20_I2C_write(sr300pc20_wb_cloudy,"sr300pc20_wb_cloudy");
				break;
			case 3:
				SR300PC20_I2C_write(sr300pc20_wb_incandescent,"sr300pc20_wb_incandescent ");
				break;
			case 4:
				SR300PC20_I2C_write(sr300pc20_wb_fluorescent,"sr300pc20_wb_fluorescent");
				break;

			default:
				CAM_DEBUG("[AWB]Invalid value is ordered!!!\n");
				break;
		}

	 //SR300PC20_I2C_write((SENSOR_REG_T*) sr300pc20_awb_tab[mode],"sr300pc20_awb_tab");

	SENSOR_PRINT("SR300PC20_set_awb_mode: mode = %d\n", mode);
	return 0;

}

/******************************************************************************/
// Description: SR300PC20_set_preview_mode
// Global resource dependence:
// Author:
// Note:
//
/******************************************************************************/
LOCAL uint32_t SR300PC20_set_scene_mode(uint32_t scene_mode)
{

	CAM_DEBUG("SENSOR : SR300PC20_set_scene_mode \n");
	if(scene_mode > 12)
		return SENSOR_OP_PARAM_ERR;

		//SR300PC20_I2C_write(sr300pc20_scene_off,"sr300pc20_scene_off");
/*
			case CAM_SCENEMODE_AUTO:
				iSceneValue = 0;
				bSetSceneOff = true;
				iRetryTime = 300;
				break;
			case CAM_SCENEMODE_PORTRAIT:	iSceneValue = 1; break;
			case CAM_SCENEMODE_LANDSCAPE: iSceneValue = 2; break;
			case CAM_SCENEMODE_SPORTS:    iSceneValue = 10; break;
			case CAM_SCENEMODE_NIGHT:     iSceneValue = 7; break;
			case CAM_SCENEMODE_INDOOR:    iSceneValue = 3; break;
			case CAM_SCENEMODE_BEACH:     iSceneValue = 11; break;
			case CAM_SCENEMODE_SUNSET:    iSceneValue = 4; break;
			case CAM_SCENEMODE_DAWN:      iSceneValue = 5; break;
			case CAM_SCENEMODE_FALL:      iSceneValue = 6; break;
			case CAM_SCENEMODE_FIREWORKS: iSceneValue = 8; break;
			case CAM_SCENEMODE_CANDLE:    iSceneValue = 9; break;
			case CAM_SCENEMODE_BACKLIGHT: iSceneValue = 12; break;
*/
		sr_scenemode = scene_mode;
		switch(scene_mode)
		{
			case 0:
				SR300PC20_I2C_write(sr300pc20_scene_off,"sr300pc20_scene_off");
				break;
			case 1:
				SR300PC20_I2C_write(sr300pc20_scene_portrait,"sr300pc20_scene_portrait");
				break;
			case 2:
				SR300PC20_I2C_write(sr300pc20_scene_landscape,"sr300pc20_scene_landscape");
				break;
			case 3:
				SR300PC20_I2C_write(sr300pc20_scene_party,"sr300pc20_scene_party");
				break;
			case 4:
				SR300PC20_I2C_write(sr300pc20_scene_sunset,"sr300pc20_scene_sunset");
				break;
			case 5:
				SR300PC20_I2C_write(sr300pc20_scene_dawn,"sr300pc20_scene_dawn");
				break;
			case 6:
				SR300PC20_I2C_write(sr300pc20_scene_fall,"sr300pc20_scene_fall");
				break;
			case 7:
				SR300PC20_I2C_write( sr300pc20_scene_nightshot,"sr300pc20_scene_nightshot");
				break;
			case 8:
				SR300PC20_I2C_write(sr300pc20_scene_firework,"sr300pc20_scene_firework");
				break;
			case 9:
				SR300PC20_I2C_write(sr300pc20_scene_candle,"sr300pc20_scene_candle");
				break;
			case 10:
				SR300PC20_I2C_write(sr300pc20_scene_sports,"sr300pc20_scene_sports");

				break;
			case 11:
				SR300PC20_I2C_write(sr300pc20_scene_beach,"sr300pc20_scene_beach");

				break;
			case 12:
				SR300PC20_I2C_write(sr300pc20_scene_backlight,"sr300pc20_scene_backlight");
				break;
			default:
				CAM_DEBUG("[SCENE]Invalid value is ordered!!!\n");
				break;
		}

	// SR300PC20_I2C_write((SENSOR_REG_T*) sr300pc20_scene_mode_tab[0], "sr300pc20_scene_mode_tab"); // Scene Off before setting Scene mode
	// SR300PC20_I2C_write((SENSOR_REG_T*) sr300pc20_scene_mode_tab[scene_mode], "sr300pc20_scene_mode_tab");

	SENSOR_PRINT("SR300PC20_set_scene_mode : mode = %d", scene_mode);
	return 0;
}

/******************************************************************************/
// Description: SR300PC20_set_quality
// Global resource dependence:
// Author:
// Note:
//
//******************************************************************************/
LOCAL uint32_t SR300PC20_set_quality(uint32_t quality_type)
{
#if 0
#ifdef CONFIG_LOAD_FILE
	CAM_DEBUG("SENSOR Tuning: SR300PC20_set_quality ");

	switch(quality_type)
		{
			case 0:
				Sensor_regs_table_write("SR300PC20_quality_tab_superfine");
				break;
			case 1:
				Sensor_regs_table_write("SR300PC20_quality_tab_fine");
				break;
			case 2:
				Sensor_regs_table_write("SR300PC20_quality_tab_normal");
				break;
			default:
				CAM_DEBUG("[Quality]Invalid value is ordered!!!\n");
				break;
		}
#else

	if(quality_type >= 3)
		return SENSOR_OP_PARAM_ERR;

	 SR300PC20_I2C_write((SENSOR_REG_T*) SR300PC20_quality_tab[quality_type]);
#endif
       SENSOR_Sleep(5);
#endif
	SENSOR_PRINT("SR300PC20_set_quality : level = %d", quality_type);

	return 0;
}


/******************************************************************************/
// Description: SR300PC20_set_Metering
// Global resource dependence:
// Author:
// Note:
//
//******************************************************************************/
LOCAL uint32_t SR300PC20_set_Metering(uint32_t metering_mode)
{

	CAM_DEBUG("SENSOR: SR300PC20_set_Metering_mode ");
	if(metering_mode >= 3)
		return SENSOR_OP_PARAM_ERR;

	switch(metering_mode)
		{
			case 0:
				SR300PC20_I2C_write(sr300pc20_metering_normal,"sr300pc20_metering_normal");
				break;
			case 1:
				SR300PC20_I2C_write(sr300pc20_metering_spot,"sr300pc20_metering_spot");
				break;
			case 2:
				SR300PC20_I2C_write(sr300pc20_metering_center,"sr300pc20_metering_center");
				break;
			default:
				CAM_DEBUG("[Metering]Invalid value is ordered!!!\n");
				break;
		}

	//SR300PC20_I2C_write((SENSOR_REG_T*) sr300pc20_metering_mode_tab[metering_mode], "sr300pc20_metering_mode_tab");

	SENSOR_PRINT("SR300PC20_set_Metering_mode : level = %d", metering_mode);
	return 0;
}


/******************************************************************************/
// Description: SR300PC20_set_DTP
// Global resource dependence:
// Author:
// Note:
//
//******************************************************************************/
LOCAL uint32_t SR300PC20_set_DTP(uint32_t dtp_mode)
{
	CAM_DEBUG("SENSOR : SR300PC20_set_DTP_mode ");
	if(dtp_mode >= 2)
		return SENSOR_OP_PARAM_ERR;

	switch(dtp_mode)
		{
			case 0:
				SR300PC20_I2C_write(sr300pc20_dtp_off,"sr300pc20_dtp_off");
				break;
			case 1:
				SR300PC20_I2C_write(sr300pc20_dtp_on,"sr300pc20_dtp_on");
				break;
			default:
				SENSOR_PRINT("[DTP]Invalid value is ordered!!!\n");
				break;
		}

	// SR300PC20_I2C_write((SENSOR_REG_T*) sr300pc20_dtp_mode_tab[dtp_mode], "sr300pc20_dtp_mode_tab");

	SENSOR_PRINT("SR300PC20_set_DTP_mode : level = %d", dtp_mode);
	return 0;
}


/******************************************************************************/
// Description: SR300PC20_set_FPS
// Global resource dependence:
// Author:
// Note:
//
//******************************************************************************/
LOCAL uint32_t SR300PC20_set_FPS(uint32_t fps_mode)
{

	CAM_DEBUG("SENSOR : SR300PC20_set_FPS  fps_mode :%d\n", fps_mode);
/*
			case 0:   iFramerateValue = 7; break;
			case 5:   iFramerateValue = 0; break;
			case 7:   iFramerateValue = 1; break;
			case 10:  iFramerateValue = 2; break;
			case 12:  iFramerateValue = 3; break;
			case 15:  iFramerateValue = 4; break;
			case 25:  iFramerateValue = 5; break;
			case 30:  iFramerateValue = 6; break;
*/
	if(fps_mode >= 8)
		return SENSOR_OP_PARAM_ERR;
	if (s_firstInit)
	{
		s_firstInit = 0;
		if  (fps_mode == 7)
			return 0;
	}
	s_firstInit = 0;
	switch(fps_mode)
		{
			case 7:
				SR300PC20_I2C_write(sr300pc20_fps_auto,"sr300pc20_fps_auto");
				break;
			case 4:
				SR300PC20_I2C_write(sr300pc20_fps_15fix,"sr300pc20_fps_15fix");
				break;
			case 5:
				SR300PC20_I2C_write(sr300pc20_fps_24fix,"sr300pc20_fps_24fix");
				break;
			case 6:
				SR300PC20_I2C_write(sr300pc20_fps_30fix,"sr300pc20_fps_30fix");
				break;
			default:
				SR300PC20_I2C_write(sr300pc20_fps_auto,"sr300pc20_fps_auto");
				CAM_DEBUG("[FPS]Invalid value is ordered!!!\n");
				break;
		}

	//SR300PC20_I2C_write((SENSOR_REG_T*) sr300pc20_fps_mode_tab[fps_mode], "sr300pc20_fps_mode_tab");

	SENSOR_PRINT("SR300PC20_set_FPS : level = %d", fps_mode);
	return 0;
}


/******************************************************************************/
// Description:SR300PC20_BeforeSnapshot
// Global resource dependence:
// Author:
// Note:
/******************************************************************************/
LOCAL uint32_t SR300PC20_BeforeSnapshot(uint32_t param)
{
	int sensorlight = 20;

	printk("SR300PC20_BeforeSnapsho: mode = %d", param);
	s_caputuresize = param;
	if ((sr_scenemode == 7) || (sr_scenemode == 8))
	{
		Sensor_WriteReg_8bits(0x03, 0x20);
		Sensor_ReadReg_8bits(0xb1, &sensorlight);
		printk("SR300PC20_BeforeSnapshot sensor light:%d \n", sensorlight);
		if ( (sensorlight < 16) && (sr_scenemode == 8))
		{
			switch(param)
			{
				case 0: //(width ==  2048 && height == 1536)
					SR300PC20_I2C_write((SENSOR_REG_T*) sr300pc20_fireworks_snapshot, "sr300pc20_fireworks_snapshot");
					break;
				case 2://(width ==  1600  && height == 1200)
					SR300PC20_I2C_write((SENSOR_REG_T*) sr300pc20_1600x1200_fireworks_snapshot, "sr300pc20_1600x1200_fireworks_snapshot");
					break;
				case 4:
					SR300PC20_I2C_write((SENSOR_REG_T*) sr300pc20_1280x960_fireworks_snapshot, "sr300pc20_1280x960_fireworks_snapshot");
					break;
				case 10:
					SR300PC20_I2C_write((SENSOR_REG_T*) sr300pc20_640x480_preview_set, "sr300pc20_640x480_preview_set");
					break;
				default:
					CAM_DEBUG("[Capture_mode]Invalid value is ordered!!!\n");
					break;
			}
			return 0;
		}
		else if ( (sensorlight < 16) && (sr_scenemode == 7))
		{
			switch(param)
			{
				case 0: //(width ==  2048 && height == 1536)
					SR300PC20_I2C_write((SENSOR_REG_T*) sr300pc20_night_snapshot, "sr300pc20_night_snapshot");
					break;
				case 2://(width ==  1600  && height == 1200)
					SR300PC20_I2C_write((SENSOR_REG_T*) sr300pc20_1600x1200_night_snapshot, "sr300pc20_1600x1200_night_snapshot");
					break;
				case 4:
					SR300PC20_I2C_write((SENSOR_REG_T*) sr300pc20_1280x960_night_snapshot, "sr300pc20_1280x960_night_snapshot");
					break;
				case 10:
					SR300PC20_I2C_write((SENSOR_REG_T*) sr300pc20_640x480_preview_set, "sr300pc20_640x480_preview_set");
					break;
				default:
					CAM_DEBUG("[Capture_mode]Invalid value is ordered!!!\n");
					break;
			}
			return 0;
		}
	}



	switch(param)
	{
		case 0: //(width ==  2048 && height == 1536)
			SR300PC20_I2C_write((SENSOR_REG_T*) sr300pc20_snapshot, "sr300pc20_snapshot");
			break;
		case 2://(width ==  1600  && height == 1200)
			SR300PC20_I2C_write((SENSOR_REG_T*) sr300pc20_1600x1200_snapshot, "sr300pc20_1600x1200_snapshot");
			break;
		case 4:
			SR300PC20_I2C_write((SENSOR_REG_T*) sr300pc20_1280x960_snapshot, "sr300pc20_1280x960_snapshot");
			break;
		case 10:
			SR300PC20_I2C_write((SENSOR_REG_T*) sr300pc20_640x480_preview_set, "sr300pc20_640x480_preview_set");
			break;
		default:
			CAM_DEBUG("[Capture_mode]Invalid value is ordered!!!\n");
			break;
	}

	//SR300PC20_I2C_write((SENSOR_REG_T*) sr300pc20_snapshot, "sr300pc20_snapshot");


	return 0;
}

/******************************************************************************/
// Description:SR300PC20_after_snapshot
// Global resource dependence:
// Author:
// Note:
/******************************************************************************/
LOCAL uint32_t SR300PC20_after_snapshot(uint32_t param)
{

    uint32_t  preview_mode = (param >= SENSOR_MODE_PREVIEW_TWO) ? \
                            SENSOR_MODE_PREVIEW_TWO:SENSOR_MODE_PREVIEW_ONE;

	CAM_DEBUG("SENSOR : SR300PC20_after_snapshot \n");
	SR300PC20_I2C_write(sr300pc20_update_preview_setting,"sr300pc20_update_preview_setting");
	SENSOR_PRINT("SR300PC20_AfterSnapsho: mode = %d\n", param);
	if (param == 0) //Songww 20130320 add for 720*480 record
	{
	 	SENSOR_PRINT("sww sr300pc20_720_480_30fix ");
	      SR300PC20_I2C_write(sr300pc20_720_480_30fix,"sr300pc20_720_480_30fix");
      }
	return 0;
}

/******************************************************************************/
// Description:SR300PC20_set_preview_mode
// Global resource dependence:
// Author:
// Note:
/******************************************************************************/

LOCAL uint32_t SR300PC20_set_preview_mode(uint32_t preview_mode)
{

#if 0

#ifdef CONFIG_LOAD_FILE
			CAM_DEBUG("SENSOR Tuning: SR300PC20_set_preview_mode ");

			switch(preview_mode)
				{
					case 0:
						Sensor_regs_table_write("SR300PC20_preview_tab_720_480");
						break;
					case 1:
						Sensor_regs_table_write("SR300PC20_preview_tab_640_480");
						break;
					case 2:
						Sensor_regs_table_write("SR300PC20_preview_tab_480_320");
						break;
					case 3:
						Sensor_regs_table_write("SR300PC20_preview_tab_424_318");
						break;
					case 4:
						Sensor_regs_table_write("SR300PC20_preview_tab_352_288");
						break;
					case 5:
						Sensor_regs_table_write("SR300PC20_preview_tab_320_240");
						break;
					case 6:
						Sensor_regs_table_write("SR300PC20_preview_tab_176_144");
						break;
					case 7:
						Sensor_regs_table_write("SR300PC20_preview_tab_160_120");
						break;
					case 8:
						Sensor_regs_table_write("SR300PC20_preview_tab_144_176");
						break;

					default:
						SENSOR_TRACE("[PREVIEW]Invalid value is ordered!!!\n");
						break;
				}
#else
		if(preview_mode >= 9)
			return SENSOR_OP_PARAM_ERR;

		SR300PC20_I2C_write((SENSOR_REG_T*) SR300PC20_preview_mode_tab[preview_mode]);
#endif
#endif
		SENSOR_PRINT("SR300PC20_set_preview_mode : mode = %d", preview_mode);
		return 0;

}

/******************************************************************************/
// Description:SR300PC20_set_capture_mode
// Global resource dependence:
// Author:
// Note:
/******************************************************************************/

LOCAL uint32_t SR300PC20_set_capture_mode(uint32_t capture_mode)
{
#if 0
#ifdef CONFIG_LOAD_FILE
			CAM_DEBUG("SENSOR Tuning: SR300PC20_set_capture_mode ");

			switch(capture_mode)
				{
					case 0:
						Sensor_regs_table_write("SR300PC20_capture_tab_2048_1536");
						break;
					case 1:
						Sensor_regs_table_write("SR300PC20_capture_tab_2048_1360");
						break;
					case 2:
						Sensor_regs_table_write("SR300PC20_capture_tab_1600_1200");
						break;
					case 3:
						Sensor_regs_table_write("SR300PC20_capture_tab_1600_1072");
						break;
					case 4:
						Sensor_regs_table_write("SR300PC20_capture_tab_1280_960");
						break;
					case 5:
						Sensor_regs_table_write("SR300PC20_capture_tab_1280_848");
						break;
					case 6:
						Sensor_regs_table_write("SR300PC20_capture_tab_1024_768");
						break;
					case 7:
						Sensor_regs_table_write("SR300PC20_capture_tab_800_600");
						break;
					case 8:
						Sensor_regs_table_write("SR300PC20_capture_tab_720_480");
						break;
					case 9:
						Sensor_regs_table_write("SR300PC20_capture_tab_640_480");
						break;
					case 10:
						Sensor_regs_table_write("SR300PC20_capture_tab_480_320");
						break;
					case 11:
						Sensor_regs_table_write("SR300PC20_capture_tab_352_288");
						break;
					case 12:
						Sensor_regs_table_write("SR300PC20_capture_tab_320_240");
						break;
					case 13:
						Sensor_regs_table_write("SR300PC20_capture_tab_176_144");
						break;
					case 14:
						Sensor_regs_table_write("SR300PC20_capture_tab_160_120");
						break;

					default:
						SENSOR_TRACE("[CAPTURE]Invalid value is ordered!!!\n");
						break;
				}
#else
		if(capture_mode >= 15)
			return SENSOR_OP_PARAM_ERR;

		SR300PC20_I2C_write((SENSOR_REG_T*) SR300PC20_capture_mode_tab[capture_mode]);

#endif
#endif
		SENSOR_PRINT("SR300PC20_set_capture_mode : mode = %d", capture_mode);
		return 0;

}

/******************************************************************************/
// Description:SR300PC20_GetExifInfo
// Global resource dependence:
// Author:
// Note:
/******************************************************************************/
LOCAL uint32_t SR300PC20_Get_Exif_Exporsure(uint32_t level)
{
	uint32_t shutter_speed =0;
	uint32_t exposure_time = 0;
	uint8_t shutter_speed_a = 0;
	uint8_t shutter_speed_b= 0;
	uint8_t shutter_speed_c= 0;
	uint8_t shutter_speed_d= 0;
	int32 err;

	CAM_DEBUG( "Exposure_Time() s_caputuresize:%d \r\n", s_caputuresize);

	err = Sensor_WriteReg_8bits(0x03, 0x20);

	Sensor_ReadReg_8bits(0xa0, &shutter_speed_a);
	Sensor_ReadReg_8bits(0xa1, &shutter_speed_b);
	Sensor_ReadReg_8bits(0xa2, &shutter_speed_c);
	Sensor_ReadReg_8bits(0xa3, &shutter_speed_d);

	if (err == -1)
	{
		CAM_DEBUG( "Exposure_Time() Read REg Error \r\n");
	}

	shutter_speed = (shutter_speed_a << 24)  | (shutter_speed_b << 16) | (shutter_speed_c << 8) |shutter_speed_d;

	CAM_DEBUG( "shutter_speed : %d abcd:0x%x,0x%x,0x%x,0x%x,\n",shutter_speed,shutter_speed_a,shutter_speed_b,shutter_speed_c,shutter_speed_d);
	if(shutter_speed ==0 ) shutter_speed =1;
	if (s_caputuresize == 10)
	{
		exposure_time = 54000000/shutter_speed;
		CAM_DEBUG( "exposure_time VGA : %d \n",exposure_time);

	}
	else
	{
		exposure_time = 27000000/shutter_speed;
		CAM_DEBUG( "exposure_time : %d \n",exposure_time);
	}



	return exposure_time;

}


LOCAL uint32_t SR300PC20_Get_Exif_ISO(uint32_t level)
{
	uint8_t iso_gain= 0;
	uint16_t iso_value= 0;
	uint32_t err;

	CAM_DEBUG( "SR300PC20_Get_ISO() \r\n");

	err = Sensor_WriteReg_8bits(0x03, 0x20);

	err = Sensor_ReadReg_8bits(0xa0, &iso_gain);

	if (err == -1)
	{
		CAM_DEBUG( "SR300PC20_Get_Exif_ISO() Read REg Error \r\n");
	}
	CAM_DEBUG( "iso_gain() %d \n",iso_gain);

	if(iso_gain < 0x25)
		iso_value = 50;
	else if(iso_gain < 0x5c)
		iso_value =100;
	else if(iso_gain < 0x83)
		iso_value =200;
	else if(iso_gain < 0xf1)
		iso_value =400;

	SENSOR_TRACE( "iso_value : %d \n",iso_value);

	return iso_value;

}

LOCAL uint32_t SR300PC20_Get_Exif_Flash(uint32_t level)
{
	CAM_DEBUG( "SR300PC20_Get_Exif_Flash() \r\n");

	return 0;
}

LOCAL uint32_t SR300PC20_I2C_write(SENSOR_REG_T* sensor_reg_ptr, char* reg_name)
{
	uint16_t 	i;

       printk("SR300PC20_I2C_write  E: count = %d reg_name: %s \n", i, reg_name);

#ifdef CONFIG_LOAD_FILE
    SENSOR_PRINT("SR300PC20_I2C_write:tuning from file" );
    Sensor_regs_table_write(reg_name);
#else
       for(i = 0; (0xFF != sensor_reg_ptr[i].reg_addr) || (0xFF != sensor_reg_ptr[i].reg_value) ; i++)
       {
	     //SENSOR_PRINT("SR300PC20_I2C_write : addr = %x, data = %x \n",sensor_reg_ptr[i].reg_addr,sensor_reg_ptr[i].reg_value);
            Sensor_WriteReg(sensor_reg_ptr[i].reg_addr, sensor_reg_ptr[i].reg_value);
            //Sensor_WriteReg(sensor_reg_ptr[i].reg_addr, sensor_reg_ptr[i].reg_value);
       }
#endif
	 SENSOR_PRINT("SR300PC20_I2C_write  X: count = %d\n", i);

	 return 0;
}




#define BURST_MODE_BUFFER_MAX_SIZE 2700
uint8_t SR300PC20_buf_for_burstmode[BURST_MODE_BUFFER_MAX_SIZE];
LOCAL uint32_t SR300PC20_init_by_burst_write(uint32_t param)
{
    int i = 0;
    int idx;
    int err = -1;
    int retry = 0;
    uint8_t subaddr=0;//next_subaddr=0;
    uint8_t value=0;
    int burst_flag = 0;

    struct i2c_client *i2c_client = Sensor_GetI2CClien();
//    struct timeval time1, time2;

    struct i2c_msg msg = {i2c_client->addr, 0, 0, SR300PC20_buf_for_burstmode };
    uint32_t init_table_size = s_SR300PC20_resolution_Tab_YUV[SENSOR_MODE_COMMON_INIT].reg_count;
    SENSOR_REG_T_PTR  reg_table = s_SR300PC20_resolution_Tab_YUV[SENSOR_MODE_COMMON_INIT].sensor_reg_tab_ptr;

    s_firstInit = 1;
    sr_scenemode = 0;//bst chris.shen add reset the scene mode value of driver during enter camera.
  //  do_gettimeofday(&time1);

I2C_RETRY:

    idx = 0;


    for (i = 0; i < init_table_size; i++)
    {
        if(idx > (BURST_MODE_BUFFER_MAX_SIZE-10))
        {
            SENSOR_PRINT_ERR("SR300PC20_sensor_burst_write_buffer_overflow!!!\n");
            return err;
        }

        subaddr = reg_table[i].reg_addr;
	value = reg_table[i].reg_value;
	if ( burst_flag == 0)
	{
	        switch(subaddr)
	        {
	            case 0x0e :
				if (value != 0x00) burst_flag = 1;
	            break;

	            case 0xFF :
				if(value == 0xFF)
				{
	           			 break;
				}
				else if(value >= 10)
				{
					SENSOR_Sleep(value);
				}
				else
				{
					SENSOR_Sleep(value);
				}
	                //SENSOR_PRINT_HIGH("SR300PC20_sensor_burst_write, delay %dms\n",value);
	            break;

			default:
	                idx=0;
	                err = Sensor_WriteReg(subaddr,value);
	            break;

	        }
	}
	else if (burst_flag == 1)
	{
		if(subaddr == 0x0e && value == 0x00)
		{
			msg.len = idx;
			err = i2c_transfer(i2c_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
			//printk("SR300PC20_init_by_burst_write, idx = %d\n",idx);
			idx = 0;
			burst_flag = 0;
		}
		else
		{
			if(idx == 0)
			{
				SR300PC20_buf_for_burstmode[idx++] = subaddr;
				SR300PC20_buf_for_burstmode[idx++] = value;
			}
			else
				SR300PC20_buf_for_burstmode[idx++] = value;
		}
}
	}

	if (err < 0)
        {
            SENSOR_PRINT_HIGH("[SR300PC20]%s: register set failed. try again.\n",__func__);
		retry++;
            if((retry++)<3) goto I2C_RETRY;
            return err;
        }

//    do_gettimeofday(&time2);
//    printk("SENSOR: _SR300PC20_InitExt time=%d.\n",((time2.tv_sec-time1.tv_sec)*1000+(time2.tv_usec-time1.tv_usec)/1000));
    SENSOR_PRINT_HIGH("SENSOR: burst _SR300PC20_InitExt, success \n");

    return 0;
}


#if 1
LOCAL uint32_t SR300PC20_InitExt(uint32_t param)
{
	uint32_t              rtn = SENSOR_SUCCESS;
//	int ret = 0;

	printk("***** %s %d\n", __func__, __LINE__);
	Sensor_SendRegTabToSensor(&s_SR300PC20_resolution_Tab_YUV);
	/*
	uint32_t              i = 0;
	uint32_t              written_num = 0;
	uint16_t              wr_reg = 0;
	uint16_t              wr_val = 0;
	uint32_t              wr_num_once = 0;
	uint32_t              wr_num_once_ret = 0;
	uint32_t		   alloc_size = 0;
	//uint32_t              init_table_size = NUMBER_OF_ARRAY(reg_main_init);
	//SENSOR_REG_T_PTR    p_reg_table = reg_main_init;
	uint32_t              init_table_size = s_SR300PC20_resolution_Tab_YUV[SENSOR_MODE_COMMON_INIT].reg_count;
	SENSOR_REG_T_PTR    p_reg_table = s_SR300PC20_resolution_Tab_YUV[SENSOR_MODE_COMMON_INIT].sensor_reg_tab_ptr;
	uint8_t               *p_reg_val_tmp = 0;
	//struct i2c_msg msg_w;
	//struct i2c_client *i2c_client = Sensor_GetI2CClien();
	//struct timeval time1, time2;

	CAM_DEBUG("SENSOR: _SR300PC20_InitExt, init_table_size = %d \n", init_table_size);
	if(0 == i2c_client)
	{
		CAM_DEBUG("SENSOR: _SR300PC20_InitExt:error,i2c_client is NULL!.\n");
	}

	do_gettimeofday(&time1);

	alloc_size = init_table_size*sizeof(uint16_t) + 16;
	p_reg_val_tmp = (uint8_t*)kzalloc(alloc_size, GFP_KERNEL);

	if(0 == p_reg_val_tmp)
	{
		CAM_DEBUG("_SR300PC20_InitExt: kzalloc failed, size = %d \n", alloc_size);
		return 1;
	}

	while(written_num < init_table_size)
	{
		wr_reg = p_reg_table[written_num].reg_addr;
		wr_val = p_reg_table[written_num].reg_value;
		if(SENSOR_WRITE_DELAY == wr_reg)
		{
			if(wr_val >= 10)
			{
				SENSOR_Sleep(wr_val);
			}
			else
			{
				SENSOR_Sleep(wr_val);
			}
		}
		else
		{
			p_reg_val_tmp[0] = (uint8)((wr_reg >> 8) & 0xFF);
			p_reg_val_tmp[1] = (uint8)(wr_reg & 0xFF);
			p_reg_val_tmp[2] = (uint8)((wr_val >> 8) & 0xFF);
			p_reg_val_tmp[3] = (uint8)(wr_val & 0xFF);
			wr_num_once = 2;
			for(i = written_num + 1; i< init_table_size; i++)
			{
				if(p_reg_table[i].reg_addr != wr_reg)
				{
					break;
				}
				else
				{
					wr_val = p_reg_table[i].reg_value;
					p_reg_val_tmp[2*wr_num_once] = (uint8)((wr_val >> 8) & 0xFF);
					p_reg_val_tmp[2*wr_num_once+1] = (uint8)(wr_val & 0xFF);
					wr_num_once ++;
#if 0
					if(wr_num_once >= I2C_WRITE_BURST_LENGTH)
					{
						break;
					}
#endif
				}
			}

			msg_w.addr = i2c_client->addr;
			msg_w.flags = 0;
			msg_w.buf = p_reg_val_tmp;
			msg_w.len = (uint32)(wr_num_once*2);
			ret = i2c_transfer(i2c_client->adapter, &msg_w, 1);
			if(ret!=1)
			{
				CAM_DEBUG("SENSOR: _SR300PC20_InitExt, i2c write once error \n");
				rtn = 1;
				break;
			}
			else
			{
#if 0
				SENSOR_PRINT("SENSOR: _SR300PC20_InitExt, i2c write once from %d {0x%x 0x%x}, total %d registers {0x%x 0x%x}",
				      written_num,cmd[0],cmd[1],wr_num_once,p_reg_val_tmp[0],p_reg_val_tmp[1]);
				if(wr_num_once > 1)
				{
					SENSOR_PRINT("SENSOR: _SR300PC20_InitExt, val {0x%x 0x%x} {0x%x 0x%x} {0x%x 0x%x} {0x%x 0x%x} {0x%x 0x%x} {0x%x 0x%x}.\n",
				          p_reg_val_tmp[0],p_reg_val_tmp[1],p_reg_val_tmp[2],p_reg_val_tmp[3],
				          p_reg_val_tmp[4],p_reg_val_tmp[5],p_reg_val_tmp[6],p_reg_val_tmp[7],
				          p_reg_val_tmp[8],p_reg_val_tmp[9],p_reg_val_tmp[10],p_reg_val_tmp[11]);

				}
#endif
			}
		}
		written_num += wr_num_once-1;
	}

    kfree(p_reg_val_tmp);

    do_gettimeofday(&time2);
    CAM_DEBUG("SENSOR: _SR300PC20_InitExt time=%d.\n",((time2.tv_sec-time1.tv_sec)*1000+(time2.tv_usec-time1.tv_usec)/1000));

    CAM_DEBUG("SENSOR: _SR300PC20_InitExt, success \n");
	*/
    return rtn;
}
#endif

//#if 0

struct sensor_drv_cfg sensor_SR300PC20 = {
	.sensor_pos = CONFIG_DCAM_SENSOR_POS_SR300PC20,
	.sensor_name = "SR300PC20",
	.driver_info = &g_sr300pc20_yuv_info,
};


static ssize_t Rear_Cam_Sensor_ID(struct device *dev, struct device_attribute *attr, char *buf)
{
	CAM_DEBUG("Rear_Cam_Sensor_ID\n");
	return  sprintf(buf, "SR300PC20");
}

static DEVICE_ATTR(rear_camfw, S_IRUGO | S_IWUSR , Rear_Cam_Sensor_ID, NULL);
static int __init sensor_SR300PC20_init(void)
{

	struct device *dev_t;

	camera_class_sr300pc20 = class_create(THIS_MODULE, "camera");

	if (IS_ERR(camera_class_sr300pc20))
	{
	 CAM_DEBUG("Failed to create camera_class_sr300pc20!\n");
	 return PTR_ERR( camera_class_sr300pc20 );
	}

     dev_t = device_create(camera_class_sr300pc20,
				NULL, 0, NULL, "rear");

	if (device_create_file(dev_t, &dev_attr_rear_camfw) < 0)
	 CAM_DEBUG("Failed to create device file(%s)!\n", dev_attr_rear_camfw.attr.name);


	return dcam_register_sensor_drv(&sensor_SR300PC20);
}

subsys_initcall(sensor_SR300PC20_init);
//#endif

#ifdef __cplusplus
}
#endif
