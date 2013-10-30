/* drivers/input/touchscreen/qt602240.c
 *
 * Quantum TSP driver.
 *
 * Copyright (C) 2009 Samsung Electronics Co. Ltd.
 *
 * 2010 Modified by LG Electronics Co., Ltd.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */


#include <linux/module.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/earlysuspend.h>
#include <linux/jiffies.h>
#include <linux/wakelock.h>
#include <linux/miscdevice.h>
#include <linux/i2c-gpio.h>
#include <linux/uaccess.h>
#include <mach/vreg.h>
#include <mach/board_lge.h>
#include <linux/slab.h>


#include "qt602240.h"

#define QT_TOUCH_DEBUG 1
 #if QT_TOUCH_DEBUG
 #define DEBUG_M(args...)  if (l_debug_trace_on == true)	\
 				printk(args)
 #else
 #define DEBUG_M(args...)
 #endif

//#define QT_TOUCH_DEBUG_IRQ_CHECK
#ifdef QT_TOUCH_DEBUG_IRQ_CHECK
static void irq_handler_message_print(void);
uint8_t quantum_msg_for_print[9] = {0};
#endif

struct i2c_driver qt602240_i2c_driver;
struct workqueue_struct *qt602240_wq;
struct qt602240_data *qt602240;
struct touch_platform_data *qt602240_pdata;

static report_finger_info_t fingerinfo[MAX_USING_FINGER_NUM];


static info_block_t info_block_data;
static info_block_t *info_block = &info_block_data;

static report_id_map_t report_id_map_data[30];
static report_id_map_t *report_id_map = &report_id_map_data[0];

static object_t info_object_table[25];
static object_t *info_object_ptr = &info_object_table[0];

/*! Message buffer holding the latest message received. */
uint8_t quantum_msg[9] = {0};

static int max_report_id = 0;

uint8_t max_message_length;

uint16_t message_processor_address;

/*! Command processor address. */
static uint16_t command_processor_address;

/*! Flag indicating if driver setup is OK. */
static enum driver_setup_t driver_setup = DRIVER_SETUP_INCOMPLETE;

/*! \brief The current address pointer. */
static U16 address_pointer;

static uint8_t tsp_version;
static uint8_t cal_check_flag = 0u;

static unsigned char *QT602240_firmware_ptr;
static int QT602240_firmware_size;

static int qt_boot_download;

struct wake_lock tsp_firmware_wake_lock;
struct wake_lock tsp_sensing_wake_lock;

static int qt_read_mem_fail_count = 0;
static bool l_debug_trace_on = false;
static int nCheckMultiTouchErrCheck = 0;
//static bool bIsMultiTouchPass = false;
static bool bCalibrationAndResetChip = false;

#ifdef LG_FW_TOUCH_SOFT_KEY

#define PRESSED 	1
#define RELEASED 	0
#define MOVE		2

static int nCurrentState = -1;

#define QT_EDGE_COMPENSATION_Y 25
enum
{
	QT_TOUCHKEY_MENU = 0,
	QT_TOUCHKEY_HOME,
	QT_TOUCHKEY_BACK,
	QT_TOUCHKEY_SEARCH,
	QT_TOUCHKEY_MAX
};

typedef struct
{
	int16_t x1;         /* x start        */
	int16_t y1;         /* y start        */
	int16_t x2;         /* x end          */
	int16_t y2;         /* y end          */
	int16_t amp_thrs;	/* amp threshold  */
	int16_t size_thrs;	/* size threshold */
	int16_t key;        /* linux key type */
	int16_t id;         /* pressed id     */

} touchkey_area_info_t;

static touchkey_area_info_t touchkey_area[QT_TOUCHKEY_MAX] = {
/*     x1,   y1,   x2,    y2,  amp, size,        key,      id                          */
    {    9,  515,   49,  552,  500,   50,   KEY_MENU,    0xFF }, /* QT_TOUCHKEY_MENU   */
    {   92,  515,  136,  552,  500,   50,   KEY_HOME,    0xFF }, /* QT_TOUCHKEY_HOME   */
    {  177,  515,  222,  552,  500,   50,   KEY_BACK,    0xFF }, /* QT_TOUCHKEY_BACK   */
    {  263,  515,  306,  552,  500,   50, KEY_SEARCH,    0xFF }  /* QT_TOUCHKEY_SEARCH */
};

static int nTouchKeyPressedId = -1;
static bool bIsTouchKeyPressed = false;
static bool bNeedCheckResetConfition = true;

enum
{	
	QT_TOUCHKEY_STATE_PRESS = 0,
	QT_TOUCHKEY_STATE_MOVE_VALID_AREA,
	QT_TOUCHKEY_STATE_MOVE_INVALID_AREA,
	QT_TOUCHKEY_STATE_RELEASE,
	QT_TOUCHKEY_STATE_NONE,
};

static int nTouchKeyState = QT_TOUCHKEY_STATE_NONE;
#endif

struct __reg_i2c_read_config readConfig[16];
static unsigned int read_config_array_count = 0;
static void qt_saving_regs_values(void);
static void refresh_input(void);

/*------------------- for tunning ATmel - start --------------------*/
struct class *touch_class;
EXPORT_SYMBOL(touch_class);

struct device *switch_test;
EXPORT_SYMBOL(switch_test);


/*********************************************************************
*
*       QT602240 Object table init
*
* ********************************************************************/

/* General Object */
gen_commandprocessor_t6_config_t command_config = {0};
gen_powerconfig_t7_config_t power_config = {0};
gen_acquisitionconfig_t8_config_t acquisition_config = {0};


/* Touch Object */
touch_multitouchscreen_t9_config_t touchscreen_config = {0};
touch_keyarray_t15_config_t keyarray_config = {0};
touch_proximity_t23_config_t proximity_config = {0};


/* Signal Processing Objects */
proci_gripfacesuppression_t20_config_t gripfacesuppression_config = {0};
procg_noisesuppression_t22_config_t noise_suppression_config = {0};
proci_onetouchgestureprocessor_t24_config_t onetouch_gesture_config = {0};
#ifndef __VER_2_0__
 proci_twotouchgestureprocessor_t27_config_t twotouch_gesture_config = {0};
#endif


/* Support Objects */
spt_gpiopwm_t19_config_t  gpiopwm_config = {0};
spt_selftest_t25_config_t selftest_config = {0};
spt_cteconfig_t28_config_t cte_config = {0};
spt_comcconfig_t18_config_t   comc_config = {0};


/*--------------- for tunning ATmel - end --------------------------*/


#define __QT_CONFIG__
/*********************************************************************
*
*
*       QT602240  Configuration Block
*
*
*********************************************************************/

void qt_power_config_init(void)
{
    printk(KERN_INFO"\n[TSP]%s real board \n", __func__);

    /* Set Idle Acquisition Interval to 255(Free-run Mode) */
    power_config.idleacqint = 64;//255;

    /* Set Active Acquisition Interval to 255(Free-run Mode) */
    power_config.actvacqint = 255;


    /* Set Active to Idle Timeout to 10s (one unit = 200ms). */
    power_config.actv2idleto = 50;//0;
    
    /* Write power config to chip. */
    if (write_power_config(power_config) != CFG_WRITE_OK)
		QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
}

void qt_acquisition_config_init(void)
{
    acquisition_config.chrgtime = 10;
    acquisition_config.reserved = 0;

    acquisition_config.tchdrift = 5;//20;
    acquisition_config.driftst = 0;//20;

    acquisition_config.tchautocal = 0;   /* infinite(Disable) */
    acquisition_config.sync = 0;         /* Disabled */

    acquisition_config.atchcalst = 9;    /* 2400ms */	//0902
    acquisition_config.atchcalsthr = 5;//35; //anti-touch solution

#ifdef __VER_2_0__
    acquisition_config.atchfrccalthr = 0;
    acquisition_config.atchfrccalratio = 0;
#endif
    										       
    if (write_acquisition_config(acquisition_config) != CFG_WRITE_OK)
		QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
}

void qt_multitouchscreen_init(void)
{
    touchscreen_config.ctrl = 143;////0x8F; //0x8F;     /* Set ENABLE, RPTEN, DISAMP, DISVECT, SCANEN */

    touchscreen_config.xorigin = 0;
    touchscreen_config.yorigin = 0;

    touchscreen_config.xsize = 17;
    touchscreen_config.ysize = 11;

    touchscreen_config.akscfg = 0;    /* AKS disable */
    touchscreen_config.blen = 32; //0x21

    touchscreen_config.tchthr = 50; //65; //60; //40;//45;

    touchscreen_config.tchdi = 2;
    touchscreen_config.orient = 3;

    touchscreen_config.mrgtimeout = 0;
    touchscreen_config.movhysti = 3;	// 6;
    touchscreen_config.movhystn = 1;	// 5; ///1;

    touchscreen_config.movfilter = 31; //0;// 0x2e;	// 75; //0
    touchscreen_config.numtouch = MAX_USING_FINGER_NUM;	/* 5 (MAX : 10) */

    touchscreen_config.mrghyst = 5;//0;//5;
    touchscreen_config.mrgthr = 20;//0;//40; //20;	//5;

    touchscreen_config.amphyst = 5;//0; //10;
#ifdef LG_FW_TOUCH_SOFT_KEY
    touchscreen_config.xrange = qt602240_pdata->ts_y_scrn_max+65;   // 552 -> 545 20110627 by suker //0.155mm : 1 pixel
#else
    touchscreen_config.xrange = qt602240_pdata->ts_y_scrn_max-1;
#endif
    touchscreen_config.yrange = qt602240_pdata->ts_x_max-1;	//320-1 = 319
    touchscreen_config.xloclip = 0;
    touchscreen_config.xhiclip = 0;
    touchscreen_config.yloclip = 18; //0;
    touchscreen_config.yhiclip = 18; //0;

    touchscreen_config.xedgectrl = 140; //0; //143;
    touchscreen_config.xedgedist = 80; //0; //40;
    touchscreen_config.yedgectrl = 140; //0; //143;
    touchscreen_config.yedgedist = 85; //0; //80;

    touchscreen_config.jumplimit = 0; //18;
	

#ifdef __VER_2_0__
	touchscreen_config.tchhyst = 0;
#endif

    if (write_multitouchscreen_config(0, touchscreen_config) != CFG_WRITE_OK)
		QT_printf("[TSP] qt_multitouchscreen_init Configuration Fail!!! , Line %d \n\r", __LINE__);
}

void qt_keyarray_init(void)
{
    	keyarray_config.ctrl = 0;
	keyarray_config.xorigin = 0;
	keyarray_config.yorigin = 0;
	keyarray_config.xsize = 0;
	keyarray_config.ysize = 0;
	keyarray_config.akscfg = 0;
	keyarray_config.blen = 0;
	keyarray_config.tchthr = 0;
	keyarray_config.tchdi = 0;
    	keyarray_config.reserved[0] = 0;
    	keyarray_config.reserved[1] = 0;
    	
    	if (write_keyarray_config(0, keyarray_config) != CFG_WRITE_OK)
		QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
}

void qt_comcconfig_init(void)
{
    comc_config.ctrl = 0x00;
    comc_config.cmd = NO_COMMAND;
    
    if (get_object_address(SPT_COMCONFIG_T18, 0) != OBJECT_NOT_FOUND) {
		if (write_comc_config(0, comc_config) != CFG_WRITE_OK)
			QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
    }
}


void qt_gpio_pwm_init(void)
{
    gpiopwm_config.ctrl = 0;
    gpiopwm_config.reportmask = 0;
    gpiopwm_config.dir = 0;
    gpiopwm_config.intpullup = 0;
    gpiopwm_config.out = 0;
    gpiopwm_config.wake = 0;
    gpiopwm_config.pwm = 0;
    gpiopwm_config.period = 0;
    gpiopwm_config.duty[0] = 0;
    gpiopwm_config.duty[1] = 0;
    gpiopwm_config.duty[2] = 0;
    gpiopwm_config.duty[3] = 0;
#ifdef __VER_2_0__
    gpiopwm_config.trigger[0] = 0;
    gpiopwm_config.trigger[1] = 0;
    gpiopwm_config.trigger[2] = 0;
    gpiopwm_config.trigger[3] = 0;
#endif
    
    if (write_gpio_config(0, gpiopwm_config) != CFG_WRITE_OK)
		QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
}


void qt_grip_face_suppression_config_init(void)
{

    gripfacesuppression_config.ctrl = 0; //7;		//-> disable PALM bit
    gripfacesuppression_config.xlogrip = 0;
    gripfacesuppression_config.xhigrip = 0;
    gripfacesuppression_config.ylogrip = 0;
    gripfacesuppression_config.yhigrip = 0;
    gripfacesuppression_config.maxtchs = 0;
    gripfacesuppression_config.reserved = 0;
    gripfacesuppression_config.szthr1 = 0; //80;
    gripfacesuppression_config.szthr2 = 0; //40;
    gripfacesuppression_config.shpthr1 = 0; //4;
    gripfacesuppression_config.shpthr2 = 0; //35;

    gripfacesuppression_config.supextto = 0; //10;//5;
    
    /* Write grip suppression config to chip. */
    if (get_object_address(PROCI_GRIPFACESUPPRESSION_T20, 0) != OBJECT_NOT_FOUND) {
		if (write_gripsuppression_config(0, gripfacesuppression_config) != CFG_WRITE_OK)
			QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
    }
}


void qt_noise_suppression_config_init(void)
{

    noise_suppression_config.ctrl = 5; //13; //5;		

    noise_suppression_config.reserved = 0;

    noise_suppression_config.reserved1 = 0;
    noise_suppression_config.gcaful = 0;
    noise_suppression_config.gcafll = 0;

    noise_suppression_config.actvgcafvalid = 3;

	noise_suppression_config.noisethr = 30; //40; //0; //30; 	//35;

	noise_suppression_config.freqhopscale = 0;///1;

    noise_suppression_config.freq[0] = 8; //5; //29; 	//5;	// 6;//10;
    noise_suppression_config.freq[1] = 10; //34;	//15;	// 11;//15;
    noise_suppression_config.freq[2] = 15; //16; //15; //39;	//25;	//16;//20;
    noise_suppression_config.freq[3] = 20; //49;	//35;	// 19;//25;
    noise_suppression_config.freq[4] = 25; //24; //25; //58;	//45;	// 21;//30;
    
    noise_suppression_config.idlegcafvalid = 3;

    /* Write Noise suppression config to chip. */
    if (get_object_address(PROCG_NOISESUPPRESSION_T22, 0) != OBJECT_NOT_FOUND) {
		if (write_noisesuppression_config(0, noise_suppression_config) != CFG_WRITE_OK)
			QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
	}
}


void qt_proximity_config_init(void)
{
    proximity_config.ctrl = 0;
    proximity_config.xorigin = 0;
    proximity_config.xsize = 0;
    proximity_config.ysize = 0;
    proximity_config.reserved_for_future_aks_usage = 0;
    proximity_config.blen = 0;
    proximity_config.tchthr = 0;
    proximity_config.tchdi = 0;
    proximity_config.average = 0;
    proximity_config.rate = 0;
    
    if (get_object_address(TOUCH_PROXIMITY_T23, 0) != OBJECT_NOT_FOUND) {
		if (write_proximity_config(0, proximity_config) != CFG_WRITE_OK)
			QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
    }
}


void qt_one_touch_gesture_config_init(void)
{
    /* Disable one touch gestures. */
    onetouch_gesture_config.ctrl = 0;
    onetouch_gesture_config.numgest = 0;

    onetouch_gesture_config.gesten = 0;
    onetouch_gesture_config.pressproc = 0;
    onetouch_gesture_config.tapto = 0;
    onetouch_gesture_config.flickto = 0;
    onetouch_gesture_config.dragto = 0;
    onetouch_gesture_config.spressto = 0;
    onetouch_gesture_config.lpressto = 0;
    onetouch_gesture_config.reppressto = 0;
    onetouch_gesture_config.flickthr = 0;
    onetouch_gesture_config.dragthr = 0;
    onetouch_gesture_config.tapthr = 0;
    onetouch_gesture_config.throwthr = 0;
 
	if (get_object_address(PROCI_ONETOUCHGESTUREPROCESSOR_T24, 0) != OBJECT_NOT_FOUND) {
		if (write_onetouchgesture_config(0, onetouch_gesture_config) != CFG_WRITE_OK)
			QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
	}
}


void qt_selftest_init(void)
{
    selftest_config.ctrl = 0;
    selftest_config.cmd = 0;
 
    if (get_object_address(SPT_SELFTEST_T25, 0) != OBJECT_NOT_FOUND) {
		if (write_selftest_config(0, selftest_config) != CFG_WRITE_OK)
			QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
	}
}


#ifndef __VER_2_0__
void qt_two_touch_gesture_config_init(void)
{
    /* Disable two touch gestures. */
    twotouch_gesture_config.ctrl = 0;
    twotouch_gesture_config.numgest = 0;

    twotouch_gesture_config.reserved2 = 0;
    twotouch_gesture_config.gesten = 0;
    twotouch_gesture_config.rotatethr = 0;
    twotouch_gesture_config.zoomthr = 0;
 
	if (get_object_address(PROCI_TWOTOUCHGESTUREPROCESSOR_T27, 0) != OBJECT_NOT_FOUND) {
		if (write_twotouchgesture_config(0, twotouch_gesture_config) != CFG_WRITE_OK)
			QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
	}
}
#endif

void qt_cte_config_init(void)
{
	/* Set CTE config */
	cte_config.ctrl = 0; //1;
	cte_config.cmd = 0;	
	cte_config.mode = 3;      /* 19 X by 11 Y extended mode */
	
	cte_config.idlegcafdepth = 32; //16;///4;
	cte_config.actvgcafdepth = 63; //16; //63;	//8;
	
	cte_config.voltage = 15; //0x3c;
	
	/* Write CTE config to chip. */
	if (get_object_address(SPT_CTECONFIG_T28, 0) != OBJECT_NOT_FOUND) {
		if (write_cte_config(cte_config) != CFG_WRITE_OK)
			QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
	}
}

uint8_t reset_chip(void)
{
	uint8_t data = 1u;
	int ret = WRITE_MEM_OK;

	printk(KERN_INFO"\n[TSP][%s] \n", __func__);

	cal_check_flag = 1u;

	ret = write_mem(command_processor_address + RESET_OFFSET, 1, &data);

    return ret;
}

static void hw_reset_chip(void)
{
#ifdef CONFIG_MAINTOUCH_CE
	printk(KERN_INFO"[qt][%s] CE low \n", __func__);
	gpio_set_value(qt602240->ce_gpio, 0);
	mdelay(300);
	printk(KERN_INFO"[qt][%s] CE high \n", __func__);
	gpio_set_value(qt602240->ce_gpio, 1);
	mdelay(2000);
#endif
}


uint8_t calibrate_chip(void)
{
    uint8_t data = 1u;
	int ret = WRITE_MEM_OK;
	uint8_t atchcalst, atchcalsthr;

	printk("\n[TSP]calibrate_chip \n");
	if (tsp_version >= 0x16 && cal_check_flag == 0) {
		/* change calibration suspend settings to zero until calibration confirmed good */
		/* store normal settings */
		atchcalst = acquisition_config.atchcalst;
		atchcalsthr = acquisition_config.atchcalsthr;

		/* resume calibration must be performed with zero settings */
		acquisition_config.atchcalst = 0;
		acquisition_config.atchcalsthr = 0;

		dprintk("\n[TSP][%s] \n", __func__);
		printk("[TSP] reset acq atchcalst=%d, atchcalsthr=%d\n", acquisition_config.atchcalst, acquisition_config.atchcalsthr);

		/* Write temporary acquisition config to chip. */
		if (write_acquisition_config(acquisition_config) != CFG_WRITE_OK) {
			/* "Acquisition config write failed!\n" */
			dprintk("\n[TSP][ERROR] line : %d\n", __LINE__);
			ret = WRITE_MEM_FAILED; /* calling function should retry calibration call */
		}

		/* restore settings to the local structure so that when we confirm the
		* cal is good we can correct them in the chip */
		/* this must be done before returning */
		acquisition_config.atchcalst = atchcalst;
		acquisition_config.atchcalsthr = atchcalsthr;
	}

	/* send calibration command to the chip */
	if (ret == WRITE_MEM_OK) {
		ret = write_mem(command_processor_address + CALIBRATE_OFFSET, 1, &data);

		/* set flag for calibration lockup recovery if cal command was successful */
		if (ret == WRITE_MEM_OK)
			/* set flag to show we must still confirm if calibration was good or bad */
			cal_check_flag = 1u;
	}

	return ret;
}


uint8_t backup_config(void)
{
    /* Write 0x55 to BACKUPNV register to initiate the backup. */
    uint8_t data = 0x55u;
	int ret = WRITE_MEM_OK;

	ret = write_mem(command_processor_address + BACKUP_OFFSET, 1, &data);
    return ret;
}


uint8_t get_version(uint8_t *version)
{
    if (info_block)
		*version = info_block->info_id.version;
    else
		return ID_DATA_NOT_AVAILABLE;

    return ID_DATA_OK;
}


uint8_t get_family_id(uint8_t *family_id)
{
    if (info_block)
		*family_id = info_block->info_id.family_id;
    else
		return ID_DATA_NOT_AVAILABLE;

    return ID_DATA_OK;
}


uint8_t get_build_number(uint8_t *build)
{
    if (info_block)
		*build = info_block->info_id.build;
    else
		return ID_DATA_NOT_AVAILABLE;

    return ID_DATA_OK;
}


uint8_t get_variant_id(uint8_t *variant)
{
    if (info_block)
		*variant = info_block->info_id.variant_id;
	else
		return ID_DATA_NOT_AVAILABLE;

    return ID_DATA_OK;
}


uint8_t write_command_config(gen_commandprocessor_t6_config_t cfg)
{
	int ret = WRITE_MEM_OK;

	ret = write_simple_config(GEN_COMMANDPROCESSOR_T6, 0, (void *) &cfg);

    return ret;
}


uint8_t write_power_config(gen_powerconfig_t7_config_t cfg)
{
	int ret = WRITE_MEM_OK;

	ret = write_simple_config(GEN_POWERCONFIG_T7, 0, (void *) &cfg);

    return ret;
}


uint8_t write_acquisition_config(gen_acquisitionconfig_t8_config_t cfg)
{
	int ret = WRITE_MEM_OK;

	ret = write_simple_config(GEN_ACQUISITIONCONFIG_T8, 0, (void *) &cfg);

	return ret;
}


uint8_t write_multitouchscreen_config(uint8_t instance, touch_multitouchscreen_t9_config_t cfg)
{
    uint16_t object_address;
    uint8_t status;
    uint8_t object_size;
	uint8_t *tmp = NULL;

    object_size = get_object_size(TOUCH_MULTITOUCHSCREEN_T9);
    if (object_size == 0)
		return CFG_WRITE_FAILED;

    tmp = kmalloc(object_size, GFP_KERNEL | GFP_ATOMIC);

    if (tmp == NULL)
		return CFG_WRITE_FAILED;

    memset(tmp, 0, object_size);


    /* 18 elements at beginning are 1 byte. */
    memcpy(tmp, &cfg, 18);

    /* Next two are 2 bytes. */
    *(tmp + 18) = (uint8_t) (cfg.xrange &  0xFF);
    *(tmp + 19) = (uint8_t) (cfg.xrange >> 8);

    *(tmp + 20) = (uint8_t) (cfg.yrange &  0xFF);
    *(tmp + 21) = (uint8_t) (cfg.yrange >> 8);

    /* And the last 4(8) 1 bytes each again. */
    *(tmp + 22) = cfg.xloclip;
    *(tmp + 23) = cfg.xhiclip;
    *(tmp + 24) = cfg.yloclip;
    *(tmp + 25) = cfg.yhiclip;

//#if defined(__VER_1_4__)
    *(tmp + 26) = cfg.xedgectrl;
    *(tmp + 27) = cfg.xedgedist;
    *(tmp + 28) = cfg.yedgectrl;
    *(tmp + 29) = cfg.yedgedist;
//#endif
	if (tsp_version >= 0x16)
		*(tmp + 30) = cfg.jumplimit;

#ifdef __VER_2_0__
	*(tmp + 31) = cfg.tchhyst;
#endif

    object_address = get_object_address(TOUCH_MULTITOUCHSCREEN_T9, instance);

    if (object_address == 0) {
		kfree(tmp);
		return CFG_WRITE_FAILED;
    }

    status = write_mem(object_address, object_size, tmp);

    kfree(tmp);

    return status;
}


uint8_t write_keyarray_config(uint8_t instance, touch_keyarray_t15_config_t cfg)
{
    return write_simple_config(TOUCH_KEYARRAY_T15, instance, (void *) &cfg);
}


uint8_t write_comc_config(uint8_t instance, spt_comcconfig_t18_config_t cfg)
{
    return write_simple_config(SPT_COMCONFIG_T18, instance, (void *) &cfg);
}


uint8_t write_gpio_config(uint8_t instance, spt_gpiopwm_t19_config_t cfg)
{
    return write_simple_config(SPT_GPIOPWM_T19, instance, (void *) &cfg);
}


uint8_t write_gripsuppression_config(uint8_t instance, proci_gripfacesuppression_t20_config_t cfg)
{
    return write_simple_config(PROCI_GRIPFACESUPPRESSION_T20, instance, (void *) &cfg);
}


uint8_t write_noisesuppression_config(uint8_t instance, procg_noisesuppression_t22_config_t cfg)
{
    uint16_t object_address;
    uint8_t status;
    uint8_t object_size;
	uint8_t *tmp = NULL;

    object_size = get_object_size(PROCG_NOISESUPPRESSION_T22);
    if (object_size == 0)
		return CFG_WRITE_FAILED;

    tmp = kmalloc(object_size, GFP_KERNEL | GFP_ATOMIC);

    if (tmp == NULL)
		return CFG_WRITE_FAILED;

    memset(tmp, 0, object_size);

    /* 18 elements at beginning are 1 byte. */
    memcpy(tmp, &cfg, 3);

    /* Next two are 2 bytes. */
    *(tmp + 3) = (uint8_t) (cfg.gcaful & 0xFF);
    *(tmp + 4) = (uint8_t) (cfg.gcaful >> 8);

    *(tmp + 5) = (uint8_t) (cfg.gcafll & 0xFF);
    *(tmp + 6) = (uint8_t) (cfg.gcafll >> 8);

     memcpy((tmp+7), &cfg.actvgcafvalid, 10);

    object_address = get_object_address(PROCG_NOISESUPPRESSION_T22, instance);

    if (object_address == 0) {
		kfree(tmp);
		return CFG_WRITE_FAILED;
    }

    status = write_mem(object_address, object_size, tmp);

    kfree(tmp);

    return status;
}


uint8_t write_proximity_config(uint8_t instance, touch_proximity_t23_config_t cfg)
{
    uint16_t object_address;
    uint8_t status;
    uint8_t object_size;
	uint8_t *tmp = NULL;

    object_size = get_object_size(TOUCH_PROXIMITY_T23);
    if (object_size == 0)
		return CFG_WRITE_FAILED;

    tmp = kmalloc(object_size, GFP_KERNEL | GFP_ATOMIC);

    if (tmp == NULL)
		return CFG_WRITE_FAILED;

    memset(tmp, 0, object_size);

    *(tmp + 0) = cfg.ctrl;
    *(tmp + 1) = cfg.xorigin;
    *(tmp + 2) = cfg.yorigin;
    *(tmp + 3) = cfg.xsize;
    *(tmp + 4) = cfg.ysize;
    *(tmp + 5) = cfg.reserved_for_future_aks_usage;
    *(tmp + 6) = cfg.blen;

    *(tmp + 7) = (uint8_t) (cfg.tchthr & 0x00FF);
    *(tmp + 8) = (uint8_t) (cfg.tchthr >> 8);

    *(tmp + 9) = cfg.tchdi;
    *(tmp + 10) = cfg.average;

    *(tmp + 11) = (uint8_t) (cfg.rate & 0x00FF);
    *(tmp + 12) = (uint8_t) (cfg.rate >> 8);

    object_address = get_object_address(TOUCH_PROXIMITY_T23, instance);

    if (object_address == 0)
		return CFG_WRITE_FAILED;

	status = write_mem(object_address, object_size, tmp);

	kfree(tmp);

	return status;
}


uint8_t write_onetouchgesture_config(uint8_t instance, proci_onetouchgestureprocessor_t24_config_t cfg)
{

    uint16_t object_address;
    uint8_t status;
    uint8_t object_size;
	uint8_t *tmp = NULL;

    object_size = get_object_size(PROCI_ONETOUCHGESTUREPROCESSOR_T24);
    if (object_size == 0)
		return CFG_WRITE_FAILED;

    tmp = kmalloc(object_size, GFP_KERNEL | GFP_ATOMIC);

    if (tmp == NULL)
		return CFG_WRITE_FAILED;

    memset(tmp, 0, object_size);

    *(tmp + 0) = cfg.ctrl;
    *(tmp + 1) = cfg.numgest;
    *(tmp + 2) = (uint8_t) (cfg.gesten & 0x00FF);
    *(tmp + 3) = (uint8_t) (cfg.gesten >> 8);

    memcpy((tmp+4), &cfg.pressproc, 7);

    *(tmp + 11) = (uint8_t) (cfg.flickthr & 0x00FF);
    *(tmp + 12) = (uint8_t) (cfg.flickthr >> 8);

    *(tmp + 13) = (uint8_t) (cfg.dragthr & 0x00FF);
    *(tmp + 14) = (uint8_t) (cfg.dragthr >> 8);

    *(tmp + 15) = (uint8_t) (cfg.tapthr & 0x00FF);
    *(tmp + 16) = (uint8_t) (cfg.tapthr >> 8);

    *(tmp + 17) = (uint8_t) (cfg.throwthr & 0x00FF);
    *(tmp + 18) = (uint8_t) (cfg.throwthr >> 8);

    object_address = get_object_address(PROCI_ONETOUCHGESTUREPROCESSOR_T24, instance);

    if (object_address == 0)
		return CFG_WRITE_FAILED;

	status = write_mem(object_address, object_size, tmp);

	kfree(tmp);

	return status;
}


uint8_t write_selftest_config(uint8_t instance, spt_selftest_t25_config_t cfg)
{
    uint16_t object_address;
    uint8_t status;
    uint8_t object_size;
	uint8_t *tmp = NULL;

    object_size = get_object_size(SPT_SELFTEST_T25);

    if (object_size == 0)
		return CFG_WRITE_FAILED;

	tmp = kmalloc(object_size, GFP_KERNEL | GFP_ATOMIC);

    if (tmp == NULL)
		return CFG_WRITE_FAILED;

    memset(tmp, 0, object_size);

    *(tmp + 0) = cfg.ctrl;
    *(tmp + 1) = cfg.cmd;

    object_address = get_object_address(SPT_SELFTEST_T25, instance);

    if (object_address == 0)
		return CFG_WRITE_FAILED;

    status = write_mem(object_address, object_size, tmp);

    kfree(tmp);
    return status;
}

#ifndef __VER_2_0__
uint8_t write_twotouchgesture_config(uint8_t instance, proci_twotouchgestureprocessor_t27_config_t cfg)
{
    uint16_t object_address;
    uint8_t status;
    uint8_t object_size;
	uint8_t *tmp = NULL;

    object_size = get_object_size(PROCI_TWOTOUCHGESTUREPROCESSOR_T27);
    if (object_size == 0)
		return CFG_WRITE_FAILED;

    tmp = kmalloc(object_size, GFP_KERNEL | GFP_ATOMIC);

    if (tmp == NULL)
		return CFG_WRITE_FAILED;

    memset(tmp, 0, object_size);

    *(tmp + 0) = cfg.ctrl;
    *(tmp + 1) = cfg.numgest;
    *(tmp + 2) = 0;
    *(tmp + 3) = cfg.gesten;
    *(tmp + 4) = cfg.rotatethr;
    *(tmp + 5) = (uint8_t) (cfg.zoomthr & 0x00FF);
    *(tmp + 6) = (uint8_t) (cfg.zoomthr >> 8);

    object_address = get_object_address(PROCI_TWOTOUCHGESTUREPROCESSOR_T27, instance);

    if (object_address == 0)
		return CFG_WRITE_FAILED;

    status = write_mem(object_address, object_size, tmp);

    kfree(tmp);

    return status;
}
#endif

uint8_t write_cte_config(spt_cteconfig_t28_config_t cfg)
{
    return write_simple_config(SPT_CTECONFIG_T28, 0, (void *) &cfg);
}


uint8_t write_simple_config(uint8_t object_type, uint8_t instance, void *cfg)
{
    uint16_t object_address;
    uint8_t object_size;

    object_address = get_object_address(object_type, instance);
    object_size = get_object_size(object_type);

    if ((object_size == 0) || (object_address == 0))
		return CFG_WRITE_FAILED;

    return write_mem(object_address, object_size, cfg);
}


uint8_t get_object_size(uint8_t object_type)
{
    uint8_t object_table_index = 0;
    uint8_t object_found = 0;
    uint16_t size = OBJECT_NOT_FOUND;

    object_t *object_table;
    object_t obj;

    object_table = info_block->objects;

    while ((object_table_index < info_block->info_id.num_declared_objects) && !object_found) {
		obj = object_table[object_table_index];
		/* Does object type match? */
		if (obj.object_type == object_type) {
			object_found = 1;
			size = obj.size + 1;
		}
		object_table_index++;
    }

    return size;
}


uint8_t report_id_to_type(uint8_t report_id, uint8_t *instance)
{
	if (report_id <= max_report_id) {
		*instance = report_id_map[report_id].instance;
		return report_id_map[report_id].object_type;
    }

	return ID_MAPPING_FAILED;
}


uint8_t read_id_block(info_id_t *id)
{
    uint8_t status;

    status = read_mem(0, 1, (void *) &id->family_id);
    if (status != READ_MEM_OK)
		return status;

    status = read_mem(1, 1, (void *) &id->variant_id);
    if (status != READ_MEM_OK)
		return status;

    status = read_mem(2, 1, (void *) &id->version);
    if (status != READ_MEM_OK)
		return status;

    status = read_mem(3, 1, (void *) &id->build);
    if (status != READ_MEM_OK)
		return status;

    status = read_mem(4, 1, (void *) &id->matrix_x_size);
    if (status != READ_MEM_OK)
		return status;

    status = read_mem(5, 1, (void *) &id->matrix_y_size);
    if (status != READ_MEM_OK)
		return status;

    status = read_mem(6, 1, (void *) &id->num_declared_objects);

    return status;
}


uint8_t get_max_report_id()
{
	return max_report_id;
}


uint16_t get_object_address(uint8_t object_type, uint8_t instance)
{
    uint8_t object_table_index = 0;
    uint8_t address_found = 0;
    uint16_t address = OBJECT_NOT_FOUND;

    object_t *object_table;
    object_t obj;
    object_table = info_block->objects;
    while ((object_table_index < info_block->info_id.num_declared_objects) && !address_found) {
		obj = object_table[object_table_index];

		/* Does object type match? */
		if (obj.object_type == object_type) {
			address_found = 1;

			/* Are there enough instances defined in the FW? */
			if (obj.instances >= instance)
				address = obj.i2c_address + (obj.size + 1) * instance;
		}
		object_table_index++;
	}

	return address;
}


uint32_t get_stored_infoblock_crc()
{
    uint32_t crc;
    crc = (uint32_t) (((uint32_t) info_block->CRC_hi) << 16);
    crc = crc | info_block->CRC;
    return crc;
}


uint8_t calculate_infoblock_crc(uint32_t *crc_pointer)
{
    uint32_t crc = 0;
    uint16_t crc_area_size;

    uint8_t i;

    uint8_t status;
	uint8_t *mem = NULL;

    /* 7 bytes of version data, 6 * NUM_OF_OBJECTS bytes of object table. */
    crc_area_size = 7 + info_block->info_id.num_declared_objects * 6;

    mem = kmalloc(crc_area_size, GFP_KERNEL | GFP_ATOMIC);

    if (mem == NULL)
		return CRC_CALCULATION_FAILED;

    status = read_mem(0, crc_area_size, mem);

    if (status != READ_MEM_OK)
		return CRC_CALCULATION_FAILED;

    i = 0;
    while (i < (crc_area_size - 1)) {
		crc = CRC_24(crc, *(mem + i), *(mem + i + 1));
		i += 2;
    }

    crc = CRC_24(crc, *(mem + i), 0);

    kfree(mem);

    /* Return only 24 bit CRC. */
    *crc_pointer = (crc & 0x00FFFFFF);
    return CRC_CALCULATION_OK;
}


uint32_t CRC_24(uint32_t crc, uint8_t byte1, uint8_t byte2)
{
    static const uint32_t crcpoly = 0x80001B;
    uint32_t result;
    uint16_t data_word;

    data_word = (uint16_t) ((uint16_t) (byte2 << 8u) | byte1);
    result = ((crc << 1u) ^ (uint32_t) data_word);

    if (result & 0x1000000)
		result ^= crcpoly;

    return result;
}


/*------------------------------ main block -----------------------------------*/
int quantum_touch_probe(void)
{
	struct input_dev *input_dev = qt602240->input_dev;
	U8 report_id;
	U8 object_type, instance;
	U32 stored_crc;
	U8 family_id, variant, build;
	uint32_t crc;

	max_report_id = 0;
	
	DEBUG_M(KERN_DEBUG "[TSP_FFCHK] quantum_touch_probe\n");
	
	if (init_touch_driver(QT602240_I2C_ADDR_A, QT602240_I2C_ADDR_B) == DRIVER_SETUP_OK) {
		/* "\nTouch device found\n" */
		printk(KERN_INFO"[TSP] Touch device found\n");
	} else{
		/* "\nTouch device NOT found\n" */
		printk(KERN_INFO"[TSP][ERROR] Touch device NOT found\n");
		return (-1);
	}

	mutex_init(&input_dev->mutex);
	
	/* Get and show the version information. */
	get_family_id(&family_id);
	get_variant_id(&variant);
	get_version(&tsp_version);
	get_build_number(&build);

	QT_printf("Version: 	   0x%x\n\r", tsp_version);
	QT_printf("Family:		   0x%x\n\r", family_id);
	QT_printf("Variant: 	   0x%x\n\r", variant);
	QT_printf("Build number:   0x%x\n\r", build);

	QT_printf("Matrix X size : %d\n\r", info_block->info_id.matrix_x_size);
	QT_printf("Matrix Y size : %d\n\r", info_block->info_id.matrix_y_size);

	if (calculate_infoblock_crc(&crc) != CRC_CALCULATION_OK) {
		QT_printf("Calculating CRC failed, skipping check!\n\r");
	} else {
		QT_printf("Calculated CRC:\t");
		write_message_to_usart((uint8_t *) &crc, 4);
		QT_printf("\n\r");
	}

	stored_crc = get_stored_infoblock_crc();
	QT_printf("Stored CRC:\t");
	write_message_to_usart((uint8_t *) &stored_crc, 4);
	QT_printf("\n\r");

	if (stored_crc != crc)
		QT_printf("Warning: info block CRC value doesn't match the calculated!\n\r");
	else
		QT_printf("Info block CRC value OK.\n\n\r");

	/* Test the report id to object type / instance mapping: get the maximum
	* report id and print the report id map. */

	max_report_id = get_max_report_id();
	for (report_id = 0; report_id <= max_report_id; report_id++)
		object_type = report_id_to_type(report_id, &instance);

	//inittailzing
	memset(readConfig, 0x00, 16*sizeof(struct __reg_i2c_read_config));

	qt_power_config_init();

#ifdef OPTION_WRITE_CONFIG
	qt_acquisition_config_init();

	qt_multitouchscreen_init();

	qt_keyarray_init();

	qt_comcconfig_init();

	qt_gpio_pwm_init();

	qt_grip_face_suppression_config_init();

	qt_noise_suppression_config_init();

	qt_proximity_config_init();

	qt_one_touch_gesture_config_init();

	qt_selftest_init();

#ifndef __VER_2_0__
	qt_two_touch_gesture_config_init();
#endif

	qt_cte_config_init();

	/* Backup settings to NVM. */
	if (backup_config() != WRITE_MEM_OK) {
		QT_printf("Failed to backup, exiting...\n\r");
		return (-2);
	} else {
		/* QT_printf("Backed up the config to non-volatile memory!\n\r"); */
	}

#else
		QT_printf("Chip setup sequence was bypassed!\n\r");
#endif /* OPTION_WRITE_CONFIG */

	//save init values for show reg values by adb shell
	qt_saving_regs_values();
	
	/* reset the touch IC. */
	if (reset_chip() != WRITE_MEM_OK) {
		QT_printf("Failed to reset, exiting...\n\r");
		return (-3);
	} else {		
		QT_printf("Chip reset OK!\n\r");
		msleep(100);
	}

	return 0;
}

static void qt_saving_regs_values(void)
{
    read_config_array_count = 0;
	
    //power_config
    strcpy(readConfig[read_config_array_count].table_item_name, "power_config");
    readConfig[read_config_array_count].table_item_value[0] = power_config.idleacqint;
    readConfig[read_config_array_count].table_item_value[1] = power_config.actvacqint;
    readConfig[read_config_array_count].table_item_value[2] = power_config.actv2idleto;
    readConfig[read_config_array_count].table_item_value[3] = 0xEF;
    read_config_array_count++; //1
    
    //acquisition_config
    strcpy(readConfig[read_config_array_count].table_item_name, "acquisition_config");
    readConfig[read_config_array_count].table_item_value[0] = acquisition_config.chrgtime;
    readConfig[read_config_array_count].table_item_value[1] = acquisition_config.reserved;
    readConfig[read_config_array_count].table_item_value[2] = acquisition_config.tchdrift;
    readConfig[read_config_array_count].table_item_value[3] = acquisition_config.driftst;
    readConfig[read_config_array_count].table_item_value[4] = acquisition_config.tchautocal;
    readConfig[read_config_array_count].table_item_value[5] = acquisition_config.sync;
    readConfig[read_config_array_count].table_item_value[6] = acquisition_config.atchcalst;
    readConfig[read_config_array_count].table_item_value[7] = acquisition_config.atchcalsthr;    
#ifdef __VER_2_0__    							
    readConfig[read_config_array_count].table_item_value[8] = acquisition_config.atchfrccalthr;
    readConfig[read_config_array_count].table_item_value[9] = acquisition_config.atchfrccalratio;
    readConfig[read_config_array_count].table_item_value[10] = 0xEF;
#else
    readConfig[read_config_array_count].table_item_value[8] = 0xEF;    
#endif
    read_config_array_count++;	//2

    //touchscreen_config1
    strcpy(readConfig[read_config_array_count].table_item_name, "touchscreen_config1");
    readConfig[read_config_array_count].table_item_value[0] = touchscreen_config.ctrl;
    readConfig[read_config_array_count].table_item_value[1] = touchscreen_config.xorigin;
    readConfig[read_config_array_count].table_item_value[2] = touchscreen_config.yorigin;
    readConfig[read_config_array_count].table_item_value[3] = touchscreen_config.xsize;
    readConfig[read_config_array_count].table_item_value[4] = touchscreen_config.ysize;
    readConfig[read_config_array_count].table_item_value[5] = touchscreen_config.akscfg;
    readConfig[read_config_array_count].table_item_value[6] = touchscreen_config.blen;
    readConfig[read_config_array_count].table_item_value[7] = touchscreen_config.tchthr;
    readConfig[read_config_array_count].table_item_value[8] = touchscreen_config.tchdi;
    readConfig[read_config_array_count].table_item_value[9] = touchscreen_config.orient;
    readConfig[read_config_array_count].table_item_value[10] = touchscreen_config.mrgtimeout;
    readConfig[read_config_array_count].table_item_value[11] = touchscreen_config.movhysti;
    readConfig[read_config_array_count].table_item_value[12] = touchscreen_config.movhystn;
    readConfig[read_config_array_count].table_item_value[13] = touchscreen_config.movfilter;
    readConfig[read_config_array_count].table_item_value[14] = touchscreen_config.numtouch;
    readConfig[read_config_array_count].table_item_value[15] = touchscreen_config.mrghyst;
    read_config_array_count++;	//3
    
    //touchscreen_config2
    strcpy(readConfig[read_config_array_count].table_item_name, "touchscreen_config2");
    readConfig[read_config_array_count].table_item_value[0] = touchscreen_config.mrgthr;
    readConfig[read_config_array_count].table_item_value[1] = touchscreen_config.amphyst;
    readConfig[read_config_array_count].table_item_value[2] = touchscreen_config.xrange;
    readConfig[read_config_array_count].table_item_value[3] = touchscreen_config.yrange;
    readConfig[read_config_array_count].table_item_value[4] = touchscreen_config.xloclip;
    readConfig[read_config_array_count].table_item_value[5] = touchscreen_config.xhiclip;
    readConfig[read_config_array_count].table_item_value[6] = touchscreen_config.yloclip;
    readConfig[read_config_array_count].table_item_value[7] = touchscreen_config.yhiclip;
    readConfig[read_config_array_count].table_item_value[8] = touchscreen_config.xedgectrl;
    readConfig[read_config_array_count].table_item_value[9] = touchscreen_config.xedgedist;
    readConfig[read_config_array_count].table_item_value[10] = touchscreen_config.yedgectrl;
    readConfig[read_config_array_count].table_item_value[11] = touchscreen_config.yedgedist;
    readConfig[read_config_array_count].table_item_value[12] = touchscreen_config.jumplimit;    
#ifdef __VER_2_0__
    readConfig[read_config_array_count].table_item_value[13] = touchscreen_config.tchhyst;
    readConfig[read_config_array_count].table_item_value[14] = 0xEF;
#else
    readConfig[read_config_array_count].table_item_value[13] = 0xEF;     
#endif
    read_config_array_count++;	//4
    
    //keyarray_config
    strcpy(readConfig[read_config_array_count].table_item_name, "keyarray_config");
    readConfig[read_config_array_count].table_item_value[0] = keyarray_config.ctrl;
    readConfig[read_config_array_count].table_item_value[1] = keyarray_config.xorigin;
    readConfig[read_config_array_count].table_item_value[2] = keyarray_config.yorigin;
    readConfig[read_config_array_count].table_item_value[3] = keyarray_config.xsize;
    readConfig[read_config_array_count].table_item_value[4] = keyarray_config.ysize;
    readConfig[read_config_array_count].table_item_value[5] = keyarray_config.akscfg;
    readConfig[read_config_array_count].table_item_value[6] = keyarray_config.blen;
    readConfig[read_config_array_count].table_item_value[7] = keyarray_config.tchthr;
    readConfig[read_config_array_count].table_item_value[8] = keyarray_config.tchdi;
    readConfig[read_config_array_count].table_item_value[9] = keyarray_config.reserved[0];
    readConfig[read_config_array_count].table_item_value[10] = keyarray_config.reserved[1];
    readConfig[read_config_array_count].table_item_value[11] = 0xEF;
    read_config_array_count++;	//5
    
    //comc_config
    strcpy(readConfig[read_config_array_count].table_item_name, "comc_config");
    readConfig[read_config_array_count].table_item_value[0] = comc_config.ctrl;
    readConfig[read_config_array_count].table_item_value[1] = comc_config.cmd;
    readConfig[read_config_array_count].table_item_value[2] = 0xEF;
    read_config_array_count++;	//6
    
    //gpiopwm_config
    strcpy(readConfig[read_config_array_count].table_item_name, "gpiopwm_config");
    readConfig[read_config_array_count].table_item_value[0] = gpiopwm_config.ctrl;
    readConfig[read_config_array_count].table_item_value[1] = gpiopwm_config.reportmask;
    readConfig[read_config_array_count].table_item_value[2] = gpiopwm_config.intpullup;
    readConfig[read_config_array_count].table_item_value[3] = gpiopwm_config.out;
    readConfig[read_config_array_count].table_item_value[4] = gpiopwm_config.wake;
    readConfig[read_config_array_count].table_item_value[5] = gpiopwm_config.pwm;
    readConfig[read_config_array_count].table_item_value[6] = gpiopwm_config.period;
    readConfig[read_config_array_count].table_item_value[7] = gpiopwm_config.duty[0];
    readConfig[read_config_array_count].table_item_value[8] = gpiopwm_config.duty[1];
    readConfig[read_config_array_count].table_item_value[9] = gpiopwm_config.duty[2];
    readConfig[read_config_array_count].table_item_value[10] = gpiopwm_config.duty[3];
#ifdef __VER_2_0__  
    readConfig[read_config_array_count].table_item_value[11] = gpiopwm_config.trigger[0];
    readConfig[read_config_array_count].table_item_value[12] = gpiopwm_config.trigger[1];
    readConfig[read_config_array_count].table_item_value[13] = gpiopwm_config.trigger[2];
    readConfig[read_config_array_count].table_item_value[14] = gpiopwm_config.trigger[3];
    readConfig[read_config_array_count].table_item_value[15] = 0xEF;
#else
    readConfig[read_config_array_count].table_item_value[11] = 0xEF;
#endif
    read_config_array_count++;	//7
    
    //gripfacesuppression_config
    strcpy(readConfig[read_config_array_count].table_item_name, "gripfacesuppression_config");
    readConfig[read_config_array_count].table_item_value[0] = gripfacesuppression_config.ctrl;
    readConfig[read_config_array_count].table_item_value[1] = gripfacesuppression_config.xlogrip;
    readConfig[read_config_array_count].table_item_value[2] = gripfacesuppression_config.xhigrip;
    readConfig[read_config_array_count].table_item_value[3] = gripfacesuppression_config.ylogrip;
    readConfig[read_config_array_count].table_item_value[4] = gripfacesuppression_config.yhigrip;
    readConfig[read_config_array_count].table_item_value[5] = gripfacesuppression_config.maxtchs;
    readConfig[read_config_array_count].table_item_value[6] = gripfacesuppression_config.reserved;
    readConfig[read_config_array_count].table_item_value[7] = gripfacesuppression_config.szthr1;
    readConfig[read_config_array_count].table_item_value[8] = gripfacesuppression_config.szthr2;
    readConfig[read_config_array_count].table_item_value[9] = gripfacesuppression_config.shpthr1;
    readConfig[read_config_array_count].table_item_value[10] = gripfacesuppression_config.shpthr2;
    readConfig[read_config_array_count].table_item_value[11] = gripfacesuppression_config.supextto;
    readConfig[read_config_array_count].table_item_value[12] = 0xEF;
    read_config_array_count++;	//8
    
    //noise_suppression_config
    strcpy(readConfig[read_config_array_count].table_item_name, "noise_suppression_config");
    readConfig[read_config_array_count].table_item_value[0] = noise_suppression_config.ctrl;
    readConfig[read_config_array_count].table_item_value[1] = noise_suppression_config.reserved;
    readConfig[read_config_array_count].table_item_value[2] = noise_suppression_config.reserved1;
    readConfig[read_config_array_count].table_item_value[3] = noise_suppression_config.gcaful;
    readConfig[read_config_array_count].table_item_value[4] = noise_suppression_config.gcafll;
    readConfig[read_config_array_count].table_item_value[5] = noise_suppression_config.actvgcafvalid;
    readConfig[read_config_array_count].table_item_value[6] = noise_suppression_config.noisethr;
    readConfig[read_config_array_count].table_item_value[7] = noise_suppression_config.freqhopscale;
    readConfig[read_config_array_count].table_item_value[8] = noise_suppression_config.freq[0];
    readConfig[read_config_array_count].table_item_value[9] = noise_suppression_config.freq[1];
    readConfig[read_config_array_count].table_item_value[10] = noise_suppression_config.freq[2];
    readConfig[read_config_array_count].table_item_value[11] = noise_suppression_config.freq[3];
    readConfig[read_config_array_count].table_item_value[12] = noise_suppression_config.freq[4];
    readConfig[read_config_array_count].table_item_value[13] = noise_suppression_config.idlegcafvalid;
    readConfig[read_config_array_count].table_item_value[14] = 0xEF;
    read_config_array_count++;	//9
    
    //proximity_config
    strcpy(readConfig[read_config_array_count].table_item_name, "proximity_config");
    readConfig[read_config_array_count].table_item_value[0] = proximity_config.ctrl;
    readConfig[read_config_array_count].table_item_value[1] = proximity_config.xorigin;
    readConfig[read_config_array_count].table_item_value[2] = proximity_config.xsize;
    readConfig[read_config_array_count].table_item_value[3] = proximity_config.ysize;
    readConfig[read_config_array_count].table_item_value[4] = proximity_config.reserved_for_future_aks_usage;
    readConfig[read_config_array_count].table_item_value[5] = proximity_config.blen;
    readConfig[read_config_array_count].table_item_value[6] = proximity_config.tchthr;
    readConfig[read_config_array_count].table_item_value[7] = proximity_config.tchdi;
    readConfig[read_config_array_count].table_item_value[8] = proximity_config.average;
    readConfig[read_config_array_count].table_item_value[9] = proximity_config.rate;
    readConfig[read_config_array_count].table_item_value[10] = 0xEF;
    read_config_array_count++;	//10
    
    //onetouch_gesture_config
    strcpy(readConfig[read_config_array_count].table_item_name, "onetouch_gesture_config");
    readConfig[read_config_array_count].table_item_value[0] = onetouch_gesture_config.ctrl;
    readConfig[read_config_array_count].table_item_value[1] = onetouch_gesture_config.numgest;
    readConfig[read_config_array_count].table_item_value[2] = onetouch_gesture_config.gesten;
    readConfig[read_config_array_count].table_item_value[3] = onetouch_gesture_config.pressproc;
    readConfig[read_config_array_count].table_item_value[4] = onetouch_gesture_config.tapto;
    readConfig[read_config_array_count].table_item_value[5] = onetouch_gesture_config.flickto;
    readConfig[read_config_array_count].table_item_value[6] = onetouch_gesture_config.dragto;
    readConfig[read_config_array_count].table_item_value[7] = onetouch_gesture_config.spressto;
    readConfig[read_config_array_count].table_item_value[8] = onetouch_gesture_config.lpressto;
    readConfig[read_config_array_count].table_item_value[9] = onetouch_gesture_config.reppressto;
    readConfig[read_config_array_count].table_item_value[10] = onetouch_gesture_config.flickthr;
    readConfig[read_config_array_count].table_item_value[11] = onetouch_gesture_config.dragthr;
    readConfig[read_config_array_count].table_item_value[12] = onetouch_gesture_config.tapthr;
    readConfig[read_config_array_count].table_item_value[13] = onetouch_gesture_config.throwthr;
    readConfig[read_config_array_count].table_item_value[14] = 0xEF;        
    read_config_array_count++;	//11
    
    //selftest_config
    strcpy(readConfig[read_config_array_count].table_item_name, "selftest_config");
    readConfig[read_config_array_count].table_item_value[0] = selftest_config.ctrl;
    readConfig[read_config_array_count].table_item_value[1] = selftest_config.cmd;
    readConfig[read_config_array_count].table_item_value[2] = 0xEF;
    read_config_array_count++;	//12
    
#ifndef __VER_2_0__    
    //twotouch_gesture_config
    strcpy(readConfig[read_config_array_count].table_item_name, "twotouch_gesture_config");
    readConfig[read_config_array_count].table_item_value[0] = twotouch_gesture_config.ctrl;
    readConfig[read_config_array_count].table_item_value[1] = twotouch_gesture_config.numgest;
    readConfig[read_config_array_count].table_item_value[2] = twotouch_gesture_config.reserved2;
    readConfig[read_config_array_count].table_item_value[3] = twotouch_gesture_config.gesten;
    readConfig[read_config_array_count].table_item_value[4] = twotouch_gesture_config.rotatethr;
    readConfig[read_config_array_count].table_item_value[5] = twotouch_gesture_config.zoomthr;
    readConfig[read_config_array_count].table_item_value[6] = 0xEF;
    read_config_array_count++;	//13
#endif
    
    //cte_config
    strcpy(readConfig[read_config_array_count].table_item_name, "cte_config");
    readConfig[read_config_array_count].table_item_value[0] = cte_config.ctrl;
    readConfig[read_config_array_count].table_item_value[1] = cte_config.cmd;
    readConfig[read_config_array_count].table_item_value[2] = cte_config.mode;
    readConfig[read_config_array_count].table_item_value[3] = cte_config.idlegcafdepth;
    readConfig[read_config_array_count].table_item_value[4] = cte_config.actvgcafdepth;
    readConfig[read_config_array_count].table_item_value[5] = cte_config.voltage;
    readConfig[read_config_array_count].table_item_value[6] = 0xEF;
    read_config_array_count++;	//14
}

/*------------------------------ Sub functions -----------------------------------*/
/*!
  \brief Initializes touch driver.

  This function initializes the touch driver: tries to connect to given
  address, sets the message handler pointer, reads the info block and object
  table, sets the message processor address etc.

  @param I2C_address is the address where to connect.
  @param (*handler) is a pointer to message handler function.
  @return DRIVER_SETUP_OK if init successful, DRIVER_SETUP_FAILED
  otherwise.
  */

uint8_t touch_i2c_address_probe(uint8_t i2c_address_a, uint8_t i2c_address_b)
{
    uint8_t status;
    uint8_t buf;

	qt602240->client->addr = i2c_address_a;;
	
	DEBUG_M(KERN_DEBUG "[TSP_FFCHK] touch_i2c_address_probe\n");
	printk(KERN_INFO"[QT] %s 0x%02x probe \n", __func__, i2c_address_a);

    status = read_mem(0, 1, (void *) &buf);
    if (status != READ_MEM_OK) {
		printk(KERN_INFO"[QT] %s 0x%02x probe fail\n", __func__, i2c_address_a);
		printk(KERN_INFO"[QT] %s 0x%02x probe\n", __func__, i2c_address_b);
		qt602240->client->addr = i2c_address_b;;
	    status = read_mem(0, 1, (void *) &buf);
	    if (status != READ_MEM_OK)
			printk(KERN_INFO"[QT] %s 0x%02x probe fail\n", __func__, i2c_address_b);
		else
			printk(KERN_INFO"[QT] %s 0x%02x probe success\n", __func__, i2c_address_b);
    } else
		printk(KERN_INFO"[QT] %s 0x%02x probe success\n", __func__, i2c_address_a);

	return status;
}


uint8_t init_touch_driver(uint8_t i2c_address_a, uint8_t i2c_address_b)
{
	int i;

	int current_report_id = 0;

	uint8_t tmp;
	uint16_t current_address;
	uint16_t crc_address;
	object_t *object_table;
	info_id_t *id;
	uint8_t status;

	printk(KERN_INFO"[QT] %s start\n", __func__);

	touch_i2c_address_probe(i2c_address_a, i2c_address_b);

	/* Read the info block data. */
	id = &info_block->info_id;

	if (read_id_block(id) != 1) {
		printk(KERN_INFO"[TSP][ERROR] 2\n");
		return DRIVER_SETUP_INCOMPLETE;
	}

	/* Read object table. */
	info_block->objects = info_object_ptr;
	object_table = info_block->objects;


	/* Reading the whole object table block to memory directly doesn't work cause sizeof object_t
	   isn't necessarily the same on every compiler/platform due to alignment issues. Endianness
	   can also cause trouble. */

	current_address = OBJECT_TABLE_START_ADDRESS;


	for (i = 0; i < id->num_declared_objects; i++) {
		status = read_mem(current_address, 1, &(object_table[i]).object_type);
		if (status != READ_MEM_OK) {
			printk(KERN_INFO"[TSP][ERROR] 4\n");
			return DRIVER_SETUP_INCOMPLETE;
		}
		current_address++;

		status = read_U16(current_address, &object_table[i].i2c_address);
		if (status != READ_MEM_OK) {
			printk(KERN_INFO"[TSP][ERROR] 5\n");
			return DRIVER_SETUP_INCOMPLETE;
		}
		current_address += 2;

		status = read_mem(current_address, 1, (U8 *)&object_table[i].size);
		if (status != READ_MEM_OK) {
			printk(KERN_INFO"[TSP][ERROR] 6\n");
			return DRIVER_SETUP_INCOMPLETE;
		}
		current_address++;

		status = read_mem(current_address, 1, &object_table[i].instances);
		if (status != READ_MEM_OK) {
			printk(KERN_INFO"[TSP][ERROR] 7\n");
			return DRIVER_SETUP_INCOMPLETE;
		}
		current_address++;

		status = read_mem(current_address, 1, &object_table[i].num_report_ids);
		if (status != READ_MEM_OK) {
			printk(KERN_INFO"[TSP][ERROR] 8\n");
			return DRIVER_SETUP_INCOMPLETE;
		}
		current_address++;

		max_report_id += object_table[i].num_report_ids;

		/* Find out the maximum message length. */
		if (object_table[i].object_type == GEN_MESSAGEPROCESSOR_T5)
			max_message_length = object_table[i].size + 1;
	}

	/* Check that message processor was found. */
	if (max_message_length == 0) {
		printk(KERN_INFO"[TSP][ERROR] 9\n");
		return DRIVER_SETUP_INCOMPLETE;
	}

	/* Read CRC. */
	crc_address = OBJECT_TABLE_START_ADDRESS +
		id->num_declared_objects * OBJECT_TABLE_ELEMENT_SIZE;

	status = read_mem(crc_address, 1u, &tmp);
	if (status != READ_MEM_OK) {
		printk(KERN_INFO"[TSP][ERROR] 11\n");
		return DRIVER_SETUP_INCOMPLETE;
	}
	info_block->CRC = tmp;

	status = read_mem(crc_address + 1u, 1u, &tmp);
	if (status != READ_MEM_OK) {
		printk(KERN_INFO"[TSP][ERROR] 12\n");
		return DRIVER_SETUP_INCOMPLETE;
	}
	info_block->CRC |= (tmp << 8u);

	status = read_mem(crc_address + 2, 1, &info_block->CRC_hi);
	if (status != READ_MEM_OK) {
		printk(KERN_INFO"[TSP][ERROR] 13\n");
		return DRIVER_SETUP_INCOMPLETE;
	}

	/* Store message processor address, it is needed often on message reads. */
	message_processor_address = get_object_address(GEN_MESSAGEPROCESSOR_T5, 0);

	if (message_processor_address == 0) {
		printk(KERN_INFO"[TSP][ERROR] 14 !!\n");
		return DRIVER_SETUP_INCOMPLETE;
	}

	/* Store command processor address. */
	command_processor_address = get_object_address(GEN_COMMANDPROCESSOR_T6, 0);
	if (command_processor_address == 0) {
		printk(KERN_INFO"[TSP][ERROR] 15\n");
		return DRIVER_SETUP_INCOMPLETE;
	}

	/* Report ID 0 is reserved, so start from 1. */

	report_id_map[0].instance = 0;
	report_id_map[0].object_type = 0;
	current_report_id = 1;

	for (i = 0; i < id->num_declared_objects; i++) {
		if (object_table[i].num_report_ids != 0) {
			int instance;
			for (instance = 0; instance <= object_table[i].instances; instance++) {
				int start_report_id = current_report_id;
				for (;
						current_report_id <
						(start_report_id + object_table[i].num_report_ids);
						current_report_id++) {

					report_id_map[current_report_id].instance = instance;
					report_id_map[current_report_id].object_type =
						object_table[i].object_type;
				}
			}
		}
	}
	driver_setup = DRIVER_SETUP_OK;

	/* Initialize the pin connected to touch ic pin CHANGELINE to catch the
	 * falling edge of signal on that line. */

	return DRIVER_SETUP_OK;
}

static unsigned int qt_time_point;
static unsigned int qt_time_diff;
static unsigned int qt_timer_state;
static int check_abs_time(void)
{
	qt_time_diff = 0;

	if (!qt_time_point)
		return 0;

	qt_time_diff = jiffies_to_msecs(jiffies) - qt_time_point;
	if (qt_time_diff > 0)
		return 1;
	else
		return 0;
}


void check_chip_calibration(void)
{
	uint8_t data_buffer[100] = { 0 };
	uint8_t try_ctr = 0;
	uint8_t data_byte = 0xF3; /* dianostic command to get touch flags */
	uint16_t diag_address;
	uint16_t multi_touch_T9_address = 0;
	uint8_t tch_ch = 0, atch_ch = 0;
	uint8_t check_mask;
	uint8_t i;
	uint8_t j;
	uint8_t x_line_limit;
	uint8_t config_byte = 0;

	if (tsp_version >= 0x16) {
		config_byte = 255;
		multi_touch_T9_address = get_object_address(PROCG_NOISESUPPRESSION_T22, 0);
		write_mem(multi_touch_T9_address+6,1,&config_byte);
		/* we have had the first touchscreen or face suppression message
		 * after a calibration - check the sensor state and try to confirm if
		 * cal was good or bad */

		/* get touch flags from the chip using the diagnostic object */
		/* write command to command processor to get touch flags - 0xF3 Command required to do this */
		write_mem(command_processor_address + DIAGNOSTIC_OFFSET, 1, &data_byte);
		/* get the address of the diagnostic object so we can get the data we need */
		diag_address = get_object_address(DEBUG_DIAGNOSTIC_T37, 0);

		msleep(10);

		/* read touch flags from the diagnostic object - clear buffer so the while loop can run first time */
		memset(data_buffer , 0xFF, sizeof(data_buffer));

		/* wait for diagnostic object to update */
		while (!((data_buffer[0] == 0xF3) && (data_buffer[1] == 0x00))) {
			/* wait for data to be valid  */
			if (try_ctr > 10) {
				/* Failed! */
				dprintk("[TSP] Diagnostic Data did not update!!\n");
				qt_timer_state = 0;
				break;
			}
			msleep(2);
			try_ctr++; /* timeout counter */
			read_mem(diag_address, 2, data_buffer);
			dprintk("[TSP] Waiting for diagnostic data to update, try %d\n", try_ctr);
		}


		/* data is ready - read the detection flags */
		read_mem(diag_address, 82, data_buffer);

		/* data array is 20 x 16 bits for each set of flags, 2 byte header, 40 bytes for touch flags 40 bytes for antitouch flags*/

		/* count up the channels/bits if we recived the data properly */
		if ((data_buffer[0] == 0xF3) && (data_buffer[1] == 0x00)) {
			/* mode 0 : 16 x line, mode 1 : 17 etc etc upto mode 4.*/
			x_line_limit = 16 + cte_config.mode;

			if (x_line_limit > 20) {
				/* hard limit at 20 so we don't over-index the array */
				x_line_limit = 20;
			}

			/* double the limit as the array is in bytes not words */
			x_line_limit = x_line_limit << 1;

			/* count the channels and print the flags to the log */
			/* check X lines - data is in words so increment 2 at a time */
			for (i = 0; i < x_line_limit; i += 2) {
				/* print the flags to the log - only really needed for debugging */

				/* count how many bits set for this row */
				for (j = 0; j < 8; j++) {
					/* create a bit mask to check against */
					check_mask = 1 << j;

					/* check detect flags */
					if (data_buffer[2+i] & check_mask)
						tch_ch++;
					if (data_buffer[3+i] & check_mask)
						tch_ch++;

					/* check anti-detect flags */
					if (data_buffer[42+i] & check_mask)
						atch_ch++;
					if (data_buffer[43+i] & check_mask)
						atch_ch++;
				}
			}


			/* print how many channels we counted */
			dprintk("[TSP] Flags Counted channels: t:%d a:%d \n", tch_ch, atch_ch);

			/* send page up command so we can detect when data updates next time,
			 * page byte will sit at 1 until we next send F3 command */
			data_byte = 0x01;
			write_mem(command_processor_address + DIAGNOSTIC_OFFSET, 1, &data_byte);

			/* process counters and decide if we must re-calibrate or if cal was good */
			printk(KERN_DEBUG "[TSP][SUKER] atch_ch = %d\n", atch_ch);
			printk(KERN_DEBUG "[TSP][SUKER] tch_ch = %d\n", tch_ch);
			printk(KERN_DEBUG "[TSP][SUKER] qt_time_diff = %d\n", qt_time_diff);
			
			if (tch_ch >= 0 && tch_ch <= 15 && atch_ch == 0) {
				/* cal was good - don't need to check any more */
				if (!check_abs_time())
					qt_time_diff = 501;

				if (qt_timer_state == 1) {
					if (qt_time_diff > 500) {
						dprintk("[TSP] calibration was good\n");
						printk("[TSP] calibration was good\n");
						config_byte = 30;
						multi_touch_T9_address = get_object_address(PROCG_NOISESUPPRESSION_T22, 0);
						write_mem(multi_touch_T9_address+6,1,&config_byte);

						cal_check_flag = 0;
						qt_timer_state = 0;
						qt_time_point = jiffies_to_msecs(jiffies);

						printk("[TSP] reset acq atchcalst=%d, atchcalsthr=%d\n", acquisition_config.atchcalst, acquisition_config.atchcalsthr);

						//ignore the 'calibration was bad1' routine.
						if (bCalibrationAndResetChip == true)	{
							bCalibrationAndResetChip = false;
						}
						/* Write normal acquisition config back to the chip. */
						if (write_acquisition_config(acquisition_config) != CFG_WRITE_OK)
							/* "Acquisition config write failed!\n" */
							printk(KERN_DEBUG "\n[TSP][ERROR] line : %d\n", __LINE__);
				    } else
					   cal_check_flag = 1;
				} else {
					qt_timer_state = 1;
					qt_time_point = jiffies_to_msecs(jiffies);
					cal_check_flag = 1;
				}
			}
			else if (atch_ch >= 8 || tch_ch > 15)
			{
				printk(KERN_DEBUG "[TSP] calibration was bad1\n");
				/* cal was bad - must recalibrate and check afterwards */
				if (nCurrentState == MOVE || nCurrentState == PRESSED)	{
					bCalibrationAndResetChip = true;
				}
				//refresh_input();
				//qt_timer_state = 0;
				//qt_time_point = jiffies_to_msecs(jiffies);
			} else if (tch_ch > 18) {
				printk(KERN_DEBUG "[TSP] calibration was bad2\n");
				/* cal was bad - must recalibrate and check afterwards */
				calibrate_chip();
				qt_timer_state = 0;
				qt_time_point = jiffies_to_msecs(jiffies);
			} else {
				dprintk("[TSP] calibration was not decided yet\n");
				/* we cannot confirm if good or bad - we must wait for next touch  message to confirm */
				cal_check_flag = 1u;
				/* Reset the 100ms timer */
				qt_timer_state = 0;
				qt_time_point = jiffies_to_msecs(jiffies);
			}
		}
	}
}

#ifdef LG_FW_TOUCH_SOFT_KEY

enum{
	NO_KEY_TOUCHED,
	KEY1_TOUCHED,
	KEY2_TOUCHED,
	KEY3_TOUCHED,
	KEY4_TOUCHED,
	MAX_KEY_TOUCH
};

static __inline void qt_key_event_touch(int touch_reg,  int value)
{

	input_report_key(qt602240->input_dev, touch_reg, value);
	input_sync(qt602240->input_dev);

	printk(KERN_DEBUG "[TSP]%s Touch Key Code %d, Value %d\n", __FUNCTION__, touch_reg, value);

	return;
}
#endif


void  get_message(struct work_struct *p)
{
	struct input_dev *input_dev = qt602240->input_dev;

    uint8_t object_type, instance;

	int x, y;
	int size = 2;
	int i;
	uint8_t touch_message_flag = 0;

	static int nprevid = -1;
	uint8_t id = 0;
	int bchangeupdn = 0;

	static long cnt;

	cnt = 0;
	cnt++;

	mutex_lock(&input_dev->mutex);	

	if (driver_setup == DRIVER_SETUP_OK) {
		if (read_mem(message_processor_address, 6, quantum_msg) == READ_MEM_OK) {

			object_type = report_id_to_type(quantum_msg[0] , &instance);

			DEBUG_M(KERN_DEBUG "[TSP_DEBUG] object_type [%d]\n", object_type);
			DEBUG_M(KERN_DEBUG "[TSP_DEBUG] msg id =  %x %x %x %x %x %x %x %x %x\n", quantum_msg[0], quantum_msg[1], quantum_msg[2],\
					 quantum_msg[3], quantum_msg[4], quantum_msg[5], quantum_msg[6], quantum_msg[7], quantum_msg[8]);

			/* Invalid message : End of Touch Event */
			if (quantum_msg[0] == 0xFF) {
                		printk("[TSP] 0xFF Detected : msg id =  %x %x %x %x %x %x %x %x %x\n", quantum_msg[0], quantum_msg[1], quantum_msg[2],quantum_msg[3], quantum_msg[4], quantum_msg[5], quantum_msg[6], quantum_msg[7], quantum_msg[8]);
				DEBUG_M(KERN_DEBUG "[TSP_DEBUG] Touch IRQ End!!\n");
							
				qt_read_mem_fail_count++;
				
				//touch sensor reset
				if (qt_read_mem_fail_count >= 3)	{
					qt_read_mem_fail_count = 0;
					DEBUG_M(KERN_DEBUG "[TSP_FFCHK] sw_reset_chip run");
					reset_chip();//hw_reset_chip();
				}
				goto EXIT_PROC;
			}
			 else {
				qt_read_mem_fail_count = 0;
			}

			switch (object_type) {
/*			case RESERVED_T0:
				printk(KERN_INFO"\n\n [TSP] [ERROR] object tyle is zero !!\n\n");
				reset_chip();
				msleep(100);						
				break;*/
			case TOUCH_MULTITOUCHSCREEN_T9:
				id = quantum_msg[0] - 2;

				x = quantum_msg[2];
				x = x << 2;
				x = x | quantum_msg[4] >> 6;

				y = quantum_msg[3];
				y = y << 2;
				y = y | ((quantum_msg[4] & 0x0C)  >> 2);

				size = quantum_msg[5];

				if (quantum_msg[1] & MXT_MSGB_T9_RELEASE) {
					if (bCalibrationAndResetChip == true)	{
						printk(KERN_INFO"\n[TSP_SUKER][INFO] ***** bCalibrationAndResetChip true **** \n");
						refresh_input();
						qt_timer_state = 0;
						qt_time_point = jiffies_to_msecs(jiffies);
						
						wake_unlock(&tsp_sensing_wake_lock);
						enable_irq(qt602240->client->irq);						
						
						bCalibrationAndResetChip = false;
						mutex_unlock(&input_dev->mutex);

						return;
					}
					
					fingerinfo[id].size_id = (id << 8)|size;
#ifdef LG_FW_TOUCH_SOFT_KEY
					if (qt602240_pdata->ts_y_scrn_max+QT_EDGE_COMPENSATION_Y <= fingerinfo[id].y)
					{
						if (nCurrentState == MOVE)	{
							DEBUG_M(KERN_DEBUG "[TSP_SUKER] ##out## RELEASE Action -doing move in screen-\n");
							goto EXIT_PROC;
						}
						
						DEBUG_M(KERN_DEBUG "[TSP_SUKER] ##out## RELEASE Action --start-- \n");
						DEBUG_M(KERN_DEBUG "[TSP_SUKER] ##out## RELEASE Finger id = %d, x=%d,y=%d  status=0x%02x \n",
											 id, fingerinfo[id].x, fingerinfo[id].y , quantum_msg[1]);
						DEBUG_M(KERN_DEBUG "[TSP_SUKER] ##out## *** nTouchKeyPressedId  =%d ***\n",nTouchKeyPressedId);								

						if (nTouchKeyPressedId != -1)	{							
							for(i=0;i<QT_TOUCHKEY_MAX;i++) //validation check for touch key area
							{
								if (	touchkey_area[i].x1 <= fingerinfo[id].x &&
									touchkey_area[i].x2 >= fingerinfo[id].x &&
									touchkey_area[i].y1 <= fingerinfo[id].y &&
									touchkey_area[i].y2 >= fingerinfo[id].y )
								{									
									if (nTouchKeyState != QT_TOUCHKEY_STATE_MOVE_INVALID_AREA)
									{
			                        				touchkey_area[i].id = 0xFF;		                        			
										qt_key_event_touch(touchkey_area[i].key,  RELEASED);
										nCurrentState = RELEASED;
										DEBUG_M(KERN_DEBUG "[TSP_SUKER] ##out## RELEASE Action1  key=%d, touchKeyId=%d\n", nTouchKeyPressedId, id);
									}
									else	{
										touchkey_area[nTouchKeyPressedId].id = 0xFF;		                        			
										qt_key_event_touch(touchkey_area[nTouchKeyPressedId].key,  RELEASED);
										nCurrentState = RELEASED;
										DEBUG_M(KERN_DEBUG "[TSP_SUKER] ##out## RELEASE Action2  key=%d, touchKeyId=%d\n", nTouchKeyPressedId, id);
									}						
									break;									
								}	else	{
									touchkey_area[nTouchKeyPressedId].id = 0xFF;		                        			
									qt_key_event_touch(touchkey_area[nTouchKeyPressedId].key,  RELEASED);
									nCurrentState = RELEASED;
									DEBUG_M(KERN_DEBUG "[TSP_SUKER] ##out## RELEASE invalid Action3  key=%d, touchKeyId=%d\n", nTouchKeyPressedId, id);
								}
							}
						}
						else	{
							touchkey_area[2].id = 0xFF;		                        			
							qt_key_event_touch(touchkey_area[2].key,  RELEASED);
							nCurrentState = RELEASED;
						}
						bIsTouchKeyPressed = false;
						nTouchKeyPressedId = -1;
						nTouchKeyState = QT_TOUCHKEY_STATE_NONE;
						
						// put invalid pressure value;
						fingerinfo[id].size_id = 0;
						fingerinfo[id].pressure = -1;
						fingerinfo[id].x = (int16_t)0;
						fingerinfo[id].y = (int16_t)0;
					}
					else
#endif			
					{
						nCurrentState = RELEASED;
						fingerinfo[id].pressure = 0;
						bchangeupdn = 1;
						DEBUG_M(KERN_DEBUG "[TSP_SUKER] ##in## RELEASE Action on Screen, Finger id = %d, x=%d,y=%d  status=0x%02x \n",
											 id, fingerinfo[id].x, fingerinfo[id].y , quantum_msg[1]);						
					}
				} else if ((quantum_msg[1] & MXT_MSGB_T9_DETECT) && (quantum_msg[1] & MXT_MSGB_T9_PRESS)) {					
					touch_message_flag = 1;					
					bNeedCheckResetConfition = false;						
					
					fingerinfo[id].size_id = (id<<8)|size;
					fingerinfo[id].pressure = 40;
#ifdef LG_FW_TOUCH_SOFT_KEY
					if (qt602240_pdata->ts_y_scrn_max+QT_EDGE_COMPENSATION_Y <= y)
					{
						bIsTouchKeyPressed = false;
						nTouchKeyPressedId = -1;
						nTouchKeyState = QT_TOUCHKEY_STATE_NONE;
						
						if (nCurrentState == MOVE)	{
							DEBUG_M(KERN_DEBUG "[TSP_SUKER] ##out## PRESS Action -doing move in screen-\n");
							goto EXIT_PROC;
						}
						
						DEBUG_M(KERN_DEBUG "[TSP_SUKER] ##out## PRESS Action --start--, Finger id = %d, x=%d,y=%d  status=0x%02x \n",
											 id, x, y , quantum_msg[1]);
						for(i=0;i<QT_TOUCHKEY_MAX;i++)
						{
							if(	touchkey_area[i].x1 <= x &&
								touchkey_area[i].x2 >= x &&
								touchkey_area[i].y1 <= y &&
								touchkey_area[i].y2 >= y )
							{
								DEBUG_M(KERN_DEBUG "[TSP_SUKER] ##out## PRESS Action  key=%d, touchKeyId=%d\n", i, id);								
								bIsTouchKeyPressed = true;
								
								nTouchKeyPressedId = i;
		                        			touchkey_area[i].id = id;
								qt_key_event_touch(touchkey_area[i].key,  PRESSED);
								nCurrentState = PRESSED;
								fingerinfo[id].x = (int16_t)x;
								fingerinfo[id].y = (int16_t)y;
								break;
							}
						}
						// put invalid pressure value;
						// reset fingerinfo
						for (i = 0; i < MAX_USING_FINGER_NUM; i++) {							
							fingerinfo[i].size_id = 0;
							fingerinfo[i].pressure = -1;
						}
					}
					else
#endif			
					{
						nCurrentState = PRESSED;
						fingerinfo[id].x = (int16_t)x;
						fingerinfo[id].y = (int16_t)y;
						bchangeupdn = 1;
						
						nCheckMultiTouchErrCheck = 0;	//initialize						
						bNeedCheckResetConfition = true;
						
						DEBUG_M(KERN_DEBUG "[TSP_SUKER] ##in## PRESS Action on Screen, Finger id = %d, x=%d,y=%d  status=0x%02x \n",
											 id, fingerinfo[id].x, fingerinfo[id].y , quantum_msg[1]);						
					}
				} else if ((quantum_msg[1] & MXT_MSGB_T9_DETECT) && (quantum_msg[1] & MXT_MSGB_T9_MOVE)) {
					touch_message_flag = 1;

					fingerinfo[id].size_id = (id << 8)|size;															
#ifdef LG_FW_TOUCH_SOFT_KEY
					if (qt602240_pdata->ts_y_scrn_max+QT_EDGE_COMPENSATION_Y <= y)
					{
						bNeedCheckResetConfition = false;
						DEBUG_M(KERN_DEBUG "[TSP_SUKER] ##out## MOVE Action --start-- \n");
						if (bIsTouchKeyPressed == true )	{
							DEBUG_M(KERN_DEBUG "[TSP_SUKER] ##out## MOVE Action already pressed touch key \n");

							for(i=0;i<QT_TOUCHKEY_MAX;i++) //validation check for touch key area
							{
								if(	touchkey_area[i].x1 < x &&
									touchkey_area[i].x2 > x &&
									touchkey_area[i].y1 < y &&
									touchkey_area[i].y2 > y )
								{
									if (i == nTouchKeyPressedId)	{
										nTouchKeyState = QT_TOUCHKEY_STATE_MOVE_VALID_AREA;
										DEBUG_M(KERN_DEBUG "[TSP_SUKER] ##out## MOVE First Pressed touch region OK ignore~~ \n"); //ignore
									}
									else	{//another touch area entering
										bIsTouchKeyPressed = false;
										nTouchKeyState = QT_TOUCHKEY_STATE_MOVE_INVALID_AREA;
										DEBUG_M(KERN_DEBUG "[TSP_SUKER] ##out## MOVE First Pressed touch another region OK ignore~~ \n"); //ignore
									}					
									break;
								}
								else	{ //not departure region
									DEBUG_M(KERN_DEBUG "[TSP_SUKER] ##out## MOVE First Pressed touch region Nothing but ignore~~ = %d\n", i); //ignore
								}
							}
							// put invalid pressure value;
							fingerinfo[id].size_id = 0;
							fingerinfo[id].pressure = -1;
							fingerinfo[id].x = (int16_t)0;
							fingerinfo[id].y = (int16_t)0;
						}
						else	{
							if (nTouchKeyState == QT_TOUCHKEY_STATE_MOVE_INVALID_AREA)	{
								;//nothing to do
							}
							else	{
								nCurrentState = RELEASED;
								fingerinfo[id].pressure = 0;								
								bchangeupdn = 1;
								bNeedCheckResetConfition = true;
								DEBUG_M(KERN_DEBUG "[TSP_SUKER] ##out## MOVE Action --end-0- not touch key\n");
							}
						}
					}
					else if (qt602240_pdata->ts_y_scrn_max < y)
					{
						bNeedCheckResetConfition = false;
						DEBUG_M(KERN_DEBUG "[TSP_SUKER] ##out-in## MOVE Action --start-1- \n");
						if (nTouchKeyPressedId <= -1 || nTouchKeyPressedId >= 4)	{
							nTouchKeyPressedId = 2;//KEY_BACK
						}

						touchkey_area[nTouchKeyPressedId].id = 0xFF;
						qt_key_event_touch(touchkey_area[nTouchKeyPressedId].key,  RELEASED);
						
						nCurrentState = RELEASED;
					}
					else
#endif
					{				
#ifdef LG_FW_TOUCH_SOFT_KEY
						if (qt602240_pdata->ts_y_scrn_max+QT_EDGE_COMPENSATION_Y <= fingerinfo[id].y)
						{
							bNeedCheckResetConfition = false;
							if (nTouchKeyState == QT_TOUCHKEY_STATE_MOVE_VALID_AREA)	{
								DEBUG_M(KERN_DEBUG "[TSP_SUKER] ##in## MOVE Action on Screen QT_TOUCHKEY_STATE_MOVE_VALID_AREA\n");
								if (nTouchKeyPressedId <= -1 || nTouchKeyPressedId >= 4)	{
									nTouchKeyPressedId = 2;//KEY_BACK
								}
								touchkey_area[nTouchKeyPressedId].id = 0xFF;
								qt_key_event_touch(touchkey_area[nTouchKeyPressedId].key,  RELEASED);
								nTouchKeyState = QT_TOUCHKEY_STATE_NONE;
								nCurrentState = RELEASED;
							}
							else	{								
								DEBUG_M(KERN_DEBUG "[TSP_SUKER] ##in## MOVE Action on screen normal move1\n");
								fingerinfo[id].pressure = 40;
								nCurrentState = MOVE;
							}
						}
						else
#endif
						{
							nCurrentState = MOVE;
							//performance by log message
							//DEBUG_M(KERN_DEBUG "[TSP_SUKER] ##in## MOVE Action on screen normal move2\n");//DEBUG_M("##### Finger[%d] Move (%d,%d)!\n", id, x, y );
						}
					}

					if (nCheckMultiTouchErrCheck == 0 && bNeedCheckResetConfition == true)	{
						DEBUG_M(KERN_DEBUG "[TSP_SUKER][ERROR]---- nCheckMultiTouchErrCheck have to 1 ----\n");
						DEBUG_M(KERN_DEBUG "[TSP_SUKER][ERROR]---- because nCheckMultiTouchErrCheck is %d\n",nCheckMultiTouchErrCheck);
						
						reset_chip();
						msleep(100);						
						DEBUG_M(KERN_DEBUG "[TSP_SUKER][ERROR]---- enable irq after reset_chip --\n");
						goto EXIT_PROC;
					}
					
					fingerinfo[id].x = (int16_t)x;
					fingerinfo[id].y = (int16_t)y;
					DEBUG_M(KERN_DEBUG "[TSP_SUKER] #### MOVE Action --end-real- \n");
				} 
				else	{
					DEBUG_M(KERN_INFO"[TSP] T9 Unknown state 0x%x 0x%x!\n ", quantum_msg[0], quantum_msg[1]);
				}

				if (nprevid >= id || bchangeupdn) {
					for (i = 0; i < MAX_USING_FINGER_NUM; ++i) {
						int16_t comp_y;

						// at edge, compensate y coordinate.
						#ifdef LG_FW_TOUCH_SOFT_KEY
						if(fingerinfo[i].y >= qt602240_pdata->ts_y_scrn_max)
						{
							comp_y = qt602240_pdata->ts_y_scrn_max-1;
						}
						else
						#endif
						{
							comp_y = fingerinfo[i].y;
						}

						if (fingerinfo[i].pressure == -1)
							continue;

						//0902
						if ((fingerinfo[i].y >= qt602240_pdata->ts_y_scrn_max &&
							fingerinfo[i].y < qt602240_pdata->ts_y_scrn_max+65) && (i != 0) )	{
							DEBUG_M(KERN_INFO"[TSP] C8C8 fuck issue\n ");
							continue;
						}
						
						input_report_abs(qt602240->input_dev, ABS_MT_POSITION_X, fingerinfo[i].x);
						input_report_abs(qt602240->input_dev, ABS_MT_POSITION_Y, comp_y);
						input_report_abs(qt602240->input_dev, ABS_MT_TOUCH_MAJOR, fingerinfo[i].pressure);
						input_report_abs(qt602240->input_dev, ABS_MT_WIDTH_MAJOR, fingerinfo[i].size_id);
						input_report_abs(qt602240->input_dev, ABS_MT_TRACKING_ID, i);

						input_mt_sync(qt602240->input_dev);
						
						DEBUG_M(KERN_DEBUG "[TSP_DEBUG] [%d] x(%d) y(%d) size(%d) press(%d)\n", i, fingerinfo[i].x, fingerinfo[i].y, fingerinfo[i].size_id,\
							 fingerinfo[i].pressure);

						if (fingerinfo[i].pressure == 0)	{		//0 equal release
							fingerinfo[i].pressure = -1;
							if (nCurrentState == RELEASED)
								--nCheckMultiTouchErrCheck;
						}	else	{
							if (nCurrentState == PRESSED)
								nCheckMultiTouchErrCheck++;
						}
					}
					//DEBUG_M(KERN_DEBUG "[TSP_SUKER]  nCheckMultiTouchErrCheck = %d",nCheckMultiTouchErrCheck);		
					input_sync(qt602240->input_dev);

					DEBUG_M(KERN_DEBUG "[TSP_DEBUG] Touch Event End!!\n\n");
				}

				nprevid = id;

				break;
			}			
		} else
			printk(KERN_INFO"\n [TSP] read_mem is failed -- check your TSP chip!!\n\n");

	}

EXIT_PROC:
	if (touch_message_flag && (cal_check_flag))
		check_chip_calibration();
		
	wake_unlock(&tsp_sensing_wake_lock);
	enable_irq(qt602240->client->irq);

	mutex_unlock(&input_dev->mutex);

	return;
}


static void refresh_input(void)
{
	int i;
	
	if (nTouchKeyPressedId <= -1 || nTouchKeyPressedId >= 4)	{
		nTouchKeyPressedId = 2;//KEY_BACK
	}
	touchkey_area[nTouchKeyPressedId].id = 0xFF;		                        			
	qt_key_event_touch(touchkey_area[nTouchKeyPressedId].key,  RELEASED);					
						
	for (i = 0; i < MAX_USING_FINGER_NUM; i++) {				
		fingerinfo[i].size_id = 0;
		fingerinfo[i].pressure = -1;
	}
	
	calibrate_chip();
	msleep(64);
	reset_chip();
	msleep(100);
}

/*------------------------------ I2C Driver block -----------------------------------*/

#define I2C_M_WR 0 /* for i2c */
#define I2C_MAX_SEND_LENGTH     300

int qt602240_i2c_write(u16 reg, u8 *read_val, unsigned int len)
{
	struct i2c_msg wmsg;

	unsigned char data[I2C_MAX_SEND_LENGTH];
	int ret, i;

	address_pointer = reg;
	
	DEBUG_M(KERN_DEBUG "[TSP_DEBUG] qt602240_i2c_write\n");
	
	if (len + 2 > I2C_MAX_SEND_LENGTH) {
		printk(KERN_INFO"[TSP][ERROR] %s() data length error\n", __func__);
		return -ENODEV;
	}

	wmsg.addr = qt602240->client->addr;
	wmsg.flags = I2C_M_WR;
	wmsg.len = len + 2;
	wmsg.buf = data;

	data[0] = reg & 0x00ff;
	data[1] = reg >> 8;

	for (i = 0; i < len; i++)
		data[i+2] = *(read_val+i);

	ret = i2c_transfer(qt602240->client->adapter, &wmsg, 1);
	return ret;
}

int boot_qt602240_i2c_write(u16 reg, u8 *read_val, unsigned int len)
{
	struct i2c_msg wmsg;

	unsigned char data[I2C_MAX_SEND_LENGTH];
	int ret, i;

	if (len+2 > I2C_MAX_SEND_LENGTH) {
		printk(KERN_INFO"[TSP][ERROR] %s() data length error\n", __func__);
		return -ENODEV;
	}

	wmsg.addr = 0x24;
	wmsg.flags = I2C_M_WR;
	wmsg.len = len;
	wmsg.buf = data;

	for (i = 0; i < len; i++)
		data[i] = *(read_val+i);

	ret = i2c_transfer(qt602240->client->adapter, &wmsg, 1);
	return ret;
}


int qt602240_i2c_read(u16 reg, unsigned char *rbuf, int buf_size)
{
	static unsigned char first_read = 1;
	struct i2c_msg rmsg;
	int ret;
	unsigned char data[2];

	rmsg.addr = qt602240->client->addr;

	//DEBUG_M(KERN_DEBUG "[TSP_DEBUG] qt602240_i2c_read\n");
    	if (first_read == 1) {
		first_read = 0;
		address_pointer = reg+1;
	}

	if ((address_pointer != reg) || (reg != message_processor_address)) {

		address_pointer = reg;

		rmsg.flags = I2C_M_WR;
		rmsg.len = 2;
		rmsg.buf = data;
		data[0] = reg & 0x00ff;
		data[1] = reg >> 8;
		ret = i2c_transfer(qt602240->client->adapter, &rmsg, 1);
	}

	rmsg.flags = I2C_M_RD;
	rmsg.len = buf_size;
	rmsg.buf = rbuf;
	ret = i2c_transfer(qt602240->client->adapter, &rmsg, 1);

	return ret;
}


/*! \brief Maxtouch Memory read by I2C bus */
U8 read_mem(U16 start, U8 size, U8 *mem)
{
	int ret;

	memset(mem, 0xFF, size);
	//DEBUG_M(KERN_DEBUG "[TSP_FFCHK] U8 read_mem\n");

	ret = qt602240_i2c_read(start, mem, size);
	if (ret < 0) {
		printk(KERN_INFO"%s : i2c read failed\n", __func__);
		return READ_MEM_FAILED;
	}

#ifdef QT_TOUCH_DEBUG_IRQ_CHECK
	memset(quantum_msg_for_print, 0x00, 9);
	if (size <= 9)
		memcpy(quantum_msg_for_print, mem, size);
#endif
		
	return READ_MEM_OK;
}

U8 boot_read_mem(U16 start, U8 size, U8 *mem)
{
	struct i2c_msg rmsg;
	int ret;
	
	DEBUG_M(KERN_DEBUG "[TSP_FFCHK] U8 boot_read_mem\n");

	rmsg.addr = 0x24;
	rmsg.flags = I2C_M_RD;
	rmsg.len = size;
	rmsg.buf = mem;
	ret = i2c_transfer(qt602240->client->adapter, &rmsg, 1);

	return ret;
}

U8 read_U16(U16 start, U16 *mem)
{
	U8 status;

	DEBUG_M(KERN_DEBUG "[TSP_FFCHK] U8 read_U16\n");
	status = read_mem(start, 2, (U8 *) mem);

	return status;
}

U8 write_mem(U16 start, U8 size, U8 *mem)
{
	int ret;

	DEBUG_M(KERN_DEBUG "[TSP_FFCHK] U8 write_mem\n");
	ret = qt602240_i2c_write(start, mem, size);
	if (ret < 0)
		return WRITE_MEM_FAILED;
	else
		return WRITE_MEM_OK;
}

U8 boot_write_mem(U16 start, U16 size, U8 *mem)
{
	int ret;

	DEBUG_M(KERN_DEBUG "[TSP_FFCHK] U8 boot_write_mem\n");
	ret = boot_qt602240_i2c_write(start, mem, size);
	if (ret < 0) {
		printk(KERN_INFO"boot write mem fail: %d \n", ret);
		return WRITE_MEM_FAILED;
	} else
		return WRITE_MEM_OK;
}


void write_message_to_usart(uint8_t msg[], uint8_t length)
{
    int i;

    for (i = 0; i < length; i++)
		QT_printf("0x%02x ", msg[i]);

    QT_printf("\n\r");
}


irqreturn_t qt602240_irq_handler(int irq, void *dev_id)
{
	disable_irq_nosync(qt602240->client->irq);

	wake_lock(&tsp_sensing_wake_lock);

#ifdef USE_DELAYED_WORK
	queue_delayed_work(qt602240_wq, &qt602240->ts_event_delayed_work, msecs_to_jiffies(DELAYED_SEC));
#else
	queue_work(qt602240_wq, &qt602240->ts_event_work);
#endif

#ifdef QT_TOUCH_DEBUG_IRQ_CHECK
	if (quantum_msg_for_print[0] != 0)	{
		irq_handler_message_print();
	}
#endif

	return IRQ_HANDLED;
}

#ifdef QT_TOUCH_DEBUG_IRQ_CHECK
void irq_handler_message_print(void)
{
	DEBUG_M(KERN_DEBUG "[TSP_FFCHK] qt602240_irq_handler \n");
	DEBUG_M(KERN_DEBUG "[TSP_FFCHK] msg id =  %x %x %x %x %x %x %x %x %x\n", quantum_msg_for_print[0], quantum_msg_for_print[1], quantum_msg_for_print[2],\
		 quantum_msg_for_print[3], quantum_msg_for_print[4], quantum_msg_for_print[5], quantum_msg_for_print[6], quantum_msg_for_print[7], quantum_msg_for_print[8]);
}
#endif

int qt602240_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	int err = 0;
	int ret;

	printk(KERN_INFO"qt602240_i2c is attached..\n");

	dprintk("+-----------------------------------------+\n");
	printk(KERN_DEBUG "|  Quantum Touch Driver Probe!            |\n");
	dprintk("+-----------------------------------------+\n");

#ifdef USE_DELAYED_WORK
	INIT_DELAYED_WORK(&qt602240->ts_event_delayed_work, get_message);
#else
	INIT_WORK(&qt602240->ts_event_work, get_message);
#endif

	qt602240_pdata = client->dev.platform_data;

	qt602240->client = client;

	qt602240->power = qt602240_pdata->power;
	qt602240->num_irq = qt602240_pdata->irq;
	qt602240->intr_gpio = qt602240_pdata->gpio_int;
	qt602240->hw_i2c =  qt602240_pdata->hw_i2c;
	qt602240->sda_gpio = qt602240_pdata->sda;
	qt602240->scl_gpio  = qt602240_pdata->scl;
	qt602240->ce_gpio  = qt602240_pdata->ce;

	qt602240->client->irq = qt602240_pdata->irq;

	printk(KERN_DEBUG "TS Power On. \n");
	qt602240->power(1);
	
	gpio_tlmm_config(GPIO_CFG(qt602240->intr_gpio, 0, GPIO_CFG_INPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

#ifdef CONFIG_MAINTOUCH_CE
	if (gpio_is_valid(qt602240->ce_gpio)) {
		if (gpio_request(qt602240->ce_gpio, "maintouch_ce"))
			printk("Failed to request GPIO_TOUCH_EN!\n");
		gpio_direction_output(qt602240->ce_gpio, 1);
	}
#endif

	printk(KERN_DEBUG "qt602240 GPIO Status\n");

#ifdef CONFIG_MAINTOUCH_CE
	printk(KERN_DEBUG "TOUCH_EN  : %s\n", gpio_get_value(qt602240->ce_gpio)? "High":"Low");
#endif
	printk(KERN_DEBUG "TOUCH_INT : %s\n", gpio_get_value(qt602240->intr_gpio) ? "High" : "Low");

	msleep(100);

	qt602240->input_dev = input_allocate_device();
	if (qt602240->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_DEBUG "qt602240_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	qt602240->input_dev->name = "qt602240_ts_input";
	qt602240->input_dev->id.vendor = 0; //VENDOR_LGE;
	set_bit(EV_SYN, qt602240->input_dev->evbit);
	set_bit(EV_KEY, qt602240->input_dev->evbit);
#ifdef LG_FW_TOUCH_SOFT_KEY
	set_bit(KEY_MENU, qt602240->input_dev->keybit);
	set_bit(KEY_HOME, qt602240->input_dev->keybit);
	set_bit(KEY_BACK, qt602240->input_dev->keybit);
	set_bit(KEY_SEARCH, qt602240->input_dev->keybit);
#endif	
	set_bit(EV_ABS, qt602240->input_dev->evbit);

	input_set_abs_params(qt602240->input_dev, ABS_X, qt602240_pdata->ts_x_min, qt602240_pdata->ts_x_max, 0, 0);
	input_set_abs_params(qt602240->input_dev, ABS_Y, qt602240_pdata->ts_y_min, qt602240_pdata->ts_y_max, 0, 0);
	input_set_abs_params(qt602240->input_dev, ABS_MT_POSITION_X, qt602240_pdata->ts_x_min, qt602240_pdata->ts_x_max, 0, 0);
	input_set_abs_params(qt602240->input_dev, ABS_MT_POSITION_Y, qt602240_pdata->ts_y_min, qt602240_pdata->ts_y_max, 0, 0);

	input_set_abs_params(qt602240->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(qt602240->input_dev, ABS_TOOL_WIDTH, 0, 15, 0, 0);
	input_set_abs_params(qt602240->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(qt602240->input_dev, ABS_MT_WIDTH_MAJOR, 0, 30, 0, 0);
	input_set_abs_params(qt602240->input_dev, ABS_MT_TRACKING_ID, 0, MAX_USING_FINGER_NUM, 0, 0);

	ret = input_register_device(qt602240->input_dev);
	if (ret) {
		printk(KERN_DEBUG "qt602240_probe: Unable to register %s input device\n", qt602240->input_dev->name);
		goto err_input_register_device_failed;
	}

	i2c_set_clientdata(client, qt602240);

	err = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);

	if (!err) {
		printk(KERN_ERR "%s: fucntionality check failed\n",
				__func__);
		return err;
	}

#ifdef QT_FIRMUP_ENABLE
{
	qt_reprogram();
}
#else
	quantum_touch_probe();
#endif

	gpio_request(qt602240->intr_gpio, "qt602240_interrupt");
	err = gpio_direction_input(qt602240->intr_gpio);
	if (err < 0) {
		printk(KERN_ERR "%s: gpio input direction fail\n", __func__);
		return err;
	}

	ret = request_irq(qt602240->client->irq, qt602240_irq_handler,IRQF_DISABLED | IRQF_TRIGGER_FALLING, "qt602240 irq", qt602240);

	if (ret == 0)
		dprintk("[TSP] qt602240_probe: Start touchscreen %s\n", qt602240->input_dev->name);
	else
		printk(KERN_INFO"[TSP] request_irq failed\n");

	dprintk("%s ,  %d\n", __func__, __LINE__);
#ifdef CONFIG_HAS_EARLYSUSPEND
	qt602240->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	qt602240->early_suspend.suspend = qt602240_early_suspend;
	qt602240->early_suspend.resume = qt602240_late_resume;
	register_early_suspend(&qt602240->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */


	return 0;

err_input_register_device_failed:
	input_free_device(qt602240->input_dev);

err_input_dev_alloc_failed:
	kfree(qt602240);
	return ret;
}


static int qt602240_remove(struct i2c_client *client)
{
	struct qt602240_data *data = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

	printk(KERN_INFO"\n[TSP][IMPORTANT] qt602240_remove");
	free_irq(data->num_irq, data);
	free_irq(qt602240->client->irq, 0);
	input_unregister_device(qt602240->input_dev);
	kfree(qt602240);
	i2c_set_clientdata(client, NULL);

	return 0;
}


#ifndef CONFIG_HAS_EARLYSUSPEND
static int qt602240_suspend(struct platform_device *dev, pm_message_t mesg)
{
	gen_powerconfig_t7_config_t power_config_sleep = {0};
	int i = 0;

	ENTER_FUNC;

	disable_irq(qt602240->client->irq);

	/* Set power config. */
	/* Set Idle Acquisition Interval to 32 ms. */
	power_config_sleep.idleacqint = 0;
	/* Set Active Acquisition Interval to 16 ms. */
	power_config_sleep.actvacqint = 0;

	/* Write power config to chip. */
	if (write_power_config(power_config_sleep) != CFG_WRITE_OK) {
		/* "Power config write failed!\n" */
		printk(KERN_INFO"\n[TSP][ERROR] line : %d\n", __LINE__);
	}

	for (i = 0; i < MAX_USING_FINGER_NUM ; i++)
		fingerinfo[i].pressure = -1;

	LEAVE_FUNC;
	return 0;
}


static int qt602240_resume(struct platform_device *dev)
{
	int ret, i;	
	
	ENTER_FUNC;

	printk(KERN_INFO"\n[TSP][%s] \n", __func__);

	ret = write_power_config(power_config);
	if (ret != CFG_WRITE_OK) {
		/* "Power config write failed!\n" */
		for (i = 0; i < 10; i++) {
			printk(KERN_INFO"\n[TSP][ERROR] config error %d,  line : %d i %d\n", ret, __LINE__, i);
			msleep(20);

			ret = write_power_config(power_config);
			if (ret == CFG_WRITE_OK)
				break;
		}
		if (i == 10)
			return -1;
	}

	calibrate_chip();
	enable_irq(qt602240->client->irq);
	LEAVE_FUNC;
	return 0;

}
#endif


#ifdef CONFIG_HAS_EARLYSUSPEND
static void qt602240_early_suspend(struct early_suspend *h)
{
	struct input_dev *input_dev = qt602240->input_dev;
	gen_powerconfig_t7_config_t power_config_sleep = {0};
	int i = 0;

	ENTER_FUNC;
	printk(KERN_DEBUG "\n[TSP][%s] \n", __func__);

#ifdef USE_DELAYED_WORK
	flush_delayed_work(qt602240_wq);
#else
	flush_workqueue(qt602240_wq);
#endif

	if(qt_boot_download == 1) 
		return;

	mutex_lock(&input_dev->mutex);	
	disable_irq(qt602240->client->irq);

	power_config_sleep.idleacqint = 0;
	power_config_sleep.actvacqint = 0;
	power_config_sleep.actv2idleto = 0;
	
	/* Write power config to chip. */
	if (write_power_config(power_config_sleep) != CFG_WRITE_OK) {
		/* "Power config write failed!\n" */
		printk(KERN_INFO"\n[TSP][ERROR] line : %d\n", __LINE__);
	}
		
	for (i = 0; i < MAX_USING_FINGER_NUM; i++)
		fingerinfo[i].pressure = -1;

	qt_timer_state = 0;
	bCalibrationAndResetChip = false;
	
	printk(KERN_DEBUG "TS Power Off. \n");

	msleep(64);
	qt602240_pdata->power(0);
	mutex_unlock(&input_dev->mutex);

	LEAVE_FUNC;
}


static void qt602240_late_resume(struct early_suspend *h)
{
	struct input_dev *input_dev = qt602240->input_dev;
	int ret, i;
	
	ENTER_FUNC;
	
	printk(KERN_DEBUG "\n[TSP][%s] \n", __func__);
	printk(KERN_DEBUG "TS Power On. \n");

	mutex_lock(&input_dev->mutex);
	
	qt602240->power(1);
	msleep(100);
	
	if(qt_boot_download == 1) 	{
		printk(KERN_INFO"\n[TSP][warning] qt_boot_download \n");
		mutex_unlock(&input_dev->mutex);
		return ;
	}

	ret = write_power_config(power_config);
	
	if (ret != CFG_WRITE_OK) {
		/* "Power config write failed!\n" */
		for (i = 0; i < 10; i++) {
			printk(KERN_INFO"\n[TSP][ERROR] config error %d,  line : %d i %d\n", ret, __LINE__, i);
			msleep(64);

			ret = write_power_config(power_config);
			if (ret == CFG_WRITE_OK)
				break;
		}
		if (i >= 10)
			printk(KERN_INFO"\n[TSP][ERROR] ----- Touch config write Error!!! -------- \n");
	}

	cal_check_flag = 0;

	calibrate_chip();
	msleep(64);
	enable_irq(qt602240->client->irq);
	msleep(20);
	mutex_unlock(&input_dev->mutex);

	LEAVE_FUNC;
}
#endif	/* End of CONFIG_HAS_EARLYSUSPEND */


static struct i2c_device_id qt602240_idtable[] = {
	{ "qt602240_ts", 0 },
	{ }
};


MODULE_DEVICE_TABLE(i2c, qt602240_idtable);


struct i2c_driver qt602240_i2c_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name	= "qt602240_ts",
	},
	.id_table	= qt602240_idtable,
	.probe		= qt602240_probe,
	.remove		= __devexit_p(qt602240_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= qt602240_suspend,
	.resume		= qt602240_resume,
#endif /* CONFIG_HAS_EARLYSUSPEND */
};

struct class *sec_class;
struct device *ts_dev;

static ssize_t i2c_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	unsigned char read_buf[5];

	ret = qt602240_i2c_read(0, read_buf, 5);
	if (ret < 0)
		printk(KERN_INFO"qt602240 i2c read failed.\n");

	printk(KERN_INFO"qt602240 read: %x, %x, %x, %x, %x\n", read_buf[0], read_buf[1], read_buf[2], read_buf[3], read_buf[4]);

	return sprintf(buf, "%s\n", buf);
}

static ssize_t i2c_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	return size;
}

static ssize_t gpio_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO"qt602240 GPIO Status\n");
#ifdef CONFIG_MAINTOUCH_CE
	printk(KERN_INFO"TOUCH_EN  : %s\n", gpio_get_value(qt602240->ce_gpio)? "High":"Low");
#endif
	printk(KERN_INFO"TOUCH_INT : %s\n", gpio_get_value(qt602240->intr_gpio) ? "High" : "Low");

	return sprintf(buf, "%s\n", buf);
}

static ssize_t gpio_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
#ifdef CONFIG_MAINTOUCH_CE
	if(strncmp(buf, "ENHIGH", 6) == 0 || strncmp(buf, "enhigh", 6) == 0) {
		gpio_set_value(qt602240->ce_gpio, 1);
		printk("set TOUCH_EN High.\n");
		mdelay(100);
	}
	if(strncmp(buf, "ENLOW", 5) == 0 || strncmp(buf, "enlow", 5) == 0) {
		gpio_set_value(qt602240->ce_gpio, 0);
		printk("set TOUCH_EN Low.\n");
		mdelay(100);
	}
#endif
	return size;
}

static ssize_t firmware1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
/*
	char a, b;
	a = tsp_version & 0xf0;
	a = a >> 4;
	b = tsp_version & 0x0f;
	return sprintf(buf, "%d.%d", a, b);
*/
	return sprintf(buf,"%d",tsp_version);
}

static ssize_t firmware1_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	return size;
}

static ssize_t key_threshold_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", touchscreen_config.tchthr);
}

static ssize_t key_threshold_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int i;
	if (sscanf(buf, "%d", &i) == 1)	{
		touchscreen_config.tchthr = i;
		printk(KERN_INFO"threshold is changed to %d\n", i);
	} else
		printk(KERN_INFO"threshold write error\n");

	return 0;
}


#ifdef QT_FIRMUP_ENABLE

uint8_t boot_unlock(void)
{
	int ret;
	unsigned char data[2];

	data[0] = QT_UNLOCK_CMD_LSB;
	data[1] = QT_UNLOCK_CMD_MSB;

	ret = boot_qt602240_i2c_write(0, data, 2);
	if (ret < 0) {
		printk(KERN_INFO"%s : i2c write failed\n", __func__);
		return WRITE_MEM_FAILED;
	}

	return WRITE_MEM_OK;
}



uint8_t qt_boot(void)
{
	unsigned char boot_status;
	unsigned char boot_ver;
	unsigned char retry_cnt;
	unsigned long int character_position;
	unsigned long int data_size;
	unsigned int frame_size = 0;
	unsigned int next_frame;
	unsigned int crc_error_count;
	unsigned int size1, size2;
	unsigned int j, read_status_flag;
	uint8_t data = QT_BOOT_VALUE;


	uint8_t reset_result = 0;
	unsigned char  *firmware_data ;

	data_size = 0;

	firmware_data = QT602240_firmware_ptr;

	crc_error_count = 0;
	character_position = 0;

	next_frame = 0;

	reset_result = write_mem(command_processor_address + RESET_OFFSET, 1, &data);


	if (reset_result != WRITE_MEM_OK) {
		for (retry_cnt = 0; retry_cnt < 3; retry_cnt++) {
			mdelay(100);
			reset_result = write_mem(command_processor_address + RESET_OFFSET, 1, &data);
			if (reset_result == WRITE_MEM_OK)
				break;
		}

	}
	if (reset_result == WRITE_MEM_OK)
		printk(KERN_INFO"Boot reset OK\r\n");

	mdelay(100);

/*
	for(retry_cnt = 0; retry_cnt < 30; retry_cnt++)
*/
	{
		if ((boot_read_mem(0, 1, &boot_status) == READ_MEM_OK) &&
			(boot_status & QT_WAITING_BOOTLOAD_CMD) == QT_WAITING_BOOTLOAD_CMD) {

			boot_ver = boot_status & QT_BOOT_STATUS_MASK;
			crc_error_count = 0;
			character_position = 0;
			next_frame = 0;

			data_size = QT602240_firmware_size;

			while (character_position < data_size) {
				for (j = 0; j < 5; j++) {
					mdelay(60);
					if (boot_read_mem(0, 1, &boot_status) == READ_MEM_OK)	{
						read_status_flag = 1;
						break;
					} else
						read_status_flag = 0;
				}

				if (read_status_flag == 1) {

					retry_cnt  = 0;
					printk(KERN_INFO"TSP boot status is %x		stage 2 \n", boot_status);

					if ((boot_status & QT_WAITING_BOOTLOAD_CMD) == QT_WAITING_BOOTLOAD_CMD) {

						if (boot_unlock() == WRITE_MEM_OK) {
							mdelay(10);
							#ifdef TOUCH_LOG_ENABLE
								printk(KERN_INFO"Unlock OK\n");
							#endif
						} else {
							#ifdef TOUCH_LOG_ENABLE
								printk(KERN_INFO"Unlock fail\n");
							#endif
						}
					} else if ((boot_status & QT_WAITING_BOOTLOAD_CMD) == QT_WAITING_FRAME_DATA) {

						/* Add 2 to frame size, as the CRC bytes are not included */
						size1 =  *(firmware_data + character_position);
						size2 =  *(firmware_data + character_position + 1) + 2;
						frame_size = (size1 << 8) + size2;
						#ifdef TOUCH_LOG_ENABLE
							printk(KERN_INFO"Frame size:%d\n", frame_size);
							printk(KERN_INFO"Firmware pos:%lu\n", character_position);
						#endif
						/* Exit if frame data size is zero */
						if (0 == frame_size) {
							printk(KERN_INFO"0 == frame_size\n");
							return 1;
						}
						next_frame = 1;
						boot_write_mem(0, frame_size, (firmware_data + character_position));
						mdelay(10);
						printk(KERN_INFO".");
					} else if (boot_status == QT_FRAME_CRC_CHECK) {
						#ifdef TOUCH_LOG_ENABLE
							printk(KERN_INFO"CRC Check\n");
						#endif
					} else if (boot_status == QT_FRAME_CRC_PASS) {
						if (next_frame == 1) {
							#ifdef TOUCH_LOG_ENABLE
								printk(KERN_INFO"CRC Ok\n");
							#endif
							character_position += frame_size;
							next_frame = 0;
						} else
							printk(KERN_INFO"next_frame != 1\n");
					} else if (boot_status  == QT_FRAME_CRC_FAIL) {
						printk(KERN_INFO"CRC Fail\n");
						crc_error_count++;
					}
					if (crc_error_count > 10)
						return QT_FRAME_CRC_FAIL;
				} else
					return 0;
			}
		} else
			printk(KERN_INFO"[TSP] read_boot_state() or (boot_status & 0xC0) == 0xC0) is fail!!!\n");
	}

	printk(KERN_INFO"QT_Boot end \n");
	return 0;
}

uint8_t qt_boot_no_reset(void)
{
	unsigned char boot_status;
	unsigned char boot_ver;
	unsigned char retry_cnt;
	unsigned long int character_position;
	unsigned long int data_size;
	unsigned int frame_size = 0;
	unsigned int next_frame;
	unsigned int crc_error_count;
	unsigned int size1, size2;
	unsigned int j, read_status_flag;

	unsigned char  *firmware_data ;

	firmware_data = QT602240_firmware_ptr;

	crc_error_count = 0;
	character_position = 0;
	next_frame = 0;


/*
	for(retry_cnt = 0; retry_cnt < 30; retry_cnt++)
*/
	{
		if ((boot_read_mem(0, 1, &boot_status) == READ_MEM_OK) &&
			(boot_status & QT_WAITING_BOOTLOAD_CMD) == QT_WAITING_BOOTLOAD_CMD) {

			boot_ver = boot_status & QT_BOOT_STATUS_MASK;
			crc_error_count = 0;
			character_position = 0;
			next_frame = 0;

			data_size = QT602240_firmware_size;

			while (character_position < data_size) {
				for (j = 0; j < 5; j++) {
					mdelay(60);
					if (boot_read_mem(0, 1, &boot_status) == READ_MEM_OK)	{
						read_status_flag = 1;
						break;
					} else
						read_status_flag = 0;
				}

				if (read_status_flag == 1) {

					retry_cnt  = 0;
					printk(KERN_INFO"TSP boot status is %x		stage 2 \n", boot_status);

					if ((boot_status & QT_WAITING_BOOTLOAD_CMD) == QT_WAITING_BOOTLOAD_CMD) {

						if (boot_unlock() == WRITE_MEM_OK) {
							mdelay(10);
							#ifdef TOUCH_LOG_ENABLE
								printk(KERN_INFO"Unlock OK\n");
							#endif
						} else {
							#ifdef TOUCH_LOG_ENABLE
								printk(KERN_INFO"Unlock fail\n");
							#endif
						}
					} else if ((boot_status & QT_WAITING_BOOTLOAD_CMD) == QT_WAITING_FRAME_DATA) {

						/* Add 2 to frame size, as the CRC bytes are not included */
						size1 =  *(firmware_data + character_position);
						size2 =  *(firmware_data + character_position + 1) + 2;
						frame_size = (size1 << 8) + size2;
						#ifdef TOUCH_LOG_ENABLE
							printk(KERN_INFO"Frame size:%d\n", frame_size);
							printk(KERN_INFO"Firmware pos:%lu\n", character_position);
						#endif
						/* Exit if frame data size is zero */
						if (0 == frame_size) {
							printk(KERN_INFO"0 == frame_size\n");
							return 1;
						}
						next_frame = 1;
						boot_write_mem(0, frame_size, (firmware_data + character_position));
						mdelay(10);
						printk(KERN_INFO".");
					} else if (boot_status == QT_FRAME_CRC_CHECK) {
						#ifdef TOUCH_LOG_ENABLE
							printk(KERN_INFO"CRC Check\n");
						#endif
					} else if (boot_status == QT_FRAME_CRC_PASS) {
						if (next_frame == 1) {
							#ifdef TOUCH_LOG_ENABLE
								printk(KERN_INFO"CRC Ok\n");
							#endif
							character_position += frame_size;
							next_frame = 0;
						} else
							printk(KERN_INFO"next_frame != 1\n");
					} else if (boot_status  == QT_FRAME_CRC_FAIL) {
						printk(KERN_INFO"CRC Fail\n");
						crc_error_count++;
					}
					if (crc_error_count > 10)
						return QT_FRAME_CRC_FAIL;
				} else
					return 0;
			}
		} else
			printk(KERN_INFO"[TSP] read_boot_state() or (boot_status & 0xC0) == 0xC0) is fail!!!\n");
	}

	printk("QT_Boot_no_reset end \n");
	return 0;

}



void qt_reprogram(void)
{
	uint8_t version, build;
	unsigned char rxdata;

	printk(KERN_INFO"qt_reprogram check\n");

	if(boot_read_mem(0,1,&rxdata) == READ_MEM_OK)
	{
		printk("Enter to new firmware : boot mode\n");

 		qt_boot_download = 1;

        QT602240_firmware_ptr = &QT602240_firmware_2_0[0];
		QT602240_firmware_size = sizeof(QT602240_firmware_2_0);

        if(!qt_boot_no_reset())
		{
			mdelay(2000);
			hw_reset_chip();
		}
		qt_boot_download = 0;

        printk("Reprogram done : boot mode\n");
	}

	if(quantum_touch_probe() == -1)  /* find and initialise QT device */
    {
       printk(KERN_INFO"[TSP][ERROR] Touch device NOT found. Don't check firmware version\n");
       return;
    }

	get_version(&version);
	get_build_number(&build);

	if ((version < QT_LAST_FIRMWARE_VERSION) || (version == QT_LAST_FIRMWARE_VERSION && build < QT_LAST_BUILD_NUMBER)) {
		printk(KERN_INFO"Enter to new firmware (0x%02x,0x%02x) : ADDR = Other Version\n", QT_LAST_FIRMWARE_VERSION,QT_LAST_BUILD_NUMBER);

 		qt_boot_download = 1;

        QT602240_firmware_ptr = &QT602240_firmware_2_0[0];
		QT602240_firmware_size = sizeof(QT602240_firmware_2_0);

		if (!qt_boot())
		{
			int i;

			mdelay(2000);
			hw_reset_chip();

		    for(i=0; i<3; i++)
			{
				if(quantum_touch_probe() == 0)
				{
					break;
				}	
				else
				{
					printk(KERN_INFO"[QT]touch probe fail!!! retry %d\n",i);
					mdelay(2000);
					hw_reset_chip();
				}
			}
		}

		qt_boot_download = 0;

		printk(KERN_INFO"Reprogram done : ADDR = Other Version\n");
	}
}
#endif



/*------------------------------ for tunning ATmel - start ----------------------------*/

static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf)
{	/* v1.2 = 18 , v1.4 = 20 , v1.5 = 21 , v1.6 = 22 */
	uint8_t version, build;

	get_version(&version);
	get_build_number(&build);

	printk(KERN_DEBUG "[TSP] QT602240 Firmware Ver.\n");
	printk(KERN_DEBUG "[TSP] version = %x\n", version);
	printk(KERN_DEBUG "[TSP] Build = %x\n", build);

    return sprintf(buf, "0X%x", version);
}


static ssize_t firmware_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long after;
	int firmware_ret_val = -1;
	uint8_t version, build;
	unsigned long value;


	get_version(&version);
	get_build_number(&build);

	value = strict_strtoul(buf, 10, &after);
	printk(KERN_INFO "[TSP] %s\n", __func__);

	/* auto update. */
	if (value == 1)	{
		printk(KERN_DEBUG "[TSP] Firmware update start!!\n");
		printk(KERN_DEBUG "[TSP] version = 0x%x\n", tsp_version);

	if ((version < QT_LAST_FIRMWARE_VERSION) || (version == QT_LAST_FIRMWARE_VERSION && build < QT_LAST_BUILD_NUMBER)) {
		printk(KERN_INFO"Enter to new firmware (0x%02x,0x%02x) : ADDR = Other Version\n", QT_LAST_FIRMWARE_VERSION,QT_LAST_BUILD_NUMBER);

			wake_lock(&tsp_firmware_wake_lock);

			qt_boot_download = 1;

	        QT602240_firmware_ptr = &QT602240_firmware_2_0[0];
			QT602240_firmware_size = sizeof(QT602240_firmware_2_0);

			printk(KERN_DEBUG "Enter to new firmware : ADDR = Other Version\n");

			firmware_ret_val = qt_boot();

			mdelay(2000);
			hw_reset_chip();

			if (!firmware_ret_val)
			{
				int i;

			    for(i=0; i<3; i++)
				{
					if(quantum_touch_probe() == 0)
					{
						break;
					}	
					else
					{
						printk(KERN_INFO"[QT]touch probe fail!!! retry %d\n",i);
						mdelay(2000);
						hw_reset_chip();
					}
				}
			}

			qt_boot_download = 0;
	
			printk(KERN_DEBUG "Reprogram done : ADDR = Other Version\n");

			wake_unlock(&tsp_firmware_wake_lock);
		} else
			firmware_ret_val = 1;

		printk(KERN_DEBUG "[TSP] Firmware result = %d\n", firmware_ret_val);
	}

	return size;
}


static DEVICE_ATTR(gpio, S_IRUGO | S_IWUSR, gpio_show, gpio_store);
static DEVICE_ATTR(i2c, S_IRUGO | S_IWUSR, i2c_show, i2c_store);
static DEVICE_ATTR(firmware, S_IRUGO | S_IWUSR, firmware_show, firmware_store);
static DEVICE_ATTR(firmware1, S_IRUGO | S_IWUSR, firmware1_show, firmware1_store);
static DEVICE_ATTR(key_threshold, S_IRUGO | S_IWUSR, key_threshold_show, key_threshold_store);


static ssize_t set_power_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "[TSP] %s : operate nothing\n", __func__);

	return 0;
}

static ssize_t set_power_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int cmd_no, config_value = 0;
	unsigned long after;

	unsigned long value = strict_strtoul(buf, 10, &after);

	printk(KERN_INFO "[TSP] %s\n", __func__);
	cmd_no = (int)(value / 1000);
	config_value = (int)(value % 1000);

	if (cmd_no == 0) {
		printk(KERN_INFO"[%s] CMD 0 , power_config.idleacqint = %d\n", __func__, config_value);
		power_config.idleacqint = config_value;
	} else if (cmd_no == 1) {
		printk(KERN_INFO"[%s] CMD 1 , power_config.actvacqint = %d\n", __func__, config_value);
		power_config.actvacqint = config_value;
	} else if (cmd_no == 2) {
		printk(KERN_INFO"[%s] CMD 2 , power_config.actv2idleto= %d\n", __func__, config_value);
		power_config.actv2idleto = config_value;
	} else
		printk(KERN_INFO"[%s] unknown CMD\n", __func__);

	if (write_power_config(power_config) != CFG_WRITE_OK)
		QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);

	return size;
}
static DEVICE_ATTR(set_power, S_IRUGO | S_IWUSR | /*S_IWOTH |*/ S_IXOTH, set_power_show, set_power_store);

static ssize_t set_acquisition_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "[TSP] %s : operate nothing\n", __func__);

	return 0;
}

static ssize_t set_acquisition_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int cmd_no, config_value = 0;
	unsigned long after;

	unsigned long value = strict_strtoul(buf, 10, &after);
	printk(KERN_INFO "[TSP] %s\n", __func__);
	cmd_no = (int)(value / 1000);
	config_value = (int)(value % 1000);

	if (cmd_no == 0) {
		printk(KERN_INFO"[%s] CMD 0 , acquisition_config.chrgtime = %d\n", __func__, config_value);
		acquisition_config.chrgtime = config_value;
	} else if (cmd_no == 1) {
		printk(KERN_INFO"[%s] CMD 1 , acquisition_config.tchdrift = %d\n", __func__, config_value);
		acquisition_config.tchdrift = config_value;
	} else if (cmd_no == 2) {
		printk(KERN_INFO"[%s] CMD 2 , acquisition_config.driftst = %d\n", __func__, config_value);
		acquisition_config.driftst = config_value;
	} else if (cmd_no == 3) {
		printk(KERN_INFO"[%s] CMD 3 , acquisition_config.tchautocal = %d\n", __func__, config_value);
		acquisition_config.tchautocal = config_value;
	} else if (cmd_no == 4) {
		printk(KERN_INFO"[%s] CMD 4 , acquisition_config.sync = %d\n", __func__, config_value);
		acquisition_config.sync = config_value;
	} else if (cmd_no == 5) {
		printk(KERN_INFO"[%s] CMD 5 , acquisition_config.atchcalst = %d\n", __func__, config_value);
		acquisition_config.atchcalst = config_value;
	} else if (cmd_no == 6) {
		printk(KERN_INFO"[%s] CMD 6 , acquisition_config.atchcalsthr = %d\n", __func__, config_value);
		acquisition_config.atchcalsthr = config_value;
	} else
		printk(KERN_INFO"[%s] unknown CMD\n", __func__);

	if (write_acquisition_config(acquisition_config) != CFG_WRITE_OK)
		QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);

	return size;
}

static DEVICE_ATTR(set_acquisition, S_IRUGO | S_IWUSR | /*S_IWOTH |*/ S_IXOTH, set_acquisition_show, set_acquisition_store);


static ssize_t set_touchscreen_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "[TSP] %s : operate nothing\n", __func__);

	return 0;
}

static ssize_t set_touchscreen_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int  cmd_no, config_value = 0;
	unsigned long after;

	unsigned long value = strict_strtoul(buf, 10, &after);
	printk(KERN_INFO "[TSP] %s\n", __func__);
	cmd_no = (int)(value / 1000);
	config_value = (int)(value % 1000);

	if (cmd_no == 0) {
		printk(KERN_INFO"[%s] CMD 0 , touchscreen_config.ctrl = %d\n", __func__, config_value);
		touchscreen_config.ctrl = config_value;
	} else if (cmd_no == 1) {
		printk(KERN_INFO"[%s] CMD 1 , touchscreen_config.xorigin  = %d\n", __func__, config_value);
		touchscreen_config.xorigin = config_value;
	} else if (cmd_no == 2) {
		printk(KERN_INFO"[%s] CMD 2 , touchscreen_config.yorigin = %d\n", __func__, config_value);
		touchscreen_config.yorigin = config_value;
	} else if (cmd_no == 3) {
		printk(KERN_INFO"[%s] CMD 3 , touchscreen_config.xsize = %d\n", __func__, config_value);
		touchscreen_config.xsize = config_value;
	} else if (cmd_no == 4) {
		printk(KERN_INFO"[%s] CMD 4 , touchscreen_config.ysize = %d\n", __func__, config_value);
		touchscreen_config.ysize = config_value;
	} else if (cmd_no == 5) {
		printk(KERN_INFO"[%s] CMD 5 , touchscreen_config.akscfg = %d\n", __func__, config_value);
		touchscreen_config.akscfg = config_value;
	} else if (cmd_no == 6) {
		printk(KERN_INFO"[%s] CMD 6 , touchscreen_config.blen = %d\n", __func__, config_value);
		touchscreen_config.blen = config_value;
	} else if (cmd_no == 7) {
		printk(KERN_INFO"[%s] CMD 7 , touchscreen_config.tchthr = %d\n", __func__, config_value);
		touchscreen_config.tchthr = config_value;
	} else if (cmd_no == 8) {
		printk(KERN_INFO"[%s] CMD 8 , touchscreen_config.tchdi = %d\n", __func__, config_value);
		touchscreen_config.tchdi = config_value;
	} else if (cmd_no == 9) {
		printk(KERN_INFO"[%s] CMD 9 , touchscreen_config.orient = %d\n", __func__, config_value);
		touchscreen_config.orient = config_value;
	} else if (cmd_no == 10) {
		printk(KERN_INFO"[%s] CMD 10 , touchscreen_config.mrgtimeout = %d\n", __func__, config_value);
		touchscreen_config.mrgtimeout = config_value;
	} else if (cmd_no == 11) {
		printk(KERN_INFO"[%s] CMD 11 , touchscreen_config.movhysti = %d\n", __func__, config_value);
		touchscreen_config.movhysti = config_value;
	} else if (cmd_no == 12) {
		printk(KERN_INFO"[%s] CMD 12 , touchscreen_config.movhystn = %d\n", __func__, config_value);
		touchscreen_config.movhystn = config_value;
	} else if (cmd_no == 13) {
		printk(KERN_INFO"[%s] CMD 13 , touchscreen_config.movfilter = %d\n", __func__, config_value);
		touchscreen_config.movfilter = config_value;
	} else if (cmd_no == 14) {
		printk(KERN_INFO"[%s] CMD 14 , touchscreen_config.numtouch = %d\n", __func__, config_value);
		touchscreen_config.numtouch = config_value;
	} else if (cmd_no == 15) {
		printk(KERN_INFO"[%s] CMD 15 , touchscreen_config.mrghyst = %d\n", __func__, config_value);
		touchscreen_config.mrghyst = config_value;
	} else if (cmd_no == 16) {
		printk(KERN_INFO"[%s] CMD 16 , touchscreen_config.mrgthr = %d\n", __func__, config_value);
		touchscreen_config.mrgthr = config_value;
	} else if (cmd_no == 17) {
		printk(KERN_INFO"[%s] CMD 17 , touchscreen_config.tchamphyst = %d\n", __func__, config_value);
		touchscreen_config.amphyst = config_value;
	} else if (cmd_no == 18) {
		printk(KERN_INFO"[%s] CMD 18 , touchscreen_config.xrange = %d\n", __func__, config_value);
		touchscreen_config.xrange = config_value;
	} else if (cmd_no == 19) {
		printk(KERN_INFO"[%s] CMD 19 , touchscreen_config.yrange = %d\n", __func__, config_value);
		touchscreen_config.yrange = config_value;
	} else if (cmd_no == 20) {
		printk(KERN_INFO"[%s] CMD 20 , touchscreen_config.xloclip = %d\n", __func__, config_value);
		touchscreen_config.xloclip = config_value;
	} else if (cmd_no == 21) {
		printk(KERN_INFO"[%s] CMD 21 , touchscreen_config.xhiclip = %d\n", __func__, config_value);
		touchscreen_config.xhiclip = config_value;
	} else if (cmd_no == 22) {
		printk(KERN_INFO"[%s] CMD 22 , touchscreen_config.yloclip = %d\n", __func__, config_value);
		touchscreen_config.yloclip = config_value;
	} else if (cmd_no == 23) {
		printk(KERN_INFO"[%s] CMD 23 , touchscreen_config.yhiclip = %d\n", __func__, config_value);
		touchscreen_config.yhiclip = config_value;
	} else if (cmd_no == 24) {
		printk(KERN_INFO"[%s] CMD 24 , touchscreen_config.xedgectrl  = %d\n", __func__, config_value);
		touchscreen_config.xedgectrl = config_value;
	} else if (cmd_no == 25) {
		printk(KERN_INFO"[%s] CMD 25 , touchscreen_config.xedgedist   = %d\n", __func__, config_value);
		touchscreen_config.xedgedist = config_value;
	} else if (cmd_no == 26) {
		printk(KERN_INFO"[%s] CMD 26 , touchscreen_config.yedgectrl    = %d\n", __func__, config_value);
		touchscreen_config.yedgectrl = config_value;
	} else if (cmd_no == 27) {
		printk(KERN_INFO"[%s] CMD 27 , touchscreen_config.yedgedist     = %d\n", __func__, config_value);
		touchscreen_config.yedgedist = config_value;
	} else if (cmd_no == 28) {
		if (tsp_version >= 0x16) {
			printk(KERN_INFO"[%s] CMD 28 , touchscreen_config.jumplimit      = %d\n", __func__, config_value);
			touchscreen_config.jumplimit = config_value;
		} else
			printk(KERN_INFO"[%s] CMD 28 , touchscreen_config.jumplimit  is not supported in this version.\n", __func__);
	} else
		printk(KERN_INFO"[%s] unknown CMD\n", __func__);

	if (write_multitouchscreen_config(0, touchscreen_config) != CFG_WRITE_OK)
		QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);

	return size;
}
static DEVICE_ATTR(set_touchscreen, S_IRUGO | S_IWUSR | /*S_IWOTH |*/ S_IXOTH, set_touchscreen_show, set_touchscreen_store);

static unsigned int qt602240_init_fail;

static ssize_t set_qt_init_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned int i2c_state;

	printk(KERN_INFO"[TSP] %s : operate nothing\n", __func__);

	i2c_state = qt602240_init_fail;

	return sprintf(buf, "%u\n", i2c_state);
}
static DEVICE_ATTR(set_qt_init_state, S_IRUGO | S_IWUSR | /*S_IWOTH |*/ S_IXOTH, set_qt_init_state_show, NULL);


static ssize_t set_noise_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "[TSP] %s : operate nothing\n", __func__);

	return 0;
}
static ssize_t set_noise_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int cmd_no, config_value = 0;
	unsigned long after;

	unsigned long value = strict_strtoul(buf, 10, &after);
	printk(KERN_INFO "[TSP] %s\n", __func__);

	if (value < 10000) {
		cmd_no = (int)(value / 1000);
		config_value = (int)(value % 1000);
	} else {
		cmd_no = (int)(value / 100000);
		config_value = (int)(value % 100000);
	}

	if (cmd_no == 0) {
		printk(KERN_INFO"[%s] CMD 0 , noise_suppression_config.ctrl = %d\n", __func__, config_value);
		noise_suppression_config.ctrl = config_value;
	} else if (cmd_no == 1) {
		printk(KERN_INFO"[%s] CMD 1 , noise_suppression_config.reserved = %d\n", __func__, config_value);
	#if defined(__VER_1_2__)
		noise_suppression_config.outflen = config_value;
	#elif defined(__VER_1_4__)
		noise_suppression_config.reserved = config_value;
	#endif
	} else if (cmd_no == 2) {
		printk(KERN_INFO"[%s] CMD 2 , noise_suppression_config.reserved1 = %d\n", __func__, config_value);
		noise_suppression_config.reserved1 = config_value;
	} else if (cmd_no == 3) {
		printk(KERN_INFO"[%s] CMD 3 , noise_suppression_config.gcaful = %d\n", __func__, config_value);
		noise_suppression_config.gcaful = config_value;
	} else if (cmd_no == 4) {
		printk(KERN_INFO"[%s] CMD 4 , noise_suppression_config.gcafll = %d\n", __func__, config_value);
		noise_suppression_config.gcafll = config_value;
	} else if (cmd_no == 5) {
		printk(KERN_INFO"[%s] CMD 5 , noise_suppression_config.actvgcafvalid = %d\n", __func__, config_value);
	#if defined(__VER_1_4__)
		noise_suppression_config.actvgcafvalid = config_value;
	#endif
	} else if (cmd_no == 6) {
		printk(KERN_INFO"[%s] CMD 6 , noise_suppression_config.noisethr = %d\n", __func__, config_value);
		noise_suppression_config.noisethr = config_value;
	} else if (cmd_no == 7) {
		printk(KERN_INFO"[%s] CMD 7 , noise_suppression_config.freqhopscale = %d\n", __func__, config_value);
		noise_suppression_config.freqhopscale = config_value;
	} else if (cmd_no == 8) {
		printk(KERN_INFO"[%s] CMD 8 , noise_suppression_config.freq[0] = %d\n", __func__, config_value);
		noise_suppression_config.freq[0] = config_value;
	} else if (cmd_no == 9) {
		printk(KERN_INFO"[%s] CMD 9 , noise_suppression_config.freq[1] = %d\n", __func__, config_value);
		noise_suppression_config.freq[1] = config_value;
	} else if (cmd_no == 10) {
		printk(KERN_INFO"[%s] CMD 10 , noise_suppression_config.freq[2] = %d\n", __func__, config_value);
		noise_suppression_config.freq[2] = config_value;
	} else if (cmd_no == 11) {
		printk(KERN_INFO"[%s] CMD 11 , noise_suppression_config.freq[3] = %d\n", __func__, config_value);
		noise_suppression_config.freq[3] = config_value;
	} else if (cmd_no == 12) {
		printk(KERN_INFO"[%s] CMD 12 , noise_suppression_config.freq[4] = %d\n", __func__, config_value);
		noise_suppression_config.freq[4] = config_value;
	} else if (cmd_no == 13) {
		printk(KERN_INFO"[%s] CMD 13 , noise_suppression_config.idlegcafvalid = %d\n", __func__, config_value);
		noise_suppression_config.idlegcafvalid = config_value;
	} else
		printk(KERN_INFO"[%s] unknown CMD\n", __func__);

    /* Write Noise suppression config to chip. */
	if (get_object_address(PROCG_NOISESUPPRESSION_T22, 0) != OBJECT_NOT_FOUND) {
		if (write_noisesuppression_config(0, noise_suppression_config) != CFG_WRITE_OK)
			QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
    }


    /* Write grip suppression config to chip. */
	if (get_object_address(PROCI_GRIPFACESUPPRESSION_T20, 0) != OBJECT_NOT_FOUND) {
		if (write_gripsuppression_config(0, gripfacesuppression_config) != CFG_WRITE_OK)
			QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
    }

	return size;
}
static DEVICE_ATTR(set_noise, S_IRUGO | S_IWUSR | /*S_IWOTH |*/ S_IXOTH, set_noise_show, set_noise_store);

static ssize_t set_grip_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "[TSP] %s : operate nothing\n", __func__);

	return 0;
}
static ssize_t set_grip_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int cmd_no, config_value = 0;
	unsigned long after;

	unsigned long value = strict_strtoul(buf, 10, &after);
	printk(KERN_INFO "[TSP] %s\n", __func__);
	cmd_no = (int)(value / 1000);
	config_value = (int)(value % 1000);

	if (cmd_no == 0) {
		printk(KERN_INFO"[%s] CMD 0 , gripfacesuppression_config.ctrl = %d\n", __func__, config_value);
		gripfacesuppression_config.ctrl = config_value;
	} else if (cmd_no == 1) {
		printk(KERN_INFO"[%s] CMD 1 , gripfacesuppression_config.xlogrip = %d\n", __func__, config_value);
		gripfacesuppression_config.xlogrip = config_value;
	} else if (cmd_no == 2) {
		printk(KERN_INFO"[%s] CMD 2 , gripfacesuppression_config.xhigrip is not set = %d\n", __func__, config_value);
		gripfacesuppression_config.xhigrip = config_value;
	} else if (cmd_no == 3) {
		printk(KERN_INFO"[%s] CMD 3 , gripfacesuppression_config.ylogrip = %d\n", __func__, config_value);
		gripfacesuppression_config.ylogrip = config_value;
	} else if (cmd_no == 4) {
		printk(KERN_INFO"[%s] CMD 4 , gripfacesuppression_config.yhigrip = %d\n", __func__, config_value);
		gripfacesuppression_config.yhigrip =  config_value;
	} else if (cmd_no == 5) {
		printk(KERN_INFO"[%s] CMD 5 , gripfacesuppression_config.maxtchs = %d\n", __func__, config_value);
		gripfacesuppression_config.maxtchs = config_value;
	} else if (cmd_no == 6) {
		printk(KERN_INFO"[%s] CMD 6 , gripfacesuppression_config.reserved = %d\n", __func__, config_value);
		gripfacesuppression_config.reserved = config_value;
	} else if (cmd_no == 7) {
		printk(KERN_INFO"[%s] CMD 7 , gripfacesuppression_config.szthr1 = %d\n", __func__, config_value);
		gripfacesuppression_config.szthr1 = config_value;
	} else if (cmd_no == 8) {
		printk(KERN_INFO"[%s] CMD 8 , gripfacesuppression_config.szthr2 = %d\n", __func__, config_value);
		gripfacesuppression_config.szthr2 = config_value;
	} else if (cmd_no == 9) {
		printk(KERN_INFO"[%s] CMD 9 , gripfacesuppression_config.shpthr1 = %d\n", __func__, config_value);
		gripfacesuppression_config.shpthr1 = config_value;
	} else if (cmd_no == 10) {
		printk(KERN_INFO"[%s] CMD 10 , gripfacesuppression_config.shpthr2 = %d\n", __func__, config_value);
		gripfacesuppression_config.shpthr2 = config_value;
	} else
		printk(KERN_INFO"[%s] unknown CMD\n", __func__);

    /* Write grip suppression config to chip. */
	if (get_object_address(PROCI_GRIPFACESUPPRESSION_T20, 0) != OBJECT_NOT_FOUND) {
		if (write_gripsuppression_config(0, gripfacesuppression_config) != CFG_WRITE_OK)
			QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
    }

	return size;
}

static DEVICE_ATTR(set_grip, S_IRUGO | S_IWUSR | /*S_IWOTH |*/ S_IXOTH, set_grip_show, set_grip_store);

static ssize_t set_total_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "[TSP] %s : operate nothing\n", __func__);

	return 0;
}

static ssize_t set_total_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int cmd_no, config_value = 0;
	unsigned long after;

	unsigned long value = strict_strtoul(buf, 10, &after);
	printk(KERN_INFO "[TSP] %s\n", __func__);
	cmd_no = (int)(value / 1000);
	config_value = (int)(value % 1000);

#ifndef __VER_2_0__
	if (cmd_no == 0) {
		printk(KERN_INFO"[%s] CMD 0 , twotouch_gesture_config.ctrl = %d\n", __func__, config_value);
		twotouch_gesture_config.ctrl = config_value;
	} else if (cmd_no == 1) {
#else
	if (cmd_no == 1) {
#endif
		printk(KERN_INFO"[%s] CMD 1 , onetouch_gesture_config.ctrl = %d\n", __func__, config_value);
		onetouch_gesture_config.ctrl = config_value;
	} else if (cmd_no == 2) {
		printk(KERN_INFO"[%s] CMD 2 , noise_suppression_config.ctrl is not set = %d\n", __func__, config_value);
		noise_suppression_config.ctrl = config_value;
	} else if (cmd_no == 3) {
		printk(KERN_INFO"[%s] CMD 3 , selftest_config.ctrl = %d\n", __func__, config_value);
		selftest_config.ctrl = config_value;
	} else if (cmd_no == 4) {
		printk(KERN_INFO"[%s] CMD 4 , gripfacesuppression_config.ctrl = %d\n", __func__, config_value);
		gripfacesuppression_config.ctrl = config_value;
	} else if (cmd_no == 5) {
		printk(KERN_INFO"[%s] CMD 5 , cte_config.ctrl = %d\n", __func__, config_value);
		cte_config.ctrl = config_value;
	} else if (cmd_no == 6) {
		printk(KERN_INFO"[%s] CMD 6 , cte_config.idlegcafdepth= %d\n", __func__, config_value);
		cte_config.idlegcafdepth = config_value;
	} else if (cmd_no == 7) {
		printk(KERN_INFO"[%s] CMD 7 , cte_config.actvgcafdepth = %d\n", __func__, config_value);
		cte_config.actvgcafdepth = config_value;
	} else
		printk(KERN_INFO"[%s] unknown CMD\n", __func__);

	return size;
}
static DEVICE_ATTR(set_total, S_IRUGO | S_IWUSR | /*S_IWOTH |*/ S_IXOTH, set_total_show, set_total_store);

static ssize_t set_write_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	printk(KERN_INFO"power_config.idleacqint = %d\n", power_config.idleacqint);
	printk(KERN_INFO"power_config.actvacqint = %d\n", power_config.actvacqint);
	printk(KERN_INFO"power_config.actv2idleto= %d\n", power_config.actv2idleto);

	if (write_power_config(power_config) != CFG_WRITE_OK) {
		/* "Power config write failed!\n" */
		printk(KERN_INFO"\n[TSP][ERROR] config erroe line : %d\n", __LINE__);
		return CFG_WRITE_FAILED;
	}

	printk(KERN_INFO"acquisition_config.chrgtime = %d\n", acquisition_config.chrgtime);
	printk(KERN_INFO"acquisition_config.tchdrift = %d\n", acquisition_config.tchdrift);
	printk(KERN_INFO"acquisition_config.driftst = %d\n", acquisition_config.driftst);
	printk(KERN_INFO"acquisition_config.tchautocal = %d\n", acquisition_config.tchautocal);
	printk(KERN_INFO"acquisition_config.sync = %d\n", acquisition_config.sync);
	printk(KERN_INFO"acquisition_config.atchcalst = %d\n", acquisition_config.atchcalst);
	printk(KERN_INFO"acquisition_config.atchcalsthr = %d\n", acquisition_config.atchcalsthr);

	/* Write acquisition config to chip. */
	if (write_acquisition_config(acquisition_config) != CFG_WRITE_OK) {
		/* "Acquisition config write failed!\n" */
		printk(KERN_INFO"\n[TSP][ERROR] line : %d\n", __LINE__);
		return CFG_WRITE_FAILED;
	}

	printk(KERN_INFO"0 , touchscreen_config.ctrl = %d\n",  touchscreen_config.ctrl);
	printk(KERN_INFO"1 , touchscreen_config.xorigin  = %d\n", touchscreen_config.xorigin);
	printk(KERN_INFO"2 , touchscreen_config.yorigin = %d\n",  touchscreen_config.yorigin);
	printk(KERN_INFO"3 , touchscreen_config.xsize = %d\n", touchscreen_config.xsize);
	printk(KERN_INFO"4 , touchscreen_config.ysize = %d\n", touchscreen_config.ysize);
	printk(KERN_INFO"5 , touchscreen_config.akscfg = %d\n", touchscreen_config.akscfg);
	printk(KERN_INFO"6 , touchscreen_config.blen = %d\n", touchscreen_config.blen);
	printk(KERN_INFO"7 , touchscreen_config.tchthr = %d\n",  touchscreen_config.tchthr);
	printk(KERN_INFO"8 , touchscreen_config.tchdi = %d\n", touchscreen_config.tchdi);
	printk(KERN_INFO"9 , touchscreen_config.orient = %d\n", touchscreen_config.orient);
	printk(KERN_INFO"10 , touchscreen_config.mrgtimeout = %d\n", touchscreen_config.mrgtimeout);
	printk(KERN_INFO"11 , touchscreen_config.movhysti = %d\n", touchscreen_config.movhysti);
	printk(KERN_INFO"12 , touchscreen_config.movhystn = %d\n", touchscreen_config.movhystn);
	printk(KERN_INFO"13 , touchscreen_config.movfilter = %d\n", touchscreen_config.movfilter);
	printk(KERN_INFO"14 , touchscreen_config.numtouch = %d\n", touchscreen_config.numtouch);
	printk(KERN_INFO"15 , touchscreen_config.mrghyst = %d\n", touchscreen_config.mrghyst);
	printk(KERN_INFO"16 , touchscreen_config.mrgthr = %d\n", touchscreen_config.mrgthr);
	printk(KERN_INFO"17 , touchscreen_config.amphyst = %d\n", touchscreen_config.amphyst);
	printk(KERN_INFO"18 , touchscreen_config.xrange = %d\n", touchscreen_config.xrange);
	printk(KERN_INFO"19 , touchscreen_config.yrange = %d\n", touchscreen_config.yrange);
	printk(KERN_INFO"20 , touchscreen_config.xloclip = %d\n", touchscreen_config.xloclip);
	printk(KERN_INFO"21 , touchscreen_config.xhiclip = %d\n", touchscreen_config.xhiclip);
	printk(KERN_INFO"22 , touchscreen_config.yloclip = %d\n", touchscreen_config.yloclip);
	printk(KERN_INFO"23 , touchscreen_config.yhiclip = %d\n", touchscreen_config.yhiclip);
	printk(KERN_INFO"24 , touchscreen_config.xedgectrl  = %d\n", touchscreen_config.xedgectrl);
	printk(KERN_INFO"25 , touchscreen_config.xedgedist  = %d\n", touchscreen_config.xedgedist);
	printk(KERN_INFO"26 , touchscreen_config.yedgectrl  = %d\n", touchscreen_config.yedgectrl);
	printk(KERN_INFO"27 , touchscreen_config.yedgedist  = %d\n", touchscreen_config.yedgedist);
	if (tsp_version >= 0x16)
		printk(KERN_INFO"28 , touchscreen_config.jumplimit  = %d\n", touchscreen_config.jumplimit);
	else
		printk(KERN_INFO"28 , touchscreen_config.jumplimit is not supported in this version.\n");

	/* Write touchscreen (1st instance) config to chip. */
	if (write_multitouchscreen_config(0, touchscreen_config) != CFG_WRITE_OK) {
		/* "Multitouch screen config write failed!\n" */
		printk(KERN_INFO"\n[TSP][ERROR] line : %d\n", __LINE__);
		return CFG_WRITE_FAILED;
	}

	printk(KERN_INFO"0 , keyarray_config.ctrl = %d\n", keyarray_config.ctrl);
	printk(KERN_INFO"1 , keyarray_config.xorigin = %d\n", keyarray_config.xorigin);
	printk(KERN_INFO"2 , keyarray_config.xsize = %d\n", keyarray_config.xsize);
	printk(KERN_INFO"3 , keyarray_config.yorigin = %d\n", keyarray_config.yorigin);
	printk(KERN_INFO"4 ,	keyarray_config.ysize = %d\n", keyarray_config.ysize);
	printk(KERN_INFO"5 ,	keyarray_config.akscfg = %d\n", keyarray_config.akscfg);
	printk(KERN_INFO"6 ,	keyarray_config.blen = %d\n", keyarray_config.blen);
	printk(KERN_INFO"7 ,	keyarray_config.tchthr = %d\n", keyarray_config.tchthr);
	printk(KERN_INFO"8 ,	keyarray_config.tchdi = %d\n", keyarray_config.tchdi);

	/* Write key array (1st instance) config to chip. */
	if (write_keyarray_config(0, keyarray_config) != CFG_WRITE_OK) {
		/* "Key array config write failed!\n" */
		printk(KERN_INFO"\n[TSP][ERROR] line : %d\n", __LINE__);
		return CFG_WRITE_FAILED;
	}

#ifndef __VER_2_0__
	printk(KERN_INFO"0 , twotouch_gesture_config.ctrl = %d\n", twotouch_gesture_config.ctrl);
#endif
	printk(KERN_INFO"1 , onetouch_gesture_config.ctrl = %d\n", onetouch_gesture_config.ctrl);
	printk(KERN_INFO"2 , noise_suppression_config.ctrl is not set = %d\n", noise_suppression_config.ctrl);
	printk(KERN_INFO"3 ,	selftest_config.ctrl is not set = %d\n", selftest_config.ctrl);
	printk(KERN_INFO"4 ,	gripfacesuppression_config.ctrl = %d\n", gripfacesuppression_config.ctrl);
	printk(KERN_INFO"5 ,	cte_config.ctrl = %d\n", cte_config.ctrl);
	printk(KERN_INFO"6 ,	cte_config.idlegcafdepth = %d\n", cte_config.idlegcafdepth);
	printk(KERN_INFO"7 ,	cte_config.actvgcafdepth = %d\n", cte_config.actvgcafdepth);

	/* Write twotouch gesture config to chip. */
#ifndef __VER_2_0__
	if (get_object_address(PROCI_TWOTOUCHGESTUREPROCESSOR_T27, 0) != OBJECT_NOT_FOUND) {
		if (write_twotouchgesture_config(0, twotouch_gesture_config) != CFG_WRITE_OK) {
			/* "Two touch gesture config write failed!\n" */
			printk(KERN_INFO"\n[TSP][ERROR] line : %d\n", __LINE__);
			return CFG_WRITE_FAILED;
		}
	}
#endif

	/* Write one touch gesture config to chip. */
	if (get_object_address(PROCI_ONETOUCHGESTUREPROCESSOR_T24, 0) != OBJECT_NOT_FOUND) {
		if (write_onetouchgesture_config(0, onetouch_gesture_config) != CFG_WRITE_OK) {
			/* "One touch gesture config write failed!\n" */
			printk(KERN_INFO"\n[TSP][ERROR] line : %d\n", __LINE__);
			return CFG_WRITE_FAILED;
		}
	}

	printk(KERN_INFO"gripfacesuppression_config.ctrl  = %d\n", gripfacesuppression_config.ctrl);
	printk(KERN_INFO"gripfacesuppression_config.xlogrip  = %d\n", gripfacesuppression_config.xlogrip);
	printk(KERN_INFO"gripfacesuppression_config.xhigrip  = %d\n", gripfacesuppression_config.xhigrip);
	printk(KERN_INFO"gripfacesuppression_config.ylogrip  = %d\n", gripfacesuppression_config.ylogrip);
	printk(KERN_INFO"gripfacesuppression_config.yhigrip  = %d\n", gripfacesuppression_config.yhigrip);
	printk(KERN_INFO"gripfacesuppression_config.maxtchs  = %d\n", gripfacesuppression_config.maxtchs);
	printk(KERN_INFO"gripfacesuppression_config.reserved  = %d\n", gripfacesuppression_config.reserved);
	printk(KERN_INFO"gripfacesuppression_config.szthr1   = %d\n", gripfacesuppression_config.szthr1);
	printk(KERN_INFO"gripfacesuppression_config.szthr2   = %d\n", gripfacesuppression_config.szthr2);
	printk(KERN_INFO"gripfacesuppression_config.shpthr1   = %d\n", gripfacesuppression_config.shpthr1);
	printk(KERN_INFO"gripfacesuppression_config.shpthr2   = %d\n", gripfacesuppression_config.shpthr2);

	/* Write grip suppression config to chip. */
	if (get_object_address(PROCI_GRIPFACESUPPRESSION_T20, 0) != OBJECT_NOT_FOUND) {
		if (write_gripsuppression_config(0, gripfacesuppression_config) != CFG_WRITE_OK) {
			/* "Grip suppression config write failed!\n" */
			printk(KERN_INFO"\n[TSP][ERROR] line : %d\n", __LINE__);
			return CFG_WRITE_FAILED;
		}
	}

	printk(KERN_INFO"noise_suppression_config.ctrl   = %d\n", noise_suppression_config.ctrl);
	printk(KERN_INFO"noise_suppression_config.reserved   = %d\n", noise_suppression_config.reserved);
	printk(KERN_INFO"noise_suppression_config.reserved1   = %d\n", noise_suppression_config.reserved1);
	printk(KERN_INFO"noise_suppression_config.gcaful   = %d\n", noise_suppression_config.gcaful);
	printk(KERN_INFO"noise_suppression_config.gcafll   = %d\n", noise_suppression_config.gcafll);
	printk(KERN_INFO"noise_suppression_config.actvgcafvalid   = %d\n", noise_suppression_config.actvgcafvalid);
	printk(KERN_INFO"noise_suppression_config.noisethr   = %d\n", noise_suppression_config.noisethr);
	printk(KERN_INFO"noise_suppression_config.freqhopscale    = %d\n", noise_suppression_config.freqhopscale);
	printk(KERN_INFO"noise_suppression_config.freq[0]   = %d\n", noise_suppression_config.freq[0]);
	printk(KERN_INFO"noise_suppression_config.freq[1]   = %d\n", noise_suppression_config.freq[1]);
	printk(KERN_INFO"noise_suppression_config.freq[2]   = %d\n", noise_suppression_config.freq[2]);
	printk(KERN_INFO"noise_suppression_config.freq[3]   = %d\n", noise_suppression_config.freq[3]);
	printk(KERN_INFO"noise_suppression_config.freq[4]   = %d\n", noise_suppression_config.freq[4]);
	printk(KERN_INFO"noise_suppression_config.idlegcafvalid  = %d\n", noise_suppression_config.idlegcafvalid);

    /* Write Noise suppression config to chip. */
	if (get_object_address(PROCG_NOISESUPPRESSION_T22, 0) != OBJECT_NOT_FOUND) {
		if (write_noisesuppression_config(0, noise_suppression_config) != CFG_WRITE_OK)
			QT_printf("[TSP] Configuration Fail!!! , Line %d \n\r", __LINE__);
    }


	/* Write CTE config to chip. */
	if (get_object_address(SPT_CTECONFIG_T28, 0) != OBJECT_NOT_FOUND) {
		if (write_cte_config(cte_config) != CFG_WRITE_OK) {
			/* "CTE config write failed!\n" */
			printk(KERN_INFO"\n[TSP][ERROR] line : %d\n", __LINE__);
			return CFG_WRITE_FAILED;
		}
	}

	/* Backup settings to NVM. */
	if (backup_config() != WRITE_MEM_OK) {
		/* "Failed to backup, exiting...\n" */
		return WRITE_MEM_FAILED;
	}

	/* Calibrate the touch IC. */
	if (calibrate_chip() != WRITE_MEM_OK) {
		printk(KERN_INFO"Failed to calibrate, exiting...\n");
		return WRITE_MEM_FAILED;
	}

	printk(KERN_INFO"\n[TSP] configs saved : %d\n", __LINE__);

	return 0;
}

static ssize_t set_write_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	printk(KERN_INFO "[TSP] %s : operate nothing\n", __func__);

	return size;
}
static DEVICE_ATTR(set_write, S_IRUGO | S_IWUSR | /*S_IWOTH |*/ S_IXOTH, set_write_show, set_write_store);


#ifdef QT_ATCOM_TEST
struct device *qt602240_atcom_test;
struct work_struct qt_touch_update_work;

unsigned int qt_firm_status_data;
void set_qt_update_exe(struct work_struct *p)
{
	int i;

	disable_irq(qt602240->client->irq);

    printk(KERN_INFO"Enter to Firmware download by AT command \n");

	qt_boot_download = 1;

    if (!qt_boot()) {
		qt_firm_status_data = 2;
		printk(KERN_INFO"Reprogram done : Firmware update Success~~~~~~~~~~\n");
    } else {
		qt_firm_status_data = 3;
		printk(KERN_INFO"[QT]Reprogram done : Firmware update Fail~~~~~~~~~~\n");
	}

	mdelay(2000);
	hw_reset_chip();

    for(i=0; i<3; i++)
	{
		if(quantum_touch_probe() == 0)
		{
			break;
		}	
		else
		{
			printk(KERN_INFO"[QT]touch probe fail!!! retry %d\n",i);
			mdelay(2000);
			hw_reset_chip();
		}
	}

	qt_boot_download = 0;

	enable_irq(qt602240->client->irq);
}


static ssize_t set_qt_firm_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", tsp_version);
}

static ssize_t set_qt_firm_version_read_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", info_block->info_id.version);
}

static ssize_t set_qt_firm_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;

	printk(KERN_INFO"Enter set_qt_firm_status_show by AT command \n");

	if (qt_firm_status_data == 1)
		count = sprintf(buf, "Downloading\n");
	else if (qt_firm_status_data == 2)
		count = sprintf(buf, "PASS\n");
	else if (qt_firm_status_data == 3)
		count = sprintf(buf, "FAIL\n");
	else
		count = sprintf(buf, "PASS\n");

	return count;
}

static DEVICE_ATTR(set_qt_firm_version, S_IRUGO | S_IWUSR | /*S_IWOTH |*/ S_IXOTH, set_qt_firm_version_show, NULL);
static DEVICE_ATTR(set_qt_firm_status, S_IRUGO | S_IWUSR | /*S_IWOTH |*/ S_IXOTH, set_qt_firm_status_show, NULL);
static DEVICE_ATTR(set_qt_firm_version_read, S_IRUGO | S_IWUSR | /*S_IWOTH |*/ S_IXOTH, set_qt_firm_version_read_show, NULL);
#endif


static ssize_t regs_i2c_read_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	char *tmpStr, *tmpStr2;
	uint8_t i;
	unsigned int j;
		
	tmpStr = kmalloc(63*sizeof(char), GFP_KERNEL | GFP_ATOMIC);
	tmpStr2 = kmalloc(33*sizeof(char), GFP_KERNEL | GFP_ATOMIC);
	
	if (tmpStr == NULL || tmpStr2 == NULL)	{
		sprintf(buf, "kmalloc error!!!!\n");
		return 32;
	}
	
	sprintf(buf, "regs_i2c_read_show Start!  array count = %d\n", read_config_array_count);
	for(i = 0; i < read_config_array_count; i++)
	{
		if (readConfig[i].table_item_name != 0x00)	{
			memset(tmpStr, 0x00, 63*sizeof(char));
			for(j = 0; j < 16; j++)	{
				if (readConfig[i].table_item_value[j] == 0xEF)	{
					j = 16;
				}
				else	{					
					sprintf(tmpStr2, "%d ", readConfig[i].table_item_value[j]);
					strcat(tmpStr, tmpStr2);
				}
			}
			strcat(buf, readConfig[i].table_item_name);
			strcat(buf, " : ");
			strcat(buf, tmpStr);
			strcat(buf, "\n");
		}
	}
	strcat(buf, "regs_i2c_read_show \nEnd!\n");
	
	if (tmpStr2 != NULL)	{
		kfree(tmpStr2);
		tmpStr2 = NULL;
	}
	if (tmpStr != NULL)	{
		kfree(tmpStr);
		tmpStr = NULL;
	}
	
	return 1024;
}

static DEVICE_ATTR(regs_i2c_read, 0644, regs_i2c_read_show, NULL);

static ssize_t main_touch_debug_trace_onoff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (l_debug_trace_on == true)
		strcpy(buf, "Main Touch Debug Trace On\n");
	else
		strcpy(buf, "Main Touch Debug Trace Off\n");
	
	return (strlen(buf)+1)*sizeof(char);
}

static ssize_t main_touch_debug_trace_onoff_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int value = 0;
	
	sscanf(buf, "%d\n", &value);
	
	if (value <= 0)
		l_debug_trace_on = false;
	else
		l_debug_trace_on = true;

	return 0;
}
static DEVICE_ATTR(main_touch_debug_trace_onoff, 0644, main_touch_debug_trace_onoff_show, main_touch_debug_trace_onoff_store);
/*------------------------------ for tunning ATmel - end ----------------------------*/


int __init qt602240_init(void)
{
	int ret;
	int i = 0;

	DEBUG;

	qt602240_wq = NULL;
	qt_time_point = 0;

	qt602240_wq = create_singlethread_workqueue("qt602240_wq");
	if (!qt602240_wq)
		return -ENOMEM;

	qt602240_init_fail = 0;
	qt602240 = kzalloc(sizeof(struct qt602240_data), GFP_KERNEL);

	if (qt602240 == NULL)
		return -ENOMEM;

	qt_time_point = jiffies_to_msecs(jiffies);

	wake_lock_init(&tsp_firmware_wake_lock, WAKE_LOCK_SUSPEND, "TSP");
	wake_lock_init(&tsp_sensing_wake_lock, WAKE_LOCK_IDLE, "TSP_SENSING");
	
	ret = i2c_add_driver(&qt602240_i2c_driver);

	if (ret)
		printk(KERN_INFO"[%s], i2c_add_driver failed...(%d)\n", __func__, ret);

	printk(KERN_INFO"[QT] ret : %d, qt602240->client name : %s\n", ret, qt602240->client->name);

	if (!(qt602240->client)) {
		printk(KERN_INFO"[%s],slave address changed try to firmware reprogram \n", __func__);
		i2c_del_driver(&qt602240_i2c_driver);

		ret = i2c_add_driver(&qt602240_i2c_driver);

		if (ret)
			printk(KERN_INFO"[%s], i2c_add_driver failed...(%d)\n", __func__, ret);

		printk(KERN_INFO"[QT] ret : %d, qt602240->client name : %s\n", ret, qt602240->client->name);

		if (qt602240->client) {
			qt_reprogram();
			i2c_del_driver(&qt602240_i2c_driver);

			ret = i2c_add_driver(&qt602240_i2c_driver);
			if (ret)
				printk(KERN_INFO"[%s], i2c_add_driver failed...(%d)\n", __func__, ret);

			printk(KERN_INFO"[QT] ret : %d, qt602240->client name : %s\n", ret, qt602240->client->name);
		}
	}


	for (i = 0; i < MAX_USING_FINGER_NUM; i++)
		fingerinfo[i].pressure = -1;

	



/*
 *  Test file creating
 */
	sec_class = class_create(THIS_MODULE, "lge");
    if (IS_ERR(sec_class))
		pr_err("Failed to create class(lge)!\n");

	ts_dev = device_create(sec_class, NULL, 0, NULL, "ts");
	if (IS_ERR(ts_dev))
		pr_err("Failed to create device(ts)!\n");
	if (device_create_file(ts_dev, &dev_attr_set_qt_init_state) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_set_qt_init_state.attr.name);

	if (!(qt602240->client)) {
		printk(KERN_INFO"###################################################\n");
		printk(KERN_INFO"##                                               ##\n");
		printk(KERN_INFO"##    WARNING! TOUCHSCREEN DRIVER CAN'T WORK.    ##\n");
		printk(KERN_INFO"##    PLEASE CHECK YOUR TOUCHSCREEN CONNECTOR!   ##\n");
		printk(KERN_INFO"##                                               ##\n");
		printk(KERN_INFO"###################################################\n");
		qt602240_init_fail = 1;
		i2c_del_driver(&qt602240_i2c_driver);
		return 0;
	}

	printk(KERN_INFO"[QT] %s/%d\n", __func__, __LINE__);

	if (device_create_file(ts_dev, &dev_attr_gpio) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_gpio.attr.name);
	if (device_create_file(ts_dev, &dev_attr_i2c) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_i2c.attr.name);
	if (device_create_file(ts_dev, &dev_attr_firmware) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_firmware.attr.name);
	if (device_create_file(ts_dev, &dev_attr_firmware1) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_firmware1.attr.name);
	if (device_create_file(ts_dev, &dev_attr_key_threshold) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_key_threshold.attr.name);

	///////for registers dump////////////////////////////////////////////////////////////////////
	if (device_create_file(ts_dev, &dev_attr_regs_i2c_read) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_regs_i2c_read.attr.name);
	/////////////////////////////////////////////////////////////////////////////////////////
	if (device_create_file(ts_dev, &dev_attr_main_touch_debug_trace_onoff) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_main_touch_debug_trace_onoff.attr.name);
	/////////////////////////////////////////////////////////////////////////////////////////
	printk(KERN_INFO"[QT] %s/%d, platform_driver_register!!\n", __func__, __LINE__);

	/*------------------------------ for tunning ATmel - start ----------------------------*/
	touch_class = class_create(THIS_MODULE, "touch");
	if (IS_ERR(touch_class))
		pr_err("Failed to create class(touch)!\n");

	switch_test = device_create(touch_class, NULL, 0, NULL, "switch");
	if (IS_ERR(switch_test))
		pr_err("Failed to create device(switch)!\n");

	if (device_create_file(switch_test, &dev_attr_set_power) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_set_power.attr.name);
	if (device_create_file(switch_test, &dev_attr_set_acquisition) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_set_acquisition.attr.name);
	if (device_create_file(switch_test, &dev_attr_set_touchscreen) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_set_touchscreen.attr.name);
	if (device_create_file(switch_test, &dev_attr_set_total) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_set_total.attr.name);
	if (device_create_file(switch_test, &dev_attr_set_write) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_set_write.attr.name);
	if (device_create_file(switch_test, &dev_attr_set_noise) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_set_noise.attr.name);
	if (device_create_file(switch_test, &dev_attr_set_grip) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_set_grip.attr.name);
	/*------------------------------ for tunning ATmel - end ----------------------------*/


	/*------------------------------	 AT COMMAND TEST 		---------------------*/
#ifdef QT_ATCOM_TEST
	qt602240_atcom_test = device_create(sec_class, NULL, 0, NULL, "qt602240_atcom_test");
	if (IS_ERR(qt602240_atcom_test))
		printk(KERN_INFO"Failed to create device(qt602240_atcom_test)!\n");

	if (device_create_file(qt602240_atcom_test, &dev_attr_set_qt_firm_version) < 0)
		printk(KERN_INFO"Failed to create device file(%s)!\n", dev_attr_set_qt_firm_version.attr.name);
	if (device_create_file(qt602240_atcom_test, &dev_attr_set_qt_firm_status) < 0)
		printk(KERN_INFO"Failed to create device file(%s)!\n", dev_attr_set_qt_firm_status.attr.name);
	if (device_create_file(qt602240_atcom_test, &dev_attr_set_qt_firm_version_read) < 0)
		printk(KERN_INFO"Failed to create device file(%s)!\n", dev_attr_set_qt_firm_version_read.attr.name);
#endif
	/*------------------------------	 AT COMMAND TEST 		---------------------*/

	return 0;
}

void __exit qt602240_exit(void)
{
	wake_lock_destroy(&tsp_firmware_wake_lock);

	i2c_del_driver(&qt602240_i2c_driver);
	if (qt602240_wq)
		destroy_workqueue(qt602240_wq);
}

late_initcall(qt602240_init);
module_exit(qt602240_exit);

MODULE_DESCRIPTION("Quantum Touchscreen Driver");
MODULE_LICENSE("GPL");
