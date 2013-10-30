/*
 * drivers/media/video/msm/lm2759_flash.c
 *
 * Flash (LM2759) driver
 *
 * Copyright (C) 2010 LGE, Inc.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <../../../drivers/staging/android/timed_output.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <mach/board.h>
#include <mach/camera.h>
#include <linux/i2c.h>

#define	CAMERA_LED_OFF	 	 			0
#define CAMERA_LED_LOW   	 			1 	
#define CAMERA_LED_HIGH	 	 			2 
#define CAMERA_LED_AGC_STATE 			3
#define CAMERA_LED_TORCH 				4

/*for isx006 sensor*/
#define AGC_THRESHOLD					0x06CC  


/* Register Descriptions */
#define LM2759_REG_ENABLE				0x10
#define LM2759_REG_GPIO					0x20
#define LM2759_REG_VLED_MONITOR			0x30
#define LM2759_REG_ADC_DELAY			0x31
#define LM2759_REG_VIN_MONITOR			0x80
#define LM2759_REG_LAST_FLASH			0x81
#define LM2759_REG_TORCH_BRIGHTNESS		0xA0
#define LM2759_REG_FLASH_BRIGHTNESS		0xB0
#define LM2759_REG_FLASH_DURATION		0xC0
#define LM2759_REG_FLAGS				0xD0
#define LM2759_REG_CONFIGURATION1		0xE0
#define LM2759_REG_CONFIGURATION2		0xF0
#define LM2759_REG_PRIVACY				0x11
#define LM2759_REG_MESSAGE_INDICATOR	0x12
#define LM2759_REG_INDICATOR_BLINKING	0x13
#define LM2759_REG_PRIVACY_PWM			0x14


#define LM2759_I2C_NAME  "lm2759"

//LGE_UPDATE_S minhobb2.kim@lge.com for univaq camera flash enable
#if defined (LGE_MODEL_C800)
#define CAM_FLASH_EN_GPIO 19
#endif
//LGE_UPDATE_E minhobb2.kim@lge.com for univaq camera flash enable

struct led_flash_platform_data *lm2759_pdata = NULL;
struct i2c_client *lm2759_client = NULL;

static int32_t lm2759_write_reg(struct i2c_client *client, unsigned char* buf, int length)
{
	int32_t err = 0;
	
	struct i2c_msg	msg[] = {
		{
			.addr  = client->addr, 
			.flags = 0, 
			.len   = length, 
			.buf   = buf, 
		},
	};
	
	if ((err = i2c_transfer(client->adapter, &msg[0], 1)) < 0) {
		dev_err(&client->dev, "i2c write error [%d]\n",err);
	}
	
	return err;
}

static int32_t lm2759_i2c_write(struct i2c_client *client,unsigned char addr, unsigned char data)
{
	unsigned char buf[2] ={0,};
	int32_t rc = -EIO;

	if(client == NULL)
		return rc;


	buf[0] = addr;
	buf[1] = data;

	rc = lm2759_write_reg(client,&buf[0],2);

	return rc;
}

static int32_t lm2759_read_reg(struct i2c_client *client, unsigned char* buf, int length)
{
	int32_t err = 0;
	
	struct i2c_msg	msgs[] = {	
		{ 
			.addr  = client->addr, 
			.flags = 0, 
			.len   = 2,
			.buf   = buf, 
		},
		{ 
			.addr  = client->addr, 
			.flags = I2C_M_RD, 
			.len   = length,
			.buf   = buf, 
		},
	};
	
	if ((err = i2c_transfer(client->adapter, msgs, 1)) < 0) {
		dev_err(&client->dev, "i2c write error [%d]\n",err);
	}
	
	return err;
	
}

static int32_t lm2759_i2c_read(struct i2c_client *client,unsigned char addr, unsigned char *data)
{
	unsigned char buf[2] ={0,};
	int32_t rc = -EIO;

	if((client == NULL)||(data == NULL))
		return rc;


	buf[0] = addr;

	rc = lm2759_read_reg(client,&buf[0],1);
    if(rc < 0)
		return rc;

	*data = buf[0];
	
	return rc;

}

void lm2759_led_shutdown(void)
{	
	lm2759_i2c_write(lm2759_client,LM2759_REG_ENABLE,0x18);
}

void lm2759_enable_torch_mode(int state)
{
	unsigned char data = 0;

	lm2759_i2c_read(lm2759_client,LM2759_REG_TORCH_BRIGHTNESS,&data);	 			 
	data &= 0xF8;

    if(state == CAMERA_LED_LOW){
		/* 000 : 28.125 mA (56.25 mA total) */
		lm2759_i2c_write(lm2759_client,LM2759_REG_TORCH_BRIGHTNESS,data);
	}
	else{
		/* 110 : 196.875 mA (393.75 mA total) */ 
#if defined (LGE_MODEL_C800)
 		data = 0x07;
#else
 #if defined (LGE_MODEL_C729_REV_A) || defined(LGE_MODEL_C729_REV_B)
		data |= 0x06;
 #else
 		data = 0x06;
 #endif
#endif
		lm2759_i2c_write(lm2759_client,LM2759_REG_TORCH_BRIGHTNESS,data);
	}
#if defined (LGE_MODEL_C800)
	lm2759_i2c_write(lm2759_client,LM2759_REG_ENABLE,0x0a);
#else
#if defined (LGE_MODEL_C729_REV_A) || defined(LGE_MODEL_C729_REV_B)
	lm2759_i2c_write(lm2759_client,LM2759_REG_ENABLE,0x09);
#else
	lm2759_i2c_write(lm2759_client,LM2759_REG_ENABLE,0x0a);
#endif
#endif
}

void lm2759_enable_flash_mode(int state)
{
	unsigned char data = 0;

	lm2759_i2c_read(lm2759_client,LM2759_REG_FLASH_BRIGHTNESS,&data);					 
	data &= 0xF0;
			 
	if(state == CAMERA_LED_LOW){ 		
		/* 0000 : 56.25 mA (112.5 mA total) */
		lm2759_i2c_write(lm2759_client,LM2759_REG_FLASH_BRIGHTNESS,data);
	}
	else{
		/* 1101 : 787.5 mA (1575 mA total) Default */
#if defined (LGE_MODEL_C800)
 		data = 0x0d; // 0x0b
#else
 #if defined (LGE_MODEL_C729_REV_A) || defined(LGE_MODEL_C729_REV_B)
		data |= 0x0A;
 #else
 		data = 0x0b;
 #endif
#endif
		lm2759_i2c_write(lm2759_client,LM2759_REG_FLASH_BRIGHTNESS,data);
	}
#if defined (LGE_MODEL_C800)
	lm2759_i2c_write(lm2759_client,LM2759_REG_ENABLE,0x0B);
#else
#if defined (LGE_MODEL_C729_REV_A) || defined(LGE_MODEL_C729_REV_B)
	lm2759_i2c_write(lm2759_client,LM2759_REG_ENABLE,0x0B);
#else
	lm2759_i2c_write(lm2759_client,LM2759_REG_ENABLE,0x0B);
#endif
#endif

}

int lm2759_flash_set_led_state(int led_state)
{	
	int current_agc = 0;
	int rc = 0;

	switch (led_state) {
	case CAMERA_LED_OFF:
		printk("[LM2759]CAMERA_LED_OFF\n");
		lm2759_led_shutdown();
		break;
	case CAMERA_LED_LOW:
		printk("[LM2759]CAMERA_LED_LOW\n");
		lm2759_enable_flash_mode(CAMERA_LED_LOW);
		break;
	case CAMERA_LED_HIGH:	
		printk("[LM2759]CAMERA_LED_HIGH\n");
		lm2759_enable_flash_mode(CAMERA_LED_HIGH);
		break;
    	case CAMERA_LED_AGC_STATE:
    	        printk("[LM2759]CAMERA_LED_AGC_STATE\n");
//		rc = isx006_get_current_agc(&current_agc);
		
		if(current_agc >= AGC_THRESHOLD){
			lm2759_enable_flash_mode(CAMERA_LED_HIGH);
		}
		break;
	case CAMERA_LED_TORCH:	
		printk("[LM2759]CAMERA_LED_TORCH\n");
		lm2759_enable_torch_mode(CAMERA_LED_HIGH);
		break;			
	default:
		rc = -EFAULT;
		break;
	}

	return rc;
	
}

EXPORT_SYMBOL(lm2759_flash_set_led_state);

static int lm2759_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	if (i2c_get_clientdata(client))
		return -EBUSY;

	lm2759_client = client;
//	lm2759_i2c_write(lm2759_client,LM2759_REG_ENABLE,0x11);

//LGE_UPDATE_S minhobb2.kim@lge.com for univaq camera flash enable
#if defined (LGE_MODEL_C800)
	gpio_request(CAM_FLASH_EN_GPIO, "cam_flash_en");
	gpio_tlmm_config(GPIO_CFG(CAM_FLASH_EN_GPIO, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(CAM_FLASH_EN_GPIO, 1);
#endif
//LGE_UPDATE_E minhobb2.kim@lge.com for univaq camera flash enable
   
        return 0;
	
}

static int lm2759_remove(struct i2c_client *client)
{
	return 0;
}	

static const struct i2c_device_id lm2759_ids[] = {
	{ LM2759_I2C_NAME, 0 },	/* lm2759 */
	{ /* end of list */ },
};

static struct i2c_driver lm2759_driver = {
	.probe 	  = lm2759_probe,
	.remove   = lm2759_remove,
	.id_table = lm2759_ids,
	.driver   = {
		.name =  LM2759_I2C_NAME,
		.owner= THIS_MODULE,
    },
};
static int __init lm2759_init(void)
{
    return i2c_add_driver(&lm2759_driver);
}

static void __exit lm2759_exit(void)
{
	i2c_del_driver(&lm2759_driver);
}

module_init(lm2759_init);
module_exit(lm2759_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("LM2759 Flash Driver");
MODULE_LICENSE("GPL");

