/*  $Date: 2010/09/10 11:40:00 $
 *  $Revision: 1.0 $ 
 */

/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2010 Bosch Sensortec GmbH
 * All Rights Reserved
 */



/*! \file BMA250_driver.c
    \brief This file contains all function implementations for the BMA250 in linux
    
    Details.
*/
//#define BMA250_HAS_EARLYSUSPEND

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <asm/uaccess.h>
#include <linux/unistd.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/timer.h>
#include <linux/jiffies.h>

#define BMA250_ATCMD
//#define BMA250_DEBUG
#define	BMA250_ENABLE_IRQ
// 2011-05-31 by baborobo@lge.com
// don't used the early suspend for IRQ pair
//#define BMA250_HAS_EARLYSUSPEND

#ifdef BMA250_HAS_EARLYSUSPEND
#include<linux/earlysuspend.h>
#endif
#ifdef	BMA250_ENABLE_IRQ
#include <asm/gpio.h>
#endif

#include <linux/poll.h>
#include "bma250.h"
#include "bma250_driver.h"

#include <mach/board_lge.h>

#ifdef BMA250_MODULES 
#include "bma250.c"
#endif

#include "define.h"

bma250_t * p_bma250;

#ifdef BMA250_ENABLE_IRQ
static int bma250_interrupt_config(int enable);

static struct workqueue_struct *acceler_wq;
#endif

struct acceleration_platform_data *bma250_pdata;

/* globe variant */
static struct i2c_client *bma250_client = NULL;
struct bma250_data {
	bma250_t			bma250;
	struct fasync_struct *async_queue;
#ifdef BMA250_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
#ifdef	BMA250_ENABLE_IRQ
	int irq;
	int irq2;
	struct input_dev		*input_dev;		/* input device for android */
	struct delayed_work		dwork;			/* delayed work for bh */
  struct timer_list  flip_check_timer;            
	int	debounce;		/* Delayed work schedule time */
	spinlock_t			lock;			/* protect resources */
	int	flip_Status;		/* Flip Status */
	int	sensor_enable;		/* sensor enable status */
#endif
};

#ifdef	BMA250_ENABLE_IRQ
static irqreturn_t bma250_irq_handler(int irq, void *_id);
#endif

int bma250_init(bma250_t *bma250)
   {
   int comres= C_Zero_U8X ;
   unsigned char data;
   
   p_bma250 = bma250;

   /* assign bma250 ptr */
   p_bma250->dev_addr = BMA250_I2C_ADDR;                                                   /* preset bma250 I2C_addr */
   comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_CHIP_ID__REG, &data, 1);     /* read Chip Id */
   
   p_bma250->chip_id = data ;                                          /* get bitslice */
   
   comres += p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_ML_VERSION__REG, &data, 1); /* read Version reg */
   p_bma250->ml_version = bma250_GET_BITSLICE(data, bma250_ML_VERSION);                            /* get ML Version */
   p_bma250->al_version = bma250_GET_BITSLICE(data, bma250_AL_VERSION);                            /* get AL Version */
   return comres;
   }

int bma250_soft_reset(void)
   {
   int comres = C_Zero_U8X ;
   unsigned char data = bma250_EN_SOFT_RESET_VALUE ;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_SOFT_RESET__REG,&data ,C_One_U8X);
      }
   return comres;
   }

int bma250_write_reg(unsigned char addr, unsigned char *data, unsigned char len)
   {
   int comres = C_Zero_U8X ;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, addr, data, len);
      }
   return comres;
   }

int bma250_read_reg(unsigned char addr, unsigned char *data, unsigned char len)
   {
   int comres = C_Zero_U8X ;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, addr, data, len);
      }
   return comres;
   }

unsigned char bma250_set_mode(unsigned char Mode)
   {
   int comres=C_Zero_U8X ;
   unsigned char data1;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      if (Mode < C_Three_U8X)
         {
         comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_LOW_POWER__REG, &data1, C_One_U8X );
         switch (Mode)
            {
            case bma250_MODE_NORMAL:
               data1  = bma250_SET_BITSLICE(data1, bma250_EN_LOW_POWER, C_Zero_U8X);
               data1  = bma250_SET_BITSLICE(data1, bma250_EN_SUSPEND, C_Zero_U8X);
               break;

            case bma250_MODE_LOWPOWER:
               data1  = bma250_SET_BITSLICE(data1, bma250_EN_LOW_POWER, C_One_U8X);
               data1  = bma250_SET_BITSLICE(data1, bma250_EN_SUSPEND, C_Zero_U8X);
               break;

            case bma250_MODE_SUSPEND:
               data1  = bma250_SET_BITSLICE(data1, bma250_EN_LOW_POWER, C_Zero_U8X);
               data1  = bma250_SET_BITSLICE(data1, bma250_EN_SUSPEND, C_One_U8X);
               break;

            default:
               break;
            }
         comres += p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_LOW_POWER__REG, &data1, C_One_U8X);
         p_bma250->mode = Mode;
         }
      else
         {
         comres = E_OUT_OF_RANGE ;
         }
      }
   return comres;
   }

int bma250_get_mode(unsigned char * Mode )
   {
   int comres= C_Zero_U8X;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_LOW_POWER__REG, Mode, C_One_U8X );
      *Mode  = (*Mode) >> C_Six_U8X;
      /* EasyCASE - */
      p_bma250->mode = *Mode;
      }
   return comres;
   }

unsigned char bma250_set_range(unsigned char Range)
   {
   int comres=C_Zero_U8X ;
   unsigned char data1;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {

      #ifdef CONFIG_LGE_MODEL_SU610
      if (Range < C_Five_U8X)
         {
         comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_RANGE_SEL_REG, &data1, C_One_U8X );
         switch (Range)
            {
            case C_Zero_U8X:
               data1  = bma250_SET_BITSLICE(data1, bma250_RANGE_SEL, C_Zero_U8X);
               break;
            case C_One_U8X:
               data1  = bma250_SET_BITSLICE(data1, bma250_RANGE_SEL, C_Three_U8X);
               break;
            case C_Two_U8X:
               data1  = bma250_SET_BITSLICE(data1, bma250_RANGE_SEL, C_Five_U8X);
               break;
            case C_Three_U8X:
               data1  = bma250_SET_BITSLICE(data1, bma250_RANGE_SEL, C_Eight_U8X);
               break;
            case C_Four_U8X:
               data1  = bma250_SET_BITSLICE(data1, bma250_RANGE_SEL, C_Twelve_U8X);
               break;
            default:
               break;
            }
         comres += p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_RANGE_SEL_REG, &data1, C_One_U8X);
         }
         
        #else

      if (Range < C_Four_U8X)
         {
         comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_RANGE_SEL_REG, &data1, C_One_U8X );
         switch (Range)
            {
            case C_Zero_U8X:
               data1  = bma250_SET_BITSLICE(data1, bma250_RANGE_SEL, C_Zero_U8X);
               break;
            case C_One_U8X:
               data1  = bma250_SET_BITSLICE(data1, bma250_RANGE_SEL, C_Five_U8X);
               break;
            case C_Two_U8X:
               data1  = bma250_SET_BITSLICE(data1, bma250_RANGE_SEL, C_Eight_U8X);
               break;
            case C_Three_U8X:
               data1  = bma250_SET_BITSLICE(data1, bma250_RANGE_SEL, C_Twelve_U8X);
               break;
            default:
               break;
            }
         comres += p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_RANGE_SEL_REG, &data1, C_One_U8X);
         }
       #endif

      else
         {
         comres = E_OUT_OF_RANGE ;
         }
      }
   return comres;
   }

int bma250_get_range(unsigned char * Range )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_RANGE_SEL__REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_RANGE_SEL);
      *Range = data;
      }
   return comres;
   }

int bma250_set_bandwidth(unsigned char BW)
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   int Bandwidth = C_Zero_U8X ;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      if (BW < C_Eight_U8X)
         {
         switch (BW)
            {
            case C_Zero_U8X:
               Bandwidth = bma250_BW_7_81HZ;
               
               /*  7.81 Hz      64000 uS   */
               break;
            case C_One_U8X:
               Bandwidth = bma250_BW_15_63HZ;
               
               /*  15.63 Hz     32000 uS   */
               break;
            case C_Two_U8X:
               Bandwidth = bma250_BW_31_25HZ;
               
               /*  31.25 Hz     16000 uS   */
               break;
            case C_Three_U8X:
               Bandwidth = bma250_BW_62_50HZ;
               
               /*  62.50 Hz     8000 uS   */
               break;
            case C_Four_U8X:
               Bandwidth = bma250_BW_125HZ;
               
               /*  125 Hz       4000 uS   */
               break;
            case C_Five_U8X:
               Bandwidth = bma250_BW_250HZ;
               
               /*  250 Hz       2000 uS   */
               break;
            case C_Six_U8X:
               Bandwidth = bma250_BW_500HZ;
               
               /*  500 Hz       1000 uS   */
               break;
            case C_Seven_U8X:
               Bandwidth = bma250_BW_1000HZ;
               
               /*  1000 Hz      500 uS   */
               break;
            default:
               break;
            }
         comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_BANDWIDTH__REG, &data, C_One_U8X );
         data = bma250_SET_BITSLICE(data, bma250_BANDWIDTH, Bandwidth );
         comres += p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_BANDWIDTH__REG, &data, C_One_U8X );
         }
      else
         {
         comres = E_OUT_OF_RANGE ;
         }
      }
   return comres;
   }

int bma250_get_bandwidth(unsigned char * BW)
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_BANDWIDTH__REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_BANDWIDTH);
      if (data <= C_Eight_U8X)
         {
         *BW = C_Zero_U8X;
         }
      else
         {
         if (data >= 0x0F)
            {
            *BW = C_Seven_U8X;
            }
         else
            {
            *BW = data - C_Eight_U8X;
            }
         }
      }
   return comres;
   }

int bma250_read_accel_xyz(bma250acc_t * acc)
   {
   int comres;
   unsigned char data[6];
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_ACC_X_LSB__REG, data,6);
      
      acc->x = bma250_GET_BITSLICE(data[0],bma250_ACC_X_LSB) | (bma250_GET_BITSLICE(data[1],bma250_ACC_X_MSB)<<bma250_ACC_X_LSB__LEN);
      acc->x = acc->x << (sizeof(short)*8-(bma250_ACC_X_LSB__LEN + bma250_ACC_X_MSB__LEN));
      acc->x = acc->x >> (sizeof(short)*8-(bma250_ACC_X_LSB__LEN + bma250_ACC_X_MSB__LEN));
    
      acc->y = bma250_GET_BITSLICE(data[2],bma250_ACC_Y_LSB) | (bma250_GET_BITSLICE(data[3],bma250_ACC_Y_MSB)<<bma250_ACC_Y_LSB__LEN);
      acc->y = acc->y << (sizeof(short)*8-(bma250_ACC_Y_LSB__LEN + bma250_ACC_Y_MSB__LEN));
      acc->y = acc->y >> (sizeof(short)*8-(bma250_ACC_Y_LSB__LEN + bma250_ACC_Y_MSB__LEN));
      //  acc->y = -(acc->y);  
      acc->z = bma250_GET_BITSLICE(data[4],bma250_ACC_Z_LSB) | (bma250_GET_BITSLICE(data[5],bma250_ACC_Z_MSB)<<bma250_ACC_Z_LSB__LEN);
      acc->z = acc->z << (sizeof(short)*8-(bma250_ACC_Z_LSB__LEN + bma250_ACC_Z_MSB__LEN));
      acc->z = acc->z >> (sizeof(short)*8-(bma250_ACC_Z_LSB__LEN + bma250_ACC_Z_MSB__LEN));
      //  acc->z = -(acc->z);
      }
   return comres;
   }

int bma250_read_accel_x(short *a_x)
   {
   int comres;
   unsigned char data[2];
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_ACC_X_LSB__REG, data, 2);
      *a_x = bma250_GET_BITSLICE(data[0],bma250_ACC_X_LSB) | (bma250_GET_BITSLICE(data[1],bma250_ACC_X_MSB)<<bma250_ACC_X_LSB__LEN);
      *a_x = *a_x << (sizeof(short)*8-(bma250_ACC_X_LSB__LEN+bma250_ACC_X_MSB__LEN));
      *a_x = *a_x >> (sizeof(short)*8-(bma250_ACC_X_LSB__LEN+bma250_ACC_X_MSB__LEN));
      }
   return comres;
   }

int bma250_read_accel_y(short *a_y)
   {
   int comres;
   unsigned char data[2];
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_ACC_Y_LSB__REG, data, 2);
      *a_y = bma250_GET_BITSLICE(data[0],bma250_ACC_Y_LSB) | (bma250_GET_BITSLICE(data[1],bma250_ACC_Y_MSB)<<bma250_ACC_Y_LSB__LEN);
      *a_y = *a_y << (sizeof(short)*8-(bma250_ACC_Y_LSB__LEN+bma250_ACC_Y_MSB__LEN));
      *a_y = *a_y >> (sizeof(short)*8-(bma250_ACC_Y_LSB__LEN+bma250_ACC_Y_MSB__LEN));
      }
   return comres;
   }

int bma250_read_accel_z(short *a_z)
   {
   int comres;
   unsigned char data[2];
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_ACC_Z_LSB__REG, data, 2);
      *a_z = bma250_GET_BITSLICE(data[0],bma250_ACC_Z_LSB) | bma250_GET_BITSLICE(data[1],bma250_ACC_Z_MSB)<<bma250_ACC_Z_LSB__LEN;
      *a_z = *a_z << (sizeof(short)*8-(bma250_ACC_Z_LSB__LEN+bma250_ACC_Z_MSB__LEN));
      *a_z = *a_z >> (sizeof(short)*8-(bma250_ACC_Z_LSB__LEN+bma250_ACC_Z_MSB__LEN));
      }
   return comres;
   }

int bma250_reset_interrupt(void)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_RESET_LATCHED__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_INT_RESET_LATCHED, C_One_U8X );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_INT_RESET_LATCHED__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_interruptstatus1(unsigned char *intstatus )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS1_REG, &data, C_One_U8X );
      *intstatus = data;
      }
   return comres;
   }

int bma250_get_interruptstatus2(unsigned char *intstatus )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS2_REG, &data, C_One_U8X );
      *intstatus = data;
      }
   return comres;
   }

int bma250_get_Low_G_interrupt(unsigned char *intstatus )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS1_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_LOWG_INT_S);
      *intstatus = data;
      }
   return comres;
   }

int bma250_get_High_G_Interrupt(unsigned char *intstatus )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS1_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_HIGHG_INT_S);
      *intstatus = data;
      }
   return comres;
   }

int bma250_get_slope_interrupt(unsigned char *intstatus )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS1_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_SLOPE_INT_S);
      *intstatus = data;
      }
   return comres;
   }

int bma250_get_double_tap_interrupt(unsigned char *intstatus )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS1_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_DOUBLE_TAP_INT_S);
      *intstatus = data;
      }
   return comres;
   }

int bma250_get_single_tap_interrupt(unsigned char *intstatus )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS1_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_SINGLE_TAP_INT_S);
      *intstatus = data;
      }
   return comres;
   }

int bma250_get_orient_interrupt(unsigned char *intstatus )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS1_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_ORIENT_INT_S);
      *intstatus = data;
      }
   return comres;
   }

int bma250_get_flat_interrupt(unsigned char *intstatus )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS1_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_FLAT_INT_S);
      *intstatus = data;
      }
   return comres;
   }

int bma250_get_data_interrupt(unsigned char *intstatus )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS2_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_DATA_INT_S);
      *intstatus = data;
      }
   return comres;
   }

int bma250_get_slope_first(unsigned char param,unsigned char *intstatus )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      switch (param)
         {
         case 0:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS_TAP_SLOPE_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_SLOPE_FIRST_X);
            *intstatus = data;
            break;
         case 1:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS_TAP_SLOPE_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_SLOPE_FIRST_Y);
            *intstatus = data;
            break;
         case 2:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS_TAP_SLOPE_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_SLOPE_FIRST_Z);
            *intstatus = data;
            break;
         default:
            break;
         }
      }
   return comres;
   }

int bma250_get_slope_sign(unsigned char *intstatus )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS_TAP_SLOPE_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_SLOPE_SIGN_S);
      *intstatus = data;
      }
   return comres;
   }

int bma250_get_tap_first(unsigned char param,unsigned char *intstatus )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      switch (param)
         {
         case 0:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS_TAP_SLOPE_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_TAP_FIRST_X);
            *intstatus = data;
            break;
         case 1:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS_TAP_SLOPE_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_TAP_FIRST_Y);
            *intstatus = data;
            break;
         case 2:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS_TAP_SLOPE_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_TAP_FIRST_Z);
            *intstatus = data;
            break;
         default:
            break;
         }
      }
   return comres;
   }

int bma250_get_tap_sign(unsigned char *intstatus )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS_TAP_SLOPE_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_TAP_SIGN_S);
      *intstatus = data;
      }
   return comres;
   }

int bma250_get_HIGH_first(unsigned char param,unsigned char *intstatus )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      switch (param)
         {
         case 0:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS_ORIENT_HIGH_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_HIGHG_FIRST_X);
            *intstatus = data;
            break;
         case 1:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS_ORIENT_HIGH_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_HIGHG_FIRST_Y);
            *intstatus = data;
            break;
         case 2:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS_ORIENT_HIGH_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_HIGHG_FIRST_Z);
            *intstatus = data;
            break;
         default:
            break;
         }
      }
   return comres;
   }

int bma250_get_HIGH_sign(unsigned char *intstatus )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS_ORIENT_HIGH_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_HIGHG_SIGN_S);
      *intstatus = data;
      }
   return comres;
   }

int bma250_get_orient_status(unsigned char *intstatus )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS_ORIENT_HIGH_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_ORIENT_S);
      *intstatus = data;
      }
   return comres;
   }

int bma250_get_orient_flat_status(unsigned char *intstatus )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS_ORIENT_HIGH_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_FLAT_S);
      *intstatus = data;
      }
   return comres;
   }

int bma250_get_sleep_duration(unsigned char *sleep )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_MODE_CTRL_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_SLEEP_DUR);
      *sleep = data;
      }
   return comres;
   }

int bma250_set_sleep_duration(unsigned char sleepdur )
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_SLEEP_DUR__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_SLEEP_DUR, sleepdur );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_SLEEP_DUR__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_set_suspend(unsigned char state)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_SUSPEND__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_EN_SUSPEND, state );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_SUSPEND__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_suspend(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_MODE_CTRL_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_EN_SUSPEND);
      *status = data;
      }
   return comres;
   }

int bma250_set_lowpower(unsigned char state)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_LOW_POWER__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_EN_LOW_POWER, state );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_LOW_POWER__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_lowpower_en(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_MODE_CTRL_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_EN_LOW_POWER);
      *status = data;
      }
   return comres;
   }

int bma250_set_low_noise_ctrl(unsigned char state)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_LOW_NOISE__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_EN_LOW_NOISE, state );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_LOW_NOISE__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_low_noise_ctrl(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_LOW_NOISE_CTRL_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_EN_LOW_NOISE);
      *status = data;
      }
   return comres;
   }

int bma250_set_shadow_disable(unsigned char state)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   //unsigned char state = 1;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_DIS_SHADOW_PROC__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_DIS_SHADOW_PROC, state );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_DIS_SHADOW_PROC__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_shadow_disable(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_DATA_CTRL_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_DIS_SHADOW_PROC);
      *status = data;
      }
   return comres;
   }

int bma250_set_unfilt_acc(unsigned char state)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
  // unsigned char state = 1;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_UNFILT_ACC__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_EN_UNFILT_ACC, state );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_UNFILT_ACC__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_unfilt_acc(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_DATA_CTRL_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_EN_UNFILT_ACC);
      *status = data;
      }
   return comres;
   }

int bma250_set_enable_slope_interrupt(unsigned char slope)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   unsigned char state;
   state = 0x01;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      switch (slope)
         {
         case 0:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_SLOPE_X_INT__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_SLOPE_X_INT, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_SLOPE_X_INT__REG, &data, C_One_U8X);
            break;
         case 1:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_SLOPE_Y_INT__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_SLOPE_Y_INT, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_SLOPE_Y_INT__REG, &data, C_One_U8X);
            break;
         case 2:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_SLOPE_Z_INT__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_SLOPE_Z_INT, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_SLOPE_Z_INT__REG, &data, C_One_U8X);
            break;
         case 3:
            state = 0x07;
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_SLOPE_XYZ_INT__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_SLOPE_XYZ_INT, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_SLOPE_XYZ_INT__REG, &data, C_One_U8X);
            break;
         default:
            break;
         }
      }
   return comres;
   }

int bma250_get_enable_slope_interrupt(unsigned char param,unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      switch (param)
         {
         case 0:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_ENABLE1_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_SLOPE_X_INT);
            *status = data;
            break;
         case 1:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_ENABLE1_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_SLOPE_Y_INT);
            *status = data;
            break;
         case 2:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_ENABLE1_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_SLOPE_Z_INT);
            *status = data;
            break;
         default:
            break;
         }
      }
   return comres;
   }

int bma250_set_enable_tap_interrupt(unsigned char tapinterrupt)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   unsigned char state;
   state = 0x01;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      switch (tapinterrupt)
         {
         case 0:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_DOUBLE_TAP_INT__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_DOUBLE_TAP_INT, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_DOUBLE_TAP_INT__REG, &data, C_One_U8X);
            break;
         case 1:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_SINGLE_TAP_INT__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_SINGLE_TAP_INT, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_SINGLE_TAP_INT__REG, &data, C_One_U8X);
            break;
         case 2:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_ORIENT_INT__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_ORIENT_INT, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_ORIENT_INT__REG, &data, C_One_U8X);
            break;
         case 3:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_FLAT_INT__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_FLAT_INT, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_FLAT_INT__REG, &data, C_One_U8X);
            break;
         default:
            break;
         }
      }
   return comres;
   }

int bma250_get_enable_tap_interrupt(unsigned char param,unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      switch (param)
         {
         case 0:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_ENABLE1_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_DOUBLE_TAP_INT);
            *status = data;
            break;
         case 1:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_ENABLE1_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_SINGLE_TAP_INT);
            *status = data;
            break;
         case 2:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_ENABLE1_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_ORIENT_INT);
            *status = data;
            break;
         case 3:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_ENABLE1_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_FLAT_INT);
            *status = data;
            break;
         default:
            break;
         }
      }
   return comres;
   }

int bma250_set_enable_high_g_interrupt(unsigned char highinterrupt)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   unsigned char state;
   state = 0x01;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      switch (highinterrupt)
         {
         case 0:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_HIGHG_X_INT__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_HIGHG_X_INT, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_HIGHG_X_INT__REG, &data, C_One_U8X);
            break;
         case 1:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_HIGHG_Y_INT__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_HIGHG_Y_INT, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_HIGHG_Y_INT__REG, &data, C_One_U8X);
            break;
         case 2:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_HIGHG_Z_INT__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_HIGHG_Z_INT, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_HIGHG_Z_INT__REG, &data, C_One_U8X);
            break;
         case 3:
            state = 0x07;
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_HIGHG_XYZ_INT__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_HIGHG_XYZ_INT, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_HIGHG_XYZ_INT__REG, &data, C_One_U8X);
            break;
         default:
            break;
         }
      }
   return comres;
   }

int bma250_get_enable_high_g_interrupt(unsigned char param,unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      switch (param)
         {
         case 0:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_ENABLE2_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_HIGHG_X_INT);
            *status = data;
            break;
         case 1:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_ENABLE2_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_HIGHG_Y_INT);
            *status = data;
            break;
         case 2:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_ENABLE2_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_HIGHG_Z_INT);
            *status = data;
            break;
         case 3:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_ENABLE2_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_HIGHG_XYZ_INT);
            *status = data;
            break;
         default:
            break;
         }
      }
   return comres;
   }

int bma250_set_enable_low_g_interrupt(void)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   unsigned char state;
   state= 0x01;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_LOWG_INT__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_EN_LOWG_INT, state );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_LOWG_INT__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_enable_low_g_interrupt(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_ENABLE2_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_EN_LOWG_INT);
      *status = data;
      }
   return comres;
   }

int bma250_set_enable_data_interrupt(void)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   unsigned char state;
   state= 0x01;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_NEW_DATA_INT__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_EN_NEW_DATA_INT, state );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_NEW_DATA_INT__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_enable_data_interrupt(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_ENABLE2_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_EN_NEW_DATA_INT);
      *status = data;
      }
   return comres;
   }

int bma250_set_int1_pad_sel(unsigned char int1sel)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   unsigned char state;
   state = 0x01;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      switch (int1sel)
         {
         case 0:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_INT1_PAD_LOWG__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_INT1_PAD_LOWG, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_INT1_PAD_LOWG__REG, &data, C_One_U8X);
            break;
         case 1:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_INT1_PAD_HIGHG__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_INT1_PAD_HIGHG, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_INT1_PAD_HIGHG__REG, &data, C_One_U8X);
            break;
         case 2:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_INT1_PAD_SLOPE__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_INT1_PAD_SLOPE, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_INT1_PAD_SLOPE__REG, &data, C_One_U8X);
            break;
         case 3:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_INT1_PAD_DB_TAP__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_INT1_PAD_DB_TAP, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_INT1_PAD_DB_TAP__REG, &data, C_One_U8X);
            break;
         case 4:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_INT1_PAD_SNG_TAP__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_INT1_PAD_SNG_TAP, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_INT1_PAD_SNG_TAP__REG, &data, C_One_U8X);
            break;
         case 5:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_INT1_PAD_ORIENT__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_INT1_PAD_ORIENT, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_INT1_PAD_ORIENT__REG, &data, C_One_U8X);
            break;
         case 6:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_INT1_PAD_FLAT__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_INT1_PAD_FLAT, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_INT1_PAD_FLAT__REG, &data, C_One_U8X);
            break;
         default:
            break;
         }
      }
   return comres;
   }

int bma250_get_int1_pad_sel(unsigned char param,unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      switch (param)
         {
         case 0:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT1_PAD_SEL_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_INT1_PAD_LOWG);
            *status = data;
            break;
         case 1:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT1_PAD_SEL_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_INT1_PAD_HIGHG);
            *status = data;
            break;
         case 2:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT1_PAD_SEL_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_INT1_PAD_SLOPE);
            *status = data;
            break;
         case 3:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT1_PAD_SEL_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_INT1_PAD_DB_TAP);
            *status = data;
            break;
         case 4:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT1_PAD_SEL_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_INT1_PAD_SNG_TAP);
            *status = data;
            break;
         case 5:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT1_PAD_SEL_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_INT1_PAD_ORIENT);
            *status = data;
            break;
         case 6:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT1_PAD_SEL_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_INT1_PAD_FLAT);
            *status = data;
            break;
         default:
            break;
         }
      }
   return comres;
   }

int bma250_set_int_data_sel(unsigned char intsel)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   unsigned char state;
   state = 0x01;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      switch (intsel)
         {
         case 0:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_INT1_PAD_NEWDATA__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_INT1_PAD_NEWDATA, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_INT1_PAD_NEWDATA__REG, &data, C_One_U8X);
            break;
         case 1:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_INT2_PAD_NEWDATA__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_INT2_PAD_NEWDATA, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_INT2_PAD_NEWDATA__REG, &data, C_One_U8X);
            break;
         default:
            break;
         }
      }
   return comres;
   }

int bma250_get_int_data_sel(unsigned char param,unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      switch (param)
         {
         case 0:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_DATA_SEL_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_INT1_PAD_NEWDATA);
            *status = data;
            break;
         case 1:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_DATA_SEL_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_INT2_PAD_NEWDATA);
            *status = data;
            break;
         default:
            break;
         }
      }
   return comres;
   }

int bma250_set_int2_pad_sel(unsigned char int2sel)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   unsigned char state;
   state = 0x01;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      switch (int2sel)
         {
         case 0:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_INT2_PAD_LOWG__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_INT2_PAD_LOWG, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_INT2_PAD_LOWG__REG, &data, C_One_U8X);
            break;
         case 1:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_INT2_PAD_HIGHG__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_INT2_PAD_HIGHG, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_INT2_PAD_HIGHG__REG, &data, C_One_U8X);
            break;
         case 2:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_INT2_PAD_SLOPE__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_INT2_PAD_SLOPE, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_INT2_PAD_SLOPE__REG, &data, C_One_U8X);
            break;
         case 3:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_INT2_PAD_DB_TAP__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_INT2_PAD_DB_TAP, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_INT2_PAD_DB_TAP__REG, &data, C_One_U8X);
            break;
         case 4:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_INT2_PAD_SNG_TAP__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_INT2_PAD_SNG_TAP, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_INT2_PAD_SNG_TAP__REG, &data, C_One_U8X);
            break;
         case 5:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_INT2_PAD_ORIENT__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_INT2_PAD_ORIENT, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_INT2_PAD_ORIENT__REG, &data, C_One_U8X);
            break;
         case 6:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_INT2_PAD_FLAT__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_INT2_PAD_FLAT, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_INT2_PAD_FLAT__REG, &data, C_One_U8X);
            break;
         default:
            break;
         }
      }
   return comres;
   }

int bma250_get_int2_pad_sel(unsigned char param,unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      switch (param)
         {
         case 0:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT2_PAD_SEL_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_INT2_PAD_LOWG);
            *status = data;
            break;
         case 1:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT2_PAD_SEL_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_INT2_PAD_HIGHG);
            *status = data;
            break;
         case 2:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT2_PAD_SEL_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_INT2_PAD_SLOPE);
            *status = data;
            break;
         case 3:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT2_PAD_SEL_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_INT2_PAD_DB_TAP);
            *status = data;
            break;
         case 4:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT2_PAD_SEL_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_INT2_PAD_SNG_TAP);
            *status = data;
            break;
         case 5:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT2_PAD_SEL_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_INT2_PAD_ORIENT);
            *status = data;
            break;
         case 6:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT2_PAD_SEL_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_INT2_PAD_FLAT);
            *status = data;
            break;
         default:
            break;
         }
      }
   return comres;
   }

int bma250_set_int_src(unsigned char intsrc)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   unsigned char state;
   state = 0x01;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      switch (intsrc)
         {
         case 0:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_UNFILT_INT_SRC_LOWG__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_UNFILT_INT_SRC_LOWG, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_UNFILT_INT_SRC_LOWG__REG, &data, C_One_U8X);
            break;
         case 1:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_UNFILT_INT_SRC_HIGHG__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_UNFILT_INT_SRC_HIGHG, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_UNFILT_INT_SRC_HIGHG__REG, &data, C_One_U8X);
            break;
         case 2:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_UNFILT_INT_SRC_SLOPE__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_UNFILT_INT_SRC_SLOPE, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_UNFILT_INT_SRC_SLOPE__REG, &data, C_One_U8X);
            break;
         case 3:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_UNFILT_INT_SRC_TAP__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_UNFILT_INT_SRC_TAP, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_UNFILT_INT_SRC_TAP__REG, &data, C_One_U8X);
            break;
         case 4:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_UNFILT_INT_SRC_DATA__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_UNFILT_INT_SRC_DATA, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_UNFILT_INT_SRC_DATA__REG, &data, C_One_U8X);
            break;
         default:
            break;
         }
      }
   return comres;
   }

int bma250_get_int_src(unsigned char param,unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      switch (param)
         {
         case 0:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_SRC_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_UNFILT_INT_SRC_LOWG);
            *status = data;
            break;
         case 1:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_SRC_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_UNFILT_INT_SRC_HIGHG);
            *status = data;
            break;
         case 2:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_SRC_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_UNFILT_INT_SRC_SLOPE);
            *status = data;
            break;
         case 3:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_SRC_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_UNFILT_INT_SRC_TAP);
            *status = data;
            break;
         case 4:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_SRC_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_UNFILT_INT_SRC_DATA);
            *status = data;
            break;
         default:
            break;
         }
      }
   return comres;
   }

int bma250_set_int_set(unsigned char intset,unsigned char state)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      switch (intset)
         {
         case 0:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT1_PAD_ACTIVE_LEVEL__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_INT1_PAD_ACTIVE_LEVEL, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_INT1_PAD_ACTIVE_LEVEL__REG, &data, C_One_U8X);
            break;
         case 1:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT1_PAD_OUTPUT_TYPE__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_INT1_PAD_OUTPUT_TYPE, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_INT1_PAD_OUTPUT_TYPE__REG, &data, C_One_U8X);
            break;
         case 2:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT2_PAD_ACTIVE_LEVEL__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_INT2_PAD_ACTIVE_LEVEL, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_INT2_PAD_ACTIVE_LEVEL__REG, &data, C_One_U8X);
            break;
         case 3:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT2_PAD_OUTPUT_TYPE__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_INT2_PAD_OUTPUT_TYPE, state );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_INT2_PAD_OUTPUT_TYPE__REG, &data, C_One_U8X);
            break;
         default:
            break;
         }
      }
   return comres;
   }

int bma250_get_int_set(unsigned char param,unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      switch (param)
         {
         case 0:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_SET_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_INT1_PAD_ACTIVE_LEVEL);
            *status = data;
            break;
         case 1:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_SET_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_INT1_PAD_OUTPUT_TYPE);
            *status = data;
            break;
         case 2:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_SET_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_INT2_PAD_ACTIVE_LEVEL);
            *status = data;
            break;
         case 3:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_SET_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_INT2_PAD_OUTPUT_TYPE);
            *status = data;
            break;
         default:
            break;
         }
      }
   return comres;
   }

int bma250_get_mode_ctrl(unsigned char *mode)
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_MODE_CTRL_REG, &data, C_One_U8X );
      *mode = data;
      }
   return comres;
   }

int bma250_set_low_duration(unsigned char duration)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_LOWG_DUR__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_LOWG_DUR, duration );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_LOWG_DUR__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_low_duration(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_LOW_DURN_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_LOWG_DUR);
      *status = data;
      }
   return comres;
   }

int bma250_set_low_g_threshold(unsigned char threshold)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_LOWG_THRES__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_LOWG_THRES, threshold );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_LOWG_THRES__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_low_g_threshold(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_LOW_THRES_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_LOWG_THRES);
      *status = data;
      }
   return comres;
   }

int bma250_set_high_g_duration(unsigned char duration)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_HIGHG_DUR__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_HIGHG_DUR, duration );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_HIGHG_DUR__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_high_g_duration(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_HIGH_DURN_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_HIGHG_DUR);
      *status = data;
      }
   return comres;
   }

int bma250_set_high_g_threshold(unsigned char threshold)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_HIGHG_THRES__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_HIGHG_THRES, threshold );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_HIGHG_THRES__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_high_g_threshold(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_HIGH_THRES_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_HIGHG_THRES);
      *status = data;
      }
   return comres;
   }

int bma250_set_slope_duration(unsigned char duration)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_SLOPE_DUR__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_SLOPE_DUR, duration );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_SLOPE_DUR__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_slope_duration(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_SLOPE_DURN_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_SLOPE_DUR);
      *status = data;
      }
   return comres;
   }

int bma250_set_slope_threshold(unsigned char threshold)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_SLOPE_THRES__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_SLOPE_THRES, threshold );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_SLOPE_THRES__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_slope_threshold(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_SLOPE_THRES_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_SLOPE_THRES);
      *status = data;
      }
   return comres;
   }

int bma250_set_tap_duration(unsigned char duration)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_TAP_DUR__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_TAP_DUR, duration );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_TAP_DUR__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_tap_duration(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_TAP_PARAM_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_TAP_DUR);
      *status = data;
      }
   return comres;
   }

int bma250_set_tap_shock(unsigned char setval)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_TAP_SHOCK_DURN__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_TAP_SHOCK_DURN, setval );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_TAP_SHOCK_DURN__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_tap_shock(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_TAP_PARAM_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_TAP_SHOCK_DURN);
      *status = data;
      }
   return comres;
   }

int bma250_set_tap_quiet_duration(unsigned char duration)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_TAP_QUIET_DURN__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_TAP_QUIET_DURN, duration );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_TAP_QUIET_DURN__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_tap_quiet(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_TAP_PARAM_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_TAP_QUIET_DURN);
      *status = data;
      }
   return comres;
   }

int bma250_set_tap_threshold(unsigned char threshold)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_TAP_THRES__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_TAP_THRES, threshold );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_TAP_THRES__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_tap_threshold(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_TAP_THRES_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_TAP_THRES);
      *status = data;
      }
   return comres;
   }

int bma250_set_tap_samp(unsigned char samp)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_TAP_SAMPLES__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_TAP_SAMPLES, samp );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_TAP_SAMPLES__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_tap_samp(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_TAP_THRES_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_TAP_SAMPLES);
      *status = data;
      }
   return comres;
   }

int bma250_set_orient_mode(unsigned char mode)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_ORIENT_MODE__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_ORIENT_MODE, mode );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_ORIENT_MODE__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_orient_mode(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_ORIENT_PARAM_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_ORIENT_MODE);
      *status = data;
      }
   return comres;
   }

int bma250_set_orient_blocking(unsigned char samp)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_ORIENT_BLOCK__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_ORIENT_BLOCK, samp );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_ORIENT_BLOCK__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_orient_blocking(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_ORIENT_PARAM_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_ORIENT_BLOCK);
      *status = data;
      }
   return comres;
   }

int bma250_set_orient_hyst(unsigned char orienthyst)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_ORIENT_HYST__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_ORIENT_HYST, orienthyst );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_ORIENT_HYST__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_orient_hyst(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_ORIENT_PARAM_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_ORIENT_HYST);
      *status = data;
      }
   return comres;
   }

int bma250_set_theta_blocking(unsigned char thetablk)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_THETA_BLOCK__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_THETA_BLOCK, thetablk );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_THETA_BLOCK__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_theta_blocking(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_THETA_BLOCK_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_THETA_BLOCK);
      *status = data;
      }
   return comres;
   }

int bma250_set_orient_ex(unsigned char orientex)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_ORIENT_AXIS__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_ORIENT_AXIS, orientex );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_ORIENT_AXIS__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_orient_ex(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_THETA_BLOCK_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_ORIENT_AXIS);
      *status = data;
      }
   return comres;
   }

int bma250_set_theta_flat(unsigned char thetaflat)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_THETA_FLAT__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_THETA_FLAT, thetaflat );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_THETA_FLAT__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_theta_flat(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_THETA_FLAT_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_THETA_FLAT);
      *status = data;
      }
   return comres;
   }

int bma250_set_flat_hold_time(unsigned char holdtime)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_FLAT_HOLD_TIME__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_FLAT_HOLD_TIME, holdtime );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_FLAT_HOLD_TIME__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_flat_hold_time(unsigned char *holdtime )
   {
   int comres= C_Zero_U8X;
   unsigned char data1;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_FLAT_HOLD_TIME_REG, &data1, C_One_U8X );
      data1  = bma250_GET_BITSLICE(data1, bma250_FLAT_HOLD_TIME);
      *holdtime = data1 ;
      }
   return comres;
   }

int bma250_get_low_power_state(unsigned char *Lowpower )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS_LOW_POWER_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_LOW_POWER_MODE_S);
      *Lowpower = data;
      }
   return comres;
   }

int bma250_set_selftest_st(unsigned char selftest)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_SELF_TEST__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_EN_SELF_TEST, selftest );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_SELF_TEST__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_selftest_st(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_SELF_TEST_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_EN_SELF_TEST);
      *status = data;
      }
   return comres;
   }

int bma250_set_selftest_stn(unsigned char stn)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_NEG_SELF_TEST__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_NEG_SELF_TEST, stn );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_NEG_SELF_TEST__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_selftest_stn(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_SELF_TEST_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_NEG_SELF_TEST);
      *status = data;
      }
   return comres;
   }

int bma250_set_selftest_st_amp(unsigned char stamp)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_SELF_TEST_AMP__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_SELF_TEST_AMP, stamp );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_SELF_TEST_AMP__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_selftest_st_amp(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_SELF_TEST_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_SELF_TEST_AMP);
      *status = data;
      }
   return comres;
   }

int bma250_set_ee_w(unsigned char eew)
   {
   unsigned char data;
         int comres;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR ;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_UNLOCK_EE_WRITE_SETTING__REG,&data, C_One_U8X);
      data = bma250_SET_BITSLICE(data, bma250_UNLOCK_EE_WRITE_SETTING, eew);
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_UNLOCK_EE_WRITE_SETTING__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_ee_w(unsigned char *eew)
   {
   int comres;
   if (p_bma250==C_Zero_U8X)
      {
      comres =E_SMB_NULL_PTR ;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EE_WRITE_SETTING_S__REG,eew, C_One_U8X);
      *eew = bma250_GET_BITSLICE(*eew, bma250_EE_WRITE_SETTING_S);
      }
   return comres;
   }

int bma250_set_ee_prog_trig(unsigned char eeprog)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_START_EE_WRITE_SETTING__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_START_EE_WRITE_SETTING, eeprog );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_START_EE_WRITE_SETTING__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_eeprom_writing_status(unsigned char *eewrite )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EEPROM_CTRL_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_EE_WRITE_SETTING_S);
      *eewrite = data;
      }
   return comres;
   }

int bma250_set_update_image(void)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   unsigned char update;
   update = 0x01;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_UPDATE_IMAGE__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_UPDATE_IMAGE, update );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_UPDATE_IMAGE__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_set_3wire_spi(void)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   unsigned char update;
   update = 0x01;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_SPI_MODE_3__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_EN_SPI_MODE_3, update );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_SPI_MODE_3__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_3wire_spi(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_SERIAL_CTRL_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_EN_SPI_MODE_3);
      *status = data;
      }
   return comres;
   }

int bma250_set_i2c_wdt_timer(unsigned char timedly)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_I2C_WATCHDOG_PERIOD__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_I2C_WATCHDOG_PERIOD, timedly );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_I2C_WATCHDOG_PERIOD__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_i2c_wdt_timer(unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_SERIAL_CTRL_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_I2C_WATCHDOG_PERIOD);
      *status = data;
      }
   return comres;
   }



int bma250_set_hp_en(unsigned char param,unsigned char hpval)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      switch (param)
         {
         case 0:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_SLOW_COMP_X__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_SLOW_COMP_X, hpval );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_SLOW_COMP_X__REG, &data, C_One_U8X);
            break;
         case 1:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_SLOW_COMP_Y__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_SLOW_COMP_Y, hpval );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_SLOW_COMP_Y__REG, &data, C_One_U8X);
            break;
         case 2:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_SLOW_COMP_Z__REG, &data, C_One_U8X );
            data = bma250_SET_BITSLICE(data, bma250_EN_SLOW_COMP_Z, hpval );
            comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_SLOW_COMP_Z__REG, &data, C_One_U8X);
            break;
         default:
            break;
         }
      }
   return comres;
   }

int bma250_get_hp_en(unsigned char param,unsigned char *status )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      switch (param)
         {
         case 0:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_OFFSET_CTRL_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_SLOW_COMP_X);
            *status = data;
            break;
         case 1:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_OFFSET_CTRL_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_SLOW_COMP_Y);
            *status = data;
            break;
         case 2:
            comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_OFFSET_CTRL_REG, &data, C_One_U8X );
            data = bma250_GET_BITSLICE(data, bma250_EN_SLOW_COMP_Z);
            *status = data;
            break;
         default:
            break;
         }
      }
   return comres;
   }

int bma250_get_cal_ready(unsigned char *calrdy )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_OFFSET_CTRL_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_FAST_COMP_RDY_S);
      *calrdy = data;
      }
   return comres;
   }

int bma250_set_cal_trigger(unsigned char caltrigger)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EN_FAST_COMP__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_EN_FAST_COMP, caltrigger );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_EN_FAST_COMP__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_set_offset_reset(void)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   unsigned char offsetreset;
   offsetreset = C_One_U8X;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_RESET_OFFSET_REGS__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_RESET_OFFSET_REGS, offsetreset );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_RESET_OFFSET_REGS__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_set_offset_cutoff(unsigned char offsetcutoff)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_COMP_CUTOFF__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_COMP_CUTOFF, offsetcutoff );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_COMP_CUTOFF__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_offset_cutoff(unsigned char *cutoff )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_OFFSET_PARAMS_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_COMP_CUTOFF);
      *cutoff = data;
      }
   return comres;
   }

int bma250_set_offset_target_x(unsigned char offsettarget)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_COMP_TARGET_OFFSET_X__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_COMP_TARGET_OFFSET_X, offsettarget );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_COMP_TARGET_OFFSET_X__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_offset_target_x(unsigned char *offsettarget )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_OFFSET_PARAMS_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_COMP_TARGET_OFFSET_X);
      *offsettarget = data;
      }
   return comres;
   }

int bma250_set_offset_target_y(unsigned char offsettarget)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_COMP_TARGET_OFFSET_Y__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_COMP_TARGET_OFFSET_Y, offsettarget );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_COMP_TARGET_OFFSET_Y__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_offset_target_y(unsigned char *offsettarget )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_OFFSET_PARAMS_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_COMP_TARGET_OFFSET_Y);
      *offsettarget = data;
      }
   return comres;
   }

int bma250_set_offset_target_z(unsigned char offsettarget)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_COMP_TARGET_OFFSET_Z__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_COMP_TARGET_OFFSET_Z, offsettarget );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_COMP_TARGET_OFFSET_Z__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_offset_target_z(unsigned char *offsettarget )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_OFFSET_PARAMS_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_COMP_TARGET_OFFSET_Z);
      *offsettarget = data;
      }
   return comres;
   }

int bma250_set_offset_filt_x(unsigned char offsetfilt)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      data =  offsetfilt;
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_OFFSET_FILT_X_REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_offset_filt_x(unsigned char *offsetfilt )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_OFFSET_FILT_X_REG, &data, C_One_U8X );
      *offsetfilt = data;
      }
   return comres;
   }

int bma250_set_offset_filt_y(unsigned char offsetfilt)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      data =  offsetfilt;
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_OFFSET_FILT_Y_REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_offset_filt_y(unsigned char *offsetfilt )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_OFFSET_FILT_Y_REG, &data, C_One_U8X );
      *offsetfilt = data;
      }
   return comres;
   }

int bma250_set_offset_filt_z(unsigned char offsetfilt)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      data =  offsetfilt;
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_OFFSET_FILT_Z_REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_offset_filt_z(unsigned char *offsetfilt )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_OFFSET_FILT_Z_REG, &data, C_One_U8X );
      *offsetfilt = data;
      }
   return comres;
   }

int bma250_set_offset_unfilt_x(unsigned char offsetfilt)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      data =  offsetfilt;
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_OFFSET_UNFILT_X_REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_offset_unfilt_x(unsigned char *offsetfilt )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_OFFSET_UNFILT_X_REG, &data, C_One_U8X );
      *offsetfilt = data;
      }
   return comres;
   }

int bma250_set_offset_unfilt_y(unsigned char offsetfilt)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      data =  offsetfilt;
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_OFFSET_UNFILT_Y_REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_offset_unfilt_y(unsigned char *offsetfilt )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_OFFSET_UNFILT_Y_REG, &data, C_One_U8X );
      *offsetfilt = data;
      }
   return comres;
   }

int bma250_set_offset_unfilt_z(unsigned char offsetfilt)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      data =  offsetfilt;
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_OFFSET_UNFILT_Z_REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_offset_unfilt_z(unsigned char *offsetfilt )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_OFFSET_UNFILT_Z_REG, &data, C_One_U8X );
      *offsetfilt = data;
      }
   return comres;
   }
int bma250_set_Int_Mode(unsigned char Mode )
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_MODE_SEL__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_INT_MODE_SEL, Mode );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_INT_MODE_SEL__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_Int_Mode(unsigned char * Mode )
   {
   int comres= C_Zero_U8X;
   unsigned char data1;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_MODE_SEL__REG, &data1, C_One_U8X );
      data1  = bma250_GET_BITSLICE(data1, bma250_INT_MODE_SEL);
      *Mode = data1 ;
      }
   return comres;
   }

int bma250_set_Int_Enable(unsigned char InterruptType , unsigned char value )
   {
   int comres=C_Zero_U8X;
   unsigned char data1,data2;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_ENABLE1_REG, &data1, C_One_U8X );
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_INT_ENABLE2_REG, &data2, C_One_U8X );
      
      value = value & C_One_U8X;
      switch (InterruptType)
         {
         case C_Zero_U8X:
            /* Low G Interrupt  */
            data2 = bma250_SET_BITSLICE(data2, bma250_EN_LOWG_INT, value );
            break;
         case C_One_U8X:
            /* High G X Interrupt */
            /* EasyCASE - */
            data2 = bma250_SET_BITSLICE(data2, bma250_EN_HIGHG_X_INT, value );
            break;
         case C_Two_U8X:
            /* High G Y Interrupt */
            /* EasyCASE - */
            data2 = bma250_SET_BITSLICE(data2, bma250_EN_HIGHG_Y_INT, value );
            break;
         case C_Three_U8X:
            /* High G Z Interrupt */
            /* EasyCASE - */
            data2 = bma250_SET_BITSLICE(data2, bma250_EN_HIGHG_Z_INT, value );
            break;
         case C_Four_U8X:
            /* New Data Interrupt  */
            /* EasyCASE - */
            data2 = bma250_SET_BITSLICE(data2, bma250_EN_NEW_DATA_INT, value );
            break;
         case C_Five_U8X:
            /* Slope X Interrupt */
            /* EasyCASE - */
            data1 = bma250_SET_BITSLICE(data1, bma250_EN_SLOPE_X_INT, value );
            break;
         case C_Six_U8X:
            /* Slope Y Interrupt */
            /* EasyCASE - */
            data1 = bma250_SET_BITSLICE(data1, bma250_EN_SLOPE_Y_INT, value );
            break;
         case C_Seven_U8X:
            /* Slope Z Interrupt */
            /* EasyCASE - */
            data1 = bma250_SET_BITSLICE(data1, bma250_EN_SLOPE_Z_INT, value );
            break;
         case C_Eight_U8X:
            /* Single Tap Interrupt */
            /* EasyCASE - */
            data1 = bma250_SET_BITSLICE(data1, bma250_EN_SINGLE_TAP_INT, value );
            break;
         case C_Nine_U8X:
            /* Double Tap Interrupt */
            /* EasyCASE - */
            data1 = bma250_SET_BITSLICE(data1, bma250_EN_DOUBLE_TAP_INT, value );
            break;
         case C_Ten_U8X:
            /* Orient Interrupt  */
            /* EasyCASE - */
            data1 = bma250_SET_BITSLICE(data1, bma250_EN_ORIENT_INT, value );
            break;
         case C_Eleven_U8X:
            /* Flat Interrupt */
            /* EasyCASE - */
            data1 = bma250_SET_BITSLICE(data1, bma250_EN_FLAT_INT, value );
            break;
         default:
            break;
         }
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_INT_ENABLE1_REG, &data1, C_One_U8X );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_INT_ENABLE2_REG, &data2, C_One_U8X );
      }
   return comres;
   }

int bma250_write_ee(unsigned char addr, unsigned char data)
   {
   int comres;
   unsigned char ee_busy;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      if (p_bma250->delay_msec == C_Zero_U8X)
         {
         comres = E_SMB_NULL_PTR;
         }
      else
         {
         comres = bma250_get_ee_w( & ee_busy );
         if (ee_busy)
            {
            comres = E_EEPROM_BUSY;
            }
         else
            {
            comres += p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, addr, &data, C_One_U8X );
                addr += bma250_EEP_OFFSET;
                comres += p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, addr, &data, C_One_U8X );
                p_bma250->delay_msec( bma250_EE_W_DELAY );
            }
         }
      }
   return comres;
   }

int bma250_set_low_hy(unsigned char hysval)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_LOWG_HYST__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_LOWG_HYST, hysval );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_LOWG_HYST__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_set_high_hy(unsigned char hysval)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_HIGHG_HYST__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_HIGHG_HYST, hysval );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_HIGHG_HYST__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_set_low_mode(unsigned char state)
   {
   int comres=C_Zero_U8X;
   unsigned char data;
   if (p_bma250==C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_LOWG_INT_MODE__REG, &data, C_One_U8X );
      data = bma250_SET_BITSLICE(data, bma250_LOWG_INT_MODE, state );
      comres = p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_LOWG_INT_MODE__REG, &data, C_One_U8X);
      }
   return comres;
   }

int bma250_get_update_image_status(unsigned char *imagestatus )
   {
   int comres = C_Zero_U8X ;
   unsigned char data;
   if (p_bma250 == C_Zero_U8X)
      {
      comres = E_SMB_NULL_PTR;
      }
   else
      {
      comres = p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_EEPROM_CTRL_REG, &data, C_One_U8X );
      data = bma250_GET_BITSLICE(data, bma250_UPDATE_IMAGE);
      *imagestatus = data;
      }
   return comres;
   }

int bma250_set_accelerometer(unsigned char enable)
{
	int err = 0;
	struct bma250_data *data = i2c_get_clientdata(bma250_client);

#ifdef BMA250_DEBUG
	printk(KERN_INFO "bma250_set_accelerometer(%d)\n", enable);
#endif

	if(enable) {
		if(data->sensor_enable == 0) {
			err = bma250_set_mode(bma250_MODE_NORMAL);
		}

		data->sensor_enable |= BMA250_ACCELERMETER;
	}else
	{
		data->sensor_enable &= ~BMA250_ACCELERMETER;

		if(data->sensor_enable == 0) {
			err = bma250_set_mode(bma250_MODE_SUSPEND);
		}
	}

	return 0;
}

int bma250_set_gesture(unsigned char enable)
{
	struct bma250_data *data = i2c_get_clientdata(bma250_client);
	unsigned char status_flat_orient;

#ifdef BMA250_DEBUG
	printk(KERN_INFO "bma250_set_gesture(%d)\n", enable);
#endif

	if(enable) {
		if(data->sensor_enable & BMA250_GESTURE)
			return 0;

		if(data->sensor_enable == 0) {
			bma250_set_mode(bma250_MODE_NORMAL);
		}
		data->sensor_enable |= BMA250_GESTURE;

		p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS_ORIENT_HIGH_REG, &status_flat_orient, C_One_U8X );

		if(status_flat_orient & 0x40)
			data->flip_Status = 0;
		else
			data->flip_Status = 1;

		bma250_interrupt_config(1);
		enable_irq(data->irq);
	}else
	{
		if((data->sensor_enable & BMA250_GESTURE) == 0)
			return 0;

		data->sensor_enable &= ~BMA250_GESTURE;

		disable_irq(data->irq);
		bma250_interrupt_config(0);

		if(data->sensor_enable == 0) {
			bma250_set_mode(bma250_MODE_SUSPEND);
		}
		mdelay(10);
	}

	return 0;
}

int bma250_set_calibration(unsigned char enable)
{
	struct bma250_data *data = i2c_get_clientdata(bma250_client);

#ifdef BMA250_DEBUG
	printk(KERN_INFO "bma250_set_calibration(%d)\n", enable);
#endif

	if(enable) {
		//2011-06-16 by baborobo  the Calibration need the enable of Accelrometer
		if(data->sensor_enable == 0) {
			bma250_set_mode(bma250_MODE_NORMAL);
		}

		data->sensor_enable |= BMA250_CALIBRATION;
	}else
	{
		data->sensor_enable &= ~BMA250_CALIBRATION;

		if(data->sensor_enable == 0) {
			bma250_set_mode(bma250_MODE_SUSPEND);
		}
	}

	return 0;
}

#ifdef BMA250_HAS_EARLYSUSPEND
static void bma250_early_suspend(struct early_suspend *h);
static void bma250_late_resume(struct early_suspend *h);
#endif


/* i2c operation for bma250 API */
static char bma250_i2c_write(unsigned char reg_addr, unsigned char *data, unsigned char len);
static char bma250_i2c_read(unsigned char reg_addr, unsigned char *data, unsigned char len);
static void bma250_i2c_delay(unsigned int msec);

#ifdef BMA250_ENABLE_IRQ
// 2011-06-08 by baborobo@lge.com
// for vrey fast flip detectting
void bma250_flip_timefunc(unsigned long arg )
{
    struct bma250_data *pdata;
    int flip_type = 3;
    unsigned char status_flat_orient;
    unsigned char flat_orient;
//    signed char   status_accel_z;
    short  status_accel_z;
    
   if( arg )
   {
        pdata = (struct bma250_data *) arg;

//        p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_Z_AXIS_MSB_REG, &status_accel_z, C_One_U8X );
		bma250_read_accel_z(&status_accel_z);
        p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS_ORIENT_HIGH_REG, &status_flat_orient, C_One_U8X );
#ifdef BMA250_DEBUG
        printk(KERN_INFO "%s\n",__FUNCTION__);
        printk(KERN_INFO "BMA250: status_accel_z = %d \n", status_accel_z); 
        printk(KERN_INFO "BMA250: status_flat_orient = 0x%02x \n", status_flat_orient); 
#endif

#if 0
        if(status_accel_z > 30) {
            flip_type = 1;
        } else if(status_accel_z < -30)
        {
            flip_type = 0;
        }
#else
        flat_orient = status_flat_orient&0x40;
        if(flat_orient == 0x00) {
            flip_type = 1;
        }else
		if(flat_orient == 0x40 && (status_accel_z < -150) && (status_accel_z > -280)) {
		// bluewave96@lge.com checking for more than 30 degree slope
            flip_type = 0;
        }
#endif

        if((pdata->flip_Status != flip_type) 
            && (flip_type <= 1)) {
            if(pdata->sensor_enable & BMA250_GESTURE) {
                input_report_key(pdata->input_dev, BTN_TASK, flip_type);
                input_sync(pdata->input_dev);
#ifdef BMA250_DEBUG
            printk(KERN_INFO "BMA250: send the Flip event - %d !!!\n", flip_type);
#endif
            }
            pdata->flip_Status = flip_type;
        }
   }
}

void bma250_register_flip_timer( struct bma250_data *pdata, unsigned long timeover)
{
     del_timer( &(pdata->flip_check_timer) );

     init_timer( &pdata->flip_check_timer );
     pdata->flip_check_timer.expires  = get_jiffies_64() + timeover;
     pdata->flip_check_timer.data     = (unsigned long) pdata;
     pdata->flip_check_timer.function = bma250_flip_timefunc;
     add_timer( &(pdata->flip_check_timer) );
}

static void bma250_work_func(struct work_struct *work)
{
    int tap_type = -1;
    int tap_num = -1;
    int flip_type = 3;
    unsigned char status_interrupt;
    unsigned char status_tap_slop;
    unsigned char status_flat_orient;
    unsigned char flat_orient;
    unsigned char tap_slop;
    struct bma250_data *data;
	short status_accel_z;

    if(bma250_client == NULL) {
        printk(KERN_INFO "I2C driver not install\n"); 
        return;
    }
    data = i2c_get_clientdata(bma250_client);

    p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS1_REG, &status_interrupt, C_One_U8X );
    p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS_ORIENT_HIGH_REG, &status_flat_orient, C_One_U8X );
    p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS_TAP_SLOPE_REG, &status_tap_slop, C_One_U8X );
	bma250_read_accel_z(&status_accel_z);	
#ifdef BMA250_DEBUG
    printk(KERN_INFO "BMA250: status_interrupt = 0x%x \n", status_interrupt); 
    printk(KERN_INFO "BMA250: status_flat_orient = 0x%x \n", status_flat_orient); 
    printk(KERN_INFO "BMA250: status_tap_slop = 0x%x \n", status_tap_slop); 
#endif

    flat_orient = status_flat_orient & 0xC0;
    tap_slop = status_tap_slop & 0xF0;

    if(status_interrupt & bma250_FLAT_INT_S__MSK)
    {
		// bluewave96@lge.com Excluding abnormal value of accel z
		if ( (status_accel_z < 280) && (status_accel_z > -280)) {
        	if(flat_orient == 0x80) {
        	    flip_type = 1;
        	}else
        	if(flat_orient == 0xC0) {
        	    flip_type = 0;
        	}
		}
    }else if(status_interrupt & bma250_ORIENT_INT_S__MSK) {
        // 2011-06-08 by baborobo@lge.com
        // for very fast flip detectting
        bma250_register_flip_timer(data, (3*HZ/10));	//300msec
    }else
    if(status_interrupt & (bma250_SINGLE_TAP_INT_S__MSK|bma250_DOUBLE_TAP_INT_S__MSK)) 
    {
        if(status_interrupt & bma250_DOUBLE_TAP_INT_S__MSK)
              tap_num = 2;
        else
              tap_num = 1;

        switch (tap_slop & 0xF0) {
        case 0x10 : // positive x-axis (BIT4)- Left Tap
              tap_type = BTN_LEFT;
            break;

        case 0x20 : // positive y-axis (BIT5)- Bottom Tap
            tap_type = BTN_EXTRA;
            break;

        case 0x40 : // positive z-axis (BIT6)- Back Tap
            tap_type = BTN_BACK;
            break;

        case 0x90 : // negative x-axis (BIT7|BIT4)- Right Tap
              tap_type = BTN_RIGHT;
            break;

        case 0xa0 : // negative y-axis (BIT7|BIT5)- Top Tap
            tap_type = BTN_SIDE;
            break;

        case 0xc0 : // negative z-axis (BIT7|BIT6)- Front Tap
            tap_type = BTN_FORWARD;
            break;

        default : // error - don't care
            tap_type = -1;
            tap_num = -1;
            break;
        }
    }

    if(tap_type >= 0 && tap_num > 0) {
        input_report_key(data->input_dev, tap_type, tap_num);
        input_report_key(data->input_dev, tap_type, 0);
        input_sync(data->input_dev);
#ifdef BMA250_DEBUG
        printk(KERN_INFO "BMA250: send the  Tap event - %d !!!\n", tap_type);
#endif
    }

    if((data->flip_Status != flip_type) 
        && (flip_type <= 1)) {
        input_report_key(data->input_dev, BTN_TASK, flip_type);
        input_sync(data->input_dev);
#ifdef BMA250_DEBUG
        printk(KERN_INFO "BMA250: send the Flip event - %d !!!\n", flip_type);
#endif

        data->flip_Status = flip_type;
    }
    return;
}

static const unsigned char init_config[] = {
//  reg address,                 data
    bma250_INT1_PAD_SEL_REG,     bma250_EN_INT1_PAD_ORIENT__MSK
                                |bma250_EN_INT1_PAD_SNG_TAP__MSK
                                |bma250_EN_INT1_PAD_FLAT__MSK,

   /* Setting for Tap Function */
    bma250_TAP_PARAM_REG,        0x47, // Tap-Quiet = 0 / Tap-Shock = 1 / Tap-Dur = 7
    bma250_TAP_THRES_REG,        0x10, // Tap-Sample = 0 / Tap-Thresold = 0x0D
    bma250_INT_SRC_REG,          0x00, // 0x10

    /* Setting for Flip Function */
    bma250_ORIENT_PARAM_REG,     0x14,//0x18|0x02,//0x14,
//    bma250_THETA_BLOCK_REG,      0x3f,//0x14,
//    bma250_FLAT_HOLD_TIME_REG,   0x00,
    bma250_THETA_FLAT_REG,       0x10,

    0xff,                        0xff
};

static int bma250_init_config(void)
{
    unsigned char reg, reg_data;
    unsigned char i = 0;
    struct bma250_data *data;	

    data = i2c_get_clientdata(bma250_client);

#ifdef BMA250_DEBUG
    printk(KERN_INFO "%s\n",__FUNCTION__);
#endif
//    bma250_set_bandwidth(4);		//bandwidth 125Hz
    bma250_set_bandwidth(5);		//bandwidth 25Hz - bma250_BW_125HZ
    bma250_set_range(bma250_RANGE_2G);			//range +/-2G

    while(init_config[i*2] != 0xff) {
        reg = init_config[i*2];
        reg_data = init_config[i*2+1];
#ifdef BMA250_DEBUG
        printk("BMA250: init_config[%d] = 0x%x / 0x%x \n", i, reg, reg_data);
#endif
        p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, reg, &reg_data, C_One_U8X);
        i+=1;
    }
    return 0;
} 

static int bma250_interrupt_config(int enable)
{
    unsigned char int_enable;
    struct bma250_data *data;	

    data = i2c_get_clientdata(bma250_client);

#ifdef BMA250_DEBUG
    printk(KERN_INFO "%s\n",__FUNCTION__);
#endif
	if(enable)	
		int_enable = bma250_EN_FLAT_INT__MSK|bma250_EN_ORIENT_INT__MSK
			 |bma250_EN_SINGLE_TAP_INT__MSK;
	else
		int_enable = 0;

	p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, bma250_INT_ENABLE1_REG, &int_enable, C_One_U8X);

    return 0;
} 

static irqreturn_t bma250_irq_handler(int irq, void *_id)
{
    unsigned long delay;
    struct bma250_data *data;	

    if(bma250_client == NULL)
        return IRQ_HANDLED;

    data = i2c_get_clientdata(bma250_client);

    spin_lock(&data->lock);
    delay = msecs_to_jiffies(data->debounce);
    queue_delayed_work(acceler_wq, &data->dwork, delay);

    spin_unlock(&data->lock);
    return IRQ_HANDLED;
}
#endif /* BMA250_ENABLE_IRQ */
 
/*	i2c delay routine for eeprom	*/
static inline void bma250_i2c_delay(unsigned int msec)
{
	mdelay(msec);
}

/*	i2c write routine for bma250	*/
static inline char bma250_i2c_write(unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	s32 dummy;
#ifndef BMA250_SMBUS
	unsigned char buffer[2];
#endif

	if( bma250_client == NULL )	/*	No global client pointer?	*/
		return -1;

	while(len--)
	{
#ifdef BMA250_SMBUS
		dummy = i2c_smbus_write_byte_data(bma250_client, reg_addr, *data);
#else
		buffer[0] = reg_addr;
		buffer[1] = *data;
		dummy = i2c_master_send(bma250_client, (char*)buffer, 2);
#endif
		reg_addr++;
		data++;
		if(dummy < 0)
			return -1;
	}
	return 0;
}

/*	i2c read routine for bma250	*/
static inline char bma250_i2c_read(unsigned char reg_addr, unsigned char *data, unsigned char len) 
{
	s32 dummy;

	if( bma250_client == NULL )	/*	No global client pointer?	*/
		return -1;

	while(len--)
	{        
#ifdef BMA250_SMBUS
		dummy = i2c_smbus_read_byte_data(bma250_client, reg_addr);
		if(dummy < 0)
			return -1;
		*data = dummy & 0x000000ff;
#else
		dummy = i2c_master_send(bma250_client, (char*)&reg_addr, 1);
		if(dummy < 0)
			return -1;
		dummy = i2c_master_recv(bma250_client, (char*)data, 1);
		if(dummy < 0)
			return -1;
#endif
		reg_addr++;
		data++;
	}
	return 0;
}


/*	read command for BMA250 device file	*/
static ssize_t bma250_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{	
	bma250acc_t acc;	
	int ret;
	if( bma250_client == NULL )
	{
#ifdef BMA250_DEBUG
		printk(KERN_INFO "I2C driver not install\n");
#endif
		return -1;
	}
	bma250_read_accel_xyz(&acc);
#ifdef BMA250_DEBUG
	printk(KERN_INFO "BMA250: X/Y/Z axis: %-8d %-8d %-8d\n" ,
		(int)acc.x, (int)acc.y, (int)acc.z);  
#endif

	if( count != sizeof(acc) )
	{
		return -1;
	}
	ret = copy_to_user(buf,&acc, sizeof(acc));
	if( ret != 0 )
	{
#ifdef BMA250_DEBUG
	printk(KERN_INFO "BMA250: copy_to_user result: %d\n", ret);
#endif
	}


   
	return sizeof(acc);
}

/*	write command for BMA250 device file	*/
static ssize_t bma250_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
	if( bma250_client == NULL )
		return -1;
#ifdef BMA250_DEBUG
	printk(KERN_INFO "BMA250 should be accessed with ioctl command\n");
#endif
	return 0;
}


static unsigned int bma250_poll(struct file *file, poll_table *wait)
{
    unsigned int mask=0;
    if( bma250_client == NULL)
	{
#ifdef BMA250_DEBUG
		printk(KERN_INFO "I2C driver not install\n"); 
#endif
		return -1;
	}
    mask |= POLLIN|POLLRDNORM|POLLOUT|POLLWRNORM;

#ifdef BMA250_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);	
#endif
    return mask;
}


/*	open command for BMA250 device file	*/
static int bma250_open(struct inode *inode, struct file *file)
{
#ifdef BMA250_DEBUG
		printk(KERN_INFO "%s\n",__FUNCTION__); 
#endif

	if( bma250_client == NULL)
	{
#ifdef BMA250_DEBUG
		printk(KERN_INFO "I2C driver not install\n"); 
#endif
		return -1;
	}

#ifdef BMA250_DEBUG
	printk(KERN_INFO "BMA250 has been opened\n");
#endif
	return 0;
}

/*	release command for BMA250 device file	*/
static int bma250_close(struct inode *inode, struct file *file)
{
#ifdef BMA250_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);	
#endif
	return 0;
}


/*	ioctl command for BMA250 device file	*/
/* LGE_UPDATE_S 20111216 kideok.kim@lge.com kernel 3.0.x Adaptation */
#if 0
static int bma250_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
#else
static long bma250_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#endif
/* LGE_UPDATE_E 20111216 kideok.kim@lge.com kernel 3.0.x Adaptation */
{
	int err = 0;
	unsigned char data[6];
/* LGE_CHANGE_S [adwardk.kim@lge.com] 2011-01-25 */	
	unsigned int cmd_ad=0;
/* LGE_CHANGE_E [adwardk.kim@lge.com] 2011-01-25 */
	struct bma250_data* pdata;
	pdata = i2c_get_clientdata(bma250_client);


#ifdef BMA250_DEBUG
//	printk(KERN_INFO "%s\n",__FUNCTION__);	
#endif

	/* check cmd */
	if(_IOC_TYPE(cmd) != BMA250_IOC_MAGIC)	
	{
#ifdef BMA250_DEBUG		
		printk(KERN_INFO "cmd magic type error\n");
#endif
		return -ENOTTY;
	}
	if(_IOC_NR(cmd) > BMA250_IOC_MAXNR)
	{
#ifdef BMA250_DEBUG
		printk(KERN_INFO "cmd number error\n");
#endif
		return -ENOTTY;
	}

	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
	if(err)
	{
#ifdef BMA250_DEBUG
		printk(KERN_INFO "cmd access_ok error\n");
#endif
		return -EFAULT;
	}
	/* check bam150_client */
	if( bma250_client == NULL)
	{
#ifdef BMA250_DEBUG
		printk(KERN_INFO "I2C driver not install\n"); 
#endif
		return -EFAULT;
	}
/* LGE_CHANGE_S [adwardk.kim@lge.com] 2011-01-24 */
/* ------ explan by ad --------
ioctl 32bit data
----------------------------------------------
2bit    14bit          8bit        8bit     
R/W   data size    magic number  cmd number
-----------------------------------------------
ex) BMA250_READ_ACCEL_XYZ => c002 420d
 c0          02          42(B)      0d
---------------------------------------------
1010 0000   0000 0010   0100 0010  0000 1101
*/
	cmd_ad=cmd&0x000000ff;
/* LGE_CHANGE_E [adwardk.kim@lge.com] 2011-01-24 */

	switch(cmd)
	{
	case BMA250_SOFT_RESET:
		err = bma250_soft_reset();
		return err;

	case BMA250_SET_RANGE:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_range(*data);
		return err;

	case BMA250_GET_RANGE:
		err = bma250_get_range(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA250_SET_MODE:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_mode(*data);
		return err;

	case BMA250_GET_MODE:
		err = bma250_get_mode(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	
	case BMA250_SET_BANDWIDTH:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_bandwidth(*data);
		return err;

	case BMA250_GET_BANDWIDTH:
		err = bma250_get_bandwidth(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;


    case BMA250_READ_REG:
		if(copy_from_user(data,(unsigned char*)arg,3)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_read_reg(data[0], data+1,data[2]);
		if(copy_to_user((unsigned char*)arg,data,3)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA250_WRITE_REG:
		if(copy_from_user(data,(unsigned char*)arg,3)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_write_reg(data[0], data+1,data[2]);
		return err;



	case BMA250_RESET_INTERRUPT:
		err = bma250_reset_interrupt();
		return err;

	case BMA250_READ_ACCEL_X:
		err = bma250_read_accel_x((short*)data);
		if(copy_to_user((short*)arg,(short*)data,2)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA250_READ_ACCEL_Y:
		err = bma250_read_accel_y((short*)data);
		if(copy_to_user((short*)arg,(short*)data,2)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA250_READ_ACCEL_Z:
		err = bma250_read_accel_z((short*)data);
		if(copy_to_user((short*)arg,(short*)data,2)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA250_GET_INTERRUPTSTATUS1:
		err = bma250_get_interruptstatus1(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_GET_INTERRUPTSTATUS2:
		err = bma250_get_interruptstatus2(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_READ_ACCEL_XYZ:
		err = bma250_read_accel_xyz((bma250acc_t*)data);
/* LGE_CHANGE_S [adwardk.kim@lge.com] 2011-02-08 */
		//if(copy_to_user((bma250acc_t*)arg,(bma250acc_t*)data,6)!=0)
		if(copy_to_user((bma250acc_t*)arg,(bma250acc_t*)data,sizeof(int)*3)!=0)
/* LGE_CHANGE_E [adwardk.kim@lge.com] 2011-02-08 */
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to error\n");
#endif
			return -EFAULT;
		}
		return err;




    case BMA250_GET_LOW_G_INTERRUPT:
		err = bma250_get_Low_G_interrupt(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_GET_HIGH_G_INTERRUPT:
		err = bma250_get_High_G_Interrupt(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_GET_SLOPE_INTERRUPT:
		err = bma250_get_slope_interrupt(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_GET_DOUBLE_TAP_INTERRUPT:
		err = bma250_get_double_tap_interrupt(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_GET_SINGLE_TAP_INTERRUPT:
		err = bma250_get_single_tap_interrupt(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_GET_ORIENT_INTERRUPT:
		err = bma250_get_orient_interrupt(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_GET_FLAT_INTERRUPT:
		err = bma250_get_flat_interrupt(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_GET_DATA_INTERRUPT:
		err = bma250_get_data_interrupt(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_GET_SLOPE_FIRST:
		if(copy_from_user(data,(unsigned char*)arg,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_get_slope_first(data[0], data+1);
		if(copy_to_user((unsigned char*)arg,data,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_GET_SLOPE_SIGN:
		err = bma250_get_slope_sign(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_GET_TAP_FIRST:
		if(copy_from_user(data,(unsigned char*)arg,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_get_tap_first(data[0], data+1);
		if(copy_to_user((unsigned char*)arg,data,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_GET_TAP_SIGN:
		err = bma250_get_tap_sign(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_GET_HIGH_FIRST:
		if(copy_from_user(data,(unsigned char*)arg,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_get_HIGH_first(data[0], data+1);
		if(copy_to_user((unsigned char*)arg,data,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_GET_HIGH_SIGN:
		err = bma250_get_HIGH_sign(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

     case BMA250_GET_ORIENT_STATUS:
		err = bma250_get_orient_status(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_GET_ORIENT_FLAT_STATUS:
		err = bma250_get_orient_flat_status(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

     case BMA250_GET_SLEEP_DURATION:
		err = bma250_get_sleep_duration(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_SLEEP_DURATION:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_sleep_duration(*data);
		return err;

    case BMA250_SET_SUSPEND:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_suspend(*data);
		return err;

    case BMA250_GET_SUSPEND:
		err = bma250_get_suspend(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;


    case BMA250_SET_LOWPOWER:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_lowpower(*data);
		return err;

    case BMA250_GET_LOWPOWER_EN:
		err = bma250_get_lowpower_en(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_LOW_NOISE_CTRL:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_low_noise_ctrl(*data);
		return err;

    case BMA250_GET_LOW_NOISE_CTRL:
		err = bma250_get_low_noise_ctrl(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_SHADOW_DISABLE:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_shadow_disable(*data);
		return err;

    case BMA250_GET_SHADOW_DISABLE:
		err = bma250_get_shadow_disable(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_UNFILT_ACC:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_unfilt_acc(*data);
		return err;

    case BMA250_GET_UNFILT_ACC:
		err = bma250_get_unfilt_acc(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_ENABLE_TAP_INTERRUPT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_enable_tap_interrupt(*data);
		return err;

    case BMA250_GET_ENABLE_TAP_INTERRUPT:
		if(copy_from_user(data,(unsigned char*)arg,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_get_enable_tap_interrupt(data[0], data+1);
		if(copy_to_user((unsigned char*)arg,data,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;



    case BMA250_SET_ENABLE_HIGH_G_INTERRUPT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_enable_high_g_interrupt(*data);
		return err;

    case BMA250_GET_ENABLE_HIGH_G_INTERRUPT:
		if(copy_from_user(data,(unsigned char*)arg,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_get_enable_high_g_interrupt(data[0], data+1);
		if(copy_to_user((unsigned char*)arg,data,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;


    case BMA250_SET_ENABLE_SLOPE_INTERRUPT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_enable_slope_interrupt(*data);
		return err;

    case BMA250_GET_ENABLE_SLOPE_INTERRUPT:
		if(copy_from_user(data,(unsigned char*)arg,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_get_enable_slope_interrupt(data[0], data+1);
		if(copy_to_user((unsigned char*)arg,data,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_ENABLE_LOW_G_INTERRUPT:
		err = bma250_set_enable_low_g_interrupt();
		return err;

        
    case BMA250_GET_ENABLE_LOW_G_INTERRUPT:
		err = bma250_get_enable_low_g_interrupt(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_ENABLE_DATA_INTERRUPT:
		err = bma250_set_enable_data_interrupt();
		return err;

        
    case BMA250_GET_ENABLE_DATA_INTERRUPT:
		err = bma250_get_enable_data_interrupt(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;



    case BMA250_SET_INT1_PAD_SEL:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_int1_pad_sel(*data);
		return err;

    case BMA250_GET_INT1_PAD_SEL:
		if(copy_from_user(data,(unsigned char*)arg,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_get_int1_pad_sel(data[0], data+1);
		if(copy_to_user((unsigned char*)arg,data,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_INT_DATA_SEL:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_int_data_sel(*data);
		return err;

    case BMA250_GET_INT_DATA_SEL:
		if(copy_from_user(data,(unsigned char*)arg,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_get_int_data_sel(data[0], data+1);
		if(copy_to_user((unsigned char*)arg,data,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_INT2_PAD_SEL:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_int2_pad_sel(*data);
		return err;

    case BMA250_GET_INT2_PAD_SEL:
		if(copy_from_user(data,(unsigned char*)arg,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_get_int2_pad_sel(data[0], data+1);
		if(copy_to_user((unsigned char*)arg,data,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;


    case BMA250_SET_INT_SRC:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_int_src(*data);
		return err;

    case BMA250_GET_INT_SRC:
		if(copy_from_user(data,(unsigned char*)arg,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_get_int_src(data[0], data+1);
		if(copy_to_user((unsigned char*)arg,data,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_GET_INT_SET:
		if(copy_from_user(data,(unsigned char*)arg,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_get_int_set(data[0], data+1);
		if(copy_to_user((unsigned char*)arg,data,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

     case BMA250_SET_INT_SET:
		if(copy_from_user(data,(unsigned char*)arg,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_int_set(data[0], data[1]);
		return err;

    case BMA250_GET_MODE_CTRL:
		err = bma250_get_mode_ctrl(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_LOW_DURATION:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_low_duration(*data);
		return err;

    case BMA250_GET_LOW_DURATION:
		err = bma250_get_low_duration(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_LOW_G_THRESHOLD:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_low_g_threshold(*data);
		return err;

    case BMA250_GET_LOW_G_THRESHOLD:
		err = bma250_get_low_g_threshold(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_HIGH_G_DURATION:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_high_g_duration(*data);
		return err;

    case BMA250_GET_HIGH_G_DURATION:
		err = bma250_get_high_g_duration(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_HIGH_G_THRESHOLD:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_high_g_threshold(*data);
		return err;

    case BMA250_GET_HIGH_G_THRESHOLD:
		err = bma250_get_high_g_threshold(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_SLOPE_DURATION:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_slope_duration(*data);
		return err;

    case BMA250_GET_SLOPE_DURATION:
		err = bma250_get_slope_duration(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_SLOPE_THRESHOLD:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_slope_threshold(*data);
		return err;

    case BMA250_GET_SLOPE_THRESHOLD:
		err = bma250_get_slope_threshold(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_TAP_DURATION:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_tap_duration(*data);
		return err;

    case BMA250_GET_TAP_DURATION:
		err = bma250_get_tap_duration(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_TAP_SHOCK:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_tap_shock(*data);
		return err;

    case BMA250_GET_TAP_SHOCK:
		err = bma250_get_tap_shock(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_TAP_QUIET_DURATION:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_tap_quiet_duration(*data);
		return err;

    case BMA250_GET_TAP_QUIET:
		err = bma250_get_tap_quiet(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_TAP_THRESHOLD:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_tap_threshold(*data);
		return err;

    case BMA250_GET_TAP_THRESHOLD:
		err = bma250_get_tap_threshold(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_TAP_SAMP:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_tap_samp(*data);
		return err;

    case BMA250_GET_TAP_SAMP:
		err = bma250_get_tap_samp(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_ORIENT_MODE:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_orient_mode(*data);
		return err;

    case BMA250_GET_ORIENT_MODE:
		err = bma250_get_orient_mode(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_ORIENT_BLOCKING:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_orient_blocking(*data);
		return err;

    case BMA250_GET_ORIENT_BLOCKING:
		err = bma250_get_orient_blocking(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;
    case BMA250_SET_ORIENT_HYST:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_orient_hyst(*data);
		return err;

    case BMA250_GET_ORIENT_HYST:
		err = bma250_get_orient_hyst(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;
    case BMA250_SET_THETA_BLOCKING:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_theta_blocking(*data);
		return err;

    case BMA250_GET_THETA_BLOCKING:
		err = bma250_get_theta_blocking(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_ORIENT_EX:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_orient_ex(*data);
		return err;

    case BMA250_GET_ORIENT_EX:
		err = bma250_get_orient_ex(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_THETA_FLAT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_theta_flat(*data);
		return err;

    case BMA250_GET_THETA_FLAT:
		err = bma250_get_theta_flat(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;
    case BMA250_SET_FLAT_HOLD_TIME:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_flat_hold_time(*data);
		return err;

    case BMA250_GET_FLAT_HOLD_TIME:
		err = bma250_get_flat_hold_time(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;
    
   case BMA250_GET_LOW_POWER_STATE:
		err = bma250_get_low_power_state(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;


    case BMA250_SET_SELFTEST_ST:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_selftest_st(*data);
		return err;

    case BMA250_GET_SELFTEST_ST:
		err = bma250_get_selftest_st(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;
    

    case BMA250_SET_SELFTEST_STN:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_selftest_stn(*data);
		return err;

    case BMA250_GET_SELFTEST_STN:
		err = bma250_get_selftest_stn(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;
    
    case BMA250_SET_SELFTEST_ST_AMP:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_selftest_st_amp(*data);
		return err;

    case BMA250_GET_SELFTEST_ST_AMP:
		err = bma250_get_selftest_st_amp(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;
  
    case BMA250_SET_EE_W:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_ee_w(*data);
		return err;

    case BMA250_GET_EE_W:
		err = bma250_get_ee_w(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;
    

    case BMA250_SET_EE_PROG_TRIG:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_ee_prog_trig(*data);
		return err;
   
    case BMA250_GET_EEPROM_WRITING_STATUS:
		err = bma250_get_eeprom_writing_status(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;
    
    case BMA250_SET_UPDATE_IMAGE:
		err = bma250_set_update_image();
		return err;

    case BMA250_SET_I2C_WDT_TIMER:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_i2c_wdt_timer(*data);
		return err;

    case BMA250_GET_I2C_WDT_TIMER:
		err = bma250_get_i2c_wdt_timer(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

  /*  case BMA250_SET_UNLOCK_TRIMMING_PART:
		err = bma250_set_unlock_trimming_part();
		return err;
*/
    case BMA250_GET_HP_EN:
		if(copy_from_user(data,(unsigned char*)arg,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_get_hp_en(data[0], data+1);
		if(copy_to_user((unsigned char*)arg,data,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

     case BMA250_SET_HP_EN:
		if(copy_from_user(data,(unsigned char*)arg,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_hp_en(data[0], data[1]);
		return err;

    case BMA250_SET_CAL_TRIGGER:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_cal_trigger(*data);
		return err;

    case BMA250_GET_CAL_READY:
		err = bma250_get_cal_ready(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_OFFSET_RESET:
		err = bma250_set_offset_reset();
		return err;

    case BMA250_SET_OFFSET_CUTOFF:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_offset_cutoff(*data);
		return err;

    case BMA250_GET_OFFSET_CUTOFF:
		err = bma250_get_offset_cutoff(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_OFFSET_TARGET_X:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_offset_target_x(*data);
		return err;

    case BMA250_GET_OFFSET_TARGET_X:
		err = bma250_get_offset_target_x(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_OFFSET_TARGET_Y:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_offset_target_y(*data);
		return err;

    case BMA250_GET_OFFSET_TARGET_Y:
		err = bma250_get_offset_target_y(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_OFFSET_TARGET_Z:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_offset_target_z(*data);
		return err;

     case BMA250_GET_OFFSET_TARGET_Z:
		err = bma250_get_offset_target_z(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_OFFSET_FILT_X:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_offset_filt_x(*data);
		return err;

     case BMA250_GET_OFFSET_FILT_X:
		err = bma250_get_offset_filt_x(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

     case BMA250_SET_OFFSET_FILT_Y:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_offset_filt_y(*data);
		return err;

     case BMA250_GET_OFFSET_FILT_Y:
		err = bma250_get_offset_filt_y(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA250_SET_OFFSET_FILT_Z:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_offset_filt_z(*data);
		return err;

   case BMA250_GET_OFFSET_FILT_Z:
		err = bma250_get_offset_filt_z(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

   case BMA250_SET_OFFSET_UNFILT_X:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_offset_unfilt_x(*data);
		return err;

   case BMA250_GET_OFFSET_UNFILT_X:
		err = bma250_get_offset_unfilt_x(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

   case BMA250_SET_OFFSET_UNFILT_Y:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_offset_unfilt_y(*data);
		return err;

   case BMA250_GET_OFFSET_UNFILT_Y:
		err = bma250_get_offset_unfilt_y(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

   case BMA250_SET_OFFSET_UNFILT_Z:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_offset_unfilt_z(*data);
		return err;

   case BMA250_GET_OFFSET_UNFILT_Z:
		err = bma250_get_offset_unfilt_z(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

   case BMA250_SET_INT_MODE:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_Int_Mode(*data);
		return err;

    case BMA250_GET_INT_MODE:
		err = bma250_get_Int_Mode(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

   case BMA250_SET_INT_ENABLE:
		if(copy_from_user(data,(unsigned char*)arg,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_Int_Enable(data[0], data[1]);
		return err;

  case BMA250_WRITE_EE:
		if(copy_from_user(data,(unsigned char*)arg,2)!=0)
		{
#ifdef BMA250_DEBUG			
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_write_ee(data[0], data[1]);
		return err;

   case BMA250_SET_LOW_HY:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_low_hy(*data);
		return err;

  case BMA250_SET_HIGH_HY:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_high_hy(*data);
		return err;

  case BMA250_SET_LOW_MODE:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma250_set_low_mode(*data);
		return err;

   case BMA250_GET_UPDATE_IMAGE_STATUS:
		err = bma250_get_update_image_status(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA250_SET_ACCELEROMETER_ENABLE:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		bma250_set_accelerometer(data[0]);
		return 0;

	case BMA250_SET_GESTURE_ENABLE:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		bma250_set_gesture(data[0]);
		return 0;

	case BMA250_SET_CALIBRATION_ENABLE:
		//2011-06-16 by baborobo  the Calibration need the enable of Accelrometer
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA250_DEBUG
			printk(KERN_INFO "copy_from_user error\n");
#endif
			return -EFAULT;
		}
		bma250_set_calibration(data[0]);
		return 0;

	default:
		return 0;
	}
}

static int bma250_fasync(int fd, struct file *file, int mode)
{
    struct bma250_data* data;
#ifdef BMA250_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);	
#endif
 	data=i2c_get_clientdata(bma250_client); 
	return fasync_helper(fd,file,mode,&data->async_queue);
	return 0;
}


/* LGE_UPDATE_S 20111216 kideok.kim@lge.com kernel 3.0.x Adaptation */
static const struct file_operations bma250_fops = {
	.owner = THIS_MODULE,
	.read = bma250_read,
	.write = bma250_write,
    .poll = bma250_poll,
	.open = bma250_open,
	.release = bma250_close,
/*	.ioctl = bma250_ioctl,	*/
	.unlocked_ioctl = bma250_ioctl,
	.fasync = bma250_fasync,
};
/* LGE_UPDATE_E 20111216 kideok.kim@lge.com kernel 3.0.x Adaptation */


static struct miscdevice bma_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "bma250",
	.fops = &bma250_fops,
};

static int bma250_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
#ifdef BMA250_DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	strlcpy(info->type, "bma250", I2C_NAME_SIZE);

	return 0;
}

#ifdef	BMA250_ENABLE_IRQ
static int bma250_input_init(struct bma250_data *data)
{
    int err=0;

#ifdef BMA250_DEBUG
    printk(KERN_INFO "bma250 : bma250_input_init.....\n");
#endif
    data->input_dev = input_allocate_device();

    if (!data->input_dev) {
        err = -ENOMEM;
        printk(KERN_INFO "bma250 : Failed to allocate input device\n");
        goto exit_input_dev_alloc_failed;
    }

    data->input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_SYN);
    data->input_dev->keybit[BIT_WORD(BTN_MOUSE)] |= BIT_MASK(BTN_LEFT) | BIT_MASK(BTN_RIGHT)
                                           | BIT_MASK(BTN_FORWARD) | BIT_MASK(BTN_BACK)
                                           | BIT_MASK(BTN_SIDE) | BIT_MASK(BTN_EXTRA)
                                           | BIT_MASK(BTN_TASK);
    
    data->input_dev->name = "Gesture";

    err = input_register_device(data->input_dev);
    if (err) {
        printk(KERN_INFO "bma250 : Unable to register input device: %s\n",
               data->input_dev->name);
        goto exit_input_register_device_failed;
    }

    input_report_key(data->input_dev, BTN_TASK, 2);
    input_sync(data->input_dev);

    return 0;

exit_input_register_device_failed:
    input_free_device(data->input_dev);	
exit_input_dev_alloc_failed:
    return err;	
}
#endif /* BMA250_ENABLE_IRQ */

#ifdef BMA250_DEBUG
static int debug_reg = 0;
static ssize_t
bma250_debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char val;

	p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, (unsigned char)debug_reg, &val, C_One_U8X);

	return snprintf(buf, PAGE_SIZE, "debug_reg(0x%x) = 0x%x\n", debug_reg, val);
}

static ssize_t
bma250_debug_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int reg_val;
	int ret;
	unsigned char val;

	ret = sscanf(buf, "0x%x 0x%x", &debug_reg, &reg_val);

	if(ret == 2) {
		printk(KERN_INFO "bma250 : reg = 0x%x / 0x%x \n", debug_reg, reg_val);
		val = (unsigned char)reg_val;
		p_bma250->bma250_BUS_WRITE_FUNC(p_bma250->dev_addr, (unsigned char)debug_reg, &val, C_One_U8X);
	} else
	{
		printk(KERN_INFO "bma250 : Input parameter errot \n");
	}	

	return count;
}

static ssize_t
bma250_status_interrupt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char val;

	p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS1_REG, &val, C_One_U8X);

	return snprintf(buf, PAGE_SIZE, "status_interrupt(0x%x) = 0x%x\n", bma250_STATUS1_REG, val);
}
static ssize_t
bma250_status_flat_orient_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char val;

	p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS_ORIENT_HIGH_REG, &val, C_One_U8X);

	return snprintf(buf, PAGE_SIZE, "status_interrupt(0x%x) = 0x%x\n", bma250_STATUS_ORIENT_HIGH_REG, val);
}
static ssize_t
bma250_status_tap_slop_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char val;

	p_bma250->bma250_BUS_READ_FUNC(p_bma250->dev_addr, bma250_STATUS_TAP_SLOPE_REG, &val, C_One_U8X);

	return snprintf(buf, PAGE_SIZE, "status_interrupt(0x%x) = 0x%x\n", bma250_STATUS_ORIENT_HIGH_REG, val);
}
#endif /* BMA250_DEBUG */

/* taegyun.an 2011-08-20*/
#ifdef BMA250_ATCMD
static ssize_t 
show_enable_value(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	char strbuf[256];
	struct i2c_client *client = i2c_verify_client(dev);
	struct bma250_data *bma = i2c_get_clientdata(client);
	
	sprintf(strbuf, "%d", bma->sensor_enable);
	return sprintf(buf, "%s\n", strbuf);
}

static ssize_t 
store_enable_value(struct device *dev, 
		struct device_attribute *attr, const char *buf, size_t count)
{
	int mode=0;
	struct i2c_client *client = i2c_verify_client(dev);
	struct bma250_data *bma = i2c_get_clientdata(client);

	printk(KERN_INFO "store_enable_value -> mode : %d\n", mode);
	sscanf(buf, "%d", &mode);
	if (mode) {
			bma->sensor_enable = 0;
			bma250_set_accelerometer(1);
			bma250_pdata->power(1);
			mdelay(5);
			printk(KERN_INFO "Power On Enable\n");
	}
	else {
			bma->sensor_enable = 1;
			bma250_set_accelerometer(0);
			bma250_pdata->power(0);
			mdelay(5);
			printk(KERN_INFO "Power Off Disable\n");
	}
	return 0;
}

static ssize_t 
show_sensordata_value(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	char strbuf[30] ={0x00,};
	bma250acc_t bma_acc;

	bma250_read_accel_xyz(&bma_acc);
	sprintf(strbuf, "%d %d %d", bma_acc.x, bma_acc.y, bma_acc.z);
	return sprintf(buf, "%s\n",strbuf);
}
/* taegyun.an 2011-08-19*/
#endif

//ex) # echo 0x11 0x12 > sys/devices/platform/i2c-gpio.11/i2c-11/11-0018/debug
static struct device_attribute bma250_device_attrs[] = {
#ifdef BMA250_DEBUG
		__ATTR(debug, S_IRUGO | S_IWUSR, bma250_debug_show, bma250_debug_store),
		__ATTR(int_stutus, S_IRUGO, bma250_status_interrupt_show, NULL),
		__ATTR(flat_orient, S_IRUGO, bma250_status_flat_orient_show, NULL),
		__ATTR(tap_slop, S_IRUGO, bma250_status_tap_slop_show, NULL),
#endif 
#ifdef BMA250_ATCMD
		/* atcmd attribute*/
		__ATTR(enable, S_IRUGO | S_IWUSR, show_enable_value, store_enable_value),
		__ATTR(sensordata, S_IRUGO, show_sensordata_value, NULL)
#endif

};

static int bma250_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int err = 0;
	int tempvalue;
	struct bma250_data *data;
	struct acceleration_platform_data *pdata;

#ifdef BMA250_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(KERN_INFO "i2c_check_functionality error\n");
		goto exit;
	}
	data = kzalloc(sizeof(struct bma250_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	pdata = client->dev.platform_data;
	bma250_pdata = pdata;
	pdata->power(1);
	mdelay(5);

	/* read chip id */
	tempvalue = 0;
#ifdef BMA250_SMBUS
	tempvalue = i2c_smbus_read_word_data(client, 0x00);
#else
	i2c_master_send(client, (char*)&tempvalue, 1);
	i2c_master_recv(client, (char*)&tempvalue, 1);
#endif

	if((tempvalue&0x00FF) == 0x0003)
	{
		printk(KERN_INFO "Bosch Sensortec Device detected!\n BMA250 registered I2C driver!\n");
		bma250_client = client;
	}
	else
	{
		printk(KERN_INFO "Bosch Sensortec Device not found, i2c error %d \n", tempvalue);
				
		bma250_client = NULL;
		err = -1;
		goto kfree_exit;
	}
	i2c_set_clientdata(bma250_client, data);
	
	err = misc_register(&bma_device);
	if (err) {
		printk(KERN_ERR "bma250 device register failed\n");
		goto kfree_exit;
	}
	printk(KERN_INFO "bma250 device create ok\n");

	/* bma250 sensor initial */
	data->bma250.bus_write = bma250_i2c_write;
	data->bma250.bus_read = bma250_i2c_read;
	data->bma250.delay_msec = bma250_i2c_delay;

	bma250_init(&data->bma250);

	bma250_init_config();

#ifdef	BMA250_ENABLE_IRQ
    bma250_set_mode(bma250_MODE_NORMAL);

    /* register interrupt */
    bma250_interrupt_config(0);

    spin_lock_init(&data->lock);

    err = bma250_input_init(data);
    if(err)
        goto exit_dereg;

    acceler_wq = create_singlethread_workqueue("acceler_wq");
    if (!acceler_wq) {
  #ifdef BMA250_DEBUG
        printk(KERN_INFO "bma250 : failed to create singnalthread workqueue \n");
  #endif
        goto exit_dereg;
    }

    INIT_DELAYED_WORK(&data->dwork, bma250_work_func);

    data->irq = gpio_to_irq(pdata->irq_num);
    err = request_irq(data->irq, bma250_irq_handler, IRQF_TRIGGER_RISING, "bma250", &data->bma250);
    if (err) {
        printk(KERN_ERR "could not request irq\n");
        destroy_workqueue(acceler_wq);
        goto exit_dereg;
    }
//	enable_irq(data->irq);

	/* create sysfs attribute files */
	data->sensor_enable = 0;

	disable_irq(data->irq);
	bma250_set_mode(bma250_MODE_SUSPEND);
#endif /* BMA250_ENABLE_IRQ */

#ifdef BMA250_HAS_EARLYSUSPEND
    data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    data->early_suspend.suspend = bma250_early_suspend;
    data->early_suspend.resume = bma250_late_resume;
    register_early_suspend(&data->early_suspend);
#endif

#if defined(BMA250_DEBUG) || defined(BMA250_ATCMD)
	/* create sysfs attribute files */
	for (tempvalue = 0; tempvalue < ARRAY_SIZE(bma250_device_attrs); tempvalue++) {
		err = device_create_file(&client->dev, &bma250_device_attrs[tempvalue]);
		if (err) {
			printk(KERN_ERR "could not make debug file[%d]\n", tempvalue);
		}
	}
#endif

	return 0;

#ifdef BMA250_ENABLE_IRQ
exit_dereg:
    misc_deregister(&bma_device);
#endif
kfree_exit:
	kfree(data);
exit:
	return err;
}

#ifdef BMA250_HAS_EARLYSUSPEND
static void bma250_early_suspend(struct early_suspend *h)
{
    struct bma250_data* data;

    data = i2c_get_clientdata(bma250_client); 

#ifdef BMA250_DEBUG
    printk(KERN_INFO "%s\n",__FUNCTION__);
#endif

  #ifdef	BMA250_ENABLE_IRQ
      del_timer( &(data->flip_check_timer) );
      if(data->sensor_enable & BMA250_GESTURE) {
          disable_irq(data->irq);
          cancel_delayed_work_sync(&data->dwork);
          flush_workqueue(acceler_wq);
      }
  #endif

    bma250_set_mode(bma250_MODE_SUSPEND);

    bma250_pdata->power(0);
}


static void bma250_late_resume(struct early_suspend *h)
{
    struct bma250_data* data;

    data = i2c_get_clientdata(bma250_client); 

#ifdef BMA250_DEBUG
    printk(KERN_INFO "%s\n",__FUNCTION__);
#endif

    bma250_pdata->power(1);
    mdelay(5);

  #ifdef BMA250_ENABLE_IRQ
    bma250_init_config();

    if(data->sensor_enable) {
        bma250_set_mode(bma250_MODE_NORMAL);

        if(data->sensor_enable & BMA250_GESTURE) {
            bma250_interrupt_config(1);
            enable_irq(data->irq);
        }
    }
  #endif
}
#endif
static int bma250_suspend(struct i2c_client *client,pm_message_t mesg)
{
#ifndef BMA250_HAS_EARLYSUSPEND
    struct bma250_data* data;

    data = i2c_get_clientdata(client); 

  #ifdef	BMA250_ENABLE_IRQ
    del_timer( &(data->flip_check_timer) );
    if(data->sensor_enable & BMA250_GESTURE) {
        disable_irq(data->irq);
        cancel_delayed_work_sync(&data->dwork);
        flush_workqueue(acceler_wq);
    }
  #endif

    bma250_set_mode(bma250_MODE_SUSPEND);

    bma250_pdata->power(0);
#endif

    printk(KERN_INFO "%s\n",__FUNCTION__);
    return 0;	
}
static int bma250_resume(struct i2c_client *client)
{
#ifndef BMA250_HAS_EARLYSUSPEND
    struct bma250_data* data;

    data = i2c_get_clientdata(client); 

    printk(KERN_INFO "%s\n",__FUNCTION__);

    bma250_pdata->power(1);
    mdelay(5);

  #ifdef BMA250_ENABLE_IRQ
    bma250_init_config();

    if(data->sensor_enable) {
        bma250_set_mode(bma250_MODE_NORMAL);

        if(data->sensor_enable & BMA250_GESTURE) {
            bma250_interrupt_config(1);
            enable_irq(data->irq);
        }
    }
  #endif
#endif	

    printk(KERN_INFO "%s\n",__FUNCTION__);

    return 0;	
}

static int bma250_remove(struct i2c_client *client)
{
	struct bma250_data *data = i2c_get_clientdata(client);

#ifdef BMA250_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif	

#ifdef	BMA250_ENABLE_IRQ
  del_timer( &(data->flip_check_timer) );
	if (acceler_wq)
		destroy_workqueue(acceler_wq);

	input_unregister_device(data->input_dev);
#endif

#ifdef BMA250_HAS_EARLYSUSPEND
    unregister_early_suspend(&data->early_suspend);
#endif
	misc_deregister(&bma_device);
#ifdef BMA250_ENABLE_IRQ
	free_irq(data->irq, &data->bma250);
#endif
	
	kfree(data);
	bma250_client = NULL;
	return 0;
}

#if 1 /* not used */
static unsigned short normal_i2c[] = { I2C_CLIENT_END};
#endif

#ifdef BMA250_MODULES 
I2C_CLIENT_INSMOD_1(bma250);
#endif

static const struct i2c_device_id bma250_id[] = {
	{ "bma250", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bma250_id);

static struct i2c_driver bma250_driver = {	
	.driver = {
		.owner	= THIS_MODULE,	
		.name	= "bma250",
	},
	.class		= I2C_CLASS_HWMON,
	.id_table	= bma250_id,
#if 0 /* FIXME: */
	.address_data	= &addr_data,
#endif
	.address_list   = normal_i2c,
	.probe		= bma250_probe,
	.remove		= bma250_remove,
	.detect		= bma250_detect,
    .suspend    = bma250_suspend,
    .resume     = bma250_resume,
};

static int __init BMA250_init(void)
{
#ifdef BMA250_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif
	return i2c_add_driver(&bma250_driver);
}

static void __exit BMA250_exit(void)
{
	i2c_del_driver(&bma250_driver);

	printk(KERN_ERR "BMA250 exit\n");
}

MODULE_DESCRIPTION("BMA250 driver");

module_init(BMA250_init);
module_exit(BMA250_exit);

