/*
 * Samsung Mobile VE Group.
 *
 * drivers/gpio/gpio_dvs/rhea_gpio_dvs.c - Read GPIO info. from BCM2165x(RHEA)
 * 
 * Copyright (C) 2013, Samsung Electronics.
 *
 * This program is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation
 */
 
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/syscalls.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/cpu.h>
#include <linux/cpuset.h>
#include <linux/cgroup.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include <mach/gpio.h>
#include <mach/regs_glb.h>
#include <mach/hardware.h>
#include <linux/secgpio_dvs.h>
#include <linux/io.h>
#include <mach/gpio.h>

/****************************************************************/
/* Define value in accordance with
	the specification of each BB vendor. */
#define AP_GPIO_COUNT 199
/****************************************************************/

#define GPIODATA_offset 0x0
#define GPIODIR_offset 0x8

#define SPRD_GPIO_COUNT	199

#define GetBit(dwData, i)	(dwData & (0x00000001 << i))
#define SetBit(dwData, i)	(dwData | (0x00000001 << i))
#define ClearBit(dwData, i)	(dwData & ~(0x00000001 << i))

#define GET_RESULT_GPIO(a, b, c)	\
	((a<<4 & 0xF0) | (b<<1 & 0xE) | (c & 0x1))

/****************************************************************/
/* Pre-defined variables. (DO NOT CHANGE THIS!!) */
static unsigned char checkgpiomap_result[GDVS_PHONE_STATUS_MAX][AP_GPIO_COUNT];
static struct gpiomap_result_t gpiomap_result = {
	.init = checkgpiomap_result[PHONE_INIT],
	.sleep = checkgpiomap_result[PHONE_SLEEP]
};

static int  total_gpio_index= 0;


#ifdef SECGPIO_SLEEP_DEBUGGING
static struct sleepdebug_gpiotable sleepdebug_table;
#endif
/****************************************************************/

void Set_GPIO_Group(unsigned char arg_curr_pin, unsigned int * ARM_offset, unsigned int * ret_bit_position )
{
	unsigned char  pin_num=0;
	pin_num = arg_curr_pin;


	if (pin_num<=31)
	{
		*ARM_offset=0x00;
		*ret_bit_position=pin_num-16;
	}	
	else if (pin_num<=47)
	{
		*ARM_offset=0x80;
		*ret_bit_position=pin_num-32;

	}	
	else if (pin_num<=63)
	{
		*ARM_offset=0x100;
		*ret_bit_position=pin_num-48;
	}	
	else if (pin_num<=79)
	{
		*ARM_offset=0x180;
		*ret_bit_position=pin_num-64;
	}	
	else if (pin_num<=95)
	{
		*ARM_offset=0x200;
		*ret_bit_position=pin_num-80;
	}	
	else if (pin_num<=111)
	{
		*ARM_offset=0x280;
		*ret_bit_position=pin_num-96;
	}	
	else if (pin_num<=127)
	{
		*ARM_offset=0x300;
		*ret_bit_position=pin_num-112;
	}	
	else if (pin_num<=143)
	{
		*ARM_offset=0x380;
		*ret_bit_position=pin_num-128;
	}	
	else if (pin_num<=159)
	{
		*ARM_offset=0x400;
		*ret_bit_position=pin_num-144;
	}	
	else if (pin_num<=175)
	{
		*ARM_offset=0x480;
		*ret_bit_position=pin_num-160;
	
	}
	else if (pin_num<=191)
	{
		*ARM_offset=0x500;
		*ret_bit_position=pin_num-176;
	}
	else if (pin_num<=207)
	{
		*ARM_offset=0x580;
		*ret_bit_position=pin_num-192;
	}
	else if (pin_num<=223)
	{
		*ARM_offset=0x600;
		*ret_bit_position=pin_num-208;
	}

}

unsigned char Get_string_gpio_altFunc(unsigned short PIN_NAME_sel ,unsigned char func_mask)
{
 	unsigned long tmp_func=0, ret_func=0;;
 	
 	tmp_func=0x1<<PIN_NAME_sel;
 	tmp_func=(tmp_func)&(func_mask);
 	if (tmp_func !=0)
 	 	ret_func= 0 ;//"Func"
 	else
 		ret_func=1;//"RSV"
  
return ret_func ;
}



Get_Sprd_Register(unsigned char phonestate, unsigned char arg_curr_pin , unsigned int offset ,unsigned short  func_mask ,unsigned char gpio_value)
{
	unsigned int  GPIO_Control_Reg=0, rControl_Reg=0, GPIODATA_Reg=0, GPIODIR_Reg=0 , rGPIODIR=0, rGPIODATA=0;
	unsigned short PIN_NAME_sel=0, PIN_NAME_func_wpud=0, bDir=0, bData=0,   PIN_NAME_ie=0, PIN_NAME_oe=0;
	unsigned int  ARM_offset=0, bit_pos=0;
	//make control register address

	GPIO_Control_Reg=SPRD_PIN_BASE+offset;

 	// read control register	
 	rControl_Reg=readl(GPIO_Control_Reg);

	
 	// get PIN_NAME_sel bits

	PIN_NAME_sel=( (GetBit(rControl_Reg,5)  | GetBit(rControl_Reg,4)) >> 4);  // check mutiplex(mode) setting
	
/* 3 bit (000:func, 001:input, 010:output, 011:int, 100 Prev )  */
/* 3 bit (000:NP, 001:PD, 010:PU, 111:error) */	
/* 1 bit (0:LOW, 1:HIGH) */
		
	if (phonestate==PHONE_SLEEP)
	{
		PIN_NAME_func_wpud= ((GetBit(rControl_Reg,3)  | GetBit(rControl_Reg,2)) >>2);  //,2.,3.bit read   pupd in deep sleep mode

		PIN_NAME_ie= (GetBit(rControl_Reg,1)>>1 );
		PIN_NAME_oe = GetBit(rControl_Reg,0);
	}
	else
	{
		PIN_NAME_func_wpud= ((GetBit(rControl_Reg,7)  | GetBit(rControl_Reg,6)) >> 6);   //,6.,7.bit read pupd in normal mode
	}
		
		
	if (PIN_NAME_sel==gpio_value)  // gpio value == multiplex value at GPIO setting
	{
		// Get PIN Name base address & bit position
		Set_GPIO_Group(arg_curr_pin,&ARM_offset,&bit_pos);


		// GPIO address for each GPIO Group
		GPIODATA_Reg=SPRD_GPIO_BASE+ARM_offset+GPIODATA_offset;
		GPIODIR_Reg=SPRD_GPIO_BASE+ARM_offset+GPIODIR_offset;


		// Direction IN? OUT? 

		if( phonestate ==PHONE_SLEEP)
		{
			if((PIN_NAME_ie | PIN_NAME_oe) ==0)
				bDir = 0; //func
			else if (PIN_NAME_ie == 1)
				bDir = 1;  //in
			else if(PIN_NAME_oe ==1)
				bDir =2;  //out
		}
		else
		{
			rGPIODIR=readl(GPIODIR_Reg);
			bDir=(GetBit(rGPIODIR,bit_pos) >> bit_pos );   // direction 0 == in, 1==out 
			bDir = bDir +1;
		}


	       // Data H? L?
		rGPIODATA=readl(GPIODATA_Reg);
		bData=(GetBit(rGPIODATA,bit_pos) >>bit_pos) ;  // 0 == L,  1==H
		//printk("[GpioDVS] %d  bDir %x  PIN_NAME_func_wpud  %x  bData %x  final result %x \n",arg_curr_pin , bDir, PIN_NAME_func_wpud, bData, GET_RESULT_GPIO(bDir,PIN_NAME_func_wpud,bData));


	}
	else  // func mode
	{
		bDir = Get_string_gpio_altFunc(PIN_NAME_sel ,func_mask);  //0 ==func, 1==RSV
		if (bDir == 0)
		   bDir = 0; //RSV
		else if(bDir == 1)
			bDir = 0; //func  // all  func mode.  no rsv mode
 
		//printk("[GpioDVS] %d  altFunc %x  PIN_NAME_func_wpud=%x  final result %x  \n",arg_curr_pin ,bDir, PIN_NAME_func_wpud, GET_RESULT_GPIO(bDir,PIN_NAME_func_wpud,bData));

	
	}
	

	checkgpiomap_result[phonestate][total_gpio_index] = GET_RESULT_GPIO(bDir,PIN_NAME_func_wpud,bData); // a= in/out/func , b=pdpu, c=lh)
       total_gpio_index++;

}


/****************************************************************/
/* Define this function in accordance with the specification of each BB vendor */
static void check_gpio_status(unsigned char phonestate)
{
	
		printk("[GpioDVS][%s] ++\n", __func__);
	total_gpio_index = 0;
	Get_Sprd_Register(phonestate,16,  0x018, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,17,  0x01c, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,18,  0x020, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,19,  0x024, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,20,  0x028, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,21,  0x02c, 0x09, 0x3  );

	Get_Sprd_Register(phonestate,211,  0x030, 0x09, 0x3  );//

	Get_Sprd_Register(phonestate,22,  0x034, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,23,  0x038, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,24,  0x03c, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,25,  0x040, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,26,  0x044, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,27,  0x0d0, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,28,  0x0d8, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,29,  0x0dc, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,30,  0x0d4, 0x09, 0x3  );

	Get_Sprd_Register(phonestate,31,  0x0e0, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,32,  0x0e4, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,33,  0x0e8, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,34,  0x0ec, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,35,  0x0f0, 0x0b, 0x3  );
	Get_Sprd_Register(phonestate,36,  0x0f4, 0x0b, 0x3  );
	Get_Sprd_Register(phonestate,37,  0x0f8, 0x0b, 0x3  );
	Get_Sprd_Register(phonestate,38,  0x0fc, 0x0b, 0x3  );
	Get_Sprd_Register(phonestate,39,  0x100, 0x0b, 0x3  );
	Get_Sprd_Register(phonestate,40,  0x104, 0x0b, 0x3  );
	Get_Sprd_Register(phonestate,41,  0x108, 0x0b, 0x3  );
	Get_Sprd_Register(phonestate,42,  0x10c, 0x0b, 0x3  );
	Get_Sprd_Register(phonestate,43,  0x110, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,44,  0x114, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,45,  0x118, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,46,  0x11c, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,47,  0x120, 0x0d, 0x3  );
	Get_Sprd_Register(phonestate,48,  0x124, 0x0d, 0x3  );
	Get_Sprd_Register(phonestate,49,  0x128, 0x0d, 0x3  );
	Get_Sprd_Register(phonestate,50,  0x12c, 0x0d, 0x3  );

	Get_Sprd_Register(phonestate,51,  0x130, 0x0d, 0x3  );

	Get_Sprd_Register(phonestate,135,  0x134, 0x05, 0x0  );
	Get_Sprd_Register(phonestate,136,  0x138, 0x07, 0x0  );
	Get_Sprd_Register(phonestate,137,  0x13c, 0x07, 0x0  );
	Get_Sprd_Register(phonestate,138,  0x140, 0x05, 0x0  );
	Get_Sprd_Register(phonestate,139,  0x144, 0x05, 0x0 );
	Get_Sprd_Register(phonestate,140,  0x148, 0x05, 0x0  );
	
	Get_Sprd_Register(phonestate,52,  0x14c, 0x0d, 0x3  );
	Get_Sprd_Register(phonestate,53,  0x150, 0x0f, 0x3  );
	Get_Sprd_Register(phonestate,54,  0x154, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,55,  0x158, 0x0d, 0x3  );
	Get_Sprd_Register(phonestate,56,  0x15c, 0x0d, 0x3  );
	Get_Sprd_Register(phonestate,57,  0x160, 0x0d, 0x3  );
	Get_Sprd_Register(phonestate,58,  0x164, 0x0d, 0x3  );
	Get_Sprd_Register(phonestate,59,  0x168, 0x0d, 0x3  );

	Get_Sprd_Register(phonestate,212,  0x16c, 0x0D, 0x3  );
	Get_Sprd_Register(phonestate,213,  0x170, 0x0F, 0x3  );

	
	Get_Sprd_Register(phonestate,60,  0x174, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,61,  0x178, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,62,  0x17c, 0x0d, 0x3  );
	Get_Sprd_Register(phonestate,63,  0x180, 0x0d, 0x3  );
	Get_Sprd_Register(phonestate,64,  0x184, 0x0d, 0x3  );
	Get_Sprd_Register(phonestate,65,  0x188, 0x0d, 0x3  );
	Get_Sprd_Register(phonestate,66,  0x18c, 0x0d, 0x3  );
	Get_Sprd_Register(phonestate,67,  0x190, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,68,  0x194, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,69,  0x198, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,70,  0x19c, 0x09, 0x3  );

	Get_Sprd_Register(phonestate,71,  0x1a0, 0x0b, 0x3  );
	Get_Sprd_Register(phonestate,72,  0x1a4, 0x0b, 0x3  );
	Get_Sprd_Register(phonestate,73,  0x1a8, 0x0b, 0x3  );
	Get_Sprd_Register(phonestate,74,  0x1ac, 0x0b, 0x3  );
	Get_Sprd_Register(phonestate,75,  0x1c0, 0x0b, 0x3  );
	Get_Sprd_Register(phonestate,76,  0x1c4, 0x0b, 0x3  );
	Get_Sprd_Register(phonestate,77,  0x1c8, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,78,  0x1bc, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,79,  0x1c0, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,80,  0x1c4, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,81,  0x1c8, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,82,  0x1cd, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,83,  0x1d0, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,84,  0x1d4, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,85,  0x1d8, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,86,  0x1dc, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,87,  0x1e0, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,88,  0x1e4, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,89,  0x1e8, 0x0B, 0x3  );
  Get_Sprd_Register(phonestate,90,  0x1ec, 0x0B, 0x3  );

	Get_Sprd_Register(phonestate,91,  0x1f0, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,92,  0x1f4, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,93,  0x1f8, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,94,  0x1fc, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,95,  0x200, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,96,  0x204, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,97,  0x208, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,98,  0x20c, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,99,  0x210, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,100,  0x214, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,101,  0x218, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,102,  0x21c, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,103,  0x220, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,104,  0x224, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,105,  0x228, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,106,  0x22c, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,107,  0x230, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,108,  0x234, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,109,  0x238, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,110,  0x23c, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,111,  0x240, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,112,  0x244, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,113,  0x248, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,114,  0x24c, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,115,  0x250, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,116,  0x254, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,117,  0x258, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,118,  0x25c, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,119,  0x260, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,120,  0x264, 0x0F, 0x3  );

	Get_Sprd_Register(phonestate,121,  0x268, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,122,  0x26c, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,123,  0x270, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,124,  0x274, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,125,  0x278, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,126,  0x27c, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,127,  0x280, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,128,  0x284, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,129,  0x288, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,130,  0x28c, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,131,  0x290, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,132,  0x294, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,133,  0x298, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,134,  0x29c, 0x09, 0x3  );


	Get_Sprd_Register(phonestate,145,  0x2a0, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,146,  0x2a4, 0x09, 0x3  );


	Get_Sprd_Register(phonestate,141,  0x2e0, 0x03, 0x0  );
	Get_Sprd_Register(phonestate,142,  0x2e4, 0x01, 0x0  );

	Get_Sprd_Register(phonestate,147,  0x2a8, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,148,  0x2ac, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,149,  0x2b0, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,150,  0x2b4, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,151,  0x2b8, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,152,  0x2bc, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,153,  0x2c0, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,154,  0x2c4, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,155,  0x2c8, 0x0B, 0x3  );

	Get_Sprd_Register(phonestate,214,  0x2cc, 0x0f, 0x3  );
	
	Get_Sprd_Register(phonestate,156,  0x2d0, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,157,  0x2d4, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,158,  0x2d8, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,159,  0x2dc, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,160,  0x2e8, 0x0F, 0x3  );


	Get_Sprd_Register(phonestate,161,  0x2ec, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,162,  0x2f0, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,163,  0x2f4, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,164,  0x2f8, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,165,  0x2fc, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,166,  0x300, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,167,  0x304, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,168,  0x308, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,169,  0x30c, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,170,  0x310, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,171,  0x314, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,172,  0x318, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,173,  0x31c, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,174,  0x320, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,175,  0x324, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,176,  0x328, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,177,  0x32c, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,178,  0x330, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,179,  0x334, 0x0F, 0x3  );
	Get_Sprd_Register(phonestate,180,  0x338, 0x0F, 0x3  );

	
	Get_Sprd_Register(phonestate,181,  0x33c, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,182,  0x340, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,183,  0x344, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,184,  0x348, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,185,  0x34c, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,186,  0x350, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,187,  0x354, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,188,  0x358, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,189,  0x35c, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,190,  0x360, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,191,  0x364, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,192,  0x368, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,193,  0x36c, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,194,  0x370, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,195,  0x374, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,196,  0x378, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,197,  0x37c, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,198,  0x380, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,199,  0x384, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,200,  0x388, 0x0B, 0x3  );

	Get_Sprd_Register(phonestate,201,  0x38c, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,202,  0x390, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,203,  0x394, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,204,  0x398, 0x0B, 0x3  );
	Get_Sprd_Register(phonestate,205,  0x39c, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,206,  0x3a0, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,207,  0x3a4, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,208,  0x3a8, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,209,  0x3ac, 0x09, 0x3  );
	Get_Sprd_Register(phonestate,210,  0x3b0, 0x09, 0x3  );


	Get_Sprd_Register(phonestate,143,  0x3b8, 0x05, 0x0  );
	Get_Sprd_Register(phonestate,144,  0x3bc, 0x05, 0x0  );
	return;
}
/****************************************************************/

/****************************************************************/
/* Define this function in accordance with the specification of each BB vendor */
void setgpio_for_sleepdebug(int gpionum, unsigned char  io_pdpu_lh)
{

}
/****************************************************************/

/****************************************************************/
/* Define this function in accordance with the specification of each BB vendor */
static void undo_sleepgpio(void)
{
	int i;
	int gpio_num;

	pr_info("[GPIO_DVS][%s] ++\n", __func__);

	for (i = 0; i < sleepdebug_table.gpio_count; i++) {
		gpio_num = sleepdebug_table.gpioinfo[i].gpio_num;
		/* 
		 * << Caution >> 
		 * If it's necessary, 
		 * change the following function to another appropriate one 
		 * or delete it 
		 */
		setgpio_for_sleepdebug(gpio_num, gpiomap_result.sleep[gpio_num]);
	}

	pr_info("[GPIO_DVS][%s] --\n", __func__);
	return;
}
/****************************************************************/

/********************* Fixed Code Area !***************************/
static void set_sleepgpio(void)
{
	int i;
	int gpio_num;
	unsigned char set_data;

	pr_info("[GPIO_DVS][%s] ++, cnt=%d\n",
		__func__, sleepdebug_table.gpio_count);

	for (i = 0; i < sleepdebug_table.gpio_count; i++) {
		pr_info("[GPIO_DVS][%d] gpio_num(%d), io(%d), pupd(%d), lh(%d)\n",
			i, sleepdebug_table.gpioinfo[i].gpio_num,
			sleepdebug_table.gpioinfo[i].io,
			sleepdebug_table.gpioinfo[i].pupd,
			sleepdebug_table.gpioinfo[i].lh);

		set_data = GET_RESULT_GPIO(
			sleepdebug_table.gpioinfo[i].io,
			sleepdebug_table.gpioinfo[i].pupd,
			sleepdebug_table.gpioinfo[i].lh);

		gpio_num = sleepdebug_table.gpioinfo[i].gpio_num;
		setgpio_for_sleepdebug(gpio_num, set_data);
	}

	pr_info("[GPIO_DVS][%s] --\n", __func__);
	return;
}

static struct gpio_dvs_t gpio_dvs = {
	.result = &gpiomap_result,
	.check_gpio_status = check_gpio_status,
	.count = AP_GPIO_COUNT,
#ifdef SECGPIO_SLEEP_DEBUGGING
	.sdebugtable = &sleepdebug_table,
	.set_sleepgpio = set_sleepgpio,
	.undo_sleepgpio = undo_sleepgpio,
#endif
};

static struct platform_device secgpio_dvs_device = {
	.name	= "secgpio_dvs",
	.id		= -1,
	.dev.platform_data = &gpio_dvs,
};

static struct platform_device *secgpio_dvs_devices[] __initdata = {
	&secgpio_dvs_device,
};

static int __init secgpio_dvs_device_init(void)
{
	return platform_add_devices(
		secgpio_dvs_devices, ARRAY_SIZE(secgpio_dvs_devices));
}
arch_initcall(secgpio_dvs_device_init);
/****************************************************************/


