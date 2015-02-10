/*
#include "sci_types.h"
#include "sys_init.h"
#include "os_api.h"
#include "arm_reg.h"
#include "tiger_reg_base.h"
#include "tiger_reg_ahb.h"
#include "tiger_reg_global.h"
*/
#include <linux/io.h>
#include <mach/hardware.h>
#include <mach/globalregs.h>
#include <linux/delay.h>
#include <mach/sci.h>
#include <mach/regs_glb.h>
#include "spi_simple_drv.h"

#define SPI_USED_BASE SPRD_SPI2_BASE


 /**---------------------------------------------------------------------------*
 **                         Globle Variable                                  *
 **---------------------------------------------------------------------------*/

//endian switch mode
typedef enum
{
    ENDIAN_SWITCH_NONE    = 0x00,    //  2'b00: 0xABCD => 0xABCD
    ENDIAN_SWITCH_ALL     =0x01,     //  2'b01: 0xABCD => 0xDCBA
    ENDIAN_SWITCH_MODE0   =0x02 ,    //  2'b01: 0xABCD => 0xBADC
    ENDIAN_SWITCH_MODE1   = 0x03 ,   //  2'b01: 0xABCD => 0xCDAB
    ENDIAN_SWITCH_MAX
} DMA_ENDIANSWITCH_E ;  //should be added to dma_hal_new.h
//data width
typedef enum DMA_DATAWIDTH
{
    DMA_DATAWIDTH_BYTE = 0,
    DMA_DATAWIDTH_HALFWORD,
    DMA_DATAWIDTH_WORD,
    DMA_DATAWIDTH_MAX
}  DMA_DATAWIDTH_E;
//request mode
typedef enum DMA_CHN_REQMODE
{
    DMA_CHN_REQMODE_NORMAL = 0,
    DMA_CHN_REQMODE_TRASACTION,
    DMA_CHN_REQMODE_LIST,
    DMA_CHN_REQMODE_INFINITE,
    DMA_CHN_REQMODE_MAX
} DMA_CHN_REQMODE_E;


typedef enum LCM_DMA_RETURN_E
{
    LCM_ERR_ID    = 0,
    LCM_ERR_BUSWIDTH =1,   //only support:8-bit,16-bit 80_sys_bus
    LCM_ERR_SRCADDR =2,    //not HalfWORD_align
    LCM_ERR_ENDIAN
} LCM_DMA_RETURN_E;       //SHOULD define in lcd.h

 /**---------------------------------------------------------------------------*
 **                         Function Define                                    *
 **---------------------------------------------------------------------------*/
 
void SPI_Enable( uint32_t spi_id, bool is_en)
{
    if(is_en)
    {
		switch(spi_id){
		case SPI0_ID:	
				sci_glb_set(REG_GLB_GEN0, BIT_SPI0_EB);
			break;
		case SPI1_ID:
				sci_glb_set(REG_GLB_GEN0, BIT_SPI1_EB);
			break;
		case SPI2_ID:
				sci_glb_set(REG_GLB_GEN0, BIT_SPI2_EB);
			break;
		default:

			break;
			}
	}else{
		switch(spi_id){
		case SPI0_ID:
			sci_glb_clr(REG_GLB_GEN0, BIT_SPI0_EB);
			break;
		case SPI1_ID:
			sci_glb_clr(REG_GLB_GEN0, BIT_SPI1_EB);
			break;
		case SPI2_ID:
			sci_glb_clr(REG_GLB_GEN0, BIT_SPI2_EB);
			break;
		default:
			break;
			}
    }

}

void SPI_Reset( uint32_t spi_id, uint32_t ms)
{
	uint32_t rst_bit = SWRST_SPI0_RST;
	if(0 == spi_id){
		;
	}else if(1 == spi_id){
		rst_bit = SWRST_SPI1_RST;
	}else if(2 == spi_id){
		rst_bit = SWRST_SPI2_RST;
	}else{
		printk("SPRDFB [%s], %d is SPI  error channel bit! ", __FUNCTION__, spi_id);
		return;
	}
	
	sci_glb_set(REG_GLB_SOFT_RST, rst_bit);
	udelay(100);
	sci_glb_clr(REG_GLB_SOFT_RST, rst_bit);

}

#if 0
static void SPI_PinConfig(void)
{
#ifndef FPGA_TEST
    *(volatile uint32_t *)(0x8C000100) = 0x18A; 
    *(volatile uint32_t *)(0x8C000104) = 0x107;
    *(volatile uint32_t *)(0x8C000108) = 0x109;
    *(volatile uint32_t *)(0x8C00010C) = 0x109;
    *(volatile uint32_t *)(0x8C000110) = 0x109;
#endif
}
#endif

// The dividend is clk_spiX_div[1:0] + 1
void SPI_ClkSetting(uint32_t spi_id, uint32_t clk_src, uint32_t clk_div)
{
	uint32_t div_reg_val = 0;
	uint32_t src_reg_val = 0;
	uint32_t	tmp = 0;

	printk("SPRDFB [%s], clk src is %d clk div is %d\n ", __FUNCTION__, clk_src, clk_div);

	if(0 == spi_id)
	{
		src_reg_val = sci_glb_read(REG_GLB_CLKDLY,0xffffffff);
		div_reg_val = sci_glb_read(REG_GLB_GEN2, 0xffffffff);
		
		sci_glb_write(REG_GLB_CLKDLY, ((src_reg_val&(~(0x3<<26)) )|(clk_src << 26)), 
			0x3 << 26);
		sci_glb_write(REG_GLB_GEN2, ((div_reg_val&(~(0x7<<21))) |(clk_div << 21)),
			0x7 << 21);
	} else if(1 == spi_id) {
		src_reg_val = sci_glb_read(REG_GLB_CLKDLY,0xffffffff);
		div_reg_val = sci_glb_read(REG_GLB_GEN2,0xffffffff);
		sci_glb_write(REG_GLB_CLKDLY, ((src_reg_val&(~(0x3<<30))) |(clk_src << 30)),
			0x3 << 30);
		sci_glb_write(REG_GLB_GEN2, ((src_reg_val&(~(0x7<<11))) |(clk_div << 11)), 
			0x7 << 11);
	}else if(2 == spi_id){
		src_reg_val = sci_glb_read(REG_GLB_GEN3, 0xffffffff);
		div_reg_val = sci_glb_read(REG_GLB_GEN3, 0xffffffff);

		tmp = (src_reg_val&(~(0x1f<<3))) |(clk_src << 3)|(clk_div << 5);
		sci_glb_write(REG_GLB_GEN3, tmp, 0x1f << 3);
	}else{
		printk("SPRDFB [%s], %d is SPI  error channel bit! ", __FUNCTION__, spi_id);
		return;
	}
    
}


#define SPI_SEL_CS_SHIFT 8
#define SPI_SEL_CS_MASK (0x0F<<SPI_SEL_CS_SHIFT)
void SPI_SetCsLow( uint32_t spi_sel_csx , bool is_low)
{
   volatile SPI_CTL_REG_T *spi_ctr_ptr = (volatile SPI_CTL_REG_T*)(SPI_USED_BASE);

    if(is_low)     {
        /* spi_ctl0[11:8]:cs3<->cs0 chip select, 0-selected;1-none */
        spi_ctr_ptr->ctl0 &= ~(SPI_SEL_CS_MASK);
        spi_ctr_ptr->ctl0 &= ~((1<<spi_sel_csx)<<SPI_SEL_CS_SHIFT);
    }
    else
    {
        /* spi_ctl0[11:8]:cs3<->cs0 chip select, 0-selected;1-none */
        spi_ctr_ptr->ctl0 |= ((1<<spi_sel_csx)<<SPI_SEL_CS_SHIFT);
    }
}

#define SPI_CD_MASK  BIT(15)
void SPI_SetCd( uint32_t cd)
{
    volatile SPI_CTL_REG_T *spi_ctr_ptr = (volatile SPI_CTL_REG_T*)(SPI_USED_BASE);
    
    /* 0-command;1-data */
    if(cd == 0)
        spi_ctr_ptr->ctl8 &= ~(SPI_CD_MASK);
    else
        spi_ctr_ptr->ctl8 |= (SPI_CD_MASK);
}

/* USE spi interface to write cmd/data to the lcm	*/
/* pay attention to the data_format */
typedef enum data_width
{
  DATA_WIDTH_7bit =7,
  DATA_WIDTH_8bit =8,
  DATA_WIDTH_9bit =9,
  DATA_WIDTH_10bit=10,
  DATA_WIDTH_11bit=11,
  DATA_WIDTH_12bit=12,
}  SPI_DATA_WIDTH;

/* Set spi work mode for LCM with spi interface //[5:3] */
#define SPI_MODE_SHIFT    3
#define SPI_MODE_MASK     (0x07<<SPI_MODE_SHIFT)
void SPI_SetSpiMode(uint32_t spi_mode)
{
    volatile SPI_CTL_REG_T *spi_ctr_ptr = (volatile SPI_CTL_REG_T *)(SPI_USED_BASE);
    uint32_t temp = spi_ctr_ptr->ctl7;
  
    temp &= ~SPI_MODE_MASK;
    temp |= (spi_mode<<SPI_MODE_SHIFT);

    spi_ctr_ptr->ctl7 = temp;
}

/* Transmit data bit number:spi_ctl0[6:2]  */
void  SPI_SetDatawidth(uint32_t datawidth)
{
    volatile SPI_CTL_REG_T *spi_ctr_ptr = (volatile SPI_CTL_REG_T *)(SPI_USED_BASE);
    uint32_t temp = spi_ctr_ptr->ctl0;

    if( 32 == datawidth )
    {
      spi_ctr_ptr->ctl0 &= ~0x7C;
      return;
    }

    temp &= ~0x0000007C;
    temp |= (datawidth<<2);

    spi_ctr_ptr->ctl0 = temp;
}

#define TX_MAX_LEN_MASK     0xFFFFF
#define TX_DUMY_LEN_MASK    0x3F    //[09:04]
#define TX_DATA_LEN_H_MASK  0x0F    //[03:00]
#define TX_DATA_LEN_L_MASK  0xFFFF  //[15:00]
/*****************************************************************************/
//  Description:  Set rxt data length with dummy_len
//  Author     :  lichd
//    Note       :  the unit is identical with datawidth you set
/*****************************************************************************/ 
void SPI_SetTxLen(uint32_t data_len, uint32_t dummy_bitlen)
{
    volatile SPI_CTL_REG_T *spi_ctr_ptr = (volatile SPI_CTL_REG_T *)(SPI_USED_BASE);
    uint32_t ctl8 = spi_ctr_ptr->ctl8;
    uint32_t ctl9 = spi_ctr_ptr->ctl9;

    data_len &= TX_MAX_LEN_MASK;

    ctl8 &= ~((TX_DUMY_LEN_MASK<<4) | TX_DATA_LEN_H_MASK);
    ctl9 &= ~( TX_DATA_LEN_L_MASK );

    /* set dummy_bitlen in bit[9:4] and data_len[19:16] in bit[3:0] */
    spi_ctr_ptr->ctl8 = (ctl8 | (dummy_bitlen<<4) | (data_len>>16));
    /* set data_len[15:00] */
    spi_ctr_ptr->ctl9 = (ctl9 | (data_len&0xFFFF));
}


#define RX_MAX_LEN_MASK     0xFFFFFF
#define RX_DUMY_LEN_MASK    0x3F    //[09:04]
#define RX_DATA_LEN_H_MASK  0x0F    //[03:00]
#define RX_DATA_LEN_L_MASK  0xFFFF  //[15:00]
/*****************************************************************************/
//  Description:  Set rxt data length with dummy_len
//  Author     :  lichd
//    Note       :  the unit is identical with datawidth you set
/*****************************************************************************/
void SPI_SetRxLen(uint32_t data_len, uint32_t dummy_bitlen)
{
    volatile SPI_CTL_REG_T *spi_ctr_ptr = (volatile SPI_CTL_REG_T *)(SPI_USED_BASE);
    uint32_t ctl10 = spi_ctr_ptr->ctl10;
    uint32_t ctl11 = spi_ctr_ptr->ctl11;

    data_len &= RX_MAX_LEN_MASK;

    ctl10 &= ~((RX_DUMY_LEN_MASK<<4) | RX_DATA_LEN_H_MASK);
    ctl11 &= ~( RX_DATA_LEN_L_MASK );

    /* set dummy_bitlen in bit[9:4] and data_len[19:16] in bit[3:0] */
    spi_ctr_ptr->ctl10 = (ctl10 | (dummy_bitlen<<4) | (data_len>>16));
    /* set data_len[15:00] */
    spi_ctr_ptr->ctl11 = (ctl11 | (data_len&0xFFFF));

	/* in SPIMODE_3WIRE_9BIT_SDI ,DO
	the tx len has to set 0 to generate 8 clk, or generate 9clk */

	if(0x10==(spi_ctr_ptr->ctl7&(0x7 << 3))){
		spi_ctr_ptr->ctl8 &= 0xfff0;
		spi_ctr_ptr->ctl9 &= 0x0;
		}
}

/* Request txt trans before send data */
#define SW_RX_REQ_MASK BIT(0)
#define SW_TX_REQ_MASK BIT(1)
void SPI_TxReq( void )
{
    volatile SPI_CTL_REG_T *spi_ctr_ptr = (volatile SPI_CTL_REG_T *)(SPI_USED_BASE);

    spi_ctr_ptr->ctl12 |= SW_TX_REQ_MASK;
}

void SPI_RxReq( void )
{
    volatile SPI_CTL_REG_T *spi_ctr_ptr = (volatile SPI_CTL_REG_T *)(SPI_USED_BASE);

    spi_ctr_ptr->ctl12 |= SW_RX_REQ_MASK;
}

/*****************************************************************************/
//  Description:      To enable or disable DMA depending on parameter(is_enable)
//  Author:         @Vine.Yuan 2010.5.10
//    Note:
/*****************************************************************************/ 
bool SPI_EnableDMA(uint32_t spi_index,bool is_enable)
{
    volatile SPI_CTL_REG_T* spi_ctl;

    spi_ctl = (volatile SPI_CTL_REG_T *)(SPI_USED_BASE+0x3000*spi_index);

    if (is_enable)
    {
        spi_ctl->ctl2 |= BIT(6);
    }
    else
    {
        spi_ctl->ctl2 &= ~(BIT(6));
    }
    
    /* spi_ctl->ctl7 |= (BIT_7 | BIT_8); */
    spi_ctl->ctl7 |= (BIT(7));

    return true;
}

#define SPI_DMA_TIME_OUT        0x80000
#define BURST_SIZE              16
#define BURST_SIZE_MARK         0xF
#define LENGTH_4_DIVIDE         4

void SPI_Init(SPI_INIT_PARM *spi_parm)
{
    volatile SPI_CTL_REG_T *spi_ctr_ptr = (volatile SPI_CTL_REG_T *)(SPI_USED_BASE);
    uint32_t temp;
    
    SPI_Reset(2, 100);  //Reset spi0&spi1
    
    spi_ctr_ptr->clkd = spi_parm->clk_div;

    temp  = 0;
    temp |= (spi_parm->tx_edge << 1)    |
            (spi_parm->rx_edge << 0)    |
            (0x1 << 13)					|
            (spi_parm->msb_lsb_sel<< 7) ;
    spi_ctr_ptr->ctl0 = temp;
    spi_ctr_ptr->ctl1 |=  BIT(12) | BIT(13);     // set rx/tx mode

	/*rx fifo full watermark is 16*/
	spi_ctr_ptr->ctl3 = 0x10;

	spi_ctr_ptr->ctl7 &= ~(0x7 << 3);
	spi_ctr_ptr->ctl7 |= SPIMODE_3WIRE_9BIT_SDIO << 3;

}

void SPI_WaitTxFinish()
{
     volatile SPI_CTL_REG_T *spi_ctr_ptr = (volatile SPI_CTL_REG_T *)(SPI_USED_BASE);

    while( !((spi_ctr_ptr->iraw)&BIT(8)) ) // IS tx finish
    {
    }
    spi_ctr_ptr->iclr |= BIT(8);
    
    /* Wait for spi bus idle */
    while((spi_ctr_ptr->sts2)&BIT(8))
    {
    }
    /* Wait for tx real empty */
    while( !((spi_ctr_ptr->sts2)&BIT(7)) )
    {
    }
}

void SPI_WriteData(uint32_t data, uint32_t data_len, uint32_t dummy_bitlen)
{
     volatile SPI_CTL_REG_T *spi_ctr_ptr = (volatile SPI_CTL_REG_T *)(SPI_USED_BASE);

    /* The unit of data_len is identical with buswidth */
    SPI_SetTxLen(data_len, dummy_bitlen);
    SPI_TxReq( );

    spi_ctr_ptr->data = data;

    SPI_WaitTxFinish();
}

uint32_t SPI_ReadData( uint32_t data_len, uint32_t dummy_bitlen )
{
    uint32_t read_data=0, rxt_cnt=0;
    volatile SPI_CTL_REG_T *spi_ctr_ptr = (volatile SPI_CTL_REG_T *)(SPI_USED_BASE);

    /* The unit of data_len is identical with buswidth */
    SPI_SetRxLen(data_len, dummy_bitlen);
    SPI_RxReq( );

    /* Wait for spi receive finish */
    while( !((spi_ctr_ptr->iraw)&BIT(9)) )
    {
        /* wait rxt fifo full */
        if((spi_ctr_ptr->iraw)&BIT(6))
        {
            rxt_cnt = (spi_ctr_ptr->ctl3)&0x1F;
            printk("---FIFOFULL:rxt_cnt=0x%x", rxt_cnt);
            while(rxt_cnt--)
            {
                read_data = spi_ctr_ptr->data;
                printk("---FIFOFULL: SPI_ReadData =0x%x", read_data);
            }
        }
    }

    /* Wait for spi bus idle */
    while((spi_ctr_ptr->sts2)&BIT(8))
    {
    }
    
    while(data_len--)
    {
        read_data = spi_ctr_ptr->data;
        printk("---Finish: SPI_ReadData =0x%x", read_data);
    }

     return (read_data);
}
