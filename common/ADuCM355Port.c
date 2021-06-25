/******************************************************************************
Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/

#include "ad5940.h"
#include "DioLib.h"
#include "IntLib.h"
#include<stdbool.h>

#define SYSTICK_MAXCOUNT ((1L<<24)-1) /* we use Systick to complete function Delay10uS(). This value only applies to ADuCM355. */
#define SYSTICK_CLKFREQ   26000000L   /* Systick clock frequency in Hz. This only appies to ADuCM355 */
volatile static uint32_t ucInterrupted = 0;       /* Flag to indicate interrupt occurred */

extern volatile bool bNewCmdFlg;

void AD5940_Delay10us(uint32_t time)
{
  if(time==0)return;
  if(time*10<SYSTICK_MAXCOUNT/(SYSTICK_CLKFREQ/1000000)){
    SysTick->LOAD = time*10*(SYSTICK_CLKFREQ/1000000);
    SysTick->CTRL = (1 << 2) | (1<<0);    /* Enable SysTick Timer, using core clock */
    while(!((SysTick->CTRL)&(1<<16)));    /* Wait until count to zero */
    SysTick->CTRL = 0;                    /* Disable SysTick Timer */
  }
  else {
    AD5940_Delay10us(time/2);
    AD5940_Delay10us(time/2 + (time&1));
  }
}

uint32_t AD5940_GetMCUIntFlag(void)
{
   return ucInterrupted;
}

uint32_t AD5940_ClrMCUIntFlag(void)
{
   ucInterrupted = 0;
   return 1;
}

/* Functions that used to initialize MCU platform */
uint32_t AD5940_MCUResourceInit(void *pCfg)
{
  /**
   * Note: In ADuCM355, the AFE/AD5940 INTC output is connected to EXTI3 of MCU.
  */
  /* Initialize internal GPIO connected to M355 AFE die INTC*/
  DioCfgPin(pADI_GPIO2, PIN1, 0);       
  DioIenPin(pADI_GPIO2, PIN1, 1); /* Enable P2.1 input path. */
  EiCfg(EXTINT3, INT_EN, INT_FALL); /* Falling edge. */
  
  NVIC_EnableIRQ(AFE_EVT3_IRQn);
  return 0;
}

#define MCU_STATUS_WAKEUP   2
extern volatile uint8_t wakeup;

/**
 * Check XINT_EXT_STAT to determine interrupt source.
 * BIT[3]: AFE INTC interrupt.
 * BIT[5]: UART_RX
*/
void Afe_Int3_Handler()
{
  EiClr(BITM_XINT_CLR_AFEIRQ);  /* Clear the interrupt flag. */
  ucInterrupted = 1;

	
	/*Enable UART interrupt since it's not retained in hibernate mode*/
	pADI_UART0->COMIEN |= (BITM_UART_COMIEN_ERBFI | BITM_UART_COMIEN_ELSI);
	/*clear UARTRX intterrupt status*/
	pADI_XINT0->CLR = BITM_XINT_CLR_UART_RX_CLR;
	/*Disable UART_RX wakeup interrupt while MCU is active*/
	pADI_XINT0->CFG0 &= (~EXTUARTRX);
	/*update MCU status*/
	wakeup = MCU_STATUS_WAKEUP;
	// bNewCmdFlg=true;

}


