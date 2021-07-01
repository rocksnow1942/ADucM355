#include "UrtLib.h"
#include "ClkLib.h"
#include "DioLib.h"
#include "IntLib.h"
#include <stdio.h>
#include <string.h>
#include "ad5940.h"

void UartInit(void);
void ClockInit(void);
void GPIOInit(void);
void AD5940_Main(void);
/* Starting code */
int main(void)
{
  ClockInit();
	GPIOInit();
  UartInit();
  AD5940_MCUResourceInit(0);    /* Inialize all peripherals etc. used for AD5940/AFE. */
  AD5940_Main();
}


/* Have to initialize the clock */
void ClockInit(void)
{
   DigClkSel(DIGCLK_SOURCE_HFOSC);
   ClkDivCfg(1,1);   //HCLK = PCLK = 26MHz
}

/* Initialize UART for 115200-8-N-1 
Note: Don't go above 115200 because long messages start to clip data which is vital for json, inputs get messed up and the embedded code is easily breakable
*/
void UartInit(void)
{
	DioCfgPin(pADI_GPIO0,PIN10|PIN11,1);               // Setup P0.10, P0.11 as UART pin
	UrtCfg(pADI_UART0,B115200,
				(BITM_UART_COMLCR_WLS|3),0);         // Configure UART for 115200 baud rate
	UrtFifoCfg(pADI_UART0, RX_FIFO_8BYTE,      // Configure the UART FIFOs for 8 bytes deep
						BITM_UART_COMFCR_FIFOEN);
	UrtFifoClr(pADI_UART0, BITM_UART_COMFCR_RFCLR// Clear the Rx/TX FIFOs
						|BITM_UART_COMFCR_TFCLR);
	UrtIntCfg(pADI_UART0,	BITM_UART_COMIEN_ERBFI |
												BITM_UART_COMIEN_ETBEI |
												BITM_UART_COMIEN_ELSI);                  // Enable Rx, Tx and Rx buffer full Interrupts
	NVIC_EnableIRQ(UART_EVT_IRQn);              // Enable UART interrupt source in NVIC
}


void GPIOInit(void)
{
	/* GPIO 2 - P1.5 - Chip Insertion Detection */
	DioCfgPin(pADI_GPIO1,PIN5,0);          	// configure P1.5 as gpio
	DioIenPin(pADI_GPIO1,PIN5,1);          	// enable P1.5 as input
	DioPulPin(pADI_GPIO1,PIN5,1);          	// enable pull-up
	
	/* GPIO0/GPIO1 - P1.2/P1.4 - MUX selection bits */
	/* Select 0 = GPIO0, Select 1 = GPIO1 */
	DioCfgPin(pADI_GPIO1,PIN2|PIN4,0);      // configure P1.2/P1.4 as gpio
	DioOenPin(pADI_GPIO1,PIN2|PIN4,1);			// enable P1.2/P1.4 as output
	DioPulPin(pADI_GPIO1,PIN2|PIN4,0);			// enable pull-down
	
	/* P2.4 LED for ADuCM355 breakout board */
	//DioCfgPin(pADI_GPIO2,PIN4,0); 		// configure P2.4 as gpio
	//DioOenPin(pADI_GPIO2,PIN4,1);			// enable P2.4 as output
	//DioPulPin(pADI_GPIO2,PIN4,1);			// enable pull-up
	
}
