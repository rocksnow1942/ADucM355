/* This code is gutted from the AD5940 which is the measurement engine */
#include "SqrWaveVoltammetry.h"
#include "Temperature.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "ad5940.h"
#include "UrtLib.h"
#include "DioLib.h"
#include "IntLib.h"
#include "cJSON.h"
#include "math.h"

/* Version History
v2.0.8 working version, with ps1 and ps2 both working.
v2.0.9 test remove temmperature check routine.
*/
#define FMWARE_VERSION "2.0.9"

//#define FIFO_THRESHOLD	16	
#define APPBUFF_SIZE 1024
uint32_t AppBuff[APPBUFF_SIZE];
volatile uint8_t resultSent = 0;
float LFOSCFreq;    /* Measured LFOSC frequency */


int UNDEFINED = -99999; // random out-of-bounds number for checking if key is there


//volatile unsigned char ucCOMSTA0 = 0;         // Variable used to store COMSTA0, UART status register

char recvString[256] = "";	// Received string from UART
uint8_t stringReceived = 0;			// Flag to indiciate if a string has been received
uint8_t iForwardFlag = 0;
uint8_t iReverseFlag = 0;

uint8_t firstCommandRecv = 0;
uint8_t pinState =  0;
uint8_t pinEnable =  0;
int counter = 0;




/* All available resistor values for RTIA tuning. The index is the number which chooses the values according to ad5940.h */
int RTIAvals[] = {0,200,1000,2000,3000,4000,6000,8000,10000,12000,16000,20000,24000,30000,32000,40000,48000,64000,85000,96000,100000,
									120000,128000,160000,196000,256000,512000};
/**
	* @brief Compute the best RTIA value for the given maximum current
	* @param maxCurrent: uA of peak-to-peak current we will expect at maximum. Usually less than 50uA
	* @return return RTIA index
    the unit of maxCurrent is uA.
*/
int getRTIA(float maxCurrent){
	int32_t idealResistor = (int32_t)(9e5f/maxCurrent);
	int32_t currentDiff = idealResistor-RTIAvals[0];
	int i = 1, j=0;
	int32_t nextDiff = idealResistor-RTIAvals[i];
	while(nextDiff<currentDiff){
		if(i==26){
			return 26;
		}
		currentDiff = nextDiff;
		i++;
		nextDiff = abs(idealResistor - RTIAvals[i]);
	}
	// fermi made the change so that always using resistor smaller than ideal resistor.
	if(RTIAvals[i-1]>idealResistor) {
		j=i-2;
	} else {
	  j=i-1;
	}
	return j;
}


/**
	* @brief Print the SWV scan data through UART in JSON format
	* @param pData: the buffer stored data for this application. The data from FIFO has been pre-processed.
	* @param DataCount: The available data count in buffer pData.
	* @return return 0.
*/
static void uartPrint(float *pData, uint32_t DataCount)
{
	int i;
	if( DataCount >= 2){
		printf("{");
		/* Forward Current */
		if(iForwardFlag){
			printf("\"f\":[");
			for(i=0;i< (DataCount/2 - 1);i++)
				printf("%.3f,", pData[i * 2]);			
			printf("%.3f],", pData[i * 2]);		
		}
			
		/* Reverse Current */	
		if(iReverseFlag){
			printf("\"r\":[");
			for(i=0;i< (DataCount/2 - 1);i++)
				printf("%.3f,", pData[i * 2 + 1]);
			printf("%.3f],", pData[i * 2 + 1]);			
		}

		/* Subtracted Current */
		printf("\"c\":[");
		for(i=0;i<(DataCount/2 - 1);i++)
			printf("%.3f,", pData[i * 2 + 1] - pData[i * 2]);
		printf("%.3f]}*", pData[i * 2 + 1] - pData[i * 2]);	
	}
}

/**
 * @brief Checks if the chip is inserted, aka gpio2 is pulled to ground 
 * @return returns uint8_t : 1 if true, 0 is false
*/
uint8_t isChipInserted(void){
	uint32_t gpioMask = DioRd(pADI_GPIO1);
	if ((gpioMask & PIN5) == PIN5){
		return 0;
	}
	return 1;
}

/**
 * @brief Sets GPIO 0 - Select 0, either high or low
 * @note uint8_t is either 1 or 0 
 * @return returns void
*/
void setSelect0(uint8_t value){
	if(value){
		DioSetPin(pADI_GPIO1,PIN2);
	}
	else{
		DioClrPin(pADI_GPIO1,PIN2);
	}
}

/**
 * @brief Sets GPIO 1 - Select 1, either high or low
 * @note uint8_t is either 1 or 0 
 * @return returns void
*/
void setSelect1(uint8_t value){
	if(value){
		DioSetPin(pADI_GPIO1,PIN4);
	}
	else{
		DioClrPin(pADI_GPIO1,PIN4);
	}
}

/**
Turn LED to a state. Use a bit mask, RGB order
bit positions are 0 , 3, 4, 
currently, 0 is R, 3 is Blue, 4 is Green

*/
void turnLED(uint8_t state){
	DioSetPin(pADI_GPIO1,(PIN0|PIN3) & state );
	DioSetPin(pADI_GPIO2,PIN4 & state );
	DioClrPin(pADI_GPIO1,(PIN0|PIN3) ^ state);
	DioClrPin(pADI_GPIO2,PIN4 ^ state);		
}

/**
 * @brief Sets GPIO0 and GPIO1 in binary from the given decimal
 * @note uint8_t is either 0-3, 0=00, 1=01, 2=10, 3=11
 * @return returns void
*/
void setSelectPins(uint8_t value){
	switch(value){
		case(0):
			setSelect1(0);
			setSelect0(0);
			
			break;
		case(1):
			setSelect1(0);
			setSelect0(1);
			break;
		case(2):
			setSelect1(1);
			setSelect0(0);
			break;
		case(3):
			setSelect1(1);
			setSelect0(1);
			break;
	}
}

/**
 * @brief The general configuration to AD5940 like FIFO/Sequencer/Clock. 
 * @note This function will firstly reset AD5940 using reset pin.
 * @return return 0.
*/
static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  SEQCfg_Type seq_cfg;  
  FIFOCfg_Type fifo_cfg;
  LFOSCMeasure_Type LfoscMeasure;
	
	/* Platform configuration */
  AD5940_Initialize();    /* Call this right after AFE reset */
	
  /* Step1. Configure clock */
	clk_cfg.ADCClkDiv = ADCCLKDIV_1;
	clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
	clk_cfg.SysClkDiv = SYSCLKDIV_1;
	clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
	clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);
	
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bTRUE;           /* We will enable FIFO after all parameters configured */
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_2KB;   /* 2kB for FIFO, The reset 4kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_SINC2NOTCH;   /* Fermi Original = FIFOSRC_SINC3*/
		fifo_cfg.FIFOThresh = FIFO_THRESHOLD; 					 /* Fermi Original=4 but FIFO_THRESHOLD=16 */
		AD5940_FIFOCfg(&fifo_cfg);	
	/* Configure sequencer and stop it */
  seq_cfg.SeqMemSize = SEQMEMSIZE_4KB;  /* 4kB SRAM is used for sequencer, others for data FIFO */
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bTRUE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg); 
  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);   /* Enable all interrupt in INTC1, so we can check INTC flags */
	AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
	AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH|AFEINTSRC_ENDSEQ|AFEINTSRC_CUSTOMINT0, bTRUE); 
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  
  /* Measure LFOSC frequency */
  /**@note Calibrate LFOSC using system clock. The system clock accuracy decides measurment accuracy. Use XTAL to get better result. */
  LfoscMeasure.CalDuration = 1000.0;  /* 1000ms used for calibration. */
  LfoscMeasure.CalSeqAddr = 0;        /* Put sequence commands from start address of SRAM */
  LfoscMeasure.SystemClkFreq = 16000000.0f; /* 16MHz in this firmware. */
  AD5940_LFOSCMeasure(&LfoscMeasure, &LFOSCFreq);
  //printf("LFOSC Freq:%f\n", LFOSCFreq);
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);         /*  */
  return 0;
}


/**
 * @brief The interface for user to change application paramters.
 * @return return 0.
*/
void AD5940RampStructInit(float vStart, float vEnd, float vIncrement, float vAmplitude, float frequency, float maxCurrent, uint8_t pstat,int32_t vPretreatment,int32_t secsPretreatment)
{
  AppSWVCfg_Type *pRampCfg;
  
  AppSWVGetCfg(&pRampCfg);
  /* Step1: configure general parmaters */
  pRampCfg->SeqStartAddr = 0x10;                /* leave 16 commands for LFOSC calibration.  */
  pRampCfg->MaxSeqLen = 1024-0x10;              /* 4kB/4 = 1024  */
	/*
		This RcalVal changes based on the AppSWVDataProcess in "SWV".c
		Not entirely sure why, but there's a slight offset
	*/
  pRampCfg->RcalVal = 200.0;                  /* 200 Ohm RCAL on EVAL-ADuCM355QSPZ */
  pRampCfg->ADCRefVolt = 1820.0f;               /* The real ADC reference voltage. Measure it from capacitor C12 with DMM. */
  pRampCfg->FifoThresh = 480;                   /* Maximum value is 2kB/4-1 = 512-1. Set it to higher value to save power. */
  pRampCfg->SysClkFreq = 16000000.0f;           /* System clock is 16MHz by default */
  pRampCfg->LFOSCClkFreq = LFOSCFreq;           /* LFOSC frequency */
  
	/* Step 2:Configure square wave signal parameters */
	/* Note: The scan runs in inverted of what is indicated and then shifted by the amplitude  */
	/* Note: The algorithm is -1 * voltage + peak-to-peak amplitude to match pico */
	
	//pRampCfg->RampStartVolt = -100.0f;           /* 0V */
  //pRampCfg->RampPeakVolt = 500.0f;            	/* -0.6V */
	pRampCfg->RampStartVolt = -1.0 * (vStart + vAmplitude);           /* 0V */
	pRampCfg->RampPeakVolt = -1.0 * (vEnd + vAmplitude);            	/* -0.6V */
  pRampCfg->VzeroStart = 1300.0f;              /* 1.3V, Chosen for widest range of votlage scan. Change with caution	*/
  pRampCfg->VzeroPeak = 1300.0f;               /* 1.3V, Chosen for widest range of votlage scan. Change with caution	*/
  //pRampCfg->Frequency = 100;                   /* Frequency of square wave in Hz */
  pRampCfg->Frequency = frequency;                   /* Frequency of square wave in Hz */
  //pRampCfg->SqrWvAmplitude = 100;              /* Peak-to-peak amplitude of square wave in mV */
	pRampCfg->SqrWvAmplitude = vAmplitude;              /* Peak-to-peak amplitude of square wave in mV */
  //pRampCfg->SqrWvRampIncrement = +5.0f;        /* Increment in mV. Make sure that the sign is correct */
	pRampCfg->SqrWvRampIncrement = -1.0 * vIncrement;        /* Increment in mV. Make sure that the sign is correct */
  //pRampCfg->SampleDelay = 4.5f;                /* Time delay between DAC update and ADC sample. Unit is ms. Calculate theoretical period/2 and then subtract .5ms */
	pRampCfg->SampleDelay = (500.0 / frequency) - 0.5f;                /* Time delay between DAC update and ADC sample. Unit is ms. Calculate theoretical period/2 and then subtract .5ms */
  //pRampCfg->LPTIARtiaSel = LPTIARTIA_10K;      /* Maximum current decides RTIA value: RTIA = .6V/Imax, Imax = full-scale current in amps*/
	int rtiaVal = getRTIA(maxCurrent);
	// if rtia value is changed, or sensor is changed, then re do the SWVInit, (RTIA cal)
	if((rtiaVal != pRampCfg->LPTIARtiaSel) ) {
		pRampCfg->SWVInited = bFALSE;
	}
	pRampCfg->LPTIARtiaSel = rtiaVal;      /* Maximum current decides RTIA value: RTIA = .6V/Imax, Imax = full-scale current */
	pRampCfg->AdcPgaGain = ADCPGA_1P5;
	pRampCfg->vPretreatment = vPretreatment;
	pRampCfg->secsPretreatment = secsPretreatment;
	pRampCfg->pstat = pstat;
	if(pstat==1){
		//printf("Channel 0\n");
		pRampCfg->LPAMP = LPAMP1;
		pRampCfg->LPDAC = LPDAC1;
		pRampCfg->REG_AFE_LPDACDAT=REG_AFE_LPDACDAT1;
		pRampCfg->adcMuxN=ADCMUXN_LPTIA1_N;
		pRampCfg->adcMuxP=ADCMUXP_LPTIA1_P;		
	}
	else {
		pRampCfg->LPAMP = LPAMP0;
		pRampCfg->LPDAC = LPDAC0;
		pRampCfg->REG_AFE_LPDACDAT=REG_AFE_LPDACDAT0;
		pRampCfg->adcMuxN=ADCMUXN_LPTIA0_N;
		pRampCfg->adcMuxP=ADCMUXP_LPTIA0_P;		
	}
	pRampCfg->bParaChanged = bTRUE;
}

// Simple Delay routine
// time in milli seconds
void delay (int time)
{
	long int loop = time * 2500;
	while (loop  >0)
    	loop--;
}



void UART_Int_Handler()
{
	static volatile  uint16_t IID = 0;         // Variable used to store COMIID0, UART Interrupt status register
	static volatile  uint16_t ucComRx = 0;           // Variable used to read UART Rx buffer contents into
	static volatile  uint16_t LStatus = 0;         // Variable used to store COMSTA0, UART status register
	LStatus = UrtLinSta(pADI_UART0); //pPort->COMLSR;
	IID = UrtIntSta(pADI_UART0); // pPort->COMIIR
	//  ucCOMIID0 = pADI_UART0->COMIIR;
	
	if (((IID & 0xE) == 0x4) || ((IID & 0xE) == 0xC)) 
	//0x4receie buffer full interrupt, 0xc is Receive FIFO timeout
	{   
		// read the Num of bytes in FIFO; pADI_UART0->COMRFC is the number of bytes in FIFO
		for (int i=0; i < pADI_UART0->COMRFC ;i++)
		{
			ucComRx = UrtRx(pADI_UART0); // pPort->COMRX&0xff;
			char tempChar = (char) ucComRx;			
			if(tempChar == '*'){				
				stringReceived = 1;
				break;
			}
			else if (tempChar == '\n'){
				continue;
			}
			else{
				strncat(recvString, &tempChar, 1);
			}
		}
	} 
	
}

/*
calculate the resistance from appBuff.
Assuming fixed scan protocoland voltage range.
*/
float calcResistor(float *pData,uint32_t DataCount) {
	float resistance = 0.0;
	for (int i=0; i<DataCount; i+=2) {
		resistance += (-800.0f + 25.0f * i) / pData[i] ;
		resistance += (-700.0f + 25.0f * i) / pData[i + 1] ;
	}
	resistance /= DataCount;
	return resistance;
}


/*Run a SWV measurement*/
void SWVMeasure(float vStart, float vEnd, float vIncrement, float vAmplitude,\
	float frequency, float maxCurrent, uint8_t pstat,\
	int32_t vPretreatment,int32_t secsPretreatment,uint8_t channel) {
	// always Set MUX pins				
	setSelectPins(channel);	
	
	AD5940RampStructInit(vStart,vEnd,vIncrement,vAmplitude,frequency,maxCurrent,pstat,vPretreatment,secsPretreatment); // Initialize the SWV values				
	
	AppSWVInit(AppBuff, APPBUFF_SIZE);    /* Initialize RAMP application. Provide a buffer, which is used to store sequencer commands */
	
	AppSWVCtrl(APPCTRL_START, 0);          /* Control IMP measurement to start. Second parameter has no meaning with this command. */
}


/**
Run the board QC sequence.
*/
void BoardQCProcess(void) {
	// resistor orders are : C1-C4 and MeasureResisotr -> Fluid Fill Resistor
	// in kOhm
	float QCResistors[8] ={10.0,47.0,22.0,10.0,33.0,22.0,47.0,33.0}; 
	float R;
	uint32_t dataCount;
	uint8_t errorFound=0;
	
	for (int i=0; i<8;i++) {
		
		turnLED(0); // turn on LED
		SWVMeasure(-800.0,-400.0,50.0,100,100,100,i%2,0,0,i / 2);		
		while(!AD5940_GetMCUIntFlag());
		turnLED(0x19); // turn off led when measure is done
		if(AD5940_GetMCUIntFlag())
		{
			AD5940_ClrMCUIntFlag();
			dataCount = APPBUFF_SIZE;
			AppSWVISR(AppBuff, &dataCount);
			// uartPrint((float*)AppBuff, dataCount);
			R = calcResistor((float*)AppBuff, dataCount);
			printf("Channel %d R%d = %.3f < %f\n",i/2,i,R,QCResistors[i]);
			if(fabs(R - QCResistors[i]) / QCResistors[i]  > 0.1) { 
				errorFound += 1;
			}
			delay(200);
		}
		
		
	}
	turnLED(0x19); // turn off the LED
	while (isChipInserted()) {
		if (errorFound) {			
			turnLED(pinState);
			pinState ^= 0x19;
			delay(500);
		} else {
			turnLED(0); // turn on LED and exit
			break;
		}
	}
	
	

}





/**
	* @brief Main code which waits for UART, scans temperature, runs SWV with UART JSON parameters, and then outputs the data
	* @return Infinite loop
*/
void AD5940_Main(void)
{
  uint32_t temp;  
	strcpy(recvString, "");
	/*
		Recalling AD5940PlatformCfg() causes it to get stuck in AD5940_LFOSCMeasure(&LfoscMeasure, &LFOSCFreq).
		This issue popped up randomly and I'm not sure how to fix it besides only calling it once.
		Unsure of the severity of this.
	*/
	AD5940PlatformCfg(); 
	AD5940_ClrMCUIntFlag();

	BoardQCProcess(); 
	// turn on board LED on:
	// DioSetPin(pADI_GPIO0, PIN5);

	while(1)
	{
		if(AD5940_GetMCUIntFlag())
		{
			AD5940_ClrMCUIntFlag();
			temp = APPBUFF_SIZE;
			AppSWVISR(AppBuff, &temp);
			uartPrint((float*)AppBuff, temp);			
		} 

		// if hasn't receive any command, swith the GPIO0 on and off.
		// if (!firstCommandRecv) {
		// 	counter += 1;
		// 	if( counter % 200 == 0) 
		// 		{
		// 			turnLED(pinState);
		// 			pinState ^=0x19;
		// 			counter = 0;
		// 		}			
		// 	delay(5);
		// }
		
		
		if (stringReceived == 1) // any received string will rerun the code ... for now
		{	
			firstCommandRecv = 1;
			stringReceived = 0;
			
			// // Not sure why but the last character is getting cut off sometimes?? I think it's a UART processing issue
			// if(recvString[(strlen(recvString)-1)] != '}'){
			// 	strncat(recvString, "}", 1);
			// }
			 
			
			cJSON *json = cJSON_Parse(recvString);
			cJSON *temp;
			strcpy(recvString, "");
			

			// if received LED, 
			temp = cJSON_GetObjectItemCaseSensitive(json, "led");
			float led = temp ? temp->valuedouble : UNDEFINED;
			if (led!=UNDEFINED) {
				turnLED(pinState);
				DioSetPin(pADI_GPIO0, PIN5 & (pinState << 1));
				DioClrPin(pADI_GPIO0, PIN5 ^ (pinState << 1));
				pinState ^=0x19;
				
			}

			temp = cJSON_GetObjectItemCaseSensitive(json, "ledoff");
			float ledoff = temp ? temp->valuedouble : UNDEFINED;
			if (ledoff!=UNDEFINED) {
				DioOenPin(pADI_GPIO1,PIN2,pinEnable);			// toggle enable disable pin
				pinEnable ^=1;
			}

			// s key for checking status
			temp = cJSON_GetObjectItemCaseSensitive(json, "s"); // expecting the string {"s":1}* as the check to see if the ADuCM355 is connected, and will return the same json
			int status = temp ? temp->valueint : UNDEFINED;
			
			temp = cJSON_GetObjectItemCaseSensitive(json, "cI"); // expecting the string {"cI":1}* as the check to see if the chip is connected, and will return the same json
			int chipInserted = temp ? temp->valueint : UNDEFINED;
			
			temp = cJSON_GetObjectItemCaseSensitive(json, "v"); // expecting the string {"v":1}* as the check to return firmware version
			int version = temp ? temp->valueint : UNDEFINED;
			
			// starting voltage is always in mV
			temp = cJSON_GetObjectItemCaseSensitive(json, "vS");
			float vStart = temp ? temp->valuedouble : UNDEFINED;
			
			if(status != UNDEFINED){
				printf("{\"s\":1}*");
			}
			else if(chipInserted != UNDEFINED){
				if(isChipInserted()){
					printf("{\"cI\":1}*");
				}
				else{
					printf("{\"cI\":0}*");
				}
			}
			else if (version != UNDEFINED) {
				printf("{\"v\":\"%s\"}*",FMWARE_VERSION);				
			}
			else if (vStart != UNDEFINED) {
	
				
				// Ending voltage is always in mV				
				temp = cJSON_GetObjectItemCaseSensitive(json, "vE");
				float vEnd = temp ? temp->valuedouble : UNDEFINED;
				
				// Scan voltage incremental step in mV
				temp = cJSON_GetObjectItemCaseSensitive(json, "vI");
				float vIncrement =  temp ? temp->valuedouble : UNDEFINED;
				
				// Scan voltage amplitude in mV
				temp = cJSON_GetObjectItemCaseSensitive(json, "vA");
				float vAmplitude = temp ? temp->valuedouble : UNDEFINED;
				// scan frequency in Hz
				temp = cJSON_GetObjectItemCaseSensitive(json, "Hz");
				float frequency = temp ? temp->valuedouble : UNDEFINED;
				
								
				// current Scale (or max expected current)
				temp = cJSON_GetObjectItemCaseSensitive(json, "iS");
				float maxCurrent = temp ? temp->valuedouble : UNDEFINED;
				
				// whether return forward current or not
				temp = cJSON_GetObjectItemCaseSensitive(json, "f");
				iForwardFlag = temp ? (uint8_t) (temp->valueint) : 0;
				
				temp = cJSON_GetObjectItemCaseSensitive(json, "r");
				iReverseFlag = temp ? (uint8_t) (temp->valueint) : 0;
				
				// ps = 0 to use potentiostat 0, ps = 1 to use potentiostat 1
				temp = cJSON_GetObjectItemCaseSensitive(json, "ps");
				uint8_t pstat = temp ? (uint8_t) (temp->valueint) : 0;
				
				// pretreatment voltage in mV
				temp = cJSON_GetObjectItemCaseSensitive(json, "vP");
				int32_t vPretreatment = temp ? (int32_t)(temp->valuedouble) : (int32_t)(vStart);
				// pretreatment time in milli seconds
				temp = cJSON_GetObjectItemCaseSensitive(json, "tP");
				int32_t secsPretreatment = temp ? (int32_t)(temp->valuedouble) : 200;
				
				// select channel 0-3 to set corresonding mux select pins
				temp = cJSON_GetObjectItemCaseSensitive(json, "ch"); // expecting the string {"setMuxSelect":0-3}* to set the MUX select pins, and will return the same json
				uint8_t channel = temp ? (uint8_t)(temp->valueint): 0;
				
				SWVMeasure( vStart,  vEnd,  vIncrement,  vAmplitude,\
							frequency,  maxCurrent,  pstat,\
							vPretreatment, secsPretreatment, channel);		

					
			}
			else {
				// error
				printf("{\"e\":1}*");
			}
			
			cJSON_Delete(json);
		}
	}
}

