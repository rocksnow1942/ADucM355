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

extern AppSWVCfg_Type AppSWVCfg;

#define APPBUFF_SIZE 1024
uint32_t AppBuff[APPBUFF_SIZE];

float LFOSCFreq;    /* Measured LFOSC frequency */


int UNDEFINED = -99999; // random out-of-bounds number for checking if key is there
void delay(long int);

volatile unsigned char ucCOMSTA0 = 0;         // Variable used to store COMSTA0, UART status register
volatile unsigned char ucCOMIID0 = 0;         // Variable used to store COMIID0, UART Interrupt status register
volatile unsigned char ucComRx = 0;           // Variable used to read UART Rx buffer contents into
unsigned char ucTxBufferEmpty  = 0;	       		// Used to indicate that the UART Tx buffer is empty
unsigned char szTemp[64] = "";		       			// Used to build string before printing to UART
char buffer[64] = "";	       									// Used to acquire incoming string from the UART
unsigned char ucPacketReceived = 0;           // Flag to indicate UART byte received
unsigned char ucInCnt = 0;                    // Used to count incoming bytes over the UART
int iNumBytesInFifo = 0;                      // Used to determine the number of bytes in the UART FIFO

char recvString[256] = "";	// Received string from UART
int stringReceived = 0;			// Flag to indiciate if a string has been received
int iForwardFlag = 0;
int iReverseFlag = 0;

/* All available resistor values for RTIA tuning. The index is the number which chooses the values according to ad5940.h */
int RTIAvals[] = {0,200,1000,2000,3000,4000,6000,8000,10000,12000,16000,20000,24000,30000,32000,40000,48000,64000,85000,96000,100000,
									120000,128000,160000,196000,256000,512000};
/**
	* @brief Compute the best RTIA value for the given maximum current
	* @param maxCurrent: uA of peak-to-peak current we will expect at maximum. Usually less than 50uA
	* @return return RTIA index
									
*/
int getRTIA(float maxCurrent){
	int32_t idealResistor = (int32_t)(0.9f/maxCurrent);
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
static int32_t uartPrint(float *pData, uint32_t DataCount)
{
	if( DataCount <= 2){
		return 0;
	}
	AppSWVCfg_Type *pRampCfg;
  AppSWVGetCfg(&pRampCfg);

  /* Data Prep */
	float forward,reverse,subtract;
	float forwardData[DataCount/2];
	float reverseData[DataCount/2];
	float subtractData[DataCount/2];

	int index = 0;
	for(int i=0;i<DataCount;i+=2)
	{
		
		forward = pData[i];
		reverse = pData[i+1];
		subtract = -1.0 * (forward-reverse);
		forwardData[index] = forward;
		reverseData[index] = reverse;
		subtractData[index] = subtract;
		index++;
	}
	
	/* Print data as JSON : Honestly quicker just printing than dealing with json encoding */
	int vIncrement = -1.0 * pRampCfg->SqrWvRampIncrement;
	int vStart = -1.0 * (pRampCfg->RampStartVolt + pRampCfg->SqrWvAmplitude);
	int vEnd = -1.0 * (pRampCfg->RampPeakVolt + pRampCfg->SqrWvAmplitude);
	printf("{");
	printf("\"vUnit\":1E-3,");
	printf("\"vStart\":%d,", vStart);
	printf("\"vEnd\":%d,", vEnd);
	printf("\"vIncrement\":%d,", vIncrement);
	printf("\"iScale\":1E-6,");
	
	/* Forward Current */
	printf("\"forwardCurrent\":[");
	if(iForwardFlag){
		for(int i=0;i<index;i++)
		{
			if(i!=index-1){
				printf("%.3f,", forwardData[i]);
			}
			else{
				printf("%.3f", forwardData[i]);
			}
		}
	}
	printf("],");
	
	/* Reverse Current */
	printf("\"reverseCurrent\":[");
	if(iReverseFlag){
		for(int i=0;i<index;i++)
		{
			if(i!=index-1){
				printf("%.3f,",reverseData[i]);
			}
			else{
				printf("%.3f",reverseData[i]);
			}
			
		}
	}
	printf("],");
	
	/* Subtracted Current */
	printf("\"subtractCurrent\":[");
  for(int i=0;i<index;i++)
  {
		if(i!=index-1){
			printf("%.3f,",subtractData[i]);
		}
		else{
			printf("%.3f",subtractData[i]);
		}
  }
	printf("]");
	printf("}*");
  return 0;
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
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
	clk_cfg.HfOSC32MHzMode = bFALSE;
	clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
	clk_cfg.SysClkDiv = SYSCLKDIV_1;
	clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
	clk_cfg.ADCClkDiv = ADCCLKDIV_1;
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
void AD5940RampStructInit(float vStart, float vEnd, float vIncrement, float vAmplitude, float frequency, float maxCurrent, float channel) {
  AppSWVCfg_Type *pRampCfg;
  
  AppSWVGetCfg(&pRampCfg);
  /* Step1: configure general parmaters */
  pRampCfg->SeqStartAddr = 0x10;                /* leave 16 commands for LFOSC calibration.  */
  pRampCfg->MaxSeqLen = 1024-0x10;              /* 4kB/4 = 1024  */
  pRampCfg->RcalVal = 200.0;                    /* 200 Ohm RCAL on EVAL-ADuCM355QSPZ */
  pRampCfg->ADCRefVolt = 1820.0f;               /* The real ADC reference voltage. Measure it from capacitor C12 with DMM. */
  pRampCfg->FifoThresh = 480;                   /* Maximum value is 2kB/4-1 = 512-1. Set it to higher value to save power. */
  pRampCfg->SysClkFreq = 16000000.0f;           /* System clock is 16MHz by default */
  pRampCfg->LFOSCClkFreq = LFOSCFreq;           /* LFOSC frequency */
  
	/* Step 2:Configure square wave signal parameters */
	/* Note: The scan runs in inverted of what is indicated and then shifted by the amplitude  */
	/* Note: The algorithm is -1 * voltage + peak-to-peak amplitude to match pico */
  // Default values : vStart=-600.0f; vEnd=0; vIncrement=5.0f; vAmplitude=100.0f; frequency=100.0f; maxCurrent=9.99999975e-05f; channel=-99999.0f;	
	pRampCfg->RampStartVolt = -1.0 * (vStart + vAmplitude);           /* 0V */
	pRampCfg->RampPeakVolt = -1.0 * (vEnd + vAmplitude);            	/* -0.6V */
  pRampCfg->VzeroStart = 1300.0f;              /* 1.3V, Chosen for widest range of votlage scan. Change with caution	*/
  pRampCfg->VzeroPeak = 1300.0f;               /* 1.3V, Chosen for widest range of votlage scan. Change with caution	*/
  pRampCfg->Frequency = frequency;                   /* Frequency of square wave in Hz */
	pRampCfg->SqrWvAmplitude = vAmplitude;              /* Peak-to-peak amplitude of square wave in mV */
	pRampCfg->SqrWvRampIncrement = -1.0 * vIncrement;        /* Increment in mV. Make sure that the sign is correct */
	pRampCfg->SampleDelay = (500.0 / frequency) - 0.5f;                /* Time delay between DAC update and ADC sample. Unit is ms. Calculate theoretical period/2 and then subtract .5ms */

  int rtiaVal = getRTIA(maxCurrent);
  // to use the widest range of voltage scan, we use 1.3[V] for VzeroStart/VzeroPeak. In this case, RTIA value should be 200[Ohm] - Fermi.
 // pRampCfg->LPTIARtiaSel = LPTIARTIA_200R; 		 
  pRampCfg->LPTIARtiaSel = rtiaVal; 		 
  pRampCfg->AdcPgaGain = ADCPGA_1P5;
}

// Simple Delay routine
void delay (long int length)
{
	while (length >0)
    	length--;
}


/* Given UART handler didn't work so I frankensteined it until it did. If UART is acting funny, look here */
void UART_Int_Handler()
{
	int i = 0;

	ucCOMSTA0 = UrtLinSta(pADI_UART0);
	ucCOMIID0 = UrtIntSta(pADI_UART0);
	//  ucCOMIID0 = pADI_UART0->COMIIR;
	if ((ucCOMIID0 & 0xE) == 0x2)	          // Transmit buffer empty
	{
		ucTxBufferEmpty = 1;
	}
	if ((ucCOMIID0 & 0xE) == 0x4)	          // Receive byte
	{
		iNumBytesInFifo = pADI_UART0->COMRFC;    // read the Num of bytes in FIFO
		for (i=0; i<iNumBytesInFifo;i++)
		{
			ucComRx = UrtRx(pADI_UART0);
			char tempChar = (char) ucComRx;
			if(tempChar == '*'){
				//printf("%s\n",recvString);
				//strcpy(recvString, "");
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
	if ((ucCOMIID0 & 0xE) == 0xC)	          // UART Time-out condition
	{
		iNumBytesInFifo = pADI_UART0->COMRFC;    // read the Num of bytes in FIFO
		for (i=0; i<iNumBytesInFifo;i++)
		{
			ucComRx = UrtRx(pADI_UART0);
			char tempChar = (char) ucComRx;
			if(tempChar == '*'){
				//printf("%s\n",recvString);
				//strcpy(recvString, "");
				stringReceived = 1;
				break;
			}
			else if (tempChar == '\n'){
				continue;
			}
			else
			{
				strncat(recvString, &tempChar, 1);
			}
		}
	}
}


/**
	* @brief Main code which waits for UART, scans temperature, runs SWV with UART JSON parameters, and then outputs the data
	* @return Infinite loop
*/
int32_t SensorNum=0;
int32_t gvPretreatment=0, gsecsPretreatment=0;
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
	while(1)
	{
		if(AD5940_GetMCUIntFlag())
		{
			AD5940_ClrMCUIntFlag();
			temp = APPBUFF_SIZE;
			AppSWVISR(AppBuff, &temp);
			uartPrint((float*)AppBuff, temp);
		}
		if (stringReceived == 1) // any received string will rerun the code ... for now
		{
			stringReceived = 0;
			
			// Not sure why but the last character is getting cut off sometimes?? I think it's a UART processing issue
			if(recvString[(strlen(recvString)-1)] != '}'){
				strncat(recvString, "}", 1);
			}
			cJSON *json = cJSON_Parse(recvString);
			cJSON *temp;
			strcpy(recvString, "");
			
			// Adding sanity checks for each key to make sure nothing breaks the code ... doing ternary because it's cool
			// If any key is missing besides "status", then we will continue the while loop and not scan
			temp = cJSON_GetObjectItemCaseSensitive(json, "status"); // expecting the string {"status":1}* as the check to see if the ADuCM355 is connected, and will return the same json
			int status = temp ? temp->valueint : UNDEFINED;
			
			temp = cJSON_GetObjectItemCaseSensitive(json, "chipInserted"); // expecting the string {"chipInserted":1}* as the check to see if the chip is connected, and will return the same json
			int chipInserted = temp ? temp->valueint : UNDEFINED;
			
			temp = cJSON_GetObjectItemCaseSensitive(json, "version"); // expecting the string {"version":1}* as the check to return chip version
			int version = temp ? temp->valueint : UNDEFINED;
			
			temp = cJSON_GetObjectItemCaseSensitive(json, "vScale");
			float vScale = temp ? temp->valuedouble : UNDEFINED;
			
			if(status != UNDEFINED){
				printf("{\"status\":1}*");
			}
			else if(chipInserted != UNDEFINED){
				if(isChipInserted()){
					printf("{\"chipInserted\":1}*");
				}
				else{
					printf("{\"chipInserted\":0}*");
				}
			}
			else if (version != UNDEFINED) {
				printf("{\"version\":\"1.2.0\"}*");
			}
			else if (vScale != UNDEFINED) {
	
				float vFactor = vScale/.001; // turn everything into mV
				
				temp = cJSON_GetObjectItemCaseSensitive(json, "vStart");
				float vStart = temp ? temp->valuedouble * vFactor : UNDEFINED;
				
				temp = cJSON_GetObjectItemCaseSensitive(json, "vEnd");
				float vEnd = temp ? temp->valuedouble * vFactor : UNDEFINED;
				
				temp = cJSON_GetObjectItemCaseSensitive(json, "vIncrement");
				float vIncrement =  temp ? temp->valuedouble * vFactor : UNDEFINED;
				
				temp = cJSON_GetObjectItemCaseSensitive(json, "vAmplitude");
				float vAmplitude = temp ? temp->valuedouble * vFactor : UNDEFINED;
				
				temp = cJSON_GetObjectItemCaseSensitive(json, "freqHz");
				float frequency = temp ? temp->valuedouble : UNDEFINED;
				
				temp = cJSON_GetObjectItemCaseSensitive(json, "iScale");
				float iScale = temp ? temp->valuedouble : UNDEFINED;
				float iFactor = iScale/1; // turn everything into A
				
				temp = cJSON_GetObjectItemCaseSensitive(json, "maxCurrent");
				float maxCurrent = temp ? temp->valuedouble * iFactor : UNDEFINED;
				
				temp = cJSON_GetObjectItemCaseSensitive(json, "iForwardRecv");
				iForwardFlag = temp ? (int) (temp->valuedouble) : UNDEFINED;
				
				temp = cJSON_GetObjectItemCaseSensitive(json, "iReverseRecv");
				iReverseFlag = temp ? (int) (temp->valuedouble) : UNDEFINED;
				
				temp = cJSON_GetObjectItemCaseSensitive(json, "channel");
				float channel = temp ? (int) (temp->valuedouble) : UNDEFINED;
				SensorNum=(int32_t)channel;
				/* Pretreatment doesn't work yet */
				gvPretreatment = (int32_t)(cJSON_GetObjectItemCaseSensitive(json, "vPretreatment")->valuedouble * vFactor);
				gsecsPretreatment = (int32_t)(cJSON_GetObjectItemCaseSensitive(json, "secsPretreatment")->valuedouble);
				
				temp = cJSON_GetObjectItemCaseSensitive(json, "muxSelect"); // expecting the string {"setMuxSelect":0-3}* to set the MUX select pins, and will return the same json
				int muxSelect = temp ? temp->valueint : UNDEFINED;
				
				AppSWVCfg.bParaChanged = bTRUE;				
				AD5940_TemperatureInit();
				AD5940_WUPTCtrl(bTRUE);				
				while(1){
				 /* Check if interrupt flag which will be set when interrupt occured. */
					if(AD5940_GetMCUIntFlag()){
						AD5940_ClrMCUIntFlag(); /* Clear this flag */
						AD5940_TemperatureInit();
						AD5940_TemperatureISR();
						AD5940_WUPTCtrl(bFALSE);
						AD5940_PrintTemperatureResult();
						break;
					}
				}
				
				// Set MUX pins
				if(muxSelect != UNDEFINED){
					setSelectPins((uint8_t) muxSelect);
					//printf("{\"setMuxSelect\":%d}*",muxSelect);
				}
			
				AD5940RampStructInit(vStart,vEnd,vIncrement,vAmplitude,frequency,maxCurrent,channel); // Initialize the SWV values
				
				AppSWVInit(AppBuff, APPBUFF_SIZE);    /* Initialize RAMP application. Provide a buffer, which is used to store sequencer commands */
				
				AppSWVCtrl(APPCTRL_START, 0);          /* Control IMP measurement to start. Second parameter has no meaning with this command. */
				
			}
			cJSON_Delete(json);
		}
	}
}

