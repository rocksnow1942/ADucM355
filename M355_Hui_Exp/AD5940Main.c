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
#define FMWARE_VERSION "2.1.0"

//#define FIFO_THRESHOLD	16
#define APPBUFF_SIZE 1024
uint32_t AppBuff[APPBUFF_SIZE];

char recvString[256] = "";	// Received string from UART
float LFOSCFreq; /* Measured LFOSC frequency */
uint8_t stringReceived = 0; // Flag to indiciate if a string has been received
uint8_t firstCommandRecv = 0;
uint8_t pinState = 0;
uint8_t pinEnable = 0;
uint8_t iForwardFlag = 0;
uint8_t iReverseFlag = 0;

/* All available resistor values for RTIA tuning. The index is the number which chooses the values according to ad5940.h */

/**
	* @brief Compute the best RTIA value for the given maximum current
	* @param maxCurrent: uA of peak-to-peak current we will expect at maximum. Usually less than 50uA
	* @return return RTIA index
    the unit of maxCurrent is uA.
	ideally RTIA < 0.9V/Imax
*/
uint8_t getRTIA(float maxCurrent)
{
	static int RTIAvals[] = {0, 200, 1000, 2000, 3000, 4000, 6000, 8000, 10000, 12000,
							 16000, 20000, 24000, 30000, 32000, 40000, 48000, 64000, 85000, 96000,
							 100000, 120000, 128000, 160000, 196000, 256000, 512000};
	int32_t idealResistor = (int32_t)(9e5f / maxCurrent);
	uint8_t i = 0;
	while (idealResistor >= RTIAvals[i + 1])
	{
		i++;
		if (i == 26)
			return i;
	}
	return i;
}

/**
	* @brief Print the SWV scan data through UART in JSON format
	* @param pData: the buffer stored data for this application. The data from FIFO are in order of F,R,F,R....
	* @param DataCount: The available data count in buffer pData.
	* @param iForwardFlag: whether to return forward data. 0: no return 1:return forward data
	* @param iReverseFlag: whether to return reverse data. 0: no return 1:return reverse data
	* @return return 0.
*/
static void uartPrint(float *pData, uint32_t DataCount)
{
	int i;
	if (DataCount >= 2)
	{
		printf("{");
		/* Forward Current */
		if (iForwardFlag)
		{
			printf("\"f\":[");
			for (i = 0; i < (DataCount / 2 - 1); i++)
				printf("%.3f,", pData[i * 2]);
			printf("%.3f],", pData[i * 2]);
		}
		/* Reverse Current */
		if (iReverseFlag)
		{
			printf("\"r\":[");
			for (i = 0; i < (DataCount / 2 - 1); i++)
				printf("%.3f,", pData[i * 2 + 1]);
			printf("%.3f],", pData[i * 2 + 1]);
		}
		/* Subtracted Current */
		printf("\"c\":[");
		for (i = 0; i < (DataCount / 2 - 1); i++)
			printf("%.3f,", pData[i * 2 + 1] - pData[i * 2]);
		printf("%.3f]}*", pData[i * 2 + 1] - pData[i * 2]);
	}	
}

/**
 * @brief Checks if the chip is inserted, aka gpio2 is pulled to ground 
 * @return returns uint8_t : 1 if true, 0 is false
*/
uint8_t isChipInserted(void)
{
	uint32_t gpioMask = DioRd(pADI_GPIO1);
	if ((gpioMask & PIN5) == PIN5)
	{
		return 0;
	}
	return 1;
}


/**
Turn LED to a state. Use a bit mask, RGB order
bit positions are 0 , 3, 4, 
currently, 0 is R, 3 is Blue, 4 is Green
*/
void turnLED(uint8_t state)
{
	DioSetPin(pADI_GPIO1, (PIN0 | PIN3) & state);
	DioSetPin(pADI_GPIO2, PIN4 & state);
	DioClrPin(pADI_GPIO1, (PIN0 | PIN3) ^ state);
	DioClrPin(pADI_GPIO2, PIN4 ^ state);
}

/*turn onboard LED on or off*/
void turnBoardLED(uint8_t state) {
	// DioSetPin(pADI_GPIO0, PIN5 & (state << 5));
	// DioClrPin(pADI_GPIO0, PIN5 ^ (state << 5));
	DioSetPin(pADI_GPIO2, PIN4 & (state << 4));
	DioClrPin(pADI_GPIO2, PIN4 ^ (state << 4));
}

/**
 * @brief Sets GPIO0 and GPIO1 in binary from the given decimal
 * @note channel is either 0-3, 0=00, 1=01, 2=10, 3=11
 * @return returns void
*/
void selectChannel(uint8_t channel)
{	
	uint8_t mask = ((channel & 1) << 2) | ((channel & 2) << 3);
	DioSetPin(pADI_GPIO1, (PIN2 | PIN4) & mask);
	DioClrPin(pADI_GPIO1, (PIN2 | PIN4) ^ mask);
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
	AD5940_Initialize(); /* Call this right after AFE reset */

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
	fifo_cfg.FIFOEn = bTRUE; /* We will enable FIFO after all parameters configured */
	fifo_cfg.FIFOMode = FIFOMODE_FIFO;
	fifo_cfg.FIFOSize = FIFOSIZE_2KB;	   /* 2kB for FIFO, The reset 4kB for sequencer */
	fifo_cfg.FIFOSrc = FIFOSRC_SINC2NOTCH; /* Fermi Original = FIFOSRC_SINC3*/
	fifo_cfg.FIFOThresh = FIFO_THRESHOLD;  /* Fermi Original=4 but FIFO_THRESHOLD=16 */
	AD5940_FIFOCfg(&fifo_cfg);
	/* Configure sequencer and stop it */
	seq_cfg.SeqMemSize = SEQMEMSIZE_4KB; /* 4kB SRAM is used for sequencer, others for data FIFO */
	seq_cfg.SeqBreakEn = bFALSE;
	seq_cfg.SeqIgnoreEn = bTRUE;
	seq_cfg.SeqCntCRCClr = bTRUE;
	seq_cfg.SeqEnable = bFALSE;
	seq_cfg.SeqWrTimer = 0;
	AD5940_SEQCfg(&seq_cfg);
	/* Step3. Interrupt controller */
	AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE); /* Enable all interrupt in INTC1, so we can check INTC flags */
	AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
	AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH | AFEINTSRC_ENDSEQ | AFEINTSRC_CUSTOMINT0, bTRUE);
	AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

	/* Measure LFOSC frequency */
	/**@note Calibrate LFOSC using system clock. The system clock accuracy decides measurment accuracy. Use XTAL to get better result. */
	LfoscMeasure.CalDuration = 1000.0;		  /* 1000ms used for calibration. */
	LfoscMeasure.CalSeqAddr = 0;			  /* Put sequence commands from start address of SRAM */
	LfoscMeasure.SystemClkFreq = 16000000.0f; /* 16MHz in this firmware. */
	AD5940_LFOSCMeasure(&LfoscMeasure, &LFOSCFreq);
	//printf("LFOSC Freq:%f\n", LFOSCFreq);
	AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK); /*  */
	return 0;
}

/**
 * @brief The interface for user to change application paramters.
 * @return return 0.
*/
void AD5940RampStructInit(float vStart, float vEnd, float vIncrement, float vAmplitude, float frequency, float maxCurrent, uint8_t pstat, int32_t vPretreatment, int32_t secsPretreatment)
{
	AppSWVCfg_Type *pRampCfg;
	AppSWVGetCfg(&pRampCfg);
	/* Step1: configure general parmaters */
	pRampCfg->SeqStartAddr = 0x10;		/* leave 16 commands for LFOSC calibration.  */
	pRampCfg->MaxSeqLen = 1024 - 0x10;	/* 4kB/4 = 1024  */
										/*
		This RcalVal changes based on the AppSWVDataProcess in "SWV".c
		Not entirely sure why, but there's a slight offset
	*/
	pRampCfg->RcalVal = 200.0;			/* 200 Ohm RCAL on EVAL-ADuCM355QSPZ */
	pRampCfg->ADCRefVolt = 1820.0f;		/* The real ADC reference voltage. Measure it from capacitor C12 with DMM. */
	pRampCfg->FifoThresh = 480;			/* Maximum value is 2kB/4-1 = 512-1. Set it to higher value to save power. */
	pRampCfg->SysClkFreq = 16000000.0f; /* System clock is 16MHz by default */
	pRampCfg->LFOSCClkFreq = LFOSCFreq; /* LFOSC frequency */

	/* Step 2:Configure square wave signal parameters */
	/* Note: The scan runs in inverted of what is indicated and then shifted by the amplitude  */
	/* Note: The algorithm is -1 * voltage + peak-to-peak amplitude to match pico */

	//pRampCfg->RampStartVolt = -100.0f;           /* 0V */
	//pRampCfg->RampPeakVolt = 500.0f;            	/* -0.6V */
	pRampCfg->RampStartVolt = -1.0 * (vStart + vAmplitude); /* 0V */
	pRampCfg->RampPeakVolt = -1.0 * (vEnd + vAmplitude);	/* -0.6V */
	pRampCfg->VzeroStart = 1300.0f;							/* 1.3V, Chosen for widest range of votlage scan. Change with caution	*/
	pRampCfg->VzeroPeak = 1300.0f;							/* 1.3V, Chosen for widest range of votlage scan. Change with caution	*/
	//pRampCfg->Frequency = 100;                   /* Frequency of square wave in Hz */
	pRampCfg->Frequency = frequency;					/* Frequency of square wave in Hz */
														//pRampCfg->SqrWvAmplitude = 100;              /* Peak-to-peak amplitude of square wave in mV */
	pRampCfg->SqrWvAmplitude = vAmplitude;				/* Peak-to-peak amplitude of square wave in mV */
														//pRampCfg->SqrWvRampIncrement = +5.0f;        /* Increment in mV. Make sure that the sign is correct */
	pRampCfg->SqrWvRampIncrement = -1.0 * vIncrement;	/* Increment in mV. Make sure that the sign is correct */
														//pRampCfg->SampleDelay = 4.5f;                /* Time delay between DAC update and ADC sample. Unit is ms. Calculate theoretical period/2 and then subtract .5ms */
	pRampCfg->SampleDelay = (500.0 / frequency) - 0.5f; /* Time delay between DAC update and ADC sample. Unit is ms. Calculate theoretical period/2 and then subtract .5ms */
														//pRampCfg->LPTIARtiaSel = LPTIARTIA_10K;      /* Maximum current decides RTIA value: RTIA = .6V/Imax, Imax = full-scale current in amps*/
	int rtiaVal = getRTIA(maxCurrent);
	// if rtia value is changed, or sensor is changed, then re do the SWVInit, (RTIA cal)
	if ((rtiaVal != pRampCfg->LPTIARtiaSel))
	{
		pRampCfg->SWVInited = bFALSE;
	}
	pRampCfg->LPTIARtiaSel = rtiaVal; /* Maximum current decides RTIA value: RTIA = .6V/Imax, Imax = full-scale current */
	pRampCfg->AdcPgaGain = ADCPGA_1P5;
	pRampCfg->vPretreatment = vPretreatment;
	pRampCfg->secsPretreatment = secsPretreatment;
	pRampCfg->pstat = pstat;
	if (pstat == 1)
	{
		//printf("Channel 0\n");
		pRampCfg->LPAMP = LPAMP1;
		pRampCfg->LPDAC = LPDAC1;
		pRampCfg->REG_AFE_LPDACDAT = REG_AFE_LPDACDAT1;
		pRampCfg->adcMuxN = ADCMUXN_LPTIA1_N;
		pRampCfg->adcMuxP = ADCMUXP_LPTIA1_P;
	}
	else
	{
		pRampCfg->LPAMP = LPAMP0;
		pRampCfg->LPDAC = LPDAC0;
		pRampCfg->REG_AFE_LPDACDAT = REG_AFE_LPDACDAT0;
		pRampCfg->adcMuxN = ADCMUXN_LPTIA0_N;
		pRampCfg->adcMuxP = ADCMUXP_LPTIA0_P;
	}
	pRampCfg->bParaChanged = bTRUE;
}

// Simple Delay routine
// time in milli seconds roughly
void delay(uint16_t time)
{
	long int loop = time * 2500;
	while (loop > 0)
		loop--;
}

void UART_Int_Handler()
{
	static volatile uint16_t IID = 0;	  // Variable used to store COMIID0, UART Interrupt status register
	static volatile uint16_t ucComRx = 0; // Variable used to read UART Rx buffer contents into
	static volatile uint16_t LStatus = 0; // Variable used to store COMSTA0, UART status register
	LStatus = UrtLinSta(pADI_UART0);	  //pPort->COMLSR;
	IID = UrtIntSta(pADI_UART0);		  // pPort->COMIIR
	//  ucCOMIID0 = pADI_UART0->COMIIR;
	if (((IID & 0xE) == 0x4) || ((IID & 0xE) == 0xC))
	//0x4receie buffer full interrupt, 0xc is Receive FIFO timeout
	{
		// read the Num of bytes in FIFO; pADI_UART0->COMRFC is the number of bytes in FIFO
		for (int i = 0; i < pADI_UART0->COMRFC; i++)
		{
			ucComRx = UrtRx(pADI_UART0); // pPort->COMRX&0xff;
			char tempChar = (char)ucComRx;
			if (tempChar == '*')
			{
				stringReceived = 1;
				break;
			}
			else if (tempChar == '\n')
			{
				continue;
			}
			else
			{
				strncat(recvString, &tempChar, 1);
			}
		}
	}
}

/*Run a SWV measurement*/
void SWVMeasure(float vS, float vE, float vI, float vA,
				float f, float mC, uint8_t ps,
				int32_t vP, int32_t sP, uint8_t ch)
{	
	selectChannel(ch);						// always Set MUX pins
	AD5940RampStructInit(vS, vE, vI, vA, f, mC, ps, vP, sP); // Initialize the SWV values
	AppSWVInit(AppBuff, APPBUFF_SIZE);						 /* Initialize RAMP application. Provide a buffer, which is used to store sequencer commands */
	AppSWVCtrl(APPCTRL_START, 0);							 /* Control IMP measurement to start. Second parameter has no meaning with this command. */
}

/*
calculate the resistance from appBuff.
Assuming fixed scan protocoland voltage range.
*/
float calcResistor(float *pData, uint32_t DataCount)
{
	float resistance = 0.0;
	for (int i = 0; i < DataCount; i += 2)
	{
		resistance += (-800.0f + 25.0f * i) / pData[i];
		resistance += (-700.0f + 25.0f * i) / pData[i + 1];
	}
	resistance /= DataCount;
	return resistance;
}

/**
Run the board QC sequence.
return QC status.
0: success; chip inserted and QC passed.
1-8: fail; chip inserted but QC failed on 1-8 channels.
10: default; chip not inserted.
*/
uint8_t BoardQCProcess(void)
{
	// resistor orders are : C1-C4 and MeasureResisotr -> Fluid Fill Resistor
	// in kOhm
	static float QCResistors[8] = {10.0, 47.0, 22.0, 10.0, 33.0, 22.0, 47.0, 33.0};
	float R;
	uint32_t dataCount;
	uint8_t errorFound = 0;
	for (int i = 0; i < 8; i++)
	{
		turnBoardLED(1);; // turn off onbooard LED when measurement starts
		SWVMeasure(-800.0, -400.0, 50.0, 100, 100, 100, i % 2, 0, 0, i / 2);
		while (!AD5940_GetMCUIntFlag())
			;
		turnBoardLED(0); // turn on led when measure is done
		if (AD5940_GetMCUIntFlag())
		{
			AD5940_ClrMCUIntFlag();
			dataCount = APPBUFF_SIZE;
			AppSWVISR(AppBuff, &dataCount);
			// uartPrint((float*)AppBuff, dataCount);
			R = calcResistor((float *)AppBuff, dataCount);
			// printf("Channel %d R%d = %.3f < %f\n", i / 2, i, R, QCResistors[i]);
			if (fabs(R - QCResistors[i]) / QCResistors[i] > 0.1)
			{
				errorFound += 1;
			}
			delay(200);
		}
	}
	if (isChipInserted()) {
		return errorFound;
	} else { 
		return 10;
	}	
}

// send {a:0} to master
static void ack(uint8_t i)
{
	printf("{\"a\":%d}*", i);
}

void handleJson(cJSON *json) //uint32_t * pDataCount
{
	cJSON *temp;
	
	if (cJSON_GetObjectItemCaseSensitive(json, "s"))
	{
		ack(1);
	}
	else if (cJSON_GetObjectItemCaseSensitive(json, "cI"))
	{
		printf("{\"cI\":%d}*", isChipInserted());
	}
	else if (cJSON_GetObjectItemCaseSensitive(json, "v"))
	{
		printf("{\"v\":\"%s\"}*", FMWARE_VERSION);
	}
	else if (cJSON_GetObjectItemCaseSensitive(json, "vS"))
	{
		temp = cJSON_GetObjectItemCaseSensitive(json, "vS"); // Starting voltage is always in mV
		float vStart = temp ? temp->valuedouble : -600.0f;
		temp = cJSON_GetObjectItemCaseSensitive(json, "vE"); // Ending voltage is always in mV
		float vEnd = temp ? temp->valuedouble : 0.0f;
		temp = cJSON_GetObjectItemCaseSensitive(json, "vI"); // Increment voltage
		float vIncrement = temp ? temp->valuedouble : 5.0f;
		temp = cJSON_GetObjectItemCaseSensitive(json, "vA"); // Amplitude of voltage
		float vAmplitude = temp ? temp->valuedouble : 100.0f;
		temp = cJSON_GetObjectItemCaseSensitive(json, "Hz"); //frequency in Hz
		float frequency = temp ? temp->valuedouble : 100.0f;
		temp = cJSON_GetObjectItemCaseSensitive(json, "iS"); //max expected current in uA
		float maxCurrent = temp ? temp->valuedouble : 100.0f;
		temp = cJSON_GetObjectItemCaseSensitive(json, "ps"); // which pstat to use, 0 or 1
		uint8_t pstat = temp ? (uint8_t)(temp->valueint) : 0;
		temp = cJSON_GetObjectItemCaseSensitive(json, "vP"); // pretreatment voltage in mV
		int32_t vPretreatment = temp ? (int32_t)(temp->valuedouble) : (int32_t)(vStart);
		temp = cJSON_GetObjectItemCaseSensitive(json, "tP"); // pretreatment time in ms
		int32_t secsPretreatment = temp ? (int32_t)(temp->valuedouble) : 200;
		temp = cJSON_GetObjectItemCaseSensitive(json, "ch"); // which channel to scan 0-3 for channel 1-4
		uint8_t channel = temp ? (uint8_t)(temp->valueint) : 0;

		temp = cJSON_GetObjectItemCaseSensitive(json, "f"); // wether to return forward current
		iForwardFlag = temp ? (uint8_t)(temp->valueint) : 0;
		temp = cJSON_GetObjectItemCaseSensitive(json, "r"); // wether to return reverse current
		iReverseFlag = temp ? (uint8_t)(temp->valueint) : 0;

		SWVMeasure(vStart, vEnd, vIncrement, vAmplitude,
				   frequency, maxCurrent, pstat,
				   vPretreatment, secsPretreatment, channel);	
	}
	else
	{			
		// error, not a valid command
		printf("{\"e\":1}*");
		
	}
}


/*
Depends on the board QC status, show different LED state
*/
#define INTERVAL 99999
void showStatusLED(uint8_t* status) {
	static uint8_t ledState = 0x19;
	static uint32_t count = 0;	
	if (*status == 0) { 
		// QC success then blink green
		if (count % INTERVAL == 0) {			
			ledState ^= 0x10;
		}
	} else if (*status == 10) {
		// not doing a QC, then blink white 
		if (count % INTERVAL == 0) {			
			ledState ^= 0x19;			
		}
	} else {
		// QC fail or have sensor inserted, then blick red
		if (count % INTERVAL == 0) {			
			ledState ^= 0x1;
		}
		// if chip is unplugged, then change status to blink white.
		if (!isChipInserted()) {
			ledState = 0x0;
			*status = 10;
		}
	}
	turnLED(ledState);
	// let the onboard LED follow red-led
	// so if not doing a QC or if QC fail, then onboard LED will blink, suggest a QC-incomplete.
	turnBoardLED(ledState & 0x1);
	count++;
	if (count == 0xFFFFFFFE) {
		count = 0;
	}
}


/**
	* @brief Main code which waits for UART, scans temperature, runs SWV with UART JSON parameters, and then outputs the data
	* @return Infinite loop
*/
void AD5940_Main(void)
{	
	uint32_t dataCount;
	strcpy(recvString, "");

	AD5940PlatformCfg();
	AD5940_ClrMCUIntFlag();
	
	// turn on board LED on:
	turnBoardLED(0);

	uint8_t boardQC = BoardQCProcess();
	
	while (1) {
		if (stringReceived==1) {
			stringReceived = 0;
			cJSON *json = cJSON_Parse(recvString);
			strcpy(recvString, "");
			if (cJSON_GetObjectItemCaseSensitive(json, "s")) {
				ack(1);
				// disable the RGB-LEDs
				DioOenPin(pADI_GPIO1,PIN0|PIN3,0); 
				DioOenPin(pADI_GPIO2,PIN4,0);
				break;
			}		
		} else {
			showStatusLED(&boardQC);
		}		
	}

	// turn off board LED after first string received:
	turnBoardLED(1);

	while (1)
	{	
		if(AD5940_GetMCUIntFlag())
		{
			turnBoardLED(0);
			AD5940_ClrMCUIntFlag();
			dataCount = APPBUFF_SIZE;
			AppSWVISR(AppBuff, &dataCount);
			uartPrint((float *)AppBuff, dataCount);
			turnBoardLED(1);
		}

		if (stringReceived == 1) // any received string will rerun the code ... for now
		{	
			// turn on board LED after received a command;
			turnBoardLED(0);
			stringReceived = 0;

			cJSON *json = cJSON_Parse(recvString);
			strcpy(recvString, "");
			if (json == NULL)
			{
				// error 0 in parsing JSON from string
				printf("{\"e\":0}*");				
			}
			else
			{
				handleJson(json);
			}			
			cJSON_Delete(json);
			turnBoardLED(1);
		}
	}
}
