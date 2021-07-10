#include "ad5940.h"
#include <stdio.h>
#include "string.h"
#include "Temperature.h"

/**
 * This example shows how to configure temperature sensor and using sequencer to take
 * measurements. There is 'chop' function to remove offset errors from circuit, this
 * feature is in register REG_AFE_TEMPSENS and is not included in this example. Enable
 * this function will have better accuracy.
*/


//this buffer will be used by sequence generator and used to store result from AD5940
uint32_t buff[BUFF_SIZE];
uint32_t data_count = 0;  //the temperature data count in buffer.

/**
	* @brief Initializes the temperature sensor
	* @return return 0.
*/
void _ad5940_analog_init(void){
  AFERefCfg_Type aferef_cfg;
  ADCBaseCfg_Type adc_base;
  ADCFilterCfg_Type adc_filter;
  //init ad5940 for temeprature measurement.
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);  /* Init all to disable state */
  aferef_cfg.HpBandgapEn = bTRUE;
  aferef_cfg.Hp1V1BuffEn = bTRUE;
  aferef_cfg.Hp1V8BuffEn = bTRUE;       /* The High speed buffers are automatically turned off during hibernate */
  aferef_cfg.Disc1V1Cap = bFALSE;
  aferef_cfg.Disc1V8Cap = bFALSE;
  aferef_cfg.Hp1V8ThemBuff = bFALSE;
  aferef_cfg.Hp1V8Ilimit = bFALSE;
  aferef_cfg.Lp1V1BuffEn = bFALSE;
  aferef_cfg.Lp1V8BuffEn = bFALSE;
  /* LP reference control - turn off them to save powr*/
  aferef_cfg.LpBandgapEn = bFALSE;
  aferef_cfg.LpRefBufEn = bFALSE;
  aferef_cfg.LpRefBoostEn = bFALSE;
  AD5940_REFCfgS(&aferef_cfg);	
  /* Initialize ADC basic function */
  adc_base.ADCMuxP = ADCMUXP_TEMPP;
  adc_base.ADCMuxN = ADCMUXN_TEMPN;
  adc_base.ADCPga = ADCPGA_1P5;
  AD5940_ADCBaseCfgS(&adc_base);
  /* Initialize ADC filters ADCRawData-->SINC3-->SINC2+NOTCH */
  adc_filter.ADCSinc3Osr = SINC3OSR_SEL;
  adc_filter.ADCSinc2Osr = SINC2OSR_SEL;
  adc_filter.ADCAvgNum = ADCAVGNUM_2;         /* Don't care about it. Average function is only used for DFT */
  adc_filter.ADCRate = ADCRATE_800KHZ;        /* If ADC clock is 32MHz, then set it to ADCRATE_1P6MHZ. Default is 16MHz, use ADCRATE_800KHZ. */
  adc_filter.BpNotch = bTRUE;                 /* SINC2+Notch is one block, when bypass notch filter, we can get fresh data from SINC2 filter. */
  adc_filter.BpSinc3 = bFALSE;                /* We use SINC3 filter. */

  // this part is giving the blocking when removed- Hui. 3/19
  adc_filter.Sinc3ClkEnable = bTRUE;          /* Enable SINC3 clock.  (removed for some reason?)*/
	adc_filter.Sinc2NotchClkEnable = bTRUE;			/* (removed for some reason?)*/

  adc_filter.Sinc2NotchEnable = bTRUE;        /* Enable the SINC2+Notch block. You can also use function AD5940_AFECtrlS */
	
	adc_filter.DFTClkEnable = bTRUE;						/* (removed for some reason?)*/
	adc_filter.WGClkEnable = bTRUE;							/* (removed for some reason?)*/

  AD5940_ADCFilterCfgS(&adc_filter);
  AD5940_AFECtrlS(AFECTRL_TEMPSPWR, bTRUE);   /* Turn on temperature sensor power */
}

/**
 * @brief Initialize everything we need to measure temperature.
 */
void AD5940_TemperatureInit(void){
  uint32_t const *pSeqCmd;
  uint32_t seq_len;
  SEQInfo_Type seq_info;
  WUPTCfg_Type wupt_cfg;
  ClksCalInfo_Type clks_cal;
  uint32_t WaitClks;
  clks_cal.DataType = DATATYPE_SINC2;
  clks_cal.DataCount = 1;             /* Sample one data when wakeup */
  clks_cal.ADCSinc2Osr = SINC2OSR_SEL;
  clks_cal.ADCSinc3Osr = SINC3OSR_SEL;
  clks_cal.ADCAvgNum = 0;
  clks_cal.RatioSys2AdcClk = 1; /* Assume ADC clock is same as system clock */
  AD5940_ClksCalculate(&clks_cal, &WaitClks);

  _ad5940_analog_init();
  //generate sequence to measure temperature sensor output
  AD5940_SEQGenInit(buff, BUFF_SIZE); //init sequence generator
  AD5940_SEQGenCtrl(bTRUE); //from now on, record all register operations rather than write them to AD5940 through SPI.

  AD5940_SEQGpioCtrlS(AGPIO_Pin1);  //pull high AGPIO1 so we know the sequencer is running by observing pin status with ossilloscope etc.
  AD5940_SEQGenInsert(SEQ_WAIT(16*200));  /* Time for reference settling(if ad5940 is just wake up from hibernate mode) */
  AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE); /* Turn ON ADC power */
  AD5940_SEQGenInsert(SEQ_WAIT(16*50));   /* wait another 50us for ADC to settle. */
  AD5940_AFECtrlS(AFECTRL_TEMPCNV|AFECTRL_ADCCNV, bTRUE);  /* Start ADC convert */
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));
  AD5940_AFECtrlS(AFECTRL_TEMPCNV|AFECTRL_ADCPWR, bFALSE);    /* Stop ADC */
  AD5940_SEQGenInsert(SEQ_WAIT(20));			/* Add some delay before put AD5940 to hibernate, needs some clock to move data to FIFO. */
  AD5940_SEQGpioCtrlS(0);     /* pull low AGPIO so we know end of sequence.*/
  AD5940_EnterSleepS();/* Goto hibernate */
  AD5940_SEQGenCtrl(bFALSE);  /* stop sequence generator */
  if(AD5940_SEQGenFetchSeq(&pSeqCmd, &seq_len) != AD5940ERR_OK){
    puts("Sequence generator error!");
  }
  seq_info.pSeqCmd = pSeqCmd;
  seq_info.SeqId = SEQID_0; //use SEQ0 to run this sequence
  seq_info.SeqLen = seq_len;
  seq_info.SeqRamAddr = 0;  //place this sequence from start of SRAM.
  seq_info.WriteSRAM = bTRUE;// we need to write this sequence to AD5940 SRAM.
  AD5940_SEQInfoCfg(&seq_info);
  
  //now configure wakeup timer to trigger above sequence periodically to measure temperature data.
  wupt_cfg.WuptEn = bFALSE; // do not start it right now.
  wupt_cfg.WuptEndSeq = WUPTENDSEQ_A;
  wupt_cfg.WuptOrder[0] = SEQID_0;
  wupt_cfg.SeqxSleepTime[SEQID_0] = 4-1;
  wupt_cfg.SeqxWakeupTime[SEQID_0] = (uint32_t)(32e3f/MEASURE_FREQ)-4-1;
  AD5940_WUPTCfg(&wupt_cfg);
  //enable sequencer
  AD5940_SEQCtrlS(bTRUE); //now sequencer is ready to be triggerd.
}


/**
	* @brief Process the data from the AD5940 FIFO
	* @return return none.
*/
void AD5940_TemperatureISR(void){
  uint32_t FifoCnt, IntcFlag;
  if(AD5940_WakeUp(10) > 10){  /* Wakeup AFE by read register, read 10 times at most */
    printf("Failed to wakeup AD5940!\n");
    return;
  }
  AD5940_SleepKeyCtrlS(SLPKEY_LOCK);  /* We need time to read data from FIFO, so, do not let AD5940 goes to hibernate automatically */
  IntcFlag = AD5940_INTCGetFlag(AFEINTC_0);
  if(IntcFlag&AFEINTSRC_DATAFIFOTHRESH){
    FifoCnt = AD5940_FIFOGetCnt();
    FifoCnt = FifoCnt>BUFF_SIZE?BUFF_SIZE:FifoCnt;
    data_count = FifoCnt;
    AD5940_FIFORd(buff, FifoCnt);
    AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);    /* Allow AFE to enter sleep mode. AFE will stay at active mode untill sequencer trigger sleep */
    AD5940_EnterSleepS();	//If MCU is too slow, comment this line, otherwise there is chance the sequencer is running at this point.
  }
}

/**
	* @brief Print the averaged temperature data through UART
	* @return return none
*/
void AD5940_PrintTemperatureResult(void){
	float total = 0.0;
  for(int i=0; i<data_count; i++){
    int32_t data = buff[i]&0xffff;
    data -= 0x8000;	//data from SINC2 is added 0x8000, while data from register TEMPSENSDAT has no 0x8000 offset.
    //printf("Result[%d] = %ld, %.2f(C)\n", i, data, data/8.13f/1.5f-273.15f);
		total+=data/8.13f/1.5f-273.15f;
  }
	float averageT = total/data_count;
	//printf("Averaged Temperature: %.2f(C)\n", averageT);
}

