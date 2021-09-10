#include "ad5940.h"
#include <stdio.h>
#include "string.h"
#include "math.h"

#define SINC3OSR_SEL  ADCSINC3OSR_4
#define SINC2OSR_SEL  ADCSINC2OSR_22
#define MEASURE_FREQ	16.0f	// 16Hz(16SPS)
#define FIFO_THRESHOLD	16		//generate FIFO threshold interrupt every 16 data -> matches the buff size because interrupt isn't triggered correctly at threshold

#define BUFF_SIZE 16 // lowest buffer size without throwing errors

void _ad5940_analog_init(void);
void AD5940_TemperatureInit(void);
void AD5940_TemperatureISR(void);
void AD5940_PrintTemperatureResult(void);
AD5940Err AppTemperatureCtrl(uint32_t Command);
