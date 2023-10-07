/*
 * adc.h
 *
 *  Created on: Aug 19, 2023
 *      Author: user
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include <stdint.h>

void MeasureSystemVoltageCurrent(void);
void SetupInjectedDualADCReadForIRQ(void);
uint32_t ConvertVsenseADCtomV(uint32_t adc_count);
uint32_t ConvertStageCurrentADCtomA(uint32_t adc_count, uint32_t shunt_uOhms);
void ADCInit(void);
void LiveCalibrateSingleStageCurrent(uint32_t stage_num);

#endif /* INC_ADC_H_ */

