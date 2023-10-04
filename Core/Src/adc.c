/*
 * adc.c
 *
 *  Created on: Aug 19, 2023
 *      Author: user
 */

#include "adc.h"
#include "state.h"
#include <stdio.h>

uint32_t ConvertSystemCurrentADCtomA(uint32_t adc_count){
	uint32_t result_mA = 100 * 3000 * adc_count / 4096;	//100mA/mV, 3000mV Vref, 12 bit ADC
	return result_mA;
}

uint32_t ConvertVsenseADCtomV(uint32_t adc_count){
	uint32_t result_mV = 5 * 3000 * adc_count / 4096;	//5x attenuation on input, 3000mV Vref, 12 bit ADC
	return result_mV;
}

uint32_t ConvertStageCurrentADCtomA(uint32_t adc_count, uint32_t shunt_uOhms){
	uint32_t result_mA = ((uint64_t)adc_count * 3000 * 1000000) / ((uint64_t)4096 * 100 * shunt_uOhms);		//3000mV Vref, 1000000 due to uOhms, 100V/V current sense amp gain
	return result_mA;
}

//void SetupInjectedDualADCReadForIRQ(void){
//	ADC1->CR1 |= 1 << ADC_CR1_DUALMOD_Pos;	// Combined regular simultaneous + injected simultaneous mode
//
//	ADC1->CR2 |= 7 << ADC_CR2_JEXTSEL_Pos;	//JSWSTART for injection trigger
//	ADC1->CR2 |= ADC_CR2_JEXTTRIG;	//Enable JSWSTART
//
//	ADC2->CR2 |= 7 << ADC_CR2_JEXTSEL_Pos;	//JSWSTART for injection trigger
//	ADC2->CR2 |= ADC_CR2_JEXTTRIG;
//
//	ADC1->JSQR |= ADC_ISENSE << ADC_JSQR_JSQ4_Pos;
//	ADC2->JSQR |= ADC_VSENSE << ADC_JSQR_JSQ4_Pos;
//}

//measure Vsense and isense simultaneously and then store in state buffers
void MeasureSystemVoltageCurrent(void){

	ADC1->SR = ~ADC_SR_JEOC;		//If we cleared entire status register, we could clear the normal conversion done bit used elsewhere if we interrupt at the wrong time
	ADC1->CR2 |= ADC_CR2_JSWSTART;
	while(!(ADC1->SR & ADC_SR_JEOC));	//assuming when ADC1 is done, ADC2 is done too


	uint32_t current_adc = ADC1->JDR1;
	uint32_t voltage_adc = ADC2->JDR1;

	//printf("current_adc, voltage_adc, %lu, %lu\n", current_adc, voltage_adc);

	//convert readings from ADC counts to real units
	uint32_t current_mA = ConvertSystemCurrentADCtomA(current_adc);
	uint32_t voltage_mV = ConvertVsenseADCtomV(voltage_adc);

	//store in average buffers
	RecordSystemCurrentMeasurement(current_mA);
	RecordSystemVoltageMeasurement(voltage_mV);
}


