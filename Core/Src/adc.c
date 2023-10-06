/*
 * adc.c
 *
 *  Created on: Aug 19, 2023
 *      Author: user
 */

#include "adc.h"
#include "state.h"
#include <stdio.h>

static bool adc_initialized = false;

uint32_t ConvertSystemCurrentADCtomA(uint32_t adc_count){
	uint32_t result_mA = (uint64_t)1000 * 3000 * adc_count / 4096;	//1000mA/mV, 3000mV Vref, 12 bit ADC
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

void ADCInit(void){

	//Main clock at 168MHz, /2 for bus = 84MHz, /4 for ADC in CubeMX = 21Mhz ADC clock

	// Clear control register for clean start, also to make sure EOCS is not set
	ADC1->CR2 = 0;
	ADC2->CR2 = 0;
	ADC3->CR2 = 0;

	// Enable ADCs
	ADC1->CR2 |= ADC_CR2_ADON;
	ADC2->CR2 |= ADC_CR2_ADON;
	ADC3->CR2 |= ADC_CR2_ADON;

	// Enable scan mode so we can just vary sequence length as needed
	// EOCS not being set, and with SCAN mode enabled, EOC should set when sequence is done
	ADC1->CR1 |= ADC_CR1_SCAN;
	ADC2->CR1 |= ADC_CR1_SCAN;
	ADC3->CR1 |= ADC_CR1_SCAN;

	// Setup injected simultaneous ADC read for current and voltage measurements
	ADC123_COMMON->CCR |= 0x11 << ADC_CCR_MULTI_Pos;	// 10001: Triple Combined regular simultaneous + injected simultaneous mode

	// Using default channel sample time of 3 clock cycles = 143ns, setting 00

	// Using default injected sequence length of 1, setting 00

	// Configure injected channels for current and voltage, with placeholder for ADC3
	ADC1->JSQR |= ADC_VSENSE << ADC_JSQR_JSQ4_Pos;
	ADC2->JSQR |= ADC_ISUM << ADC_JSQR_JSQ4_Pos;
	ADC3->JSQR |= ADC3_NULL << ADC_JSQR_JSQ4_Pos;

    /* Delay for ADC stabilization time */
    /* Compute number of CPU cycles to wait for */
	uint32_t coreclock = HAL_RCC_GetSysClockFreq();
    uint32_t counter = (ADC_STAB_DELAY_US * (coreclock / 1000000U));
    while(counter != 0U)
    {
      counter--;
    }

	adc_initialized = true;

}


//measure Vsense and isense simultaneously and then store in state buffers
void MeasureSystemVoltageCurrent(void){

	if (adc_initialized){
		ADC1->SR = ~ADC_SR_JEOC;		//If we cleared entire status register, we could clear the normal conversion done bit used elsewhere if we interrupt at the wrong time
		ADC1->CR2 |= ADC_CR2_JSWSTART;
		while(!(ADC1->SR & ADC_SR_JEOC));	//assuming when ADC1 is done, ADC2 is done too

		uint32_t voltage_adc = ADC1->JDR1;
		uint32_t current_adc = ADC2->JDR1;
		// ignoring ADC3 result

		//printf("current_adc, voltage_adc, %lu, %lu\n", current_adc, voltage_adc);

		//convert readings from ADC counts to real units
		uint32_t current_mA = ConvertSystemCurrentADCtomA(current_adc);
		uint32_t voltage_mV = ConvertVsenseADCtomV(voltage_adc);

		//store in average buffers
		RecordSystemCurrentMeasurement(current_mA);
		RecordSystemVoltageMeasurement(voltage_mV);
	}
}


