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

// accepts stage_num as argument
// stores measured stage current in state location
// stores calibration value in state location
void LiveCalibrateSingleStageCurrent(uint32_t stage_num){

	const LoadStageConfiguration* stage_being_calibrated = GetPointerToSingleStageConfig(stage_num);

	//get mux address and decode
	uint32_t imux_address = stage_being_calibrated->imux_addr;
	uint32_t imux_addr0 = imux_address & 0x1;
	uint32_t imux_addr1 = (imux_address >> 1) & 0x1;
	HAL_GPIO_WritePin(IMUX_S0_GPIO_Port, IMUX_S0_Pin, imux_addr0);
	HAL_GPIO_WritePin(IMUX_S1_GPIO_Port, IMUX_S1_Pin, imux_addr1);

	//configure ADC
	//proper triple mode config already done in ADC init func.

	ADC1->SR = ~(ADC_SR_EOC | ADC_SR_STRT);
	ADC2->SR = ~(ADC_SR_EOC | ADC_SR_STRT);
	ADC3->SR = ~(ADC_SR_EOC | ADC_SR_STRT);

	//setup SW trigger on ADC1 and enable DMA
	ADC1->CR2 &= ~(ADC_CR2_EXTEN_Msk | ADC_CR2_EXTSEL_Msk);	// Clear bits to leave in default SW trigger mode
	ADC123_COMMON->CCR |= (1 << ADC_CCR_DMA_Pos); // Enable DMA with ADCs in triple mode

	//clear sequence registers before configuration
	ADC1->SQR1 = 0;
	ADC1->SQR2 = 0;
	ADC1->SQR3 = 0;
	ADC2->SQR1 = 0;
	ADC2->SQR2 = 0;
	ADC2->SQR3 = 0;
	ADC3->SQR1 = 0;
	ADC3->SQR2 = 0;
	ADC3->SQR3 = 0;

	#define NUM_CAL_ADC_READS 16

	//configure channels ADC1 = vsense, ADC2 = imux sometimes, ADC3 = imux sometimes

	ADC1->SQR3 |= ADC_VSENSE << ADC_SQR3_SQ1_Pos;
	ADC1->SQR3 |= ADC_VSENSE << ADC_SQR3_SQ2_Pos;
	ADC1->SQR3 |= ADC_VSENSE << ADC_SQR3_SQ3_Pos;
	ADC1->SQR3 |= ADC_VSENSE << ADC_SQR3_SQ4_Pos;
	ADC1->SQR3 |= ADC_VSENSE << ADC_SQR3_SQ5_Pos;
	ADC1->SQR3 |= ADC_VSENSE << ADC_SQR3_SQ6_Pos;
	ADC1->SQR2 |= ADC_VSENSE << ADC_SQR2_SQ7_Pos;
	ADC1->SQR2 |= ADC_VSENSE << ADC_SQR2_SQ8_Pos;
	ADC1->SQR2 |= ADC_VSENSE << ADC_SQR2_SQ9_Pos;
	ADC1->SQR2 |= ADC_VSENSE << ADC_SQR2_SQ10_Pos;
	ADC1->SQR2 |= ADC_VSENSE << ADC_SQR2_SQ11_Pos;
	ADC1->SQR2 |= ADC_VSENSE << ADC_SQR2_SQ12_Pos;
	ADC1->SQR1 |= ADC_VSENSE << ADC_SQR1_SQ13_Pos;
	ADC1->SQR1 |= ADC_VSENSE << ADC_SQR1_SQ14_Pos;
	ADC1->SQR1 |= ADC_VSENSE << ADC_SQR1_SQ15_Pos;
	ADC1->SQR1 |= ADC_VSENSE << ADC_SQR1_SQ16_Pos;
	ADC1->SQR1 |= (NUM_CAL_ADC_READS - 1) << ADC_SQR1_L_Pos;


	// Configure ADC 2 or 3 for specific IMUX pin to that sequence
	uint32_t imux_channel_num = stage_being_calibrated->imux->channel_num;
	uint32_t imux_adc_num = stage_being_calibrated->imux->ADC_num;
	ADC_TypeDef* imux_adc_instance = stage_being_calibrated->imux->ADC_instance;

	imux_adc_instance->SQR3 |= imux_channel_num << ADC_SQR3_SQ1_Pos;
	imux_adc_instance->SQR3 |= imux_channel_num << ADC_SQR3_SQ2_Pos;
	imux_adc_instance->SQR3 |= imux_channel_num << ADC_SQR3_SQ3_Pos;
	imux_adc_instance->SQR3 |= imux_channel_num << ADC_SQR3_SQ4_Pos;
	imux_adc_instance->SQR3 |= imux_channel_num << ADC_SQR3_SQ5_Pos;
	imux_adc_instance->SQR3 |= imux_channel_num << ADC_SQR3_SQ6_Pos;
	imux_adc_instance->SQR2 |= imux_channel_num << ADC_SQR2_SQ7_Pos;
	imux_adc_instance->SQR2 |= imux_channel_num << ADC_SQR2_SQ8_Pos;
	imux_adc_instance->SQR2 |= imux_channel_num << ADC_SQR2_SQ9_Pos;
	imux_adc_instance->SQR2 |= imux_channel_num << ADC_SQR2_SQ10_Pos;
	imux_adc_instance->SQR2 |= imux_channel_num << ADC_SQR2_SQ11_Pos;
	imux_adc_instance->SQR2 |= imux_channel_num << ADC_SQR2_SQ12_Pos;
	imux_adc_instance->SQR1 |= imux_channel_num << ADC_SQR1_SQ13_Pos;
	imux_adc_instance->SQR1 |= imux_channel_num << ADC_SQR1_SQ14_Pos;
	imux_adc_instance->SQR1 |= imux_channel_num << ADC_SQR1_SQ15_Pos;
	imux_adc_instance->SQR1 |= imux_channel_num << ADC_SQR1_SQ16_Pos;
	imux_adc_instance->SQR1 |= (NUM_CAL_ADC_READS - 1) << ADC_SQR1_L_Pos;

	// Determine the unused ADC that needed to be configured for a safe channel
	ADC_TypeDef* unused_adc_instance = 0;
	uint32_t unused_adc_channel = 0;
	if (imux_adc_num == 2){
		unused_adc_instance = ADC3;
		unused_adc_channel = ADC3_NULL;
	}
	else {
		unused_adc_instance = ADC2;
		unused_adc_channel = ADC_ISUM;	//ISUM is safe to read because we definitely won't be measuring that on ADC1 or ADC3. If not controlled, it would be possible that ADC1 is set to CH0 for vsense, ADC2 could also be unintentionally left set to CH0 causing a conflict, while ADC3 is measuring some imux
	}

	// Configure the unused ADC for a null pin that won't conflict
	unused_adc_instance->SQR3 |= unused_adc_channel << ADC_SQR3_SQ1_Pos;
	unused_adc_instance->SQR1 |= 0 << ADC_SQR1_L_Pos;	// Leaving at default of zero, for a single read, This will finish before other ADCs do but I think this should be okay? Not sure how DMA will behave with this. Hopefully just transferring same data repeatedly.

	typedef struct {
		uint16_t voltage_adc;
		uint16_t ADC2_result;
		uint16_t ADC3_result;
	} CombinedResults;

	volatile CombinedResults results[NUM_CAL_ADC_READS] = {0};

	//setup DMA to variable for result data
	//ADC1 (master) uses DMA2, Stream 0, channel 0
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
	DMA2_Stream0->CR &= ~DMA_SxCR_EN_Msk;	//Disable DMA channel for config
	while (DMA2_Stream0->CR & DMA_SxCR_EN);	//Wait for EN bit to reset
	DMA2->LIFCR |= DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;	// Clear interrupt flags
	DMA2_Stream0->PAR = (uint32_t)&(ADC123_COMMON->CDR);	// Set peripheral address
	DMA2_Stream0->M0AR = (uint32_t)results; // Set memory address
	DMA2_Stream0->NDTR = 3*NUM_CAL_ADC_READS;	//Set number of data transfers, 3 ADCs x 16 reads each
	DMA2_Stream0->CR |= (0 << DMA_SxCR_CHSEL_Pos) | (2 << DMA_SxCR_PL_Pos) | (1 << DMA_SxCR_MSIZE_Pos) | (1 << DMA_SxCR_PSIZE_Pos) | DMA_SxCR_MINC; 		// Channel 0, Priority High, 16-bit mem size, 16 bit periph. size, Periph to memory mode (default)
	DMA2_Stream0->CR |= DMA_SxCR_EN;	//Enable DMA

	//TESTING
	//Enable EOC interrupt
//	ADC1->CR1 |= ADC_CR1_EOCIE;
//	NVIC_EnableIRQ(ADC1_IRQn);

	//Enable overrun IRQ for testing
//	ADC1->CR1 |= ADC_CR1_OVRIE;
//	ADC2->CR1 |= ADC_CR1_OVRIE;
//	ADC3->CR1 |= ADC_CR1_OVRIE;
//	NVIC_EnableIRQ(ADC_IRQn);

	//send SW start to ADC
	ADC1->CR2 |= ADC_CR2_SWSTART;


	//wait for adc read to be done
	while (!(DMA2->LISR & DMA_LISR_TCIF0));

	//Disable DMA on ADCs for other uses
	ADC123_COMMON->CCR &= ~ADC_CCR_DMA_Msk;
	while (DMA2_Stream0->CR & DMA_SxCR_EN);	//Wait for EN bit to reset

	uint32_t current_adc_sum = 0;
	uint32_t voltage_adc_sum = 0;
	for (int i = 0; i < NUM_CAL_ADC_READS; i++){
		if (imux_adc_num == 2){
			current_adc_sum += results[i].ADC2_result;
		}
		else {
			current_adc_sum += results[i].ADC3_result;
		}

		voltage_adc_sum += results[i].voltage_adc;
	}


	uint32_t voltage_mV = ConvertVsenseADCtomV(voltage_adc_sum) / NUM_CAL_ADC_READS;
	uint32_t current_mA = ConvertStageCurrentADCtomA(current_adc_sum, stage_being_calibrated->size->shunt_uOhms) / NUM_CAL_ADC_READS;

	uint32_t conductance_mA_per_mV = 1000 * current_mA / voltage_mV;

	SetStageCalibratedConductance(stage_num, conductance_mA_per_mV);
	SetSingleStageCurrent(stage_num, current_mA);

	#undef NUM_CAL_ADC_READS
}



