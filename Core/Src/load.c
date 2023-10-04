/*
 * sequencing.c
 *
 *  Created on: Aug 7, 2023
 *      Author: user
 */

#include <stdlib.h>

//#include <stdio.h>
//#include <string.h>
#include <math.h>

#include "main.h"
#include "state.h"
#include "adc.h"
#include "load.h"


static const LoadStageConfiguration* stage_being_calibrated = 0;
static uint32_t sinewave[20];
static const uint32_t sinewave_size = 20;


extern TIM_HandleTypeDef htim5;

void StageControl(LoadStageCombo state);
void GenerateSineData(uint32_t current_mA);
void UpdateSineWaveOutput(void);


//checks system_config data and enables/disables load accordingly, enables/disables correct stages to regulate
void LoadControl(void){

	static bool last_enabled = false;

	uint32_t set_current = Get_Constant_ISet();
	LoadMode mode = GetMode();
	bool now_enabled = GetSystemEnabled();

	//if in sine wave mode and and transitioning to on, generate new sine wave data
	if (now_enabled && !last_enabled && mode == kSineWaveMode){
		GenerateSineData(set_current);
	}

	if (now_enabled && mode == kSineWaveMode){
		UpdateSineWaveOutput();
	}
	else if (now_enabled && mode != kSineWaveMode){
		//get stages to enable
		LoadStageCombo stagebits = StageComboSelect(set_current);
		StageControl(stagebits);
	}
	else {
		StageControl(0);
	}

	last_enabled = now_enabled;
}

void GenerateSineData(uint32_t current_mA){
	for (int i = 0; i < sinewave_size; i++){
		sinewave[i] = (current_mA/2)*sin(2*M_PI*i/sinewave_size)+(current_mA/2);
	}
}

void UpdateSineWaveOutput(void){
	static uint32_t i = 0;
	LoadStageCombo stagebits = StageComboSelect(sinewave[i]);
	StageControl(stagebits);
	i++;
	if (i == sinewave_size){
		i = 0;
	}
}

void StageControl(LoadStageCombo state){
	for (int i = 0; i < NUM_STAGES; i++){
		bool stage_i_enabled = (state >> i) & 1;
		const LoadStageConfiguration *stage = GetPointerToSingleStageConfig(i);
		HAL_GPIO_WritePin(stage->io_port, stage->io_pin, stage_i_enabled);
	}
}

void sequenceOn(void){
//	HAL_GPIO_WritePin(IO2_GPIO_Port, IO2_Pin, 1);
//	for (int i = 0; i < 8; i++){
//		int stage_id = stages_to_enable[i];
//		if (stage_id != -1){
//			HAL_GPIO_WritePin(Stage[stage_id].GPIOx, Stage[stage_id].GPIO_Pin, 1);
//		}
//		else {
//			break;
//		}
//
//	}

}

void sequenceOff(void){
//	HAL_GPIO_WritePin(IO2_GPIO_Port, IO2_Pin, 0);
//
//	for (int i = 0; i < 8; i++){
//		int stage_id = stages_to_enable[i];
//		if (stage_id != -1){
//			HAL_GPIO_WritePin(Stage[stage_id].GPIOx, Stage[stage_id].GPIO_Pin, 0);
//		}
//		else {
//			break;
//		}
//
//	}
}


void enableLoad(void){
	// TODO: Confirm HAL starts timer counter at 0
	HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_3);
	HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_4);
}

void disableLoad(void){
	HAL_TIM_OC_Stop_IT(&htim5, TIM_CHANNEL_3);
	HAL_TIM_OC_Stop_IT(&htim5, TIM_CHANNEL_4);
	sequenceOff();
}

enum {
		kStageNum = 0,
		kStageConductance = 1,
		kSortedStageArrayWidth,
	};

//takes pointer to a subarray where element 0 is the stage number and element 1 is it's conductance
int cmp(const void *a, const void *b){
	uint32_t a_conductance = ((const uint32_t*)a)[kStageConductance];
	uint32_t b_conductance = ((const uint32_t*)b)[kStageConductance];

	if (a_conductance > b_conductance){
		return -1;
	}
	else if (b_conductance > a_conductance){
		return 1;
	}
	else {
		return 0;
	}

}

LoadStageCombo StageComboSelect(uint32_t current_set_point_mA){

	//Determine target conductance for specified current set point and input voltage
	uint32_t target_conductance = 1000 * current_set_point_mA / GetVSenseAverage();	//TODO: Fix divide by zero here, probably by implementing minimum voltage check

	//Prepare array with stage number and associated conductance value, use calibrated if present or nominal if not.
	//Not bothering to filter out empty stages now, just handle them in actual subset sum solver

	uint32_t load_stages_sorting[NUM_STAGES][kSortedStageArrayWidth] = {0};	// Note: don't use checking if stage_num == 0 to determine end of array, stage 0 is a valid stage. Check conductance instead, or we could use a sentinel value.

	for (uint32_t i = 0; i < NUM_STAGES; i++) {
		uint32_t nominal_conductance = GetStageNominalConductance(i);
		uint32_t cal_conductance = GetStageCalibratedConductance(i);
			load_stages_sorting[i][kStageNum] = i;
			if (cal_conductance != 0) {		// Use calibrated value if available
				load_stages_sorting[i][kStageConductance] = cal_conductance;
			}
			else {	//Use nominal conductance otherwise
				load_stages_sorting[i][kStageConductance] = nominal_conductance;
			}
	}

	//Sort load stages by calibrated conductance

	qsort(load_stages_sorting, NUM_STAGES, sizeof(load_stages_sorting[0]), cmp);

	//Feed sorted list to subset sum solver, which will return selected stages to turn on as uint64_t (LoadStageCombo)

	LoadStageCombo output_config = 0;
	uint32_t remaining_conductance = target_conductance;

	for (int i = 0; i < NUM_STAGES; i++) {
		uint32_t stage_num = load_stages_sorting[i][kStageNum];
		uint32_t stage_conductance = load_stages_sorting[i][kStageConductance];
		if ((stage_conductance > 0)		//Make sure stage isn't empty
		&& ((int32_t)(remaining_conductance - stage_conductance) >= 0)	//and load stage conductance doesn't exceed remaining needed
				) {
			output_config |= (1 << stage_num);//Set the stage num bit to mark as enabled
			remaining_conductance -= stage_conductance;
		}

	}

	return output_config;

}

//gets measured conductance value for each stage and stores in load stage data
void CalibrateAllStages(void){
	// iterate through all of NUM_STAGES, and run single stage calibration for each that isn't of SIZE_NULL
	for (uint32_t i = 0; i < NUM_STAGES; i++){
		if (GetStageNominalLoadResistance(i) != 0){
			uint32_t cal_conductance = CalibrateSingleStage(i);
			SetStageCalibratedConductance(i, cal_conductance);
		}
	}

}


//accepts the stage number as an argument
//returns measured conductance value
//uint32_t CalibrateSingleStage(uint32_t stage_num){
//	// Turn on stage while simultaneously starting a timer that will trigger an ADC read after 50us
//	// The counter of the timer will overflow after 100us and stop due to one shot mode.
//	// TIM overflow will trigger an interrupt which will turn off the load stage.
//
//	stage_being_calibrated = GetPointerToSingleStageConfig(stage_num);
//
//	//Using timer 3
//	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
//	TIM3->PSC = 0;
//	TIM3->CCR1 = 12599;	//do ADC read after 175us	(leaving enough time in case SysTick IRQ causes ADC delay during 16x sampling
//	TIM3->ARR = 14399;	//200us pulse
//	TIM3->CCMR1 |= 7 << TIM_CCMR1_OC1M_Pos;	//PWM mode 2, OC1REF signal high when CNT >= CCR
//	TIM3->CR2 |= 4 << TIM_CR2_MMS_Pos;	//OC1REF signal is used as trigger output (TRGO), used for ADC triggering
//	TIM3->SR = 0;
//	TIM3->DIER |= TIM_DIER_UIE;	//Enable interrupt on overflow/update
//	NVIC_EnableIRQ(TIM3_IRQn);
//
//	//get mux address and decode
//	uint32_t imux_address = stage_being_calibrated->imux_addr;
//	uint32_t imux_addr0 = imux_address & 0x1;
//	uint32_t imux_addr1 = (imux_address >> 1) & 0x1;
//	HAL_GPIO_WritePin(IMUX_S0_GPIO_Port, IMUX_S0_Pin, imux_addr0);
//	HAL_GPIO_WritePin(IMUX_S1_GPIO_Port, IMUX_S1_Pin, imux_addr1);
//
//	//get adc pin to read
//	uint32_t adc_pin_to_read = stage_being_calibrated->imux_adc_pin;
//
//	//configure ADC
//	//proper dual mode config already done in injected ADC setup func.
//	ADC1->SR = ~(ADC_SR_EOC | ADC_SR_STRT);
//
//	//setup timer trigger on ADC1 and enable
//	ADC1->CR2 &= ~((7 << ADC_CR2_EXTSEL_Pos) | ADC_CR2_EXTTRIG);	//clear the bits so a lower value can be set than present, and so ADON isn't retriggered
//	ADC1->CR2 |= (4 << ADC_CR2_EXTSEL_Pos) | ADC_CR2_EXTTRIG | ADC_CR2_DMA;	// Timer 3 TRGO event, enable trigger, enable DMA
//
//	//setup SW trigger on ADC2, and enable trigger
//	ADC2->CR2 |= (7 << ADC_CR2_EXTSEL_Pos) | ADC_CR2_EXTTRIG;
//
//	//enable ADC scan mode
//	ADC1->CR1 |= ADC_CR1_SCAN;
//	ADC2->CR1 |= ADC_CR1_SCAN;
//
//	//clear sequence registers before configuration
//	ADC1->SQR1 = 0;
//	ADC1->SQR2 = 0;
//	ADC1->SQR3 = 0;
//	ADC2->SQR1 = 0;
//	ADC2->SQR2 = 0;
//	ADC2->SQR3 = 0;
//
//	#define NUM_CAL_ADC_READS 16
//	//configure channels ADC1 = mux, ADC2 = vsense
//	//sequence of four of same channel
//	ADC1->SQR3 |= adc_pin_to_read << ADC_SQR3_SQ1_Pos;
//	ADC1->SQR3 |= adc_pin_to_read << ADC_SQR3_SQ2_Pos;
//	ADC1->SQR3 |= adc_pin_to_read << ADC_SQR3_SQ3_Pos;
//	ADC1->SQR3 |= adc_pin_to_read << ADC_SQR3_SQ4_Pos;
//	ADC1->SQR3 |= adc_pin_to_read << ADC_SQR3_SQ5_Pos;
//	ADC1->SQR3 |= adc_pin_to_read << ADC_SQR3_SQ6_Pos;
//	ADC1->SQR2 |= adc_pin_to_read << ADC_SQR2_SQ7_Pos;
//	ADC1->SQR2 |= adc_pin_to_read << ADC_SQR2_SQ8_Pos;
//	ADC1->SQR2 |= adc_pin_to_read << ADC_SQR2_SQ9_Pos;
//	ADC1->SQR2 |= adc_pin_to_read << ADC_SQR2_SQ10_Pos;
//	ADC1->SQR2 |= adc_pin_to_read << ADC_SQR2_SQ11_Pos;
//	ADC1->SQR2 |= adc_pin_to_read << ADC_SQR2_SQ12_Pos;
//	ADC1->SQR1 |= adc_pin_to_read << ADC_SQR1_SQ13_Pos;
//	ADC1->SQR1 |= adc_pin_to_read << ADC_SQR1_SQ14_Pos;
//	ADC1->SQR1 |= adc_pin_to_read << ADC_SQR1_SQ15_Pos;
//	ADC1->SQR1 |= adc_pin_to_read << ADC_SQR1_SQ16_Pos;
//	ADC1->SQR1 |= ADC_SQR1_L_SHIFT(NUM_CAL_ADC_READS);
//
//
//	ADC2->SQR3 |= ADC_VSENSE << ADC_SQR3_SQ1_Pos;
//	ADC2->SQR3 |= ADC_VSENSE << ADC_SQR3_SQ2_Pos;
//	ADC2->SQR3 |= ADC_VSENSE << ADC_SQR3_SQ3_Pos;
//	ADC2->SQR3 |= ADC_VSENSE << ADC_SQR3_SQ4_Pos;
//	ADC2->SQR3 |= ADC_VSENSE << ADC_SQR3_SQ5_Pos;
//	ADC2->SQR3 |= ADC_VSENSE << ADC_SQR3_SQ6_Pos;
//	ADC2->SQR2 |= ADC_VSENSE << ADC_SQR2_SQ7_Pos;
//	ADC2->SQR2 |= ADC_VSENSE << ADC_SQR2_SQ8_Pos;
//	ADC2->SQR2 |= ADC_VSENSE << ADC_SQR2_SQ9_Pos;
//	ADC2->SQR2 |= ADC_VSENSE << ADC_SQR2_SQ10_Pos;
//	ADC2->SQR2 |= ADC_VSENSE << ADC_SQR2_SQ11_Pos;
//	ADC2->SQR2 |= ADC_VSENSE << ADC_SQR2_SQ12_Pos;
//	ADC2->SQR1 |= ADC_VSENSE << ADC_SQR1_SQ13_Pos;
//	ADC2->SQR1 |= ADC_VSENSE << ADC_SQR1_SQ14_Pos;
//	ADC2->SQR1 |= ADC_VSENSE << ADC_SQR1_SQ15_Pos;
//	ADC2->SQR1 |= ADC_VSENSE << ADC_SQR1_SQ16_Pos;
//	ADC2->SQR1 |= ADC_SQR1_L_SHIFT(NUM_CAL_ADC_READS);
//
//	//IF YOU DO A READ-MODIFY-WRITE ON ADC CR2 WITH ADON ALREADY ENABLED,
//	//AND DON'T ACTUALLY CHANGE ANYTHING, A CONVERSION WILL TRIGGER
//
//	typedef struct {
//		uint16_t current_adc;
//		uint16_t voltage_adc;
//	} CombinedResults;
//
//	volatile CombinedResults results[NUM_CAL_ADC_READS] = {0};
//
//	//setup DMA to variable for result data
//	//ADC1 uses DMA ch 1
//	DMA1_Channel1->CCR = 0;	//Disable DMA channel before changing settings, in case of repeat calls
//	DMA1->IFCR = DMA_IFCR_CGIF1;	//Clear all DMA ch 1 interrupt flags
//	DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);
//	DMA1_Channel1->CMAR = (uint32_t)results;
//	DMA1_Channel1->CNDTR = NUM_CAL_ADC_READS;
//	DMA1_Channel1->CCR = (1 << DMA_CCR_PL_Pos) | (2 << DMA_CCR_MSIZE_Pos) | (2 << DMA_CCR_PSIZE_Pos) | DMA_CCR_MINC | DMA_CCR_EN;	//Medium priority, 32 bit mem and periph. size, increment mem address, enabled DMA Channel
//
//
//
//	//TESTING
//	//Enable EOC interrupt
////	ADC1->CR1 |= ADC_CR1_EOCIE;
////	NVIC_EnableIRQ(ADC1_IRQn);
//
//	TIM3->CR1 |= TIM_CR1_CEN | TIM_CR1_OPM;	//Enable timer in oneshot mode
//	stage_being_calibrated->io_port->BSRR = stage_being_calibrated->io_pin << 0;		//Turn on stage
//
//	//wait for adc read to be done
//	while (!((DMA1->ISR & DMA_ISR_TCIF1) && (DMA1_Channel1->CNDTR == 0)));
//	HAL_GPIO_WritePin(IO1_GPIO_Port, IO1_Pin, 1);
//	HAL_GPIO_WritePin(IO1_GPIO_Port, IO1_Pin, 0);
//
//	uint32_t current_adc_sum = 0;
//	uint32_t voltage_adc_sum = 0;
//	for (int i = 0; i < NUM_CAL_ADC_READS; i++){
//		current_adc_sum += results[i].current_adc;
//		voltage_adc_sum += results[i].voltage_adc;
//	}
//
//
//	uint32_t voltage_mV = ConvertVsenseADCtomV(voltage_adc_sum) / NUM_CAL_ADC_READS;
//	uint32_t current_mA = ConvertStageCurrentADCtomA(current_adc_sum, stage_being_calibrated->size->shunt_uOhms) / NUM_CAL_ADC_READS;
//
//	uint32_t conductance_mA_per_mV = 1000 * current_mA / voltage_mV;
//
//	while(TIM3->CR1 & TIM_CR1_CEN);	//Wait for timer to be done
//
//	return conductance_mA_per_mV;
//	#undef NUM_CAL_ADC_READS
//}

void TIM3_IRQHandler(void){

	if (TIM3->SR & TIM_SR_UIF){
		stage_being_calibrated->io_port->BSRR = stage_being_calibrated->io_pin << 16;	//Turn off stage
		TIM3->SR &= ~TIM_SR_UIF;	//Clear interrupt flag
	}

}

//void ADC1_2_IRQHandler(void){
//
//
//	if (ADC1->SR & ADC_SR_EOC){
//		HAL_GPIO_WritePin(IO1_GPIO_Port, IO1_Pin, 1);
//		HAL_GPIO_WritePin(IO1_GPIO_Port, IO1_Pin, 0);
//		ADC1->SR &= ~ADC_SR_EOC;
//		//ADC1->CR1 &= ~ADC_CR1_EOCIE;	//Disable interrupt so it doesn't loop
//	}
//}
