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

static LoadStageCombo pulse_high_combo = 0;
static LoadStageCombo pulse_low_combo = 0;


void StageControl(LoadStageCombo state);
void GenerateSineData(uint32_t current_mA);
void UpdateSineWaveOutput(void);
void EnablePulseLoad(void);
void DisablePulseLoad(void);
void UpdatePulseLoad(void);


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

	if (now_enabled && !last_enabled && mode == kPulsedMode){
		EnablePulseLoad();
	}

	if (now_enabled && mode == kSineWaveMode){
		UpdateSineWaveOutput();
	}
	else if (now_enabled && mode == kConstantMode){
		//get stages to enable
		LoadStageCombo stagebits = StageComboSelect(set_current);
		StageControl(stagebits);
	}
	else if (now_enabled && mode == kPulsedMode){
		UpdatePulseLoad();
	}
	else {
		StageControl(0);
		DisablePulseLoad();	//Fix this later
	}

	last_enabled = now_enabled;

	// MOnitor if there is a change of enablement or mode

	//if mode changes, disable load and cleanup from last mode?


}

void EnablePulseLoad(void){
	// If this function called is on each LoadUpdate task call, StageComboSelect will use latest voltage which might have been taken when pulse was high or low or alternating.
	// This might cause fluctuation in pulse levels
	// Ideally we'd sample voltage during high and low periods and use those to determine high and low stage solutions respectively.

	// Configure timer for correct frequency and duty cycle, and to fire IRQ for pulse high and pulse low.
	// Using TIM2, 32-bit
	// APB1 timer clock 84Mhz

	// Frequency range from 1 - 3500 Hz
	// Period range from 1s to 285.7us
	// clock cycle period range from 84E6 to 24000

	//Prescaler will be 0, since cycle period range will fit within 32-bit timer


	//Enable clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	TIM2->CR1 &= ~TIM_CR1_CEN;	//Disable, just in case

	TIM2->SR = 0;	//Clear status

	//TIM2->CR1 |= TIM_CR1_ARPE;	//Enable ARR buffer

	// Enable Update and CC1 IRQ
	TIM2->DIER |= TIM_DIER_UIE | TIM_DIER_CC1IE;
	NVIC_EnableIRQ(TIM2_IRQn);

	UpdatePulseLoad();

	TIM2->CR1 |= TIM_CR1_CEN;	//Enable timer
}

void UpdatePulseLoad(void){
	// calculate stage combos for high and low levels
	uint32_t pulse_high_current = GetPulsedIHigh();
	pulse_high_combo = StageComboSelect(pulse_high_current);

	uint32_t pulse_low_current = GetPulsedILow();
	pulse_low_combo = StageComboSelect(pulse_low_current);

	uint32_t freq = GetPulsedFreq();
	uint32_t arr = 84000000/freq;
	TIM2->ARR = arr - 1;

	uint32_t dutycycle = GetPulsedDutyCycle();
	uint32_t cc1 = arr - ((uint64_t)dutycycle * arr / 10000) - 1;	// Divide by 10000 to convert from dutycycle where 9999 = 99.99%.
	TIM2->CCR1 = cc1;

	if (TIM2->CNT > TIM2->ARR){
		TIM2->EGR |= TIM_EGR_UG;
	}

}

void TIM2_IRQHandler(void){
	if (TIM2->SR & TIM_SR_CC1IF){	//pulse high
		TIM2->SR = ~TIM_SR_CC1IF;
		StageControl(pulse_high_combo);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
		HAL_GPIO_WritePin(IO1_GPIO_Port, IO1_Pin, 1);
	}

	else if (TIM2->SR & TIM_SR_UIF){	//pulse low
		TIM2->SR = ~TIM_SR_UIF;
		StageControl(pulse_low_combo);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
		HAL_GPIO_WritePin(IO1_GPIO_Port, IO1_Pin, 0);
	}

}

void DisablePulseLoad(void){
	TIM2->CR1 &= ~TIM_CR1_CEN;	// Disable timer
	StageControl(0); // Turn off all stages
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
	HAL_GPIO_WritePin(IO1_GPIO_Port, IO1_Pin, 0);
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

	//Precalculate each IO port pins to enable and disable
	uint32_t port_d = 0;
	uint32_t port_e = 0;
	uint32_t port_g = 0;
	uint32_t port_i = 0;
	uint32_t port_k = 0;

	for (int i = 0; i < NUM_STAGES; i++){
		bool stage_i_enabled = (state >> i) & 1;
		const LoadStageConfiguration *stage = GetPointerToSingleStageConfig(i);
		//HAL_GPIO_WritePin(stage->io_port, stage->io_pin, stage_i_enabled);
		//uint32_t io_port = stage->io_port;
		switch((uint32_t)stage->io_port){
		case (uint32_t)GPIOD:
			port_d |= stage->io_pin << (16*!stage_i_enabled);
			break;
		case (uint32_t)GPIOE:
			port_e |= stage->io_pin << (16*!stage_i_enabled);
			break;
		case (uint32_t)GPIOG:
			port_g |= stage->io_pin << (16*!stage_i_enabled);
			break;
		case (uint32_t)GPIOI:
			port_i |= stage->io_pin << (16*!stage_i_enabled);
			break;
		case (uint32_t)GPIOK:
			port_k |= stage->io_pin << (16*!stage_i_enabled);
			break;
		default:
			while(1);
			break;
		}
	}

	//Quickly set all the IO pins
	GPIOD->BSRR = port_d;
	GPIOE->BSRR = port_e;
	GPIOG->BSRR = port_g;
	GPIOI->BSRR = port_i;
	GPIOK->BSRR = port_k;

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

bool StageOverpowerOK (uint32_t stage_num, uint32_t vsense_mV){
	const LoadStageConfiguration* stage = GetPointerToSingleStageConfig(stage_num);
	uint32_t stage_rated_power = stage->size->watt_rating;
	uint32_t stage_mOhm = stage->size->load_mOhm;
	uint32_t stage_nominal_conductance = stage->size->mA_per_V;
	uint32_t stage_calibrated_conductance = GetStageCalibratedConductance(stage_num);

	uint32_t stage_conductance = 0;	// mA per V
	if (stage_calibrated_conductance |= 0){
		stage_conductance = stage_calibrated_conductance;
	}
	else {
		stage_conductance = stage_nominal_conductance;
	}


	// Use stage calibrated conductance to calculate current that would flow
	// More accurate than using nominal resistance
	uint32_t stage_estimated_current_mA = vsense_mV * stage_conductance / 1000;

	// Then use that current to determine power dissipation via I^2 * R
	uint32_t stage_estimated_power = (uint64_t) stage_estimated_current_mA * stage_estimated_current_mA * stage_mOhm / 1000 / 1000 / 1000;

	if (stage_estimated_power > stage_rated_power){
		return false;
	}
	else {
		return true;
	}

}

LoadStageCombo StageComboSelect(uint32_t current_set_point_mA){

	//Determine target conductance for specified current set point and input voltage
	//uint32_t vsense_mV = GetVSenseAverage();
	uint32_t vsense_mV = GetVSenseLatest();

	//Disable load if input voltage is too low to prevent divide by zero
	if (vsense_mV < 2000){
		SetSystemEnabled(false);
		return 0;
	}

	uint32_t target_conductance = 1000 * current_set_point_mA / vsense_mV;

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
		bool stage_overpower_OK = StageOverpowerOK(stage_num, vsense_mV) || (GetMode() != kConstantMode);
		if ((stage_conductance > 0)		//Make sure stage isn't empty
		&& ((int32_t)(remaining_conductance - stage_conductance) >= 0)	//and load stage conductance doesn't exceed remaining needed
		&& (stage_overpower_OK)
				) {
			output_config |= ((uint64_t)1 << stage_num); //Set the stage num bit to mark as enabled
			remaining_conductance -= stage_conductance;
		}

	}

	// Don't use a new stage solution if it isn't that much better than the last one, to avoid oscillation between stages due to noise
	// Only if set point didn't change though, otherwise a new solution is needed even if error is higher
	int32_t error = remaining_conductance;
	static int32_t last_error = INT32_MAX;
	static LoadStageCombo last_config = 0;
	static uint32_t last_current_setpoint = 0;

	if ((last_error - error  > 5) || (last_current_setpoint != current_set_point_mA)){	//Improvement is large enough
		last_error = error;
		last_config = output_config;
		last_current_setpoint = current_set_point_mA;
		return output_config;
	}
	else {
		return last_config;
	}




}

//gets measured conductance value for each stage and stores in load stage data
void CalibrateAllStages(void){
	// iterate through all of NUM_STAGES, and run single stage calibration for each that isn't of SIZE_NULL
	for (uint32_t i = 0; i < NUM_STAGES; i++){
		if (GetStageNominalLoadResistance(i) != 0){
			uint32_t cal_conductance = CalibrateSingleStage(i);
			SetStageCalibratedConductance(i, cal_conductance);
			HAL_Delay(15);
		}
	}

}


//accepts the stage number as an argument
//returns measured conductance value
uint32_t CalibrateSingleStage(uint32_t stage_num){
	// Turn on stage while simultaneously starting a timer that will trigger an ADC read after 175us
	// The counter of the timer will overflow after 200us and stop due to one shot mode.
	// TIM overflow will trigger an interrupt which will turn off the load stage.

	stage_being_calibrated = GetPointerToSingleStageConfig(stage_num);

	// Using timer 4
	// APB1 Timer clock is 84 MHz
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	TIM4->PSC = 0;
	TIM4->CCR4 = 14700 - 1;	//do ADC read after 175us	(leaving enough time in case SysTick IRQ causes ADC delay during 16x sampling
	TIM4->ARR = 16800 - 1;	//200us pulse
	TIM4->CCMR2 |= 7 << TIM_CCMR2_OC4M_Pos;	//PWM mode 2, OC4REF signal high when CNT >= CCR
	TIM4->CCER |= TIM_CCER_CC4E; //Enable CC4 output signal, but it won't go to any pin since alt. function registers not setup
	TIM4->SR = 0;
	TIM4->DIER |= TIM_DIER_UIE;	//Enable interrupt on overflow/update
	NVIC_EnableIRQ(TIM4_IRQn);

	//get mux address and decode
	uint32_t imux_address = stage_being_calibrated->imux_addr;
	uint32_t imux_addr0 = imux_address & 0x1;
	uint32_t imux_addr1 = (imux_address >> 1) & 0x1;
	HAL_GPIO_WritePin(IMUX_S0_GPIO_Port, IMUX_S0_Pin, imux_addr0);
	HAL_GPIO_WritePin(IMUX_S1_GPIO_Port, IMUX_S1_Pin, imux_addr1);

	//configure ADC
	//proper triple mode config already done in ADC init func.

	// If we don't clear ADC2 and 3 EOC bits, the temp measurement function will start a triple simultaneous measurement
	// but only read two values, leaving one EOC bit still set I think. Then when we don't clear that
	// EOC bit but enable DMA here, that enables the overrun detection. The overrun detection might know
	// that there is data in one of the ADCs from the temp measurement that wasn't ever read, and that
	// it is overwritten by the new conversions being triggered now, thus raising an overrun fault,
	// disabling the DMA, and causing this code to hang when waiting for the DMA to finish.

	// I think the issue goes away when breakpoints are enabled and the SFR ADC3 data is opened,
	// because that reads the ADC3 DR reg which clears the EOC bit which makes it work, unintentionally.

	ADC1->SR = ~(ADC_SR_EOC | ADC_SR_STRT);
	ADC2->SR = ~(ADC_SR_EOC | ADC_SR_STRT);
	ADC3->SR = ~(ADC_SR_EOC | ADC_SR_STRT);

	//setup timer trigger on ADC1 and enable
	ADC1->CR2 &= ~(ADC_CR2_EXTEN_Msk | ADC_CR2_EXTSEL_Msk);	// Clear bits to confirm starting with zeros
	ADC1->CR2 |= (9 << ADC_CR2_EXTSEL_Pos) | (1 << ADC_CR2_EXTEN_Pos);	// EXTSEL 1001: Timer 4 CC4 event, EXTEN 01: Trigger detection on the rising edge
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


	TIM4->CR1 |= TIM_CR1_CEN | TIM_CR1_OPM;	//Enable timer in oneshot mode
	stage_being_calibrated->io_port->BSRR = stage_being_calibrated->io_pin << 0;		//Turn on stage

	//wait for adc read to be done
	while (!(DMA2->LISR & DMA_LISR_TCIF0));
	HAL_GPIO_WritePin(IO2_GPIO_Port, IO2_Pin, 1);
	HAL_GPIO_WritePin(IO2_GPIO_Port, IO2_Pin, 0);

	// Note: Could actually just turn off stage right here and not need the IRQ to fire and do it.
	// No real need for calibration pulse to be precisely 200us long
	// Timing only matters for time until ADC reads start.
	// Pulse can end any time after ADC reads are finished.

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

	while(TIM4->CR1 & TIM_CR1_CEN);	//Wait for timer to be done

	return conductance_mA_per_mV;
	#undef NUM_CAL_ADC_READS
}

void TIM4_IRQHandler(void){


	HAL_GPIO_WritePin(IO2_GPIO_Port, IO2_Pin, 1);
	HAL_GPIO_WritePin(IO2_GPIO_Port, IO2_Pin, 0);

	if (TIM4->SR & TIM_SR_UIF){
		//flag must be cleared first, otherwise IRQ handler is entered twice due to internal clearing delay
		TIM4->SR &= ~TIM_SR_UIF;	//Clear interrupt flag
		stage_being_calibrated->io_port->BSRR = stage_being_calibrated->io_pin << 16;	//Turn off stage
	}

}

//void ADC_IRQHandler(void){
//
//	while(1);
//
//	if (ADC1->SR & ADC_SR_OVR) {
//		ADC1->SR = ~ADC_SR_OVR;
//	}
//	if (ADC2->SR & ADC_SR_OVR) {
//		ADC2->SR = ~ADC_SR_OVR;
//	}
//	if (ADC3->SR & ADC_SR_OVR) {
//		ADC3->SR = ~ADC_SR_OVR;
//	}
//
//
//		HAL_GPIO_WritePin(IO2_GPIO_Port, IO2_Pin, 1);
//		HAL_GPIO_WritePin(IO2_GPIO_Port, IO2_Pin, 0);
//
//
//}

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
