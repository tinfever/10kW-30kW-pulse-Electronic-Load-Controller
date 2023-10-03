/*
 * state.c
 *
 *  Created on: Aug 5, 2023
 *      Author: user
 */
#include "state.h"
#include <stdbool.h>

const uint32_t system_max_current_mA = 999900;

const LoadStageParameters kLoadStageParamsBySize[kNumberOfStageSizes] = {	//Order corresponds to LoadStageSize enum order
		{.load_mOhm = 8000,	.mA_per_V = 125, .watt_rating = 40, .shunt_uOhms = 16000},			//SIZE_8R
		{.load_mOhm = 4000,	.mA_per_V = 250, .watt_rating = 50, .shunt_uOhms = 8000},			//SIZE_4R
		{.load_mOhm = 2000,	.mA_per_V = 500, .watt_rating = 100, .shunt_uOhms = 4000},			//SIZE_2R
		{.load_mOhm = 1000,	.mA_per_V = 1000, .watt_rating = 300, .shunt_uOhms = 2000},		//SIZE_1R
		{.load_mOhm = 500,	.mA_per_V = 2000, .watt_rating = 300, .shunt_uOhms = 1000},		//SIZE_0R5
		{.load_mOhm = 100,	.mA_per_V = 10000, .watt_rating = 300, .shunt_uOhms = 200},		//SIZE_0R1
		{.load_mOhm = 0,	.mA_per_V = 0, .watt_rating = 0, .shunt_uOhms = 0},				//SIZE_NULL
};

const LoadStageConfiguration load_stage_configs[NUM_STAGES] = {
		{.size = &kLoadStageParamsBySize[SIZE_8R],		.io_port = FET_EN0_GPIO_Port, .io_pin = FET_EN0_Pin, .tmux_ID = 0, .tmux_addr = 0x0, .imux_adc_pin = ADC_IMUX_0B, .imux_addr = 0x0},	//Stage 0
		{.size = &kLoadStageParamsBySize[SIZE_4R],		.io_port = FET_EN1_GPIO_Port, .io_pin = FET_EN1_Pin, .tmux_ID = 0, .tmux_addr = 0x1, .imux_adc_pin = ADC_IMUX_0B, .imux_addr = 0x2},	//Stage 1
		{.size = &kLoadStageParamsBySize[SIZE_0R1],		.io_port = FET_EN2_GPIO_Port, .io_pin = FET_EN2_Pin, .tmux_ID = 0, .tmux_addr = 0x2, .imux_adc_pin = ADC_IMUX_0B, .imux_addr = 0x3},	//Stage 2
		{.size = &kLoadStageParamsBySize[SIZE_2R],		.io_port = FET_EN3_GPIO_Port, .io_pin = FET_EN3_Pin, .tmux_ID = 0, .tmux_addr = 0x3, .imux_adc_pin = ADC_IMUX_0B, .imux_addr = 0x1},	//Stage 3
		{.size = &kLoadStageParamsBySize[SIZE_1R],		.io_port = FET_EN4_GPIO_Port, .io_pin = FET_EN4_Pin, .tmux_ID = 1, .tmux_addr = 0x0, .imux_adc_pin = ADC_IMUX_0A, .imux_addr = 0x3},	//Stage 4
		{.size = &kLoadStageParamsBySize[SIZE_0R5],		.io_port = FET_EN5_GPIO_Port, .io_pin = FET_EN5_Pin, .tmux_ID = 1, .tmux_addr = 0x1, .imux_adc_pin = ADC_IMUX_0A, .imux_addr = 0x0},	//Stage 5
		{.size = &kLoadStageParamsBySize[SIZE_NULL],	.io_port = FET_EN6_GPIO_Port, .io_pin = FET_EN6_Pin, .tmux_ID = 1, .tmux_addr = 0x2, .imux_adc_pin = ADC_IMUX_0A, .imux_addr = 0x1},	//Stage 6
		{.size = &kLoadStageParamsBySize[SIZE_NULL],	.io_port = FET_EN7_GPIO_Port, .io_pin = FET_EN7_Pin, .tmux_ID = 1, .tmux_addr = 0x3, .imux_adc_pin = ADC_IMUX_0A, .imux_addr = 0x2},	//Stage 7
};
//Just iterate through all TMUX inhibit signals and pull high except for one marked by tmux ID

//Decode imux_addr and tmux_addr to S0 and S1 mux control signals for bit 0 and bit 1 respectively.

//This is all so we can call
//int test  = load_stage_configs[3].size->watt_rating

LoadStageData load_stage_data[NUM_STAGES] = {0};

struct {
	uint32_t current_mA_buf[512];
	uint32_t current_mA_buf_size;
	uint32_t current_mA_buf_head;
	uint32_t voltage_mV_buf[512];
	uint32_t voltage_mV_buf_size;
	uint32_t voltage_mV_buf_head;
	uint32_t power_mW;				//Not used right now
} system_data = {
	.current_mA_buf_size = 512,
	.voltage_mV_buf_size = 512,
};


struct {
	int32_t constant_Iset_mA;
	int32_t pulsed_IHigh;
	int32_t pulsed_ILow;
	int32_t pulsed_freq;
	int32_t pulsed_dutycycle;
	LoadMode set_mode;
	bool load_enabled;
} system_config = {0};

void LoadStageInit(void){

}

const LoadStageConfiguration* GetPointerToSingleStageConfig(uint32_t stage_num){
	return &load_stage_configs[stage_num];
}

bool GetSystemEnabled(void){
	return system_config.load_enabled;
}

void SetSystemEnabled(bool val){
	system_config.load_enabled = val;
}

uint32_t Get_Constant_ISet(void){
	return system_config.constant_Iset_mA;
}


void Set_Constant_ISet(int32_t set){
	if (set < 0){
		system_config.constant_Iset_mA = 0;
	}
	else if (set > system_max_current_mA) {
		system_config.constant_Iset_mA = system_max_current_mA;
	}
	else {
		system_config.constant_Iset_mA = set;
	}
}

LoadMode GetMode(void){
	return system_config.set_mode;
}

void SetMode(LoadMode set){
	if (set <= kInvalidMode){
		system_config.set_mode = kInvalidMode + 1;
	}
	else if (set >= kNumModes){
		system_config.set_mode = kNumModes - 1;
	}
	else {
		system_config.set_mode = set;
	}
}

void RecordSystemCurrentMeasurement(uint32_t current_mA){
	system_data.current_mA_buf[system_data.current_mA_buf_head] = current_mA;
	system_data.current_mA_buf_head++;
	if (system_data.current_mA_buf_head >= system_data.current_mA_buf_size){
		system_data.current_mA_buf_head = 0;
	}

}

void RecordSystemVoltageMeasurement(uint32_t voltage_mV){
	system_data.voltage_mV_buf[system_data.voltage_mV_buf_head] = voltage_mV;
		system_data.voltage_mV_buf_head++;
		if (system_data.voltage_mV_buf_head >= system_data.voltage_mV_buf_size){
			system_data.voltage_mV_buf_head = 0;
		}
}

uint32_t GetVSenseAverage(void){
	uint32_t sum = 0;
	uint32_t size = system_data.voltage_mV_buf_size;
	for (int i = 0; i < size; i++){
		sum += system_data.voltage_mV_buf[i];
	}
	uint32_t result = sum / size;
	return result;
}

uint32_t GetISenseAverage(void){
	uint32_t sum = 0;
	uint32_t size = system_data.current_mA_buf_size;

	for (int i = 0; i < size; i++){
		sum += system_data.current_mA_buf[i];
	}
	return sum / size;
}

void SetStageCalibratedConductance(uint32_t stage_num, uint32_t conductance){
	load_stage_data[stage_num].calibrated_mA_per_V = conductance;
}

uint32_t GetStageNominalLoadResistance(uint32_t stage_num){
	return load_stage_configs[stage_num].size->load_mOhm;
}

uint32_t GetStageNominalConductance(uint32_t stage_num){
	return load_stage_configs[stage_num].size->mA_per_V;
}

uint32_t GetStageCalibratedConductance(uint32_t stage_num){
	return load_stage_data[stage_num].calibrated_mA_per_V;
}

void SetStageTemps(uint32_t stage_num, int32_t tc_temp, int32_t thermistor_temp){
	load_stage_data[stage_num].thermocouple_temp_mC = tc_temp;
	load_stage_data[stage_num].thermistor_temp_mC = thermistor_temp;
}

bool isStagePresent(uint32_t stage_num){
	if (load_stage_configs[stage_num].size->load_mOhm != 0){
		return true;
	}
	//else
	return false;
}

// returns thermocouple temperature of selected stage
int32_t GetStageThermocoupleTemp(uint32_t stage_num){
	return load_stage_data[stage_num].thermocouple_temp_mC;
}

//returns number of stage with the highest thermocouple temperature
uint32_t GetMaxThermocoupleTempStageNum(void){
	uint32_t max_stage = 0;
	for (int i = 1; i < NUM_STAGES; i++){
		if (load_stage_data[i].thermocouple_temp_mC > load_stage_data[max_stage].thermocouple_temp_mC){
			max_stage = i;
		}
	}
	return max_stage;
}
