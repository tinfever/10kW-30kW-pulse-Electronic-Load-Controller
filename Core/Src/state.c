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

// imux_ID referenced array that contains channel number in form of ADC_IMUX_OUT0, ADC number 2 or 3 variable form for programatic access, and the pointer to the ADC instance from ADC2 or ADC3 macro
#define NUM_IMUX 16
const ImuxConfig imux_configs[NUM_IMUX] = {
		{.channel_num = ADC_IMUX_OUT0, .ADC_num = 2, .ADC_instance = ADC2},
		{.channel_num = ADC_IMUX_OUT1, .ADC_num = 2, .ADC_instance = ADC2},
		{.channel_num = ADC_IMUX_OUT2, .ADC_num = 2, .ADC_instance = ADC2},
		{.channel_num = ADC_IMUX_OUT3, .ADC_num = 2, .ADC_instance = ADC2},
		{.channel_num = ADC_IMUX_OUT4, .ADC_num = 2, .ADC_instance = ADC2},
		{.channel_num = ADC_IMUX_OUT5, .ADC_num = 2, .ADC_instance = ADC2},
		{.channel_num = ADC_IMUX_OUT6, .ADC_num = 2, .ADC_instance = ADC2},
		{.channel_num = ADC_IMUX_OUT7, .ADC_num = 2, .ADC_instance = ADC2},

		{.channel_num = ADC_IMUX_OUT8, .ADC_num = 2, .ADC_instance = ADC2},
		{.channel_num = ADC_IMUX_OUT9, .ADC_num = 2, .ADC_instance = ADC2},
		{.channel_num = ADC_IMUX_OUT10, .ADC_num = 3, .ADC_instance = ADC3},
		{.channel_num = ADC_IMUX_OUT11, .ADC_num = 3, .ADC_instance = ADC3},
		{.channel_num = ADC_IMUX_OUT12, .ADC_num = 3, .ADC_instance = ADC3},
		{.channel_num = ADC_IMUX_OUT13, .ADC_num = 3, .ADC_instance = ADC3},
		{.channel_num = ADC_IMUX_OUT14, .ADC_num = 3, .ADC_instance = ADC3},
		{.channel_num = ADC_IMUX_OUT15, .ADC_num = 2, .ADC_instance = ADC2},
};

const LoadStageConfiguration load_stage_configs[NUM_STAGES] = {

		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN0_GPIO_Port, .io_pin = FET_EN0_Pin, .tmux_ID = 0, .tmux_addr = 0, .imux = &imux_configs[0], .imux_addr = 0},          // Stage 0
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN1_GPIO_Port, .io_pin = FET_EN1_Pin, .tmux_ID = 0, .tmux_addr = 1, .imux = &imux_configs[1], .imux_addr = 0},          // Stage 1
		{.size = &kLoadStageParamsBySize[SIZE_8R], .io_port = FET_EN2_GPIO_Port, .io_pin = FET_EN2_Pin, .tmux_ID = 0, .tmux_addr = 2, .imux = &imux_configs[0], .imux_addr = 1},          // Stage 2
		{.size = &kLoadStageParamsBySize[SIZE_0R1], .io_port = FET_EN3_GPIO_Port, .io_pin = FET_EN3_Pin, .tmux_ID = 0, .tmux_addr = 3, .imux = &imux_configs[1], .imux_addr = 1},          // Stage 3
		{.size = &kLoadStageParamsBySize[SIZE_0R1], .io_port = FET_EN4_GPIO_Port, .io_pin = FET_EN4_Pin, .tmux_ID = 1, .tmux_addr = 0, .imux = &imux_configs[0], .imux_addr = 2},          // Stage 4
		{.size = &kLoadStageParamsBySize[SIZE_4R], .io_port = FET_EN5_GPIO_Port, .io_pin = FET_EN5_Pin, .tmux_ID = 1, .tmux_addr = 1, .imux = &imux_configs[1], .imux_addr = 2},          // Stage 5
		{.size = &kLoadStageParamsBySize[SIZE_0R1], .io_port = FET_EN6_GPIO_Port, .io_pin = FET_EN6_Pin, .tmux_ID = 1, .tmux_addr = 2, .imux = &imux_configs[0], .imux_addr = 3},          // Stage 6
		{.size = &kLoadStageParamsBySize[SIZE_2R], .io_port = FET_EN7_GPIO_Port, .io_pin = FET_EN7_Pin, .tmux_ID = 1, .tmux_addr = 3, .imux = &imux_configs[1], .imux_addr = 3},          // Stage 7

		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN8_GPIO_Port, .io_pin = FET_EN8_Pin, .tmux_ID = 2, .tmux_addr = 0, .imux = &imux_configs[2], .imux_addr = 0},          // Stage 8
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN9_GPIO_Port, .io_pin = FET_EN9_Pin, .tmux_ID = 2, .tmux_addr = 1, .imux = &imux_configs[3], .imux_addr = 0},          // Stage 9
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN10_GPIO_Port, .io_pin = FET_EN10_Pin, .tmux_ID = 2, .tmux_addr = 2, .imux = &imux_configs[2], .imux_addr = 1},          // Stage 10
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN11_GPIO_Port, .io_pin = FET_EN11_Pin, .tmux_ID = 2, .tmux_addr = 3, .imux = &imux_configs[3], .imux_addr = 1},          // Stage 11
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN12_GPIO_Port, .io_pin = FET_EN12_Pin, .tmux_ID = 3, .tmux_addr = 0, .imux = &imux_configs[2], .imux_addr = 2},          // Stage 12
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN13_GPIO_Port, .io_pin = FET_EN13_Pin, .tmux_ID = 3, .tmux_addr = 1, .imux = &imux_configs[3], .imux_addr = 2},          // Stage 13
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN14_GPIO_Port, .io_pin = FET_EN14_Pin, .tmux_ID = 3, .tmux_addr = 2, .imux = &imux_configs[2], .imux_addr = 3},          // Stage 14
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN15_GPIO_Port, .io_pin = FET_EN15_Pin, .tmux_ID = 3, .tmux_addr = 3, .imux = &imux_configs[3], .imux_addr = 3},          // Stage 15

		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN16_GPIO_Port, .io_pin = FET_EN16_Pin, .tmux_ID = 4, .tmux_addr = 0, .imux = &imux_configs[4], .imux_addr = 0},          // Stage 16
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN17_GPIO_Port, .io_pin = FET_EN17_Pin, .tmux_ID = 4, .tmux_addr = 1, .imux = &imux_configs[5], .imux_addr = 0},          // Stage 17
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN18_GPIO_Port, .io_pin = FET_EN18_Pin, .tmux_ID = 4, .tmux_addr = 2, .imux = &imux_configs[4], .imux_addr = 1},          // Stage 18
		{.size = &kLoadStageParamsBySize[SIZE_0R1], .io_port = FET_EN19_GPIO_Port, .io_pin = FET_EN19_Pin, .tmux_ID = 4, .tmux_addr = 3, .imux = &imux_configs[5], .imux_addr = 1},          // Stage 19
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN20_GPIO_Port, .io_pin = FET_EN20_Pin, .tmux_ID = 5, .tmux_addr = 0, .imux = &imux_configs[4], .imux_addr = 2},          // Stage 20
		{.size = &kLoadStageParamsBySize[SIZE_0R1], .io_port = FET_EN21_GPIO_Port, .io_pin = FET_EN21_Pin, .tmux_ID = 5, .tmux_addr = 1, .imux = &imux_configs[5], .imux_addr = 2},          // Stage 21
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN22_GPIO_Port, .io_pin = FET_EN22_Pin, .tmux_ID = 5, .tmux_addr = 2, .imux = &imux_configs[4], .imux_addr = 3},          // Stage 22
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN23_GPIO_Port, .io_pin = FET_EN23_Pin, .tmux_ID = 5, .tmux_addr = 3, .imux = &imux_configs[5], .imux_addr = 3},          // Stage 23

		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN24_GPIO_Port, .io_pin = FET_EN24_Pin, .tmux_ID = 6, .tmux_addr = 0, .imux = &imux_configs[6], .imux_addr = 0},          // Stage 24
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN25_GPIO_Port, .io_pin = FET_EN25_Pin, .tmux_ID = 6, .tmux_addr = 1, .imux = &imux_configs[7], .imux_addr = 0},          // Stage 25
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN26_GPIO_Port, .io_pin = FET_EN26_Pin, .tmux_ID = 6, .tmux_addr = 2, .imux = &imux_configs[6], .imux_addr = 1},          // Stage 26
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN27_GPIO_Port, .io_pin = FET_EN27_Pin, .tmux_ID = 6, .tmux_addr = 3, .imux = &imux_configs[7], .imux_addr = 1},          // Stage 27
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN28_GPIO_Port, .io_pin = FET_EN28_Pin, .tmux_ID = 7, .tmux_addr = 0, .imux = &imux_configs[6], .imux_addr = 2},          // Stage 28
		{.size = &kLoadStageParamsBySize[SIZE_0R1], .io_port = FET_EN29_GPIO_Port, .io_pin = FET_EN29_Pin, .tmux_ID = 7, .tmux_addr = 1, .imux = &imux_configs[7], .imux_addr = 2},          // Stage 29
		{.size = &kLoadStageParamsBySize[SIZE_0R1], .io_port = FET_EN30_GPIO_Port, .io_pin = FET_EN30_Pin, .tmux_ID = 7, .tmux_addr = 2, .imux = &imux_configs[6], .imux_addr = 3},          // Stage 30
		{.size = &kLoadStageParamsBySize[SIZE_0R1], .io_port = FET_EN31_GPIO_Port, .io_pin = FET_EN31_Pin, .tmux_ID = 7, .tmux_addr = 3, .imux = &imux_configs[7], .imux_addr = 3},          // Stage 31

		{.size = &kLoadStageParamsBySize[SIZE_0R1], .io_port = FET_EN32_GPIO_Port, .io_pin = FET_EN32_Pin, .tmux_ID = 8, .tmux_addr = 0, .imux = &imux_configs[8], .imux_addr = 0},          // Stage 32
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN33_GPIO_Port, .io_pin = FET_EN33_Pin, .tmux_ID = 8, .tmux_addr = 1, .imux = &imux_configs[9], .imux_addr = 0},          // Stage 33
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN34_GPIO_Port, .io_pin = FET_EN34_Pin, .tmux_ID = 8, .tmux_addr = 2, .imux = &imux_configs[8], .imux_addr = 1},          // Stage 34
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN35_GPIO_Port, .io_pin = FET_EN35_Pin, .tmux_ID = 8, .tmux_addr = 3, .imux = &imux_configs[9], .imux_addr = 1},          // Stage 35
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN36_GPIO_Port, .io_pin = FET_EN36_Pin, .tmux_ID = 9, .tmux_addr = 0, .imux = &imux_configs[8], .imux_addr = 2},          // Stage 36
		{.size = &kLoadStageParamsBySize[SIZE_1R], .io_port = FET_EN37_GPIO_Port, .io_pin = FET_EN37_Pin, .tmux_ID = 9, .tmux_addr = 1, .imux = &imux_configs[9], .imux_addr = 2},          // Stage 37
		{.size = &kLoadStageParamsBySize[SIZE_0R1], .io_port = FET_EN38_GPIO_Port, .io_pin = FET_EN38_Pin, .tmux_ID = 9, .tmux_addr = 2, .imux = &imux_configs[8], .imux_addr = 3},          // Stage 38
		{.size = &kLoadStageParamsBySize[SIZE_0R1], .io_port = FET_EN39_GPIO_Port, .io_pin = FET_EN39_Pin, .tmux_ID = 9, .tmux_addr = 3, .imux = &imux_configs[9], .imux_addr = 3},          // Stage 39

		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN40_GPIO_Port, .io_pin = FET_EN40_Pin, .tmux_ID = 10, .tmux_addr = 0, .imux = &imux_configs[10], .imux_addr = 0},          // Stage 40
		{.size = &kLoadStageParamsBySize[SIZE_0R1], .io_port = FET_EN41_GPIO_Port, .io_pin = FET_EN41_Pin, .tmux_ID = 10, .tmux_addr = 1, .imux = &imux_configs[11], .imux_addr = 0},          // Stage 41
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN42_GPIO_Port, .io_pin = FET_EN42_Pin, .tmux_ID = 10, .tmux_addr = 2, .imux = &imux_configs[10], .imux_addr = 1},          // Stage 42
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN43_GPIO_Port, .io_pin = FET_EN43_Pin, .tmux_ID = 10, .tmux_addr = 3, .imux = &imux_configs[11], .imux_addr = 1},          // Stage 43
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN44_GPIO_Port, .io_pin = FET_EN44_Pin, .tmux_ID = 11, .tmux_addr = 0, .imux = &imux_configs[10], .imux_addr = 2},          // Stage 44
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN45_GPIO_Port, .io_pin = FET_EN45_Pin, .tmux_ID = 11, .tmux_addr = 1, .imux = &imux_configs[11], .imux_addr = 2},          // Stage 45
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN46_GPIO_Port, .io_pin = FET_EN46_Pin, .tmux_ID = 11, .tmux_addr = 2, .imux = &imux_configs[10], .imux_addr = 3},          // Stage 46
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN47_GPIO_Port, .io_pin = FET_EN47_Pin, .tmux_ID = 11, .tmux_addr = 3, .imux = &imux_configs[11], .imux_addr = 3},          // Stage 47

		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN48_GPIO_Port, .io_pin = FET_EN48_Pin, .tmux_ID = 12, .tmux_addr = 0, .imux = &imux_configs[12], .imux_addr = 0},          // Stage 48
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN49_GPIO_Port, .io_pin = FET_EN49_Pin, .tmux_ID = 12, .tmux_addr = 1, .imux = &imux_configs[13], .imux_addr = 0},          // Stage 49
		{.size = &kLoadStageParamsBySize[SIZE_0R1], .io_port = FET_EN50_GPIO_Port, .io_pin = FET_EN50_Pin, .tmux_ID = 12, .tmux_addr = 2, .imux = &imux_configs[12], .imux_addr = 1},          // Stage 50
		{.size = &kLoadStageParamsBySize[SIZE_0R1], .io_port = FET_EN51_GPIO_Port, .io_pin = FET_EN51_Pin, .tmux_ID = 12, .tmux_addr = 3, .imux = &imux_configs[13], .imux_addr = 1},          // Stage 51
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN52_GPIO_Port, .io_pin = FET_EN52_Pin, .tmux_ID = 13, .tmux_addr = 0, .imux = &imux_configs[12], .imux_addr = 2},          // Stage 52
		{.size = &kLoadStageParamsBySize[SIZE_0R1], .io_port = FET_EN53_GPIO_Port, .io_pin = FET_EN53_Pin, .tmux_ID = 13, .tmux_addr = 1, .imux = &imux_configs[13], .imux_addr = 2},          // Stage 53
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN54_GPIO_Port, .io_pin = FET_EN54_Pin, .tmux_ID = 13, .tmux_addr = 2, .imux = &imux_configs[12], .imux_addr = 3},          // Stage 54
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN55_GPIO_Port, .io_pin = FET_EN55_Pin, .tmux_ID = 13, .tmux_addr = 3, .imux = &imux_configs[13], .imux_addr = 3},          // Stage 55

		{.size = &kLoadStageParamsBySize[SIZE_0R1], .io_port = FET_EN56_GPIO_Port, .io_pin = FET_EN56_Pin, .tmux_ID = 14, .tmux_addr = 0, .imux = &imux_configs[14], .imux_addr = 0},          // Stage 56
		{.size = &kLoadStageParamsBySize[SIZE_0R1], .io_port = FET_EN57_GPIO_Port, .io_pin = FET_EN57_Pin, .tmux_ID = 14, .tmux_addr = 1, .imux = &imux_configs[15], .imux_addr = 0},          // Stage 57
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN58_GPIO_Port, .io_pin = FET_EN58_Pin, .tmux_ID = 14, .tmux_addr = 2, .imux = &imux_configs[14], .imux_addr = 1},          // Stage 58
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN59_GPIO_Port, .io_pin = FET_EN59_Pin, .tmux_ID = 14, .tmux_addr = 3, .imux = &imux_configs[15], .imux_addr = 1},          // Stage 59
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN60_GPIO_Port, .io_pin = FET_EN60_Pin, .tmux_ID = 15, .tmux_addr = 0, .imux = &imux_configs[14], .imux_addr = 2},          // Stage 60
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN61_GPIO_Port, .io_pin = FET_EN61_Pin, .tmux_ID = 15, .tmux_addr = 1, .imux = &imux_configs[15], .imux_addr = 2},          // Stage 61
		{.size = &kLoadStageParamsBySize[SIZE_0R5], .io_port = FET_EN62_GPIO_Port, .io_pin = FET_EN62_Pin, .tmux_ID = 15, .tmux_addr = 2, .imux = &imux_configs[14], .imux_addr = 3},          // Stage 62
		{.size = &kLoadStageParamsBySize[SIZE_0R1], .io_port = FET_EN63_GPIO_Port, .io_pin = FET_EN63_Pin, .tmux_ID = 15, .tmux_addr = 3, .imux = &imux_configs[15], .imux_addr = 3},          // Stage 63
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

uint32_t GetVSenseLatest(void){

	// TODO: This function does not take into account a startup scenario where the head is at 0 because nothing has been recorded yet.
	// This might not be an issue though since no one will press enable button in less than 1ms after startup before a voltage has been read.

	int location = system_data.voltage_mV_buf_head - 1;
	if (location < 0){
		location = system_data.voltage_mV_buf_size - 1;
	}
	return system_data.voltage_mV_buf[location];
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
