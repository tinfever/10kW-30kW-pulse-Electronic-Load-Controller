/*
 * state.h
 *
 *  Created on: Aug 5, 2023
 *      Author: user
 */

#ifndef INC_STATE_H_
#define INC_STATE_H_

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

#define NUM_STAGES 64

typedef enum {
	SIZE_8R = 0,
	SIZE_4R,
	SIZE_2R,
	SIZE_1R,
	SIZE_0R5,
	SIZE_0R1,
	SIZE_NULL,
	kNumberOfStageSizes = 7,
} LoadStageSize;

typedef struct {
	const uint32_t load_mOhm;
	const uint32_t mA_per_V;
	const uint32_t watt_rating;
	const uint32_t shunt_uOhms;
} LoadStageParameters;

typedef struct {
	const uint32_t channel_num;
	const uint32_t ADC_num;
	 ADC_TypeDef* const ADC_instance;
} ImuxConfig;

typedef struct {
	const LoadStageParameters *size;
	GPIO_TypeDef *io_port;
	uint16_t io_pin;
	uint32_t tmux_ID;
	uint32_t tmux_addr;
	const ImuxConfig * imux;
	uint32_t imux_addr;
} LoadStageConfiguration;

typedef struct {
	uint32_t calibrated_mA_per_V;
	uint32_t current_mA;
	uint32_t thermistor_temp_mC;
	int32_t thermocouple_temp_mC;
} LoadStageData;

typedef enum {
	kInvalidMode = -1,
	kConstantMode = 0,
	kPulsedMode,
	kSineWaveMode,
	kNumModes,
} LoadMode;

//each bit represents the on/off state of a specific stage. Bit 0 = stage 0, bit 5 = stage 5, etc.
//64 bit accommodates up to 64 stages
typedef uint64_t LoadStageCombo;

void LoadStageInit(void);
const LoadStageConfiguration* GetPointerToSingleStageConfig(uint32_t stage_num);
bool GetSystemEnabled(void);
void SetSystemEnabled(bool val);
uint32_t Get_Constant_ISet(void);
void Set_Constant_ISet(int32_t set);
LoadMode GetMode(void);
void SetMode(LoadMode set);
void RecordSystemCurrentMeasurement(uint32_t current_mA);
void RecordSystemVoltageMeasurement(uint32_t voltage_mV);
uint32_t GetVSenseAverage(void);
uint32_t GetISenseAverage(void);
void SetStageCalibratedConductance(uint32_t stage_num, uint32_t conductance);
uint32_t GetStageCalibratedConductance(uint32_t stage_num);
uint32_t GetStageNominalConductance(uint32_t stage_num);
uint32_t GetStageNominalLoadResistance(uint32_t stage_num);
void SetStageTemps(uint32_t stage_num, int32_t tc_temp, int32_t thermistor_temp);
bool isStagePresent(uint32_t stage_num);
uint32_t GetMaxThermocoupleTempStageNum(void);
int32_t GetStageThermocoupleTemp(uint32_t stage_num);
uint32_t GetVSenseLatest(void);
uint32_t GetPulsedIHigh(void);
void SetPulsedIHigh(int32_t set);
uint32_t GetPulsedILow(void);
void SetPulsedILow(int32_t set);
uint32_t GetPulsedFreq(void);
void SetPulsedFreq(int32_t set);
uint32_t GetPulsedDutyCycle(void);
void SetPulsedDutyCycle(int32_t set);


#endif /* INC_STATE_H_ */

