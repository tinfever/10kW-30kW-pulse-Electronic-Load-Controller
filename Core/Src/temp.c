/*
 * temp.c
 *
 *  Created on: Aug 31, 2023
 *      Author: user
 */

#include <stdint.h>
#include "main.h"
#include "state.h"

typedef struct {
	int32_t tc_temp;
	int32_t thermistor_temp;
} TempResults;

typedef enum {
	TEMP_OK,
	TEMP_LOW,
	TEMP_HIGH,
} TempConversionExitStatus;

static int32_t thermistorADCtoTemp(uint32_t adc_reading);
static int32_t thermocoupleADCtoVoltage(uint32_t adc_reading);
static int32_t thermocoupleTempToVoltage(int32_t temp);
static int32_t thermocoupleVoltagetoTemp(int32_t input_uV);
static TempConversionExitStatus ReadSingleStageTemp(uint32_t stage_num, TempResults * temp_results);

typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin;
} TmuxInhibitPin;

#define NUM_TMUX 16

// This can be used like the following to write to specific TMUX ID numbers:
// HAL_GPIO_WritePin(tmux_inh_pins[1].port, tmux_inh_pins[1].pin, 1);
static const TmuxInhibitPin tmux_inh_pins[NUM_TMUX] = {
		{.port = TMUX_INH0_GPIO_Port, .pin = TMUX_INH0_Pin},
		{.port = TMUX_INH1_GPIO_Port, .pin = TMUX_INH1_Pin},
		{.port = TMUX_INH2_GPIO_Port, .pin = TMUX_INH2_Pin},
		{.port = TMUX_INH3_GPIO_Port, .pin = TMUX_INH3_Pin},
		{.port = TMUX_INH4_GPIO_Port, .pin = TMUX_INH4_Pin},
		{.port = TMUX_INH5_GPIO_Port, .pin = TMUX_INH5_Pin},
		{.port = TMUX_INH6_GPIO_Port, .pin = TMUX_INH6_Pin},
		{.port = TMUX_INH7_GPIO_Port, .pin = TMUX_INH7_Pin},

		{.port = TMUX_INH8_GPIO_Port, .pin = TMUX_INH8_Pin},
		{.port = TMUX_INH9_GPIO_Port, .pin = TMUX_INH9_Pin},
		{.port = TMUX_INH10_GPIO_Port, .pin = TMUX_INH10_Pin},
		{.port = TMUX_INH11_GPIO_Port, .pin = TMUX_INH11_Pin},
		{.port = TMUX_INH12_GPIO_Port, .pin = TMUX_INH12_Pin},
		{.port = TMUX_INH13_GPIO_Port, .pin = TMUX_INH13_Pin},
		{.port = TMUX_INH14_GPIO_Port, .pin = TMUX_INH14_Pin},
		{.port = TMUX_INH15_GPIO_Port, .pin = TMUX_INH15_Pin},
};




void UpdateLoadStageTemps(void){
	for (int i = 0; i < NUM_STAGES; i++){
		if (isStagePresent(i)){
			TempResults temp_results;
			TempConversionExitStatus status = ReadSingleStageTemp(i, &temp_results);
			switch(status){
			case TEMP_OK:
				SetStageTemps(i, temp_results.tc_temp, temp_results.thermistor_temp);
				break;
			case TEMP_LOW:
				SetStageTemps(i, -999999, -999999);
				break;
			case TEMP_HIGH:
				SetStageTemps(i, 999999, 999999);
				break;

			}
		}

	}
}

// returns stage thermocouple temperature in celsius
static TempConversionExitStatus ReadSingleStageTemp(uint32_t stage_num, TempResults * temp_results){

	const LoadStageConfiguration *stage = GetPointerToSingleStageConfig(stage_num);

	//inhibit all tmux's to ensure two tmux's aren't momentarily shorted together
	for (int i = 0; i < NUM_TMUX; i++){
		HAL_GPIO_WritePin(tmux_inh_pins[i].port, tmux_inh_pins[i].pin, 1);
	}

	//set address pins
	uint32_t tmux_addr = stage->tmux_addr;
	uint32_t tmux_s0 = tmux_addr & 1;
	uint32_t tmux_s1 = (tmux_addr >> 1) & 1;
	HAL_GPIO_WritePin(TMUX_S0_GPIO_Port, TMUX_S0_Pin, tmux_s0);
	HAL_GPIO_WritePin(TMUX_S1_GPIO_Port, TMUX_S1_Pin, tmux_s1);

	//uninhibit the TMUX we want
	uint32_t tmux_number = stage->tmux_ID;
	HAL_GPIO_WritePin(tmux_inh_pins[tmux_number].port, tmux_inh_pins[tmux_number].pin, 0);

	//read thermistor and thermocouple signal simultaneously
	//configure ADC
	ADC1->SR = ~(ADC_SR_EOC | ADC_SR_STRT);
	ADC1->CR2 &= ~(ADC_CR2_EXTEN_Msk | ADC_CR2_EXTSEL_Msk);	// Clear bits to leave in default SW trigger mode

	//clear sequence registers before configuration
	ADC1->SQR1 = 0;
	ADC1->SQR2 = 0;
	ADC1->SQR3 = 0;
	ADC2->SQR1 = 0;
	ADC2->SQR2 = 0;
	ADC2->SQR3 = 0;

	#define NUM_TMUX_ADC_READS 1
	//configure channels ADC1 = Thermocouple mux, ADC2 = Thermistor mux
	ADC1->SQR3 |= ADC_TC_MUX << ADC_SQR3_SQ1_Pos;
	ADC1->SQR1 |= (NUM_TMUX_ADC_READS - 1) << ADC_SQR1_L_Pos;


	ADC2->SQR3 |= ADC_THERMISTOR_MUX << ADC_SQR3_SQ1_Pos;
	ADC2->SQR1 |= (NUM_TMUX_ADC_READS - 1) << ADC_SQR1_L_Pos;

	//ADC3 unused
	ADC3->SQR3 |= ADC3_NULL << ADC_SQR3_SQ1_Pos;
	ADC3->SQR1 |= (NUM_TMUX_ADC_READS - 1) << ADC_SQR1_L_Pos;

//
//	typedef struct {
//		uint16_t thermocouple_adc;
//		uint16_t thermistor_adc;
//	} CombinedResults;
//
//	volatile CombinedResults results = {0};



	//send SW start to ADC
	ADC1->CR2 |= ADC_CR2_SWSTART;

	//wait for adc read to be done
	while (!(ADC1->SR & ADC_SR_EOC));

	uint32_t thermocouple_adc = ADC1->DR;
	uint32_t thermistor_adc = ADC2->DR;

	int32_t thermistor_temp = thermistorADCtoTemp(thermistor_adc);

	//check for out of bounds thermistor reading
	if (thermistor_temp == 999999){
		return TEMP_HIGH;
	}
	else if (thermistor_temp == -999999){
		return TEMP_LOW;
	}

	int32_t thermistor_uV_equiv = thermocoupleTempToVoltage(thermistor_temp);

	//check for out of bounds temp conversion
	if (thermistor_uV_equiv == 999999){
		return TEMP_HIGH;
	}
	else if (thermistor_uV_equiv == -999999){
		return TEMP_LOW;
	}

	int32_t thermocouple_uV = thermocoupleADCtoVoltage(thermocouple_adc);

	int32_t sum_uV = thermocouple_uV + thermistor_uV_equiv;

	int32_t thermocouple_temp = thermocoupleVoltagetoTemp(sum_uV);

	//check for out of bounds temp conversion
	if (thermocouple_temp == 999999){
		return TEMP_HIGH;
	}
	else if (thermocouple_temp == -999999){
		return TEMP_LOW;
	}


	temp_results->tc_temp = thermocouple_temp;
	temp_results->thermistor_temp = thermistor_temp;
	return TEMP_OK;

	#undef NUM_TMUX_ADC_READS
}

// Thermistor PN: NCU18XH103F60RB
//  Opamp Gain: 3
// V_in: 3.3V
// Linearization Resistor: 4.7k
// R_1 or pull-up resistor: 10k
// Total ADC counts: 4096
// ADC Vref: 3.0V
// Each value is the number of ADC counts corresponding to a specific temperature in C.
//  The temperature is determine by the array index - 2
// Example, thermistor_ADC_counts[0] = 3902, meaning at 0 - 2 = -2C, the ADC reading would be 3902 counts.
// Index 127 corresponds to 125C
static const uint32_t thermistor_array_size = 128;
static const uint16_t thermistor_ADC_counts[128] = {
		3902, 3885, 3867, 3850, 3831, 3813, 3794, 3774, 3754, 3733,
		3712, 3690, 3668, 3645, 3621, 3598, 3573, 3548, 3523, 3497,
		3471, 3444, 3417, 3390, 3362, 3333, 3304, 3275, 3245, 3215,
		3184, 3153, 3121, 3090, 3058, 3025, 2993, 2960, 2927, 2893,
		2859, 2826, 2792, 2757, 2723, 2688, 2654, 2619, 2584, 2549,
		2514, 2479, 2444, 2409, 2374, 2339, 2304, 2269, 2235, 2200,
		2166, 2131, 2097, 2064, 2031, 1998, 1965, 1932, 1900, 1868,
		1837, 1806, 1775, 1744, 1713, 1683, 1653, 1624, 1595, 1566,
		1538, 1510, 1482, 1455, 1428, 1402, 1375, 1350, 1324, 1300,
		1275, 1251, 1227, 1204, 1181, 1158, 1135, 1113, 1092, 1071,
		1050, 1029, 1009, 989, 970, 951, 933, 914, 896, 879,
		862, 845, 828, 812, 796, 780, 765, 750, 735, 721,
		707, 693, 679, 666, 653, 640, 628, 616};

// accepts number of ADC counts measured, returns value in Celsius * 1000
// valid return range -2000 to 125000
// error values 999999 (overtemp) or -999999 (undertemp)
static int32_t thermistorADCtoTemp(uint32_t adc_reading){

    // handle input outside of thermistor value table
    if (adc_reading < thermistor_ADC_counts[thermistor_array_size - 1])
    {
        return 999999;
    }
    else if (adc_reading > thermistor_ADC_counts[0])
    {
        return -999999;
    }

    // binary search
    int low = 0;
    int high = thermistor_array_size - 1;
    while (low <= high){
        int mid  = (high + low)/2;
        if (adc_reading == thermistor_ADC_counts[mid]){
            return 1000 * (mid - 2);
        }
        else if (adc_reading < thermistor_ADC_counts[mid]){
            low = mid + 1;
        }
        else{
            high = mid - 1;
        }
    }

    // linear interpolation between selected position and next position that adc_reading is between

    // get change in x
    int x_change = thermistor_ADC_counts[low] - thermistor_ADC_counts[high];    //-27

    // get change in y
    int y_change = low - high; //1

    // float slope = (float)y_change / x_change;              //-0.037
    int x_diff = adc_reading - thermistor_ADC_counts[low]; // 3000-2977 = 23
    // float y_diff = x_diff * slope;                         // 23 * -0.037 = -0.851
    int y_diff = 1000 * x_diff * y_change / x_change; // milliCelsius, calculating slope simultaneously lets us so multiply as int instead of float, at least when we were using floats //-851
    int y_result = y_diff + 1000 * (low - 2);         //-2 is to compensate for index value vs celsius value offset, order of operations for int calc instead of float  //-0.851 + 27 - 2 = 24.148
    return y_result;
}


//takes adc reading in counts
//returns thermocouple voltage in uV
//valid return 0 to 15000
static int32_t thermocoupleADCtoVoltage(uint32_t adc_reading){
    //simplified to allow 32 bit calculation
    //equivalent to:
    // thermocouple_uV = (adc_reading * 3000 * 1000) / (4096 * 200); 3000 = 3Vref, 1000 for mV to uV conversion, 4096 = 12bit adc, 200 = thermocouple gain
    int32_t thermocouple_uV = adc_reading * 3 * 5000 / 4096;

    return thermocouple_uV;
}

//array index value is temperature in C
//array contents is type k thermocouple voltage at temp C in uV
//all is relative to assumed 0C cold junction
static const uint32_t type_k_thermocouple_size = 300;
static const uint16_t type_k_thermocouple[300] = {
    0, 39, 79, 119, 158, 198, 238, 277, 317, 357,
    397, 437, 477, 517, 557, 597, 637, 677, 718, 758,
    798, 838, 879, 919, 960, 1000, 1041, 1081, 1122, 1163,
    1203, 1244, 1285, 1326, 1366, 1407, 1448, 1489, 1530, 1571,
    1612, 1653, 1694, 1735, 1776, 1817, 1858, 1899, 1941, 1982,
    2023, 2064, 2106, 2147, 2188, 2230, 2271, 2312, 2354, 2395,
    2436, 2478, 2519, 2561, 2602, 2644, 2685, 2727, 2768, 2810,
    2851, 2893, 2934, 2976, 3017, 3059, 3100, 3142, 3184, 3225,
    3267, 3308, 3350, 3391, 3433, 3474, 3516, 3557, 3599, 3640,
    3682, 3723, 3765, 3806, 3848, 3889, 3931, 3972, 4013, 4055,
    4096, 4138, 4179, 4220, 4262, 4303, 4344, 4385, 4427, 4468,
    4509, 4550, 4591, 4633, 4674, 4715, 4756, 4797, 4838, 4879,
    4920, 4961, 5002, 5043, 5084, 5124, 5165, 5206, 5247, 5288,
    5328, 5369, 5410, 5450, 5491, 5532, 5572, 5613, 5653, 5694,
    5735, 5775, 5815, 5856, 5896, 5937, 5977, 6017, 6058, 6098,
    6138, 6179, 6219, 6259, 6299, 6339, 6380, 6420, 6460, 6500,
    6540, 6580, 6620, 6660, 6701, 6741, 6781, 6821, 6861, 6901,
    6941, 6981, 7021, 7060, 7100, 7140, 7180, 7220, 7260, 7300,
    7340, 7380, 7420, 7460, 7500, 7540, 7579, 7619, 7659, 7699,
    7739, 7779, 7819, 7859, 7899, 7939, 7979, 8019, 8059, 8099,
    8138, 8178, 8218, 8258, 8298, 8338, 8378, 8418, 8458, 8499,
    8539, 8579, 8619, 8659, 8699, 8739, 8779, 8819, 8860, 8900,
    8940, 8980, 9020, 9061, 9101, 9141, 9181, 9222, 9262, 9302,
    9343, 9383, 9423, 9464, 9504, 9545, 9585, 9626, 9666, 9707,
    9747, 9788, 9828, 9869, 9909, 9950, 9991, 10031, 10072, 10113,
    10153, 10194, 10235, 10276, 10316, 10357, 10398, 10439, 10480, 10520,
    10561, 10602, 10643, 10684, 10725, 10766, 10807, 10848, 10889, 10930,
    10971, 11012, 11053, 11094, 11135, 11176, 11217, 11259, 11300, 11341,
    11382, 11423, 11465, 11506, 11547, 11588, 11630, 11671, 11712, 11753,
    11795, 11836, 11877, 11919, 11960, 12001, 12043, 12084, 12126, 12167
    };

//takes temperature in Celsius * 1000 (millicelsius) (valid range 0 - 299000) (0 to 299C)
//returns respective voltage in uV (valid range 0-12167)
static int32_t thermocoupleTempToVoltage(int32_t temp){
	//bounds check
	if (temp < 0){
		return -999999;
	}
	else if (temp > 299000){
		return 999999;
	}
	else if (temp == 299000){	//special handling for max value, otherwise would cause out of bounds access of type_k_thermocouple array in "type_k_thermocouple[high/1000]"
		return type_k_thermocouple[299];
	}


    int low = (temp / 1000) * 1000;
    int high = low + 1000;

    //linear interpolation
    // get change in x
    int x_change = high - low;   //1000

    // get change in y
    int y_change = type_k_thermocouple[high/1000] - type_k_thermocouple[low/1000];    //change in uV

    // float slope = (float)y_change / x_change;              //-0.037
    int x_diff = temp - low;
    // float y_diff = x_diff * slope;                         // 23 * -0.037 = -0.851
    int y_diff = x_diff * y_change / x_change; // milliCelsius, calculating slope simultaneously lets us so multiply as int instead of float, at least when we were using floats //-24
    int y_result = y_diff + type_k_thermocouple[low/1000];         //-.024 + 295
    return y_result;    //294.976

}

// takes input voltage in uV
// returns temperature in Celsius * 1000 (millicelsius)
// valid input 0 - 12167
static int32_t thermocoupleVoltagetoTemp(int32_t input_uV){
    // handle input outside of array bounds
    if (input_uV > type_k_thermocouple[type_k_thermocouple_size - 1])
    {
        return 999999;
    }
    else if (input_uV < type_k_thermocouple[0])
    {
        return -999999;
    }

    // binary search
    int mid = -1;
    int low = 0;
    int high = type_k_thermocouple_size - 1;
    while (low <= high){
        mid  = (high + low)/2;
        if (input_uV == type_k_thermocouple[mid]){
            return mid * 1000;
        }
        else if (input_uV < type_k_thermocouple[mid]){
            high = mid - 1;
        }
        else{
            low = mid + 1;
        }
    }

    //linear interpolation
    // get change in x
    int x_change = type_k_thermocouple[low] - type_k_thermocouple[high];    //41

    // get change in y
    int y_change = low - high; //1

    int x_diff = input_uV - type_k_thermocouple[low]; // 12000-12001 = -1
    int y_diff = 1000 * x_diff * y_change / x_change; // milliCelsius, calculating slope simultaneously lets us so multiply as int instead of float, at least when we were using floats //-24
    int y_result = y_diff + 1000 * low;         //-.024 + 295
    return y_result;    //294.976
}

void FanSpeedControl(void){
	int32_t max_temp_stage = GetMaxThermocoupleTempStageNum();
	int32_t temp = GetStageThermocoupleTemp(max_temp_stage) / 1000;		// Divide by 1000 to convert to just C
	int32_t pwm_compare_val = (14 * temp - 400) - 1;		//PWM val from 0 - 999 for 0-100%; 	30% at 50C and below, 100% at 100C and above
	if (pwm_compare_val < 300){
		pwm_compare_val = 300;	// Don't go below 30%
	}
	else if (pwm_compare_val > 999){
		pwm_compare_val = 999;
	}


	__HAL_TIM_SET_COMPARE(&FAN_PWM_TIM, TIM_CHANNEL_4, pwm_compare_val);
}
