/*
 * display.c
 *
 *  Created on: Aug 5, 2023
 *      Author: user
 */

//Max char per line at 160px screen width
//Font8		160/5 = 32
//Font12 	160/7 = 22 char wide, 9 line high after buttons
//Font16	160/11 = 14 char wide, 7 lines high after buttons
//Font20	160/14 = 11
//Font24	160/17 = 9;



#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "display.h"



#include "state.h"
#include "main.h"

// TODO: I think perhaps this should all be refactored to be more modular?
// We have several different parts:
// Physical button reading and debouncing handled during SysTick ISR
// Button interpretation and conversion to a specific event, depending on the configuration of the button, currently handled by callback functions in the button structs that are reconfigured on the fly
// (sort of) However, LeftButtonEvent, for example, still results in a different action depending on if the specific menu item is selected or not
// Also, each setting field has its own handling for rendering if it is selected
// Thinking perhaps there are different "focus levels" like overall settings screen, and the specific settings can be focused.
// Then any events should be sent to the currently focused object?
// But there has to be a distinction between focused and selected? On main settings screen, focus only impacts highlighting of field, and action taken when select button (rotary encoder sw) is pressed.

typedef enum {
	kInvalidFieldName = -1,
	kSetConstantCurrent,
	kSetMode,
	kSetPulsedIHigh,
	kNumFieldNames,
} FieldName;

typedef struct {
	int32_t cursor_pos;
	uint32_t field_length;
	int32_t invalid_cursor_pos;		//position to skip over with cursor. intended for decimal place. -1 means not applicable
	bool field_visible;
	uint32_t cursor_value_by_pos[10];
} FieldData;

typedef struct {
	FieldName focused_field;	//which field is highlighted
	bool is_selected;	//is field selected
} MenuState;

typedef struct {
	uint32_t x_size;
	uint32_t x_pos;
	char* text;
	sFONT* font;
	void (*action)(void);
} Button;

typedef struct {
	char* label;
	void (*DrawSetting[10])(void);
} ModeParams;

void DisableEvent(void);
void EnableEvent(void);
void LeftButtonEvent(void);
void RightButtonEvent(void);
void CursorLeftArrow(FieldData *selected_field_data);
void CursorRightArrow(FieldData *selected_field_data);
void CursorClampPosition(FieldData *selected_field_data);
void MoveCursor(FieldData *selected_field_data, int dir);
int uint32ToDecimalString(char* buffer, int buf_length, uint32_t number, int precision,
		int space_padding, int zero_padding);
void Draw_Constant_ISet(void);
void Draw_Pulsed_IHigh(void);
void Draw_Pulsed_ILow(void);
void Draw_Pulsed_Freq(void);
void Draw_Pulsed_DutyCycle(void);
void DrawDynamicField(FieldName field, int xpos, int ypos, char* text);
int32_t VelocityFactorSettingsAdjustment(uint32_t current_val, int32_t encoder_delta);



FieldData field_data[kNumFieldNames] = {
		{.cursor_pos = 2, .field_length = 4, .invalid_cursor_pos = 3, .field_visible = false, .cursor_value_by_pos = {1000, 100, 10, 0, 1}},	//SetConstantCurrent
		{.cursor_pos = 0, .field_length = 0, .invalid_cursor_pos = -1, .field_visible = false},	//SetMode, no specific cursor needed
		{.cursor_pos = 3, .field_length = 5, .invalid_cursor_pos = 4, .field_visible = false, .cursor_value_by_pos = {10000, 1000, 100, 10, 0, 1}}, // kSetPulsedIHigh
};

//initialize to default settings
static MenuState menu_state = {.focused_field = kSetConstantCurrent, .is_selected = false};

static struct {
	bool sw1_press;
	bool sw2_press;
	bool sw3_press;
	bool enc_press;
	uint32_t encoder_pos;	//currently unused
} input_data;



// Store the setting string for each mode, and
ModeParams modes[kNumModes] = {
		{.label = "Constant Current", .DrawSetting = {Draw_Constant_ISet}},	// Constant Mode
		{.label = "Pulsed          ", .DrawSetting = {Draw_Pulsed_IHigh, Draw_Pulsed_ILow, Draw_Pulsed_Freq, Draw_Pulsed_DutyCycle}}, // Pulsed Mode
		{.label = "Sine Wave       "},	// Sinewave Mode
};

//typedef enum {
//	kInvalidMode = -1,
//	kConstantMode = 0,
//	kPulsedMode,
//	kSineWaveMode,
//	kNumModes,
//} LoadMode;



void RegisterButtonPress(uint32_t button){
	switch(button){
		case 1:
			input_data.sw1_press = true;
			break;
		case 2:
			input_data.sw2_press = true;
			break;
		case 3:
			input_data.sw3_press = true;
			break;
		case 4:
			input_data.enc_press = true;
			break;
	}
}



static Button button1 = {.x_size = 52, .x_pos = 0, .text = "ENABLE ", .font=&Font12, .action = EnableEvent};
static Button button2 = {.x_size = 52, .x_pos = 54, .text = "   <   ", .font=&Font12, .action = LeftButtonEvent};
static Button button3 = {.x_size = 52, .x_pos = 108, .text = "   >   ", .font=&Font12, .action = RightButtonEvent};


void DrawButton(Button* button){
	int y_size = button->font->Height + 3;	//+ 2 for the rectangle line width size, + 1 for spacing between top line and text
	int y_pos = 127-(y_size-1);
	BSP_LCD_DrawRect(button->x_pos, y_pos, button->x_size, y_size);

	BSP_LCD_SetFont(button->font);
	int strY = 127 - button->font->Height;
	int strX = button->x_pos + 2;
	BSP_LCD_DisplayStringAt(strX, strY, (uint8_t*)button->text, LEFT_MODE);
}

void EnableEvent(void){
	SetSystemEnabled(true);

	//This is just for testing
	static bool is_calibrated = false;
	if (!is_calibrated && GetVSenseAverage() > 2000){	//Make sure voltage is reasonable
		CalibrateAllStages();
		is_calibrated = true;
	}

	button1.text = "DISABLE";
	button1.action = DisableEvent;
}

void DisableEvent(void){
	SetSystemEnabled(false);
	button1.text = "ENABLE ";
	button1.action = EnableEvent;
}

void LeftButtonEvent(void){
	FieldData *selected_field_data = &field_data[menu_state.focused_field];
	if (menu_state.is_selected){
		MoveCursor(selected_field_data, -1);
		CursorClampPosition(selected_field_data);
	}
	else {
		menu_state.focused_field--;

		//check validity
		if (menu_state.focused_field >= kNumFieldNames) {
			menu_state.focused_field = 0;
		}
		if (menu_state.focused_field < 0) {
			menu_state.focused_field = kNumFieldNames-1;
		}
	}
}

void RightButtonEvent(void){
	FieldData *selected_field_data = &field_data[menu_state.focused_field];
	if (menu_state.is_selected){
		MoveCursor(selected_field_data, 1);
		CursorClampPosition(selected_field_data);
	}
	else {
		menu_state.focused_field++;

		//check validity
		if (menu_state.focused_field >= kNumFieldNames) {
			menu_state.focused_field = 0;
		}
		if (menu_state.focused_field < 0) {
			menu_state.focused_field = kNumFieldNames-1;
		}
	}
}


//Only run after TIM4 for rotary encoder is setup
void DisplayInit(void){
	  BSP_LCD_Init();
	  BSP_LCD_Clear(LCD_COLOR_BLACK);
	  ROTARY_ENCODER_TIM.Instance->SR &= !TIM_SR_UIF;	//Clear under/overflow flag for detection on rotary encoder timer
}


void MoveCursor(FieldData *selected_field_data, int dir){
	//clamp dir to +1 or -1
	if (dir < -1){
		dir = -1;
	}
	else if (dir > 1){
		dir = 1;
	}

    selected_field_data->cursor_pos += dir;

    // Skip invalid cursor positions if necessary
    if (selected_field_data->cursor_pos == selected_field_data->invalid_cursor_pos) {
    	selected_field_data->cursor_pos += dir;
    }
}

void CursorClampPosition(FieldData *selected_field_data){

	if (selected_field_data->cursor_pos < 0){
		selected_field_data->cursor_pos = selected_field_data->field_length;		//going left too far wraps around to right position
		}

	if (selected_field_data->cursor_pos > selected_field_data->field_length){
		selected_field_data->cursor_pos = 0;		//going right too far wraps around to left position
	}
}

void HandleRotaryEncoder(void){
	static uint32_t old_count = 0;
	uint32_t new_count = __HAL_TIM_GET_COUNTER(&ROTARY_ENCODER_TIM);

	//figure out delta enc counts
	int32_t delta_count = new_count - old_count;


	//correct for overflow/underflow by checking UIF flag
	//There is a small chance of the follow happening:
	//Counter could be at 0
	//User could move knob left (counter = -1) and underflow flag is set
	//user quickly moves knob right again by one places (counter = 0), all before next input handler call
	//Due to presence of underflow flag, and counter less than 0x7FFF, a correct delta of 0 would be interpreted as a delta of 65535.

	bool overflow_detected = ROTARY_ENCODER_TIM.Instance->SR & TIM_SR_UIF;
	if (overflow_detected && new_count > 0x7FFF){	//underflow
		delta_count -= 0x10000;		//Has to be the full 16-bit 0xFFFF + 1 due to weird zero index math
		ROTARY_ENCODER_TIM.Instance->SR &= ~TIM_SR_UIF;	//Clear the over/underflow flag
	}
	else if (overflow_detected && new_count < 0x7FFF){	//overflow
		delta_count += 0x10000;
		ROTARY_ENCODER_TIM.Instance->SR &= ~TIM_SR_UIF;
	}

	old_count = new_count;	//prepare for next time function is called

	//if setting is selected, apply delta as setting change
	if (menu_state.is_selected && delta_count){
		switch (menu_state.focused_field){

			case kSetConstantCurrent:
				uint32_t present_current_setting = Get_Constant_ISet();
				int32_t new_current_setting = VelocityFactorSettingsAdjustment(present_current_setting, delta_count);
				Set_Constant_ISet(new_current_setting);	//additional bounds checking here
				break;

			case kSetMode:
				LoadMode mode = GetMode();
				mode += delta_count;
				SetMode(mode);
				break;

			case kSetPulsedIHigh:
				uint32_t present_PulsedIHigh = GetPulsedIHigh();
				int32_t new_PulsedIHigh = VelocityFactorSettingsAdjustment(present_PulsedIHigh, delta_count);
				SetPulsedIHigh(new_PulsedIHigh);
				break;

			default:
				break;
		}
	}
	else if (!menu_state.is_selected && delta_count){
		//change the menu item selected
		menu_state.focused_field += delta_count;

		//check validity and roll over

		//loop through fields until we find one that is visible and in a valid range

		while (menu_state.focused_field < 0 || menu_state.focused_field > kNumFieldNames || field_data[menu_state.focused_field].field_visible == false){

			if (menu_state.focused_field >= kNumFieldNames) {
				menu_state.focused_field = 0;
			}

			if (menu_state.focused_field < 0) {
				menu_state.focused_field = kNumFieldNames-1;
			}

			if (field_data[menu_state.focused_field].field_visible == false){
				menu_state.focused_field += 1;
			}

		}


	}


}

int32_t VelocityFactorSettingsAdjustment(uint32_t current_val, int32_t encoder_delta){

	//use cursor position to determine amount of field to increment
	//uint32_t cursor_factor_by_position[5] = {1000, 100, 10, 0, 1};	//cursor at 0 means factor of 1000, cursor 3 is invalid, cursor 4 = 1x
	uint32_t cursor_position = field_data[menu_state.focused_field].cursor_pos;

	//add additional velocity control
	uint32_t velocity_factor = 1 << (abs(encoder_delta) / 2);

	int32_t new_setting = current_val + (encoder_delta * 100 * field_data[menu_state.focused_field].cursor_value_by_pos[cursor_position] * velocity_factor);	//multiply by 100 since display shows in 0.1A steps
	return new_setting;
}

void InputHandler(void){

	//handle SW1 being enable button
	if (input_data.sw1_press) {
		button1.action();
	}

	//handle rotary encoder button to toggle is_selected
	if (input_data.enc_press){
		menu_state.is_selected ^= 1;
	}

	if (input_data.sw2_press){
		button2.action();
	}

	if (input_data.sw3_press){
		button3.action();
	}

	//clear button presses now that all are handled
	input_data.sw1_press = false;
	input_data.sw2_press = false;
	input_data.sw3_press = false;
	input_data.enc_press = false;


	//handle rotary encoder changing set value of selected
	HandleRotaryEncoder();

}





void DrawLiveData(void){
	BSP_LCD_SetFont(&Font16);
	int fontwidth = BSP_LCD_GetFont()->Width;

	// Voltage
	uint32_t voltage_mV = GetVSenseAverage();
	char vtext[7];
	uint32ToDecimalString(vtext, 7, voltage_mV, 3, 5, 4);

	BSP_LCD_DisplayStringAt(fontwidth*6, LINE(0), (uint8_t*)vtext, LEFT_MODE);
	BSP_LCD_DisplayStringAt(fontwidth*13, LINE(0), (uint8_t*)"V", LEFT_MODE);


	// Current
	uint32_t current_mA = GetISenseAverage();
	char itext[8];
	uint32ToDecimalString(itext, 8, current_mA/10, 2, 6, 3);

	BSP_LCD_DisplayStringAt(fontwidth*5, LINE(1), (uint8_t*)itext, LEFT_MODE);
	BSP_LCD_DisplayStringAt(fontwidth*13, LINE(1), (uint8_t*)"A", LEFT_MODE);

	// Power
	uint32_t power_mW = (uint64_t) voltage_mV * current_mA / 1000;
	char ptext[9];
	uint32ToDecimalString(ptext, 9, power_mW/10, 2, 7, 3);
	BSP_LCD_DisplayStringAt(fontwidth*4, LINE(2), (uint8_t*)ptext, LEFT_MODE);
	BSP_LCD_DisplayStringAt(fontwidth*13, LINE(2), (uint8_t*)"W", LEFT_MODE);

	// On/Off state
	if (GetSystemEnabled() == true){
	  BSP_LCD_DisplayStringAt(0, LINE(0), (uint8_t*)"ON ", LEFT_MODE);
	}
	else {
	  BSP_LCD_DisplayStringAt(0, LINE(0), (uint8_t*)"OFF", LEFT_MODE);
	}


	BSP_LCD_SetFont(&Font12);
	fontwidth = BSP_LCD_GetFont()->Width;

	// Temperature of hottest stage, and its stage number
	uint32_t new_max_temp_stage = GetMaxThermocoupleTempStageNum();
	int32_t new_max_temp = GetStageThermocoupleTemp(new_max_temp_stage);

	static uint32_t last_max_temp_stage = 0;
	int32_t last_max_temp = GetStageThermocoupleTemp(last_max_temp_stage);

	// if new max temp is more than 0.5C different than last stage, use new max stage, otherwise use last max stage
	int32_t max_temp = 0;
	uint32_t max_temp_stage = 0;

	if (abs((int32_t)(new_max_temp - last_max_temp)) > 500){
		max_temp = new_max_temp / 100; // Use new max stage, convert from millicelsius to celsius in 0.1C increments
		max_temp_stage = new_max_temp_stage;
		last_max_temp_stage = new_max_temp_stage;
	}
	else{
		max_temp = last_max_temp / 100; // Use last max stage, convert from millicelsius to celsius in 0.1C increments
		max_temp_stage = last_max_temp_stage;
	}



	//int32_t max_temp = GetStageThermocoupleTemp(max_temp_stage) / 100; //convert from millicelsius to celsius in 0.1C increments
	char max_temp_text[6];
	uint32ToDecimalString(max_temp_text, 6, max_temp, 1, 0, 2);
	int max_temp_text_length = strlen(max_temp_text);

	char max_stage_text[3];
	snprintf(max_stage_text, 3, "%2lu", max_temp_stage);

	const int ypos_start_temp_info = 16;
	BSP_LCD_DisplayStringAt(0, ypos_start_temp_info, (uint8_t*)"MaxT:", LEFT_MODE);
	BSP_LCD_DisplayStringAt(0, ypos_start_temp_info + 12, (uint8_t*)max_temp_text, LEFT_MODE);
	BSP_LCD_DisplayStringAt(max_temp_text_length*fontwidth, ypos_start_temp_info+12, (uint8_t*)"C", LEFT_MODE);
	BSP_LCD_DisplayStringAt(0, ypos_start_temp_info + 24, (uint8_t*)"Stg", LEFT_MODE);
	BSP_LCD_DisplayStringAt(3*fontwidth, ypos_start_temp_info + 24, (uint8_t*)max_stage_text, LEFT_MODE);

	//if temp length is reduced ("123.5C" to "98.5C", clear text where the "C" used to be.
	static int prev_temp_length = 0;
	if (max_temp_text_length < prev_temp_length){
		int diff = prev_temp_length - max_temp_text_length;
		BSP_LCD_DisplayStringAt((max_temp_text_length + diff)*fontwidth, ypos_start_temp_info+12, (uint8_t*)"  ", LEFT_MODE);
	}
	prev_temp_length = max_temp_text_length;

}


void Draw_Constant_ISet(void) {
	int fontwidth = BSP_LCD_GetFont()->Width;
	int fontheight = BSP_LCD_GetFont()->Height;
	int ypos = 16 + fontheight*4;
	int xpos = 0 * fontwidth;

	BSP_LCD_DisplayStringAt(xpos, ypos, (uint8_t*) "I: ", LEFT_MODE);

	//get setting now
	uint32_t set_current = Get_Constant_ISet() / 100;//convert to increments of 100mA for display 123.4

	//format for printing
	char buffer[6];
	uint32ToDecimalString(buffer, 6, set_current, 1, 0, 4);

	xpos = 3 * fontwidth;
	DrawDynamicField(kSetConstantCurrent, xpos, ypos, buffer);

	xpos = 8 * fontwidth;
	BSP_LCD_DisplayStringAt(xpos, ypos, (uint8_t*) " A            ", LEFT_MODE);
}

void DrawDynamicField(FieldName field, int xpos, int ypos, char* text){
	int fontwidth = BSP_LCD_GetFont()->Width;
	int cursor_pos = field_data[field].cursor_pos;

	if (menu_state.focused_field == field) {
		//change to selected format for text
		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

		int length = strlen(text);
		for (int i = 0; i < length; i++) {
			//print char i with correct formatting
			if ((i == cursor_pos || field_data[field].field_length == 0) && menu_state.is_selected) {		//If field_length is zero, the whole field should be highlighted like the cursor
				BSP_LCD_SetBackColor(LCD_COLOR_GREEN);
				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
				BSP_LCD_DisplayChar(xpos, ypos, text[i]);
				BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			} else {
				BSP_LCD_DisplayChar(xpos, ypos, text[i]);
			}
			//increment xpos by correct value for next char
			xpos += fontwidth;
		}

		//revert colors
		BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	} else {
		//print normally with no cursor
		BSP_LCD_DisplayStringAt(xpos, ypos, (uint8_t*) text,
				LEFT_MODE);
	}

	//mark field as visible for cursor usage
	field_data[field].field_visible = true;
}

void Draw_Pulsed_IHigh(void){
		int fontwidth = BSP_LCD_GetFont()->Width;
		int fontheight = BSP_LCD_GetFont()->Height;
		int ypos = 16 + fontheight*4;
		int xpos = 0 * fontwidth;

		BSP_LCD_DisplayStringAt(xpos, ypos, (uint8_t*) "I High: ", LEFT_MODE);

		//get setting now
		uint32_t ihigh = GetPulsedIHigh() / 100;	//Convert to increments of 100mA	1234.5A

		char buffer[7];
		//format for printing
		uint32ToDecimalString(buffer, 7, ihigh, 1, 0, 5);

		xpos = 8 * fontwidth;
		DrawDynamicField(kSetPulsedIHigh, xpos, ypos, buffer);

		xpos = 14 * fontwidth;
		BSP_LCD_DisplayStringAt(xpos, ypos, (uint8_t*) " A      ", LEFT_MODE);



}

void Draw_Pulsed_ILow(void){

}

void Draw_Pulsed_Freq(void){

}

void Draw_Pulsed_DutyCycle(void){

}

void DrawModeField(void){
	int fontwidth = BSP_LCD_GetFont()->Width;
	int fontheight = BSP_LCD_GetFont()->Height;
	int ypos = 16 + fontheight*3;
	int xpos = 0 * fontwidth;

	BSP_LCD_DisplayStringAt(xpos, ypos, (uint8_t*)"Mode:", LEFT_MODE);

	//determine text to print
	LoadMode current_mode = GetMode();

	char* text = modes[current_mode].label;

	xpos = 6 * fontwidth;
	DrawDynamicField(kSetMode, xpos, ypos, text);

//	if (menu_state.focused_field == kSetMode && !menu_state.is_selected){
//		//print highlighted but without "cursor"
//		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
//		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
//		BSP_LCD_DisplayStringAt(6*fontwidth, ypos, (uint8_t*)text, LEFT_MODE);
//
//		//revert colors
//		BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
//		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
//	}
//	else if (menu_state.focused_field == kSetMode && menu_state.is_selected){
//		//print highlighted with cursor
//		BSP_LCD_SetBackColor(LCD_COLOR_GREEN);
//		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
//		BSP_LCD_DisplayStringAt(6*fontwidth, ypos, (uint8_t*)text, LEFT_MODE);
//
//		//revert colors
//		BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
//		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
//	} else {
//		//print normally
//		BSP_LCD_DisplayStringAt(6*fontwidth, ypos, (uint8_t*)text, LEFT_MODE);
//	}

}

void DrawSettings(void){
	BSP_LCD_SetFont(&Font12);

	// mark all fields as invisible for cursor usage.
	// Draw dynamic will automatically mark as visible again
	// Just need to add bounds check somewhere at the end I think?

	for (int i = 0; i < kNumFieldNames; i++){
		field_data[i].field_visible = false;
	}

	//Draw the actual mode selection field
	DrawModeField();

	//Draw associated settings for selected mode
	ModeParams *mode_params = &modes[GetMode()];
	int num_possible_draw_functions = sizeof(mode_params->DrawSetting) / sizeof(mode_params->DrawSetting[0]);
	int number_drawn = 0;
	for (int i = 0; i < num_possible_draw_functions; i++){
		if (mode_params->DrawSetting[i] != NULL){
			mode_params->DrawSetting[i]();
			number_drawn++;
		}
	}

	// Clear any lines that aren't used to draw fields, so text doesn't linger from previous options
	while(number_drawn < 4){
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_FillRect(0, 16 + 12*(4+number_drawn), 160, 12);
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		number_drawn++;
	}

	//TODO: add timeout for selection
	//TODO: consider implementing state machine for current field draw. states = none, highlighted, selected. None is black, highlighted is white, select is highlighted + green cursor
}

void DisplayUpdate(){
//	HAL_GPIO_WritePin(IO2_GPIO_Port, IO2_Pin, 1);
//	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);

	InputHandler();

	DrawLiveData();
	DrawSettings();

	DrawButton(&button1);
	DrawButton(&button2);
	DrawButton(&button3);


//	HAL_GPIO_WritePin(IO2_GPIO_Port, IO2_Pin, 0);
//	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
}

// buffer = location to store resulting string
// buf_length = size of buffer
// number = number to be converted
// precision = location of decimal place insertion. Ex. 25863 with precision 2 = 258.63
// space_padding = extra spaces will be added to left-hand size of resulting string so that result is 'padding' characters long (not including decimal place), if there aren't enough numbers as determined by input number and precision. Ex. input 5, space_padding 8, precision 3, zero_padding 5 = '   00.005'
// zero_padding = zeros will be added to left of number so that result has at least 'zero_padding' number of digits. Ex.
int uint32ToDecimalString(char* buffer, int buf_length, uint32_t number, int precision,
		int space_padding, int zero_padding){
	int status = snprintf(buffer, buf_length, "%*.*lu", space_padding, zero_padding, number);

    //find location of null char
    int null_char_location = strlen(buffer);

    //verify buffer is big enough
    if (buf_length < null_char_location + 2){   //+1 due to zero indexing and +1 for room for decimal point
        return -1;  //buffer not big enough
    }

    if (precision > null_char_location){
        return -1;  //invalid precision level
    }

    if (status < 0){
        return status;
    }

    //moving from right to left, move 'precision' number of digits over by one to leave gap for decimal place
    //includes moving null char
    memmove(&buffer[null_char_location - precision + 1], &buffer[null_char_location - precision], precision+1);

    //insert decimal place
    buffer[null_char_location - precision] = '.';

    return 0; //OK
}
