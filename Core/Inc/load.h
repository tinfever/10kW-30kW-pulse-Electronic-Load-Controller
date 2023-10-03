/*
 * sequencing.h
 *
 *  Created on: Aug 7, 2023
 *      Author: user
 */

#ifndef INC_LOAD_H_
#define INC_LOAD_H_

void sequenceOn(void);
void sequenceOff(void);
void enableLoad(void);
void disableLoad(void);
void LoadControl(void);

uint32_t CalibrateSingleStage(uint32_t stage_num);
void CalibrateAllStages(void);
LoadStageCombo StageComboSelect(uint32_t current_set_point_mA);


#endif /* INC_LOAD_H_ */
