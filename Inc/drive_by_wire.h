/*
 * drive_by_wire.h
 *
 *  Created on: 10 Oct 2023
 *      Author: umut
 */

#ifndef INC_DRIVE_BY_WIRE_H_
#define INC_DRIVE_BY_WIRE_H_

#endif /* INC_DRIVE_BY_WIRE_H_ */


/****** User Function Prototypes *********/
void setBrakeLevel(uint8_t brakeLevel);
void resetBrakeLevel(uint8_t brakeLevel);       /*** Brake-by-wire ***/
int getBrakeLevel(uint8_t brakeLevel);

int getSteeringAngle(uint8_t steeringAngle);    /*** Steer-by-wire ***/
void setSteeringAngle(uint8_t steeringAngle);

int map(int value,
        int inputMin, int inputMax,
        int outputMin, int outputMax );

void setSpeed(uint16_t speed);                  /*** Throttle-by-wire ***/

void getValues(uint8_t data[]);                 // Get uart data
void defaultValues(void);                       // If not any uart data use default values
void delay_us (TIM_HandleTypeDef * htim, uint16_t us);
