/*
 * drive_by_wire.c
 *
 *  Created on: 10 Oct 2023
 *      Author: umut
 */

#include "main.h"
#include "stdlib.h"


/********************************************************************
 * getValues: Takes incoming data from uart as a argument.          *
 *            After sets steeringAngle and speed external variables.*
 *                                                                  *
 *******************************************************************/
void getValues(uint8_t data[]){
    // Get data and parse.
    // Set steeringAngle and speed external variables.
}


/******************************************************************
 * defaultValues: Sets default value for speed and steeringAngle  *
 *                external variables.                             *
 *                                                                *
 *****************************************************************/
void defaultValues(void){
    // Set steeringAngle and speed external variables with default values.
}


void delay_us (TIM_HandleTypeDef * htim, uint16_t us)
{
	__HAL_TIM_SET_COUNTER(htim,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(htim) < us);  // wait for the counter to reach the us input in the parameter
}


/******************************************************************
 * map: Maps value by input interval to output interval.          *
 *      Takes value, inputMin, inputMax, outputMin, outputMax     *
 *      as a argument. Returns mapped value.                      *
 *                                                                *
 *****************************************************************/
int map(int value, int inputMin, int inputMax, int outputMin, int outputMax ){
	return (((((1.0*abs(inputMin-value)) / (abs(inputMax - inputMin))) * (abs(outputMax - outputMin))) + outputMin)) ;
}


/******************************************************************
 * setSpeed: Sets speed of vehicle. Maps speed argument and       *
 *           sets mappedSpeed value to speed of vehicle.          *
 *                                                                *
 *****************************************************************/
void setSpeed(uint16_t speed){
    // Get mapped speed by speed argument
    // Set speed level of vehicle with PWM.
}


/********************************************************************
 * getBrakeLevel: Maps brakeLevel argument. Returns mapped brake    *
 *                level.                                            *
 *                                                                  *
 *******************************************************************/
int getBrakeLevel(uint8_t brakeLevel){
	return  map(brakeLevel, 0, 10 ,0 , 100);
}


/********************************************************************
 * setBrakeLevel: Sets brake level of vehicle. Gets mapped brake    *
 *                level and controls brake stepper motor with       *
 *                GPIO.                                             *
 *                                                                  *
 *******************************************************************/
void setBrakeLevel(uint8_t brakeLevel){
    // Get mapped brake level
    // Set brake level of vehicle with controlling stepper motor with GPIO
}


/********************************************************************
 * resetBrakeLevel: Resets brake level of vehicle to starting       *
 *                  position(stop braking). Gets mapped brake level *
 *                  and controls brake stepper motor with GPIO.     *
 *                                                                  *
 *******************************************************************/
void resetBrakeLevel(uint8_t brakeLevel){
    // Get mapped brake level
    // Stop braking with controlling stepper motor with GPIO
}


/*
 void setRegenerativeBrake(){
}

void resetRegenerativeBrake(){
}
*/

/*********************************************************************
 * getSteeringAngle: Gets steering angle of vehicle and returns it.  *
 *                  Takes steeringAngle argument and calculates      *
 *                  angle change by comparing with previous steering *
 *                  degree.                                          *
 *                  !Vehicle should start at straight steering angle.*
 *                                                                   *
 ********************************************************************/
int getSteeringAngle(uint8_t steeringAngle){
    // Get angle change by comparing steeringAngle with prevAngle value
    // Return mapped angle value
	return 1;
}


/*********************************************************************
 * setSteeringAngle: Gets steering angle of vehicle and returns it.  *
 *                  Takes steeringAngle argument and calculates      *
 *                  angle change by comparing with previous steering *
 *                  degree.                                          *
 *                  !Vehicle should start at straight steering angle.*
 *                                                                   *
 ********************************************************************/
void setSteeringAngle(uint8_t steeringAngle){
    // Get mapped steering angle
    // Control steering stepper motor with GPIO
}



