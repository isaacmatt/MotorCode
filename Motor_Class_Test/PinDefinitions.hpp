#pragma once

// PIN DEFINITIONS -----------------------------------------------------
	// Input pins for controller (not related to pitch motors)
	#define INPUT_VOLTAGE_PIN A0
	#define BOOST_VOLTAGE_PIN A1
	#define INPUT_CURRENT_PIN A2
	#define BOOST_CURRENT_PIN A3
	#define WIND_SPEED_PIN A4
  
	#define HALL_EFFECT_SENSOR A6
  #define STABLE_HALL_EFFECT_OUTPUT 208
  #define STABLE_HALL_EFFECT_BUFFER 10

	// Output pins
	#define BOOST_CONVERTER_PIN_1 12
	#define BOOST_CONVERTER_PIN_2 11

	#define YAW_DRIVER_DIRECTION_PIN_1 32
	#define YAW_DRIVER_DIRECTION_PIN_2 33
	#define YAW_DRIVER_PWM_PIN 5
  #define YAW_MOTOR_SPEED 100


	// Pitch Motor Pin definitions:
	// Pitch Motor 1
	#define ENCODER_A1 2    //Mega Interrupt Pin
	#define ENCODER_B1 23  //Non-interrupt Pin
	#define PWM_1 6 
	#define DIRECTION_A1 24
	#define DIRECTION_B1 25

	// 	#define ENCODER_A1 3    //Mega Interrupt Pin
	// #define ENCODER_B1 2  //Non-interrupt Pin
	// #define PWM_1 5 
	// #define DIRECTION_A1 7
	// #define DIRECTION_B1 6


	// Pitch Motor 2
	#define ENCODER_A2 8 //Mega Interrupt Pin
	#define ENCODER_B2 26 //Non-interrupt Pin
	#define PWM_2 4   //7 switching to pin
	#define DIRECTION_A2 27
	#define DIRECTION_B2 28

	// Pitch Motor 3
	#define ENCODER_A3 18 //Mega Interrupt Pin
	#define ENCODER_B3 29 //Non-interrupt Pin
	#define PWM_3 8
	#define DIRECTION_A3 30
	#define DIRECTION_B3 31