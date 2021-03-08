#pragma once
/*
 * Description:     Class containing various defines needed in main
 *
 * Author:			Erik Lindby
 *					University of Southern Denmark
 * Creation date:   12-02-2021
 */

 /*******************************************************************************
  *******************************************************************************
  * CLASS
  *******************************************************************************
  ******************************************************************************/

/* Without Matrix Voice
#define TB6612_RIGHT_MOTOR_PWMA		12 // (Orange)
#define TB6612_LEFT_MOTOR_PWMB		13 // (Green)
#define TB6612_RIGHT_MOTOR_AIN1		16 // (Blue)
#define TB6612_RIGHT_MOTOR_AIN2		26 // (Brown)
#define TB6612_LEFT_MOTOR_BIN1		5  // (Grey)
#define TB6612_LEFT_MOTOR_BIN2		6  // (Pink)


// With Matrix Voice but using Raspberry Pi GPIO
#define  TB6612_RIGHT_MOTOR_PWMA        13 // (Orange)
#define  TB6612_LEFT_MOTOR_PWMB         16 // (Green)
#define  TB6612_RIGHT_MOTOR_AIN1        19 // (Blue)
#define  TB6612_RIGHT_MOTOR_AIN2        26 // (Brown)
#define  TB6612_LEFT_MOTOR_BIN1         20 // (Grey)
#define  TB6612_LEFT_MOTOR_BIN2         21 // (Pink)
*/

// GPIO via Matrix Voice
#define  TB6612_RIGHT_MOTOR_PWMA        14 // (Orange)
#define  TB6612_LEFT_MOTOR_PWMB         8  // (Green)
#define  TB6612_RIGHT_MOTOR_AIN1        12 // (Blue)
#define  TB6612_RIGHT_MOTOR_AIN2        10 // (Brown)
#define  TB6612_LEFT_MOTOR_BIN1         6  // (Grey)
#define  TB6612_LEFT_MOTOR_BIN2         4  // (Pink)

// Matrix Voice LEDs
#define MATRIX_LED_R_1          0
#define MATRIX_LED_R_2          1
#define MATRIX_LED_R_3          2
#define MATRIX_LED_R_4          3
#define MATRIX_LED_R_5          4
#define MATRIX_LED_R_6          5
#define MATRIX_LED_R_7          6
#define MATRIX_LED_R_8          7
#define MATRIX_LED_R_9          8

#define MATRIX_LED_L_1          9
#define MATRIX_LED_L_2          10
#define MATRIX_LED_L_3          11
#define MATRIX_LED_L_4          12
#define MATRIX_LED_L_5          13
#define MATRIX_LED_L_6          14
#define MATRIX_LED_L_7          15
#define MATRIX_LED_L_8          16
#define MATRIX_LED_L_9          17

//ODAS
#define ENERGY_COUNT 36		// ENERGY_COUNT : Number of sound energy slots to maintain. - Implicit also defines angle precision, eg. 36 has angle precision of 10 degrees
#define MAX_VALUE 200		// MAX_VALUE : controls smoothness
#define INCREMENT 20		// INCREMENT : controls sensitivity
#define DECREMENT 1			// DECREMENT : controls delay in the dimming
#define MIN_THRESHOLD 8//10	// MAX_BRIGHTNESS: Filters out low energy
#define MAX_BRIGHTNESS 50	// MAX_BRIGHTNESS: 0 - 255
