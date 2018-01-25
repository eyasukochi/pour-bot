/*
 * Stepper.cpp - Stepper library for Wiring/Arduino - Version 1.1.0
 *
 * Original library        (0.1)   by Tom Igoe.
 * Two-wire modifications  (0.2)   by Sebastian Gassner
 * Combination version     (0.3)   by Tom Igoe and David Mellis
 * Bug fix for four-wire   (0.4)   by Tom Igoe, bug fix from Noah Shibley
 * High-speed stepping mod         by Eugene Kozlenko
 * Timer rollover fix              by Eugene Kozlenko
 * Five phase five wire    (1.1.0) by Ryan Orendorff
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *
 * Drives a unipolar, bipolar, or five phase stepper motor.
 *
 * When wiring multiple stepper motors to a microcontroller, you quickly run
 * out of output pins, with each motor requiring 4 connections.
 *
 * By making use of the fact that at any time two of the four motor coils are
 * the inverse of the other two, the number of control connections can be
 * reduced from 4 to 2 for the unipolar and bipolar motors.
 *
 * A slightly modified circuit around a Darlington transistor array or an
 * L293 H-bridge connects to only 2 microcontroler pins, inverts the signals
 * received, and delivers the 4 (2 plus 2 inverted ones) output signals
 * required for driving a stepper motor. Similarly the Arduino motor shields
 * 2 direction pins may be used.
 *
 * The sequence of control signals for 5 phase, 5 control wires is as follows:
 *
 * Step C0 C1 C2 C3 C4
 *    1  0  1  1  0  1
 *    2  0  1  0  0  1
 *    3  0  1  0  1  1
 *    4  0  1  0  1  0
 *    5  1  1  0  1  0
 *    6  1  0  0  1  0
 *    7  1  0  1  1  0
 *    8  1  0  1  0  0
 *    9  1  0  1  0  1
 *   10  0  0  1  0  1
 *
 * The sequence of control signals for 4 control wires is as follows:
 *
 * Step C0 C1 C2 C3
 *    1  1  0  1  0
 *    2  0  1  1  0
 *    3  0  1  0  1
 *    4  1  0  0  1
 *
 * The sequence of controls signals for 2 control wires is as follows
 * (columns C1 and C2 from above):
 *
 * Step C0 C1
 *    1  0  1
 *    2  1  1
 *    3  1  0
 *    4  0  0
 *
 * The circuits can be found at
 *
 * http://www.arduino.cc/en/Tutorial/Stepper
 */

//#include "Arduino.h"
#include <esp_log.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "stepper.h"
#include "sdkconfig.h"

static uint32_t HIGH = 1;
static uint32_t LOW = 0;

static const char* LOG_TAG = "Stepper";


gpio_num_t mapFromInt(int integerPin)
{
	switch ( integerPin )
	{
		case 0: return GPIO_NUM_0;
		case 1: return GPIO_NUM_1;
		case 2: return GPIO_NUM_2;
		case 3: return GPIO_NUM_3;
		case 4: return GPIO_NUM_4;
		case 5: return GPIO_NUM_5;
		case 6: return GPIO_NUM_6;
		case 7: return GPIO_NUM_7;
		case 8: return GPIO_NUM_8;
		case 9: return GPIO_NUM_9;
		case 10: return GPIO_NUM_10;
		case 11: return GPIO_NUM_11;
		case 12: return GPIO_NUM_12;
		case 13: return GPIO_NUM_13;
		case 14: return GPIO_NUM_14;
		case 15: return GPIO_NUM_15;
		case 16: return GPIO_NUM_16;
		case 17: return GPIO_NUM_17;
		case 18: return GPIO_NUM_18;
		case 19: return GPIO_NUM_19;
		case 21: return GPIO_NUM_21;
		case 22: return GPIO_NUM_22;
		case 23: return GPIO_NUM_23;
		case 25: return GPIO_NUM_25;
		case 26: return GPIO_NUM_26;
		case 27: return GPIO_NUM_27;
		case 32: return GPIO_NUM_32;
		case 33: return GPIO_NUM_33;
		case 34: return GPIO_NUM_34;
		case 35: return GPIO_NUM_35;
		case 36: return GPIO_NUM_36;
		case 37: return GPIO_NUM_37;
		case 38: return GPIO_NUM_38;
		case 39: return GPIO_NUM_39;
		default: return GPIO_NUM_MAX;
	}
}

/*
 * two-wire constructor.
 * Sets which wires should control the motor.
 */
Stepper::Stepper(int number_of_steps, int motor_pin_1, int motor_pin_2)
{
	this->step_number = 0;    // which step the motor is on
	this->direction = 0;      // motor direction
	this->last_step_time = 0; // time stamp in us of the last step taken
	this->number_of_steps = number_of_steps; // total number of steps for this motor

	// Arduino pins for the motor control connection:
	this->motor_pin_1 = mapFromInt(motor_pin_1);
	this->motor_pin_2 = mapFromInt(motor_pin_2);

	// setup the pins on the microcontroller:
	gpio_config_t io_conf;
	//disable interrupt
	io_conf.intr_type = GPIO_INTR_DISABLE;
	//set as output mode
	io_conf.mode = GPIO_MODE_OUTPUT;
	//bit mask of the pins that you want to set,e.g.GPIO18/19
	io_conf.pin_bit_mask = ((1ULL<<motor_pin_1) | (1ULL<<motor_pin_2));
	//disable pull-down mode
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	//disable pull-up mode
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	//configure GPIO with the given settings
	gpio_config(&io_conf);

	// When there are only 2 pins, set the others to Max but I don't know why:
	this->motor_pin_3 = GPIO_NUM_MAX;
	this->motor_pin_4 = GPIO_NUM_MAX;
	this->motor_pin_5 = GPIO_NUM_MAX;

	// pin_count is used by the stepMotor() method:
	this->pin_count = 2;
}


/*
 *   constructor for four-pin version
 *   Sets which wires should control the motor.
 */
Stepper::Stepper(int number_of_steps, int motor_pin_1, int motor_pin_2,
                                      int motor_pin_3, int motor_pin_4)
{
	this->step_number = 0;    // which step the motor is on
	this->direction = 0;      // motor direction
	this->last_step_time = 0; // time stamp in us of the last step taken
	this->number_of_steps = number_of_steps; // total number of steps for this motor

	// Arduino pins for the motor control connection:
	this->motor_pin_1 = mapFromInt(motor_pin_1);
	this->motor_pin_2 = mapFromInt(motor_pin_2);
	this->motor_pin_3 = mapFromInt(motor_pin_3);
	this->motor_pin_4 = mapFromInt(motor_pin_4);

	// setup the pins on the microcontroller:
	gpio_config_t io_conf;
	//disable interrupt
	io_conf.intr_type = GPIO_INTR_DISABLE;
	//set as output mode
	io_conf.mode = GPIO_MODE_OUTPUT;
	//bit mask of the pins that you want to set,e.g.GPIO18/19
	io_conf.pin_bit_mask = ((1ULL<<motor_pin_1) | (1ULL<<motor_pin_2) | (1ULL<<motor_pin_3) | (1ULL<<motor_pin_4));
	//disable pull-down mode
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	//disable pull-up mode
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	//configure GPIO with the given settings
	gpio_config(&io_conf);

//	// When there are 4 pins, set the others to 0:
	this->motor_pin_5 = GPIO_NUM_MAX;

	// pin_count is used by the stepMotor() method:
	this->pin_count = 4;
}

/*
 *   constructor for five phase motor with five wires
 *   Sets which wires should control the motor.
 */
Stepper::Stepper(int number_of_steps, int motor_pin_1, int motor_pin_2,
                                      int motor_pin_3, int motor_pin_4,
                                      int motor_pin_5)
{
	this->step_number = 0;    // which step the motor is on
	this->direction = 0;      // motor direction
	this->last_step_time = 0; // time stamp in us of the last step taken
	this->number_of_steps = number_of_steps; // total number of steps for this motor

	// Arduino pins for the motor control connection:
	this->motor_pin_1 = mapFromInt(motor_pin_1);
	this->motor_pin_2 = mapFromInt(motor_pin_2);
	this->motor_pin_3 = mapFromInt(motor_pin_3);
	this->motor_pin_4 = mapFromInt(motor_pin_4);
	this->motor_pin_5 = mapFromInt(motor_pin_5);

	// setup the pins on the microcontroller:
	gpio_config_t io_conf;
	//disable interrupt
	io_conf.intr_type = GPIO_INTR_DISABLE;
	//set as output mode
	io_conf.mode = GPIO_MODE_OUTPUT;
	//bit mask of the pins that you want to set,e.g.GPIO18/19
	io_conf.pin_bit_mask = ((1ULL<<motor_pin_1) | (1ULL<<motor_pin_2) | (1ULL<<motor_pin_3) | (1ULL<<motor_pin_4) | (1ULL<<motor_pin_5));
	//disable pull-down mode
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	//disable pull-up mode
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	//configure GPIO with the given settings
	gpio_config(&io_conf);

	// pin_count is used by the stepMotor() method:
	this->pin_count = 5;
}

/**
 * Returns the current time in microseconds.
 */
long micros(){
	clock_t elapsedClocks = clock();
	//1 elapsed clocks
	//1000 clocks per second. this would mean one clock every millisecond
	long time = (elapsedClocks/CLOCKS_PER_SEC) * 1000000;
	return time;
}

/*
 * Sets the speed in revs per minute
 */
void Stepper::setSpeed(long whatSpeed)
{
  this->step_delay = 60L * 1000L * 1000L / this->number_of_steps / whatSpeed;
  ESP_LOGD(LOG_TAG, "Step delay now set to %ld", this->step_delay);
}

/*
 * Moves the motor steps_to_move steps.  If the number is negative,
 * the motor moves in the reverse direction.
 */
void Stepper::step(int steps_to_move)
{
	ESP_LOGD(LOG_TAG, "Attempting to move %d steps", steps_to_move);
	int steps_left = abs(steps_to_move);  // how many steps to take

	// determine direction based on whether steps_to_mode is + or -:
	if (steps_to_move > 0) { this->direction = 1; }
	if (steps_to_move < 0) { this->direction = 0; }


	// decrement the number of steps, moving one step each time:
	while (steps_left > 0)
	{
		unsigned long now = micros();
		//TODO don't actually wait, it would be closer to schedule a delay via vTaskDelay probably
		// Definitely more efficient, probably slightly less accurate.
		// move only if the appropriate delay has passed:
		if (now - this->last_step_time >= this->step_delay)
		{
		  // get the timeStamp of when you stepped:
		  this->last_step_time = now;
		  // increment or decrement the step number,
		  // depending on direction:
		  if (this->direction == 1)
		  {
			this->step_number++;
			if (this->step_number == this->number_of_steps) {
			  this->step_number = 0;
			}
		  }
		  else
		  {
			if (this->step_number == 0) {
			  this->step_number = this->number_of_steps;
			}
			this->step_number--;
		  }
		  // decrement the steps left:
		  steps_left--;
		  // step the motor to step number 0, 1, ..., {3 or 10}
		  if (this->pin_count == 5)
			stepMotor(this->step_number % 10);
		  else
			stepMotor(this->step_number % 4);
		}
	}
}

/*
 * Moves the motor forward or backwards.
 */
void Stepper::stepMotor(int thisStep)
{
	ESP_LOGD(LOG_TAG, "Executing step number %d", thisStep);
  if (this->pin_count == 2) {
    switch (thisStep) {
      case 0:  // 01
        gpio_set_level(motor_pin_1, LOW);
        gpio_set_level(motor_pin_2, HIGH);
      break;
      case 1:  // 11
        gpio_set_level(motor_pin_1, HIGH);
        gpio_set_level(motor_pin_2, HIGH);
      break;
      case 2:  // 10
        gpio_set_level(motor_pin_1, HIGH);
        gpio_set_level(motor_pin_2, LOW);
      break;
      case 3:  // 00
        gpio_set_level(motor_pin_1, LOW);
        gpio_set_level(motor_pin_2, LOW);
      break;
    }
  }
  if (this->pin_count == 4) {
    switch (thisStep) {
      case 0:  // 1010
        gpio_set_level(motor_pin_1, HIGH);
        gpio_set_level(motor_pin_2, LOW);
        gpio_set_level(motor_pin_3, HIGH);
        gpio_set_level(motor_pin_4, LOW);
      break;
      case 1:  // 0110
        gpio_set_level(motor_pin_1, LOW);
        gpio_set_level(motor_pin_2, HIGH);
        gpio_set_level(motor_pin_3, HIGH);
        gpio_set_level(motor_pin_4, LOW);
      break;
      case 2:  //0101
        gpio_set_level(motor_pin_1, LOW);
        gpio_set_level(motor_pin_2, HIGH);
        gpio_set_level(motor_pin_3, LOW);
        gpio_set_level(motor_pin_4, HIGH);
      break;
      case 3:  //1001
        gpio_set_level(motor_pin_1, HIGH);
        gpio_set_level(motor_pin_2, LOW);
        gpio_set_level(motor_pin_3, LOW);
        gpio_set_level(motor_pin_4, HIGH);
      break;
    }
  }

  if (this->pin_count == 5) {
    switch (thisStep) {
      case 0:  // 01101
        gpio_set_level(motor_pin_1, LOW);
        gpio_set_level(motor_pin_2, HIGH);
        gpio_set_level(motor_pin_3, HIGH);
        gpio_set_level(motor_pin_4, LOW);
        gpio_set_level(motor_pin_5, HIGH);
        break;
      case 1:  // 01001
        gpio_set_level(motor_pin_1, LOW);
        gpio_set_level(motor_pin_2, HIGH);
        gpio_set_level(motor_pin_3, LOW);
        gpio_set_level(motor_pin_4, LOW);
        gpio_set_level(motor_pin_5, HIGH);
        break;
      case 2:  // 01011
        gpio_set_level(motor_pin_1, LOW);
        gpio_set_level(motor_pin_2, HIGH);
        gpio_set_level(motor_pin_3, LOW);
        gpio_set_level(motor_pin_4, HIGH);
        gpio_set_level(motor_pin_5, HIGH);
        break;
      case 3:  // 01010
        gpio_set_level(motor_pin_1, LOW);
        gpio_set_level(motor_pin_2, HIGH);
        gpio_set_level(motor_pin_3, LOW);
        gpio_set_level(motor_pin_4, HIGH);
        gpio_set_level(motor_pin_5, LOW);
        break;
      case 4:  // 11010
        gpio_set_level(motor_pin_1, HIGH);
        gpio_set_level(motor_pin_2, HIGH);
        gpio_set_level(motor_pin_3, LOW);
        gpio_set_level(motor_pin_4, HIGH);
        gpio_set_level(motor_pin_5, LOW);
        break;
      case 5:  // 10010
        gpio_set_level(motor_pin_1, HIGH);
        gpio_set_level(motor_pin_2, LOW);
        gpio_set_level(motor_pin_3, LOW);
        gpio_set_level(motor_pin_4, HIGH);
        gpio_set_level(motor_pin_5, LOW);
        break;
      case 6:  // 10110
        gpio_set_level(motor_pin_1, HIGH);
        gpio_set_level(motor_pin_2, LOW);
        gpio_set_level(motor_pin_3, HIGH);
        gpio_set_level(motor_pin_4, HIGH);
        gpio_set_level(motor_pin_5, LOW);
        break;
      case 7:  // 10100
        gpio_set_level(motor_pin_1, HIGH);
        gpio_set_level(motor_pin_2, LOW);
        gpio_set_level(motor_pin_3, HIGH);
        gpio_set_level(motor_pin_4, LOW);
        gpio_set_level(motor_pin_5, LOW);
        break;
      case 8:  // 10101
        gpio_set_level(motor_pin_1, HIGH);
        gpio_set_level(motor_pin_2, LOW);
        gpio_set_level(motor_pin_3, HIGH);
        gpio_set_level(motor_pin_4, LOW);
        gpio_set_level(motor_pin_5, HIGH);
        break;
      case 9:  // 00101
        gpio_set_level(motor_pin_1, LOW);
        gpio_set_level(motor_pin_2, LOW);
        gpio_set_level(motor_pin_3, HIGH);
        gpio_set_level(motor_pin_4, LOW);
        gpio_set_level(motor_pin_5, HIGH);
        break;
    }
  }
  vTaskDelay(2);
}

/*
  version() returns the version of the library:
*/
int Stepper::version(void)
{
  return 1;
}
