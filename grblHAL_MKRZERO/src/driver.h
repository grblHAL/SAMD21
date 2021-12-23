/*
  driver.h - driver code for Atmel SAMD21 ARM processor

  Part of grblHAL

  Copyright (c) 2018-2020 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

//
// NOTE: do NOT change configuration here - edit my_machine.h instead!
//

#ifndef __DRIVER_H__
#define __DRIVER_H__

#include "grbl/hal.h"

#ifndef OVERRIDE_MY_MACHINE
#include "my_machine.h"
#endif

#include "grbl/driver_opts.h"

// clock definitions

#define CLKTCC_0_1 GCLK_CLKCTRL_GEN_GCLK4

// timer definitions

#define STEP_TIMER          TC3
#define STEP_TIMER_IRQn     TC3_IRQn

#define STEPPER_TIMER       TC4 // 32bit - TC4 & TC5 combined!
#define STEPPER_TIMER_IRQn  TC4_IRQn

#define DEBOUNCE_TIMER      TCC1
#define DEBOUNCE_TIMER_IRQn TCC1_IRQn

#ifdef BOARD_CNC_BOOSTERPACK
  #include "cnc_boosterpack_map.h"
#elif defined(BOARD_MY_MACHINE)
  #include "my_machine_map.h"
#else
  #include "generic_map.h"
#endif

// Adjust STEP_PULSE_LATENCY to get accurate step pulse length when required, e.g if using high step rates.
// The default value is calibrated for 10 microseconds length.
// NOTE: step output mode, number of axes and compiler optimization settings may all affect this value.
#ifndef STEP_PULSE_LATENCY
#define STEP_PULSE_LATENCY 2.3f // microseconds
#endif

// End configuration

#if TRINAMIC_ENABLE
#ifndef TRINAMIC_MIXED_DRIVERS
#define TRINAMIC_MIXED_DRIVERS 1
#endif
#include "motors/trinamic.h"
#include "trinamic/common.h"
#endif

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PIN (5u)
#endif

// Define SD card detect pin.
#define SD_CD_PIN   30

void IRQRegister(int32_t IRQnum, void (*IRQhandler)(void));
void IRQUnRegister(int32_t IRQnum);

#if IOEXPAND_ENABLE
#undef I2C_ENABLE
#define I2C_ENABLE 1
#endif

#if I2C_ENABLE
// Define I2C port/pins
#define I2C_PORT SERCOM0
#define I2C_SDA_PIN 11
#define I2C_SCL_PIN 12
#define I2C_CLOCK 100000
#endif

#endif
