/*
  generic_map.h - driver code for Atmel SAMD21 ARM processor

  Part of grblHAL

  Copyright (c) 2020-2023 Terje Io

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

// Define step pulse output pins.
#define X_STEP_PIN              (19u)
#define Y_STEP_PIN              (20u)
#define Z_STEP_PIN              (21u)

// Define step direction output pins.
#define X_DIRECTION_PIN         (2u)
#define Y_DIRECTION_PIN         (3u)
#define Z_DIRECTION_PIN         (4u)

// Define stepper driver enable/disable output pin(s).
#define STEPPERS_DISABLE_PIN    (10u)

// Define homing/hard limit switch input pins.
#define X_LIMIT_PIN             (0u)
#define Y_LIMIT_PIN             (1u)
#define Z_LIMIT_PIN             (8u)

// Define driver spindle pins

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_TIMER       TCC0
#define SPINDLE_PWM_CCREG       2
#define SPINDLE_PWM_PIN         (6u)
#endif

#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PIN   (15u)
#endif

#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PIN      (7u)
#endif

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PIN       (12u)
#define COOLANT_MIST_PIN        (11u)

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PIN               (9u)
#define FEED_HOLD_PIN           (17u)
#define CYCLE_START_PIN         (16u)
#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN         (5u)
#endif

// Define probe switch input pin.
#define PROBE_PIN               (18U)

/**/

