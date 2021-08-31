/*
 *  TinyIR.h
 *
 *
 *  Copyright (C) 2021  chris miller
 *  miller4@rose-hulman.edu
 *
 *  Modified 2021 chris miller to support MSP432 on Energia
 *  Modified 2020-2021  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of IRMP https://github.com/ukw100/IRMP.
 *  This file is part of Arduino-IRremote https://github.com/Arduino-IRremote/Arduino-IRremote.
 *
 *  TinyIR is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#ifndef TINY_IR_H
#define TINY_IR_H

#include <Energia.h>

#include "LongUnion.h"

/*
 * Set sensible receive pin for different CPU's
 */
#if defined(__MSP432P401R__) || defined(__MSP432P4111__)
#define IR_INPUT_PIN    33  /* P5.1 */

#else
#define IR_INPUT_PIN    2
#endif


/*
 * Uncomment the following line to include repeats for available IR results. 
 *  Otherwise, repeats are ignored. Must be defined before including header
 */
//#define INCLUDE_REPEATS

/*
 * Uncommenting following line saves 12 bytes and reduces ISR handling time
 */
//#define DO_NOT_USE_FEEDBACK_LED 
#if !defined(IR_FEEDBACK_LED_PIN)
#define IR_FEEDBACK_LED_PIN    YELLOW_LED
#endif

/*
 * Uncomment the following line (or define HANDLE_IR_EVENT in user code prior 
 *  to include) in order to define handler for IR event in user code
 */
//#define HANDLE_IR_EVENT

#include "TinyNECRX.cpp.h"

/** @}*/

#endif // TINY_IR_H

#pragma once
