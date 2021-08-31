/*
 *  TinyNECRX.h
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

#ifndef TINY_NEC_RX_H
#define TINY_NEC_RX_H

#include <Energia.h>

#include "LongUnion.h"

/** \addtogroup TinyReceiver Minimal receiver for NEC protocol
 * @{
 */


#if defined(HANDLE_IR_EVENT)
/*
 * This function is called if a complete command was received and must be 
 *  implemented by the including file (user code)
 */
void handleReceivedTinyIRData(uint16_t aAddress, uint8_t aCommand, bool isRepeat);
#endif

// LSB first, 1 start bit + 16 bit address + 8 bit data + 8 bit inverted data + 1 stop bit.
// see: https://www.sbprojects.net/knowledge/ir/nec.php

#define NEC_ADDRESS_BITS        16 // 16 bit address or 8 bit address and 8 bit inverted address
#define NEC_COMMAND_BITS        16 // Command and inverted command

#define NEC_BITS                (NEC_ADDRESS_BITS + NEC_COMMAND_BITS)
#define NEC_UNIT                560

#define NEC_HEADER_MARK         (16 * NEC_UNIT) // 9000
#define NEC_HEADER_SPACE        (8 * NEC_UNIT)  // 4500

#define NEC_BIT_MARK            NEC_UNIT
#define NEC_ONE_SPACE           (3 * NEC_UNIT)  // 1690
#define NEC_ZERO_SPACE          NEC_UNIT

#define NEC_REPEAT_HEADER_SPACE (4 * NEC_UNIT)  // 2250
#define NEC_REPEAT_PERIOD       110000 // Not used yet - Commands are repeated every 110 ms (measured from start to start) for as long as the key on the remote control is held down.

/*
 * Macros for comparing timing values
 */
#define lowerValue25Percent(aDuration)   (aDuration - (aDuration / 4))
#define upperValue25Percent(aDuration)   (aDuration + (aDuration / 4))
#define lowerValue(aDuration)   (aDuration - (aDuration / 2))
#define upperValue(aDuration)   (aDuration + (aDuration / 2))

/*
 * The states for the state machine
 */
#define IR_RECEIVER_STATE_WAITING_FOR_START_MARK        0
#define IR_RECEIVER_STATE_WAITING_FOR_START_SPACE       1
#define IR_RECEIVER_STATE_WAITING_FOR_FIRST_DATA_MARK   2
#define IR_RECEIVER_STATE_WAITING_FOR_DATA_SPACE        3
#define IR_RECEIVER_STATE_WAITING_FOR_DATA_MARK         4
#define IR_RECEIVER_STATE_WAITING_FOR_STOP_MARK         5
/**
 * Control and data variables of the state machine for TinyReceiver
 */
struct TinyIRReceiverStruct {
    /*
     * State machine
     */
    uint32_t LastChangeMicros;      ///< microseconds of last Pin Change Interrupt.
    uint8_t IRReceiverState;        ///< the state of the state machine.
    uint8_t IRRawDataBitCounter;
    /*
     * Data
     */
    uint32_t IRRawDataMask;
    LongUnion IRRawData;
    bool IRRepeatDetected;
    bool newCommandAvailable;
};

/**
 * An enum consisting of all formats supported by IRRemote library.
 * Included for consistancy with existing libraries.
 */
typedef enum {
    UNKNOWN = 0,
    PULSE_DISTANCE,
    PULSE_WIDTH,
    DENON,
    DISH,
    JVC,
    LG,
    LG2,
    NEC,
    PANASONIC,
    KASEIKYO,
    KASEIKYO_JVC,
    KASEIKYO_DENON,
    KASEIKYO_SHARP,
    KASEIKYO_MITSUBISHI,
    RC5,
    RC6,
    SAMSUNG,
    SHARP,
    SONY,
    ONKYO,
    APPLE,
    BOSEWAVE,
    LEGO_PF,
    MAGIQUEST,
    WHYNTER,
} decode_type_t;

const __FlashStringHelper* getProtocolString(decode_type_t aProtocol);

/**
 * Data structure for the user application, adapted from IRRemote library.
 * Minimal struct to only include necessary data.
 */
typedef struct IRDataStruct {
    decode_type_t protocol;     ///< UNKNOWN, NEC, SONY, RC5, ...
    uint16_t address;           ///< Decoded address
    uint16_t command;           ///< Decoded command
    bool isRepeat;
} IRData;

void initTinyIRReceiver();
bool decodeIR(IRData *results);

/** @}*/

#endif // TINY_NEC_RX_H

#pragma once
