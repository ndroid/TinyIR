/*
 *  MinimalReceiver.ino
 *
 *  Small memory footprint and no timer usage!
 *
 *  Receives IR protocol data of NEC protocol using pin change interrupts.
 *  On complete received IR command the function decodeIR(IRData *results) may 
 *  be called to fetch results. Return value indicates if new data is available.
 *  Repeats are ignored unless INCLUDE_REPEATS is defined. 
 *  
 *  IR input pin must be pin 33 (P5.1)
 *
 *
 *  TinyIR is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */


/*
 * Uncomment the following line to change default feedback LED pin
 */
//#define IR_FEEDBACK_LED_PIN    LED1
/*
 * Uncommenting following line saves 12 bytes and reduces ISR handling time
 */
//#define DO_NOT_USE_FEEDBACK_LED 
/*
 * Uncomment the following line to include repeats in decodeIR(). 
 *  Otherwise, repeats are ignored and decodeIR() only returns new commands.
 */
//#define INCLUDE_REPEATS
/*
 * Uncomment the following line in order to define handler for IR event.
 * If enabled, must define 
 *  void handleReceivedTinyIRData(uint16_t aAddress, uint8_t aCommand, bool isRepetition)
 */
//#define HANDLE_IR_EVENT

// Preceding defines (if enabled) must be included prior to including header file
#include "TinyIR.h"


/*
 * Helper macro for getting a macro definition as string
 */
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

IRData IRresults;

void setup() {
    Serial.begin(57600);
    delay(500); // To be able to connect Serial monitor after reset or power up 
    Serial.println(F("START " __FILE__ " from " __DATE__));
    /*
     * Must be called to initialize and set up IR receiver pin.
     * IR receiver must be on pin 33 (P5.1)
     */
    initTinyIRReceiver();
    Serial.println(F("Ready to receive NEC IR signals at pin " STR(IR_INPUT_PIN)));
}

void loop() {
    // decodeIR updates results and returns true if new command is available, 
    //  otherwise results struct is unchanged and returns false
    if (decodeIR(&IRresults)) {
        Serial.print("A=0x");
        Serial.print(IRresults.address, HEX);
        Serial.print(" C=0x");
        Serial.print(IRresults.command, HEX);
        Serial.print(" is repeat: ");
        Serial.print(IRresults.isRepeat ? "TRUE" : "FALSE");
        Serial.println();
    }
}
