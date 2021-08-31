# TinyNECRX
https://github.com/ndroid/TinyIR

Adaptation of TinyIRReceiver from Arduino IRRemote library <a href="https://github.com/Arduino-IRremote/"> 
Modified to work with MSP432 boards in Energia.


*Define IR_FEEDBACK_LED_PIN to change default feedback LED pin

*Define DO_NOT_USE_FEEDBACK_LED to save 12 bytes and reduce ISR handling time

*By default, repeats are ignored and decodeIR() only returns new commands.
Define INCLUDE_REPEATS to include repeats in decodeIR(). 

*Define HANDLE_IR_EVENT in order to provide user-defined handler for IR event.
If enabled, must define 
  void handleReceivedTinyIRData(uint16_t aAddress, uint8_t aCommand, bool isRepetition)
This function is called if a complete command was received, regardless if it is 
a repeat (not impacted by INCLUDE_REPEATS constant).
!!!!!!!!!!!!!!!!!!!!!!
  Function called in interrupt context - should be running as short as possible, so if you 
  require longer action, save the data (address + command) and handle it in the main loop.
!!!!!!!!!!!!!!!!!!!!!
