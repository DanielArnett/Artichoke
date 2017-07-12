/*	Open Bionics - Artichoke
*	Author - Olly McBride
*	Date - December 2015
*
*	This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.
*	To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/.
*
*	Website - http://www.openbionics.com/
*	GitHub - https://github.com/Open-Bionics
*	Email - ollymcbride@openbionics.com
*
*	OpenBionics_Artichoke.inom
*
*/

#include <FingerLib.h>			// This library can be downloaded from https://github.com/Open-Bionics
#include <Wire.h>
#include <EEPROM.h>

// Bleufuit libraries for the bluetooth radio
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
    #include <SoftwareSerial.h>
#endif

#include "Globals.h"

#include "GripControl.h"
#include "CircleBuff.h"
#include "Demo.h"
#include "EMGControl.h"
#include "MotorControl.h"
#include "PinManagement.h"
#include "SerialControl.h"
#include "TimerManagement.h"
#include "Utils.h"

#ifdef USE_I2C_ADC
#include <I2C_ADC.h>			// This library can be downloaded from https://github.com/Open-Bionics
#endif


#ifdef HANDLE_EN
#include "Wiichuck.h"			// Nunchuck library, written by jnw.walker@gmail.com
#include "HANDle.h"
#endif



/***************************************************************************************************
*
*	Open Bionics - Artichoke Release Notes
*
*	Version	|	Date		|	Notes
*	V1.0.0	|	08/01/16	|	Initial release for Ada hand and Almond boards using Atmega 2560
*	V1.0.1	|	03/02/16	|	Modified formatting and cleaned up
*	V1.1.0	|	31/03/16	|	Added research and HANDle mode. Fixed motorEn and muscle graph
*	V1.1.1	|	17/05/16	|	Increased PWM timer freq to prevent hum and implemented customDelay() instead of delay()
*	V1.2.0	|	22/08/16	|	Re-written EMG control (now allows both 1 & 2 channel control)
*
*
*	Artichoke Description
*
*		- Simple hand control software designed to run on the Open Bionics Almond hand controller
*		- Uses FingerLib.h for low level finger control, which allows fingers to be treated as servos
*		- Can be controlled via the following methods:
*			- Serial control (baud 38400)
*			- Muscle control (EMG)
*			- HANDle control (Nunchuck)
*		- Uses either inbuilt ADC or external I2C ADC for muscle sensing and hand control
*		- Enter '?' to view available serial commands
*
****************************************************************************************************/


Adafruit_BluefruitLE_UART ble(Serial3, BLUEFRUIT_UART_MODE_PIN);
int debug;
// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void setup()
{
        debug = 0;
        // Radio Code
        if (debug) {
          Serial.begin(115200);
          Serial.println(F("Adafruit Bluefruit Command Mode Example"));
          Serial.println(F("---------------------------------------"));

          /* Initialise the module */
          Serial.print(F("Initialising the Bluefruit LE module: "));
        }
        if ( !ble.begin(VERBOSE_MODE) && debug)
        {
          error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
        }
        if (debug)
          Serial.println( F("OK!") );

        if ( FACTORYRESET_ENABLE )
        {
          if (debug) {
            /* Perform a factory reset to make sure everything is in a known state */
            Serial.println(F("Performing a factory reset: "));
            if ( ! ble.factoryReset() ){
              error(F("Couldn't factory reset"));
            }
        }
        }

        /* Disable command echo from Bluefruit */
        ble.echo(false);
        if (debug) {
          Serial.println("Requesting Bluefruit info:");
        if (debug)
          /* Print Bluefruit information */
          ble.info();

          Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
          Serial.println(F("Then Enter characters to send to Bluefruit"));
          Serial.println();
        }
        ble.verbose(false);  // debug info is a little annoying after this point!

        if (debug) {
          /* Wait for connection */
          while (! ble.isConnected()) {
              delay(500);
          }
        }
        // LED Activity command is only supported from 0.6.6
        if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) && debug)
        {
          // Change Mode LED Activity
          Serial.println(F("******************************"));
          Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
          ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
          Serial.println(F("******************************"));
        }


        // Hand Code
      //  MYSERIAL.begin(115200);		// start serial

//        #if defined(USE_I2C_ADC) || defined(HANDLE_EN)
//      	  Wire.begin();				// if using I2C for I2C_ADC or for HANDle control, initialise I2C
//        #endif
        advancedSettings.handFlag = RIGHT;
      	timerSetup();				// start timer interrupts
        if (debug) {
        	setDefaults();				// initialise serialCmd.buffs, finger positions and muscle control, read EEPROM presets
        }
      	IOconfig();					// config finger pins, initialise port expander
  			EEPROM_writeStruct(ADVANCED_CTRL_LOC,advancedSettings);
        // Ensure that the handflag is still RIGHT
        if (debug) {
          startUpMessages();			// print welcome message, current hand configuration/settings
        }

        // gripMovement(0, 0);
}

void loop()
{
        // Radio
        // Check for user input
        char inputs[BUFSIZE+1];
        if (debug) {
          if ( getUserInput(inputs, BUFSIZE) )
          {
            if (debug) {
              // Send characters to Bluefruit
              Serial.print("[Send] ");
              Serial.println(inputs);
              ble.print("AT+BLEUARTTX=");
            }
            ble.println(inputs);

            // check response stastus
            if (! ble.waitForOK() && debug) {
              Serial.println(F("Failed to send?"));
            }
          }
        }
        // Check for incoming characters from Bluefruit
//        ble.println("AT+BLEUARTRX");
        ble.readline();
        if (strcmp(ble.buffer, "OK") == 0 || strcmp(ble.buffer, "") == 0) {
          // no data
          return;
        }

        // Subtracting 48 converts from ASCII char to an int
        int grip = ble.buffer[0] - 48;
        // Multiplying by 11 converts the range 0-9 to 0-99
        int pos = (ble.buffer[1] - 48) * 11;
        if (debug) {
          Serial.print(F("[Recv] ")); Serial.print(grip); Serial.print(", "); Serial.println(pos);
        }
        // Control Hand
        if (0 <= grip && grip <= FINGER_ROLL && 0 <= pos && pos <= 100) {
          gripMovement(grip, pos);
        }
        ble.waitForOK();
}

/**************************************************************************/
/*!
    @brief  Checks for user input (via the Serial Monitor)
*/
/**************************************************************************/
bool getUserInput(char buffer[], uint8_t maxSize)
{
  // timeout in 100 milliseconds
  TimeoutTimer timeout(100);

  memset(buffer, 0, maxSize);
  while( (!Serial.available()) && !timeout.expired() ) { delay(1); }

  if ( timeout.expired() ) return false;

  delay(2);
  uint8_t count=0;
  do
  {
    count += Serial.readBytes(buffer+count, maxSize);
    delay(2);
  } while( (count < maxSize) && (Serial.available()) );
  Serial.println("User Input Received.\n");
  return true;
}
