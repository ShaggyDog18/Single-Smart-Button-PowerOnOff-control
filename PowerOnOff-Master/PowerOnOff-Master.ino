// Power ON Slave - Attiny85
//
// PIN 1 Reset
//
//          +-\/-+
//  PB5 - 1 |*   | 8 - VCC
//  PB3 - 2 |    | 7 - PB2 (PCINT2)
//  PB4 - 3 |    | 6 - PB1
//  GNG   4 |    | 5 - PB0
//          +----+
//
// Small and fast
//
// 1016 bytes (99%)/1024; 32 bytes/64 (50%) with standard functions
// 990 (96%) with _delay_ms() instead of delay();
// 970 (94%) if no digitalWrite
// 964 (94%) if no digitalRead
//
// digitalWrite <=> setBit/clearBit
// digitalRead  <=> (PINB & (1<<PINB3))
//
//
//#define __AVR_ATtiny85__
//#define __AVR_ATtiny13__

//#define COMPACT_CODE
//#define SERIAL_DEBUG

#include "Arduino.h"
#include <avr/interrupt.h>
#include <avr/delay.h>
#ifdef SERIAL_DEBUG
  #include <SendOnlySoftwareSerial.h>
#endif

// Interrupt pin PB2 aka PCINT2 aka physiacal pin #7
// this is the only pin  we can configure for HIGH,LOW interrupt; others are PIN_CHANGE only
#define INT_PIN PB2

// Output LOW pin to keep MOSFET running
#define PWR_PIN PB4 // PINB4

// Power ON LED pin (will flash on shutdown) pin 6
#define PWR_LED PB1 // PINB1

// KILL interrupt for extermal MAIN microcontroller uC
#define KILL PB3    // PINB3

#define BUTTON_DEBOUNCE_TIME 50 	//mSec

// ISR variables
volatile bool togglePowerRequest = false;
bool shutDownAllowed = false;

// State of output ON/OF
bool powerIsOn = false;
bool shutDownInProgress = false;

// When did we switch ON (so we can delay switching off)
unsigned long millisOnTime = 0;

// How long to hold down button to force turn off (mS)
#define KILL_TIME 5000

// Minimum time before shutDown request accepted (mS)
#define MIN_ON_TIME 5000

// When did we press and hold the off button
unsigned long millisOffTime = 0;

// Foorward declareations
void powerDownLedFlash();
void shutDownPower();

//mySerial comms - plug in a USB-2-mySerial device to PB0 (pin #5)
#ifdef SERIAL_DEBUG
  SendOnlySoftwareSerial mySerial( PB0 );
#endif

//--------------------
//    SETUP
//--------------------
void setup() {
  #ifdef SERIAL_DEBUG
    mySerial.begin(9600);
    mySerial.println("Setup starts");
  #endif
  
  //keep the KILL command a WEAK HIGH untill we need it
  pinMode( KILL, INPUT_PULLUP );

  // keep the MOSFET switched off initially
  pinMode( PWR_PIN, OUTPUT );
  // Power ON LED
  pinMode( PWR_LED, OUTPUT );
  
  #ifdef COMPACT_CODE
    bitClear( PORTB, PWR_PIN );
    bitClear( PORTB, PWR_LED );
  #else
    digitalWrite( PWR_PIN, LOW);
    digitalWrite( PWR_LED, LOW );
  #endif

  // Interrupt pin when we press power pin again
  pinMode( INT_PIN, INPUT_PULLUP );
	// The Arduino preferred syntax of attaching interrupt not possible
	// attachInterupt(digitalPinToInterrupt(INT_PIN), PWR_PIN_ISR, LOW);

	// We want only on FALLING (not LOW, if possible) set bits 1:0 to zero
	// MCUCR &= ~(_BV(ISC01) | _BV(ISC00)); // LOW
	// GIFR |= bit(INTF0);  // clear any pending INT0
	// GIMSK |= bit(INT0);  // enable INT0
	// attachInterrupt(0, btnISR, FALLING);
	MCUCR = (MCUCR & ~((1 << ISC00) | (1 << ISC01))) | 2 << ISC00;

	// Equivalent to 1 << (6) That is, set bit 6 for ext interrupts
	// in the Gene3ral Interrupts Mask Register
	// GIMSK |= _BV(INT0); equivalent to
	//GIMSK |= 0b01000000;
	GIMSK |= (1 << INT0);

	/* You can also turn on pin CHANGE interrupts on selected pin(s) by
	 * setting the pin in the bit pattern:
	 *
	 * // turn on interrupts on pins PB1, PB2, & PB4
	 * PCMSK = 0b00010110;
	 */

  // Time to Power up
  togglePowerRequest = true;
}


// Standard interrupt vector (only on LOW / FALLING )
ISR( INT0_vect ) {
  // keep track of when we entered this routine
  static volatile unsigned long prevMillis = 0;

  #ifdef SERIAL_DEBUG
    mySerial.println("ISR is called");
  #endif
  // Power on button is pushed. Check for debounce 10mS 
  if( millis() > prevMillis + BUTTON_DEBOUNCE_TIME ) {
    togglePowerRequest = true;
    #ifdef SERIAL_DEBUG
      mySerial.println( "ISR is OK" );
    #endif   
    prevMillis = millis();
  }
}  


//-----------
//    LOOP
//-----------
void loop() {
  
  // only after a few seconds do we allow shutdown request
  if( (millis() > millisOnTime + MIN_ON_TIME) && powerIsOn ) {
    if( !shutDownAllowed ) {
      shutDownAllowed = true;
      #ifdef SERIAL_DEBUG
        mySerial.println("Shutdown request now accepted." );
      #endif
    }
  }

  /*    
  // ShaggyDog MAR-2020
  // Power Off request from Main uC- a short LOW pulse, 10 mS - active by going LOW; may have a Power off option in the menu
  if( !digitalRead( KILL ) ) {
    togglePowerRequest = true;
    #ifdef SERIAL_DEBUG
      mySerial.println("Shutdown request arrived from main uController" );
    #endif
    while( !digitalRead( KILL ) ); // waiting for KILL to get HIGH again
  }
*/

  // button was pressed and no shut down already running
  if( togglePowerRequest && ! shutDownInProgress ) {
    // Is the power currently ON?
    if( powerIsOn ) {
      if( shutDownAllowed ) {
        #ifdef SERIAL_DEBUG
          mySerial.println( "Shutdown Request Received." );
        #endif
        shutDownInProgress = true;

        // Send 100mS KILL command to external/main microProcessor (uP)
        pinMode( KILL, OUTPUT );
        #ifdef COMPACT_CODE
          bitClear( PORTB, KILL );
        #else
          digitalWrite( KILL, LOW );
        #endif
        #ifdef SERIAL_DEBUG
          mySerial.println("Sent SHUTDOWN message to external uC" );
        #endif
        _delay_ms(100);
        #ifdef COMPACT_CODE
          bitSet( PORTB, KILL );
        #else
          digitalWrite( KILL, HIGH );
        #endif

        
        // now we wait for confirmation from main uC
        pinMode( KILL, INPUT_PULLUP );

        // Kill INT0 ext interrupt as we don't need it again
        // untill shutDown completed (when we can power up again )
        #if defined(__AVR_ATtiny13__) || defined(__AVR_ATtiny13A__) 
          GIMSK = 0b00000000;
        #else
          detachInterrupt(0);
        #endif
      }
    } else {  // power on process
      // Power pin to MOSFET gate (HIGH to turn on N-ch; LOW to turn on P-ch MOSFET)(
      #ifdef COMPACT_CODE
        bitSet( PORTB, PWR_PIN );
        // Turn on Power LED to indicate we are up and running
        bitSet( PORTB, PWR_LED );
      #else
        digitalWrite( PWR_PIN, HIGH );
        digitalWrite( PWR_LED, HIGH );
      #endif
      #ifdef SERIAL_DEBUG
        mySerial.println( "Power is ON" );
        // accept power OFF shutdown request in 5 sec
        mySerial.println( "ShutDown requets not yet accepted" );
      #endif
      
      millisOnTime = millis();

      // set the flag rhat power is ON now
      powerIsOn = true;
    }
    // clear the button press state
    togglePowerRequest = false;
  }
      
  //if we shutting down request do not accept further process
  // and wait for config from main uC before killing power
  if( shutDownInProgress ){
    // Flash the power LED to indicate shutdown request sent
    powerDownLedFlash();

    // if we get the KILL command from main uC shutdown power 
    if( digitalRead( KILL ) == LOW ) {
      // main uC requested/confirmed KILL power
      #ifdef SERIAL_DEBUG
        mySerial.println("Kill command received");
      #endif
      // shut the power off
      shutDownPower();
    }
    
    // if the power switch is held down long enough KILL anyway
    if( digitalRead(INT_PIN) == HIGH) {
      // button was relased, so reset start time
      millisOffTime = 0;
    } else {
      // if we haven't captured start of long press , do it now
      if( millisOffTime == 0 ) millisOffTime = millis();

      // button pressed long enogh for KILL power?
      if( millis() > millisOffTime + KILL_TIME ) {
        #ifdef SERIAL_DEBUG
          mySerial.println( "Emergebcy KILL detected" );
		  _delay_ms(2000); // debug only
        #endif
        //shut off the power (instant off)
        shutDownPower(); 
      }
    }
  }
}


// flash Power LED to indicate shutting down.
void powerDownLedFlash() {
  static unsigned long flashMillis = millis();
  if( powerIsOn ) {
    if( millis() > flashMillis + 200 ) {
      #ifdef COMPACT_CODE
		PORTB ^= _BV( PWR_LED );
      #else
        digitalWrite( PWR_LED, !digitalRead( PWR_LED )); 
      #endif
      flashMillis = millis();  
    }
  }
}

void shutDownPower() {
  // reset all flags to initial state
  /*
  powerIsOn = false;
  shutDownAllowed = false;
  shutDownInProgress = false;
  millisOnTime = 0;
  */

  // wait for button to be released or we will start up again!
  while( digitalRead(INT_PIN) == LOW ) {
    #ifdef SERIAL_DEBUG
      mySerial.println( "Waiting for button release" );
    #endif
    //delay(10);
    _delay_ms(BUTTON_DEBOUNCE_TIME);

  }

  // Turn off power on LED and MOSFET
  //mySerial.println( "Power goes off" );
  #ifdef COMPACT_CODE
    bitClear( PORTB, PWR_PIN );
  #else
    digitalWrite( PWR_PIN, LOW );
  #endif
  //digitalWriite( PWR_LED, LOW );
}
