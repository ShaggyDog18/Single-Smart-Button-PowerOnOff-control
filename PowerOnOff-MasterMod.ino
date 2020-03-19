// Initial development:
//   Smart Power ON/OFF Button: #173-ATTiny85-Push-Button-On-Off-control
// by Ralph S Bacon: https://youtu.be/S2y1oAVmxdA 
//   https://github.com/RalphBacon/173-ATTiny85-Push-Button-On-Off-control
//
// Creatively modified alghorytm, introduced a Finite-state Machine and ported to ATtiny13 
// by ShaggyDog18@gmail.com, MAR-2020
//
// uController: ATtiny13A
// Environment: MicroCore, 600kHz internal clock (also set up by fuses), BOD disabled, Micros enabled 
// 600kHz low_fuses=0x29
// 
// Size with both ExternalControl and Sleep features: 838 bytes / 32 bytes -> great for Attiny13 ! 
// 
// 100% compatible at firmware level if compiled for ATtiny25/45/85!
// Incompatible with the original author's solution at pins/PCB level because ATtiny13 and ATtiny25/45/85
// have different pins for INT0 interrupt:
//    ATtiny85 - PB2
//    ATtiny13 - PB1
// thus, I shifted POO button pin from PB2 to PB1 and 
//                        LED pin from PB1 to PB0
//
//
//                        ATtiny13A
// PIN 1 Reset             +-\/-+
//                 PB5 - 1 |*   | 8 - VCC
//  KILL pulse <-> PB3 - 2 |    | 7 - PB2       -> debug only: serial out
//  PowerOn/Off <- PB4 - 3 |    | 6 - PB1(INT0) <- input: Power On/Off Button          
//                 GNG   4 |    | 5 - PB0       -> output:Power LED (optional)
//                         +----+
//
//              ATtiny25 / ATtiny45 / ATtiny85 
// PIN 1 Reset             +-\/-+
//                 PB5 - 1 |*   | 8 - VCC
//  KILL pulse <-> PB3 - 2 |    | 7 - PB2 (INT0) <- input: Power On/Off Button          
//  PowerOn/Off <- PB4 - 3 |    | 6 - PB1        -> output:Power LED (optional)
//                 GNG   4 |    | 5 - PB0        -> debug only: serial out
//                         +----+
//
//============
// Can be also compiled for Atmega328 just for test purpose (without sleep function).
//============
// EasyEDA links to universal PCBs (compatible with ATtinny13/25/45/85):
// - Small MOSFET AO3401A, up to 4A load: https://easyeda.com/Sergiy/smart-power-switch-attiny13
// - Dual  MOSFET AO4606,  up to 6A load: https://easyeda.com/Sergiy/smart-power-switch-attiny13_copy_copy
// - Powerful MOSFET AO4407A,  up to 10A: https://easyeda.com/Sergiy/smart-power-switch-attiny13_copy
//============
//
/*
Modfications Log: 
 - Re-done re-code application algorithm using a Finite-state Machine Concept: https://en.wikipedia.org/wiki/Finite-state_machine
 - Ported to Attiny13A microController; use MicroCore environment
 - Added SLEEP_MODE_PWR_DOWN for extremely low power consumptoin (important for gadgets run on batteries) 
 - Added an option of initiate a power off procedure by Slave microController by 
   sending a request pulse to KILL_PIN 
 - Added a new feature: if PowerOnOff button is pressed and released followed with no confirmation from the main uC, 
   then it gets back to ON_STATE after a short tomeout and start-up delay; 
   Essentially, it becomes ready to the next attempt for Power Off or Emergency Shutdown 
 - Added __AVR_ATtiny25__ , __AVR_ATtiny45__ and __AVR_ATtiny85__ as valid options
*/

//--------------------
// CONFIGURATION
//--------------------
//#define SERIAL_DEBUG

// Allow an external KILL/"Power Off" request that can come from the main uController (may have a "Power Off" item in a menu)
// Performs an instant shut down implying that all necessary shut down processes are already performed by calling uC.
#define ALLOW_EXTERNAL_KILL_REQUEST

#define ENABLE_SLEEP_MODE   // for Attiny13/25/45/85 only for extra low power consumption during operation

//--------------------
// INCLUDE SECTION
//--------------------
#include "Arduino.h"
#include <avr/interrupt.h>
#include <avr/delay.h>

#ifdef ENABLE_SLEEP_MODE
  #include <avr/sleep.h>
#endif

#ifdef SERIAL_DEBUG
#if defined(__AVR_ATtiny13__) || defined(__AVR_ATtiny13A__) || defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__)|| defined(__AVR_ATtiny25__)
  #include <SendOnlySoftwareSerial.h>
#endif
#endif

//--------------------
// DEFINES
//--------------------
// INT0 is the only interrupt that can be configured for HIGH, LOW; others are PIN_CHANGE only
#if defined(__AVR_ATtiny13__) || defined(__AVR_ATtiny13A__) 
  #define INT_PIN PB1  // Power On/Off (POO) button
  #define PWR_LED PB0  // Power ON LED pin (will flash on shutdown)
#elif defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny25__) 
  #define INT_PIN PB2  // Power On/Off (POO) button 
  #define PWR_LED PB1  // Power ON LED  
#elif defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) 
  #define INT_PIN PD2  // INT0 interrupt pin -> Atmega 328 pin 2
  #define PWR_LED PB5  // Pin 13 with Build in LED
#else
  #error "Unsupported type of microController"
#endif

// Output LOW pin to keep MOSFET running
#define PWR_PIN PB4  // PINB4 -> Atmega 328 pin 12

// KILL communication bus: signal goes to main uC for shutdown confirmtion, 
// as well as incoming interrupt for extermal shut down request by main uC
#define KILL_PIN PB3  // PINB3 -> Atmega 328 pin 11

// ISR variables for POO Button
volatile bool togglePowerRequest = false;

#ifdef ALLOW_EXTERNAL_KILL_REQUEST
  // new feature - external Shutdown request by the main uController through KILL_PIN
  volatile bool externalShutDownRequest = false;
#endif

// Delay timer for switching ON/Off
unsigned long millisOnOffTime = 0;

// How long to hold down POO button to force turn off (mS)
#define KILL_TIME 4000  //mSec

// length of KILL pulse that to be sent to maim uC to request a KILL confirmation
#define KILL_PULSE 100  // mSec

// KILL time out - how long do we wait for KILL confirmation
#define KILL_TIMEOUT 1000  // mSec

// Minimum time before shutDown request accepted (mS)
#define MIN_ON_TIME 3000  // mSec

#define BUTTON_DEBOUNCE_TIME 10  //mSec

// Foorward declareations
void powerDownLedFlash();
void shutDownPower();
void enableInterruptForOnOffButton(void);
#ifdef ALLOW_EXTERNAL_KILL_REQUEST
  void enablePCInterruptForKillPin(void);
  void disablePCInterruptForKillPin(void);
#endif


//mySerial comms - plug in a USB-2-mySerial device to PB0 (pin #5) for Attiny85
#ifdef SERIAL_DEBUG
  #if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny25__)
    SendOnlySoftwareSerial Serial( PB0 );
  #elif defined(__AVR_ATtiny13__) || defined(__AVR_ATtiny13A__) 
	  SendOnlySoftwareSerial Serial( PB2 ); // did not test :-(
  #endif
#endif


// Define Finite-state Machine
enum stateMachine_t : byte {
  POWER_ON_PROCESS = 0,
  START_UP_DELAY,
  ON_STATE,
  SHUTDOWN_PROCESS
} stateMachine = POWER_ON_PROCESS;


//--------------------
//    SETUP
//--------------------
void setup() {
  #ifdef SERIAL_DEBUG
    Serial.begin(9600);
    Serial.println("Setup starts");
  #endif
  
  //keep the KILL command a WEAK HIGH untill we need it; listen to the external shut down request if allowed
  pinMode( KILL_PIN, INPUT_PULLUP );  // KILL pin to interface with main uController
  pinMode( INT_PIN, INPUT_PULLUP );  // Interrupt pin for Power On/Off button
  pinMode( PWR_PIN, OUTPUT );  // Gate of driving N-ch MOSFET
  pinMode( PWR_LED, OUTPUT );  // Power ON LED (optional)

  // initial state: switch on instantly - do that in POWER_ON_PROCESS state; no need for doing it here in setup()
  //bitSet( PORTB, PWR_PIN );
  //bitSet( PORTB, PWR_LED );
  
  #if defined(__AVR_ATtiny13__) || defined(__AVR_ATtiny13A__) || defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny25__)
    // For low power consumption: 
    ADCSRA &= ~_BV(ADEN); // set ADC OFF
    ACSR |= _BV(ACD);     // switch off Comparator
    #ifdef ENABLE_SLEEP_MODE 
      set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // set up sleep mode for Attiny13
    #endif

    #ifdef ALLOW_EXTERNAL_KILL_REQUEST
      //  Allow PCI - Pin Change Interrupt in general // Attiny13 DataSheet, Page 49
      GIMSK |= _BV(PCIE);
    #endif
  #elif defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) 
    #ifdef ALLOW_EXTERNAL_KILL_REQUEST
      // Allow PCI - pin change interrupt for PORTB
      PCICR |= _BV(PCIE0);  
    #endif
  #endif
} //-- end of setup()


#ifdef ALLOW_EXTERNAL_KILL_REQUEST
void enablePCInterruptForKillPin(void){ // enable PC Interrupt for input pin PB3  
  #if defined(__AVR_ATtiny13__) || defined(__AVR_ATtiny13A__)  || defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny25__) 
    PCMSK |= _BV( PCINT3 );  // KILL_PIN is at PB3
  #elif defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) 
    PCMSK0 |= _BV( PCINT3 ); // pin11, PCINT3 is in the PCMSK0; set PCI for pin 11
  #endif  
}

void disablePCInterruptForKillPin(void){ // disable PC Interrupt for input pin PB3 
  #if defined(__AVR_ATtiny13__) || defined(__AVR_ATtiny13A__)  || defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny25__)
    PCMSK &= ~_BV( PCINT3 );  // KILL_PIN is at PB3
  #elif defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) 
    PCMSK0 &= ~_BV( PCINT3 ); // pin11, PCINT3 is in the PCMSK0; disable PCI for pin 11
  #endif
}
#endif


void enableInterruptForOnOffButton(void){
  // The Arduino preferred syntax of attaching interrupt not possible for AtTiny13A 
  #if defined(__AVR_ATtiny13__) || defined(__AVR_ATtiny13A__)  || defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny25__) 
    // We want only on FALLING (not LOW, if possible) set bits 1:0 to zero
    // MCUCR &= ~(_BV(ISC01) | _BV(ISC00)); // LOW
    // GIFR  |= bit(INTF0);  // clear any pending INT0
    // GIMSK |= bit(INT0);  // enable INT0
    // attachInterrupt(0, btnISR, FALLING);
    MCUCR = (MCUCR & ~((1 << ISC00) | (1 << ISC01))) | 2 << ISC00;

    // in the General Interrupts Mask Register
    GIMSK |= _BV(INT0); // enable INT0 interrupt
   
   #elif defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) 	// to test on Atmega 328 UNO, Nano or Pro Mini boards
     attachInterrupt( digitalPinToInterrupt(INT_PIN), myISR, FALLING );
   #endif
}


//------
//
//------
// Standard interrupt vector (only on LOW / FALLING )
#if defined(__AVR_ATtiny13__) || defined(__AVR_ATtiny13A__) || defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny25__) 
  ISR( INT0_vect ) {
#elif defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)  // ATMega 328 for testing
  void myISR(void) {
#endif
  #ifdef ALLOW_EXTERNAL_KILL_REQUEST
    disablePCInterruptForKillPin();
  #endif

  #ifdef ENABLE_SLEEP_MODE
  #if defined(__AVR_ATtiny13__) || defined(__AVR_ATtiny13A__) || defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny25__) 
    // wake up and disable sleep
    sleep_disable();
  #endif
  #endif
  
  #ifdef SERIAL_DEBUG
    Serial.println("ISR for button is called");
  #endif
   
  // keep track of when we entered this routine
  static volatile unsigned long prevMillis = 0;  
  // POO button is pushed. Check for debounce 10mS 
  if( millis() > prevMillis + BUTTON_DEBOUNCE_TIME ) {
    // disble itself: disable INT0 ext interrupt for POO button
    // untill shutDown completed (when we can power up again )
    #if defined(__AVR_ATtiny13__) || defined(__AVR_ATtiny13A__) || defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny25__) 
      GIMSK &= ~_BV(INT0);  // disable INT0 interrupt  
    #elif defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) 
      detachInterrupt(0);
    #endif    
    
    #ifdef SERIAL_DEBUG
      Serial.println( "ISR is OK" );
    #endif   
    prevMillis = millis();
    togglePowerRequest = true;
  }
}  



#ifdef ALLOW_EXTERNAL_KILL_REQUEST
ISR(PCINT0_vect) {  // PCI vect0 is for PCIinterrupt for PortB
  // disable itself; disable PC interrupt for pin PB3  
  #if defined(__AVR_ATtiny13__) || defined(__AVR_ATtiny13A__)  || defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny25__)
    PCMSK &= ~_BV( PCINT3 );  // KILL_PIN
  #elif defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) 
    PCMSK0 &= ~_BV( PCINT3 ); // pin11, PCINT3 is in the PCMSK0; disable PCINT3 to trigger an interrupt on state change 
  #endif
    
  // disble INT0 ext interrupt for POO button
  #if defined(__AVR_ATtiny13__) || defined(__AVR_ATtiny13A__) || defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny25__) 
    GIMSK &= ~_BV(INT0);  // disable INT0 interrupt   
  #elif defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) 
    detachInterrupt(0);
  #endif
  
  #ifdef ENABLE_SLEEP_MODE
  #if defined(__AVR_ATtiny13__) || defined(__AVR_ATtiny13A__) || defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny25__)  
    // wake up, do not sleep any more...-> shut down :-)
    sleep_disable();
  #endif 
  #endif

  #ifdef SERIAL_DEBUG
    Serial.println("ISR:Shutdown request from main uController" );
  #endif
  externalShutDownRequest = true; 
}
#endif


//-----------
//    LOOP
//-----------
void loop() {
  switch( stateMachine ) {
    case POWER_ON_PROCESS:
      bitSet( PORTB, PWR_PIN );  // Power pin to MOSFET gate (HIGH to turn on N-ch; LOW to turn on P-ch MOSFET)
      bitSet( PORTB, PWR_LED );  // Turn on Power LED to indicate we are up and running
      #ifdef SERIAL_DEBUG
        Serial.println( "Power is ON" ); 
        Serial.println( "ShutDown requets not yet accepted" );
      #endif
      millisOnOffTime = millis();
      stateMachine = START_UP_DELAY;
      break;
      
    case START_UP_DELAY:
      if( millis() > millisOnOffTime + MIN_ON_TIME ) {  // only after a few seconds we do allow a shutdown request to come
        #ifdef SERIAL_DEBUG
          Serial.println("Shutdown request is now allowed." );
        #endif
        
        // enable Interrupts
        enableInterruptForOnOffButton();
        #ifdef ALLOW_EXTERNAL_KILL_REQUEST  
          enablePCInterruptForKillPin();
        #endif
        stateMachine = ON_STATE;

        #ifdef ENABLE_SLEEP_MODE
        #if defined(__AVR_ATtiny13__) || defined(__AVR_ATtiny13A__) || defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny25__) 
          // go to sleep
          sleep_enable();
          sleep_cpu();   
        #endif
        #endif
      }
      break;
      
    case ON_STATE:  // waiting for shut down request from either of channels: the POO button or request from main uC, if allowed
      if( togglePowerRequest ) {	// shut down initiated by POO button
        #ifdef SERIAL_DEBUG
          Serial.println( "Shutdown request by POO Button" );
          _delay_ms(1000); // for debug only
        #endif

        // Send a KILL command, a LOW 100mS pulse, to the main microController (uC)
        #ifdef SERIAL_DEBUG
          Serial.println("Sent KILL message to main uC for confirmation" );
        #endif
        pinMode( KILL_PIN, OUTPUT );        
        bitClear( PORTB, KILL_PIN );
        _delay_ms( KILL_PULSE );
        bitSet( PORTB, KILL_PIN );
        
        // now listen for confirmation from main uC
        pinMode( KILL_PIN, INPUT_PULLUP );

        millisOnOffTime = millis();
        stateMachine = SHUTDOWN_PROCESS;
      }
      
      #ifdef ALLOW_EXTERNAL_KILL_REQUEST
      if( externalShutDownRequest ) {  // shut down initiated/requested by main uC
        #ifdef SERIAL_DEBUG
          Serial.println( "Shutdown request from main uC" );
          _delay_ms(1000); // for debug only
        #endif
        stateMachine = SHUTDOWN_PROCESS;
      }
      #endif  
      break; 
      
    case SHUTDOWN_PROCESS:
      powerDownLedFlash();  // Flash the power LED to indicate we are in shutdown process
      if( togglePowerRequest ) {  // shut down was requestd by pressing POO button
        if( !digitalRead(KILL_PIN) ) {  // KILL pin is LOW -> confirmation pulse received 
                                        // main uC confirmed to KILL power
          #ifdef SERIAL_DEBUG
            Serial.println("KILL confirmation received");
          #endif
          
          // wait for POO button to be released or we will start up again!
          while( !digitalRead(INT_PIN) ) {  //POO is LOW, i.e. pushed 
            #ifdef SERIAL_DEBUG
              Serial.println( "Waiting for POO button to be released" );
            #endif
            _delay_ms(BUTTON_DEBOUNCE_TIME); 
          }
          shutDownPower(); //shut off the power (instant off)
          
        } else { // no confirmation from main uC is received; KILL_PIN is HIGH
           #ifdef ALLOW_EXTERNAL_KILL_REQUEST  
             enablePCInterruptForKillPin();
           #endif
           if( digitalRead(INT_PIN) ) { // POO button was released, but no confirmation from the main uC; so wait for Timeout; 
                                        // may be break out by confirmation anytime before timeout
             if( millis() > millisOnOffTime + KILL_TIMEOUT ) { // timeout and back to ON_STATE
               #ifdef SERIAL_DEBUG
                 Serial.println( "No confirmation from main uC; 1 sec KILL timeout; get back to ON state" );
               #endif
               togglePowerRequest = false;
               millisOnOffTime = millis();
               stateMachine = START_UP_DELAY;
               break;
             }
           } else { // POO button is still pressed, not released
              // EMERGENCY SHUTDOWN - long press on Power ON/OFF button
              if( millis() > millisOnOffTime + KILL_TIME ) {
                // is POO button pressed long enogh to shutdown power without confirmation from the main uC?
                // may be break out by confirmation anytime before timeout
                #ifdef SERIAL_DEBUG
                  Serial.println( "Emergency shutdown is detected" );
                  _delay_ms(2000); // debug only
                #endif
                shutDownPower(); //shut off the power (instant off)
              }
           }
        }
      }
      
      #ifdef ALLOW_EXTERNAL_KILL_REQUEST
      if( externalShutDownRequest ) { // immediate shut down, request was received from the main uC
        #ifdef SERIAL_DEBUG
          Serial.println( "Shuting down; requested by main uC" );
          _delay_ms(2000); // debug only
        #endif
        shutDownPower(); //shut off the power (instant off)
      }
      #endif 
       
      break;
  } //switch( stateMachine )
} //--- end of loop()



// flash Power LED to indicate shutting down.
void powerDownLedFlash() {
  static unsigned long flashMillis = millis();
  if( millis() > flashMillis + 200 ) {
    PORTB ^= _BV( PWR_LED ); // have to invert LED twice in order to keep it ON after exiting the function
    _delay_ms(50);
    PORTB ^= _BV( PWR_LED );
    flashMillis = millis();  
  }
}

void shutDownPower() {
  // Turn off power on LED and MOSFET
  // Serial.println( "Power goes off" );
    bitClear( PORTB, PWR_PIN );
    _delay_ms(5000); // delay to allow power off, 
	                   // need it in case of large decupling capacitors that can still provide energy after MOSFET cuts the line 
    // no life after that point :-)
}
