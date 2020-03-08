# Push-Button-On-Off-Control-ATtiny13/25/85
 Push-Button-On-Off-Control-ATtiny13/25/85
=====
Smart Power ON/OFF Button: #173-ATTiny85-Push-Button-On-Off-control
Developped by Ralph S Bacon: https://youtu.be/S2y1oAVmxdA 
Reference to the Ralph's project: https://github.com/RalphBacon/173-ATTiny85-Push-Button-On-Off-control

Creatively modified alghorytms, introduced Finite-state Machine and ported to ATtiny13 
by ShaggyDog18@gmail.com, MAR-2020

uController: ATtiny13A
Environment: MicroCore, 600kHz internal clock (also set up by fuses), BOD disabled, Micros enabled 
Size with both ExternalControll and Sleep features: 842 bytes / 32 bytes -> great for Attiny13 ! 
 
100% compatible at firmware level if compiled for ATtiny25/85!
Incompatible with the original author's solution at pins/PCB level because ATtiny13 and ATtiny25/85
have different pins for INT0 interrupt:
   ATtiny85 - PB2
   ATtiny13 - PB1
thus, I shifted POO button pin from PB2 to PB1 and 
                       LED pin from PB1 to PB0

                        ATtiny13A
PIN 1 Reset             +-\/-+
                PB5 - 1 |*   | 8 - VCC
 KILL pulse <-> PB3 - 2 |    | 7 - PB2       -> debug only: serial out
 PowerOn/Off <- PB4 - 3 |    | 6 - PB1(INT0) <- input: Power On/Off Button          
                GNG   4 |    | 5 - PB0       -> output:Power LED (optional)
                        +----+

                  ATtiny25 / ATtiny85 
PIN 1 Reset             +-\/-+
                PB5 - 1 |*   | 8 - VCC
 KILL pulse <-> PB3 - 2 |    | 7 - PB2 (INT0) <- input: Power On/Off Button          
 PowerOn/Off <- PB4 - 3 |    | 6 - PB1        -> output:Power LED (optional)
                GNG   4 |    | 5 - PB0        -> debug only: serial out
                        +----+

Can be also compiled for Atmega328 just for test purpose (without sleep function).


Modfications Log: 
 - Re-done re-code application algorithm using a Finite-state Machine Concept: https://en.wikipedia.org/wiki/Finite-state_machine
 - Ported to Attiny13A microController; use MicroCore environment
 - Added SLEEP_MODE_PWR_DOWN for extremely low power consumptoin (important for gadgets run on batteries) 
 - Added an option of initiate a power off procedure by Slave microController by 
   sending a request pulse to KILL_PIN 
 - Added a new feature: if PowerOnOff button is pressed and released followed with no confirmation from the main uC, 
   then it gets back to ON_STATE after a short tomeout and start-up delay; 
   Essentially, ready for the next attempt for power off or Emergency Shutdown 
