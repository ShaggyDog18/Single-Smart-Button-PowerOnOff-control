# Push-Button-On-Off-Control-ATtiny13/25/45/85
Smart Power ON/OFF Button: #173-ATTiny85-Push-Button-On-Off-control

Initial Development by Ralph S Bacon: https://youtu.be/S2y1oAVmxdA 

Reference to Ralph's project: https://github.com/RalphBacon/173-ATTiny85-Push-Button-On-Off-control

Creatively modified algorithm, introduced a Finite-state Machine and ported to ATtiny13.

Supported uControllers: ATtiny13/13A/25/45/85

Environment: MicroCore, 600kHz internal clock (also set up by fuses), BOD disabled, Micros enabled 
Size with both ExternalControl and Sleep features: 842 bytes / 32 bytes -> great for Attiny13 ! 
 
100% compatible at firmware level if compiled for ATtiny25/45/85!

Incompatible with the original author's solution at pins/PCB level because ATtiny13 and ATtiny25/45/85
have different pins for INT0 interrupt:

   ATtiny85/45/25 - PB2
   ATtiny13       - PB1

thus, I shifted POO button pin from PB2 to PB1 and 
                       LED pin from PB1 to PB0

                          ATtiny13A
	PIN 1 Reset             +-\/-+
	                PB5 - 1 |*   | 8 - VCC
	KILL pulse  <-> PB3 - 2 |    | 7 - PB2       -> debug only: serial out
	PowerOn/Off  <- PB4 - 3 |    | 6 - PB1(INT0) <- input: Power On/Off Button
	                GNG   4 |    | 5 - PB0       -> output:Power LED (optional)
	                        +----+


                        ATtiny25/45/85 
	PIN 1 Reset             +-\/-+
	                PB5 - 1 |*   | 8 - VCC
	KILL pulse  <-> PB3 - 2 |    | 7 - PB2 (INT0) <- input: Power On/Off Button  
	PowerOn/Off  <- PB4 - 3 |    | 6 - PB1        -> output:Power LED (optional)
	                GNG   4 |    | 5 - PB0        -> debug only: serial out
	                        +----+


Can be also compiled for Atmega328 just for test purpose (without sleep function).

# Modfications Log: 
 - Re-done re-code application algorithm using a Finite-state Machine Concept: https://en.wikipedia.org/wiki/Finite-state_machine
 - Ported to ATtiny13A microController; MicroCore environment
 - Added SLEEP_MODE_PWR_DOWN for extremely low power consumption (important for gadgets run on batteries) 
 - Added an option of initiating a power off procedure by main microController (uC) by 
   sending a "Power Off" request pulse to KILL_PIN bus 
 - Added a new feature: if PowerOnOff button is pressed and released followed with no confirmation from the main uC, 
   then it gets back to ON_STATE after a short tomeout and start-up delay; 
   Essentially, it becomes ready to the next attempt for Power Off or Emergency Shutdown 

# EasyEDA PCB links:
EasyEDA links to universal PCBs (compatible with ATtinny13/25/45/85):
 - Small SOT23 MOSFET AO3401A, up to 4A load: https://easyeda.com/Sergiy/smart-power-switch-attiny13
 - Dual SOIC-8 MOSFET AO4606,  up to 6A load: https://easyeda.com/Sergiy/smart-power-switch-attiny13_copy_copy
 - Powerful SOIC-8 MOSFET AO4407A, up to 10A: https://easyeda.com/Sergiy/smart-power-switch-attiny13_copy

**If you like and use this solution, please, consider making a small "cup of coffee" donation using [PayPal](https://paypal.me/shaggyDog18/3USD)**
