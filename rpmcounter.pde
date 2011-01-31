//@------------------------------ rpm counter ----------------- -*- C++ -*- --
// 
// This program implements an RPM and surface speed display for lathes,
// bandsaws etc.
//
// It expects a single magnet epoxied to a shaft or pulley, adjacent to a
// fixed hall-effect sensor.
//
// An interrupt is used to count pulses, and the pulse count is periodically 
// converted to the calculated revolutions per minute since the previous
// calculation.
//
// A diameter value is maintained (changeable via two buttons) and the 
// linear speed (in surface metres/minute) is also calculated at that
// diameter.
//
// This allows the surface speed of a workpiece or tool to be calculated.
// 
#include <LiquidCrystal.h>

//@******************************* constants *********************************

#define PIN_SENSOR      2

#define PIN_LCD_RS       3
#define PIN_LCD_EN       4
#define PIN_LCD_D4       5
#define PIN_LCD_D5       6
#define PIN_LCD_D6       7
#define PIN_LCD_D7       8

#define PIN_SPKR         9

#define PIN_UNUSED_10   10
#define PIN_UNUSED_11   11
#define PIN_UNUSED_12   12

#define PIN_LED_DIAG    13

#define PIN_BTN_MODE    A0
#define PIN_BTN_UP      A1
#define PIN_BTN_DOWN    A2

#define PIN_LED_HOLD    A3
#define PIN_LED_RUN     A4
#define PIN_UNUSED_A5   A5


// set count_revs to show total revolutions ever (useful for debugging
// calculations or intermittent sensor triggering)
#define COUNT_REVS             1

#define INITIAL_DIAMETER      25
#define INITIAL_BUTTON_DELAY 512
#define MIN_BUTTON_DELAY       8

#define HEARTBEAT_INTERVAL   500

#define MAX_REVS	     250
#define MAX_UPDATE_INTERVAL 6000


//@********************************** data ***********************************

volatile byte revs;  // number of revolutions counted since last calculation
volatile byte diag_led;   // current value of the diagnostic LED output
byte diameter_mm = INITIAL_DIAMETER;    // assumed workpiece diameter in millimetres
unsigned long rpm;                      // calculated RPM
unsigned long mpm;                      // calculated surface speed in m/m
unsigned long allrevs;                  // total number of revolutions ever counted
unsigned long last_update_time;         // time (in millis) of last recalculation
unsigned long last_heartbeat;
unsigned long last_button_event;      // debouncing for button
byte button_delay = INITIAL_BUTTON_DELAY;

#define MILLIS_SINCE(when) (long)(millis()-(when))

// initialize the LCD library with the numbers of the interface pins
//               RS,  RW, EN,D7,D6,D5,D4
//LiquidCrystal lcd(12, 11, 10, 7, 6, 5, 4);
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

//
//@******************************* functions *********************************

//
//@@------------------------- handle_button_events ---------------------------
void handle_button_events() 
{
  if (MILLIS_SINCE(last_button_event) < button_delay) {
    // still within debounce interval of last button event, do nothing
    return;
  }
  
  if (digitalRead(PIN_BTN_UP)==0) {
    ++diameter_mm;
    update_display();
    // fall thru to delay update
  }
  else if (digitalRead(PIN_BTN_DOWN)==0) {
    --diameter_mm;
    update_display();
    // fall thru to delay update
  }
  else {
    // No input, reset the debounce delay to 'slow' mode
    button_delay = INITIAL_BUTTON_DELAY;
    return;
  }

  // since a button down event happened, set the debounce timer
  last_button_event = millis();

  // delay update - halve the button delay for consecutive events.
  // The longer you hold down a button the faster the button event repeats
  if (button_delay > MIN_BUTTON_DELAY) {
    button_delay <<= 2;
  }
}

//@@------------------------------- flipled ----------------------------------
//
// Toggle the state of the diagnostic LED
//
void flipled() {
  digitalWrite(PIN_LED_DIAG,diag_led=!diag_led);
}
//
//@@------------------------------ heartbeat ---------------------------------

void heartbeat() {
  if (MILLIS_SINCE(last_heartbeat) < HEARTBEAT_INTERVAL) return;
  
  flipled();
  last_heartbeat=millis();
  lcd.setCursor(15,0);
  lcd.print(diag_led?'o':'.');
}


//@@---------------------------- recalculate_rpm -----------------------------
// 
// Every 6 seconds (or less*) output the mean rotational speed over that
// interval.   
//
// * At 2500RPM the counter will be at 250 after 6 seconds.   If the
// speed is higher than this, trip at 250 anyway (merely results in
// more frequent updating).   The intended application is unlikely
// to ever go above 4000RPM.
//
// At the application's lower bound of about 60RPM, 6 seconds is only 6 revs, 
// so speed will only be accurate within about 15%. 
//
// TODO consider increasing timebase when low-revs detected, to
// count revolutions over 12s instead.
//
// 
void recalculate_rpm() 
{
  long elapsed = MILLIS_SINCE(last_update_time);
  
  if ((elapsed < MAX_UPDATE_INTERVAL) && (revs < MAX_REVS)) { 
    // Not yet time to recalculate the RPM
    return;
  }
  
  rpm = 60000 * revs / elapsed;
  allrevs += revs;    		// count total revs for testing

  revs = 0; 			// reset the rev counter for next sample interval
  last_update_time = millis();

  // Update the display 
  update_display();
}

//@@---------------------------- update_display ------------------------------
//
// Update the display to show the current calculated RPM etc.
// 
void update_display()
{
  // 
  // Line 0: Print the RPM
  // 
  Serial.println(rpm, DEC);
  lcd.setCursor(0, 0);
  lcd.print(rpm);
  lcd.print(" r/m    ");

  // 
  // Line 1: Print the surface speed in metres/minute
  // (in debug mode, show the raw rev count and uptime instead)
  // 
  lcd.setCursor(0, 1);
  lcd.print(" ");

#ifdef COUNT_REVS
  lcd.print(allrevs);
  lcd.print(" revs. ut");
  lcd.print(millis()/1000);
#else
  // At an assumed diameter of diameter_mm, convert revoutions/min to tangential metres/min
  // As we're truncating result to whole metres, can use totally bogus approximation for pi=22/7
  mpm =  diameter_mm * 22 * rpm / 7000;
  lcd.print(mpm);
  lcd.print(" m/m      ");
#endif

}

//@@------------------------------ sensor_isr --------------------------------
//
// Called once for every rising edge on pin2 (INT0).
// No debouncing, do that in hardware if necessary 
// 
void sensor_isr()
{
  revs++;
}

//@****************************** entry points *******************************
//@@-------------------------------- setup -----------------------------------
// 
// Configure the LCD and serial port.
// Attach an interrupt handler to pin 2
// Output a hello world message
// 
void setup() {
  // Set up the diagnostic LED
  pinMode(PIN_LED_DIAG, OUTPUT);
  flipled();

  // set up the LCD's number of columns and rows: 
  lcd.begin(8, 2);

  // Print a message to the LCD.
  lcd.print("hello,");
  lcd.setCursor(0, 1);
  lcd.print("world!");
  
  // Configure the serial port
  Serial.begin(9600);
  Serial.println("rpmcounter 1.0");
  
  // Configure the button inputs
  pinMode(PIN_BTN_MODE, INPUT); // button from oatley board - active high no pullup required
  pinMode(PIN_BTN_UP, INPUT);   // up/down buttons are active low, no pullup required
  digitalWrite(PIN_BTN_UP, 1);
  pinMode(PIN_BTN_DOWN, INPUT);
  digitalWrite(PIN_BTN_DOWN, 1);

  // Configure the (other) LED pins
  pinMode(PIN_LED_HOLD, OUTPUT);
  digitalWrite(PIN_LED_HOLD, 0);
  pinMode(PIN_LED_RUN, OUTPUT);
  digitalWrite(PIN_LED_RUN, 0);

  // Configure sensor input - using interrupt handler
  pinMode(PIN_SENSOR, INPUT);
  digitalWrite(PIN_SENSOR, 1); // pullup
  attachInterrupt(0, sensor_isr, FALLING);

  digitalWrite(PIN_LED_RUN, 1); // green light
}

//@@--------------------------------- loop -----------------------------------
// 
// Check the accumulated cound elapsed 
//
// 
void loop()
{
  // Toggle the diagnostic LED every 500ms
  heartbeat();

  // 
  // Reflect the sensor state in the RUN LED
  // 
  digitalWrite(PIN_LED_RUN,digitalRead(PIN_SENSOR));

  // 
  // Check the value of the revolutions-since-last-recalculation 
  // 
  recalculate_rpm();

  // 
  // Handle the plus/minus diameter change buttons.
  // Ignore button input if there has been a button event within last {button_delay} ms
  // 
  handle_button_events();
}


//@@----------------------------- end of file --------------------------------
