//@------------------------------ spin doctor ----------------- -*- C++ -*- --
// 
// This program implements an RPM and surface speed display for lathes,
// bandsaws etc.
//
// It expects a single magnet epoxied to a shaft or pulley, adjacent to a
// fixed hall-effect sensor.
//
// An interrupt is used to count pulses, and the pulse count is periodically 
// converted to the calculated revolutions per minute (averaged over the 
// interval since the previous calculation).
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


// set count_revs=1 to show total revolutions ever (useful for debugging
// calculations or intermittent sensor triggering)
// set count_revs=0 to show diameter
#define COUNT_REVS             0

#define INITIAL_DIAMETER      25
#define MAX_DIAMETER	    1000
#define INITIAL_BUTTON_DELAY 256
#define HOLD_BUTTON_DELAY    512
#define MIN_BUTTON_DELAY       2

#define HEARTBEAT_INTERVAL   500

#define MAX_REVS	     250
#define MAX_UPDATE_INTERVAL 6000


//@********************************** data ***********************************

volatile byte revs;  // number of revolutions counted since last calculation
volatile byte spinner;   // animated heartbeat cursor
volatile byte hold;

unsigned int  diameter_mm = INITIAL_DIAMETER;    // assumed workpiece diameter in millimetres
unsigned long rpm;                      // calculated RPM
unsigned long mpm;                      // calculated surface speed in m/m
unsigned long allrevs;                  // total number of revolutions ever counted
unsigned long last_update_time;         // time (in millis) of last recalculation
unsigned long last_heartbeat;
unsigned long last_button_poll;      // debouncing for button
unsigned int  button_delay = INITIAL_BUTTON_DELAY;

#define MILLIS_SINCE(when) (long)(millis() - (when))
//#define MILLIS_SINCE(when) (millis() - (when))

// initialize the LCD library with the numbers of the interface pins
//               RS,  RW, EN,D7,D6,D5,D4
//LiquidCrystal lcd(12, 11, 10, 7, 6, 5, 4);
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

//
//@******************************* functions *********************************

//@@---------------------------- update_display ------------------------------
//
// Update the display to show the current calculated RPM etc.
// 
void update_display(boolean clear=0)
{
  if (clear)
    lcd.clear();
  
  // 
  // Line 0: Print the RPM
  // 
  Serial.println(rpm, DEC);
  lcd.setCursor(0, 0);
  lcd.print(rpm);
  lcd.print(" r/m ");
  if (hold) lcd.print("HOLD");
  lcd.print("    ");

  // 
  // Line 1: Print the surface speed in metres/minute
  // (in debug mode, show the raw rev count and uptime instead)
  // 
  lcd.setCursor(0, 1);

#if COUNT_REVS
  lcd.print(allrevs);
  lcd.print(" revs. ut");
  lcd.print(millis()/1000);
#else
  // At an assumed diameter of diameter_mm, convert revoutions/min to tangential metres/min
  // As we're truncating result to whole metres, can use totally bogus approximation for pi=22/7
  mpm =  diameter_mm * 22 * rpm / 7000;
  lcd.print(mpm);
  lcd.print(" m/m @ d=");
  lcd.print((int)diameter_mm);
  lcd.print("mm  ");
#endif

}

//
//@@------------------------- handle_button_events ---------------------------
void handle_button_events() 
{
  if (MILLIS_SINCE(last_button_poll) < button_delay) {
    // still within debounce interval of last button event, do nothing
    return;
  }

  // since a button down event happened, set the debounce timer
  last_button_poll = millis();
  
  // 
  // Check the UP and DOWN buttons
  // 
  if (digitalRead(PIN_BTN_UP)==0) {
    if (diameter_mm < MAX_DIAMETER) ++diameter_mm;
    tone(PIN_SPKR,880,min(8,button_delay));
    update_display();
    // fall thru to delay update
  }
  else if (digitalRead(PIN_BTN_DOWN)==0) {
    if (diameter_mm > 1) --diameter_mm;
    tone(PIN_SPKR,440,min(8,button_delay));
    update_display();
    // fall thru to delay update
  }
  else if (digitalRead(PIN_BTN_MODE)==0) {
    // hold button active high
    hold = !hold;
    tone(PIN_SPKR,440*3,256);
    digitalWrite(PIN_LED_HOLD, hold);
    update_display();
    button_delay = HOLD_BUTTON_DELAY;
    return;
  }
  else {
    // No input, reset the debounce delay to 'slow' mode
    button_delay = INITIAL_BUTTON_DELAY;
    return;
  }

  // delay update - halve the button delay for consecutive events.
  // The longer you hold down a button the faster the button event repeats
  if (button_delay > MIN_BUTTON_DELAY) {
    button_delay = button_delay * 9 / 10;
  }
}

//
//@@------------------------------ heartbeat ---------------------------------

void heartbeat() 
{
  //static const char spinner_chars[4] = {'|','/','-','\\'}; // no good on .jp LCDs
  static const char spinner_chars[4] = {'.','o','O','o'}; // no good on .jp LCDs

  if (MILLIS_SINCE(last_heartbeat) < HEARTBEAT_INTERVAL) return;
  
  spinner = ++spinner % 4;
  digitalWrite(PIN_LED_DIAG,spinner%2);
  lcd.setCursor(15,0);
  lcd.print(spinner_chars[spinner]);

  last_heartbeat=millis();
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
  boolean clear;
  
  if ((elapsed < MAX_UPDATE_INTERVAL) && (revs < MAX_REVS)) { 
    // Not yet time to recalculate the RPM
    return;
  }
  clear = (allrevs==0); // clear whole screen first time

  if (!hold) {
    // do not update if HOLD mode
    rpm = 60000 * revs / elapsed;
  }
  allrevs += revs;    		// count total revs for testing

  revs = 0; 			// reset the rev counter for next sample interval
  last_update_time = millis();

  tone(PIN_SPKR,660,4);

  // Update the display 
  update_display(clear);
}

//@@------------------------------ sensor_isr --------------------------------
//
// Called once for every rising edge on pin2 (INT0).
//
// It turns out the Hall Effect sensor is really noisy, at least
// when doing 'wave a magnet past the sensor' tests.
//
// Ignore pulses within 10ms of another pulse.   
// This limits measurable revolutions to less than 100*60 = 6,000 RPM.
// 
void sensor_isr()
{
  static unsigned long last_interrupt = 0;
  if (MILLIS_SINCE(last_interrupt) > 10) {
    revs++;
    last_interrupt=millis();
  }
}

//@****************************** entry points *******************************
//@@-------------------------------- setup -----------------------------------
// 
// Configure the LCD and serial port.
// Attach an interrupt handler to pin 2
// Output a hello world message
// 
void setup() 
{
  static const char *version = "SpinDoctor v1.0";
  static const char *author = "unixbigot.id.au";
	
  // Set up the diagnostic LED
  pinMode(PIN_LED_DIAG, OUTPUT);
  digitalWrite(PIN_LED_DIAG, 1);

  // set up the LCD's number of columns and rows: 
  lcd.begin(8, 2);

  // Print a message to the LCD.
  lcd.print(version);
  lcd.setCursor(0, 1);
  lcd.print(author);
  
  // Configure the serial port
  Serial.begin(9600);
  Serial.print(version);
  Serial.print(' ');
  Serial.println(author);
  
  // Configure the button inputs
  pinMode(PIN_BTN_MODE, INPUT); // button from oatley board - active high no pullup required
  digitalWrite(PIN_BTN_MODE, 1);
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
  // Sensor is active-low so invert it.
  // 
  digitalWrite(PIN_LED_RUN,!digitalRead(PIN_SENSOR));

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
