//@------------------------------ spin doctor ----------------- -*- C++ -*- --
// 
// This program implements a general purpose tachometer/speedometer.
//
// It can be used to retrofit digital readout to  lathes and bandsaws etc.
// It is also usable as a bike computer or vehicle speedometer.
//
// It expects a revolutions sensor consisting of a moving magnet affixed to a 
// shaft or pulley, passing a fixed hall-effect sensor once per revolution.
// Optical sensing using a transmissive or reflective IR sensor is also possible.
//
// An interrupt is used to count pulses, and the pulse count is periodically 
// converted to the calculated revolutions per minute (averaged over the 
// interval since the previous calculation).  
//
// A diameter value is maintained (changeable via two buttons) and the 
// linear speed (in surface metres/minute) is also calculated at that
// diameter.
//
// This allows the surface speed of a workpiece, tool or vehicle to be calculated,
// giving you a direct readout in surface metres per minute 
// (or feet per minute if you are from one of Those Places).   In vehicle
// mode a speedometer/odometer reading in kmh or mph is displayed.
// 
//@********************************** BUGS ***********************************
//
// * Primitive software debouncing of noisy sensor limits high speed sensing
//
//@********************************** TODO ***********************************
// 
// * refactor display code into small subroutines
// * use short-press for mode change, long-press for HOLD
// * support 4way or 5way hat switches
// * support a second sensor (for engine/crank revs) (for bike computers etc.)
// * store preferences (display mode, last diameter, units) in EEPROM
// * accept configuration commands over RS232 (mode set, change units etc)
// * make the RS232 output parsable
// * battery voltage sensing

#include <LiquidCrystal.h>

//@******************************* CONSTANTS *********************************

// 
//@@ Pin Assignments
// 
#define PIN_RX		 0
#define PIN_TX           1

#define PIN_SENSOR       2

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


// 
// Configuration constants
// 
// Don't change the value of PI unless you are from another universe.
//
#define PI 355/113

// Control auto-repeat and debounce on buttons
#define INITIAL_BUTTON_DELAY 256
#define HOLD_BUTTON_DELAY    512
#define MIN_BUTTON_DELAY       2

// Speed of the heartbeat display
#define HEARTBEAT_INTERVAL   500

// Display is updated whenever MAX_REVS are counted or MAX_UPDATE_INTERVAL has elapsed
#define MAX_REVS	     250
#define MAX_UPDATE_INTERVAL 6000

// A diameter value is maintained for converting angular speed to linear speed
//
// Metric diameter is stored as whole millimetres.
// Imperial diameter is stored as "whiskers" (1 whisker = 1/40 inch, or 25 thousandths of an inch)
//
// Buttons can be used to change the current diameter.  For small diameters
// (<= than the initial diameter of 50mm/2") each button press changes the
// entered size by 1 unit; at larger diameters, each button press changes by
// 5 units (5mm or 1/8 inch).
//

#define INITIAL_DIAMETER_MET   50
#define INITIAL_DIAMETER_IMP   80
#define MAX_DIAMETER_MET     1000
#define MAX_DIAMETER_IMP     1440

// 
// Display configuration
//
// The ubiquitous 16x2 LCD is used for output.
//
// The first line is always the RPM
// The second line is changeable according to the mode flag:
// 
// DISPLAY_MODE_TANGENTIAL  - surface speed at entered diameter
// DISPLAY_MODE_ODOMETER    - total revolutions and distance
// DISPLAY_MODE_SPEEDOMETER - speed and distance
//
// In each mode either metric or Imperial measures are used depending on the
// units flag.
//
#define DISPLAY_MODE_TANGENTIAL  0
#define DISPLAY_MODE_ODOMETER    1
#define DISPLAY_MODE_SPEEDOMETER 2

#define UNITS_MET  0
#define UNITS_IMP  1

#define DEFAULT_UNITS 	UNITS_MET

// Changing DEFAULT_MODE to ODOMETER is useful for testing 
#define DEFAULT_MODE    DISPLAY_MODE_TANGENTIAL


//@********************************** DATA ***********************************

// 
// Configuration information (as stored in EEPROM)
// 
struct _flags {
  byte units: 1;
  byte hold: 1;
  byte mode: 2;
  byte _unused_flags: 4;
} flags = { DEFAULT_UNITS, 0, DEFAULT_MODE, 0 };

struct _state {
  byte spinner: 2;
  byte unused_state: 6;
} state;
  
// 
// Other RAM variables
// 
unsigned int button_delay  = INITIAL_BUTTON_DELAY; // repeat interval for button presses

volatile byte revs;                     // number of revolutions counted in this sample window
unsigned long allrevs;                  // total number of revolutions ever counted
unsigned long last_update_time;         // time (in millis) of start of sample window
unsigned long last_sensor_event;        // time (in millis) of most recent sensor event
unsigned int  diameter;			// assumed workpiece diameter in mm or whiskers (1/40")

unsigned long rpm;                      // calculated RPM
unsigned long speed;                    // calculated surface speed in mm/m or whisker/m

unsigned long last_heartbeat;           // time of last heartbeat animation change
unsigned long last_button_poll;         // debouncing for button

// macros for handling time intervals
// (casting to signed neatly handles millisecond counter wrap after 50 days uptime)
#define MILLIS_SINCE(when) (long)(millis() - (when))
#define MILLIS_BETWEEN(start,stop) (long)((stop) - (start))

// LCD output object - see pin assignments above
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

//
//@******************************* FUNCTIONS *********************************

//
//@@------------------------------ format_03d --------------------------------
// 
// given a value between 0 and 999, write as a 3 digit decimal 
// with leading zeros
// 
void format_03d(Print *stream, unsigned int d) 
{
  if (d < 10) {
    stream->print("00");
  }
  else if (d < 100) {
    stream->print("0");
  }
  stream->print(d);
}

//
//@@-------------------------- format_thousandths ----------------------------
//
// given a value in thousandths, display the whole number value in 
// 4 columns (.NNN, N.NN, or NNNN).
//
// eg. if    33 is passed, print ".033"
//     if   723 is passed, print ".723"
//     if  2762 is passed, print "2.76"
//     if 98765 is passed, print "99"
//
void format_thousandths(Print *stream, unsigned int speed, char *units) 
{
    if (speed < 1000) {
      stream->print(".");
      format_03d(stream, speed%1000);
    }
    else if (speed < 10000) {
      stream->print(speed/1000);
      stream->print('.');
      format_03d(stream, (speed%1000+50)/100);
    }
    else {
      stream->print((speed+500)/1000);
    }
    stream->print(units);
}

//
//@@----------------------------- max_diameter -------------------------------
unsigned long max_diameter(void) 
{
  switch (flags.units) {
  case UNITS_MET:
    return MAX_DIAMETER_MET;
  case UNITS_IMP:
    return MAX_DIAMETER_IMP;
  }
}

//
//@@----------------------- default_initial_diameter -------------------------
unsigned long default_initial_diameter() 
{
  switch (flags.units) {
  case UNITS_MET:
    return INITIAL_DIAMETER_MET;
  case UNITS_IMP:
    return INITIAL_DIAMETER_IMP;
  }
}

//
//@@---------------------------- diameter_plus -------------------------------
void diameter_plus() 
{
  diameter += (diameter >= default_initial_diameter())?5:1;
}

//
//@@---------------------------- diameter_minus ------------------------------
void diameter_minus() 
{
  diameter -= (diameter > default_initial_diameter())?5:1;
}

//@@---------------------------- update_display ------------------------------
//
// Update the display to show the current calculated RPM etc.
// 
void update_display(boolean clear=0)
{
  unsigned long dist;
  
  if (clear)
    lcd.clear();
  
  // 
  // Line 0: Print the RPM
  // 
  Serial.print(rpm, DEC);

  lcd.setCursor(0, 0);
  lcd.print(rpm);
  lcd.print(" rpm ");
  if (flags.hold) lcd.print("HOLD");
  lcd.print("    ");

  // 
  // Line 1: Print the surface speed in metres/minute
  // (in debug mode, show the raw rev count and uptime instead)
  // 
  lcd.setCursor(0, 1);
  speed = diameter * rpm * PI ;
  Serial.print(" raw-speed="); Serial.print(speed); // FIXME: remove once speed calc trusted

  switch (flags.mode) {
  case DISPLAY_MODE_TANGENTIAL:
    // Show surface speed
    //
    // At an assumed diameter of diameter_mm, convert revoutions/min to tangential metres/min
    // As we're truncating result to three places, can use a handy approximation for 
    // pi=355/113 (good to 7 decimal places, unlike 22/7 only good to two places)
    switch (flags.units) {
    case UNITS_MET:
      format_thousandths(&lcd, speed, "m/m @d ");
      lcd.print((int)diameter);
      lcd.print("mm  ");
      break;
    case UNITS_IMP:
      // Imperial speed is in crazy units of whiskers (1/40") per minute.
      // For consistency with mm/m, multiply by 25/12 to get still crazier millifoot/min
      speed = speed * 25 / 12;
      format_thousandths(&lcd, speed, "fpm @d ");
      format_thousandths(&lcd, diameter*25, "\"");// whiskers*25 => milli-inches
      break;
    }
    break;
  case DISPLAY_MODE_ODOMETER:
    // Show total revolutions (odometer)
    lcd.print(allrevs);
    lcd.print(" revs ");
    dist = allrevs * diameter * PI;  // overflows after 4294km or 1694mi
    switch (flags.units) {
    case UNITS_MET:
      // dist is in mm, convert to metres (thousandths of a km)
      format_thousandths(&lcd, dist/1000, "km");
      break;
    case UNITS_IMP:
      // dist is in whiskers (1/40"), convert to thousandths of a mile 
      // miles = whiskers / ( 40 * 12 * 5280)
      format_thousandths(&lcd, dist/2534, "mi");
      break;
    }
    break;
  case DISPLAY_MODE_SPEEDOMETER:
    switch (flags.units) {
    case UNITS_MET:
      // Show speed and distance in km/h and km
      // TODO: implement
      break;
    case UNITS_IMP:
      // Show speed and distance in mph and miles
      // TODO: implement
      break;
    }
  }
  
  Serial.println("");
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
    if (diameter < max_diameter()) diameter_plus();
    tone(PIN_SPKR,880,min(8,button_delay));
    update_display();
    // fall thru to delay update
  }
  else if (digitalRead(PIN_BTN_DOWN)==0) {
    if (diameter > 1) diameter_minus();
    tone(PIN_SPKR,440,min(8,button_delay));
    update_display();
    // fall thru to delay update
  }
  else if (digitalRead(PIN_BTN_MODE)==0) {
    // hold button active high
    flags.hold = !flags.hold;
    tone(PIN_SPKR,440*3,256);
    digitalWrite(PIN_LED_HOLD, flags.hold);
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
  //static const char spinner_chars[4] = {'|','/','-','\\'}; 
  static const char spinner_chars[4] = {'.','o','O','o'};     // japanese LCDs lack backslash!

  if (MILLIS_SINCE(last_heartbeat) < HEARTBEAT_INTERVAL) return;
  
  state.spinner++;
  digitalWrite(PIN_LED_DIAG,state.spinner%2);
  lcd.setCursor(15,0);
  lcd.print(spinner_chars[state.spinner]);

  last_heartbeat=millis();
}


//@@---------------------------- recalculate_rpm -----------------------------
// 
// Every 6 seconds (or less*) output the mean rotational speed over that
// interval.   
//
// * At 2500RPM the counter will be at 250 after 6 seconds.   If the
// speed is higher than this, trip at 250 anyway (merely results in
// more frequent updating).   The intended installation is unlikely
// to ever go above 4000RPM.
//
// At the installation's lower bound of about 60RPM, 6 seconds is only 6
// revs, but speed is calculated in number of complete revolutions divided
// by the time taken in milliseconds for those N revolutions, so accuracy
// should still be good.   Accuracy should be OK down to 0RPM.
//
// 
void recalculate_rpm() 
{
  if (rpm==0 && !revs     // really low revs, extend the sample window
      ||
      (MILLIS_SINCE(last_update_time) < MAX_UPDATE_INTERVAL) && (revs < MAX_REVS)) { 
    // No data, or not yet time to recalculate the RPM
    return;
  }

  boolean clear = (allrevs==0); // if this is very first update, clear whole screen

  if (!flags.hold) {
    // do not update if HOLD mode
    if (revs == 0) {
      rpm = 0;
    }
    else {
      rpm = 60000 * revs / MILLIS_BETWEEN(last_update_time, last_sensor_event);
    }
  }
  last_update_time = revs?last_sensor_event:millis(); // record start time of new sample window
  allrevs += revs;    		        // count total revs for 'odometer mode'
  revs = 0; 			        // reset the rev counter for next sample interval

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
  if (MILLIS_SINCE(last_sensor_event) > 10) {
    revs++;
    last_sensor_event=millis();
  }
}

//@****************************** ENTRY POINTS *******************************
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

  // TODO Read settings from EEPROM

  diameter = default_initial_diameter();
	
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
  Serial.print("# ");
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

  last_update_time = millis();
  attachInterrupt(0, sensor_isr, FALLING); // start counting pulses

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

//@****************************** END OF FILE ********************************

