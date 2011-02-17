//@------------------------------ spin doctor ----------------- -*- C++ -*- --
// 
// This program implements an RPM and surface speed display for lathes,
// bandsaws etc.
//
// It expects a single magnet epoxied to a shaft or pulley, adjacent to a
// fixed hall-effect sensor.  Optical sensing is also possible.
//
// An interrupt is used to count pulses, and the pulse count is periodically 
// converted to the calculated revolutions per minute (averaged over the 
// interval since the previous calculation).
//
// A diameter value is maintained (changeable via two buttons) and the 
// linear speed (in surface metres/minute) is also calculated at that
// diameter.
//
// This allows the surface speed of a workpiece or tool to be calculated,
// giving your tool a direct readout in surface metres per miniute 
// (or feet per minute if you are from one of Those Places).
// 
//
//@********************************** BUGS ***********************************
//
// * Primitive software debouncing of noisy sensor limits high speed sensing
//
//@********************************** TODO ***********************************
// 
// * store preferences (last diameter, units) in EEPROM
// * accept configuration commands over RS232 (change units etc)
// * make the RS232 output parsable
// * battery voltage sensing

#include <LiquidCrystal.h>

//@******************************* CONSTANTS *********************************

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


// set count_revs=1 to show total revolutions ever (useful for debugging
// calculations or intermittent sensor triggering)
// set count_revs=0 to show diameter

#define INITIAL_BUTTON_DELAY 256
#define HOLD_BUTTON_DELAY    512
#define MIN_BUTTON_DELAY       2

#define HEARTBEAT_INTERVAL   500

#define MAX_REVS	     250
#define MAX_UPDATE_INTERVAL 6000

#define UNITS_MET  0
#define UNITS_IMP  1

// Metric diameter stored as whole MM.  
// Imperial diameter stored as fortieths of an inch (25 thou)

// In either case, under initial diameter size can be varied by unit, 
// over it, each button press changes by 5 units (5mm or 1/8 inch).

#define INITIAL_DIAMETER_MET   50
#define INITIAL_DIAMETER_IMP   80
#define MAX_DIAMETER_MET     1000
#define MAX_DIAMETER_IMP     1440

#define DISPLAY_MODE_TANGENTIAL  0
#define DISPLAY_MODE_ODOMETER    1
#define DISPLAY_MODE_SPEEDOMETER 2

#define DEFAULT_UNITS 	UNITS_MET
#define DEFAULT_MODE    DISPLAY_MODE_TANGENTIAL
// Changing DEFAULT_MODE to ODOMETER is useful for testing debounce
// Speedometer displays speed/distance in km or miles


//@********************************** DATA ***********************************

struct _flags {
  byte units: 1;
  byte hold: 1;
  byte mode:2;
  byte spinner: 2;
} flags = { DEFAULT_MODE, DEFAULT_UNITS, 0 };

unsigned int  button_delay  = INITIAL_BUTTON_DELAY;;

volatile byte revs;                     // number of revolutions counted in this sample window
unsigned long allrevs;                  // total number of revolutions ever counted
unsigned long last_update_time;         // time (in millis) of start of sample window
unsigned long last_sensor_event;        // time (in millis) of most recent sensor event
unsigned int  diameter;			// assumed workpiece diameter in mm or inch/40

unsigned long rpm;                      // calculated RPM
unsigned long speed;                    // calculated surface speed in mm/m or inch/m

unsigned long last_heartbeat;           // time of last heartbeat animation change
unsigned long last_button_poll;         // debouncing for button


#define MILLIS_SINCE(when) (long)(millis() - (when))
#define MILLIS_BETWEEN(start,stop) (long)((stop) - (start))

// initialize the LCD library with the numbers of the interface pins
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

//
//@******************************* FUNCTIONS *********************************

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
void format_thousandths(Print *stream, int speed, char *units) 
{
    if (speed < 1000) {
      stream->print(".");
      stream->print(speed%1000);
    }
    else if (speed < 10000) {
      stream->print(speed/1000);
      stream->print('.');
      stream->print((speed%1000+50)/100);
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
  diameter -= (diameter >= default_initial_diameter())?5:1;
}

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
  Serial.print(rpm, DEC);

  lcd.setCursor(0, 0);
  lcd.print(rpm);
  lcd.print(" r/m ");
  if (flags.hold) lcd.print("HOLD");
  lcd.print("    ");

  // 
  // Line 1: Print the surface speed in metres/minute
  // (in debug mode, show the raw rev count and uptime instead)
  // 
  lcd.setCursor(0, 1);

  switch (flags.mode) {
  case DISPLAY_MODE_TANGENTIAL:
    // Show surface speed
    //
    // At an assumed diameter of diameter_mm, convert revoutions/min to tangential metres/min
    // As we're truncating result to three places, can use a handy approximation for 
    // pi=355/113 (good to 7 decimal places, unlike 22/7 only good to two places)
    speed =  diameter * 355 * rpm / 113;
    switch (flags.units) {
    case UNITS_MET:
      format_thousandths(&lcd, speed, " m/m @ d=");
      lcd.print((int)diameter);
      lcd.print("mm  ");
      break;
    case UNITS_IMP:
      // Imperial speed is in crazy units of fortieths of an inch per minute.
      // For consistency with mm/m, multiply by 25/12 to get still crazier thousandths of a foot/min
      speed = speed * 25 / 12;
      format_thousandths(&lcd, speed, " fpm @ d=");
      format_thousandths(&lcd, diameter*40, "\"");
      break;
    }
    break;
  case DISPLAY_MODE_ODOMETER:
    // Show total revolutions (odometer)
    lcd.print(allrevs);
    lcd.print(" revs. ut");
    lcd.print(millis()/1000);
    break;
  case DISPLAY_MODE_SPEEDOMETER:
    // Show speed and distance in km/h or mph
    break;
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
  
  flags.spinner++;
  digitalWrite(PIN_LED_DIAG,flags.spinner%2);
  lcd.setCursor(15,0);
  lcd.print(spinner_chars[flags.spinner]);

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
  if (!revs ||
      (MILLIS_SINCE(last_update_time) < MAX_UPDATE_INTERVAL) && (revs < MAX_REVS)) { 
    // No data, or not yet time to recalculate the RPM
    return;
  }

  boolean clear = (allrevs==0); // if this is very first update, clear whole screen

  if (!flags.hold) {
    // do not update if HOLD mode
    rpm = 60000 * revs / MILLIS_BETWEEN(last_update_time, last_sensor_event);
  }
  allrevs += revs;    		        // count total revs for 'odometer mode'
  revs = 0; 			        // reset the rev counter for next sample interval
  last_update_time = last_sensor_event; // record start time of new sample window

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

