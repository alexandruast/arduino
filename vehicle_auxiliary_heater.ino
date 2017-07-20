#include <math.h>
#include <EEPROM.h>
// #include <Wire.h>
// #include <SPI.h>
// #include <RtcDS1307.h>
// #include <TM1637Display.h>
// #include <LiquidCrystal_I2C.h>
// #include <Oregon.h>

// https://github.com/Makuna/Rtc
// https://github.com/avishorp/TM1637
// https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library
// https://github.com/Mickaelh51/Arduino-Oregon-Library

/* ToDo:
    fuel consumption statistics
    monitor fp only if heater on
    remove ignition from interrupt and use dpdt relay for blower
    custom characters for lcd/blink (?)
    433MHz remote actions
    real time clock - schedules
    lcd backlight 10 seconds timeout
*/

#define VOLTAGE 5.0
#define TEMP_CORRECTION 0.5
#define R1 30000.0
#define R2 7500.0

#define BUTTON_SHORTMS 15UL // minimum time for a button to be registered
#define BUTTON_LONGMS 500UL // long button press (at least 0.5 seconds)
#define BUTTON_POWERMS 2000UL // Power on/off (at least 2 seconds)
#define BUTTON_MFMS 5000UL // button malfunction

// Production board settings
  // #define POLLINGMS1 10000UL // sensor bank1 polling time (10 seconds)
  // #define POLLINGMS2 3600000UL // sensor bank2 polling time (60 minutes)
  // #define MENU_TIMEOUTMS 15000UL // After 15s of inactivity, revert to main screen
  // #define PUMP_TIMEOUTMS 30000UL // Stop pumps 30 seconds after heater off
  // #define BLOWER_PWM_TIMEOUTMS 15000UL // Stop blower 15 seconds after heater off
  // #define WINTER_MODE_RUNTIMEMS 720000UL // run manual mode for 12 minutes max. (720000UL)
  // #define STATIONARY_MODE_RUNTIMEMS 21600000UL // run stationary mode for 6h max. (21600000UL)
  // #define SUMMER_MODE_RUNTIMEMS 120000UL // run summer mode for 2 minutes max. (120000UL)
  // #define HEATER_BEEP_INTERVALMS 60000UL // beep every minute for heater on (60000UL)
 // #define FPFEED_TIMEOUTMS 1000UL // burner is off is fuel pump is off for at least one second
  // #define WARNING_BEEP_INTERVALMS 23000UL // beep every 23 seconds for warnings (23000UL)
  // #define LOOP_REPORT_INTERVALMS 60000UL // report average loop performance every minute (60000UL)
// End production board settings

// Prototype/simulator board settings
  #define POLLINGMS1 1000UL // sensor bank1 polling time (10 seconds)
  #define POLLINGMS2 2000UL // sensor bank2 polling time (60 minutes)
  #define MENU_TIMEOUTMS 7500UL // After 15s of inactivity, revert to main screen
  #define PUMP_TIMEOUTMS 3000UL // Stop pumps 30 seconds after heater off
  #define BLOWER_PWM_TIMEOUTMS 6000UL // Stop blower 15 seconds after heater off
  #define WINTER_MODE_RUNTIMEMS 15000UL // run manual winter mode for 12 minutes max. (720000UL)
  #define STATIONARY_MODE_RUNTIMEMS 30000UL // run stationary mode for 6h max. (21600000UL)
  #define REMOTE_MODE_RUNTIMEMS 10000UL // run remote mode for 6 minutes max. (360000UL)
  #define SUMMER_MODE_RUNTIMEMS 5000UL // run manual summer mode for 2 minutes max. (120000UL)
  #define HEATER_BEEP_INTERVALMS 2000UL // beep every minute for heater on (60000UL)
  #define FPFEED_TIMEOUTMS 1000UL // burner is off is fuel pump is off for at least one second
  #define WARNING_BEEP_INTERVALMS 3000UL // beep every 23 seconds for warnings (23000UL)
  #define LOOP_REPORT_INTERVALMS 10000UL // report average loop performance every minute (60000UL)
// End production board settings

#define BLOWER_PWM_WINTER 35 // in winter, run at 35%
#define BLOWER_PWM_CAMPING 15 // in stationary mode, run at minimum
#define BLOWER_PWM_SUMMER 60 // in summer, run at 60%

/* Seasons: 1 - Auto, 2 - Summer, 3 - Winter
   stationary mode is disabled in summer
   manual mode blower runs at max. for 2 minutes flat in summer (overheating warning!)
*/

// store season and temperature in a single EEPROM byte
#define SETTINGS_BANK1_EEPROM_ADDR 0

/* Temperature is adjustable from 5c deg. to 24c deg. in 1c deg. increments
   Do not change these numbers!
*/
#define SETUP_MODE_CABIN_TEMPERATURE_MIN 5
#define SETUP_MODE_CABIN_TEMPERATURE_MAX 24
#define SUMMER_CABIN_TEMPERATURE_MAX 34 // max. desired degrees in summer for stationary mode
#define EXPECTED_CABIN_TEMPERATURE_DEFAULT 20 // default degrees = 20c
#define TEMPERATURE_UNIT_INCREMENT 1

#define FUEL_PULSE_ML 0.5
#define FUEL_USAGE_MAX_ML 950.0

#define BEEP_SILENCE_DURATIONMS 50 // silence between beeps on same request
#define BEEP_LONG_DURATIONMS 500 // long beep duration
#define BEEP_SHORT_DURATIONMS 50 // short beep duration
#define BEEP_SET_DURATIONMS 100 // action set beep duration
#define BEEP_SET_TIMES 3 // action set beep times
#define BEEP_UNAVAILABLE_TIMES 2 // action unavailable beep times
#define BEEP_WARNING_TIMES 4 // warning beep times
#define BEEP_FREQ_OK 440
#define BEEP_FREQ_NOK 880


#define IGNSWITCH_PIN 2 // ignition switch on (interrupt)
#define FPFEED_PIN 3 // heater fuel pump feedback pin (interrupt)
#define BUTTON1_PIN 4 // main button
#define BUZZER_PIN 5 // buzzer
#define TM1637_CLK 6 // 4x7segment led display
#define TM1637_DIO 7 // 4x7segment led display
#define TEMPERATURE_PIN 8 // temperature sensors
#define BLOWER_PWM_PIN 9 // blower pwm module pin
#define R433_PIN 10 // 433 Mhz receiver pin

#define VOLTAGE_PIN A0 // voltage monitoring pin
#define HEATER_PIN A1 // heater command pin
#define PUMP1_PIN A2 // pump1 command pin
#define PUMP2_PIN A3 // pump2 command pin
#define SDA_PIN A4 // serial devices - rtc, i2c display
#define SCL_PIN A5 // serial devices - rtc, i2c display

#define BATT_MIN_VOLTAGE 12.0
#define BATT_MAX_VOLTAGE 12.8
#define CHARGING_MIN_VOLTAGE 13.5
#define CHARGING_MAX_VOLTAGE 14.5

volatile bool ignswitch_on = false;
volatile bool fpfeed_on = false;
bool engine_running = false;

bool button1_active = false;
bool setup_mode = false;
bool beep_on = false;
bool warning_on = false;

bool batt_voltage_input_changed = false;
bool cabin_temperature_input_changed = false;
bool system_on = false;
bool winter_on = false;
bool cabin_temperature_reached = true; // temperature is reached at init (no command given)
bool manual_mode_on = false;
bool remote_mode_on = false;
bool stationary_mode_on = false;
bool engine_mode_on = false;
bool blower_on = false;
bool heater_on = false;
bool fpfeed_active = false;
bool p1_on = false;
bool p2_on = false;

/* screens
   1 - batt_voltage and status
   2 - heater
   3 - temperature
   4 - season
*/
const byte screens = 4;
byte current_screen = 1;

unsigned int beep_duration = 0;
unsigned int beep_freq = 0;
byte beep_times = 0;
float loop_avg_ms = 0.0;

float fuel_consumption_run_ml = 0.0;
unsigned int fuel_consumption_total_ml = 0;

unsigned long loop_times, millis_t, boot_t, loop_t, loop_report_t, system_warning_t;
unsigned long sensors_bank1_t, sensors_bank2_t, button1_t, button1_td, menu_t;
unsigned long heater_t, fpfeed_t, pump_t, blower_t, manual_mode_t, engine_t, stationary_mode_t, remote_mode_t;
unsigned long beep_t, beep_silence_t, heater_beep_t;
byte expected_cabin_temperature, setup_mode_cabin_temperature;
bool setup_mode_winter_on;

// analog adjustment stuff
const float ANALOG_UNIT = VOLTAGE / 1024;

// a divider circuit is used for measuring batt_voltage
const float VOLTAGE_DIVIDER = R2/(R1+R2);

float batt_voltage = BATT_MAX_VOLTAGE; // assume battery is charged
float cabin_temperature = SETUP_MODE_CABIN_TEMPERATURE_MAX; // assume internal temperature is reached
byte blower_pwm = 0; // assume blower pwm is zero

void setup() {

  Serial.begin(9600);

  // analog I/O
  pinMode(VOLTAGE_PIN, INPUT);

  // digital I/O
  pinMode(BUTTON1_PIN, INPUT);
  digitalWrite(BUTTON1_PIN, INPUT_PULLUP);

  pinMode(IGNSWITCH_PIN, INPUT);
  digitalWrite(IGNSWITCH_PIN, INPUT_PULLUP);

  pinMode(FPFEED_PIN, INPUT);
  digitalWrite(FPFEED_PIN, INPUT_PULLUP);

  pinMode(TEMPERATURE_PIN, INPUT);

  digitalWrite(PUMP1_PIN, HIGH);
  pinMode(PUMP1_PIN, OUTPUT);

  digitalWrite(PUMP2_PIN, HIGH);
  pinMode(PUMP2_PIN, OUTPUT);

  digitalWrite(HEATER_PIN, HIGH);
  pinMode(HEATER_PIN, OUTPUT);

  digitalWrite(BLOWER_PWM_PIN, HIGH);
  pinMode(BLOWER_PWM_PIN, OUTPUT);


  attachInterrupt(digitalPinToInterrupt(IGNSWITCH_PIN), ignition_switch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FPFEED_PIN), fpfeed_switch, CHANGE);

  // set variables to default
  loop_times = 0UL;
  button1_active = false;
  button1_t = 0UL; // timer for button1
  button1_td = 0UL; // time delta for button1
  sensors_bank1_t = 0UL; // timer for sensors polling
  sensors_bank2_t = 0UL; // timer for sensors polling
  heater_t = 0UL; // timer for heater
  fpfeed_t = 0UL; // timer for fuel pump feed
  pump_t = 0UL; // timer for pumps
  manual_mode_t = 0UL; // runtime for manual mode
  stationary_mode_t = 0UL; // runtime for stationary mode
  remote_mode_t = 0UL; // runtime for remote mode
  heater_beep_t = 0UL; // periodic beep for heater on

  millis_t = millis();

  beep_t = millis_t; // timer for beep duration
  beep_silence_t = millis_t; // timer for silence duration
  menu_t = millis_t; // timer for menu
  loop_t = millis_t; // timer for loop
  loop_report_t = millis_t; // periodic loop performance reporting
  system_warning_t = millis_t; // timer for system warnings

  // read settings from EEPROM
  byte eeprom_byte = EEPROM.read(SETTINGS_BANK1_EEPROM_ADDR);
  if (eeprom_byte == 255 || eeprom_byte == 0) {
    Serial.println("Settings in EEPROM are not valid, writing defaults.");
    eeprom_byte = EXPECTED_CABIN_TEMPERATURE_DEFAULT;
    EEPROM.write(SETTINGS_BANK1_EEPROM_ADDR, eeprom_byte);
  }

  (eeprom_byte < 100) ? winter_on = false : winter_on = true;
  expected_cabin_temperature = eeprom_byte % 100;

  // get ignition switch status
  ignition_switch();

  // system start confirmation beep
  beep_action(BEEP_LONG_DURATIONMS, 1, BEEP_FREQ_OK);
}

void loop() {
  // start a timer at loop start
  loop_t = millis();

  // sensor bank1 polling
  if (loop_t - sensors_bank1_t > POLLINGMS1) {
    read_sensors_bank1();
    sensors_bank1_t = millis();
  }

  // triggers for warnings and malfunctions
  if (batt_voltage < BATT_MIN_VOLTAGE) {
    if (system_on) {
      system_turn_off();
    } else {
      if (!warning_on) {
        Serial.println("Warning: LOW battery voltage!");
        warning_on = true;
      }
    }
  }

  if (batt_voltage > CHARGING_MAX_VOLTAGE) {
    if (!warning_on) {
      Serial.println("Warning: HIGH charging voltage!");
      warning_on = true;
    }
  }

  if (warning_on && loop_t - system_warning_t > WARNING_BEEP_INTERVALMS) {
    beep_action(BEEP_SHORT_DURATIONMS, BEEP_WARNING_TIMES, BEEP_FREQ_NOK);
    system_warning_t = loop_t;
  }

  // restore screen if inactive
  if (loop_t - button1_t > MENU_TIMEOUTMS && current_screen != 1) {
    current_screen = 1;
    setup_mode = false;
  }

  // get button active duration
  if (button1_pressed()) {
    if (!button1_active) {
      button1_active = true;
      button1_t = loop_t;
    }
  } else {
    if (button1_active) {
      button1_td = loop_t - button1_t;
      button1_active = false;
    }
  }

  // different actions for button press short, long and very long
  if (button1_td > BUTTON_SHORTMS) { // condition to avoid noise from switches
    menu_t = loop_t;
    // short press
    if (button1_td < BUTTON_LONGMS && system_on) {
      button1_short_press();
    }
    //long press
    if (button1_td >= BUTTON_LONGMS && button1_td < BUTTON_POWERMS && system_on) {
      button1_long_press();
    }
    // power on or off
    if (button1_td >= BUTTON_POWERMS && button1_td  < BUTTON_MFMS) {
      setup_mode = false;
      current_screen = 1;
      (system_on) ? system_turn_off() : system_turn_on();
    }
    if (button1_td >= BUTTON_MFMS) {
      // button error
      beep_action(BEEP_SET_DURATIONMS, BEEP_UNAVAILABLE_TIMES, BEEP_FREQ_NOK);
    }
    // after actions were performed, reset the button delta
    button1_td = 0UL;
  }

  if (batt_voltage_input_changed) {
    if (batt_voltage > CHARGING_MIN_VOLTAGE && ignswitch_on) {
      if (!engine_running) {
        Serial.println("Engine is running");
        engine_running = true;
        engine_mode_on = true;
        engine_t = loop_t;
      }
    } else {
      if (engine_running) {
        Serial.println("Engine is stopped");
        engine_running = false;
        engine_mode_on = false;
      }
    }
    batt_voltage_input_changed = false;
  }

  if (system_on) {
    // when temperature changes, re-evaluate conditions
    if (cabin_temperature_input_changed) {
      if (winter_on) {
        (cabin_temperature < expected_cabin_temperature) ? cabin_temperature_reached = false : cabin_temperature_reached = true;
      } else {
        (cabin_temperature > SUMMER_CABIN_TEMPERATURE_MAX) ? cabin_temperature_reached = false : cabin_temperature_reached = true;
      }
      cabin_temperature_input_changed = false;
    }

    if (winter_on) {
    // winter is here
      if (engine_mode_on) {
        if (!p1_on) { pump1_turn_on(); }
        if (p2_on) { pump2_turn_off(); }
        if (!heater_on && fuel_consumption_run_ml < FUEL_USAGE_MAX_ML) { heater_turn_on(); }
      } else if (manual_mode_on || remote_mode_on) {
        if (!p1_on) { pump1_turn_on(); }
        if (p2_on) { pump2_turn_off(); }
        if (!heater_on && fuel_consumption_run_ml < FUEL_USAGE_MAX_ML) { heater_turn_on(); }
        if (!blower_on || blower_pwm != BLOWER_PWM_WINTER ) { blower_turn_on(BLOWER_PWM_WINTER); }
      } else if (stationary_mode_on) {
        if (p1_on) { pump1_turn_off(); }
        if (!cabin_temperature_reached) {
          if (!p2_on) { pump2_turn_on(); }
          if (!heater_on && fuel_consumption_run_ml < FUEL_USAGE_MAX_ML) { heater_turn_on(); }
          if (!blower_on || blower_pwm != BLOWER_PWM_CAMPING) { blower_turn_on(BLOWER_PWM_CAMPING); }
        } else {
          if (heater_on) { heater_turn_off(); }
        }
      } else {
        if (heater_on) { heater_turn_off(); }
      }
    } else {
      // summer is here
      if (heater_on) { heater_turn_off(); }
      if (manual_mode_on || remote_mode_on) {
          if (!blower_on || blower_pwm != BLOWER_PWM_SUMMER) { blower_turn_on(BLOWER_PWM_SUMMER); }
      } else if (stationary_mode_on) {
        if (!cabin_temperature_reached) {
          if (!blower_on || blower_pwm != BLOWER_PWM_SUMMER) { blower_turn_on(BLOWER_PWM_SUMMER); }
        } else {
          if (blower_on) { blower_turn_off(); }
        }
      } else {
        if (blower_on) { blower_turn_off(); }
      }
    }

    // runtime will complete no matter the season
    if (winter_on) {
      if (manual_mode_on && loop_t - manual_mode_t > WINTER_MODE_RUNTIMEMS) {
        Serial.println("Runtime reached for manual mode (winter)");
        manual_mode_on = false;
      }
    } else {
      if (manual_mode_on && loop_t - manual_mode_t > SUMMER_MODE_RUNTIMEMS) {
        Serial.println("Runtime reached for manual mode (summer)");
        manual_mode_on = false;
      }
    }

    if (stationary_mode_on && loop_t - stationary_mode_t > STATIONARY_MODE_RUNTIMEMS) {
      Serial.println("Runtime reached for stationary mode");
      stationary_mode_on = false;
    }

    if (remote_mode_on && loop_t - remote_mode_t > REMOTE_MODE_RUNTIMEMS) {
      Serial.println("Runtime reached for remote mode");
      remote_mode_on = false;
    }

    /* beep every minute if heater on
       we could get rid of heater_beep_t use heater_t % HEATER_BEEP_INTERVALMS == 0,
       but we might miss the event and managing this is additionally expensive
    */
    if (heater_on && loop_t - heater_t > HEATER_BEEP_INTERVALMS && loop_t - heater_beep_t > HEATER_BEEP_INTERVALMS) {
      beep_action(BEEP_SHORT_DURATIONMS, 2, BEEP_FREQ_OK);
      heater_beep_t = loop_t;
    }
    // blower will run for a while even after stop command
    if (!heater_on && winter_on && loop_t - blower_t > BLOWER_PWM_TIMEOUTMS) {
      if (blower_on) { blower_turn_off(); }
    }
  }

  // stop heater if max. fuel consumption value is reached
  if (fuel_consumption_run_ml >= FUEL_USAGE_MAX_ML && heater_on) {
    heater_turn_off(); // blower stops automatically on timer
    Serial.println("Heater stopped due to max fuel usage");
  }
  // pumps will run for a while on heater stop even after off command
  if (!heater_on && (p1_on || p2_on) && loop_t - pump_t > PUMP_TIMEOUTMS) {
    if (p1_on) { pump1_turn_off(); }
    if (p2_on) { pump2_turn_off(); }
  }

  // manage beeps
  if (beep_times > 0) { // request for beep
    if (!beep_on) { // we do not have beep_on if silence period
      if (loop_t - beep_silence_t > BEEP_SILENCE_DURATIONMS) { // are we not on silence period?
        beep_turn_on();
      }
    } else { // we have beep_on
      if (loop_t - beep_t > beep_duration) { // beep timer reached
        beep_turn_off();
        beep_times--;
      }
    }
  }

  loop_times++;

  // get loop performance data
  if (loop_t - loop_report_t > LOOP_REPORT_INTERVALMS) {
    // loop report resets with the data before the current loop
    loop_avg_ms = (loop_t - loop_report_t)/(loop_times - 1.0);
    Serial.print("Loop performance:");
    Serial.println(loop_avg_ms,2);
    loop_report_t = loop_t;
    loop_times = 0;
  }
}

void button1_short_press() {
  if (!setup_mode) {
    if (current_screen == screens) {
      current_screen = 1;
      beep_action(BEEP_SHORT_DURATIONMS, 2, BEEP_FREQ_OK);
    } else {
      current_screen ++;
      beep_action(BEEP_SHORT_DURATIONMS, 1, BEEP_FREQ_OK);
    }
    Serial.print("New current screen:");
    Serial.println(current_screen);
  } else {
    switch (current_screen) {
      case 3:
        // increment degrees
        if (setup_mode_cabin_temperature == SETUP_MODE_CABIN_TEMPERATURE_MAX) {
          setup_mode_cabin_temperature = SETUP_MODE_CABIN_TEMPERATURE_MIN;
          beep_action(BEEP_SHORT_DURATIONMS, 2, BEEP_FREQ_OK);
        } else {
          setup_mode_cabin_temperature += TEMPERATURE_UNIT_INCREMENT;
          beep_action(BEEP_SHORT_DURATIONMS, 1, BEEP_FREQ_OK);

        }
        Serial.print("setup_mode_cabin_temperature:");
        Serial.println(setup_mode_cabin_temperature);
        break;
      case 4:
        // switch between summer and winter modes
        if (!setup_mode_winter_on) {
          setup_mode_winter_on = true;
          beep_action(BEEP_SHORT_DURATIONMS, 2, BEEP_FREQ_OK);
        } else {
          setup_mode_winter_on = false;
          beep_action(BEEP_SHORT_DURATIONMS, 1, BEEP_FREQ_OK);
        }
        Serial.print("setup_mode_winter_on:");
        Serial.println(setup_mode_winter_on);
        break;
    }
  }
}

void button1_long_press() {
  if (!setup_mode) {
    setup_mode = true;
    switch (current_screen) {
      case 1:
        if (engine_running && engine_mode_on) {
          beep_action(BEEP_LONG_DURATIONMS, 1, BEEP_FREQ_OK);
          engine_mode_on = false;
        } else if (!engine_running && !ignswitch_on) {
          if (manual_mode_on) {
            beep_action(BEEP_LONG_DURATIONMS, 1, BEEP_FREQ_OK);
            manual_mode_on = false;
          } else {
            beep_action(BEEP_SET_DURATIONMS, BEEP_SET_TIMES, BEEP_FREQ_OK);
            manual_mode_on = true;
            remote_mode_on = false;
            stationary_mode_on = false;
            manual_mode_t = loop_t;
            fuel_consumption_run_ml = 0;
          }
          Serial.print("manual_mode_on:");
          Serial.println(manual_mode_on);
        } else {
          beep_action(BEEP_SET_DURATIONMS, BEEP_UNAVAILABLE_TIMES, BEEP_FREQ_NOK);
        }
        setup_mode = false;
        break;
      case 2:
        if (!engine_running && !ignswitch_on) {
          if (stationary_mode_on) {
            beep_action(BEEP_LONG_DURATIONMS, 1, BEEP_FREQ_OK);
            stationary_mode_on = false;
          } else {
            beep_action(BEEP_SET_DURATIONMS, BEEP_SET_TIMES, BEEP_FREQ_OK);
            stationary_mode_on = true;
            manual_mode_on = false;
            remote_mode_on = false;
            stationary_mode_t = loop_t;
            fuel_consumption_run_ml = 0;
          }
          Serial.print("stationary_mode_on:");
          Serial.println(stationary_mode_on);
        } else {
          beep_action(BEEP_SET_DURATIONMS, BEEP_UNAVAILABLE_TIMES, BEEP_FREQ_NOK);
        }
        setup_mode = false;
        break;
      case 3:
        beep_action(BEEP_LONG_DURATIONMS, 1, BEEP_FREQ_OK);
        Serial.println("Setup mode for temperature");
        setup_mode_cabin_temperature = expected_cabin_temperature;
        break;
      case 4:
        beep_action(BEEP_LONG_DURATIONMS, 1, BEEP_FREQ_OK);
        Serial.println("Setup mode for season.");
        setup_mode_winter_on = winter_on;
        break;
    }

  } else {
    switch (current_screen) {
      case 3:
        if (expected_cabin_temperature != setup_mode_cabin_temperature) {
          beep_action(BEEP_SET_DURATIONMS, BEEP_SET_TIMES, BEEP_FREQ_OK);
          expected_cabin_temperature = setup_mode_cabin_temperature;
          EEPROM.write(SETTINGS_BANK1_EEPROM_ADDR, (winter_on * 100) + expected_cabin_temperature);
        } else {
          beep_action(BEEP_SET_DURATIONMS, BEEP_UNAVAILABLE_TIMES, BEEP_FREQ_NOK);
        }
        Serial.print("Expected degrees:");
        Serial.println(expected_cabin_temperature);
        Serial.print("Exiting setup mode for screen:");
        Serial.println(current_screen);
        setup_mode = false;
        // do not exit to main screen, wait for timeout
        break;
      case 4:
        if (winter_on != setup_mode_winter_on) {
          beep_action(BEEP_SET_DURATIONMS, BEEP_SET_TIMES, BEEP_FREQ_OK);
          winter_on = setup_mode_winter_on;
          cabin_temperature_input_changed = true;
          EEPROM.write(SETTINGS_BANK1_EEPROM_ADDR, (winter_on * 100) + expected_cabin_temperature);
        } else {
          beep_action(BEEP_SET_DURATIONMS, BEEP_UNAVAILABLE_TIMES, BEEP_FREQ_NOK);
        }
        Serial.print("Current season:");
        Serial.println(winter_on);
        Serial.print("Exiting setup mode for screen:");
        Serial.println(current_screen);
        setup_mode = false;
        // do not exit to main screen, wait for timeout
        break;
    }
  }
}

void fpfeed_switch() {
  if (digitalRead(FPFEED_PIN)) { // high = no feed
    fpfeed_on = false;
  } else {
    fpfeed_on = true;
    fuel_consumption_run_ml += FUEL_PULSE_ML;
    fuel_consumption_total_ml += FUEL_PULSE_ML;
    Serial.println(fuel_consumption_run_ml);
  }
}

void ignition_switch() {
  if (blower_on) { blower_turn_off(); }
  if (heater_on) { heater_turn_off(); }
  digitalRead(IGNSWITCH_PIN) == LOW ? ignswitch_on = true : ignswitch_on = false;
  Serial.print("Ignition switch:");
  Serial.println(ignswitch_on);
  fuel_consumption_run_ml = 0;
  manual_mode_on = false;
  stationary_mode_on = false;
  remote_mode_on = false;
  batt_voltage_input_changed = true;
  current_screen = 1;
}

bool button1_pressed() {
  return !digitalRead(BUTTON1_PIN);
}

void read_sensors_bank1() {
  float value = 0.0;
  // read battery voltage
  value = read_batt_voltage(VOLTAGE_PIN);
  if (value != batt_voltage) {
    batt_voltage_input_changed = true;
    batt_voltage = value;
  }
  // read temperature sensors
  value = read_temperature(TEMPERATURE_PIN);
  if (value != cabin_temperature) {
    cabin_temperature_input_changed = true;
    cabin_temperature = value;
    Serial.println(cabin_temperature,2);
  }
}

float read_batt_voltage(int pin) {
  int raw = analogRead(pin);
  int value = round(((raw * ANALOG_UNIT) / VOLTAGE_DIVIDER) * 10.0);
  return value / 10.0;
}

float read_temperature(int pin) {
  // int raw = analogRead(pin);
  int raw = 124; // manually setting temperature for simulator to work without sensors
  float value = round((((raw * ANALOG_UNIT) - TEMP_CORRECTION) * 100) * 10.0);
  return value / 10.0;
}

void system_turn_off() {
  beep_action(BEEP_LONG_DURATIONMS, 1, BEEP_FREQ_OK);
  heater_turn_off;
  blower_turn_off;
  manual_mode_on = false;
  stationary_mode_on = false;
  remote_mode_on = false;
  system_on = false;
  Serial.println("Power OFF");
}

void system_turn_on() {
  beep_action(BEEP_SET_DURATIONMS, BEEP_SET_TIMES, BEEP_FREQ_OK);
  if (warning_on) {
    Serial.println("Warning reset!");
    warning_on = false;
  }
  system_on = true;
  Serial.println("Power ON");
}

void heater_turn_on() {
  digitalWrite(HEATER_PIN, LOW);
  Serial.println("Heater turned on");
  heater_on = true;
  heater_t = loop_t;
}

void heater_turn_off() {
  digitalWrite(HEATER_PIN, HIGH);
  Serial.println("Heater turned off");
  pump_t = loop_t;
  blower_t = loop_t;
  heater_on = false;
}

void blower_turn_on(int power) {
  // digitalWrite(BLOWER_PWM_PIN, power);
  digitalWrite(BLOWER_PWM_PIN, LOW);
  Serial.print("Blower turned on:");
  Serial.println(power);
  blower_on = true;
  blower_pwm = power;
}

void blower_turn_off() {
  digitalWrite(BLOWER_PWM_PIN, HIGH);
  Serial.println("Blower turned off");
  blower_on = false;
  blower_pwm = 0;
}

void pump1_turn_on() {
  digitalWrite(PUMP1_PIN, LOW);
  Serial.println("Pump1 turned on");
  p1_on = true;
}

void pump1_turn_off() {
  digitalWrite(PUMP1_PIN, HIGH);
  Serial.println("Pump1 turned off");
  p1_on = false;
}

void pump2_turn_on() {
  digitalWrite(PUMP2_PIN, LOW);
  Serial.println("Pump2 turned on");
  p2_on = true;
}

void pump2_turn_off() {
  digitalWrite(PUMP2_PIN, HIGH);
  Serial.println("Pump2 turned off");
  p2_on = false;
}

void beep_action(int ms, byte times, int freq) {
  beep_times = times;
  beep_duration = ms;
  beep_freq = freq;
}

void beep_turn_on() {
  tone(BUZZER_PIN, beep_freq);
  beep_on = true;
  beep_t = loop_t;
}

void beep_turn_off() {
  noTone(BUZZER_PIN);
  beep_on = false;
  beep_silence_t = loop_t;
}
