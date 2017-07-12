#include <math.h>
#include <EEPROM.h>

#define VOLTAGE 5.0
#define TEMP_CORRECTION 0.5
#define R1 30000.0
#define R2 7500.0
#define POLLINGMS1 100UL // ic_deg_bank polling time (10 seconds)
#define POLLINGMS2 2000UL // ext_deg_bank polling time (60 minutes)
#define BUTTON_SHORTMS 10UL // minimum time for a button to be registered
#define BUTTON_LONGMS 500UL // long button press (1000ms<t>5000ms)
#define BUTTON_POWERMS 2000UL // Power on/off (5000ms<t>7000ms)
#define BUTTON_MFMS 7000UL // >7000ms register this as button malfunction
#define MENU_TIMEOUTMS 5000UL // After 15000ms, menu is reverting to main screen
#define PUMP_TIMEOUTMS 3000UL // Stop circulation 30 seconds after receiving heater off
#define BLOWER_PWM_TIMEOUTMS 3000UL // Stop circulation 30 seconds after receiving heater off
#define BLOWER_PWM_WINTER 35 // in winter, run at 35%
#define BLOWER_PWM_CAMPING 15 // in camping mode, run at minimum (ToDo: TBD)
#define BLOWER_PWM_SUMMER 60 // in summer, run at 60%
#define HVAC_WINTER_MODE_RUNTIMEMS 5000UL // run manual HVAC for 12 minutes max. (720000UL)
#define HVAC_CAMPING_MODE_RUNTIMEMS 15000UL // run camping mode HVAC for 6h max. (21600000UL)
#define HVAC_SUMMER_MODE_RUNTIMEMS 2000UL // run summer HVAC for 2 minutes max. (120000UL)
#define HEATER_BEEP_INTERVAL 6000UL // beep every minute for heater on (60000UL)

/* Seasons: 1 - Auto, 2 - Summer, 3 - Winter
   camping mode is disabled in summer
   manual mode blower_pwm runs at max. for 2 minutes flat in summer (overheating warning!)
*/
#define EXPECTED_SEASON_EEPROM_ADDR 0
#define EXPECTED_SEASON_MIN 1
#define EXPECTED_SEASON_MAX 3
#define EXPECTED_SEASON_DEFAULT 1 // default season = AUTO
#define SEASON_AUTO_SWITCH_DEG 7 // winter is here below this temperature

/* Temperature is adjustable from 5c deg. to 24c deg. in 1c deg. increments
   Do not change these numbers!
*/
#define EXPECTED_DEG_EEPROM_ADDR 1
#define EXPECTED_DEG_MIN 5
#define EXPECTED_DEG_MAX 24
#define EXPECTED_DEG_SUMMER_MAX 34 // max. desired degrees in summer for camping mode
#define EXPECTED_DEG_DEFAULT 20 // default degrees = 20c
#define EXPECTED_DEG_UNIT_INCREMENT 1
#define EXPECTED_DEG_COOLANT 85 // heater should be off above this value

#define LED_BUILTIN 13
#define BUTTON1_PIN 2
#define IGNSWITCH_PIN 3
#define PUMP1_PIN 4
#define PUMP2_PIN 5
#define HEATER_PIN 6
#define BLOWER_PWM_PIN 7

#define BATTERY_PIN 0
#define BATTERY_CHARGING_VOLTAGE 13.5
#define IDEG_PIN 1
#define CDEG_PIN 2
#define EDEG_PIN 3

unsigned long millis_t, loop_t, voltage_bank_t, ic_deg_bank_t, ext_deg_bank_t, button1_t, button1_td, menu_t;
unsigned long pump_t, blower_pwm_t, hvac_manual_mode_t, hvac_camping_mode_t, heater_beep_t;
bool button1_active = false;
bool voltage_bank_changed = false;
bool ic_deg_bank_changed = false;
bool ext_deg_bank_changed = false;
bool system_on = false;
bool winter_season = false;
bool engine_running = false;
bool int_deg_reached = true; // temperature is reached at init (no command given)
bool coolant_deg_reached = true; // temperature is reached at init (no command given)

/* screens
   0 - battery_voltage and status
   1 - heater
   2 - temperature
   3 - season
*/
const unsigned int screens = 3;
unsigned int current_screen = 0;
bool setup_mode = false;
bool reset_menu_on_timeout = false;

bool hvac_manual_mode_on = false;
bool hvac_remote_mode_on = false;
bool hvac_camping_mode_on = false;

volatile bool ignswitch_on = false;

bool blower_pwm_on = false;
bool heater_on = false;
bool pump1_on = false;
bool pump2_on = false;

byte expected_int_deg, setup_mode_deg, expected_season, setup_mode_season;

// analog adjustment stuff
const float ANALOG_UNIT = VOLTAGE / 1024;

// a simple divider circuit is used for measuring battery_voltage (percent of main)
const float VOLTAGE_DIVIDER = R2/(R1+R2);

float battery_voltage = 9.5; // assume battery malfunction
int int_deg = EXPECTED_DEG_MAX; // assume internal temperature is MAX
int coolant_deg = 90; // assume coolant temperature is 90 deg.
int ext_deg = EXPECTED_DEG_MAX; // assume external temperature is MAX (summer)
byte blower_pwm_value = 0; // assume blower pwm is zero

void setup() {

  Serial.begin(9600);
  Serial.println("This is a new run.");

  pinMode(LED_BUILTIN, OUTPUT);

  // running indicator
  // ToDo: add code for buzzer
  for (int i=0; i<5; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
  }

  // analog I/O
  pinMode(BATTERY_PIN, INPUT);
  pinMode(IDEG_PIN, INPUT);
  pinMode(CDEG_PIN, INPUT);
  pinMode(EDEG_PIN, INPUT);

  // digital I/O
  pinMode(BUTTON1_PIN, INPUT);
  digitalWrite(BUTTON1_PIN, INPUT_PULLUP);

  pinMode(IGNSWITCH_PIN, INPUT);
  digitalWrite(IGNSWITCH_PIN, INPUT_PULLUP);

  pinMode(PUMP1_PIN, OUTPUT);
  digitalWrite(PUMP1_PIN, LOW);

  pinMode(PUMP2_PIN, OUTPUT);
  digitalWrite(PUMP2_PIN, LOW);

  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);

  pinMode(BLOWER_PWM_PIN, OUTPUT);
  digitalWrite(BLOWER_PWM_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(IGNSWITCH_PIN), ignSwitch, CHANGE);

  // get ignition switch status
  ignSwitch();

  millis_t = millis();

  button1_active = false;
  button1_t = 0UL; // timer for button1
  button1_td = 0UL; // time delta for button1
  ic_deg_bank_t = 0UL; // timer for sensors polling
  ext_deg_bank_t = 0UL; // timer for sensors polling
  voltage_bank_t = 0UL; // timer for sensors polling
  pump_t = 0UL; // timer for pumps
  hvac_manual_mode_t = 0UL; // runtime for manual mode HVAC
  hvac_camping_mode_t = 0UL; // runtime for camping mode HVAC
  heater_beep_t = 0UL; // periodic beep for heater on
  menu_t = millis_t; // timer for menu
  loop_t = millis_t; // timer for loop

  // read settings from EEPROM
  expected_season = EEPROM.read(EXPECTED_SEASON_EEPROM_ADDR);
  expected_int_deg = EEPROM.read(EXPECTED_DEG_EEPROM_ADDR);

  // if settings are not present in EEPROM, store defaults
  if (expected_season < EXPECTED_SEASON_MIN || expected_season > EXPECTED_SEASON_MAX || expected_int_deg < EXPECTED_DEG_MIN || expected_int_deg > EXPECTED_DEG_MAX) {
    Serial.println("Settings in EEPROM are not valid, re-writing defaults...");
    expected_int_deg = EXPECTED_DEG_DEFAULT;
    expected_season = EXPECTED_SEASON_DEFAULT;
    writeEEPROM(EXPECTED_SEASON_EEPROM_ADDR, expected_season, EXPECTED_SEASON_MIN, EXPECTED_SEASON_MAX);
    writeEEPROM(EXPECTED_DEG_EEPROM_ADDR, expected_int_deg, EXPECTED_DEG_MIN, EXPECTED_DEG_MAX);
  }
}

void loop() {
  // start a timer at loop start
  loop_t = millis();

  // restore screen if inactive
  if (loop_t - button1_t > MENU_TIMEOUTMS && current_screen != 0) {
    current_screen = 0;
    setup_mode = false;
  }

  // get button active duration
  if (readButton1()) {
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
    // resetting the menu timeout
    menu_t = loop_t;
    reset_menu_on_timeout = true;
    // short press
    if (button1_td < BUTTON_LONGMS && system_on) {
      button1ShortPress();
    }
    //long press
    if (button1_td >= BUTTON_LONGMS && button1_td < BUTTON_POWERMS && system_on) {
      button1LongPress();
    }
    // power on/off
    if (button1_td >= BUTTON_POWERMS && button1_td  < BUTTON_MFMS) {
      setup_mode = false;
      current_screen = 0;
      switchOnOff();
    }
    if (button1_td >= BUTTON_MFMS) {
      buttonError();
    }
    // after actions were performed, reset the button delta
    button1_td = 0UL;
  }

  //ToDo: monitor (and display) voltage when off, alerts if problems
  if (voltage_bank_changed) {
    if (battery_voltage > BATTERY_CHARGING_VOLTAGE && ignswitch_on) {
      if (!engine_running) {
        Serial.println("Engine is running.");
        engine_running = true;
      }
    } else {
      if (engine_running) {
        Serial.println("Engine is stopped.");
        engine_running = false;
      }
    }
    voltage_bank_changed = false;
  }

  if (system_on) {
    // sensors polling
    if (loop_t - ic_deg_bank_t > POLLINGMS1) {
      ic_deg_bank_changed = readICDegBank();
      ic_deg_bank_t = millis();
    }
    if (loop_t - voltage_bank_t > POLLINGMS1) {
      voltage_bank_changed = readVoltageBank();
      voltage_bank_t = millis();
    }
    if (loop_t - ext_deg_bank_t > POLLINGMS2) {
      ext_deg_bank_changed = readEDegBank();
      ext_deg_bank_t = millis();
    }

    // determine current season
    if (expected_season == 1) {
      if (ext_deg_bank_changed) {
        if (isWinterSeason(ext_deg) && !winter_season) {
          Serial.println("Winter season.");
          winter_season = true;
          ic_deg_bank_changed = true;
        } else if (!isWinterSeason(ext_deg) && winter_season) {
          Serial.println("Summer season.");
          winter_season = false;
          ic_deg_bank_changed = true;
        }
        ext_deg_bank_changed = false;
      }
    } else if (expected_season == 3 && !winter_season) {
      Serial.println("Winter season.");
      winter_season = true;
      ic_deg_bank_changed = true;
    } else if (expected_season == 2 && winter_season) {
      Serial.println("Summer season.");
      winter_season = false;
      ic_deg_bank_changed = true;
    }

    // when temperature changes, re-evaluate conditions
    if (ic_deg_bank_changed) {
      if (winter_season) {
        int_deg < expected_int_deg ? int_deg_reached = false : int_deg_reached = true;
      } else {
        int_deg > EXPECTED_DEG_SUMMER_MAX ? int_deg_reached = false : int_deg_reached = true;
      }
      coolant_deg < EXPECTED_DEG_COOLANT ? coolant_deg_reached = false : coolant_deg_reached = true;
      ic_deg_bank_changed = false;
    }

    if (winter_season) {
    // winter is here
      if (engine_running && ignswitch_on) {
        if (!coolant_deg_reached) {
          if (!pump1_on) { pump1_turn_on(); }
          if (pump2_on) { pump2_turn_off(); }
          if (!heater_on) { heater_turn_on(); }
        } else {
          if (heater_on) { heater_turn_off(); }
        }
      } else if (hvac_manual_mode_on || hvac_remote_mode_on) {
        if (!pump1_on) { pump1_turn_on(); }
        if (pump2_on) { pump2_turn_off(); }
        if (!heater_on) { heater_turn_on(); }
        if (!blower_pwm_on || blower_pwm_value != BLOWER_PWM_WINTER ) { blower_pwm_turn_on(BLOWER_PWM_WINTER); }
      } else if (hvac_camping_mode_on) {
        if (pump1_on) { pump1_turn_off(); }
        if (!int_deg_reached) {
          if (!pump2_on) { pump2_turn_on(); }
          if (!heater_on) { heater_turn_on(); }
          if (!blower_pwm_on || blower_pwm_value != BLOWER_PWM_CAMPING) { blower_pwm_turn_on(BLOWER_PWM_CAMPING); }
        } else {
          if (heater_on) { heater_turn_off(); }
        }
      } else {
        if (heater_on) { heater_turn_off(); }
      }
    } else {
      // summer is here
      if (heater_on) { heater_turn_off(); }
      if (hvac_manual_mode_on || hvac_remote_mode_on) {
          if (!blower_pwm_on || blower_pwm_value != BLOWER_PWM_SUMMER) { blower_pwm_turn_on(BLOWER_PWM_SUMMER); }
      } else if (hvac_camping_mode_on) {
        if (!int_deg_reached) {
          if (!blower_pwm_on || blower_pwm_value != BLOWER_PWM_SUMMER) { blower_pwm_turn_on(BLOWER_PWM_SUMMER); }
        } else {
          if (blower_pwm_on) { blower_pwm_turn_off(); }
        }
      } else {
        if (blower_pwm_on) { blower_pwm_turn_off(); }
      }
    }

    // runtime will complete no matter the season
    if (winter_season) {
      if (hvac_manual_mode_on && loop_t - hvac_manual_mode_t > HVAC_WINTER_MODE_RUNTIMEMS) {
        Serial.println("Runtime reached for winter.");
        hvac_manual_mode_on = false;
      }
    } else {
      if (hvac_manual_mode_on && loop_t - hvac_manual_mode_t > HVAC_SUMMER_MODE_RUNTIMEMS) {
        Serial.println("Runtime reached for summer.");
        hvac_manual_mode_on = false;
      }
    }
    if (hvac_camping_mode_on && loop_t - hvac_camping_mode_t > HVAC_CAMPING_MODE_RUNTIMEMS) {
      Serial.println("Runtime reached for camping.");
      hvac_camping_mode_on = false;
    }
    // beep every minute if heater on
    if (heater_on && loop_t - heater_beep_t > HEATER_BEEP_INTERVAL) {
      // ToDo: short beep
      Serial.println("Heater on beep!");
      heater_beep_t = loop_t;
    }
    // blower_pwm will run for a while even after stop command
    if (!heater_on && winter_season && loop_t - blower_pwm_t > BLOWER_PWM_TIMEOUTMS) {
      if (blower_pwm_on) { blower_pwm_turn_off(); }
    }
  } else {
    // system is off
    if (heater_on) { heater_turn_off(); }
    if (blower_pwm_on) { blower_pwm_turn_off(); }
    hvac_camping_mode_on = false;
    hvac_manual_mode_on = false;
  }

  // pumps will run for a while on heater stop even after off command
  if (!heater_on && (pump1_on || pump2_on) && loop_t - pump_t > PUMP_TIMEOUTMS) {
    if (pump1_on) { pump1_turn_off(); }
    if (pump2_on) { pump2_turn_off(); }
  }
}

void ignSwitch() {
  if (blower_pwm_on) { blower_pwm_turn_off(); }
  if (heater_on) { heater_turn_off(); }
  digitalRead(IGNSWITCH_PIN) == LOW ? ignswitch_on = true : ignswitch_on = false;
  Serial.print("Ignition switch:");
  Serial.println(ignswitch_on);
  hvac_manual_mode_on = false;
  hvac_camping_mode_on = false;
  hvac_remote_mode_on = false;
}

bool isWinterSeason(int deg) {
  if (expected_season == 1 && deg < SEASON_AUTO_SWITCH_DEG) {
    return true;
  }
  if (expected_season == 3) {
    return true;
  }
  return false;
}

void button1ShortPress() {
  // ToDo: beep short press
  if (!setup_mode) {
    current_screen < screens ? current_screen ++ : current_screen = 0;
    Serial.print("New current screen:");
    Serial.println(current_screen);
  } else {
    switch (current_screen) {
      case 2:
        // increment degrees
        setup_mode_deg == EXPECTED_DEG_MAX ? setup_mode_deg = EXPECTED_DEG_MIN : setup_mode_deg += EXPECTED_DEG_UNIT_INCREMENT;
        Serial.print("Setting degrees:");
        Serial.println(setup_mode_deg);
        break;
      case 3:
        // switch between auto, summer and winter modes
        setup_mode_season == EXPECTED_SEASON_MAX ? setup_mode_season = EXPECTED_SEASON_MIN : setup_mode_season++;
        Serial.print("Setting season:");
        Serial.println(setup_mode_season);
        break;
    }
  }
}

void button1LongPress() {
  // ToDo: beep long press
  if (!setup_mode) {
    setup_mode = true;
    switch (current_screen) {
      case 0:
        /* the main screen is zero, showing battery_voltage and status
           on long press, hvac manual mode is toggled
        */
        if (!engine_running && !ignswitch_on) {
          // ToDo: beep setup
          if (hvac_manual_mode_on) {
            // ToDo: beep off
            hvac_manual_mode_on = false;
          } else {
            // ToDo: beep on
            hvac_manual_mode_on = true;
            hvac_camping_mode_on = false;
            hvac_manual_mode_t = loop_t;
          }
          Serial.print("HVAC manual on:");
          Serial.println(hvac_manual_mode_on);
        } else {
          // ToDo: beep unavailable
        }
        setup_mode = false;
        break;
      case 1:
        /* screen one shows heater info (HE:--/minutes/On)
           on long press, camping mode is toggled
        */
        if (!engine_running && !ignswitch_on) {
          // ToDo: beep setup
          if (hvac_camping_mode_on) {
            // ToDo: beep off
            hvac_camping_mode_on = false;
          } else {
            // ToDo: beep on
            hvac_camping_mode_on = true;
            hvac_manual_mode_on = false;
            hvac_camping_mode_t = loop_t;
          }
          Serial.print("Camping mode on:");
          Serial.println(hvac_camping_mode_on);
        } else {
          // ToDo: beep unavailable
        }
        setup_mode = false;
        break;
      case 2:
        // ToDo: beep setup
        Serial.println("Adjust mode enabled for temperature.");
        setup_mode_deg = expected_int_deg;
        break;
      case 3:
        // ToDo: beep setup
        Serial.println("Adjust mode enabled for season.");
        setup_mode_season = expected_season;
        break;
    }

  } else {
    switch (current_screen) {
      case 2:
        if (expected_int_deg != setup_mode_deg) {
          // ToDo: setting stored
          expected_int_deg = setup_mode_deg;
          writeEEPROM(EXPECTED_DEG_EEPROM_ADDR, expected_int_deg, EXPECTED_DEG_MIN, EXPECTED_DEG_MAX);
        } else {
          // ToDo: beep setting the same
        }
        Serial.print("Expected degrees:");
        Serial.println(expected_int_deg);
        Serial.print("Exiting setup mode for screen:");
        Serial.println(current_screen);
        setup_mode = false;
        // do not exit to main screen, wait for timeout
        break;
      case 3:
        if (expected_season != setup_mode_season) {
          // ToDo: beep setting stored
          expected_season = setup_mode_season;
          writeEEPROM(EXPECTED_SEASON_EEPROM_ADDR, expected_season, EXPECTED_SEASON_MIN, EXPECTED_SEASON_MAX);
        } else {
          // ToDo: beep setting the same
        }
        Serial.print("Current season:");
        Serial.println(expected_season);
        Serial.print("Exiting setup mode for screen:");
        Serial.println(current_screen);
        setup_mode = false;
        // do not exit to main screen, wait for timeout
        break;
    }
  }
}

void switchOnOff() {
  if (system_on) {
    // ToDo: beep system off
    heater_turn_off;
    blower_pwm_turn_off;
    system_on = false;
  } else {
    // ToDo: beep system on
    system_on = true;
  }
  Serial.print("Power:");
  Serial.println(system_on);
}

void buttonError() {
  // ToDo: beep button error
  Serial.println("Button contact error!");
}

bool readButton1() {
  return !digitalRead(BUTTON1_PIN);
}

bool readICDegBank() {
  bool changed = false;
  int value = 0;
  value = readTemperature(IDEG_PIN);
  if (value != int_deg) {
    changed = true;
    int_deg = value;
  }
  value = readTemperature(CDEG_PIN);
  if (value != coolant_deg) {
    changed = true;
    coolant_deg = value;
  }
  return changed;
}

bool readVoltageBank() {
  bool changed = false;
  float value = 0.0;
  value = readBatteryVoltage(BATTERY_PIN);
  if (value != battery_voltage) {
    changed = true;
    battery_voltage = value;
  }
  return changed;
}

bool readEDegBank() {
  bool changed = false;
  int value = 0;
  value = readTemperature(EDEG_PIN);
  if (value != ext_deg) {
    changed = true;
    ext_deg = value;
  }
  return changed;
}

float readBatteryVoltage(int pin) {
  int raw = analogRead(pin);
  int value = round(((raw * ANALOG_UNIT) / VOLTAGE_DIVIDER) * 10.0);
  return value / 10.0;
}

int readTemperature(int pin) {
  int raw = analogRead(pin);
  float value = ((raw * ANALOG_UNIT) - 0.5) * 100;
  return round(value);
}

void writeEEPROM(int addr, byte value, byte min, byte max) {
  if (value < min && value > max) {
    Serial.println("Refusing to write bogus values to EEPROM!");
  } else {
    EEPROM.write(addr, value);
  }
}

void heater_turn_on() {
  digitalWrite(HEATER_PIN, HIGH);
  Serial.println("Heater turned on!");
  heater_on = true;
}

void heater_turn_off() {
  digitalWrite(HEATER_PIN, LOW);
  Serial.println("Heater turned off!");
  pump_t = loop_t;
  blower_pwm_t = loop_t;
  heater_on = false;
}

void blower_pwm_turn_on(int power) {
  digitalWrite(BLOWER_PWM_PIN, power);
  Serial.print("Blower turned on:");
  Serial.println(power);
  blower_pwm_on = true;
  blower_pwm_value = power;
}

void blower_pwm_turn_off() {
  digitalWrite(BLOWER_PWM_PIN, LOW);
  Serial.println("Blower turned off!");
  blower_pwm_on = false;
  blower_pwm_value = 0;
}

void pump1_turn_on() {
  digitalWrite(PUMP1_PIN, HIGH);
  Serial.println("Pump1 turned on!");
  pump1_on = true;
}

void pump1_turn_off() {
  digitalWrite(PUMP1_PIN, LOW);
  Serial.println("Pump1 turned off!");
  pump1_on = false;
}

void pump2_turn_on() {
  digitalWrite(PUMP2_PIN, HIGH);
  Serial.println("Pump2 turned on!");
  pump2_on = true;
}

void pump2_turn_off() {
  digitalWrite(PUMP2_PIN, LOW);
  Serial.println("Pump2 turned off!");
  pump2_on = false;
}
