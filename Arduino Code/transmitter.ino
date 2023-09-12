/*
   Project:- LENS CONTROL SYSTEM
   Developer:- SAMRAT DUTTA
   3DprintingAndRobotics
   TRANSMITTER CODE
*/

/* Pinouts
   A7- NULL
   A6- Voltage Probe Pin
   A5- SCL
   A4- SDA
   A3- Rotary Encoder Button_3
   A2- Rotary Encoder Button_2
   A1- Rotary Encoder dt_1
   A0- Rotary Encoder clk_1
   D13- SCK
   D12- MISO
   D11- MOSI
   D10- CSN
   D9- CS
   D8- Rotary Encoder dt_3
   D7- Rotary Encoder clk_3
   D6- Rotary Encoder dt_2
   D5- Rotary Encoder clk_2
   D4- Buzzer
   D3- Exit Button
   D2- Rotary Encoder Button_1
   D1- NULL
   D0- NULL
*/

/*
   voltage constant value is '0.015329245' [R1-22K, R2-47K]
   LCD A displays all Menu and Settings
   LCD B displays live coordinates of Zoom, Focus and Battery Health
*/

// Required Libraries
#include "Arduino.h"
#include "PinChangeInterrupt.h"
#include <SPI.h>
#include "RF24.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

#define settingsMenuLength 8

// Pin decleration
#define voltageProbePin     A6
// #define SCL              A5
// #define SDA              A4
#define rotary_button_3     A3
#define rotary_button_2     A2
#define dt_1                A1
#define clk_1               A0
// #define clk              13
// #define MOSI             12
// #define MISO             11
#define csn                 10
#define ce                  9
#define dt_3                8
#define clk_3               7
#define dt_2                6
#define clk_2               5
#define photo_switch        4
#define exit_switch         3
#define rotary_button_1     2

RF24 myRadio (ce, csn); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

LiquidCrystal_I2C lcdA(EEPROM.read(9), 16, 2);
LiquidCrystal_I2C lcdB(EEPROM.read(10), 16, 2);

int lcdAcursor = 0;
int maxZTA = 0, maxFTA = 0, picTime = 0, velocity = 650, lastReadVoltage = 0, kf = 1, totalPic = 0;
float x = 0.00, focus = 0.00, zoomUp = 0.00, zoomDown = 0.00, liveZoom_F = 0.00, liveFocus_F = 0.00, prescalerZoom_F = 0.00, prescalerFocus_F = 0.00, voltage = 0.00;
volatile bool updateLCDB = false, actualCoordinates = false, change = false, sync = false;
volatile bool exitPressed = false, photoPressed = false, motor_state = false;
volatile bool manualMode = false, keyFrameMode = false, RadioPower = false;
volatile bool buttonPressed_1 = false, TurnDetected_1 = false, up_1 = false, prev_clk_1 = LOW, prev_data_1 = LOW;
volatile bool buttonPressed_2 = false, TurnDetected_2 = false, up_2 = false, up_3 = false, buttonPressed_3 = false, TurnDetected_3 = false;
volatile int liveZoom = 0, liveFocus = 0, prescalerZoom = 1, prescalerFocus = 1;

unsigned long Time = 0, cache = 0;

struct package0
{
  byte d1 = 0; // mode
  int d2 = 0; // focus
  int d3 = 0; // Zoom
  int d4 = 0; // Speed
  bool d5 = false; // Sync or not sync
  bool d6 = false; // Photograph
  int d7 = 0; // delay time
};
typedef struct package0 Package0;
Package0 Data;

void initialiseDataPackage() {
  Data.d1 = 0; // mode
  Data.d2 = 0; // focus
  Data.d3 = 0; // Zoom
  Data.d4 = 0; // Speed
  Data.d5 = false; // sync or not sync
  Data.d6 = false; // photograph
  Data.d7 = 0;
}
void resetInputVariables() {
  TurnDetected_1 = false;
  TurnDetected_2 = false;
  TurnDetected_3 = false;
  buttonPressed_1 = false;
  buttonPressed_2 = false;
  buttonPressed_3 = false;
  exitPressed = false;
  photoPressed = false;
}
void factoryReset() {
  EEPROM.write(0, 0);
  EEPROM.write(1, 0);
  EEPROM.write(2, 1);
  EEPROM.write(3, 1);
  EEPROM.write(4, 0);
  EEPROM.write(5, 16);
  EEPROM.write(6, 80);
  EEPROM.write(7, 27);
  EEPROM.write(8, 115);
  EEPROM.write(9, 39);
  EEPROM.write(10, 63);
  EEPROM.write(11, 140);
  EEPROM.write(12, 18);
  EEPROM.write(13, 3);
  EEPROM.write(14, 235);
  EEPROM.write(15, 3);
  EEPROM.write(16, 235);
}


String string[28] = {
  "Connecting",       // 0
  "Please Wait!",     // 1
  "Press Exit",       // 2
  "When it Beeps!",   // 3
  "Set Frames",       // 4
  "Manual",           // 5
  "Settings",         // 6
  "Program ERROR",    // 7
  "Z-",               // 8
  "F-",               // 9
  "Sync-",            // 10
  "Yes",              // 11
  "No",               // 12
  "S-",               // 13
  "Homing",           // 14
  "Set Zoom to 18mm", // 15
  "when completed",   // 16
  "Low Battery!",     // 17
  "F",                // 18
  "Back",             // 19
  "OK",               // 20
  "Pictures-",        // 21
  "Run-",             // 22
  "Start:",           // 23
  "Auto",             // 24
  "Next",             // 25
  "T-",               // 26
  "R-",               // 27
};

String settings[settingsMenuLength] = {
  "Home",             // 0  Data.d1= 3
  "Swap F/Z Frame",   // 1
  "L/U Motors",       // 2  Data.d1= 4
  "Swap LCD",         // 3
  "Swap F/Z Knob",    // 4
  "Radio Power",      // 5  Data.d1= 5
  "Battery",          // 6  Data.d1= 6
  "Factory Reset",    // 7  Data.d1= 7
};

// Buttons Handler
void exitButtonPressed() {
  exitPressed = true;
}
void photoButtonPressed() {
  photoPressed = true;
}
void rotaryButtonPressed_1() {
  buttonPressed_1 = true;
  if (manualMode) {
    sync = !sync;
  }
}
void rotaryButtonPressed_2() {
  updateLCDB = true;
  if (!manualMode && !keyFrameMode) buttonPressed_2 = true;
  else {
    if (buttonPressed_2) buttonPressed_2 = false;
    else buttonPressed_2 = true;
  }
}
void rotaryButtonPressed_3() {
  updateLCDB = true;
  if (!manualMode && !keyFrameMode) buttonPressed_3 = true;
  else {
    if (buttonPressed_3) buttonPressed_3 = false;
    else buttonPressed_3 = true;
  }
}

// Encoders Handler
void rotaryMotion_1() {
  // clk- A0     dt- A1
  if ((prev_clk_1 == LOW) && (prev_data_1 == HIGH)) {
    if ((PINC & B00000001) == 1 && (PINC & B00000010) == 0) {
      up_1 = true;
      TurnDetected_1 = true;
      return;
    }
    if ((PINC & B00000001) == 1 && (PINC & B00000010) == 1) {
      up_1 = false;
      TurnDetected_1 = true;
      return;
    }
  }
  if ((prev_clk_1 == HIGH) && (prev_data_1 == LOW)) {
    if ((PINC & B00000001) == 0 && (PINC & B00000010) == 1) {
      up_1 = true;
      TurnDetected_1 = true;
      return;
    }
    if ((PINC & B00000001) == 0 && (PINC & B00000010) == 0) {
      up_1 = false;
      TurnDetected_1 = true;
      return;
    }
  }
  if ((prev_clk_1 == HIGH) && (prev_data_1 == HIGH)) {
    if ((PINC & B00000001) == 0 && (PINC & B00000010) == 1) {
      up_1 = true;
      TurnDetected_1 = true;
      return;
    }
    if ((PINC & B00000001) == 0 && (PINC & B00000010) == 0) {
      up_1 = false;
      TurnDetected_1 = true;
      return;
    }
  }
  if ((prev_clk_1 == LOW) && (prev_data_1 == LOW)) {
    if ((PINC & B00000001) == 1 && (PINC & B00000010) == 0) {
      up_1 = true;
      TurnDetected_1 = true;
      return;
    }
    if ((PINC & B00000001) == HIGH && (PINC & B00000010) == HIGH) {
      up_1 = false;
      TurnDetected_1 = true;
      return;
    }
  }
  if (manualMode) {
    if (up_1) {
      velocity++;
      if (velocity > 1000) velocity = 1000;
      Data.d4 = (1000 - velocity);
    }
    else {
      velocity--;
      if (velocity < 1) velocity = 1;
      Data.d4 = (1100 - velocity);
    }
  }
}

void rotaryMotion_2 () {
  TurnDetected_2 = true;
  updateLCDB = true;
  up_2 = ((digitalRead(clk_2) == digitalRead(dt_2)));

  if ((manualMode || keyFrameMode) && !buttonPressed_2) {
    if (up_2) {
      liveZoom = (liveZoom + prescalerZoom);
      if (liveZoom > maxZTA) liveZoom = maxZTA;
    }
    else {
      liveZoom = (liveZoom - prescalerZoom);
      if (liveZoom < 0) liveZoom = 0;
    }
    Data.d3 = liveZoom;
    change = true;
    if (!actualCoordinates) liveZoom_F = (((float)(liveZoom)) * ((zoomUp - zoomDown) / ((float)(maxZTA)))) + zoomDown;
    return;
  }
  else if ((manualMode || keyFrameMode) && buttonPressed_2) {
    if (up_2) {
      prescalerZoom++;
      if (prescalerZoom > 250) prescalerZoom = 250;
    }
    else {
      prescalerZoom--;
      if (prescalerZoom < 1) prescalerZoom = 1;
    }
    if (!actualCoordinates) prescalerZoom_F = (((float)(prescalerZoom)) * (zoomUp / ((float)(maxZTA))));
    return;
  }
}
void rotaryMotion_3 () {
  TurnDetected_3 = true;
  updateLCDB = true;
  up_3 = ((digitalRead(clk_3) == digitalRead(dt_3)));
  if ((manualMode || keyFrameMode) && !buttonPressed_3) {
    if (up_3)  {
      liveFocus = (liveFocus + prescalerFocus);
      if (liveFocus > maxFTA) liveFocus = maxFTA;
    }
    else {
      liveFocus = (liveFocus - prescalerFocus);
      if (liveFocus < 0) liveFocus = 0;
    }
    Data.d2 = liveFocus;
    change = true;
    if (!actualCoordinates) liveFocus_F = liveFocus * (focus / ((float)(maxFTA)));
    return;
  }
  else if ((manualMode || keyFrameMode) && buttonPressed_3) {
    if (up_3) {
      prescalerFocus += 1;
      if (prescalerFocus > 250) prescalerFocus = 250;
    }
    else {
      prescalerFocus -= 1;
      if (prescalerFocus < 1) prescalerFocus = 1;
    }
    if (!actualCoordinates) prescalerFocus_F = ((float)(prescalerFocus)) * (focus / ((float)(maxFTA)));
    return;
  }
}

byte batteryLevelGraphics[8] = {
  0b01110,
  0b11111,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b11111
};
byte plusMinusGraphics [8] = {
  0b00100,
  0b00100,
  0b11111,
  0b00100,
  0b00100,
  0b00000,
  0b11111,
  0b00000
};
byte navigatingArrowGraphics [8] = {
  0b00000,
  0b01000,
  0b01100,
  0b01110,
  0b01110,
  0b01100,
  0b01000,
  0b00000
};
byte navigatingArrowInvertedGraphics [8] = {
  0b00000,
  0b00010,
  0b00110,
  0b01110,
  0b01110,
  0b00110,
  0b00010,
  0b00000
};
byte photographGraphics[8] = {
  0b00000,
  0b11000,
  0b11111,
  0b11111,
  0b11001,
  0b11001,
  0b11111,
  0b11111
};
byte radioHighPower_Graphics[8] = {
  0b00001,
  0b00010,
  0b00100,
  0b01000,
  0b11111,
  0b11001,
  0b11001,
  0b11111
};

bool checkVoltage() {
  bool dv = false;
  voltage = analogRead(voltageProbePin);
  if (abs(lastReadVoltage - voltage) > 5.00) {
    lastReadVoltage = voltage;
    voltage *= 0.015329245;
    voltage -= 0.06; // 0.06 is the +error voltage, thus it is removed
    if ((abs(voltage) - x) > 0.02) {
      x = voltage;
      dv = true;
    }
    if (x < 6.80) {
      byte counter = 0;
      bool temp1 = manualMode;
      bool temp2 = keyFrameMode;
      manualMode = false;
      keyFrameMode = false;
      lcdA.clear();
      lcdA.setCursor(2, 0);
      lcdA.print(string[17]);
      while (true) {
        delay(10);
        voltage = analogRead(voltageProbePin);
        if (abs(lastReadVoltage - voltage) > 5.00) {
          lastReadVoltage = voltage;
          voltage *= 0.015329245;
          voltage -= 0.06; // 0.06 is the +error voltage, thus it is removed
          if ((abs(voltage) - x) > 0.02) {
            x = voltage;
            dv = true;
          }
        }
        if (x > 6.8) {
          delay(1000);
          counter++;
        }
        if (counter > 5) {
          resetInputVariables();
          updateLCDB = false;
          manualMode = temp1;
          keyFrameMode = temp2;
          break;
        }
      }
    }
  }
  else {
    voltage = lastReadVoltage;
    voltage *= 0.015329245;
    voltage -= 0.06; // 0.06 is the +error voltage, thus it is removed
    if (abs((voltage) - x) > 0.02) {
      x = voltage;
      dv = false;
    }
    if (x < 6.80) {
      byte counter = 0;
      bool temp1 = manualMode;
      bool temp2 = keyFrameMode;
      manualMode = false;
      keyFrameMode = false;
      lcdA.clear();
      lcdA.setCursor(2, 0);
      lcdA.print(string[17]);
      while (true) {
        delay(10);
        voltage = analogRead(voltageProbePin);
        if (abs(lastReadVoltage - voltage) > 5.00) {
          lastReadVoltage = voltage;
          voltage *= 0.015329245;
          voltage -= 0.06; // 0.06 is the +error voltage, thus it is removed
          if ((abs(voltage) - x) > 0.02) {
            x = voltage;
            dv = true;
          }
        }
        if (x > 6.8) {
          delay(1000);
          counter++;
        }
        if (counter > 5) {
          resetInputVariables();
          updateLCDB = false;
          manualMode = temp1;
          keyFrameMode = temp2;
          break;
        }
      }
    }
  }
  if (x >= 7.80) { // full charge
    batteryLevelGraphics[2] = 0b11111;
    batteryLevelGraphics[3] = 0b11111;
    batteryLevelGraphics[4] = 0b11111;
    batteryLevelGraphics[5] = 0b11111;
    batteryLevelGraphics[6] = 0b11111;
  }
  else if (x >= 7.60 && x < 7.80) { // 80% charge
    batteryLevelGraphics[2] = 0b10001;
    batteryLevelGraphics[3] = 0b11111;
    batteryLevelGraphics[4] = 0b11111;
    batteryLevelGraphics[5] = 0b11111;
    batteryLevelGraphics[6] = 0b11111;
  }
  else if (x >= 7.40 && x < 7.60) { // 60% charge
    batteryLevelGraphics[2] = 0b10001;
    batteryLevelGraphics[3] = 0b10001;
    batteryLevelGraphics[4] = 0b11111;
    batteryLevelGraphics[5] = 0b11111;
    batteryLevelGraphics[6] = 0b11111;
  }
  else if (x >= 7.20 && x < 7.40) { // 40% charge
    batteryLevelGraphics[2] = 0b10001;
    batteryLevelGraphics[3] = 0b10001;
    batteryLevelGraphics[4] = 0b10001;
    batteryLevelGraphics[5] = 0b11111;
    batteryLevelGraphics[6] = 0b11111;
  }
  else if (x >= 7.00 && x < 7.20) { // 20% charge
    batteryLevelGraphics[2] = 0b10001;
    batteryLevelGraphics[3] = 0b10001;
    batteryLevelGraphics[4] = 0b10001;
    batteryLevelGraphics[5] = 0b10001;
    batteryLevelGraphics[6] = 0b11111;
  }
  else { // 1% charge
    batteryLevelGraphics[2] = 0b10001;
    batteryLevelGraphics[3] = 0b10001;
    batteryLevelGraphics[4] = 0b10001;
    batteryLevelGraphics[5] = 0b10001;
    batteryLevelGraphics[6] = 0b10001;
  }
  lcdA.createChar(1, batteryLevelGraphics);
  return dv;
}

void update_LCDA_Graphics() {
  lcdA.clear();
  lcdA.setCursor(1, 0);
  lcdA.print(string[4]);
  lcdA.setCursor(13, 0);
  if (RadioPower) lcdA.write(4);
  lcdA.setCursor(15, 0);
  lcdA.write(1);
  lcdA.setCursor(1, 1);
  lcdA.print(string[5]);
  lcdA.setCursor(8, 1);
  lcdA.print(string[6]);
  if (lcdAcursor == 0) {
    lcdA.setCursor(0, 0);
    lcdA.write(2);
  }
  else if (lcdAcursor == 1) {
    lcdA.setCursor(0, 1);
    lcdA.write(2);
  }
  else if (lcdAcursor == 2) {
    lcdA.setCursor(7, 1);
    lcdA.write(2);
  }
  else {
    lcdA.clear();
    lcdA.print(string[7]);
  }
}

void update_LCDB_Graphics() {
  if (actualCoordinates) {
    lcdB.clear();
    lcdB.setCursor(0, 0);
    lcdB.print(string[8]);
    lcdB.print(liveZoom);

    lcdB.setCursor(0, 1);
    lcdB.print(string[9]);
    lcdB.print(liveFocus);

    if (manualMode || keyFrameMode) { // printing prescalers

      lcdB.setCursor(9, 0);
      if (buttonPressed_2) lcdB.write(2);
      else lcdB.write(3);

      lcdB.setCursor(9, 1);
      if (buttonPressed_3) lcdB.write(2);
      else lcdB.write(3);

      lcdB.setCursor(11, 0);
      lcdB.write(1);
      lcdB.setCursor(13, 0);
      lcdB.print(prescalerZoom);

      lcdB.setCursor(11, 1);
      lcdB.write(1);
      lcdB.setCursor(13, 1);
      lcdB.print(prescalerFocus);
    }
  }
  else {
    lcdB.clear();
    lcdB.setCursor(0, 0);
    lcdB.print(string[8]);
    lcdB.print(liveZoom_F);

    lcdB.setCursor(0, 1);
    lcdB.print(string[9]);
    lcdB.print(liveFocus_F);

    if (manualMode || keyFrameMode) { // printing prescalers

      lcdB.setCursor(9, 0);
      if (buttonPressed_2) lcdB.write(2);
      else lcdB.write(3);

      lcdB.setCursor(9, 1);
      if (buttonPressed_3) lcdB.write(2);
      else lcdB.write(3);

      lcdB.setCursor(11, 0);
      lcdB.write(1);
      lcdB.print(prescalerZoom_F);

      lcdB.setCursor(11, 1);
      lcdB.write(1);
      lcdB.print(prescalerFocus_F);
    }
  }
}


void manual_Mode() {
  // user is asked for selecting synced or not synced motion
  initialiseDataPackage();
  myRadio.stopListening();
  Data.d1 = 1; // mode manual
  Data.d2 = liveFocus;
  Data.d3 = liveZoom;
  if (velocity > 1000) velocity = 1000;
  Data.d4 = 1100 - velocity;
  Data.d5 = false; // non synchronus mode
  Data.d6 = false;
  Data.d7 = 145; // parity value


  sync = false; // non synchronus mode
  manualMode = true; // all encoders become active
  update_LCDB_Graphics();

  checkVoltage();
  manual_LCDA();

  change = true;
  resetInputVariables();

  Time = millis();
  while (true) {
    prev_clk_1 = (PINC & B00000001); // reads A0
    prev_data_1 = (PINC & B00000010); // reades A1

    if (change) {
      myRadio.write(&Data, sizeof(Data));
      change = false;
    }

    if (millis() - Time >= 2000) {
      if (Data.d6) {
        Data.d6 = false;
        manual_LCDA();
      }
      Time = millis();
      if (checkVoltage()) manual_LCDA();
    }

    if (photoPressed) {
      Data.d6 = true; // take photo
      manual_LCDA();
      photoPressed = false;
      change = true;
      Time = (millis() - 1850);
    }

    if (updateLCDB) {
      updateLCDB = false;
      update_LCDB_Graphics();
    }

    if (exitPressed) {
      exitPressed = false;
      manualMode = false;
      resetInputVariables();
      initialiseDataPackage();
      myRadio.write(&Data, sizeof(Data));
      lcdAcursor = 0;
      update_LCDA_Graphics();
      update_LCDB_Graphics();
      return;
    }

    if (TurnDetected_1) {
      TurnDetected_1 = false;
      manual_LCDA();
    }
    if (buttonPressed_1) {
      change = true;
      buttonPressed_1 = false;
      if (sync) Data.d5 = true;
      else Data.d5 = false;
      manual_LCDA();
    }
  }
}
void manual_LCDA() {
  lcdA.clear();
  lcdA.setCursor(0, 0);
  lcdA.print(string[13]);
  lcdA.print(velocity);
  if (Data.d6) {
    lcdA.setCursor(13, 0);
    lcdA.write(3);
  }
  lcdA.setCursor(15, 0);
  lcdA.write(1);
  lcdA.setCursor(0, 1);
  lcdA.print(string[10]);
  if (sync) lcdA.print(string[11]);
  else lcdA.print(string[12]);
}

void settings_LCDA() {
  lcdA.clear();
  lcdA.setCursor(0, 0);
  lcdA.write(2);
  lcdA.setCursor(2, 0);
  lcdA.print(settings[lcdAcursor]);
  if (lcdAcursor < (settingsMenuLength - 1)) {
    lcdA.setCursor(2, 1);
    lcdA.print(settings[lcdAcursor + 1]);
  }
}
void settings_Mode() {
  lcdAcursor = 0;
  settings_LCDA();
  resetInputVariables();
  Time = millis();
  cache = millis();
  while (true) {
    prev_clk_1 = (PINC & B00000001); // reads A0
    prev_data_1 = (PINC & B00000010); // reades A1

    if (millis() - Time > 10000) exitPressed = true;

    if (millis() - cache >= 2000) {
      cache = millis();
      checkVoltage();
    }

    if (TurnDetected_1) {
      if (up_1) {
        lcdAcursor++;
        if (lcdAcursor > (settingsMenuLength - 1)) lcdAcursor = 0;
      }
      else {
        lcdAcursor--;
        if (lcdAcursor < 0) lcdAcursor = (settingsMenuLength - 1);
      }
      TurnDetected_1 = false;
      settings_LCDA();
      Time = millis();
    }


    if (buttonPressed_1) {
      buttonPressed_1 = false;
      int q = lcdAcursor;
      initialiseDataPackage();

      if (lcdAcursor == 0) { // home
        Data.d1 = 3;
        while (!myRadio.write(&Data, sizeof(Data)));
        initialiseDataPackage();
        liveZoom = 0;
        liveFocus = 0;
        liveZoom_F = 0.00;
        liveFocus_F = 0.00;
        update_LCDB_Graphics();
        Time = millis();
      }

      if (lcdAcursor == 1) { // Swap F/Z Frame
        if (EEPROM.read(0) == 0) {
          EEPROM.write(0, 1);
          actualCoordinates = true;
        }
        else {
          EEPROM.write(0, 0);
          actualCoordinates = false;
        }
        update_LCDB_Graphics();
        Time = millis();
      }

      if (lcdAcursor == 2) { // L/U Motor
        if (motor_state) motor_state = false;
        else motor_state = true;
        Data.d1 = 4;
        Data.d5 = motor_state;
        while (!myRadio.write(&Data, sizeof(Data)));
        initialiseDataPackage();
        Time = millis();
      }

      if (lcdAcursor == 3) { // Swap LCD
        byte c = EEPROM.read(9);
        EEPROM.write(9, EEPROM.read(10));
        EEPROM.write(10, c);
        reboot();
      }

      if (lcdAcursor == 4) { // Swap F/Z Knobs
        if (EEPROM.read(1) == 0) EEPROM.write(1, 1);
        else EEPROM.write(1, 0);
        reboot();
      }

      if (lcdAcursor == 5) { // RADIO POWER
        /////////////////////////////////////
        Data.d1 = 5;
        while (!myRadio.write(&Data, sizeof(Data)));
        Data.d1 = 0;
        if (EEPROM.read(4) == 0) {
          EEPROM.write(4, 1);
          RadioPower = true;
          Data.d5 = true;
        }
        else {
          EEPROM.write(4, 0);
          RadioPower = false;
          Data.d5 = false;
        }
        reboot();
      }

      if (lcdAcursor == 6) { // Battery
        Data.d1 = 6;
        checkVoltage();

        lcdA.clear();
        lcdA.setCursor(0, 0);
        lcdA.print(string[26]);
        lcdA.setCursor(3, 0);
        lcdA.print(x);
        lcdA.setCursor(10, 0);
        lcdA.print(x * 12.5);
        lcdA.setCursor(15, 0);
        lcdA.print("%");
        lcdA.setCursor(0, 1);
        lcdA.print(string[27]);
        lcdA.setCursor(3, 1);
        bool c = true;
        Time = millis();
        while (!myRadio.write(&Data, sizeof(Data))) {
          if (millis() - Time >= 3000) {
            initialiseDataPackage();
            c = false;
            break;
          }
        }
        if (c) {
          initialiseDataPackage();
          myRadio.startListening();
          delay(50);
          Time = millis();
          while (!myRadio.available()) {
            if (millis() - Time >= 3000) {
              initialiseDataPackage();
              c = false;
              break;
            }
          }
          if (c) {
            myRadio.read(&Data, sizeof(Data));
            myRadio.stopListening();

            lcdA.print((float)(Data.d1 + (Data.d2 / 100.00)));
            lcdA.setCursor(10, 1);
            lcdA.print((((float)(Data.d1)) + (((float)(Data.d2)) / 100.00)) * 8.33);
            lcdA.setCursor(15, 1);
            lcdA.print("%");

            initialiseDataPackage();
            delay(3000);
          }
        }
        settings_LCDA();
        Time = millis();
      }

      if (lcdAcursor == 7) { // factory reset
        lcdAcursor = 0;
        if (startYesNo()) {
          Data.d1 = 7;
          while (!myRadio.write(&Data, sizeof(Data)));
          factoryReset();
          initialiseDataPackage();
          reboot();
        }
        lcdAcursor = q;
      }
    }


    if (exitPressed) {
      exitPressed = false;
      resetInputVariables();
      lcdAcursor = 0;
      update_LCDA_Graphics();
      return;
    }
  }

}
void reboot() {
  lcdA.clear();
  lcdB.clear();
  lcdA.setCursor(3, 0);
  lcdA.print("Reboot Now");
  while (true);
}


bool startYesNo() {
  resetInputVariables();
  bool upd = true;
  while (true) {
    prev_clk_1 = (PINC & B00000001); // reads A0
    prev_data_1 = (PINC & B00000010); // reades A1
    if (millis() - Time >= 1000) {
      Time = millis();
      if (checkVoltage()) updateSetFrameLCDA();
    }
    if (upd) {
      upd = false;
      lcdA.clear();
      lcdA.setCursor(15, 0);
      lcdA.write(1);
      lcdA.setCursor(5, 0);
      lcdA.print(string[23]);
      lcdA.setCursor(2, 1);
      lcdA.print(string[11]);
      lcdA.setCursor(12, 1);
      lcdA.print(string[12]);
      if (lcdAcursor == 0) {
        lcdA.setCursor(1, 1);
        lcdA.write(2);
      }
      if (lcdAcursor == 1) {
        lcdA.setCursor(11, 1);
        lcdA.write(2);
      }
    }
    if (TurnDetected_1) {
      if (up_1) {
        lcdAcursor++;
        if (lcdAcursor > 1) lcdAcursor = 0;
      }
      else {
        lcdAcursor--;
        if (lcdAcursor < 0) lcdAcursor = 1;
      }
      TurnDetected_1 = false;
      upd = true;
    }
    if (buttonPressed_1) {
      buttonPressed_1 = false;
      resetInputVariables();
      if (lcdAcursor == 0) return true;
      if (lcdAcursor == 1) return false;
    }
  }
}
void Key_Frames_Mode() {
  myRadio.stopListening();
  initialiseDataPackage();
  resetInputVariables();
  velocity = 9500;
  Data.d1 = 2; // Key frame mode
  Data.d2 = liveFocus;
  Data.d3 = liveZoom;
  Data.d4 = 300;
  Data.d5 = false;
  Data.d7 = 0;

  checkVoltage();
  keyFrameMode = true; // activates encoders and auto updates
  kf = 1;
  totalPic = 0;
  lcdAcursor = 3;

  update_LCDB_Graphics();
  updateSetFrameLCDA();

  myRadio.write(&Data, sizeof(Data));

  Time = millis();
  while (true) {
    prev_clk_1 = (PINC & B00000001); // reads A0
    prev_data_1 = (PINC & B00000010); // reades A1

    if (change) {
      myRadio.write(&Data, sizeof(Data));
      change = false;
    }

    if (millis() - Time >= 2000) {
      Time = millis();
      if (checkVoltage()) updateSetFrameLCDA();
    }

    if (photoPressed) {
      photoPressed = false;
      if (Data.d6) Data.d6 = false;
      else Data.d6 = true;
      updateSetFrameLCDA();
    }

    if (updateLCDB) {
      updateLCDB = false;
      update_LCDB_Graphics();
    }

    if (TurnDetected_1) {
      if (up_1) {
        lcdAcursor++;
        if (lcdAcursor > 3) lcdAcursor = 0;
      }
      else {
        lcdAcursor--;
        if (lcdAcursor < 0) lcdAcursor = 3;
      }
      TurnDetected_1 = false;
      updateSetFrameLCDA();
    }

    if (buttonPressed_1) {
      buttonPressed_1 = false;

      if (lcdAcursor == 2) { // select sync
        if (sync) sync = false;
        else sync = true;
        Data.d5 = sync;
        updateSetFrameLCDA();
      }

      if (lcdAcursor == 3) { // select speed
        keyFrameMode = false;
        manualMode = false;
        TurnDetected_1 = false;
        TurnDetected_2 = false;
        TurnDetected_3 = false;
        buttonPressed_1 = false;
        buttonPressed_2 = false;
        buttonPressed_3 = false;
        while (true) {

          prev_clk_1 = (PINC & B00000001); // reads A0
          prev_data_1 = (PINC & B00000010); // reades A1

          if (millis() - Time >= 2000) {
            Time = millis();
            if (checkVoltage()) updateSetFrameLCDA();
          }

          if (buttonPressed_1) {
            TurnDetected_1 = false;
            TurnDetected_2 = false;
            TurnDetected_3 = false;
            buttonPressed_1 = false;
            buttonPressed_2 = false;
            buttonPressed_3 = false;
            keyFrameMode = true;
            Data.d4 = (10099 - velocity);
            break;
          }
          if (TurnDetected_1) {
            if (up_1) {
              velocity++;
              if (velocity > 9999)  velocity = 9999;
            }
            else {
              velocity--;
              if (velocity < 1) velocity = 1;
            }
            TurnDetected_1 = false;
            updateSetFrameLCDA();
          }

          if (TurnDetected_2) {
            if (up_2) {
              velocity += 10;
              if (velocity > 9999)  velocity = 9999;
            }
            else {
              velocity -= 10;
              if (velocity < 1) velocity = 1;
            }
            TurnDetected_2 = false;
            updateSetFrameLCDA();
          }
          if (TurnDetected_3) {
            if (up_3) {
              velocity += 50;
              if (velocity > 9999)  velocity = 9999;
            }
            else {
              velocity -= 50;
              if (velocity < 1) velocity = 1;
            }
            TurnDetected_3 = false;
            updateSetFrameLCDA();
          }
        }
      }

      if (lcdAcursor == 0 && kf <= 999) { // next frame  OK Pressed
        if (Data.d7 > 0) { // 126-back, 255=next
          Data.d1 = 126;
          if (!Data.d6) totalPic--;
          if (totalPic < 0) totalPic = 0;
        }
        else {
          Data.d1 = 255;
          if (Data.d6) totalPic++;
        }
        myRadio.write(&Data, sizeof(Data));
        Data.d1 = 2;
        Data.d7 = 0;
        kf++;
        if (kf > 999) kf = 999;
        update_LCDB_Graphics();
        updateSetFrameLCDA();
        delay(40);
      }
      if (lcdAcursor == 1) { // prev frame BACK Pressed
        if (kf <= 1) {
          Data.d1 = 0;
          myRadio.write(&Data, sizeof(Data));
          manualMode = false;
          keyFrameMode = false; // activates encoders and auto updates
          resetInputVariables();
          lcdAcursor = 0;
          update_LCDA_Graphics();
          update_LCDB_Graphics();
          return;
        }
        Data.d7++;
        kf--;
        update_LCDB_Graphics();
        updateSetFrameLCDA();
      }
    }

    if (exitPressed) {
      if (kf <= 1) {
        Data.d1 = 0;
        myRadio.write(&Data, sizeof(Data));
        manualMode = false;
        keyFrameMode = false; // activates encoders and auto updates
        resetInputVariables();
        lcdAcursor = 0;
        update_LCDA_Graphics();
        update_LCDB_Graphics();
        return;
      }
      kf--;
      exitPressed = false;
      manualMode = false;
      keyFrameMode = false;
      Data.d1 = 200;
      Data.d2 = kf;
      Data.d7 = picTime;
      myRadio.write(&Data, sizeof(Data));
      break;
    }

  }
  manualMode = false;
  keyFrameMode = false;

  initialiseDataPackage();

  lcdB.clear();
  lcdB.setCursor(0, 0);
  lcdB.print(string[4].substring(4, 9));
  lcdB.setCursor(6, 0);
  lcdB.print(kf);
  lcdB.setCursor(0, 1);
  lcdB.print(string[21]);
  lcdB.setCursor(10, 1);
  lcdB.print(totalPic);
  if (autoManual()) {
    bool man = false;
    if (lcdAcursor == 0) man = false;
    else man = true;
    lcdAcursor = 0;
    bool result = startYesNo();
    if (result) {
      if (!man) Data.d1 = 126;
      if (man) Data.d1 = 130;
      myRadio.write(&Data, sizeof(Data));

      if (man) { // manual mode
        bool upd = true;
        int x = 0;
        //kf--;
        manualMode = false;
        keyFrameMode = false;
        while (true) {

          if (millis() - Time >= 2000) {
            Time = millis();
            if (checkVoltage()) upd = true;
          }

          if (upd) {
            upd = false;
            lcdA.clear();
            lcdA.setCursor(0, 0);
            lcdA.print(string[4].substring(4, 9));
            lcdA.setCursor(6, 0);
            lcdA.print(x);
            lcdA.setCursor(15, 0);
            lcdA.write(1);
            lcdA.setCursor(2, 1);
            lcdA.print(string[25]);
            lcdA.setCursor(10, 1);
            lcdA.print(string[19]);
          }

          if (buttonPressed_1) { // next
            buttonPressed_1 = false;
            x++;
            if (x > kf) {
              Data.d1 = 0;
              myRadio.write(&Data, sizeof(Data));
              break;
            }
            Data.d1 = 100;
            myRadio.write(&Data, sizeof(Data));
            upd = true;
          }

          if (buttonPressed_2 || buttonPressed_3) { // prev
            buttonPressed_2 = false;
            buttonPressed_3 = false;

            if (x > 1) {
              x--;
              Data.d1 = 180;
              myRadio.write(&Data, sizeof(Data));
              upd = true;
            }
          }

          if (exitPressed) {
            exitPressed = false;
            Data.d1 = 0;
            myRadio.write(&Data, sizeof(Data));
            break;
          }
        }
      }
    }
  }
  lcdA.clear();
  lcdB.clear();
  lcdA.setCursor(2, 0);
  lcdA.print(string[1]);
  myRadio.startListening();
  delay(100);
  while (true) {
    while (!myRadio.available()) delayMicroseconds(1);
    while (myRadio.available()) myRadio.read(&Data, sizeof(Data));
    if (Data.d1 == 126) {
      liveZoom = 0;
      liveFocus = 0;
      liveFocus_F = 0.00;
      liveZoom_F = 18.00;
      myRadio.stopListening();
      delay(100);
      initialiseDataPackage();
      resetInputVariables();
      lcdAcursor = 0;
      update_LCDA_Graphics();
      update_LCDB_Graphics();
      return;
    }
  }

  Data.d1 = 0;
  myRadio.write(&Data, sizeof(Data));
  manualMode = false;
  keyFrameMode = false;
  initialiseDataPackage();
  resetInputVariables();
  lcdAcursor = 0;
  update_LCDA_Graphics();
  update_LCDB_Graphics();
}



bool autoManual() {
  lcdAcursor = 0;
  bool upd = true;
  while (true) {
    prev_clk_1 = (PINC & B00000001); // reads A0
    prev_data_1 = (PINC & B00000010); // reades A1
    if (millis() - Time >= 5000) {
      Time = millis();
      if (checkVoltage()) upd = true;
    }
    if (upd) {
      lcdA.clear();
      lcdA.setCursor(0, 0);
      lcdA.print(string[22]);
      lcdA.setCursor(6, 0);
      lcdA.print(string[24]);
      lcdA.setCursor(15, 0);
      lcdA.write(1);
      lcdA.setCursor(6, 1);
      lcdA.print(string[5]);
      if (lcdAcursor == 0) {
        lcdA.setCursor(5, 0);
        lcdA.write(2);
      }
      if (lcdAcursor == 1) {
        lcdA.setCursor(5, 1);
        lcdA.write(2);
      }
      upd = false;
    }
    if (TurnDetected_1) {
      if (up_1) {
        lcdAcursor++;
        if (lcdAcursor > 1) lcdAcursor = 0;
      }
      else {
        lcdAcursor--;
        if (lcdAcursor < 0) lcdAcursor = 0;
      }
      TurnDetected_1 = false;
      upd = true;
    }
    if (buttonPressed_1) {
      buttonPressed_1 = false;
      return true;
    }
  }
}


void updateSetFrameLCDA() {
  lcdA.clear();
  lcdA.setCursor(0, 0);
  lcdA.print(string[18]);
  lcdA.print(kf);
  lcdA.setCursor(4, 0);
  lcdA.print(string[25]);
  lcdA.setCursor(10, 0);
  lcdA.print(string[19]);
  if (Data.d6) {
    lcdA.setCursor(14, 0);
    lcdA.write(3);
  }
  lcdA.setCursor(15, 0);
  lcdA.write(1);
  lcdA.setCursor(1, 1);
  lcdA.print(string[13]);
  lcdA.print(velocity);
  lcdA.setCursor(8, 1);
  lcdA.print(string[10]);
  if (sync) {
    lcdA.setCursor(13, 1);
    lcdA.print(string[11]);
  }
  else {
    lcdA.setCursor(13, 1);
    lcdA.print(string[12]);
  }

  if (lcdAcursor == 0) {
    lcdA.setCursor(3, 0);
    lcdA.write(2);
  }
  else if (lcdAcursor == 1) {
    lcdA.setCursor(9, 0);
    lcdA.write(2);
  }
  else if (lcdAcursor == 2) {
    lcdA.setCursor(7, 1);
    lcdA.write(2);
  }
  else if (lcdAcursor == 3) {
    lcdA.setCursor(0, 1);
    lcdA.write(2);
  }
  else return;
}




bool establishConnection() {
  lcdB.clear();
  lcdB.setCursor(3, 0);
  lcdB.print(string[0]);
  lcdB.setCursor(2, 1);
  lcdB.print(string[1]);
  Data.d2 = random(1, 500);
  Data.d3 = random(1, 500);
  Data.d4 = (Data.d2 + Data.d3);
  myRadio.stopListening(); // sends binding data out
  exitPressed = false;
  while (!(myRadio.write(&Data, sizeof(Data)))) {
    if (exitPressed) {
      exitPressed = false;
      Data.d1 = 0;
      return false;
    }
  }
  return true;
}



void homeZoomandFocus() {
  myRadio.stopListening();
  lcdB.clear();
  lcdB.setCursor(5, 0);
  lcdB.print(string[14]);
  lcdB.setCursor(0, 1);
  lcdB.print(string[15]);
  lcdA.clear();
  lcdA.setCursor(3, 0);
  lcdA.print(string[2]);
  lcdA.setCursor(1, 1);
  lcdA.print(string[16]);

  exitPressed = false;
  while (true) {
    if (exitPressed) {
      exitPressed = false;
      Data.d7 = 255;
      while (!myRadio.write(&Data, sizeof(Data)));
      break;
    }
  }

  lcdA.clear();
  lcdB.clear();
  lcdA.setCursor(2, 0);
  lcdA.print(string[1]);
  lcdA.setCursor(5, 1);
  lcdA.print(string[14]);
  myRadio.startListening();
  delay(500);
  while (true) {
    while (!myRadio.available()) delayMicroseconds(1);
    while (myRadio.available()) myRadio.read(&Data, sizeof(Data));
    if (Data.d7 == 125) {
      myRadio.stopListening();
      initialiseDataPackage();
      return;
    }

  }

}


void setup() {
  Wire.begin();
  delay(10);
  //Serial.begin(9600);
  DDRD &= B11111100; // D2, D3, D4, D5, D6, D7 set as INPUT
  DDRB &= B00000001; // D8 set as INPUT
  // D9-D13 are output for Radio Communication
  DDRC &= B00001111; // A0, A1, A2, A3 set as INPUT
  pinMode(A6, INPUT);
  lastReadVoltage = analogRead(A6);

  // Fetching configuration data from EEPROM
  if (EEPROM.read(0) == 0) actualCoordinates = false;
  else actualCoordinates = true;
  if (EEPROM.read(4) == 0) RadioPower = false;
  else RadioPower = true;

  maxZTA = (((int)(EEPROM.read(5) * 255)) + ((int)EEPROM.read(6))); // maximum allowed Zoom travel default- 7000
  maxFTA = (((int)(EEPROM.read(7) * 255)) + ((int)EEPROM.read(8))); // maximum allowed Focus travel default- 4160
  prescalerZoom = EEPROM.read(2);
  prescalerFocus = EEPROM.read(3);
  zoomUp = (float)(EEPROM.read(11));
  zoomDown = (float)(EEPROM.read(12));
  //zoomUp -= zoomDown;
  focus = (((float)(EEPROM.read(13))) * 255.00) + ((float)(EEPROM.read(14)));
  picTime = ((int)(EEPROM.read(15) * 255)) + ((int)EEPROM.read(16));

  lcdA.begin();
  lcdB.begin();
  checkVoltage();
  lcdA.clear();
  lcdB.clear();
  lcdA.createChar(2, navigatingArrowGraphics);
  lcdA.createChar(3, photographGraphics);
  lcdA.createChar(4, radioHighPower_Graphics);
  lcdB.createChar(1, plusMinusGraphics);
  lcdB.createChar(2, navigatingArrowGraphics);
  lcdB.createChar(3, navigatingArrowInvertedGraphics);


  attachInterrupt(1, exitButtonPressed, RISING);
  attachPCINT(digitalPinToPCINT(photo_switch), photoButtonPressed, RISING);

  attachPCINT(digitalPinToPCINT(clk_1), rotaryMotion_1, CHANGE); // General purpose encoder
  attachInterrupt(0, rotaryButtonPressed_1, RISING); // General purpose encoder button

  if (EEPROM.read(1) == 0) {
    attachPCINT(digitalPinToPCINT(clk_2), rotaryMotion_2, CHANGE); // For Zoom
    attachPCINT(digitalPinToPCINT(clk_3), rotaryMotion_3, CHANGE); // For Focus
    attachPCINT(digitalPinToPCINT(rotary_button_2), rotaryButtonPressed_2, RISING); // Zoom encoder button
    attachPCINT(digitalPinToPCINT(rotary_button_3), rotaryButtonPressed_3, RISING); // Focus encoder button
  }
  else {
    attachPCINT(digitalPinToPCINT(clk_3), rotaryMotion_2, CHANGE); // For Focus
    attachPCINT(digitalPinToPCINT(clk_2), rotaryMotion_3, CHANGE); // For Zoom
    attachPCINT(digitalPinToPCINT(rotary_button_3), rotaryButtonPressed_2, RISING); // Fous encoder button
    attachPCINT(digitalPinToPCINT(rotary_button_2), rotaryButtonPressed_3, RISING); // Zoom encoder button
  }

  if (!actualCoordinates) {
    liveZoom_F = (((float)(liveZoom)) * ((zoomUp - zoomDown) / ((float)(maxZTA)))) + zoomDown;
    prescalerZoom_F = (((float)(prescalerZoom)) * (zoomUp / ((float)(maxZTA))));
    liveFocus_F = liveFocus * (focus / ((float)(maxFTA)));
    prescalerFocus_F = ((float)(prescalerFocus)) * (focus / ((float)(maxFTA)));
  }

  myRadio.begin();
  myRadio.openWritingPipe(addresses[1]); // 00002
  myRadio.openReadingPipe(1, addresses[0]); // 00001
  if (EEPROM.read(4) == 1) myRadio.setPALevel(RF24_PA_MAX);
  else myRadio.setPALevel(RF24_PA_MIN);

  if (establishConnection()) homeZoomandFocus();

  Time = millis();
  lcdAcursor = 0;
  update_LCDA_Graphics();
  update_LCDB_Graphics();
}

void loop() {
  prev_clk_1 = (PINC & B00000001); // reads A0
  prev_data_1 = (PINC & B00000010); // reades A1

  if (millis() - Time > 1000) {
    Time = millis();
    if (checkVoltage()) update_LCDA_Graphics();
  }

  if (photoPressed) {
    photoPressed = false;
    Data.d1 = 9;
    myRadio.write(&Data, sizeof(Data));
    delay(30);
    initialiseDataPackage();
  }

  if (TurnDetected_1) {
    if (up_1) {
      lcdAcursor++;
      if (lcdAcursor > 2) lcdAcursor = 0;
    }
    else {
      lcdAcursor--;
      if (lcdAcursor < 0) lcdAcursor = 2;
    }
    update_LCDA_Graphics();
    TurnDetected_1 = false;
  }
  if (buttonPressed_1) {
    buttonPressed_1 = false;
    if (lcdAcursor == 0) Key_Frames_Mode();
    if (lcdAcursor == 1) manual_Mode();
    if (lcdAcursor == 2) settings_Mode();
  }
}
