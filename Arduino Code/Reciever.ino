/*
   Project:- LENS CONTROL SYSTEM
   Developer:- SAMRAT DUTTA
   3DprintingAndRobotics
   RECIEVER CODE
*/

/* Pinouts
   A7- NULL
   A6- Voltage Probe Pin
   A5- SCL
   A4- SDA
   A3- NULL
   A2- NULL
   A1- enable1 FOCUS
   A0- enable2 ZOOM
   D13- SCK
   D12- MISO
   D11- MOSI
   D10- CSN
   D9- CS
   D8- dir2 ZOOM
   D7- dir1 FOCUS
   D6- step2 ZOOM
   D5- step1 FOCUS
   D4- Buzzer
   D3- IR Module
   D2-
   D1- NULL
   D0- NULL
*/

// Required Libraries
#include <SPI.h>
#include "RF24.h"
#include <Wire.h>
#include <EEPROM.h>

#define voltageProbePin     A6

#define FOCUS_HIGH  PORTD |= B00100000;       // D5 HIGH FOCUS
#define FOCUS_LOW  PORTD &= B11011111;        // D5 LOW FOCUS
#define ZOOM_HIGH  PORTD |= B01000000;        // D6 HIGH ZOOM
#define ZOOM_LOW  PORTD &= B10111111;         // D6 LOW ZOOM
#define ZOOM_FOCUS_HIGH  PORTD |= B01100000;  // D5, D6 HIGH FOCUS AND ZOOM
#define ZOOM_FOCUS_LOW  PORTD &= B10011111;   // D5, D6 LOW FOCUS AND ZOOM
#define FOCUS_ACTIVATE  PORTC &= B11111101;   // A1 LOW [Activates the FOCUS motor]
#define FOCUS_DEACTIVATE  PORTC |= B00000010; // A1 HIGH [Deactivates the FOCUS motor]
#define ZOOM_ACTIVATE  PORTC &= B11111110;    // A0 LOW [ZOOM Motor activated]
#define ZOOM_DEACTIVATE   PORTC |= B00000001; // A0 HIGH [ZOOM MOTOR deactivated]
#define FOCUS_DIR_A  PORTD |= B10000000;      // D7 HIGH focus
#define FOCUS_DIR_B  PORTD &= B01111111;      // D7 LOW focus
#define ZOOM_DIR_A  PORTB |= B00000001;       // D8 HIGH zoom
#define ZOOM_DIR_B  PORTB &= B11111110;       // D8 LOW zoom
#define IR_ON  PORTD |= B00001000;            // D3 HIGH
#define IR_OFF  PORTD &= B11110111;           // D3 LOW
#define BUZZER_ON  PORTD |= B00010000;        // D4 HIGH
#define BUZZER_OFF  PORTD &= B11101111;       // D4 LOW

#define TIMER1_RESET_COUNTER  TCNT1 = 0;           // resetting counter to 0
#define TIMER1_COMPA_ENABLE  TIMSK1 |= B00000010;  // enabling COMPA_VECT
#define TIMER1_COMPA_DISABLE  TIMSK1 &= B11111101; // disables COMPA_VECT
#define TIMER1_COMPB_ENABLE  TIMSK1 |= B00000100;  // enabling COMPB_VECT
#define TIMER1_COMPB_DISABLE  TIMSK1 &= B11111011; // disables COMPB_VECT
#define TIMER1_CTC_ENABLE  TCCR1B |= (1 << WGM12); // CTC mode enabled

#define EEPROM_I2C_ADDRESS 0x50

int maxFTA = 7000, maxZTA = 4160, currentFocus = 0, currentZoom = 0, zoomHomingTime = 4000; // Max steps the focus and zoom motor are allowed to take
int add = 0, kf = 1, picTime = 0, lastReadVoltage = 0;
float voltage = 0.00, x = 0.00;
bool man = false;
volatile bool f_dir = false, z_dir = false, focusMotor = false, zoomMotor = false, motion = false, ovA = false, ovB = false;
volatile bool mode1 = false, mode2 = false;
volatile unsigned long stpZOOM = 0, stpFOCUS = 0, a = 0, b = 0, cacheA = 0, cacheB = 0;

RF24 myRadio (9, 10); // (CE, CSN)
const byte addresses[][6] = {"00001", "00002"};
/*
  void activate_recieveData() {
  myRadio.begin();
  myRadio.setChannel(124);
  if (EEPROM.read(0) == 1) myRadio.setPALevel(RF24_PA_MAX);
  else myRadio.setPALevel(RF24_PA_MIN);
  myRadio.setDataRate( RF24_250KBPS ) ;
  myRadio.openReadingPipe(1, addresses[0]);
  myRadio.startListening();
  delay(1);
  }

  void activate_transmitData() {
  myRadio.begin();
  myRadio.setChannel(124);
  if (EEPROM.read(0) == 1) myRadio.setPALevel(RF24_PA_MAX);
  else myRadio.setPALevel(RF24_PA_MIN);
  myRadio.setDataRate( RF24_250KBPS ) ;
  myRadio.openWritingPipe( addresses[0]);
  myRadio.stopListening();
  delay(1);
  }
*/
void EEPROMwrite(int address, byte val)
{
  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  Wire.write((int)(address >> 8));   // MSB
  Wire.write((int)(address & 0xFF)); // LSB
  Wire.write(val);
  Wire.endTransmission();
  delay(5);
}
byte EEPROMread(int address)
{
  byte rcvData = 0xFF;
  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  Wire.write((int)(address >> 8));   // MSB
  Wire.write((int)(address & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(EEPROM_I2C_ADDRESS, 1);
  rcvData =  Wire.read();
  return rcvData;
}

bool checkVoltage() {
  voltage = analogRead(voltageProbePin);
  if (abs(lastReadVoltage - voltage) > 5.00) {
    lastReadVoltage = voltage;
    voltage *= 0.015329245;
    voltage -= 0.23; // 0.23 is the +error voltage, thus it is removed
    if ((abs(voltage) - x) > 0.02) {
      x = voltage;
    }
  }
  else {
    voltage = lastReadVoltage;
    voltage *= 0.015329245;
    voltage -= 0.23; // 0.23 is the +error voltage, thus it is removed
    if (abs((voltage) - x) > 0.02) x = voltage;
  }
  if (x >= 6.8) return true;
  else return false;
}

struct package0
{
  byte d1 = 0; // mode
  int d2 = 0; // focus
  int d3 = 0; // Zoom
  int d4 = 0; // Speed
  bool d5 = 0; // Sync or not sync
  bool d6 = 0; // Photograph
  int d7 = 0;
};
typedef struct package0 Package0;
Package0 Data;

void initialiseDataPackage() {
  Data.d1 = 0; // mode
  Data.d2 = 0;  // focus
  Data.d3 = 0;  // Zoom
  Data.d4 = 0;  // Speed
  Data.d5 = false;  // sync or not sync
  Data.d6 = false;  // photograph
  Data.d7 = 0; // parity data
}

void factoryReset() {
  EEPROM.write(0, 0);
  EEPROM.write(1, 16);
  EEPROM.write(2, 80);
  EEPROM.write(3, 27);
  EEPROM.write(4, 115);
  EEPROM.write(5, 3);
  EEPROM.write(6, 235);
}

void establishConnection() {
  myRadio.startListening(); // recieves binding data out
  while (true) {
    while (!myRadio.available()) delayMicroseconds(1);
    while (myRadio.available()) myRadio.read(&Data, sizeof(Data));
    if ((Data.d2 + Data.d3) == Data.d4) {
      BUZZER_ON
      delay(100);
      BUZZER_OFF
      delay(100);
      BUZZER_ON
      delay(100);
      BUZZER_OFF
      break;
    }

  }
}

/*
  void show_Incoming_Data() {
  Serial.println();
  Serial.println();
  Serial.println(Data.d1);
  Serial.println(Data.d2);
  Serial.println(Data.d3);
  Serial.println(Data.d4);
  Serial.println(Data.d5);
  Serial.println(Data.d6);
  Serial.println(Data.d7);
  Serial.println();
  Serial.println();
  }*/

void KeyFrameMode() {
  myRadio.startListening();
  FOCUS_ACTIVATE
  ZOOM_ACTIVATE
  while (motion);
  moveMotors(Data.d2, Data.d3, Data.d4, Data.d5);
  while (motion);
  add = 0;
  while (true) { // setting key frames

    while (!myRadio.available()) delayMicroseconds(1);

    while (myRadio.available()) myRadio.read( &Data, sizeof(Data) );

    if ((Data.d1 == 126 || Data.d1 == 255) && add <= 7992) {

      if (Data.d1 == 126 && add > 7) add -= (Data.d7 * 8);
      if (add < 0) add = 0;
      EEPROMwrite(add, ((byte)(Data.d2 / 255))); // 0
      add++;
      EEPROMwrite(add, ((byte)(Data.d2 % 255))); // 1
      add++;
      EEPROMwrite(add, ((byte)(Data.d3 / 255))); // 2
      add++;
      EEPROMwrite(add, ((byte)(Data.d3 % 255))); // 3
      add++;
      EEPROMwrite(add, ((byte)(Data.d4 / 255))); // 4
      add++;
      EEPROMwrite(add, ((byte)(Data.d4 % 255))); // 5
      add++;
      if (Data.d5) EEPROMwrite(add, 1);  // 6
      else EEPROMwrite(add, 0);
      add++;
      if (Data.d6) EEPROMwrite(add, 1);  // 7
      else EEPROMwrite(add, 0);
      add++;
      initialiseDataPackage();
      Data.d1 = 10;
    }

    if (Data.d1 == 0) {
      FOCUS_DEACTIVATE // FOCUS Motor activated
      ZOOM_DEACTIVATE // ZOOM MOTOR deactivated
      return;
    }

    if (Data.d1 == 200) {
      kf = Data.d2;
      picTime = Data.d7;
      moveMotors(0, 0, 300, true);
      while (motion);
      break;
    }
    if (Data.d1 == 2) {
      if (!motion) moveMotors(Data.d2, Data.d3, 400, Data.d5);
      initialiseDataPackage();
      Data.d1 = 10;
    }
  }

  moveMotors(0, 0, 250, true);   // homing
  while (motion);

  // start yes no
  while (true) {
    while (!myRadio.available()) delayMicroseconds(1);
    while (myRadio.available()) myRadio.read( &Data, sizeof(Data) );
    if (Data.d1 == 0) { // exit
      initialiseDataPackage();
      FOCUS_DEACTIVATE
      ZOOM_DEACTIVATE
      return;
    }
    if (Data.d1 == 126) { // auto mode
      man = false;
      break;
    }
    if (Data.d1 == 130) {
      man = true;
      break;
    }
  }

  add = 0;
  if (!man) {
    for (int i = 1; i <= kf; i++) {
      createCoordinates();
      while (motion) delayMicroseconds(1);
      moveMotors(Data.d2, Data.d3, Data.d4, Data.d5);
      while (motion) delayMicroseconds(1);
      if (Data.d6) {
        IR_ON
        delay(30);
        IR_OFF
        delay(picTime);
      }
    }
  }
  else {
    add = 0;
    int i = 0;
    while (i <= kf) {

      if (myRadio.available()) {
        while (myRadio.available()) myRadio.read(&Data, sizeof(Data));

        if (Data.d1 == 100) { //  next frame
          Data.d1 = 10;
          createCoordinates();
          while (motion) delayMicroseconds(1);
          moveMotors(Data.d2, Data.d3, Data.d4, Data.d5);
          while (motion) delayMicroseconds(1);
          if (Data.d6) {
            IR_ON
            delay(30);
            IR_OFF
            delay(picTime);
          }
          i++;
        }

        if (Data.d1 == 180) { // prev frame
          Data.d1 = 10;

          i--;
          if (i < 1) i = 1;
          add -= 16;
          if (add < 0) add = 0;

          createCoordinates();
          while (motion) delayMicroseconds(1);
          moveMotors(Data.d2, Data.d3, Data.d4, Data.d5);
          while (motion) delayMicroseconds(1);
          if (Data.d6) {
            IR_ON
            delay(30);
            IR_OFF
            delay(picTime);
          }
        }

        if (Data.d1 == 0) break;

      }
    }
  }


  Data.d1 = 126;
  myRadio.stopListening();
  BUZZER_ON
  delay(100);
  BUZZER_OFF
  delay(2000);
  while (motion) delayMicroseconds(1);
  moveMotors(0, 0, 250, true);   // homing
  while (motion) delayMicroseconds(1);
  FOCUS_DEACTIVATE
  ZOOM_DEACTIVATE
  myRadio.write(&Data, sizeof(Data));
  initialiseDataPackage();
  myRadio.startListening();
}

void createCoordinates() {
  Data.d2 = (((int)(EEPROMread(add))) * 255);
  add++;
  Data.d2 += ((int)(EEPROMread(add)));
  add++;
  Data.d3 = (((int)(EEPROMread(add))) * 255);
  add++;
  Data.d3 += ((int)(EEPROMread(add)));
  add++;
  Data.d4 = (((int)(EEPROMread(add))) * 255);
  add++;
  Data.d4 += ((int)(EEPROMread(add)));
  add++;
  if (EEPROMread(add) == 1) Data.d5 = true;
  else Data.d5 = false;
  add++;
  if (EEPROMread(add) == 1) Data.d6 = true;
  else Data.d6 = false;
  add++;
}

void manualMode() {
  FOCUS_ACTIVATE
  ZOOM_ACTIVATE
  while (motion);
  moveMotors(Data.d2, Data.d3, Data.d4, Data.d5);
  while (motion);

  while (true) {
    while (!myRadio.available()) delayMicroseconds(1);
    while (myRadio.available())
      myRadio.read( &Data, sizeof(Data) );
    if (Data.d1 == 0) {
      initialiseDataPackage();
      FOCUS_DEACTIVATE
      ZOOM_DEACTIVATE
      return;
    }
    if (Data.d7 == 145 && Data.d1 == 1) {
      if (!motion) moveMotors(Data.d2, Data.d3, Data.d4, Data.d5);
      if (Data.d6) {
        IR_ON
        delay(30);
        IR_OFF
      }
    }

  }
  FOCUS_DEACTIVATE
  ZOOM_DEACTIVATE
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(10);
  // defining input and output pins
  DDRD |= B11111000; // Sets D7,D6,D5,D4,D3 as OUTPUT
  DDRB |= B00000001; // Sets D8 as OUTPUT
  DDRC |= B00000011; // Sets A0,A1 as OUTPUT
  FOCUS_DEACTIVATE // FOCUS Motor activated
  ZOOM_DEACTIVATE // ZOOM MOTOR deactivated

  pinMode(A6, INPUT);
  lastReadVoltage = analogRead(A6);
  checkVoltage();
  //delay(1000);

  maxZTA = (((int)(EEPROM.read(1) * 255)) + ((int)EEPROM.read(2))); // maximum allowed Zoom travel default- 7000
  maxFTA = (((int)(EEPROM.read(3) * 255)) + ((int)EEPROM.read(4))); // maximum allowed Focus travel default- 4160
  picTime = ((int)(EEPROM.read(5) * 255)) + ((int)EEPROM.read(6));

  myRadio.begin();
  myRadio.openWritingPipe(addresses[0]); // 00001
  myRadio.openReadingPipe(1, addresses[1]); // 00002
  if (EEPROM.read(0) == 1) myRadio.setPALevel(RF24_PA_MAX);
  else myRadio.setPALevel(RF24_PA_MIN);

  establishConnection();
  myRadio.startListening();
  initialiseDataPackage();

  while (true) {
    while (!myRadio.available()) delayMicroseconds(1);
    while (myRadio.available())
      myRadio.read( &Data, sizeof(Data) );
    if (Data.d7 == 255) {
      BUZZER_ON
      delay(100);
      BUZZER_OFF
      break;
    }

  }
  Data.d7 = 125;
  myRadio.stopListening();
  delay(500);
  home_focus();
  FOCUS_DEACTIVATE // FOCUS Motor activated
  ZOOM_DEACTIVATE // ZOOM MOTOR deactivated
  delay(100);
  myRadio.write(&Data, sizeof(Data));
  initialiseDataPackage();
  myRadio.startListening();
  delay(50);
  initialiseDataPackage();
}


void loop() {

  while (!myRadio.available()) delayMicroseconds(1);
  while (myRadio.available()) myRadio.read( &Data, sizeof(Data) );
  //show_Incoming_Data();

  if (Data.d1 == 1) {
    manualMode();
    initialiseDataPackage();
  }
  if (Data.d1 == 2) {
    KeyFrameMode();
    initialiseDataPackage();
  }



  if (Data.d1 == 3) {
    softwareHome(); // homing
    initialiseDataPackage();
  }
  if (Data.d1 == 4) {               // lock unlock motor
    if (Data.d5) {
      PORTC &= B11111110;
      PORTC &= B11111101;
    }
    else {
      PORTC |= B00000010;
      PORTC |= B00000001;
    }
    initialiseDataPackage();
    BUZZER_ON
    delay(100);
    BUZZER_OFF
  }
  if (Data.d1 == 5) { // radio power
    if (Data.d5) EEPROM.write(0, 1);
    else EEPROM.write(0, 0);
    initialiseDataPackage();
    reboot();
  }
  if (Data.d1 == 6) { // battery
    sendVoltage();
    initialiseDataPackage();
  }
  if (Data.d1 == 7) { // factory reset
    factoryReset();
    initialiseDataPackage();
    reboot();
  }
  if (Data.d1 == 9) {
    IR_ON;
    delay(30);
    IR_OFF;
    initialiseDataPackage();
  }
}

void reboot() {
  FOCUS_DEACTIVATE
  ZOOM_DEACTIVATE
  for (int i = 0; i < 10; i++) {
    BUZZER_ON
    delay(150);
    BUZZER_OFF
    delay(1850);
  }
  while (true);
}

void resetTime1_register() {
  TCCR1A = 0; // resetting registers
  TCCR1B = 0; // resetting registers
}

void home_focus() {
  FOCUS_DIR_A // D7 LOW direction of focus
  FOCUS_ACTIVATE // A1 LOW [Activates the FOCUS motor]
  for (int i = 0; i < (maxFTA + 1000); i++) { // HOMING OF FOCUS DIAL
    FOCUS_HIGH
    FOCUS_LOW
    delayMicroseconds(200);
  }
}
void softwareHome() {
  FOCUS_ACTIVATE;
  ZOOM_ACTIVATE;
  moveMotors(0, 0, 400, true);
  while (motion) delayMicroseconds(1);
  FOCUS_DEACTIVATE;
  ZOOM_DEACTIVATE;
}
void sendVoltage() {
  myRadio.stopListening();
  delay(100);
  checkVoltage();
  Data.d1 = (int)(x);
  Data.d2 = (int)((x - Data.d1) * 100);
  myRadio.write(&Data, sizeof(Data));
  myRadio.startListening();
  delay(100);
}
//motor1-focus
//motor2-zoom
void moveMotors(int focus, int zoom, int sp, bool sync) {
  //if ((currentFocus - focus) == 0 && (currentZoom - zoom) == 0) return;
  if (sp <= 0) sp = 100; // default delay time is 100

  // if positions violated function does not executes
  if ( (currentFocus - (focus)) < 0 && (currentFocus + (abs(currentFocus - (focus)))) > maxFTA) return;
  if ( (currentZoom - (zoom)) < 0 && (currentZoom + (abs(currentZoom - (zoom)))) > maxZTA) return;

  // Setting up the direction of both motors
  if (currentFocus - focus > 0) {
    FOCUS_DIR_A // D7 HIGH focus
    f_dir = false;
  }
  else {
    FOCUS_DIR_B // D7 LOW focus
    f_dir = true;
  }
  if (currentZoom - zoom > 0) {
    ZOOM_DIR_A // D8 HIGH zoom
    z_dir = false;
  }
  else {
    ZOOM_DIR_B // D8 LOW zoom
    z_dir = true;
  }

  if (sync == false) { // no synced motions
    stpFOCUS = (abs(currentFocus - (focus)));
    stpZOOM = (abs(currentZoom - (zoom)));

    // updating current positions
    if (f_dir) currentFocus += stpFOCUS;
    else currentFocus -= stpFOCUS;
    if (z_dir) currentZoom += stpZOOM;
    else currentZoom -= stpZOOM;

    // activating motors
    ZOOM_ACTIVATE // A0 LOW [Activates the motor]
    FOCUS_ACTIVATE // A1 LOW [Activates the motor]

    cli();
    resetTime1_register();
    TIMER1_CTC_ENABLE   // CTC mode

    cacheA = ( sp * 16 );

    // TIMER1 used for motor executing less steps here zoom
    if (cacheA > 0 && cacheA <= 65536) { // prescaler 1
      TCCR1B |= B00000001; // prescaler set to 1
      OCR1A = cacheA;
    }
    else if (cacheA > 65536 && cacheA <= 524288) { // prescaler 8
      TCCR1B |= B00000010; // prescaler set to 8
      OCR1A = (cacheA / 8);
    }
    else if (cacheA > 524288 && cacheA <= 4194304) { // prescaler 64
      TCCR1B |= B00000011; // prescaler set to 64
      OCR1A = (cacheA / 64);
    }
    else if (cacheA > 4194304 && cacheA <= 16777216) { // prescaler 256
      TCCR1B |= B00000100; // prescaler set to 256
      OCR1A = (cacheA / 256);
    }
    else if (cacheA > 16777216 && cacheA <= 67108864) { // prescaler 1024
      TCCR1B |= B00000101; // prescaler set to 1024
      OCR1A = (cacheA / 1024);
    }
    else return;
    sei();
    motion = true;
    mode1 = true;
    mode2 = false;
    cli();
    TIMER1_RESET_COUNTER // resetting counter to 0
    TIMER1_COMPA_ENABLE // enabling COMPA_vect
    sei();
  }
  else {
    if ((abs(currentFocus - (focus))) > 0 && (abs(currentZoom - (zoom)) > 0)) { // Both motor moves

      if ((abs(currentFocus - (focus))) > (abs(currentZoom - (zoom)))) { // more focus travel than zoom

        stpFOCUS = (abs(currentFocus - (focus)));
        stpZOOM = (abs(currentZoom - (zoom)));
        // updating current positions
        if (f_dir) currentFocus += stpFOCUS;
        else currentFocus -= stpFOCUS;
        if (z_dir) currentZoom += stpZOOM;
        else currentZoom -= stpZOOM;

        cacheA = ((stpFOCUS * 16 * sp) / stpZOOM); // total time in 16MHz crystal pulses format
        cacheB = ((stpFOCUS * 16 * sp) % stpZOOM);
        b = (sp * 16);

        if (cacheA > 65536 || b > 65536 || cacheB > 0) {
          //Serial.println("OLD");
          cli();
          resetTime1_register();
          TIMER1_CTC_ENABLE   // CTC mode

          // TIMER1 used for motor executing less steps here zoom
          if (cacheA > 0 && cacheA <= 65536) { // prescaler 1
            TCCR1B |= B00000001; // prescaler set to 1
            OCR1A = cacheA;
          }
          else if (cacheA > 65536 && cacheA <= 524288) { // prescaler 8
            TCCR1B |= B00000010; // prescaler set to 8
            OCR1A = (cacheA / 8);
          }
          else if (cacheA > 524288 && cacheA <= 4194304) { // prescaler 64
            TCCR1B |= B00000011; // prescaler set to 64
            OCR1A = (cacheA / 64);
          }
          else if (cacheA > 4194304 && cacheA <= 16777216) { // prescaler 256
            TCCR1B |= B00000100; // prescaler set to 256
            OCR1A = (cacheA / 256);
          }
          else if (cacheA > 16777216 && cacheA <= 67108864) { // prescaler 1024
            TCCR1B |= B00000101; // prescaler set to 1024
            OCR1A = (cacheA / 1024);
          }
          else return;
          sei();
          motion = true;
          ZOOM_FOCUS_HIGH // D5,D6 HIGH focus and zoom
          ZOOM_FOCUS_LOW // D5,D6 LOW focus and zoom
          stpFOCUS--;
          stpZOOM--;
          focusMotor = false; // focus moves in loop
          zoomMotor = true; // zoom moves in interrupt
          mode2 = true;
          cli();
          TIMER1_RESET_COUNTER
          TIMER1_COMPA_ENABLE // enabling COMPA_vect
          sei();

          while (true) {
            delayMicroseconds(sp);
            FOCUS_HIGH // D5 HIGH focus
            FOCUS_LOW // D5 LOW focus
            stpFOCUS--;

            // if (stpFOCUS == 0 && stpZOOM != 0) Serial.println("ZOOM");
            //if (stpFOCUS != 0 && stpZOOM == 0) Serial.println("FOCUS");

            if (stpFOCUS == 0 && stpZOOM == 0) {
              cli(); // disables all interrupts
              TIMER1_COMPA_DISABLE // disables timer interrupt
              focusMotor = false;
              zoomMotor = false;
              motion = false;
              mode2 = false;
              sei(); // enables all interrupts
              break;
            }
          }
          return;
        }
        else {
          b = ((stpFOCUS * 16 * sp) / stpZOOM); // total time in 16MHz crystal pulses format
          a = (sp * 16);

          cacheA = a;
          cacheB = b;

          motion = true;
          // first step
          ZOOM_FOCUS_HIGH // D5,D6 HIGH focus and zoom
          ZOOM_FOCUS_LOW // D5,D6 LOW focus and zoom
          stpFOCUS--;
          stpZOOM--;

          cli();

          resetTime1_register();
          TCCR1B |= B00000001; // prescaler set to 1

          if (cacheA < cacheB) {
            OCR1B = cacheA;
            focusMotor = true;
            zoomMotor = false;
          }
          else if (cacheB < cacheA) {
            OCR1B = cacheB;
            focusMotor = false;
            zoomMotor = true;
          }
          else  {
            OCR1B = cacheA;
            focusMotor = true;
            zoomMotor = true;
          }

          TIMER1_RESET_COUNTER
          TIMER1_COMPB_ENABLE // enabling COMPB_vect
          sei();
        }
      }
      else if ((abs(currentZoom - (zoom))) > (abs(currentFocus - (focus)))) { // more zoom travel than focus

        stpFOCUS = (abs(currentFocus - (focus)));
        stpZOOM = (abs(currentZoom - (zoom)));

        // updating current positions
        if (f_dir) currentFocus += stpFOCUS;
        else currentFocus -= stpFOCUS;
        if (z_dir) currentZoom += stpZOOM;
        else currentZoom -= stpZOOM;

        cacheA = (stpZOOM * 16 * sp) / stpFOCUS; // total time in 16MHz crystal pulses format
        b = sp * 16;
        cacheB = ((stpFOCUS * 16 * sp) % stpZOOM);

        if (cacheA > 65536 || b > 65536 || cacheB > 0) {
          cli();

          resetTime1_register();
          TIMER1_CTC_ENABLE   // CTC mode

          // TIMER1 used for motor executing less steps here zoom
          if (cacheA > 0 && cacheA <= 65536) { // prescaler 1
            TCCR1B |= B00000001; // prescaler set to 1
            OCR1A = cacheA;
          }
          else if (cacheA > 65536 && cacheA <= 524288) { // prescaler 8
            TCCR1B |= B00000010; // prescaler set to 8
            OCR1A = (cacheA / 8);
          }
          else if (cacheA > 524288 && cacheA <= 4194304) { // prescaler 64
            TCCR1B |= B00000011; // prescaler set to 64
            OCR1A = (cacheA / 64);
          }
          else if (cacheA > 4194304 && cacheA <= 16777216) { // prescaler 256
            TCCR1B |= B00000100; // prescaler set to 256
            OCR1A = (cacheA / 256);
          }
          else if (cacheA > 16777216 && cacheA <= 67108864) { // prescaler 1024
            TCCR1B |= B00000101; // prescaler set to 1024
            OCR1A = (cacheA / 1024);
          }
          else return;
          sei();
          motion = true;
          ZOOM_FOCUS_HIGH // D5,D6 HIGH focus and zoom
          ZOOM_FOCUS_LOW // D5,D6 LOW focus and zoom
          stpFOCUS--;
          stpZOOM--;
          focusMotor = true; // focus moves in interrupt
          zoomMotor = false; // zoom moves in loop
          mode2 = true;
          cli();
          TIMER1_RESET_COUNTER
          TIMER1_COMPA_ENABLE // enabling COMPA_vect
          sei();

          while (true) {
            delayMicroseconds(sp);
            ZOOM_HIGH // D6 HIGH zoom
            ZOOM_LOW // D6 LOW zoom
            stpZOOM--;

            // if (stpFOCUS == 0 && stpZOOM != 0) Serial.println("ZOOM");
            //if (stpFOCUS != 0 && stpZOOM == 0) Serial.println("FOCUS");

            if (stpFOCUS == 0 && stpZOOM == 0) {
              cli(); // disables all interrupts
              TIMER1_COMPA_DISABLE // disables timer interrupt
              focusMotor = false;
              zoomMotor = false;
              motion = false;
              mode2 = false;
              sei(); // enables all interrupts
              break;
            }
          }
          return;
        }

        else {
          a = ((stpZOOM * 16 * sp) / stpFOCUS); // total time in 16MHz crystal pulses format
          b = (sp * 16);

          cacheA = a;
          cacheB = b;

          motion = true;
          // first step
          ZOOM_FOCUS_HIGH // D5,D6 HIGH focus and zoom
          ZOOM_FOCUS_LOW // D5,D6 LOW focus and zoom
          stpFOCUS--;
          stpZOOM--;

          cli(); // stops all interrupts
          resetTime1_register();
          TCCR1B |= B00000001; // prescaler set to 1

          if (cacheA < cacheB) {
            OCR1B = cacheA;
            focusMotor = true;
            zoomMotor = false;
          }
          else if (cacheB < cacheA) {
            OCR1B = cacheB;
            focusMotor = false;
            zoomMotor = true;
          }
          else  {
            OCR1B = cacheA;
            focusMotor = true;
            zoomMotor = true;
          }

          TIMER1_RESET_COUNTER
          TIMER1_COMPB_ENABLE // enabling COMPB_vect
          sei();
        }
      }
      else moveMotors(focus, zoom, sp, false); // no synced motion
    }
    else moveMotors(focus, zoom, sp, false); // no synced motion
  }
}

// Compare register A
ISR(TIMER1_COMPA_vect) {
  if (mode1) {
    if (stpFOCUS > 0) {
      FOCUS_HIGH // D5 HIGH focus
      FOCUS_LOW // D5 LOW focus
      stpFOCUS--;
    }
    if (stpZOOM > 0) {
      ZOOM_HIGH // D6 HIGH zoom
      ZOOM_LOW // D6 LOW zoom
      stpZOOM--;
    }

    if (stpFOCUS == 0 && stpZOOM == 0) {
      cli();
      TIMER1_COMPA_DISABLE // disables timer interrupt
      mode1 = false;
      motion = false;
      sei();
    }
  }
  if (mode2) {
    if (zoomMotor && stpZOOM > 0) {
      cli();
      ZOOM_HIGH // D6 HIGH zoom
      ZOOM_LOW // D6 LOW zoom
      stpZOOM--;
      sei();
    }
    if (focusMotor && stpFOCUS > 0) {
      cli();
      FOCUS_HIGH // D5 HIGH focus
      FOCUS_LOW // D5 LOW focus
      stpFOCUS--;
      sei();
    }
  }
}

// Compare register B
ISR(TIMER1_COMPB_vect) {
  if (focusMotor && stpFOCUS > 0) {
    cli();
    FOCUS_HIGH // D5 HIGH focus
    FOCUS_LOW // D5 LOW focus
    stpFOCUS--;
    focusMotor = false;
    a = a + cacheA;
    sei();
  }
  if (zoomMotor && stpZOOM > 0) {
    cli();
    ZOOM_HIGH // D6 HIGH zoom
    ZOOM_LOW // D6 LOW zoom
    stpZOOM--;
    zoomMotor = false;
    b = b + cacheB;
    sei();
  }
  // if (stpFOCUS == 0 && stpZOOM != 0) Serial.println("ZOOM");
  //if (stpFOCUS != 0 && stpZOOM == 0) Serial.println("FOCUS");
  if (stpFOCUS <= 0 && stpZOOM <= 0) {
    cli(); // disables all interrupts
    TIMER1_COMPB_DISABLE // disables timer interrupt
    focusMotor = false;
    zoomMotor = false;
    ovA = false;
    ovB = false;
    motion = false;
    sei(); // enables all interrupts
  }

  if (a > 65536 && b > 65536) {
    a -= 65536;
    b -= 65536;
    ovA = false;
    ovB = false;
    if (a < b) {
      OCR1B = a;
      focusMotor = true;
      zoomMotor = false;
    }
    else if (b < a) {
      OCR1B = b;
      focusMotor = false;
      zoomMotor = true;
    }
    else {
      OCR1B = a;
      focusMotor = true;
      zoomMotor = true;
    }
  }
  else if (a > 65536 && b <= 65536) {
    a -= 65536;
    ovA = true;
    //Serial.println("ovf");
    if (ovA && !ovB) {
      OCR1B = b;
      zoomMotor = true;
      focusMotor = false;
    }
    else {
      ovA = false;
      ovB = false;
      if (a < b) {
        OCR1B = a;
        focusMotor = true;
        zoomMotor = false;
      }
      else if (b < a) {
        OCR1B = b;
        focusMotor = false;
        zoomMotor = true;
      }
      else {
        OCR1B = a;
        focusMotor = true;
        zoomMotor = true;
      }
    }
  }
  else if (b > 65536 && a <= 65536) {
    b -= 65536;
    ovB = true;
    //Serial.println("ovb");
    if (ovB && !ovA) {
      OCR1B = a;
      focusMotor = true;
      zoomMotor = false;
    }
    else {
      ovA = false;
      ovB = false;
      if (a < b) {
        OCR1B = a;
        focusMotor = true;
        zoomMotor = false;
      }
      else if (b < a) {
        OCR1B = b;
        focusMotor = false;
        zoomMotor = true;
      }
      else {
        OCR1B = a;
        focusMotor = true;
        zoomMotor = true;
      }
    }
  }
  else {
    if (!ovA && !ovB) {
      if (a < b) {
        OCR1B = a;
        focusMotor = true;
        zoomMotor = false;
      }
      else if (b < a) {
        OCR1B = b;
        focusMotor = false;
        zoomMotor = true;
      }
      else {
        OCR1B = a;
        focusMotor = true;
        zoomMotor = true;
      }
    }
    else if (!ovA && ovB) {
      OCR1B = a;
      focusMotor = true;
      zoomMotor = false;
    }
    else if (ovA && !ovB) {
      OCR1B = b;
      focusMotor = false;
      zoomMotor = true;
    }
    else {
      ovA = false;
      ovB = false;
      if (a < b) {
        OCR1B = a;
        focusMotor = true;
        zoomMotor = false;
      }
      else if (b < a) {
        OCR1B = b;
        focusMotor = false;
        zoomMotor = true;
      }
      else {
        OCR1B = a;
        focusMotor = true;
        zoomMotor = true;
      }
    }
  }
}
