///////////////Definitions for IMU//////////////////////////////
///////////////////////////////////////////////////////////////
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();
////////////////////////////////////////////////////
///////////////Definitions for LCD//////////////////////////////
///////////////////////////////////////////////////////////////
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>

#define TFT_CS     10
#define TFT_RST    9  // you can also connect this to the Arduino reset
// in which case, set this #define pin to 0!
#define TFT_DC     6

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

// Option 2: use any pins but a little slower!
#define TFT_SCLK 13   // set these to be whatever pins you like!
#define TFT_MOSI 11   // set these to be whatever pins you like!
float p = 3.1415926;
/////////////////////////////////////////////////////////////////////////////////

////////////////////Definitions for MAX30100///////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"

#define REPORTING_PERIOD_MS     1000
PulseOximeter pox;

uint32_t tsLastReport = 0;

///////////////Definitions for Temp sense//////////////////////////////
///////////////////////////////////////////////////////////////

int val;
int tempPin = 1;

// Callback (registered below) fired when a pulse is detected
void onBeatDetected()
{
  Serial.println("Beat!");
}

//////////////////////////////////////////////////////////////////////


/*Definitions for BLE */
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"
#define button 5

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

void error(const __FlashStringHelper*err)
{
  Serial.println(err);
  while (1);
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{

  pinMode(button, INPUT);
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  /* Initialise the module */
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  Serial.print(F("Initialising the Bluefruit LE module: "));
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);
  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  // LCD

  Serial.print("Hello! ST7735 TFT Test");

  // Use this initializer (uncomment) if you're using a 1.44" TFT
  tft.initR(INITR_144GREENTAB);   // initialize a ST7735S chip, black tab

  Serial.println("Initialized");

  uint16_t time = millis();
  //tft.fillScreen(ST7735_BLACK);
  time = millis() - time;

  Serial.println(time, DEC);
  delay(500);

  //HEart Rate
  //Serial.begin(115200);

  Serial.println("Initializing MAX30100");
  // Initialize the PulseOximeter instance and register a beat-detected callback
  pox.begin();
  pox.setOnBeatDetectedCallback(onBeatDetected);

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set module to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));
}

void testdrawtext(char *text, uint16_t color) {
  tft.setCursor(0, 0);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  // Heart Rate Print
  // Make sure to call update as fast as possible
  pox.update();
  static char outstr[15];
  // Asynchronously dump heart rate and oxidation levels to the serial
  // For both, a value of 0 means "invalid"
  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
    tft.setTextWrap(false);
    tft.fillScreen(ST7735_BLACK);
    tft.setCursor(0, 0);
    tft.setTextSize(0.1);

    //Print Heart Rate
    tft.setTextColor(ST7735_WHITE);
    tft.print("Heart rate: ");
    ble.print("Heart rate: ");
    Serial.print("Heart rate:");
    tft.setTextColor(ST7735_RED);
    tft.print(pox.getHeartRate(), 2);
    ble.print(pox.getHeartRate());
    Serial.print(pox.getHeartRate());
    //Print Heart Rate Unit
    tft.setTextColor(ST7735_WHITE);
    tft.println(" bpm");
    ble.println("bpm");

    //Print Blood Oxidation
    tft.print("SpO2: ");
    ble.print("SpO2: ");
    Serial.print("bpm / SpO2:");
    tft.setTextColor(ST7735_RED);
    tft.print(pox.getSpO2());
    ble.print(pox.getSpO2());
    Serial.print(pox.getSpO2());
    //Print SpO2 Unit
    tft.setTextColor(ST7735_WHITE);
    tft.println(" %");
    ble.println(" %");
    //Print Temp
    tft.print("Temperature: ");
    ble.print("Temperature: ");
    Serial.print("% / temp:");
    tft.setTextColor(ST7735_RED);
    tft.print(pox.getTemperature(), 2);
    ble.print(pox.getTemperature());
    Serial.print(pox.getTemperature());
    //Print Temp Unit
    tft.setTextColor(ST7735_WHITE);
    tft.println(" C");
    ble.println(" C");
    Serial.println("C");


    //Temp Sense Voltage to Temp conversion
    //
    //Temp Sensing IC
    int sensorValue = analogRead(tempPin);
    float mvolts = (sensorValue * (5.0 / 1023.0)) * 1000;
    float Temp_cel = (10.888 - sqrt((-10.888) * (-10.888) + 4 * 0.00347 * (1777.3 - mvolts))) / (2 * (-0.00347)) + 30;
    float Temp_farh = Temp_cel * (9 / 5) + 32;
    Serial.print(" TEMPRATURE = ");
    Serial.print(Temp_cel);
    Serial.print("*C / ");
    Serial.print(Temp_farh);
    Serial.println("*F");
    tft.setTextColor(ST7735_WHITE);
    tft.print("Skin Temp: ");
    ble.print("Skin Temp: ");
    tft.setTextColor(ST7735_RED);
    tft.print(Temp_cel);
    ble.print(Temp_cel);
    tft.setTextColor(ST7735_WHITE);
    tft.println(" C / ");
    ble.println(" C / ");
    tft.setTextColor(ST7735_RED);
    tft.print(Temp_farh);
    tft.setTextColor(ST7735_WHITE);
    tft.println(" F");
    delay(1000);

    //IMU
    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    /* Display the floating point data */
    Serial.print("X: ");
    Serial.print(euler.x());
    Serial.print(" Y: ");
    Serial.print(euler.y());
    Serial.print(" Z: ");
    Serial.print(euler.z());
    Serial.print("\t\t");


    /* Display calibration status for each sensor. */
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("CALIBRATION: Sys=");
    Serial.print(system, DEC);
    Serial.print(" Gyro=");
    Serial.print(gyro, DEC);
    Serial.print(" Accel=");
    Serial.print(accel, DEC);
    Serial.print(" Mag=");
    Serial.println(mag, DEC);

    delay(BNO055_SAMPLERATE_DELAY_MS);
    tsLastReport = millis();

  }
  int bv = digitalRead(button);
  // Check for user input
  char n, inputs[BUFSIZE + 1];

  //  if(bv)
  //  {
  //    ble.print(bv);
  //  }
  if (Serial.available())
  {
    n = Serial.readBytes(inputs, BUFSIZE);
    inputs[n] = 0;
    // Send characters to Bluefruit
    Serial.print("Sending: ");
    Serial.println(inputs);

    // Send input data to host via Bluefruit
    ble.print(inputs);

  }

  // Echo received data
  while ( ble.available() )
  {
    int c = ble.read();

    Serial.print((char)c);

    // Hex output too, helps w/debugging!
    Serial.print(" [0x");
    if (c <= 0xF) Serial.print(F("0"));
    Serial.print(c, HEX);
    Serial.print("] ");
  }
}

