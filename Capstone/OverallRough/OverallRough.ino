#include "TempConfig.h";
#include "IMUConfig.h";
#include "Max30100Config.h";
#include "BLEConfig.h";
#include "TFTConfig.h";
#include "stdio.h";

// Callback (registered below) fired when a pulse is detected
void onBeatDetected()
{
  Serial.println("Beat!");
}

void setup() {
  Serial.begin(115200);
  //IMU
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

  // Use this initializer (uncomment) if you're using a 1.44" TFT
  tft.initR(INITR_144GREENTAB);   // initialize a ST7735S chip, black tab

  Serial.println("Initialized");

  uint16_t time = millis();
  time = millis() - time;

  Serial.println(time, DEC);
  delay(500);

  Serial.println("Initializing MAX30100");
  // Initialize the PulseOximeter instance and register a beat-detected callback
  pox.begin();
  pox.setOnBeatDetectedCallback(onBeatDetected);
  ble_init();

}
//Initiate BLE
void ble_init() {
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
    tft.fillScreen(ST7735_BLACK);
    tft.setCursor(0, 0);
    tft.setTextColor(ST7735_WHITE);
    tft.println("Connect to BLE app");
    delay(2000);
  }

  tft.println("Thanks babe");
  delay(3000);

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
void ble_sending_phone() {
  if (Serial.available())
  {
    char n, inputs[BUFSIZE + 1];
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
    char c = ble.read();

    Serial.print(c);

    // Hex output too, helps w/debugging!
    //Serial.print(" [0x");
    //if (c <= 0xF) Serial.print(F("0"));
    // Serial.print(c);//, HEX);
    //Serial.print("] ");
    tft.print(c);
  }
}

void TempSense_Calc_Print()
{
  //Calculation
  int sensorValue = analogRead(tempPin);
  float mvolts = (sensorValue * (3.3 / 1023.0)) * 1000;
  Serial.print(mvolts);
  float Temp_cel = (10.888 - sqrt((-10.888) * (-10.888) + 4 * 0.00347 * (1777.3 - mvolts))) / (2 * (-0.00347)) + 30;
  float Temp_farh = Temp_cel * (9 / 5) + 32;
  //Serial Print
  Serial.print(" TEMPRATURE = ");
  Serial.print(Temp_cel);
  Serial.print("*C / ");
  Serial.print(Temp_farh);
  Serial.println("*F");
  //TFT Print
  tft.setTextColor(ST7735_WHITE);
  tft.print("Body Temp: ");
  tft.setTextColor(ST7735_RED);
  tft.print(Temp_cel);
  tft.setTextColor(ST7735_WHITE);
  tft.println(" C / ");
  tft.setTextColor(ST7735_RED);
  tft.print(Temp_farh);
  tft.setTextColor(ST7735_WHITE);
  tft.println(" F");
  //print BLE
  ble.print(" TEMPRATURE = ");
  ble.print(Temp_cel);
  ble.print("*C / ");
  ble.print(Temp_farh);
  ble.println("*F");
}

void HR_SPO2_Calc_Print()
{
 pox.update();

  // Asynchronously dump heart rate and oxidation levels to the serial
  // For both, a value of 0 means "invalid"
  //    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
  /////////////////////////////////Heart Rate/////////////////////////////////
  //TFT
  tft.setTextColor(ST7735_WHITE);
  tft.print("Heart rate: ");
  tft.setTextColor(ST7735_RED);
  tft.print(pox.getHeartRate(), 2);
  tft.setTextColor(ST7735_WHITE);
  tft.println(" bpm");
  //Serial
  Serial.print("Heart rate:");
  Serial.print(pox.getHeartRate());
  Serial.print("bpm");
  //BLE
  ble.print("Heart rate:");
  ble.print(pox.getHeartRate());
  ble.print("bpm");
  /////////////////////////////////Blood Oxidation/////////////////////////////////
  //TFT
  tft.print(" / SpO2: ");
  tft.setTextColor(ST7735_RED);
  tft.print(pox.getSpO2());
  tft.setTextColor(ST7735_WHITE);
  tft.println(" %");
  //Serial
  Serial.print(" SpO2:");
  Serial.print(pox.getSpO2());
  Serial.print("%");
  //BLE
  ble.print(" SpO2:");
  ble.print(pox.getSpO2());
  ble.print("%");
  /////////////////////////////////Temperature/////////////////////////////////
  //TFT
  tft.print("Temperature: ");
  tft.setTextColor(ST7735_RED);
  tft.print(pox.getTemperature(), 2);
  tft.setTextColor(ST7735_WHITE);
  tft.println(" C");
  //Serial
  Serial.print(" / temp:");
  Serial.print(pox.getTemperature());
  Serial.println("C");
  //BLE
  ble.print(" / temp:");
  ble.print(pox.getTemperature());
  ble.println("C");
  //        tsLastReport = millis();
  //    }
}

void IMU_Calc_Print()
{
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
  //BLE
  ble.print("X: ");
  ble.print(euler.x());
  ble.print(" Y: ");
  ble.print(euler.y());
  ble.print(" Z: ");
  ble.print(euler.z());
  ble.print("\t\t");

  /*
    // Quaternion data
    imu::Quaternion quat = bno.getQuat();
    Serial.print("qW: ");
    Serial.print(quat.w(), 4);
    Serial.print(" qX: ");
    Serial.print(quat.y(), 4);
    Serial.print(" qY: ");
    Serial.print(quat.x(), 4);
    Serial.print(" qZ: ");
    Serial.print(quat.z(), 4);
    Serial.print("\t\t");
  */

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
  //BLE
  ble.print("CALIBRATION: Sys=");
  ble.print(system, DEC);
  ble.print(" Gyro=");
  ble.print(gyro, DEC);
  ble.print(" Accel=");
  ble.print(accel, DEC);
  ble.print(" Mag=");
  ble.println(mag, DEC);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
void loop() {


  static char outstr[15];
  // Asynchronously dump heart rate and oxidation levels to the serial
  // For both, a value of 0 means "invalid"
  pox.update();
  //TFT Setup
  tft.setTextWrap(false);
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(0, 0);
  tft.setTextSize(0.1);
  
  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
    //Heart Rate and Blood Oxidation
    HR_SPO2_Calc_Print();
    //Temperature Sensing IC
    TempSense_Calc_Print();
    //Inertial Measurement Unit
    IMU_Calc_Print();
    //BLE Send
    ble_sending_phone();
    tsLastReport = millis();
  }


}
