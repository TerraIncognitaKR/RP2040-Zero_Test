
/**
  ******************************************************************************
  * @file           : RP2040_Zero_Test.ino
  * @brief          : For testing 'RP2040-Zero' board and 'sensor board'
  * @link           :
  ******************************************************************************
  * @attention
  *
  *  2024 TeIn
  *  https://blog.naver.com/bieemiho92
  *
  *  Target Device :
  *     RP2040-Zero
  *     https://www.waveshare.com/wiki/RP2040-Zero
  *
  *     RP2040 Board Managers
  *     https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
  *
  *  IDE           :
  *     Arduino IDE 2.3.2 ~
  *
  *  Dependancy    :
  *     NeoPixelConnect : https://github.com/MrYsLab/NeoPixelConnect
  *     VCNL4040        : https://github.com/adafruit/Adafruit_VCNL4040
  *     AHTx0           : https://github.com/adafruit/Adafruit_AHTX0
  *     BMP280          : https://github.com/adafruit/Adafruit_BMP280_Library
  *     ... and some more ...
  *
  *
  * @note
  *     to use RGB LED : https://github.com/earlephilhower/arduino-pico/discussions/756
  *     Proxmimity     : https://github.com/sparkfun/SparkFun_VCNL4040_Arduino_Library
  *     AHT20 + BMP280 : https://embedded-things.blogspot.com/2021/02/test-aht20bmp280-temperature-humidity.html
  *
  ******************************************************************************
  */

/******************************************************************************
  * @brief      feature enable ctrl
  * @note       _
  */
// #define I2C_SCANNER_EN     1
// #define VCNL4040_EN        1
#define AHT20_EN          1
#define BMP280_EN         1



/******************************************************************************
  * @brief      includes
  * @note       _
  */
#include <Wire.h>

#include <NeoPixelConnect.h>      // onboard LED

#include <Adafruit_VCNL4040.h>    // VCNL4040 Proximity Sensor

#include <Adafruit_AHTX0.h>       // AHT20

#include <Adafruit_BMP280.h>      // BMP280


/******************************************************************************
  * @brief      macros
  * @note       _
  */
// onboard LED
#define WS2812_LED_DIN      16     //GP16 // SPI

// VCNL4040 Proximity
#define PROX_I2C_ADDR       0x60   // 7-bit unshifted

// AHT20
#define AHT20_I2C_ADDR      0x38

// BMP280
#define BMP280_I2C_ADDR     0x77


/******************************************************************************
  * @brief      global variables
  * @note       _
  */
uint32_t g_dwCnt = 0;
uint8_t  b_LEDVal = 64;



/******************************************************************************
  * @brief      constructor
  * @note       _
  */
NeoPixelConnect   p(WS2812_LED_DIN, 1);
Adafruit_VCNL4040 vcnl4040 = Adafruit_VCNL4040();
Adafruit_BMP280   bmp;
Adafruit_AHTX0    aht;



/******************************************************************************
  * @brief      function prototypes
  * @note       _
  */



/******************************************************************************
  * @brief      user functions
  * @note       _
  */



/******************************************************************************
  * @brief      arduino setup()
  * @param      none
  * @return     none
  * @note       put your setup code here, to run once
  */
void setup() {

  Serial.begin(115200); // UART0? UART1?
  delay(2000);
  Serial.printf("Hello RP2040! (%08d) %s %s\r\n", g_dwCnt, __TIME__, __DATE__);

  Wire.setSDA(8);
  Wire.setSCL(9);
  Wire.setClock(100000);
  Wire.begin();

#ifdef VCNL4040_EN

  if (!vcnl4040.begin()) {
    Serial.println("Couldn't find VCNL4040 chip");
    while (1);
  }
  Serial.println("Found VCNL4040 chip");



  //vcnl4040.setProximityLEDCurrent(VCNL4040_LED_CURRENT_200MA);
  Serial.print("Proximity LED current set to: ");
  switch(vcnl4040.getProximityLEDCurrent()) {
    case VCNL4040_LED_CURRENT_50MA: Serial.println("50 mA"); break;
    case VCNL4040_LED_CURRENT_75MA: Serial.println("75 mA"); break;
    case VCNL4040_LED_CURRENT_100MA: Serial.println("100 mA"); break;
    case VCNL4040_LED_CURRENT_120MA: Serial.println("120 mA"); break;
    case VCNL4040_LED_CURRENT_140MA: Serial.println("140 mA"); break;
    case VCNL4040_LED_CURRENT_160MA: Serial.println("160 mA"); break;
    case VCNL4040_LED_CURRENT_180MA: Serial.println("180 mA"); break;
    case VCNL4040_LED_CURRENT_200MA: Serial.println("200 mA"); break;
  }

  //vcnl4040.setProximityLEDDutyCycle(VCNL4040_LED_DUTY_1_40);
  Serial.print("Proximity LED duty cycle set to: ");
  switch(vcnl4040.getProximityLEDDutyCycle()) {
    case VCNL4040_LED_DUTY_1_40: Serial.println("1/40"); break;
    case VCNL4040_LED_DUTY_1_80: Serial.println("1/80"); break;
    case VCNL4040_LED_DUTY_1_160: Serial.println("1/160"); break;
    case VCNL4040_LED_DUTY_1_320: Serial.println("1/320"); break;
  }

  //vcnl4040.setAmbientIntegrationTime(VCNL4040_AMBIENT_INTEGRATION_TIME_80MS);
  Serial.print("Ambient light integration time set to: ");
  switch(vcnl4040.getAmbientIntegrationTime()) {
    case VCNL4040_AMBIENT_INTEGRATION_TIME_80MS: Serial.println("80 ms"); break;
    case VCNL4040_AMBIENT_INTEGRATION_TIME_160MS: Serial.println("160 ms"); break;
    case VCNL4040_AMBIENT_INTEGRATION_TIME_320MS: Serial.println("320 ms"); break;
    case VCNL4040_AMBIENT_INTEGRATION_TIME_640MS: Serial.println("640 ms"); break;
  }


  //vcnl4040.setProximityIntegrationTime(VCNL4040_PROXIMITY_INTEGRATION_TIME_8T);
  Serial.print("Proximity integration time set to: ");
  switch(vcnl4040.getProximityIntegrationTime()) {
    case VCNL4040_PROXIMITY_INTEGRATION_TIME_1T: Serial.println("1T"); break;
    case VCNL4040_PROXIMITY_INTEGRATION_TIME_1_5T: Serial.println("1.5T"); break;
    case VCNL4040_PROXIMITY_INTEGRATION_TIME_2T: Serial.println("2T"); break;
    case VCNL4040_PROXIMITY_INTEGRATION_TIME_2_5T: Serial.println("2.5T"); break;
    case VCNL4040_PROXIMITY_INTEGRATION_TIME_3T: Serial.println("3T"); break;
    case VCNL4040_PROXIMITY_INTEGRATION_TIME_3_5T: Serial.println("3.5T"); break;
    case VCNL4040_PROXIMITY_INTEGRATION_TIME_4T: Serial.println("4T"); break;
    case VCNL4040_PROXIMITY_INTEGRATION_TIME_8T: Serial.println("8T"); break;
  }

  //vcnl4040.setProximityHighResolution(false);
  Serial.print("Proximity measurement high resolution? ");
  Serial.println(vcnl4040.getProximityHighResolution() ? "True" : "False");

  Serial.println("");

#endif /** #ifdef VCNL4040_EN **/

#ifdef AHT20_EN
  if(!aht.begin())
  {
    Serial.println("Could not find AHT? Check wiring");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("AHT10 or AHT20 found");
#endif

#ifdef BMP280_EN
  if(!bmp.begin())
  {
    Serial.println("Could not find BMP? Check wiring");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("BMP280 found");

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

#endif

}



/******************************************************************************
  * @brief      arduino loop()
  * @param      none
  * @return     none
  * @note       put your main code here, to run repeatedly:
  */
void loop() {

#ifdef I2C_SCANNER_EN
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {


        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
  {
    Serial.println("No I2C devices found\n");
  }
  else
  {
    Serial.println("done\n");
  }
  delay(5000);
#endif /** #ifdef I2C_SCANNER_EN **/

#ifdef VCNL4040_EN
  Serial.print("Proximity:"); Serial.println(vcnl4040.getProximity());
  // Serial.print("Ambient light:"); Serial.println(vcnl4040.getLux());
  // Serial.print("Raw white light:"); Serial.println(vcnl4040.getWhiteLight());
  // Serial.println("");

  delay(100);
#endif /** #ifdef VCNL4040_EN **/

#ifdef AHT20_EN
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  Serial.print(">> AHT20 Humidity(% rH) : ");
  Serial.println(humidity.relative_humidity);

  Serial.print(">> AHT20 Temp('C') : ");
  Serial.println(temp.temperature);
#endif

#ifdef BMP280_EN
  Serial.print(">> BMP280 Temp('C) : ");
  Serial.println(bmp.readTemperature());

  Serial.print(">> BMP280 Pressure(Pa) : ");
  Serial.println(bmp.readPressure());

  Serial.print(">> BMP280 Altitude(m) : ");
  Serial.println(bmp.readAltitude(1012.43));
#endif


#if 0     // onboard RGB LED
  g_dwCnt++;
  Serial.printf("Hello RP2040! (%08d)\r\n", g_dwCnt);

  switch( g_dwCnt%4)
  {
    case 0 :
      p.neoPixelSetValue(0, b_LEDVal, 0, 0, true);
      break;
    case 1 :
      p.neoPixelSetValue(0, 0, b_LEDVal, 0, true);
      break;
    case 2 :
      p.neoPixelSetValue(0, 0, 0, b_LEDVal, true);
      break;
    case 3 :
      p.neoPixelSetValue(0, b_LEDVal, b_LEDVal, b_LEDVal, true);
      break;
    default :
      break;
  }
  delay(500);
  p.neoPixelClear();
  delay(500);
#endif    // onboard RGB LED

  delay(1000);

}
