

#include <Arduino.h>

//note:  when using esp32 with the wire lib, I found I needed this to get Wire to init ok...
#define CONFIG_DISABLE_HAL_LOCKS 1

#include <Wire.h>

#include <SimpleFOC.h>

#include "SparkFun_TMAG5273_Arduino_Library.h"
#include "sparkfun_iot_brushless_driver.hpp"
#include <FastLED.h>

#define NUM_LEDS 1
#define DATA_PIN WS2812_RGB


#define DEG2RAD( dd ) ((float)dd*(float)0.01745329251)

CRGB leds[NUM_LEDS];

// note:  TMAG5273 is a multi axis sensor; in the physical configuration
//  on the sparkfun iot board, the magnet on the back of the motor is 
//  diametrically polorized with the sensor XY plane parallel to the 
//  diametric polarization.  Therefore the X or Y sensing axes should both
//  work.
//Sparkfun
//https://github.com/sparkfun/SparkFun_TMAG5273_Arduino_Library/...
//  blob/main/src/SparkFun_TMAG5273_Arduino_Library.h
TMAG5273 sensor; // Initialize hall-effect sensor

uint8_t i2cAddress = TMAG5273_I2C_ADDRESS_INITIAL;
//uint8_t i2cAddress = 0x35;

// Set constants for setting up device
uint8_t conversionAverage = TMAG5273_X32_CONVERSION;
uint8_t magneticChannel = TMAG5273_XYX_ENABLE;
uint8_t angleCalculation = TMAG5273_XY_ANGLE_CALCULATION;


//  https://docs.simplefoc.com/magnetic_sensor_i2c




//interrupt example
// https://github.com/sparkfun/SparkFun_IoT_Brushless_Motor_Driver/...
//  blob/main/docs/assets/arduino_examples/IoT_MotorDriver/IoT_MotorDriver.ino

//generic sensor for simplefoc
//https://docs.simplefoc.com/generic_sensor
float get_mag_angle( TMAG5273 &sensor){
  uint8_t lsb = sensor.getReg(0x1A); //note:  this is a method I wrote for low level reg access
  uint8_t msb = sensor.getReg(0x19);
  uint16_t val = msb << 8 | lsb;
  float ang = (float)val / 16.0;
  return ang;
}


void genericSensorInit( void ){
  uint8_t i2cAddress = TMAG5273_I2C_ADDRESS_INITIAL;
  uint8_t conversionAverage = TMAG5273_X32_CONVERSION;
  uint8_t magneticChannel = TMAG5273_XYX_ENABLE;
  uint8_t angleCalculation = TMAG5273_XY_ANGLE_CALCULATION;
  Wire.begin();
  sensor.begin(i2cAddress, Wire);
  // Set the device at 32x average mode 
  sensor.setConvAvg(conversionAverage);
  // Choose new angle to calculate from
  // Can calculate angles between XYX, YXY, YZY, and XZX
  sensor.setMagneticChannel(magneticChannel);
  // Enable the angle calculation register
  // Can choose between XY, YZ, or XZ priority
  sensor.setAngleEn(angleCalculation);
}

//Generic sensor to return [0..2*pi]
float genericSensorReadCallback( void ){
  return DEG2RAD( get_mag_angle(sensor) );
}

GenericSensor genSensor = GenericSensor(genericSensorReadCallback, genericSensorInit);

void setup() {


  //RGB LED init
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);
  leds[0] = CRGB::Red;
  FastLED.setBrightness(10);
  FastLED.show();
  
  //Serial port init
  Serial.begin(921600);
  Serial.println("Serial initialized...");

  //

  #if 0
  //I2C / TMAG init
  Wire.begin();
  int8_t ret = sensor.begin(i2cAddress, Wire);
  if( ret){
    Serial.println("Wire init...");
  }
  // Set the device at 32x average mode 
  sensor.setConvAvg(conversionAverage);
  // Choose new angle to calculate from
  // Can calculate angles between XYX, YXY, YZY, and XZX
  sensor.setMagneticChannel(magneticChannel);
  // Enable the angle calculation register
  // Can choose between XY, YZ, or XZ priority
  sensor.setAngleEn(angleCalculation);
  #endif





}//end setup




void loop() {
  #if 0
  // put your main code here, to run repeatedly:
  leds[0] = CRGB::Red;
  FastLED.setBrightness(100);
  FastLED.show();
  Serial.println("red");
  delay(1000);

  leds[0] = CRGB::Green;
  FastLED.setBrightness(100);
  FastLED.show();
  Serial.println("green");
  delay(1000);

  leds[0] = CRGB::Blue;
  FastLED.setBrightness(100);
  FastLED.show();
  Serial.println("blue");
  delay(1000);
  #endif

  if((sensor.getMagneticChannel() != 0) && (sensor.getAngleEn() != 0)) // Checks if mag channels are on - turns on in setup
  {
    //float angleCalculation = sensor.getAngleResult();

    //Serial.print("XYX: ");
    //Serial.print(angleCalculation, 4);
    //Serial.println("°");
    
    //uint8_t lsb = sensor.getReg(0x1A);
    //uint8_t msb = sensor.getReg(0x19);
    //uint16_t val = msb << 8 | lsb;
    //float ang = (float)val / 16.0;
    //Serial.printf("%0.2f\n", ang);

    //Serial.printf("%0.2f\n", get_mag_angle(sensor) );
    Serial.printf("%0.3f\n", genericSensorReadCallback() );

  }
  else
  {
    Serial.println("Mag Channels disabled, stopping..");
    while(1);
  }

  delay(25);

} //end loop

