//youtube tutorial  https://www.youtube.com/watch?v=3B88qCny7Kg&t=451s



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


#define DEG2RAD(dd) ((float)dd*(float)0.01745329251)
#define RAD2DEG(rr)


CRGB leds[NUM_LEDS];

//https://github.com/dani007200964/Shellminator/blob/main/examples/...
//  Shellminator_ESP32_server_with_Commander/Shellminator_ESP32_server_with_Commander.ino
#include "Shellminator.hpp"
#include "Shellminator-IO.hpp"

// NOTE:  THERE IS A NAMESPACE COLLISION WITH THE COMMANDER INTERFACE FOR
//  SIMPLEFOC; TO USE THE SHELLMINATOR / COMMANDER SHELL, THE FOLLOWING
//  LINE MUST BE COMMENTED OUT IN SimpleFOC.h
//  #include "communication/Commander.h"

// Necessary includes
#include "Commander-API.hpp"
#include "Commander-IO.hpp"
#include "Commander-API-Commands.hpp"

// Create a Shellminator object, and initialize it to use Serial
Shellminator shell( &Serial );


const char logo[] = "SHELLMINATOR\r\n";

// We have to create an object from Commander class.
Commander commander;

// We have to create the prototype functions for our commands.
// The arguments has to be the same for all command functions.
void cat_func(char *args, Stream *response);
void dog_func( char *args, Stream *response );
void sum_func( char *args, Stream *response );
void led_func( char *args, Stream *response );

void mtr_ena_func(char *args, Stream *response);
static float target = 0.0;
void set_tgt_func(char *args, Stream *response){
  sscanf(args, "%f", &target);
}

void vel_kp_func(char *args, Stream *response);
void vel_ki_func(char *args, Stream *response);
void vel_kd_func(char *args, Stream *response);
void vel_ra_func(char *args, Stream *response); //todo
void vel_tf_func(char *args, Stream *response); //todo

void pos_kp_func(char *args, Stream *response);
void pos_ki_func(char *args, Stream *response);
void pos_kd_func(char *args, Stream *response);
void pos_ra_func(char *args, Stream *response);//todo
void pos_lm_func(char *args, Stream *response);//todo

// Commander API-tree
Commander::API_t API_tree[] = {
    apiElement( "cat", "Description for cat command.", cat_func ),
    apiElement( "dog", "Description for dog command.", dog_func ),
    apiElement( "led", "Toggle the buit-in LED.", led_func ),
    apiElement( "sum", "This function sums two number from the argument list.", sum_func ),

    apiElement("ena", "Enable/disable motor amplifier", mtr_ena_func),
    apiElement("tgt", "Set target value", set_tgt_func ),

    apiElement("vkp", "Get / set velocity kp gain", vel_kp_func ),
    apiElement("vki", "Get / set velocity ki gain", vel_ki_func ),
    apiElement("vkd", "Get / set velocity kd gain", vel_kd_func ),

    apiElement("pkp", "Get / set position kp gain", pos_kp_func ),
    apiElement("pki", "Get / set position ki gain", pos_ki_func ),
    apiElement("pkd", "Get / set position kd gain", pos_kd_func )

};





// note:  TMAG5273 is a multi axis sensor; in the physical configuration
//  on the sparkfun iot board, the magnet on the back of the motor is 
//  diametrically polorized with the sensor XY plane parallel to the 
//  diametric polarization.  Therefore the X or Y sensing axes should both
//  work.
//Sparkfun
//https://github.com/sparkfun/SparkFun_TMAG5273_Arduino_Library/...
//  blob/main/src/SparkFun_TMAG5273_Arduino_Library.h
TMAG5273 tmag; // Initialize hall-effect sensor


//  https://docs.simplefoc.com/magnetic_sensor_i2c




//interrupt example
// https://github.com/sparkfun/SparkFun_IoT_Brushless_Motor_Driver/...
//  blob/main/docs/assets/arduino_examples/IoT_MotorDriver/IoT_MotorDriver.ino

//generic sensor for simplefoc
//https://docs.simplefoc.com/generic_sensor
float get_mag_angle( TMAG5273 &sensor){

  if((sensor.getMagneticChannel() != 0) && (sensor.getAngleEn() != 0)){
    uint8_t lsb = sensor.getReg(0x1A); //note:  this is a method I wrote for low level reg access
    uint8_t msb = sensor.getReg(0x19);
    uint16_t val = msb << 8 | lsb;
    float ang = (float)val / 16.0;
    return ang;
  }
  else{
    return -1;
  }
}


void genericSensorInit( void ){
  uint8_t i2cAddress = TMAG5273_I2C_ADDRESS_INITIAL;
  uint8_t conversionAverage = TMAG5273_X32_CONVERSION;
  uint8_t magneticChannel = TMAG5273_XYX_ENABLE;
  uint8_t angleCalculation = TMAG5273_XY_ANGLE_CALCULATION;
  Wire.begin();
  tmag.begin(i2cAddress, Wire);
  // Set the device at 32x average mode 
  tmag.setConvAvg(conversionAverage);
  // Choose new angle to calculate from
  // Can calculate angles between XYX, YXY, YZY, and XZX
  tmag.setMagneticChannel(magneticChannel);
  // Enable the angle calculation register
  // Can choose between XY, YZ, or XZ priority
  tmag.setAngleEn(angleCalculation);
  _delay(5);
}

//Generic sensor to return [0..2*pi]
//NOTE:  negative number not allowed to be returned
float genericSensorReadCallback( void ){
  return DEG2RAD( get_mag_angle(tmag) );
}

GenericSensor genSensor = GenericSensor(genericSensorReadCallback, genericSensorInit);
BLDCMotor motor = BLDCMotor(MTR_POLES);
BLDCDriver6PWM driver = BLDCDriver6PWM(UH_PHASE, UL_PHASE, VH_PHASE, VL_PHASE, WH_PHASE, WL_PHASE);
LowPassFilter velLPF = LowPassFilter(SENS_VEL_LPF_TF);
LowPassFilter posLPF = LowPassFilter(SENS_POS_LPF_TF);

// set up current sensing
// https://docs.simplefoc.com/low_side_current_sense
LowsideCurrentSense phase_current = LowsideCurrentSense(
    CURR_SENSE_PHASE_RES, CURR_SENSE_PHASE_GAIN, CURR_SENSE_PHASE_U, CURR_SENSE_PHASE_V, CURR_SENSE_PHASE_W);

void setup() {


  //RGB LED init
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);
  leds[0] = CRGB::Red;
  FastLED.setBrightness(10);
  FastLED.show();
  
  //Serial port init
  Serial.begin(921600);
  Serial.println("Serial initialized...");

  motor.useMonitoring(Serial);

  // Initialize the magnetic sensor
  genSensor.init();

  _delay(10);

  // Clear the terminal
  shell.clear();
  // Attach the logo.
  shell.attachLogo( logo );
  // Print start message
  Serial.println( "Program begin..." );
  // There is an option to attach a debug channel to Commander.
  // It can be handy to find any problems during the initialization
  // phase. In this example we will use Serial for this.
  commander.attachDebugChannel( &Serial );
  // At start, Commander does not know anything about our commands.
  // We have to attach the API_tree array from the previous steps
  // to Commander to work properly.
  commander.attachTree( API_tree );
  // Initialize Commander.
  commander.init();
  shell.attachCommander( &commander );
  // initialize shell object.
  shell.begin( "snortman" );

  //Init IO needed to enable motor contorl
  pinMode(nSTDBY, OUTPUT);
  digitalWrite(nSTDBY, 1);
  // Set TMC6300 DIAG connection to input
  pinMode(DIAG, INPUT);
  //TODO: set up interrupt for DIAG 


  //SimpleFOC Motor Setup
  driver.voltage_power_supply = DRV_VOLT_SUPP;
  driver.pwm_frequency = DRV_PWM_FREQ;
  driver.voltage_limit = DRV_VOLT_LIMIT;
  driver.init();


  phase_current.linkDriver(&driver);
 

  motor.linkDriver(&driver);
  motor.voltage_limit = MTR_VOLT_LIMIT;
  motor.velocity_limit = MTR_VEL_LIMIT;
  motor.LPF_velocity.Tf = MTR_LPF_TF;
  motor.voltage_sensor_align = 1;

  motor.linkSensor(&genSensor);
  

  //open loop
  //motor.controller = MotionControlType::velocity_openloop;
  motor.torque_controller = TorqueControlType::voltage; // NOTE: voltage seems to have the best performance
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  motor.controller = MotionControlType::angle;
  //motor.controller = MotionControlType::velocity;
  //motor.controller = MotionControlType::torque;

  // Default values found empirically with no load
  motor.PID_velocity.P = VEL_KP_DFLT;
  motor.PID_velocity.I = VEL_KI_DFLT;
  motor.PID_velocity.D = VEL_KD_DFLT;
  motor.PID_velocity.limit = VEL_RAMP_DFLT;

  motor.P_angle.P = POS_KP_DFLT;
  motor.P_angle.I = POS_KI_DFLT;
  motor.P_angle.D = POS_KD_DFLT;
  motor.P_angle.limit = POS_LIM_DFLT;
  motor.P_angle.output_ramp = POS_RAMP_DFLT;

  motor.init();

  phase_current.init();
  motor.linkCurrentSense(&phase_current);

  motor.initFOC();

  //voltage close loop

 

  //motor.disable();
  motor.enable();

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

// CLI example code

//end CLI example code

void loop() {

  //genSensor.update();
  //Serial.printf("%0.2f\t\t%0.2f\r\n", posLPF(genSensor.getAngle()), velLPF(genSensor.getVelocity()) );
  //delay(10);

  shell.update();

  motor.loopFOC();
  motor.move(target);

  //PhaseCurrent_s currents = phase_current.getPhaseCurrents();
  //float current_magnitude = phase_current.getDCCurrent();

  //Serial.printf("%0.2f\t%0.2f\t%0.2f\t%0.2f\r\n", currents.a * 1000.0, currents.b*1000.0, currents.c*1000.0, current_magnitude*1000.0);

  //motor.monitor();

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

  #if 0
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

  #endif

} //end loop

/// This is an example function for the cat command
void cat_func(char *args, Stream *response )
{

  response -> print("Hello from cat function!\r\n");

}

/// This is an example function for the dog command
void dog_func(char *args, Stream *response )
{

  response -> print("Hello from dog function!\r\n");

}

/// This is an example function for the led command
void led_func(char *args, Stream *response )
{

  // Toggle your LED pin here, if you have on your board
  // digitalWrite( LED_PIN, !digitalRead( LED_PIN ) );
  response -> print("LED toggle!\r\n");

}

/// This is an example function for the sum command
void sum_func(char *args, Stream *response )
{

  // These variables will hold the value of the
  // two numbers, that has to be summed.
  int a = 0;
  int b = 0;

  // This variable will hold the result of the
  // argument parser.
  int argResult;

  // This variable will hold the sum result.
  int sum = 0;

  argResult = sscanf( args, "%d %d", &a, &b );

  // We have to check that we parsed successfully  the two
  // numbers from the argument string.
  if( argResult != 2 ){

    // If we could not parse two numbers, we have an argument problem.
    // We print out the problem to the response channel.
    response -> print( "Argument error! Two numbers required, separated with a blank space.\r\n" );

    // Sadly we have to stop the command execution and return.
    return;

  }

  // Calculate the sum.
  sum = a + b;

  // Print out the result.
  response -> print( a );
  response -> print( " + " );
  response -> print( b );
  response -> print( " = " );
  response -> println( sum );

}

//commander function to enable / disable
// example
//  > ena 1
//  > ena 0
void mtr_ena_func(char *args, Stream *response){
  // note:  the TMC6300 driver IC input pin 11 VIO/nSTDBY is driven by
  //  ESP32_GPIO5
  //

  int val = 0;

  int argResult = sscanf(args, "%d", &val);

  if( argResult!=1 ){
    Serial.println("Single [0|1] argument required...");
    return;
  }

  if(0==val)
    motor.disable();
  else if(1==val)
    motor.enable();
  else
    Serial.println("Single [0|1] argument required...");

}//mtr_ena_func

void vel_kp_func(char *args, Stream *response){
  int n = 0;
  float v = 0.0;
  //todo: error checking
  n=sscanf(args, "%f", &v);
  if(EOF==n)
    response->printf("VP:[%f]\r\n", motor.PID_velocity.P);
  else if(1==n){  
    motor.PID_velocity.P = v;
    response->printf("VP:[%f]\r\n", motor.PID_velocity.P);
  }
  else
    response->printf("Incorrect args, [%d,%f]...\r\n",n,v);
}
void vel_ki_func(char *args, Stream *response){
  int n = 0;
  float v = 0.0;
  //todo: error checking
  n=sscanf(args, "%f", &v);
  if(EOF==n)
    response->printf("VI:[%f]\r\n", motor.PID_velocity.I);
  else if(1==n){  
    motor.PID_velocity.I = v;
    response->printf("VI:[%f]\r\n", motor.PID_velocity.I);
  }
  else
    response->printf("Incorrect args...\r\n");
}
void vel_kd_func(char *args, Stream *response){
  int n = 0;
  float v = 0.0;
  //todo: error checking
  n=sscanf(args, "%f", &v);
  if(EOF==n)
    response->printf("VD:[%f]\r\n", motor.PID_velocity.D);
  else if(1==n){  
    motor.PID_velocity.D = v;
    response->printf("VD:[%f]\r\n", motor.PID_velocity.D);
  }
  else
    response->printf("Incorrect args...\r\n");
}
void pos_kp_func(char *args, Stream *response){
  int n = 0;
  float v = 0.0;
  //todo: error checking
  n=sscanf(args, "%f", &v);
  if(EOF==n)
    response->printf("PP:[%f]\r\n", motor.P_angle.P);
  else if(1==n){  
    motor.P_angle.P = v;
    response->printf("PP:[%f]\r\n", motor.P_angle.P);
  }
  else
    response->printf("Incorrect args...\r\n");
};
void pos_ki_func(char *args, Stream *response){
  int n = 0;
  float v = 0.0;
  //todo: error checking
  n=sscanf(args, "%f", &v);
  if(EOF==n)
    response->printf("PI:[%f]\r\n", motor.P_angle.I);
  else if(1==n){  
    motor.P_angle.I = v;
    response->printf("PI:[%f]\r\n", motor.P_angle.I);
  }
  else
    response->printf("Incorrect args...\r\n");
};
void pos_kd_func(char *args, Stream *response){
  int n = 0;
  float v = 0.0;
  //todo: error checking
  n=sscanf(args, "%f", &v);
  if(EOF==n)
    response->printf("PD:[%f]\r\n", motor.P_angle.D);
  else if(1==n){  
    motor.P_angle.D = v;
    response->printf("PD:[%f]\r\n", motor.P_angle.D);
  }
  else
    response->printf("Incorrect args...\r\n");
};

