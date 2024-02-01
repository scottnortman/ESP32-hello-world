//youtube tutorial  https://www.youtube.com/watch?v=3B88qCny7Kg&t=451s


#include <math.h>

#include <Arduino.h>

//https://community.simplefoc.com/t/possible-bug-with-esp32-micros-function/122
#include <esp_timer.h>

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

//float to int
//https://jkorpela.fi/round.html
#define ROUND(x) ((x)>=0?(uint64_t)((x)+0.5):(uint64_t)((x)-0.5))


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

void mtr_ena_func(char *args, Stream *response);
static float target = 0.0;
// Note
// $ tgt <float>
// $ tgt sin <ampl> <ofst> <freq_hz)
// % tgt sqw <ampl> <ofst> <freq_hz>
void set_tgt_func(char *args, Stream *response);

void mon_en_func(char *args, Stream *response); // Monitor output enable / disable
void rgb_func(char *args, Stream *response);   // RGB LED control

void vel_kp_func(char *args, Stream *response); // Velocity Kp gain
void vel_ki_func(char *args, Stream *response); // Velocity Ki gain
void vel_kd_func(char *args, Stream *response); // Velocity Kd gain
void vel_ra_func(char *args, Stream *response); // Velocity output ramp       [rad/s^2]
void vel_tf_func(char *args, Stream *response); // Velocity Tf filter period  [s]

void pos_kp_func(char *args, Stream *response); // Position Kp gain
void pos_ki_func(char *args, Stream *response); // Position Ki gain
void pos_kd_func(char *args, Stream *response); // Position Kd gain
void pos_ra_func(char *args, Stream *response); // Position ramp,   [rad/s]
void pos_lm_func(char *args, Stream *response); // Position limit,  [rad]

void pid_shell_func(char *args, Stream *response);

//Flag to enable monitoring
static bool mon_enabled = false;

// Commander API-tree
Commander::API_t API_tree[] = {

    apiElement("ena", "Enable/Disable motor amplifier", mtr_ena_func),
    apiElement("tgt", "Set target value", set_tgt_func),
    apiElement("mon", "Enable/Disable monitoring", mon_en_func),
    apiElement("rgb", "Get / set RGB values of LED", rgb_func),

    apiElement("vkp", "Get / set velocity kp gain", vel_kp_func),
    apiElement("vki", "Get / set velocity ki gain", vel_ki_func),
    apiElement("vkd", "Get / set velocity kd gain", vel_kd_func),
    apiElement("vra", "Get / set velocity ramp", vel_ra_func),
    apiElement("vtf", "Get / set velocity Tf", vel_tf_func),

    apiElement("pkp", "Get / set position kp gain", pos_kp_func),
    apiElement("pki", "Get / set position ki gain", pos_ki_func),
    apiElement("pkd", "Get / set position kd gain", pos_kd_func),
    apiElement("pra", "Get / set position ramp", pos_ra_func),
    apiElement("plm", "Get / set position limit", pos_lm_func),

    apiElement("pid", "Get / set PID Controller parameters", pid_shell_func),

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

//https://docs.simplefoc.com/dc_current_torque_mode
InlineCurrentSense phase_current = InlineCurrentSense(
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
  //motor.torque_controller = TorqueControlType::voltage; // NOTE: voltage seems to have the best performance
  //motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  //debug/test FOC
  motor.torque_controller = TorqueControlType::dc_current;


  //motor.controller = MotionControlType::angle;
  //motor.controller = MotionControlType::velocity;
  motor.controller = MotionControlType::torque;

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

  motor.PID_current_d.P = 0.0;
  motor.PID_current_d.I = 0.0;
  motor.PID_current_d.D = 0.0;
  motor.PID_current_d.limit = motor.voltage_limit;
  motor.PID_current_d.output_ramp = 1e6;
  motor.LPF_current_d.Tf = 0.01;

  motor.PID_current_q.P = 0.0;
  motor.PID_current_q.I = 0.0;
  motor.PID_current_q.D = 0.0;
  motor.PID_current_q.limit = motor.voltage_limit;
  motor.PID_current_q.output_ramp = 1e6;
  motor.LPF_current_q.Tf = 0.01;
  

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
typedef enum
{
  TGT_WAVE_CONST = 0,
  TGT_WAVE_SIN = 1,
  TGT_WAVE_SQW = 2,
} TGT_WAVE_TYPE;

static TGT_WAVE_TYPE tgt_wave = TGT_WAVE_CONST;
static float tgt_ampl = 0.0;
static float tgt_ofst = 0.0;  
static float tgt_freq = 0.0;        // [Hz]
static uint64_t tgt_tstart_us = 0;  //

float run_tgt_func(void);

static bool pstate = 0;
void loop() {

  genSensor.update();
  //https://teleplot.fr/
  //Serial.printf(">ang:%f\n>vel:%f\n", posLPF(genSensor.getAngle()), velLPF(genSensor.getVelocity()) );
  //delay(10);

  shell.update();

  motor.loopFOC();


  motor.move( run_tgt_func() );

  PhaseCurrent_s currents = phase_current.getPhaseCurrents();
  float current_magnitude = phase_current.getDCCurrent();

  //https://teleplot.fr/
  if(mon_enabled){
  
    //Serial.printf(">a:%f\n>b:%f\n>c:%f\n>d:%f\n", currents.a * 1000.0, currents.b*1000.0, currents.c*1000.0, current_magnitude*1000.0);
    Serial.printf(">tgt:%f\n>act:%f\n", motor.target, current_magnitude);
  }
  
  
  //motor.monitor();

  //float tt = run_tgt_func();
  //Serial.printf("%f\r\n", tt);

  //float dac_scale = 255.0 * tgt_ampl / tt;

  //dacWrite(26, (uint8_t)tt);
  //Serial.printf(">tt:%f\n", tt);

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

//commander function to enable / disable
// example
//  > ena 1
//  > ena 0
void mtr_ena_func(char *args, Stream *response){

  int val = 0;
  int argResult = sscanf(args, "%d", &val);
  
  if(EOF==argResult){
    //No additional args, print current enabled state
    response->println(motor.enabled);
    return;
  }
  if(1==argResult){
    // Single arg, check value to enable / disable
    if(0==val){
      motor.disable();
      response->println(motor.enabled);
    }
    else if(1==val){
      motor.enable();
      response->println(motor.enabled);
    }
    else{
      // Otherwise incorrect arg
      Serial.println("Single [0|1] argument required...");
    }
  }

}//mtr_ena_func

// Monitor output enable / disable
void mon_en_func(char *args, Stream *response){
  int val = 0;
  int argResult = sscanf(args, "%d", &val);

  if(EOF==argResult){
    //No additional args, print current enabled state
    response->println(mon_enabled);
    return;
  }
  if(1==argResult){
    // Single arg, check value to enable / disable
    if(0==val){
      mon_enabled = false;
      response->println(mon_enabled);
    }
    else if(1==val){
      mon_enabled = true;
      response->println(mon_enabled);
    }
    else{
      // Otherwise incorrect arg
      Serial.println("Single [0|1] argument required...");
    }
  }
}//mon_en_func

// RGB LED control
// $ rgb <red> <grn> <blu> <pwr>  #set r g b and power level
// $ rgb                          # query current values
void rgb_func(char *args, Stream *response){
  int r, g, b, p;
  CRGB *led;
  p = FastLED.getBrightness();
  led = FastLED.leds();

  int n = sscanf(args, "%d %d %d %d", &r, &g, &b, &p);
  
  if(EOF==n)
    response->printf("%d %d %d %d", led->red, led->green, led->blue, p);
  else if(4==n){
    led->red = r;
    led->green = g;
    led->blue = b;
    leds[0] = *led;
    FastLED.setBrightness(p);
    FastLED.show();
    response->printf("%d %d %d %d", led->red, led->green, led->blue, p);
  }
  else{
    response->printf("Incorrect args....\r\n");
  }

}// rgb_func

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

 // Velocity output ramp       [rad/s^2]
void vel_ra_func(char *args, Stream *response){}
// Velocity Tf filter period  [s]
void vel_tf_func(char *args, Stream *response){}

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
}

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
}

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
}

// Position ramp,   [rad/s]
void pos_ra_func(char *args, Stream *response){

}
// Position limit,  [rad]
void pos_lm_func(char *args, Stream *response){

}



// run_tgt_func
// this function to get called repeatedly to implement the needed waveform
float run_tgt_func(void){

  if(tgt_freq < 1e-3){
    // Freq too small, set to static
    tgt_wave = TGT_WAVE_CONST;
  }
    
  switch(tgt_wave){
    case TGT_WAVE_CONST:
    {
      // Constant value, return the amplitude
      return tgt_ampl;
      break;
    }
    case TGT_WAVE_SIN:
    {
      // sine wave; we need to see where in the cycle we are
      uint64_t tgt_tdiff_us = micros() - tgt_tstart_us; //TODO handle rollover
      // get period in us
      uint64_t tgt_period_us = ROUND(1e6 / tgt_freq);
      uint64_t tgt_modulo_us = tgt_tdiff_us % tgt_period_us;
      //Given the remainder, we can calulate where we are in the cycle
      float tgt_cycle_scale = (float)tgt_modulo_us / (float)tgt_period_us;
      float val = 2.0 * PI * tgt_cycle_scale;
      float ampl = tgt_ampl * sin(val) + tgt_ofst;
      return (float)ampl;
      break;
    }
    case TGT_WAVE_SQW:
    {
      // 50% DC square wave; we need to see where in the cycle we are
      uint64_t tgt_tdiff_us = micros() - tgt_tstart_us; //TODO handle rollover
      // get period in us
      uint64_t tgt_period_us = (uint64_t)(1e6 / tgt_freq);
      uint64_t tgt_modulo_us = tgt_tdiff_us % tgt_period_us;
      float tgt_cycle_scale = (float)tgt_modulo_us / (float)tgt_period_us;
      if( tgt_cycle_scale < 0.5 ){
        // First half of the cycle; output high value
        float ampl = tgt_ampl + tgt_ofst;
        return (float)ampl;
      }
      else{
        float ampl = tgt_ofst;
        return (float)ampl;
      }
      break;
    }
    default:
      break;
  }

}
// Note
// $ tgt con <ampl>
// $ tgt sin <ampl> <freq_hz> <ofst> 
// % tgt sqw <ampl> <freq_hz> <ofst>
void set_tgt_func(char *args, Stream *response){
  int n = 0;
  //local vals
  char wave_str[10] = {'\0'};
  float ampl = 0.0;
  float ofst = 0.0;
  float freq = 0.0;

  // TODO: prevent buffer overflow
  n = sscanf(args, "%s %f %f %f", wave_str, &ampl, &freq, &ofst);

  //note:  the first string, 'tgt' is not passed as part of the args
  if(EOF==n){
    //no args, return current target based on mode
    switch(tgt_wave){
      case TGT_WAVE_CONST:
        // Show single target value
        response->printf("con a[%f]\r\n", tgt_ampl);
        break;
      case TGT_WAVE_SIN:
        // show sin wave values
        response->printf("sin a[%f] f[%f] o[%f]\r\n", tgt_ampl, tgt_freq, tgt_ofst);
        break;
      case TGT_WAVE_SQW:
        // show square wave values
        response->printf("sqw a[%f] f[%f] o[%f]\r\n", tgt_ampl, tgt_freq, tgt_ofst);
        break;
      default:
        // error
        break;
    }
  }
  else if(1==n){
    // 1 arg can be val, sin, sqw, this just changes the mode and will not affect other values
    if(0==strcmp("con", wave_str)){
      // constant value
      tgt_wave = TGT_WAVE_CONST;
      tgt_tstart_us = micros();
    }
    else if(0==strcmp("sin", wave_str)){
      // sine wave
      tgt_wave = TGT_WAVE_SIN;
      tgt_tstart_us = micros();
    }
    else if(0==strcmp("sqw", wave_str)){
      // square wave
      tgt_wave = TGT_WAVE_SQW;
      tgt_tstart_us = micros();
    }
    else{
      //error
      response->printf("Invalid arg [%s]\r\n", wave_str);
    }
  }
  else if(2==n){
    //2 args:type and ampl
    if(0==strcmp("con", wave_str)){
      // constant value
      tgt_wave = TGT_WAVE_CONST;
      tgt_tstart_us = micros();
      tgt_ampl = ampl;
    }
    else if(0==strcmp("sin", wave_str)){
      // sine wave
      tgt_wave = TGT_WAVE_SIN;
      tgt_tstart_us = micros();
      tgt_ampl = ampl;
    }
    else if(0==strcmp("sqw", wave_str)){
      // square wave
      tgt_wave = TGT_WAVE_SQW;
      tgt_tstart_us = micros();
      tgt_ampl = ampl;
    }
    else{
      //error
      response->printf("Invalid arg [%s %f]\r\n", wave_str, ampl);
    }
  }
  else if(3==n){
    // 3 args: type ampl freq
    if(0==strcmp("con", wave_str)){
      // constant value
      tgt_wave = TGT_WAVE_CONST;
      tgt_tstart_us = micros();
      tgt_ampl = ampl;
      tgt_freq = freq;
    }
    else if(0==strcmp("sin", wave_str)){
      // sine wave
      tgt_wave = TGT_WAVE_SIN;
      tgt_tstart_us = micros();
      tgt_ampl = ampl;
      tgt_freq = freq;
    }
    else if(0==strcmp("sqw", wave_str)){
      // square wave
      tgt_wave = TGT_WAVE_SQW;
      tgt_tstart_us = micros();
      tgt_ampl = ampl;
      tgt_freq = freq;
    }
    else{
      //error
      response->printf("Invalid arg [%s %f %f]\r\n", wave_str, ampl, freq);
    }
  }
  else if(4==n){
    // 4 args: type ampl freq ofst
    if(0==strcmp("con", wave_str)){
      // constant value
      tgt_wave = TGT_WAVE_CONST;
      tgt_tstart_us = micros();
      tgt_ampl = ampl;
      tgt_freq = freq;
      tgt_ofst = ofst;
    }
    else if(0==strcmp("sin", wave_str)){
      // sine wave
      tgt_wave = TGT_WAVE_SIN;
      tgt_tstart_us = micros();
      tgt_ampl = ampl;
      tgt_freq = freq;
      tgt_ofst = ofst;
    }
    else if(0==strcmp("sqw", wave_str)){
      // square wave
      tgt_wave = TGT_WAVE_SQW;
      tgt_tstart_us = micros();
      tgt_ampl = ampl;
      tgt_freq = freq;
      tgt_ofst = ofst;
    }
    else{
      //error
      response->printf("Invalid arg [%s %f %f %f]\r\n", wave_str, ampl, freq, ofst);
    }
  }
  else{
    // error
  }
} //set_tgt_func

// command
//  $ pid <pos|vel|ud|uq> <kp|ki|kd|tf|ra|pl|vl|lm> [<float>]
//  $ pid pos kp # returns kp gain for position control
//  $ pid uq ki 0.102 # sets the integral gain for the Uq PID controller
void pid_shell_func( char *args, Stream *response ){
  #define MAX_STRLEN 10

  int n = 0;
  char type[MAX_STRLEN];   //TODO: handle buffer overflow
  char param[MAX_STRLEN];  //TODO: hadle buffer overflow
  float value = 0.0;
  n = sscanf(args, "%s %s %f", type, param, &value);
  //debug
  //response->printf("{%s %s %f}\r\n");
  if(EOF==n)
    response->printf("no args");
  else if(2==n){
    // get
    if(0==strcmp("pos", type)){
      // get pos
      if(0==strcmp("kp", param)){
        // get pos kp
        response->printf("%f\r\n", motor.P_angle.P);
      }
      else if(0==strcmp("ki", param)){
        // get pos ki
        response->printf("%f\r\n", motor.P_angle.I);
      }
      else if(0==strcmp("kd", param)){
        // get pos kd
        response->printf("%f\r\n", motor.P_angle.D);
      }
      else if(0==strcmp("tf", param)){
        // get pos tf
        response->printf("%f\r\n", motor.LPF_angle.Tf);
      }
      else if(0==strcmp("ra", param)){
        // get pos ramp
        response->printf("%f\r\n", motor.P_angle.output_ramp);
      }
      else if(0==strcmp("pl", param)){
        // get pos limit
        response->printf("%f\r\n", motor.P_angle.limit);
      }
      else if(0==strcmp("vl", param)){
        // get pos velocity limit
        response->printf("%f\r\n", motor.velocity_limit);
      }
      else{
        //error
        response->printf("Invalid param [%s]\r\n", param);
      }
    }
    else if (0==strcmp("vel", type)){
      // get vel
      if(0==strcmp("kp", param)){
        // get vel kp
        response->printf("%f\r\n", motor.PID_velocity.P);
      }
      else if(0==strcmp("ki", param)){
        // get vel ki
        response->printf("%f\r\n", motor.PID_velocity.I);
      }
      else if(0==strcmp("kd", param)){
        // get vel kd
        response->printf("%f\r\n", motor.PID_velocity.D);
      }
      else if(0==strcmp("tf", param)){
        // get vel tf     [s]
        response->printf("%f\r\n", motor.LPF_velocity.Tf);
      }
      else if(0==strcmp("ra", param)){
        // get vel ramp,  [V/s]
        response->printf("%f\r\n", motor.PID_velocity.output_ramp);
      }
      else if(0==strcmp("lm", param)){
        response->printf("%f\r\n", motor.PID_velocity.limit);
      }
      else{
        //error
        response->printf("Invalid param [%s]\r\n", param);
      }
    }
    else if (0==strcmp("ud", type)){
      if(0==strcmp("kp", param)){
        // get ud kp
        response->printf("%f\r\n", motor.PID_current_d.P);
      }
      else if(0==strcmp("ki", param)){
        // get ud ki
        response->printf("%f\r\n", motor.PID_current_d.I);
      }
      else if(0==strcmp("kd", param)){
        // get ud kd
        response->printf("%f\r\n", motor.PID_current_d.D);
      }
      else if(0==strcmp("ra", param)){
        // get ud tf
        response->printf("%f\r\n", motor.PID_current_d.output_ramp);
      }
      else if(0==strcmp("lm", param)){
        // get ud limit
        response->printf("%f\r\n", motor.PID_current_d.limit);
      }
      else if(0==strcmp("tf", param)){
        //get filter period
        response->printf("%f\r\n", motor.LPF_current_d.Tf);
      }
      else{
        //error
        response->printf("Invalid param [%s]\r\n", param);
      }
    }
    else if (0==strcmp("uq", type)){
      if(0==strcmp("kp", param)){
        // get uq kp
        response->printf("%f\r\n", motor.PID_current_q.P);
      }
      else if(0==strcmp("ki", param)){
        // get uq ki
        response->printf("%f\r\n", motor.PID_current_q.I);
      }
      else if(0==strcmp("kd", param)){
        // get uq kd
        response->printf("%f\r\n", motor.PID_current_q.D);
      }
      else if(0==strcmp("ra", param)){
        // get uq tf
        response->printf("%f\r\n", motor.PID_current_q.output_ramp);
      }
      else if(0==strcmp("lm", param)){
        // get uq limit
        response->printf("%f\r\n", motor.PID_current_q.limit);
      }
      else if(0==strcmp("tf", param)){
        //get filter period
        response->printf("%f\r\n", motor.LPF_current_q.Tf);
      }
      else{
        //error
        response->printf("Invalid param [%s]\r\n", param);
      }
    }
    else{
      // Error
      response->printf("Invalid type [%s]\r\n", type);
    }
  }
  else if(3==n){
    // Set value
    if(0==strcmp("pos", type)){
      if(0==strcmp("kp", param)){
        // set pos kp
        motor.P_angle.P = value;
      }
      else if(0==strcmp("ki", param)){
        // set pos ki
        motor.P_angle.I = value;
      }
      else if(0==strcmp("kd", param)){
        // set pos kd
        motor.P_angle.D = value;
      }
      else if(0==strcmp("tf", param)){
        // set pos tf
        motor.LPF_angle.Tf = value;
      }
      else if(0==strcmp("ra", param)){
        // set pos ramp
        motor.P_angle.output_ramp = value;
      }
      else if(0==strcmp("lm", param)){
        // set pos limit
        motor.P_angle.limit = value;
      }
      else{
        //error
        response->printf("Invalid param [%s]\r\n", param);
      }
    }
    else if (0==strcmp("vel", type)){
      if(0==strcmp("kp", param)){
        // set vel kp
        motor.PID_velocity.P = value;
      }
      else if(0==strcmp("ki", param)){
        // set vel ki
        motor.PID_velocity.I = value;
      }
      else if(0==strcmp("kd", param)){
        // set vel kd
        motor.PID_velocity.D = value;
      }
      else if(0==strcmp("tf", param)){
        // set vel tf     [s]
        motor.LPF_velocity.Tf = value;
      }
      else if(0==strcmp("ra", param)){
        // set vel ramp,  [V/s]
        motor.PID_velocity.output_ramp = value;
      }
      else if(0==strcmp("lm", param)){
        // set vel limit [rad/s]?
        motor.PID_velocity.limit = value;
      }
      else{
        //error
        response->printf("Invalid param [%s]\r\n", param);
      }
    }
    else if (0==strcmp("ud", type)){
      if(0==strcmp("kp", param)){
        // set ud kp
        motor.PID_current_d.P = value;
      }
      else if(0==strcmp("ki", param)){
        // set ud ki
        motor.PID_current_d.I = value;
      }
      else if(0==strcmp("kd", param)){
        // set ud kd
        motor.PID_current_d.D = value;
      }
      else if(0==strcmp("ra", param)){
        // set ud ra
        motor.PID_current_d.output_ramp = value;
      }
      else if(0==strcmp("lm", param)){
        // set ud limit
        motor.PID_current_d.limit = value;
      }
      else if(0==strcmp("tf", param)){
        motor.LPF_current_d.Tf = value;
      }
      else{
        //error
        response->printf("Invalid param [%s]\r\n", param);
      }
    }
    else if (0==strcmp("uq", type)){
      if(0==strcmp("kp", param)){
        // set uq kp
        motor.PID_current_q.P = value;
      }
      else if(0==strcmp("ki", param)){
        // set uq ki
        motor.PID_current_q.I = value;
      }
      else if(0==strcmp("kd", param)){
        // set uq kd
        motor.PID_current_q.D = value;
      }
      else if(0==strcmp("ra", param)){
        // set uq ra
        motor.PID_current_q.output_ramp = value;
      }
      else if(0==strcmp("lm", param)){
        // set uq lm
        motor.PID_current_q.limit = value;
      }
      else if(0==strcmp("tf", param)){
        motor.LPF_current_q.Tf = value;
      }
      else{
        //error
        response->printf("Invalid param [%s]\r\n", param);
      }
    }
    else{
      // Error
      response->printf("Invalid type [%s]\r\n", type);
    }
  }
  else{
    response->printf("err args");
  }

}// pid_shell_func









