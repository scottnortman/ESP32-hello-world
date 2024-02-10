///////////////////////////////////////////////////////////////////////////////
//  File:   shell_func.cpp
//  Desc:   Contains shell functions for the command line interface
//  Auth:   scott.nortman@gmail.com
//  Date:   Feb 2024
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//  Includes
///////////////////////////////////////////////////////////////////////////////
#include "shell_func.hpp"

///////////////////////////////////////////////////////////////////////////////
//  File Variables
///////////////////////////////////////////////////////////////////////////////
static tgt_wave_struct tgt_wave;

///////////////////////////////////////////////////////////////////////////////
//  Function Implementations
///////////////////////////////////////////////////////////////////////////////

void tgt_set_shell_func(char *args, Stream *response){

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
    switch(tgt_wave.shape){
      case TGT_WAVE_CONST:
        // Show single target value
        response->printf("con a[%f]\r\n", tgt_wave.ampl);
        break;
      case TGT_WAVE_SIN:
        // show sin wave values
        response->printf("sin a[%f] f[%f] o[%f]\r\n", 
            tgt_wave.ampl, tgt_wave.freq, tgt_wave.ofst);
        break;
      case TGT_WAVE_SQW:
        // show square wave values
        response->printf("sqw a[%f] f[%f] o[%f]\r\n", 
            tgt_wave.ampl, tgt_wave.freq, tgt_wave.ofst);
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
      tgt_wave.shape = TGT_WAVE_CONST;
      tgt_wave.tstart_us = micros();
    }
    else if(0==strcmp("sin", wave_str)){
      // sine wave
      tgt_wave.shape = TGT_WAVE_SIN;
      tgt_wave.tstart_us = micros();
    }
    else if(0==strcmp("sqw", wave_str)){
      // square wave
      tgt_wave.shape = TGT_WAVE_SQW;
      tgt_wave.tstart_us = micros();
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
      tgt_wave.shape = TGT_WAVE_CONST;
      tgt_wave.tstart_us = micros();
      tgt_wave.ampl = ampl;
    }
    else if(0==strcmp("sin", wave_str)){
      // sine wave
      tgt_wave.shape = TGT_WAVE_SIN;
      tgt_wave.tstart_us = micros();
      tgt_wave.ampl = ampl;
    }
    else if(0==strcmp("sqw", wave_str)){
      // square wave
      tgt_wave.shape = TGT_WAVE_SQW;
      tgt_wave.tstart_us = micros();
      tgt_wave.ampl = ampl;
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
      tgt_wave.shape = TGT_WAVE_CONST;
      tgt_wave.tstart_us = micros();
      tgt_wave.ampl = ampl;
      tgt_wave.freq = freq;
    }
    else if(0==strcmp("sin", wave_str)){
      // sine wave
      tgt_wave.shape = TGT_WAVE_SIN;
      tgt_wave.tstart_us = micros();
      tgt_wave.ampl = ampl;
      tgt_wave.freq = freq;
    }
    else if(0==strcmp("sqw", wave_str)){
      // square wave
      tgt_wave.shape = TGT_WAVE_SQW;
      tgt_wave.tstart_us = micros();
      tgt_wave.ampl = ampl;
      tgt_wave.freq = freq;
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
      tgt_wave.shape = TGT_WAVE_CONST;
      tgt_wave.tstart_us = micros();
      tgt_wave.ampl = ampl;
      tgt_wave.freq = freq;
      tgt_wave.ofst = ofst;
    }
    else if(0==strcmp("sin", wave_str)){
      // sine wave
      tgt_wave.shape = TGT_WAVE_SIN;
      tgt_wave.tstart_us = micros();
      tgt_wave.ampl = ampl;
      tgt_wave.freq = freq;
      tgt_wave.ofst = ofst;
    }
    else if(0==strcmp("sqw", wave_str)){
      // square wave
      tgt_wave.shape = TGT_WAVE_SQW;
      tgt_wave.tstart_us = micros();
      tgt_wave.ampl = ampl;
      tgt_wave.freq = freq;
      tgt_wave.ofst = ofst;
    }
    else{
      //error
      response->printf("Invalid arg [%s %f %f %f]\r\n", wave_str, ampl, freq, ofst);
    }
  }
  else{
      response->printf("Incorrect args: [%s]\r\n", args);
  }
    
}// tgt_shell_func

float tgt_run_func(void){

  if(tgt_wave.freq < 1e-3){
    // Freq too small, set to static
    tgt_wave.shape = TGT_WAVE_CONST;
  }
    
  switch(tgt_wave.shape){

    case TGT_WAVE_CONST:
    {
      // Constant value, return the amplitude
      return tgt_wave.ampl;
      break;
    }
    case TGT_WAVE_SIN:
    {
      // sine wave; we need to see where in the cycle we are
      uint64_t tgt_tdiff_us = micros() - tgt_wave.tstart_us; //TODO handle rollover
      // get period in us
      uint64_t tgt_period_us = ROUND(1e6 / tgt_wave.freq);
      uint64_t tgt_modulo_us = tgt_tdiff_us % tgt_period_us;
      //Given the remainder, we can calulate where we are in the cycle
      float tgt_cycle_scale = (float)tgt_modulo_us / (float)tgt_period_us;
      float val = 2.0 * PI * tgt_cycle_scale;
      float ampl = tgt_wave.ampl * sin(val) + tgt_wave.ofst;
      return (float)ampl;
      break;
    }
    case TGT_WAVE_SQW:
    {
      // 50% DC square wave; we need to see where in the cycle we are
      uint64_t tgt_tdiff_us = micros() - tgt_wave.tstart_us; //TODO handle rollover
      // get period in us
      uint64_t tgt_period_us = (uint64_t)(1e6 / tgt_wave.freq);
      uint64_t tgt_modulo_us = tgt_tdiff_us % tgt_period_us;
      float tgt_cycle_scale = (float)tgt_modulo_us / (float)tgt_period_us;
      if( tgt_cycle_scale < 0.5 ){
        // First half of the cycle; output high value
        float ampl = tgt_wave.ampl + tgt_wave.ofst;
        return (float)ampl;
      }
      else{
        float ampl = tgt_wave.ofst;
        return (float)ampl;
      }
      break;
    }
    default:
      break;
  }

}//tgt_run_func

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

plt_shell_func(char *args, Stream *response){

}//plt_shell_func

