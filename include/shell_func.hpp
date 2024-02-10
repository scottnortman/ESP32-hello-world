///////////////////////////////////////////////////////////////////////////////
//  File:   shell_func.hpp
//  Desc:   Contains shell functions for the command line interface
//  Auth:   scott.nortman@gmail.com
//  Date:   Jan 2024
///////////////////////////////////////////////////////////////////////////////
#pragma once

///////////////////////////////////////////////////////////////////////////////
//  Includes
///////////////////////////////////////////////////////////////////////////////
#include <Arduino.h>

///////////////////////////////////////////////////////////////////////////////
// Macros
///////////////////////////////////////////////////////////////////////////////
//https://jkorpela.fi/round.html
#define ROUND(x) ((x)>=0?(uint64_t)((x)+0.5):(uint64_t)((x)-0.5))

///////////////////////////////////////////////////////////////////////////////
//  Types
///////////////////////////////////////////////////////////////////////////////
typedef enum
{
  TGT_WAVE_CONST = 0,
  TGT_WAVE_SIN = 1,
  TGT_WAVE_SQW = 2,
} TGT_WAVE_TYPE;

typedef struct{
    static TGT_WAVE_TYPE shape;
    static float ampl;
    static float ofst;  
    static float freq;          // [Hz]
    static uint64_t tstart_us;  // [us]
} tgt_wave_struct;


///////////////////////////////////////////////////////////////////////////////
//  Function Prototypes
///////////////////////////////////////////////////////////////////////////////

//  Func:   set_tgt_shell_func
//  Args:   char *args
//          Stream *response
//  Ret:    void
//  Note:   This function is passed to the shell commander to permit the below
//          shell commands.
//  The following arguments are supported:
//      $ tgt                           ; Query current target
//      $ tgt con <value>               ; Set constant value
//      $ tgt sin <ampl> <freq> <ofst>  ; Set sine wave all params
//      $ tgt sin <ampl>                ; Set sine amplitude only
//      $ tgt sin <ampl> <freq>         ; Set sine ampliture and freq only
//      $ tgt sqw <ampl> <freq> <ofst>  ; Set square wave all param
//      $ tgt sqw <ampl>                ; Set square wave amplitude only
//      $ tgt sqw <ampl> <freq>         ; Set square wave amplitude and freq only
//
//  The related function 'tgt_run_func' has to be repeatedly called to generate the
//  proper target value.  
void tgt_set_shell_func(char *args, Stream *response);

//  Func:   run_tgt_func
//  Args:   void
//  Ret:    float, target value
//  Note:   This function works in conjunction with 'set_tgt_func' and uses shared
//          variables.
//          This function must be called repeatedly in a loop or task to generate
//          the correct periodic output based on the state defined by a prior call
//          to 'tgt_set_shell_func' which is called from the shell commander.
float tgt_run_func(void);

//  Func:   pid_shell_func
//  Args:   char *args
//          Stream *response
//  Ret:    void
//  Note:   This function is passed to the shell commander to permit the below
//          shell commands:
//  The following arguments are supported:
//      $ pid <pos|vel|ud|uq> <kp|ki|kd|tf|ra|pl|vl|lm> [<float>]
//  where [<float>] is an optional argument; when omitted, parameter value is returned.
//  Note the parameters:
//      pos =>  Position controller
//      vel =>  Velocity controller
//      ud =>   FOC direct axis current controller
//      uq =>   FOC quarature axis current controller
//      kp =>   Proportional gain
//      ki =>   Integral gain
//      kd =>   Derivative gain
//      tf =>   Low pass filter sample period
//      ra =>   Controller output ramp
//      pl =>   Controller position limit (Position contoller only)
//      vl =>   Velocity limit output (position controller only)
//      lm =>   Velocity limit (velocity contoller only)
//  Examples
//      $ pid pos kp 10.1   ; Set position controller proportional gain to 10.1
//      $ pid pos kp        ; Query position proportional gain
//      $ pid ud ki 500     ; Set integral gain for current direct axis
//      $ pid uq tf         ; Query the low pass filter time constant for the quadrature axis
void pid_shell_func(char *args, Stream *response);

//  Func:   rgb_shell_func
//  Args:   char *args
//          Stream *response
//  Ret:    void
//  Note:   This function is passed to the shell commander to permit the below
//          shell commands:
//      $ rgb <red> <grn> <blu> <pwr>   ; set r g b and power level
//      $ rgb                           ; Query current RGB / power levels
void rgb_func(char *args, Stream *response);

//  Func:   plt_shell_func
//  Args:   char *args
//          Stream *response
//  Ret:    void
//  Note:   This function is passed to the shell commander to permit the below
//          shell commands:
//      tgt => target, actual target value, changes based on controller
//      ffv => actual feed_forward_velocity, rad/s
//      san => actual shaft_angle, rad
//      ean => actual electrical_angle, rad
//      svl => actual shaft_velocity, rad/s
//      csp => current_sp, current set point, amps
//      vsp => shaft_velocity_sp, shaft velocity set point, rad/s
//      ssp => shaft_angle_sp, shaft angle set point, rad
//      vol => actual voltage
//      cur => actual d,q current
//      bev => estimated back emf voltage, if KV provided
//
//  The values latch and can be turned on/off by a subsequent call with the str passed
//
//  Example:
//      $ plt tgt ffv svl ; toggle target, feed forward velocity, and actual shaft velocity
//      $ plt   ; Query what is being plotted
//      
//  The plotted values are compatible with the Teleplot tool (https://teleplot.fr/) which 
//  requires the values to prepend ">" in front of the value, ie ">tgt:%f\r\n"
//
plt_shell_func(char *args, Stream *response);

//  Func:   mtr_shell_func
//  Args:   char *args
//          Stream *response
//  Ret:    void
//  Note:   This function is passed to the shell commander to permit the below shell commands:
//      pp  => pole pairs number
//      R   => motor phase resistance
//      KV  => motor KV rating
//      L   => motor phase inductance
//mtr_shell_func();

//drv_shell_func();
