// File: sparkfun_iot_brushless_driver.hpp
// Schematic files located here
//  https://github.com/sparkfun/SparkFun_IoT_Brushless_Motor_Driver/blob/main/docs/assets/board_files/schematic.pdf


#pragma once

#include <stdint.h>

// PIN35-GPIO1-TX0 => CH340 USB to TTL Serial IC Pin 2 TXO
// PIN34-GPIO3-RX0 => CH340 USB to TTL Serial IC Pin 3 RXI

// These 5 pins are used for bootloading / auto power control
//  ESP32_PIN25-GPIO0-QPWR Power control for the boad
static const uint8_t QPWR = 0;
//  ESP32_PIN24-GPIO2-RGB_DI into WS2812 RGB LED
static const uint8_t WS2812_RGB = 2;
//  ESP32_PIN29-GPIO5-nSTDBY Logic IO Voltage supply to the TMC6300 3 phase driver IC
static const uint8_t nSTDBY = 5;
//  ESP32_PIN14-GPIO12-PCB general purpose IO 1
//  ESP32_PIN23-GPIO14-PCB general purpose IO 4
//
// ESP32_PIN26-GPIO4-nINT_MAG IRQ from the TMAG5273 magnetic sensor IC
// ESP32_PIN16-GPIO13-AUX2 general purpose IO to S4 push button (GND when pressed)
// ESP32_PIN13-GPIO14-AUX1 general purpose IO to S3 push button (GND when pressed)

// ESP32_PIN27-GPIO16-UH phase control
static const uint8_t UH_PHASE = 16;

// ESP32_PIN28-GPIO17-UL phase control
static const uint8_t UL_PHASE = 17;

// ESP32_PIN30-GPIO18-VH phase control
static const uint8_t VH_PHASE = 18;

// ESP32_PIN31-GPIO19-WH phase control
static const uint8_t WH_PHASE = 19;

// ESP32_PIN33-GPIO21-SDA to TMAG and QWIIC connector
// ESP32_PIN36-GPIO22-SCL to TMAG and QWIIC connector

// ESP32_PIN37-GPIO23-VL phase control
static const uint8_t VL_PHASE = 23;

// ESP32_PIN10-GPIO25-Analog In 1
// ESP32_PIN11-GPIO26-Analog In 0
// ESP32_PIN12-GPIO27-PCB general purpose IO 2
// ESP32_PIN8-GPIO32-CURR_SENSE out from amplifier
static const uint8_t CURR_SENSE = 32;
// ESP32_PIN9-GPIO33-WL phase control
static const uint8_t WL_PHASE = 33;
// ESP32_PIN6-GPIO-DIAG output from TMC6300 3 phase bridge
static const uint8_t DIAG = 34;
// ESP32_PIN7-U_OUT U phase sense output
// ESP32_PIN4-V_OUT V phase sense output
// ESP32_PIN5-W_OUT W phase sense output


static const float DRV_VOLT_SUPP = 3.3;             // [V]
static const long DRV_PWM_FREQ = 20000;             // [Hz]
static const float DRV_VOLT_LIMIT = 3.3;  // [V]

static const uint8_t    MTR_POLES = 8;         // [n]
static const float MTR_VOLT_LIMIT = 3.3;            // [V]
static const float MTR_VEL_LIMIT = 6.28;            // [rad/s]

//static const int 

// endfile