// Tamariw configuration file
// 2024-04-13

#ifndef _CONFIG_BLACK_H_
#define _CONFIG_BLACK_H_

#define EN_CHG_BAT GPIO_038 //PC6, Charging Enable Pin
#define BATT_MES_ADC_CH ADC_CH_014 // ADC3_CH14, PC4, Battery Voltage Monitor
#define ADC_NO_BAT_MES ADC_IDX1 //Using ADC 1 for PC4

// ToF sensor
#define TOF_I2C_HAL_IDX I2C_IDX1
#define TOF_I2C_HAL_GPIO_SCL GPIO_022
#define TOF_I2C_HAL_GPIO_SDA GPIO_023
#define TOF_I2C_ADDRESS 0x29
#define TOF_I2C_MUX_ADDRESS 0x70
#define TOF_CALIBRATE_TARGET_MM 100
#define TOF_CALIBRATE_SAMPLES 20

// ToF dimensions
#define TOF_DIMENSION_WIDTH_MM 44.95
#define TOF_DIMENSION_LENGTH_MM 101.5

// Electromagnet enable pin
#define EM_ENABLE_PIN GPIO_056
#define EM_SAFETY_THRESHOLD 50
#define EM_SAFETY_INTERMEDIATE 20

// PWMs for electromagnets
#define EM_PWM_FREQUENCY 15000
#define EM_PWM_INCREMENTS 5000
#define EM0_PWM_IDX PWM_IDX06
#define EM1_PWM_IDX PWM_IDX07
#define EM2_PWM_IDX PWM_IDX05
#define EM3_PWM_IDX PWM_IDX04

// Direction pins for electromagnets
#define EM0_PIN_IN2 GPIO_076
#define EM0_PIN_IN1 GPIO_075
#define EM1_PIN_IN2 GPIO_074
#define EM1_PIN_IN1 GPIO_073
#define EM2_PIN_IN2 GPIO_060
#define EM2_PIN_IN1 GPIO_061
#define EM3_PIN_IN2 GPIO_062
#define EM3_PIN_IN1 GPIO_063

// ADC channels for electromagnets
#define EM_ADC_IDX ADC_IDX3
#define EM0_ADC_CH ADC_CH_012
#define EM1_ADC_CH ADC_CH_013
#define EM2_ADC_CH ADC_CH_011
#define EM3_ADC_CH ADC_CH_010

// Periods of threads
#define PERIOD_TOF_MILLIS 60
#define PERIOD_CONTROL_MILLIS 50

// Control params
#define PID_DISTANCE_UMAX 2500*2500
#define PID_DISTANCE_UMIN 0
#define PID_DISTANCE_KP 3000
#define PID_DISTANCE_KI 80

#define PID_VELOCITY_UMAX 2500
#define PID_VELOCITY_UMIN 0
#define PID_VELOCITY_KP 10.0
#define PID_VELOCITY_KI 0.0

#endif // config_black.h
