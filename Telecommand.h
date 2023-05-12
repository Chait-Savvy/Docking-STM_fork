/*****************************************************************
Telecommand.h

Original Created by: Atheel Redah @ University of W�rzburg
Original Creation Date: March 8, 2015

Development environment specifics:
	Software Platform: Rodos (Realtime Onboard Dependable Operating System).
	Hardware Platform: STM32F4 + W�rzburg Uni Informatik 8 Discovery AddOn Board Version 2.0.
*****************************************************************/


#ifndef __Telecommand_h__
#define __Telecommand_h_


#define MaxLength  12

#define TelecommandStart                               '$'
#define TelecommandStop                                '#'
#define TelecommandAck                                 '&'

#define TelemetryID                                    'T'
#define SystemModeID                                   'M'
#define AHRSModeID                                     'A'
#define VelocityID                                     'V'
#define PositionID                                     'P'
#define MotorSpeedID                                   'S'
#define ControllerID                                   'C'
#define ControllerParameterID                          'p'
#define ControllerParameterGainID                      'g'


#define HBridgeUnit1EN                               	'H'
#define Unit2EN                               			'h'
#define DeployCMD                               	    'D'
#define HBridge1PWM                               	    'i'
#define HBridge2PWM                               	    'j'
#define HBridge3PWM                               	    'k'
#define HBridge4PWM                               	    'l'
#define TakeImage                               	    'c'

//#define HBridge2PWM                               	    'HB2PWM'
//#define HBridge3PWM                               	    'HB3PWM'
//#define HBridge4PWM                               	    'HB4PWM'
//#define HBridgeDirection                                'HBDR'
/* Includes ------------------------------------------------------------------*/
#include "rodos.h"
#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "math.h"

/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint8_t Decode(uint8_t RxBuffer);
uint8_t Decode2(uint8_t RxBuffer);
uint8_t Command(uint8_t TelecommandID);

#endif /* Telecommand_H_ */
