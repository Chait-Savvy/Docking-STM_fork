#ifndef _SATELLITE_CONFIG_H_
#define _SATELLITE_CONFIG_H_

// Global variable declarations
extern float desired_current[4];

#ifdef BLACK_SAT
#include "config_black.h"
#endif

#ifdef WHITE_SAT
#include "config_white.h"
#endif

#endif // satellite_config.h
