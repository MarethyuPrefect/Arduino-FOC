#ifndef ECCENTRICITY_H
#define ECCENTRICITY_H


#include "common/time_utils.h"
#include "common/foc_utils.h"


float V_CAL = 0.15; // What is this??

int NPP = 11; // Number of pole pairs of the actuator
int offset_lut[128]; // Integer lut 