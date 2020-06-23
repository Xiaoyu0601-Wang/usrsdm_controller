/*
  October 2012

  aq32Plus Rev -

  Copyright (c) 2012 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick
  6)UAVX

  Designed to run on the AQ32 Flight Control Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#include "firstOrderFilter.h"

///////////////////////////////////////////////////////////////////////////////

// TAU = Filter Time Constant
// T   = Filter Sample Time

// A   = 2 * TAU / T

// Low Pass:
// GX1 = 1 / (1 + A)
// GX2 = 1 / (1 + A)
// GX3 = (1 - A) / (1 + A)

// High Pass:
// GX1 =  A / (1 + A)
// GX2 = -A / (1 + A)
// GX3 = (1 - A) / (1 + A)
///////////////////////////////////////
#define IMU_SAMPLE_TIME 0.004f
#define PRESSURE_SAMPLE_TIME 0.04f

#define PRESSURE_ALT_LOWPASS_TAU         0.6f
#define PRESSURE_ALT_LOWPASS_SAMPLE_TIME PRESSURE_SAMPLE_TIME
#define PRESSURE_ALT_LOWPASS_A           (2.0f * PRESSURE_ALT_LOWPASS_TAU / PRESSURE_ALT_LOWPASS_SAMPLE_TIME)
#define PRESSURE_ALT_LOWPASS_GX1         (1.0f / (1.0f + PRESSURE_ALT_LOWPASS_A))
#define PRESSURE_ALT_LOWPASS_GX2         (1.0f / (1.0f + PRESSURE_ALT_LOWPASS_A))
#define PRESSURE_ALT_LOWPASS_GX3         ((1.0f - PRESSURE_ALT_LOWPASS_A) / (1.0f + PRESSURE_ALT_LOWPASS_A))
///////////////////////////////////////

#define MCAP_X_LOWPASS_TAU         0.15f
#define MCAP_X_LOWPASS_SAMPLE_TIME 0.01f
#define MCAP_X_LOWPASS_A           (2.0f * MCAP_X_LOWPASS_TAU / MCAP_X_LOWPASS_SAMPLE_TIME)
#define MCAP_X_LOWPASS_GX1         (1.0f / (1.0f + MCAP_X_LOWPASS_A))
#define MCAP_X_LOWPASS_GX2         (1.0f / (1.0f + MCAP_X_LOWPASS_A))
#define MCAP_X_LOWPASS_GX3         ((1.0f - MCAP_X_LOWPASS_A) / (1.0f + MCAP_X_LOWPASS_A))

///////////////////////////////////////

#define MCAP_Y_LOWPASS_TAU         0.05f
#define MCAP_Y_LOWPASS_SAMPLE_TIME 0.01f
#define MCAP_Y_LOWPASS_A           (2.0f * MCAP_Y_LOWPASS_TAU / MCAP_Y_LOWPASS_SAMPLE_TIME)
#define MCAP_Y_LOWPASS_GX1         (1.0f / (1.0f + MCAP_Y_LOWPASS_A))
#define MCAP_Y_LOWPASS_GX2         (1.0f / (1.0f + MCAP_Y_LOWPASS_A))
#define MCAP_Y_LOWPASS_GX3         ((1.0f - MCAP_Y_LOWPASS_A) / (1.0f + MCAP_Y_LOWPASS_A))

///////////////////////////////////////

#define MCAP_Z_LOWPASS_TAU         0.15f
#define MCAP_Z_LOWPASS_SAMPLE_TIME 0.01f
#define MCAP_Z_LOWPASS_A           (2.0f * MCAP_Z_LOWPASS_TAU / MCAP_Z_LOWPASS_SAMPLE_TIME)
#define MCAP_Z_LOWPASS_GX1         (1.0f / (1.0f + MCAP_Z_LOWPASS_A))
#define MCAP_Z_LOWPASS_GX2         (1.0f / (1.0f + MCAP_Z_LOWPASS_A))
#define MCAP_Z_LOWPASS_GX3         ((1.0f - MCAP_Z_LOWPASS_A) / (1.0f + MCAP_Z_LOWPASS_A))

///////////////////////////////////////////////////////////////////////////////
void initMCapFirstOrderFilter(void)
{
	firstOrderFilters[MCAP_X_LOWPASS].gx1 = MCAP_X_LOWPASS_GX1;
	firstOrderFilters[MCAP_X_LOWPASS].gx2 = MCAP_X_LOWPASS_GX2;
	firstOrderFilters[MCAP_X_LOWPASS].gx3 = MCAP_X_LOWPASS_GX3;
	firstOrderFilters[MCAP_X_LOWPASS].previousInput  = 0.0f;
	firstOrderFilters[MCAP_X_LOWPASS].previousOutput = 0.0f;

	firstOrderFilters[MCAP_Y_LOWPASS].gx1 = MCAP_Y_LOWPASS_GX1;
	firstOrderFilters[MCAP_Y_LOWPASS].gx2 = MCAP_Y_LOWPASS_GX2;
	firstOrderFilters[MCAP_Y_LOWPASS].gx3 = MCAP_Y_LOWPASS_GX3;
	firstOrderFilters[MCAP_Y_LOWPASS].previousInput  = 0.0f;
	firstOrderFilters[MCAP_Y_LOWPASS].previousOutput = 0.0f;

	firstOrderFilters[MCAP_Z_LOWPASS].gx1 = MCAP_Z_LOWPASS_GX1;
	firstOrderFilters[MCAP_Z_LOWPASS].gx2 = MCAP_Z_LOWPASS_GX2;
	firstOrderFilters[MCAP_Z_LOWPASS].gx3 = MCAP_Z_LOWPASS_GX3;
	firstOrderFilters[MCAP_Z_LOWPASS].previousInput  = 0.0f;
	firstOrderFilters[MCAP_Z_LOWPASS].previousOutput = 0.0f;
}

///////////////////////////////////////////////////////////////////////////////

float firstOrderFilter(float input, struct firstOrderFilterData *filterParameters)
{
    float output;

    output = filterParameters->gx1 * input +
             filterParameters->gx2 * filterParameters->previousInput -
             filterParameters->gx3 * filterParameters->previousOutput;

    filterParameters->previousInput  = input;
    filterParameters->previousOutput = output;

    return output;
}

///////////////////////////////////////////////////////////////////////////////
