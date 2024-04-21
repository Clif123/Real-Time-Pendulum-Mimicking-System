/***********************************************************************
* FILENAME :        FastPID.h
*
* DESCRIPTION :
*      A fast, integer based PID controller.
*
* AUTHOR :    @mike-matera
*
* DATE :    March 2017
*
* LINK	: https://github.com/mike-matera/FastPID
***********************************************************************/

#ifndef FastPID_H
#define FastPID_H

#include <stdint.h>

#define bool	_Bool
#define true	1
#define false	0

#define INTEG_MAX    (INT32_MAX)
#define INTEG_MIN    (INT32_MIN)
#define DERIV_MAX    (INT16_MAX)
#define DERIV_MIN    (INT16_MIN)

#define PARAM_SHIFT  8
#define PARAM_BITS   16
#define PARAM_MAX    (((0x1ULL << PARAM_BITS)-1) >> PARAM_SHIFT)
#define PARAM_MULT   (((0x1ULL << PARAM_BITS)) >> (PARAM_BITS - PARAM_SHIFT))

// Configuration
uint32_t _p, _i, _d;
int64_t _outmax, _outmin;
bool _cfg_err;

// State
int16_t _last_sp, _last_out;
int64_t _sum;
int32_t _last_err;

bool configureFastPID(float kp, float ki, float kd, float hz, int bits, bool sign);
bool setCoefficients(float kp, float ki, float kd, float hz);
bool setOutputConfig(int bits, bool sign);
void clear();

int16_t step(int16_t sp, int16_t fb);
void InitializeFastPID(float kp, float ki, float kd, float hz, int bits, bool sign);
bool err();
uint32_t floatToParam(float);
void setCfgErr();

#endif