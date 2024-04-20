#include "PID.h"

PID::PID(float T, float Kp, float Ki, float Kd)
{
	pid.integrator = 0.0f;
	pid.prevError = 0.0f;

	pid.differentiator = 0.0f;
	pid.prevMeasurement = 0.0f;

	pid.out = 0.0f;

	pid.Kd = Kd;
	pid.Ki = Ki;
	pid.Kp = Kp;
	pid.T = T;
}

PID::PID(const PIDController& pidInit)
{
	pid = pidInit;
}

float PID::update(float setpoint, float measurement)
{
	/*
	* Error signal
	*/
	float error = setpoint - measurement;


	/*
	* Proportional
	*/
	float proportional = pid.Kp * error;


	/*
	* Integral
	*/
	pid.integrator = pid.integrator + 0.5f * pid.Ki * pid.T * (error + pid.prevError);

	/* Anti-wind-up via integrator clamping */
	if (pid.integrator > pid.limMaxInt)
	{

		pid.integrator = pid.limMaxInt;

	} else if (pid.integrator < pid.limMinInt)
	{

		pid.integrator = pid.limMinInt;

	}


	/*
	* Derivative (band-limited differentiator)
	*/
	if (0 == pid.Kd)
		pid.differentiator = 0;
	else
	{
		pid.differentiator = -(2.0f * pid.Kd * (measurement - pid.prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
							   + (2.0f * pid.tau - pid.T) * pid.differentiator)
			/ (2.0f * pid.tau + pid.T);
	}

	/*
	* Compute output and apply limits
	*/
	pid.out = proportional + pid.integrator + pid.differentiator;

	if (pid.out > pid.limMax)
	{

		pid.out = pid.limMax;

	} else if (pid.out < pid.limMin)
	{

		pid.out = pid.limMin;

	}

	/* Store error and measurement for later use */
	pid.prevError = error;
	pid.prevMeasurement = measurement;

	/* Return controller output */
	return error;
}

void PID::updateConfig(const PIDController& ctrl)
{
	pid = ctrl;
}
