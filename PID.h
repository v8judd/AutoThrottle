#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct
{

	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMin;
	float limMax;

	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	float integrator;
	float prevError;			/* Required for integrator */
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	float out;

} PIDController;

class PID
{
	PIDController pid;

public:
	explicit PID(float T, float Kp, float Ki, float Kd);
	explicit PID(const PIDController& pid);

	float update(float setpoint, float measurement);
	void updateConfig(const PIDController& ctrl);
	void setTime(float t) { pid.T = t; }
	PIDController& data() { return pid; }
	void setLimits(float lower, float upper) { pid.limMax = upper, pid.limMin = lower; }
	void setMinLimit(float lower) { pid.limMin = lower; }
	void setMaxLimit(float upper) { pid.limMax = upper; }
};

#endif
