#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/* Controller parameters */
#define PID_KP  2.0
#define PID_KI  0.5
#define PID_KD  0.25

#define PID_TAU 0.02

#define PID_LIM_MIN -10.0
#define PID_LIM_MAX  10.0

#define PID_LIM_MIN_INT -5.0
#define PID_LIM_MAX_INT  5.0

#define SAMPLE_TIME_S 0.01

/* Maximum run-time of simulation */
#define SIMULATION_TIME_MAX 4.0

typedef struct {
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

void  PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);

#endif
