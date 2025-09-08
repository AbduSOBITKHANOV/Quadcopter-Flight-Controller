#define ROLL_RATE_KP        20.0f
#define ROLL_RATE_KI         1.8f
#define ROLL_RATE_KD         1.1f

#define PITCH_RATE_KP       20.0f
#define PITCH_RATE_KI        1.7f
#define PITCH_RATE_KD        1.0f

#define YAW_RATE_KP         10.0f
#define YAW_RATE_KI          0.0f
#define YAW_RATE_KD          0.0f

#define ROLL_ANGULAR_GAIN    5.0f
#define PITCH_ANGULAR_GAIN   5.0f

float Roll_Rate_Error = 0.0f;

float Pitch_Rate_Error = 0.0f;

float Yaw_Rate_Error = 0.0f;

float Roll_Angle_Error  = 0.0f;
float Pitch_Angle_Error = 0.0f;

float AngularGain[2] = { ROLL_ANGULAR_GAIN, PITCH_ANGULAR_GAIN };

void setup_pid_controllers(void) {
  // ========================== Roll rates Controllers ==========================
  /* PID Gains */
  pidController[0].Kp = ROLL_RATE_KP;
  pidController[0].Ki = ROLL_RATE_KI;
  pidController[0].Kd = ROLL_RATE_KD;

  /* PID's Limits*/
  pidController[0].limMin = -200.0f;
  pidController[0].limMax =  200.0f;

  /* Integrator's Limits */
  pidController[0].limMinInt = -200.0f;
  pidController[0].limMaxInt =  200.0f;
  
  PID_Init(&pidController[0], SAMPLING_PERIOD);
  // ============================================================================

  // ========================== Pitch rates Controllers ==========================
  /* PID Gains */
  pidController[1].Kp = PITCH_RATE_KP;
  pidController[1].Ki = PITCH_RATE_KI;
  pidController[1].Kd = PITCH_RATE_KD;

  /* PID's Limits*/
  pidController[1].limMin = -200.0f;
  pidController[1].limMax =  200.0f;

  /* Integrator's Limits */
  pidController[1].limMinInt = -200.0f;
  pidController[1].limMaxInt =  200.0f;
  
  PID_Init(&pidController[1], SAMPLING_PERIOD);
  // ============================================================================

  // ========================== Yaw rates Controllers ==========================
  /* PID Gains */
  pidController[3].Kp = YAW_RATE_KP;
  pidController[3].Ki = YAW_RATE_KI;
  pidController[3].Kd = YAW_RATE_KD;

  /* PID's Limits*/
  pidController[3].limMin = -200.0f;
  pidController[3].limMax =  200.0f;

  /* Integrator's Limits */
  pidController[3].limMinInt = -200.0f;
  pidController[3].limMaxInt =  200.0f;
  
  PID_Init(&pidController[3], SAMPLING_PERIOD);
  // ============================================================================
}

void PID_Init(PIDController *pid, float dt) {

  /* Set the Sampling Period */
  pid->T = dt;

  /* Clear controller variables */
  pid->integrator = 0.0f;
	pid->prevError  = 0.0f;

	pid->differentiator = 0.0f;

  pid->out = 0.0f;
}

float PID_Update(PIDController *pid, float error) {

  /* Proportional */
  float proportional = pid->Kp * error;

  /* Integral */
  pid->integrator += pid->Ki * (error + pid->prevError) * pid->T / 2.0f;

  /* Anti-wind-up via integrator clamping */
  if      (pid->integrator > pid->limMaxInt) pid->integrator = pid->limMaxInt;
  else if (pid->integrator < pid->limMinInt) pid->integrator = pid->limMinInt;

  /* Derivative */
  pid->differentiator = pid->Kd * (error - pid->prevError) / pid->T;

  /* Compute output and apply limits */
  pid->out = proportional + pid->integrator + pid->differentiator;

  if      (pid->out > pid->limMax) pid->out = pid->limMax;
  else if (pid->out < pid->limMin) pid->out = pid->limMin;

  /* Store error for later use */
  pid->prevError = error;

  /* Return Controller Output */
  return pid->out;
}
