/* ===================================================== */
/* =================== HEADER FILES ==================== */
#include <ESP32Servo.h>
#include <DFRobot_BMI160.h>                              // This lib includes Wire.h
#include <PPMReader.h>                                   // This file's content has been changed
#include <math.h>
/* ===================================================== */


typedef struct {

  /* Controller gains */
  float Kp;
  float Ki;
  float Kd;

  /* Controller Variables */
  float integrator;
  float prevError;  // Required for integrator
  float differentiator;

  /* Sample time (in seconds) */
	float T;

  /* Output limits */
	float limMin;
	float limMax;

  /* Integrator limits */
	float limMinInt;
	float limMaxInt;
  
  /* Controller output */
	float out;

} PIDController;

#define USE_WIFI // if you don't want to use wifi, just comment out this macro

/* =========================================================== */
/* ========================= MACROS ========================== */
#define LED_PIN                       38                       // ESP32S3 Built-In LED
#define TURN_ON_YELLOW_LED            neopixelWrite(LED_PIN, 50, 50,  0)
#define TURN_ON_RED_LED               neopixelWrite(LED_PIN, 50,  0,  0)
#define TURN_ON_GREEN_LED             neopixelWrite(LED_PIN,  0, 50,  0)
#define TURN_ON_BLUE_LED              neopixelWrite(LED_PIN,  0,  0, 50)
#define TURN_OFF_LED                  neopixelWrite(LED_PIN,  0,  0,  0)
#define SAMPLING_PERIOD                0.01f                   // Sampling period of the sensor in seconds
#define MTR_NUMBER                     4                       // Number of motors
#define RIGHT_FRONT_MOTOR             21                       // The pin that receives PWM signal from the front right ESC
#define RIGHT_BACK_MOTOR              14                       // The pin that receives PWM signal from the back right ESC
#define LEFT_BACK_MOTOR                4                       // The pin that receives PWM signal from the back left ESC
#define LEFT_FRONT_MOTOR               2                       // The pin that receives PWM signal from the front left ESC
#define GYRO_CALIBRATION_SAMPLES    2000                       // It takes this number of samples to calibrate the gyroscope
#define PPM_PIN                       47                       // The reciever pin of the flight controller
#define CHANNEL_AMOUNT                 6                       // The number of RC channels
#define DEFAULT_RC_R_P_Y            1500                       // Default {Roll, Pitch, Yaw} values when the signal is lost
#define DEFAULT_RC_SWITCH           1000                       // Default {SWITCH1, SWITCH2} values when the signal is lost
#define DEFAULT_RC_THROTTLE         1250                       // Default {Thtottle} value when the signal is lost
#define DEGREES_OF_CONTROL             4                       // Degrees of control: { ROLL, PITCH, THROTTLE, YAW }
#define GYRO_ROLL_NOISE_VARIANCE       0.0000002437f           // Gyroscope roll rate measurement noise variance [rad/s]^2
#define GYRO_PITCH_NOISE_VARIANCE      0.0000001828f           // Gyroscope pitch rate measurement noise variance [rad/s]^2
#define ACCEL_ROLL_NOISE_VARIANCE      0.0000003046f           // Accelerometer roll angle estimation noise variance [rad]^2
#define ACCEL_PITCH_NOISE_VARIANCE     0.0000003046f           // Accelerometer pitch angle estimation noise variance [rad]^2
#define DEGREE_LIMIT_30                0.06f                   // Limits the desired angle to [-30°, 30°]
#define RATE_LIMIT_100                 0.2f                    // Limits the desired rotational rate to [-100°/s, 100°/s]
/* =========================================================== */


/* =========================================================== */
/* ================== CLASSES & STRUCTURES =================== */
/* MTR */ Servo motor[MTR_NUMBER];                             // Servo class for BLDC Motors
/* IMU */ DFRobot_BMI160 bmi160;
/* RC  */ PPMDecoder ppm(PPM_PIN, CHANNEL_AMOUNT);             // PPMDecoder class
/* PID */ PIDController pidController[DEGREES_OF_CONTROL];     // PID Controller struct: { ROLL, PITCH, THROTTLE, YAW }
/* =========================================================== */


/* =================================================================================== */
/* ================================ GLOBAL VARIABLES ================================= */
/* MTR */ const int motorPins[MTR_NUMBER] = { RIGHT_FRONT_MOTOR, RIGHT_BACK_MOTOR, LEFT_BACK_MOTOR, LEFT_FRONT_MOTOR }; // The GPIOs connected to the ESCs
/* IMU */ const int BMI160_ADDR = 0x68;                                                // The address of BMI160 sensor
/* IMU */ float AngleRoll = 0.0f;                                                      // Roll Angle [radian] (estimated from the accelerometer)
/* IMU */ float AnglePitch = 0.0f;                                                     // Pitch Angle [radian] (estimated from the accelerometer)
/* IMU */ float accelData[3] = { 0.0f };                                               // The accelerometer's data (acceleration): { X, Y, Z }
/* IMU */ float gyroData[3] = { 0.0f };                                                // The gyroscopic data (rotational speed): { Roll, Pitch, Yaw } [rad/s]
/* IMU */ float gyroCalibration[3] = { 0.0f };                                         // The gyro calibration values: { Roll, Pitch, Yaw }
/* RC  */ int RCValue[CHANNEL_AMOUNT] = { 0 };                                         // Data from the Remote Controller: { Roll, Pitch, Throttle, Yaw, Switch1, Switch2 }
/* SYS */ unsigned long ST = 0;                                                        // Start Time [us]
/* PID */ int Roll_Rate_CS  = 0;                                                       // Control Signal of Roll Rate
/* PID */ int Pitch_Rate_CS = 0;                                                       // Control Signal of Pitch Rate
/* PID */ int Yaw_Rate_CS   = 0;                                                       // Control Signal of Yaw Rate
/* PID */ unsigned int inputThrottle;                                                  // The throttle command received from the RC
/* PID */ unsigned int armSwitch;                                                      // The Arm command received from the RC
/* PID */ int motorInputs[MTR_NUMBER] = { 1000, 1000, 1000, 1000 };                    // PWM duty cycle Inputs to the motors
/* =================================================================================== */


/* =================================================================================== */
/* ============================== FUNCTION DECLARATOINS ============================== */
/*     */ void avoid_sudden_start(void);                                               // Safe start
/* MTR */ void motors_init(void);                                                      // Initialize the motors
/* IMU */ void imu_init(void);                                                         // Initialize the BMI160
/* IMU */ void gyro_signals(void);                                                     // Read gyroscope data from BMI160
/* IMU */ void gyro_calibration(void);                                                 // Calibrate the gyro data
/* IMU */ void accel_signals(void);                                                    // Read accelerometer data from BMI160
/* RC  */ void read_receiver(void);                                                    // Read signals from the RC
/* PID */ void setup_pid_controllers(void);
/* PID */ void PID_Init(PIDController *pid, float dt);
/* PID */ float PID_Update(PIDController *pid, float error);
/* =================================================================================== */