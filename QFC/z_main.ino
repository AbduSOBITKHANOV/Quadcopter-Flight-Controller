/*
  This folder is named "z_main" due to the way Arduino processes and builds sketches.  
  Refer to the official documentation: https://docs.arduino.cc/arduino-cli/sketch-build-process/
*/

// Set the offset to 0 and put the quadcopter on flat surface, then read the offset data, then write it here
#define ROLL_OFFSET    0.0213  // roll  angle error [rad] due to oblique mounting
#define PITCH_OFFSET   0.0637  // pitch angle error [rad] due to oblique mounting

void setup() {
  Serial.begin(115200);
  
  TURN_ON_GREEN_LED;

  motors_init(); // Initialize the motors (Set freq, Attach to pins, Write 0, Wait 3s)

  ppm.begin(); // Start reading ppm signals from the RC

  read_receiver();
  while (RCValue[5] <= 1500) {
    read_receiver();
    TURN_OFF_LED;
    delay(10);
  }
  
  delay(2000); TURN_ON_YELLOW_LED;

  #ifdef USE_WIFI
    wifi_setup(); // Initialize the WI-FI
  #endif

  Wire.setClock(400000); // Set the clock speed of I2C
  Wire.begin();
  delay(250);
  
  imu_init(); // Initialize the IMU on I2C

  gyro_calibration(); // Perform gyroscope calibration (Keep the IMU completely still during this process!)

  setup_pid_controllers(); // Initialize the PID controllers

  avoid_sudden_start(); // Avoid accidental lift off

  #ifdef USE_WIFI
    lastEventTime = millis();
  #endif
}

void loop() {
  // Dynamic Period
  ST = micros();

  // Receive data from the RC
  read_receiver(); // The values are stored in RCValue

  // ================================================================================================================== //
  // ============================================ FEEDBACK (MEASUREMENTS) ============================================= //

  // Receive data from the Gyroscope
  gyro_signals(); // The values are stored in gyroData

  // Calibrate the gyroData
  for (uint8_t i = 0; i < 3; i++) gyroData[i] -= gyroCalibration[i];

  // Receive data from the Accelerometer and compute Roll & Pitch angles
  accel_signals();
  
  // Compute angular rates in Euler coordinates
    float phi   = AngleRoll_Kalman;  // Roll Angle [rad]
    float theta = AnglePitch_Kalman; // Pitch Angle [rad]

    float phi_dot   = gyroData[0] + gyroData[1] * sin(phi) * tan(theta) + gyroData[2] * cos(phi) * tan(theta); // Roll Rate [rad/s]
    float theta_dot =               gyroData[1] * cos(phi)              - gyroData[2] * sin(phi);              // Pitch Rate [rad/s]
  //

  // Calculate Roll Angle with Kalman Filter
  Kalman_Filter_1D(
    &AngleRoll_Kalman,
    &AngleRoll_KalmanUncertainty,
    phi_dot,
    GYRO_ROLL_NOISE_VARIANCE,
    AngleRoll,
    ACCEL_ROLL_NOISE_VARIANCE
  );

  // Calculate Pitch Angle with Kalman Filter
  Kalman_Filter_1D(
    &AnglePitch_Kalman,
    &AnglePitch_KalmanUncertainty,
    theta_dot,
    GYRO_PITCH_NOISE_VARIANCE,
    AnglePitch,
    ACCEL_PITCH_NOISE_VARIANCE
  );

  // ================================================================================================================== //

  // ================================================================================================================== //
  // ================================================= PID CONTROLLER ================================================= //
  
  // Update the PID & Angle Gains
    // Roll Rate PID
    pidController[0].Kp = Tuned_Kp[0];
    pidController[0].Ki = Tuned_Ki[0];
    pidController[0].Kd = Tuned_Kd[0];

    // Pitch Rate PID
    pidController[1].Kp = Tuned_Kp[1];
    pidController[1].Ki = Tuned_Ki[1];
    pidController[1].Kd = Tuned_Kd[1];

    // Yaw Rate PID
    pidController[3].Kp = Tuned_Kp[2];
    pidController[3].Ki = Tuned_Ki[2];
    pidController[3].Kd = Tuned_Kd[2];

    // Angular Gains: Roll then Pitch
    AngularGain[0] = Tuned_AngularGain[0];
    AngularGain[1] = Tuned_AngularGain[1];
  //

  // Angular PID
    Roll_Angle_Error  = DEGREE_LIMIT_30 * (RCValue[0] - 1500) * DEG_TO_RAD - (AngleRoll_Kalman + ROLL_OFFSET);
    Pitch_Angle_Error = DEGREE_LIMIT_30 * (RCValue[1] - 1500) * DEG_TO_RAD - (AnglePitch_Kalman + PITCH_OFFSET);
  //

  // Calculate the rotational rate errors
    Roll_Rate_Error  = AngularGain[0] * Roll_Angle_Error  - gyroData[0];
    Pitch_Rate_Error = AngularGain[1] * Pitch_Angle_Error - gyroData[1];

    Yaw_Rate_Error   = RATE_LIMIT_100 * (RCValue[3] - 1500) * DEG_TO_RAD - gyroData[2];
  //
  
  // Update Rate PID controllers
    Roll_Rate_CS  = (int)PID_Update(&pidController[0], Roll_Rate_Error); 
    Pitch_Rate_CS = (int)PID_Update(&pidController[1], Pitch_Rate_Error);
    Yaw_Rate_CS   = (int)PID_Update(&pidController[3], Yaw_Rate_Error);
  //

  // ================================================================================================================== //

  // Limit the throttle to make room for rate corrections
  if (inputThrottle > 1800) inputThrottle = 1800;

  // Motor Mixing Algorithm
    motorInputs[0] = inputThrottle - Roll_Rate_CS - Pitch_Rate_CS - Yaw_Rate_CS;
    motorInputs[1] = inputThrottle - Roll_Rate_CS + Pitch_Rate_CS + Yaw_Rate_CS;
    motorInputs[2] = inputThrottle + Roll_Rate_CS + Pitch_Rate_CS - Yaw_Rate_CS;
    motorInputs[3] = inputThrottle + Roll_Rate_CS - Pitch_Rate_CS + Yaw_Rate_CS;
  //

  // Saturate the motor inputs
  for (uint8_t i = 0; i < MTR_NUMBER; i++) {
    motorInputs[i] = constrain(motorInputs[i], 1180, 1999);
  }

  // Actuate the motors
  for (uint8_t i = 0; i < MTR_NUMBER; i++) {

    if (armSwitch < 1500 || inputThrottle < 1050) {
      TURN_ON_BLUE_LED;
      motorInputs[i] = 1000;
    } else {
      TURN_ON_GREEN_LED;
    }
    
    motor[i].write(motorInputs[i]); // Ranges between [1000, 2000]

  }

  // Reset the PID if the motors are turned off
  if (armSwitch < 1500) {
    for (uint8_t i = 0; i < DEGREES_OF_CONTROL; i++) {
      pidController[i].integrator = 0.0f;
    }
    Roll_Rate_Error  = 0.0f;
    Pitch_Rate_Error = 0.0f;
    Yaw_Rate_Error   = 0.0f;
  }

  // ============================================ //
  // =============== Serial Print =============== //
  

  Serial.print(armSwitch); Serial.print("\t");
  // Yaw Rate PID
  Serial.print(pidController[3].Kp); Serial.print("\t");
  Serial.print(pidController[3].Ki); Serial.print("\t");
  Serial.print(pidController[3].Kd); Serial.print("\t");

  // Angular Gains: Roll then Pitch
  Serial.print(AngularGain[0]); Serial.print("\t");
  Serial.print(AngularGain[1]); Serial.print("\t");

  Serial.print((AngleRoll_Kalman + ROLL_OFFSET) * RAD_TO_DEG); Serial.print("\t");
  Serial.print((AnglePitch_Kalman + PITCH_OFFSET) * RAD_TO_DEG); Serial.print("\t");

  Serial.println();
  
  #ifdef USE_WIFI
    // WI-FI loop (send Roll & Pitch angles, and the motor inputs)
    wifi_loop((AngleRoll_Kalman + ROLL_OFFSET), (AnglePitch_Kalman + PITCH_OFFSET));
  #endif

  // Wait until the period finishes
  while ((micros() - ST) / 1000000.0 <= SAMPLING_PERIOD);
}
