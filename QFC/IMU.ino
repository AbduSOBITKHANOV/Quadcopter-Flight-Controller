void imu_init(void) {
  //init the hardware bmin160  
  if (bmi160.softReset() != BMI160_OK){
    Serial.println("reset false");
    neopixelWrite(LED_PIN, 220, 0, 255); // purple
    while(1);
  }
  
  //set and init the bmi160 i2c address
  if (bmi160.I2cInit(BMI160_ADDR) != BMI160_OK){
    Serial.println("init false");
    neopixelWrite(LED_PIN, 220, 0, 255); // purple
    while(1);
  }
}

void gyro_signals(void) {
  /* { GYROSCOPE } */

  // DIGITAL FILTER: ON (OSR4: Oversampling Rate of 4)
  // ODR (Output Data Rate) = 100 Hz
  // Fc (Cutoff Frequency) ≈ 10.12 Hz
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(0x42);  // GYR_CONFIG register
  Wire.write(0x08);  // 0b0000 1000
  Wire.endTransmission();

  // Set the gyro range: ±500 °/s
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(0x43);  // GYR_RANGE register
  Wire.write(0x02);
  Wire.endTransmission();

  /* Access registers storing gyro measurements */
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(0x0C);  // GYR_X_LSB
  Wire.endTransmission(false);

  /*
  #####################################################
  ##  REGISTER  ##  DATA[X]  ##       ACRONYM        ##
  #####################################################
  ##    0x0C    ##    X= 8   ##  GYR_X <7:0>  (LSB)  ##
  ##    0x0D    ##    X= 9   ##  GYR_X <15:8> (MSB)  ##
  ##    0x0E    ##    X=10   ##  GYR_Y <7:0>  (LSB)  ##
  ##    0x0F    ##    X=11   ##  GYR_Y <15:8> (MSB)  ##
  ##    0x10    ##    X=12   ##  GYR_Z <7:0>  (LSB)  ##
  ##    0x11    ##    X=13   ##  GYR_Z <15:8> (MSB)  ##
  #####################################################
  */

  Wire.requestFrom(BMI160_ADDR, 6);
  if (Wire.available() >= 6) {
    int16_t gyroX = Wire.read() | (Wire.read() << 8);
    int16_t gyroY = Wire.read() | (Wire.read() << 8);
    int16_t gyroZ = Wire.read() | (Wire.read() << 8);

    float gyroScale = 65.6f;  // LSB per °/s (±500°/s full-scale)

    gyroData[0] = gyroX / gyroScale * DEG_TO_RAD;
    gyroData[1] = gyroY / gyroScale * DEG_TO_RAD;
    gyroData[2] = gyroZ / gyroScale * DEG_TO_RAD;
  }
  
}

void gyro_calibration(void) {

  for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
    gyro_signals();

    gyroCalibration[0] += gyroData[0];
    gyroCalibration[1] += gyroData[1];
    gyroCalibration[2] += gyroData[2];

    delay(1);
  }

  for (uint8_t i = 0; i < 3; i++) {
    gyroCalibration[i] /= GYRO_CALIBRATION_SAMPLES;
  }

}

void accel_signals(void) {
  /* { ACCELEROMETER } */

  // DIGITAL FILTER: ON (OSR4: Oversampling Rate of 4)
  // ODR (Output Data Rate) = 100 Hz
  // Fc (Cutoff Frequency) ≈ 10.12 Hz
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(0x40);  // ACC_CONF register
  Wire.write(0x08);  // 0b0000 1000
  Wire.endTransmission();

  // Set the accel range = ±8 g
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(0x41);  // ACC_RANGE register
  Wire.write(0x08);  // 0b0000 1000
  Wire.endTransmission();

  // Access registers storing accel measurements
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(0x12);  // ACC_X_LSB
  Wire.endTransmission(false);

  /*
  #####################################################
  ##  REGISTER  ##  DATA[X]  ##       ACRONYM        ##
  #####################################################
  ##    0x12    ##    X=14   ##   ACC_X <7:0>  (LSB)  ##
  ##    0x13    ##    X=15   ##   ACC_X <15:8> (MSB)  ##
  ##    0x14    ##    X=16   ##   ACC_Y <7:0>  (LSB)  ##
  ##    0x15    ##    X=17   ##   ACC_Y <15:8> (MSB)  ##
  ##    0x16    ##    X=18   ##   ACC_Z <7:0>  (LSB)  ##
  ##    0x17    ##    X=19   ##   ACC_Z <15:8> (MSB)  ##
  #####################################################
  */

  Wire.requestFrom(BMI160_ADDR, 6);
  if (Wire.available() >= 6) {
    int16_t AccX_LSB = Wire.read() | (Wire.read() << 8);
    int16_t AccY_LSB = Wire.read() | (Wire.read() << 8);
    int16_t AccZ_LSB = Wire.read() | (Wire.read() << 8);

    float AccScale = 4096.0f;

    accelData[0] = AccX_LSB / AccScale + 0.02f; // Those errors are our BMI160 sensor's error,
    accelData[1] = AccY_LSB / AccScale + 0.06f; // it should be calibrated for each sensor separetly
    accelData[2] = AccZ_LSB / AccScale - 0.01f;
  }

  // Calculate the angles
  // atan2() returns an angle in the range −π to +π
  float Ax = accelData[0], Ay = accelData[1], Az = accelData[2];
  AngleRoll  =  atan2(Ay, sqrt(Ax*Ax + Az*Az));
  AnglePitch = -atan2(Ax, sqrt(Ay*Ay + Az*Az));
}


// 1D KALMAN FILTER - ROLL ANGLE & PITCH ANGLE

// (angle) predictions and (angle) uncertainties
float AngleRoll_Kalman = 0.0f; // Filtered roll angle [rad]
float AngleRoll_KalmanUncertainty = (2.0f * DEG_TO_RAD) * (2.0f * DEG_TO_RAD); // Initial uncertinty is ±2 degrees
float AnglePitch_Kalman = 0.0f; // Filtered pitch angle [rad]
float AnglePitch_KalmanUncertainty = (2.0f * DEG_TO_RAD) * (2.0f * DEG_TO_RAD);

void Kalman_Filter_1D(float *KalmanState, float *KalmanUncertainty, float KalmanInput, float Variance1, float KalmanMeasurement, float Variance2) {
  // Predict the current state of the system
  *KalmanState = *KalmanState + SAMPLING_PERIOD * KalmanInput;

  // Calculate the uncertainty of the prediction
  *KalmanUncertainty = *KalmanUncertainty + (SAMPLING_PERIOD * SAMPLING_PERIOD) * Variance1;

  // Calculate the Kalman gain from (the uncertainties on prediction) and (measurements)
  float KalmanGain = *KalmanUncertainty / (*KalmanUncertainty + Variance2);

  // Update the predicted state with the measurement of the state through the Kalman gain
  *KalmanState = *KalmanState + KalmanGain * (KalmanMeasurement - *KalmanState);

  // Update the uncertainty of the predicted state
  *KalmanUncertainty = (1.0f - KalmanGain) * *KalmanUncertainty;
}


/* 1D KALMAN FILTER

  To compute the variance, We have used this code:

  long N = 5000; float sum = 0, sumsq = 0;

  for (long i = 0; i < N; i++) {
    gyro_signals(); // accel_signals() for Roll/Pitch angles
    float g = gyroData[1];  // Pitch/Roll rate (from gyro). Pitch/Roll angle (from accel)
    sum   += g;
    sumsq += g*g;
    delay(1);
  }
  float mean = sum / N;
  float variance = (sumsq / N) - (mean * mean);
  Serial.print("Gyro [Pitch] noise variance: "); Serial.println(variance, 10);
*/