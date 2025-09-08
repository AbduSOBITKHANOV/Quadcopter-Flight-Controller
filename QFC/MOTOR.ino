void motors_init(void) {

  for (uint8_t i = 0; i < MTR_NUMBER; i++) {
    motor[i].setPeriodHertz(50); // The ESC works with 50HZ
    motor[i].attach(motorPins[i], 1000, 2000); // Attach the motor to its pin, and set its throttle to the range [1000, 2000]

    motor[i].write(1000); // Start with zero throttle to initialize the ESCs
  }

  delay(3000);  // Let the ESC initialize itself
}

