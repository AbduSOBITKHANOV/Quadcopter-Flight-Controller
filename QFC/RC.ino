void read_receiver(void) {
  /*
    If the signal between microcontroller and receiver is lost,
    return defaul RC Value. Otherwise, return the actual values.
  */
  if (ppm.isSignalLost()) {

    RCValue[0] = DEFAULT_RC_R_P_Y;
    RCValue[1] = DEFAULT_RC_R_P_Y;
    RCValue[2] = DEFAULT_RC_THROTTLE;
    RCValue[3] = DEFAULT_RC_R_P_Y;
    RCValue[4] = 2000;
    RCValue[5] = DEFAULT_RC_SWITCH;

  } else if (ppm.available()) {

    for (uint8_t i = 0; i < CHANNEL_AMOUNT; i++)
      RCValue[i] = ppm.getChannelValue(i);

  }

  inputThrottle = RCValue[2];
  armSwitch = RCValue[4];
}

void avoid_sudden_start(void) {
  read_receiver();
  while (inputThrottle < 1020 || inputThrottle > 1050) {
    read_receiver();
    delay(10);
    TURN_ON_RED_LED;
  }
}