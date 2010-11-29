void initTelemetry() {
  tlmSequence = 0;
}

void tlm_out(int data) {
  tlmChecksum += data;
  Serial.print(data);
  Serial.print(" ");
}

void sendTelemetry() {
  tlmChecksum = 0;
  tlm_out(tlmSequence);
  tlm_out(flightMode);

  tlm_out(inputArray[INPUT_LIFT]);

  tlm_out(int(accelSmooth[X_AXIS]));
  tlm_out(int(accelSmooth[Y_AXIS]));
  tlm_out(int(accelSmooth[Z_AXIS]));

  tlm_out(int(flightAngle[ROLL_AXIS]));
  tlm_out(int(flightAngle[PITCH_AXIS]));

  tlm_out(cmdLift);
  tlm_out(cmdRoll);
  tlm_out(cmdPitch);
  tlm_out(cmdYaw);
  
  tlm_out(int(PID[ROLL_AXIS].P*100));
  tlm_out(int(PID[ROLL_AXIS].I*100));
  
  tlm_out(int(PID[PITCH_AXIS].integratedError));
  tlm_out(int(PID[ROLL_AXIS].integratedError));

  tlm_out(motorCommands[0]);
  tlm_out(motorCommands[1]);
  tlm_out(motorCommands[2]);
  tlm_out(motorCommands[3]);
  tlm_out(motorCommands[4]);
  tlm_out(motorCommands[5]);

    /*
  tlm_out(gyroRaw[ROLL_AXIS]);
     tlm_out(gyroRaw[PITCH_AXIS]);
     tlm_out(gyroRaw[YAW_AXIS]);
     tlm_out(int(gyroSmooth[ROLL_AXIS]));
     tlm_out(int(gyroSmooth[PITCH_AXIS]));
     tlm_out(int(gyroSmooth[YAW_AXIS]));
     
     // tlm_out(accelRaw[X_AXIS]);
     // tlm_out(accelRaw[Y_AXIS]);
     // tlm_out(accelRaw[Z_AXIS]);
     tlm_out(int(accelSmooth[X_AXIS]));
     tlm_out(int(accelSmooth[Y_AXIS]));
     tlm_out(int(accelSmooth[Z_AXIS]));
     
     tlm_out(int(resultant));
     tlm_out(int(flightAngle[ROLL_AXIS]));
     tlm_out(int(flightAngle[PITCH_AXIS]));
     tlm_out(int(flightAngle[YAW_AXIS]));
     tlm_out(dutyCycle);
     tlm_out(micros());
     tlm_out(motorCommands[0]);
     //  tlm_out(motorCommands[1]);
     //  tlm_out(motorCommands[2]);
     //  tlm_out(motorCommands[3]);
     //  tlm_out(motorCommands[4]);
     //  tlm_out(motorCommands[5]);*/
    Serial.print(tlmChecksum);
  Serial.println();
  tlmSequence++;
}







