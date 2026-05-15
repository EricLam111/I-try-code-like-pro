package com.nextinnovation.team8214.util.genericsystem;

import org.littletonrobotics.junction.AutoLog;

public interface GenericRollerIO {
  @AutoLog
  class GenericRollerIOInputs {
    public boolean connected;

    public double velRadPerSec;
    public double outputVoltageVolt;
    public double supplyCurrentAmp;
    public double statorCurrentAmp;
    public double tempCelsius;

    public boolean[] followersConnected;
    public double[] followersSupplyCurrentAmp;
  }

  default void updateInputs(GenericRollerIOInputs inputs) {}

  default void setPdf(double kp, double kd, double kv, double ks) {}

  default void setVoltage(double voltageVolt) {}

  default void setVel(double velRadPerSec) {}

  default void setVelBangbang(
      double targetVelRadPerSec,
      double currentVelRadPerSec,
      double currentControlToleranceVelRadPerSec) {}

  default void setVel(double velRadPerSec, double accelRadPerSec2) {}

  default void setCurrent(double currentAmp) {}

  default void stop() {}
}
