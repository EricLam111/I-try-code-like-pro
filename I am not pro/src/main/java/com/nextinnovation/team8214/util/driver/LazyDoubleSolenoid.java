package com.nextinnovation.team8214.util.driver;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class LazyDoubleSolenoid extends DoubleSolenoid {
  private boolean hasInit = false;
  private boolean lastOn = false;
  private final boolean isInvert;

  public LazyDoubleSolenoid(DoubleSolenoidId id, boolean isInvert) {
    super(id.moduleType(), id.forward(), id.reverse());
    this.isInvert = isInvert;
  }

  public void set(boolean on) {
    if (!hasInit || on != lastOn) {
      hasInit = true;
      lastOn = on;
      if (isInvert) {
        super.set(on ? Value.kForward : Value.kReverse);
      } else {
        super.set(on ? Value.kReverse : Value.kForward);
      }
    }
  }
}
