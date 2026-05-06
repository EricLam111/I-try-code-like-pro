package com.nextinnovation.team8214.util.driver;

import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import java.time.Duration;

public class BeamBreak {
  private final DigitalInput source;
  private final DigitalGlitchFilter glitchFilter;
  private final boolean isInvert;

  private boolean lastValue = false;
  private boolean value = false;

  public BeamBreak(int digital_channel, boolean is_invert) {
    source = new DigitalInput(digital_channel);
    glitchFilter = new DigitalGlitchFilter();
    glitchFilter.setPeriodNanoSeconds(Duration.ofMillis(5).toNanos());
    glitchFilter.add(source);
    isInvert = is_invert;
  }

  public void update() {
    lastValue = value;
    value = isInvert != source.get();
  }

  public boolean hasRisingEdge() {
    return !lastValue && value;
  }

  public boolean hasFallingEdge() {
    return lastValue && !value;
  }

  public boolean hasAny() {
    return value;
  }
}
