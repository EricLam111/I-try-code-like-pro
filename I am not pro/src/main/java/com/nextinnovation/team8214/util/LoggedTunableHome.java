package com.nextinnovation.team8214.util;

public class LoggedTunableHome {
  private final LoggedTunableNumber stallTimeSec;
  private final LoggedTunableNumber currentAmp;

  public LoggedTunableHome(String logRoot, double stallTimeSec, double currentAmp) {
    this("Default", logRoot, stallTimeSec, currentAmp);
  }

  public LoggedTunableHome(
      String logGroup, String logRoot, double stallTimeSec, double currentAmp) {
    var cleanLogGroup = logGroup.replaceFirst("/$", "");
    var cleanLogRoot = logRoot.replaceFirst("/$", "") + "/home";

    this.stallTimeSec =
        new LoggedTunableNumber(cleanLogGroup, cleanLogRoot + "/stallTimeSec", stallTimeSec);
    this.currentAmp =
        new LoggedTunableNumber(cleanLogGroup, cleanLogRoot + "/currentAmp", currentAmp);
  }

  public double getStallTimeSec() {
    return stallTimeSec.getAsDouble();
  }

  public double getCurrentAmp() {
    return currentAmp.getAsDouble();
  }
}
