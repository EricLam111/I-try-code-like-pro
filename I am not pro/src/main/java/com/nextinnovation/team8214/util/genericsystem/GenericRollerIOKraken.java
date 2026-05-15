package com.nextinnovation.team8214.util.genericsystem;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.nextinnovation.team8214.util.EqualsUtil;
import com.nextinnovation.team8214.util.driver.CanId;
import com.nextinnovation.team8214.util.driver.Phoenix6Helper;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import java.util.ArrayList;
import java.util.List;

public class GenericRollerIOKraken implements GenericRollerIO {
  private final String name;

  private final TalonFX master;
  private final List<TalonFX> slaves = new ArrayList<>();

  private final TalonFXConfiguration masterConfig;

  private final StatusSignal<AngularVelocity> vel;
  private final StatusSignal<Voltage> outputVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> temperature;

  private final VoltageOut voltageSetter = new VoltageOut(0.0);
  private final VelocityVoltage velSetter = new VelocityVoltage(0.0).withSlot(0);
  private final VelocityDutyCycle velBangbangDutyCycleSetter =
      new VelocityDutyCycle(0.0).withSlot(1);
  private final VelocityTorqueCurrentFOC velBangbangCurrentSetter =
      new VelocityTorqueCurrentFOC(0.0).withSlot(1);
  private final Debouncer velBangbangCurrentControlDebouncer =
      new Debouncer(0.025, Debouncer.DebounceType.kFalling);

  private final MotionMagicVelocityVoltage motionMagicVelSetter =
      new MotionMagicVelocityVoltage(0.0);
  private final TorqueCurrentFOC currentSetter = new TorqueCurrentFOC(0.0);
  private final NeutralOut neutralSetter = new NeutralOut();
  private final List<Follower> slaveSetters = new ArrayList<>();

  private boolean wantInputOnly = false;
  private boolean wantEnableFOC = true;

  public GenericRollerIOKraken(String name, CanId id, TalonFXConfiguration config) {
    this.name = name;
    master = new TalonFX(id.id(), id.bus());
    masterConfig = config;
    var wrappedName = "[" + name + "]";
    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " clear sticky fault", master::clearStickyFaults);
    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " config", () -> master.getConfigurator().apply(masterConfig));

    vel = master.getVelocity();
    outputVoltage = master.getMotorVoltage();
    supplyCurrent = master.getSupplyCurrent();
    statorCurrent = master.getStatorCurrent();
    temperature = master.getDeviceTemp();

    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " set signals update frequency",
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100.0, vel, outputVoltage, supplyCurrent, statorCurrent, temperature));
    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " optimize CAN utilization", master::optimizeBusUtilization);
  }

  public GenericRollerIOKraken withFollower(CanId id, boolean isInvert) {
    var slave = new TalonFX(id.id(), id.bus());

    var slaveConfig = new TalonFXConfiguration();
    slaveConfig.MotorOutput.NeutralMode = masterConfig.MotorOutput.NeutralMode;

    var slaveSetter =
        new Follower(
            master.getDeviceID(),
            isInvert ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned);

    var wrappedName = "[" + name + "Slave" + slaves.size() + "]";
    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " clear sticky fault", slave::clearStickyFaults);
    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " config", () -> slave.getConfigurator().apply(slaveConfig));
    Phoenix6Helper.checkErrorAndRetry(wrappedName + " follow", () -> slave.setControl(slaveSetter));
    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " optimize CAN utilization", slave::optimizeBusUtilization);

    slaves.add(slave);
    slaveSetters.add(slaveSetter);

    return this;
  }

  public GenericRollerIOKraken withInputOnly() {
    wantInputOnly = true;

    return this;
  }

  public GenericRollerIOKraken withDisableFOC() {
    wantEnableFOC = false;

    return this;
  }

  @Override
  public void updateInputs(GenericRollerIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(vel, outputVoltage, supplyCurrent, statorCurrent, temperature)
            .isOK();

    inputs.velRadPerSec = Units.rotationsToRadians(vel.getValueAsDouble());
    inputs.outputVoltageVolt = outputVoltage.getValueAsDouble();
    inputs.supplyCurrentAmp = supplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmp = statorCurrent.getValueAsDouble();
    inputs.tempCelsius = temperature.getValueAsDouble();

    if (!slaves.isEmpty()) {
      var followersConnected = new boolean[slaves.size()];
      var followersSupplyCurrentAmp = new double[slaves.size()];
      for (var i = 0; i < slaves.size(); i++) {
        final var supplyCurrent = slaves.get(i).getSupplyCurrent();
        followersConnected[i] = supplyCurrent.refresh().getStatus().isOK();
        followersSupplyCurrentAmp[i] =
            followersConnected[i] ? supplyCurrent.getValueAsDouble() : 0.0;
      }
      inputs.followersConnected = followersConnected;
      inputs.followersSupplyCurrentAmp = followersSupplyCurrentAmp;
    } else {
      inputs.followersConnected = new boolean[0];
      inputs.followersSupplyCurrentAmp = new double[0];
    }
  }

  @Override
  public void setVoltage(double voltageVolt) {
    if (!wantInputOnly) {
      master.setControl(voltageSetter.withOutput(voltageVolt).withEnableFOC(wantEnableFOC));
      setSlavesFollow();
    }
  }

  @Override
  public void setVel(double velRadPerSec) {
    if (!wantInputOnly) {
      master.setControl(
          velSetter
              .withVelocity(Units.radiansToRotations(velRadPerSec))
              .withEnableFOC(wantEnableFOC));
      setSlavesFollow();
    }
  }

  @Override
  public void setVelBangbang(
      double targetVelRadPerSec,
      double currentVelRadPerSec,
      double currentControlToleranceVelRadPerSec) {
    var inTolerance =
        EqualsUtil.epsilonEquals(
            currentVelRadPerSec, targetVelRadPerSec, currentControlToleranceVelRadPerSec);
    var useCurrentControl = velBangbangCurrentControlDebouncer.calculate(inTolerance);

    master.setControl(
        useCurrentControl
            ? velBangbangCurrentSetter.withVelocity(Units.radiansToRotations(targetVelRadPerSec))
            : velBangbangDutyCycleSetter
                .withVelocity(Units.radiansToRotations(targetVelRadPerSec))
                .withEnableFOC(wantEnableFOC));
  }

  @Override
  public void setVel(double velRadPerSec, double accelRadPerSec2) {
    if (!wantInputOnly) {
      master.setControl(
          motionMagicVelSetter
              .withVelocity(Units.radiansToRotations(velRadPerSec))
              .withAcceleration(Units.radiansToRotations(accelRadPerSec2))
              .withEnableFOC(wantEnableFOC));
      setSlavesFollow();
    }
  }

  @Override
  public void setPdf(double kp, double kd, double kv, double ks) {
    masterConfig.Slot0.kP = kp;
    masterConfig.Slot0.kD = kd;
    masterConfig.Slot0.kV = kv;
    masterConfig.Slot0.kS = ks;
    master.getConfigurator().apply(masterConfig);
  }

  @Override
  public void setCurrent(double currentAmp) {
    if (!wantInputOnly) {
      master.setControl(currentSetter.withOutput(currentAmp));
      setSlavesFollow();
    }
  }

  @Override
  public void stop() {
    master.setControl(neutralSetter);
    setSlavesFollow();
  }

  private void setSlavesFollow() {
    for (int i = 0; i < slaves.size(); i++) {
      slaves.get(i).setControl(slaveSetters.get(i));
    }
  }
}
