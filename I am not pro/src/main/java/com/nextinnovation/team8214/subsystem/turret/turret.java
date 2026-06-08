package com.nextinnovation.team8214.subsystem.turret;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.nextinnovation.team8214.Config;
import com.nextinnovation.team8214.Ports;
import com.nextinnovation.team8214.util.LoggedTunableNumber;
import com.nextinnovation.team8214.util.LoggedTunableGains;
import com.nextinnovation.team8214.util.genericsystem.GenericArmIO;
import com.nextinnovation.team8214.util.genericsystem.GenericArmIOInputsAutoLogged;
import com.nextinnovation.team8214.util.genericsystem.GenericArmIOKraken;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Turret extends SubsystemBase {

    private final LoggedTunableGains TURRET_GAINS =
            new LoggedTunableGains(
                    TurretConfig.LOG_GROUP,
                    TurretConfig.LOG_ROOT + "/turret",
                    0.43,
                    0.0,
                    0.0,
                    12.0 / (5800.0 / 60.0) * TurretConfig.TURRET_GEAR_RATIO,
                    0.0,
                    0.0
            );

    public enum ControlMode {
        IDLE,
        EJECT,
        TRANSPORT,
        SCORE,
        FENCE,
        PRESET
    }
    private final GenericArmIO turretIO;
    private final GenericArmIOInputAutoLogged turretInput = new GenericArmIOInputsAutoLogged();

    private final GenericRollerIO flywheelIO;
    private final GenericArmIOInputAutoLogged flywheelInput = new GenericArmIOInputsAutoLogged();

    private final GenericArmIO pitchIO;
    private final GenericArmIOInputAutoLogged pitchInput = new GenericArmIOInputsAutoLogged();

    private final Alert turretOfflineAlert = new Alert("Turret motor offline", Alert.AlertType.WARNING);

    @Getter
    @AutoLogOutput(key = TurretConfig.LOG_ROOT + "/mode")
    private ControlMode mode = ControlMode.IDLE;

    @Getter
    @AutoLogOutput(key = TurretConfig.LOG_ROOT + "/turretOnTarget")
    private boolean turretOnTarget = false;

    @Getter
    @AutoLogOutput(key = TurretConfig.LOG_ROOT + "/flywheelOnTarget")
    private boolean flywheelOnTarget = false;

    @Getter
    @AutoLogOutput(key = TurretConfig.LOG_ROOT + "/pitchOnTarget")
    private boolean pitchOnTarget = false;

    private ScoreController scoreController = new ScoreController();
    private PresetController presetController = null;

    public Turret() {
        switch (Config.MODE) {
            case REAL -> {
                var turretConfig = new TalonFXConfiguration();
                turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                turretConfig.Slot0 = 
                        new Slot0Configs()
                                .withKP(TURRET_GAINS.getKp())
                                .withKD(TURRET_GAINS.getKd())
                                .withKS(TURRET_GAINS.getKs())
                                .withKG(TURRET_GAINS.getKg())
                                .withGravityType(GravityTypeValue.Arm_Cosine);

                turretConfig.Feedback.SensorToMechanismRatio = (40.0/8.0) * (160.0 / 10.0);
                turretIO =
                        new GenericArmIOKraken(
                                "turret/turret", 
                                Ports.Can.TURRET_SPIN, 
                                turretConfig,
                                Unit.degreesToRadians(turretConfig.START_ANGLE_DEGREE)
                        );

                var flywheelConfig = new TalonFXConfiguration();
                flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                flywheelConfig.Slot0 =
                        new Slot0Config(
                                .withKP(FLYWHEEL_GAINS.getKp())
                                .withKD(FLYWHEEL_GAINS.getKd())
                                .withKS(FLYWHEEL_GAINS.getKs())
                                .withKG(FLYWHEEL_GAINS.getKg());
                        )
                flywheelConfig.Slot1 = new Slot1Config().withKP(999999.0);
                flywheelConfig.CurrentLimits.StatorCurrentLimit = 80.0;
                flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
                flywheelConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
                flywheelConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
                flywheelConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
                flywheelConfig.MotorOutput.PeakReverseDutyCycle = 0.0;
                flywheelConfig.Feedback.SensorToMechanismRatio = turretConfig.FLYWHEEL_GEAR_RATIO;

                flywheelIO =
                        new GenericRollerIOKraken(
                                "shooter/flywheel", Ports.Can.TURRET_FLYWHEEL_UP_LEFT_MASTER, flywheelConfig
                                .withFollower(Ports.Can.TURRET_FLYWHEEL_DOWN_SALVE, false)
                                .withFollower(Ports.Can.TURRET_FLYWHEEL_UP_SALVE, true));
                var pitchConfig = new TalonFXConfiguration();
                pitchConfig.MotorOutput.NeuturalMode = NeutralModeValue.Brake;
                pitchConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                pitchConfig.Slot0 =
                        new Slot0Config()
                                .withKP(PITCH_GAINS.getKp())
                                .withKD(PITCH_GAINS.getKd())
                                .withKS(PITCH_GAINS.getKs())
                                .withKG(PITCH_GAINS.getKg())
                                .withGravityType(GravityTypeValue.Arm_Cosine);
                pitchConfig.Feedback.SensorToMechanismRatio = (50.0 / 8.0) * (156.0 / 10.0);

                pitchIO =
                        new GenericArmIOKraken(
                                "shooter/pitch",
                                Ports.Can.SHOOTER_PITCH,
                                pitchConfig,
                                Units.degreesToRadians(turretConfig.START_ANGLE_DEGREE));

            }

            case SIM -> {
                turretIO = new GenericArmIOSim(
                        Units.degreesToRadians(turretConfig.START_ANGLE_DEGREE),
                        Units.degreesToRadians(50.0),
                        Units.degreesToRadians(85.0)
                );
            }

            default -> {
                turretIO = new GenericArmIO() {};
                flywheelIO = new GenericRollerIO() {};
                pitchIO = new GenericArmIO() {};
            }
        }
    }
}