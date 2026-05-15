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

    private final GenericArmIO pitchIO;
    private final GenericArmIOInputAutoLogged pitchInput = new GenericArmIOInputsAutoLogged();

    @Getter
    @AutoLogOutput(key = TurretConfig.LOG_ROOT + "/mode")
    private ControlMode mode = ControlMode.IDLE;

    @Getter
    @AutoLogOutput(key = TurretConfig.LOG_ROOT + "/turretOnTarget")
    private boolean turretOnTarget = false;

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

            }
        }
    }
}