public class Turret extends SubsystemBase {
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

    
}