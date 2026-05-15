@ExtensionMethod({GeomUtil.class})
public class ScoreController {
    private static final LoggedTunableNumber flywheelPreSpinMaxAccelMeterPerSec2 =
            new LoggedTunableNumber (
                    ShooterConfig.LOG_GROUP,
                    ShooterConfig.LOG_ROOT
                    + "/controller/scoreController/flywheelPreSpinMaxAccelMeterPerSec2",
                    5.0
            );
}