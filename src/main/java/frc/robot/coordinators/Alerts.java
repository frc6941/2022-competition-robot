package frc.robot.coordinators;

import org.frcteam2910.common.robot.UpdateManager.Updatable;
import org.frcteam6328.utils.Alert;
import org.frcteam6328.utils.Alert.AlertType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class Alerts implements Updatable {
    /** Define hardware items. */
    TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();
    ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();

    /** CheckList Items. */
    public Alert turretCalibrationWarning = new Alert(
            "CheckList",
            "Turret is not calibrated.",
            AlertType.WARNING);
    public Alert climberCalibrationWarning = new Alert(
            "CheckList",
            "Climber is not calibrated.",
            AlertType.WARNING);
    public static Alerts getInstance() {
        if (instance == null) {
            instance = new Alerts();
        }
        return instance;
    }

    private Alerts() {

    }

    private static Alerts instance;

    @Override
    public void update(double time, double dt) {
        turretCalibrationWarning.set(!turretSubsystem.isCalibrated());
        climberCalibrationWarning.set(!climberSubsystem.isClimberCalibrated());
    }
}
