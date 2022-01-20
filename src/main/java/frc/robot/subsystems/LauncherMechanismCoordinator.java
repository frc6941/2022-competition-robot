package frc.robot.subsystems;

import org.frcteam2910.common.robot.UpdateManager.Updatable;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;
import org.frcteam6941.utils.AngleNormalization;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherMechanismCoordinator extends SubsystemBase implements Updatable{
    private ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private TurrentSubsystem turrent = TurrentSubsystem.getInstance();
    private SJTUSwerveMK5Drivebase drivebase = SJTUSwerveMK5Drivebase.getInstance();

    private static LauncherMechanismCoordinator instance;

    public static LauncherMechanismCoordinator getInstance() {
        if (instance == null) {
            instance = new LauncherMechanismCoordinator();
        }
        return instance;
    }

    /**
     * Function used to calculate the needed change in turrent and drivetrain with
     * respect to the input of a new field-oriented angle.
     * 
     * @param desiredAngle The desired field-oriented angle.
     * @return An array showing the set angle of the 
     */
    // TODO: NEED A LOT OF TESTING
    public double[] calculateTurrentDrivetrainAngle(double desiredAngle, boolean isLimited) {
        double currentDrivetrainAngle = AngleNormalization.getAbsoluteAngleDegree(drivebase.getYaw().getDegrees());
        double currentTurrentAngle = currentDrivetrainAngle + turrent.getTurrentAngle();
        double delta = AngleNormalization.placeInAppropriate0To360Scope(currentTurrentAngle, desiredAngle) - currentTurrentAngle;
        double availableTurrentDelta;
        if(isLimited){
            availableTurrentDelta = Math.copySign(Constants.TURRENT_SAFE_ZONE_DEGREE, delta) - turrent.getTurrentAngle();
        } else{
            availableTurrentDelta = Math.copySign(Constants.TURRENT_MAX_ROTATION_DEGREE, delta) - turrent.getTurrentAngle();
        }
        if(Math.abs(delta) < Math.abs(availableTurrentDelta)){
            return new double[] {delta + turrent.getTurrentAngle(), 0};
        } else{
            return new double[] {availableTurrentDelta + turrent.getTurrentAngle(), delta - availableTurrentDelta + currentDrivetrainAngle};
        }
        
    }

    private LauncherMechanismCoordinator() {
    }

    @Override
    public void update(double time, double dt) {

    }
}
