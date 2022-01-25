package frc.robot.coordinators;

import org.frcteam2910.common.robot.UpdateManager.Updatable;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;
import org.frcteam6941.utils.AngleNormalization;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.TurretSubsystem.STATE;
import frc.robot.subsystems.VisionSubsystem.VISION_STATE;

public class LauncherMechanismCoordinator extends SubsystemBase implements Updatable {
    private ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private TurretSubsystem turret = TurretSubsystem.getInstance();
    private VisionSubsystem vision = VisionSubsystem.getInstance();
    private SJTUSwerveMK5Drivebase drivebase = SJTUSwerveMK5Drivebase.getInstance();

    private STATE state = STATE.OFF;

    private double targetAngle;
    private boolean isLimited = true;

    private static LauncherMechanismCoordinator instance;

    public static LauncherMechanismCoordinator getInstance() {
        if (instance == null) {
            instance = new LauncherMechanismCoordinator();
        }
        return instance;
    }

    private LauncherMechanismCoordinator() {
    }

    /**
     * Function used to calculate the needed change in turret and drivetrain with
     * respect to the input of a new field-oriented angle.
     * 
     * @param desiredAngle The desired field-oriented angle.
     * @return An array showing the set angle of the
     */
    // TODO: NEED A LOT OF TESTING
    // TODO: Passed basic logic test, need to see implementation on field.
    // TODO: It seems to be OK with only the turret working and out put the angle that need to be adjusted by the drivetrain.
    private double[] calculateTurretDrivetrainAngle(double desiredAngle, boolean isLimited) {
        double currentDrivetrainAngle = AngleNormalization.getAbsoluteAngleDegree(drivebase.getFieldOrientedHeading());
        double currentTurretAngle = currentDrivetrainAngle + turret.getTurretAngle();
        double delta = AngleNormalization.placeInAppropriate0To360Scope(currentTurretAngle, desiredAngle)
                - currentTurretAngle;
        double availableTurretDelta;
        if (isLimited) {
            availableTurretDelta = Math.copySign(Constants.TURRET_SAFE_ZONE_DEGREE, delta) - turret.getTurretAngle();
        } else {
            availableTurretDelta = Math.copySign(Constants.TURRET_MAX_ROTATION_DEGREE, delta)
                    - turret.getTurretAngle();
        }
        if (Math.abs(delta) <= Math.abs(availableTurretDelta)) {
            return new double[] { delta + turret.getTurretAngle(), 0 };
        } else {
            return new double[] { availableTurretDelta + turret.getTurretAngle(),
                    delta - availableTurretDelta + currentDrivetrainAngle };
        }
    }


    private void aimAtFieldOrientedAngleOnce(double angle, boolean isLimited) {
        double[] targetArray = this.calculateTurretDrivetrainAngle(angle, isLimited);
        if (targetArray[1] == 0.0) { // In this case, the drivebase does not need to be turned. So lockHeading is
                                     // not needed.
            this.turret.lockAngle(targetArray[0]);
        } else { // Or, both the drivebase and the turret need to be rotated.
            this.turret.lockAngle(targetArray[0]);
            this.drivebase.setHeadingTarget(targetArray[1]);
        }
    }

    public void aimAtFieldOrientedAngle(double angle, boolean isLimited){
        this.targetAngle = angle;
        this.isLimited = isLimited;
        if(this.getState() == STATE.OFF){
            this.setState(STATE.LOCKING);
        }
    }

    public void aimAtTurretOrientedAngle(double turretAngle){
        this.aimAtFieldOrientedAngle(this.drivebase.getFieldOrientedHeading() + turretAngle, isLimited);
    }

    public void aimAtTurretOrientedAngleDelta(double delta, boolean isLimited) {
        this.aimAtFieldOrientedAngle(this.drivebase.getFieldOrientedHeading() + this.turret.getTurretAngle() + delta,
                isLimited);
    }

    public void aimAtFieldPosition(Translation2d fieldTarget, boolean isLimited) {
        Translation2d currentPosition = this.drivebase.getPose().getTranslation();
        Translation2d travel = fieldTarget.minus(currentPosition);
        this.aimAtFieldOrientedAngle(new Rotation2d(travel.getX(), travel.getY()).getDegrees(), isLimited);
    }

    public void aimAtVisionTarget(){
        System.out.println(this.vision.getUpperhubState());
        if(this.vision.getUpperhubState() == VISION_STATE.HAS_TARGET){
            this.aimAtTurretOrientedAngleDelta(this.vision.getUpperHubDeltaAngleDegrees(), isLimited);
        } else{
            this.exitLocking();
        }
    }

    public void exitLocking(){
        this.setState(STATE.OFF);
    }

    @Override
    public void update(double time, double dt) {
        if(getState() != STATE.OFF){
            if(this.shooter.isReady() && this.turret.isOnTarget()){
                this.setState(STATE.READY);
            } else if(this.shooter.isReady() && !this.turret.isOnTarget()){
                this.setState(STATE.ON_SPEED);
            } else if(!this.shooter.isReady() && this.turret.isOnTarget()){
                this.setState(STATE.ON_TARGET);
            } else if (!this.shooter.isReady() && !this.turret.isOnTarget()){
                this.setState(STATE.LOCKING);
            }
                
        }

        switch(state){
            case LOCKING:
                this.aimAtFieldOrientedAngleOnce(this.targetAngle, this.isLimited);
                break;
            case ON_TARGET:
                this.aimAtFieldOrientedAngleOnce(this.targetAngle, this.isLimited);
                break;
            case ON_SPEED:
                this.aimAtFieldOrientedAngleOnce(this.targetAngle, this.isLimited);
                break;
            case READY:
                this.aimAtFieldOrientedAngleOnce(this.targetAngle, false);
                break;
            case OFF:
                this.turret.turnOff();
                break;
        }
    }

    public static enum STATE {
        LOCKING,
        ON_TARGET,
        ON_SPEED,
        READY,
        OFF
    }

    public STATE getState() {
        return this.state;
    }

    public void setState(STATE state) {
        this.state = state;
    }
}
