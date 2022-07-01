package frc.robot.coordinators;

import org.frcteam2910.common.robot.UpdateManager.Updatable;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;
import org.frcteam6941.utils.AngleNormalization;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Launcher extends SubsystemBase implements Updatable {
    private ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private TurretSubsystem turret = TurretSubsystem.getInstance();
    private SJTUSwerveMK5Drivebase drivebase = SJTUSwerveMK5Drivebase.getInstance();
    private VisionSubsystem vision = VisionSubsystem.getInstance();

    private PIDController angleDeltaController = new PIDController(0.75, 0.0001, 0.0);

    private STATE state = STATE.LOSS_TARGET;
    private double targetAngle;
    private double guessAimAngle;
    private boolean isLimited = true;
    private boolean isDrivebaseFirst = true;
    private boolean enableMotionCompensation = false;
    private boolean forceMaintain = false;

    private static Launcher instance;

    public static Launcher getInstance() {
        if (instance == null) {
            instance = new Launcher();
        }
        return instance;
    }

    private Launcher() {
        this.angleDeltaController.setSetpoint(0.0);
    }

    public double getFieldOrientedDrivetrainHeading() {
        return AngleNormalization.getAbsoluteAngleDegree(this.drivebase.getYaw().getDegrees());
    }

    public double getFieldOrientedTurretHeading() {
        return AngleNormalization
                .getAbsoluteAngleDegree(this.turret.getTurretAngle() + this.getFieldOrientedDrivetrainHeading());
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
    // TODO: It seems to be OK with only the turret working and out put the angle
    // that need to be adjusted by the drivetrain.
    private double[] calculateTurretDrivetrainAngle(double desiredAngle, boolean isLimited) {
        double delta = AngleNormalization.placeInAppropriate0To360Scope(this.getFieldOrientedTurretHeading(),
                desiredAngle)
                - this.getFieldOrientedTurretHeading();
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
                    delta - availableTurretDelta + this.getFieldOrientedDrivetrainHeading() };
        }
    }

    private void aimAtFieldOrientedAngleOnce(double angle, boolean isLimited) {
        double[] targetArray = this.calculateTurretDrivetrainAngle(angle, isLimited);
        // In this case, the drivebase does not need to be turned. So lockHeading is not
        // needed.
        if (targetArray[1] == 0.0) {
            this.turret.lockAngle(targetArray[0]);
            // Or, both the drivebase and the turret need to be rotated.
            // If the system is not in drivebase first mode, turn freely.
        } else if (!this.isDrivebaseFirst) {
            this.turret.lockAngle(targetArray[0]);
            this.drivebase.setHeadingTarget(targetArray[1]);
            // Else, limit the change of launcher to only turn the turret.
        } else {
            this.turret.lockAngle(targetArray[0]);
        }
    }

    public void aimAtFieldOrientedAngle(double angle, boolean isLimited) {
        this.targetAngle = angle;
        this.isLimited = isLimited;
    }

    public void aimAtFieldOrientedAngleGuess(double angle) {
        this.guessAimAngle = angle;
        this.isLimited = true;
    }

    public void aimAtTurretOrientedAngle(double turretAngle) {
        this.aimAtFieldOrientedAngle(this.drivebase.getYaw().getDegrees() + turretAngle, isLimited);
    }

    public void aimAtTurretOrientedAngleDelta(double delta, boolean isLimited) {
        this.aimAtFieldOrientedAngle(this.drivebase.getYaw().getDegrees() + this.turret.getTurretAngle() + delta,
                isLimited);
    }

    public void aimAtFieldPosition(Translation2d fieldTarget, boolean isLimited) {
        Translation2d currentPosition = this.drivebase.getPose().getTranslation();
        Translation2d travel = fieldTarget.minus(currentPosition);
        this.aimAtFieldOrientedAngle(new Rotation2d(travel.getX(), travel.getY()).getDegrees(), isLimited);
    }

    public void aimAtVisionTarget(double time) {
        if (this.vision.getUpperhubState().equals(VisionSubsystem.VISION_STATE.HAS_TARGET)) {
            this.aimAtTurretOrientedAngleDelta(
                    -angleDeltaController
                            .calculate(this.vision.getCompensatedUpperHubDeltaAngleDegreesAtTime(time, false)),
                    isLimited);
        }
    }

    public void changeLimited(boolean status) {
        this.isLimited = status;
    }

    public void changeDrivebaseFirst(boolean isDrivebaseFirst) {
        this.isDrivebaseFirst = isDrivebaseFirst;
    }

    public void changeMaintain(boolean forceMaintain) {
        this.forceMaintain = forceMaintain;
    }

    /**
     * An enhanced method in order to coordinate turret and drivetrain movement.
     * Called through commands.
     * 
     * @param translation     Translation, x and y ranging from -1 to 1.
     * @param rotation        Rotation velocity. No specific range, but recommended
     *                        to have it in -1 to 1.
     * @param isFieldOriented Is the drive field oriented.
     */
    public void driveCoordinated(Translation2d translation, double rotation, boolean isFieldOriented) {
        if (!isDrivebaseFirst) {
            if ((rotation >= 0 && this.turret.forwardSafeWithZone(Constants.TURRET_MANUAL_ROTATION_GAP_ZONE))
                    || (rotation <= 0 && this.turret.reverseSafeWithZone(Constants.TURRET_MANUAL_ROTATION_GAP_ZONE))) {
                rotation = 0;
            }
        }
        drivebase.drive(translation, rotation, isFieldOriented);
    }

    private double calculateMotionCompensationFeedforward(double time, double dt) {
        return 0.0;
    }

    @Override
    public void update(double time, double dt) {
        // State Transition
        switch (state) {
            case LOSS_TARGET:
                if (this.vision.getUpperhubState().equals(VisionSubsystem.VISION_STATE.HAS_TARGET)) {
                    this.setState(STATE.HAS_TARGET);
                }
                break;
            case HAS_TARGET:
                if (this.vision.getUpperhubState().equals(VisionSubsystem.VISION_STATE.LOSS_TARGET)) {
                    this.setState(STATE.LOSS_TARGET);
                }
                if (this.shooter.isHighReady()) {
                    this.setState(STATE.READY);
                }
                break;
            case READY:
                if (this.vision.getUpperhubState().equals(VisionSubsystem.VISION_STATE.LOSS_TARGET)) {
                    this.setState(STATE.LOSS_TARGET);
                }
                if (!this.shooter.isHighReady()) {
                    this.setState(STATE.HAS_TARGET);
                }
                break;
        }

        // Launcher Actions
        switch (state) {
            case LOSS_TARGET:
                if (!forceMaintain) {
                    this.shooter.setState(ShooterSubsystem.STATE.LOW_SPEED);
                } else {
                    this.shooter.setState(ShooterSubsystem.STATE.HIGH_SPEED);
                }
                this.aimAtFieldOrientedAngleOnce(guessAimAngle, isLimited);
                this.changeLimited(true);
                break;
            case HAS_TARGET:
                this.aimAtVisionTarget(time);
                this.shooter.setState(ShooterSubsystem.STATE.HIGH_SPEED);
                this.aimAtFieldOrientedAngleOnce(targetAngle, isLimited);
                this.changeLimited(true);
                break;
            case READY:
                this.aimAtVisionTarget(time);
                this.shooter.setState(ShooterSubsystem.STATE.HIGH_SPEED);
                break;
        }
        SmartDashboard.putBoolean("Drivebase First", this.isDrivebaseFirst);
        SmartDashboard.putNumber("Field Target Angle", this.targetAngle);
        SmartDashboard.putNumber("Field Guess Angle", this.guessAimAngle);
        SmartDashboard.putNumber("Turret Target Angle", this.turret.getTurretTargetAngle());
        SmartDashboard.putBoolean("Drivebase Compensation Enabled", this.drivebase.isLockHeading());
        SmartDashboard.putNumber("Drivebase Target Angle", this.drivebase.getHeadingTarget());
    }

    public static enum STATE {
        LOSS_TARGET,
        HAS_TARGET,
        READY
    }

    public STATE getState() {
        return this.state;
    }

    public void setState(STATE state) {
        this.state = state;
    }
}
