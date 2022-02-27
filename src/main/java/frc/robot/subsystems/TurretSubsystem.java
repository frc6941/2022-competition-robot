package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.frcteam1678.lib.math.Conversions;
import org.frcteam2910.common.robot.UpdateManager.Updatable;
import org.frcteam6941.utils.LazyTalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase implements Updatable {

    private LazyTalonFX turretMotor = new LazyTalonFX(Constants.CANID.TURRET_MOTOR);

    private double forwardMaxPosition = Constants.TURRET_FORWARD_MAX_POSITION;
    private double reverseMaxPosition = Constants.TURRET_REVERSE_MAX_POSITION;

    private double angleLockTarget = 0.0;
    private boolean isForwardCalibrated = false;
    private boolean isReverseCalibrated = false;

    private static TurretSubsystem instance;
    private STATE state = STATE.OFF;

    public static TurretSubsystem getInstance() {
        if (instance == null) {
            instance = new TurretSubsystem();
        }
        return instance;
    }

    private TurretSubsystem() {
        turretMotor.configFactoryDefault();
        turretMotor.setInverted(InvertType.None);
        turretMotor.setNeutralMode(NeutralMode.Brake);
        turretMotor.config_kP(0, Constants.TURRET_KP);
        turretMotor.config_kI(0, Constants.TURRET_KI);
        turretMotor.config_kD(0, Constants.TURRET_KD);
        turretMotor.config_kF(0, Constants.TURRET_KF);
        turretMotor.configMotionCruiseVelocity(Constants.TURRET_MOTION_CRUISE_VELOCITY);
        turretMotor.config_IntegralZone(0, Constants.TURRET_INTEGRAL_ZONE);
        turretMotor.configMotionAcceleration(Constants.TURRET_MOTION_ACCELERATION);
        turretMotor.enableVoltageCompensation(true);
        turretMotor.configNeutralDeadband(Constants.TURRET_NEUTRAL_DEADBAND);
    }

    public double getTurretAngle() {
        // As cw is the true positive and ccw is what the motor thinks is positive, the
        // angle read by the sensor need to be reverted again.
        return -Conversions.falconToDegrees(
                this.turretMotor.getSelectedSensorPosition() - ((forwardMaxPosition + reverseMaxPosition) / 2.0),
                Constants.TURRET_GEAR_RATIO);
    }

    public Translation2d getTurretToDrivetrainTranslation(){
        Rotation2d turretRotation = Rotation2d.fromDegrees(getTurretAngle() - 180.0 + 90.0);
        return Constants.VisionConstants.Turret.TURRET_RING_CENTER_TO_ROBOT_CENTER.plus(
            new Translation2d(Constants.VisionConstants.Turret.TURRET_RING_RADIUS, turretRotation)
        );
    }

    /**
     * Setting the degree of the turret with limit protection.
     * 
     * @param angle The target angle relative to the zero position. Cw is positive
     *              by default.
     */
    public void setTurretAngle(double angle) {
        while (angle > 180.0) {
            angle -= 360.0;
        }
        while (angle < -180.0) {
            angle += 360.0;
        }

        double delta = angle - this.getTurretAngle();

        // First, determine if the set angle is out of reach. If so, set the angle to
        // the reachable maximum or minimum.
        if (Math.abs(angle) < Constants.TURRET_MAX_ROTATION_DEGREE) {
            // Second, judge actively if the turret has reached the limit. If so, actively
            // stop the turret from going any further.
            if ((delta >= 0 && this.forwardSafe())
                    || (delta <= 0 && this.reverseSafe())) {
                // As ccw is positive here due to sensor reasons, angle need to be reverted.
                this.turretMotor.set(ControlMode.MotionMagic,
                        Conversions.degreesToFalcon(-angle, Constants.TURRET_GEAR_RATIO)
                                + (forwardMaxPosition + reverseMaxPosition) / 2.0);
            } else {
                this.turnOff();
            }
        } else {
            this.turretMotor.set(ControlMode.MotionMagic,
                    Conversions.degreesToFalcon(-Math.copySign(90.0, angle), Constants.TURRET_GEAR_RATIO)
                            + (forwardMaxPosition + reverseMaxPosition) / 2.0);
        }

    }

    public void setTurretPercentage(double power) {
        this.turretMotor.set(ControlMode.PercentOutput, -power);
    }

    public void lockAngle(double angle) {
        this.angleLockTarget = angle;
        setState(STATE.LOCK_ANGLE);
    }

    public void stopLock() {
        setState(STATE.IDLE);
    }

    public void turnOff() {
        setState(STATE.OFF);
    }

    public boolean isOnTarget() {
        return Math.abs(this.getTurretAngle() - this.angleLockTarget) < Constants.TURRET_ERROR_TOLERANCE;
    }

    public boolean isCalibrated(){
        return isForwardCalibrated && isReverseCalibrated;
    }

    public boolean forwardSafe() {
        return getTurretAngle() <= Constants.TURRET_MAX_ROTATION_DEGREE;
    }

    public boolean reverseSafe() {
        return getTurretAngle() >= -Constants.TURRET_MAX_ROTATION_DEGREE;
    }

    @Override
    public void update(double time, double dt) {
        if(DriverStation.isDisabled() || Math.abs(getTurretAngle()) < Constants.TURRET_SAFE_ZONE_DEGREE){
            this.turretMotor.setNeutralMode(NeutralMode.Coast);
        } else{
            this.turretMotor.setNeutralMode(NeutralMode.Brake);
        }

        if (this.turretMotor.isFwdLimitSwitchClosed() == 1) {
            this.forwardMaxPosition = turretMotor.getSelectedSensorPosition();
            this.isForwardCalibrated = true;
        }
        if (this.turretMotor.isRevLimitSwitchClosed() == 1) {
            this.reverseMaxPosition = turretMotor.getSelectedSensorPosition();
            this.isReverseCalibrated = true;
        }

        // Carry out calibration according to sensor status. Reverse and forward must
        // both be calibrated either by hand or through motor before further actions.
        if (this.isForwardCalibrated && this.isReverseCalibrated) {
            switch (state) {
                case OFF:
                    this.setTurretPercentage(0.0);
                    break;
                case IDLE:
                    this.setTurretAngle(0.0);
                    break;
                case LOCK_ANGLE:
                    this.setTurretAngle(this.angleLockTarget);
                    break;
            }

            SmartDashboard.putNumber("Forward Max Position", this.forwardMaxPosition);
            SmartDashboard.putNumber("Reverse Max Position", this.reverseMaxPosition);
            SmartDashboard.putNumber("Current Position", this.turretMotor.getSelectedSensorPosition());
            SmartDashboard.putNumber("Current Angle", this.getTurretAngle());
            SmartDashboard.putNumber("Lock Angle", this.angleLockTarget);
            SmartDashboard.putBoolean("Forward Safe", this.forwardSafe());
            SmartDashboard.putBoolean("Reverse Safe", this.reverseSafe());
        } else if (!this.isForwardCalibrated && !this.isReverseCalibrated) {
            this.turretMotor.set(ControlMode.PercentOutput, 0.2);
        } else if (this.isForwardCalibrated && !this.isReverseCalibrated) {
            this.turretMotor.set(ControlMode.PercentOutput, -0.2);
        } else if (!this.isForwardCalibrated && this.isReverseCalibrated) {
            this.turretMotor.set(ControlMode.PercentOutput, 0.2);
        }

    }

    public enum STATE {
        IDLE,
        LOCK_ANGLE,
        OFF
    }

    public void setState(STATE state) {
        this.state = state;
    }

    public STATE getState() {
        return this.state;
    }
}
