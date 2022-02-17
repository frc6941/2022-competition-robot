package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.frcteam1678.lib.math.Conversions;
import org.frcteam2910.common.robot.UpdateManager.Updatable;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase implements Updatable {

    private TalonFX turretMotor = new TalonFX(Constants.CANID.TURRET_MOTOR);

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
        // As ccw is positive, the angle read by the sensor need to be reverted again.
        return -Conversions.falconToDegrees(
                this.turretMotor.getSelectedSensorPosition() - ((forwardMaxPosition + reverseMaxPosition) / 2.0),
                Constants.TURRET_GEAR_RATIO);
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

        // See if the turret is turing to the 'unsafe' direction: if so, immediately stop its motion.
        if ((delta >= 0 && this.forwardSafe())
                || (delta <= 0 && this.reverseSafe())) {
            // As ccw is positive here due to sensor reasons, angle need to be reverted.
            this.turretMotor.set(ControlMode.MotionMagic,
                    Conversions.degreesToFalcon(-angle, Constants.TURRET_GEAR_RATIO)
                            + (forwardMaxPosition + reverseMaxPosition) / 2.0);
        } else {
            this.turnOff();
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

    public boolean forwardSafe() {
        return getTurretAngle() <= Constants.TURRET_MAX_ROTATION_DEGREE;
    }

    public boolean reverseSafe() {
        return getTurretAngle() >= -Constants.TURRET_MAX_ROTATION_DEGREE;
    }

    @Override
    public void update(double time, double dt) {
        if (this.turretMotor.isFwdLimitSwitchClosed() == 1) {
            this.forwardMaxPosition = turretMotor.getSelectedSensorPosition();
            System.out.println("Forward is being Calibrated.");
            this.isForwardCalibrated = true;
        }
        if (this.turretMotor.isRevLimitSwitchClosed() == 1) {
            this.reverseMaxPosition = turretMotor.getSelectedSensorPosition();
            System.out.println("Reverse is being Calibrated.");
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
