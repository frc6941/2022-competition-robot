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
        turretMotor.setInverted(InvertType.InvertMotorOutput);
        turretMotor.setNeutralMode(NeutralMode.Brake);
        turretMotor.config_kP(0, 0.6);
        turretMotor.config_kD(0, 10.0);
        turretMotor.config_kF(0, 1024.0 * 1.0 / 86942.0);
        turretMotor.configMotionCruiseVelocity(24000);
        turretMotor.configMotionAcceleration(24000 / 0.3);
        turretMotor.enableVoltageCompensation(true);
    }

    public double getTurretAngle() {
        return Conversions.falconToDegrees(
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

        boolean forwardSoftLimit = getTurretAngle() <= Constants.TURRET_MAX_ROTATION_DEGREE;
        boolean reverseSoftLimit = getTurretAngle() >= -Constants.TURRET_MAX_ROTATION_DEGREE;
        double delta = angle - this.getTurretAngle();

        if ((delta >= 0 & forwardSoftLimit)
                || (delta <= 0 & reverseSoftLimit)) {
            this.turretMotor.set(ControlMode.MotionMagic,
                    Conversions.degreesToFalcon(angle, Constants.TURRET_GEAR_RATIO) + (forwardMaxPosition + reverseMaxPosition) / 2.0);
        } else {
            this.turnOff();
        }
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

    @Override
    public void update(double time, double dt) {
        if (this.turretMotor.isFwdLimitSwitchClosed() == 1) {
            this.forwardMaxPosition = turretMotor.getSelectedSensorPosition();
            System.out.println("Forward is being Calibrated.");
        }
        if (this.turretMotor.isRevLimitSwitchClosed() == 1) {
            this.reverseMaxPosition = turretMotor.getSelectedSensorPosition();
            System.out.println("Reverse is being Calibrated.");
        }

        switch (state) {
            case OFF:
                this.turretMotor.set(ControlMode.PercentOutput, 0.0);
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
        SmartDashboard.putString("TURRET STATE", this.getState().getClass().getName());
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
