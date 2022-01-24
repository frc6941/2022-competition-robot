package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.frcteam1678.lib.math.Conversions;
import org.frcteam2910.common.robot.UpdateManager.Updatable;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase implements Updatable {

    private TalonFX turretMotor = new TalonFX(Constants.CANID.TURRET_MOTOR);

    private double forwardMaxPosition = Constants.TURRET_FORWARD_MAX_POSITION;
    private double reverseMaxPosition = Constants.TURRET_REVERSE_MAX_POSITION;
    private int calibrationStatus = 0;

    private double angleLockTarget = 0.0;
    private static TurretSubsystem instance;
    private STATE state = STATE.IDLE;

    public static TurretSubsystem getInstance() {
        if (instance == null) {
            instance = new TurretSubsystem();
        }
        return instance;
    }

    private TurretSubsystem() {
        turretMotor.configFactoryDefault();
        turretMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        turretMotor.setNeutralMode(NeutralMode.Brake);
        turretMotor.setInverted(InvertType.InvertMotorOutput);

        turretMotor.config_kP(0, 0.5);
        turretMotor.config_kD(0, 20.0);
        turretMotor.config_IntegralZone(0, 50);
    }

    public double getTurretAngle() {
        return Conversions.falconToDegrees(this.turretMotor.getSelectedSensorPosition() - ((forwardMaxPosition + reverseMaxPosition) / 2.0), Constants.TURRET_GEAR_RATIO);
    }

    /**
     * Setting the degree of the turret with limit protection.
     * 
     * @param angle The target angle relative to the zero position. Cw is positive by default.
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
        if ((delta >= 0 & !forwardSoftLimit)
                || (delta <= 0 & !reverseSoftLimit)) {
            this.turretMotor.set(ControlMode.Position,
                    Conversions.degreesToFalcon(angle, Constants.TURRET_GEAR_RATIO));
            // TODO: Implement Motion Magic Control
        } else {
            this.turretMotor.set(ControlMode.PercentOutput, 0.0);
        }
    }

    public void lockAngle(double angle) {
        this.angleLockTarget = angle;
        setState(STATE.LOCK_ANGLE);
    }

    public void stopLock(){
        setState(STATE.IDLE);
    }

    public void turnOff(){
        setState(STATE.OFF);
    }

    public boolean isOnTarget(){
        return Math.abs(this.getTurretAngle() - this.angleLockTarget) < Constants.TURRET_ERROR_TOLERANCE;
    }

    @Override
    public void update(double time, double dt) {
        if(this.turretMotor.isFwdLimitSwitchClosed() == 1){
            this.forwardMaxPosition = turretMotor.getSelectedSensorPosition();
        }
        if(this.turretMotor.isRevLimitSwitchClosed() == 1){
            this.reverseMaxPosition = turretMotor.getSelectedSensorPosition();
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
