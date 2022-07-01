package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team254.lib.util.Util;

import org.frcteam1678.lib.math.Conversions;
import org.frcteam6941.looper.UpdateManager.Updatable;
import org.frcteam6941.utils.LazyTalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Turret implements Updatable {
    public static class PeriodicIO {
        // INPUT
        public double turretCurrent = 0.0;
        public double turretVoltage = 0.0;
        public double turretPosition = 0.0;
        public boolean turretForwardLimitSwitch = false;
        public boolean turretReverseLimitSwitch = false;
    
        // OUTPUT
        public double turretDemand = 0.0;
    }
    
    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private LazyTalonFX turretMotor = new LazyTalonFX(Constants.CANID.TURRET_MOTOR);

    private double forwardMaxPosition = Constants.TURRET_FORWARD_MAX_POSITION;
    private double reverseMaxPosition = Constants.TURRET_REVERSE_MAX_POSITION;

    private double angleLockTarget = 0.0;

    private boolean isForwardCalibrated = false;
    private boolean isReverseCalibrated = false;

    private static Turret instance;
    private STATE state = STATE.OFF;

    public static Turret getInstance() {
        if (instance == null) {
            instance = new Turret();
        }
        return instance;
    }

    private Turret() {
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

    public synchronized double getTurretAngle() {
        // As cw is the true positive and ccw is what the motor thinks is positive, the
        // angle read by the sensor need to be reverted again.
        return -Conversions.falconToDegrees(
                mPeriodicIO.turretPosition - ((forwardMaxPosition + reverseMaxPosition) / 2.0),
                Constants.TURRET_GEAR_RATIO);
    }

    public synchronized Translation2d getTurretToDrivetrainTranslation(){
        Rotation2d turretRotation = Rotation2d.fromDegrees(getTurretAngle() - 180.0 + 90.0);
        return Constants.VisionConstants.Turret.TURRET_RING_CENTER_TO_ROBOT_CENTER.plus(
            new Translation2d(Constants.VisionConstants.Turret.TURRET_RING_RADIUS, turretRotation)
        );
    }

    public void setTurretPercentage(double power) {
        if(getState() != STATE.PERCENTAGE){
            setState(STATE.PERCENTAGE);
        }
        mPeriodicIO.turretDemand = -power;
    }

    public void setTurretAngle(double angle) {
        if(getState() != STATE.ANGLE){
            setState(STATE.ANGLE);
        }
        mPeriodicIO.turretDemand = angle;
    }

    public void turnOff() {
        setState(STATE.OFF);
    }

    public synchronized boolean isOnTarget() {
        if(getState() == STATE.ANGLE){
            return Util.epsilonEquals(getTurretAngle(), mPeriodicIO.turretDemand, Constants.TURRET_ERROR_TOLERANCE);
        } else {
            return false;
        }
        
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

    public boolean forwardSafeWithZone(double zone){
        zone = Math.abs(zone);
        return getTurretAngle() <= Constants.TURRET_MAX_ROTATION_DEGREE - zone;
    }

    public boolean reverseSafeWithZone(double zone){
        zone = Math.abs(zone);
        return getTurretAngle() >= - Constants.TURRET_MAX_ROTATION_DEGREE + zone;
    }

    @Override
    public synchronized void read(double time, double dt){
        mPeriodicIO.turretCurrent = turretMotor.getStatorCurrent();
        mPeriodicIO.turretVoltage = turretMotor.getMotorOutputVoltage();
        mPeriodicIO.turretPosition = turretMotor.getSelectedSensorPosition();
        mPeriodicIO.turretForwardLimitSwitch = turretMotor.isFwdLimitSwitchClosed() == 1;
        mPeriodicIO.turretReverseLimitSwitch = turretMotor.isRevLimitSwitchClosed() == 1;
    }
    
    @Override
    public void update(double time, double dt) {
        
        turretMotor.setNeutralMode(NeutralMode.Brake);

        if (mPeriodicIO.turretForwardLimitSwitch) {
            forwardMaxPosition = mPeriodicIO.turretPosition;
            isForwardCalibrated = true;
            turretMotor.configForwardSoftLimitThreshold(forwardMaxPosition);
        }
        if (mPeriodicIO.turretReverseLimitSwitch) {
            reverseMaxPosition = mPeriodicIO.turretPosition;
            isReverseCalibrated = true;
            turretMotor.configReverseSoftLimitThreshold(reverseMaxPosition);
        }

        // Carry out calibration according to sensor status. Reverse and forward must
        // both be calibrated either by hand or through motor before further actions.
        switch(state){
            case HOMING:
                if (this.isForwardCalibrated && this.isReverseCalibrated) {
                    setState(STATE.OFF);
                } else if (!this.isForwardCalibrated && !this.isReverseCalibrated) {
                    mPeriodicIO.turretDemand = 0.2;
                } else if (this.isForwardCalibrated && !this.isReverseCalibrated) {
                    mPeriodicIO.turretDemand = -0.2;
                } else if (!this.isForwardCalibrated && this.isReverseCalibrated) {
                    mPeriodicIO.turretDemand = 0.2;
                }
                break;
            case PERCENTAGE:
                break;
            case ANGLE:
                break;
            case OFF:
                mPeriodicIO.turretDemand = 0.0;
        }
    }
    
    @Override
    public synchronized void write(double time, double dt){
        switch(state){
            case HOMING:
                turretMotor.set(ControlMode.PercentOutput, mPeriodicIO.turretDemand);
                break;
            case ANGLE:
                double angle = mPeriodicIO.turretDemand;
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
                        turretMotor.set(ControlMode.MotionMagic,
                                Conversions.degreesToFalcon(-angle, Constants.TURRET_GEAR_RATIO)
                                        + (forwardMaxPosition + reverseMaxPosition) / 2.0);
                    }
                } else {
                    turretMotor.set(ControlMode.MotionMagic,
                            Conversions.degreesToFalcon(-Math.copySign(90.0, angle), Constants.TURRET_GEAR_RATIO)
                                    + (forwardMaxPosition + reverseMaxPosition) / 2.0);
                }
                break;
            case PERCENTAGE:
                turretMotor.set(ControlMode.PercentOutput, mPeriodicIO.turretDemand);
                break;
            case OFF:
                turretMotor.set(ControlMode.PercentOutput, 0.0);
                break;
        }
    }
    
    @Override
    public synchronized void telemetry(){
        SmartDashboard.putNumber("Forward Max Position", this.forwardMaxPosition);
        SmartDashboard.putNumber("Reverse Max Position", this.reverseMaxPosition);
        SmartDashboard.putNumber("Current Position", this.turretMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Current Angle", this.getTurretAngle());
        SmartDashboard.putNumber("Lock Angle", this.angleLockTarget);
        SmartDashboard.putBoolean("Forward Safe", this.forwardSafe());
        SmartDashboard.putBoolean("Reverse Safe", this.reverseSafe());
    }
    
    @Override
    public synchronized void stop(){
        turnOff();
    }
    
    @Override
    public synchronized void disabled(double time, double dt){
        if(Math.abs(getTurretAngle()) < Constants.TURRET_SAFE_ZONE_DEGREE){
            this.turretMotor.setNeutralMode(NeutralMode.Coast);
        }
    }

    public enum STATE {
        OFF,
        HOMING,
        ANGLE,
        PERCENTAGE
    }

    public void setState(STATE state) {
        this.state = state;
    }

    public STATE getState() {
        return this.state;
    }
}
