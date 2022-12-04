package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.team254.lib.util.MovingAverage;
import com.team254.lib.util.Util;

import org.frcteam1678.lib.math.Conversions;
import org.frcteam6941.looper.UpdateManager.Updatable;
import org.frcteam6941.utils.LazyTalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Turret implements Updatable {
    public static class PeriodicIO {
        // INPUT
        public double turretCurrent = 0.0;
        public double turretPosition = 0.0;
        public double turretVelocity = 0.0;
        public boolean turretForwardLimitSwitch = false;
        public boolean turretReverseLimitSwitch = false;

        // OUTPUT
        public double turretDemand = 0.0;
        public double turretFeedforward = 0.0;
    }

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private final LazyTalonFX turretMotor = new LazyTalonFX(Constants.CANID.TURRET_MOTOR);

    private double zeroPosition = 0.0;
    private final MovingAverage feedforwardMovingAverage = new MovingAverage(7);

    private boolean isCalibrated = false;

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
        turretMotor.setNeutralMode(NeutralMode.Coast);
        turretMotor.config_kP(0, Constants.TURRET_KP);
        turretMotor.config_kI(0, Constants.TURRET_KI);
        turretMotor.config_kD(0, Constants.TURRET_KD);
        turretMotor.config_kF(0, Constants.TURRET_KF);
        turretMotor.configMotionCruiseVelocity(Constants.TURRET_MOTION_CRUISE_VELOCITY);
        turretMotor.config_IntegralZone(0, Constants.TURRET_INTEGRAL_ZONE);
        turretMotor.configMotionAcceleration(Constants.TURRET_MOTION_ACCELERATION);
        turretMotor.enableVoltageCompensation(true);
        turretMotor.configNeutralDeadband(Constants.TURRET_NEUTRAL_DEADBAND);
        turretMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 40, 0.01));
    }

    public synchronized double getTurretAngle() {
        return Conversions.falconToDegrees(
                mPeriodicIO.turretPosition - zeroPosition,
                Constants.TURRET_GEAR_RATIO);
    }

    public void setTurretPercentage(double power) {
        if (getState() != STATE.PERCENTAGE && getState() != STATE.HOMING) {
            setState(STATE.PERCENTAGE);
        }
        mPeriodicIO.turretDemand = power;
    }

    public void setTurretAngle(double angle) {
        if (getState() != STATE.ANGLE && getState() != STATE.HOMING) {
            setState(STATE.ANGLE);
        }
        mPeriodicIO.turretDemand = angle;
        mPeriodicIO.turretFeedforward = 0.0;
    }

    public void setTurretAngle(double angle, double drivetrainAngularVelocity, double drivetrainAngularAcceleration) {
        if (getState() != STATE.ANGLE && getState() != STATE.HOMING) {
            setState(STATE.ANGLE);
        }
        mPeriodicIO.turretDemand = angle;
        feedforwardMovingAverage.addNumber(Constants.TURRET_FEEDFORWARD.calculate(drivetrainAngularVelocity, drivetrainAngularAcceleration) / 12.0);
        mPeriodicIO.turretFeedforward = feedforwardMovingAverage.getAverage();
    }

    public synchronized boolean isOnTarget() {
        if (getState() == STATE.ANGLE) {
            return Util.epsilonEquals(getTurretAngle(), mPeriodicIO.turretDemand, Constants.TURRET_ERROR_TOLERANCE);
        } else {
            return false;
        }

    }

    public boolean isCalibrated() {
        return isCalibrated;
    }

    public boolean forwardSafe() {
        return getTurretAngle() <= Constants.TURRET_MAX_ROTATION_DEGREE;
    }

    public boolean reverseSafe() {
        return getTurretAngle() >= -Constants.TURRET_MAX_ROTATION_DEGREE;
    }

    public boolean forwardSafeWithZone(double zone) {
        zone = Math.abs(zone);
        return getTurretAngle() <= Constants.TURRET_MAX_ROTATION_DEGREE - zone;
    }

    public boolean reverseSafeWithZone(double zone) {
        zone = Math.abs(zone);
        return getTurretAngle() >= -Constants.TURRET_MAX_ROTATION_DEGREE + zone;
    }

    @Override
    public synchronized void read(double time, double dt) {
        mPeriodicIO.turretCurrent = turretMotor.getStatorCurrent();
        mPeriodicIO.turretPosition = turretMotor.getSelectedSensorPosition();
        mPeriodicIO.turretVelocity = turretMotor.getSelectedSensorVelocity();
        mPeriodicIO.turretForwardLimitSwitch = turretMotor.isFwdLimitSwitchClosed() == 1;
        mPeriodicIO.turretReverseLimitSwitch = turretMotor.isRevLimitSwitchClosed() == 1;
    }

    @Override
    public void update(double time, double dt) {
        if (mPeriodicIO.turretForwardLimitSwitch && mPeriodicIO.turretReverseLimitSwitch) {
            zeroPosition = mPeriodicIO.turretPosition - Constants.TURRET_REVERSE_TO_CENTER_TRAVEL_DISTANCE;

            turretMotor.configForwardSoftLimitThreshold(zeroPosition
                    + Conversions.degreesToFalcon(Constants.TURRET_MAX_ROTATION_DEGREE, Constants.TURRET_GEAR_RATIO));
            turretMotor.configReverseSoftLimitThreshold(zeroPosition
                    - Conversions.degreesToFalcon(Constants.TURRET_MAX_ROTATION_DEGREE, Constants.TURRET_GEAR_RATIO));
            isCalibrated = true;
            turretMotor.configForwardSoftLimitEnable(true);
            turretMotor.configReverseSoftLimitEnable(true);
        }

        // Carry out calibration according to sensor status. Reverse and forward must
        // both be calibrated either by hand or through motor before further actions.
        if (!isCalibrated()) {
            setState(STATE.HOMING);
        }
        switch (state) {
            case HOMING:
                if (this.isCalibrated) {
                    setState(STATE.OFF);
                } else {
                    mPeriodicIO.turretDemand = 0.0;
                }
                break;
            case PERCENTAGE:
                break;
            case ANGLE:
                double angle = mPeriodicIO.turretDemand;
                while (angle > 180.0) {
                    angle -= 360.0;
                }
                while (angle < -180.0) {
                    angle += 360.0;
                }

                // Determine if the set angle is out of reach. If so, set the angle to
                // the reachable maximum or minimum.
                if (Math.abs(angle) < Constants.TURRET_MAX_ROTATION_DEGREE) {
                    turretMotor.set(ControlMode.MotionMagic,
                            Conversions.degreesToFalcon(angle, Constants.TURRET_GEAR_RATIO) + zeroPosition,
                            DemandType.ArbitraryFeedForward, mPeriodicIO.turretFeedforward);
                } else {
                    turretMotor.set(ControlMode.MotionMagic,
                            Conversions.degreesToFalcon(Math.copySign(Constants.TURRET_MAX_ROTATION_DEGREE, angle),
                                    Constants.TURRET_GEAR_RATIO) + zeroPosition,
                            DemandType.ArbitraryFeedForward, mPeriodicIO.turretFeedforward);
                }
                break;
            case OFF:
                mPeriodicIO.turretDemand = 0.0;
        }
    }

    @Override
    public synchronized void write(double time, double dt) {
    }

    @Override
    public synchronized void telemetry() {
        SmartDashboard.putNumber("Turret Raw Velocity", mPeriodicIO.turretVelocity);
    }

    @Override
    public synchronized void start() {
        turretMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public synchronized void stop() {
        setState(STATE.OFF);
        turretMotor.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public synchronized void disabled(double time, double dt) {
        setState(STATE.OFF);
        turretMotor.setNeutralMode(NeutralMode.Coast);
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
