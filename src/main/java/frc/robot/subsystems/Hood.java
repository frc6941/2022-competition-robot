package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team254.lib.util.Util;

import org.frcteam1678.lib.math.Conversions;
import org.frcteam6941.looper.UpdateManager.Updatable;
import org.frcteam6941.utils.LazyTalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Hood implements Updatable {
    public static class PeriodicIO {
        // INPUT
        public double hoodCurrent;
        public double hoodPosition;
        public double hoodVelocity;
        public double hoodVoltage;

        // OUTPUT
        public double hoodDemand;
    }

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    public LazyTalonFX hoodMotor = new LazyTalonFX(Constants.CANID.HOOD_MOTOR);

    private boolean isCalibrated = false;

    private Hood() {
        hoodMotor.configFactoryDefault();
        hoodMotor.setNeutralMode(NeutralMode.Coast);
        hoodMotor.config_kP(0, Constants.HOOD_KP);
        hoodMotor.config_kI(0, Constants.HOOD_KI);
        hoodMotor.config_kD(0, Constants.HOOD_KD);
        hoodMotor.config_kF(0, Constants.HOOD_KF);
        hoodMotor.configMotionCruiseVelocity(Constants.HOOD_CRUISE_V);
        hoodMotor.configMotionAcceleration(Constants.HOOD_CRUISE_ACC);
        hoodMotor.enableVoltageCompensation(true);
        hoodMotor.config_IntegralZone(0, 50);
        hoodMotor.configNeutralDeadband(0.01);
    }

    private STATE state = STATE.HOMING;

    public static Hood getInstance() {
        if (instance == null) {
            instance = new Hood();
        }
        return instance;
    }

    private static Hood instance;

    public synchronized void resetHood(double angle) {
        hoodMotor.setSelectedSensorPosition(
                Conversions.degreesToFalcon(angle, Constants.HOOD_GEAR_RATIO));
        isCalibrated = true;
    }

    public synchronized void setHoodPercentage(double power) {
        if (getState() != STATE.PERCENTAGE) {
            setState(STATE.PERCENTAGE);
        }
        mPeriodicIO.hoodDemand = power;
    }

    public synchronized void setHoodAngle(double angle) {
        if (getState() != STATE.ANGLE) {
            setState(STATE.ANGLE);
        }
        angle = Util.clamp(angle, Constants.HOOD_MINIMUM_ANGLE, Constants.HOOD_MAXIMUM_ANGLE);
        mPeriodicIO.hoodDemand = angle;
    }

    public synchronized double getHoodAngle() {
        return Conversions.falconToDegrees(mPeriodicIO.hoodPosition, Constants.HOOD_GEAR_RATIO);
    }

    public synchronized boolean isCalibrated() {
        return isCalibrated;
    }

    @Override
    public synchronized void read(double time, double dt) {
        mPeriodicIO.hoodCurrent = hoodMotor.getStatorCurrent();
        mPeriodicIO.hoodPosition = hoodMotor.getSelectedSensorPosition();
        mPeriodicIO.hoodVelocity = hoodMotor.getSelectedSensorVelocity();
    }

    @Override
    public synchronized void update(double time, double dt) {
        if (!isCalibrated) {
            setState(STATE.HOMING);
        }

        switch (state) {
            case HOMING:
                mPeriodicIO.hoodDemand = -0.2;
                if (isCalibrated) {
                    setState(STATE.OFF);
                }
                if (mPeriodicIO.hoodCurrent > Constants.HOOD_HOMING_CURRENT_THRESHOLD) {
                    resetHood(Constants.HOOD_MINIMUM_ANGLE - 0.5);
                }
                break;
            case PERCENTAGE:
                break;
            case ANGLE:
                break;
            case OFF:
                break;
        }
    }

    @Override
    public synchronized void write(double time, double dt) {
        switch (state) {
            case HOMING:
                hoodMotor.set(ControlMode.PercentOutput, mPeriodicIO.hoodDemand);
                break;
            case PERCENTAGE:
                hoodMotor.set(ControlMode.PercentOutput, mPeriodicIO.hoodDemand);
                break;
            case ANGLE:
                hoodMotor.set(ControlMode.MotionMagic,
                        Conversions.degreesToFalcon(mPeriodicIO.hoodDemand, Constants.HOOD_GEAR_RATIO));
                break;
            case OFF:
                hoodMotor.set(ControlMode.PercentOutput, 0.0);
                break;
        }
    }

    @Override
    public synchronized void telemetry() {
        SmartDashboard.putNumber("Hood Demand", mPeriodicIO.hoodDemand);
        SmartDashboard.putNumber("Hood Angle", getHoodAngle());
    }

    @Override
    public synchronized void start() {
        isCalibrated = false;
        setState(STATE.HOMING);
    }

    @Override
    public synchronized void stop() {
    }

    @Override
    public synchronized void disabled(double time, double dt) {
        // Auto Generated Method
    }

    public enum STATE {
        OFF,
        HOMING,
        PERCENTAGE,
        ANGLE
    }

    public void setState(STATE state) {
        this.state = state;
    }

    public STATE getState() {
        return this.state;
    }
}
