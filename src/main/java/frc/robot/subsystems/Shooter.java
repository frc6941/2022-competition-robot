package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.team254.lib.util.Util;

import org.frcteam1678.lib.math.Conversions;
import org.frcteam6941.looper.UpdateManager.Updatable;
import org.frcteam6941.utils.LazyTalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Shooter implements Updatable {
    public static class PeriodicIO {
        // INPUTS
        public double leadVelocity = 0.0;
        public double leadCurret = 0.0;
        public double leadTemperature = 0.0;
        public double followerVelocity = 0.0;
        public double followerCurret = 0.0;
        public double followerTemperature = 0.0;

        // OUTPUTS
        public double shooterDemand = 0.0;
    }

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private final LazyTalonFX shooterLeadMotor = new LazyTalonFX(Constants.CANID.SHOOTER_LEAD_MOTOR);
    private final LazyTalonFX shooterFollowerMotor = new LazyTalonFX(Constants.CANID.SHOOTER_FOLLOWER_MOTOR);

    private static Shooter instance;
    private STATE state = STATE.OFF;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {
        shooterLeadMotor.configFactoryDefault();
        shooterLeadMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        shooterLeadMotor.configVoltageCompSaturation(12.0);
        shooterLeadMotor.enableVoltageCompensation(true);
        shooterLeadMotor.setInverted(InvertType.InvertMotorOutput);
        shooterLeadMotor.setNeutralMode(NeutralMode.Coast);
        shooterLeadMotor.config_kF(0, Constants.SHOOTER_KF);
        shooterLeadMotor.config_kP(0, Constants.SHOOTER_KP);
        shooterLeadMotor.config_kI(0, Constants.SHOOTER_KI);
        shooterLeadMotor.config_kD(0, Constants.SHOOTER_KD);
        shooterLeadMotor.config_IntegralZone(0, Constants.SHOOTER_IZONE);
        shooterLeadMotor.configVelocityMeasurementWindow(2);
        shooterLeadMotor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
        shooterLeadMotor.configClosedloopRamp(Constants.SHOOTER_RAMP);

        shooterFollowerMotor.set(ControlMode.Follower, Constants.CANID.SHOOTER_LEAD_MOTOR);
        shooterFollowerMotor.setInverted(InvertType.FollowMaster);
        shooterFollowerMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void setShooterPercentage(double percentage) {
        if (state != STATE.OPEN_LOOP) {
            setState(STATE.OPEN_LOOP);
        }
        mPeriodicIO.shooterDemand = percentage;
    }

    public void setShooterRPM(double rpm) {
        if (state != STATE.CLOSE_LOOP) {
            setState(STATE.CLOSE_LOOP);
        }
        mPeriodicIO.shooterDemand = rpm;
    }

    public void turnOff() {
        setState(STATE.OFF);
    }

    public double getShooterRPM() {
        return Conversions.falconToRPM(mPeriodicIO.leadVelocity, Constants.SHOOTER_GEAR_RATIO);
    }

    public synchronized boolean spunUp() {
        if (mPeriodicIO.shooterDemand > 0) {
            boolean flywheelSpunUp = Util.epsilonEquals(
                    getShooterRPM(),
                    mPeriodicIO.shooterDemand,
                    Constants.SHOOTER_ERROR_TOLERANCE);
            return flywheelSpunUp;
        }
        return false;
    }

    @Override
    public synchronized void read(double time, double dt) {
        mPeriodicIO.leadCurret = shooterLeadMotor.getSupplyCurrent();
        mPeriodicIO.leadVelocity = shooterLeadMotor.getSelectedSensorVelocity();
        mPeriodicIO.leadTemperature = shooterLeadMotor.getTemperature();

        mPeriodicIO.followerCurret = shooterFollowerMotor.getSupplyCurrent();
        mPeriodicIO.followerVelocity = shooterFollowerMotor.getSelectedSensorVelocity();
        mPeriodicIO.followerTemperature = shooterFollowerMotor.getTemperature();
    }

    @Override
    public synchronized void update(double time, double dt) {
        switch (state) {
            case OFF:
                mPeriodicIO.shooterDemand = 0.0;
                break;
            case OPEN_LOOP:
                mPeriodicIO.shooterDemand = mPeriodicIO.shooterDemand <= 1.0 ? mPeriodicIO.shooterDemand : 1.0;
                break;
            case CLOSE_LOOP:
                break;
        }
    }

    @Override
    public synchronized void write(double time, double dt) {
        if (getState() == STATE.CLOSE_LOOP) {
            shooterLeadMotor.set(ControlMode.Velocity,
                    Conversions.RPMToFalcon(mPeriodicIO.shooterDemand, Constants.SHOOTER_GEAR_RATIO));
        } else {
            shooterLeadMotor.set(ControlMode.PercentOutput, mPeriodicIO.shooterDemand);
        }
    }

    @Override
    public synchronized void telemetry() {
        SmartDashboard.putNumber("Shooter RPM Real", getShooterRPM());
        SmartDashboard.putNumber("Shooter Demand", mPeriodicIO.shooterDemand);
        SmartDashboard.putNumber("Lead Temperature", mPeriodicIO.leadTemperature);
        SmartDashboard.putNumber("Follow Temperature", mPeriodicIO.followerTemperature);
        SmartDashboard.putNumber("Shooter Lead Voltage", shooterLeadMotor.getMotorOutputVoltage());
        SmartDashboard.putNumber("Shooter Follower Voltage", shooterFollowerMotor.getMotorOutputVoltage());
    }

    @Override
    public synchronized void start() {

    }

    @Override
    public synchronized void stop() {
        setShooterPercentage(0.0);
    }

    @Override
    public synchronized void disabled(double time, double dt) {

    }

    public enum STATE {
        OFF,
        OPEN_LOOP,
        CLOSE_LOOP
    }

    public void setState(STATE state) {
        this.state = state;
    }

    public STATE getState() {
        return this.state;
    }
}
