package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
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
        public double leadVoltage = 0.0;
        public double leadCurret = 0.0;
        public double followerVelocity = 0.0;
        public double followerVoltage = 0.0;
        public double followerCurret = 0.0;

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
        SupplyCurrentLimitConfiguration currentLimit = new SupplyCurrentLimitConfiguration(true, 40, 100, 0.02);

        shooterLeadMotor.configFactoryDefault();
        shooterLeadMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        shooterLeadMotor.configVoltageCompSaturation(12.0, 10);
        shooterLeadMotor.enableVoltageCompensation(true);
        shooterLeadMotor.setNeutralMode(NeutralMode.Coast);
        shooterLeadMotor.config_kF(0, Constants.SHOOTER_KF);
        shooterLeadMotor.config_kP(0, Constants.SHOOTER_KP);
        shooterLeadMotor.config_kD(0, Constants.SHOOTER_KD);
        shooterLeadMotor.configVelocityMeasurementWindow(2);
        shooterLeadMotor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
        shooterLeadMotor.configClosedloopRamp(Constants.SHOOTER_RAMP);
        shooterLeadMotor.configSupplyCurrentLimit(currentLimit);

        shooterFollowerMotor.configFactoryDefault();
        shooterFollowerMotor.setInverted(InvertType.OpposeMaster);
        shooterFollowerMotor.setNeutralMode(NeutralMode.Coast);
        shooterFollowerMotor.configVoltageCompSaturation(12.0, 10);
        shooterFollowerMotor.enableVoltageCompensation(true);
        shooterFollowerMotor.changeMotionControlFramePeriod(255);
        shooterFollowerMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        shooterFollowerMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
        shooterFollowerMotor.configSupplyCurrentLimit(currentLimit);
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

    public double getShooterRPM() {
        return Conversions.falconToRPM(mPeriodicIO.leadVelocity, Constants.SHOOTER_GEAR_RATIO);
    }

    public synchronized boolean spunUp() {
        if (mPeriodicIO.shooterDemand > 0) {
            boolean flywheelSpunUp = Util.epsilonEquals(
                    Conversions.falconToRPM(mPeriodicIO.leadVelocity, Constants.SHOOTER_GEAR_RATIO),
                    mPeriodicIO.shooterDemand,
                    Constants.SHOOTER_ERROR_TOLERANCE);
            return flywheelSpunUp;
        }
        return false;
    }

    @Override
    public void read(double time, double dt) {
        mPeriodicIO.leadCurret = shooterLeadMotor.getSupplyCurrent();
        mPeriodicIO.leadVelocity = shooterLeadMotor.getSelectedSensorVelocity();
        mPeriodicIO.leadVoltage = shooterLeadMotor.getMotorOutputVoltage();

        mPeriodicIO.followerCurret = shooterLeadMotor.getSupplyCurrent();
        mPeriodicIO.followerVelocity = shooterLeadMotor.getSelectedSensorVelocity();
        mPeriodicIO.followerVoltage = shooterLeadMotor.getMotorOutputVoltage();
    }

    @Override
    public void update(double time, double dt) {
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
    public void write(double time, double dt) {
        if (getState() == STATE.CLOSE_LOOP) {
            shooterLeadMotor.set(ControlMode.Velocity,
                    Conversions.RPMToFalcon(mPeriodicIO.shooterDemand, Constants.SHOOTER_GEAR_RATIO));
        } else {
            shooterLeadMotor.set(ControlMode.PercentOutput, mPeriodicIO.shooterDemand);
        }

        shooterFollowerMotor.set(ControlMode.Follower, Constants.CANID.SHOOTER_LEAD_MOTOR);
    }

    @Override
    public void telemetry() {
        SmartDashboard.putNumber("RPM", getShooterRPM());
        SmartDashboard.putBoolean("Ready", spunUp());
    }

    @Override
    public void stop() {
        setShooterPercentage(0.0);
    }

    @Override
    public void disabled(double time, double dt) {

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
