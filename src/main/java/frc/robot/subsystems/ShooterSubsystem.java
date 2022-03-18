package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import org.frcteam1678.lib.math.Conversions;
import org.frcteam2910.common.robot.UpdateManager.Updatable;
import org.frcteam6941.utils.LazyTalonFX;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase implements Updatable {
    private final LazyTalonFX shooterLeadMotor = new LazyTalonFX(Constants.CANID.SHOOTER_LEAD_MOTOR);
    private final LazyTalonFX shooterFollowerMotor = new LazyTalonFX(Constants.CANID.SHOOTER_FOLLOWER_MOTOR);

    private static ShooterSubsystem instance;
    private STATE state = STATE.OFF;

    public static ShooterSubsystem getInstance() {
        if (instance == null) {
            instance = new ShooterSubsystem();
        }
        return instance;
    }

    private ShooterSubsystem() {
        shooterFollowerMotor.configFactoryDefault();
        // CRUCIAL! DO NOT USE motor.follow()! This will not be effective on TalonFX and may cause the motors to lock
        shooterFollowerMotor.set(ControlMode.Follower, Constants.CANID.SHOOTER_LEAD_MOTOR);
        shooterFollowerMotor.setInverted(InvertType.OpposeMaster);
        shooterFollowerMotor.setNeutralMode(NeutralMode.Coast);
        shooterFollowerMotor.configVoltageCompSaturation(12.0, 10);
        shooterFollowerMotor.enableVoltageCompensation(true);

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
    }

    public void setShooterPercentage(double percentage){
        this.shooterLeadMotor.set(ControlMode.PercentOutput, percentage);
    }

    public void setShooterRPM(double rpm){
        this.shooterLeadMotor.set(ControlMode.Velocity, Conversions.RPMToFalcon(rpm, Constants.SHOOTER_GEAR_RATIO));
    }

    public double getShooterRPM(){
        return Conversions.falconToRPM(this.shooterLeadMotor.getSelectedSensorVelocity(), Constants.SHOOTER_GEAR_RATIO);
    }

    public double getShooterError(){
        return Conversions.falconToRPM(this.shooterLeadMotor.getClosedLoopError(), Constants.SHOOTER_GEAR_RATIO);
    }

    public boolean isHighReady(){
        return getShooterError() <= Constants.SHOOTER_ERROR_TOLERANCE && this.getState() != STATE.OFF && this.getShooterRPM() >= Constants.SHOOTER_LOW_SPEED_RPM + 500.0;
    }

    public void update(double time, double dt) {
        switch(state){
            case OFF:
                this.setShooterPercentage(0.0);
                break;
            case LOW_SPEED:
                this.setShooterRPM(Constants.SHOOTER_LOW_SPEED_RPM);
                break;
            case HIGH_SPEED:
                this.setShooterRPM(Constants.SHOOTER_HIGH_SPEED_RPM);
                break;
        }

        SmartDashboard.putNumber("RPM", this.getShooterRPM());
        SmartDashboard.putBoolean("Ready", this.isHighReady());
        SmartDashboard.putNumber("Shooter Error", this.getShooterError());
    }

    public enum STATE {
        OFF,
        LOW_SPEED,
        HIGH_SPEED
    }

    public void setState(STATE state) {
        this.state = state;
    }

    public STATE getState() {
        return this.state;
    }
}
