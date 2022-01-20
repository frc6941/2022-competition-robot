package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.frcteam1678.lib.math.Conversions;
import org.frcteam2910.common.robot.UpdateManager.Updatable;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase implements Updatable {
    private final TalonFX shooterLeadMotor = new TalonFX(Constants.CANID.SHOOTER_LEAD_MOTOR);
    private final TalonFX shooterFollowerMotor = new TalonFX(Constants.CANID.SHOOTER_FOLLOWER_MOTOR);

    private static ShooterSubsystem instance;
    private STATE state = STATE.LOW_SPEED;

    public static ShooterSubsystem getInstance() {
        if (instance == null) {
            instance = new ShooterSubsystem();
        }
        return instance;
    }

    private ShooterSubsystem() {
        shooterFollowerMotor.follow(shooterLeadMotor);
        shooterFollowerMotor.setInverted(InvertType.OpposeMaster);
        shooterFollowerMotor.setNeutralMode(NeutralMode.Coast);

        shooterLeadMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        shooterLeadMotor.enableVoltageCompensation(true);
        shooterLeadMotor.setInverted(InvertType.None);
        shooterLeadMotor.setNeutralMode(NeutralMode.Coast);
        shooterLeadMotor.config_kF(0, Constants.SHOOTER_KF);
        shooterLeadMotor.config_kP(0, Constants.SHOOTER_KP);
        shooterLeadMotor.config_kD(0, Constants.SHOOTER_KD);
    }

    public double getShooterRPM(){
        return Conversions.falconToRPM(this.shooterLeadMotor.getSelectedSensorVelocity(), 1.0);
    }

    public double getShooterError(){
        return Conversions.falconToRPM(this.shooterLeadMotor.getClosedLoopError(), 1.0);
    }

    public boolean shooterReady(){
        return getShooterError() <= Constants.SHOOTER_ERROR_TOLERANCE;
    }

    public void update(double time, double dt) {
        switch(state){
            case OFF:
                this.shooterLeadMotor.set(ControlMode.PercentOutput, 0.0);
            case LOW_SPEED:
                this.shooterLeadMotor.set(ControlMode.Velocity, Conversions.RPMToFalcon(Constants.SHOOTER_LOW_SPEED, 1.0));
            case HIGH_SPEED:
                this.shooterLeadMotor.set(ControlMode.Velocity, Conversions.RPMToFalcon(Constants.SHOOTER_HIGH_SPEED, 1.0));
        }
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
