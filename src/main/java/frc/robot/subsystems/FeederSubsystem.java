package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.frcteam2910.common.robot.UpdateManager.Updatable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase implements Updatable {
    private TalonFX feederMotor = new TalonFX(Constants.CANID.FEEDER_MOTOR);

    private static FeederSubsystem instance;
    private STATE state = STATE.IDLE;

    private boolean flag = true;
    private double tempRecording;
    private int loopStable;

    private FeederSubsystem() {
        feederMotor.setInverted(InvertType.InvertMotorOutput);
        feederMotor.setNeutralMode(NeutralMode.Brake);
        feederMotor.enableVoltageCompensation(true);
        feederMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        feederMotor.setSelectedSensorPosition(0);
        tempRecording = this.feederMotor.getSelectedSensorPosition();

        // Position
        feederMotor.config_kP(0, Constants.FEEDER_POSITION_KP);
    }

    public static FeederSubsystem getInstance() {
        if (instance == null) {
            instance = new FeederSubsystem();
        }
        return instance;
    }

    public void setFeederPercent(double power) {
        this.feederMotor.set(ControlMode.PercentOutput, power);
    }

    public void setFeederStep(double step) {
        this.feederMotor.selectProfileSlot(0, 0);
        this.feederMotor.set(ControlMode.Position, this.tempRecording + step);
    }

    public boolean feederAtTarget(){
        return this.loopStable > Constants.FEEDER_END_MECHANISM_LOOPS_COUNT;
    }

    public enum STATE {
        IDLE,
        STEPPER,
        EXPEL,
        REVERSE
    }

    public STATE getState() {
        return this.state;
    }

    public void setState(STATE state) {
        this.state = state;
    }

    @Override
    public void update(double time, double dt) {
        switch (state) {
            case IDLE:
                this.setFeederPercent(0.0);
                this.flag = true;
                break;
            case STEPPER:
                if (this.flag) {
                    this.tempRecording = feederMotor.getSelectedSensorPosition();
                    this.flag = false;
                }
                this.setFeederStep(Constants.FEEDER_POSITION_CONSTANT);
                if(this.feederAtTarget()){
                    this.setState(STATE.IDLE);
                }
                break;
            case EXPEL:
                this.setFeederPercent(1.0);
                this.flag = true;
                break;
            case REVERSE:
                this.setFeederPercent(-1.0);
                this.flag = true;
                break;
        }

        if (feederMotor.getClosedLoopError() < +Constants.FEEDER_FINAL_POSITION_TOLERANCE &&
            feederMotor.getClosedLoopError() > -Constants.FEEDER_FINAL_POSITION_TOLERANCE) {
            ++loopStable;
        } else {
            loopStable = 0;
        }

        

        System.out.println(state);
    }
}
