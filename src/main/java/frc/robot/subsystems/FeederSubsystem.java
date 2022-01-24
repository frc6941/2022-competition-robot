package frc.robot.subsystems;

import java.lang.reflect.Array;

import javax.swing.GroupLayout.Alignment;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.ColorSensorV3;

import org.frcteam2910.common.robot.UpdateManager.Updatable;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase implements Updatable {
    private TalonFX feederMotor = new TalonFX(Constants.CANID.FEEDER_MOTOR);
    private ColorSensorV3 ballColorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    private AnalogInput ballEnterDetector = new AnalogInput(Constants.ANALOG_ID.BALL_ENTER_DETECTOR_ID);
    private AnalogInput ballExitDetector = new AnalogInput(Constants.ANALOG_ID.BALL_EXIT_DETECTOR_ID);

    private static FeederSubsystem instance;
    private STATE state = STATE.IDLE;

    private boolean stepFlag = true; // A flag that shows the status of stepping. If true, the step action is
                                     // finished and
                                     // the next step can be taken. If false, the step is still in action.
    private boolean ballIntakeFlag = true; // A flag that shows the status of ball intake. If true, the intake is clear
                                           // for intake. If false, the current ball is still being processed and the
                                           // storage status should not be updated.
    private double tempRecording;
    private int loopStable;

    private FeederSubsystem() {
        feederMotor.configFactoryDefault();
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

    private boolean feederAtTarget() {
        return this.loopStable > Constants.FEEDER_END_MECHANISM_LOOPS_COUNT;
    }

    private boolean ballAtEntrance() {
        return this.ballEnterDetector.getVoltage() > 2.0;
    }

    @Override
    public void update(double time, double dt) {
        switch (state) {
            case IDLE:
                this.setFeederPercent(0.0);
                this.stepFlag = true;
                break;
            case STEPPER:
                if (this.stepFlag) {
                    this.tempRecording = feederMotor.getSelectedSensorPosition();
                    this.stepFlag = false;
                }
                this.setFeederStep(Constants.FEEDER_POSITION_CONSTANT);
                if (this.feederAtTarget()) {
                    this.setState(STATE.IDLE);
                }
                break;
            case EXPEL:
                this.setFeederPercent(1.0);
                this.stepFlag = true;
                break;
            case REVERSE:
                this.setFeederPercent(-1.0);
                this.stepFlag = true;
                break;
        }

        if (feederMotor.getClosedLoopError() < +Constants.FEEDER_FINAL_POSITION_TOLERANCE &&
                feederMotor.getClosedLoopError() > -Constants.FEEDER_FINAL_POSITION_TOLERANCE) {
            ++loopStable;
        } else {
            loopStable = 0;
        }
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
}
