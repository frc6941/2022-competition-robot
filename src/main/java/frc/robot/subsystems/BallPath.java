package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team254.lib.util.TimeDelayedBoolean;

import org.frcteam6941.looper.UpdateManager.Updatable;
import org.frcteam6941.utils.LazyTalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class BallPath implements Updatable {
    public static class PeriodicIO {
        // INPUTS
        public boolean breakPosition1 = false;
        public boolean breakPosition2 = false;
        public double feederCurrent = 0.0;
        public double feederVoltage = 0.0;
        public double feederVelocity = 0.0;
        public double triggerCurrent = 0.0;
        public double triggerVoltage = 0.0;
        public double triggerVelocity = 0.0;
        public double triggerPosition = 0.0;

        // OUTPUTS
        public double feederDemand = 0.0;
        public double triggerDemand = 0.0;
        public boolean triggerLock = false;
    }

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private final LazyTalonFX feederMotor = new LazyTalonFX(Constants.CANID.FEEDER_MOTOR);
    private final CANSparkMax triggerMotor = new CANSparkMax(Constants.CANID.TRIGGER_MOTOR, MotorType.kBrushless);

    private final AnalogInput ballPositionOneDetector = new AnalogInput(Constants.ANALOG_ID.BALL_POSITION_ONE_DETECTOR);
    private final AnalogInput ballPositionTwoDetector = new AnalogInput(Constants.ANALOG_ID.BALL_POSITION_TWO_DETECTOR);

    private final ColorSensor colorSensor = ColorSensor.getInstance();

    private static BallPath instance;

    private boolean enableColorEject = false;
    private boolean continueProcess = false;
    private boolean isLocking = false;
    private double lockingPositionRecorder = 0.0;
    private final TimeDelayedBoolean slowProcessBoolean = new TimeDelayedBoolean();
    private final TimeDelayedBoolean launchReverseBoolean = new TimeDelayedBoolean();
    private boolean hasReversed = false;
    private final TimeDelayedBoolean ejectBoolean = new TimeDelayedBoolean();
    private STATE state = STATE.IDLE;

    private BallPath() {
        feederMotor.configFactoryDefault();
        feederMotor.setInverted(InvertType.None);
        feederMotor.setNeutralMode(NeutralMode.Brake);
        feederMotor.enableVoltageCompensation(true);

        triggerMotor.restoreFactoryDefaults();
        triggerMotor.setIdleMode(IdleMode.kBrake);
        triggerMotor.enableVoltageCompensation(12.0);
        triggerMotor.setSmartCurrentLimit(25, 5);
        triggerMotor.setInverted(true);

        triggerMotor.getPIDController().setP(Constants.TRIGGER_KP_V_SLOT_0, 0);
        triggerMotor.getPIDController().setI(Constants.TRIGGER_KI_V_SLOT_0, 0);
        triggerMotor.getPIDController().setD(Constants.TRIGGER_KD_V_SLOT_0, 0);
        triggerMotor.getPIDController().setFF(Constants.TRIGGER_KF_V_SLOT_0, 0);
        triggerMotor.getPIDController().setIZone(Constants.TRIGGER_IZONE_SLOT_0, 0);
        triggerMotor.getPIDController().setP(Constants.TRIGGER_KP_V_SLOT_1, 1);
        triggerMotor.getPIDController().setI(Constants.TRIGGER_KI_V_SLOT_1, 1);
        triggerMotor.getPIDController().setD(Constants.TRIGGER_KD_V_SLOT_1, 1);
        triggerMotor.getPIDController().setFF(Constants.TRIGGER_KF_V_SLOT_1, 1);

        feederMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
        feederMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);

        slowProcessBoolean.update(false, 0.0);
        launchReverseBoolean.update(false, 0.0);
    }

    public static BallPath getInstance() {
        if (instance == null) {
            instance = new BallPath();
        }
        return instance;
    }

    public boolean ballAtPosition1() {
        return mPeriodicIO.breakPosition1;
    }

    public boolean ballAtPosition2() {
        return mPeriodicIO.breakPosition2;
    }

    public synchronized boolean wrongBallAtPositionTwo() {
        return enableColorEject && colorSensor.hasOppositeColor() && ballAtPosition2();
    }

    public synchronized boolean rightBallAtPositionTwo() {
        if (!enableColorEject) {
            return ballAtPosition2();
        } else {
            return colorSensor.hasCorrectColor() && ballAtPosition2();
        }
    }

    public synchronized boolean isFull() {
        return ballAtPosition1() && ballAtPosition2();
    }

    public synchronized void setEnableEject(boolean value) {
        enableColorEject = value;
    }

    public synchronized void setContinueProcess(boolean value) {
        continueProcess = value;
    }

    public synchronized boolean shouldInEject() {
        return wrongBallAtPositionTwo();
    }

    public synchronized void eject() {
        if (getState() != STATE.EJECTING && shouldInEject()) { // If not ejecting, enter ejecting
            setState(STATE.EJECTING);
        }
    }

    public synchronized void feed() {
        if (getState() != STATE.EJECTING && !wrongBallAtPositionTwo()) { // Except rejecting, enter feedering
            setState(STATE.FEEDING);
        }
    }

    public synchronized void spit() {
        setState(STATE.SPITTING);
    }

    public synchronized void backToProcess() {
        if (getState() == STATE.EJECTING || getState() == STATE.FEEDING || getState() == STATE.SPITTING) {
            setState(STATE.PROCESSING);
        }
    }

    public synchronized void goToProcess() {
        if (!isFull() && getState() == STATE.IDLE) {
            setState(STATE.PROCESSING);
        }
    }

    public synchronized void clearReverse() {
        hasReversed = false;
        launchReverseBoolean.update(false, 0.0);
    }

    @Override
    public synchronized void read(double time, double dt) {
        mPeriodicIO.breakPosition1 = ballPositionOneDetector.getVoltage() > 2.0;
        mPeriodicIO.breakPosition2 = ballPositionTwoDetector.getVoltage() > 2.0;

        mPeriodicIO.triggerPosition = triggerMotor.getEncoder().getPosition();
        mPeriodicIO.triggerVelocity = triggerMotor.getEncoder().getVelocity();
    }

    @Override
    public synchronized void update(double time, double dt) {
        // switch (state) {
        //     case IDLE:
        //         mPeriodicIO.feederDemand = 0.0;
        //         mPeriodicIO.triggerDemand = 0.0;
        //         mPeriodicIO.triggerLock = true;
        //         slowProcessBoolean.update(false, 0.0);
        //         break;
        //     case PROCESSING:
        //         if(isFull()){
        //             if (slowProcessBoolean.update(true, Constants.BALLPATH_SLOW_PROCESS_TIME)) {
        //                 setState(STATE.IDLE);
        //                 slowProcessBoolean.update(false, 0.0);
        //             } else {
        //                 mPeriodicIO.feederDemand = 0.0;
        //                 mPeriodicIO.triggerDemand = Constants.TRIGGER_REVERSING_VELOCITY;
        //             }
        //         } else {
        //             if (continueProcess) {
        //                 mPeriodicIO.feederDemand = Constants.FEEDER_FAST_PERCENTAGE;
        //                 mPeriodicIO.triggerDemand = Constants.TRIGGER_REVERSING_VELOCITY;
        //                 mPeriodicIO.triggerLock = false;
        //                 slowProcessBoolean.update(false, 0.0);
        //             } else {
        //                 if (slowProcessBoolean.update(true, Constants.BALLPATH_SLOW_PROCESS_TIME)) {
        //                     setState(STATE.IDLE);
        //                     slowProcessBoolean.update(false, 0.0);
        //                 } else {
        //                     mPeriodicIO.feederDemand = 0.0;
        //                     mPeriodicIO.triggerDemand = Constants.TRIGGER_REVERSING_VELOCITY;
        //                 }
        //             }
        //         }
        //         break;
        //     case EJECTING:
        //         if (rightBallAtPositionTwo()) { // If has a right Ball
        //             ejectBoolean.update(false, 0.0); // reset eject controlling boolean
        //             mPeriodicIO.feederDemand = 0.0;
        //             if (slowProcessBoolean.update(true, Constants.BALLPATH_SLOW_PROCESS_TIME)) {
        //                 setState(STATE.IDLE);
        //                 slowProcessBoolean.update(false, 0.0);
        //             }
        //             mPeriodicIO.feederDemand = Constants.FEEDER_SLOW_PERCENTAGE;
        //             mPeriodicIO.triggerDemand = Constants.TRIGGER_REVERSING_VELOCITY;
        //             mPeriodicIO.triggerLock = false;
        //             break;
        //         } else if (wrongBallAtPositionTwo()) { // If has a wrong Ball
        //             // Do nothing and continue ejecting as the wrong ball is not out yet
        //             ejectBoolean.update(false, 0.0); // reset eject controlling boolean
        //             mPeriodicIO.feederDemand = Constants.FEEDER_EJECT_PERCENTAGE;
        //             mPeriodicIO.triggerDemand = Constants.TRIGGER_SLOW_EJECT_VELOCITY;
        //             mPeriodicIO.triggerLock = false;
        //             break;
        //         } else { // No Ball
        //             if (ejectBoolean.update(true, Constants.BALLPATH_EXPEL_TIME)) { // Start to count down
        //                 // Enter PROCESSING after a certain period of time, as the ballpath is seen to
        //                 // be cleared
        //                 mPeriodicIO.feederDemand = 0.0;
        //                 mPeriodicIO.triggerDemand = 0.0;
        //                 mPeriodicIO.triggerLock = true;
        //                 ejectBoolean.update(false, 0.0); // reset eject controlling boolean
        //             } else {
        //                 mPeriodicIO.feederDemand = Constants.FEEDER_EJECT_PERCENTAGE;
        //                 mPeriodicIO.triggerDemand = Constants.TRIGGER_SLOW_EJECT_VELOCITY;
        //                 mPeriodicIO.triggerLock = false;
        //             }
        //             break;
        //         }
        //     case FEEDING:
        //         if (wrongBallAtPositionTwo()) { // If has a wrong Ball
        //             // stop feeding
        //             mPeriodicIO.feederDemand = -0.1;
        //             mPeriodicIO.triggerDemand = 0.0;
        //             mPeriodicIO.triggerLock = true;
        //             slowProcessBoolean.update(false, 0.0);
        //             break;
        //         }
        //         if (slowProcessBoolean.update(true, Constants.BALLPATH_SLOW_PROCESS_TIME)) {
        //             mPeriodicIO.feederDemand = Constants.FEEDER_FEED_PERCENTAGE;
        //             mPeriodicIO.triggerLock = false;
        //             mPeriodicIO.triggerDemand = Constants.TRIGGER_FEEDING_VELOCITY.get();
        //         } else {
        //             mPeriodicIO.feederDemand = Constants.FEEDER_SLOW_PERCENTAGE;
        //             mPeriodicIO.triggerDemand = Constants.TRIGGER_REVERSING_VELOCITY;
        //             mPeriodicIO.triggerLock = false;
        //         }
        //         break;
        //     case SPITTING:
        //         mPeriodicIO.feederDemand = Constants.FEEDER_SPIT_PERCENTAGE;
        //         mPeriodicIO.triggerLock = false;
        //         mPeriodicIO.triggerDemand = Constants.TRIGGER_REVERSING_VELOCITY;
        //         slowProcessBoolean.update(false, 0.0);
        //         break;
        // }
        switch (state) {
            case IDLE:
                mPeriodicIO.feederDemand = 0.0;
                mPeriodicIO.triggerDemand = 0.0;
                mPeriodicIO.triggerLock = true;
                break;
            case PROCESSING:
                if (isFull()) { // If ballpath is full of balls
                    setState(STATE.IDLE);
                    slowProcessBoolean.update(false, 0.0);
                    mPeriodicIO.feederDemand = 0.0;
                } else {
                    if (continueProcess) {
                        mPeriodicIO.feederDemand = Constants.FEEDER_FAST_PERCENTAGE;
                        mPeriodicIO.triggerDemand = 0.0;
                        mPeriodicIO.triggerLock = true;
                        slowProcessBoolean.update(false, 0.0);
                    } else {
                        if (slowProcessBoolean.update(true, Constants.BALLPATH_SLOW_PROCESS_TIME)) {
                            setState(STATE.IDLE);
                            slowProcessBoolean.update(false, 0.0);
                        }
                        mPeriodicIO.feederDemand = Constants.FEEDER_SLOW_PERCENTAGE;
                        mPeriodicIO.triggerDemand = 0.0;
                        mPeriodicIO.triggerLock = true;
                    }
                }
                break;
            case EJECTING:
                if (rightBallAtPositionTwo()) { // If has a right Ball
                    ejectBoolean.update(false, 0.0); // reset eject controlling boolean
                    mPeriodicIO.feederDemand = 0.0;
                    mPeriodicIO.triggerDemand = 0.0;
                    mPeriodicIO.triggerLock = true;
                    break;
                } else if (wrongBallAtPositionTwo()) { // If has a wrong Ball
                    // Do nothing and continue ejecting as the wrong ball is not out yet
                    ejectBoolean.update(false, 0.0); // reset eject controlling boolean
                    mPeriodicIO.feederDemand = Constants.FEEDER_EJECT_PERCENTAGE;
                    mPeriodicIO.triggerDemand = Constants.TRIGGER_SLOW_EJECT_VELOCITY;
                    mPeriodicIO.triggerLock = false;
                    break;
                } else { // No Ball
                    if (ejectBoolean.update(true, Constants.BALLPATH_EXPEL_TIME)) { // Start to count down
                        // Enter PROCESSING after a certain period of time, as the ballpath is seen to
                        // be cleared
                        mPeriodicIO.feederDemand = 0.0;
                        mPeriodicIO.triggerDemand = 0.0;
                        mPeriodicIO.triggerLock = true;
                        ejectBoolean.update(false, 0.0); // reset eject controlling boolean
                    } else {
                        mPeriodicIO.feederDemand = Constants.FEEDER_EJECT_PERCENTAGE;
                        mPeriodicIO.triggerDemand = Constants.TRIGGER_SLOW_EJECT_VELOCITY;
                        mPeriodicIO.triggerLock = false;
                    }
                    break;
                }
            case FEEDING:
                if(!hasReversed){
                    if (launchReverseBoolean.update(true, Constants.BALLPATH_FEEDING_REVERSE_TIME)){
                        hasReversed = true;
                    } else {
                        mPeriodicIO.feederDemand = Constants.FEEDER_FEED_REVERSE_PERCENTAGE;
                        mPeriodicIO.triggerLock = false;
                        mPeriodicIO.triggerDemand = Constants.TRIGGER_REVERSING_VELOCITY;
                    }
                } else {
                    if (wrongBallAtPositionTwo()) { // If has a wrong Ball
                        // stop feeding
                        mPeriodicIO.feederDemand = 0.0;
                        mPeriodicIO.triggerDemand = 0.0;
                        mPeriodicIO.triggerLock = true;
                        launchReverseBoolean.update(false, 0.0);
                        break;
                    } else {
                        mPeriodicIO.feederDemand = Constants.FEEDER_FEED_PERCENTAGE;
                        mPeriodicIO.triggerLock = false;
                        mPeriodicIO.triggerDemand = Constants.TRIGGER_FEEDING_VELOCITY;
                    }
                }
                
                break;
            case SPITTING:
                mPeriodicIO.feederDemand = Constants.FEEDER_SPIT_PERCENTAGE;
                mPeriodicIO.triggerLock = false;
                mPeriodicIO.triggerDemand = Constants.TRIGGER_REVERSING_VELOCITY;
                break;
        }

    }

    @Override
    public synchronized void write(double time, double dt) {
        feederMotor.set(ControlMode.PercentOutput, mPeriodicIO.feederDemand);
        if (mPeriodicIO.triggerLock) {
            if (!isLocking) {
                isLocking = true;
                lockingPositionRecorder = triggerMotor.getEncoder().getPosition() - 2.0;
            }
            triggerMotor.getPIDController().setReference(lockingPositionRecorder, ControlType.kPosition, 1);
        } else {
            isLocking = false;
            triggerMotor.getPIDController().setReference(mPeriodicIO.triggerDemand * Constants.TRIGGER_GEAR_RATIO,
                    ControlType.kVelocity, 0);
        }
    }

    @Override
    public synchronized void telemetry() {
        SmartDashboard.putNumber("Trigger Velocity", mPeriodicIO.triggerVelocity / Constants.TRIGGER_GEAR_RATIO);
        SmartDashboard.putNumber("Trigger Demand", mPeriodicIO.triggerDemand);
        SmartDashboard.putBoolean("Wrong Ball At Position 2", wrongBallAtPositionTwo());
    }

    @Override
    public synchronized void start() {
        setState(STATE.IDLE);
    }

    @Override
    public synchronized void stop() {
        setState(STATE.IDLE);
    }

    @Override
    public synchronized void disabled(double time, double dt) {
    }

    public enum STATE {
        IDLE,
        PROCESSING,
        FEEDING,
        SPITTING,
        EJECTING
    }

    public STATE getState() {
        return this.state;
    }

    public void setState(STATE state) {
        this.state = state;
    }
}
