package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team254.lib.util.TimeDelayedBoolean;

import org.frcteam1678.lib.math.Conversions;
import org.frcteam6941.looper.UpdateManager.Updatable;
import org.frcteam6941.utils.LazyTalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class BallPath implements Updatable {
    public static class PeriodicIO {
        // INPUTS
        public boolean breakEntrance = false;
        public boolean breakPosition1 = false;
        public boolean breakPosition2 = false;
        public double feederCurrent = 0.0;
        public double feederVoltage = 0.0;
        public double feederVelocity = 0.0;
        public double triggerCurrent = 0.0;
        public double triggerVoltage = 0.0;
        public double triggerVelocity = 0.0;

        // OUTPUTS
        public double feederDemand = 0.0;
        public double triggerDemand = 0.0;
    }

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private LazyTalonFX feederMotor = new LazyTalonFX(Constants.CANID.FEEDER_MOTOR);
    private LazyTalonFX triggerMotor = new LazyTalonFX(Constants.CANID.TRIGGER_MOTOR);

    private AnalogInput ballEntranceDetector = new AnalogInput(Constants.ANALOG_ID.BALL_ENTRANCE_DETECTOR);
    private AnalogInput ballPositionOneDetector = new AnalogInput(Constants.ANALOG_ID.BALL_POSITION_ONE_DETECTOR);
    private AnalogInput ballPositionTwoDetector = new AnalogInput(Constants.ANALOG_ID.BALL_POSITION_TWO_DETECTOR);

    private ColorSensor colorSensor = ColorSensor.getInstance();

    private static BallPath instance;

    private boolean enableColorEject = true;
    private TimeDelayedBoolean ejectBoolean = new TimeDelayedBoolean();
    private STATE state = STATE.PROCESSING;

    private BallPath() {
        feederMotor.configFactoryDefault();
        feederMotor.setInverted(InvertType.InvertMotorOutput);
        feederMotor.setNeutralMode(NeutralMode.Brake);
        feederMotor.enableVoltageCompensation(true);

        triggerMotor.configFactoryDefault();
        triggerMotor.setNeutralMode(NeutralMode.Brake);
        triggerMotor.enableVoltageCompensation(true);
    }

    public static BallPath getInstance() {
        if (instance == null) {
            instance = new BallPath();
        }
        return instance;
    }

    public boolean ballAtEntrance() {
        return mPeriodicIO.breakEntrance;
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
            return true;
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
    
    public synchronized void eject(){
        if(getState() != STATE.EJECTING){
            setState(STATE.EJECTING);
        }
    }

    public synchronized void feed(){
        if(getState() != STATE.EJECTING){
            setState(STATE.FEEDING);
        }
    }

    public synchronized void spit(){
        setState(STATE.SPITTING);
    }

    public synchronized void process(){
        if(getState() != STATE.EJECTING){
            setState(STATE.PROCESSING);
        }
    }

    @Override
    public synchronized void read(double time, double dt) {
        mPeriodicIO.breakEntrance = ballEntranceDetector.getVoltage() > 2.0;
        mPeriodicIO.breakPosition1 = ballPositionOneDetector.getVoltage() < 2.0;
        mPeriodicIO.breakPosition2 = ballPositionTwoDetector.getVoltage() < 2.0;

        mPeriodicIO.feederCurrent = feederMotor.getStatorCurrent();
        mPeriodicIO.feederVoltage = feederMotor.getMotorOutputVoltage();
        mPeriodicIO.feederVelocity = feederMotor.getSelectedSensorVelocity();

        mPeriodicIO.triggerCurrent = triggerMotor.getStatorCurrent();
        mPeriodicIO.triggerVoltage = triggerMotor.getMotorOutputVoltage();
        mPeriodicIO.triggerVelocity = triggerMotor.getSelectedSensorVelocity();
    }

    @Override
    public synchronized void update(double time, double dt) {
        switch (state) {
            case OFF:
                mPeriodicIO.feederDemand = 0.0;
                mPeriodicIO.triggerDemand = 0.0;
                break;
            case PROCESSING:
                if (ballAtEntrance() && !isFull()) { // If there's ball at entrance and ballpath is not full
                    mPeriodicIO.feederDemand = Constants.FEEDER_NORMAL_PERCENTAGE;
                } else {
                    mPeriodicIO.feederDemand = 0.0;
                }
                mPeriodicIO.triggerDemand = Constants.TRIGGER_PASSIVE_REVERSE_VELOCITY;
                break;
            case EJECTING:
                mPeriodicIO.feederDemand = Constants.FEEDER_EJECT_PERCENTAGE;
                mPeriodicIO.triggerDemand = Constants.TRIGGER_SLOW_EJECT_VELOCITY;
                if(rightBallAtPositionTwo()){ // Right Ball
                    // Enter PROCESSING as now the right ball is at the exit
                    ejectBoolean.update(false, 0.0); // reset eject controlling boolean
                    setState(STATE.PROCESSING);
                } else if (wrongBallAtPositionTwo()) { // Wrong Ball
                    // Do nothing and continue ejecting as the wrong ball is not out yet
                    ejectBoolean.update(false, 0.0); // reset eject controlling boolean
                } else { // No Ball
                    if(ejectBoolean.update(true, Constants.BALLPATH_EXPEL_TIME)){
                        // Enter PROCESSING as now nothing is in the ball path
                        setState(STATE.PROCESSING);
                        ejectBoolean.update(false, 0.0); // reset eject controlling boolean
                    }
                }
                break;
            case FEEDING:
                if(wrongBallAtPositionTwo()){
                    setState(STATE.EJECTING);
                    break;
                }
                mPeriodicIO.feederDemand = Constants.FEEDER_NORMAL_PERCENTAGE;
                break;
            case SPITTING:
                mPeriodicIO.feederDemand = -Constants.FEEDER_NORMAL_PERCENTAGE;
                break;
        }
    }

    @Override
    public synchronized void write(double time, double dt) {
        feederMotor.set(ControlMode.PercentOutput, mPeriodicIO.feederDemand);
        triggerMotor.set(ControlMode.Velocity,
                Conversions.RPMToFalcon(mPeriodicIO.triggerDemand, Constants.TRIGGER_GEAR_RATIO));
    }

    @Override
    public synchronized void telemetry() {
        SmartDashboard.putNumber("Feeder Speed", mPeriodicIO.feederDemand);
        SmartDashboard.putBoolean("Correct Color", colorSensor.hasCorrectColor());
        SmartDashboard.putBoolean("Opposite Color", colorSensor.hasOppositeColor());
        SmartDashboard.putBoolean("Sees Ball", colorSensor.seesBall());

    }

    @Override
    public synchronized void start() {
    }

    @Override
    public synchronized void stop() {
    }

    @Override
    public synchronized void disabled(double time, double dt) {
    }

    public static enum STATE {
        OFF,
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
