package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;

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
        public boolean colorSensorSeesBall = false;

        // OUTPUTS
        public double feederDemand = 0.0;
        public boolean feederDirection = true; // True means positive power
    }

    public PeriodicIO mPeriodicIO = new PeriodicIO();
    private boolean readyForWrongBallExpel = false;
    private boolean isEnabled = false;
    private boolean sawBallAtEntrance = false;

    private LazyTalonFX feederMotor = new LazyTalonFX(Constants.CANID.FEEDER_MOTOR);

    private AnalogInput ballEntranceDetector = new AnalogInput(Constants.ANALOG_ID.BALL_ENTRANCE_DETECTOR);
    private AnalogInput ballPositionOneDetector = new AnalogInput(Constants.ANALOG_ID.BALL_POSITION_ONE_DETECTOR);
    private AnalogInput ballPositionTwoDetector = new AnalogInput(Constants.ANALOG_ID.BALL_POSITION_TWO_DETECTOR);

    private ColorSensor colorSensor = ColorSensor.getInstance();

    private static BallPath instance;
    private IndexerSlot positionOneSlot = new IndexerSlot();
    private IndexerSlot positionTwoSlot = new IndexerSlot();

    private SITUATION situation = SITUATION.EMPTY;
    private SITUATION targetSituation = SITUATION.EMPTY;
    private STATE state = STATE.PROCESSING;

    private BallPath() {
        feederMotor.configFactoryDefault();
        feederMotor.setInverted(InvertType.InvertMotorOutput);
        feederMotor.setNeutralMode(NeutralMode.Brake);
        feederMotor.enableVoltageCompensation(true);
        feederMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        feederMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
    }

    public static BallPath getInstance() {
        if (instance == null) {
            instance = new BallPath();
        }
        return instance;
    }

    public boolean ballAtEntrance(){
        return mPeriodicIO.breakEntrance;
    }

    public boolean ballAtPosition1(){
        return mPeriodicIO.breakPosition1;
    }

    public boolean ballAtPosition2(){
        return mPeriodicIO.breakPosition2;
    }

    public synchronized boolean wrongBallAtPositionTwo(){
        return positionTwoSlot.hasBall() && !positionTwoSlot.hasCorrectColor();
    }

    public synchronized void changeIfReadyForWrongBall(boolean ready){
        readyForWrongBallExpel = ready;
    }
    
    public synchronized void queueNewBall(boolean correctColor){
        switch(situation){
            case EMPTY:
                positionOneSlot.queueBall(correctColor);
            case FIRST:
                positionOneSlot.move(positionTwoSlot);
                positionOneSlot.queueBall(correctColor);
                break;
            case SECOND:
                break;
            case FULL:
                break;
                
        }
    }

    public synchronized void updateSetPoint(){
        positionOneSlot.updateHasBall(ballAtPosition1(), isEnabled);
        positionTwoSlot.updateHasBall(ballAtPosition2(), isEnabled);

        if(positionOneSlot.hasBall() && positionTwoSlot.hasBall()){
            situation = SITUATION.FULL; // Ballpath is full
        } else if (!positionOneSlot.hasBall() && !positionTwoSlot.hasBall()){
            situation = SITUATION.EMPTY; // Ballpath is empty
        } else if (positionOneSlot.hasBall() && !positionTwoSlot.hasBall()){
            situation = SITUATION.FIRST; // Only one cargo at the first position
        } else if (!positionOneSlot.hasBall() && positionTwoSlot.hasBall()){
            situation = SITUATION.SECOND; // Only one cargo at the second position
        }

        switch(situation){
            case EMPTY:
                if(positionOneSlot.hasQueuedBall()){
                    targetSituation = SITUATION.FIRST;
                    mPeriodicIO.feederDirection = true;
                }
                break;
            case FIRST:
                if(positionOneSlot.hasQueuedBall()){
                    targetSituation = SITUATION.FULL;
                    mPeriodicIO.feederDirection = true;
                }
                break;
            case SECOND:
                if(wrongBallAtPositionTwo() && readyForWrongBallExpel){
                    targetSituation = SITUATION.EMPTY;
                    mPeriodicIO.feederDirection = true;
                } else {
                    positionTwoSlot.move(positionOneSlot);
                    targetSituation = SITUATION.FIRST;
                    mPeriodicIO.feederDirection = false;
                }
                
                break;
            case FULL:
                if(wrongBallAtPositionTwo() && readyForWrongBallExpel){
                    positionOneSlot.move(positionTwoSlot);
                    targetSituation = SITUATION.SECOND;
                    mPeriodicIO.feederDirection = true;
                }
                break;
        }

        if(situation == targetSituation){
            mPeriodicIO.feederDemand = 0.0;
        } else {
            mPeriodicIO.feederDemand = mPeriodicIO.feederDirection ? Constants.BALLPATH_NORMAL_PERCENTAGE : -Constants.BALLPATH_NORMAL_PERCENTAGE;
        }
    }

    @Override
    public void read(double time, double dt){
        mPeriodicIO.breakEntrance = ballEntranceDetector.getVoltage() > 2.0;
        mPeriodicIO.breakPosition1 = ballPositionOneDetector.getVoltage() < 2.0;
        mPeriodicIO.breakPosition2 = ballPositionTwoDetector.getVoltage() < 2.0;
        mPeriodicIO.colorSensorSeesBall = colorSensor.seesBall();
    }

    @Override
    public void update(double time, double dt) {
        switch(state){
            case OFF:
                mPeriodicIO.feederDemand = 0.0;
                mPeriodicIO.feederDirection = true;
                break;
            case PROCESSING:
                if(ballAtEntrance() && !sawBallAtEntrance && mPeriodicIO.colorSensorSeesBall && isEnabled && situation != SITUATION.FULL){
                    
                    queueNewBall(colorSensor.hasCorrectColor());
                    sawBallAtEntrance = true;
                    System.out.println("QUEUE NEW");
                } 
                if(!ballAtEntrance()) {
                    sawBallAtEntrance = false;
                }
                updateSetPoint();
                break;
            case FEEDING:
                mPeriodicIO.feederDemand = Constants.BALLPATH_NORMAL_PERCENTAGE;
                targetSituation = SITUATION.EMPTY;
                positionOneSlot.clearQueue();
                positionTwoSlot.clearQueue();
                break;
            case EJECTING:
                mPeriodicIO.feederDemand = -Constants.BALLPATH_NORMAL_PERCENTAGE;
                targetSituation = SITUATION.EMPTY;
                positionOneSlot.clearQueue();
                positionTwoSlot.clearQueue();
                break;
        }
    }

    @Override
    public void write(double time, double dt){
        feederMotor.set(ControlMode.PercentOutput, mPeriodicIO.feederDemand);
    }

    @Override
    public void telemetry(){
        switch(situation){
            case EMPTY:
                SmartDashboard.putNumber("Feeder State", 0);
                break;
            case FIRST:
                SmartDashboard.putNumber("Feeder State", 1);
                break;
            case SECOND:
                SmartDashboard.putNumber("Feeder State", 2);
                break;
            case FULL:
                SmartDashboard.putNumber("Feeder State", 3);
                break;
        }

        switch(targetSituation){
            case EMPTY:
                SmartDashboard.putNumber("Feeder Target State", 0);
                break;
            case FIRST:
                SmartDashboard.putNumber("Feeder Target State", 1);
                break; 
            case SECOND:
                SmartDashboard.putNumber("Feeder Target State", 2);
                break;
            case FULL:
                SmartDashboard.putNumber("Feeder Target State", 3);
                break;
        }

        SmartDashboard.putNumber("Feeder Speed", mPeriodicIO.feederDemand);
        SmartDashboard.putBoolean("Correct Color", colorSensor.hasCorrectColor());
        SmartDashboard.putBoolean("Sees Ball", colorSensor.seesBall());
    }

    @Override
    public void start(){
        isEnabled = true;
    }

    @Override
    public void stop(){
        isEnabled = false;
    }

    @Override
    public void disabled(double time, double dt){
        targetSituation = situation;
    }

    private static enum SITUATION {
        FULL,
        EMPTY,
        FIRST,
        SECOND
    }

    public static enum STATE {
        OFF,
        PROCESSING,
        FEEDING,
        EJECTING
    }

    public STATE getState() {
        return this.state;
    }

    public void setState(STATE state) {
        this.state = state;
    }

    private class IndexerSlot {
        private boolean hasBall;
        private boolean correctColor;

        private boolean hasQueuedBall;
        private boolean queuedBallColor;

        public IndexerSlot() {
            hasBall = false;
            correctColor = false;
            hasQueuedBall = false;
            queuedBallColor = false;
        }

        public boolean hasBall() {
            return hasBall;
        }

        public boolean hasCorrectColor() {
            return hasBall && correctColor;
        }

        public void updateHasBall(boolean beamBreakRead, boolean gapRequired) {
            if(gapRequired){
                if(beamBreakRead && !hasBall) {
                    hasBall = true;
                    if (hasQueuedBall) {
                        hasQueuedBall = false;
                        correctColor = queuedBallColor;
                    }
                }
                if(!beamBreakRead) {
                    hasBall = false;
                }
            } else {
                if(beamBreakRead) {
                    hasBall = true;
                }
                if(!beamBreakRead) {
                    hasBall = false;
                }
            }
        }

        public void queueBall(boolean correctColor) {
            hasQueuedBall = true;
            queuedBallColor = correctColor;
        }

        public boolean hasQueuedBall() {
            return hasQueuedBall;
        }

        public void clearQueue() {
            hasQueuedBall = false;
        }

        public void move(IndexerSlot newSlot){
            newSlot.queueBall(this.correctColor);
            this.clearQueue();
        }
    }

}
