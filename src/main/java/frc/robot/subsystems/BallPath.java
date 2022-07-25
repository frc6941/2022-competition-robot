package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

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

    private LazyTalonFX feederMotor = new LazyTalonFX(Constants.CANID.FEEDER_MOTOR);

    private AnalogInput ballEntranceDetector = new AnalogInput(Constants.ANALOG_ID.BALL_ENTRANCE_DETECTOR);
    private AnalogInput ballPositionOneDetector = new AnalogInput(Constants.ANALOG_ID.BALL_POSITION_ONE_DETECTOR);
    private AnalogInput ballPositionTwoDetector = new AnalogInput(Constants.ANALOG_ID.BALL_POSITION_TWO_DETECTOR);

    private ColorSensor colorSensor = ColorSensor.getInstance();

    private static BallPath instance;

    private SITUATION situation = SITUATION.EMPTY;
    private int feederTarget = 0;
    private boolean intakeFlag = false;
    private STATE state = STATE.PROCESSING;

    private BallPath() {
        feederMotor.configFactoryDefault();
        feederMotor.setInverted(InvertType.InvertMotorOutput);
        feederMotor.setNeutralMode(NeutralMode.Brake);
        feederMotor.enableVoltageCompensation(true);
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
        return false;
    }
    

    public synchronized SITUATION getBallpathSituation(){
        return situation;
    }

    @Override
    public synchronized void read(double time, double dt){
        mPeriodicIO.breakEntrance = ballEntranceDetector.getVoltage() > 2.0;
        mPeriodicIO.breakPosition1 = ballPositionOneDetector.getVoltage() < 2.0;
        mPeriodicIO.breakPosition2 = ballPositionTwoDetector.getVoltage() < 2.0;
        mPeriodicIO.colorSensorSeesBall = colorSensor.seesBall();
    }

    @Override
    public synchronized void update(double time, double dt) {
        switch(state){
            case OFF:
                mPeriodicIO.feederDemand = 0.0;
                mPeriodicIO.feederDirection = true;
                break;
            case PROCESSING:
                if (this.ballAtEntrance() && !this.intakeFlag & situation!=SITUATION.FULL) {
                    this.intakeFlag = true;
                    if (!mPeriodicIO.breakPosition1) {
                        this.feederTarget = 1;
                    } else {
                        this.feederTarget = 2;
                    }
                }
                switch (feederTarget) {
                    case 0:
                        mPeriodicIO.feederDemand = 0.0;
                        this.intakeFlag = false;
                        break;
                    case 1:
                        mPeriodicIO.feederDemand = Constants.BALLPATH_NORMAL_PERCENTAGE;
                        if (mPeriodicIO.breakPosition1) {
                            this.feederTarget = 0;
                        }
                        break;
                    case 2:
                        mPeriodicIO.feederDemand = Constants.BALLPATH_NORMAL_PERCENTAGE;
                        if (mPeriodicIO.breakPosition2) {
                            this.feederTarget = 0;
                        }
                        break;
                }
                break;
            case FEEDING:
                mPeriodicIO.feederDemand = Constants.BALLPATH_NORMAL_PERCENTAGE;
                break;
            case SPITTING:
                mPeriodicIO.feederDemand = -Constants.BALLPATH_NORMAL_PERCENTAGE;
                break;
        }
    }

    @Override
    public synchronized void write(double time, double dt){
        feederMotor.set(ControlMode.PercentOutput, mPeriodicIO.feederDemand);
    }

    @Override
    public synchronized void telemetry(){
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

        SmartDashboard.putNumber("Feeder Speed", mPeriodicIO.feederDemand);
        SmartDashboard.putBoolean("Correct Color", colorSensor.hasCorrectColor());
        SmartDashboard.putBoolean("Sees Ball", colorSensor.seesBall());
    }

    @Override
    public synchronized void start(){
    }

    @Override
    public synchronized void stop(){
    }

    @Override
    public synchronized void disabled(double time, double dt){
    }

    public static enum SITUATION {
        FULL,
        EMPTY,
        FIRST,
        SECOND
    }

    public static enum STATE {
        OFF,
        PROCESSING,
        FEEDING,
        SPITTING
    }

    public STATE getState() {
        return this.state;
    }

    public void setState(STATE state) {
        this.state = state;
    }
}
