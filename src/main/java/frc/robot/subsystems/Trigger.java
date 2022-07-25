package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.frcteam1678.lib.math.Conversions;
import org.frcteam6941.looper.UpdateManager.Updatable;
import org.frcteam6941.utils.LazyTalonFX;

import frc.robot.Constants;

public class Trigger implements Updatable{
    public static class PeriodicIO {
        // INPUT
        public double triggerVoltage = 0.0;
        public double triggerCurrent = 0.0;
        public double triggerVelocity = 0.0;
    
        // OUTPUT
        public double triggerDemand = 0.0;
    }
    
    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private LazyTalonFX triggerMotor = new LazyTalonFX(Constants.CANID.TRIGGER_MOTOR);

    private static Trigger instance;
    private STATE state = STATE.IDLE;

    public static Trigger getInstance() {
        if (instance == null) {
            instance = new Trigger();
        }
        return instance;
    }
    
    private Trigger() {
        triggerMotor.configFactoryDefault();
    }

    public double getTriggerVelocity(){
        return Conversions.falconToRPM(mPeriodicIO.triggerVelocity, Constants.TRIGGER_GEAR_RATIO);
    }

    @Override
    public synchronized void read(double time, double dt){
        mPeriodicIO.triggerCurrent = triggerMotor.getStatorCurrent();
        mPeriodicIO.triggerVoltage = triggerMotor.getMotorOutputVoltage();
        mPeriodicIO.triggerVelocity = triggerMotor.getSelectedSensorVelocity();
    }
    
    @Override
    public synchronized void update(double time, double dt){
        switch(state){
            case IDLE:
                mPeriodicIO.triggerDemand = 0.0;
                break;
            case PASSIVE_REVERSING:
                mPeriodicIO.triggerDemand = Constants.TRIGGER_PASSIVE_REVERSE_VELOCITY;
                break;
            case FEEDING:
                mPeriodicIO.triggerDemand = Constants.TRIGGER_FEEDING_VELOCITY;
                break;
            case SLOW_FEEDING:
                mPeriodicIO.triggerDemand = Constants.TRIGGER_SLOW_FEEDING_VELOCITY;
                break;
            case REVERSING:
                mPeriodicIO.triggerDemand = Constants.TRIGGER_REVERSING_VELOCITY;
                break;
        }
    }
    
    @Override
    public synchronized void write(double time, double dt){
        triggerMotor.set(ControlMode.Velocity, Conversions.RPMToFalcon(mPeriodicIO.triggerDemand, Constants.TRIGGER_GEAR_RATIO));
    }
    
    @Override
    public synchronized void telemetry(){
        // Auto Generated Method
    }
    
    @Override
    public synchronized void start(){
        // Auto Generated Method
    }
    
    @Override
    public synchronized void stop(){
        // Auto Generated Method
    }
    
    @Override
    public synchronized void disabled(double time, double dt){
        // Auto Generated Method
    }

    public enum STATE {
        IDLE,
        PASSIVE_REVERSING,
        FEEDING,
        SLOW_FEEDING,
        REVERSING
    }

    public void setState(STATE state) {
        this.state = state;
    }

    public STATE getState() {
        return this.state;
    }
}
