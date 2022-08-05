package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team254.lib.util.Util;

import org.frcteam1678.lib.math.Conversions;
import org.frcteam6941.looper.UpdateManager.Updatable;
import org.frcteam6941.utils.LazyTalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Hood implements Updatable{
    public static class PeriodicIO {
        // INPUT
        public double hoodCurrent;
        public double hoodPosition;
        public double hoodVelocity;
        public double hoodVoltage;
    
        // OUTPUT
        public double hoodDemand;
    }
    
    public PeriodicIO mPeriodicIO = new PeriodicIO();

    public LazyTalonFX hoodMotor = new LazyTalonFX(Constants.CANID.HOOD_MOTOR);

    private boolean isCalibrated = false;

    private Hood() {
        hoodMotor.configFactoryDefault();
        hoodMotor.setNeutralMode(NeutralMode.Coast);
        hoodMotor.config_kF(0, Constants.HOOD_KF);
        hoodMotor.config_kP(0, Constants.HOOD_KP);
        hoodMotor.config_kI(0, Constants.HOOD_KI);
        hoodMotor.config_kD(0, Constants.HOOD_KD);
    }

    private STATE state = STATE.OFF;

    public static Hood getInstance() {
        if (instance == null) {
            instance = new Hood();
        }
        return instance;
    }
    private static Hood instance;

    public synchronized void resetHood(double angle){
        hoodMotor.setSelectedSensorPosition(
            Conversions.degreesToFalcon(angle, Constants.HOOD_GEAR_RATIO)
        );
        isCalibrated = true;
    }

    public synchronized void setHoodPercentage(double power){
        if(getState() != STATE.PERCENTAGE && getState() != STATE.HOMING){
            setState(STATE.PERCENTAGE);
        }
        mPeriodicIO.hoodDemand = power;
    }

    public synchronized void setHoodAngle(double angle){
        if(getState() != STATE.ANGLE && getState() != STATE.HOMING){
            setState(STATE.ANGLE);
        }
        angle = Util.clamp(angle, Constants.HOOD_MINIMUM_ANGLE, Constants.HOOD_MAXIMUM_ANGLE);
        mPeriodicIO.hoodDemand = angle;
    }

    public synchronized double getHoodAngle() {
        return Conversions.falconToDegrees(mPeriodicIO.hoodPosition, Constants.HOOD_GEAR_RATIO);
    }

    public synchronized boolean isCalibrated() {
        return isCalibrated;
    }

    @Override
    public synchronized void read(double time, double dt){
        mPeriodicIO.hoodCurrent = hoodMotor.getStatorCurrent();
        mPeriodicIO.hoodVoltage = hoodMotor.getMotorOutputVoltage();
        mPeriodicIO.hoodPosition = hoodMotor.getSelectedSensorPosition();
        mPeriodicIO.hoodVelocity = hoodMotor.getSelectedSensorVelocity();
    }
    
    @Override
    public synchronized void update(double time, double dt){
        if(!isCalibrated){
            setState(STATE.HOMING);
        }

        switch(state){
            case HOMING:
                if(isCalibrated){
                    setState(STATE.OFF);
                }
                if(mPeriodicIO.hoodCurrent > Constants.HOOD_HOMING_CURRENT_THRESHOLD){
                    resetHood(Constants.HOOD_MINIMUM_ANGLE);
                    isCalibrated = true;
                    setState(STATE.OFF);
                }
                break;
            case PERCENTAGE:
                break;
            case ANGLE:
                break;
            case OFF:
                break;
        }
    }
    
    @Override
    public synchronized void write(double time, double dt){
        switch(state){
            case HOMING:
                hoodMotor.set(ControlMode.PercentOutput, -0.3);
                break;
            case ANGLE:
                hoodMotor.set(ControlMode.MotionMagic, Conversions.degreesToFalcon(mPeriodicIO.hoodDemand, Constants.HOOD_GEAR_RATIO));
                break;
            case PERCENTAGE:
                hoodMotor.set(ControlMode.PercentOutput, mPeriodicIO.hoodDemand);
                break;
            case OFF:
                hoodMotor.set(ControlMode.PercentOutput, 0.0);
                break;
        }
    }
    
    @Override
    public synchronized void telemetry(){
        SmartDashboard.putNumber("Hood Demand", mPeriodicIO.hoodDemand);
        SmartDashboard.putNumber("Hood Position", mPeriodicIO.hoodPosition);
        SmartDashboard.putNumber("Hood Angle", getHoodAngle());
        SmartDashboard.putNumber("Hood Current", mPeriodicIO.hoodCurrent);
    }
    
    @Override
    public synchronized void start(){
    }
    
    @Override
    public synchronized void stop(){
        isCalibrated = false;
    }
    
    @Override
    public synchronized void disabled(double time, double dt){
        // Auto Generated Method
    }

    public static enum STATE {
        OFF,
        HOMING,
        PERCENTAGE,
        ANGLE
    }

    public void setState(STATE state) {
        this.state = state;
    }

    public STATE getState() {
        return this.state;
    }
}
