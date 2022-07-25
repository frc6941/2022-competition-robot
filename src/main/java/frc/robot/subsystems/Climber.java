package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.team254.lib.util.TimeDelayedBoolean;
import com.team254.lib.util.Util;

import org.frcteam1678.lib.math.Conversions;
import org.frcteam6941.looper.UpdateManager.Updatable;
import org.frcteam6941.utils.LazyTalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Climber implements Updatable {
    public static class PeriodicIO {
        // INPUT
        public boolean limitSwith = false;
        public double climberPosition = 0.0;
        public double climberVoltage = 0.0;
        public double climberCurret = 0.0;
        public double climberVelocity = 0.0;

        public DoubleSolenoid.Value climberExtenderStatus = DoubleSolenoid.Value.kReverse;
    
        // OUTPUT
        public double climberDemand = 0.0;
        public DoubleSolenoid.Value climberExtenderDemand = DoubleSolenoid.Value.kReverse;
    }
    
    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private LazyTalonFX climberMotor = new LazyTalonFX(Constants.CANID.CLIMBER_MOTOR);
    private DoubleSolenoid climberExtender = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            Constants.PNEUMATICS_ID.CLIMBER_EXTENDER_FORWARD,
            Constants.PNEUMATICS_ID.CLIMBER_EXTENDER_REVERSE);

    private Climber() {
        StatorCurrentLimitConfiguration currentLimit = new StatorCurrentLimitConfiguration(true, 60, 60, .2);
        climberMotor.configFactoryDefault();
        climberMotor.setNeutralMode(NeutralMode.Brake);
        climberMotor.enableVoltageCompensation(true);
        // Well, limit switch problem again. The motor will be inverted and the value will be inverted
        climberMotor.setInverted(InvertType.InvertMotorOutput);
        climberMotor.configClearPositionOnLimitF(true, 10); // Set position of the elevator to zero when it reaches
                                                            // zero. So, no recording of zeroing position is required.
        climberMotor.config_kP(0, Constants.CLIMBER_KP);
        climberMotor.config_kI(0, Constants.CLIMBER_KI);
        climberMotor.config_kD(0, Constants.CLIMBER_KD);
        climberMotor.config_kF(0, Constants.CLIMBER_KF);
        climberMotor.configMotionCruiseVelocity(Constants.CLIMBER_MOTION_CRUISE_VELOCITY);
        climberMotor.configMotionAcceleration(Constants.CLIMBER_MOTION_ACCELERATION);
        climberMotor.config_IntegralZone(0, 50.0);
        climberMotor.configNeutralDeadband(0.04);
        climberMotor.configStatorCurrentLimit(currentLimit);
    }

    private static Climber instance;
    private boolean isClimberCalibrated = false;
    private boolean isExtended = false;
    private boolean tryToExtend = false;
    private TimeDelayedBoolean climberHookTimer = new TimeDelayedBoolean();

    private STATE state = STATE.HOMING;

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    public void setClimberPercentage(double power) {
        if(getState() != STATE.PERCENTAGE){
            setState(STATE.PERCENTAGE);
        }
        mPeriodicIO.climberDemand = power > 1.0 ? 1.0 : power;
    }

    public boolean freeToExtend(){
        return this.getClimberHeight() > Constants.CLIMBER_SAFE_EXTENSION_MINIMUM;
    }

    public void retractClimber() {
        this.isExtended = false;
        this.tryToExtend = false;
    }

    public void extendClimber(){
        this.tryToExtend = true;
    }

    public synchronized double getClimberHeight() {
        // Sensor Position -> Degrees -> Number of Rotations -> Distance Traveled.
        return Conversions.falconToDegrees(-mPeriodicIO.climberPosition,
                Constants.CLIMBER_GEAR_RATIO) / 360.0 * (Constants.CLIMBER_PULLER_DIAMETER * Math.PI);
    }

    public synchronized double getClimberVelocity() {
        return Conversions.falconToMPS(-mPeriodicIO.climberVelocity,
                Constants.CLIMBER_PULLER_DIAMETER * Math.PI, Constants.CLIMBER_GEAR_RATIO);
    }

    public synchronized boolean isClimberCalibrated() {
        return this.isClimberCalibrated;
    }

    public synchronized boolean isClimberOnTarget() {
        if(getState() == STATE.HEIGHT){
            return Util.epsilonEquals(getClimberHeight(), mPeriodicIO.climberDemand, Constants.CLIMBER_ON_TARGET_TOLERANCE);
        } else {
            return false;
        }
        
    }

    public void setClimberHeight(double height) {
        if(getState() != STATE.HEIGHT){
            setState(STATE.HEIGHT);
        }
        if(height > Constants.CLIMBER_MAX_TRAVEL_DISTANCE){
            mPeriodicIO.climberDemand = -Constants.CLIMBER_MAX_TRAVEL_DISTANCE;
        } else if (height < 0){
            mPeriodicIO.climberDemand = 0.0;
        } else {
            mPeriodicIO.climberDemand = -height;
        }
    }

    public synchronized void resetClimberPosition() {
        climberMotor.setSelectedSensorPosition(0.0);
    }

    public synchronized void setClimberExtend(){
        if(getState() != STATE.PERCENTAGE){
            setState(STATE.PERCENTAGE);
        }
        setClimberPercentage(Constants.CLIMBER_OPENLOOP_CONTROL_PERCENTAGE);
    }

    public synchronized void setExtentionHeight() {
        if(getState() != STATE.HEIGHT){
            setState(STATE.HEIGHT);
        }
        setClimberHeight(Constants.CLIMBER_EXTENSION_HEIGHT);
    }

    public synchronized void setMinimumHeight() {
        if(getState() != STATE.HEIGHT){
            setState(STATE.HEIGHT);
        }
        setClimberHeight(0.05);
    }

    @Override
    public synchronized void read(double time, double dt){
        mPeriodicIO.climberCurret = climberMotor.getStatorCurrent();
        mPeriodicIO.climberVelocity = climberMotor.getSelectedSensorVelocity();
        mPeriodicIO.climberPosition = climberMotor.getSelectedSensorPosition();
        mPeriodicIO.climberVoltage = climberMotor.getMotorOutputVoltage();

        mPeriodicIO.climberExtenderStatus = climberExtender.get();
    }
    
    @Override
    public synchronized void update(double time, double dt){
        // Calibration if the switch is closed.
        if (climberMotor.isFwdLimitSwitchClosed() == 1) {
            resetClimberPosition();
            isClimberCalibrated = true;
        }

        if(!isClimberCalibrated) {
            setState(STATE.HOMING);
        }

        if(this.tryToExtend && this.freeToExtend()){
            this.isExtended = true;
            this.tryToExtend = false;
        }

        if(this.isExtended && this.freeToExtend()){ // At good height and received signal to extend
            mPeriodicIO.climberExtenderDemand = Value.kForward;
        } else if (this.isExtended && !this.freeToExtend()) { // Not a good height to extend yet desired to extend
            if (!climberHookTimer.update(false, 1.0)){ // If climber is out, start a time delayed boolean, cotinue after fully retraction
                if(getState() == STATE.PERCENTAGE){
                    mPeriodicIO.climberDemand = 0.0;
                    mPeriodicIO.climberExtenderDemand = Value.kReverse;
                } else if (getState() == STATE.HEIGHT){
                    mPeriodicIO.climberDemand = getClimberHeight();
                    mPeriodicIO.climberExtenderDemand = Value.kReverse;
                }
            } else { // Climber fully retracted
                mPeriodicIO.climberExtenderDemand = Value.kReverse;
            }
        } else {
            mPeriodicIO.climberExtenderDemand = Value.kReverse;
        }
    }
    
    @Override
    public synchronized void write(double time, double dt){
        switch(state){
            case HOMING:
                climberMotor.set(ControlMode.PercentOutput, -0.3);
                climberExtender.set(Value.kReverse);
                break;
            case PERCENTAGE:
                climberMotor.set(ControlMode.PercentOutput, mPeriodicIO.climberDemand);
                break;
            case HEIGHT:
                this.climberMotor.set(ControlMode.MotionMagic,
                Conversions.degreesToFalcon(mPeriodicIO.climberDemand / (Constants.CLIMBER_PULLER_DIAMETER * Math.PI) * 360.0,
                        Constants.CLIMBER_GEAR_RATIO));
        }
    }
    
    @Override
    public synchronized void telemetry(){
        SmartDashboard.putBoolean("Is Climber Calibrated", this.isClimberCalibrated);
        SmartDashboard.putNumber("Elevator Height", this.getClimberHeight());
        SmartDashboard.putBoolean("Free to Extend", this.freeToExtend());
    }

    @Override
    public void start(){
        
    }
    
    @Override
    public synchronized void stop(){
        // Auto Generated Method
    }
    
    @Override
    public synchronized void disabled(double time, double dt){
        // Auto Generated Method
    }

    public static enum STATE {
        HOMING,
        PERCENTAGE,
        HEIGHT
    }

    public STATE getState() {
        return this.state;
    }

    public void setState(STATE state) {
        this.state = state;
    }

}
