package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.team254.lib.util.TimeDelayedBoolean;

import org.frcteam6941.looper.UpdateManager.Updatable;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intaker extends SubsystemBase implements Updatable {
    public static class PeriodicIO {
        // OUTPUTS
        private double intakerDemand = 0.0;
        private DoubleSolenoid.Value intakerExtenderDemand = DoubleSolenoid.Value.kReverse;
        private DoubleSolenoid.Value feederExtenderDemand = DoubleSolenoid.Value.kReverse;
    }

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private DoubleSolenoid intakerExtender = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            Constants.PNEUMATICS_ID.INTAKER_EXTENDER_FORWARD, Constants.PNEUMATICS_ID.INTAKER_EXTENDER_REVERSE);

    private DoubleSolenoid feederExtender = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            Constants.PNEUMATICS_ID.FEEDER_EXTENDER_FORWARD, Constants.PNEUMATICS_ID.FEEDER_EXTENDER_REVERSE);

    private CANSparkMax intakerMotor = new CANSparkMax(Constants.CANID.INTAKER_MOTOR, MotorType.kBrushless);

    private static Intaker instance;
    private TimeDelayedBoolean tBoolean = new TimeDelayedBoolean();
    private boolean spin = false;
    private boolean reverse = false;
    private STATE state = STATE.OFF;

    public static Intaker getInstance() {
        if (instance == null) {
            instance = new Intaker();
        }
        return instance;
    }

    private Intaker(){
        intakerMotor.restoreFactoryDefaults();
        intakerMotor.setIdleMode(IdleMode.kBrake);
        intakerMotor.enableVoltageCompensation(12.0);
        intakerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
        intakerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 60000);
        intakerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 60000);
    }

    public void spinIntaker(boolean spin){
        this.spin = spin;
    }

    public void reverseIntaker(boolean reverse){
        this.reverse = reverse;
    }

    @Override
    public synchronized void read(double time, double dt){
    }

    @Override
    public synchronized void update(double time, double dt) {
        switch (state) {
            case OFF:
                mPeriodicIO.intakerExtenderDemand = DoubleSolenoid.Value.kReverse;
                mPeriodicIO.feederExtenderDemand = DoubleSolenoid.Value.kReverse;
                mPeriodicIO.intakerDemand = 0.0;
                tBoolean.update(false, 0.0);
                break;
            case EXTENDING:
                mPeriodicIO.intakerExtenderDemand = DoubleSolenoid.Value.kReverse;
                mPeriodicIO.feederExtenderDemand = DoubleSolenoid.Value.kForward;
                if (tBoolean.update(true, Constants.INTAKER_WAITING_TIME_EXTEND)) {
                    if(reverse){
                        setState(STATE.REVERSE);
                    } else {
                        setState(STATE.EXTENDED);
                    }
                }
                break;
            case EXTENDED:
                mPeriodicIO.feederExtenderDemand = DoubleSolenoid.Value.kForward;
                mPeriodicIO.intakerExtenderDemand = DoubleSolenoid.Value.kForward;
                if(this.spin){
                    mPeriodicIO.intakerDemand = Constants.INTAKER_FAST_INTAKE_PERCENTAGE;
                } else {
                    mPeriodicIO.intakerDemand = 0.0;
                }
                tBoolean.update(false, 0.0);
                break;
            case REVERSE:
                mPeriodicIO.feederExtenderDemand = DoubleSolenoid.Value.kForward;
                mPeriodicIO.intakerExtenderDemand = DoubleSolenoid.Value.kForward;
                if(this.spin){
                    mPeriodicIO.intakerDemand = -Constants.INTAKER_FAST_INTAKE_PERCENTAGE;
                } else {
                    mPeriodicIO.intakerDemand = 0.0;
                }
                tBoolean.update(false, 0.0);
                break;
            case RETRACTING:
                mPeriodicIO.feederExtenderDemand = DoubleSolenoid.Value.kForward;
                mPeriodicIO.intakerExtenderDemand = DoubleSolenoid.Value.kReverse;
                mPeriodicIO.intakerDemand = 0.0;
                if (tBoolean.update(true, Constants.INTAKER_WAITING_TIME_RETRACT)) {
                    setState(STATE.OFF);
                }
                break;
        }
    }

    @Override
    public synchronized void write(double time, double dt){
        intakerMotor.set(mPeriodicIO.intakerDemand);
        intakerExtender.set(mPeriodicIO.intakerExtenderDemand);
        feederExtender.set(mPeriodicIO.feederExtenderDemand);
    }

    @Override
    public void telemetry(){
    }

    @Override
    public void start(){
        
    }

    @Override
    public void stop(){
    }

    @Override
    public void disabled(double time, double dt){
    }

    public static enum STATE {
        OFF,
        EXTENDING,
        EXTENDED,
        REVERSE,
        RETRACTING,
    }

    public STATE getState() {
        return this.state;
    }

    public void setState(STATE state) {
        this.state = state;
    }
}
