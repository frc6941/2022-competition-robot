package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.team254.lib.util.TimeDelayedBoolean;

import org.frcteam6941.looper.UpdateManager.Updatable;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class Intaker implements Updatable {
    public static class PeriodicIO {
        // OUTPUTS
        private double intakerDemand = 0.0;
        private DoubleSolenoid.Value intakerExtenderDemand = DoubleSolenoid.Value.kReverse;
    }

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    private DoubleSolenoid intakerExtender = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            Constants.PNEUMATICS_ID.INTAKER_EXTENDER_FORWARD, Constants.PNEUMATICS_ID.INTAKER_EXTENDER_REVERSE);

    private CANSparkMax intakerMotor = new CANSparkMax(Constants.CANID.INTAKER_MOTOR, MotorType.kBrushless);

    private static Intaker instance;
    private TimeDelayedBoolean intakerRetractTurningBoolean = new TimeDelayedBoolean();
    private boolean spin = false;
    private boolean reverse = false;
    private STATE state = STATE.RETRACTING;

    public static Intaker getInstance() {
        if (instance == null) {
            instance = new Intaker();
        }
        return instance;
    }

    private Intaker() {
        intakerMotor.restoreFactoryDefaults();
        intakerMotor.setIdleMode(IdleMode.kCoast);
        intakerMotor.enableVoltageCompensation(12.0);
        intakerMotor.setSmartCurrentLimit(25, 10);
        intakerMotor.setInverted(true);
        intakerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
        intakerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 60000);
        intakerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 60000);
    }

    public void spinIntaker(boolean spin) {
        this.spin = spin;
    }

    public void reverseIntaker(boolean reverse) {
        this.reverse = reverse;
    }

    @Override
    public synchronized void read(double time, double dt) {
    }

    @Override
    public synchronized void update(double time, double dt) {
        switch (state) {
            case EXTENDING:
                mPeriodicIO.intakerExtenderDemand = DoubleSolenoid.Value.kForward;
                if (this.spin) {
                    if (this.reverse) {
                        mPeriodicIO.intakerDemand = Constants.INTAKER_REVERSE_INTAKE_PERCENTAGE;
                    } else {
                        mPeriodicIO.intakerDemand = Constants.INTAKER_FAST_INTAKE_PERCENTAGE;
                    }
                } else {
                    mPeriodicIO.intakerDemand = 0.0;
                }
                intakerRetractTurningBoolean.update(false, 0.0);
                break;
            case RETRACTING:
                mPeriodicIO.intakerExtenderDemand = DoubleSolenoid.Value.kReverse;
                if(intakerRetractTurningBoolean.update(true, 0.1)){
                    mPeriodicIO.intakerDemand = 0.0;
                } else {
                    mPeriodicIO.intakerDemand = Constants.INTAKER_FAST_INTAKE_PERCENTAGE;
                }
                break;
        }
    }

    @Override
    public synchronized void write(double time, double dt) {
        intakerMotor.set(mPeriodicIO.intakerDemand);
        intakerExtender.set(mPeriodicIO.intakerExtenderDemand);
    }

    @Override
    public synchronized void telemetry() {
    }

    @Override
    public synchronized void start() {
    }

    @Override
    public synchronized void stop() {
        setState(STATE.RETRACTING);
    }

    @Override
    public synchronized void disabled(double time, double dt) {

    }

    public void extend() {
        if (getState() == STATE.RETRACTING) {
            setState(STATE.EXTENDING);
        }
    }

    public void retract() {
        if (getState() == STATE.EXTENDING) {
            setState(STATE.RETRACTING);
        }
    }

    public static enum STATE {
        EXTENDING,
        RETRACTING,
    }

    public STATE getState() {
        return this.state;
    }

    public void setState(STATE state) {
        this.state = state;
    }

}