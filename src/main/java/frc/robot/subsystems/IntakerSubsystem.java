package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team254.lib.util.TimeDelayedBoolean;

import org.frcteam2910.common.robot.UpdateManager.Updatable;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakerSubsystem extends SubsystemBase implements Updatable {
    private DoubleSolenoid intakerExtender = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            Constants.PNEUMATICS_ID.INTAKER_EXTENDER_FORWARD, Constants.PNEUMATICS_ID.INTAKER_EXTENDER_REVERSE);

    private DoubleSolenoid feederExtender = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            Constants.PNEUMATICS_ID.FEEDER_EXTENDER_FORWARD, Constants.PNEUMATICS_ID.FEEDER_EXTENDER_REVERSE);

    private CANSparkMax intakerMotor = new CANSparkMax(Constants.CANID.INTAKER_MOTOR, MotorType.kBrushed);

    private static IntakerSubsystem instance;
    private TimeDelayedBoolean tBoolean = new TimeDelayedBoolean();
    private STATE state = STATE.OFF;

    public static IntakerSubsystem getInstance() {
        if (instance == null) {
            instance = new IntakerSubsystem();
        }
        return instance;
    }

    private IntakerSubsystem(){
        intakerMotor.restoreFactoryDefaults();
        intakerMotor.setIdleMode(IdleMode.kBrake);
        intakerMotor.enableVoltageCompensation(12.0);
    }

    public void extendIntaker() {
        this.intakerExtender.set(DoubleSolenoid.Value.kForward);
    }

    public void retractIntaker() {
        this.intakerExtender.set(DoubleSolenoid.Value.kReverse);
    }

    public boolean isIntakerExtended() {
        return this.intakerExtender.get().equals(DoubleSolenoid.Value.kForward);
    }

    public void extendFeeder() {
        this.feederExtender.set(DoubleSolenoid.Value.kForward);
    }

    public void retractFeeder() {
        this.feederExtender.set(DoubleSolenoid.Value.kReverse);
    }

    public boolean isFeederExtended() {
        return this.feederExtender.get().equals(DoubleSolenoid.Value.kForward);
    }

    public void setIntakerPercent(double power) {
        this.intakerMotor.set(power);
    }

    @Override
    public void update(double time, double dt) {
        switch (state) {
            case OFF:
                retractIntaker();
                retractFeeder();
                break;
            case EXTENDING:
                retractIntaker();
                extendFeeder();
                if (tBoolean.update(true, Constants.INTAKER_WAITING_TIME)) {
                    extendIntaker();
                    setState(STATE.EXTENDED);
                }
                break;
            case EXTENDED:
                extendIntaker();
                extendFeeder();
                setIntakerPercent(Constants.INTAKER_FAST_INTAKE_PERCENTAGE);
                break;
            case REVERSE:
                extendIntaker();
                extendFeeder();
                setIntakerPercent(-Constants.INTAKER_FAST_INTAKE_PERCENTAGE);
                break;
            case RETRACTING:
                retractIntaker();
                extendFeeder();
                setIntakerPercent(0.0);
                if (!tBoolean.update(false, Constants.INTAKER_WAITING_TIME)) {
                    retractFeeder();
                    setState(STATE.OFF);
                }
                break;
        }

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
