package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.frcteam2910.common.robot.UpdateManager.Updatable;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakerSubsystem extends SubsystemBase implements Updatable {
    private TalonSRX intakeMotor = new TalonSRX(Constants.CANID.INTAKER_MOTOR);
    private DoubleSolenoid leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
    private DoubleSolenoid rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);

    private static IntakerSubsystem instance;
    private STATE state = STATE.OFF;

    public static IntakerSubsystem getInstance() {
        if (instance == null) {
            instance = new IntakerSubsystem();
        }
        return instance;
    }

    private IntakerSubsystem() {

    }

    @Override
    public void update(double time, double dt) {
        switch(state){
            case OFF:
                this.leftSolenoid.set(Value.kReverse);
                this.rightSolenoid.set(Value.kReverse);
                this.intakeMotor.set(ControlMode.PercentOutput, 0.0);
            case FAST_INTAKE:
                this.leftSolenoid.set(Value.kForward);
                this.rightSolenoid.set(Value.kForward);
                this.intakeMotor.set(ControlMode.PercentOutput, Constants.INTAKER_FAST_INTAKE_PERCENTAGE);
            case SLOW_INTAKE:
                this.leftSolenoid.set(Value.kForward);
                this.rightSolenoid.set(Value.kForward);
                this.intakeMotor.set(ControlMode.PercentOutput, Constants.INTAKER_SLOW_INTAKE_PERCENTAGE);
            case REVERSE:
                this.leftSolenoid.set(Value.kForward);
                this.rightSolenoid.set(Value.kForward);
                this.intakeMotor.set(ControlMode.PercentOutput, -Constants.INTAKER_FAST_INTAKE_PERCENTAGE);
        }
    }

    public enum STATE {
        OFF,
        FAST_INTAKE,
        SLOW_INTAKE,
        REVERSE
    }
}
