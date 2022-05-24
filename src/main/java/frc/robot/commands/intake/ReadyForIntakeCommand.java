package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakerSubsystem;

public class ReadyForIntakeCommand extends CommandBase{
    IntakerSubsystem intakerSubsystem;

    public ReadyForIntakeCommand(IntakerSubsystem intaker) {
        this.intakerSubsystem = intaker;
        addRequirements(intakerSubsystem);
    }

    @Override
    public void initialize() {
        this.intakerSubsystem.spinIntaker(true);
        this.intakerSubsystem.setState(IntakerSubsystem.STATE.EXTENDING);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean iterrupted) {
        this.intakerSubsystem.spinIntaker(false);
        this.intakerSubsystem.setState(IntakerSubsystem.STATE.RETRACTING);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
