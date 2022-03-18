package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakerSubsystem;

public class ReadyForIntakeCommand extends CommandBase{
    IntakerSubsystem intakerSubsystem;

    public ReadyForIntakeCommand(IntakerSubsystem intaker) {
        addRequirements(intakerSubsystem);
        this.intakerSubsystem = intaker;
    }

    @Override
    public void initialize() {
        this.intakerSubsystem.setState(IntakerSubsystem.STATE.EXTENDING);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean iterrupted) {
        this.intakerSubsystem.setState(IntakerSubsystem.STATE.RETRACTING);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
