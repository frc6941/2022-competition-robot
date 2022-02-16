package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallPathSubsystem;
import frc.robot.subsystems.IntakerSubsystem;
import frc.robot.subsystems.ShooterSubsystem.STATE;

public class ReadyForIntakeCommand extends CommandBase{
    IntakerSubsystem intakerSubsystem = IntakerSubsystem.getInstance();

    public ReadyForIntakeCommand() {
        addRequirements(intakerSubsystem);
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
