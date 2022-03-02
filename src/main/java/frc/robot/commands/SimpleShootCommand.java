package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.coordinators.Launcher;
import frc.robot.subsystems.BallPathSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SimpleShootCommand extends CommandBase {
    ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    BallPathSubsystem ballPathSubsystem = BallPathSubsystem.getInstance();
    Launcher launcher = Launcher.getInstance();

    public SimpleShootCommand() {
        addRequirements(shooterSubsystem, ballPathSubsystem);
    }

    @Override
    public void initialize() {
        this.shooterSubsystem.setState(ShooterSubsystem.STATE.HIGH_SPEED);
    }

    @Override
    public void execute() {
        if(this.launcher.getState() == Launcher.STATE.READY){
            this.ballPathSubsystem.setState(BallPathSubsystem.STATE.EXPELLING);
        }
    }

    @Override
    public void end(boolean iterrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
