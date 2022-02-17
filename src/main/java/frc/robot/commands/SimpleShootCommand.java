package frc.robot.commands;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.coordinators.LauncherMechanismCoordinator;
import frc.robot.subsystems.BallPathSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.TurretSubsystem.STATE;

public class SimpleShootCommand extends CommandBase {
    ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    BallPathSubsystem ballPathSubsystem = BallPathSubsystem.getInstance();
    private boolean shoot = false;

    public SimpleShootCommand() {
        addRequirements(shooterSubsystem, ballPathSubsystem);
    }

    @Override
    public void initialize() {
        this.shooterSubsystem.setState(ShooterSubsystem.STATE.HIGH_SPEED);
    }

    @Override
    public void execute() {
        if(this.shooterSubsystem.isReady()){
            shoot = true;
        }
        if(shoot){
            this.ballPathSubsystem.setState(BallPathSubsystem.STATE.EXPELLING);
        }
    }

    @Override
    public void end(boolean iterrupted) {
        this.shooterSubsystem.setState(ShooterSubsystem.STATE.OFF);
        this.ballPathSubsystem.setState(BallPathSubsystem.STATE.PROCESSING);
        shoot = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
