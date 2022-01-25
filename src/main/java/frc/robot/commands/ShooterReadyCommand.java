package frc.robot.commands;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.coordinators.LauncherMechanismCoordinator;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.TurretSubsystem.STATE;

public class ShooterReadyCommand extends CommandBase {
    ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();

    public ShooterReadyCommand() {
    }

    @Override
    public void initialize() {
        this.shooterSubsystem.setState(ShooterSubsystem.STATE.HIGH_SPEED);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean iterrupted) {
        this.shooterSubsystem.setState(ShooterSubsystem.STATE.OFF);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
