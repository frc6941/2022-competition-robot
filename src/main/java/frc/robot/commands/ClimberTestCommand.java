package frc.robot.commands;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.coordinators.LauncherMechanismCoordinator;
import frc.robot.subsystems.BallPathSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.TurretSubsystem.STATE;

public class ClimberTestCommand extends CommandBase {
    ClimberSubsystem mClimberSubsystem = ClimberSubsystem.getInstance();
    double power;

    public ClimberTestCommand(double power) {
        addRequirements(mClimberSubsystem);
        this.power = power;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        this.mClimberSubsystem.setClimberPercentage(power);
        System.out.println("Clim");
    }

    @Override
    public void end(boolean iterrupted) {
        this.mClimberSubsystem.setClimberPercentage(0.0);
        System.out.println("Stop");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
