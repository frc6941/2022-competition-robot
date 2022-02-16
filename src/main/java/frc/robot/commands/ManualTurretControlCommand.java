package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.coordinators.LauncherMechanismCoordinator;
import frc.robot.subsystems.BallPathSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.TurretSubsystem.STATE;

public class ManualTurretControlCommand extends CommandBase {
    TurretSubsystem mTurretSubsystem = TurretSubsystem.getInstance();
    DoubleSupplier x;
    DoubleSupplier y;

    public ManualTurretControlCommand(DoubleSupplier x, DoubleSupplier y) {
        addRequirements(mTurretSubsystem);
        this.x = x;
        this.y = y;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (Math.abs(x.getAsDouble()) > 0.1 && Math.abs(y.getAsDouble()) > 0.1) {
            double angle = new Rotation2d(x.getAsDouble(), y.getAsDouble()).getDegrees();
            this.mTurretSubsystem.lockAngle(angle);
        }

    }

    @Override
    public void end(boolean iterrupted) {
        this.mTurretSubsystem.turnOff();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
