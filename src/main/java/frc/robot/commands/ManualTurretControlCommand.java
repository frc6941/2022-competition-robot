package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class ManualTurretControlCommand extends CommandBase {
    TurretSubsystem mTurretSubsystem;
    DoubleSupplier x;
    DoubleSupplier y;

    public ManualTurretControlCommand(TurretSubsystem turret, DoubleSupplier x, DoubleSupplier y) {
        addRequirements(mTurretSubsystem);
        this.mTurretSubsystem = turret;
        this.x = x;
        this.y = y;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (Math.abs(x.getAsDouble()) > 0.1 && Math.abs(y.getAsDouble()) > 0.1) {
            double angle = new Rotation2d(x.getAsDouble(), y.getAsDouble()).getDegrees() - 90.0;
            this.mTurretSubsystem.lockAngle(-angle);
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
