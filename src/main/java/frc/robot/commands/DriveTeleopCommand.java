package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;

public class DriveTeleopCommand extends CommandBase {
    SJTUSwerveMK5Drivebase mDrivebase;
    ArmAndExtender mSupersturcture;
    Supplier<Translation2d> translationSupplier;
    Supplier<Double> rotationSupplier;
    Supplier<Double> multiplierSupplier;
    boolean isOpenLoop;

    public DriveTeleopCommand(
            SJTUSwerveMK5Drivebase mDrivebase,
            ArmAndExtender mSuperstructure,
            Supplier<Translation2d> translationSupplier,
            Supplier<Double> rotationSupplier,
            Supplier<Double> multiplierSupplier,
            boolean isOpenLoop) {
        this.mDrivebase = mDrivebase;
        this.mSupersturcture = mSuperstructure;
        this.translationSupplier = translationSupplier;
        this.rotationSupplier = rotationSupplier;
        this.multiplierSupplier = multiplierSupplier;
        this.isOpenLoop = isOpenLoop;
        addRequirements(mDrivebase);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Translation2d translation = translationSupplier.get();
        double rotation = rotationSupplier.get();

        mDrivebase.setLockHeading(false);
        mDrivebase.drive(translation.times(multiplierSupplier.get()), rotation, true, isOpenLoop);
    }

    @Override
    public void end(boolean interrupted) {
        mDrivebase.stopMovement();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
