package frc.robot.commands.launcher;

import java.util.function.DoubleSupplier;

import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.coordinators.Launcher;

public class AimAtGuessAngleCommand extends CommandBase {
    Launcher mLauncher;
    DoubleSupplier rotationX;
    DoubleSupplier rotationY;

    public AimAtGuessAngleCommand(Launcher launcher, DoubleSupplier rotationAxisX, DoubleSupplier rotationAxisY) {
        this.mLauncher = launcher;
        this.rotationX = rotationAxisX;
        this.rotationY = rotationAxisY;
    }

    @Override
    public void execute() {
        double x = this.rotationX.getAsDouble();
        double y = this.rotationY.getAsDouble();
        if (new Translation2d(x, y).norm() > 0.15) {
            this.mLauncher.aimAtFieldOrientedAngleGuess(new Rotation2d(x, y).getDegrees()); // TODO: Need to determine
                                                                                            // the direction of the base
                                                                                            // right and make sure all
                                                                                            // the turning will work
                                                                                            // properly.
        }
    }
}