package org.frcteam6941.commands.basic;

import org.frcteam6941.swerve.SwerveDrivetrainBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ZeroPositionCommand extends InstantCommand{
    public ZeroPositionCommand(SwerveDrivetrainBase drivetrainBase, Pose2d pose2d) {
        super(() -> drivetrainBase.resetOdometry(pose2d));
    }
}
