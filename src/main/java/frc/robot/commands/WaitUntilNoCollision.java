package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldConstants;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.TargetSelector;
import frc.robot.utils.AllianceFlipUtil;

public class WaitUntilNoCollision extends SequentialCommandGroup{
    public static final double minDriveX = FieldConstants.Grids.outerX + 0.7;
    public WaitUntilNoCollision(Supplier<Pose2d> pose, ArmAndExtender mSuperstructure, Intaker mIntaker, TargetSelector mTargetSelector) {
        addCommands(
            new WaitUntilCommand(() -> AllianceFlipUtil.apply(pose.get()).getX() > minDriveX)
        );
    }
}
