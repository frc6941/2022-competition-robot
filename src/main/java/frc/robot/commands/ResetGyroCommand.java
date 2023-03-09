package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;
import frc.robot.utils.AllianceFlipUtil;

public class ResetGyroCommand extends InstantCommand {
    public ResetGyroCommand(SJTUSwerveMK5Drivebase mDrivebase, Rotation2d angle) {
        super(() -> {
            mDrivebase.resetYaw(AllianceFlipUtil.apply(angle).getDegrees());
            mDrivebase.resetPose(new Pose2d(mDrivebase.getLocalizer().getLatestPose().getTranslation(), AllianceFlipUtil.apply(angle)));
        });
    }
}
