package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import org.frcteam6941.commands.auto.WaitToNearPoint;
import org.frcteam6941.commands.basic.ZeroGyroCommand;
import org.frcteam6941.commands.basic.ZeroPositionCommand;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.utils.FollowTrajectory;
import frc.robot.subsystems.IntakerSubsystem;

public class FiveBallAuto extends SequentialCommandGroup {
    SJTUSwerveMK5Drivebase mDrivebase = SJTUSwerveMK5Drivebase.getInstance();
    IntakerSubsystem mIntaker = IntakerSubsystem.getInstance();

    PathPlannerTrajectory targetTrajectory = PathPlanner.loadPath("5 Ball Auto",
            Constants.AutoConstants.AUTO_MAX_VELOCITY,
            Constants.AutoConstants.AUTO_MAX_ACCELERATION);

    public FiveBallAuto(){
        addCommands(
            new ZeroPositionCommand(mDrivebase, targetTrajectory.getInitialPose()),
            new InstantCommand(()->mIntaker.setState(IntakerSubsystem.STATE.EXTENDING)),
            new FollowTrajectory(targetTrajectory, true)
        );
    }
}
