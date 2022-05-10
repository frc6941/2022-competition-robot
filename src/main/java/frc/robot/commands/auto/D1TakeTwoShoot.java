package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakerSubsystem;

public class D1TakeTwoShoot extends SequentialCommandGroup {
    SJTUSwerveMK5Drivebase mDrivebase = SJTUSwerveMK5Drivebase.getInstance();
    IntakerSubsystem mIntaker = IntakerSubsystem.getInstance();

    PathPlannerTrajectory twoBallThenShoot = PathPlanner.loadPath("D1 - 2 Ball Then Shoot", 4.0, 2.0);

    public D1TakeTwoShoot(){
        addCommands(
            SimpleActions.extendIntaker,
            new WaitCommand(0.5),
            new FollowTrajectory(mDrivebase, twoBallThenShoot, true, true, true),
            SimpleActions.shoot.withTimeout(3.0)
        );
    }
}
