package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intaker;

public class D2ToTerminalWithBounceTakeTwoBackThenShoot extends SequentialCommandGroup {
    SJTUSwerveMK5Drivebase mDrivebase = SJTUSwerveMK5Drivebase.getInstance();
    Intaker mIntaker = Intaker.getInstance();

    PathPlannerTrajectory toTerminalWithBounce = PathPlanner.loadPath("D2 - To Terminal With Bounce", 4.0, 2.0);
    PathPlannerTrajectory backFromTerminal = PathPlanner.loadPath("D2 - Back From Terminal Then Shoot", 4.0, 2.0);

    public D2ToTerminalWithBounceTakeTwoBackThenShoot(){
        addCommands(
            SimpleActions.extendIntaker,
            new FollowTrajectory(mDrivebase, toTerminalWithBounce, true, false, true),
            new WaitCommand(5.0),
            new FollowTrajectory(mDrivebase, backFromTerminal, true, false, true),
            SimpleActions.shoot
        );
    }
}
