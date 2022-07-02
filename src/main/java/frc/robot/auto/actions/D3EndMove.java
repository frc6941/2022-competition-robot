package frc.robot.auto.actions;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.basics.FollowTrajectory;
import frc.robot.auto.basics.SimpleActions;
import frc.robot.subsystems.Intaker;

public class D3EndMove extends SequentialCommandGroup{
    SJTUSwerveMK5Drivebase mDrivebase = SJTUSwerveMK5Drivebase.getInstance();
    Intaker mIntaker = Intaker.getInstance();

    PathPlannerTrajectory endMove = PathPlanner.loadPath("D3 - End Move", 4.0, 2.0);

    public D3EndMove(){
        addCommands(
            SimpleActions.retractIntaker,
            new FollowTrajectory(mDrivebase, endMove, true, false, true)
        );
    }
}
