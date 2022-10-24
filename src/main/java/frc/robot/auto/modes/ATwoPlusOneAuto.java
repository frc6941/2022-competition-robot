package frc.robot.auto.modes;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.auto.basics.FollowTrajectory;
import frc.robot.coordinators.Superstructure;
import frc.robot.coordinators.Superstructure.STATE;

import static frc.robot.auto.modes.ATwoPlusTwoAuto.getCommand;

public class ATwoPlusOneAuto extends AutoModeBase{
    protected String autoName = "A - Two Plus One Auto";
    private final PathPlannerTrajectory trajectoryPart1 = PathPlanner.loadPath("A 2+1 Ball Auto - Part 1", 3.5, 1.5);
    private final PathPlannerTrajectory trajectoryPart2 = PathPlanner.loadPath("A 2+1 Ball Auto - Part 2", 3.5, 1.5);
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final SJTUSwerveMK5Drivebase mSwerve = SJTUSwerveMK5Drivebase.getInstance();


    @Override
    public Pose2d getStartingPose() {
        return trajectoryPart1.getInitialPose();
    };

    @Override
    public Command getAutoCommand() {
        return getCommand(mSuperstructure, mSwerve, trajectoryPart1, trajectoryPart2);
    };

    public ATwoPlusOneAuto() {
    }
}
