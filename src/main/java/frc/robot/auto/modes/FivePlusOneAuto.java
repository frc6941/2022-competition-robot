package frc.robot.auto.modes;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.basics.FollowTrajectory;
import frc.robot.coordinators.Superstructure;
import frc.robot.coordinators.Superstructure.STATE;

public class FivePlusOneAuto extends AutoModeBase{
    protected String autoName = "Five Plus One Auto";
    private PathPlannerTrajectory trajectoryPart1 = PathPlanner.loadPath("5+1 Ball Auto - Part 1", 3.5, 1.5);
    private PathPlannerTrajectory trajectoryPart2 = PathPlanner.loadPath("5+1 Ball Auto - Part 2", 3.5, 1.5);
    private PathPlannerTrajectory trajectoryPart3 = PathPlanner.loadPath("5+1 Ball Auto - Part 3", 3.5, 1.5);
    private PathPlannerTrajectory trajectoryPart4 = PathPlanner.loadPath("5+1 Ball Auto - Part 4", 3.5, 1.5);
    private Superstructure mSuperstructure = Superstructure.getInstance();
    private SJTUSwerveMK5Drivebase mSwerve = SJTUSwerveMK5Drivebase.getInstance();


    @Override
    public Pose2d getStartingPose() {
        return trajectoryPart1.getInitialPose();
    };

    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> mSuperstructure.setWantEject(false)),
            new InstantCommand(() -> mSuperstructure.setWantIntake(true)),
            new InstantCommand(() -> mSuperstructure.setWantMaintain(true)),
            new FollowTrajectory(mSwerve, trajectoryPart1, true, true, true),
            new InstantCommand(() -> mSuperstructure.setState(STATE.SHOOTING)),
            new WaitCommand(1.5),
            new InstantCommand(() -> mSuperstructure.setState(STATE.CHASING)),
            new FollowTrajectory(mSwerve, trajectoryPart2, true, false, true),
            new InstantCommand(() -> mSuperstructure.setWantSpit(true)),
            new InstantCommand(() -> mSuperstructure.setWantIntake(false)),
            new WaitCommand(1.5),
            new InstantCommand(() -> mSuperstructure.setWantSpit(false)),
            new FollowTrajectory(mSwerve, trajectoryPart3, true, false, true),
            new InstantCommand(() -> mSuperstructure.setWantIntake(true)),
            new WaitCommand(3.0),
            new FollowTrajectory(mSwerve, trajectoryPart4, true, false, true),
            new InstantCommand(() -> mSuperstructure.setState(STATE.SHOOTING)),
            new WaitCommand(1.5),
            new InstantCommand(() -> mSuperstructure.setState(STATE.CHASING)),
            new InstantCommand(() -> mSuperstructure.setWantEject(true))
        );
    };

    public FivePlusOneAuto() {
    }
}
