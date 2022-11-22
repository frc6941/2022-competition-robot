package frc.robot.auto.modes;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.auto.basics.FollowTrajectory;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.coordinators.Superstructure;

public class ATwoPlusTwoAuto extends AutoModeBase {
    protected String autoName = "A - Two Plus Two Auto";
    private final PathPlannerTrajectory trajectoryPart1 = PathPlanner.loadPath("A 2+2 Ball Auto - Part 1", 3.5, 2.5);
    private final PathPlannerTrajectory trajectoryPart2 = PathPlanner.loadPath("A 2+2 Ball Auto - Part 2", 3.5, 2.5);
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final SJTUSwerveMK5Drivebase mSwerve = SJTUSwerveMK5Drivebase.getInstance();


    @Override
    public Pose2d getStartingPose() {
        return trajectoryPart1.getInitialPose();
    };

    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                // Start Settings
                new InstantCommand(() -> mSuperstructure.setWantEject(false)),
                new InstantCommand(() -> mSuperstructure.setWantIntake(true)),
                new InstantCommand(() -> mSuperstructure.setWantMaintain(true)),
                // Part 1: Collect one cargo and shoot
                new FollowTrajectory(mSwerve, trajectoryPart1, true, true, true),
                new InstantCommand(() -> mSuperstructure.setWantIntake(false)),
                new ParallelCommandGroup(
                        new WaitUntilCommand(mSuperstructure::isReady).withTimeout(1.0),
                        new InstantCommand(() -> mSuperstructure.setState(Superstructure.STATE.SHOOTING))
                ),
                new WaitCommand(1.0),
                new InstantCommand(() -> mSuperstructure.setState(Superstructure.STATE.CHASING)),
                new InstantCommand(() -> mSuperstructure.setWantMaintain(false)),
                // Part 2: Collect two wrong cargo and spit
                new InstantCommand(() -> mSuperstructure.setWantIntake(true)),
                new InstantCommand(() -> mSwerve.setHeadingTarget(trajectoryPart2.getInitialPose().getRotation().getDegrees())),
                new InstantCommand(() -> mSwerve.setLockHeading(true)),
                new WaitUntilCommand(mSwerve::isHeadingOnTarget).withTimeout(2.0),
                new InstantCommand(() -> mSwerve.setLockHeading(false)),
                new FollowTrajectory(mSwerve, trajectoryPart2, true, false, true),
                new InstantCommand(() -> mSuperstructure.setWantIntake(false)),
                new InstantCommand(() -> mSuperstructure.setWantSpit(true)),
                new WaitCommand(1.0),
                new InstantCommand(() -> mSuperstructure.setWantIntake(false)),
                new InstantCommand(() -> mSuperstructure.setWantSpit(false)),
                // End Settings
                new InstantCommand(() -> mSuperstructure.setWantEject(true))
        );
    }

    public ATwoPlusTwoAuto() {
    }
}
