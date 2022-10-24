package frc.robot.auto.modes;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.auto.basics.FollowTrajectory;
import frc.robot.coordinators.Superstructure;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

public abstract class AutoModeBase {
    protected String autoName;
    protected boolean autoFinished;

    public abstract Pose2d getStartingPose();
    public abstract Command getAutoCommand();

    static Command getCommand(Superstructure mSuperstructure, SJTUSwerveMK5Drivebase mSwerve, PathPlannerTrajectory trajectoryPart1, PathPlannerTrajectory trajectoryPart2) {
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

    public AutoModeBase() {
        
    }
}
