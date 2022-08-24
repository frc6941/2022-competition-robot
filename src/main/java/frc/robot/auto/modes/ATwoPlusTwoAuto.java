package frc.robot.auto.modes;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.auto.basics.FollowTrajectory;
import frc.robot.coordinators.Superstructure;
import frc.robot.coordinators.Superstructure.STATE;

public class ATwoPlusTwoAuto extends AutoModeBase{
    protected String autoName = "A - Two Plus Two Auto";
    private PathPlannerTrajectory trajectoryPart1 = PathPlanner.loadPath("A 2+2 Ball Auto - Part 1", 3.5, 1.5);
    private PathPlannerTrajectory trajectoryPart2 = PathPlanner.loadPath("A 2+2 Ball Auto - Part 2", 3.5, 1.5);
    private Superstructure mSuperstructure = Superstructure.getInstance();
    private SJTUSwerveMK5Drivebase mSwerve = SJTUSwerveMK5Drivebase.getInstance();


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
            new WaitUntilCommand(() -> mSuperstructure.isReady()).withTimeout(0.5),
            new InstantCommand(() -> mSuperstructure.setState(STATE.SHOOTING)),
            new WaitCommand(0.8),
            new InstantCommand(() -> mSuperstructure.setState(STATE.CHASING)),
            new InstantCommand(() -> mSuperstructure.setWantMaintain(false)),
            // Part 2: Collect two wrong cargo and spit
            new InstantCommand(() -> mSuperstructure.setWantIntake(true)),
            new FollowTrajectory(mSwerve, trajectoryPart2, true, false, true),
            new InstantCommand(() -> mSuperstructure.setWantIntake(false)),
            new InstantCommand(() -> mSuperstructure.setWantSpit(true)),
            new WaitCommand(1.0),
            new InstantCommand(() -> mSuperstructure.setWantIntake(false)),
            new InstantCommand(() -> mSuperstructure.setWantSpit(false)),
            // End Settings
            new InstantCommand(() -> mSuperstructure.setWantEject(true))
        );
    };

    public ATwoPlusTwoAuto() {
    }
}
