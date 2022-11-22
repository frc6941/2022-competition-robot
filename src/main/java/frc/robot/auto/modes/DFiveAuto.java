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

public class DFiveAuto extends AutoModeBase{
    protected String autoName = "D - Five Auto";
    private final PathPlannerTrajectory trajectoryPart1 = PathPlanner.loadPath("D 5 Ball Auto - Part 1", 3.5, 3.5);
    private final PathPlannerTrajectory trajectoryPart2 = PathPlanner.loadPath("D 5 Ball Auto - Part 2", 3.5, 3.5);
    private final PathPlannerTrajectory trajectoryPart3 = PathPlanner.loadPath("D 5 Ball Auto - Part 3", 3.5, 3.5);
    private final PathPlannerTrajectory trajectoryPart4 = PathPlanner.loadPath("D 5 Ball Auto - Part 4", 3.5, 3.5);
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final SJTUSwerveMK5Drivebase mSwerve = SJTUSwerveMK5Drivebase.getInstance();


    @Override
    public Pose2d getStartingPose() {
        return trajectoryPart1.getInitialPose();
    };

    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
            // Start settings
            new InstantCommand(() -> mSuperstructure.setWantEject(false)),
            new InstantCommand(() -> mSuperstructure.setWantMoveAndShoot(false)),
            // Part 1: collect 3 cargos and shoot them together
            new InstantCommand(() -> mSuperstructure.setWantIntake(true)),
            new InstantCommand(() -> mSuperstructure.setWantMaintain(true)),
            new FollowTrajectory(mSwerve, trajectoryPart1, true, true, true),
            new WaitUntilCommand(mSuperstructure::isReady).withTimeout(0.5),
            new InstantCommand(() -> mSuperstructure.setState(STATE.SHOOTING)),
            new WaitCommand(1.2),
            new InstantCommand(() -> mSuperstructure.setState(STATE.CHASING)),
            new InstantCommand(() -> mSuperstructure.setWantMaintain(false)),
            // Part 2: go to terminal and wait for ball feed
            new FollowTrajectory(mSwerve, trajectoryPart2, true, false, true),
            new WaitCommand(1.0),
            // Part 3: come back and shoot
            new FollowTrajectory(mSwerve, trajectoryPart3, true, false, true),
            new InstantCommand(() -> mSuperstructure.setWantIntake(false)),
            new InstantCommand(() -> mSuperstructure.setWantMaintain(true)),
            new WaitUntilCommand(mSuperstructure::isReady).withTimeout(0.5),
            new InstantCommand(() -> mSuperstructure.setState(STATE.SHOOTING)),
            new WaitCommand(0.8),
            new InstantCommand(() -> mSuperstructure.setWantMaintain(false)),
            new InstantCommand(() -> mSuperstructure.setState(STATE.CHASING)),
            // Part 4: end move and ready for invading opponent half
            new FollowTrajectory(mSwerve, trajectoryPart4, true, false, true),
            // End settings
            new InstantCommand(() -> mSuperstructure.setWantMaintain(false)),
            new InstantCommand(() -> mSuperstructure.setWantEject(true)),
            new InstantCommand(() -> mSuperstructure.setWantMoveAndShoot(true))
        );
    };

    public DFiveAuto() {
    }
}
