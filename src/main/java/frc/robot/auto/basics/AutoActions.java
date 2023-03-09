package frc.robot.auto.basics;

import java.util.List;
import java.util.function.Supplier;

import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldConstants;
import frc.robot.commands.AutoScore;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.RequestSuperstructureStateAutoRetract;
import frc.robot.commands.RequestSuperstructureStateCommand;
import frc.robot.commands.WaitUntilNoCollision;
import frc.robot.states.Direction;
import frc.robot.states.GamePiece;
import frc.robot.states.LoadingTarget;
import frc.robot.states.LoadingTarget.LOADING_LOCATION;
import frc.robot.states.ScoringTarget;
import frc.robot.states.SuperstructureState;
import frc.robot.states.SuperstructureStateBuilder;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;
import frc.robot.subsystems.TargetSelector;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.PathPointUtil;

public class AutoActions {
    SJTUSwerveMK5Drivebase mDrivebase;
    ArmAndExtender mSuperstructure;
    Intaker mIntaker;
    TargetSelector mTargetSelector;

    AutoScore autoScore;
    Supplier<SuperstructureState> scoreSuperstructureStateSupplier;
    Supplier<SuperstructureState> scoreSuperstructureStateSupplierLower;
    Supplier<Pose2d> scoreDriveTargetSupplier;

    private static final double xInnerMiddle = (FieldConstants.Grids.outerX + FieldConstants.Community.chargingStationInnerX) / 2.0;
    private static final double xOuter = FieldConstants.Community.chargingStationOuterX + 1.5;
    private static final double yLeftMiddle = FieldConstants.Community.leftY - 0.5;
    private static final double yRightMiddle = FieldConstants.Community.rightY + 0.5;

    private static final Translation2d innerLeftTransit = new Translation2d(xInnerMiddle, yLeftMiddle);
    private static final Translation2d innerRightTransit = new Translation2d(xInnerMiddle, yRightMiddle);
    private static final Translation2d outerLeftTransit = new Translation2d(xOuter, yLeftMiddle);
    private static final Translation2d outerRightTrandit = new Translation2d(xOuter, yRightMiddle);

    public AutoActions(SJTUSwerveMK5Drivebase mDrivebase, ArmAndExtender mSuperstructure, Intaker mIntaker, TargetSelector mTargetSelector) {
        this.mDrivebase = mDrivebase;
        this.mSuperstructure = mSuperstructure;
        this.mIntaker = mIntaker;
        this.mTargetSelector = mTargetSelector;

        autoScore = new AutoScore(mDrivebase, mSuperstructure, mIntaker, mTargetSelector, () -> false, () -> true);
        scoreSuperstructureStateSupplier = autoScore.getSuperstructureTargetSupplier();
        scoreSuperstructureStateSupplierLower = autoScore.getSuperstructureTargetSupplierLower();
        scoreDriveTargetSupplier = autoScore.getDrivetrainTargetSupplier();
    }

    public Command groundIntake() {
        return new InstantCommand(mTargetSelector.getTargetGamePiece() == GamePiece.CONE ? mIntaker::runIntakeCone : mIntaker::runIntakeCube)
        .andThen(new RequestSuperstructureStateAutoRetract(
            mSuperstructure,
            () -> mTargetSelector.getLoadSuperstructureState()
        ))
        .andThen(new WaitUntilCommand(mIntaker::hasGamePiece))
        .andThen(commute());
    }

    public Command prepScore() {
        return new RequestSuperstructureStateAutoRetract(mSuperstructure, scoreSuperstructureStateSupplier);
    }

    public Command score() {
        return new RequestSuperstructureStateCommand(mSuperstructure, scoreSuperstructureStateSupplierLower)
        .andThen(new InstantCommand(() -> mIntaker.runIntake(mTargetSelector::getTargetGamePiece)))
        .andThen(new WaitCommand(0.5));
    }

    public Command commute() {
        return new RequestSuperstructureStateAutoRetract(mSuperstructure, () -> SuperstructureStateBuilder.buildCommutingSuperstructureState(mTargetSelector.getCommutingDirection()));
    }

    private Pose2d getInnerTransitPose() {
        Rotation2d face;
        switch(mTargetSelector.getScoringDirection()){
            case NEAR:
                face = Rotation2d.fromDegrees(180.0);
                break;
            case FAR:
            default:
                face = Rotation2d.fromDegrees(0.0);
                break;
        }

        Pose2d target;
        switch(mTargetSelector.getScoringTarget().getScoringGrid()) {
            case INNER:
                target = new Pose2d(innerLeftTransit, face);
                break;
            case OUTER:
                target = new Pose2d(innerRightTransit, face);
                break;
            case COOPERTITION:
            default:
                target = new Pose2d(innerLeftTransit, face);
                break;
        }
        
        return AllianceFlipUtil.apply(target);
    }

    private Pose2d getOuterTransitPose() {
        Rotation2d face;
        switch(mTargetSelector.getLoadingDirection()){
            case NEAR:
                face = Rotation2d.fromDegrees(0.0);
                break;
            case FAR:
            default:
                face = Rotation2d.fromDegrees(180.0);
                break;
        }

        Pose2d target;
        switch(mTargetSelector.getScoringTarget().getScoringGrid()) {
            case INNER:
                target = new Pose2d(outerLeftTransit, face);
                break;
            case OUTER:
                target = new Pose2d(outerRightTrandit, face);
                break;
            case COOPERTITION:
            default:
                target = new Pose2d(outerLeftTransit, face);
                break;
        }

        return AllianceFlipUtil.apply(target);
    }

    public Command driveToInnerTransit() {
        return new DriveToPoseCommand(mDrivebase, this::getInnerTransitPose);
    }

    public Command pathInnerToIntake(Translation2d endIntakeTarget) {
        Pose2d innerTransit = getInnerTransitPose();
        Pose2d outerPose = getOuterTransitPose();
        Rotation2d intakeDirection = endIntakeTarget.minus(outerPose.getTranslation()).getAngle();
        Rotation2d delta = Rotation2d.fromDegrees(mTargetSelector.getLoadingDirection() == Direction.NEAR ? 0.0 : 180.0);
        Pose2d outerTransformed = new Pose2d(outerPose.getTranslation(), intakeDirection);
        Pose2d intakeTransformed = new Pose2d(endIntakeTarget, intakeDirection.plus(delta));
        Pose2d middleTransformed = new Pose2d(innerTransit.getTranslation().interpolate(outerTransformed.getTranslation(), 0.5), intakeDirection);
        
        return new FollowTrajectory(
            mDrivebase,
            PathPointUtil.transfromPose2dToPathPoints(
                List.of(
                    innerTransit,
                    middleTransformed,
                    outerTransformed,
                    intakeTransformed
                )
            )
        );
    }

    public Command pathIntakeToInner(Translation2d endIntakeTarget) {
        Pose2d innerTransit = getInnerTransitPose();
        Pose2d outerPose = getOuterTransitPose();
        Rotation2d intakeDirection = endIntakeTarget.minus(outerPose.getTranslation()).getAngle();
        Rotation2d delta = Rotation2d.fromDegrees(mTargetSelector.getLoadingDirection() == Direction.NEAR ? 0.0 : 180.0);
        Pose2d outerTransformed = new Pose2d(outerPose.getTranslation(), intakeDirection);
        Pose2d intakeTransformed = new Pose2d(endIntakeTarget, intakeDirection.plus(delta).unaryMinus());
        
        return new FollowTrajectory(
            mDrivebase,
            PathPointUtil.transfromPose2dToPathPoints(
                List.of(
                    intakeTransformed,
                    outerTransformed,
                    innerTransit
                )
            )
        );
    }

    public Command driveToScoreTarget() {
        return new DriveToPoseCommand(mDrivebase, scoreDriveTargetSupplier);
    }

    public Command waitUntilRetractSafe() {
        return new WaitUntilNoCollision(mDrivebase.getLocalizer()::getLatestPose, mSuperstructure, mIntaker, mTargetSelector);
    }

    public Command waitUntilDirectionFit(Pose2d targetPose) {
        return new WaitUntilCommand(() -> Util.epsilonEquals(targetPose.getRotation().getDegrees(), mDrivebase.getLocalizer().getLatestPose().getRotation().getDegrees(), 30.0));
    }

    public Command waitUntilHomed() {
        return new WaitUntilCommand(mSuperstructure::isHomed);
    }

    public Command configTargetSelector(ScoringTarget scoringTarget) {
        return Commands.runOnce(() -> mTargetSelector.setScoringTarget(scoringTarget));
    }

    public Command configGroundIntake() {
        return Commands.runOnce(() -> mTargetSelector.setLoadingTarget(new LoadingTarget(LOADING_LOCATION.GROUND)));
    }

    public Command scorePreload(ScoringTarget target) {
        return Commands.sequence(
            configGroundIntake(),
            configTargetSelector(target),
            driveToInnerTransit()
            .alongWith(
                Commands.sequence(
                    waitUntilHomed(),
                    prepScore()
                )
            ),
            driveToScoreTarget(),
            score(),
            driveToInnerTransit()
        );
    }

    public Command intakeAndScore(ScoringTarget target, Translation2d intakePosition) {
        return Commands.sequence(
            configGroundIntake(),
            driveToInnerTransit(),
            configTargetSelector(target),
            pathInnerToIntake(intakePosition)
            .deadlineWith(
                Commands.sequence(
                    commute(),
                    waitUntilDirectionFit(getOuterTransitPose()),
                    groundIntake()
                )
            ),
            pathIntakeToInner(intakePosition)
            .alongWith(
                Commands.sequence(
                    commute(),
                    waitUntilDirectionFit(getInnerTransitPose()),
                    prepScore()
                )
            ),
            driveToScoreTarget(),
            score(),
            driveToInnerTransit()
        );
    }

    public Command scoreTwo(ScoringTarget target1, ScoringTarget target2, Translation2d intakeTarget1) {
        return scorePreload(target1).andThen(intakeAndScore(target2, intakeTarget1)).andThen(commute());
    }

    public Command scoreLink(ScoringTarget target1, ScoringTarget target2, ScoringTarget target3, Translation2d intakeTarget1, Translation2d intakeTarget2) {
        return scorePreload(target1).andThen(intakeAndScore(target2, intakeTarget1)).andThen(intakeAndScore(target3, intakeTarget2)).andThen(commute());
    }

}
