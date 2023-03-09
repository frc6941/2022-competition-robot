package frc.robot.states;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import frc.robot.utils.AllianceFlipUtil;

public class AssistedPoseBuilder {
    // TODO: change this to fit real field
    // public static final Pose2d doubleSubstationLeftPose = new Pose2d(
    //         FieldConstants.LoadingZone.doubleSubstationX - 1.2,
    //         FieldConstants.LoadingZone.doubleSubstationCenterY + 0.65,
    //         new Rotation2d());
    // public static final Pose2d doubleSubstationRightPose = new Pose2d(
    //         FieldConstants.LoadingZone.doubleSubstationX - 1.2,
    //         FieldConstants.LoadingZone.doubleSubstationCenterY - 0.65,
    //         new Rotation2d());

    public static final Pose2d doubleSubstationLeftPose = new Pose2d(
            FieldConstants.StagingLocations.translations[1].getX() - 1.2,
            FieldConstants.StagingLocations.translations[1].getY() + 0.65,
            new Rotation2d());
    public static final Pose2d doubleSubstationRightPose = new Pose2d(
        FieldConstants.StagingLocations.translations[1].getX() - 1.2,
        FieldConstants.StagingLocations.translations[1].getY() - 0.65,
            new Rotation2d());

    public static final double minDriveX = FieldConstants.Grids.outerX + 0.5;
    public static final double minDriveY = 0.5;
    public static final double maxDriveY = FieldConstants.Community.leftY - 0.5;

    public static final Transform2d turn = new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180.0));

    public static Pose2d buildLoadingPose2d(LoadingTarget loadingTarget, Direction direction) {
        switch (loadingTarget.getLoadingLocation()) {
            case DOUBLE_SUBSTATION_OUTER:
                return AllianceFlipUtil.apply(direction == Direction.NEAR ? doubleSubstationRightPose
                        : doubleSubstationLeftPose.transformBy(turn));
            case DOUBLE_SUBSTATION_INNER:
                return AllianceFlipUtil.apply(direction == Direction.NEAR ? doubleSubstationLeftPose
                        : doubleSubstationRightPose.transformBy(turn));
            case GROUND:
                return null;
            default:
                return null;
        }
    }

    public static Pose2d buildScoringPose2d(ScoringTarget scoringTarget, Direction direction, GamePiece gamePiece) {
        double directionDegrees;
        if (direction == Direction.NEAR) {
            directionDegrees = 180.0;
        } else {
            directionDegrees = 0.0;
        }

        double targetY = FieldConstants.Grids.complexLowTranslations[scoringTarget.getPosition()].getY();
        return AllianceFlipUtil.apply(
            new Pose2d(minDriveX, targetY, Rotation2d.fromDegrees(directionDegrees))
        );
    }
}
