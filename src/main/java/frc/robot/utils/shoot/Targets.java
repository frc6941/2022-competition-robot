package frc.robot.utils.shoot;

import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.FieldConstants;

public class Targets {
    public static Translation2d getTestTarget() {
        return new Translation2d(1.6, 0.0);
    }

    public static Translation2d getDefaultTarget() {
        return FieldConstants.hubCenter;
    }

    private static boolean isReachable(Pose2d robotPose, Translation2d targetLocation) {
        Translation2d targetRelative = targetLocation.minus(robotPose.getTranslation());
        return Util.epsilonEquals(robotPose.getRotation().getDegrees(),
                new Rotation2d(targetRelative.getX(), targetRelative.getY()).getDegrees(),
                Constants.TURRET_MAX_ROTATION_DEGREE);
    }

    private static final Translation2d[] TARGET_SELECTION_SEQUENCE = new Translation2d[] {
            FieldConstants.hangarCenter,
            FieldConstants.hangarBottomLeftCorner,
            FieldConstants.hangarTopLeftCorner,
            FieldConstants.hangarBottomRightCorner,
            FieldConstants.hangarTopRightCorner,
            FieldConstants.terminalCenter.getTranslation(),
            FieldConstants.hubCenter.plus(new Translation2d(0.0, -2.0)),
            FieldConstants.hubCenter.plus(new Translation2d(0.0, 2.0))
    };

    public static Translation2d getWrongballTarget(Pose2d robotPose, boolean isShooting) {
        Translation2d rawWrongBallTarget = null;
        if (isShooting) {
            if (isReachable(robotPose, FieldConstants.hubCenter.plus(new Translation2d(0.0, -1.0)))) {
                rawWrongBallTarget = FieldConstants.hubCenter.plus(new Translation2d(0.0, -1.0));
            } else {
                rawWrongBallTarget = FieldConstants.hubCenter.plus(new Translation2d(0.0, 1.0));
            }
        } else {
            for (Translation2d location : TARGET_SELECTION_SEQUENCE) {
                if (isReachable(robotPose, location)) {
                    rawWrongBallTarget = location;
                    break;
                }
            }
        }

        double angle;
        double distance;

        if (rawWrongBallTarget == null) {
            angle = robotPose.getRotation().getDegrees();
            distance = Constants.ShootingConstants.MIN_SHOOTING_DISTANCE;
        } else {
            Translation2d targetRelative = rawWrongBallTarget.minus(robotPose.getTranslation());
            angle = new Rotation2d(targetRelative.getX(), targetRelative.getY()).getDegrees();
            distance = Util.clamp(targetRelative.getNorm() - Constants.ShootingConstants.WRONG_BALL_OFFSET_DISTANCE,
                    Constants.ShootingConstants.MIN_SHOOTING_DISTANCE,
                    Constants.ShootingConstants.MAX_SHOOTING_DISTANCE);
        }

        return new Translation2d(distance, Rotation2d.fromDegrees(angle)).plus(robotPose.getTranslation());
    }

    
    public static Translation2d getRawWrongballTarget(Pose2d robotPose, boolean isShooting) {
        Translation2d rawWrongBallTarget = null;
        if (isShooting) {
            if (isReachable(robotPose, FieldConstants.hubCenter.plus(new Translation2d(0.0, -1.0)))) {
                rawWrongBallTarget = FieldConstants.hubCenter.plus(new Translation2d(0.0, -1.0));
            } else {
                rawWrongBallTarget = FieldConstants.hubCenter.plus(new Translation2d(0.0, 1.0));
            }
        } else {
            for (Translation2d location : TARGET_SELECTION_SEQUENCE) {
                if (isReachable(robotPose, location)) {
                    rawWrongBallTarget = location;
                    break;
                } else {
                    continue;
                }
            }
        }

        if (rawWrongBallTarget == null) {
            double angle = robotPose.getRotation().getDegrees();
            double distance = Constants.ShootingConstants.MIN_SHOOTING_DISTANCE;
            rawWrongBallTarget = new Translation2d(distance, Rotation2d.fromDegrees(angle));
        } 
       
        return rawWrongBallTarget;
    }
    
}
