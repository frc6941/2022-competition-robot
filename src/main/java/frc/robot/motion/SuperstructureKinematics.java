package frc.robot.motion;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.SUBSYSTEM_SUPERSTRUCTURE.STRUCTURE;
import frc.robot.states.SuperstructureState;

public class SuperstructureKinematics {
    public static final Pose3d forwardKinematics(SuperstructureState q, Pose2d drivetrainPose) {
        Transform3d transformFromArmPivotToEffector = new Transform3d(
                new Translation3d(q.extenderLength, new Rotation3d(0.0, q.armAngle.getRadians(), 0.0)),
                new Rotation3d(0.0, Units.degreesToRadians(q.armAngle.getRadians()), 0.0));
        Pose3d drivetrainPose3d = new Pose3d(drivetrainPose);
        return drivetrainPose3d
                .plus(STRUCTURE.ROBOT_CENTER_TO_HIGH_PIVOT)
                .plus(transformFromArmPivotToEffector);
    }

    public static final SuperstructureState inverseKinematics(Translation3d endEffectorPosition,
            Pose2d drivetrainPose) {
        Pose3d drivetrainPose3d = new Pose3d(drivetrainPose);
        Pose3d topTowerPivot = drivetrainPose3d.plus(STRUCTURE.ROBOT_CENTER_TO_HIGH_PIVOT);
        Transform3d endEffectorToTower = topTowerPivot.minus(new Pose3d(endEffectorPosition, new Rotation3d()));
        Rotation2d armAngle = new Rotation2d(endEffectorToTower.getX(), endEffectorToTower.getZ());
        double extenderLength = endEffectorToTower.getTranslation().getNorm();
        return new SuperstructureState(armAngle, extenderLength);
    }

    public static final Translation2d forwardKinematics2d(SuperstructureState q) {
        Translation2d highPivotToEndEffector = new Translation2d(q.extenderLength, q.armAngle);
        return STRUCTURE.HIGH_PIVOT_2D_LOCATION.plus(highPivotToEndEffector);
    }

    public static final SuperstructureState inverseKinematics2d(Translation2d endEffectorPosition) {
        Translation2d highPivotToEndEffector = endEffectorPosition.minus(STRUCTURE.HIGH_PIVOT_2D_LOCATION);
        // Get angle in correct range - ~-100 deg to ~250 deg for our robot
        // As the method may return negative angle when exceeding 180.0 deg, scope
        // referencing is important
        // Adding or minusing 360.0 to get the correct angle in the scope
        double angle = highPivotToEndEffector.getAngle().getDegrees();
        while (!Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.ARM_RANGE.inRange(angle)) {
            if(angle < Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.ARM_RANGE.min) {
                angle += 360.0;
            } else {
                angle -= 360.0;
            }
        }

        return new SuperstructureState(
                Rotation2d.fromDegrees(angle),
                highPivotToEndEffector.getNorm());
    }
}
