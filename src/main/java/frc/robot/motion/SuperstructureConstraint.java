package frc.robot.motion;

import org.frcteam6941.utils.Range;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.states.SuperstructureState;

public class SuperstructureConstraint {
    private Range heightRange;
    private Range armRange;
    private Range extenderRange;
    private Range dangerousPositiveArmRange;
    private Range dangerousNegativeArmRange;

    public SuperstructureConstraint(Range heightRange, Range armRange, Range extenderRange,
            Range dangerousPositiveArmRange, Range dangerousNegativeArmRange) {
        this.heightRange = heightRange;
        this.armRange = armRange;
        this.extenderRange = extenderRange;
        this.dangerousPositiveArmRange = dangerousPositiveArmRange;
        this.dangerousNegativeArmRange = dangerousNegativeArmRange;
    }

    public SuperstructureState optimize(SuperstructureState desiredState, SuperstructureState currentState) {
        // Clamp state into max and min
        SuperstructureState clampedDesiredState = new SuperstructureState(
                Rotation2d.fromDegrees(this.armRange.clamp(desiredState.armAngle.getDegrees())),
                this.extenderRange.clamp(desiredState.extenderLength));

        // Judge dangerous positive and negative, retract if needed
        if (dangerousPositiveArmRange.inRange(clampedDesiredState.armAngle.getDegrees())) {
            clampedDesiredState.extenderLength = extenderRange.min;
            if (!currentState.isExtenderOnTarget(clampedDesiredState,
                    Constants.SUBSYSTEM_SUPERSTRUCTURE.THRESHOLD.EXTENDER)) {
                clampedDesiredState.armAngle = Rotation2d.fromDegrees(dangerousPositiveArmRange.min);
            }
        } else if (dangerousNegativeArmRange.inRange(clampedDesiredState.armAngle.getDegrees())) {
            clampedDesiredState.extenderLength = extenderRange.min;
            if (!currentState.isExtenderOnTarget(clampedDesiredState,
                    Constants.SUBSYSTEM_SUPERSTRUCTURE.THRESHOLD.EXTENDER)) {
                clampedDesiredState.armAngle = Rotation2d.fromDegrees(dangerousNegativeArmRange.max);
            }
        } else {
            // Use forward kinematics to determine final position, then clamp height to
            // limited zone
            Translation2d finalPosition = SuperstructureKinematics.forwardKinematics2d(clampedDesiredState);
            Translation2d newPosition = new Translation2d(finalPosition.getX(),
                    this.heightRange.clamp(finalPosition.getY()));
            clampedDesiredState = SuperstructureKinematics.inverseKinematics2d(newPosition);
            // clamp again to regulate the angles into arm range
            clampedDesiredState = new SuperstructureState(
                    Rotation2d.fromDegrees(this.armRange.clamp(clampedDesiredState.armAngle.getDegrees())),
                    this.extenderRange.clamp(clampedDesiredState.extenderLength));
        }

        return clampedDesiredState;
    }

    public static void main(String[] args) {

    }
}
