package frc.robot.states;

import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Rotation2d;

/*
 * Representing the state of superstructure.
 * 
 * Positive direction definition:
 * The tip of the "triangle" form by the supporting rod and the drive train is the "head" of the drivetrain.
 * 
 */
public class SuperstructureState {
    // The angle of the large base arm - viewed from the right, CCW positive.
    public Rotation2d armAngle = new Rotation2d();
    // The length of the extender installed on the base arm.
    public double extenderLength = 10.0;

    public SuperstructureState(Rotation2d armAngle, double extenderLength) {
        this.armAngle = armAngle;
        this.extenderLength = extenderLength;
    }

    public SuperstructureState() {

    }

    public boolean hasChangedScoringGoal(SuperstructureState other) {
        return other.armAngle == armAngle && other.extenderLength == this.extenderLength;
    }

    public boolean isOnTarget(SuperstructureState desiredState, double armThreshold, double extenderThreshold) {
        return isArmOnTarget(desiredState, armThreshold) && isExtenderOnTarget(desiredState, extenderThreshold);
    }

    public boolean isArmOnTarget(SuperstructureState desiredState, double armThreshold) {
        return Util.epsilonEquals(desiredState.armAngle.getDegrees(), armAngle.getDegrees(), armThreshold);
    }

    public boolean isExtenderOnTarget(SuperstructureState desiredState, double extenderThreshold) {
        return Util.epsilonEquals(desiredState.extenderLength, extenderLength, extenderThreshold);
    }

    @Override
    public boolean equals(Object o) {
        if (o == this) {
            return true;
        }
 
        if (!(o instanceof SuperstructureState)) {
            return false;
        }
         
        SuperstructureState c = (SuperstructureState) o;
        return c.armAngle.equals(armAngle) && c.extenderLength == extenderLength;
    }

    @Override
    public String toString() {
        return String.format("Superstructure State: Arm: %.2f deg; Extender: %.2f m", armAngle.getDegrees(), extenderLength);
    }
}
