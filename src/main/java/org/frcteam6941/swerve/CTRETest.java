package org.frcteam6941.swerve;

import org.frcteam6941.utils.AngleNormalization;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants.Turret;

public class CTRETest {
    static double angle = 0.0;
    static double turrentAngle = 10.0;
    static double fieldOrientedTurretHeading = angle + turrentAngle;

    public static double[] calculateTurretDrivetrainAngle(double desiredAngle, boolean isLimited) {
        double delta = AngleNormalization.placeInAppropriate0To360Scope(fieldOrientedTurretHeading,
                desiredAngle)
                - fieldOrientedTurretHeading;
        double availableTurretDelta;
        if (isLimited) {
            availableTurretDelta = Math.copySign(Constants.TURRET_SAFE_ZONE_DEGREE, delta) - turrentAngle;
        } else {
            availableTurretDelta = Math.copySign(Constants.TURRET_MAX_ROTATION_DEGREE, delta) - turrentAngle;
        }
        if (Math.abs(delta) <= Math.abs(availableTurretDelta)) {
            return new double[] { delta + turrentAngle, 0 };
        } else {
            return new double[] { availableTurretDelta + turrentAngle,
                    delta - availableTurretDelta + angle };
        }
    }

    public static void main(String[] vars) {
        System.out.println(calculateTurretDrivetrainAngle(200.0, true)[0]);
        System.out.println(calculateTurretDrivetrainAngle(200.0, true)[1]);
    }
}
