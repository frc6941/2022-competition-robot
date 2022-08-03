package org.frcteam6941.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TestKinematics {
    public static void main(String[] args) {
        double distance = 5.0;
        double turretAngle = 40;
        double targetAngleDelta = -20;

        Translation2d robotVelocitytoField = new Translation2d(1.0, 1.0);

        double facingAngle = turretAngle + targetAngleDelta;

        System.out.println(robotVelocitytoField.rotateBy(Rotation2d.fromDegrees(facingAngle)));
    }
}
