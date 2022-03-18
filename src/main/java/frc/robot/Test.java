package frc.robot;

import java.lang.reflect.Field;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Test {
    public static void main(String[] args) {
        double turretFieldOrientedAngle = -190.0;
        Vector2 visionTargetVector = new Vector2(2.5, 0.5);

        System.out.println(visionTargetVector.getAngle().toDegrees() + turretFieldOrientedAngle);
        Translation2d fieldEstimatedPosition = new Translation2d(visionTargetVector.length, Rotation2d.fromDegrees(visionTargetVector.getAngle().toDegrees() + turretFieldOrientedAngle));
        System.out.println(fieldEstimatedPosition);
    }
}
