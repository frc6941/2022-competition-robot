package frc.robot;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class Test {
    public static void main(String[] args) {
        edu.wpi.first.math.geometry.Translation2d cameraToFieldTranslation = new edu.wpi.first.math.geometry.Translation2d().minus(new edu.wpi.first.math.geometry.Translation2d(2, 0));
        Pose2d cameraToField = new Pose2d(new com.team254.lib.geometry.Translation2d(cameraToFieldTranslation.getX(), cameraToFieldTranslation.getY()), new com.team254.lib.geometry.Rotation2d());
        Pose2d vehicleToTurret = new Pose2d(
        new Pose2d(new Translation2d(0.068, 0.0), Rotation2d.fromDegrees(90)).getTranslation(),
        Rotation2d.fromDegrees(3.0)
        );
        Pose2d turretToCamera = new Pose2d(0.06, 0.0, new Rotation2d());
        Pose2d estimatedVehicleToField = vehicleToTurret.transformBy(turretToCamera).transformBy(cameraToField);
        System.out.println(estimatedVehicleToField.toString());
    }
}
