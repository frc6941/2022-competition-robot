package frc.robot;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class Test {
    public static void main(String[] args) {
        edu.wpi.first.math.geometry.Rotation2d test = new edu.wpi.first.math.geometry.Rotation2d(8.0, -4.0);
        System.out.println(test.getDegrees());
    }
}
