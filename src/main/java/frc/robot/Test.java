package frc.robot;

import java.lang.annotation.Target;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.shoot.Targets;

public class Test {
    public static void main(String[] args) {
        System.out.println(Constants.TURRET_FEEDFORWARD.calculate(60.0, 20.0));
    }
}
