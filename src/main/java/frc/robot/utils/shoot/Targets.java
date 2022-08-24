package frc.robot.utils.shoot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;

public class Targets {
    public static Translation2d getTestTarget(){
        return new Translation2d(1.6, 0.0);
    }

    public static Translation2d getDefaultTarget(){
        return FieldConstants.hubCenter;
    }

    public static Translation2d getWrongballTarget(Pose2d robotPose) {
        return robotPose.getTranslation().plus(new Translation2d(2.0, -1.0));
    }
}
