package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;

public class PathPointUtil {
    public static List<PathPoint> transfromPose2dToPathPoints(List<Pose2d> poses) {
        ArrayList<PathPoint> result = new ArrayList<>();
        for(int i = 0; i < poses.size() - 2; i++) {
            result.add(
                new PathPoint(poses.get(i).getTranslation(), poses.get(i + 1).getTranslation().minus(poses.get(i).getTranslation()).getAngle(), poses.get(i).getRotation())
            );
        }
        Pose2d finalPose = poses.get(poses.size() - 1);
        result.add(
            new PathPoint(finalPose.getTranslation(), finalPose.getRotation().unaryMinus(), finalPose.getRotation())
        );
        
        return result;
    }
}
