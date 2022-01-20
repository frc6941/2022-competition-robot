package org.frcteam6941.utils;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


/**
 * This is essentially a tool made easy for 'inventing' after you 'steal' from the best.
 * There's only one core function: transform, which enables you to convert any kind of input to a given type of output, if possible.
 */
public final class MasterKey {
    private static RigidTransform2 fromPose2dWPIToRigidTransform22910(Pose2d input){
        return new RigidTransform2(
            new Vector2(input.getTranslation().getX(), input.getTranslation().getY()),
            Rotation2.fromDegrees(input.getRotation().getDegrees())
        );
    }

    private static Rotation2d from254Rotation2dtoWPRotation2d(com.team254.lib.geometry.Rotation2d input){
        return new Rotation2d(input.getRadians());
    }

    private static com.team254.lib.geometry.Pose2d fromWPIPose2dto254Pose2d(Pose2d input){
        return new com.team254.lib.geometry.Pose2d(input.getX(), input.getY(), com.team254.lib.geometry.Rotation2d.fromDegrees(input.getRotation().getDegrees()));
    }

    private static Pose2d from254Pose2dtoWPIPose2d(com.team254.lib.geometry.Pose2d input){
        return new Pose2d(
            new Translation2d(input.getTranslation().x(), input.getTranslation().y()), Rotation2d.fromDegrees(input.getRotation().getDegrees())
        );
    }

    @SuppressWarnings("unchecked")
    public static <T> T transform(Object input, Class<T> objective){
        T result = null;
        if(input.getClass().equals(Pose2d.class)){
            if(objective.equals(RigidTransform2.class)){
                Object o = fromPose2dWPIToRigidTransform22910((Pose2d) input);
                result = (T)o;
            } else if(objective.equals(com.team254.lib.geometry.Pose2d.class)){
                Object o = fromWPIPose2dto254Pose2d((Pose2d) input);
                result = (T)o;
            }
        } else if (input.getClass().equals(com.team254.lib.geometry.Pose2d.class)){
            if(objective.equals(Pose2d.class)){
                Object o = from254Pose2dtoWPIPose2d((com.team254.lib.geometry.Pose2d) input);
                result = (T)o;
            }
        } else if (input.getClass().equals(com.team254.lib.geometry.Rotation2d.class)){
            if(objective.equals(Rotation2d.class)){
                Object o = from254Rotation2dtoWPRotation2d((com.team254.lib.geometry.Rotation2d) input);
                result = (T)o;
            }
        }

        return result;
    }
}
