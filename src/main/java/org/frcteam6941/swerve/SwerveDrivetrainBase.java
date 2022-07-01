package org.frcteam6941.swerve;

import org.frcteam6941.looper.UpdateManager.Updatable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveDrivetrainBase extends Updatable{
    abstract void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop);

    abstract SwerveModuleState[] getStates();

    abstract void drive(Translation2d translation, double rotation, boolean fieldRelative);
    
    abstract void setHeadingTarget(double heading);

    abstract void setLockHeading(boolean lockHeading);

    abstract Pose2d getPose();

    abstract SwerveDriveKinematics getKinematics();
    
    abstract void resetOdometry(Pose2d pose);

    abstract void resetGyro(double degree);

    
}
