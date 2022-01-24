package org.frcteam6941.swerve;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.UpdateManager.Updatable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SwerveDrivetrainBase extends Subsystem, Updatable{
    abstract void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop);

    abstract SwerveModuleState[] getStates();

    abstract void drive(Translation2d translation, double rotation, boolean fieldRelative);
    
    abstract void setHeadingTarget(double heading);

    abstract void setLockHeading(boolean lockHeading);

    abstract Pose2d getPose();

    abstract Rotation2d getYaw();

    abstract SwerveDriveKinematics getKinematics();

    abstract void resetGyroAngle(Rotation2 angle);
    
    abstract void resetOdometry(Pose2d pose);

    abstract void resetGyro(double degree);

    
}
