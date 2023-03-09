package org.frcteam6941.swerve;

import org.frcteam6941.looper.UpdateManager.Updatable;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveDrivetrainBase extends Updatable{
    void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop);

    SwerveModuleState[] getModuleStates();

    SwerveModulePosition[] getModulePositions();

    void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop);
    
    void setHeadingTarget(double heading);

    void setLockHeading(boolean lockHeading);

    SwerveDriveKinematics getKinematics();
}
