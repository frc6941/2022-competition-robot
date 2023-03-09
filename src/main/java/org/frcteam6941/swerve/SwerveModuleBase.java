package org.frcteam6941.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleBase {
    int getModuleNumber();

    void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean overrideMotion);

    Rotation2d getEncoder();

    Rotation2d getEncoderUnbound();

    SwerveModuleState getState();

    SwerveModulePosition getPosition();
}
