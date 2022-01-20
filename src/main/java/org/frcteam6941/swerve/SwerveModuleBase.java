package org.frcteam6941.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleBase {
    abstract void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean overrideMotion);

    abstract Rotation2d getEncoder();

    abstract Rotation2d getEncoderUnbound();

    abstract SwerveModuleState getState();
}
