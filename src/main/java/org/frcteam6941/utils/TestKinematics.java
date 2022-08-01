package org.frcteam6941.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class TestKinematics {
    static Translation2d[] swerveModulePositions = new Translation2d[] { new Translation2d(0.35, 0.35),
            new Translation2d(0.35, -0.35),
            new Translation2d(-0.35, 0.35)};
    static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(swerveModulePositions);

    public static void main(String[] args) {
        for (SwerveModuleState state: swerveKinematics.toSwerveModuleStates(
            new ChassisSpeeds(0, 0, 0)
        )){
            System.out.println(
            state
        );
        }
        
    }
}
