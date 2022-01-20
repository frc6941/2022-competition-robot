package org.frcteam6941.swerve;

import org.frcteam1678.lib.util.CTREModuleState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class CTRETest {
    public static void main(String[] vars){
        // Locations for the swerve drive modules relative to the robot center.
        Translation2d m_frontLeftLocation = new Translation2d(-0.381, 0.381);
        Translation2d m_frontRightLocation = new Translation2d(-0.381, -0.381);
        Translation2d m_backLeftLocation = new Translation2d(0.381, 0.381);
        Translation2d m_backRightLocation = new Translation2d(0.381, -0.381);

        // Creating my kinematics object using the module locations
        SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
        );
        SwerveModuleState[] swerveModuleStates =
            m_kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    1.0, 
                    0.0, 
                    0.0, 
                    Rotation2d.fromDegrees(180.0)
            ));

        for(SwerveModuleState i : swerveModuleStates){
            System.out.println(CTREModuleState.optimize(i, Rotation2d.fromDegrees(0)));
        }
    }
}
