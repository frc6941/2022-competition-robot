package frc.robot.shuffleboard.tabs;

import java.util.Map;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;
import org.frcteam6941.utils.AngleNormalization;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;
import frc.robot.shuffleboard.ShuffleboardTabBase;

public class SwerveTab extends ShuffleboardTabBase {
    private final SJTUSwerveMK5Drivebase mSwerve = SJTUSwerveMK5Drivebase.getInstance();

    private final NetworkTableEntry[] swerveModuleVelocities;
    private final NetworkTableEntry[] swerveModuleAngles;
    private NetworkTableEntry gyroAngle;
    private NetworkTableEntry swerveState;

    public SwerveTab(){
        swerveModuleVelocities = new NetworkTableEntry[mSwerve.getSwerveModuleStates().length];
        swerveModuleAngles = new NetworkTableEntry[mSwerve.getSwerveModuleStates().length];
    }

    @Override
    public void createEntries() {
        mTab = Shuffleboard.getTab("Swerve");
        for (int i = 0; i < mSwerve.getSwerveModuleStates().length; i++) {
            swerveModuleVelocities[i] = mTab
                    .add("Mod " + i + " Velocity", 0.0)
                    .withWidget(BuiltInWidgets.kNumberBar)
                    .withProperties(Map.of("Min", 0, "Max", Constants.MODULE_MAX_VELOCITY))
                    .withSize(1, 2)
                    .getEntry();
            swerveModuleAngles[i] = mTab
                    .add("Mod " + i + " Angle", 0.0)
                    .withWidget(BuiltInWidgets.kGyro)
                    .withSize(2, 2)
                    .getEntry();
        }
        gyroAngle = mTab
                .add("Swerve Gyro", 0.0)
                .withWidget(BuiltInWidgets.kGyro)
                .withSize(2, 2)
                .getEntry();
        swerveState = mTab
                .add("Swerve State", "DRIVE")
                .withSize(2, 1)
                .getEntry();
    }

    @Override
    public void update() {
        SwerveModuleState[] moduleStates = mSwerve.getSwerveModuleStates();
        int counter = 0;
        for (SwerveModuleState state : moduleStates) {
            double speed = state.speedMetersPerSecond;
            double angle;
            if(speed < 0.0){
                angle = AngleNormalization.getAbsoluteAngleDegree(state.angle.getDegrees() + 180.0);
            } else {
                angle = AngleNormalization.getAbsoluteAngleDegree(state.angle.getDegrees());
            }
            
            swerveModuleVelocities[counter].setDouble(speed);
            swerveModuleAngles[counter].setDouble(angle);
            counter++;
        }
        gyroAngle.setDouble(mSwerve.getYaw());
        swerveState.setString(mSwerve.getState().toString());
    }
}
