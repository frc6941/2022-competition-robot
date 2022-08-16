package frc.robot.shuffleboard;

import java.util.Optional;

import com.team254.lib.vision.AimingParameters;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.coordinators.Superstructure;

public class FieldView {
    private Field2d mField2d = new Field2d();
    private SJTUSwerveMK5Drivebase mSwerve = SJTUSwerveMK5Drivebase.getInstance();
    private RobotState mRobotState = RobotState.getInstance();

    private Pose2d[] mModulePoses = new Pose2d[4];
    private Pose2d mRobotPose = new Pose2d();

    public FieldView() {
        SmartDashboard.putData(mField2d);
    }

    private void updateSwervePoses() {
        if(mSwerve.getPose() != null) mRobotPose = mSwerve.getPose();
        else mRobotPose = new Pose2d();
        for (int i = 0; i < mModulePoses.length; i++) {
            Translation2d updatedPosition = mSwerve.getSwerveModulePositions()[i].rotateBy(mRobotPose.getRotation()).plus(mRobotPose.getTranslation());
            Rotation2d updatedRotation = mSwerve.getSwerveModuleStates()[i].angle.plus(mRobotPose.getRotation());
            if(mSwerve.getSwerveModuleStates()[i].speedMetersPerSecond < 0.0) {
                updatedRotation = updatedRotation.plus(Rotation2d.fromDegrees(180));;
            }
            mModulePoses[i] = new Pose2d(updatedPosition, updatedRotation);
        }
    }

    public void update() {
        updateSwervePoses();
        mField2d.setRobotPose(mRobotPose);
        mField2d.getObject("Swerve Modules").setPoses(mModulePoses);
        Translation2d target = Superstructure.getInstance().coreAimTargetRelative;
        mField2d.getObject("Target")
                .setPose(new Pose2d(mRobotPose.getTranslation().plus(target), new Rotation2d()));

        mField2d.getObject("Predicted Robot Pose").setPose(mRobotState.getPredictedFieldToVehicle(0.2).getWpilibPose2d());
    }
}