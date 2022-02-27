package frc.robot.coordinators;

import java.util.Optional;

import org.frcteam2910.common.robot.UpdateManager.Updatable;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.VisionSubsystem;

public class RobotStateEstimator implements Updatable {
    SJTUSwerveMK5Drivebase mDrivebase = SJTUSwerveMK5Drivebase.getInstance();
    VisionSubsystem mVision = VisionSubsystem.getInstance();

    public static RobotStateEstimator getInstance(){
        if(instance == null){
            instance = new RobotStateEstimator();
        }
        return instance;
    }

    private RobotStateEstimator(){

    }

    private static RobotStateEstimator instance;


    @Override
    public void update(double time, double dt) {
        Optional<Translation2d> cameraToTargetTranslation = mVision.getCompensatedCameraToTargetAtTime(time);
        Optional<Pose2d> drivetrainPose = mDrivebase.getPoseAtTime(time);
        if (cameraToTargetTranslation.isPresent()) {
            SmartDashboard.putNumber("Camera To Target Translation X", cameraToTargetTranslation.get().getX());
            SmartDashboard.putNumber("Camera To Target Translation Y", cameraToTargetTranslation.get().getY());
            SmartDashboard.putNumber("Camera To Target Distance", cameraToTargetTranslation.get().getNorm());
        }

    }
}
