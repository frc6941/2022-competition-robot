package frc.robot.coordinators;

import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.UpdateManager.Updatable;
import org.frcteam2910.common.util.InterpolatingDouble;
import org.frcteam2910.common.util.InterpolatingTreeMap;
import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotStateEstimator implements Updatable {
    private SJTUSwerveMK5Drivebase mDrivebase = SJTUSwerveMK5Drivebase.getInstance();
    private TurretSubsystem mTurretSubsystem = TurretSubsystem.getInstance();
    private VisionSubsystem mVision = VisionSubsystem.getInstance();

    private InterpolatingTreeMap<InterpolatingDouble, RigidTransform2> fieldToVehicle;

    private RobotStateEstimator() {
        this.fieldToVehicle = new InterpolatingTreeMap<InterpolatingDouble, RigidTransform2>(Constants.kUniBufferSize);
        this.reset(0.0, new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    }

    private static RobotStateEstimator instance;

    public static RobotStateEstimator getInstance() {
        if (instance == null) {
            instance = new RobotStateEstimator();
        }
        return instance;
    }

    public synchronized void reset(double time, Pose2d pose) {
        this.fieldToVehicle.clear();
        this.mDrivebase.resetOdometry(pose);
        this.fieldToVehicle.put(new InterpolatingDouble(time), new RigidTransform2(
                new Vector2(pose.getX(), pose.getY()), Rotation2.fromDegrees(pose.getRotation().getDegrees())));
    }

    public synchronized Pose2d getFieldToVehicle(double time) {
        RigidTransform2 tempResult = this.fieldToVehicle.getInterpolated(new InterpolatingDouble(time));
        return new Pose2d(tempResult.translation.x, tempResult.translation.y,
                Rotation2d.fromDegrees(tempResult.rotation.toDegrees()));
    }

    public synchronized void addFieldToVehicle(double time, Pose2d observation) {
        if(this.fieldToVehicle.size() > Constants.kUniBufferSize){
            this.fieldToVehicle.remove(fieldToVehicle.firstKey());
        }
        this.fieldToVehicle.put(new InterpolatingDouble(time),
                new RigidTransform2(new Vector2(observation.getX(), observation.getY()),
                        Rotation2.fromDegrees(observation.getRotation().getDegrees())));
    }

    public synchronized Transform2d getFieldToVehicleDelta(double time, double dt) {
        Pose2d previousPose = getFieldToVehicle(time - dt);
        Pose2d currentPose = getFieldToVehicle(time);
        return currentPose.minus(previousPose);
    }

    public synchronized Translation2d getFieldOrientedVelocity(double time, double dt) {
        Pose2d previousPose = getFieldToVehicle(time - dt);
        Pose2d currentPose = getFieldToVehicle(time);
        return currentPose.minus(previousPose).getTranslation().times(1 / dt);
    }

    @Override
    public void update(double time, double dt) {
        Pose2d drivebasePose = this.mDrivebase.getPose();
        if(drivebasePose != null){
            this.addFieldToVehicle(time, drivebasePose);
        }
    }
}
