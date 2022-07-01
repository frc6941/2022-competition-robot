package frc.robot.coordinators;

import java.lang.reflect.Field;

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
import frc.robot.FieldConstants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotStateEstimator implements Updatable {
    private SJTUSwerveMK5Drivebase mDrivebase = SJTUSwerveMK5Drivebase.getInstance();
    private TurretSubsystem mTurretSubsystem = TurretSubsystem.getInstance();
    private VisionSubsystem mVision = VisionSubsystem.getInstance();

    private InterpolatingTreeMap<InterpolatingDouble, RigidTransform2> vehicleRelativeToField;
    private InterpolatingTreeMap<InterpolatingDouble, Vector2> visionRelativeToTarget;

    private RobotStateEstimator() {
        this.vehicleRelativeToField = new InterpolatingTreeMap<InterpolatingDouble, RigidTransform2>(
                Constants.kUniBufferSize);
        this.visionRelativeToTarget = new InterpolatingTreeMap<InterpolatingDouble, Vector2>(Constants.kUniBufferSize);
        this.reset(0.0, new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    }

    private static RobotStateEstimator instance;

    public static RobotStateEstimator getInstance() {
        if (instance == null) {
            instance = new RobotStateEstimator();
        }
        return instance;
    }

    public synchronized void addVehicleRelativeToField(double time, Pose2d observation) {
        if (this.vehicleRelativeToField.size() > Constants.kUniBufferSize) {
            this.vehicleRelativeToField.remove(vehicleRelativeToField.firstKey());
        }
        this.vehicleRelativeToField.put(new InterpolatingDouble(time),
                new RigidTransform2(new Vector2(observation.getX(), observation.getY()),
                        Rotation2.fromDegrees(observation.getRotation().getDegrees())));
    }

    public synchronized void addVisionRelativeToField(double time, Pose2d observation) {
        if (this.visionRelativeToTarget.size() > Constants.kUniBufferSize) {
            this.visionRelativeToTarget.remove(visionRelativeToTarget.firstKey());
        }
        this.visionRelativeToTarget.put(new InterpolatingDouble(time),
                new Vector2(observation.getX(), observation.getY()));
    }

    public synchronized void reset(double time, Pose2d pose) {
        this.vehicleRelativeToField.clear();
        this.mDrivebase.resetOdometry(pose);
        this.vehicleRelativeToField.put(new InterpolatingDouble(time), new RigidTransform2(
                new Vector2(pose.getX(), pose.getY()), Rotation2.fromDegrees(pose.getRotation().getDegrees())));
    }

    public synchronized Pose2d getVehicleRelativeToField(double time) {
        RigidTransform2 tempResult = this.vehicleRelativeToField.getInterpolated(new InterpolatingDouble(time));
        return new Pose2d(tempResult.translation.x, tempResult.translation.y,
                Rotation2d.fromDegrees(tempResult.rotation.toDegrees()));
    }

    private synchronized Translation2d getVisionToVehicle() {
        // As the forward direction is taken as 0 degrees, the angle need +90 to fit the
        // unit circle.
        Rotation2d turretAngle = Rotation2d.fromDegrees(this.mTurretSubsystem.getTurretAngle() + 90.0);
        return new Translation2d(turretAngle.getCos(), turretAngle.getSin())
                .times(Constants.VisionConstants.Turret.TURRET_RING_RADIUS);
    }

    // TODO: Need a lot of testing
    private synchronized Pose2d getVisionEstimatedVehicleRelativeToField(double time) {
        Vector2 visionTargetVector = this.visionRelativeToTarget.getInterpolated(new InterpolatingDouble(time));       
        double fieldOrientedTargetAngle = this.mDrivebase.getYaw().getDegrees() + this.mTurretSubsystem.getTurretAngle();
        return new Pose2d(
                FieldConstants.hubCenter.minus(
                        new Translation2d(
                                visionTargetVector.length,
                                Rotation2d.fromDegrees(fieldOrientedTargetAngle))),
                Rotation2d.fromDegrees(this.mDrivebase.getYaw().getDegrees()));
    }

    public synchronized Transform2d getVehicleRelativeToFieldDelta(double time, double dt) {
        Pose2d previousPose = getVehicleRelativeToField(time - dt);
        Pose2d currentPose = getVehicleRelativeToField(time);
        return currentPose.minus(previousPose);
    }

    public synchronized Translation2d getVehicleFieldOrientedVelocity(double time, double dt) {
        Pose2d previousPose = getVehicleRelativeToField(time - dt);
        Pose2d currentPose = getVehicleRelativeToField(time);
        return currentPose.minus(previousPose).getTranslation().times(1 / dt);
    }

    public synchronized double getVehicleRelativeToTargetLinearVelocity(double time, double dt){
        Translation2d relativePosition = FieldConstants.hubCenter.minus(getVehicleRelativeToField(time).getTranslation());
        Rotation2d theta = new Rotation2d(relativePosition.getX(), relativePosition.getY());
        Translation2d currentVelocity = getVehicleFieldOrientedVelocity(time, dt);
        return currentVelocity.getX() * theta.getCos() + currentVelocity.getY() * theta.getSin();
    }

    public synchronized double getVehicleRelativeToTargetAngularVelocity(double time, double dt){
        return getVehicleRelativeToTargetLinearVelocity(time, dt) / FieldConstants.hubCenter.minus(getVehicleRelativeToField(time).getTranslation()).getNorm();
    }

    public static void main(String[] args) {
        
        Translation2d relativePosition = new Translation2d(2.0, 0.0).minus(new Translation2d(0, 1));
        Rotation2d theta = new Rotation2d(relativePosition.getX(), relativePosition.getY());
        Translation2d currentVelocity = new Translation2d(-1, -1);
        System.out.println(currentVelocity.getX() * theta.getSin() + currentVelocity.getY() * theta.getCos());
    }

    @Override
    public void update(double time, double dt) {
    }
}
