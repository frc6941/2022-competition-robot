package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.frcteam6941.localization.Localizer;
import org.frcteam6941.looper.UpdateManager.Updatable;
import org.frcteam6941.vision.CameraConstants;
import org.frcteam6941.vision.EstimatedPoseProvider;
import org.frcteam6941.vision.EstimatedPoseProvider.EstimatedPoseWithDistance;
import org.frcteam6941.vision.PhotonCameraEstimatedPoseProvider;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.utils.PolynomialRegression;

public class RobotStateEstimator implements Updatable {
    private Localizer localizer = SJTUSwerveMK5Drivebase.getInstance().getLocalizer();
    private SJTUSwerveMK5Drivebase drivebase = SJTUSwerveMK5Drivebase.getInstance();
    private ArrayList<EstimatedPoseProvider> providers = new ArrayList<>();

    private PolynomialRegression xyStdDevModel = new PolynomialRegression(
            new double[] {
                    0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
                    3.223358, 4.093358
            },
            new double[] {
                    0.005, 0.01, 0.015, 0.02, 0.5, 0.7, 0.15, 0.25, 0.35, 0.50
            },
            1);

    private static RobotStateEstimator instance;

    public static RobotStateEstimator getInstance() {
        if (instance == null) {
            instance = new RobotStateEstimator();
        }
        return instance;
    }

    private RobotStateEstimator() {
        for (CameraConstants camera : Constants.SUBSYSTEM_VISION.CAMERA_CONSTANTS) {
            providers.add(new PhotonCameraEstimatedPoseProvider(camera, FieldConstants.LAYOUT));
        }
    }

    @Override
    public synchronized void read(double time, double dt) {
        // Auto Generated Method
    }

    @Override
    public synchronized void update(double time, double dt) {
        for (EstimatedPoseProvider poseProvider : providers) {
            try {
                Optional<EstimatedPoseWithDistance> ePose = poseProvider.getEstimatedPose(localizer.getLatestPose());
                if (ePose.isPresent()) {
                    if (Math.abs(ePose.get().pose.estimatedPose.toPose2d().getRotation().getDegrees())  > 1e-4  && ePose.get().distance < 3.5) {
                        Logger.getInstance().recordOutput("Vision/" + poseProvider.getName() + " hasTarget", true);
                        EstimatedPoseWithDistance eposeWithDistance = ePose.get();
                        Logger.getInstance().recordOutput("Vision/" + poseProvider.getName() + " Estimate",
                                eposeWithDistance.pose.estimatedPose.toPose2d());
                        double xyStdDev = xyStdDevModel.predict(eposeWithDistance.distance);
                        localizer.addMeasurement(eposeWithDistance.pose.timestampSeconds,
                                eposeWithDistance.pose.estimatedPose.toPose2d(),
                                new Pose2d(xyStdDev, xyStdDev, Rotation2d.fromDegrees(20.0)));
                    }
                } else {
                    Logger.getInstance().recordOutput("Vision/" + poseProvider.getName() + " hasTarget", false);
                }

                if (ePose.isPresent()) {
                    Logger.getInstance().recordOutput("Vision/Measured X", ePose.get().pose.estimatedPose.getX());
                    Logger.getInstance().recordOutput("Vision/Measured Y", ePose.get().pose.estimatedPose.getY());
                    Logger.getInstance().recordOutput("Vision/True X", drivebase.getPose().getX());
                    Logger.getInstance().recordOutput("Vision/True Y", drivebase.getPose().getY());
                    Logger.getInstance().recordOutput("Vision/Distance", ePose.get().distance);

                }

            } catch (Exception e) {
                Logger.getInstance().recordOutput("Vision/" + poseProvider.getName() + " Estimate",
                        new Pose2d(-1000.0, -1000.0, Rotation2d.fromDegrees(0.0)));
                throw e;
            }

        }
    }

    @Override
    public synchronized void write(double time, double dt) {
        // Auto Generated Method
    }

    @Override
    public synchronized void telemetry() {
    }

    @Override
    public synchronized void start() {
        // Auto Generated Method
    }

    @Override
    public synchronized void stop() {
        // Auto Generated Method
    }

    @Override
    public synchronized void simulate(double time, double dt) {
        // Auto Generated Method
    }
}
