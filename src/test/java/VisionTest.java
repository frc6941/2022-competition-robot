
import com.team254.frc2020.limelight.CameraResolution;
import com.team254.frc2020.limelight.undistort.CameraConstants;

import org.ejml.simple.SimpleMatrix;
import org.frcteam6941.vision.VisionConfiguration;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class VisionTest {
    private class VisionPoint {
        public final double x;
        public final double y;

        public VisionPoint(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    private class CameraPosition {
        public final double height;
        public final Rotation2d rotation;

        public CameraPosition(double x, Rotation2d y) {
            this.height = x;
            this.rotation = y;
        }
    }

    private static Translation2d solveCameraToTargetTranslation(VisionPoint corner, double goalHeight,
            CameraPosition cameraPosition) {
        double halfWidthPixels = 640 / 2.0;
        double halfHeightPixels = 480 / 2.0;
        Rotation2d fovHorizontal = Rotation2d.fromDegrees(59.6);
        Rotation2d fovVertical = Rotation2d.fromDegrees(49.7);
        double vpw = 2.0 * Math.tan(fovHorizontal.getRadians() / 2.0);
        double vph = 2.0 * Math.tan(fovVertical.getRadians() / 2.0);
        double nY = -((corner.x - halfWidthPixels - 0.0)
                / halfWidthPixels);
        double nZ = -((corner.y - halfHeightPixels - 0.0)
                / halfHeightPixels);

        Translation2d xzPlaneTranslation = new Translation2d(1.0, vph / 2.0 * nZ)
                .rotateBy(cameraPosition.rotation);
        double x = xzPlaneTranslation.getX();
        double y = vpw / 2.0 * nY;
        double z = xzPlaneTranslation.getY();

        double differentialHeight = cameraPosition.height - goalHeight;
        if ((z < 0.0) == (differentialHeight > 0.0)) {
            double scaling = differentialHeight / -z;
            double distance = Math.hypot(x, y) * scaling;
            Rotation2d angle = new Rotation2d(x, y);
            return new Translation2d(distance * angle.getCos(),
                    distance * angle.getSin());
        }
        return null;
    }

    public static void main(String[] args) {
        double[][] testCameraIntrinsic = new double[][] {
                { 729.2205288939331, 0.0, 484.20682264918315 },
                { 0.0, 728.0294742172647, 324.54740828467595 },
                { 0.0, 0.0, 1.0 }
        };
        double[] cameraDistortion = new double[] { -0.3580226107916401, 3.1574617019213114, 0.02323167066358097,
                -0.013430391090519293, -8.016922034682826 };
        VisionConfiguration testConfiguration = new VisionConfiguration(
                0,
                "Test", 0.85, new Pose2d(), Rotation2d.fromDegrees(30), CameraResolution.F_960x720,
                new CameraConstants(cameraDistortion, testCameraIntrinsic));
        
        double u = 600.0;
        double v = 300.0;
        System.out.println(new SimpleMatrix(testConfiguration.getCameraConstants().getCameraMatrix()).invert().mult(new SimpleMatrix(new double[][]{{u}, {v}, {1}})));
    }
}
