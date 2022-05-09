package org.frcteam6941.vision;

import com.team254.frc2020.limelight.undistort.CameraConstants;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.Translation2d;

public class VisionMatrixUtils {
    public TargetInfo calculateTargetInfo(double u, double v, VisionConfiguration visionConfiguration) {
        SimpleMatrix result = new SimpleMatrix(visionConfiguration.getCameraConstants().getCameraMatrix()).invert()
                .mult(new SimpleMatrix(new double[][] { { u }, { v }, { 1 } }));
        return new TargetInfo(result.get(1, 1), result.get(2, 1));
    }

    public Translation2d calculateTranslation(TargetInfo targetInfo, VisionConfiguration visionConfiguration, double targetHeight){
        double scale = (targetHeight - visionConfiguration.getHeight()) / targetInfo.getY();
        SimpleMatrix rotMatrix = new SimpleMatrix(new double[][] {
            {1, 0, 0},
            {0, visionConfiguration.getHorizontalPlaneToLens().getCos(), -visionConfiguration.getHorizontalPlaneToLens().getSin()},
            {0, visionConfiguration.getHorizontalPlaneToLens().getSin(), visionConfiguration.getHorizontalPlaneToLens().getCos()}
        });
        SimpleMatrix tvec = rotMatrix.invert().mult(new SimpleMatrix(targetInfo.getDoubleMatrix()));
        return new Translation2d(tvec.get(3, 1), tvec.get(1, 1)).times(scale);
    }
}
