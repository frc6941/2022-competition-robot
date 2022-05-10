package org.frcteam6941.vision;

import java.lang.annotation.Target;

import com.team254.frc2020.limelight.undistort.CameraConstants;

import org.ejml.simple.SimpleMatrix;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Translation2d;

public class VisionMatrixUtils {
    public TargetInfo calculateTargetInfo(double u, double v, VisionConfiguration visionConfiguration) {
        SimpleMatrix result = new SimpleMatrix(visionConfiguration.getCameraConstants().getCameraMatrix()).invert()
                .mult(new SimpleMatrix(new double[][] { { u }, { v }, { 1 } }));
        return new TargetInfo(result.get(1, 1), result.get(2, 1));
    }

    public static void main(String[] args) {
    }
}
