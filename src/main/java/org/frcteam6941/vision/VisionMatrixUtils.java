package org.frcteam6941.vision;

import org.ejml.simple.SimpleMatrix;

public class VisionMatrixUtils {
    public TargetInfo calculateTargetInfo(double u, double v, VisionConfiguration visionConfiguration) {
        SimpleMatrix result = new SimpleMatrix(visionConfiguration.getCameraConstants().getCameraMatrix()).invert()
                .mult(new SimpleMatrix(new double[][] { { u }, { v }, { 1 } }));
        return new TargetInfo(result.get(1, 1), result.get(2, 1));
    }

    public static void main(String[] args) {
    }
}
