package org.frcteam6941.utils;

import java.lang.reflect.Field;

import frc.robot.Constants;
import frc.robot.FieldConstants;

public class VisionTargetCalculations {
    public static double calculateYaw(
            double offsetCenterX, double targetCenterX, double horizontalFocalLength) {
        return Math.toDegrees(Math.atan((offsetCenterX - targetCenterX) / horizontalFocalLength));
    }

    public static double calculatePitch(
            double offsetCenterY, double targetCenterY, double verticalFocalLength) {
        return -Math.toDegrees(Math.atan((offsetCenterY - targetCenterY) / verticalFocalLength));
    }

    public static void main(String[] args) {
        System.out.println((FieldConstants.visionTargetHeightUpper + FieldConstants.visionTargetHeightLower) / 2.0);
    }
}
