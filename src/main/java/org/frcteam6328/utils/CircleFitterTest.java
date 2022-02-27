package org.frcteam6328.utils;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.FieldConstants;

public class CircleFitterTest {
    public static void main(String[] args) {
        long startTime = System.currentTimeMillis();
        Translation2d newCenter = CircleFitter.fit(FieldConstants.visionTargetDiameter / 2.0,
                List.of(
                        new Translation2d(1, 0),
                        new Translation2d(0, 1),
                        new Translation2d(-1, 0),
                        new Translation2d(0, -1)),
                0.005);
        long endTime = System.currentTimeMillis(); // 获取结束时间
        System.out.println("Run Time:" + (endTime - startTime) + "ms");
    }

}
