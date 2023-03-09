package frc.robot.motion;

import java.util.List;
import java.util.stream.Collectors;

import org.frcteam6941.pathplanning.astar.obstacles.InfiniteBarrierObstacle;
import org.frcteam6941.pathplanning.astar.obstacles.Obstacle;
import org.frcteam6941.pathplanning.astar.obstacles.PolygonObstacle;
import org.frcteam6941.pathplanning.astar.obstacles.RectangularObstacle;
import org.frcteam6941.pathplanning.astar.obstacles.InfiniteBarrierObstacle.AXIS;
import org.frcteam6941.pathplanning.astar.obstacles.InfiniteBarrierObstacle.DIRECTION;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import frc.robot.utils.AllianceFlipUtil;

public class FieldObstacles {
    private static InfiniteBarrierObstacle fieldYBarrierSmaller = new InfiniteBarrierObstacle(0.0, DIRECTION.SMALLER,
            AXIS.Y);
    private static InfiniteBarrierObstacle fieldYBarrierLarger = new InfiniteBarrierObstacle(FieldConstants.fieldLength,
            DIRECTION.GREATER, AXIS.Y);

    public static Obstacle[] getObstacles() {
        Obstacle minDriveX = AllianceFlipUtil.shouldFlip()
                ? new InfiniteBarrierObstacle(FieldConstants.Grids.outerX + 0.5, DIRECTION.SMALLER, AXIS.X)
                : new InfiniteBarrierObstacle(AllianceFlipUtil.apply(FieldConstants.Grids.outerX + 0.5),
                        DIRECTION.GREATER, AXIS.X);
        Obstacle boundaryChargingStation = new RectangularObstacle(
                List.of(FieldConstants.Community.chargeStationCornersBumpered).stream()
                        .map(translation -> AllianceFlipUtil.apply(translation))
                        .collect(Collectors.toList()).toArray(new Translation2d[0]));
        return new Obstacle[] {
                boundaryChargingStation, fieldYBarrierLarger, fieldYBarrierSmaller, minDriveX
        };
    }

}
