package frc.robot.motion;

import java.util.Optional;

import org.frcteam6941.pathplanning.astar.obstacles.Obstacle;
import org.frcteam6941.pathplanning.universal.Path;

import edu.wpi.first.math.geometry.Translation2d;

public interface PathProvider {
    boolean buildPath(Translation2d startingPosition, Translation2d endingPosition, Obstacle[] obstacles);
    void clear();
    Optional<Path> getPath();
}
