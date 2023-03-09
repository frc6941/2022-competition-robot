import static org.junit.jupiter.api.Assertions.assertEquals;

import org.frcteam6941.utils.InterpolatingTreeMap;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public class CalculationTest {
    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0);
    }

    @AfterEach
    void shutdown() throws Exception {

    }

    @Test
    void customInterpolatingDouble() {
        Pose2d pose1 = new Pose2d(0, 0, new Rotation2d(0.0));
        Pose2d pose2 = new Pose2d(1, 1, new Rotation2d(0.0));
        InterpolatingTreeMap<Double, Pose2d> testMap = new InterpolatingTreeMap<Double, Pose2d>(2);
        testMap.put(0.0, pose1);
        testMap.put(1.0, pose2);
        assertEquals(new Pose2d(0.5, 0.5, new Rotation2d(0.0)), testMap.getInterpolated(0.5, 0.5));
    }
}
