import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;

public class IntakerTest {
    static final double DELTA = 0.001;

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0);
    }

    @AfterEach
    void shutdown() throws Exception {

    }

    @Test
    void intakerRunningTest() {

    }

    @Test
    void intakerSpittingTest() {
        
    }
}
