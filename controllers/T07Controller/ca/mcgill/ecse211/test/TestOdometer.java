package ca.mcgill.ecse211.test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.mcgill.ecse211.project.Odometer;
import org.junit.jupiter.api.MethodOrderer.OrderAnnotation;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

/**
 * Tests the Odometer class. This test runs in Eclipse (right-click > Run as > Unit test) and
 * on the command line, not in Webots!
 * 
 * @author Younes Boubekeur
 */
@TestMethodOrder(OrderAnnotation.class)
public class TestOdometer {
  
  /** Tolerate up to this amount of error due to double imprecision. */ 
  private static final double ERROR_TOLERANCE = 0.01;
  
  // Indices to make tests easier to read
  private static final int X = 0;
  private static final int Y = 1;
  
  @Order(1) // Runs this test before all others since odometer values will be set by other tests
  @Test void testStartingPointIsNonZero() {
    var pos = Odometer.getOdometer().getXyt();
    assertTrue(pos[X] != 0 && pos[Y] != 0);
  }
  
  @Test void testSetYImplemented() {
    double expectedY = 100 * Math.random();
    Odometer.getOdometer().setY(expectedY);
    double actualY = Odometer.getOdometer().getXyt()[Y];
    assertEquals(expectedY, actualY, ERROR_TOLERANCE);
  }

  // We can add helper methods below to be used in the tests above

}
