package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Navigation.*;
import static ca.mcgill.ecse211.project.Resources.*;

import ca.mcgill.ecse211.playingfield.Point;

import static ca.mcgill.ecse211.playingfield.Point.*;

public class UltrasonicLocalizer {
  /** Buffer (array) to store US samples. */
  private static float[] usData = new float[usSensor.sampleSize()];
  // Ideal distance to wall
  static int d = 17;
  // Acceptable margin of error
  static int k = 1;
  // The data gathered from the odometer
  public static double[] odometerReading;
  // The distance remembered by the filter() method
  private static int prevDistance;
  // The number of invalid samples seen by filter() so far.
  private static int invalidSampleCount;
  // US localization variables
  private static int distance;
  private static double theta;
  private static double theta_1;
  private static double theta_2;

  /**
   * Main method of the UltrasonicLocalizer. Localizes the bot to 1,1.
   */
  public static void localize() {
    scanMin(120, 1.5); // get a rough idea of where one of the walls is
    // Make sure we're facing forwards.
    double curValue = getDistance();
    Navigation.turnBy(90);
    double nextValue = getDistance();
    System.out.println("Min: " + curValue + " cur: " + nextValue);
    if (nextValue - US_ERROR < curValue) {
      Navigation.turnBy(90);
    }
    // Robot should be facing forwards now. Time to run a better localization
    // algorithm.
    fallingEdge();
  }

  /**
   * Search method for the robot. It rotates between the start and end angles and
   * maps all unknown objects found with the ultrasonic sensor. It then adds all
   * these unknown objects' positions to the unknowns list in Resources.
   * 
   * @param startAngle Starting angle for the search (degrees).
   * @param endAngle   Ending angle for the search (degrees).
   */
  public static void search(double startAngle, double endAngle) {
  }


  // =========================================
  // ============ Helper Methods =============
  // =========================================

  /**
   * Performs falling edge localization (where robot starts facing away from
   * wall).
   */
  public static void fallingEdge() {
    System.out.println("[STATUS] Ultrasonic localization starting (FALLING EDGE)...");

    // Reset odometer; set motor speeds and acceleration
    odometer.setTheta(0);
    setSpeed(LOCAL_SPEED);
    setAcceleration(ACCELERATION);

    // Rotate while looking for wall
    leftMotor.forward();
    rightMotor.backward();

    // If distance to wall is within margin (d), get theta_1
    while (true) {
      distance = filter(readUsDistance()); // Current distance from US sensor
      odometerReading = odometer.getXyt(); // Current position from odometer
      theta = odometerReading[2]; // Current theta from odomoter

      if (distance < (d + k) && distance > k) {
        theta_1 = theta;
        System.out.println("=> Found THETA 1: " + theta_1);
        break;
      }
    }

    // Rotate in the opposite direction
    leftMotor.backward();
    rightMotor.forward();

    // Continue rotating
    // Do not take measurements until wall is out of range (>45cm)
    while (true) {
      distance = filter(readUsDistance()); // Current distance from US sensor
      odometerReading = odometer.getXyt(); // Current position from odometer
      theta = odometerReading[2]; // Current theta from odomoter

      if (distance > 45) {
        break;
      }
    }

    // Once distance to wall is within margin (d), get theta_2
    while (true) {
      distance = filter(readUsDistance()); // Current distance from US sensor
      odometerReading = odometer.getXyt(); // Current position from odometer
      theta = odometerReading[2]; // Current theta from odomoter

      if (distance < (d + k)) {
        theta_2 = theta;
        System.out.println("=> Found THETA 2: " + theta_2);
        break;
      }
    }

    // Calculate angle to zero heading
    double angleToZero = getAngleToZero(theta_1, theta_2);

    // Rotate to zero heading
    System.out.println("=> Rotating by " + angleToZero);
    turnBy(angleToZero);
  }

  /**
   * Reads a value from the US sensor and filters out noise.
   * 
   * @return US Sensor reading.
   */
  public static int getDistance() {
    usSensor.fetchSample(usData, 0);
    //System.out.println(usData[0] * 100);
    return (int) (usData[0] * 100);
  }

  /**
   * This function oscillated the direction of the bot to try to find the nearest
   * minimum value from the ultrasonic sensor.
   * 
   * @param theta an angle to sweep within.
   */
  public static void scanMin(int theta, double step) {
    int angle = theta;
    boolean same = false;
    boolean same2 = false;
    Navigation.setSpeed(ROTATE_SPEED);
    Navigation.setAcceleration(ACCELERATION);

    // Time to narrow down the minimum value

    while (true) {
      // check left and right for which is smaller
      Navigation.turnBy(-angle);
      int leftSide = getDistance();
      Navigation.turnBy(2 * angle);
      int rightSide = getDistance();
      Navigation.turnBy(-angle); // Go back to center

      // Turn to the smaller side, repeat but with a smaller angle if they're the same
      if (leftSide < rightSide) {
        Navigation.turnBy(-angle); // turn back to left side
      } else if (rightSide < leftSide) {
        Navigation.turnBy(angle); // turn back to right side
      } else if (rightSide == leftSide) {
        if (same) { // same 3x in a row
          if (same2) {
            break;
          } else {
            same2 = true;
          }
        } else {
          same = true;
        }
      }
      angle /= step;
    }
  }

  /**
   * Calculates the angle to rotate for zero heading.
   * 
   * @param angle1 the first angle found in degrees
   * @param angle2 the second angle found in degrees
   */
  public static double getAngleToZero(double angle1, double angle2) {
    double angleToZero;
    if (angle1 <= angle2) {
      System.out.println("theta_1 < theta_2");
      angleToZero = (angle1 - angle2) / 2.0 - 230.0; // 225
    } else {
      angleToZero = (angle1 - angle2) / 2.0 - 50.0; // 45
    }

    // Account for wraparound
    if (angleToZero < 0.0) {
      angleToZero += 360.0;
    } else if (angleToZero > 360.0) {
      angleToZero -= 360.0;
    }

    return angleToZero;
  }

  /**
   * Filters US distance.
   * 
   * @return ultrasonic distance
   */
  public static int readUsDistance() {
    usSensor.fetchSample(usData, 0);
    return filter((int) (usData[0] * 100));
  }

  /**
   * Rudimentary filter - toss out invalid samples corresponding to null signal.
   * 
   * @param distance raw distance measured by the sensor in cm
   * @return the filtered distance in cm
   */
  static int filter(int distance) {
    if (distance >= MAX_SENSOR_DIST && invalidSampleCount < INVALID_SAMPLE_LIMIT) {
      // bad value, increment the filter value and return the distance remembered from
      // before
      invalidSampleCount++;
      return prevDistance;
    } else {
      if (distance < MAX_SENSOR_DIST) {
        invalidSampleCount = 0; // reset filter and remember the input distance.
      }
      prevDistance = distance;
      return distance;
    }
  }

  public static void setSpeeds(int leftSpeed, int rightSpeed) {
    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(rightSpeed);
  }
}
