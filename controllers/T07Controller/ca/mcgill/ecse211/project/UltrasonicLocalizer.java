package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Navigation.*;
import static ca.mcgill.ecse211.project.Resources.*;
import static simlejos.ExecutionController.*;

import ca.mcgill.ecse211.playingfield.*;

public class UltrasonicLocalizer {
  /** Buffer (array) to store US samples. */
  private static float[] usData = new float[usSensor.sampleSize()];
  private static float[] usDataTop = new float[usSensorTop.sampleSize()];
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
    System.out.println("[STATUS] Starting ultrasonic localization...");

    // Set motor speeds and acceleration
    setSpeeds(ROTATE_SPEED, ROTATE_SPEED);
    setAcceleration(ACCELERATION);

    // Begin rotating...
    leftMotor.forward();
    rightMotor.backward();

    // Rotate until facing away from the wall
    while (true) {
      distance = filter(readUsDistance()); // Current distance from US sensor
      if (distance > 55 && distance < MAX_SENSOR_DIST) {
        System.out.println("=> Now facing away from wall. Finding thetas...");
        odometer.setTheta(0);
        break;
      }
    }

    // Continue rotating while reading distance and theta
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

    // Stop motors (LightLocalizer will restart)
    leftMotor.stop();
    rightMotor.stop();

    System.out.println("=> Ultrasonic localization complete.");
  }

  // =========================================
  // ============ Helper Methods =============
  // =========================================

  /**
   * Reads a value from the US sensor and filters out noise.
   * 
   * @return US Sensor reading.
   */
  public static int getDistance() {
    usSensor.fetchSample(usData, 0);
    // System.out.println(usData[0] * 100);
    return (int) (usData[0] * 100);
  }

  public static int getDistanceTop() {
    usSensorTop.fetchSample(usDataTop, 0);
    // System.out.println(usData[0] * 100);
    return (int) (usDataTop[0] * 100);
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
      // System.out.println("theta_1 < theta_2");
      angleToZero = (angle1 - angle2) / 2.0 - 226.0; // 230
    } else {
      angleToZero = (angle1 - angle2) / 2.0 - 46.0; // 50
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
   * Reads an unfiltered US distance from the front us Sensor.
   * 
   * @return Distance read by the front US sensor.
   */
  public static int frontUSDistance() {
    usSensor.fetchSample(usData, 0);
    return (int) (usData[0] * 100);
  }

  /**
   * Reads an unfiltered US distance from the front us Sensor.
   * 
   * @return Distance read by the front US sensor.
   */
  public static int topUSDistance() {
    usSensorTop.fetchSample(usData, 0);
    return (int) (usData[0] * 100);
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