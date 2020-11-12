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
    //turn to the start angle (position 0,0)
    double dw = 0;
    double dr = 0;
    double ds_x = 0;
    double ds_y = 0;
    double theta_1 = 0;
    double theta_2 = 0;

    //Temporary hard coding values
    isRedTeam = true;
    szr.ur.x = 10;
    szr.ur.y = 9;

    szr.ll.x = 6;
    szr.ll.y = 5;

    tnr.ur.x = 6;
    tnr.ur.y = 8;

    rr.left.x = 9;
    rr.left.y = 7;

    if (isRedTeam) {
      //turnTo(startAngle);
      turnBy(-90);

      //distance between the current position and the wall
      dw = szr.ur.y - tnr.ur.y + 0.5;
      System.out.println("dw=" + dw);
      //distance between the current position and the ramp
      dr = rr.left.x - tnr.ur.x - 0.5;
      System.out.println("dr=" + dr);

      ds_x = szr.ur.x - tnr.ur.x - 0.5;
      System.out.println("ds_x=" + ds_x);

      ds_y = tnr.ur.y - szr.ll.y - 0.5;
      System.out.println("ds_y=" + ds_y);

      theta_1 = Math.toDegrees(Math.atan(dr / dw));
      System.out.println("theta_1=" + theta_1);

      theta_2 = Math.toDegrees(Math.atan(ds_y / ds_x));
      System.out.println("theta_2=" + theta_2);

    }

    //turn to target angle 
    setSpeed(ROTATE_SPEED);
    leftMotor.rotate(convertAngle(endAngle), true);
    rightMotor.rotate(convertAngle(-endAngle), true);
   
    //condition if its an object or not
     boolean isUnknown = false;
    //read US sensor, create a point and put it in the list of unkno`1wns
    //point position = robot position + us sensor reading

    int ideal = 0;
    odometer.setTheta(0);
    while ((int)odometer.getXyt()[2] != endAngle) {
    	
      int value = filter(readUsDistance());
      
      if (odometer.getXyt()[2] < theta_1) {
        ideal = (int) (TILE_SIZE * 100 * dw / Math.cos(Math.toRadians(odometer.getXyt()[2])));
      }
      else if (theta_1 <= odometer.getXyt()[2] && odometer.getXyt()[2] < 90) {
        ideal = (int) (TILE_SIZE * 100 * dr / Math.cos(Math.toRadians(odometer.getXyt()[2]-theta_1)));
      }
      else if (90 <= odometer.getXyt()[2] && odometer.getXyt()[2] <= (90 + theta_2)) {
        ideal = (int) (TILE_SIZE * 100 * ds_x / Math.cos(Math.toRadians(odometer.getXyt()[2]-90)));
      }
      else if ((90 + theta_2) < odometer.getXyt()[2] && odometer.getXyt()[2] <= 180) {
        ideal = (int) (TILE_SIZE * 100 * ds_y / Math.cos(Math.toRadians(odometer.getXyt()[2]-theta_2-90)));
      }
      
      int diff  = Math.abs(value - ideal);
      if(diff > 20 && value < ideal && difference(value)) {
    	  isUnknown = true;
      }
      System.out.println("ideal=" + ideal + "," + "actual=" + value + "at angle=>"+odometer.getXyt()[2]);


      if(isUnknown) {
    	  //  create a point from the value
    	  double robotX = odometer.getXyt()[0];
    	  double robotY = odometer.getXyt()[1];


    	  double x = 0;
    	  double y = 0;
    	  Point newPoint = new Point(x, y);
    	  unknowns.add(newPoint);
    	  isUnknown = false;
    }
      	System.out.println("size of array: " + unknowns.size());
    }
    
  }


  // =========================================
  // ============ Helper Methods =============
  // =========================================

  /**
   * method of finite differences
   * @param current
   * @return  
   */
  private static int old = 0;
  public static boolean difference(int current) {
	  int diff = Math.abs(current - old);
	  
	  if(old != 0) {
		  if(diff > 30) {
			  old = current;
			  return true;
		  }
		  else return false;
	  }
	  else{
		  old = current;
		  return false;
	  }
	  
	  
  }
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
