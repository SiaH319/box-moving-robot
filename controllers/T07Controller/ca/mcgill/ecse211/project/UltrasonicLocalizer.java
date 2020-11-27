package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Navigation.*;
import static ca.mcgill.ecse211.project.Resources.*;
import static java.lang.Math.cos;
import static java.lang.Math.round;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;
import static simlejos.ExecutionController.*;
import java.util.ArrayList;
import java.util.ListIterator;
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


  public static boolean isObject = false;
  public static boolean isObs = false;

  public static boolean nextToObs = false;
  public static int whereObs; //0: left, 1: right, 2: up, 3: down
  public static boolean isObjectFurther = false;
  public static int isRightTurn = 1; // leftTurn = -1

  public static ArrayList<Double> objectUS = new  ArrayList<Double>();
  public static ArrayList<Double> objectAngle = new ArrayList<Double>();
  public static double angleToMove;
  public static double distToMove;

  public static int isLtoR = 1; //isRtoL = -1
  public static boolean isDecreasingY = true;

  /*
  private static double LLSZ_X = Navigation.lowerLeftSzgX;
  private static double LLSZ_Y = Navigation.lowerLeftSzgY;
  private static double URSZ_X = Navigation.upperRightSzgX;
  private static double URSZ_Y = Navigation.upperRightSzgY;
  private static double LLR_X = Navigation.lowerLeftRampX;
  private static double LLR_Y = Navigation.lowerLeftRampY;
  private static String clstSzg = Navigation.closestSzg;
   */

  //hard coding for testing purposes
  private static double LLSZ_X = 6;
  private static double LLSZ_Y = 5;
  private static double URSZ_X = 10;
  private static double URSZ_Y = 9;
  private static double LLR_X = 7;
  private static double LLR_Y = 7;
  private static String clstSzg = "UR";


  public static ArrayList<Point> cleanPoint = new  ArrayList<Point>();
  public static ArrayList<Point> obsPoint = new ArrayList<Point>();
  public static Point currPt;
  public static Point nextPt;

  static Point rampLL1 = new Point(LLR_X, LLR_Y);
  static Point rampLL2 = new Point(LLR_X, LLR_Y + 1);

  static int xPt = 0;
  static int yPt = 0;
  static void travelSearch() {
    int maxX = (int) (URSZ_X - LLSZ_X - 1);;
    int maxY = (int) (URSZ_Y - LLSZ_Y - 1);;

    obsPoint.add(rampLL1);
    obsPoint.add(rampLL2);

    if (clstSzg == "UL") {
      currPt = new Point(LLSZ_X, URSZ_Y - 1);
      isLtoR = 1; //isRtoL = -1
      isRightTurn = 1; // leftTurn = -1

      isDecreasingY = true;
    }

    if (clstSzg == "UR") {
      currPt = new Point(URSZ_X - 1, URSZ_Y - 1);
      isLtoR = -1; //isRtoL = -1
      isRightTurn = -1; // leftTurn = -1

      isDecreasingY = true;
    }

    if (clstSzg == "LL") {
      currPt = new Point(LLSZ_X, LLSZ_Y);
      isLtoR = 1; //isRtoL = -1
      isRightTurn = -1; // leftTurn = -1

      isDecreasingY = false;
    }

    if (clstSzg == "LR") {
      currPt = new Point(URSZ_X - 1, LLSZ_Y);
      isLtoR = -1; //isRtoL = -1
      isRightTurn = 1; // leftTurn = -1

      isDecreasingY = false;
    }

    backWardAdjust();

    while (true) {

      if (isLtoR == 1) {
        xPt = 1;
      }

      else {
        xPt = -1;
      }



      // searchObject();
      /*
      if (isObject) {
        moveToObject();
        break;
      }*/
      while(true) {
        //cleanPoint.add(currPt);
        nextPt = new Point(currPt.x + xPt, currPt.y);

        System.out.println("current tile LL = " + currPt);
        closeToObs(); //check if front tile is out of search zone or has an obstacle/ramp
        if (isLtoR != 1 && obsPoint.contains(new Point(currPt.x - 1, currPt.y)))  {
          System.out.println("avoid obstacle/ramp");
          break;
        }

        else if (isLtoR == 1 && obsPoint.contains(new Point(currPt.x + 1, currPt.y)))  {
          System.out.println("avoid obstacle/ramp");
          break;
        }


        if (!NextTileInSRZ()) {
          System.out.println("stay inside the search zone");
          break;
        }

        moveByOneTile();
        currPt = new Point(currPt.x + xPt, currPt.y);        

      }
      turn();
      //}
      /* if (nextToObs || isObs) {
      turn();
    }*/
    }
  }

  public static void backWardAdjust() {
    setSpeed(FORWARD_SPEED);
    leftMotor.rotate(convertDistance(-TILE_SIZE / 4), true);
    rightMotor.rotate(convertDistance(-TILE_SIZE / 4), false);
    System.out.println("Move backward a bit");
  }



  public static void turn() {
    System.out.println("turning");
    // isRightTurn = 1;  leftTurn = -1
    if (isDecreasingY) {
      yPt = -1;
    }
    else {
      yPt = 1;

    }


    double turnAngle;
    if (isRightTurn == 1) {
      turnAngle = 90;
    }
    else {
      turnAngle = -90;
    }
    turnBy(turnAngle);

    //   searchObject();
    /*
    if (isObject) {
           todo whereObs == 0)  {//0: left, 1: right, 2: up, 3: down)

        }
      moveToObject();
      // if block found, stop
      //@TODO: OBJECT FOUND?
    }*/

    moveByOneTile();
    currPt = new Point(currPt.x, currPt.y + yPt);
    System.out.println("current tile LL = " + currPt);

    turnBy(turnAngle);
    backWardAdjust();

    //    searchObject();
    /*
    if (isObject) {
      moveToObject();
      // if block found, stop
      //@TODO: OBJECT FOUND?
    }
     */
    isRightTurn = isRightTurn * (-1);
    isLtoR = isLtoR* (-1);
  }

  /**
   * Checks whether the robot is in the search zone.
   * 
   * @return true if in search zone.
   */
  public static boolean NextTileInSRZ() {
    if (nextPt.x >= LLSZ_X && nextPt.x  < URSZ_X 
        && nextPt.y >= LLSZ_Y && nextPt.y < URSZ_Y) {


      boolean one = currPt.x >= LLSZ_X;

      boolean two = currPt.x  <= URSZ_X ;

      boolean three = currPt.x >= LLSZ_X;

      boolean four = currPt.y <= URSZ_Y;

      System.out.println(
          "currPt.x >= LLSZ_X = " + one + 
          ", currPt.x  <= URSZ_X = " + two 
          + ", currPt.y > LLSZ_Y = " + three 
          + ", currPt.y < URSZ_Y = " + four);
      return true;
    } 
    else {
      return false;

    }
  }


  public static void closeToObs() { //detect whether their is an obstacle/ramp in the front tile
    Point left = new Point(currPt.x - 1, currPt.y);
    Point down = new Point(currPt.x, currPt.y - 1);
    Point right = new Point(currPt.x + 1, currPt.y);
    Point up = new Point(currPt.x, currPt.y + 1);
    nextToObs = false;

    if (obsPoint.contains(left)) {
      nextToObs = true;
      whereObs = 0; //0: left, 1: right, 2: up, 3: down
      System.out.println("obstacle on the left ");
    }
    else if (obsPoint.contains(right)) {
      nextToObs = true;
      whereObs = 1; //0: left, 1: right, 2: up, 3: down

    }
    /*else if (obsPoint.contains(up)) {
      nextToObs = true;
      wherenextToObs = 2; //0: left, 1: right, 2: up, 3: down
    }
    else if (obsPoint.contains(down)) {
      nextToObs = true;
      wherenextToObs = 3; //0: left, 1: right, 2: up, 3: down
    }*/
    whereObs = 10;
  }


  /**
   * Detect object (box or obstacle) positioned inside the front tile
   * when the sensor of the robot is positioned in the middle of the current tile
   * 
   * This method assumes that there is at most one object in a tile
   */

  public static void searchObject() {
    isObject = false;
    if (objectUS != null && objectAngle != null) {
      objectUS.clear();
      objectAngle.clear();
    }
    turnBy(-45);
    odometer.setTheta(0);
    double ideal;
    double actual;
    double error = 15;
    angleToMove = 0;
    distToMove = 0;

    leftMotor.rotate(convertAngle(90), true);
    rightMotor.rotate(convertAngle(-90), true);

    while (round(odometer.getXyt()[2]) != 90) {

      if (0 < odometer.getXyt()[2] && odometer.getXyt()[2] <= 26.6) {
        ideal =  1.5 * TILE_SIZE * 100  / (3 * cos(toRadians(45 - odometer.getXyt()[2])) 
            * Math.tan(toRadians(45 - odometer.getXyt()[2])));
        actual = getDistance();
        objectFound(actual, ideal, error);
      }


      if (26.6 <= odometer.getXyt()[2] && odometer.getXyt()[2] <= 45) {
        ideal = 1.5 * TILE_SIZE * 100  / (cos(toRadians(45-odometer.getXyt()[2])));
        actual = getDistance();

        objectFound(actual, ideal, error);

      }


      if (45 <= odometer.getXyt()[2] && odometer.getXyt()[2] <= 63.4) {
        ideal = 1.5 * TILE_SIZE * 100 / (cos(toRadians(odometer.getXyt()[2]-45)));
        actual = getDistance();

        objectFound(actual, ideal, error);

      }


      if (63.4 <= odometer.getXyt()[2] && odometer.getXyt()[2] <= 90) {
        ideal =  1.5 * TILE_SIZE * 100  / (3 * cos(toRadians(odometer.getXyt()[2] -45)) 
            * Math.tan(toRadians(odometer.getXyt()[2] - 45)));
        actual = getDistance();

        objectFound(actual, ideal, error);

      }

    }
    if (isObject) {
      if (objectUS.size() <= 10 && objectAngle.size() <= 10) { // not enough number of sample
        isObject = false;
        System.out.println("no object found in the front tile");
      }

      else {

        int mid = medianIndex(objectUS);
        angleToMove = objectAngle.get(mid + 7);
        distToMove = minDist(objectUS) + 2;
        //if (round(distToMove)>=31) {
        //isObject = false;
        System.out.println(distToMove);

      }
    }
    else {
      isObject = false;

      System.out.println("no object found in the front tile");
    }
    turnBy(-45);

  }


  public static void objectFound(double act, double id, double err) {
    if (round(act) < round(id) - err) {
      isObject = true;
      objectUS.add(act);
      objectAngle.add(odometer.getXyt()[2]); 
    }
  }

  public static void moveByOneTile() {
    setSpeed(FORWARD_SPEED);
    leftMotor.rotate(convertDistance(TILE_SIZE), true);
    rightMotor.rotate(convertDistance(TILE_SIZE), false);
  }

  public static int medianIndex(ArrayList<Double> arr) {
    int index = 0;
    if (arr != null) {
      round(arr.size() / 2);
    }

    return index;
  }

  public static Double minDist(ArrayList<Double> dist) {
    Double smallest = (double) 0;
    if (dist != null) {
      smallest = dist.get(0);

      for (int i = 1; i < dist.size(); i++) {
        if (dist.get(i) < smallest) {
          smallest = dist.get(i);
        }
      }
    }

    return smallest;
  }

  public static void moveToObject() {
    double newAngleToMove = 0;
    double newDistToMove = 0;
    double oldAngleToMove = 0;
    double oldDistToMove = 0;


    oldAngleToMove = angleToMove;
    oldDistToMove = distToMove;

    turnBy(angleToMove - 45);
    setSpeed(FORWARD_SPEED);
    leftMotor.rotate(convertDistance(distToMove / 100), true);
    rightMotor.rotate(convertDistance(distToMove / 100), false);
    isObs = false;

    if (distToMove >= 30) {
      searchObject();

      newAngleToMove = angleToMove;
      newDistToMove = distToMove - 2.5;

      turnBy(newAngleToMove - 45);
      setSpeed(FORWARD_SPEED);

      leftMotor.rotate(convertDistance(newDistToMove / 100), true);
      rightMotor.rotate(convertDistance(newDistToMove / 100), false); 
    }


    if (Navigation.blockOrObstacle()) {
      System.out.println("distToMove = " + distToMove);
      System.out.println("A block is detected");
      leftMotor.stop();
      rightMotor.stop();

    }

    else {
      System.out.println("An obstacle is detected");
      isObs = true;
      isObjectFurther = false;

      if (newAngleToMove != 0 && newDistToMove != 0) {
        moveAwayFromObject(newDistToMove, newAngleToMove);
        isObjectFurther = true;
      }
      moveAwayFromObject(oldDistToMove, oldAngleToMove);
    }
  }

  /**
   * revert method moveToObject()
   * this method is called if an obstacle is detected (isObs == true)
   * 
   * */
  public static void moveAwayFromObject(double dist, double angle) {
    setSpeed(FORWARD_SPEED);
    System.out.println(dist);
    leftMotor.rotate(convertDistance(-dist / 100), true);
    rightMotor.rotate(convertDistance(-dist / 100), false); 
    turnBy(-(angle - 45));
  }



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

  /**
   * Reads the top ultrasonic distance.
   * @return Distance in cm.
   */
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
   * 
   * @return angle required to face 0.
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
   * @return Distance read by the front US sensor in cm.
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