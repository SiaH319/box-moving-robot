package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import java.util.ArrayList;
import static ca.mcgill.ecse211.project.UltrasonicLocalizer.*;
import static java.lang.Math.*;

import ca.mcgill.ecse211.playingfield.*;
import static simlejos.ExecutionController.*;

public class Navigation {

  /** Do not instantiate this class. */
  private Navigation() {
  }

  /**
   * This function navigates to a given unknown object's position on the map and
   * checks if the object is a block or an obstacle. If it is a block, return true
   * and update the blocks list in Resources. Otherwise, return false and update
   * the obstacles list in Resources.
   * 
   * @param pt       Target object's position (point).
   * @param curr     Current position (point).
   * @param curTheta Current heading of the robot.
   * @return Returns true if the object is a block. False if it is an obstacle.
   */
  public static boolean validateBlock(Point pt, Point curr, double curTheta) {
    // It is safe to assume that there is nothing in the way as the block readings
    // are based on the unknowns list of objects that were all read radially from
    // the search position, so only objects with direct LOS were added to that list.

    // Find proxy point and navigate to it
    double dist = distanceBetween(curr, pt);
    double destAngle = getDestinationAngle(curr, pt);
    System.out.println(dist);
    turnBy(minimalAngle(curTheta, destAngle));
    // block width is 10cm, so "radius" would be 5 or more (trig). So 10cm would put
    // is within the 7cm margin.
    moveStraightFor(dist - (BLOCK_WIDTH / TILE_SIZE));
    double[] xyt = odometer.getXyt();
    Point current = new Point(xyt[0] / TILE_SIZE, xyt[1] / TILE_SIZE);
    turnTo(getDestinationAngle(current, pt));
    System.out.println("VALIDATING NOW");
    // Note: THE FOLLOWING PART ONLY WORKS IF WITHIN ~7CM (+-2cm) OF THE TARGET.
    int top = topUSDistance();
    int front = frontUSDistance();
    System.out.println("Top : " + top);
    System.out.println("Front x2 : " + front * 2);
    if (top > 2 * front && top > 10) {
      // 10 is used as it is bigger than the possible value top could see within the
      // 7m radius of an obstacle.

      // If top USS not seeing same-ish as front USS (block too short, wall isn't)
      blocks.add(new Block(pt, -1)); // Add as block with placeholder torque
      System.out.println("Is a block");
      return true;
    } else { // Both are seeing roughly the same object (within 7cm) (tall wall)
      obstacles.add(pt); // Add as obstacle
      System.out.println("Not a block");
      return false;
    }
  }

  /** Travels to the given destination. */
  public static void travelTo(Point destination) {
    double[] xyt = odometer.getXyt();
    Point currentLocation = new Point(xyt[0] / TILE_SIZE, xyt[1] / TILE_SIZE);
    double currentTheta = xyt[2];
    double destinationTheta = getDestinationAngle(currentLocation, destination);
    turnBy(minimalAngle(currentTheta, destinationTheta));
    moveStraightFor(distanceBetween(currentLocation, destination));
  }

  /**
   * Travels to a given point and stops if an obstacle is detected.
   * 
   * @param destination Point
   * @return True if reached destination, false if stopped.
   */
  public static boolean safeTravelTo(Point destination) {
    double[] xyt = odometer.getXyt();
    Point currentLocation = new Point(xyt[0] / TILE_SIZE, xyt[1] / TILE_SIZE);
    double currentTheta = xyt[2];
    double destinationTheta = getDestinationAngle(currentLocation, destination);
    turnBy(minimalAngle(currentTheta, destinationTheta));
    moveStraightForReturn(distanceBetween(currentLocation, destination));
    while (distanceBetween(new Point(odometer.getXyt()[0] / TILE_SIZE, odometer.getXyt()[1] / TILE_SIZE),
        destination) > 0.1) {
      int dist = UltrasonicLocalizer.getDistance();
      if (dist <= 15) { // within 15 cm of something
        stopMotors();
        System.out.println("Obstruction found");
        return false;
      }
      waitUntilNextStep();
    }
    System.out.println("Done");
    return true;
  }

  /**
   * Turns the robot with a minimal angle towards the given input angle in
   * degrees, no matter what its current orientation is. This method is different
   * from {@code turnBy()}.
   */
  public static void turnTo(double angle) {
    turnBy(minimalAngle(odometer.getXyt()[2], angle));
  }

  /**
   * Returns the angle that the robot should point towards to face the destination
   * in degrees.
   */
  public static double getDestinationAngle(Point current, Point destination) {
    return (toDegrees(atan2(destination.x - current.x, destination.y - current.y)) + 360) % 360;
  }

  /**
   * Returns the signed minimal angle from the initial angle to the destination
   * angle.
   */
  public static double minimalAngle(double initialAngle, double destAngle) {
    double dtheta = destAngle - initialAngle;
    if (dtheta < -180) {
      dtheta += 360;
    } else if (dtheta > 180) {
      dtheta -= 360;
    }
    return dtheta;
  }

  /** Returns the distance between the two points in tile lengths. */
  public static double distanceBetween(Point p1, Point p2) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return sqrt(dx * dx + dy * dy);
  }

  /**
   * Moves the robot straight for the given distance.
   * 
   * @param distance in feet (tile sizes), may be negative
   */
  public static void moveStraightFor(double distance) {
    setSpeed(FORWARD_SPEED);
    leftMotor.rotate(convertDistance(distance * TILE_SIZE), true);
    rightMotor.rotate(convertDistance(distance * TILE_SIZE), false);
  }

  /**
   * Moves the robot straight for the given distance.
   * 
   * @param distance in meters, may be negative
   */
  public static void moveStraightForMeters(double distance) {
    setSpeed(FORWARD_SPEED);
    leftMotor.rotate(convertDistance(distance), true);
    rightMotor.rotate(convertDistance(distance), false);
  }

  /**
   * Moves the robot straight for the given distance. Returns immediately so as to
   * not stop the execution of subsequent code.
   * 
   * @param distance in feet (tile sizes), may be negative
   */
  public static void moveStraightForReturn(double distance) {
    setSpeed(FORWARD_SPEED);
    leftMotor.rotate(convertDistance(distance * TILE_SIZE), true);
    rightMotor.rotate(convertDistance(distance * TILE_SIZE), true);
  }

  /**
   * Moves the robot straight for the given distance. Returns immediately so as to
   * not stop the execution of subsequent code.
   * 
   * @param distance in meters, may be negative
   */
  public static void moveStraightForReturnMeters(double distance) {
    setSpeed(FORWARD_SPEED);
    leftMotor.rotate(convertDistance(distance), true);
    rightMotor.rotate(convertDistance(distance), true);
  }

  /** Moves the robot forward for an indeterminate distance. */
  public static void forward() {
    setSpeed(FORWARD_SPEED);
    leftMotor.forward();
    rightMotor.forward();
  }

  /** Moves the robot backward for an indeterminate distance. */
  public static void backward() {
    setSpeed(FORWARD_SPEED);
    leftMotor.backward();
    rightMotor.backward();
  }

  /**
   * Turns the robot by a specified angle. Note that this method is different from
   * {@code turnTo()}. For example, if the robot is facing 90 degrees, calling
   * {@code turnBy(90)} will make the robot turn to 180 degrees, but calling
   * {@code turnTo(90)} should do nothing (since the robot is already at 90
   * degrees).
   * 
   * @param angle the angle by which to turn, in degrees
   */
  public static void turnBy(double angle) {
    setSpeed(ROTATE_SPEED);
    leftMotor.rotate(convertAngle(angle), true);
    rightMotor.rotate(-convertAngle(angle), false);
  }

  /** Rotates motors clockwise. */
  public static void clockwise() {
    setSpeed(ROTATE_SPEED);
    leftMotor.forward();
    rightMotor.backward();
  }

  /** Rotates motors counterclockwise. */
  public static void counterclockwise() {
    setSpeed(ROTATE_SPEED);
    leftMotor.backward();
    rightMotor.forward();
  }

  /** Stops both motors. This also resets the motor speeds to zero. */
  public static void stopMotors() {
    leftMotor.stop();
    rightMotor.stop();
  }

  /**
   * Converts input distance to the total rotation of each wheel needed to cover
   * that distance.
   * 
   * @param distance the input distance in meters
   * @return the wheel rotations necessary to cover the distance in degrees
   */
  public static int convertDistance(double distance) {
    return (int) toDegrees(distance / WHEEL_RAD);
  }

  /**
   * Converts input angle to total rotation of each wheel needed to rotate robot
   * by that angle.
   * 
   * @param angle the input angle in degrees
   * @return the wheel rotations (in degrees) necessary to rotate the robot by the
   *         angle
   */
  public static int convertAngle(double angle) {
    return convertDistance(toRadians((BASE_WIDTH / 2) * angle));
  }

  /**
   * Sets the speed of both motors to the same values.
   * 
   * @param speed the speed in degrees per second
   */
  public static void setSpeed(int speed) {
    setSpeeds(speed, speed);
  }

  /**
   * Sets the speed of both motors to different values.
   * 
   * @param leftSpeed  the speed of the left motor in degrees per second
   * @param rightSpeed the speed of the right motor in degrees per second
   */
  public static void setSpeeds(int leftSpeed, int rightSpeed) {
    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(rightSpeed);
  }

  /**
   * Sets the acceleration of both motors.
   * 
   * @param acceleration the acceleration in degrees per second squared
   */
  public static void setAcceleration(int acceleration) {
    leftMotor.setAcceleration(acceleration);
    rightMotor.setAcceleration(acceleration);
  }

  /**
   * 
   * @param curr
   * @param angle
   * @param szn
   */
  public static void carpetSearch(Point curr, double angle, Region szn) {
    ArrayList<Point> unSortedTiles = szTiles(szn);
    // sort tiles by distance from current position
    ArrayList<Point> tiles = sortDistances(unSortedTiles, curr);
    System.out.println("Sorted");
    // travel to one tile at a time
    for (int i = 0; i < tiles.size(); i++) {
      System.out.println("Checking tile number " + i);
      Point pt = new Point(tiles.get(i).x + 0.5, tiles.get(i).y + 0.5);
      System.out.println(pt.x + " and " + pt.y);
      if (distanceBetween(curr, pt) < 0.3) {
        continue;
      }
      if (safeTravelTo(pt)) {
        travelTo(curr); // return to start, nothing interesting here.
      } else {
        // if obstacle detected, check what it is
        double dist = (UltrasonicLocalizer.getDistance() / 100) / TILE_SIZE;
        double[] xyt = odometer.getXyt();
        Point unknownPt = new Point(xyt[0] + dist * cos(toRadians(xyt[2] - 90)),
            xyt[1] + dist * sin(toRadians(xyt[2] - 90)));
        if (validateBlock(unknownPt, curr, angle)) { // if block, end
          System.out.println("Block found. Ending demo here.");
          return;
        } else { // if not a block, go around
          System.out.println("Oh no, it seems this is an obstacle...");
          return;
          // TODO GO AROUND IT AND CONTINUE FORWARD
        }
      }
    }
  }

  public static ArrayList<Point> sortDistances(ArrayList<Point> list, Point curr) {
    ArrayList<Point> result = new ArrayList<Point>();

    for (int i = 0; i < list.size(); i++) { // for all individual points in list
      int index = 0;
      Point offsetPt = new Point(list.get(i).x + 0.5, list.get(i).y + 0.5);
      while (true) { // loop until point position is found
        if (result.size() <= index) {
          // if there are no more elements to compare to
          result.add(index, list.get(i));
          break;
        } else {
          Point resultOffset = new Point(result.get(index).x + 0.5, result.get(index).y + 0.5);
          if (distanceBetween(curr, offsetPt) > distanceBetween(curr, resultOffset)) {
            // current index object belongs later in the list
            index++;
          } else {
            // current index object belongs before the current list element
            result.add(index, list.get(i));
            break;
          }
        }
      }
    }
    return result;
  }

  /**
   * A method that takes as input a search zone and generates all valid tiles
   * inside the zone (valid meaning that there are no known obstacles, ie the ramp
   * and bin)
   * 
   * @param szn Search zone (region)
   * @return ArrayList of points contianing all lower left corners of valid tiles.
   */
  public static ArrayList<Point> szTiles(Region szn) {
    // Tiles are identified by their lower left corner.
    ArrayList<Point> tiles = new ArrayList<Point>();
    // Determine which tiles the ramp and bin occupy (2 tiles only)
    // Point lre = isRedTeam ? rr.left : gr.left;
    // Point rre = isRedTeam ? rr.right : gr.right;
    Point rt1 = new Point(100, 100);
    Point rt2 = new Point(100, 100);
    // if (lre.x < rre.x) { // upward facing bin
    // rt1 = new Point(lre.x, lre.y);
    // rt2 = new Point(lre.x, lre.y + 1);
    // } else if (lre.x > rre.x) { // downward facing bin
    // rt1 = new Point(rre.x, rre.y - 1);
    // rt2 = new Point(rre.x, rre.y - 2);
    // } else if (lre.y < rre.y) { // left facing bin
    // rt1 = new Point(lre.x - 1, lre.y);
    // rt2 = new Point(lre.x - 2, lre.y);
    // } else { // (lre.y > rre.y) // right facing bin
    // rt1 = new Point(rre.x, rre.y);
    // rt2 = new Point(rre.x + 1, rre.y);
    // }
    // ramp tiles stored in rt1 and rt2
    // iterate over all tiles in the region
    for (int y = (int) szn.ll.y; y < (int) szn.ur.y; y++) {
      for (int x = (int) szn.ll.x; x < (int) szn.ur.x; x++) {
        boolean sameP1 = x == rt1.x && y == rt1.y;
        boolean sameP2 = x == rt2.x && y == rt2.y;
        if (!sameP1 && !sameP2) { // Is neither ramp or bin point
          tiles.add(new Point(x, y));
        }
      }
    }
    return tiles;
  }

}
