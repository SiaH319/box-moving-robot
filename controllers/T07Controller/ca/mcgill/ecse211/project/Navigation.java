package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static ca.mcgill.ecse211.project.Main.*;
import static ca.mcgill.ecse211.project.LightLocalizer.relocalize;

import java.util.ArrayList;
import static ca.mcgill.ecse211.project.UltrasonicLocalizer.*;
import static java.lang.Math.*;
import ca.mcgill.ecse211.playingfield.*;
import static simlejos.ExecutionController.*;

public class Navigation {
  // Coordinate variables from WiFi class
  public static double Red_LL_x = red.ll.x;
  public static double Red_LL_y = red.ll.y;
  public static double Red_UR_x = red.ur.x;
  public static double Red_UR_y = red.ur.y;
  public static double Green_LL_x = green.ll.x;
  public static double Green_LL_y = green.ll.y;
  public static double Green_UR_x = green.ur.x;
  public static double Green_UR_y = green.ur.y;
  public static double TNR_LL_x = tnr.ll.x;
  public static double TNR_LL_y = tnr.ll.y;
  public static double TNR_UR_x = tnr.ur.x;
  public static double TNR_UR_y = tnr.ur.y;
  public static double TNG_LL_x = tng.ll.x;
  public static double TNG_LL_y = tng.ll.y;
  public static double TNG_UR_x = tng.ur.x;
  public static double TNG_UR_y = tng.ur.y;
  public static double SZR_LL_x = szr.ll.x;
  public static double SZR_LL_y = szr.ll.y;
  public static double SZR_UR_x = szr.ur.x;
  public static double SZR_UR_y = szr.ur.y;
  public static double SZG_LL_x = szg.ll.x;
  public static double SZG_LL_y = szg.ll.y;
  public static double SZG_UR_x = szg.ur.x;
  public static double SZG_UR_y = szg.ur.y;
  public static double Island_LL_x = island.ll.x;
  public static double Island_LL_y = island.ll.y;
  public static double Island_UR_x = island.ur.x;
  public static double Island_UR_y = island.ur.y;
  public static double RRR_x = rr.right.x;
  public static double RRR_y = rr.right.y;
  public static double RRL_x = rr.left.x;
  public static double RRL_y = rr.left.y;
  public static double GRR_x = gr.right.x;
  public static double GRR_y = gr.right.y;
  public static double GRL_x = gr.left.x;
  public static double GRL_y = gr.left.y;

  // Team coordinate variables
  public static double lowerLeftSzgX = 0;
  public static double lowerLeftSzgY = 0;
  public static double upperRightSzgX = 0;
  public static double upperRightSzgY = 0;
  public static double lowerLeftX = 0;
  public static double lowerLeftY = 0;
  public static double upperRightX = 0;
  public static double upperRightY = 0;
  public static double lowerLeftTunnelX = 0;
  public static double lowerLeftTunnelY = 0;
  public static double upperRightTunnelX = 0;
  public static double upperRightTunnelY = 0;
  public static double leftRamp_x = 0;
  public static double leftRamp_y = 0;
  public static double rightRamp_x = 0;
  public static double rightRamp_y = 0;
  public static int startCorner;
  
  // Map orientation booleans
  public static boolean upperonmap = false;
  public static boolean leftonmap = false;
  public static boolean horizontaltunnel = false;

  /** Do not instantiate this class. */
  private Navigation() {
  }
  
  /**
   * Drives the bot to the farest edge of a block then navigates around it to
   * reach a destination point.
   * 
   * @param destination Final waypoint around a block.
   * @param blockPos    Position of the block itself.
   */
  public static void navigateTo(Point destination, Point blockPos) {
    // go to the closest point on the block

    double[] xyt = odometer.getXyt();
    Point current = new Point(xyt[0] / TILE_SIZE, xyt[1] / TILE_SIZE);
    Point intermediate = farestEdge(blockPos);
    System.out.println("Current is " + current.x + " " + current.y);
    System.out.println("intermediate is " + intermediate.x + " " + intermediate.y);
    System.out.println("Dest is " + destination.x + " " + destination.y);
    travelTo(intermediate);

    // face the block
    setSpeed(LOCAL_SPEED);
    turnTo(Math.toDegrees(getDestinationAngle(intermediate, blockPos)));

    // end if the destination waypoint the same as the farest point
    if (!intermediate.equals(destination)) {
      // navigate around the block
      avoidBlock(intermediate, destination, blockPos);
      // finish facing the block again
      turnTo(Math.toDegrees(getDestinationAngle(destination, blockPos)));
    }
  }
  
  /**
   * Drives the bot to the farest edge of a block then navigates around it to
   * reach a destination point depending if the bot is in X or Y.
   * Must occur when the bot touches an object (check the torque)
   * 
   * @param destination Final waypoint around a block.
   * @param blockPos Position of the block itself.
   */
  public static void reposition(Point destination, Point blockPos) {
	  double[] xyt = odometer.getXyt();
	  Point current = new Point(xyt[0] / TILE_SIZE, xyt[1] / TILE_SIZE);
	  boolean orientation = true;
	  
	  //check if the bot is in X or Y position
	  if(Math.round(xyt[2]) <= 100 || Math.round(xyt[2]) >= 80) {
		  orientation = false;
	  }
	  Point intermediate = farestEdgeXY(blockPos, orientation);
	  System.out.println("Current is " + current.x + " " + current.y);
	  System.out.println("intermediate is " + intermediate.x + " " + intermediate.y);
	  System.out.println("Dest is " + destination.x + " " + destination.y);
	  travelTo(intermediate);
	  
	  // face the block
	    setSpeed(LOCAL_SPEED);
	    turnTo(Math.toDegrees(getDestinationAngle(intermediate, blockPos)));

	    // end if the destination waypoint the same as the closest point
	    if (!intermediate.equals(destination)) {
	      // navigate around the block
	      avoidBlock(intermediate, destination, blockPos);
	      // finish facing the block again
	      turnTo(Math.toDegrees(getDestinationAngle(destination, blockPos)));
	    }
  }

  /**
   * Returns the farest point around a block to the ramp position.
   * 
   * @param blockPos Position of the block (point)
   * @return Farest point one bot distance away from the edge of the block to the
   *         ramp position
   */
  public static Point farestEdge(Point blockPos) {
    // calculate the 4 points around the block (up down left right)
    // these points should be offset from the block by PUSH_POSITION_OFFSET in tiles
    // find the farest point of the 4 to the current position
    // return the farest point
	Point middleRamp = new Point((rightRamp_x + leftRamp_x)/2, rightRamp_y);
    Point[] blockPoints;
    blockPoints = new Point[4];
    blockPoints[0] = new Point(blockPos.x, blockPos.y + PUSH_POSITION_OFFSET); // up
    blockPoints[1] = new Point(blockPos.x, blockPos.y - PUSH_POSITION_OFFSET); // down
    blockPoints[2] = new Point(blockPos.x - PUSH_POSITION_OFFSET, blockPos.y); // left
    blockPoints[3] = new Point(blockPos.x + PUSH_POSITION_OFFSET, blockPos.y); // right

    double[] distances = new double[4];
    for (int i = 0; i < 4; i++) {
      distances[i] = distanceBetween(middleRamp, blockPoints[i]);
    }

    int index = 0;
    double max = distances[index];
    for (int i = 1; i < distances.length; i++) {
      if (distances[i] > max) {
        max = distances[i];
        index = i;
      }
    }

    Point farestPoint = blockPoints[index];
    return farestPoint;
  }
  
  /**
   * Returns the farest point in X or Y around a block to the ramp position.
   * 
   * @param blockPos Position of the block (point)
   * @param orientation True if bot is oriented in X
   * @return Farest point one bot distance away from the edge of the block to the
   *         ramp position
   */
  public static Point farestEdgeXY(Point blockPos, boolean orientation) {
    // calculate the 4 points around the block (up down left right)
    // these points should be offset from the block by PUSH_POSITION_OFFSET in tiles
    // find the farest point of the 4 to the current position
    // return the farest point
	Point middleRamp = new Point((rightRamp_x + leftRamp_x)/2, rightRamp_y);
    Point[] blockPoints;
    blockPoints = new Point[2];
    if(orientation) {
    	 blockPoints[0] = new Point(blockPos.x - PUSH_POSITION_OFFSET, blockPos.y); // left
    	 blockPoints[1] = new Point(blockPos.x + PUSH_POSITION_OFFSET, blockPos.y); // right
    }
    else {
    	 	blockPoints[0] = new Point(blockPos.x, blockPos.y + PUSH_POSITION_OFFSET); // up
    	    blockPoints[1] = new Point(blockPos.x, blockPos.y - PUSH_POSITION_OFFSET); // down
    }
   

    double[] distances = new double[2];
    for (int i = 0; i < 2; i++) {
      distances[i] = distanceBetween(middleRamp, blockPoints[i]);
    }

    int index = 0;
    double max = distances[index];
    for (int i = 1; i < distances.length; i++) {
      if (distances[i] > max) {
        max = distances[i];
        index = i;
      }
    }

    Point farestPoint = blockPoints[index];
    return farestPoint;
  }
  
  /**
   * Navigates from a far waypoint to the block to a final waypoint without
   * touching the block. The bot will always start by facing the block at the
   * start position.
   * 
   * @param start       Current position of the bot (point)
   * @param destination Target position of the bot (point)
   * @param blockPos    Position of the block (point)
   */
  public static void avoidBlock(Point start, Point destination, Point blockPos) {
    double blockAngle = Math.toDegrees(getDestinationAngle(start, blockPos));
    double destAngle = Math.toDegrees(getDestinationAngle(start, destination));
    System.out.println("base case angle: " + ((int) (destAngle - blockAngle) + 360) % 360);
    if ((int) (destAngle - blockAngle) == 0) {
      // points are along the same axis, so 4 rotations and 3 travels are required
      setSpeed(LOCAL_SPEED);
      turnBy(-90);
      setSpeed(ROTATE_SPEED);
      moveStraightFor((PUSH_POSITION_OFFSET + NAV_OFFSET) * TILE_SIZE);
      setSpeed(LOCAL_SPEED);
      turnBy(90);
      setSpeed(ROTATE_SPEED);
      moveStraightFor((2 * PUSH_POSITION_OFFSET + NAV_OFFSET) * TILE_SIZE);
      setSpeed(LOCAL_SPEED);
      turnBy(90);
      setSpeed(ROTATE_SPEED);
      moveStraightFor((PUSH_POSITION_OFFSET + NAV_OFFSET) * TILE_SIZE);
      setSpeed(LOCAL_SPEED);
      turnBy(90);
    } else if (((int) (destAngle - blockAngle) + 360) % 360 > 180) {
      // actual angle should be about 315 degrees when mod 360
      // point is on the clockwise adjacent side (left)
      setSpeed(LOCAL_SPEED);
      turnBy(-90);
      setSpeed(ROTATE_SPEED);
      moveStraightFor((PUSH_POSITION_OFFSET + NAV_OFFSET) * TILE_SIZE);
      setSpeed(LOCAL_SPEED);
      turnBy(90);
      setSpeed(ROTATE_SPEED);
      moveStraightFor(PUSH_POSITION_OFFSET * TILE_SIZE);
      setSpeed(LOCAL_SPEED);
      turnBy(90);
      setSpeed(ROTATE_SPEED);
      moveStraightFor(NAV_OFFSET * TILE_SIZE);
    } else {
      // actual angle should be about 45 degrees when mod 360
      // point is on the counter-clockwise adjacent side (right)
      setSpeed(LOCAL_SPEED);
      turnBy(90);
      setSpeed(ROTATE_SPEED);
      moveStraightFor((PUSH_POSITION_OFFSET + NAV_OFFSET) * TILE_SIZE);
      setSpeed(LOCAL_SPEED);
      turnBy(-90);
      setSpeed(ROTATE_SPEED);
      moveStraightFor(PUSH_POSITION_OFFSET * TILE_SIZE);
      setSpeed(LOCAL_SPEED);
      turnBy(-90);
      setSpeed(ROTATE_SPEED);
      moveStraightFor(NAV_OFFSET * TILE_SIZE);
    }
  }
  
  /**
   * Pushes a block forward over a fixed distance and returns the average torque.
   * This methods assumes that we are 1/2 a tile behind the block (in the dir. we
   * want to push).
   * 
   * @param dist Positive distance the block should be pushed over (in m).
   * @return Average torque over the push period.
   */
  public static double pushFor(double dist) {
    setSpeed(FORWARD_SPEED);
    moveStraightFor(Resources.PUSH_TRAVEL_OFFSET); // have the bot touching the box
    System.out.println("pushing for " + dist);
    final int distTacho = convertDistance(dist);
    leftMotor.resetTachoCount();
    rightMotor.resetTachoCount();
    moveStraightForReturn(dist * 3.28084);

    double avg = 0;
    int readings = 0;
    while (Math.abs(leftMotor.getTachoCount()) < Math.abs(distTacho)) {
      // while distance wasn't reached calculate average torque and wait.
      double trk = (leftMotor.getTorque() + rightMotor.getTorque()) / 2;
      avg = (avg * readings + trk) / ++readings;
      waitUntilNextStep();
    }

    leftMotor.stop();
    rightMotor.stop();
    return avg;
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
    // THE FOLLOWING ONLY WORKS WITHIN 7+-2 CM OF THE OBJECT.
    if (blockOrObstacle()) {
      blocks.add(new Block(pt, -1)); // Add as block with placeholder torque
      System.out.println("Is a block");
      return true;
    } else { // Both are seeing roughly the same object (within 7cm) (tall wall)
      obstacles.add(pt); // Add as obstacle
      System.out.println("Not a block");
      return false;
    }
  }

  /**
   * Compares the front and top ultrasonic sensor values to determine if the
   * obstacle in front of the robot is a block or not. Note that this method only
   * works if the robot is within ~7cm (+-2cm) of the obstacle and facing it.
   * 
   * @return True if the obstacle in front of the robot is a block, false if it is
   *         a wall.
   */
  public static boolean blockOrObstacle() {
    int top = topUSDistance();
    int front = frontUSDistance();
    System.out.println("Top : " + top);
    System.out.println("Front x2 : " + front * 2);
    // If top USS not seeing same-ish as front USS (block too short, wall is tall
    // enough to be seen)
    return (top > 2 * front && top > 10);
    // 10 is used as it is bigger than the possible value top could see within the
    // 7m radius of an obstacle.
  }

  /**
   *  Travels to the given destination.
   * @param destination A point represnting the destination.
   */
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
    System.out.println("=> Current location: (" 
          + currentLocation.x + ", " + currentLocation.y + ")");
    double currentTheta = xyt[2];
    double destinationTheta = getDestinationAngle(currentLocation, destination);
    turnBy(minimalAngle(currentTheta, destinationTheta));
    System.out.println("=> Destination: (" + destination.x + ", " + destination.y + ")");
    moveStraightForReturn(distanceBetween(currentLocation, destination));
    while (distanceBetween(new Point(odometer.getXyt()[0] 
        / TILE_SIZE, odometer.getXyt()[1] / TILE_SIZE), destination) > 0.1) {
      int dist = UltrasonicLocalizer.getDistance();
      if (dist < 15) {
        stopMotors();
        System.out.println("=> Obstruction found. Validating...");
        return false;
      }
      waitUntilNextStep();
    }
    System.out.println("=> Arrived at destination.");
    return true;
  }

  /**
   * Turns the robot with a minimal angle towards the given input angle in
   * degrees, no matter what its current orientation is. This method is different
   * from {@code turnBy()}.
   * @param angle Angle the robot should turn to in degrees.
   */
  public static void turnTo(double angle) {
    turnBy(minimalAngle(odometer.getXyt()[2], angle));
  }

  /**
   * Returns the angle that the robot should point towards to face the destination
   * in degrees.
   * @param current Current position of the robot.
   * @param destination Destination of the robot.
   * 
   * @return Destination angle.
   */
  public static double getDestinationAngle(Point current, Point destination) {
    return (toDegrees(atan2(destination.x - current.x, destination.y - current.y)) + 360) % 360;
  }

  /**
   * Returns the signed minimal angle from the initial angle to the destination
   * angle.
   * @param initialAngle Current angle of the robot in degrees.
   * @param destAngle Target heading of the robot in degrees.
   * 
   * @return Miniman angle difference in degrees.
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

  /** Returns the distance between the two points in tile lengths.
   * @param p1 First Point
   * @param p2 Second Point  
   * 
   * @return Distance between the two points in m.
   */
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
   * Finds the point before the tunnel.
   * 
   * @return Returns the point before the tunnel.
   */
  public static Point getPointBeforetunnel() {
    // Get team points first
    setPoints();

    // Calculate point before tunnel
    double angle = 0;
    double x = 0;
    double y = 0;
    Point dest = new Point(0, 0);

    // Determine starting coordinates and tunnel orientation/entrance from
    // startCorner
    if (startCorner == 3) {
      // In the UPPER-LEFT corner
      x = 1;
      y = 8;
      angle = 90;
      upperonmap = true;
      leftonmap = true;

      // Check whether tunnel is horizontal or vertical
      if (Island_LL_x > upperRightX) {
        // Tunnel is horizontal (search zone is to right of starting zone)
        dest.x = lowerLeftTunnelX - 1;
        dest.y = (upperRightTunnelY + lowerLeftTunnelY) / 2;
        horizontaltunnel = true;
      } else if (Island_UR_y < lowerLeftY) {
        // Tunnel is vertical (search zone is below starting zone)
        dest.y = upperRightTunnelY + 1;
        dest.x = (lowerLeftTunnelX + upperRightTunnelX) / 2;
      }

    } else if (startCorner == 2) {
      // In the UPPER-RIGHT corner
      x = 14;
      y = 8;
      angle = -90;
      upperonmap = true;
      leftonmap = false;

      // Check whether tunnel is horizontal or vertical
      if (Island_UR_x < lowerLeftX) {
        // Tunnel is horizontal (search zone is to left of starting zone)
        dest.x = upperRightTunnelX + 1;
        dest.y = (upperRightTunnelY + lowerLeftTunnelY) / 2;
        horizontaltunnel = true;
      } else if (Island_UR_y < lowerLeftY) {
        // Tunnel is vertical (search zone is below starting zone)
        dest.y = upperRightTunnelY + 1;
        dest.x = (lowerLeftTunnelX + upperRightTunnelX) / 2;
      }

    } else if (startCorner == 0) {
      // In the LOWER-LEFT corner
      x = 1;
      y = 1;
      angle = 90;
      upperonmap = false;
      leftonmap = true;

      // Check whether tunnel is horizontal or vertical
      if (Island_LL_x > upperRightX) {
        // Tunnel is horizontal (search zone is to right of starting zone)
        dest.x = lowerLeftTunnelX - 1;
        dest.y = (upperRightTunnelY + lowerLeftTunnelY) / 2;
        horizontaltunnel = true;
      } else if (Island_LL_y > upperRightY) {
        // Tunnel is vertical (search zone is above starting zone)
        dest.y = lowerLeftTunnelY - 1;
        dest.x = (lowerLeftTunnelX + upperRightTunnelX) / 2;
      }

    } else if (startCorner == 1) {
      // In the LOWER-RIGHT corner
      x = 14;
      y = 1;
      angle = -90;
      upperonmap = false;
      leftonmap = false;

      // System.out.println("=> Starting from Corner " + startCorner + " (" + x + ", "
      // + y + ")...");

      // Check whether tunnel is horizontal or vertical
      if (Island_UR_x < lowerLeftX) {
        // Tunnel is horizontal (search zone is to the left of starting zone)
        // System.out.println("=> Tunnel runs horizontally.");
        dest.x = upperRightTunnelX + 1;
        dest.y = (upperRightTunnelY + lowerLeftTunnelY) / 2;
        horizontaltunnel = true;
      } else if (Island_LL_y > upperRightY) {
        // Tunnel is vertical (search zone is above starting zone)
        // System.out.println("=> Tunnel runs vertically.");
        dest.y = lowerLeftTunnelY - 1;
        dest.x = (lowerLeftTunnelX + upperRightTunnelX) / 2;
      }

    }

    // Set odometer to determined destination
    odometer.setX(x * TILE_SIZE);
    odometer.setY(y * TILE_SIZE);
    odometer.setTheta(angle);
    return dest;

  }

  /**
   * Sets up the points.
   */
  public static void setPoints() {
    if (isRedTeam) {
      // Set RED TEAM coordinates
      lowerLeftSzgX = SZR_LL_x;
      lowerLeftSzgY = SZR_LL_y;
      upperRightSzgX = SZR_UR_x;
      upperRightSzgY = SZR_UR_y;
      lowerLeftX = Red_LL_x;
      lowerLeftY = Red_LL_y;
      upperRightX = Red_UR_x;
      upperRightY = Red_UR_y;
      lowerLeftTunnelX = TNR_LL_x;
      lowerLeftTunnelY = TNR_LL_y;
      upperRightTunnelX = TNR_UR_x;
      upperRightTunnelY = TNR_UR_y;
      leftRamp_x =  RRL_x;
      leftRamp_y =  RRL_y;
      rightRamp_x = RRR_x;
      rightRamp_y = RRR_y;
      startCorner = Resources.redCorner;
    } else {
      // Set GREEN TEAM coordinates
      lowerLeftSzgX = SZG_LL_x;
      lowerLeftSzgY = SZG_LL_y;
      upperRightSzgX = SZG_UR_x;
      upperRightSzgY = SZG_UR_y;
      lowerLeftX = Green_LL_x;
      lowerLeftY = Green_LL_y;
      upperRightX = Green_UR_x;
      upperRightY = Green_UR_y;
      lowerLeftTunnelX = TNG_LL_x;
      lowerLeftTunnelY = TNG_LL_y;
      upperRightTunnelX = TNG_UR_x;
      upperRightTunnelY = TNG_UR_y;
      leftRamp_x =  GRL_x;
      leftRamp_y =  GRL_y;
      rightRamp_x = GRR_x;
      rightRamp_y = GRR_y;
      startCorner = Resources.greenCorner;
    }
  }

  /**
   * Goes through the tunnel plus 0.4 tile lengths ahead.
   */
  public static void goThroughTunnel() {
    // Calculate point before tunnel, then travel to it
    Point destination = getPointBeforetunnel();
    System.out.println("[STATUS] Travelling to tunnel...");
    travelTo(destination);
    System.out.println("[STATUS] Arrived at tunnel. Passing through to island...");

    // Determine orientation of tunnel based on position of starting zone
    if (upperonmap == true) {
      // Check UPPER-X cases

      if (leftonmap) {
        // UPPER-LEFT
        if (horizontaltunnel) {
          turnTo(90);
          moveStraightFor(upperRightTunnelX - lowerLeftTunnelX + 1.4);

          odometer.setX(destination.x + (upperRightTunnelX - lowerLeftTunnelX + 1.4));
          odometer.setY(destination.y);
        } else {
          turnTo(180);
          moveStraightFor(upperRightTunnelY - lowerLeftTunnelY + 1.4);

          odometer.setX(destination.x);
          odometer.setY(destination.y - (upperRightTunnelY - lowerLeftTunnelY + 1.4));
        }
      } else {
        // UPPER-RIGHT
        if (horizontaltunnel) {
          turnTo(-90);
          moveStraightFor(upperRightTunnelX - lowerLeftTunnelX + 1.4);

          odometer.setX(destination.x - (upperRightTunnelX - lowerLeftTunnelX + 1.4));
          odometer.setY(destination.y);
        } else {
          turnTo(-180);
          moveStraightFor(upperRightTunnelY - lowerLeftTunnelY + 1.4);

          odometer.setX(destination.x);
          odometer.setY(destination.y - (upperRightTunnelY - lowerLeftTunnelY + 1.4));
        }
      }
    } else {
      // Check LOWER-X cases

      if (leftonmap) {
        // LOWER-LEFT
        if (horizontaltunnel) {
          turnTo(90);
          moveStraightFor(upperRightTunnelX - lowerLeftTunnelX + 1.4);

          odometer.setX(destination.x + upperRightTunnelX - lowerLeftTunnelX + 1.4);
          odometer.setY(destination.y);
        } else {
          turnTo(0);
          moveStraightFor(upperRightTunnelY - lowerLeftTunnelY + 1.4);

          odometer.setX(destination.x);
          odometer.setY(destination.y + (upperRightTunnelY - lowerLeftTunnelY + 1.4));
        }
      } else {
        // LOWER-RIGHT
        if (horizontaltunnel) {
          turnTo(-90);
          moveStraightFor(upperRightTunnelX - lowerLeftTunnelX + 1.4);

          odometer.setX(destination.x - (upperRightTunnelX - lowerLeftTunnelX + 1.4));
          odometer.setY(destination.y);
        } else {
          turnTo(0);
          moveStraightFor(upperRightTunnelY - lowerLeftTunnelY + 1.4);

          odometer.setX(destination.x);
          odometer.setY(destination.y + (upperRightTunnelY - lowerLeftTunnelY + 1.4));
        }
      }
    }
  }

  /**
   * Checks weather the robot is in the search zone.
   * 
   * @return true if in search zone.
   */
  public static boolean inSearchZone() {
    if (isRedTeam) {
      // If RED TEAM, use RED SEARCH ZONE coordinates
      if (odometer.getXyt()[0] > SZR_LL_x && odometer.getXyt()[0] 
          < SZR_UR_x && odometer.getXyt()[1] > SZR_LL_y
          && odometer.getXyt()[1] < SZR_UR_y) {
        return true;
      } else {
        return false;
      }
    } else {
      // If GREEN TEAM, use GREEN SEARCH ZONE coordinates
      if (odometer.getXyt()[0] > SZG_LL_x && odometer.getXyt()[0] 
          < SZG_UR_x && odometer.getXyt()[1] > SZG_LL_y
          && odometer.getXyt()[1] < SZG_UR_y) {
        return true;
      } else {
        return false;
      }
    }
  }

  /**
   * Moves robot to searchZone after having moved through tunnel. Assumes that
   * robot will be in center of tile just after tunnel
   */
  public static void goToSearchZone() {
    System.out.println("[STATUS] Navigating to search zone...");

    // Get current location from odometer; set as Point
    double[] xyt = odometer.getXyt();
    Point currentLocation = new Point(xyt[0], xyt[1]);
    double currentTheta = xyt[2];
    // System.out.println(currentLocation.x + ", " + currentLocation.y);

    // Get search zone corner coordinates
    Point LL_SZ = new Point(lowerLeftSzgX, lowerLeftSzgY);
    Point LR_SZ = new Point(upperRightSzgX, lowerLeftSzgY);
    Point UL_SZ = new Point(lowerLeftSzgX, upperRightSzgY);
    Point UR_SZ = new Point(upperRightSzgX, upperRightSzgY);
    Point[] szCorners = new Point[] { LL_SZ, LR_SZ, UL_SZ, UR_SZ };

    // Find closest corner of search zone and set as destination "dest"
    Point SZ_dest = LL_SZ;
    for (int i = 0; i < szCorners.length; i++) {
      Point currCorner = szCorners[i];
      if (distanceBetween(currentLocation, currCorner) 
          <= distanceBetween(currentLocation, SZ_dest)) {
        // Closest point found; set appropriate offset
        if (i == 0) {
          // Lower-left corner
          System.out.println("=> Lower-left corner of search zone is closest.");
          SZ_dest = new Point(currCorner.x + 1, currCorner.y + 1);
        } else if (i == 1) {
          // Lower-right corner
          System.out.println("=> Lower-right corner of search zone is closest.");
          SZ_dest = new Point(currCorner.x - 1, currCorner.y + 1);
        } else if (i == 2) {
          // Upper-left corner
          System.out.println("=> Upper-left corner of search zone is closest.");
          SZ_dest = new Point(currCorner.x + 1, currCorner.y - 1);
        } else {
          // Upper-right corner
          System.out.println("=> Upper-right corner of search zone is closest.");
          SZ_dest = new Point(currCorner.x - 1, currCorner.y - 1);
        }
      }
    }

    // Turn towards destination point
    System.out.println("=> Proceeding to (" + SZ_dest.x + ", " + SZ_dest.y + ")...");
    double destinationTheta = getDestinationAngle(currentLocation, SZ_dest);
    turnBy(minimalAngle(currentTheta, destinationTheta));
    // System.out.println("Turning by: " + minimalAngle(currentTheta,
    // destinationTheta));

    // Check for obstacles in path; adjust path as necessary
    // double usDistance = (getDistanceTop() / 100.0) / TILE_SIZE;
    double usDistance = (getDistance() / 100.0) / TILE_SIZE;
    double distanceToTravel = distanceBetween(currentLocation, SZ_dest);
    // System.out.println("=> Tiles to next obstacle: " + usDistance);
    // System.out.println("=> Tiles to travel: " + distanceToTravel);
    if (usDistance <= distanceToTravel) {
      // Path is NOT clear; adjust heading until clear and try again
      System.out.println("=> Obstacle detected. Attempting to re-route...");
      // TODO: MAKE GENERALIZED
      // TODO: Maybe add block validation to check obstacle?

      // Do 90 degree turns around object
      turnTo(odometer.getXyt()[2] - 90);
      moveStraightFor(0.66);
      // UPDATE ODO
      turnTo(odometer.getXyt()[2] + 90);
      moveStraightFor(distanceToTravel - usDistance);
      // UPDATE ODO

      // Continue...
      goToSearchZone();
    } else {
      // Path is clear; proceed to destination
      System.out.println("=> Path clear. Proceeding...");
      moveStraightFor(distanceToTravel);
      turnTo(0);
      relocalize();

      // Update odometer
      odometer.setX(SZ_dest.x);
      odometer.setY(SZ_dest.y);
      System.out.println("=> Arrived safely at destination.");
      odometer.printPosition();
    }

  }

  /**
   * Sorts an ArrayList of Points based on their distance to a given point.
   * 
   * @param list ArrayList of points
   * @param curr Point around which to sort.
   * @return Returns a new arraylist containing the same points but sorted by
   *         distance from the curr point.
   */
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
   * and bin).
   * 
   * @param szn Search zone (region)
   * @return ArrayList of points contianing all lower left corners of valid tiles.
   */
  public static ArrayList<Point> szTiles(Region szn) {
    // Tiles are identified by their lower left corner.
    ArrayList<Point> tiles = new ArrayList<Point>();
    // Determine which tiles the ramp and bin occupy (2 tiles only)
    Point lre = isRedTeam ? rr.left : gr.left;
    Point rre = isRedTeam ? rr.right : gr.right;
    Point rt1 = new Point(100, 100);
    Point rt2 = new Point(100, 100);
    if (lre.x < rre.x) { // upward facing bin
      rt1 = new Point(lre.x, lre.y);
      rt2 = new Point(lre.x, lre.y + 1);
    } else if (lre.x > rre.x) { // downward facing bin
      rt1 = new Point(rre.x, rre.y - 1);
      rt2 = new Point(rre.x, rre.y - 2);
    } else if (lre.y < rre.y) { // left facing bin
      rt1 = new Point(lre.x - 1, lre.y);
      rt2 = new Point(lre.x - 2, lre.y);
    } else { // (lre.y > rre.y) // right facing bin
      rt1 = new Point(rre.x, rre.y);
      rt2 = new Point(rre.x + 1, rre.y);
    }
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
