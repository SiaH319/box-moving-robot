package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static ca.mcgill.ecse211.project.Main.*;
import static ca.mcgill.ecse211.project.UltrasonicLocalizer.getDistance;
import static ca.mcgill.ecse211.project.UltrasonicLocalizer.getDistanceTop;
import static ca.mcgill.ecse211.project.LightLocalizer.relocalize;
import static java.lang.Math.*;

import ca.mcgill.ecse211.playingfield.*;

public class Navigation {
  // TODO: Get following variables from WIFI class
  // WIFI CLASS VARIABLES (FOR TESTING)
  public static int Red_LL_x = 0;
  public static int Red_LL_y = 5;
  public static int Red_UR_x = 4;
  public static int Red_UR_y = 9;
  public static int Green_LL_x = 10;
  public static int Green_LL_y = 0;
  public static int Green_UR_x = 15;
  public static int Green_UR_y = 4;
  public static double TNR_LL_x = 4;
  public static double TNR_LL_y = 7;
  public static double TNR_UR_x = 6;
  public static double TNR_UR_y = 8;
  public static double TNG_LL_x = 10;
  public static double TNG_LL_y = 3;
  public static double TNG_UR_x = 11;
  public static double TNG_UR_y = 5;
  public static int SZR_LL_x = 6;
  public static int SZR_LL_y = 5;
  public static int SZR_UR_x = 10;
  public static int SZR_UR_y = 9;
  public static int SZG_LL_x =11;
  public static int SZG_LL_y = 5;
  public static int SZG_UR_x = 15;
  public static int SZG_UR_y = 9;
    
  // Replace hard-coded values with:
  /*
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
  public static double SZG_LL_x =szg.ll.x;
  public static double SZG_LL_y = szg.ll.y;
  public static double SZG_UR_x = szg.ur.x;
  public static double SZG_UR_y = szg.ur.y;
*/ 
  
  // TEAM-SPECIFIC VARIABLES
  public static int lowerLeftSzgX = 0;
  public static int lowerLeftSzgY = 0;
  public static int upperRightSzgX = 0;
  public static int upperRightSzgY = 0;
  public static int lowerLeftX = 0;
  public static int lowerLeftY = 0;
  public static int upperRightX = 0;
  public static int upperRightY = 0;
  public static double lowerLeftTunnelX = 0;
  public static double lowerLeftTunnelY = 0;
  public static double upperRightTunnelX = 0;
  public static double upperRightTunnelY = 0;
  public static boolean upperonmap = false;
  public static boolean leftonmap = false;
  public static boolean horizontaltunnel = false;
	
  /** Do not instantiate this class. */
  private Navigation() {
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
   * Finds the point before the tunnel.
   * 
   * Returns the point.
   */
  public static Point getPointBeforetunnel() {
    setPoints();
    double angle=0;
    double x = 0;
    double y = 0;
    Point dest = new Point(0, 0);
    Point check = new Point(lowerLeftTunnelX, lowerLeftTunnelY);
    Point lowerRegion = new Point (lowerLeftX, lowerLeftY);
    Point upperRegion = new Point (upperRightX, upperRightY);

    if(lowerLeftX == 0 && upperRightY == 9) {
      // In the UPPER-LEFT corner
      x = 1;
      y = 8;
      angle=90;
      upperonmap = true;
      leftonmap = true;
      
      // Check whether tunnel is horizontal or vertical	
      if(isWithin(check, lowerRegion, upperRegion)) {
        // Is horizontal
        dest.x = lowerLeftTunnelX -1;
        dest.y = (upperRightTunnelY + lowerLeftTunnelY)/2;
        horizontaltunnel = true;
      } else {
        // Is vertical
        dest.y = upperRightTunnelY + 1;
        dest.x = (lowerLeftTunnelX + upperRightTunnelX)/2;
      }
      
    } else if (upperRightX == 15 && upperRightY == 9) {
      // In the UPPER-RIGHT corner
      x = 14;
      y = 8;
      angle=-90;
      upperonmap=true;
      double diffy = upperRightTunnelY - lowerLeftTunnelY;
      double diffx = upperRightTunnelX - lowerLeftTunnelX;
    	
      if(diffx > 1) {
        dest.x = upperRightTunnelX + 1;
        dest.y = (upperRightTunnelY + lowerLeftTunnelY)/2;
        horizontaltunnel = true;
      } else if(diffy > 1) {
        dest.y = upperRightTunnelY + 1;
        dest.x = (lowerLeftTunnelX + upperRightTunnelX)/2;
      } else {
        // 1 x 1 tunnel
        if (upperRightTunnelX == lowerRegion.x) {
    	dest.x = upperRightTunnelX + 1;
        	dest.y = (upperRightTunnelY + lowerLeftTunnelY)/2;
        	horizontaltunnel = true;
        } else {
    	dest.y = upperRightTunnelY + 1;
    	dest.x = (lowerLeftTunnelX + upperRightTunnelX)/2;
        }
      }
    } else if(lowerLeftX == 0 && lowerLeftY == 0) {
      // In the LOWER-LEFT corner
      x = 1;
      y = 1;
      angle=90;
      leftonmap = true;
      double diffy = upperRightTunnelY - lowerLeftTunnelY;
      double diffx = upperRightTunnelX - lowerLeftTunnelX;
      
      if (diffx >1) {
        dest.x = lowerLeftTunnelX -1;
        dest.y = (upperRightTunnelY + lowerLeftTunnelY)/2;
        horizontaltunnel = true;
      } else if (diffy >1) {
        dest.y = lowerLeftTunnelY -1;
        dest.x = (lowerLeftTunnelX + upperRightTunnelX)/2;
      } else {
        // 1 x 1 tunnel
        if(lowerLeftTunnelX == upperRegion.x) {
          dest.x = lowerLeftTunnelX -1;
          dest.y = (upperRightTunnelY + lowerLeftTunnelY)/2;
          horizontaltunnel = true;
        } else {
          dest.y = lowerLeftTunnelY -1;
          dest.x = (lowerLeftTunnelX + upperRightTunnelX)/2;
        }
      }
    } else {
      // In the LOWER-RIGHT corner
      x = 14;
      y = 1;
      angle= -90;
      
      // Check whether tunnel is horizontal or vertical	
      if(!isWithin(check, lowerRegion, upperRegion)) {
        dest.x = upperRightTunnelX + 1;
        dest.y = (upperRightTunnelY + lowerLeftTunnelY)/2;
        horizontaltunnel = true;
      } else {
        dest.y = lowerLeftTunnelY -1;
        dest.x = (lowerLeftTunnelX + upperRightTunnelX)/2;
      }
    }
    	
    // Set odometer to determined destination
    odometer.setX(x*TILE_SIZE);
    odometer.setY(y*TILE_SIZE);
    odometer.setTheta(angle);
    return dest;
    	
    }
    
  /**
   * Finds the point before the tunnel.
   * 
   * Returns the point.
   */
  public static void setPoints() {
    // TODO: Change conditional statement for isRedTeam
    if (isRedTeam != null) {
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
    }
  }
    
  /**
   * Goes through the tunnel plus 0.4 tile lengths ahead.
   * 
   * Returns the point.
   */
  public static void goThroughTunnel() {
    Point destination = getPointBeforetunnel();
    travelTo(destination);
    
    if(upperonmap == true) {
      // Check UPPER-X cases
      
      if(leftonmap) {
        // UPPER-LEFT
        if(horizontaltunnel) {
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
        if(horizontaltunnel) {
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
      
      if(leftonmap) {
        // LOWER-LEFT
        if(horizontaltunnel) {
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
        if(horizontaltunnel) {
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
   * Checks weather the robot is in the search zone
   * 
   * Returns true if in search zone.
   */
  public static boolean inSearchZone() {
    // TODO: Change isRedTeam conditional
    if(isRedTeam == null) {
      // If RED TEAM, use RED SEARCH ZONE coordinates
      if(odometer.getXyt()[0] > SZR_LL_x && odometer.getXyt()[0] < SZR_UR_x  && odometer.getXyt()[1] > SZR_LL_y && odometer.getXyt()[1] < SZR_UR_y) {
        return true;
      } else {
        return false;
      }
    } else {
      // If GREEN TEAM, use GREEN SEARCH ZONE coordinates
      if(odometer.getXyt()[0] > SZG_LL_x && odometer.getXyt()[0] < SZG_UR_x  && odometer.getXyt()[1] > SZG_LL_y && odometer.getXyt()[1] < SZG_UR_y) {
        return true;
      } else {
        return false;
      }
    }
  }
   
  /**
   * Moves robot to searchZone after having moved through tunnel.
   * Assumes that robot will be in center of tile just after tunnel
   */
  public static void goToSearchZone() {
    System.out.println("[STATUS] Navigating to search zone...");
    
    // Get current location from odometer; set as Point
    double[] xyt = odometer.getXyt();
    Point currentLocation = new Point(xyt[0], xyt[1]);
    double currentTheta = xyt[2];
    //System.out.println(currentLocation.x + ", " + currentLocation.y);
    
    // Get search zone corner coordinates
    Point LL_SZ = new Point(lowerLeftSzgX, lowerLeftSzgY);
    Point LR_SZ = new Point(upperRightSzgX, lowerLeftSzgY);
    Point UL_SZ = new Point(lowerLeftSzgX, upperRightSzgY);
    Point UR_SZ = new Point(upperRightSzgX, upperRightSzgY); 
    Point[] szCorners = new Point[]{LL_SZ, LR_SZ, UL_SZ, UR_SZ};

    // Find closest corner of search zone and set as destination "dest"
    Point SZ_dest = LL_SZ;
    for(int i = 0; i < szCorners.length; i++){
      Point currCorner = szCorners[i];
      if (distanceBetween(currentLocation, currCorner) <= distanceBetween(currentLocation, SZ_dest)) {
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
    double destinationTheta = getDestinationAngle(currentLocation, SZ_dest);
    turnBy(minimalAngle(currentTheta, destinationTheta));
    //System.out.println("Turning by: " + minimalAngle(currentTheta, destinationTheta));
    
    // Check for obstacles in path; adjust path as necessary
    //double usDistance = (getDistanceTop() / 100.0) / TILE_SIZE;
    double usDistance = (getDistance() / 100.0) / TILE_SIZE;
    double distanceToTravel = distanceBetween(currentLocation, SZ_dest);
    //System.out.println("=> Tiles to next obstacle: " + usDistance);
    //System.out.println("=> Tiles to travel: " + distanceToTravel);
    if (usDistance <= distanceToTravel) {
      // Path is NOT clear; adjust heading until clear and try again
      System.out.println("=> Obstacle detected. Readjusting...");
      // TODO: MAKE GENERALIZED
      // TODO: Maybe add block validation to check obstacle?
      
      // EXAMPLE OBSTACLE AVOIDANCE
      System.out.println("[!] EXAMPLE OBSTACLE AVOIDANCE ROUTINE");
      turnTo(0);
      usDistance = (getDistance() / 100.0) / TILE_SIZE;
      System.out.println(usDistance);
      if (usDistance >= distanceToTravel * 1.5) {
        System.out.println("Travelling...");
        moveStraightFor(1.0);
        
        // UPDATE ODOMETER
        odometer.setY(odometer.getXyt()[1] + 1);
        odometer.setTheta(0.0);
        goToSearchZone();
      }

    } else {
      // Path is clear; proceed to destination
      System.out.println("=> Path clear. Proceeding...");
      moveStraightFor(distanceToTravel);
      turnTo(0);
      relocalize();
      
      // Update odometer
      odometer.setX(SZ_dest.x);
      odometer.setY(SZ_dest.y);
      odometer.printPosition();
    }
	   
  }
  


}
