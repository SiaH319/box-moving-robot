package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static java.lang.Math.*;

import ca.mcgill.ecse211.playingfield.*;



public class Navigation {
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
	
	//red tunnel LL point
	//red tunnel UR point
	//green tunnel LL point
	//green tunnel UR point
	
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
   * Moves the robot straight for the given distance.
   * Returns immediately so as to not stop the execution of subsequent code.
   * @param distance in feet (tile sizes), may be negative
   */
  public static void moveStraightForReturn(double distance) {
    setSpeed(FORWARD_SPEED);
    leftMotor.rotate(convertDistance(distance * TILE_SIZE), true);
    rightMotor.rotate(convertDistance(distance * TILE_SIZE), true);
  }
  
  /**
   * Moves the robot straight for the given distance.
   * Returns immediately so as to not stop the execution of subsequent code.
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
  
   // public static Point getPointBeforetunnel() {
    	
    //setPoints();
    //setOdometer();
    
    
	/*
	
    if(isRedTeam != null){
  
	 x1 = 4;
	 x2 = 6;
	 y1 = 7;
	 y2 = 8; 
	 Point dest= new Point(x1 - 1, (y1 + y2)/2);
	 return dest;
    }  
	else {
	
		x1 = 10;
		x2 = 11;
		y1 = 3;
		y2 = 5;
		Point dest= new Point((x2 + x1)/2, y1 -1);
		return dest;
	  }
   */
	  
 // }
    
    public static Point getPointBeforetunnel() {
    	setPoints();
    	double angle=0;
    	double x = 0;
    	double y = 0;
    	Point dest = new Point(0, 0);
    	Point check = new Point(lowerLeftTunnelX, lowerLeftTunnelY);
    	Point lowerRegion = new Point (lowerLeftX, lowerLeftY);
    	Point upperRegion = new Point (upperRightX, upperRightY);
    	
    	
    	//upper left
    	if(lowerLeftX == 0 && upperRightY == 9) {
    		x = 1;
    		y = 8;
    		angle=90;
    		upperonmap = true;
    		leftonmap = true;
    		
    		//going horizontally
    		if(upperRightY == upperRightSzgY) {
    			dest.x = lowerLeftTunnelX -1;
    			dest.y = (upperRightTunnelY + lowerLeftTunnelY)/2;
    			horizontaltunnel = true;
    		}
    		//going vertically
    		else {
    			dest.y = upperRightTunnelY + 1;
    			dest.x = (lowerLeftTunnelX + upperRightTunnelX)/2;
    		}
    	
    		
    	}
    	//upperRight
    	else if(upperRightX == 15 && upperRightY == 9) {
    		x = 14;
    		y = 8;
    		angle=-90;
    		upperonmap=true;
    	
    		if(upperRightY == upperRightSzgY) {
    			dest.x = upperRightTunnelX + 1;
    			dest.y = (upperRightTunnelY + lowerLeftTunnelY)/2;
    			horizontaltunnel = true;
    		}
    		//going vertically
    		else {
    			dest.y = upperRightTunnelY + 1;
    			dest.x = (lowerLeftTunnelX + upperRightTunnelX)/2;
    		}
    		
    		
    	}
    	//LowerLeft
    	else if(lowerLeftX == 0 && lowerLeftY == 0) {
    		x = 1;
    		y = 1;
    		angle=90;
    		leftonmap = true;
    		
    		if(lowerLeftY == lowerLeftSzgY) {
    			dest.x = lowerLeftTunnelX -1;
    			dest.y = (upperRightTunnelY + lowerLeftTunnelY)/2;
    			horizontaltunnel = true;
    		}
    		//going vertically
    		else {
    			dest.y = lowerLeftTunnelY -1;
    			dest.x = (lowerLeftTunnelX + upperRightTunnelX)/2;
    			
    		}
    	}
    	//LowerRight
    	else {
    		x = 14;
    		y = 1;
    		angle= -90;
    		
    		if(lowerLeftY == lowerLeftSzgY) {
    			dest.x = upperRightTunnelX + 1;
    			dest.y = (upperRightTunnelY + lowerLeftTunnelY)/2;
    			horizontaltunnel = true;
    		}
    		//going vertically
    		else {
    			dest.y = lowerLeftTunnelY -1;
    			dest.x = (lowerLeftTunnelX + upperRightTunnelX)/2;
    			
    		}
    	}
    	
    	odometer.setX(x*TILE_SIZE);
	    odometer.setY(y*TILE_SIZE);
	    odometer.setTheta(angle);
	    return dest;
    	
    }
    
    public static void setPoints() {
    	if (isRedTeam == null) {
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
    	}
    	else {
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
     * Goes through the tunnel and one tile ahead.
     * 
     * Returns the point.
     */
    
   public static void goThroughTunnel() {
	   Point destination = getPointBeforetunnel();
	   travelTo(destination);
	   
	
	   if(upperonmap == true) {
		   //upperLeft
		   if(leftonmap) {
			   if(horizontaltunnel) {
				   turnTo(90);
				   moveStraightFor(upperRightTunnelX - lowerLeftTunnelX +1.4);
				   
				   odometer.setX(destination.x + (upperRightTunnelX - lowerLeftTunnelX +1.4));
				   odometer.setY(destination.y);
			   }
			   else {
				   turnTo(180);
				   moveStraightFor(upperRightTunnelY - lowerLeftTunnelY +1.4); 
				   odometer.setY(destination.y - (upperRightTunnelY - lowerLeftTunnelY +1.4));
				   odometer.setX(destination.x);
			   }
			   
		   }
		   //upperRight
		   else {
			   if(horizontaltunnel) {
				   turnTo(-90);
				   moveStraightFor(upperRightTunnelX - lowerLeftTunnelX +1.4);
				   odometer.setY(destination.y);
				   odometer.setX(destination.x - (upperRightTunnelX - lowerLeftTunnelX +1.4));
			   }
			   else {
				   turnTo(-180);
				   moveStraightFor(upperRightTunnelY - lowerLeftTunnelY +1.4); 
				   odometer.setY(destination.y - (upperRightTunnelY - lowerLeftTunnelY +1.4));
				   odometer.setX(destination.x);
			   }
		   }
	   }
	   
	   
		   
		   else{
			   
			   if(leftonmap) {
				   if(horizontaltunnel) {
					   turnTo(90);
					   moveStraightFor(upperRightTunnelX - lowerLeftTunnelX + 1.4);
					   odometer.setY(destination.y);
					   odometer.setX(destination.x + upperRightTunnelX - lowerLeftTunnelX + 1.4);
				   }
				   else {
					   turnTo(0);
					   moveStraightFor(upperRightTunnelY - lowerLeftTunnelY + 1.4); 
					   odometer.setY(destination.y + (upperRightTunnelY - lowerLeftTunnelY + 1.4));
					   odometer.setX(destination.x);
				   }
				   
			   }
			   //lowerRight
			   else {
				   if(horizontaltunnel) {
					   turnTo(-90);
					   moveStraightFor(upperRightTunnelX - lowerLeftTunnelX + 1.4);
					   odometer.setY(destination.y);
					   odometer.setX(destination.x - (upperRightTunnelX - lowerLeftTunnelX + 1.4));
				   }
				   else {
					   turnTo(0);
					   moveStraightFor(upperRightTunnelY - lowerLeftTunnelY + 1.5); 
					   odometer.setY(destination.y + (upperRightTunnelY - lowerLeftTunnelY + 1.4));
					   odometer.setX(destination.x);
				   }
		   }
		   
		   
		
	   
   }}
   
   /**
    * Checks weather the robot is in the search zone
    * 
    * Returns true if in search zone.
    */
   public static boolean inSearchZone() {
	   if(isRedTeam == null) {
		   if(odometer.getXyt()[0] > SZR_LL_x && odometer.getXyt()[0] < SZR_UR_x  && odometer.getXyt()[1] > SZR_LL_y && odometer.getXyt()[1] < SZR_UR_y) {
			   return true;
		   }
			   else {
				   return false;
			   }
	   }
		   
		else {
			
			 if(odometer.getXyt()[0] > SZG_LL_x && odometer.getXyt()[0] < SZG_UR_x  && odometer.getXyt()[1] > SZG_LL_y && odometer.getXyt()[1] < SZG_UR_y) {
				   return true;
			   }
				   else {
					   return false;
				   }
		   }
		}
   
   /**
    * Moves robot to searchZone.
    * 
    */
   public static void goToSearchZone() {
	   turnBy(90);
	   moveStraightFor(1.5);
	   LightLocalizer.relocalize();
	   
   }
  


}
