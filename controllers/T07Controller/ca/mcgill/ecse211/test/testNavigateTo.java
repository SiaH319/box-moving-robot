package ca.mcgill.ecse211.test;

import static ca.mcgill.ecse211.project.Navigation.*;
import static ca.mcgill.ecse211.project.Resources.*;
import static org.junit.jupiter.api.Assertions.*;

import ca.mcgill.ecse211.playingfield.Point;
import org.junit.jupiter.api.Test;

public static void main(String[]args){Point A=new Point(1,2);Point B=new Point(3,4);

navigateTo(Point A,Point B)}

public class testNavigateTo {

  public static void navigateTo(Point destination, Point blockPos) {
     // go to the closest point on the block
 
     double[] xyt = odometer.getXyt();
     
     Point current = new Point(xyt[0] / TILE_SIZE, xyt[1] / TILE_SIZE);
     Point intermediate = closestEdge(current, blockPos);
     System.out.println("Current is " + current.x + " " + current.y);
     System.out.println("intermediate is " + intermediate.x + " " + intermediate.y);
     System.out.println("Dest is " + destination.x + " " + destination.y);
     travelTo(intermediate);
 
     // face the block
     setSpeed(LOCAL_SPEED);
     System.out.println(getDestinationAngle(intermediate, blockPos));
     turnTo(getDestinationAngle(intermediate, blockPos));
 
     // end if the destination waypoint the same as the farest point
     if (!intermediate.equals(destination)) {
       // navigate around the block
       avoidBlock(intermediate, destination, blockPos);
       // finish facing the block again
       turnTo(getDestinationAngle(destination, blockPos));
     }
   }
