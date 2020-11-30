package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Navigation.findPath;
import static ca.mcgill.ecse211.project.Navigation.lowerLeftRampX;
import static ca.mcgill.ecse211.project.Navigation.lowerLeftRampY;
import static ca.mcgill.ecse211.project.Navigation.moveStraightFor;
import static ca.mcgill.ecse211.project.Navigation.paths;
import static ca.mcgill.ecse211.project.Navigation.pushFor;
import static ca.mcgill.ecse211.project.Navigation.returnToStart;
import static ca.mcgill.ecse211.project.Navigation.travelTo;
import static ca.mcgill.ecse211.project.Navigation.travelToSafely;
import static ca.mcgill.ecse211.project.Navigation.turnBy;
import static ca.mcgill.ecse211.project.Navigation.turnTo;
import static ca.mcgill.ecse211.project.Resources.PHYSICS_STEP_PERIOD;
import static ca.mcgill.ecse211.project.Resources.TEAM_NUMBER;
import static ca.mcgill.ecse211.project.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.project.Resources.green;
import static ca.mcgill.ecse211.project.Resources.greenTeam;
import static ca.mcgill.ecse211.project.Resources.isRedTeam;
import static ca.mcgill.ecse211.project.Resources.island;
import static ca.mcgill.ecse211.project.Resources.odometer;
import static ca.mcgill.ecse211.project.Resources.redTeam;
import static ca.mcgill.ecse211.project.Resources.rr;
import static ca.mcgill.ecse211.project.Resources.szg;
import static ca.mcgill.ecse211.project.Resources.tnr;
import static ca.mcgill.ecse211.project.Resources.wifiParameters;
import static ca.mcgill.ecse211.project.UltrasonicLocalizer.backWardAdjust;
import static ca.mcgill.ecse211.project.UltrasonicLocalizer.currPt;
import static ca.mcgill.ecse211.project.UltrasonicLocalizer.findBoxInsideTile;
import static simlejos.ExecutionController.performPhysicsStep;
import static simlejos.ExecutionController.setNumberOfParties;
import static simlejos.ExecutionController.sleepFor;
import static simlejos.ExecutionController.waitUntilNextStep;

import ca.mcgill.ecse211.playingfield.Point;
import simlejos.hardware.ev3.LocalEV3;

/**
 * Main class of the program. The robot will first localize to the nearest tile
 * corner and beep. Next, it will navigate to the entry of the tunnel and cross
 * it. After exiting the tunnel, it will beep again. The robot will then
 * navigate to the search zone and begin searching for blocks. Once the blocks
 * are found, the robot will begin pushing them into the bins. Once time is
 * almost up, the robot will return to the tunnel and to the initial starting
 * position.
 */
public class Main {

  /**
   * The number of threads used in the program (main, odometer), other than the
   * one used to perform physics steps.
   */
  public static final int NUMBER_OF_THREADS = 2;

  /**
   * Main entry point of the program.
   * 
   * @param args Typical argument for Main.
   */
  public static void main(String[] args) {
    initialize();

    // Start the odometer thread
    new Thread(odometer).start();

    if (TEAM_NUMBER == redTeam) {
      isRedTeam = true;
    } else if (TEAM_NUMBER == greenTeam) {
      isRedTeam = false;
    }
    if (isRedTeam == null) {
      System.out.println("This team should not be competing according to the wifi class.");
      System.out.println("Check the provided team values.");
      System.out.println("Current team in Resources: " + TEAM_NUMBER);
      System.out.println("Green Team Number in Wifi: " + greenTeam);
      System.out.println("Red Team Number in Wifi: " + redTeam);
      System.out.println("Stopping the program. Please restart the " + "simulation with the appropriate values.");
      return;
    } else {
      System.out.println("Identified team as being " + (isRedTeam ? "RED." : "GREEN."));
    }

    // Uncomment the parts relevant to the methods/functionality

    // ================== LOCALIZATION ===================
    UltrasonicLocalizer.localize();
    System.out.println("[STATUS] Performing light localization...");
    LightLocalizer.forwardLocalize(90);
    System.out.println("=> Light localization complete.");
    beep(3);
    // NOTE: Odometer will be reset by the following functions
    // =============== NAVIGATION TO TUNNEL ==============
    Navigation.goThroughTunnel();
    odometer.printPosition();
    // ============ NAVIGATION TO SEARCH ZONE ============
    // Go to search zone
    Navigation.goToSearchZone();
    beep(3);
    // ========== SEARCHING AND BLOCK DETECTION ==========
    UltrasonicLocalizer.travelSearch();
    System.out.println("=> First box is found.");

    // Point block = currPt;

    // determine if block is on the right or left of the ramp
    Point ramp = null;
    boolean left = false;
    boolean right = false;
    double rampX = 0;
    double rampY = 0;

    if (currPt.x < lowerLeftRampX) {
      left = true;
      rampX = lowerLeftRampX;
      rampY = lowerLeftRampY;
      ramp = new Point(rampX - 0.5, rampY - 0.5);
    } else if (currPt.x > lowerLeftRampX) {
      right = true;
      rampX = rr.right.x;
      rampY = rr.right.y;
      ramp = new Point(rampX + 0.5, rampY - 0.5);
    }

    findPath(ramp);
    travelTo(paths.get(0).startPosition);

    // point to start pushing
    Point push = null;
    double pushX = 0;
    double pushY = 0;

    if (left) {
      pushX = Math.round(paths.get(0).startPosition.x) - 0.5;
      if (currPt.y < lowerLeftRampY) {
        pushY = Math.round(paths.get(0).startPosition.y) + 0.5;
      } else if (currPt.y > lowerLeftRampY) {
        pushY = paths.get(0).startPosition.y - 0.5;
      }
      push = new Point(pushX, pushY);
    }

    if (right) {
      pushX = paths.get(0).startPosition.x + 0.5;
      if (currPt.y < rr.right.y) {
        pushY = paths.get(0).startPosition.y + 0.5;
      } else if (currPt.y > rr.right.y) {
        pushY = paths.get(0).startPosition.y - 0.5;
      }
      push = new Point(pushX, pushY);
    }

    double diff = Math.floor(rampY - pushY);
    if (diff != 0) {
      Point waypointPush = null;
      boolean up = false;
      Point approx = null;

      // block needs to go down
      if (diff < 0) {
        approx = new Point(paths.get(0).startPosition.x, paths.get(0).startPosition.y - 0.5);
        waypointPush = pushPosition(approx, 180);

      } else if (diff > 0) { // block needs to go up
        approx = new Point(paths.get(0).startPosition.x, paths.get(0).startPosition.y + 0.5);
        waypointPush = pushPosition(approx, 0);
        up = true;
      }

      double distance = Math.abs(rampY - waypointPush.y) / 3.281;
      if (up) {
        pushY = pushY + distance;
      } else {
        pushY = pushY - distance;
      }

      travelToSafely(waypointPush);
      // TODO turnTo() doesnt work properly
      turnTo(45);
      findBoxInsideTile();
      pushFor(distance);
      backWardAdjust();
    }

    // TODO off value travelTo()
    push = new Point(pushX + 1, pushY + 1);
    travelTo(push);
    // faceBlock
    // TODO turnTo() doesnt work properly
    turnTo(45);
    findBoxInsideTile();

    // Using a range to be extra sure that it is correct
    double pushDist = Math.abs((rampX - 1.3) - pushX) / 3.281;
    double torque = pushFor(TILE_SIZE);
    pushFor(pushDist);
    backWardAdjust();
    backWardAdjust();
    if (torque >= 0 && torque <= 0.08) {
      System.out.println("Container with weight 0.5 identified");

    } else if (torque >= 0.09 && torque <= 0.18) {
      System.out.println("Container with weight 1 identified");

    } else if (torque >= 0.19 && torque <= 0.28) {
      System.out.println("Container with weight 2 identified");

    } else if (torque >= 0.29 && torque <= 0.40) {
      System.out.println("Container with weight 3 identified");

    }
    
    // [NOTE] The issue here is likely the odometer.setX() and odometer.setY()
    // calls. The code will work if we can ensure that the odometer is accurate
    // after pushing/searching.
    
    // in front of ramp
    Point ramp2 = new Point(rampX + 0.5, rampY - 0.5);
    Point waypoint = pushPosition(ramp2, 0);
    odometer.setX(((pushX + (pushDist * 3.281)) / 3.281));
    odometer.setY(pushY / 3.281);

    // TODO off value travelTo()
    Point off = new Point(waypoint.x - 1, waypoint.y);
    travelTo(off);

    // push to the bin
    // TODO turnTo() doesnt work properly
    turnTo(325);
    findBoxInsideTile();

    // Point bin = new Point(rampX + 0.5, rampY + 1);
    pushFor(TILE_SIZE * 1.5);
    backWardAdjust();
    backWardAdjust();
    Point returnBack = new Point(rampX + 0.5, rampY - 0.5);
    odometer.setX(returnBack.x * TILE_SIZE);
    odometer.setY(returnBack.y * TILE_SIZE);

    turnBy(180);
    moveStraightFor(1);
    odometer.setTheta(180);
    odometer.printPosition();
    // go back to start
    returnToStart();
    beep(5);
  }

  /**
   * Rounds a number to the specified decimal places. Method inspired by stack
   * overflow.
   * 
   * @param value  value to round
   * @param places decimal places
   * @return
   */
  public static double round(double value, int places) {
    if (places < 0) {
      throw new IllegalArgumentException();
    }
    long factor = (long) Math.pow(10, places);
    value = value * factor;
    long tmp = Math.round(value);
    return (double) tmp / factor;
  }

  /**
   * Calculates the position the bot should be in to push a given box.
   * 
   * @param p     The point for the block's coordinates
   * @param theta The direction you want to push the block in. Can only be {0, 90,
   *              180, 270}.
   * @return Returns the final coordinates of the push position as a point.
   */
  public static Point pushPosition(Point p, double theta) {
    if (theta == 0) { // up
      return new Point(p.x, p.y - Resources.PUSH_POSITION_OFFSET);
    } else if (theta == 90) { // right
      return new Point(p.x - Resources.PUSH_POSITION_OFFSET, p.y);
    } else if (theta == 180) { // down
      return new Point(p.x, p.y + Resources.PUSH_POSITION_OFFSET);
    } else { // 270 degrees //left
      return new Point(p.x + Resources.PUSH_POSITION_OFFSET, p.y);
    }
  }

  /**
   * Helper method to beep for a given number of times.
   *
   * @param times Number of times to beep.
   */
  public static void beep(int times) {
    for (int i = 0; i < times; i++) {
      LocalEV3.getAudio().beep();
      for (int j = 0; j < 2000; j++) {
        waitUntilNextStep();
      }
    }
  }

  /**
   * Determines if a point is within a given region (in points). True if a point
   * is ON the edge.
   *
   * @param pt Point whose position to check.
   * @param LL Lower left corner of the region (point).
   * @param UR Upper right corner of the region (point).
   * @return True if the point is within, false if not.
   */
  public static boolean isWithin(Point pt, Point LL, Point UR) {
    boolean xGood = (pt.x <= UR.x && pt.x >= LL.x);
    boolean yGood = (pt.y <= UR.y && pt.y >= LL.y);
    return xGood && yGood;
  }

  /**
   * Example using WifiConnection to communicate with a server and receive data
   * concerning the competition such as the starting corner the robot is placed
   * in.<br>
   *
   * <p>
   * Keep in mind that this class is an <b>example</b> of how to use the Wi-Fi
   * code; you must use the WifiConnection class yourself in your own code as
   * appropriate. In this example, we simply show how to get and process different
   * types of data.<br>
   *
   * <p>
   * There are two variables you MUST set manually (in Resources.java) before
   * using this code:
   *
   * <ol>
   * <li>SERVER_IP: The IP address of the computer running the server application.
   * This will be your own laptop, until the beta beta demo or competition where
   * this is the TA or professor's laptop. In that case, set the IP to the default
   * (indicated in Resources).</li>
   * <li>TEAM_NUMBER: your project team number.</li>
   * </ol>
   *
   * <p>
   * Note: You can disable printing from the Wi-Fi code via
   * ENABLE_DEBUG_WIFI_PRINT.
   *
   * @author Michael Smith, Tharsan Ponnampalam, Younes Boubekeur, Olivier
   *         St-Martin Cormier
   */

  public static void wifiExample() {
    System.out.println("Running...");

    // Example 1: Print out all received data
    System.out.println("Map:\n" + wifiParameters);

    // Example 2: Print out specific values
    System.out.println("Red Team: " + redTeam);
    System.out.println("Green Zone: " + green);
    System.out.println("Island Zone, upper right: " + island.ur);
    System.out.println("Red tunnel footprint, lower left y value: " + tnr.ll.y);

    // Example 3: Compare value
    if (szg.ll.x >= island.ll.x && szg.ll.y >= island.ll.y) {
      System.out.println("The green search zone is on the island.");
    } else {
      System.err.println("The green search zone is in the water!");
    }

    // Example 4: Calculate the area of a region
    System.out.println("The island area is " + island.getWidth() * island.getHeight() + ".");
  }

  /**
   * Initializes the robot logic. It starts a new thread to perform physics steps
   * regularly.
   */
  private static void initialize() {
    // Run a few physics steps to make sure everything is initialized and has
    // settled properly
    for (int i = 0; i < 50; i++) {
      performPhysicsStep();
    }

    // We are going to start two threads, so the total number of parties is 2
    setNumberOfParties(NUMBER_OF_THREADS);

    // Does not count as a thread because it is only for physics steps
    new Thread(() -> {
      while (performPhysicsStep()) {
        sleepFor(PHYSICS_STEP_PERIOD);
      }
    }).start();
  }

}
