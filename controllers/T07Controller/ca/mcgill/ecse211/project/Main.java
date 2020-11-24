package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static ca.mcgill.ecse211.project.UltrasonicLocalizer.*;
import static ca.mcgill.ecse211.project.LightLocalizer.*;
import static ca.mcgill.ecse211.project.Navigation.*;
import static simlejos.ExecutionController.*;

import java.lang.Thread;
import java.util.ArrayList;
import ca.mcgill.ecse211.playingfield.*;

import simlejos.hardware.ev3.LocalEV3;

/**
 * Main class of the program.
 * The robot will first localize to the nearest tile corner and beep.
 * Next, it will navigate to the entry of the tunnel and cross it.
 * After exiting the tunnel, it will beep again.
 * The robot will then navigate to the search zone and begin searching for blocks.
 * Once the blocks are found, the robot will begin pushing them into the bins.
 * Once time is almost up, the robot will return to the tunnel and to the initial starting position.
 */
public class Main {

  /**
   * The number of threads used in the program (main, odometer), other than the
   * one used to perform physics steps.
   */
  public static final int NUMBER_OF_THREADS = 2;

  /** Main entry point of the program.
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
      System.out.println("Stopping the program. Please restart the "
          + "simulation with the appropriate values.");
      return;
    } else {
      System.out.println("Identified team as being " + (isRedTeam ? "RED." : "GREEN."));
    }

    // Uncomment the parts relevant to the methods/functionality

    // ================== LOCALIZATION ===================
    // UltrasonicLocalizer.localize();
    // System.out.println("[STATUS] Performing light localization...");
    // LightLocalizer.forwardLocalize(90);
    // System.out.println("=> Light localization complete.");
    // beep(3);
    // NOTE: Odometer will be reset by the following functions
    // =============== NAVIGATION TO TUNNEL ==============
    // Navigation.goThroughTunnel();
    // odometer.printPosition();
    // ============ NAVIGATION TO SEARCH ZONE ============
    // Go to search zone
    // if (Navigation.inSearchZone() == false) {
    // Navigation.goToSearchZone();
    // }
    // beep(3);
    // ========== SEARCHING AND BLOCK DETECTION ==========
    // TODO search
    
    Point block = new Point(7,6);
    Point farestEdege = farestEdge(block);
    navigateTo(middleRamp, block);
    
    double torque = pushFor(Resources.TILE_SIZE);
    
    while(Math.round(odometer.getXyt()[0]) != middleRamp.x && Math.round(odometer.getXyt()[1]) != middleRamp.y)
    
    if(torque > 3) { //random constant
    	reposition(middleRamp, block);
    }
    else {
    	torque = pushFor(Resources.TILE_SIZE);
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
   * <p>Keep in mind that this class is an <b>example</b> of how to use the Wi-Fi
   * code; you must use the WifiConnection class yourself in your own code as
   * appropriate. In this example, we simply show how to get and process different
   * types of data.<br>
   * 
   * <p>There are two variables you MUST set manually (in Resources.java) before
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
   * <p>Note: You can disable printing from the Wi-Fi code via
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
