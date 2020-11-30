package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Navigation.convertDistance;
import static ca.mcgill.ecse211.project.Navigation.moveStraightForMeters;
import static ca.mcgill.ecse211.project.Navigation.setSpeed;
import static ca.mcgill.ecse211.project.Navigation.stopMotors;
import static ca.mcgill.ecse211.project.Navigation.turnBy;
import static ca.mcgill.ecse211.project.Resources.COLOR_SENSOR_TO_WHEEL_DIST;
import static ca.mcgill.ecse211.project.Resources.LOCAL_SPEED;
import static ca.mcgill.ecse211.project.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.project.Resources.colorSensorL;
import static ca.mcgill.ecse211.project.Resources.colorSensorR;
import static ca.mcgill.ecse211.project.Resources.leftMotor;
import static ca.mcgill.ecse211.project.Resources.odometer;
import static ca.mcgill.ecse211.project.Resources.rightMotor;
import static simlejos.ExecutionController.waitUntilNextStep;

public class LightLocalizer {
  private static float[] sampleColorL = new float[colorSensorL.sampleSize()];
  private static float[] sampleColorR = new float[colorSensorR.sampleSize()];
  private static int lastValueL = 0;
  private static int lastValueR = 0;

  /**
   * Drive forward and adjust the angle by aligning the light sensors with a line.
   * Used specifically when running the UltrasonicLocalizer.
   * 
   * @param rotation How much the bot should rotate by when readjusting (use 90)
   */
  public static void forwardLocalize(int rotation) {
    // Like relocalize but only forwards (since the bot is in the corner and needs
    // to reach 1,1.
    leftMotor.resetTachoCount();
    rightMotor.resetTachoCount();
    setSpeed(LOCAL_SPEED);
    leftMotor.forward();
    rightMotor.forward();

    boolean motorLeftStop = false;
    boolean motorRightStop = false;

    returnColorDiffR();
    returnColorDiffL();

    while (true) {
      int colorDiffR = returnColorDiffR();
      int colorDiffL = returnColorDiffL();

      if (colorDiffR < -40) {
        rightMotor.stop();
        motorRightStop = true;
        colorDiffR = returnColorDiffR();

      }

      if (colorDiffL < -40) {
        leftMotor.stop();
        motorLeftStop = true;
        colorDiffL = returnColorDiffL();
      }

      // the robot is on a line
      if (motorLeftStop && motorRightStop) {
        System.out.println("=> Line detected. Correcting angle...");
        setSpeed(LOCAL_SPEED);
        moveStraightForMeters(COLOR_SENSOR_TO_WHEEL_DIST); // 0.11354
        // depends of the position of the bot, can be -90
        setSpeed(LOCAL_SPEED);
        turnBy(rotation);
        break;
      }
      waitUntilNextStep();
    }

    if (rotation > 0) {
      rotation = -90;
      forwardLocalize(rotation);
    }
  }

  /**
   * A method that allows us to recenter the bot around a grid corner within 15cm
   * in any direction.
   */
  public static void relocalize() {
    double[] xyt = odometer.getXyt();
    // System.out.println(xyt[0] + " " + xyt[1] + " " + xyt[2]);
    setSpeed(LOCAL_SPEED);
    System.out.println("=> Relocalizing...");
    Navigation.turnTo(0);

    if (!adjustForward()) {
      adjustBackward();
    }

    Navigation.turnBy(90);
    if (!adjustForward()) {
      adjustBackward();
    }
    setSpeed(LOCAL_SPEED);
    Navigation.turnBy(-90);

    odometer.setXyt(Math.round(xyt[0] / TILE_SIZE) * TILE_SIZE, 
          Math.round(xyt[1] / TILE_SIZE) * TILE_SIZE, 0);
    xyt = odometer.getXyt();
    // System.out.println(xyt[0] + " " + xyt[1] + " " + xyt[2]);
  }

  /**
   * Drives the bot forward 10cm and adjusts orientation if a line is found.
   * Returns to its initial position if no line was found.
   * 
   * @return True if a line was found. False otherwise.
   */
  public static boolean adjustForward() {
    leftMotor.resetTachoCount();
    rightMotor.resetTachoCount();
    double tachoL = leftMotor.getTachoCount();
    setSpeed(LOCAL_SPEED);
    leftMotor.forward();
    rightMotor.forward();

    returnColorDiffR();
    returnColorDiffL();
    boolean motorLeftStop = false;
    boolean motorRightStop = false;

    while (true) {
      int colorDiffR = returnColorDiffR();
      int colorDiffL = returnColorDiffL();

      if (colorDiffR < -40) {
        rightMotor.stop();
        motorRightStop = true;

      }

      if (colorDiffL < -40) {
        leftMotor.stop();
        motorLeftStop = true;
      }

      // the robot is on a line
      if (motorLeftStop && motorRightStop) {
        stopMotors();
        break;
      } else if (tachoL >= convertDistance(0.10)) {
        // stopped moving and didn't detect line
        return false;
      }
      tachoL = leftMotor.getTachoCount();
      waitUntilNextStep();
    }
    setSpeed(LOCAL_SPEED);
    moveStraightForMeters(COLOR_SENSOR_TO_WHEEL_DIST);
    return true;
  }

  /**
   * Drives the bot backwards 10cm and adjusts orientation if a line is found.
   * 
   * @return True if a line was found. False otherwise.
   */
  public static boolean adjustBackward() {
    leftMotor.resetTachoCount();
    rightMotor.resetTachoCount();
    double tachoL = leftMotor.getTachoCount();
    setSpeed(LOCAL_SPEED);
    leftMotor.backward();
    rightMotor.backward();

    returnColorDiffR();
    returnColorDiffL();
    boolean motorLeftStop = false;
    boolean motorRightStop = false;

    while (true) {
      int colorDiffR = returnColorDiffR();
      int colorDiffL = returnColorDiffL();

      if (colorDiffR < -40) {
        rightMotor.stop();
        motorRightStop = true;

      }

      if (colorDiffL < -40) {
        leftMotor.stop();
        motorLeftStop = true;
      }

      // the robot is on a line
      if (motorLeftStop && motorRightStop) {
        stopMotors();
        break;
      } else if (tachoL <= convertDistance(-0.20)) {
        // stopped moving and didn't detect line
        moveStraightForMeters(0.10);
        return false;
      }
      tachoL = leftMotor.getTachoCount();
      waitUntilNextStep();
    }
    setSpeed(LOCAL_SPEED);
    moveStraightForMeters(COLOR_SENSOR_TO_WHEEL_DIST);
    return true;
  }

  /**
   * A method to get the difference between the current color value and the last
   * color value of the left sensor method of finite differences.
   * 
   * @return difference (int)
   */
  public static int returnColorDiffL() {
    int sensorValueL = returnColorL();
    int diff = sensorValueL - lastValueL;
    lastValueL = sensorValueL;
    return diff;
  }

  /**
   * A method to get the difference between the current color value and the last
   * color value of the right sensor method of finite differences.
   * 
   * @return difference (int)
   */
  public static int returnColorDiffR() {
    int sensorValueR = returnColorR();
    int diff = sensorValueR - lastValueR;
    lastValueR = sensorValueR;
    return diff;
  }

  /**
   * A method to get the left sensor color sample.
   * 
   * @return Color value (int)
   */
  public static int returnColorL() {
    colorSensorL.fetchSample(sampleColorL, 0);
    return (int) sampleColorL[0];
  }

  /**
   * A method to get the right sensor color sample.
   * 
   * @return Color value (int)
   */
  public static int returnColorR() {
    colorSensorR.fetchSample(sampleColorR, 0);
    return (int) sampleColorR[0];
  }

  /**
   * A method to read and update the left color sensor data.
   */
  public static void readColorL() {
    colorSensorL.fetchSample(sampleColorL, 0);
  }

  /**
   * A method to read and update the right color sensor data.
   */
  public static void readColorR() {
    colorSensorR.fetchSample(sampleColorR, 0);
  }

}
