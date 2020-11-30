package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.playingfield.Point;

public class SafePath {
  public double lenght;
  public double angle;
  public Point startPosition;

  /**
   * Constructor for a SafePath class.
   * @param lenght length in meters.
   * @param angle Heading in degrees.
   * @param startPosition Starting position (point).
   */
  public SafePath(double lenght, double angle, Point startPosition) {
    this.lenght = lenght;
    this.angle = angle;
    this.startPosition = startPosition;

  }

}
