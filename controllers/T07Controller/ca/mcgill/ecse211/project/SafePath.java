package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.playingfield.Point;

public class SafePath {
  /** Length of the safe path in meters. */
  public double lenght;
  /** Heading of the safe path in degrees. */
  public double angle;
  /** Beginning of the safe path (point). */
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
