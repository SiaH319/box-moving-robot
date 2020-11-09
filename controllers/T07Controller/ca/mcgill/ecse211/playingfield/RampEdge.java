package ca.mcgill.ecse211.playingfield;

/** Line segment representing the approach of a ramp. */
public class RampEdge {

  /** The left endpoint of the ramp. */
  public Point left;
  
  /** The right endpoint of the ramp. */
  public Point right;
  
  /** Constructs a RampEdge. */
  public RampEdge(Point left, Point right) {
    this.left = left;
    this.right = right;
  }

}
