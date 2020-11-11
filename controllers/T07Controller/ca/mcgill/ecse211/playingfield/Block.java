package ca.mcgill.ecse211.playingfield;

/**
 * This class allows us to store information about known block positions 
 * and the torques associated with them.
 * @author adela
 */
public class Block {
  
  /** Position of the block (Point). */
  private Point position;
  /** Torque associated with the block in Nm (double). */
  private double torque;
  
  /** 
   * Constructor for the block class.
   * @param pos Position of the block (point).
   * @param trk Torque associated with the block (Nm)
   */
  public Block(Point pos, double trk) {
    this.position = pos;
    this.torque = trk;
  }
  
  /**
   * Setter for the position of the block.
   * @param pos Position of the block (point).
   */
  public void setPos(Point pos) {
    this.position = pos;
  }
  
  /**
   * Setter for the position of the block.
   * @param trk Torque associated with the block (double).
   */
  public void setTrk(double trk) {
    this.torque = trk;
  }
  
  /**
   * Getter for the torque associated with the block.
   * @return torque (double)
   */
  public double getTrk() {
    return this.torque;
  }
  
  /**
   * Getter for the position of the block.
   * @return Position of the block (point)
   */
  public Point getPos() {
    return this.position;
  }
}
