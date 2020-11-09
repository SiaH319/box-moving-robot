package ca.mcgill.ecse211.playingfield;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents a coordinate point on the playing field grid.
 * 
 * @author Younes Boubekeur
 */
public class Point {
  /** The x coordinate in tile lengths. */
  public double x;

  /** The y coordinate in tile lengths. */
  public double y;

  /** Constructs a Point. The arguments are in tile lengths. */
  public Point(double x, double y) {
    this.x = x;
    this.y = y;
  }
  
  /**
   * Makes points from a string, eg "(1,1),(2.5,3)".
   * 
   * @return a list of points
   */
  public static List<Point> makePointsFromString(String s) {
    List<Point> result = new ArrayList<Point>();
    
    if (s == null || !s.contains(")")) {
      return result;
    }
    
    s = s.replaceAll("\\s+", "").replaceAll("\\(", "").replaceAll("\\),", "\\)");
    
    for (var fragment: s.split("\\)")) {
      var xy = fragment.split(",");
      result.add(new Point(Double.parseDouble(xy[0]), Double.parseDouble(xy[1])));
    }
    
    return result;
  }
  
  @Override
  public boolean equals(Object o) {
    if (!(o instanceof Point)) {
      return false;
    }
    
    Point other = (Point) o;
    return x == other.x && y == other.y;
  }
  
  @Override
  public final int hashCode() {
    return (int) (100 * x + y);
  }

  @Override
  public String toString() {
    return "(" + x + ", " + y + ")";
  }

}
