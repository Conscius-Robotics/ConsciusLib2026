package com.team10043.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Optional;

/**
 * Utility class for 2D geometric calculations commonly used in FRC applications.
 *
 * <p>This class provides methods for:
 *
 * <ul>
 *   <li>Line-line intersection calculation
 *   <li>Line-circle intersection detection and calculation
 *   <li>Distance and angle-to-target calculations
 * </ul>
 *
 * <p>All methods are static and thread-safe. This class cannot be instantiated.
 */
public final class GeometryUtils {

  private GeometryUtils() {
    // Prevent instantiation
  }

  /**
   * Represents a 2D line in parametric form: P = origin + t * direction
   *
   * @param origin a point on the line
   * @param direction direction vector of the line (does not need to be normalized)
   */
  public record Line(Translation2d origin, Translation2d direction) {

    /**
     * Gets a point on the line at parameter t.
     *
     * @param t parameter value
     * @return point on the line
     */
    public Translation2d pointAt(double t) {
      return origin.plus(direction.times(t));
    }

    /**
     * Gets the angle of the line direction.
     *
     * @return angle of the direction vector
     */
    public Rotation2d angle() {
      return direction.getAngle();
    }
  }

  /**
   * Creates a line from two points.
   *
   * @param p1 first point on the line
   * @param p2 second point on the line
   * @return line passing through both points
   */
  public static Line lineFromPoints(Translation2d p1, Translation2d p2) {
    Translation2d direction = p2.minus(p1);
    return new Line(p1, direction);
  }

  /**
   * Creates a line from a point and an angle.
   *
   * @param origin point on the line
   * @param angle angle of the line direction
   * @return line with the specified origin and angle
   */
  public static Line lineFromPointAndAngle(Translation2d origin, Rotation2d angle) {
    Translation2d direction = new Translation2d(1.0, angle);
    return new Line(origin, direction);
  }

  /**
   * Calculates the intersection point of two lines.
   *
   * <p>Uses parametric form: Line1: P1 = origin1 + t1 * dir1 Line2: P2 = origin2 + t2 * dir2
   *
   * <p>Returns empty if lines are parallel (no intersection or infinite intersections).
   *
   * @param line1 first line
   * @param line2 second line
   * @return intersection point, or empty if lines are parallel
   */
  public static Optional<Translation2d> lineIntersection(Line line1, Line line2) {
    Translation2d o1 = line1.origin();
    Translation2d d1 = line1.direction();
    Translation2d o2 = line2.origin();
    Translation2d d2 = line2.direction();

    // Solve the system:
    // o1.x + t1 * d1.x = o2.x + t2 * d2.x
    // o1.y + t1 * d1.y = o2.y + t2 * d2.y
    //
    // Rearranged:
    // t1 * d1.x - t2 * d2.x = o2.x - o1.x
    // t1 * d1.y - t2 * d2.y = o2.y - o1.y

    double denominator = d1.getX() * d2.getY() - d1.getY() * d2.getX();

    // Check if lines are parallel (denominator = 0)
    if (Math.abs(denominator) < 1e-10) {
      return Optional.empty();
    }

    double dx = o2.getX() - o1.getX();
    double dy = o2.getY() - o1.getY();

    double t1 = (dx * d2.getY() - dy * d2.getX()) / denominator;

    Translation2d intersection = line1.pointAt(t1);
    return Optional.of(intersection);
  }

  /**
   * Checks if a line intersects a circle and returns the intersection points if any.
   *
   * <p>Returns intersection points as follows:
   *
   * <ul>
   *   <li>0 points: Line doesn't intersect circle
   *   <li>1 point: Line is tangent to circle (touches at exactly one point)
   *   <li>2 points: Line crosses through circle
   * </ul>
   *
   * <p>If the line's direction vector is near-zero (degenerate line), treats it as a point-circle
   * distance check.
   *
   * @param line the line to check
   * @param center center of the circle
   * @param radius radius of the circle
   * @return array of intersection points (0, 1, or 2 points)
   */
  public static Translation2d[] lineCircleIntersection(
      Line line, Translation2d center, double radius) {

    // Transform to a coordinate system where the circle is at the origin
    Translation2d localOrigin = line.origin().minus(center);
    Translation2d dir = line.direction();

    // Solve: |origin + t * dir|^2 = radius^2
    // (ox + t * dx)^2 + (oy + t * dy)^2 = r^2
    // t^2 * (dx^2 + dy^2) + 2t * (ox*dx + oy*dy) + (ox^2 + oy^2 - r^2) = 0

    double a = dir.getX() * dir.getX() + dir.getY() * dir.getY();

    // Guard against near-zero direction vector (degenerate line)
    if (a < 1e-10) {
      // Treat as point-circle check: is the line origin inside the circle?
      double distanceSquared =
          localOrigin.getX() * localOrigin.getX() + localOrigin.getY() * localOrigin.getY();
      if (distanceSquared <= radius * radius + 1e-10) {
        // Point is on or inside circle
        return new Translation2d[] {line.origin()};
      } else {
        // Point is outside circle
        return new Translation2d[0];
      }
    }

    double b = 2.0 * (localOrigin.getX() * dir.getX() + localOrigin.getY() * dir.getY());
    double c =
        localOrigin.getX() * localOrigin.getX()
            + localOrigin.getY() * localOrigin.getY()
            - radius * radius;

    double discriminant = b * b - 4.0 * a * c;

    if (discriminant < -1e-10) {
      // No intersection (negative discriminant)
      return new Translation2d[0];
    }

    if (discriminant < 1e-10) {
      // Tangent (one intersection point)
      double t = -b / (2.0 * a);
      Translation2d point = line.pointAt(t);
      return new Translation2d[] {point};
    }

    // Two intersection points
    double sqrtDiscriminant = Math.sqrt(discriminant);
    double t1 = (-b - sqrtDiscriminant) / (2.0 * a);
    double t2 = (-b + sqrtDiscriminant) / (2.0 * a);

    Translation2d point1 = line.pointAt(t1);
    Translation2d point2 = line.pointAt(t2);

    return new Translation2d[] {point1, point2};
  }

  /**
   * Checks if a line segment (between t1 and t2) intersects a circle.
   *
   * @param line the line
   * @param t1 start parameter of the segment
   * @param t2 end parameter of the segment
   * @param center center of the circle
   * @param radius radius of the circle
   * @return true if the line segment intersects the circle
   */
  public static boolean lineSegmentIntersectsCircle(
      Line line, double t1, double t2, Translation2d center, double radius) {

    Translation2d[] intersections = lineCircleIntersection(line, center, radius);

    if (intersections.length == 0) {
      return false;
    }

    // Check if any intersection point is within the segment bounds
    for (Translation2d intersection : intersections) {
      // Find the parameter t for this intersection point
      double t = findParameterForPoint(line, intersection);
      if (t >= Math.min(t1, t2) && t <= Math.max(t1, t2)) {
        return true;
      }
    }

    return false;
  }

  /**
   * Finds the parameter t such that line.pointAt(t) equals the given point.
   *
   * <p>Assumes the point is on the line. Returns NaN if the direction vector is zero.
   *
   * @param line the line
   * @param point point on the line
   * @return parameter t, or NaN if undefined
   */
  public static double findParameterForPoint(Line line, Translation2d point) {
    Translation2d diff = point.minus(line.origin());
    Translation2d dir = line.direction();

    // Use the larger component to avoid division by small numbers
    if (Math.abs(dir.getX()) > Math.abs(dir.getY())) {
      return diff.getX() / dir.getX();
    } else if (Math.abs(dir.getY()) > 1e-10) {
      return diff.getY() / dir.getY();
    } else {
      return Double.NaN; // Direction vector is zero
    }
  }

  /**
   * Calculates the closest point on a line to a given point.
   *
   * @param line the line
   * @param point the point
   * @return closest point on the line to the given point
   */
  public static Translation2d closestPointOnLine(Line line, Translation2d point) {
    Translation2d diff = point.minus(line.origin());
    Translation2d dir = line.direction();

    double dirLengthSquared = dir.getX() * dir.getX() + dir.getY() * dir.getY();

    if (dirLengthSquared < 1e-10) {
      // Direction vector is zero, return the origin
      return line.origin();
    }

    double t = (diff.getX() * dir.getX() + diff.getY() * dir.getY()) / dirLengthSquared;

    return line.pointAt(t);
  }

  /**
   * Calculates the perpendicular distance from a point to a line.
   *
   * @param line the line
   * @param point the point
   * @return perpendicular distance from point to line
   */
  public static double distanceToLine(Line line, Translation2d point) {
    Translation2d closest = closestPointOnLine(line, point);
    return point.getDistance(closest);
  }

  /**
   * Creates a perpendicular line passing through a given point.
   *
   * @param line the original line
   * @param point point through which the perpendicular line passes
   * @return perpendicular line
   */
  public static Line perpendicularLine(Line line, Translation2d point) {
    Translation2d dir = line.direction();
    // Perpendicular direction: rotate 90 degrees (swap x and y, negate one)
    Translation2d perpDir = new Translation2d(-dir.getY(), dir.getX());
    return new Line(point, perpDir);
  }

  /**
   * Checks if a point is on a line within a tolerance.
   *
   * @param line the line
   * @param point the point to check
   * @param tolerance maximum distance from line to be considered "on" the line
   * @return true if point is on the line within tolerance
   */
  public static boolean isPointOnLine(Line line, Translation2d point, double tolerance) {
    return distanceToLine(line, point) <= tolerance;
  }

  /**
   * Calculates the angle from one pose to another.
   *
   * @param from starting pose
   * @param to target pose
   * @return angle from 'from' to 'to'
   */
  public static Rotation2d angleToTarget(Pose2d from, Pose2d to) {
    Translation2d diff = to.getTranslation().minus(from.getTranslation());
    return diff.getAngle();
  }

  /**
   * Calculates the angle from one translation to another.
   *
   * @param from starting translation
   * @param to target translation
   * @return angle from 'from' to 'to'
   */
  public static Rotation2d angleToTarget(Translation2d from, Translation2d to) {
    Translation2d diff = to.minus(from);
    return diff.getAngle();
  }
}
