package com.team10043.lib.subsystems.swerve.config.hardware;

import edu.wpi.first.math.geometry.Translation2d;
import lombok.EqualsAndHashCode;
import lombok.Getter;
import lombok.experimental.Accessors;

/**
 * Encapsulates the physical geometry of a swerve-drive robot base.
 *
 * <p>This class holds the primary physical dimensions required to describe a swerve drivetrain: the
 * track widths along the X and Y axes, the wheel radius, and an extra robot width parameter. From
 * those values it derives convenience information used by kinematics and control code.
 *
 * <p>All linear measurements are expected to use consistent units (for example, meters). Instances
 * are immutable with respect to the primary dimension fields (trackWidthX, trackWidthY,
 * wheelRadius, robotWidth).
 *
 * <p>Derived values:
 *
 * <ul>
 *   <li>{@code moduleTranslations()} lazily computes and caches an array of Translation2d vectors
 *       that represent the translation from the robot center to each swerve module. The entries are
 *       produced in this order: (trackWidthX/2, trackWidthY/2), (trackWidthX/2, -trackWidthY/2),
 *       (-trackWidthX/2, trackWidthY/2), (-trackWidthX/2, -trackWidthY/2). These correspond to the
 *       four module corner offsets relative to the robot center using the class's coordinate
 *       convention.
 *   <li>{@code driveBaseRadius()} returns the distance from the robot center to a module, computed
 *       as the Euclidean norm of half the track widths: sqrt((trackWidthX/2)^2 +
 *       (trackWidthY/2)^2).
 * </ul>
 *
 * <p>Usage notes:
 *
 * <ul>
 *   <li>The module translations are computed once and cached; if the geometry values were to change
 *       (not expected for this immutable design), the cache would need to be invalidated.
 *   <li>The wheelRadius is provided for conversions between wheel angular velocities and linear
 *       speeds; it is not directly used by the geometric translation calculations.
 * </ul>
 */
@Getter
@Accessors(fluent = true)
@EqualsAndHashCode
public class SwerveChassisDimensions {

  private final double trackWidthX;
  private final double trackWidthY;
  private final double wheelRadius;
  private final double robotWidth;

  private Translation2d[] moduleTranslations;

  public SwerveChassisDimensions(
      double trackWidthX, double trackWidthY, double wheelRadius, double robotWidth) {
    // Validate dimensions
    if (trackWidthX <= 0.0) {
      throw new IllegalArgumentException("trackWidthX must be positive, got: " + trackWidthX);
    }
    if (trackWidthY <= 0.0) {
      throw new IllegalArgumentException("trackWidthY must be positive, got: " + trackWidthY);
    }
    if (wheelRadius <= 0.0) {
      throw new IllegalArgumentException("wheelRadius must be positive, got: " + wheelRadius);
    }
    if (robotWidth <= 0.0) {
      throw new IllegalArgumentException("robotWidth must be positive, got: " + robotWidth);
    }

    this.trackWidthX = trackWidthX;
    this.trackWidthY = trackWidthY;
    this.wheelRadius = wheelRadius;
    this.robotWidth = robotWidth;

    this.moduleTranslations =
        new Translation2d[] {
          new Translation2d(trackWidthX / 2, trackWidthY / 2),
          new Translation2d(trackWidthX / 2, -trackWidthY / 2),
          new Translation2d(-trackWidthX / 2, trackWidthY / 2),
          new Translation2d(-trackWidthX / 2, -trackWidthY / 2)
        };
  }

  /**
   * Factory method to create a SwerveChassisDimensions instance.
   *
   * @param trackWidthX the track width along the X axis (same units as wheelRadius)
   * @param trackWidthY the track width along the Y axis (same units as wheelRadius)
   * @param wheelRadius the radius of the wheels (same units as trackWidthX/Y)
   * @param robotWidth an additional robot width parameter (same units as trackWidthX/Y)
   * @return a new SwerveChassisDimensions instance with the specified dimensions
   */
  public static SwerveChassisDimensions of(
      double trackWidthX, double trackWidthY, double wheelRadius, double robotWidth) {
    return new SwerveChassisDimensions(trackWidthX, trackWidthY, wheelRadius, robotWidth);
  }

  public Translation2d[] moduleTranslations() {
    return moduleTranslations;
  }

  public double driveBaseRadius() {
    return Math.hypot(trackWidthX / 2.0, trackWidthY / 2.0);
  }
}
