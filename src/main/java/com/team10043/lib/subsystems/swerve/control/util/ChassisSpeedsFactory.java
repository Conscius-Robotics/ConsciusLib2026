package com.team10043.lib.subsystems.swerve.control.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Helper class for converting between robot-relative and field-relative coordinate frames and
 * creating ChassisSpeeds objects with proper scaling.
 */
public class ChassisSpeedsFactory {

  private final double maxLinearSpeed;
  private final double maxAngularSpeed;

  /**
   * Creates a new SwerveKinematicsHelper.
   *
   * @param maxLinearSpeed Maximum linear speed in m/s
   * @param maxAngularSpeed Maximum angular speed in rad/s
   */
  public ChassisSpeedsFactory(double maxLinearSpeed, double maxAngularSpeed) {
    this.maxLinearSpeed = maxLinearSpeed;
    this.maxAngularSpeed = maxAngularSpeed;
  }

  /**
   * Creates robot-relative ChassisSpeeds from normalized velocity inputs.
   *
   * @param linearVelocity Normalized linear velocity vector (0 to 1)
   * @param angularVelocity Normalized angular velocity (-1 to 1)
   * @return ChassisSpeeds scaled to max velocities
   */
  public ChassisSpeeds createChassisSpeeds(Translation2d linearVelocity, double angularVelocity) {
    return new ChassisSpeeds(
        linearVelocity.getX() * maxLinearSpeed,
        linearVelocity.getY() * maxLinearSpeed,
        angularVelocity * maxAngularSpeed);
  }

  /**
   * Converts robot-relative ChassisSpeeds to field-relative. Handles alliance flip for red
   * alliance.
   *
   * @param speeds Robot-relative chassis speeds
   * @param robotRotation Current robot rotation
   * @return Field-relative chassis speeds
   */
  public ChassisSpeeds toFieldRelative(ChassisSpeeds speeds, Rotation2d robotRotation) {
    // Check if we need to flip for red alliance
    boolean flipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    Rotation2d effectiveRotation =
        flipped ? robotRotation.plus(Rotation2d.fromRadians(Math.PI)) : robotRotation;

    return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, effectiveRotation);
  }

  /**
   * Creates field-relative ChassisSpeeds directly from normalized inputs.
   *
   * @param linearVelocity Normalized linear velocity vector (0 to 1)
   * @param angularVelocity Normalized angular velocity (-1 to 1)
   * @param robotRotation Current robot rotation
   * @return Field-relative chassis speeds
   */
  public ChassisSpeeds createFieldRelativeSpeeds(
      Translation2d linearVelocity, double angularVelocity, Rotation2d robotRotation) {
    ChassisSpeeds robotRelative = createChassisSpeeds(linearVelocity, angularVelocity);
    return toFieldRelative(robotRelative, robotRotation);
  }

  /**
   * Creates field-relative {@link ChassisSpeeds} for open-loop control WITHOUT scaling the linear
   * components to physical max speeds. The linear "velocity" values in the returned {@link
   * ChassisSpeeds} are unitless duty-cycle / joystick percentages in the range [-1, 1], which are
   * intended to be scaled by the caller (for example, to volts) before being sent to the drive
   * motors. This allows for maximum power output without any speed limitations imposed by this
   * helper.
   *
   * @param linearVelocity Raw unitless linear command vector from joystick (-1 to 1), interpreted
   *     as duty-cycle percentages rather than meters per second
   * @param angularVelocity Raw angular velocity from joystick (-1 to 1), scaled to max angular
   *     speed
   * @param robotRotation Current robot rotation
   * @return Field-relative chassis speeds whose linear components are unitless duty-cycle values
   *     ([-1, 1]) to be scaled (for example, to voltage) by the caller
   */
  public ChassisSpeeds createFieldRelativeSpeedsOpenLoop(
      Translation2d linearVelocity, double angularVelocity, Rotation2d robotRotation) {
    // For open loop, linear values stay as-is (duty cycle percentage)
    // Only angular velocity is scaled since turn motors still use PID
    ChassisSpeeds robotRelative =
        new ChassisSpeeds(
            linearVelocity.getX(), linearVelocity.getY(), angularVelocity * maxAngularSpeed);

    ChassisSpeeds fieldRelative = toFieldRelative(robotRelative, robotRotation);

    // CRITICAL: Normalize the linear component back to [-1, 1] range after rotation
    // Field-relative rotation can increase magnitude beyond joystick input
    double magnitude = Math.hypot(fieldRelative.vxMetersPerSecond, fieldRelative.vyMetersPerSecond);
    if (magnitude > 1.0) {
      fieldRelative.vxMetersPerSecond /= magnitude;
      fieldRelative.vyMetersPerSecond /= magnitude;
    }

    return fieldRelative;
  }
}
