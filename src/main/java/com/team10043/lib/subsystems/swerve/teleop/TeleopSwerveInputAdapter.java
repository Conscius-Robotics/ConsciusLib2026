package com.team10043.lib.subsystems.swerve.teleop;

import com.team10043.lib.subsystems.swerve.control.util.ChassisSpeedsFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.DoubleSupplier;

/**
 * Processes raw joystick inputs for teleoperated swerve drive control. Applies deadband, input
 * shaping, and converts to field-relative chassis speeds for both closed-loop and open-loop control
 * modes.
 */
public class TeleopSwerveInputAdapter {
  private final double deadband;

  private final ChassisSpeedsFactory kinematics;

  /**
   * Constructs a TeleopInputAdapter with the specified kinematics and deadband.
   *
   * @param kinematics Swerve kinematics helper
   * @param deadband Deadband threshold for joystick inputs
   */
  public TeleopSwerveInputAdapter(ChassisSpeedsFactory kinematics, double deadband) {
    this.kinematics = kinematics;
    this.deadband = deadband;
  }

  /**
   * Calculates field-relative chassis speeds for closed-loop velocity control.
   *
   * @param xSupplier Forward/backward joystick input
   * @param ySupplier Left/right joystick input
   * @param omegaSupplier Rotational joystick input
   * @param currentRotation Current robot rotation
   * @return Field-relative chassis speeds
   */
  public ChassisSpeeds calculateClosedLoop(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Rotation2d currentRotation) {

    Translation2d linear = getLinearVelocity(xSupplier, ySupplier);
    double omega = getAngularVelocity(omegaSupplier);

    return kinematics.createFieldRelativeSpeeds(linear, omega, currentRotation);
  }

  /**
   * Calculates field-relative chassis speeds for open-loop control.
   *
   * <p>Velocity values are normalized in [-1, 1] and intended for use with {@code
   * Swerve.runOpenLoop}. Downstream, drive motors will map these normalized speeds to either
   * voltage (±12 V typical) or torque current (FOC) depending on configuration; turn motors hold
   * position with PID.
   *
   * @param xSupplier Forward/backward joystick input
   * @param ySupplier Left/right joystick input
   * @param omegaSupplier Rotational joystick input
   * @param currentRotation Current robot rotation
   * @return Field-relative chassis speeds normalized for open-loop application
   */
  public ChassisSpeeds calculateOpenLoop(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Rotation2d currentRotation) {

    Translation2d linear = getLinearVelocity(xSupplier, ySupplier);
    double omega = getAngularVelocity(omegaSupplier);

    return kinematics.createFieldRelativeSpeedsOpenLoop(linear, omega, currentRotation);
  }

  /**
   * Converts joystick X/Y inputs to a linear velocity vector. Applies deadband and squares the
   * magnitude for better control.
   *
   * @param xSupplier Forward/backward input supplier (-1 to 1)
   * @param ySupplier Left/right input supplier (-1 to 1)
   * @return Translation2d representing the desired velocity direction and magnitude (0 to 1)
   */
  public Translation2d getLinearVelocity(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    double x = xSupplier.getAsDouble();
    double y = ySupplier.getAsDouble();

    double magnitude = MathUtil.applyDeadband(Math.hypot(x, y), deadband);

    // Square for better control at low speeds
    magnitude *= magnitude;

    Rotation2d direction = new Rotation2d(Math.atan2(y, x));

    return new Pose2d(new Translation2d(), direction)
        .transformBy(new Transform2d(magnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Converts joystick angular input to angular velocity. Applies deadband and squares the input
   * while preserving sign.
   *
   * @param omegaInput Raw angular input (-1 to 1)
   * @return Processed angular velocity (-1 to 1)
   */
  public double getAngularVelocity(double omegaInput) {
    double deadbanded = MathUtil.applyDeadband(omegaInput, deadband);
    return Math.copySign(deadbanded * deadbanded, deadbanded);
  }

  /**
   * Converts joystick angular input to angular velocity. Applies deadband and squares the input
   * while preserving sign.
   *
   * @param omegaSupplier Supplier for raw angular input (-1 to 1)
   * @return Processed angular velocity (-1 to 1)
   */
  public double getAngularVelocity(DoubleSupplier omegaSupplier) {
    return getAngularVelocity(omegaSupplier.getAsDouble());
  }
}
