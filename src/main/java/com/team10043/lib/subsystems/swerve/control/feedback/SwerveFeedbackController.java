package com.team10043.lib.subsystems.swerve.control.feedback;

import com.team10043.lib.util.control.ControlGains.PIDGains;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Implements pure feedback control logic for a swerve drivetrain using {@link
 * ProfiledPIDController}s.
 *
 * <p>This class is intentionally free of any tuning, logging, or NetworkTables-related concerns. It
 * operates purely on the provided PID controllers and current/target states, making it
 * deterministic, testable, and suitable for simulation or replay.
 *
 * <p>Responsibilities:
 *
 * <ul>
 *   <li>Maintain independent PID controllers for X, Y, and rotation (theta)
 *   <li>Reset controller state based on the current robot pose
 *   <li>Compute field-relative {@link ChassisSpeeds} commands
 *   <li>Expose goal-completion state for translation and rotation
 * </ul>
 *
 * <p>PID gains may be updated externally (e.g. by a tuning manager) via package-private setter
 * methods.
 */
public class SwerveFeedbackController {

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  /**
   * Constructs a new {@code SwerveFeedbackController}.
   *
   * @param xController PID controller responsible for field X translation
   * @param yController PID controller responsible for field Y translation
   * @param thetaController PID controller responsible for robot rotation (radians)
   */
  public SwerveFeedbackController(
      ProfiledPIDController xController,
      ProfiledPIDController yController,
      ProfiledPIDController thetaController) {

    this.xController = xController;
    this.yController = yController;
    this.thetaController = thetaController;
  }

  /**
   * Resets all internal PID controllers using the current robot pose.
   *
   * @param currentPose current estimated pose of the robot
   */
  public void reset(Pose2d currentPose) {
    xController.reset(currentPose.getX());
    yController.reset(currentPose.getY());
    thetaController.reset(currentPose.getRotation().getRadians());
  }

  /**
   * Resets only the rotational (theta) controller.
   *
   * @param currentRotation current robot rotation
   */
  public void resetTheta(Rotation2d currentRotation) {
    thetaController.reset(currentRotation.getRadians());
  }

  /**
   * Calculates field-relative chassis speeds required to move from the current pose toward the
   * target pose.
   *
   * @param currentPose current estimated pose of the robot
   * @param targetPose desired target pose
   * @return field-relative {@link ChassisSpeeds} command
   */
  public ChassisSpeeds calculateSpeeds(Pose2d currentPose, Pose2d targetPose) {
    double xVelocity = xController.calculate(currentPose.getX(), targetPose.getX());
    double yVelocity = yController.calculate(currentPose.getY(), targetPose.getY());
    double omegaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xVelocity, yVelocity, omegaVelocity, currentPose.getRotation());
  }

  /**
   * Calculates angular velocity required to rotate from the current rotation toward the target
   * rotation.
   *
   * @param current current robot rotation
   * @param target desired target rotation
   * @return angular velocity command in radians per second
   */
  public double calculateOmega(Rotation2d current, Rotation2d target) {
    return thetaController.calculate(current.getRadians(), target.getRadians());
  }

  /**
   * @return {@code true} if both translation and rotation controllers are at their goals
   */
  public boolean atGoal() {
    return atTranslationGoal() && atRotationGoal();
  }

  /**
   * @return {@code true} if both X and Y translation controllers are at their goals
   */
  public boolean atTranslationGoal() {
    return xController.atGoal() && yController.atGoal();
  }

  /**
   * @return {@code true} if the rotation controller is at its goal
   */
  public boolean atRotationGoal() {
    return thetaController.atGoal();
  }

  /**
   * Updates PID gains for translation controller (X).
   *
   * <p>Intended to be called by an external tuning mechanism.
   */
  void setTranslationXPID(PIDGains gains) {
    xController.setPID(gains.kP(), gains.kI(), gains.kD());
  }

  /**
   * Updates PID gains for translation controller (Y).
   *
   * <p>Intended to be called by an external tuning mechanism.
   */
  void setTranslationYPID(PIDGains gains) {
    yController.setPID(gains.kP(), gains.kI(), gains.kD());
  }

  /**
   * Updates PID gains for the rotation controller.
   *
   * <p>Intended to be called by an external tuning mechanism.
   */
  public void setRotationPID(PIDGains gains) {
    thetaController.setPID(gains.kP(), gains.kI(), gains.kD());
  }
}
