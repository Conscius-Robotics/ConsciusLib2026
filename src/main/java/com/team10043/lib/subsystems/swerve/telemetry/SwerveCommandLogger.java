package com.team10043.lib.subsystems.swerve.telemetry;

import com.team10043.lib.subsystems.swerve.control.util.SwerveTargeting;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import org.littletonrobotics.junction.Logger;

/**
 * Centralized logging utilities for swerve drive commands. Provides static methods for logging
 * command lifecycle events (start, execute, end) and state information to AdvantageKit Logger.
 *
 * <p>Keeps command code clean by extracting repetitive logging logic into reusable static methods.
 */
public class SwerveCommandLogger {

  private SwerveCommandLogger() {}

  /**
   * Logs the start of a drive-to-pose command.
   *
   * @param targetPose Target pose
   * @param mode Control mode description (e.g., "PID", "Autopilot")
   */
  public static void logDriveToPoseStart(Pose2d targetPose, String mode) {
    Logger.recordOutput("DriveToPose/Mode", mode);
    Logger.recordOutput("DriveToPose/Target", targetPose);
    Logger.recordOutput("DriveToPose/AtGoal", false);
  }

  /**
   * Logs ongoing drive-to-pose execution.
   *
   * @param currentPose Current robot pose
   * @param targetPose Target pose
   */
  public static void logDriveToPoseExecute(Pose2d currentPose, Pose2d targetPose) {
    Transform2d error = SwerveTargeting.getError(currentPose, targetPose);

    Logger.recordOutput("DriveToPose/CurrentPose", currentPose);
    Logger.recordOutput("DriveToPose/ErrorX", error.getX());
    Logger.recordOutput("DriveToPose/ErrorY", error.getY());
    Logger.recordOutput("DriveToPose/ErrorTheta", error.getRotation().getRadians());
  }

  /**
   * Logs the completion of a drive-to-pose command.
   *
   * @param durationSec Duration of the command in seconds
   */
  public static void logDriveToPoseEnd(double durationSec) {
    Logger.recordOutput("DriveToPose/AtGoal", true);
    Logger.recordOutput("DriveToPose/DurationSec", durationSec);
  }

  /** Logs the start of auto-rotate control. */
  public static void logAutoRotateStart() {
    Logger.recordOutput("AutoRotate/Active", true);
  }

  /** Logs the end of auto-rotate control. */
  public static void logAutoRotateEnd() {
    Logger.recordOutput("AutoRotate/Active", false);
  }
}
