package com.team10043.lib.subsystems.swerve.control.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Helper class for calculating angles and errors relative to target poses. Useful for auto-aiming
 * and autonomous driving.
 */
public class SwerveTargeting {

  /**
   * Calculates the angle from the current pose to the target pose.
   *
   * @param currentPose Current robot pose
   * @param targetPose Target pose
   * @return Rotation2d representing the angle to face the target
   */
  public static Rotation2d getAngleToTarget(Pose2d currentPose, Pose2d targetPose) {
    Translation2d vectorToTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
    return vectorToTarget.getAngle();
  }

  /**
   * Calculates the error between current and target poses.
   *
   * @param currentPose Current robot pose
   * @param targetPose Target pose
   * @return Transform2d representing the error in position and rotation
   */
  public static Transform2d getError(Pose2d currentPose, Pose2d targetPose) {
    return targetPose.minus(currentPose);
  }

  /**
   * Calculates the distance between two poses.
   *
   * @param pose1 First pose
   * @param pose2 Second pose
   * @return Distance in meters
   */
  public static double getDistance(Pose2d pose1, Pose2d pose2) {
    return pose1.getTranslation().getDistance(pose2.getTranslation());
  }

  /**
   * Checks if the robot is within tolerance of the target pose.
   *
   * @param currentPose Current robot pose
   * @param targetPose Target pose
   * @param positionTolerance Position tolerance in meters
   * @param angleTolerance Angle tolerance in radians
   * @return true if within tolerance
   */
  public static boolean isAtTarget(
      Pose2d currentPose, Pose2d targetPose, double positionTolerance, double angleTolerance) {
    Transform2d error = getError(currentPose, targetPose);

    double positionError = Math.hypot(error.getX(), error.getY());
    double angleError = Math.abs(error.getRotation().getRadians());

    return positionError <= positionTolerance && angleError <= angleTolerance;
  }
}
