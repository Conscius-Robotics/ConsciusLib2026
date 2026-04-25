package com.team10043.lib.subsystems.swerve.control.autonomous;

import com.team10043.lib.subsystems.swerve.control.feedback.SwerveFeedbackController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Wraps PID-based pose controllers for autonomous drive-to-pose commands. Provides simple interface
 * for resetting, calculating speeds, and checking goal achievement.
 */
public class DriveToPoseController {

  private final SwerveFeedbackController controllers;

  public DriveToPoseController(SwerveFeedbackController controllers) {
    this.controllers = controllers;
  }

  /**
   * Resets all controllers to current pose. Call before starting drive-to-pose.
   *
   * @param currentPose Current robot pose
   */
  public void reset(Pose2d currentPose) {
    controllers.reset(currentPose);
  }

  /**
   * Calculates chassis speeds to reach target pose.
   *
   * @param currentPose Current robot pose
   * @param targetPose Target pose
   * @return Chassis speeds for velocity control
   */
  public ChassisSpeeds calculate(Pose2d currentPose, Pose2d targetPose) {
    return controllers.calculateSpeeds(currentPose, targetPose);
  }

  /**
   * Checks if robot is at goal within tolerance.
   *
   * @return true if at goal
   */
  public boolean atGoal() {
    return controllers.atGoal();
  }
}
