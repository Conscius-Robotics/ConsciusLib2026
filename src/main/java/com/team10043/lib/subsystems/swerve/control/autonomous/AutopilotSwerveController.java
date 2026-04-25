package com.team10043.lib.subsystems.swerve.control.autonomous;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.team10043.lib.subsystems.swerve.control.feedback.SwerveFeedbackController;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.Optional;
import lombok.Builder;

/**
 * Isolates Autopilot library usage for autonomous driving. Handles translation via Autopilot and
 * rotation via PID controller.
 */
public class AutopilotSwerveController {

  @Builder
  public record AutopilotSwerveConfig(
      double acceleration,
      double jerk,
      Distance toleranceXY,
      Angle toleranceTheta,
      Distance beelineRadius) {}

  private final Autopilot autopilot;
  private final SwerveFeedbackController controllers;

  public AutopilotSwerveController(
      SwerveFeedbackController controllers, AutopilotSwerveConfig config) {
    this.controllers = controllers;

    APConstraints constraints =
        new APConstraints().withAcceleration(config.acceleration()).withJerk(config.jerk());

    APProfile profile =
        new APProfile(constraints)
            .withErrorXY(config.toleranceXY())
            .withErrorTheta(config.toleranceTheta())
            .withBeelineRadius(config.beelineRadius());

    this.autopilot = new Autopilot(profile);
  }

  /**
   * Resets rotation controller. Call before starting autopilot drive.
   *
   * @param currentRotation Current robot rotation
   */
  public void reset(Rotation2d currentRotation) {
    controllers.resetTheta(currentRotation);
  }

  /**
   * Calculates chassis speeds using Autopilot for translation and PID for rotation.
   *
   * @param currentPose Current robot pose
   * @param currentSpeeds Current chassis speeds
   * @param targetPose Target pose
   * @param entryAngle Optional entry angle at target
   * @return Field-relative chassis speeds
   */
  public ChassisSpeeds calculate(
      Pose2d currentPose,
      ChassisSpeeds currentSpeeds,
      Pose2d targetPose,
      Optional<Rotation2d> entryAngle) {

    APTarget target = createTarget(targetPose, entryAngle);
    APResult output = autopilot.calculate(currentPose, currentSpeeds, target);

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        output.vx().in(MetersPerSecond),
        output.vy().in(MetersPerSecond),
        controllers.calculateOmega(currentPose.getRotation(), output.targetAngle()),
        currentPose.getRotation());
  }

  /**
   * Checks if robot is at target within Autopilot tolerance.
   *
   * @param currentPose Current robot pose
   * @param targetPose Target pose
   * @param entryAngle Optional entry angle at target
   * @return true if at target
   */
  public boolean atTarget(Pose2d currentPose, Pose2d targetPose, Optional<Rotation2d> entryAngle) {
    APTarget target = createTarget(targetPose, entryAngle);

    return autopilot.atTarget(currentPose, target);
  }

  private APTarget createTarget(Pose2d targetPose, Optional<Rotation2d> entryAngle) {
    APTarget target = new APTarget(targetPose);
    if (entryAngle.isPresent()) {
      target = target.withEntryAngle(entryAngle.get());
    }
    return target;
  }
}
