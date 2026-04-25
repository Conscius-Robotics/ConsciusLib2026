package com.team10043.lib.subsystems.swerve.control.autonomous;

import com.team10043.lib.subsystems.swerve.control.feedback.SwerveFeedbackController;
import com.team10043.lib.subsystems.swerve.control.util.SwerveTargeting;
import com.team10043.lib.util.control.LoggedTunableNumber;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Builder;
import org.littletonrobotics.junction.Logger;

/**
 * Controls robot heading to face a target pose while driver controls translation. Implements
 * smoothing, deadband, and speed-based omega scaling to prevent jitter and module flipping.
 */
public class AutoRotateController {

  @Builder
  public record AutoRotateConfig(
      double maxLinearSpeed,
      double maxAngularSpeed,
      double angleDeadbandDegrees,
      double angleSmoothingFactor,
      double omegaScaleLowSpeed,
      double omegaScaleHighSpeed,
      double lowSpeedThreshold,
      double highSpeedThreshold) {}

  private final SwerveFeedbackController controllers;
  private final double maxLinearSpeed;
  private final double maxAngularSpeed;

  // Tuning parameters
  private final LoggedTunableNumber angleDeadbandDegrees;
  private final LoggedTunableNumber angleSmoothingFactor;
  private final LoggedTunableNumber omegaScaleLowSpeed;
  private final LoggedTunableNumber omegaScaleHighSpeed;
  private final LoggedTunableNumber lowSpeedThreshold;
  private final LoggedTunableNumber highSpeedThreshold;

  // Smoothed target angle (mutable state)
  private Rotation2d smoothedAngle;

  public AutoRotateController(SwerveFeedbackController controllers, AutoRotateConfig config) {
    this.controllers = controllers;
    this.maxLinearSpeed = config.maxLinearSpeed();
    this.maxAngularSpeed = config.maxAngularSpeed();

    // Initialize tunable parameters
    this.angleDeadbandDegrees =
        new LoggedTunableNumber(
            "Swerve/AutoRotate/AngleDeadbandDeg", config.angleDeadbandDegrees());
    this.angleSmoothingFactor =
        new LoggedTunableNumber(
            "Swerve/AutoRotate/AngleSmoothingFactor", config.angleSmoothingFactor());
    this.omegaScaleLowSpeed =
        new LoggedTunableNumber(
            "Swerve/AutoRotate/OmegaScaleLowSpeed", config.omegaScaleLowSpeed());
    this.omegaScaleHighSpeed =
        new LoggedTunableNumber(
            "Swerve/AutoRotate/OmegaScaleHighSpeed", config.omegaScaleHighSpeed());
    this.lowSpeedThreshold =
        new LoggedTunableNumber("Swerve/AutoRotate/LowSpeedThreshold", config.lowSpeedThreshold());
    this.highSpeedThreshold =
        new LoggedTunableNumber(
            "Swerve/AutoRotate/HighSpeedThreshold", config.highSpeedThreshold());
  }

  /**
   * Resets the controller state. Call before starting auto-rotate control.
   *
   * @param currentRotation Current robot rotation
   */
  public void reset(Rotation2d currentRotation) {
    controllers.resetTheta(currentRotation);
    smoothedAngle = null;
  }

  /**
   * Calculates normalized omega output (0.0 to 1.0) to rotate toward target pose.
   *
   * @param currentPose Current robot pose
   * @param currentRotation Current robot rotation
   * @param targetPose Target pose to aim at
   * @param translationSpeed Current translation speed (m/s)
   * @return Normalized omega output for rotation control
   */
  public double calculateNormalizedOmega(
      Pose2d currentPose, Rotation2d currentRotation, Pose2d targetPose, double translationSpeed) {

    // Calculate raw desired angle to target
    Rotation2d rawDesiredAngle = SwerveTargeting.getAngleToTarget(currentPose, targetPose);

    // Apply low-pass filter to smooth angle changes
    if (smoothedAngle == null) {
      smoothedAngle = rawDesiredAngle;
    } else {
      smoothedAngle = smoothedAngle.interpolate(rawDesiredAngle, angleSmoothingFactor.get());
    }

    // Calculate angular error
    double angleError = smoothedAngle.minus(currentRotation).getRadians();
    boolean inDeadband = Math.abs(Math.toDegrees(angleError)) <= angleDeadbandDegrees.get();

    double omega = 0.0;
    double omegaScale = 0.0;

    // Apply angular deadband
    if (!inDeadband) {
      double omegaBeforeScale = controllers.calculateOmega(currentRotation, smoothedAngle);

      // Calculate omega scale based on translation speed
      if (translationSpeed < lowSpeedThreshold.get()) {
        omegaScale = omegaScaleLowSpeed.get();
      } else if (translationSpeed > highSpeedThreshold.get()) {
        omegaScale = omegaScaleHighSpeed.get();
      } else {
        // Linear interpolation between low and high speed
        double t =
            (translationSpeed - lowSpeedThreshold.get())
                / (highSpeedThreshold.get() - lowSpeedThreshold.get());
        omegaScale = omegaScaleLowSpeed.get() * (1 - t) + omegaScaleHighSpeed.get() * t;
      }

      omega = omegaBeforeScale * omegaScale;

      Logger.recordOutput("AutoRotate/OmegaBeforeScale", omegaBeforeScale);
      Logger.recordOutput("AutoRotate/OmegaAfterScale", omega);
    }

    // Log tuning-relevant values
    Logger.recordOutput("AutoRotate/RawAngleDeg", rawDesiredAngle.getDegrees());
    Logger.recordOutput("AutoRotate/SmoothedAngleDeg", smoothedAngle.getDegrees());
    Logger.recordOutput("AutoRotate/CurrentAngleDeg", currentRotation.getDegrees());
    Logger.recordOutput("AutoRotate/AngleErrorDeg", Math.toDegrees(angleError));
    Logger.recordOutput("AutoRotate/InDeadband", inDeadband);
    Logger.recordOutput("AutoRotate/TranslationSpeed", translationSpeed);
    Logger.recordOutput("AutoRotate/OmegaScale", omegaScale);
    Logger.recordOutput("AutoRotate/OmegaOutput", omega);
    Logger.recordOutput("AutoRotate/TargetPose", targetPose);

    return omega / maxAngularSpeed;
  }

  public double maxLinearSpeed() {
    return maxLinearSpeed;
  }
}
