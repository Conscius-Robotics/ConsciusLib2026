package com.team10043.lib.subsystems.swerve.commands;

import com.team10043.lib.subsystems.swerve.Swerve;
import com.team10043.lib.subsystems.swerve.control.autonomous.AutoRotateController;
import com.team10043.lib.subsystems.swerve.control.util.ChassisSpeedsFactory;
import com.team10043.lib.subsystems.swerve.telemetry.SwerveCommandLogger;
import com.team10043.lib.subsystems.swerve.teleop.TeleopSwerveInputAdapter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Command factory for automatic rotation alignment during teleoperation.
 *
 * <p>Allows driver to control translation while robot automatically rotates to face a target.
 * Implements speed-dependent omega scaling to prevent module flipping at high speeds.
 */
public class AutoAlign {

  private AutoAlign() {}

  /**
   * Creates field-relative drive command with automatic rotation towards target.
   *
   * <p>Features smooth rotation control with:
   *
   * <ul>
   *   <li>Low-pass filtering to reduce jitter
   *   <li>Speed-based omega scaling (reduced correction at high speeds)
   *   <li>Angular deadband to prevent micro-corrections
   * </ul>
   *
   * @param drive swerve subsystem
   * @param xSupplier forward/backward input (-1 to 1)
   * @param ySupplier left/right input (-1 to 1)
   * @param targetPoseSupplier target position to aim at
   * @return command for driving with auto-rotation
   * @throws IllegalStateException if AutoRotateConfig not configured in SwerveConfig
   */
  public static Command autoRotateDrive(
      Swerve drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Pose2d> targetPoseSupplier) {

    if (drive.config().autoRotateConfig() == null) {
      throw new IllegalStateException(
          "AutoRotateConfig must be provided in SwerveConfig to use AutoAlign commands. "
              + "Add .autoRotateConfig(...) to your SwerveConfig builder.");
    }

    AutoRotateController controller =
        new AutoRotateController(drive.controllers(), drive.config().autoRotateConfig());

    ChassisSpeedsFactory kinematics =
        new ChassisSpeedsFactory(
            drive.config().derivedLimits().maxLinearSpeed(),
            drive.config().derivedLimits().maxAngularSpeed());

    TeleopSwerveInputAdapter inputAdapter = new TeleopSwerveInputAdapter(kinematics, 0.1);

    return Commands.run(
            () -> {
              Pose2d currentPose = drive.poseEstimator().getEstimatedPose();
              Rotation2d currentRotation = drive.poseEstimator().getEstimatedRotation();
              Pose2d targetPose = targetPoseSupplier.get();

              Translation2d linear = inputAdapter.getLinearVelocity(xSupplier, ySupplier);
              double translationSpeed = linear.getNorm() * controller.maxLinearSpeed();

              double omegaNormalized =
                  controller.calculateNormalizedOmega(
                      currentPose, currentRotation, targetPose, translationSpeed);

              ChassisSpeeds robotRelative = kinematics.createChassisSpeeds(linear, omegaNormalized);
              ChassisSpeeds speeds = kinematics.toFieldRelative(robotRelative, currentRotation);

              drive.runVelocity(speeds);
            },
            drive)
        .beforeStarting(
            () -> {
              controller.reset(drive.poseEstimator().getEstimatedRotation());
              SwerveCommandLogger.logAutoRotateStart();
            })
        .finallyDo(() -> SwerveCommandLogger.logAutoRotateEnd());
  }
}
