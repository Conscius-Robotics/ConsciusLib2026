package com.team10043.lib.subsystems.swerve.commands;

import com.team10043.lib.subsystems.swerve.Swerve;
import com.team10043.lib.subsystems.swerve.control.autonomous.AutopilotSwerveController;
import com.team10043.lib.subsystems.swerve.control.autonomous.DriveToPoseController;
import com.team10043.lib.subsystems.swerve.telemetry.SwerveCommandLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Command factories for autonomous drive-to-pose navigation.
 *
 * <p>Provides two controller options:
 *
 * <ul>
 *   <li>{@link #driveToPosePID} - Uses WPILib ProfiledPIDController for all axes
 *   <li>{@link #driveToPoseAutoPilot} - Uses Autopilot library for translation control
 * </ul>
 */
public final class DriveToPose {

  private DriveToPose() {}

  /**
   * Internal helper to create drive-to-pose commands with common structure.
   *
   * @param drive swerve subsystem
   * @param targetPose target pose supplier
   * @param mode controller type for logging
   * @param resetAction controller reset logic
   * @param speedsSupplier chassis speeds calculation
   * @param atGoalSupplier goal detection predicate
   * @return command that drives to target pose
   */
  private static Command runDriveToPose(
      Swerve drive,
      Supplier<Pose2d> targetPose,
      String mode,
      Runnable resetAction,
      Supplier<ChassisSpeeds> speedsSupplier,
      BooleanSupplier atGoalSupplier) {

    double[] startTime = new double[1];

    return Commands.run(
            () -> {
              Pose2d currentPose = drive.poseEstimator().getEstimatedPose();
              drive.runVelocity(speedsSupplier.get());
              SwerveCommandLogger.logDriveToPoseExecute(currentPose, targetPose.get());
            },
            drive)
        .beforeStarting(
            () -> {
              startTime[0] = Timer.getFPGATimestamp();
              resetAction.run();
              SwerveCommandLogger.logDriveToPoseStart(targetPose.get(), mode);
            })
        .until(atGoalSupplier)
        .finallyDo(
            () -> {
              drive.stop();
              SwerveCommandLogger.logDriveToPoseEnd(Timer.getFPGATimestamp() - startTime[0]);
            });
  }

  /**
   * Creates command to drive to a target pose using ProfiledPIDController.
   *
   * @param drive swerve subsystem
   * @param targetPose target pose supplier
   * @return command that drives to target using PID control
   */
  public static Command driveToPosePID(Swerve drive, Supplier<Pose2d> targetPose) {
    DriveToPoseController controller = drive.driveToPoseController();

    return runDriveToPose(
        drive,
        targetPose,
        "PID",
        () -> controller.reset(drive.poseEstimator().getEstimatedPose()),
        () -> controller.calculate(drive.poseEstimator().getEstimatedPose(), targetPose.get()),
        controller::atGoal);
  }

  /**
   * Creates command to drive to a target pose using Autopilot library.
   *
   * <p>Autopilot provides advanced trajectory generation with velocity profiling and obstacle
   * avoidance capabilities.
   *
   * @param drive swerve subsystem
   * @param targetPose target pose supplier
   * @param entryAngle optional desired approach angle at target
   * @return command that drives to target using Autopilot control
   */
  public static Command driveToPoseAutoPilot(
      Swerve drive, Supplier<Pose2d> targetPose, Optional<Rotation2d> entryAngle) {

    AutopilotSwerveController controller = drive.autopilotController();

    return runDriveToPose(
        drive,
        targetPose,
        "Autopilot",
        () -> controller.reset(drive.poseEstimator().getEstimatedRotation()),
        () ->
            controller.calculate(
                drive.poseEstimator().getEstimatedPose(),
                drive.getChassisSpeeds(),
                targetPose.get(),
                entryAngle),
        () ->
            controller.atTarget(
                drive.poseEstimator().getEstimatedPose(), targetPose.get(), entryAngle));
  }

  /**
   * Creates command to drive to a target pose using Autopilot without entry angle constraint.
   *
   * @param drive swerve subsystem
   * @param targetPose target pose supplier
   * @return command that drives to target using Autopilot control
   */
  public static Command driveToPoseAutoPilot(Swerve drive, Supplier<Pose2d> targetPose) {
    return driveToPoseAutoPilot(drive, targetPose, Optional.empty());
  }
}
