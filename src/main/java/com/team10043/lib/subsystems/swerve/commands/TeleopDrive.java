package com.team10043.lib.subsystems.swerve.commands;

import com.team10043.frc2026.subsystems.drive.DriveConstants;
import com.team10043.lib.subsystems.swerve.Swerve;
import com.team10043.lib.subsystems.swerve.control.util.ChassisSpeedsFactory;
import com.team10043.lib.subsystems.swerve.teleop.TeleopSwerveInputAdapter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Command factories for teleoperated swerve drive control.
 *
 * <p>Provides field-relative driving with joystick input processing including deadbands, rate
 * limiting, and coordinate frame transformations.
 */
public class TeleopDrive {

  private static final ChassisSpeedsFactory kinematics =
      new ChassisSpeedsFactory(
          DriveConstants.config.derivedLimits().maxLinearSpeed(),
          DriveConstants.config.derivedLimits().maxAngularSpeed());

  private static final TeleopSwerveInputAdapter inputAdapter =
      new TeleopSwerveInputAdapter(kinematics, 0.1);

  private TeleopDrive() {}

  /**
   * Creates field-relative joystick drive command with closed-loop velocity control.
   *
   * @param drive swerve subsystem
   * @param xSupplier forward/backward input (-1 to 1)
   * @param ySupplier left/right input (-1 to 1)
   * @param omegaSupplier rotation input (-1 to 1)
   * @return command for velocity-controlled driving
   */
  public static Command joystickDrive(
      Swerve drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {

    return Commands.run(
        () -> {
          ChassisSpeeds speeds =
              inputAdapter.calculateClosedLoop(
                  xSupplier,
                  ySupplier,
                  omegaSupplier,
                  drive.poseEstimator().getEstimatedRotation());
          drive.runVelocity(speeds);
        },
        drive);
  }

  /**
   * Creates field-relative joystick drive command with open-loop control.
   *
   * <p>Inputs are normalized in [-1, 1] and converted to field-relative speeds via {@link
   * TeleopSwerveInputAdapter#calculateOpenLoop}. {@link Swerve#runOpenLoop} then maps these to
   * motor commands: drive motors use voltage (±12 V typical) or FOC torque current depending on
   * configuration; turn motors hold steering via position control.
   *
   * @param drive swerve subsystem
   * @param xSupplier forward/backward input (-1 to 1)
   * @param ySupplier left/right input (-1 to 1)
   * @param omegaSupplier rotation input (-1 to 1)
   * @return command for open-loop driving
   */
  public static Command joystickDriveOpenLoop(
      Swerve drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {

    return Commands.run(
        () -> {
          ChassisSpeeds speeds =
              inputAdapter.calculateOpenLoop(
                  xSupplier,
                  ySupplier,
                  omegaSupplier,
                  drive.poseEstimator().getEstimatedRotation());
          drive.runOpenLoop(speeds);
        },
        drive);
  }

  /**
   * Creates field-relative drive command with PID-locked absolute rotation.
   *
   * <p>Driver controls translation while robot automatically maintains specified heading. Useful
   * for aiming at targets while maneuvering.
   *
   * @param drive swerve subsystem
   * @param xSupplier forward/backward input (-1 to 1)
   * @param ySupplier left/right input (-1 to 1)
   * @param targetRotation target heading to maintain
   * @return command for driving with locked rotation
   */
  public static Command joystickDriveAtAngle(
      Swerve drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> targetRotation) {

    ChassisSpeedsFactory kinematicsLocal =
        new ChassisSpeedsFactory(
            drive.config().derivedLimits().maxLinearSpeed(),
            drive.config().derivedLimits().maxAngularSpeed());

    TeleopSwerveInputAdapter inputAdapterLocal = new TeleopSwerveInputAdapter(kinematicsLocal, 0.1);

    return Commands.run(
            () -> {
              Translation2d linear = inputAdapterLocal.getLinearVelocity(xSupplier, ySupplier);
              double omega =
                  drive
                      .controllers()
                      .calculateOmega(
                          drive.poseEstimator().getEstimatedRotation(), targetRotation.get());

              ChassisSpeeds robotRelative =
                  kinematicsLocal.createChassisSpeeds(
                      linear, omega / drive.config().derivedLimits().maxAngularSpeed());
              ChassisSpeeds speeds =
                  kinematicsLocal.toFieldRelative(
                      robotRelative, drive.poseEstimator().getEstimatedRotation());

              drive.runVelocity(speeds);
            },
            drive)
        .beforeStarting(
            () -> drive.controllers().resetTheta(drive.poseEstimator().getEstimatedRotation()));
  }

  /**
   * Creates a simple robot-relative drive command with no heading control.
   *
   * <p>All three velocity components are expressed in the robot's own coordinate frame and passed
   * directly to {@link Swerve#runVelocity} without any field-relative conversion or heading PID.
   * Callers are responsible for supplying all three velocity components.
   *
   * @param drive swerve subsystem
   * @param forwardMetersPerSec forward (+) / backward (−) velocity in m/s, robot-relative
   * @param strafeMetersPerSec left (+) / right (−) velocity in m/s, robot-relative
   * @param omegaRadiansPerSec angular velocity in rad/s, counter-clockwise positive
   * @return command for pure robot-relative velocity control
   */
  public static Command driveRobotRelative(
      Swerve drive,
      DoubleSupplier forwardMetersPerSec,
      DoubleSupplier strafeMetersPerSec,
      DoubleSupplier omegaRadiansPerSec) {

    return Commands.run(
        () ->
            drive.runVelocity(
                new ChassisSpeeds(
                    forwardMetersPerSec.getAsDouble(),
                    strafeMetersPerSec.getAsDouble(),
                    omegaRadiansPerSec.getAsDouble())),
        drive);
  }

  /**
   * Creates a supplier that generates a sinusoidal strafe velocity for lateral oscillation.
   *
   * @param amplitudeMetersPerSec peak strafe speed in m/s
   * @param frequencyHz oscillation frequency in Hz
   * @param timer timer tracking elapsed time since oscillation started
   * @return supplier that returns the current strafe velocity
   */
  public static DoubleSupplier oscillationStrafeSupplier(
      double amplitudeMetersPerSec, double frequencyHz, Timer timer) {
    return () -> amplitudeMetersPerSec * Math.sin(2.0 * Math.PI * frequencyHz * timer.get());
  }

  public static DoubleSupplier oscillationStrafeSupplier(
      DoubleSupplier amplitudeMetersPerSec, DoubleSupplier frequencyHz, Timer timer) {
    return () ->
        amplitudeMetersPerSec.getAsDouble()
            * Math.sin(2.0 * Math.PI * frequencyHz.getAsDouble() * timer.get());
  }

  /**
   * Creates a command that oscillates the robot laterally using a sine-wave strafe pattern.
   *
   * <p>The robot moves side-to-side relative to its current heading with zero forward or rotational
   * velocity. The motion follows a sinusoidal pattern:
   *
   * <pre>  strafeVelocity = amplitudeMetersPerSec * sin(2π * frequencyHz * t)</pre>
   *
   * <p>Useful for evasive movement, testing oscillation behavior, or as a building block for more
   * complex movement patterns.
   *
   * @param drive the swerve drivetrain
   * @param amplitudeMetersPerSec peak strafe speed in m/s
   * @param frequencyHz oscillation frequency in Hz
   * @return a command that oscillates the robot laterally
   */
  public static Command oscillateRobotLaterally(
      Swerve drive, double amplitudeMetersPerSec, double frequencyHz) {
    Timer oscillationTimer = new Timer();
    DoubleSupplier strafeSupplier =
        oscillationStrafeSupplier(amplitudeMetersPerSec, frequencyHz, oscillationTimer);

    return Commands.sequence(
            Commands.runOnce(oscillationTimer::restart),
            driveRobotRelative(drive, () -> 0.0, strafeSupplier, () -> 0.0))
        .finallyDo(interrupted -> oscillationTimer.stop())
        .withName("TeleopDrive oscillateRobotLaterally");
  }
}
