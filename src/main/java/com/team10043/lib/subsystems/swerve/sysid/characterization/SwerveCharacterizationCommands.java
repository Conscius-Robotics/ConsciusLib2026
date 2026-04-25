package com.team10043.lib.subsystems.swerve.sysid.characterization;

import com.team10043.lib.subsystems.swerve.Swerve;
import com.team10043.lib.subsystems.swerve.sysid.FeedforwardCharacterizationModel;
import com.team10043.lib.subsystems.swerve.sysid.WheelRadiusEstimator;
import com.team10043.lib.util.characterization.StaticFrictionCharacterizationModel;
import com.team10043.lib.util.control.TunableSysIdManager;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class SwerveCharacterizationCommands {

  private static final double START_DELAY = 2.0;

  private static final TunableSysIdManager driveStaticManager;
  private static final TunableSysIdManager steeringStaticManager;
  private static final TunableSysIdManager feedforwardManager;
  private static final TunableSysIdManager wheelRadiusManager;

  static {
    driveStaticManager =
        new TunableSysIdManager("SysId/Swerve/DriveStatic", 1.0, 30.0, 0.01, params -> {});

    steeringStaticManager =
        new TunableSysIdManager("SysId/Swerve/SteeringStatic", 1.0, 30.0, 0.01, params -> {});

    feedforwardManager =
        new TunableSysIdManager("SysId/Swerve/Feedforward", 0.1, 0.0, 0.0, params -> {});

    wheelRadiusManager =
        new TunableSysIdManager("SysId/Swerve/WheelRadius", 0.5, 2.0, 0.0, params -> {});
  }

  /**
   * Initializes tunable parameters for NetworkTables.
   *
   * <p>This method must be called during robot initialization to publish SysId parameters to
   * AdvantageScope. The static initializer runs when this method is invoked.
   */
  public static void initialize() {
    // Method intentionally empty - calling this triggers static initialization
  }

  private SwerveCharacterizationCommands() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Finds the static friction current (kS) for the drive motors.
   *
   * <p>Ramps current on all drive motors while holding turn motors at zero degrees. When the
   * average drive velocity exceeds the tolerance, the applied current is recorded as kS.
   *
   * <p><b>Parameters are tunable via NetworkTables at {@code
   * /Tuning/SysId/Swerve/DriveStatic/}.</b> Defaults are: rampRate=1.0 A/s, maxCurrent=30.0 A,
   * tolerance=0.01 m/s.
   *
   * @param drive the swerve subsystem
   * @return a command that runs the drive static test
   */
  public static Command driveStaticTest(Swerve drive) {

    Timer timer = new Timer();
    StaticFrictionCharacterizationModel model = new StaticFrictionCharacterizationModel();

    return Commands.runOnce(driveStaticManager::update)
        .andThen(
            Commands.run(
                () -> {
                  if (!timer.isRunning()) {
                    timer.restart();
                  }

                  var params = driveStaticManager.get();
                  double current = Math.min(timer.get() * params.rampRate(), params.maxCurrent());
                  drive.runDriveCharacterization(current);

                  if (Math.abs(drive.getFFCharacterizationVelocity()) > params.tolerance()
                      && !model.hasSamples()) {

                    model.addSample(current);
                    drive.runDriveCharacterization(0.0);
                    model.printResults("Drive", "A");
                  }
                },
                drive))
        .finallyDo(
            interrupted -> {
              drive.runDriveCharacterization(0.0);
              timer.stop();
              model.reset();
            })
        .until(model::hasSamples);
  }

  /**
   * Finds the static friction current (kS) for the steering (turn) motors.
   *
   * <p>Ramps current on all turn motors while holding drive motors at zero. When the average turn
   * velocity exceeds the tolerance, the applied current is recorded as kS.
   *
   * <p><b>Parameters are tunable via NetworkTables at {@code
   * /Tuning/SysId/Swerve/SteeringStatic/}.</b> Defaults are: rampRate=1.0 A/s, maxCurrent=30.0 A,
   * tolerance=0.01 rad/s.
   *
   * @param drive the swerve subsystem
   * @return a command that runs the steering static test
   */
  public static Command steeringStaticTest(Swerve drive) {

    Timer timer = new Timer();
    StaticFrictionCharacterizationModel model = new StaticFrictionCharacterizationModel();

    return Commands.runOnce(steeringStaticManager::update)
        .andThen(
            Commands.run(
                () -> {
                  if (!timer.isRunning()) {
                    timer.restart();
                  }

                  var params = steeringStaticManager.get();
                  double current = Math.min(timer.get() * params.rampRate(), params.maxCurrent());
                  drive.runTurnCharacterization(current);

                  if (Math.abs(drive.getTurnFFCharacterizationVelocity()) > params.tolerance()
                      && !model.hasSamples()) {

                    model.addSample(current);
                    drive.runTurnCharacterization(0.0);
                    model.printResults("Steering", "A");
                  }
                },
                drive))
        .finallyDo(
            interrupted -> {
              drive.runTurnCharacterization(0.0);
              timer.stop();
              model.reset();
            })
        .until(model::hasSamples);
  }

  /**
   * Performs feedforward characterization by ramping voltage.
   *
   * <p><b>Parameters are tunable via NetworkTables at {@code
   * /Tuning/SysId/Swerve/Feedforward/}.</b> Default rampRate is 0.1 V/s.
   *
   * @param drive the swerve subsystem
   * @return a command that runs feedforward characterization
   */
  public static Command feedforward(Swerve drive) {
    FeedforwardCharacterizationModel model = new FeedforwardCharacterizationModel();
    Timer timer = new Timer();

    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  feedforwardManager.update();
                  model.reset();
                }),
            Commands.run(() -> drive.runDriveCharacterization(0.0), drive).withTimeout(START_DELAY),
            Commands.runOnce(timer::restart),
            Commands.run(
                () -> {
                  var params = feedforwardManager.get();
                  double voltage = timer.get() * params.rampRate();
                  drive.runDriveCharacterization(voltage);
                  model.addSample(drive.getFFCharacterizationVelocity(), voltage);
                },
                drive))
        .finallyDo(
            interrupted -> {
              if (!interrupted) {
                model.printResults();
              }
            });
  }

  /**
   * Estimates the wheel radius by spinning the robot and comparing gyro to wheel odometry.
   *
   * <p><b>Parameters are tunable via NetworkTables at {@code
   * /Tuning/SysId/Swerve/WheelRadius/}.</b> Defaults are: rampRate=0.5 rad/s² (angular
   * acceleration), maxCurrent=2.0 rad/s (max angular velocity).
   *
   * <p><b>Note:</b> NetworkTables key is "MaxCurrent" for consistency, but the value represents
   * maximum angular velocity in rad/s.
   *
   * @param drive the swerve subsystem
   * @return a command that runs wheel radius characterization
   */
  public static Command wheelRadius(Swerve drive) {
    WheelRadiusEstimator estimator = new WheelRadiusEstimator();

    return Commands.runOnce(wheelRadiusManager::update)
        .andThen(
            Commands.defer(
                () -> {
                  var params = wheelRadiusManager.get();
                  // params.rampRate() = angular acceleration (rad/s²)
                  // params.maxCurrent() = max angular velocity (rad/s)
                  SlewRateLimiter limiter = new SlewRateLimiter(params.rampRate());

                  return Commands.parallel(
                      Commands.sequence(
                          Commands.runOnce(() -> limiter.reset(0.0)),
                          Commands.run(
                              () ->
                                  drive.runVelocity(
                                      new ChassisSpeeds(
                                          0.0, 0.0, limiter.calculate(params.maxCurrent()))),
                              drive)),
                      Commands.sequence(
                          Commands.waitSeconds(START_DELAY),
                          Commands.runOnce(
                              () ->
                                  estimator.start(
                                      drive.getWheelRadiusCharacterizationPositions(),
                                      drive.poseEstimator().getEstimatedRotation())),
                          Commands.run(
                              () ->
                                  estimator.update(drive.poseEstimator().getEstimatedRotation()))));
                },
                java.util.Set.of(drive)))
        .finallyDo(
            interrupted -> {
              if (!interrupted) {
                double driveBaseRadius = drive.config().chassisDimensions().driveBaseRadius();
                estimator.printResults(
                    drive.getWheelRadiusCharacterizationPositions(), driveBaseRadius);
              }
            });
  }
}
