package com.team10043.lib.subsystems.swerve;

import com.team10043.frc2026.Constants;
import com.team10043.frc2026.Constants.Mode;
import com.team10043.lib.gyros.GyroIO;
import com.team10043.lib.gyros.GyroIOInputsAutoLogged;
import com.team10043.lib.subsystems.swerve.config.SwerveConfig;
import com.team10043.lib.subsystems.swerve.control.autonomous.AutopilotSwerveController;
import com.team10043.lib.subsystems.swerve.control.autonomous.DriveToPoseController;
import com.team10043.lib.subsystems.swerve.control.feedback.TunableSwerveFeedbackController;
import com.team10043.lib.subsystems.swerve.estimator.SwervePoseEstimator;
import com.team10043.lib.subsystems.swerve.module.ModuleArray;
import com.team10043.lib.subsystems.swerve.module.ModuleIO;
import com.team10043.lib.subsystems.swerve.path.SwervePathPlanner;
import com.team10043.lib.subsystems.swerve.telemetry.SwerveLogger;
import com.team10043.lib.util.DebouncedAlert;
import com.team10043.lib.util.phoenix6.PhoenixSignalThread;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Builder;
import lombok.Getter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.Logger;

/**
 * Swerve drive subsystem providing holonomic robot control with independent module steering and
 * drive.
 *
 * <p>This subsystem manages four swerve modules, gyroscope, pose estimation, and path following.
 * Uses hardware abstraction for testability and AdvantageKit logging for replay diagnostics.
 *
 * <p>Key architectural features:
 *
 * <ul>
 *   <li>Hardware abstraction through IO interfaces for simulation and replay
 *   <li>Modular component design with clean separation of concerns
 *   <li>Thread-safe odometry updates synchronized with Phoenix CAN signals
 *   <li>Optional components (odometry, path planning) configured at construction
 * </ul>
 */
@Getter
@Accessors(fluent = true)
public class Swerve extends SubsystemBase {
  private static final double CONTROL_LOOP_PERIOD_SECONDS = 0.02;

  private final SwerveConfig<?> config;

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final ModuleArray modules;
  private final SwerveDriveKinematics kinematics;
  private final SwerveLogger logger;
  private final TunableSwerveFeedbackController controllers;

  private SwervePathPlanner pathPlanner;
  private SwervePoseEstimator poseEstimator;
  private DriveToPoseController driveToPoseController;
  private AutopilotSwerveController autopilotController;

  private final DebouncedAlert gyroDisconnectedAlert;

  /**
   * Constructs a swerve drive subsystem.
   *
   * @param config swerve configuration defining hardware, kinematics, and control parameters
   * @param gyroIO gyroscope hardware interface
   * @param flModuleIO front-left module hardware interface
   * @param frModuleIO front-right module hardware interface
   * @param blModuleIO back-left module hardware interface
   * @param brModuleIO back-right module hardware interface
   * @param enableOdometry whether to enable pose estimation
   * @param enablePathPlanner whether to configure PathPlanner integration
   */
  @Builder
  public Swerve(
      SwerveConfig<?> config,
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      boolean enableOdometry,
      boolean enablePathPlanner) {

    this.config = config;

    this.gyroIO = gyroIO;

    this.kinematics = new SwerveDriveKinematics(config.chassisDimensions().moduleTranslations());

    this.modules = new ModuleArray(config, flModuleIO, frModuleIO, blModuleIO, brModuleIO);
    this.logger = new SwerveLogger(kinematics);

    ProfiledPIDController xController = config.translationPID().toProfiledPIDController();
    ProfiledPIDController yController = config.translationPID().toProfiledPIDController();
    ProfiledPIDController thetaController = config.rotationPID().toProfiledPIDController();
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    this.controllers =
        new TunableSwerveFeedbackController(
            "Swerve/Controllers", xController, yController, thetaController);

    this.driveToPoseController = new DriveToPoseController(controllers);

    if (config.autopilotConfig() != null) {
      this.autopilotController =
          new AutopilotSwerveController(controllers, config.autopilotConfig());
    }

    if (enableOdometry) {
      this.poseEstimator =
          new SwervePoseEstimator(
              kinematics,
              modules::getOdometryTimestamps,
              modules::getOdometryPositionSamples,
              modules::getPositions);
    }

    if (enablePathPlanner) {
      pathPlanner = new SwervePathPlanner(this, config);
      pathPlanner.configure();
    }

    gyroDisconnectedAlert =
        new DebouncedAlert(
            () -> gyroInputs.data.connected(),
            () -> Constants.CURRENT_MODE != Mode.SIM,
            "Disconnected gyro, using kinematics as fallback.");

    PhoenixSignalThread.getInstance().start();

    poseEstimator.resetYaw();
  }

  /** Updates inputs, manages disabled state, updates pose estimation, and logs telemetry. */
  @Override
  public void periodic() {
    updateInputs();

    if (DriverStation.isDisabled()) {
      handleDisabledState();
    }

    if (poseEstimator != null) {
      poseEstimator.update();
      logger.logPose(poseEstimator.getEstimatedPose());
    }

    controllers.updateControllers();

    updateAlerts();

    logCurrentState();
  }

  /**
   * Updates all hardware inputs with thread synchronization.
   *
   * <p>Locks odometry thread to ensure consistent sensor readings across gyro and modules.
   */
  private void updateInputs() {
    PhoenixSignalThread.samplingDataLock.lock();
    try {
      gyroIO.updateInputs(gyroInputs);
      Logger.processInputs("Swerve/Gyro", gyroInputs);

      if (gyroInputs.data.connected() && poseEstimator != null) {
        poseEstimator.updateGyroRotations(gyroInputs.odometryYawPositions);
      }

      modules.periodic();
    } finally {
      PhoenixSignalThread.samplingDataLock.unlock();
    }
  }

  /** Stops all modules and logs disabled state. */
  private void handleDisabledState() {
    modules.stop();
    logger.logControlMode("Disabled");
    logger.logEmptySetpoints();
  }

  /** Checks gyro connection and updates alert state. */
  private void updateAlerts() {
    gyroDisconnectedAlert.update();

    logger.logGyroStatus(gyroInputs.data.connected());
  }

  /** Logs current module states to AdvantageKit. */
  private void logCurrentState() {
    SwerveModuleState[] states = modules.getStates();
    logger.logMeasuredStates(states);
  }

  /**
   * Returns the current chassis speeds.
   *
   * @return robot-relative velocity in m/s and rad/s
   */
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(modules.getStates());
  }

  /**
   * Drives the robot using closed-loop velocity control.
   *
   * <p>Discretizes chassis speeds to account for loop time, converts to module states, and applies
   * wheel speed desaturation.
   *
   * @param speeds desired robot-relative velocities
   */
  public void runVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, CONTROL_LOOP_PERIOD_SECONDS);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, config.derivedLimits().maxLinearSpeed());

    SwerveModuleState[] unoptimizedStates = copyStates(setpointStates);

    modules.runSetpoints(setpointStates);

    logger.logControlMode("Closed Loop (Velocity)");
    logger.logSetpoints(unoptimizedStates, setpointStates, discreteSpeeds);
  }

  /**
   * Drives using open-loop control.
   *
   * <p>Receives field-relative chassis speeds normalized in [-1, 1]. Drive motors map these
   * normalized commands to either voltage (±12 V typical) or FOC torque current based on the
   * configured motor IO; turn motors hold steering via position control. Inputs are generated by
   * {@link com.team10043.lib.subsystems.swerve.teleop.TeleopSwerveInputAdapter#calculateOpenLoop}.
   *
   * @param speeds desired chassis speeds (normalized) where vx/vy are in [-1, 1]
   */
  public void runOpenLoop(ChassisSpeeds speeds) {
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);

    modules.runOpenLoopSetpoints(setpointStates);

    logger.logControlMode("Open Loop (Duty Cycle/FOC)");
    logger.logOpenLoopSetpoints(setpointStates);
  }

  /** Stops all module movement by commanding zero velocity. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Locks modules in X-formation to resist pushing.
   *
   * <p>Points each module at 45-degree angles to create maximum resistance to external forces.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = config.chassisDimensions().moduleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);

    modules.stop();
    logger.logControlMode("X-Formation");
  }

  /**
   * Runs characterization routine for feedforward or wheel radius tuning.
   *
   * @param output voltage / torque or duty cycle output to apply to drive motors
   */
  public void runDriveCharacterization(double output) {
    logger.logControlMode("Drive Characterization");
    modules.runDriveCharacterization(output);
  }

  /**
   * Runs turn motor characterization routine.
   *
   * @param output voltage / torque output to apply to turn motors
   */
  public void runTurnCharacterization(double output) {
    logger.logControlMode("Turn Characterization");
    modules.runTurnCharacterization(output);
  }

  /**
   * Returns wheel positions for wheel radius characterization.
   *
   * @return array of wheel positions in rotations
   */
  public double[] getWheelRadiusCharacterizationPositions() {
    return modules.getWheelRadiusCharacterizationPositions();
  }

  /**
   * Returns average drive velocity for feedforward characterization.
   *
   * @return average velocity across all modules in m/s
   */
  public double getFFCharacterizationVelocity() {
    return modules.getAverageFFCharacterizationVelocity();
  }

  /**
   * Returns average turn motor velocity for feedforward characterization.
   *
   * @return average turn velocity across all modules in rad/s
   */
  public double getTurnFFCharacterizationVelocity() {
    return modules.getAverageTurnVelocity();
  }

  /**
   * Returns average turn motor torque current for characterization.
   *
   * @return average turn torque current across all modules in amps
   */
  public double getAverageTurnTorqueCurrent() {
    return modules.getAverageTurnTorqueCurrent();
  }

  /**
   * Returns average drive motor torque current for characterization.
   *
   * @return average drive torque current across all modules in amps
   */
  public double getAverageDriveTorqueCurrent() {
    return modules.getAverageDriveTorqueCurrent();
  }

  /** Creates defensive copy of module states array. */
  private SwerveModuleState[] copyStates(SwerveModuleState[] states) {
    SwerveModuleState[] copy = new SwerveModuleState[states.length];
    for (int i = 0; i < states.length; i++) {
      copy[i] = new SwerveModuleState(states[i].speedMetersPerSecond, states[i].angle);
    }
    return copy;
  }

  /**
   * Returns the drive-to-pose controller for autonomous navigation.
   *
   * @return controller using ProfiledPIDController for all axes
   */
  public DriveToPoseController driveToPoseController() {
    return driveToPoseController;
  }

  /**
   * Returns the Autopilot-based controller for advanced path following.
   *
   * @return controller using Autopilot library for translation
   * @throws IllegalStateException if Autopilot was not configured in SwerveConfig
   */
  public AutopilotSwerveController autopilotController() {
    if (autopilotController == null) {
      throw new IllegalStateException(
          "AutopilotController not configured. Enable autopilotConfig in SwerveConfig.");
    }
    return autopilotController;
  }
}
