package com.team10043.lib.subsystems.swerve.config;

import com.ctre.phoenix6.CANBus;
import com.team10043.lib.drivers.CANDeviceId;
import com.team10043.lib.gyros.config.GyroConfig;
import com.team10043.lib.gyros.config.GyroConfiguration;
import com.team10043.lib.subsystems.swerve.config.control.SwerveDerivedLimits;
import com.team10043.lib.subsystems.swerve.config.control.SwerveMotionConstraints;
import com.team10043.lib.subsystems.swerve.config.hardware.SwerveChassisDimensions;
import com.team10043.lib.subsystems.swerve.config.hardware.SwerveChassisPhysics;
import com.team10043.lib.subsystems.swerve.config.hardware.SwerveModuleConfig;
import com.team10043.lib.subsystems.swerve.config.hardware.SwerveModuleCurrentLimits;
import com.team10043.lib.subsystems.swerve.config.hardware.SwerveModuleGearing;
import com.team10043.lib.subsystems.swerve.config.system.SwerveGyroConfig;
import com.team10043.lib.subsystems.swerve.config.system.SwerveLoopFrequencies;
import com.team10043.lib.subsystems.swerve.control.autonomous.AutoRotateController.AutoRotateConfig;
import com.team10043.lib.subsystems.swerve.control.autonomous.AutopilotSwerveController.AutopilotSwerveConfig;
import com.team10043.lib.util.control.ControlGains.ProfiledPIDConfig;
import lombok.Builder;
import lombok.Getter;
import lombok.experimental.Accessors;

/**
 * Top-level immutable configuration object for a swerve drivetrain.
 *
 * <p>This class aggregates all swerve-related configuration data including:
 *
 * <ul>
 *   <li>Module hardware configuration
 *   <li>Chassis geometry
 *   <li>Motion constraints
 *   <li>Electrical current limits
 *   <li>Physical properties (mass, MOI, friction)
 *   <li>Control and odometry loop frequencies
 *   <li>Gyro configuration
 * </ul>
 *
 * <p>This object is intended to be constructed once (typically in robot constants) and injected
 * into the swerve subsystem. It contains no robot-specific logic.
 */
@Getter
@Accessors(fluent = true)
public final class SwerveConfig<T extends GyroConfiguration> {

  public enum ModulePosition {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
  }

  private final CANBus canbus;

  private final SwerveModuleConfig frontLeftModule;
  private final SwerveModuleConfig frontRightModule;
  private final SwerveModuleConfig backLeftModule;
  private final SwerveModuleConfig backRightModule;

  private final SwerveModuleGearing moduleGearing;
  private final SwerveModuleCurrentLimits moduleCurrentLimits;

  private final SwerveChassisDimensions chassisDimensions;
  private final SwerveMotionConstraints motionConstraints;

  private final SwerveChassisPhysics chassisPhysics;

  private final SwerveLoopFrequencies loopFrequencies;

  private final SwerveGyroConfig<T> gyroConfig;
  private final GyroConfig<T> gyro;

  private final SwerveDerivedLimits derivedLimits;

  private final ProfiledPIDConfig translationPID;
  private final ProfiledPIDConfig rotationPID;

  private AutoRotateConfig autoRotateConfig;
  private AutopilotSwerveConfig autopilotConfig;

  private boolean usePhoenixPro = false;

  /**
   * Constructs swerve configuration with automatic derivation of motion limits and PID constraints.
   *
   * <p>This constructor:
   *
   * <ul>
   *   <li>Derives maximum speeds and accelerations from chassis geometry
   *   <li>Injects derived limits into PID controller configurations
   *   <li>Rebuilds AutoRotateConfig with proper speed limits if provided
   *   <li>Configures gyro with specified update frequency
   * </ul>
   *
   * @param canbus CAN bus identifier for all swerve hardware
   * @param frontLeftModule front-left module configuration
   * @param frontRightModule front-right module configuration
   * @param backLeftModule back-left module configuration
   * @param backRightModule back-right module configuration
   * @param moduleGearing drive and turn gear ratios
   * @param moduleCurrentLimits electrical current limits
   * @param chassisDimensions physical dimensions and module positions
   * @param motionConstraints maximum linear/angular velocities and accelerations
   * @param chassisPhysics mass, moment of inertia, and friction coefficient
   * @param loopFrequencies control and odometry update rates
   * @param gyroConfig gyroscope hardware configuration
   * @param translationPID PID config for X/Y translation (limits auto-injected)
   * @param rotationPID PID config for rotation (limits auto-injected)
   * @param autoRotateConfig optional auto-rotation configuration (can be null)
   * @param autopilotConfig optional Autopilot integration configuration (can be null)
   */
  @Builder
  public SwerveConfig(
      CANBus canbus,
      SwerveModuleConfig frontLeftModule,
      SwerveModuleConfig frontRightModule,
      SwerveModuleConfig backLeftModule,
      SwerveModuleConfig backRightModule,
      SwerveModuleGearing moduleGearing,
      SwerveModuleCurrentLimits moduleCurrentLimits,
      SwerveChassisDimensions chassisDimensions,
      SwerveMotionConstraints motionConstraints,
      SwerveChassisPhysics chassisPhysics,
      SwerveLoopFrequencies loopFrequencies,
      SwerveGyroConfig<T> gyroConfig,
      ProfiledPIDConfig translationPID,
      ProfiledPIDConfig rotationPID,
      AutoRotateConfig autoRotateConfig,
      AutopilotSwerveConfig autopilotConfig,
      boolean usePhoenixPro) {

    this.canbus = canbus;
    this.frontLeftModule = frontLeftModule;
    this.frontRightModule = frontRightModule;
    this.backLeftModule = backLeftModule;
    this.backRightModule = backRightModule;
    this.moduleGearing = moduleGearing;
    this.moduleCurrentLimits = moduleCurrentLimits;
    this.chassisDimensions = chassisDimensions;
    this.motionConstraints = motionConstraints;
    this.chassisPhysics = chassisPhysics;
    this.loopFrequencies = loopFrequencies;
    this.gyroConfig = gyroConfig;

    this.gyro =
        GyroConfig.<T>builder()
            .canId(new CANDeviceId(gyroConfig.gyroCanId(), canbus))
            .config(gyroConfig.gyroConfig())
            .updateFrequencyHz(loopFrequencies.odometryHz())
            .build();

    this.derivedLimits = SwerveDerivedLimits.from(motionConstraints, chassisDimensions);

    // Inject derived limits into PID configurations
    this.translationPID =
        ProfiledPIDConfig.builder()
            .gains(translationPID.gains())
            .iZone(translationPID.iZone())
            .tolerance(translationPID.tolerance())
            .maxVelocity(this.derivedLimits.maxLinearSpeed())
            .maxAcceleration(this.derivedLimits.maxLinearAcceleration())
            .build();

    this.rotationPID =
        ProfiledPIDConfig.builder()
            .gains(rotationPID.gains())
            .iZone(rotationPID.iZone())
            .tolerance(rotationPID.tolerance())
            .maxVelocity(this.derivedLimits.maxAngularSpeed())
            .maxAcceleration(this.derivedLimits.maxAngularAcceleration())
            .build();

    // Rebuild AutoRotateConfig with derived limits if provided
    this.autoRotateConfig =
        autoRotateConfig != null
            ? AutoRotateConfig.builder()
                .maxLinearSpeed(this.derivedLimits.maxLinearSpeed())
                .maxAngularSpeed(this.derivedLimits.maxAngularSpeed())
                .angleDeadbandDegrees(autoRotateConfig.angleDeadbandDegrees())
                .angleSmoothingFactor(autoRotateConfig.angleSmoothingFactor())
                .omegaScaleLowSpeed(autoRotateConfig.omegaScaleLowSpeed())
                .omegaScaleHighSpeed(autoRotateConfig.omegaScaleHighSpeed())
                .lowSpeedThreshold(autoRotateConfig.lowSpeedThreshold())
                .highSpeedThreshold(autoRotateConfig.highSpeedThreshold())
                .build()
            : null;

    this.autopilotConfig = autopilotConfig;

    this.usePhoenixPro = usePhoenixPro;
  }

  /**
   * Returns gyro configuration with CAN ID and update frequency.
   *
   * @return gyro config ready for hardware instantiation
   */
  public GyroConfig<T> gyro() {
    return gyro;
  }

  /**
   * Returns derived motion limits calculated from chassis geometry.
   *
   * @return maximum achievable speeds and accelerations
   */
  public SwerveDerivedLimits derivedLimits() {
    return derivedLimits;
  }

  /**
   * Returns module configuration for specified position.
   *
   * @param position module position identifier
   * @return configuration for that module
   */
  public SwerveModuleConfig moduleConfig(ModulePosition position) {
    return switch (position) {
      case FRONT_LEFT -> frontLeftModule;
      case FRONT_RIGHT -> frontRightModule;
      case BACK_LEFT -> backLeftModule;
      case BACK_RIGHT -> backRightModule;
    };
  }
}
