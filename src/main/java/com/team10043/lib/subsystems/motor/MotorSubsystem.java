package com.team10043.lib.subsystems.motor;

import com.team10043.lib.encoders.config.EncoderConfiguration;
import com.team10043.lib.encoders.config.NoEncoderConfig;
import com.team10043.lib.encoders.config.RemoteEncoderConfig;
import com.team10043.lib.motors.MotorIO;
import com.team10043.lib.motors.MotorIO.MotorIOData;
import com.team10043.lib.motors.MotorIO.NeutralMode;
import com.team10043.lib.motors.MotorIOInputsAutoLogged;
import com.team10043.lib.motors.config.MotorConfiguration;
import com.team10043.lib.subsystems.motor.components.encoder.ActiveEncoderComponent;
import com.team10043.lib.subsystems.motor.components.encoder.EncoderComponent;
import com.team10043.lib.subsystems.motor.components.encoder.NullEncoderComponent;
import com.team10043.lib.subsystems.motor.components.follower.ActiveFollowerComponent;
import com.team10043.lib.subsystems.motor.components.follower.FollowerComponent;
import com.team10043.lib.subsystems.motor.components.follower.NullFollowerComponent;
import com.team10043.lib.util.DebouncedAlert;
import com.team10043.lib.util.control.TunableSysIdRoutineManager;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

/**
 * Base subsystem for motor-driven mechanisms.
 *
 * <p>Supports:
 *
 * <ul>
 *   <li>Open-loop and closed-loop control
 *   <li>Optional external encoder integration
 *   <li>Optional follower motor support
 *   <li>Command-based control helpers
 * </ul>
 *
 * <p>Example usage:
 *
 * <pre>{@code
 * public class ElevatorSubsystem extends MotorSubsystem<SparkMaxConfiguration, CANCoderConfig> {
 *   public ElevatorSubsystem() {
 *     super(MotorSubsystem.builder()
 *         .config(config)
 *         .io(io)
 *         .encoder(encoderIO)
 *         .followers(followerIOs)
 *         .build());
 *   }
 * }
 * }</pre>
 *
 * @param <T> Motor configuration type
 * @param <U> Encoder configuration type
 */
public class MotorSubsystem<T extends MotorConfiguration, U extends EncoderConfiguration>
    extends SubsystemBase {

  /** Motor hardware abstraction layer. */
  protected final MotorIO io;

  /** Motor and encoder configuration. */
  protected final MotorSubsystemConfig<T, U> config;

  /** Auto-logged motor inputs. */
  protected final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

  /** External encoder component handler. */
  protected final EncoderComponent cancoderComponent;

  /** Follower motor component handler. */
  protected final FollowerComponent followerComponent;

  /** Last commanded position setpoint in rotations. */
  @Getter private double positionSetpointRots = Double.NaN;

  /** Last commanded velocity setpoint in rotations per second. */
  @Getter private double velocitySetpointRotPerSec = Double.NaN;

  /** Whether a position setpoint has been commanded. */
  private boolean hasPositionSetpoint = false;

  /** Whether a velocity setpoint has been commanded. */
  private boolean hasVelocitySetpoint = false;

  /** Alert for motor disconnection. */
  private final DebouncedAlert motorDisconnectedAlert;

  /** Motor tuning parameters. */
  private final MotorTuning tuning;

  /** SysId routine for characterization. */
  @Getter
  @SuppressWarnings("java:S3077") // volatile is safe for immutable reference assignment
  private volatile SysIdRoutine sysIdRoutine;

  /** SysId routine parameter manager. */
  private final TunableSysIdRoutineManager sysIdManager;

  public MotorSubsystem(MotorSubsystemConfig<T, U> config) {
    super(config.systemName);

    this.config = config;
    this.io = config.motorConfig.createIO();

    if (!config.followers.isEmpty()) {
      // Link simulation followers to leader for physics mirroring
      ActiveFollowerComponent<T, U> activeFollowerComponent = new ActiveFollowerComponent<>(config);
      activeFollowerComponent.linkSimulationFollowers(io);
      this.followerComponent = activeFollowerComponent;
    } else {
      this.followerComponent = new NullFollowerComponent();
    }

    if (isEncoderDisabled(config)) {
      this.cancoderComponent = new NullEncoderComponent();
    } else {
      this.cancoderComponent = new ActiveEncoderComponent<>(config.encoderConfig);
    }

    motorDisconnectedAlert =
        new DebouncedAlert(
            () -> inputs.data.connected(),
            () -> true,
            String.format("Motor [%s] disconnected.", config.systemName));

    tuning = new MotorTuning(io, io.getPIDF(0), config.systemName);

    // Initialize SysId manager with reactive callback to recreate routine when parameters change
    this.sysIdManager =
        new TunableSysIdRoutineManager(
            "SysId/Motor/" + config.systemName,
            params -> this.sysIdRoutine = createSysIdRoutineInternal(params));

    this.sysIdRoutine = createSysIdRoutine();
  }

  /**
   * Creates the SysId routine for characterization.
   *
   * <p>This method can be overridden by subclasses to customize SysId parameters such as ramp rate,
   * step voltage, and timeout.
   *
   * @return configured SysId routine
   */
  public SysIdRoutine createSysIdRoutine() {
    return createSysIdRoutine(
        config.sysIdConfig.rampRate(),
        config.sysIdConfig.stepVoltage(),
        config.sysIdConfig.timeout());
  }

  /**
   * Creates a SysId routine with specified parameters.
   *
   * <p>Parameters can be tuned via AdvantageScope when TUNING_MODE is enabled.
   *
   * @param rampRate voltage ramp rate for quasistatic test (default if tuning disabled)
   * @param stepVoltage step voltage for dynamic test (default if tuning disabled)
   * @param timeout maximum time for a test (default if tuning disabled)
   * @return configured SysId routine
   */
  public final SysIdRoutine createSysIdRoutine(
      Velocity<VoltageUnit> rampRate, Voltage stepVoltage, Time timeout) {

    // Initialize manager with defaults
    sysIdManager.initDefaults(rampRate, stepVoltage, timeout);

    // Get current values (falls back to defaults if tuning disabled)
    var params = sysIdManager.get();
    return createSysIdRoutineInternal(params);
  }

  /**
   * Internal method to create SysIdRoutine from parameters.
   *
   * @param params SysId routine parameters
   * @return configured SysId routine
   */
  private SysIdRoutine createSysIdRoutineInternal(
      TunableSysIdRoutineManager.SysIdRoutineParams params) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            params.rampRate(),
            params.stepVoltage(),
            params.timeout(),
            state -> Logger.recordOutput(getName() + "/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            voltage -> setVoltageOutput(voltage.in(Units.Volts)), null, this));
  }

  /** Periodic update loop for the subsystem. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    if (cancoderComponent.isEnabled()) {
      cancoderComponent.periodic(getName(), io);
    }

    if (followerComponent.isEnabled()) {
      followerComponent.periodic(getName());
    }

    motorDisconnectedAlert.update();

    tuning.update();

    if (getCurrentCommand() == null) {
      sysIdManager.update();
    }

    Logger.recordOutput(
        getName() + "/currentCommand",
        (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());

    Logger.recordOutput(getName() + "/TargetPosition", getPositionSetpointRots());
    Logger.recordOutput(getName() + "/CurrentPosition", getCurrentPosition());

    Logger.recordOutput(getName() + "/TargetVelocity", getVelocitySetpointRotPerSec());
    Logger.recordOutput(getName() + "/CurrentVelocity", getCurrentVelocity());
  }

  /**
   * Clamps a requested position to configured soft limits.
   *
   * @param positionRots desired position in rotations
   * @return clamped position in rotations
   */
  private double clampPosition(double positionRots) {
    return MathUtil.clamp(
        positionRots, config.motorConfig.kMinPositionRots, config.motorConfig.kMaxPositionRots);
  }

  /**
   * Updates the last commanded position setpoint.
   *
   * @param position position in rotations
   */
  private void updatePositionSetpoint(double position) {
    positionSetpointRots = clampPosition(position);
    hasPositionSetpoint = true;
  }

  /**
   * Updates the last commanded velocity setpoint.
   *
   * @param velocity velocity in rotations per second
   */
  private void updateVelocitySetpoint(double velocity) {
    velocitySetpointRotPerSec = velocity;
    hasVelocitySetpoint = true;
  }

  /** Stops the motor by commanding zero output. */
  public void stop() {
    setOpenLoopDutyCycle(0.0);
  }

  /**
   * Sets a direct torque current output to the motor.
   *
   * @param torqueCurrentAmps torque-producing current in amperes
   */
  public void setTorqueCurrentOutput(double torqueCurrentAmps) {
    io.setTorqueCurrentOutput(torqueCurrentAmps);
  }

  /**
   * Sets a direct voltage output to the motor.
   *
   * @param voltage output voltage
   */
  public void setVoltageOutput(double voltage) {
    io.setVoltageOutput(voltage);
  }

  /**
   * Sets an open-loop duty cycle output.
   *
   * @param dutyCycle output duty cycle (-1.0 to 1.0)
   */
  public void setOpenLoopDutyCycle(double dutyCycle) {
    io.setOpenLoopDutyCycle(dutyCycle);
  }

  /**
   * Sets a closed-loop position setpoint using the default control slot.
   *
   * @param position target position in rotations
   */
  public void setPositionSetpoint(double position) {
    updatePositionSetpoint(position);
    setPositionSetpoint(position, 0);
  }

  /**
   * Sets a closed-loop position setpoint.
   *
   * @param position target position in rotations
   * @param slot PID slot index
   */
  public void setPositionSetpoint(double position, int slot) {
    updatePositionSetpoint(position);
    io.setPositionSetpoint(position, slot);
  }

  /**
   * Sets a closed-loop velocity setpoint using the default control slot.
   *
   * @param velocity target velocity in rotations per second
   */
  public void setVelocitySetpoint(double velocity) {
    updateVelocitySetpoint(velocity);
    setVelocitySetpoint(velocity, 0);
  }

  /**
   * Sets a closed-loop velocity setpoint.
   *
   * @param velocity target velocity in rotations per second
   * @param slot PID slot index
   */
  public void setVelocitySetpoint(double velocity, int slot) {
    updateVelocitySetpoint(velocity);
    io.setVelocitySetpoint(velocity, slot);
  }

  /**
   * Sets a Motion Magic position setpoint using the default control slot.
   *
   * @param position target position in rotations
   */
  public void setMotionMagicSetpoint(double position) {
    updatePositionSetpoint(position);
    setMotionMagicSetpoint(position, 0);
  }

  /**
   * Sets a Motion Magic position setpoint.
   *
   * @param position target position in rotations
   * @param slot PID slot index
   */
  public void setMotionMagicSetpoint(double position, int slot) {
    updatePositionSetpoint(position);
    io.setMotionMagicSetpoint(position, slot);
  }

  /**
   * Sets a Motion Magic position setpoint with feedforward using the default control slot.
   *
   * @param position target position in rotations
   * @param feedforward feedforward output
   */
  public void setMotionMagicSetpoint(double position, double feedforward) {
    updatePositionSetpoint(position);
    setMotionMagicSetpoint(position, feedforward, 0);
  }

  /**
   * Sets a Motion Magic position setpoint with feedforward.
   *
   * @param position target position in rotations
   * @param feedforward feedforward output
   * @param slot PID slot index
   */
  public void setMotionMagicSetpoint(double position, double feedforward, int slot) {
    updatePositionSetpoint(position);
    io.setMotionMagicSetpoint(position, feedforward, slot);
  }

  /**
   * Creates a command that stops the motor when executed.
   *
   * @return command that stops the motor
   */
  public Command stopMotorCommand() {
    return runOnce(this::stop).withName(getName() + " Stop");
  }

  /**
   * Creates a command that sets voltage output and stops on end.
   *
   * @param voltage voltage supplier
   * @return voltage output command
   */
  public Command voltageOutputCommand(DoubleSupplier voltage) {
    return runEnd(() -> setVoltageOutput(voltage.getAsDouble()), () -> setVoltageOutput(0.0))
        .withName(getName() + " SetVoltageOutput");
  }

  /**
   * Creates a command that sets voltage output without stopping on end.
   *
   * @param voltage voltage supplier
   * @return voltage output command
   */
  public Command voltageOutputCommandNoEnd(DoubleSupplier voltage) {
    return run(() -> setVoltageOutput(voltage.getAsDouble()))
        .withName(getName() + " SetVoltageOutput");
  }

  /**
   * Creates a command that sets duty cycle output and stops on end.
   *
   * @param dutyCycle duty cycle supplier
   * @return duty cycle command
   */
  public Command openLoopDutyCycleCommand(DoubleSupplier dutyCycle) {
    return runEnd(
            () -> setOpenLoopDutyCycle(dutyCycle.getAsDouble()), () -> setOpenLoopDutyCycle(0.0))
        .withName(getName() + " SetOpenLoopDutyCycle");
  }

  /**
   * Creates a command that sets duty cycle output without stopping on end.
   *
   * @param dutyCycle duty cycle supplier
   * @return duty cycle command
   */
  public Command openLoopDutyCycleCommandNoEnd(DoubleSupplier dutyCycle) {
    return run(() -> setOpenLoopDutyCycle(dutyCycle.getAsDouble()))
        .withName(getName() + " SetOpenLoopDutyCycle");
  }

  /**
   * Creates a command that sets torque output and stops on end.
   *
   * @param torqueCurrentAmps current supplier
   * @return torque output command
   */
  public Command torqueOutputCommand(DoubleSupplier torqueCurrentAmps) {
    return runEnd(
            () -> setTorqueCurrentOutput(torqueCurrentAmps.getAsDouble()),
            () -> setTorqueCurrentOutput(0.0))
        .withName(getName() + " SetTorqueOutput");
  }

  /**
   * Creates a command that sets torque output without stopping on end.
   *
   * @param torqueCurrentAmps current supplier
   * @return torque output command
   */
  public Command torqueOutputCommandNoEnd(DoubleSupplier torqueCurrentAmps) {
    return run(() -> setTorqueCurrentOutput(torqueCurrentAmps.getAsDouble()))
        .withName(getName() + " SetTorqueOutput");
  }

  /**
   * Creates a position setpoint command using the default slot.
   *
   * @param position position supplier
   * @return position setpoint command
   */
  public Command positionSetpointCommand(DoubleSupplier position) {
    return positionSetpointCommand(position, 0);
  }

  /**
   * Creates a position setpoint command.
   *
   * @param position position supplier
   * @param slot PID slot index
   * @return position setpoint command
   */
  public Command positionSetpointCommand(DoubleSupplier position, int slot) {
    return run(() -> setPositionSetpoint(position.getAsDouble(), slot))
        .withName(getName() + " SetPositionSetpoint");
  }

  /**
   * Creates a position setpoint command that finishes when on target.
   *
   * @param setpointRots target position supplier
   * @return position setpoint command
   */
  public Command positionSetpointUntilOnTarget(DoubleSupplier setpointRots) {

    return positionSetpointCommand(setpointRots)
        .until(() -> isAtPositionSetpoint(config.positionToleranceRots))
        .finallyDo(() -> stop());
  }

  /**
   * Creates a position setpoint command that finishes when on target.
   *
   * @param setpointRots target position supplier
   * @param epsilon acceptable error tolerance
   * @return position setpoint command
   */
  public Command positionSetpointUntilOnTarget(
      DoubleSupplier setpointRots, DoubleSupplier epsilon) {

    return positionSetpointCommand(setpointRots)
        .until(() -> isAtPositionSetpoint(epsilon.getAsDouble()));
  }

  /**
   * Creates a velocity setpoint command using the default slot.
   *
   * @param velocity velocity supplier
   * @return velocity setpoint command
   */
  public Command velocitySetpointCommand(DoubleSupplier velocity) {
    return velocitySetpointCommand(velocity, 0);
  }

  /**
   * Creates a velocity setpoint command.
   *
   * @param velocity velocity supplier
   * @param slot PID slot index
   * @return velocity setpoint command
   */
  public Command velocitySetpointCommand(DoubleSupplier velocity, int slot) {
    return run(() -> setVelocitySetpoint(velocity.getAsDouble(), slot))
        .withName(getName() + " SetVelocitySetpoint");
  }

  /**
   * Creates a Motion Magic setpoint command using the default slot.
   *
   * @param position position supplier
   * @return Motion Magic command
   */
  public Command motionMagicSetpointCommand(DoubleSupplier position) {
    return motionMagicSetpointCommand(position, 0);
  }

  /**
   * Creates a Motion Magic command that finishes when on target.
   *
   * @param setpointRots target position supplier
   * @param epsilon acceptable error tolerance
   * @return Motion Magic command
   */
  public Command motionMagicSetpointUntilOnTarget(
      DoubleSupplier setpointRots, DoubleSupplier epsilon) {

    return motionMagicSetpointCommand(setpointRots)
        .until(() -> isAtPositionSetpoint(epsilon.getAsDouble()));
  }

  /**
   * Creates a Motion Magic setpoint command.
   *
   * @param position position supplier
   * @param slot PID slot index
   * @return Motion Magic command
   */
  public Command motionMagicSetpointCommand(DoubleSupplier position, int slot) {
    return run(() -> setMotionMagicSetpoint(position.getAsDouble(), slot))
        .withName(getName() + " SetMotionMagicSetpoint");
  }

  /**
   * Creates a Motion Magic command that finishes when on target.
   *
   * @param setpointRots target position supplier
   * @param epsilon acceptable error tolerance
   * @param slot PID slot index
   * @return Motion Magic command
   */
  public Command motionMagicSetpointUntilOnTarget(
      DoubleSupplier setpointRots, DoubleSupplier epsilon, int slot) {

    return motionMagicSetpointCommand(setpointRots, slot)
        .until(() -> isAtPositionSetpoint(epsilon.getAsDouble()));
  }

  /**
   * Creates a Motion Magic command with feedforward using the default slot.
   *
   * @param position position supplier
   * @param feedforward feedforward supplier
   * @return Motion Magic command
   */
  public Command motionMagicSetpointCommand(DoubleSupplier position, DoubleSupplier feedforward) {
    return motionMagicSetpointCommand(position, feedforward, 0);
  }

  /**
   * Creates a Motion Magic command with feedforward that finishes when on target.
   *
   * @param setpointRots target position supplier
   * @param feedforward feedforward supplier
   * @param epsilon acceptable error tolerance
   * @return Motion Magic command
   */
  public Command motionMagicSetpointUntilOnTarget(
      DoubleSupplier setpointRots, DoubleSupplier feedforward, DoubleSupplier epsilon) {

    return motionMagicSetpointCommand(setpointRots, feedforward)
        .until(() -> isAtPositionSetpoint(epsilon.getAsDouble()));
  }

  /**
   * Creates a Motion Magic command with feedforward.
   *
   * @param position position supplier
   * @param feedforward feedforward supplier
   * @param slot PID slot index
   * @return Motion Magic command
   */
  public Command motionMagicSetpointCommand(
      DoubleSupplier position, DoubleSupplier feedforward, int slot) {
    return run(() ->
            setMotionMagicSetpoint(position.getAsDouble(), feedforward.getAsDouble(), slot))
        .withName(getName() + " SetMotionMagicSetpointWithFF");
  }

  /**
   * Creates a Motion Magic command with feedforward that finishes when on target.
   *
   * @param setpointRots target position supplier
   * @param feedforward feedforward supplier
   * @param epsilon acceptable error tolerance
   * @param slot PID slot index
   * @return Motion Magic command
   */
  public Command motionMagicSetpointUntilOnTarget(
      DoubleSupplier setpointRots, DoubleSupplier feedforward, DoubleSupplier epsilon, int slot) {

    return motionMagicSetpointCommand(setpointRots, feedforward, slot)
        .until(() -> isAtPositionSetpoint(epsilon.getAsDouble()));
  }

  /**
   * Creates a command that changes brake mode at start and end.
   *
   * @param start brake mode at command start
   * @param end brake mode at command end
   * @return brake mode command
   */
  public Command setBrakeMode(NeutralMode start, NeutralMode end) {
    return startEnd(() -> setBrakeMode(start), () -> setBrakeMode(end))
        .withName(getName() + " SetBrakeMode");
  }

  /**
   * Sets the motor neutral mode.
   *
   * @param mode brake or coast
   */
  public void setBrakeMode(NeutralMode mode) {
    io.setBrakeMode(mode);
  }

  /** Sets the current motor position to zero. */
  public void setCurrentPositionAsZero() {
    setCurrentPosition(0.0);
  }

  /**
   * Sets the current motor position.
   *
   * @param position position in rotations
   */
  public void setCurrentPosition(double position) {
    io.setCurrentPosition(position);
  }

  /**
   * Gets the current position, averaged across followers if enabled.
   *
   * @return current position in rotations
   */
  public double getCurrentPosition() {
    if (followerComponent.isEnabled()) {
      return followerComponent.getAveragePosition(inputs.data.positionRots());
    }
    return inputs.data.positionRots();
  }

  /**
   * Gets the current velocity, averaged across followers if enabled.
   *
   * @return current velocity in rotations per second
   */
  public double getCurrentVelocity() {
    if (followerComponent.isEnabled()) {
      return followerComponent.getAverageVelocity(inputs.data.velocityRotPerSec());
    }
    return inputs.data.velocityRotPerSec();
  }

  /**
   * Checks if the motor is at the last commanded position target within tolerance.
   *
   * @return true if at position target, false if no position setpoint or not within tolerance
   */
  public boolean isAtPositionSetpoint() {
    return isAtPositionSetpoint(config.positionToleranceRots);
  }

  /**
   * Checks if the motor is at the last commanded position target within tolerance.
   *
   * @param toleranceRots acceptable error tolerance in rotations
   * @return true if at position target, false if no position setpoint or not within tolerance
   */
  public boolean isAtPositionSetpoint(double toleranceRots) {
    if (!hasPositionSetpoint) {
      return false;
    }
    return MathUtil.isNear(positionSetpointRots, getCurrentPosition(), toleranceRots);
  }

  /**
   * Checks if the motor is at a specific position target within tolerance.
   *
   * @param positionRots target position in rotations
   * @return true if at position target, false if no position setpoint or not within tolerance
   */
  public boolean isAtPositionTarget(double positionRots) {
    return isAtPositionTarget(positionRots, config.positionToleranceRots);
  }

  /**
   * Checks if the motor is at a specific position target within tolerance.
   *
   * @param positionRots target position in rotations
   * @param toleranceRots acceptable error tolerance in rotations
   * @return true if at position target, false if no position setpoint or not within tolerance
   */
  public boolean isAtPositionTarget(double positionRots, double toleranceRots) {
    return MathUtil.isNear(positionRots, getCurrentPosition(), toleranceRots);
  }

  /**
   * Checks if the motor is at the last commanded velocity target within tolerance.
   *
   * @param toleranceRotsPerSec acceptable error tolerance in rotations per second
   * @return true if at velocity target, false if no velocity setpoint or not within tolerance
   */
  public boolean isAtVelocitySetpoint() {
    return isAtVelocitySetpoint(config.velocityToleranceRotPerSec);
  }

  /**
   * Checks if the motor is at the last commanded velocity target within tolerance.
   *
   * @return true if at velocity target, false if no velocity setpoint or not within tolerance
   */
  public boolean isAtVelocitySetpoint(double toleranceRotsPerSec) {
    if (!hasVelocitySetpoint) {
      return false;
    }
    return MathUtil.isNear(velocitySetpointRotPerSec, getCurrentVelocity(), toleranceRotsPerSec);
  }

  /**
   * Checks if the motor is at a specific velocity target within tolerance.
   *
   * @param velocityRotsPerSec target velocity in rotations per second
   * @return true if at velocity target, false if no velocity setpoint or not within tolerance
   */
  public boolean isAtVelocityTarget(double velocityRotsPerSec) {
    return isAtVelocityTarget(velocityRotsPerSec, config.velocityToleranceRotPerSec);
  }

  /**
   * Checks if the motor is at a specific velocity target within tolerance.
   *
   * @param velocityRotsPerSec target velocity in rotations per second
   * @param toleranceRotsPerSec acceptable error tolerance in rotations per second
   * @return true if at velocity target, false if no velocity setpoint or not within tolerance
   */
  public boolean isAtVelocityTarget(double velocityRotsPerSec, double toleranceRotsPerSec) {
    return MathUtil.isNear(velocityRotsPerSec, getCurrentVelocity(), toleranceRotsPerSec);
  }

  /**
   * Checks if a position setpoint has been commanded.
   *
   * @return true if a position setpoint has been commanded
   */
  public boolean hasPositionSetpoint() {
    return hasPositionSetpoint;
  }

  /**
   * Checks if a velocity setpoint has been commanded.
   *
   * @return true if a velocity setpoint has been commanded
   */
  public boolean hasVelocitySetpoint() {
    return hasVelocitySetpoint;
  }

  /**
   * Checks if an external encoder is configured and enabled.
   *
   * @return true if an external encoder is enabled
   */
  public boolean hasCANCoder() {
    return cancoderComponent.isEnabled();
  }

  /**
   * Checks if follower motors are configured and enabled.
   *
   * @return true if follower motors are enabled
   */
  public boolean hasFollowers() {
    return followerComponent.isEnabled();
  }

  /**
   * Gets the raw motor input data.
   *
   * @return motor input data object
   */
  public MotorIOData getInputsData() {
    return inputs.data;
  }

  // #region SysId Characterization Commands

  /**
   * Runs a quasistatic SysId test in desired direction.
   *
   * @return SysId command
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  /**
   * Runs a dynamic SysId test in the desired direction.
   *
   * @return SysId command
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  // #endregion

  /**
   * Utility method to check if encoder is disabled in configuration.
   *
   * @param config motor configuration
   * @param <T> motor configuration type
   * @param <U> encoder configuration type
   * @return true if encoder is disabled
   */
  private static <T extends MotorConfiguration, U extends EncoderConfiguration>
      boolean isEncoderDisabled(MotorSubsystemConfig<T, U> config) {

    if (config.encoderConfig == null) {
      return true;
    }

    return config.encoderConfig.config instanceof NoEncoderConfig
        || config.encoderConfig.config instanceof RemoteEncoderConfig;
  }
}
