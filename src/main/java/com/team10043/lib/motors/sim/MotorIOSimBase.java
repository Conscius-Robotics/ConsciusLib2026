package com.team10043.lib.motors.sim;

import com.team10043.lib.drivers.CANDeviceId;
import com.team10043.lib.motors.MotorIO;
import com.team10043.lib.motors.config.MotorConfig;
import edu.wpi.first.math.MathUtil;

/**
 * Abstract base class for all simulation motor implementations.
 *
 * <p>Provides common functionality for all motor simulations:
 *
 * <ul>
 *   <li>Follower mode support with leader mirroring
 *   <li>Brake mode simulation
 *   <li>Closed-loop control state management
 *   <li>Voltage clamping and application
 * </ul>
 *
 * <p>Subclasses must implement physics-specific simulation logic.
 *
 * @param <T> The specific MotorIOSim subclass type (for type-safe follower linking)
 */
public abstract class MotorIOSimBase<T extends MotorIOSimBase<T>> implements MotorIO {

  /**
   * Control mode enumeration.
   *
   * <p>Defines the active control strategy for the motor simulation.
   */
  protected enum ControlMode {
    /** Open-loop voltage or duty cycle control. */
    OPEN_LOOP,
    /** Closed-loop position control with PID + feedforward. */
    POSITION,
    /** Closed-loop velocity control. */
    VELOCITY
  }

  /** Simulation update period in seconds. */
  protected static final double SIMULATION_PERIOD_SECONDS = 0.02;

  /** Motor configuration containing all parameters. */
  protected final MotorConfig<?> config;

  /** Whether closed-loop control is active. */
  protected boolean closedLoop = false;

  /** Current control mode. */
  protected ControlMode controlMode = ControlMode.OPEN_LOOP;

  /** Applied voltage to the motor. */
  protected double appliedVolts = 0.0;

  /** Current brake mode setting. */
  protected NeutralMode brakeMode = NeutralMode.COAST;

  /** Leader motor to follow (null if this is a leader). */
  protected T leader = null;

  /** Whether this follower's output is inverted relative to leader. */
  protected boolean inverted = false;

  /**
   * Creates a new MotorIOSimBase.
   *
   * @param config motor configuration
   */
  protected MotorIOSimBase(MotorConfig<?> config) {
    this.config = config;
  }

  @Override
  public final void updateInputs(MotorIOInputs inputs) {
    if (isFollower()) {
      updateFollowerInputs(inputs);
    } else {
      updateLeaderInputs(inputs);
    }
  }

  /**
   * Updates inputs for a follower motor by mirroring the leader's state.
   *
   * @param inputs the inputs object to populate
   */
  protected abstract void updateFollowerInputs(MotorIOInputs inputs);

  /**
   * Updates inputs for a leader motor by running physics simulation.
   *
   * @param inputs the inputs object to populate
   */
  protected abstract void updateLeaderInputs(MotorIOInputs inputs);

  /**
   * Applies brake mode damping if applicable.
   *
   * @param currentVelocity current velocity in appropriate units
   * @param dampingFactor damping coefficient (higher = more braking)
   */
  protected void applyBrakeMode(double currentVelocity, double dampingFactor) {
    if (brakeMode == NeutralMode.BRAKE && !closedLoop && Math.abs(appliedVolts) < 0.01) {
      appliedVolts = -currentVelocity * dampingFactor;
    }
  }

  /**
   * Clamps voltage to battery limits.
   *
   * @param voltage raw voltage
   * @return clamped voltage [-12.0, 12.0]
   */
  protected double clampVoltage(double voltage) {
    return MathUtil.clamp(voltage, -12.0, 12.0);
  }

  @Override
  public boolean isConnected() {
    return true; // Always connected in simulation
  }

  @Override
  public void setVoltageOutput(double voltage) {
    closedLoop = false;
    controlMode = ControlMode.OPEN_LOOP;
    appliedVolts = voltage;
  }

  @Override
  public void setOpenLoopDutyCycle(double dutyCycle) {
    closedLoop = false;
    controlMode = ControlMode.OPEN_LOOP;
    appliedVolts = dutyCycle * 12.0;
  }

  @Override
  public void setBrakeMode(NeutralMode mode) {
    this.brakeMode = mode;
  }

  @Override
  public void follow(CANDeviceId masterId, boolean opposeMasterDirection) {
    this.inverted = opposeMasterDirection;
  }

  /**
   * Sets the leader motor for this follower to mirror.
   *
   * <p>This is a simulation-specific method called by the FollowerComponent to establish the
   * leader-follower relationship.
   *
   * @param leader the leader motor to follow
   */
  public void setLeader(T leader) {
    this.leader = leader;
  }

  /**
   * Checks if this motor is configured as a follower.
   *
   * @return true if this motor is following another motor
   */
  public boolean isFollower() {
    return leader != null;
  }

  /**
   * Gets the position multiplier for follower motors (1.0 or -1.0 based on inversion).
   *
   * @return position multiplier
   */
  protected double getPositionMultiplier() {
    return inverted ? -1.0 : 1.0;
  }

  /**
   * Gets the current multiplier for follower motors (always 1.0, current is magnitude).
   *
   * @return current multiplier
   */
  protected double getCurrentMultiplier() {
    return 1.0;
  }
}
