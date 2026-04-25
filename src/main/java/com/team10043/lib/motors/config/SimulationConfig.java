package com.team10043.lib.motors.config;

import com.team10043.lib.util.control.ControlGains.PIDFConfig;

/**
 * Simulation-specific configuration parameters for motor-driven mechanisms.
 *
 * <p>Contains all physics parameters needed for different simulation types. Only used in simulation
 * mode.
 *
 * <p>Use factory methods for convenience:
 *
 * <ul>
 *   <li>{@link #simple(double)} - Simple mechanism with MOI
 *   <li>{@link #flywheel(double)} - Flywheel with MOI
 *   <li>{@link #verticalArm(double, double)} - Vertical arm with gravity
 *   <li>{@link #horizontalArm(double, double)} - Horizontal arm (turret, no gravity)
 *   <li>{@link #arm(double, double, boolean)} - Custom arm with gravity option
 * </ul>
 *
 * @param mechanismType Type of mechanism for physics simulation
 * @param momentOfInertia MOI in kg*m² (for SIMPLE, FLYWHEEL, ELEVATOR)
 * @param armLengthMeters Arm length in meters (for ARM only)
 * @param armMassKg Arm mass in kg (for ARM only)
 * @param simulateGravity Whether to simulate gravity (for ARM only)
 */
public record SimulationConfig(
    MotorConfig.MechanismType mechanismType,
    double momentOfInertia,
    double armLengthMeters,
    double armMassKg,
    boolean simulateGravity,
    PIDFConfig pidfConfig) {

  /** Default simulation config for SIMPLE mechanism. */
  public static final SimulationConfig DEFAULT = simple(0.001);

  /**
   * Creates a config for SIMPLE mechanism.
   *
   * @param momentOfInertia MOI in kg*m² (typical: 0.001-0.01)
   * @return simulation config for simple mechanism
   */
  public static SimulationConfig simple(double momentOfInertia) {
    return new SimulationConfig(
        MotorConfig.MechanismType.SIMPLE, momentOfInertia, 0.0, 0.0, false, PIDFConfig.zero());
  }

  /**
   * Creates a config for FLYWHEEL mechanism.
   *
   * @param momentOfInertia MOI in kg*m² (typical: 0.05-0.1 for shooter flywheels)
   * @return simulation config for flywheel
   */
  public static SimulationConfig flywheel(double momentOfInertia) {
    return new SimulationConfig(
        MotorConfig.MechanismType.FLYWHEEL, momentOfInertia, 0.0, 0.0, false, PIDFConfig.zero());
  }

  /**
   * Creates a config for ARM mechanism with custom gravity setting and auto-calculated MOI.
   *
   * <p>MOI is calculated as I = m * L² (point mass approximation).
   *
   * @param armLengthMeters Arm length in meters (center of rotation to center of mass)
   * @param armMassKg Arm mass in kg (including all attached components)
   * @param simulateGravity Whether to simulate gravity (true for vertical, false for horizontal)
   * @return simulation config for arm mechanism
   */
  public static SimulationConfig arm(
      double armLengthMeters, double armMassKg, boolean simulateGravity) {
    double moi = armMassKg * armLengthMeters * armLengthMeters;
    return new SimulationConfig(
        MotorConfig.MechanismType.ARM,
        moi,
        armLengthMeters,
        armMassKg,
        simulateGravity,
        PIDFConfig.zero());
  }

  /**
   * Creates a config for ARM mechanism with custom MOI and gravity setting.
   *
   * <p>Use this when you have a precise MOI value from CAD or need to override the calculated
   * value.
   *
   * @param momentOfInertia MOI in kg*m² (from CAD or manual calculation)
   * @param armLengthMeters Arm length in meters (center of rotation to center of mass)
   * @param armMassKg Arm mass in kg (including all attached components)
   * @param simulateGravity Whether to simulate gravity (true for vertical, false for horizontal)
   * @return simulation config for arm mechanism with custom MOI
   */
  public static SimulationConfig arm(
      double momentOfInertia, double armLengthMeters, double armMassKg, boolean simulateGravity) {
    return new SimulationConfig(
        MotorConfig.MechanismType.ARM,
        momentOfInertia,
        armLengthMeters,
        armMassKg,
        simulateGravity,
        PIDFConfig.zero());
  }

  /**
   * Creates a config for vertical ARM mechanism (with gravity).
   *
   * <p>Use for intake pivots, shooter hoods, climbing arms, etc. MOI is auto-calculated.
   *
   * @param armLengthMeters Arm length in meters (center of rotation to center of mass)
   * @param armMassKg Arm mass in kg (including all attached components)
   * @return simulation config for vertical arm with gravity
   */
  public static SimulationConfig verticalArm(double armLengthMeters, double armMassKg) {
    return arm(armLengthMeters, armMassKg, true);
  }

  /**
   * Creates a config for vertical ARM mechanism with custom MOI (with gravity).
   *
   * <p>Use when you have a precise MOI value from CAD.
   *
   * @param momentOfInertia MOI in kg*m² (from CAD or manual calculation)
   * @param armLengthMeters Arm length in meters (center of rotation to center of mass)
   * @param armMassKg Arm mass in kg (including all attached components)
   * @return simulation config for vertical arm with gravity and custom MOI
   */
  public static SimulationConfig verticalArm(
      double momentOfInertia, double armLengthMeters, double armMassKg) {
    return arm(momentOfInertia, armLengthMeters, armMassKg, true);
  }

  /**
   * Creates a config for horizontal ARM mechanism (turret, no gravity).
   *
   * <p>Use for turrets, horizontal rotating mechanisms that don't experience gravity torque. MOI is
   * auto-calculated.
   *
   * @param armLengthMeters Arm radius in meters (center of rotation to center of mass)
   * @param armMassKg Turret mass in kg (including all attached components)
   * @return simulation config for horizontal arm without gravity
   */
  public static SimulationConfig horizontalArm(double armLengthMeters, double armMassKg) {
    return arm(armLengthMeters, armMassKg, false);
  }

  /**
   * Creates a config for horizontal ARM mechanism with custom MOI (turret, no gravity).
   *
   * <p>Use when you have a precise MOI value from CAD for turret mechanisms.
   *
   * @param momentOfInertia MOI in kg*m² (from CAD or manual calculation)
   * @param armLengthMeters Arm radius in meters (center of rotation to center of mass)
   * @param armMassKg Turret mass in kg (including all attached components)
   * @return simulation config for horizontal arm without gravity and custom MOI
   */
  public static SimulationConfig horizontalArm(
      double momentOfInertia, double armLengthMeters, double armMassKg) {
    return arm(momentOfInertia, armLengthMeters, armMassKg, false);
  }

  public SimulationConfig withPIDFConfig(PIDFConfig pidfConfig) {
    return new SimulationConfig(
        this.mechanismType,
        this.momentOfInertia,
        this.armLengthMeters,
        this.armMassKg,
        this.simulateGravity,
        pidfConfig);
  }
}
