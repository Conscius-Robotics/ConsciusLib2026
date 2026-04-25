package com.team10043.lib.motors.config;

import com.team10043.lib.drivers.CANDeviceId;
import com.team10043.lib.motors.MotorIO;
import com.team10043.lib.motors.sim.MotorIOSim;
import com.team10043.lib.motors.sim.MotorIOSimArm;
import com.team10043.lib.motors.sim.MotorIOSimFlywheel;
import com.team10043.lib.motors.sparkmax.MotorIOSparkMax;
import com.team10043.lib.motors.talonfx.MotorIOTalonFXFOC;
import com.team10043.lib.motors.talonfx.MotorIOTalonFXVoltage;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import lombok.Builder;

/**
 * Comprehensive configuration container for a motor-driven mechanism.
 *
 * <p>The generic parameters enforce compile-time consistency between the motor controller type and
 * the encoder configuration used with it.
 *
 * @param <T> motor controller configuration type
 */
@Builder
public class MotorConfig<T extends MotorConfiguration> {

  /** Mechanism type for simulation selection. */
  public enum MechanismType {
    /** Generic motor (default) - uses basic physics simulation */
    SIMPLE,
    /** Single-jointed arm - includes gravity compensation */
    ARM,
    /** Linear elevator - includes gravity load */
    ELEVATOR,
    /** Flywheel/shooter - optimized for velocity control */
    FLYWHEEL
  }

  /** Motor type for proper simulation model selection. */
  public enum MotorType {
    KRAKEN_X60,
    KRAKEN_X44,
    KRAKEN_X60_FOC,
    KRAKEN_X44_FOC,
    NEO,
    CIM
  }

  /** CAN device identifier for the primary motor controller. */
  public final CANDeviceId canId;

  /** Motor controller–specific configuration. */
  public final T config;

  /** Motor type for simulation model selection. */
  public final MotorType motorType;

  /**
   * Ratio converting rotor rotations to sensor rotations.
   *
   * <p>Primarily used when the encoder is integrated into the motor controller.
   */
  @Builder.Default public double kRotorToSensorRatio = 1.0;

  /**
   * Ratio converting sensor rotations to mechanism rotations.
   *
   * <p>This typically represents external gearing between the encoder and the driven mechanism.
   */
  @Builder.Default public double kSensorToMechanismRatio = 1.0;

  /**
   * Minimum allowed mechanism position in rotations.
   *
   * <p>Used for soft limits or safety checks.
   */
  @Builder.Default public double kMinPositionRots = Double.NEGATIVE_INFINITY;

  /**
   * Maximum allowed mechanism position in rotations.
   *
   * <p>Used for soft limits or safety checks.
   */
  @Builder.Default public double kMaxPositionRots = Double.POSITIVE_INFINITY;

  /** Desired CAN status signal update frequency in Hz. (for CTRE products only) */
  @Builder.Default public double updateFrequencyHz = 50.0;

  /** Whether to use Phoenix Pro motor controller library (for CTRE products only). */
  @Builder.Default public boolean usePhoenixPro = false;

  /**
   * Simulation configuration (optional).
   *
   * <p>Only used in simulation mode. If null, uses {@link SimulationConfig#DEFAULT}.
   *
   * <p>Use factory methods for convenience:
   *
   * <ul>
   *   <li>{@link SimulationConfig#simple(double)} - Simple mechanism
   *   <li>{@link SimulationConfig#flywheel(double)} - Flywheel
   *   <li>{@link SimulationConfig#verticalArm(double, double)} - Vertical arm with gravity
   *   <li>{@link SimulationConfig#horizontalArm(double, double)} - Horizontal arm (turret)
   * </ul>
   */
  @Builder.Default public SimulationConfig simulationConfig = SimulationConfig.DEFAULT;

  /**
   * Determines the correct DC motor model from the motor type.
   *
   * @return appropriate DCMotor model for simulation
   */
  public DCMotor getMotorFromConfig() {
    return switch (motorType) {
      case KRAKEN_X60 -> DCMotor.getKrakenX60(1);
      case KRAKEN_X60_FOC -> DCMotor.getKrakenX60Foc(1);
      case KRAKEN_X44 -> DCMotor.getKrakenX44(1);
      case KRAKEN_X44_FOC -> DCMotor.getKrakenX44Foc(1);
      case NEO -> DCMotor.getNEO(1);
      case CIM -> DCMotor.getCIM(1);
    };
  }

  public MotorIO createIO() {
    // If you want to use simulation from vendor libraries (e.g., CTRE PhoenixSim or REV
    // Simulation),
    // you can add that logic here and set mechanismType to SIMPLE for those cases.
    // if (RobotBase.isSimulation() && simulationConfig.mechanismType() != MechanismType.SIMPLE) {

    if (RobotBase.isSimulation()) {
      MotorIO motorIO =
          switch (simulationConfig.mechanismType()) {
            case ARM -> new MotorIOSimArm(this);
            case FLYWHEEL -> new MotorIOSimFlywheel(this);
            case ELEVATOR -> new MotorIOSim(this); // TODO: Create MotorIOSimElevator
            default -> new MotorIOSim(this);
          };

      motorIO.setPIDF(simulationConfig.pidfConfig());

      return motorIO;
    }

    // Real hardware
    if (config instanceof TalonFXMotorConfig) {
      @SuppressWarnings("unchecked")
      MotorConfig<TalonFXMotorConfig> talonConfig = (MotorConfig<TalonFXMotorConfig>) this;

      if (usePhoenixPro) {
        return new MotorIOTalonFXFOC(talonConfig);
      }

      return new MotorIOTalonFXVoltage(talonConfig);
    }

    if (config instanceof SparkMaxMotorConfig) {
      @SuppressWarnings("unchecked")
      MotorConfig<SparkMaxMotorConfig> sparkConfig = (MotorConfig<SparkMaxMotorConfig>) this;
      return new MotorIOSparkMax(sparkConfig);
    }

    throw new IllegalStateException("Unsupported motor config type");
  }
}
