package com.team10043.lib.motors.sim;

import com.team10043.lib.motors.MotorIO;
import com.team10043.lib.motors.config.MotorConfig;
import com.team10043.lib.util.control.ControlGains.FeedforwardGains;
import com.team10043.lib.util.control.ControlGains.PIDFConfig;
import com.team10043.lib.util.control.ControlGains.PIDGains;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import java.util.Optional;

/**
 * Physics simulation implementation of {@link MotorIO} for flywheel mechanisms.
 *
 * <p>This class provides a simulated flywheel motor using WPILib's {@link FlywheelSim}. It
 * includes:
 *
 * <ul>
 *   <li>Realistic flywheel physics with inertia
 *   <li>Velocity-based control optimized for shooters
 *   <li>SimpleMotorFeedforward for kS, kV, kA
 *   <li>Spin-up and spin-down dynamics
 *   <li>Follower mode (mirrors leader motor without running separate physics)
 * </ul>
 *
 * <p><strong>Use cases:</strong>
 *
 * <ul>
 *   <li>Shooter flywheels
 *   <li>Intake rollers
 *   <li>Indexer wheels
 *   <li>Any mechanism optimized for velocity control
 * </ul>
 *
 * <p><strong>Key differences from MotorIOSim:</strong>
 *
 * <ul>
 *   <li>Uses FlywheelSim instead of DCMotorSim for better velocity control modeling
 *   <li>Optimized for velocity setpoints rather than position
 *   <li>No gravity compensation (flywheels rotate horizontally)
 * </ul>
 *
 * @see MotorIOSim for general-purpose motor simulation
 * @see MotorIOSimArm for arm mechanisms with gravity
 */
public class MotorIOSimFlywheel extends MotorIOSimBase<MotorIOSimFlywheel> {

  /** Simulated flywheel physics model (null if follower). */
  private final FlywheelSim flywheelSim;

  /** PID controller for closed-loop velocity control. */
  private final PIDController velocityController = new PIDController(0, 0, 0);

  /** Feedforward controller for velocity control. */
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);

  /** Target velocity setpoint (rotations/sec). */
  private double velocitySetpoint = 0.0;

  /**
   * Creates a new MotorIOSimFlywheel from motor configuration.
   *
   * <p>All parameters are derived from the config:
   *
   * <ul>
   *   <li>Motor type (Kraken X60, NEO, etc.) from config.getMotorFromConfig()
   *   <li>Gearing from kRotorToSensorRatio * kSensorToMechanismRatio
   *   <li>MOI (moment of inertia) from config.simulationConfig.momentOfInertia()
   * </ul>
   *
   * @param config motor configuration
   */
  public MotorIOSimFlywheel(MotorConfig<?> config) {
    super(config);

    DCMotor motor = config.getMotorFromConfig();
    double gearing = config.kRotorToSensorRatio * config.kSensorToMechanismRatio;
    double moi = config.simulationConfig.momentOfInertia();

    this.flywheelSim =
        new FlywheelSim(LinearSystemId.createFlywheelSystem(motor, moi, gearing), motor);
  }

  @Override
  protected void updateFollowerInputs(MotorIOInputs inputs) {
    double velocityMultiplier = getPositionMultiplier();
    double currentMultiplier = getCurrentMultiplier();

    inputs.data =
        new MotorIOData(
            true,
            leader.flywheelSim.getAngularVelocityRPM() / 60.0 * velocityMultiplier,
            Double.NaN,
            Double.NaN,
            leader.flywheelSim.getInputVoltage() * velocityMultiplier,
            Math.abs(leader.flywheelSim.getCurrentDrawAmps()) * currentMultiplier,
            Math.abs(leader.flywheelSim.getCurrentDrawAmps()) * currentMultiplier,
            Math.abs(leader.flywheelSim.getCurrentDrawAmps()) * currentMultiplier);
  }

  @Override
  protected void updateLeaderInputs(MotorIOInputs inputs) {
    double currentVelocity = flywheelSim.getAngularVelocityRPM() / 60.0;

    if (closedLoop) {
      switch (controlMode) {
        case VELOCITY:
          double feedback = velocityController.calculate(currentVelocity);
          double feedforwardVolts = feedforward.calculate(velocitySetpoint);
          appliedVolts = clampVoltage(feedback + feedforwardVolts);
          break;

        case POSITION:
          // Position control not meaningful for flywheels
          appliedVolts = 0.0;
          break;

        case OPEN_LOOP:
        default:
          // No-op, appliedVolts already set by setVoltage()
          break;
      }
    }

    applyBrakeMode(currentVelocity, 0.5);

    flywheelSim.setInputVoltage(clampVoltage(appliedVolts));
    flywheelSim.update(SIMULATION_PERIOD_SECONDS);

    inputs.data =
        new MotorIOData(
            true,
            flywheelSim.getAngularVelocityRPM() / 60.0,
            Double.NaN, // Position not tracked for velocity-optimized flywheel
            Double.NaN, // Position not tracked for velocity-optimized flywheel
            flywheelSim.getInputVoltage(),
            Double.NaN,
            Double.NaN,
            Math.abs(flywheelSim.getCurrentDrawAmps()));
  }

  @Override
  public void setPositionSetpoint(double position, int slot) {
    closedLoop = true;
    controlMode = ControlMode.POSITION;
    appliedVolts = 0.0;

    throw new UnsupportedOperationException(
        "Position control not meaningful for velocity-optimized flywheels");
  }

  @Override
  public void setVelocitySetpoint(double velocity, int slot) {
    closedLoop = true;
    controlMode = ControlMode.VELOCITY;
    velocitySetpoint = velocity;
    velocityController.setSetpoint(velocity);
  }

  @Override
  public void setMotionMagicSetpoint(double position, int slot) {
    closedLoop = true;
    controlMode = ControlMode.POSITION;
    appliedVolts = 0.0;

    throw new UnsupportedOperationException("Motion Magic control not applicable to flywheels");
  }

  @Override
  public void setMotionMagicSetpoint(double position, double feedforward, int slot) {
    closedLoop = false;
    appliedVolts = 0.0;

    throw new UnsupportedOperationException("Motion Magic control not applicable to flywheels");
  }

  @Override
  public void setBrakeMode(NeutralMode mode) {
    this.brakeMode = mode;
  }

  @Override
  public void setCurrentPosition(double position) {
    throw new UnsupportedOperationException(
        "Current position setting not applicable to velocity-optimized flywheels");
  }

  @Override
  public void setPIDF(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kG,
      double kCos,
      double kCosRatio,
      int slot) {
    velocityController.setPID(kP, kI, kD);

    feedforward = new SimpleMotorFeedforward(kS, kV, kA);
  }

  @Override
  public PIDFConfig getPIDF(int slot) {
    // kG, kCos, kCosRatio not used for flywheels
    return new PIDFConfig(
        new PIDGains(
            velocityController.getP(), velocityController.getI(), velocityController.getD()),
        new FeedforwardGains(
            feedforward.getKs(),
            feedforward.getKv(),
            feedforward.getKa(),
            Optional.empty(),
            Optional.empty(),
            Optional.empty()));
  }
}
