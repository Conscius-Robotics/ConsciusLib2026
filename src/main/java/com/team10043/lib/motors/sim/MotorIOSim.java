package com.team10043.lib.motors.sim;

import com.team10043.lib.motors.MotorIO;
import com.team10043.lib.motors.config.MotorConfig;
import com.team10043.lib.util.control.ControlGains.FeedforwardGains;
import com.team10043.lib.util.control.ControlGains.PIDFConfig;
import com.team10043.lib.util.control.ControlGains.PIDGains;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import java.util.Optional;

/**
 * Physics simulation implementation of {@link MotorIO}.
 *
 * <p>This class provides a simulated motor controller using WPILib's {@link DCMotorSim}. It
 * supports:
 *
 * <ul>
 *   <li>Voltage control
 *   <li>Closed-loop position control with feedforward
 *   <li>Closed-loop velocity control with feedforward
 *   <li>Motion Magic approximation
 *   <li>Physics-based simulation with configurable parameters
 *   <li>Follower mode (mirrors leader motor without running separate physics)
 * </ul>
 *
 * <p>Designed for testing subsystems in simulation without real hardware.
 */
public class MotorIOSim extends MotorIOSimBase<MotorIOSim> {

  /** Simulated DC motor physics model (null if follower). */
  private final DCMotorSim motorSim;

  /** PID controller for closed-loop position control. */
  private final PIDController positionController = new PIDController(0, 0, 0);

  /** PID controller for closed-loop velocity control. */
  private final PIDController velocityController = new PIDController(0, 0, 0);

  /** Feedforward controller for velocity control. */
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);

  /** Target velocity setpoint for velocity control (rotations/sec). */
  private double velocitySetpoint = 0.0;

  /** Additional arbitrary feedforward voltage for position control (volts). */
  private double arbitraryFeedforward = 0.0;

  /**
   * Creates a new MotorIOSim from motor configuration.
   *
   * <p>All parameters are derived from the config:
   *
   * <ul>
   *   <li>Motor type (Kraken X60, NEO, etc.) from config.getMotorFromConfig()
   *   <li>Gearing from kRotorToSensorRatio * kSensorToMechanismRatio
   *   <li>MOI (moment of inertia) from config.simulationConfig.momentOfInertia()
   *   <li>Position limits from kMinPositionRots/kMaxPositionRots
   * </ul>
   *
   * @param config motor configuration
   */
  public MotorIOSim(MotorConfig<?> config) {
    super(config);

    DCMotor motor = config.getMotorFromConfig();
    double gearing = config.kRotorToSensorRatio * config.kSensorToMechanismRatio;
    double moi = config.simulationConfig.momentOfInertia();

    this.motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, moi, gearing), motor);
  }

  @Override
  protected void updateFollowerInputs(MotorIOInputs inputs) {
    double positionMultiplier = getPositionMultiplier();
    double currentMultiplier = getCurrentMultiplier();

    inputs.data =
        new MotorIOData(
            true,
            leader.motorSim.getAngularVelocityRPM() / 60.0 * positionMultiplier,
            leader.motorSim.getAngularPositionRotations() * positionMultiplier,
            leader.motorSim.getAngularPositionRotations() * positionMultiplier,
            leader.motorSim.getInputVoltage() * positionMultiplier,
            Math.abs(leader.motorSim.getCurrentDrawAmps()) * currentMultiplier,
            Math.abs(leader.motorSim.getCurrentDrawAmps()) * currentMultiplier,
            Math.abs(leader.motorSim.getCurrentDrawAmps()) * currentMultiplier);
  }

  @Override
  protected void updateLeaderInputs(MotorIOInputs inputs) {
    double currentPosition = motorSim.getAngularPositionRotations();
    double currentVelocity = motorSim.getAngularVelocityRPM() / 60.0;

    if (closedLoop) {
      switch (controlMode) {
        case POSITION:
          double positionFeedback = positionController.calculate(currentPosition);
          appliedVolts = clampVoltage(positionFeedback + arbitraryFeedforward);
          break;

        case VELOCITY:
          double velocityFeedback = velocityController.calculate(currentVelocity, velocitySetpoint);
          double velocityFeedforward = feedforward.calculate(velocitySetpoint);
          appliedVolts = clampVoltage(velocityFeedback + velocityFeedforward);
          break;

        case OPEN_LOOP:
        default:
          // No-op, appliedVolts already set by setVoltage()
          break;
      }
    }

    applyBrakeMode(currentVelocity, 0.5);

    motorSim.setInputVoltage(clampVoltage(appliedVolts));
    motorSim.update(SIMULATION_PERIOD_SECONDS);

    inputs.data =
        new MotorIOData(
            true,
            motorSim.getAngularVelocityRPM() / 60.0,
            motorSim.getAngularPositionRotations(),
            motorSim.getAngularPositionRotations(),
            motorSim.getInputVoltage(),
            Double.NaN,
            Double.NaN,
            Math.abs(motorSim.getCurrentDrawAmps()));
  }

  @Override
  public void setVoltageOutput(double voltage) {
    super.setVoltageOutput(voltage);
  }

  @Override
  public void setOpenLoopDutyCycle(double dutyCycle) {
    super.setOpenLoopDutyCycle(dutyCycle);
  }

  @Override
  public void setPositionSetpoint(double position, int slot) {
    closedLoop = true;
    controlMode = ControlMode.POSITION;
    double clampedPosition =
        MathUtil.clamp(position, config.kMinPositionRots, config.kMaxPositionRots);
    positionController.setSetpoint(clampedPosition);
    arbitraryFeedforward = 0.0;
  }

  @Override
  public void setVelocitySetpoint(double velocity, int slot) {
    closedLoop = true;
    controlMode = ControlMode.VELOCITY;
    velocitySetpoint = velocity;
    velocityController.setSetpoint(velocity);
    arbitraryFeedforward = 0.0;
  }

  @Override
  public void setMotionMagicSetpoint(double position, int slot) {
    setMotionMagicSetpoint(position, 0, slot);
  }

  @Override
  public void setMotionMagicSetpoint(double position, double feedforward, int slot) {
    closedLoop = true;
    controlMode = ControlMode.POSITION;
    double clampedPosition =
        MathUtil.clamp(position, config.kMinPositionRots, config.kMaxPositionRots);
    positionController.setSetpoint(clampedPosition);
    arbitraryFeedforward = feedforward;
  }

  @Override
  public void setCurrentPosition(double position) {
    motorSim.setState(position, motorSim.getAngularVelocityRPM() / 60.0);
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
    positionController.setPID(kP, kI, kD);
    velocityController.setPID(kP, kI, kD);
    feedforward = new SimpleMotorFeedforward(kS, kV, kA);
  }

  @Override
  public PIDFConfig getPIDF(int slot) {
    return new PIDFConfig(
        new PIDGains(
            positionController.getP(), positionController.getI(), positionController.getD()),
        new FeedforwardGains(
            feedforward.getKs(),
            feedforward.getKv(),
            feedforward.getKa(),
            Optional.empty(),
            Optional.empty(),
            Optional.empty()));
  }
}
