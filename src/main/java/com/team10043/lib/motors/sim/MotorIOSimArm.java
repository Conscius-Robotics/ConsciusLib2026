package com.team10043.lib.motors.sim;

import com.team10043.lib.motors.MotorIO;
import com.team10043.lib.motors.config.MotorConfig;
import com.team10043.lib.util.control.ControlGains.FeedforwardGains;
import com.team10043.lib.util.control.ControlGains.PIDFConfig;
import com.team10043.lib.util.control.ControlGains.PIDGains;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import java.util.Optional;

/**
 * Physics simulation implementation of {@link MotorIO} for arm mechanisms.
 *
 * <p>This class provides a simulated arm motor using WPILib's {@link SingleJointedArmSim}. It
 * includes:
 *
 * <ul>
 *   <li>Realistic arm physics with optional gravity
 *   <li>Configurable arm length and mass
 *   <li>Closed-loop position control with gravity feedforward
 *   <li>Closed-loop velocity control with PID + feedforward
 *   <li>Motion Magic approximation with arbitrary feedforward
 *   <li>Follower mode (mirrors leader motor without running separate physics)
 * </ul>
 *
 * <p><strong>Use cases:</strong>
 *
 * <ul>
 *   <li>Intake pivots (vertical, with gravity)
 *   <li>Shooter hoods (vertical, with gravity)
 *   <li>Climbing arms (vertical, with gravity)
 *   <li>Turrets (horizontal, without gravity)
 *   <li>Any single-jointed rotary mechanism
 * </ul>
 *
 * <p><strong>Gravity simulation:</strong> Controlled by {@code simulateGravity} parameter in
 * MotorConfig. Set to false for horizontal rotation (turrets), true for vertical pivots (intakes,
 * hoods).
 *
 * @see MotorIOSim for general-purpose motor simulation
 * @see MotorIOSimFlywheel for velocity-optimized flywheel simulation
 */
public class MotorIOSimArm extends MotorIOSimBase<MotorIOSimArm> {

  /** Simulated single-jointed arm physics model (null if follower). */
  private final SingleJointedArmSim armSim;

  /** PID controller for closed-loop position control. */
  private final PIDController positionController = new PIDController(0, 0, 0);

  /** PID controller for closed-loop velocity control. */
  private final PIDController velocityController = new PIDController(0, 0, 0);

  /** Feedforward controller for gravity compensation. */
  private ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);

  /** Target velocity setpoint for velocity control (rotations/sec). */
  private double velocitySetpoint = 0.0;

  /** Additional arbitrary feedforward voltage for Motion Magic (volts). */
  private double arbitraryFeedforward = 0.0;

  /**
   * Creates a new MotorIOSimArm from motor configuration.
   *
   * <p>All parameters are derived from the config:
   *
   * <ul>
   *   <li>Motor type (Kraken X60, NEO, etc.) from config.getMotorFromConfig()
   *   <li>Gearing from kRotorToSensorRatio * kSensorToMechanismRatio
   *   <li>Arm length from armLengthMeters
   *   <li>Arm mass from armMassKg
   *   <li>Moment of inertia (MOI) from SimulationConfig (auto-calculated or custom)
   *   <li>Position limits from kMinPositionRots/kMaxPositionRots
   *   <li>Gravity simulation from simulateGravity (true for vertical, false for horizontal)
   * </ul>
   *
   * @param config motor configuration
   */
  public MotorIOSimArm(MotorConfig<?> config) {
    super(config);

    DCMotor motor = config.getMotorFromConfig();
    double gearing = config.kRotorToSensorRatio * config.kSensorToMechanismRatio;

    var simConfig = config.simulationConfig;

    this.armSim =
        new SingleJointedArmSim(
            motor,
            gearing,
            simConfig.momentOfInertia(),
            simConfig.armLengthMeters(),
            config.kMinPositionRots * 2 * Math.PI,
            config.kMaxPositionRots * 2 * Math.PI,
            simConfig.simulateGravity(),
            config.kMinPositionRots * 2 * Math.PI);
  }

  @Override
  protected void updateFollowerInputs(MotorIOInputs inputs) {
    double positionMultiplier = getPositionMultiplier();
    double currentMultiplier = getCurrentMultiplier();

    double positionRots = leader.armSim.getAngleRads() / (2 * Math.PI);
    double velocityRPS = leader.armSim.getVelocityRadPerSec() / (2 * Math.PI);

    inputs.data =
        new MotorIOData(
            true,
            velocityRPS * positionMultiplier,
            positionRots * positionMultiplier,
            positionRots * positionMultiplier,
            leader.appliedVolts * positionMultiplier,
            Math.abs(leader.armSim.getCurrentDrawAmps()) * currentMultiplier,
            Math.abs(leader.armSim.getCurrentDrawAmps()) * currentMultiplier,
            Math.abs(leader.armSim.getCurrentDrawAmps()) * currentMultiplier);
  }

  @Override
  protected void updateLeaderInputs(MotorIOInputs inputs) {
    if (closedLoop) {
      switch (controlMode) {
        case POSITION:
          double currentAngleRots = armSim.getAngleRads() / (2 * Math.PI);
          double feedback = positionController.calculate(currentAngleRots);
          double feedforwardVolts = feedforward.calculate(armSim.getAngleRads(), 0);

          appliedVolts = clampVoltage(feedback + feedforwardVolts + arbitraryFeedforward);
          break;

        case VELOCITY:
          // Velocity control for arms using PID + feedforward
          double currentVelocityRPS = armSim.getVelocityRadPerSec() / (2 * Math.PI);
          double velocityFeedback =
              velocityController.calculate(currentVelocityRPS, velocitySetpoint);
          double velocityRadPerSec = velocitySetpoint * 2 * Math.PI;
          double velocityFeedforwardVolts =
              feedforward.calculate(armSim.getAngleRads(), velocityRadPerSec);
          appliedVolts = clampVoltage(velocityFeedback + velocityFeedforwardVolts);
          break;

        case OPEN_LOOP:
        default:
          // No-op, appliedVolts already set by setVoltage()
          break;
      }
    }

    applyBrakeMode(armSim.getVelocityRadPerSec(), 2.0);

    armSim.setInputVoltage(clampVoltage(appliedVolts));
    armSim.update(SIMULATION_PERIOD_SECONDS);

    double positionRots = armSim.getAngleRads() / (2 * Math.PI);
    double velocityRPS = armSim.getVelocityRadPerSec() / (2 * Math.PI);

    inputs.data =
        new MotorIOData(
            true,
            velocityRPS,
            positionRots,
            positionRots,
            appliedVolts,
            Double.NaN,
            Double.NaN,
            Math.abs(armSim.getCurrentDrawAmps()));
  }

  @Override
  public void setPositionSetpoint(double position, int slot) {
    closedLoop = true;
    controlMode = ControlMode.POSITION;
    positionController.setSetpoint(
        MathUtil.clamp(position, config.kMinPositionRots, config.kMaxPositionRots));
    arbitraryFeedforward = 0.0;
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
    positionController.setSetpoint(
        MathUtil.clamp(position, config.kMinPositionRots, config.kMaxPositionRots));
    arbitraryFeedforward = 0.0;
  }

  @Override
  public void setMotionMagicSetpoint(double position, double feedforwardVolts, int slot) {
    closedLoop = true;
    controlMode = ControlMode.POSITION;
    positionController.setSetpoint(
        MathUtil.clamp(position, config.kMinPositionRots, config.kMaxPositionRots));
    arbitraryFeedforward = feedforwardVolts;
  }

  @Override
  public void setCurrentPosition(double position) {
    armSim.setState(position * 2 * Math.PI, armSim.getVelocityRadPerSec());
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

    double effectiveKg = config.simulationConfig.simulateGravity() ? kG : 0.0;
    feedforward = new ArmFeedforward(kS, effectiveKg, kV, kA);
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
            Optional.of(feedforward.getKg()),
            Optional.empty(),
            Optional.empty()));
  }

  public double getAngleDegrees() {
    return Math.toDegrees(armSim.getAngleRads());
  }

  public boolean isAtLimit() {
    return armSim.hasHitLowerLimit() || armSim.hasHitUpperLimit();
  }
}
