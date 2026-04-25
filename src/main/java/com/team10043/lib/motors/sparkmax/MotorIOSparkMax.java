package com.team10043.lib.motors.sparkmax;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team10043.lib.drivers.CANDeviceId;
import com.team10043.lib.motors.MotorIO;
import com.team10043.lib.motors.config.MotorConfig;
import com.team10043.lib.motors.config.SparkMaxMotorConfig;
import com.team10043.lib.util.control.ControlGains.FeedforwardGains;
import com.team10043.lib.util.control.ControlGains.PIDFConfig;
import com.team10043.lib.util.control.ControlGains.PIDGains;
import com.team10043.lib.util.revlib.REVUtil;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

/**
 * Spark MAX motor controller implementation of {@link MotorIO}.
 *
 * <p>This class provides a hardware abstraction layer for REV Robotics Spark MAX controllers using
 * the integrated relative encoder. It supports open-loop, closed-loop, and MAX Motion control modes
 * while maintaining consistency with {@link MotorConfig} and {@link MotorIO} APIs.
 *
 * <p>Rotor-to-mechanism conversion is handled via Spark MAX encoder conversion factors, ensuring
 * all reported positions and velocities are in mechanism units.
 *
 * @param <T> encoder configuration type associated with this motor
 */
public class MotorIOSparkMax implements MotorIO {

  /** Underlying Spark MAX motor controller. */
  protected final SparkMax motor;

  /** Motor and encoder configuration container. */
  protected final MotorConfig<SparkMaxMotorConfig> config;

  /** Integrated relative encoder. */
  private final RelativeEncoder encoder;

  /** Closed-loop controller for position, velocity, and MAX Motion control. */
  private final SparkClosedLoopController closedLoopController;

  /** Conversion factor from RPM to rotations per second. */
  private static final double RPM_TO_RPS = 1.0 / 60.0;

  /**
   * Constructs a Spark MAX motor IO instance and applies configuration.
   *
   * <p>This constructor:
   *
   * <ul>
   *   <li>Creates the Spark MAX device
   *   <li>Configures encoder conversion factors for mechanism units
   *   <li>Applies the Spark MAX configuration with retry logic
   *   <li>Initializes encoder and closed-loop controller references
   * </ul>
   *
   * @param config motor and encoder configuration
   */
  public MotorIOSparkMax(MotorConfig<SparkMaxMotorConfig> config) {
    this.config = config;
    this.motor = new SparkMax(config.canId.getDeviceNumber(), MotorType.kBrushless);

    // Convert rotor rotations to mechanism rotations
    double rotorToMechanismRatio = config.kRotorToSensorRatio * config.kSensorToMechanismRatio;
    config.config.encoder.positionConversionFactor(rotorToMechanismRatio);

    // Convert rotor RPM to mechanism rotations per second
    config.config.encoder.velocityConversionFactor(RPM_TO_RPS * rotorToMechanismRatio);

    REVUtil.applyConfigurationBlocking(motor, config.config);

    encoder = motor.getEncoder();
    closedLoopController = motor.getClosedLoopController();
  }

  /** {@inheritDoc} */
  @Override
  public void updateInputs(MotorIOInputs inputs) {
    double rawRotorPosition =
        encoder.getPosition() / (config.kSensorToMechanismRatio * config.kRotorToSensorRatio);

    inputs.data =
        new MotorIOData(
            isConnected(),
            encoder.getVelocity(),
            encoder.getPosition(),
            rawRotorPosition,
            motor.getAppliedOutput() * motor.getBusVoltage(),
            Double.NaN,
            Double.NaN,
            motor.getOutputCurrent());
  }

  @Override
  public boolean isConnected() {
    return motor.getLastError() == com.revrobotics.REVLibError.kOk;
  }

  /** {@inheritDoc} */
  @Override
  public void setVoltageOutput(double voltage) {
    motor.setVoltage(voltage);
  }

  /** {@inheritDoc} */
  @Override
  public void setOpenLoopDutyCycle(double dutyCycle) {
    motor.set(dutyCycle);
  }

  /** {@inheritDoc} */
  @Override
  public void setPositionSetpoint(double position, int slot) {
    closedLoopController.setSetpoint(position, ControlType.kPosition, REVUtil.getSlotByIndex(slot));
  }

  /** {@inheritDoc} */
  @Override
  public void setVelocitySetpoint(double velocity, int slot) {
    closedLoopController.setSetpoint(velocity, ControlType.kVelocity, REVUtil.getSlotByIndex(slot));
  }

  /** {@inheritDoc} */
  @Override
  public void setMotionMagicSetpoint(double position, int slot) {
    closedLoopController.setSetpoint(
        position, ControlType.kMAXMotionPositionControl, REVUtil.getSlotByIndex(slot));
  }

  /** {@inheritDoc} */
  @Override
  public void setMotionMagicSetpoint(double position, double feedforward, int slot) {
    closedLoopController.setSetpoint(
        position, ControlType.kMAXMotionPositionControl, REVUtil.getSlotByIndex(slot), feedforward);
  }

  /** {@inheritDoc} */
  @Override
  public void setBrakeMode(NeutralMode mode) {
    SparkMaxConfig tempConfig = new SparkMaxConfig();

    tempConfig.idleMode(
        mode == NeutralMode.BRAKE
            ? com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake
            : com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);

    REVUtil.applyConfigurationNonBlocking(motor, tempConfig);
  }

  /** {@inheritDoc} */
  @Override
  public void setCurrentPosition(double position) {
    encoder.setPosition(position);
  }

  /**
   * Configures this motor to follow another Spark MAX motor.
   *
   * <p>Following behavior is applied through Spark MAX configuration and persisted to flash memory.
   *
   * @param masterId CAN ID of the master motor
   * @param opposeMasterDirection whether to invert follower direction
   */
  @Override
  public void follow(CANDeviceId masterId, boolean opposeMasterDirection) {
    config.config.follow(masterId.getDeviceNumber(), opposeMasterDirection);

    REVUtil.applyConfiguration(
        motor, config.config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
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

    ClosedLoopSlot closedLoopSlot = REVUtil.getSlotByIndex(slot);

    ClosedLoopConfig closedLoopConfig =
        config.config.closedLoop.p(kP, closedLoopSlot).i(kI, closedLoopSlot).d(kD, closedLoopSlot);

    FeedForwardConfig feedForward =
        config
            .config
            .closedLoop
            .feedForward
            .kS(kS, closedLoopSlot)
            .kV(kV, closedLoopSlot)
            .kA(kA, closedLoopSlot);

    if (kG != 0.0 && kCos != 0.0) {
      DriverStation.reportError(
          "[REV PID, MotorIO] kG and kCos cannot be used together. Both were non-zero -> kCos won't be used.\n"
              + "Use kG for linear mechanisms, kCos (+kCosRatio) for arms.\n Values: "
              + "kG "
              + kG
              + " kCos "
              + kCos,
          false);

      kCos = 0.0;
    }

    if (kG != 0.0) {
      feedForward.kG(kG, closedLoopSlot);
    } else if (kCos != 0.0) {
      feedForward.kCos(kCos, closedLoopSlot).kCosRatio(kCosRatio, closedLoopSlot);
    }

    closedLoopConfig.apply(feedForward);

    config.config.apply(closedLoopConfig);

    REVUtil.applyConfigurationNonBlocking(motor, config.config);
  }

  @Override
  public PIDFConfig getPIDF(int slot) {
    ClosedLoopSlot closedLoopSlot = REVUtil.getSlotByIndex(slot);

    double kP = motor.configAccessor.closedLoop.getP(closedLoopSlot);
    double kI = motor.configAccessor.closedLoop.getI(closedLoopSlot);
    double kD = motor.configAccessor.closedLoop.getD(closedLoopSlot);

    double kS = motor.configAccessor.closedLoop.feedForward.getkS(closedLoopSlot);
    double kV = motor.configAccessor.closedLoop.feedForward.getkV(closedLoopSlot);
    double kA = motor.configAccessor.closedLoop.feedForward.getkA(closedLoopSlot);
    double kG = motor.configAccessor.closedLoop.feedForward.getkG(closedLoopSlot);
    double kCos = motor.configAccessor.closedLoop.feedForward.getkCos(closedLoopSlot);
    double kCosRatio = motor.configAccessor.closedLoop.feedForward.getkCosRatio(closedLoopSlot);

    Optional<Double> kGOpt = kG != 0.0 ? Optional.of(kG) : Optional.empty();
    Optional<Double> kCosOpt = kCos != 0.0 ? Optional.of(kCos) : Optional.empty();
    Optional<Double> kCosRatioOpt = kCosRatio != 0.0 ? Optional.of(kCosRatio) : Optional.empty();

    return new PIDFConfig(
        new PIDGains(kP, kI, kD), new FeedforwardGains(kS, kV, kA, kGOpt, kCosOpt, kCosRatioOpt));
  }

  @Override
  public void updateFrequency(double frequencyHz) {
    // Spark MAX does not support configurable status frame rates
  }
}
