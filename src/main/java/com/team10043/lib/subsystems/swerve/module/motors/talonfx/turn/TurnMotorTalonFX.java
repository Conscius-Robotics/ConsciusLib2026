package com.team10043.lib.subsystems.swerve.module.motors.talonfx.turn;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.team10043.lib.drivers.CANDeviceId;
import com.team10043.lib.motors.config.TalonFXMotorConfig;
import com.team10043.lib.subsystems.swerve.config.SwerveConfig;
import com.team10043.lib.subsystems.swerve.config.SwerveConfig.ModulePosition;
import com.team10043.lib.subsystems.swerve.config.hardware.SwerveTurnMotorConfig;
import com.team10043.lib.subsystems.swerve.config.system.SwerveLoopFrequencies;
import com.team10043.lib.subsystems.swerve.module.ModuleIO.ModuleIOInputs;
import com.team10043.lib.subsystems.swerve.module.motors.TurnMotorIO;
import com.team10043.lib.util.phoenix6.CANCoderFactory;
import com.team10043.lib.util.phoenix6.CTREUtil;
import com.team10043.lib.util.phoenix6.PhoenixSignalThread;
import com.team10043.lib.util.phoenix6.TalonFXFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import lombok.Getter;

/**
 * Abstract base for swerve steering (turn) motors using a CTRE TalonFX with remote absolute
 * position feedback from a {@link CANcoder}.
 *
 * <p>Key behavior:
 *
 * <ul>
 *   <li>Feedback source is selected based on {@link SwerveConfig#usePhoenixPro()}: {@code
 *       FusedCANcoder} when Pro is enabled, otherwise {@code RemoteCANcoder}.
 *   <li>Continuous wrap is enabled so commanded positions are modulo 2π, aligned to swerve usage.
 *   <li>Rotor-to-sensor ratio uses the module turn reduction from {@link SwerveConfig}.
 *   <li>Absolute CANcoder magnet offset and direction are applied via {@link CANCoderFactory}.
 *   <li>High-rate odometry via {@link PhoenixSignalThread}: motor position samples are queued and
 *       consumed in {@link #updateInputs(ModuleIOInputs)} as {@link Rotation2d} values.
 *   <li>Signal update rates and bus utilization are finalized by {@link
 *       #finalizeSignals(SwerveLoopFrequencies)}.
 * </ul>
 *
 * <p>Concrete subclasses implement the control mode:
 *
 * <ul>
 *   <li>{@link TurnMotorTalonFXFOC}: FOC torque for open-loop and position control.
 *   <li>{@link TurnMotorTalonFXVoltage}: voltage-based open-loop and position control.
 * </ul>
 */
@Getter
public abstract class TurnMotorTalonFX implements TurnMotorIO {

  protected final TalonFX motor;
  protected final CANcoder encoder;

  protected final TalonFXMotorConfig turnConfig;

  protected final StatusSignal<Angle> absolutePosition;
  protected final StatusSignal<Angle> position;
  protected final StatusSignal<AngularVelocity> velocity;
  protected final StatusSignal<Voltage> appliedVolts;
  protected final StatusSignal<Current> supplyCurrentAmps;
  protected final StatusSignal<Current> statorCurrentAmps;

  @Getter(lombok.AccessLevel.NONE)
  private final Queue<Double> positionQueue;

  protected final List<BaseStatusSignal> odometrySignals = new ArrayList<>();
  protected final List<BaseStatusSignal> controlSignals = new ArrayList<>();
  protected final List<BaseStatusSignal> allSignals = new ArrayList<>();

  /**
   * Constructs a TalonFX turn motor with CANcoder absolute position feedback.
   *
   * @param config Swerve configuration containing motor IDs, encoder settings, CAN bus, and current
   *     limits
   * @param modulePosition Module position for config lookup (FL, FR, BL, BR)
   * @param factory TalonFX factory for creating the motor instance
   */
  public TurnMotorTalonFX(
      SwerveConfig<?> config, ModulePosition modulePosition, TalonFXFactory factory) {
    SwerveTurnMotorConfig moduleConfig = config.moduleConfig(modulePosition).turn();

    motor = new TalonFX(moduleConfig.motorId(), config.canbus());

    turnConfig =
        factory
            .inverted(
                moduleConfig.motorInverted()
                    ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive)
            .feedbackRemoteId(moduleConfig.encoderChannel())
            .rotorToSensorRatio(config.moduleGearing().turnReduction())
            .continuousWrap(true)
            .statorCurrentLimit(config.moduleCurrentLimits().turnCurrentLimit())
            .peakTorqueCurrent(
                config.moduleCurrentLimits().turnCurrentLimit(),
                -config.moduleCurrentLimits().turnCurrentLimit())
            .pid(0.0, 0.0, 0.0, 0)
            .feedforward(0, 0, 0, 0, 0)
            .buildConfig();

    CTREUtil.applyConfiguration(motor, turnConfig);

    encoder =
        CANCoderFactory.create()
            .magnetOffset(moduleConfig.encoderOffset())
            .sensorDirection(
                moduleConfig.encoderInverted()
                    ? SensorDirectionValue.Clockwise_Positive
                    : SensorDirectionValue.CounterClockwise_Positive)
            .absoluteSensorDiscontinuity(0.5)
            .createCANcoder(new CANDeviceId(moduleConfig.encoderChannel(), config.canbus()));

    absolutePosition = encoder.getAbsolutePosition();
    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    supplyCurrentAmps = motor.getSupplyCurrent();
    statorCurrentAmps = motor.getStatorCurrent();

    positionQueue =
        PhoenixSignalThread.getInstance().registerPhoenixSignal(motor.getPosition().clone());

    odometrySignals.add(position);
    odometrySignals.add(absolutePosition);
    controlSignals.add(velocity);
    controlSignals.add(appliedVolts);
    controlSignals.add(supplyCurrentAmps);
    controlSignals.add(statorCurrentAmps);
  }

  public TurnMotorTalonFX(SwerveConfig<?> config, ModulePosition modulePosition) {
    this(
        config,
        modulePosition,
        TalonFXFactory.create()
            .neutralMode(NeutralModeValue.Brake)
            .feedbackSource(
                config.usePhoenixPro()
                    ? FeedbackSensorSourceValue.FusedCANcoder
                    : FeedbackSensorSourceValue.RemoteCANcoder));
  }

  protected final void finalizeSignals(SwerveLoopFrequencies frequencies) {

    BaseStatusSignal.setUpdateFrequencyForAll(
        frequencies.odometryHz(), odometrySignals.toArray(BaseStatusSignal[]::new));

    BaseStatusSignal.setUpdateFrequencyForAll(
        frequencies.controlLoopHz(), controlSignals.toArray(BaseStatusSignal[]::new));

    CTREUtil.optimizeBusUtilizationForAll(motor);

    allSignals.addAll(odometrySignals);
    allSignals.addAll(controlSignals);

    CTREUtil.registerSignals(true, allSignals.toArray(BaseStatusSignal[]::new));
  }

  protected abstract ModuleIOTurnData getModuleIOTurnData();

  @Override
  public final void updateInputs(ModuleIOInputs inputs) {
    inputs.turnData = getModuleIOTurnData();

    inputs.odometryTurnPositions =
        positionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
    positionQueue.clear();
  }

  @Override
  public abstract void runOpenLoop(double output);

  @Override
  public abstract void runPosition(Rotation2d rotation);

  @Override
  public final void setPIDFF(double kP, double kI, double kD, double kS, double kV, double kA) {
    turnConfig.Slot0.kP = kP;
    turnConfig.Slot0.kI = kI;
    turnConfig.Slot0.kD = kD;
    turnConfig.Slot0.kS = kS;
    turnConfig.Slot0.kV = kV;
    turnConfig.Slot0.kA = kA;

    CTREUtil.applyConfiguration(motor, turnConfig);
  }

  @Override
  public final void setBrakeMode(boolean enabled) {
    synchronized (turnConfig) {
      turnConfig.MotorOutput.NeutralMode =
          enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;

      CTREUtil.applyConfiguration(motor, turnConfig);
    }
  }
}
