package com.team10043.lib.subsystems.swerve.module.motors.talonfx.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team10043.lib.subsystems.swerve.config.SwerveConfig;
import com.team10043.lib.subsystems.swerve.config.SwerveConfig.ModulePosition;
import com.team10043.lib.subsystems.swerve.config.hardware.SwerveDriveMotorConfig;
import com.team10043.lib.subsystems.swerve.config.system.SwerveLoopFrequencies;
import com.team10043.lib.subsystems.swerve.module.ModuleIO.ModuleIOInputs;
import com.team10043.lib.subsystems.swerve.module.motors.DriveMotorIO;
import com.team10043.lib.util.phoenix6.CTREUtil;
import com.team10043.lib.util.phoenix6.PhoenixSignalThread;
import com.team10043.lib.util.phoenix6.TalonFXFactory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import lombok.Getter;

/**
 * Abstract base for swerve drive motors backed by a CTRE TalonFX (Phoenix 6).
 *
 * <p>This class owns the TalonFX instance and common configuration:
 *
 * <ul>
 *   <li>Neutral mode, inversion, and sensor-to-mechanism ratio derived from {@link SwerveConfig}.
 *   <li>Stator/peak torque current limits set from module current limits.
 *   <li>PID Slot0 coefficients stored in {@link #motorConfig} and applied via {@link
 *       #setPIDF(double,double,double,double,double,double)}.
 *   <li>High-rate odometry sampling using {@link PhoenixSignalThread}: the TalonFX position signal
 *       is cloned and queued; {@link #updateInputs(ModuleIOInputs)} drains the queue and converts
 *       motor rotations to radians for odometry.
 *   <li>Signal update frequencies are finalized with {@link
 *       #finalizeSignals(SwerveLoopFrequencies)} to match control and odometry loop HZ while
 *       optimizing bus utilization.
 * </ul>
 *
 * <p>Concrete subclasses provide the control strategy:
 *
 * <ul>
 *   <li>{@link DriveMotorTalonFXFOC}: FOC torque/velocity requests.
 *   <li>{@link DriveMotorTalonFXVoltage}: voltage-based open-loop and velocity control.
 * </ul>
 *
 * <p>Only documentation and signal registration are handled here; no direct set() calls are made in
 * this base type. Use the subclass appropriate to your control mode.
 */
@Getter
public abstract class DriveMotorTalonFX implements DriveMotorIO {

  protected final TalonFX motor;

  protected final TalonFXConfiguration motorConfig;

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
   * Constructs a TalonFX drive motor with configuration from the swerve config.
   *
   * @param config Swerve configuration containing motor IDs, CAN bus, gear ratios, and current
   *     limits
   * @param modulePosition Module position for config lookup (FL, FR, BL, BR)
   */
  public DriveMotorTalonFX(SwerveConfig<?> config, ModulePosition modulePosition) {
    this(config, modulePosition, TalonFXFactory.create().neutralMode(NeutralModeValue.Brake));
  }

  public DriveMotorTalonFX(
      SwerveConfig<?> config, ModulePosition modulePosition, TalonFXFactory factory) {
    SwerveDriveMotorConfig moduleConfig = config.moduleConfig(modulePosition).drive();

    motor = new TalonFX(moduleConfig.motorId(), config.canbus());

    motorConfig =
        factory
            .inverted(
                moduleConfig.inverted()
                    ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive)
            .sensorToMechanismRatio(config.moduleGearing().driveReduction())
            .statorCurrentLimit(config.moduleCurrentLimits().driveCurrentLimit())
            .peakTorqueCurrent(
                config.moduleCurrentLimits().driveCurrentLimit(),
                -config.moduleCurrentLimits().driveCurrentLimit())
            .buildConfig();

    CTREUtil.applyConfiguration(motor, motorConfig);

    CTREUtil.tryUntilOK(
        () -> motor.setPosition(0.0, 0.25), motor.getDeviceID(), "Drive TalonFX Set Position");

    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    supplyCurrentAmps = motor.getSupplyCurrent();
    statorCurrentAmps = motor.getStatorCurrent();

    positionQueue =
        PhoenixSignalThread.getInstance().registerPhoenixSignal(motor.getPosition().clone());

    odometrySignals.add(position);
    controlSignals.add(velocity);
    controlSignals.add(appliedVolts);
    controlSignals.add(supplyCurrentAmps);
    controlSignals.add(statorCurrentAmps);
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

  protected abstract ModuleIODriveData getModuleIODriveData();

  @Override
  public final void updateInputs(ModuleIOInputs inputs) {

    inputs.driveData = getModuleIODriveData();

    inputs.odometryDrivePositionsRad =
        positionQueue.stream().mapToDouble(Units::rotationsToRadians).toArray();
    positionQueue.clear();
  }

  @Override
  public abstract void runOpenLoop(double output);

  @Override
  public abstract void runVelocity(double velocityRadPerSec);

  @Override
  public final void setPIDF(double kP, double kI, double kD, double kS, double kV, double kA) {
    motorConfig.Slot0.kP = kP;
    motorConfig.Slot0.kI = kI;
    motorConfig.Slot0.kD = kD;
    motorConfig.Slot0.kS = kS;
    motorConfig.Slot0.kV = kV;
    motorConfig.Slot0.kA = kA;

    CTREUtil.applyConfiguration(motor, motorConfig);
  }

  @Override
  public final void setBrakeMode(boolean enabled) {
    synchronized (motorConfig) {
      motorConfig.MotorOutput.NeutralMode =
          enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;

      CTREUtil.applyConfiguration(motor, motorConfig);
    }
  }
}
