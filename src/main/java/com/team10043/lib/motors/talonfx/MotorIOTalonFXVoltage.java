package com.team10043.lib.motors.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.team10043.lib.motors.MotorIO;
import com.team10043.lib.motors.MotorIO.MotorIOInputs;
import com.team10043.lib.motors.config.MotorConfig;
import com.team10043.lib.motors.config.TalonFXMotorConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * TalonFX (Falcon 500 / Kraken X60) motor controller implementation of {@link MotorIO}.
 *
 * <p>This class provides a hardware abstraction layer over CTRE Phoenix 6 TalonFX controllers,
 * handling configuration application, closed-loop control modes, telemetry collection, and CAN bus
 * optimization.
 *
 * <p>It integrates with {@link MotorConfig} for configuration consistency and AdvantageKit-style
 * logging via {@link MotorIOInputs}.
 */
public class MotorIOTalonFXVoltage extends MotorIOTalonFX {

  /** Closed-loop velocity control request. */
  private final VelocityVoltage velocityControl = new VelocityVoltage(0.0);

  /** Closed-loop position control request. */
  private final PositionVoltage positionControl = new PositionVoltage(0.0);

  /** Motion Magic position control request. */
  private final MotionMagicVoltage motionMagicPositionControl = new MotionMagicVoltage(0.0);

  /** Position feedback signal (mechanism rotations). */
  private final StatusSignal<Angle> positionSignal;

  /** Velocity feedback signal (rotations per second). */
  private final StatusSignal<AngularVelocity> velocitySignal;

  /** Applied motor voltage signal. */
  private final StatusSignal<Voltage> voltageSignal;

  /** Stator current draw signal. */
  private final StatusSignal<Current> currentStatorSignal;

  /** Supply current draw signal. */
  private final StatusSignal<Current> currentSupplySignal;

  /** Raw rotor position signal (before gearing). */
  private final StatusSignal<Angle> rawRotorPositionSignal;

  public MotorIOTalonFXVoltage(MotorConfig<TalonFXMotorConfig> config) {
    super(config);

    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    currentStatorSignal = motor.getStatorCurrent();
    currentSupplySignal = motor.getSupplyCurrent();
    rawRotorPositionSignal = motor.getRotorPosition();

    initializeSignals();
  }

  @Override
  public BaseStatusSignal[] getSignals() {
    return new BaseStatusSignal[] {
      positionSignal,
      velocitySignal,
      voltageSignal,
      currentStatorSignal,
      currentSupplySignal,
      rawRotorPositionSignal
    };
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    inputs.data =
        new MotorIOData(
            isConnected(),
            velocitySignal.getValueAsDouble(),
            positionSignal.getValueAsDouble(),
            rawRotorPositionSignal.getValueAsDouble(),
            voltageSignal.getValueAsDouble(),
            Double.NaN, // torque current is not avaible for non-pro
            currentStatorSignal.getValueAsDouble(),
            currentSupplySignal.getValueAsDouble());
  }

  @Override
  public void setPositionSetpoint(double position, int slot) {
    motor.setControl(positionControl.withPosition(position).withSlot(slot));
  }

  @Override
  public void setVelocitySetpoint(double velocity, int slot) {
    motor.setControl(velocityControl.withVelocity(velocity).withSlot(slot));
  }

  @Override
  public void setMotionMagicSetpoint(double position, int slot) {
    motor.setControl(motionMagicPositionControl.withPosition(position).withSlot(slot));
  }

  @Override
  public void setMotionMagicSetpoint(double position, double feedforward, int slot) {
    motor.setControl(
        motionMagicPositionControl
            .withPosition(position)
            .withSlot(slot)
            .withFeedForward(feedforward));
  }
}
