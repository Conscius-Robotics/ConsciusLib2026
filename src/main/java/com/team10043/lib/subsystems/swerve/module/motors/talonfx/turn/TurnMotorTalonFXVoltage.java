package com.team10043.lib.subsystems.swerve.module.motors.talonfx.turn;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.team10043.lib.subsystems.swerve.config.SwerveConfig;
import com.team10043.lib.subsystems.swerve.config.SwerveConfig.ModulePosition;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import lombok.Getter;

/**
 * Steering (turn) motor implementation using Phoenix 6 voltage-based control with remote absolute
 * CANcoder feedback.
 *
 * <p>Control strategy:
 *
 * <ul>
 *   <li>Open-loop uses {@link VoltageOut} (direct voltage command).
 *   <li>Closed-loop position uses {@link PositionVoltage} with targets specified in mechanism
 *       rotations (from {@link Rotation2d}).
 * </ul>
 *
 * <p>Telemetry mirrors the base {@link TurnMotorTalonFX} and omits torque current metrics.
 */
@Getter
public class TurnMotorTalonFXVoltage extends TurnMotorTalonFX {

  private final VoltageOut voltageRequest = new VoltageOut(0).withUpdateFreqHz(0);
  private final PositionVoltage positionVoltageRequest =
      new PositionVoltage(0.0).withUpdateFreqHz(0);

  /**
   * Constructs a TalonFX turn motor with CANcoder absolute position feedback.
   *
   * @param config Swerve configuration containing motor IDs, encoder settings, CAN bus, and current
   *     limits
   * @param modulePosition Module position for config lookup (FL, FR, BL, BR)
   */
  public TurnMotorTalonFXVoltage(SwerveConfig<?> config, ModulePosition modulePosition) {
    super(config, modulePosition);

    finalizeSignals(config.loopFrequencies());
  }

  @Override
  public ModuleIOTurnData getModuleIOTurnData() {
    return new ModuleIOTurnData(
        BaseStatusSignal.isAllGood(allSignals),
        BaseStatusSignal.isAllGood(absolutePosition),
        Rotation2d.fromRotations(absolutePosition.getValueAsDouble()),
        Rotation2d.fromRadians(
            MathUtil.angleModulus(Units.rotationsToRadians(position.getValueAsDouble()))),
        Units.rotationsToRadians(velocity.getValueAsDouble()),
        appliedVolts.getValueAsDouble(),
        supplyCurrentAmps.getValueAsDouble(),
        statorCurrentAmps.getValueAsDouble(),
        Double.NaN);
  }

  @Override
  /**
   * Open-loop steering command using direct voltage control.
   *
   * <p>The input {@code output} is normalized and clamped to [-1, 1], then scaled to bus voltage
   * (±12 V typical) to produce the {@link VoltageOut} setpoint.
   *
   * @param output normalized command in [-1, 1]
   */
  public void runOpenLoop(double output) {
    output = MathUtil.clamp(output, -1.0, 1.0);
    double voltageOut = output * 12.0;
    motor.setControl(voltageRequest.withOutput(voltageOut));
  }

  @Override
  public void runPosition(Rotation2d rotation) {
    motor.setControl(positionVoltageRequest.withPosition(rotation.getRotations()));
  }
}
