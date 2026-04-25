package com.team10043.lib.subsystems.swerve.module.motors.talonfx.turn;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.team10043.lib.subsystems.swerve.config.SwerveConfig;
import com.team10043.lib.subsystems.swerve.config.SwerveConfig.ModulePosition;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import lombok.Getter;

/**
 * Steering (turn) motor implementation using Phoenix 6 Field-Oriented Control (FOC).
 *
 * <p>Control strategy:
 *
 * <ul>
 *   <li>Open-loop uses {@link TorqueCurrentFOC} (requested torque current in amps).
 *   <li>Closed-loop position uses {@link PositionTorqueCurrentFOC} with targets given in mechanism
 *       rotations (converted from {@link Rotation2d}).
 * </ul>
 *
 * <p>Telemetry:
 *
 * <ul>
 *   <li>Adds {@link #torqueCurrentAmps} to the control signal set.
 *   <li>Reports absolute CANcoder health separately via {@link BaseStatusSignal#isAllGood}.
 * </ul>
 */
@Getter
public class TurnMotorTalonFXFOC extends TurnMotorTalonFX {

  private final StatusSignal<Current> torqueCurrentAmps;

  private final TorqueCurrentFOC torqueCurrentRequest =
      new TorqueCurrentFOC(0.0).withUpdateFreqHz(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0);

  private final double maxTorqueCurrent;

  /**
   * Constructs a TalonFX turn motor with CANcoder absolute position feedback.
   *
   * @param config Swerve configuration containing motor IDs, encoder settings, CAN bus, and current
   *     limits
   * @param modulePosition Module position for config lookup (FL, FR, BL, BR)
   */
  public TurnMotorTalonFXFOC(SwerveConfig<?> config, ModulePosition modulePosition) {
    super(config, modulePosition);

    this.maxTorqueCurrent = config.moduleCurrentLimits().turnCurrentLimit();

    torqueCurrentAmps = motor.getTorqueCurrent();
    controlSignals.add(torqueCurrentAmps);

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
        torqueCurrentAmps.getValueAsDouble());
  }

  @Override
  /**
   * Open-loop steering command using FOC torque current.
   *
   * <p>The input {@code output} is normalized and clamped to [-1, 1], then scaled by the configured
   * turn current limit to produce a torque current setpoint (amps).
   *
   * @param output normalized command in [-1, 1]
   */
  public void runOpenLoop(double output) {
    output = MathUtil.clamp(output, -1.0, 1.0);
    double torqueCurrent = output * maxTorqueCurrent;
    motor.setControl(torqueCurrentRequest.withOutput(torqueCurrent));
  }

  @Override
  public void runPosition(Rotation2d rotation) {
    motor.setControl(positionTorqueCurrentRequest.withPosition(rotation.getRotations()));
  }
}
