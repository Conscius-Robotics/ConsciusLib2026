package com.team10043.lib.subsystems.swerve.module.motors.talonfx.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.team10043.lib.subsystems.swerve.config.SwerveConfig;
import com.team10043.lib.subsystems.swerve.config.SwerveConfig.ModulePosition;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import lombok.Getter;

/**
 * Drive motor implementation using Phoenix 6 voltage-based control.
 *
 * <p>Control strategy:
 *
 * <ul>
 *   <li>Open-loop uses {@link VoltageOut} (direct voltage command in volts).
 *   <li>Closed-loop velocity uses {@link VelocityVoltage} with targets specified in mechanism
 *       rotations per second (converted from radians).
 * </ul>
 *
 * <p>Telemetry focuses on voltage/current; torque current is not reported in this variant. Signal
 * update rates are finalized based on {@link SwerveConfig#loopFrequencies()}.
 */
@Getter
public class DriveMotorTalonFXVoltage extends DriveMotorTalonFX {

  private final VoltageOut voltageRequest = new VoltageOut(0).withUpdateFreqHz(0);
  private final VelocityVoltage velocityVoltageRequest =
      new VelocityVoltage(0.0).withUpdateFreqHz(0);

  /**
   * Constructs a TalonFX drive motor with configuration from the swerve config.
   *
   * @param config Swerve configuration containing motor IDs, CAN bus, gear ratios, and current
   *     limits
   * @param modulePosition Module position for config lookup (FL, FR, BL, BR)
   */
  public DriveMotorTalonFXVoltage(SwerveConfig<?> config, ModulePosition modulePosition) {
    super(config, modulePosition);

    finalizeSignals(config.loopFrequencies());
  }

  @Override
  public ModuleIODriveData getModuleIODriveData() {
    return new ModuleIODriveData(
        BaseStatusSignal.isAllGood(allSignals),
        Units.rotationsToRadians(position.getValueAsDouble()),
        Units.rotationsToRadians(velocity.getValueAsDouble()),
        appliedVolts.getValueAsDouble(),
        supplyCurrentAmps.getValueAsDouble(),
        statorCurrentAmps.getValueAsDouble(),
        Double.NaN);
  }

  @Override
  /**
   * Open-loop drive command using direct voltage control.
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
  public void runVelocity(double velocityRadPerSec) {
    motor.setControl(
        velocityVoltageRequest.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
  }
}
