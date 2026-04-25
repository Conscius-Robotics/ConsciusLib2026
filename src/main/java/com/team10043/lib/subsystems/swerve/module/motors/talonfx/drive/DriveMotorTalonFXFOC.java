package com.team10043.lib.subsystems.swerve.module.motors.talonfx.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.team10043.lib.subsystems.swerve.config.SwerveConfig;
import com.team10043.lib.subsystems.swerve.config.SwerveConfig.ModulePosition;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import lombok.Getter;

/**
 * Drive motor implementation using Phoenix 6 Field-Oriented Control (FOC).
 *
 * <p>Control strategy:
 *
 * <ul>
 *   <li>Open-loop uses {@link TorqueCurrentFOC} (requested torque current, in motor amps).
 *   <li>Closed-loop velocity uses {@link VelocityTorqueCurrentFOC} with velocity specified in
 *       mechanism rotations per second (converted from radians).
 * </ul>
 *
 * <p>Additional telemetry:
 *
 * <ul>
 *   <li>Reports {@link #torqueCurrentAmps} and includes it in control signals for bus scheduling.
 *   <li>All signal update rates are finalized based on {@link SwerveConfig#loopFrequencies()}.
 * </ul>
 */
@Getter
public class DriveMotorTalonFXFOC extends DriveMotorTalonFX {

  private final StatusSignal<Current> torqueCurrentAmps;

  private final TorqueCurrentFOC torqueCurrentRequest =
      new TorqueCurrentFOC(0.0).withUpdateFreqHz(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0);

  private final double maxTorqueCurrent;

  /**
   * Constructs a TalonFX drive motor with configuration from the swerve config.
   *
   * @param config Swerve configuration containing motor IDs, CAN bus, gear ratios, and current
   *     limits
   * @param modulePosition Module position for config lookup (FL, FR, BL, BR)
   */
  public DriveMotorTalonFXFOC(SwerveConfig<?> config, ModulePosition modulePosition) {
    super(config, modulePosition);

    this.maxTorqueCurrent = config.moduleCurrentLimits().driveCurrentLimit();

    torqueCurrentAmps = motor.getTorqueCurrent();
    controlSignals.add(torqueCurrentAmps);

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
        torqueCurrentAmps.getValueAsDouble());
  }

  @Override
  /**
   * Open-loop drive command using FOC torque current.
   *
   * <p>The input {@code output} is normalized and clamped to [-1, 1], then scaled by the configured
   * drive current limit to produce a torque current setpoint (amps).
   *
   * @param output normalized command in [-1, 1]
   */
  public void runOpenLoop(double output) {
    output = MathUtil.clamp(output, -1.0, 1.0);
    double torqueCurrent = output * maxTorqueCurrent;
    motor.setControl(torqueCurrentRequest.withOutput(torqueCurrent));
  }

  @Override
  public void runVelocity(double velocityRadPerSec) {
    motor.setControl(
        velocityTorqueCurrentRequest.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
  }
}
