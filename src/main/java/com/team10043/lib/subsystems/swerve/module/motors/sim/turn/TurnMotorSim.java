package com.team10043.lib.subsystems.swerve.module.motors.sim.turn;

import com.team10043.lib.subsystems.swerve.module.ModuleIO.ModuleIOInputs;
import com.team10043.lib.subsystems.swerve.module.motors.TurnMotorIO;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Simulation implementation of {@link TurnMotorIO} using WPILib's {@link DCMotorSim}. Provides
 * physics-based simulation of turn motor behavior with continuous PID control for azimuth
 * positioning.
 */
public class TurnMotorSim implements TurnMotorIO {

  private final DCMotorSim sim;

  private boolean isClosedLoop = false;

  private final PIDController pidController = new PIDController(0, 0, 0);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);

  private double appliedVolts = 0.0;

  /**
   * Constructs a simulated turn motor with continuous PID control.
   *
   * @param motor DCMotor model (e.g., Kraken X60, NEO)
   * @param gearReduction Gear reduction ratio (motor rotations per module rotation)
   */
  public TurnMotorSim(DCMotor motor, double gearReduction) {
    sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 0.004, gearReduction), motor);

    pidController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    if (isClosedLoop) {
      double desiredVelocityRadPerSec = pidController.calculate(sim.getAngularPositionRad());

      double ffVolts = feedforward.calculate(desiredVelocityRadPerSec);

      appliedVolts = desiredVelocityRadPerSec + ffVolts;
    } else {
      pidController.reset();
    }

    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(0.02);

    inputs.turnData =
        new ModuleIOTurnData(
            true,
            true,
            new Rotation2d(sim.getAngularPositionRad()),
            new Rotation2d(sim.getAngularPositionRad()),
            sim.getAngularVelocityRadPerSec(),
            sim.getInputVoltage(),
            Math.abs(sim.getCurrentDrawAmps()),
            Double.NaN,
            Double.NaN);

    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnData.position()};
  }

  @Override
  public void runOpenLoop(double output) {
    isClosedLoop = false;
    appliedVolts = output;
  }

  @Override
  public void runPosition(Rotation2d rotation) {
    isClosedLoop = true;
    pidController.setSetpoint(rotation.getRadians());
  }

  @Override
  public void setPIDFF(double kP, double kI, double kD, double kS, double kV, double kA) {
    pidController.setPID(kP, kI, kD);
    pidController.enableContinuousInput(-Math.PI, Math.PI);

    feedforward.setKs(kS);
    feedforward.setKv(kV);
    feedforward.setKa(kA);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    // No-op for simulation
  }
}
