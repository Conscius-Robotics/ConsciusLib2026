package com.team10043.lib.subsystems.swerve.module.motors.sim.drive;

import com.team10043.lib.subsystems.swerve.module.ModuleIO.ModuleIOInputs;
import com.team10043.lib.subsystems.swerve.module.motors.DriveMotorIO;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Simulation implementation of {@link DriveMotorIO} using WPILib's {@link DCMotorSim}. Provides
 * physics-based simulation of drive motor behavior with PID control and feedforward for testing
 * without hardware.
 */
public class DriveMotorSim implements DriveMotorIO {

  private final DCMotorSim sim;

  private boolean isClosedLoop = false;

  private final PIDController pidController = new PIDController(0, 0, 0);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);

  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  /**
   * Constructs a simulated drive motor.
   *
   * @param motor DCMotor model (e.g., Kraken X60)
   * @param gearReduction Gear reduction ratio (motor rotations per wheel rotation)
   */
  public DriveMotorSim(DCMotor motor, double gearReduction) {
    sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 0.025, gearReduction), motor);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    if (isClosedLoop) {
      appliedVolts = ffVolts + pidController.calculate(sim.getAngularVelocityRadPerSec());
    } else {
      pidController.reset();
    }

    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(0.02);

    inputs.driveData =
        new ModuleIODriveData(
            true,
            sim.getAngularPositionRad(),
            sim.getAngularVelocityRadPerSec(),
            sim.getInputVoltage(),
            Math.abs(sim.getCurrentDrawAmps()),
            Double.NaN,
            Double.NaN);

    inputs.odometryDrivePositionsRad = new double[] {inputs.driveData.positionRad()};
  }

  @Override
  public void runOpenLoop(double output) {
    isClosedLoop = false;
    appliedVolts = output;
  }

  @Override
  public void runVelocity(double velocityRadPerSec) {
    isClosedLoop = true;
    ffVolts = feedforward.calculate(velocityRadPerSec);
    pidController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setPIDF(double kP, double kI, double kD, double kS, double kV, double kA) {
    pidController.setPID(kP, kI, kD);

    feedforward.setKs(kS);
    feedforward.setKv(kV);
    feedforward.setKa(kA);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    // No-op for simulation
  }
}
