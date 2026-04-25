package com.team10043.lib.subsystems.swerve.module;

import com.team10043.lib.subsystems.swerve.config.SwerveConfig;
import com.team10043.lib.subsystems.swerve.module.motors.sim.drive.DriveMotorSim;
import com.team10043.lib.subsystems.swerve.module.motors.sim.turn.TurnMotorSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;

/**
 * Simulation implementation of {@link ModuleIO} using physics-based motor models. Provides
 * realistic swerve module behavior for testing and development without physical hardware.
 */
public class ModuleIOSim implements ModuleIO {

  private final DriveMotorSim driveMotorSim;
  private final TurnMotorSim turnMotorSim;

  public ModuleIOSim(SwerveConfig<?> config, DCMotor driveMotor, DCMotor turnMotor) {
    driveMotorSim = new DriveMotorSim(driveMotor, config.moduleGearing().driveReduction());
    turnMotorSim = new TurnMotorSim(turnMotor, config.moduleGearing().turnReduction());
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveMotorSim.updateInputs(inputs);
    turnMotorSim.updateInputs(inputs);

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
  }

  @Override
  public void runDriveOpenLoop(double output) {
    driveMotorSim.runOpenLoop(output);
  }

  @Override
  public void runTurnOpenLoop(double output) {
    turnMotorSim.runOpenLoop(output);
  }

  @Override
  public void runDriveVelocity(double velocityRadPerSec) {
    driveMotorSim.runVelocity(velocityRadPerSec);
  }

  @Override
  public void runTurnPosition(edu.wpi.first.math.geometry.Rotation2d rotation) {
    turnMotorSim.runPosition(rotation);
  }

  @Override
  public void setDrivePIDFF(double kP, double kI, double kD, double kS, double kV, double kA) {
    driveMotorSim.setPIDF(kP, kI, kD, kS, kV, kA);
  }

  @Override
  public void setTurnPIDFF(double kP, double kI, double kD, double kS, double kV, double kA) {
    turnMotorSim.setPIDFF(kP, kI, kD, kS, kV, kA);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    driveMotorSim.setBrakeMode(enabled);
    turnMotorSim.setBrakeMode(enabled);
  }
}
