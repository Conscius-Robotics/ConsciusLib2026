package com.team10043.lib.subsystems.swerve.module;

import com.team10043.lib.subsystems.swerve.module.motors.DriveMotorIO;
import com.team10043.lib.subsystems.swerve.module.motors.TurnMotorIO;
import com.team10043.lib.util.phoenix6.PhoenixSignalThread;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Queue;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

/**
 * Hardware implementation of {@link ModuleIO} for physical robot operation. Coordinates drive and
 * turn motor controllers, manages high-frequency odometry sampling via Phoenix signal thread, and
 * executes brake mode changes asynchronously to prevent blocking.
 */
public class ModuleIOHardware implements ModuleIO {

  private final DriveMotorIO driveMotor;
  private final TurnMotorIO turnMotor;

  private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(8);

  private final Queue<Double> timestampQueue;

  public ModuleIOHardware(DriveMotorIO driveMotor, TurnMotorIO turnMotor) {
    this.driveMotor = driveMotor;
    this.turnMotor = turnMotor;

    timestampQueue = PhoenixSignalThread.getInstance().createTimestampQueue();
  }

  @Override
  public void updateInputs(ModuleIO.ModuleIOInputs inputs) {
    turnMotor.updateInputs(inputs);
    driveMotor.updateInputs(inputs);

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    timestampQueue.clear();
  }

  @Override
  public void runDriveOpenLoop(double output) {
    driveMotor.runOpenLoop(output);
  }

  @Override
  public void runTurnOpenLoop(double output) {
    turnMotor.runOpenLoop(output);
  }

  @Override
  public void runDriveVelocity(double velocityRadPerSec) {
    driveMotor.runVelocity(velocityRadPerSec);
  }

  @Override
  public void runTurnPosition(Rotation2d rotation) {
    turnMotor.runPosition(rotation);
  }

  @Override
  public void setDrivePIDFF(double kP, double kI, double kD, double kS, double kV, double kA) {
    driveMotor.setPIDF(kP, kI, kD, kS, kV, kA);
  }

  @Override
  public void setTurnPIDFF(double kP, double kI, double kD, double kS, double kV, double kA) {
    turnMotor.setPIDFF(kP, kI, kD, kS, kV, kA);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    brakeModeExecutor.execute(() -> turnMotor.setBrakeMode(enabled));

    brakeModeExecutor.execute(() -> driveMotor.setBrakeMode(enabled));
  }
}
