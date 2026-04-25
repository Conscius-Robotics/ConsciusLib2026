package com.team10043.lib.subsystems.motor.commands;

import com.team10043.lib.subsystems.motor.MotorSubsystem;
import com.team10043.lib.util.characterization.StaticFrictionCharacterizationModel;
import com.team10043.lib.util.control.TunableSysIdManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class MotorCharacterizationCommands {

  private static final TunableSysIdManager staticTestManager;

  static {
    staticTestManager =
        new TunableSysIdManager("SysId/Motor/StaticTest", 1.0, 30.0, 0.01, params -> {});
  }

  /**
   * Initializes tunable parameters for NetworkTables.
   *
   * <p>This method must be called during robot initialization to publish SysId parameters to
   * AdvantageScope. The static initializer runs when this method is invoked.
   */
  public static void initialize() {
    // Method intentionally empty - calling this triggers static initialization
  }

  private MotorCharacterizationCommands() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Finds the static friction current (kS) for a motor.
   *
   * <p>Ramps current on a motor while measuring velocity. When velocity exceeds the tolerance, the
   * applied current is recorded as kS.
   *
   * <p><b>Parameters are tunable via NetworkTables at {@code /Tuning/SysId/Motor/StaticTest/}.</b>
   * Defaults are: rampRate=1.0 A/s, maxCurrent=30.0 A, tolerance=0.01.
   *
   * @param invert whether to run in reverse direction
   * @param motorSubsystem the motor subsystem to characterize
   * @return a command that runs the static friction test
   */
  public static Command staticTest(boolean invert, MotorSubsystem<?, ?> motorSubsystem) {

    Timer timer = new Timer();
    StaticFrictionCharacterizationModel model = new StaticFrictionCharacterizationModel();

    int multiplier = invert ? -1 : 1;

    return Commands.runOnce(staticTestManager::update)
        .andThen(
            Commands.run(
                () -> {
                  if (!timer.isRunning()) {
                    timer.restart();
                  }

                  var params = staticTestManager.get();
                  double current =
                      Math.min(timer.get() * params.rampRate(), params.maxCurrent()) * multiplier;
                  motorSubsystem.setTorqueCurrentOutput(current);

                  if (Math.abs(motorSubsystem.getCurrentVelocity()) > params.tolerance()
                      && !model.hasSamples()) {

                    model.addSample(motorSubsystem.getInputsData().torqueCurrentAmps());
                    motorSubsystem.setTorqueCurrentOutput(0);
                    model.printResults("Motor", "A");
                  }
                },
                motorSubsystem))
        .finallyDo(
            interrupted -> {
              motorSubsystem.setTorqueCurrentOutput(0);
              timer.stop();
              model.reset();
              model.printResults("Motor", "A");
            })
        .until(model::hasSamples);
  }
}
