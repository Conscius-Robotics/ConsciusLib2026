package com.team10043.lib.subsystems.swerve.module.diagnostics;

import com.team10043.frc2026.Constants;
import com.team10043.lib.subsystems.swerve.config.hardware.ModuleTuningConfig;
import com.team10043.lib.subsystems.swerve.module.ModuleIO;
import com.team10043.lib.util.control.TunableFFManager;
import com.team10043.lib.util.control.TunablePIDManager;
import java.util.Optional;

/**
 * Handles live-tunable PID and feedforward parameters for a swerve module.
 *
 * <p>This class wires {@link TunablePIDManager} and {@link TunableFFManager} together and applies
 * updated control constants to {@link ModuleIO}. Automatically selects real or simulation tuning
 * values based on robot mode.
 *
 * <p>The class itself contains no tuning logic; it only reacts to changes reported by the tunable
 * managers and pushes the combined values to hardware.
 */
public class ModuleTuning {

  private final ModuleIO io;

  private final TunablePIDManager drivePID;
  private final TunablePIDManager turnPID;
  private final TunableFFManager driveFF;
  private final TunableFFManager turnFF;

  /**
   * Creates a new {@code ModuleTuning} instance.
   *
   * @param io module hardware interface
   * @param tuningConfig tuning configuration (required for both real and sim)
   * @throws IllegalStateException if tuningConfig is null
   */
  public ModuleTuning(ModuleIO io, ModuleTuningConfig tuningConfig) {
    if (tuningConfig == null) {
      throw new IllegalStateException(
          "ModuleTuningConfig must be provided in SwerveModuleConfig. "
              + "Add .tuning(...) to your module config builder.");
    }

    this.io = io;

    // Select real or sim values based on robot mode
    boolean isReal = Constants.CURRENT_MODE == Constants.Mode.REAL;

    drivePID =
        new TunablePIDManager(
            "Swerve/Module/Drive/PID",
            isReal ? tuningConfig.realDrivePID().kP() : tuningConfig.simDrivePID().kP(),
            isReal ? tuningConfig.realDrivePID().kI() : tuningConfig.simDrivePID().kI(),
            isReal ? tuningConfig.realDrivePID().kD() : tuningConfig.simDrivePID().kD(),
            gains -> applyDrive());

    driveFF =
        new TunableFFManager(
            "Swerve/Module/Drive/FF",
            isReal ? tuningConfig.realDriveFF().kS() : tuningConfig.simDriveFF().kS(),
            isReal ? tuningConfig.realDriveFF().kV() : tuningConfig.simDriveFF().kV(),
            isReal ? tuningConfig.realDriveFF().kA() : tuningConfig.simDriveFF().kA(),
            Optional.empty(),
            Optional.empty(),
            Optional.empty(),
            ff -> applyDrive());

    turnPID =
        new TunablePIDManager(
            "Swerve/Module/Turn/PID",
            isReal ? tuningConfig.realTurnPID().kP() : tuningConfig.simTurnPID().kP(),
            isReal ? tuningConfig.realTurnPID().kI() : tuningConfig.simTurnPID().kI(),
            isReal ? tuningConfig.realTurnPID().kD() : tuningConfig.simTurnPID().kD(),
            gains -> applyTurn());

    turnFF =
        new TunableFFManager(
            "Swerve/Module/Turn/FF",
            isReal ? tuningConfig.realTurnFF().kS() : tuningConfig.simTurnFF().kS(),
            isReal ? tuningConfig.realTurnFF().kV() : tuningConfig.simTurnFF().kV(),
            isReal ? tuningConfig.realTurnFF().kA() : tuningConfig.simTurnFF().kA(),
            Optional.empty(),
            Optional.empty(),
            Optional.empty(),
            ff -> applyTurn());

    applyAll();
  }

  /**
   * Checks for updated PID and feedforward values and applies them if changed.
   *
   * <p>This method should be called periodically.
   */
  public void update() {
    drivePID.update();
    driveFF.update();
    turnPID.update();
    turnFF.update();
  }

  /** Applies all current drive PID and feedforward values to the module. */
  private void applyDrive() {
    var pid = drivePID.get();
    var ff = driveFF.get();

    io.setDrivePIDFF(pid.kP(), pid.kI(), pid.kD(), ff.kS(), ff.kV(), ff.kA());
  }

  /** Applies all current turn PID and feedforward values to the module. */
  private void applyTurn() {
    var turn = turnPID.get();
    var ff = turnFF.get();

    io.setTurnPIDFF(turn.kP(), turn.kI(), turn.kD(), ff.kS(), ff.kV(), ff.kA());
  }

  /**
   * Forces all tuning values to be applied to the hardware.
   *
   * <p>Typically used once during initialization.
   */
  private void applyAll() {
    applyDrive();
    applyTurn();
  }
}
