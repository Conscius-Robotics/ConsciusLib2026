package com.team10043.lib.subsystems.swerve.sysid.characterization;

import static edu.wpi.first.units.Units.*;

import com.team10043.lib.subsystems.swerve.Swerve;
import com.team10043.lib.subsystems.swerve.telemetry.SwerveLogger;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * System identification (SysId) routine for swerve drive characterization. Provides commands to
 * measure drive motor feedforward characteristics (kS, kV, kA) using WPILib's SysIdRoutine.
 */
public class SwerveSysId {
  private final SysIdRoutine routine;

  /**
   * Constructs a new SysId routine with default configuration: 0.25V/s ramp rate, 7V max, 10s
   * timeout.
   *
   * @param drive Swerve subsystem to characterize
   * @param logger Logger for state tracking
   */
  public SwerveSysId(Swerve drive, SwerveLogger logger) {
    this.routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.25).per(Second),
                Volts.of(7),
                Time.ofBaseUnits(10, Seconds),
                state -> logger.logSysIdState(state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> drive.runDriveCharacterization(voltage.in(Volts)), null, drive));
  }

  /**
   * Creates a quasistatic characterization command. Slowly ramps voltage to measure kS and kV.
   *
   * @param direction Forward or reverse direction
   * @return Command to run quasistatic test
   */
  public Command quasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  /**
   * Creates a dynamic characterization command. Applies full voltage step to measure kA.
   *
   * @param direction Forward or reverse direction
   * @return Command to run dynamic test
   */
  public Command dynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }
}
