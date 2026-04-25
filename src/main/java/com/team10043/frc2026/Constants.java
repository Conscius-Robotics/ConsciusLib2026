package com.team10043.frc2026;

import edu.wpi.first.math.geometry.Pose2d;

public final class Constants {
  public enum Mode {
    REAL,
    SIM,
    DEFAULT
  }

  // Runtime mode indicator used by library code
  public static final Mode CURRENT_MODE = Mode.REAL;

  // Tuning mode and HAL disable flags used in library utilities
  public static final boolean TUNING_MODE = false;
  public static final boolean DISABLE_HAL = false;

  // Returns a hub center pose for the current alliance. Library just needs a Pose2d.
  public static Pose2d getHubCenterForAlliance() {
    return new Pose2d();
  }
}
