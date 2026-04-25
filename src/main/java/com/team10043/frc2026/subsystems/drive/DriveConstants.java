package com.team10043.frc2026.subsystems.drive;

public final class DriveConstants {
  public static final DriveConstants config = new DriveConstants();

  public DerivedLimits derivedLimits() {
    return new DerivedLimits();
  }

  public static final class DerivedLimits {
    public double maxLinearSpeed() { return 1.0; }
    public double maxAngularSpeed() { return 1.0; }
  }
}
