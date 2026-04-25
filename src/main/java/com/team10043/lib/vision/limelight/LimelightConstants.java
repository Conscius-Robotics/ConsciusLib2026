package com.team10043.lib.vision.limelight;

public class LimelightConstants {

  /** Connection timeout threshold in microseconds */
  public static final long CONNECTION_TIMEOUT_US = 250_000;

  /** 82 deg Horizontal FOV */
  public static final double H_FOV = 82.0;

  /** 56.2 deg Vertical FOV */
  public static final double V_FOV = 56.2;

  /** Immutable configuration record for Limelight camera setup. */
  public record LimelightConfig(String cameraName, int pipelineIndex) {}
}
