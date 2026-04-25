package com.team10043.lib.util.controller;

import com.team10043.lib.util.control.LoggedTunableNumber;
import edu.wpi.first.math.MathUtil;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/**
 * Joystick input shaping with configurable sensitivity curves.
 *
 * <p>Provides preset profiles and customizable settings for swerve drive control, enabling
 * fine-tuned driver feel through various curve modes.
 */
public final class JoystickUtil {

  @RequiredArgsConstructor
  @Getter
  public enum JoystickCurveMode {
    /** Slower center response with more aggressive edge sensitivity. */
    CONVEX,

    /** More aggressive center response with softer edge sensitivity. */
    CONCAVE,

    /** Blended S-curve for balanced sensitivity across the range. */
    S_CURVE;
  }

  /**
   * Settings for joystick sensitivity shaping.
   *
   * @param globalSensitivity overall output scale, range [0, 1]
   * @param centerSensitivity center region softness, range [0, 1] (only affects S_CURVE)
   * @param edgeSensitivity edge region aggressiveness, range [1, +inf)
   * @param mode shaping curve mode
   */
  public record JoystickSensitivitySettings(
      double globalSensitivity,
      double centerSensitivity,
      double edgeSensitivity,
      JoystickCurveMode mode) {
    /**
     * Canonical constructor that enforces documented parameter ranges:
     *
     * <ul>
     *   <li>globalSensitivity is clamped to [0, 1]
     *   <li>centerSensitivity is clamped to [0, 1]
     *   <li>edgeSensitivity is constrained to [1, +inf)
     * </ul>
     */
    public JoystickSensitivitySettings {
      globalSensitivity = MathUtil.clamp(globalSensitivity, 0.0, 1.0);
      centerSensitivity = MathUtil.clamp(centerSensitivity, 0.0, 1.0);
      edgeSensitivity = Math.max(edgeSensitivity, 1.0);
    }
  }

  /**
   * Tunable settings for joystick sensitivity shaping with runtime adjustments.
   *
   * <p>Allows real-time tuning of sensitivity parameters via NetworkTables when tuning mode is
   * enabled.
   */
  public static class TunableJoystickSettings {
    private final LoggedTunableNumber globalSensitivity;
    private final LoggedTunableNumber centerSensitivity;
    private final LoggedTunableNumber edgeSensitivity;
    private final JoystickCurveMode mode;

    /**
     * Creates tunable joystick settings.
     *
     * @param prefix NetworkTables key prefix
     * @param defaultGlobal default global sensitivity [0, 1]
     * @param defaultCenter default center sensitivity [0, 1]
     * @param defaultEdge default edge sensitivity [1, +inf)
     * @param mode curve mode
     */
    public TunableJoystickSettings(
        String prefix,
        double defaultGlobal,
        double defaultCenter,
        double defaultEdge,
        JoystickCurveMode mode) {
      this.globalSensitivity =
          new LoggedTunableNumber(prefix + "/GlobalSensitivity", defaultGlobal);
      this.centerSensitivity =
          new LoggedTunableNumber(prefix + "/CenterSensitivity", defaultCenter);
      this.edgeSensitivity = new LoggedTunableNumber(prefix + "/EdgeSensitivity", defaultEdge);
      this.mode = mode;
    }

    /**
     * Creates tunable settings from a static profile.
     *
     * @param prefix NetworkTables key prefix
     * @param baseSettings base settings to use as defaults
     */
    public TunableJoystickSettings(String prefix, JoystickSensitivitySettings baseSettings) {
      this(
          prefix,
          baseSettings.globalSensitivity,
          baseSettings.centerSensitivity,
          baseSettings.edgeSensitivity,
          baseSettings.mode);
    }

    /**
     * @return Current settings snapshot
     */
    public JoystickSensitivitySettings get() {
      return new JoystickSensitivitySettings(
          globalSensitivity.get(), centerSensitivity.get(), edgeSensitivity.get(), mode);
    }
  }

  private static final double DEADBAND = 0.03;

  // #region Preset sensitivity profiles

  /** Balanced S-curve for general driving with moderate center deadening */
  public static final JoystickSensitivitySettings S_CURVE_DRIVE =
      new JoystickSensitivitySettings(1.0, 0.3, 2.5, JoystickCurveMode.S_CURVE);

  /** Aggressive early response for quick maneuvers (centerSensitivity unused for CONCAVE) */
  public static final JoystickSensitivitySettings EARLY_PUNCH =
      new JoystickSensitivitySettings(1.0, 0.0, 3.0, JoystickCurveMode.CONCAVE);

  /** Linear response with no curve modification */
  public static final JoystickSensitivitySettings SAFE_LINEAR =
      new JoystickSensitivitySettings(0.75, 0.0, 1.0, JoystickCurveMode.CONVEX);

  // #endregion

  private JoystickUtil() {
    // prevent instantiation
  }

  /**
   * Shapes joystick input using the specified sensitivity settings.
   *
   * <p>Applies curve transformation based on mode, then scales by global sensitivity. Input is
   * clamped to [-1, 1] and output preserves sign and range.
   *
   * @param input raw joystick value, automatically clamped to [-1, 1]
   * @param deadband deadband value to apply
   * @param settings sensitivity configuration
   * @return shaped output in range [-1, 1]
   */
  public static double shapeJoystick(
      double input, double deadband, JoystickSensitivitySettings settings) {

    input = MathUtil.clamp(input, -1.0, 1.0);
    input = MathUtil.applyDeadband(input, deadband);

    if (input == 0.0) {
      return 0.0;
    }

    // Preserve sign and work with absolute value
    double sign = input >= 0 ? 1.0 : -1.0;
    double x = Math.abs(input);

    double shaped;

    switch (settings.mode) {
      case CONCAVE -> shaped = Math.pow(x, 1.0 / settings.edgeSensitivity);

      case CONVEX -> shaped = Math.pow(x, settings.edgeSensitivity);

      case S_CURVE -> {
        // Blend concave and convex based on centerSensitivity
        double concave = Math.pow(x, 1.0 / settings.edgeSensitivity);
        double convex = Math.pow(x, settings.edgeSensitivity);

        shaped = (1.0 - settings.centerSensitivity) * concave + settings.centerSensitivity * convex;
      }

      default -> shaped = x;
    }

    return sign * shaped * settings.globalSensitivity;
  }

  /**
   * Shapes joystick input using the specified sensitivity settings.
   *
   * <p>Applies curve transformation based on mode, then scales by global sensitivity. Input is
   * clamped to [-1, 1] and output preserves sign and range.
   *
   * @param input raw joystick value, automatically clamped to [-1, 1]
   * @param settings sensitivity configuration
   * @return shaped output in range [-1, 1]
   */
  public static double shapeJoystick(double input, JoystickSensitivitySettings settings) {
    return shapeJoystick(input, DEADBAND, settings);
  }

  /**
   * Shapes joystick input using tunable settings.
   *
   * @param input raw joystick value, automatically clamped to [-1, 1]
   * @param tunableSettings tunable sensitivity configuration
   * @param deadband deadband value to apply
   * @return shaped output in range [-1, 1]
   */
  public static double shapeJoystick(
      double input, double deadband, TunableJoystickSettings tunableSettings) {
    return shapeJoystick(input, deadband, tunableSettings.get());
  }

  /**
   * Shapes joystick input using tunable settings.
   *
   * @param input raw joystick value, automatically clamped to [-1, 1]
   * @param tunableSettings tunable sensitivity configuration
   * @return shaped output in range [-1, 1]
   */
  public static double shapeJoystick(double input, TunableJoystickSettings tunableSettings) {
    return shapeJoystick(input, DEADBAND, tunableSettings.get());
  }
}
