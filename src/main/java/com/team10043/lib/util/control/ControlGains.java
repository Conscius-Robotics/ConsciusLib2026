package com.team10043.lib.util.control;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.Optional;
import lombok.Builder;

/**
 * Unified PID and feedforward configuration records.
 *
 * <p>This class provides a consistent set of records for PID configuration throughout the codebase:
 *
 * <ul>
 *   <li>{@link PIDGains} - Base PID gains (kP, kI, kD) for hardware motor controllers
 *   <li>{@link ProfiledPIDConfig} - Full config for WPILib ProfiledPIDController
 *   <li>{@link FeedforwardGains} - Feedforward gains (kS, kV, kA, etc.)
 * </ul>
 */
public final class ControlGains {

  private ControlGains() {
    // Prevent instantiation
  }

  /**
   * Immutable container for basic PID gains.
   *
   * <p>Used for hardware motor controllers (TalonFX, SparkMax) where gains are sent over CAN.
   *
   * @param kP proportional gain (must be non-negative)
   * @param kI integral gain (must be non-negative)
   * @param kD derivative gain (must be non-negative)
   */
  public record PIDGains(double kP, double kI, double kD) {
    public PIDGains {
      if (kP < 0 || kI < 0 || kD < 0) {
        throw new IllegalArgumentException(
            "PID gains must be non-negative: kP=" + kP + ", kI=" + kI + ", kD=" + kD);
      }
    }

    /** Creates PIDGains with only kP (kI=0, kD=0). */
    public static PIDGains ofP(double kP) {
      return new PIDGains(kP, 0.0, 0.0);
    }

    /** Creates PIDGains with kP and kD (kI=0). */
    public static PIDGains ofPD(double kP, double kD) {
      return new PIDGains(kP, 0.0, kD);
    }

    /** Creates PIDGains with kP, kI, and kD. */
    public static PIDGains ofPID(double kP, double kI, double kD) {
      return new PIDGains(kP, kI, kD);
    }

    /** Creates zero PID gains. */
    public static PIDGains zero() {
      return new PIDGains(0.0, 0.0, 0.0);
    }
  }

  /**
   * Immutable container for feedforward gains.
   *
   * <p>Supports both simple velocity feedforward and arm/elevator feedforward with gravity
   * compensation.
   *
   * @param kS static friction gain (must be non-negative)
   * @param kV velocity gain (must be non-negative)
   * @param kA acceleration gain (must be non-negative)
   * @param kG gravity compensation gain (must be non-negative)
   * @param kCos cosine feedforward gain for arm mechanisms (must be non-negative)
   * @param kCosRatio ratio for cosine calculations
   */
  public record FeedforwardGains(
      double kS,
      double kV,
      double kA,
      Optional<Double> kG,
      Optional<Double> kCos,
      Optional<Double> kCosRatio) {

    public FeedforwardGains {
      if (kS < 0
          || kV < 0
          || kA < 0
          || kG.isPresent() && kG.get() < 0
          || kCos.isPresent() && kCos.get() < 0) {
        throw new IllegalArgumentException(
            String.format(
                "Feedforward gains must be non-negative: kS=%.3f, kV=%.3f, kA=%.3f, kG=%.3f, kCos=%.3f",
                kS, kV, kA, kG, kCos));
      }
    }

    /** Creates FeedforwardGains for simple velocity control (kS, kV only). */
    public static FeedforwardGains ofSV(double kS, double kV) {
      return new FeedforwardGains(
          kS, kV, 0.0, Optional.empty(), Optional.empty(), Optional.empty());
    }

    /** Creates FeedforwardGains for velocity control with acceleration (kS, kV, kA). */
    public static FeedforwardGains ofSVA(double kS, double kV, double kA) {
      return new FeedforwardGains(kS, kV, kA, Optional.empty(), Optional.empty(), Optional.empty());
    }

    /**
     * Creates FeedforwardGains for velocity control with acceleration and gravity compensation (kS,
     * kV, kA, kG).
     */
    public static FeedforwardGains ofSVAG(double kS, double kV, double kA, double kG) {
      return new FeedforwardGains(kS, kV, kA, Optional.of(kG), Optional.empty(), Optional.empty());
    }

    /** Creates zero feedforward gains. */
    public static FeedforwardGains zero() {
      return new FeedforwardGains(
          0.0, 0.0, 0.0, Optional.empty(), Optional.empty(), Optional.empty());
    }
  }

  /**
   * Full configuration for WPILib ProfiledPIDController.
   *
   * <p>Combines PID gains with motion profile constraints and tolerance settings.
   *
   * @param gains PID gains (kP, kI, kD)
   * @param iZone integral zone (0 = full range)
   * @param maxVelocity maximum velocity for motion profile
   * @param maxAcceleration maximum acceleration for motion profile
   * @param tolerance position tolerance for goal detection
   */
  @Builder
  public record ProfiledPIDConfig(
      PIDGains gains, double iZone, double maxVelocity, double maxAcceleration, double tolerance) {

    public ProfiledPIDConfig {
      if (gains == null) {
        throw new IllegalArgumentException("PID gains cannot be null");
      }
      if (maxVelocity < 0.0) {
        throw new IllegalArgumentException("maxVelocity must be non-negative: " + maxVelocity);
      }
      if (maxAcceleration < 0.0) {
        throw new IllegalArgumentException(
            "maxAcceleration must be non-negative: " + maxAcceleration);
      }
      if (tolerance < 0.0) {
        throw new IllegalArgumentException("tolerance must be non-negative: " + tolerance);
      }
    }

    /** Converts to PathPlanner PIDConstants. */
    public PIDConstants toPIDConstants() {
      return new PIDConstants(gains.kP(), gains.kI(), gains.kD(), iZone);
    }

    /** Creates a WPILib ProfiledPIDController from this config. */
    public ProfiledPIDController toProfiledPIDController() {
      ProfiledPIDController controller =
          new ProfiledPIDController(
              gains.kP(),
              gains.kI(),
              gains.kD(),
              new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
      controller.setTolerance(tolerance);
      return controller;
    }

    /** Convenience: get kP directly. */
    public double kP() {
      return gains.kP();
    }

    /** Convenience: get kI directly. */
    public double kI() {
      return gains.kI();
    }

    /** Convenience: get kD directly. */
    public double kD() {
      return gains.kD();
    }
  }

  /**
   * Combined PID and feedforward configuration for hardware motor controllers.
   *
   * <p>Used when configuring motor controller slots that support both PID and feedforward.
   *
   * @param pid PID gains
   * @param ff feedforward gains
   */
  public record PIDFConfig(PIDGains pid, FeedforwardGains ff) {
    public PIDFConfig {
      if (pid == null) {
        throw new IllegalArgumentException("PID gains cannot be null");
      }
      if (ff == null) {
        throw new IllegalArgumentException("Feedforward gains cannot be null");
      }
    }

    /** Creates PIDFConfig with zero feedforward. */
    public static PIDFConfig pidOnly(PIDGains pid) {
      return new PIDFConfig(pid, FeedforwardGains.zero());
    }

    /** Creates zero PIDFConfig. */
    public static PIDFConfig zero() {
      return new PIDFConfig(PIDGains.zero(), FeedforwardGains.zero());
    }

    /** Convenience: get kP directly. */
    public double kP() {
      return pid.kP();
    }

    /** Convenience: get kI directly. */
    public double kI() {
      return pid.kI();
    }

    /** Convenience: get kD directly. */
    public double kD() {
      return pid.kD();
    }

    /** Convenience: get kS directly. */
    public double kS() {
      return ff.kS();
    }

    /** Convenience: get kV directly. */
    public double kV() {
      return ff.kV();
    }

    /** Convenience: get kA directly. */
    public double kA() {
      return ff.kA();
    }

    /** Convenience: get kG directly. */
    public Optional<Double> kG() {
      return ff.kG();
    }

    /** Convenience: get kCos directly. */
    public Optional<Double> kCos() {
      return ff.kCos();
    }

    /** Convenience: get kCosRatio directly. */
    public Optional<Double> kCosRatio() {
      return ff.kCosRatio();
    }
  }
}
