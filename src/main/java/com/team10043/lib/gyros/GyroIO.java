package com.team10043.lib.gyros;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction interface for a robot gyroscope.
 *
 * <p>This interface defines the data contract between the gyroscope hardware implementation and the
 * rest of the robot code. Implementations are expected to populate {@link GyroIOInputs} with the
 * latest sensor readings each cycle.
 *
 * <p>Designed for use with AdvantageKit logging via {@link AutoLog}.
 */
public interface GyroIO {

  /**
   * Container for all gyroscope inputs to be logged and consumed by subsystems.
   *
   * <p>Includes absolute orientation (yaw, pitch, roll), angular velocities, connection status, and
   * optional historical yaw samples for odometry.
   */
  @AutoLog
  public class GyroIOInputs {

    /** Current gyroscope state data. */
    public GyroIOData data =
        new GyroIOData(
            false, Rotation2d.kZero, 0, Rotation2d.kZero, 0, Rotation2d.kZero, 0, 0, 0, 0);

    /** Timestamps (in seconds) corresponding to odometry yaw samples. */
    public double[] odometryYawTimestamps = new double[] {};

    /** Yaw positions used for odometry integration. */
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  /**
   * Immutable snapshot of gyroscope measurements.
   *
   * @param connected whether the gyro is currently connected and reporting
   * @param yawPosition current yaw angle
   * @param yawVelocityRadPerSec yaw angular velocity in radians per second
   * @param pitchPosition current pitch angle
   * @param pitchVelocityRadPerSec pitch angular velocity in radians per second
   * @param rollPosition current roll angle
   * @param rollVelocityRadPerSec roll angular velocity in radians per second
   * @param accelerationX linear acceleration along the X axis in meters per second squared
   * @param accelerationY linear acceleration along the Y axis in meters per second squared
   * @param accelerationZ linear acceleration along the Z axis in meters per second squared
   */
  record GyroIOData(
      boolean connected,
      Rotation2d yawPosition,
      double yawVelocityRadPerSec,
      Rotation2d pitchPosition,
      double pitchVelocityRadPerSec,
      Rotation2d rollPosition,
      double rollVelocityRadPerSec,
      double accelerationX,
      double accelerationY,
      double accelerationZ) {}

  /**
   * Updates the provided input structure with the latest gyroscope data.
   *
   * <p>Implementations should overwrite all relevant fields in {@code inputs} during each periodic
   * update.
   *
   * @param inputs input container to populate
   */
  public default void updateInputs(GyroIOInputs inputs) {}

  /**
   * Updates the CANcoder status signal update frequency.
   *
   * @param hz desired update rate in Hertz
   */
  public default void updateFrequency(double hz) {}

  /**
   * Checks whether the canbus is connected.
   *
   * @return true if connected, false otherwise
   */
  public default boolean isConnected() {
    return true;
  }
}
