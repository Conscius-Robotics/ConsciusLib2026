package com.team10043.lib.subsystems.swerve.estimator;

import com.team10043.frc2026.Constants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Manages odometry updates for the swerve drive. Handles pose estimation using gyro and module
 * positions.
 */
public class SwervePoseEstimator {
  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;

  private final Supplier<double[]> odometryTimestampsSupplier;
  private final Supplier<SwerveModulePosition[][]> odometrySamplesSupplier;
  private final Supplier<SwerveModulePosition[]> currentPositionsSupplier;

  private Rotation2d rawGyroRotation;
  private SwerveModulePosition[] lastModulePositions;

  private Field2d field2d = new Field2d();

  public SwervePoseEstimator(
      SwerveDriveKinematics kinematics,
      Supplier<double[]> odometryTimestampsSupplier,
      Supplier<SwerveModulePosition[][]> odometrySamplesSupplier,
      Supplier<SwerveModulePosition[]> currentPositionsSupplier) {

    this.kinematics = kinematics;
    this.odometryTimestampsSupplier = odometryTimestampsSupplier;
    this.odometrySamplesSupplier = odometrySamplesSupplier;
    this.currentPositionsSupplier = currentPositionsSupplier;

    this.rawGyroRotation = new Rotation2d();
    this.lastModulePositions = createEmptyModulePositions();

    this.poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    SmartDashboard.putData("Field", field2d);
  }

  private Rotation2d[] gyroRotations = null;

  /**
   * Updates the gyro rotations from the latest gyro readings. Call this before update() if gyro is
   * connected.
   */
  public void updateGyroRotations(Rotation2d[] rotations) {
    this.gyroRotations = rotations;
  }

  public void update() {
    update(odometryTimestampsSupplier.get(), odometrySamplesSupplier.get());

    field2d.setRobotPose(getEstimatedPose());
  }

  /**
   * Updates odometry with new samples from modules. Uses previously set gyro rotations if
   * available.
   *
   * @param sampleTimestamps Array of timestamps for each sample
   * @param modulePositionSamples 2D array [sampleIndex][moduleIndex] of positions
   */
  public void update(double[] sampleTimestamps, SwerveModulePosition[][] modulePositionSamples) {

    boolean gyroConnected = (gyroRotations != null);
    int sampleCount = sampleTimestamps.length;

    for (int sampleIndex = 0; sampleIndex < sampleCount; sampleIndex++) {
      SwerveModulePosition[] currentPositions = modulePositionSamples[sampleIndex];
      SwerveModulePosition[] deltas = calculateModuleDeltas(currentPositions);

      if (gyroConnected) {
        rawGyroRotation = gyroRotations[sampleIndex];
      } else {
        rawGyroRotation = estimateRotationFromModules(deltas);
      }

      poseEstimator.updateWithTime(
          sampleTimestamps[sampleIndex], rawGyroRotation, currentPositions);

      lastModulePositions = currentPositions;
    }

    gyroRotations = null;

    double distance =
        getEstimatedPose()
            .getTranslation()
            .getDistance(Constants.getHubCenterForAlliance().getTranslation());

    Logger.recordOutput("HubDistance", distance);
  }

  /** Calculates the change in position for each module since last update. */
  private SwerveModulePosition[] calculateModuleDeltas(SwerveModulePosition[] currentPositions) {
    SwerveModulePosition[] deltas = new SwerveModulePosition[4];

    for (int i = 0; i < 4; i++) {
      double deltaDistance =
          currentPositions[i].distanceMeters - lastModulePositions[i].distanceMeters;
      deltas[i] = new SwerveModulePosition(deltaDistance, currentPositions[i].angle);
    }

    return deltas;
  }

  /** Estimates rotation change using module positions when gyro is disconnected. */
  private Rotation2d estimateRotationFromModules(SwerveModulePosition[] deltas) {
    Twist2d twist = kinematics.toTwist2d(deltas);
    return rawGyroRotation.plus(new Rotation2d(twist.dtheta));
  }

  /** Adds a vision measurement to the pose estimator. */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp, stdDevs);
  }

  public void setPose(Pose2d pose) {
    resetPose(pose, currentPositionsSupplier.get());
  }

  /** Resets the pose estimator to a new pose. */
  public void resetPose(Pose2d newPose, SwerveModulePosition[] modulePositions) {
    poseEstimator.resetPosition(rawGyroRotation, modulePositions, newPose);
    lastModulePositions = modulePositions;
  }

  public void resetYaw() {
    setPose(
        new Pose2d(
            getEstimatedPose().getTranslation(),
            DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                ? Rotation2d.fromDegrees(0)
                : Rotation2d.fromDegrees(180)));
  }

  /** Gets the current estimated pose. */
  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Gets the current estimated rotation. */
  public Rotation2d getEstimatedRotation() {
    return getEstimatedPose().getRotation();
  }

  /** Gets the current gyro rotation (raw, not from pose estimator). */
  public Rotation2d getRawGyroRotation() {
    return rawGyroRotation;
  }

  private SwerveModulePosition[] createEmptyModulePositions() {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
    };
  }
}
