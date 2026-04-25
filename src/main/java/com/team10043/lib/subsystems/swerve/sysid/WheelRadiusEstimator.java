package com.team10043.lib.subsystems.swerve.sysid;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.text.DecimalFormat;
import java.text.NumberFormat;

/**
 * Estimates wheel radius from gyro rotation and wheel position data. Pure calculation - minimal
 * dependencies.
 */
public class WheelRadiusEstimator {

  private double[] startPositions;
  private Rotation2d lastRotation;
  private double accumulatedGyroRadians;

  public void start(double[] initialPositions, Rotation2d initialRotation) {
    this.startPositions = initialPositions.clone();
    this.lastRotation = initialRotation;
    this.accumulatedGyroRadians = 0.0;
  }

  public void update(Rotation2d currentRotation) {
    accumulatedGyroRadians += Math.abs(currentRotation.minus(lastRotation).getRadians());
    lastRotation = currentRotation;
  }

  public double calculateRadius(double[] endPositions, double driveBaseRadius) {
    double wheelDelta = 0.0;
    for (int i = 0; i < 4; i++) {
      wheelDelta += Math.abs(endPositions[i] - startPositions[i]) / 4.0;
    }

    if (Math.abs(wheelDelta) < 1e-10) {
      return 0.0;
    }

    return (accumulatedGyroRadians * driveBaseRadius) / wheelDelta;
  }

  public void printResults(double[] endPositions, double driveBaseRadius) {
    double wheelDelta = 0.0;
    for (int i = 0; i < 4; i++) {
      wheelDelta += Math.abs(endPositions[i] - startPositions[i]) / 4.0;
    }

    double radius = calculateRadius(endPositions, driveBaseRadius);

    NumberFormat f = new DecimalFormat("#0.000");
    System.out.println("********** Wheel Radius Characterization **********");
    System.out.println("\tWheel Delta: " + f.format(wheelDelta) + " rad");
    System.out.println("\tGyro Delta: " + f.format(accumulatedGyroRadians) + " rad");
    System.out.println(
        "\tWheel Radius: "
            + f.format(radius)
            + " m ("
            + f.format(Units.metersToInches(radius))
            + " in)");
  }
}
