package com.team10043.lib.subsystems.swerve.sysid;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;

/**
 * Computes feedforward constants (kS, kV) using linear regression. Pure math - no robot
 * dependencies.
 */
public class FeedforwardCharacterizationModel {

  private final List<Double> velocitySamples = new ArrayList<>();
  private final List<Double> voltageSamples = new ArrayList<>();

  public void addSample(double velocity, double voltage) {
    velocitySamples.add(velocity);
    voltageSamples.add(voltage);
  }

  public void reset() {
    velocitySamples.clear();
    voltageSamples.clear();
  }

  public FeedforwardConstants calculate() {
    int n = velocitySamples.size();
    if (n == 0) {
      return new FeedforwardConstants(0.0, 0.0);
    }

    double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;

    for (int i = 0; i < n; i++) {
      double x = velocitySamples.get(i);
      double y = voltageSamples.get(i);
      sumX += x;
      sumY += y;
      sumXY += x * y;
      sumX2 += x * x;
    }

    double denominator = n * sumX2 - sumX * sumX;
    if (Math.abs(denominator) < 1e-10) {
      return new FeedforwardConstants(0.0, 0.0);
    }

    double kS = (sumY * sumX2 - sumX * sumXY) / denominator;
    double kV = (n * sumXY - sumX * sumY) / denominator;

    return new FeedforwardConstants(kS, kV);
  }

  public void printResults() {
    FeedforwardConstants constants = calculate();
    NumberFormat f = new DecimalFormat("#0.00000");
    System.out.println("********** Drive FF Characterization **********");
    System.out.println("\tkS: " + f.format(constants.kS()));
    System.out.println("\tkV: " + f.format(constants.kV()));
  }

  public record FeedforwardConstants(double kS, double kV) {}
}
