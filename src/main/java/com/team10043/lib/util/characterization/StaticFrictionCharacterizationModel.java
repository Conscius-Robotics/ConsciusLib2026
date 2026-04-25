package com.team10043.lib.util.characterization;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;

/**
 * Computes static friction constant (kS) by recording the current when motion is first detected.
 *
 * <p>Pure math model - no robot dependencies.
 */
public class StaticFrictionCharacterizationModel {

  private final List<Double> samples = new ArrayList<>();

  /** Resets the model by clearing all recorded samples. */
  public void reset() {
    samples.clear();
  }

  /** Adds a new current sample to the model. */
  public void addSample(double current) {
    samples.add(current);
  }

  /** Checks if the model has any recorded samples. */
  public boolean hasSamples() {
    return !samples.isEmpty();
  }

  public double getAverage() {
    return samples.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
  }

  /** Prints the results of the static friction characterization. */
  public void printResults(String label, String unit) {
    NumberFormat f = new DecimalFormat("#0.00000");
    System.out.println("********** " + label + " Static Characterization **********");
    System.out.println("\tkS (avg): " + f.format(getAverage()) + " " + unit);
  }
}
