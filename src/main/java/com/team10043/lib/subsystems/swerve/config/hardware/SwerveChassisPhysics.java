package com.team10043.lib.subsystems.swerve.config.hardware;

/**
 * Immutable container for robot physical properties.
 *
 * <p>Stores physical characteristics of the robot used for physics-based calculations, simulations,
 * and advanced control algorithms. These values should be measured or calculated based on the
 * actual robot's design and weight distribution.
 *
 * @param robotMass the total mass of the robot (in kilograms)
 * @param robotMOI the moment of inertia of the robot around its vertical axis (in kg·m²)
 * @param wheelCof the coefficient of friction between the wheels and the playing surface
 */
public record SwerveChassisPhysics(double robotMass, double robotMOI, double wheelCof) {

  /**
   * Factory method to create a SwerveChassisPhysics instance.
   *
   * @param robotMass the total mass of the robot (in kilograms)
   * @param robotMOI the moment of inertia of the robot around its vertical axis (in kg·m²)
   * @param wheelCof the coefficient of friction between the wheels and the playing surface
   * @return a new SwerveChassisPhysics instance
   */
  public static SwerveChassisPhysics of(double robotMass, double robotMOI, double wheelCof) {
    return new SwerveChassisPhysics(robotMass, robotMOI, wheelCof);
  }
}
