package com.team10043.lib.subsystems.swerve.path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.team10043.lib.subsystems.swerve.Swerve;
import com.team10043.lib.subsystems.swerve.config.SwerveConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.littletonrobotics.junction.Logger;

/**
 * Configures PathPlanner integration for autonomous path following. Sets up AutoBuilder with robot
 * configuration, PID controllers, and pathfinding algorithms.
 */
public class SwervePathPlanner {
  private final Swerve drive;
  private final RobotConfig robotConfig;
  private final SwerveConfig<?> config;

  /**
   * Constructs a new SwervePathPlanner.
   *
   * @param drive Swerve subsystem instance for path control
   * @param config Swerve configuration with chassis dimensions, motor specs, and PID gains
   */
  public SwervePathPlanner(Swerve drive, SwerveConfig<?> config) {
    this.drive = drive;
    this.config = config;

    this.robotConfig =
        new RobotConfig(
            config.chassisPhysics().robotMass(),
            config.chassisPhysics().robotMOI(),
            new ModuleConfig(
                config.chassisDimensions().wheelRadius(),
                config.derivedLimits().maxLinearSpeed(),
                config.chassisPhysics().wheelCof(),
                DCMotor.getKrakenX60(1).withReduction(config.moduleGearing().driveReduction()),
                config.moduleCurrentLimits().driveCurrentLimit(),
                1),
            config.chassisDimensions().moduleTranslations());
  }

  /**
   * Configures PathPlanner AutoBuilder with the swerve drive subsystem. Sets up holonomic
   * controller, alliance flipping, local pathfinding, and trajectory logging.
   *
   * <p>Call this method once during robot initialization or in drive train subsystem.
   */
  public void configure() {
    AutoBuilder.configure(
        drive.poseEstimator()::getEstimatedPose,
        drive.poseEstimator()::setPose,
        drive::getChassisSpeeds,
        drive::runVelocity,
        new PPHolonomicDriveController(
            config.translationPID().toPIDConstants(), config.rotationPID().toPIDConstants()),
        robotConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        drive);

    Pathfinding.setPathfinder(new LocalADStarAK());

    PathPlannerLogging.setLogActivePathCallback(
        (path) -> Logger.recordOutput("Odometry/Trajectory", path.toArray(new Pose2d[0])));

    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> Logger.recordOutput("Odometry/TrajectorySetpoint", pose));
  }
}
