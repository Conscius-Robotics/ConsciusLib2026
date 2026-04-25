package com.team10043.lib.subsystems.motor.components.follower;

import com.team10043.lib.encoders.config.EncoderConfiguration;
import com.team10043.lib.motors.MotorIO;
import com.team10043.lib.motors.MotorIOInputsAutoLogged;
import com.team10043.lib.motors.config.MotorConfig;
import com.team10043.lib.motors.config.MotorConfiguration;
import com.team10043.lib.motors.config.SimulationConfig;
import com.team10043.lib.motors.sim.MotorIOSim;
import com.team10043.lib.motors.sim.MotorIOSimArm;
import com.team10043.lib.motors.sim.MotorIOSimFlywheel;
import com.team10043.lib.subsystems.motor.MotorSubsystemConfig;
import com.team10043.lib.subsystems.motor.MotorSubsystemConfig.FollowerConfig;
import com.team10043.lib.util.DebouncedAlert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;

/**
 * FollowerComponent manages one or more motors configured to follow a leader motor.
 *
 * <p>This component:
 *
 * <ul>
 *   <li>Configures follower motors
 *   <li>Reads and logs follower motor inputs
 *   <li>Provides averaged position, velocity, and current data
 * </ul>
 *
 * <p>All motors are assumed to be mechanically linked.
 *
 * @param <T> Motor configuration type
 * @param <U> Encoder configuration type
 */
public class ActiveFollowerComponent<T extends MotorConfiguration, U extends EncoderConfiguration>
    implements FollowerComponent {

  private List<MotorIO> ios;
  private List<MotorIOInputsAutoLogged> inputs;
  private List<DebouncedAlert> motorDisconnectedAlerts;

  /**
   * Creates a follower component and configures all follower motors.
   *
   * @param ios follower motor IO instances
   * @param config motor configuration
   */
  public ActiveFollowerComponent(MotorSubsystemConfig<T, U> config) {
    ios = new ArrayList<>();
    inputs = new ArrayList<>();
    motorDisconnectedAlerts = new ArrayList<>();

    setupFollowers(config);
  }

  private void setupFollowers(MotorSubsystemConfig<T, U> config) {
    for (int i = 0; i < config.followers.size(); i++) {
      addFollower(config, i);
    }
  }

  private void addFollower(MotorSubsystemConfig<T, U> config, int index) {
    FollowerConfig<T> follower = config.followers.get(index);

    inputs.add(new MotorIOInputsAutoLogged());

    MotorIO io = createAndAttachFollowerIO(follower, config.motorConfig);
    ios.add(io);

    DebouncedAlert alert = createDisconnectAlert(config.systemName, index);
    motorDisconnectedAlerts.add(alert);
  }

  private MotorIO createAndAttachFollowerIO(
      FollowerConfig<T> follower, MotorConfig<T> leaderMotorConfig) {

    MotorIO io = createFollowerIO(follower, leaderMotorConfig);
    io.follow(leaderMotorConfig.canId, follower.inverted);
    return io;
  }

  /**
   * Creates the follower motor IO, ensuring simulation followers use the leader's simulation config
   * so their physics types match for proper leader-follower linking.
   */
  private MotorIO createFollowerIO(FollowerConfig<T> follower, MotorConfig<T> leaderMotorConfig) {
    if (RobotBase.isSimulation()) {
      return createSimFollowerIO(follower.motor, leaderMotorConfig.simulationConfig);
    }
    return follower.motor.createIO();
  }

  /**
   * Creates a simulation follower IO using the leader's simulation config to ensure the same
   * physics type is used for both leader and follower. This is critical for {@link
   * #linkSimulationFollowers(MotorIO)} to successfully establish the leader-follower relationship.
   */
  private MotorIO createSimFollowerIO(
      MotorConfig<T> followerMotorConfig, SimulationConfig leaderSimConfig) {
    return switch (leaderSimConfig.mechanismType()) {
      case ARM -> new MotorIOSimArm(followerMotorConfig);
      case FLYWHEEL -> new MotorIOSimFlywheel(followerMotorConfig);
      default -> new MotorIOSim(followerMotorConfig);
    };
  }

  /**
   * Links simulation follower motors to their leader for physics mirroring.
   *
   * <p>In simulation, follower motors need a reference to the leader's physics simulation to mirror
   * its state. This method establishes that connection after all motors are created.
   *
   * @param leaderIO the leader motor IO instance
   */
  public void linkSimulationFollowers(MotorIO leaderIO) {

    if (!RobotBase.isSimulation()) {
      return;
    }

    for (MotorIO followerIO : ios) {

      boolean linked = false;

      if (followerIO instanceof MotorIOSim && leaderIO instanceof MotorIOSim) {
        ((MotorIOSim) followerIO).setLeader((MotorIOSim) leaderIO);
        linked = true;
      } else if (followerIO instanceof MotorIOSimArm && leaderIO instanceof MotorIOSimArm) {
        ((MotorIOSimArm) followerIO).setLeader((MotorIOSimArm) leaderIO);
        linked = true;
      } else if (followerIO instanceof MotorIOSimFlywheel
          && leaderIO instanceof MotorIOSimFlywheel) {
        ((MotorIOSimFlywheel) followerIO).setLeader((MotorIOSimFlywheel) leaderIO);
        linked = true;
      }

      // If we couldn't link the follower to the leader, emit a warning so configuration
      // problems don't fail silently and later cause a NullPointerException inside the
      // simulation motor IO.
      if (!linked) {
        DriverStation.reportWarning(
            "Simulation follower motor of type "
                + followerIO.getClass().getSimpleName()
                + " could not be linked to leader of type "
                + leaderIO.getClass().getSimpleName()
                + ". Follower will not mirror leader in simulation.",
            false);
      }
    }
  }

  private DebouncedAlert createDisconnectAlert(String systemName, int index) {

    return new DebouncedAlert(
        () -> inputs.get(index).data.connected(),
        () -> true,
        String.format("Follower Motor [%s-%d] disconnected.", systemName, index));
  }

  @Override
  public void periodic(String systemName) {
    for (int i = 0; i < ios.size(); i++) {
      ios.get(i).updateInputs(inputs.get(i));
      Logger.processInputs(systemName + "/follower" + i, inputs.get(i));

      motorDisconnectedAlerts.get(i).update();
    }
  }

  @Override
  public double getAveragePosition(double leaderPosition) {
    return getAverageValue(leaderPosition, input -> input.data.positionRots());
  }

  @Override
  public double getAverageVelocity(double leaderVelocity) {
    return getAverageValue(leaderVelocity, input -> input.data.velocityRotPerSec());
  }

  /**
   * Calculates average value across leader and connected followers. Accounts for inverted motors by
   * correcting their sign.
   *
   * @param leaderValue value from the leader motor
   * @param valueExtractor function to extract value from follower input
   * @return average value across all connected motors
   */
  private double getAverageValue(
      double leaderValue, Function<MotorIOInputsAutoLogged, Double> valueExtractor) {

    double sum = leaderValue;
    int connectedCount = 1; // assumes that leader is always connected

    for (int i = 0; i < inputs.size(); i++) {
      if (!inputs.get(i).data.connected()) {
        continue; // Skip disconnected motors
      }

      double value = valueExtractor.apply(inputs.get(i));
      sum += value;
      connectedCount++;
    }

    return sum / connectedCount;
  }

  @Override
  public void setCurrentPositionAsZero() {
    for (var followerIO : ios) {
      followerIO.setCurrentPositionAsZero();
    }
  }

  @Override
  public void setCurrentPosition(double position) {
    for (var followerIO : ios) {
      followerIO.setCurrentPosition(position);
    }
  }

  public List<MotorIOInputsAutoLogged> getInputs() {
    return inputs;
  }

  public int getFollowerCount() {
    return ios.size();
  }

  /**
   * Gets follower input by index. Returns null if index is out of bounds.
   *
   * @param index follower motor index
   * @return follower motor inputs, or null if invalid index
   */
  public MotorIOInputsAutoLogged getFollowerInput(int index) {
    if (index < 0 || index >= inputs.size()) {
      DriverStation.reportWarning(
          "Follower index " + index + " out of bounds. Valid range: 0-" + (inputs.size() - 1),
          false);
      return null;
    }
    return inputs.get(index);
  }

  @Override
  public double getFollowerPosition(int index) {
    if (!isValidIndex(index)) {
      return 0.0;
    }
    double position = inputs.get(index).data.positionRots();
    return position;
  }

  @Override
  public double getFollowerVelocity(int index) {
    if (!isValidIndex(index)) {
      return 0.0;
    }
    double velocity = inputs.get(index).data.velocityRotPerSec();
    return velocity;
  }

  @Override
  public double getFollowerCurrent(int index) {
    if (!isValidIndex(index)) {
      return 0.0;
    }
    return inputs.get(index).data.statorCurrentAmps();
  }

  @Override
  public double getFollowerVoltage(int index) {
    if (!isValidIndex(index)) {
      return 0.0;
    }
    return inputs.get(index).data.appliedVolts();
  }

  /**
   * Validates follower index. Logs warning if invalid.
   *
   * @param index follower index to validate
   * @return true if valid, false otherwise
   */
  private boolean isValidIndex(int index) {
    if (index < 0 || index >= inputs.size()) {
      DriverStation.reportWarning(
          "Follower index " + index + " out of bounds. Valid range: 0-" + (inputs.size() - 1),
          false);
      return false;
    }
    return true;
  }

  @Override
  public double getTotalCurrent(double leaderCurrent) {
    double sum = leaderCurrent;
    for (var input : inputs) {
      sum += input.data.statorCurrentAmps();
    }
    return sum;
  }

  @Override
  public double getAverageCurrent(double leaderCurrent) {
    return getTotalCurrent(leaderCurrent) / (inputs.size() + 1);
  }
}
