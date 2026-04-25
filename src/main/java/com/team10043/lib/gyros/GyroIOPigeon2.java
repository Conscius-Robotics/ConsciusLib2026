package com.team10043.lib.gyros;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.team10043.frc2026.Constants;
import com.team10043.lib.gyros.config.GyroConfig;
import com.team10043.lib.gyros.config.Pigeon2Config;
import com.team10043.lib.util.phoenix6.CTREUtil;
import com.team10043.lib.util.phoenix6.PhoenixSignalThread;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {

  protected static final double DEFAULT_UPDATE_FREQUENCY_HZ = 50.0;
  public static final double G = 9.80665; // m/s^2

  private final Pigeon2 pigeon;
  private final StatusSignal<Angle> yaw;
  private final StatusSignal<Angle> pitch;
  private final StatusSignal<Angle> roll;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  private final StatusSignal<AngularVelocity> yawVelocity;
  private final StatusSignal<AngularVelocity> pitchVelocity;
  private final StatusSignal<AngularVelocity> rollVelocity;

  private final StatusSignal<LinearAcceleration> accelerationX;
  private final StatusSignal<LinearAcceleration> accelerationY;
  private final StatusSignal<LinearAcceleration> accelerationZ;

  private final BaseStatusSignal[] signals;

  public GyroIOPigeon2(GyroConfig<Pigeon2Config> config) {
    pigeon = new Pigeon2(config.canId.getDeviceNumber(), config.canId.getCANBus());

    yaw = pigeon.getYaw();
    pitch = pigeon.getPitch();
    roll = pigeon.getRoll();
    yawVelocity = pigeon.getAngularVelocityZWorld();
    pitchVelocity = pigeon.getAngularVelocityXWorld();
    rollVelocity = pigeon.getAngularVelocityYWorld();
    accelerationX = pigeon.getAccelerationX();
    accelerationY = pigeon.getAccelerationY();
    accelerationZ = pigeon.getAccelerationZ();

    signals =
        new BaseStatusSignal[] {
          yaw,
          pitch,
          roll,
          yawVelocity,
          pitchVelocity,
          rollVelocity,
          accelerationX,
          accelerationY,
          accelerationZ
        };

    pigeon.getConfigurator().apply(config.config);

    yaw.setUpdateFrequency(config.updateFrequencyHz);
    accelerationX.setUpdateFrequency(Constants.TUNING_MODE ? 100.0 : 4.0);
    accelerationY.setUpdateFrequency(Constants.TUNING_MODE ? 100.0 : 4.0);
    accelerationZ.setUpdateFrequency(Constants.TUNING_MODE ? 100.0 : 4.0);

    yawTimestampQueue = PhoenixSignalThread.getInstance().createTimestampQueue();
    yawPositionQueue = PhoenixSignalThread.getInstance().registerPhoenixSignal(pigeon.getYaw());

    CTREUtil.registerSignals(config.canId.getBusName(), signals);

    CTREUtil.optimizeBusUtilization(pigeon);

    CTREUtil.tryUntilOK(
        () -> pigeon.setYaw(0.0, 0.25), config.canId.getDeviceNumber(), "Set Yaw to 0");
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.data =
        new GyroIOData(
            isConnected(),
            Rotation2d.fromDegrees(yaw.getValueAsDouble()),
            Units.degreesToRadians(yawVelocity.getValueAsDouble()),
            Rotation2d.fromDegrees(pitch.getValueAsDouble()),
            Units.degreesToRadians(pitchVelocity.getValueAsDouble()),
            Rotation2d.fromDegrees(roll.getValueAsDouble()),
            Units.degreesToRadians(rollVelocity.getValueAsDouble()),
            accelerationX.getValueAsDouble() * G,
            accelerationY.getValueAsDouble() * G,
            accelerationZ.getValueAsDouble() * G);

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  @Override
  public void updateFrequency(double frequencyHz) {
    CTREUtil.tryUntilOK(
        () -> BaseStatusSignal.setUpdateFrequencyForAll(frequencyHz, signals),
        pigeon.getDeviceID(),
        "Setting status signal update frequency");
  }

  @Override
  public boolean isConnected() {
    return BaseStatusSignal.isAllGood(signals);
  }
}
