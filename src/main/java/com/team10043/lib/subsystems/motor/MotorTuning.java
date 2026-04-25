package com.team10043.lib.subsystems.motor;

import com.team10043.lib.motors.MotorIO;
import com.team10043.lib.util.control.ControlGains.PIDFConfig;
import com.team10043.lib.util.control.TunableFFManager;
import com.team10043.lib.util.control.TunablePIDManager;

public class MotorTuning {

  private final TunablePIDManager pidManager;
  private final TunableFFManager ffManager;

  private final MotorIO io;

  public MotorTuning(MotorIO io, PIDFConfig defaults, String path) {
    this.io = io;

    pidManager =
        new TunablePIDManager(
            path + "/PID", defaults.kP(), defaults.kI(), defaults.kD(), gains -> updateValues());

    ffManager =
        new TunableFFManager(
            path + "/FF",
            defaults.kS(),
            defaults.kV(),
            defaults.kA(),
            defaults.kG(),
            defaults.kCos(),
            defaults.kCosRatio(),
            gains -> updateValues());
  }

  public void update() {
    pidManager.update();
    ffManager.update();
  }

  /** Applies updated tuning values if any parameters have changed. */
  private void updateValues() {
    io.setPIDF(
        pidManager.get().kP(),
        pidManager.get().kI(),
        pidManager.get().kD(),
        ffManager.get().kS(),
        ffManager.get().kV(),
        ffManager.get().kA(),
        ffManager.get().kG().orElse(0.0),
        ffManager.get().kCos().orElse(0.0),
        ffManager.get().kCosRatio().orElse(0.0),
        0);
  }
}
