package com.team10043.lib.subsystems.swerve.control.feedback;

import com.team10043.lib.util.control.TunablePIDManager;
import edu.wpi.first.math.controller.ProfiledPIDController;

/**
 * Manages live-tunable PID parameters for swerve feedback controllers. Allows real-time adjustment
 * of translation (X/Y) and rotation (theta) PID gains via NetworkTables.
 */
public class TunableSwerveFeedbackController extends SwerveFeedbackController {

  private final TunablePIDManager translationXPID;
  private final TunablePIDManager translationYPID;
  private final TunablePIDManager rotationPID;

  /**
   * Constructs a new {@code TunableSwerveFeedbackController}.
   *
   * @param key unique key prefix for NetworkTables entries
   * @param xController PID controller responsible for field X translation
   * @param yController PID controller responsible for field Y translation
   * @param thetaController PID controller responsible for robot rotation (radians)
   */
  public TunableSwerveFeedbackController(
      String key,
      ProfiledPIDController xController,
      ProfiledPIDController yController,
      ProfiledPIDController thetaController) {

    super(xController, yController, thetaController);

    this.translationXPID =
        new TunablePIDManager(
            key + "/Translation/X",
            xController.getP(),
            xController.getI(),
            xController.getD(),
            gains -> applyTranslation());

    this.translationYPID =
        new TunablePIDManager(
            key + "/Translation/Y",
            yController.getP(),
            yController.getI(),
            yController.getD(),
            gains -> applyTranslation());

    this.rotationPID =
        new TunablePIDManager(
            key + "/Rotation",
            thetaController.getP(),
            thetaController.getI(),
            thetaController.getD(),
            gains -> applyRotation());

    applyAll();
  }

  /**
   * Checks for updated PID values and applies them if changed. Call this method periodically
   * (typically in the subsystem's periodic method).
   */
  public void updateControllers() {
    translationXPID.update();
    translationYPID.update();
    rotationPID.update();
  }

  /** Applies current translation PID values to the controller. */
  private void applyTranslation() {
    var gainsX = translationXPID.get();
    var gainsY = translationYPID.get();

    setTranslationXPID(gainsX);
    setTranslationYPID(gainsY);
  }

  /** Applies current rotation PID values to the controller. */
  private void applyRotation() {
    var gains = rotationPID.get();
    setRotationPID(gains);
  }

  /** Applies all current PID values to the controller. */
  private void applyAll() {
    applyTranslation();
    applyRotation();
  }
}
