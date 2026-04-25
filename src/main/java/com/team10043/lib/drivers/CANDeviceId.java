package com.team10043.lib.drivers;

import com.ctre.phoenix6.CANBus;
import lombok.EqualsAndHashCode;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/**
 * Represents a CAN device identifier, including its device ID and CAN bus name.
 *
 * <p>This class provides a unified way to reference CAN devices that may be connected either to the
 * roboRIO CAN bus or to an external CANivore bus.
 */
@Getter
@EqualsAndHashCode
@RequiredArgsConstructor
public class CANDeviceId {

  /**
   * Enumeration of supported CAN bus types.
   *
   * <p>This enum is intended for semantic clarity and higher-level logic, not for direct bus name
   * resolution.
   */
  public enum CANBusType {
    /** External CANivore CAN bus. */
    CANIVORE,

    /** roboRIO onboard CAN bus. */
    RIO
  }

  /** CAN device ID on the specified bus. */
  private final int deviceNumber;

  /** Name of the CAN bus. */
  private final String busName;

  /**
   * Creates a CAN device identifier.
   *
   * @param deviceNumber the CAN device ID
   */
  public CANDeviceId(int deviceNumber) {
    this(deviceNumber, "");
  }

  public CANDeviceId(int deviceNumber, CANBus canBus) {
    this(deviceNumber, canBus.getName());
  }

  @Override
  public String toString() {
    return String.format("CANDeviceId{deviceNumber=%d, busName='%s'}", deviceNumber, busName);
  }

  public CANBus getCANBus() {
    return new CANBus(busName);
  }
}
