package com.team10043.lib.util.phoenix6;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

/**
 * Phoenix 6 specific high-frequency signal sampler.
 *
 * <p>This class is vendor infrastructure. It does NOT perform odometry, state estimation, or
 * kinematic calculations.
 *
 * <p>Responsibility:
 *
 * <ul>
 *   <li>Synchronously sample Phoenix StatusSignals
 *   <li>Provide latency-compensated timestamps
 *   <li>Publish raw values into thread-safe queues
 * </ul>
 *
 * <p>Singleton by design:
 *
 * <ul>
 *   <li>Phoenix waitForAll requires global coordination
 *   <li>Multiple sampler threads would break CAN timing assumptions
 * </ul>
 */
public final class PhoenixSignalThread extends Thread {

  /** Protects signal registration and Phoenix signal array mutation */
  private final Lock registrationLock = new ReentrantLock();

  /** Protects queue writes to ensure sample batch consistency */
  public static final Lock samplingDataLock = new ReentrantLock();

  private volatile double updateFrequencyHz = 250.0;
  private static boolean isCANivoreFD = false;

  private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
  private final List<DoubleSupplier> genericSignals = new ArrayList<>();

  private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
  private final List<Queue<Double>> genericQueues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private static PhoenixSignalThread instance;

  public static PhoenixSignalThread getInstance() {
    if (instance == null) {
      instance = new PhoenixSignalThread();
    }
    return instance;
  }

  private PhoenixSignalThread() {
    setName("PhoenixSignalThread");
    setDaemon(true);
  }

  /** Must be called before start() */
  public void setUpdateFrequency(double frequencyHz) {
    if (!isAlive()) {
      this.updateFrequencyHz = frequencyHz;
    }
  }

  public void configureCANBus(CANBus bus) {
    isCANivoreFD = bus.isNetworkFD();
  }

  @Override
  public void start() {
    // Do not start unless at least one timestamp consumer exists
    if (!timestampQueues.isEmpty()) {
      super.start();
    }
  }

  /** Registers a Phoenix StatusSignal for sampling */
  public Queue<Double> registerPhoenixSignal(StatusSignal<Angle> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);

    registrationLock.lock();
    samplingDataLock.lock();
    try {
      BaseStatusSignal[] expanded = new BaseStatusSignal[phoenixSignals.length + 1];
      System.arraycopy(phoenixSignals, 0, expanded, 0, phoenixSignals.length);
      expanded[phoenixSignals.length] = signal;
      phoenixSignals = expanded;

      phoenixQueues.add(queue);
    } finally {
      samplingDataLock.unlock();
      registrationLock.unlock();
    }

    return queue;
  }

  /** Registers a generic DoubleSupplier for sampling */
  public Queue<Double> registerGenericSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);

    registrationLock.lock();
    samplingDataLock.lock();
    try {
      genericSignals.add(signal);
      genericQueues.add(queue);
    } finally {
      samplingDataLock.unlock();
      registrationLock.unlock();
    }

    return queue;
  }

  /** Creates a queue that receives timestamps for each sample batch */
  public Queue<Double> createTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);

    samplingDataLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      samplingDataLock.unlock();
    }

    return queue;
  }

  @Override
  public void run() {
    while (true) {

      /* ---- Wait for synchronized Phoenix updates ---- */
      registrationLock.lock();
      try {
        if (isCANivoreFD && phoenixSignals.length > 0) {
          BaseStatusSignal.waitForAll(2.0 / updateFrequencyHz, phoenixSignals);
        } else {
          Thread.sleep((long) (1000.0 / updateFrequencyHz));
          if (phoenixSignals.length > 0) {
            BaseStatusSignal.refreshAll(phoenixSignals);
          }
        }
      } catch (InterruptedException e) {
        e.printStackTrace();
      } finally {
        registrationLock.unlock();
      }

      /* ---- Publish sampled data ---- */
      samplingDataLock.lock();
      try {
        double timestamp = RobotController.getFPGATime() / 1e6;

        double totalLatency = 0.0;
        for (BaseStatusSignal signal : phoenixSignals) {
          totalLatency += signal.getTimestamp().getLatency();
        }

        if (phoenixSignals.length > 0) {
          timestamp -= totalLatency / phoenixSignals.length;
        }

        for (int i = 0; i < phoenixSignals.length; i++) {
          phoenixQueues.get(i).offer(phoenixSignals[i].getValueAsDouble());
        }

        for (int i = 0; i < genericSignals.size(); i++) {
          genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
        }

        for (Queue<Double> queue : timestampQueues) {
          queue.offer(timestamp);
        }

      } finally {
        samplingDataLock.unlock();
      }
    }
  }
}
