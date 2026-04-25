package com.team10043.lib.util.control;

import com.team10043.lib.util.control.ControlGains.FeedforwardGains;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.stream.Stream;

/**
 * Generic runtime-tunable feedforward manager backed by LoggedTunableNumber.
 *
 * <p>Supports optional feedforward terms: kS, kV, kA (required) kG, kCos, kCosRatio (optional)
 */
public class TunableFFManager {

  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kV;
  private final LoggedTunableNumber kA;

  private final Optional<LoggedTunableNumber> kG;
  private final Optional<LoggedTunableNumber> kCos;
  private final Optional<LoggedTunableNumber> kCosRatio;

  private final int callerId = hashCode();
  private final Consumer<FeedforwardGains> onChange;

  public TunableFFManager(
      String prefix,
      double defaultKS,
      double defaultKV,
      double defaultKA,
      Optional<Double> defaultKG,
      Optional<Double> defaultKCos,
      Optional<Double> defaultKCosRatio,
      Consumer<FeedforwardGains> onChange) {

    this.kS = new LoggedTunableNumber(prefix + "/kS", defaultKS);
    this.kV = new LoggedTunableNumber(prefix + "/kV", defaultKV);
    this.kA = new LoggedTunableNumber(prefix + "/kA", defaultKA);

    // Always create the optional tunables so they are visible in AdvantageScope and can be
    // adjusted at runtime. If a default value isn't provided, use 0.0 as the initial value.
    this.kG = Optional.of(new LoggedTunableNumber(prefix + "/kG", defaultKG.orElse(0.0)));
    this.kCos = Optional.of(new LoggedTunableNumber(prefix + "/kCos", defaultKCos.orElse(0.0)));
    this.kCosRatio =
        Optional.of(new LoggedTunableNumber(prefix + "/kCosRatio", defaultKCosRatio.orElse(0.0)));

    this.onChange = onChange;
  }

  /** Call periodically */
  public void update() {
    LoggedTunableNumber.ifChanged(callerId, () -> onChange.accept(get()), collectTunables());
  }

  public FeedforwardGains get() {
    return new FeedforwardGains(
        kS.get(),
        kV.get(),
        kA.get(),
        kG.isPresent() ? Optional.of(kG.get().get()) : Optional.empty(),
        kCos.isPresent() ? Optional.of(kCos.get().get()) : Optional.empty(),
        kCosRatio.isPresent() ? Optional.of(kCosRatio.get().get()) : Optional.empty());
  }

  private LoggedTunableNumber[] collectTunables() {
    return Stream.of(Stream.of(kS, kV, kA), kG.stream(), kCos.stream(), kCosRatio.stream())
        .flatMap(s -> s)
        .toArray(LoggedTunableNumber[]::new);
  }
}
