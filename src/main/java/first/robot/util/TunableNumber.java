package first.robot.util;

import first.robot.Constants;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * A number that can be changed live from a dashboard while {@link Constants#tuningMode} is on,
 * under {@code /Tuning/<key>}.
 *
 * <p>Built on AdvantageKit's {@link LoggedNetworkNumber}, so every value is logged as an input and
 * log replay sees exactly what the robot saw. WPILib's own {@code Tunables} are not logged by
 * AdvantageKit, which would break replay. With tuning mode off the default is returned and nothing
 * is published.
 */
public class TunableNumber implements DoubleSupplier {
  private static final String TABLE = "/Tuning";

  private final String key;
  private final double defaultValue;
  private LoggedNetworkNumber dashboardNumber;
  private final Map<Integer, Double> lastValues = new HashMap<>();

  /**
   * @param key Key under {@code /Tuning}, e.g. {@code "DriveToPose/TranslationKp"}.
   * @param defaultValue Value used at boot and whenever tuning mode is off.
   */
  public TunableNumber(String key, double defaultValue) {
    this.key = TABLE + "/" + key;
    this.defaultValue = defaultValue;
    if (Constants.tuningMode) {
      dashboardNumber = new LoggedNetworkNumber(this.key, defaultValue);
    }
  }

  /** Returns the current value. */
  public double get() {
    return dashboardNumber != null ? dashboardNumber.get() : defaultValue;
  }

  @Override
  public double getAsDouble() {
    return get();
  }

  /**
   * Returns whether the value changed since the last time this was called with the same id. Use it
   * to rebuild controllers only when a gain actually changed.
   *
   * @param id Caller id, usually {@code hashCode()} of the object that reads this.
   */
  public boolean hasChanged(int id) {
    double value = get();
    Double last = lastValues.put(id, value);
    return last == null || last != value;
  }

  /** Runs an action when any of the given numbers changed since this id last checked. */
  public static void ifChanged(int id, Runnable action, TunableNumber... numbers) {
    boolean changed = false;
    for (var number : numbers) {
      // Check every number so each one records the latest value
      changed |= number.hasChanged(id);
    }
    if (changed) {
      action.run();
    }
  }
}
