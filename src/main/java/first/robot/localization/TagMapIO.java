package first.robot.localization;

import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

/**
 * Where the learned tag map lives between boots. Loading goes through logged inputs, so log replay
 * starts from exactly the map the robot started from; saving is an output and never affects the
 * estimate.
 */
public interface TagMapIO {
  @AutoLog
  public static class TagMapIOInputs {
    public boolean loaded = false;
    public String source = "";
    public String savedAt = "";
    public int[] ids = new int[] {};
    public double[] x = new double[] {};
    public double[] y = new double[] {};
    public double[] yaw = new double[] {};
    public double[] positionInfo = new double[] {};
    public double[] yawInfo = new double[] {};
  }

  /** Reads the saved map, once at boot. */
  public default void load(TagMapIOInputs inputs) {}

  /** Saves the map. Must not block the robot loop. */
  public default void save(List<SlamEstimator.LearnedTag> map) {}

  /** Converts loaded inputs back into map entries. */
  public static List<SlamEstimator.LearnedTag> toMap(TagMapIOInputs inputs) {
    List<SlamEstimator.LearnedTag> map = new ArrayList<>();
    if (!inputs.loaded) return map;
    for (int i = 0; i < inputs.ids.length; i++) {
      map.add(
          new SlamEstimator.LearnedTag(
              inputs.ids[i],
              inputs.x[i],
              inputs.y[i],
              inputs.yaw[i],
              inputs.positionInfo[i],
              inputs.yawInfo[i]));
    }
    return map;
  }
}
