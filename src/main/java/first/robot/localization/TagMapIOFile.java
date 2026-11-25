package first.robot.localization;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.time.Instant;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import org.wpilib.driverstation.DriverStationErrors;

/**
 * Stores the learned tag map as a small CSV on the robot's disk.
 *
 * <p>Saves run on a background thread, so disabling never stalls the loop on flash writes. Each
 * save goes to a temporary file that is then atomically renamed over the old one, so a brownout or
 * power-off mid-write can never leave a half-written map. A map saved on a different field layout
 * is ignored at load.
 */
public class TagMapIOFile implements TagMapIO {
  private static final String HEADER = "id,x,y,yaw,positionInfo,yawInfo";

  private final Path path;
  private final String fieldName;
  private final ExecutorService writer =
      Executors.newSingleThreadExecutor(
          runnable -> {
            Thread thread = new Thread(runnable, "TagMapWriter");
            thread.setDaemon(true);
            thread.setPriority(Thread.MIN_PRIORITY);
            return thread;
          });

  /**
   * @param path File to read and write.
   * @param fieldName Name of the field layout; a map saved for another field is not loaded.
   */
  public TagMapIOFile(Path path, String fieldName) {
    this.path = path;
    this.fieldName = fieldName;
  }

  @Override
  public void load(TagMapIOInputs inputs) {
    inputs.loaded = false;
    inputs.source = path.toString();
    if (!Files.exists(path)) return;
    try {
      List<String> lines = Files.readAllLines(path, StandardCharsets.UTF_8);
      String savedField = "";
      List<double[]> rows = new ArrayList<>();
      for (String line : lines) {
        line = line.strip();
        if (line.startsWith("# field=")) {
          savedField = line.substring("# field=".length());
        } else if (line.startsWith("# saved=")) {
          inputs.savedAt = line.substring("# saved=".length());
        } else if (!line.isEmpty() && !line.startsWith("#") && !line.equals(HEADER)) {
          String[] parts = line.split(",");
          double[] row = new double[6];
          for (int i = 0; i < 6; i++) {
            row[i] = Double.parseDouble(parts[i]);
          }
          rows.add(row);
        }
      }
      if (!savedField.equals(fieldName)) {
        DriverStationErrors.reportWarning(
            "Ignoring saved tag map for field '" + savedField + "' (using " + fieldName + ")",
            false);
        return;
      }
      int n = rows.size();
      inputs.ids = new int[n];
      inputs.x = new double[n];
      inputs.y = new double[n];
      inputs.yaw = new double[n];
      inputs.positionInfo = new double[n];
      inputs.yawInfo = new double[n];
      for (int i = 0; i < n; i++) {
        double[] row = rows.get(i);
        inputs.ids[i] = (int) row[0];
        inputs.x[i] = row[1];
        inputs.y[i] = row[2];
        inputs.yaw[i] = row[3];
        inputs.positionInfo[i] = row[4];
        inputs.yawInfo[i] = row[5];
      }
      inputs.loaded = true;
    } catch (IOException | RuntimeException e) {
      DriverStationErrors.reportWarning("Could not read tag map " + path + ": " + e, false);
    }
  }

  @Override
  public void save(List<SlamEstimator.LearnedTag> map) {
    // Format on the caller's thread (cheap) so the writer never touches live estimator state
    StringBuilder sb = new StringBuilder();
    sb.append("# sotaTemplate learned AprilTag map\n");
    sb.append("# field=").append(fieldName).append('\n');
    sb.append("# saved=").append(Instant.now()).append('\n');
    sb.append(HEADER).append('\n');
    for (var tag : map) {
      sb.append(tag.id())
          .append(',')
          .append(tag.x())
          .append(',')
          .append(tag.y())
          .append(',')
          .append(tag.yaw())
          .append(',')
          .append(tag.positionInfo())
          .append(',')
          .append(tag.yawInfo())
          .append('\n');
    }
    String contents = sb.toString();
    writer.execute(
        () -> {
          try {
            Files.createDirectories(path.getParent());
            Path temp = path.resolveSibling(path.getFileName() + ".tmp");
            Files.writeString(temp, contents, StandardCharsets.UTF_8);
            Files.move(
                temp, path, StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.ATOMIC_MOVE);
          } catch (IOException e) {
            DriverStationErrors.reportWarning("Could not save tag map " + path + ": " + e, false);
          }
        });
  }

  /** Waits for pending saves. For tests. */
  public void awaitSaves() throws InterruptedException {
    var done = writer.submit(() -> {});
    try {
      done.get();
    } catch (java.util.concurrent.ExecutionException e) {
      throw new IllegalStateException(e);
    }
  }
}
