package first.robot.subsystems.vision;

import first.robot.subsystems.vision.VisionIO.RawFiducial;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Quality control on raw detections, before anything reaches the SLAM. It runs identically for
 * every backend, so cameras from different vendors are judged by the same rules.
 *
 * <p>Corners are put into the canonical order (bottom-left, bottom-right, top-right, top-left as
 * seen facing the tag) by geometry, so a backend's own corner order can never silently scramble the
 * reprojection.
 */
public final class TagQC {
  /** Why a detection, or its corners, was not used. */
  public enum Reason {
    UNKNOWN_ID,
    AMBIGUOUS,
    DUPLICATE_ID,
    BAD_CENTER,
    CORNERS_NOT_CONVEX,
    CORNERS_TOO_SMALL,
    CORNERS_TOO_OBLIQUE,
    RANGE_DISAGREES_WITH_SIZE
  }

  /** Outcome of QC on one frame of one camera. */
  public record Result(List<RawFiducial> accepted, Map<Reason, Integer> rejections) {}

  /** A detection's apparent tag edge must be at least this big (normalized; ~12 px at f=1000). */
  public static final double MIN_EDGE = 0.012;

  /** Opposite edges may differ by at most this ratio (very oblique views corrupt corners). */
  public static final double MAX_EDGE_RATIO = 3.0;

  /**
   * Horizontal edges shorter than this fraction of the vertical ones mean the tag is seen within
   * about 15° of edge-on; its corners are then too compressed to trust, so only the center is used.
   */
  public static final double MIN_FACE_ON_RATIO = 0.25;

  /** Reported range must agree with apparent tag size within this fraction. */
  public static final double MAX_RANGE_DISAGREEMENT = 0.5;

  private TagQC() {}

  /**
   * Checks one camera's detections. Whole detections are dropped for an unknown or duplicated ID,
   * high ambiguity or a non-finite center. Bad corners only drop the corners (the center is still
   * used), and a range that disagrees with the tag's apparent size only drops the range.
   *
   * @param fiducials Raw detections from one camera.
   * @param knownIds Tag IDs in the field layout.
   * @param tagSize Tag edge length in meters.
   * @param maxAmbiguity Highest acceptable pose ambiguity.
   */
  public static Result check(
      RawFiducial[] fiducials,
      java.util.Set<Integer> knownIds,
      double tagSize,
      double maxAmbiguity) {
    Map<Reason, Integer> rejections = new HashMap<>();
    List<RawFiducial> accepted = new ArrayList<>();

    // The same ID twice in one frame means at least one is a false detection: drop both
    Map<String, Integer> idsPerFrame = new HashMap<>();
    for (var f : fiducials) {
      idsPerFrame.merge(f.timestamp() + ":" + f.id(), 1, Integer::sum);
    }

    for (var f : fiducials) {
      if (!knownIds.contains(f.id())) {
        rejections.merge(Reason.UNKNOWN_ID, 1, Integer::sum);
        continue;
      }
      if (idsPerFrame.get(f.timestamp() + ":" + f.id()) > 1) {
        rejections.merge(Reason.DUPLICATE_ID, 1, Integer::sum);
        continue;
      }
      if (f.ambiguity() > maxAmbiguity) {
        rejections.merge(Reason.AMBIGUOUS, 1, Integer::sum);
        continue;
      }
      if (!Double.isFinite(f.txRad()) || !Double.isFinite(f.tyRad())) {
        rejections.merge(Reason.BAD_CENTER, 1, Integer::sum);
        continue;
      }

      double[] corners = f.corners();
      double distance = f.distanceMeters();
      double tx = f.txRad();
      double ty = f.tyRad();
      if (corners != null) {
        corners = canonicalOrder(corners);
        Reason problem = cornerProblem(corners);
        if (problem != null) {
          rejections.merge(problem, 1, Integer::sum);
          corners = null;
        } else {
          // The projected center of a square is exactly where its diagonals cross, so derive the
          // center from the corners: identical for every backend, whatever its own yaw/pitch
          // conventions (PhotonVision's pitch, for one, differs between its pipeline and its sim)
          double[] center = diagonalIntersection(corners);
          tx = Math.atan(center[0]);
          ty = Math.atan(center[1]);

          // Apparent size gives an independent range. A 2D pipeline reports no range at all; a
          // reported range far from it is a bad PnP solve.
          double sizeRange = rangeFromSize(corners, center, tagSize);
          if (!Double.isFinite(distance)) {
            distance = sizeRange;
          } else if (Math.abs(distance - sizeRange) > MAX_RANGE_DISAGREEMENT * sizeRange) {
            rejections.merge(Reason.RANGE_DISAGREES_WITH_SIZE, 1, Integer::sum);
            distance = Double.NaN;
          }
        }
      }
      accepted.add(
          corners != null
              ? RawFiducial.withCorners(
                  f.timestamp(), f.id(), tx, ty, distance, f.ambiguity(), corners)
              : new RawFiducial(f.timestamp(), f.id(), tx, ty, distance, f.ambiguity()));
    }
    return new Result(accepted, rejections);
  }

  /**
   * Puts corners into the canonical order: bottom-left, bottom-right, top-right, top-left as seen
   * facing the tag (x right, y up). Backends report the four corners in some cyclic order, possibly
   * starting at a different corner or going the other way round, so the only freedom is a rotation
   * and a reflection, and two facts fix both:
   *
   * <ul>
   *   <li>A tag can only be detected from the front, so in the image its corners always wind
   *       counter-clockwise in canonical order. A clockwise list is reversed.
   *   <li>An upright tag's two vertical edges keep their top corner above their bottom corner in
   *       the image however obliquely it is seen (as long as the camera isn't rolled past 45°), and
   *       they are the long edges of a foreshortened tag. Of the four rotations, the one whose
   *       "vertical" edges rise the most is the right one.
   * </ul>
   *
   * <p>Unlike sorting by angle around the centroid, this stays exact for tags seen nearly edge-on
   * or near the edge of the image, where the quad becomes a thin, tilted sliver.
   */
  public static double[] canonicalOrder(double[] c) {
    // Signed area (shoelace); positive = counter-clockwise with y up
    double area = 0.0;
    for (int k = 0; k < 4; k++) {
      int n = (k + 1) % 4;
      area += c[2 * k] * c[2 * n + 1] - c[2 * n] * c[2 * k + 1];
    }
    int[] order = area >= 0 ? new int[] {0, 1, 2, 3} : new int[] {0, 3, 2, 1};

    int bestShift = 0;
    double bestRise = Double.NEGATIVE_INFINITY;
    for (int shift = 0; shift < 4; shift++) {
      int bl = order[shift];
      int br = order[(shift + 1) % 4];
      int tr = order[(shift + 2) % 4];
      int tl = order[(shift + 3) % 4];
      // How far the right edge (BR -> TR) and left edge (BL -> TL) rise
      double rise = (c[2 * tr + 1] - c[2 * br + 1]) + (c[2 * tl + 1] - c[2 * bl + 1]);
      if (rise > bestRise) {
        bestRise = rise;
        bestShift = shift;
      }
    }
    double[] out = new double[8];
    for (int k = 0; k < 4; k++) {
      int src = order[(bestShift + k) % 4];
      out[2 * k] = c[2 * src];
      out[2 * k + 1] = c[2 * src + 1];
    }
    return out;
  }

  private static Reason cornerProblem(double[] c) {
    // Convex and consistently wound: every cross product of consecutive edges has the same sign
    int sign = 0;
    for (int k = 0; k < 4; k++) {
      int a = k;
      int b = (k + 1) % 4;
      int d = (k + 2) % 4;
      double cross =
          (c[2 * b] - c[2 * a]) * (c[2 * d + 1] - c[2 * b + 1])
              - (c[2 * b + 1] - c[2 * a + 1]) * (c[2 * d] - c[2 * b]);
      int s = cross > 0 ? 1 : cross < 0 ? -1 : 0;
      if (s == 0 || (sign != 0 && s != sign)) return Reason.CORNERS_NOT_CONVEX;
      sign = s;
    }
    double[] edges = new double[4];
    for (int k = 0; k < 4; k++) {
      int b = (k + 1) % 4;
      edges[k] = Math.hypot(c[2 * b] - c[2 * k], c[2 * b + 1] - c[2 * k + 1]);
      if (edges[k] < MIN_EDGE) return Reason.CORNERS_TOO_SMALL;
    }
    double ratio1 = Math.max(edges[0], edges[2]) / Math.min(edges[0], edges[2]);
    double ratio2 = Math.max(edges[1], edges[3]) / Math.min(edges[1], edges[3]);
    if (Math.max(ratio1, ratio2) > MAX_EDGE_RATIO) return Reason.CORNERS_TOO_OBLIQUE;
    // Nearly edge-on: the horizontal edges (0, 2) foreshorten while the vertical ones (1, 3) don't
    double horizontal = (edges[0] + edges[2]) / 2.0;
    double vertical = (edges[1] + edges[3]) / 2.0;
    if (horizontal < MIN_FACE_ON_RATIO * vertical) return Reason.CORNERS_TOO_OBLIQUE;
    return null;
  }

  /** Intersection of the diagonals BL–TR and BR–TL, in normalized image coordinates. */
  static double[] diagonalIntersection(double[] c) {
    double x1 = c[0], y1 = c[1], x2 = c[4], y2 = c[5]; // BL -> TR
    double x3 = c[2], y3 = c[3], x4 = c[6], y4 = c[7]; // BR -> TL
    double den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    double a = x1 * y2 - y1 * x2;
    double b = x3 * y4 - y3 * x4;
    return new double[] {
      (a * (x3 - x4) - (x1 - x2) * b) / den, (a * (y3 - y4) - (y1 - y2) * b) / den
    };
  }

  /**
   * Range to the tag center from its apparent size. Uses the tag's vertical edges, which don't
   * foreshorten when an upright tag is seen from the side (the horizontal ones do, by the cosine of
   * the viewing angle): their image length is about tagSize / depth.
   */
  static double rangeFromSize(double[] c, double[] center, double tagSize) {
    double right = Math.hypot(c[4] - c[2], c[5] - c[3]); // BR -> TR
    double left = Math.hypot(c[0] - c[6], c[1] - c[7]); // TL -> BL
    double depth = tagSize / ((left + right) / 2.0);
    return depth * Math.sqrt(1.0 + center[0] * center[0] + center[1] * center[1]);
  }
}
