package first.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import first.robot.subsystems.vision.VisionIO.RawFiducial;
import java.util.Set;
import org.junit.jupiter.api.Test;

class TagQCTest {
  private static final double SIZE = 0.1651;
  private static final Set<Integer> KNOWN = Set.of(1, 2, 3);

  /** A square tag 2 m away, straight ahead: corners BL, BR, TR, TL. */
  private static double[] square(double range) {
    double h = SIZE / 2 / range;
    return new double[] {-h, -h, h, -h, h, h, -h, h};
  }

  private static RawFiducial tag(int id, double[] corners, double distance, double ambiguity) {
    return RawFiducial.withCorners(1.0, id, 0.0, 0.0, distance, ambiguity, corners);
  }

  private static TagQC.Result check(RawFiducial... f) {
    return TagQC.check(f, KNOWN, SIZE, 0.3);
  }

  @Test
  void goodTagPasses() {
    var result = check(tag(1, square(2.0), 2.0, 0.05));
    assertEquals(1, result.accepted().size());
    assertTrue(result.rejections().isEmpty());
    assertArrayEquals(square(2.0), result.accepted().get(0).corners(), 1e-12);
  }

  @Test
  void anyCornerOrderBecomesCanonical() {
    double[] c = square(2.0);
    // Start at top-right and go the other way round
    double[] scrambled = {c[4], c[5], c[2], c[3], c[0], c[1], c[6], c[7]};
    var result = check(tag(1, scrambled, 2.0, 0.05));
    assertArrayEquals(c, result.accepted().get(0).corners(), 1e-12);
  }

  @Test
  void nearlyEdgeOnSliverIsOrderedCorrectly() {
    // A tag seen almost edge-on near the image edge: horizontal edges foreshortened to a tenth,
    // the whole quad tilted a few degrees by perspective. Angle-around-the-centroid ordering got
    // these wrong; winding + vertical-edge rise must not.
    double[] c = square(2.0);
    double tilt = Math.toRadians(6.0);
    double[] sliver = new double[8];
    for (int k = 0; k < 4; k++) {
      double x = c[2 * k] * 0.1;
      double y = c[2 * k + 1];
      sliver[2 * k] = 0.6 + x * Math.cos(tilt) - y * Math.sin(tilt);
      sliver[2 * k + 1] = x * Math.sin(tilt) + y * Math.cos(tilt);
    }
    // Reported clockwise, starting at the top-right: TR, BR, BL, TL
    double[] reported = {
      sliver[4], sliver[5], sliver[2], sliver[3], sliver[0], sliver[1], sliver[6], sliver[7]
    };
    assertArrayEquals(sliver, TagQC.canonicalOrder(reported), 1e-12);
  }

  @Test
  void unknownAndAmbiguousAndDuplicateTagsAreDropped() {
    var result =
        check(
            tag(9, square(2.0), 2.0, 0.0),
            tag(2, square(2.0), 2.0, 0.9),
            tag(3, square(2.0), 2.0, 0.0),
            tag(3, square(3.0), 3.0, 0.0));
    assertTrue(result.accepted().isEmpty());
    assertEquals(1, result.rejections().get(TagQC.Reason.UNKNOWN_ID));
    assertEquals(1, result.rejections().get(TagQC.Reason.AMBIGUOUS));
    assertEquals(2, result.rejections().get(TagQC.Reason.DUPLICATE_ID));
  }

  @Test
  void badCornersFallBackToTheCenter() {
    double[] bowtie = square(2.0);
    // Swap two corners: a self-intersecting quad that QC's reordering can't fix after the swap of
    // coordinates within one corner
    bowtie[2] = -bowtie[2];
    var notConvex = check(tag(1, bowtie, 2.0, 0.0));
    assertEquals(1, notConvex.accepted().size(), "the center is still usable");
    assertNull(notConvex.accepted().get(0).corners());

    var tiny = check(tag(1, square(40.0), 40.0, 0.0));
    assertNull(tiny.accepted().get(0).corners());
    assertEquals(1, tiny.rejections().get(TagQC.Reason.CORNERS_TOO_SMALL));

    double[] oblique = square(2.0);
    oblique[3] = -oblique[3] * 0.1; // Bottom-right pulled almost onto the top edge's height
    oblique[5] = oblique[5] * 0.1;
    var skewed = check(tag(1, oblique, 2.0, 0.0));
    assertNull(skewed.accepted().get(0).corners());
  }

  @Test
  void twoDPipelineGetsRangeFromApparentSize() {
    // A 2D pipeline (no PnP) reports no distance; QC fills it from the tag's size
    var result = check(tag(1, square(2.0), Double.NaN, 0.0));
    var accepted = result.accepted().get(0);
    assertEquals(2.0, accepted.distanceMeters(), 0.01);
    assertTrue(result.rejections().isEmpty());
  }

  @Test
  void rangeThatDisagreesWithApparentSizeIsDropped() {
    var result = check(tag(1, square(2.0), 6.0, 0.0));
    var accepted = result.accepted().get(0);
    assertFalse(Double.isFinite(accepted.distanceMeters()));
    assertTrue(accepted.corners() != null, "corners are fine; only the range is suspect");
    assertEquals(1, result.rejections().get(TagQC.Reason.RANGE_DISAGREES_WITH_SIZE));
  }
}
