package first.robot.bench;

import java.util.List;
import java.util.Locale;

/**
 * Grouped horizontal bar chart of the benchmark, as a standalone SVG (light or dark). Colors are
 * the first three slots of the reference categorical palette, validated for both modes.
 */
final class BenchmarkChart {
  private BenchmarkChart() {}

  private record Theme(
      String surface,
      String primary,
      String secondary,
      String muted,
      String grid,
      String axis,
      String[] series) {}

  private static final Theme LIGHT =
      new Theme(
          "#fcfcfb",
          "#0b0b0b",
          "#52514e",
          "#898781",
          "#e1e0d9",
          "#c3c2b7",
          new String[] {"#eb6834", "#1baf7a", "#2a78d6"});
  private static final Theme DARK =
      new Theme(
          "#1a1a19",
          "#f0efec",
          "#c3c2b7",
          "#898781",
          "#2c2c2a",
          "#383835",
          new String[] {"#d95926", "#199e70", "#3987e5"});

  private static final String FONT = "system-ui, -apple-system, Segoe UI, sans-serif";

  /** A chart in cm with no axis cap. */
  static String render(
      String title,
      String subtitle,
      List<String> scenarios,
      List<String> series,
      double[][] values,
      boolean dark) {
    return render(title, subtitle, scenarios, series, values, "cm", Double.NaN, dark);
  }

  /**
   * @param scenarios Row labels.
   * @param series Series names (legend order, top to bottom within each row). At most three.
   * @param values values[scenario][series].
   * @param unit Unit for labels and the last tick, e.g. "cm" or "ms".
   * @param cap Axis maximum; longer bars are drawn to the edge, labelled with their true value. NaN
   *     fits the axis to the data.
   */
  static String render(
      String title,
      String subtitle,
      List<String> scenarios,
      List<String> series,
      double[][] values,
      String unit,
      double cap,
      boolean dark) {
    Theme t = dark ? DARK : LIGHT;
    int width = 860;
    int left = 24;
    int plotLeft = 230;
    int plotRight = width - 90;
    int barHeight = 18;
    int gap = 2;
    int groupGap = 26;
    int top = 104;

    double max = 0;
    for (double[] row : values) for (double v : row) max = Math.max(max, v);
    if (Double.isFinite(cap)) max = Math.min(max, cap);
    double step = niceStep(max);
    double axisMax = Math.max(step, Math.ceil(max / step - 1e-9) * step);
    int groupHeight = series.size() * barHeight + (series.size() - 1) * gap;
    int plotBottom = top + scenarios.size() * (groupHeight + groupGap) - groupGap;
    int height = plotBottom + 44;

    StringBuilder s = new StringBuilder();
    s.append(
        String.format(
            Locale.ROOT,
            "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 %d %d\" width=\"%d\" height=\"%d\" role=\"img\" aria-label=\"%s\">%n",
            width,
            height,
            width,
            height,
            escape(title)));
    s.append(
        String.format(
            "<rect width=\"%d\" height=\"%d\" rx=\"8\" fill=\"%s\"/>%n",
            width, height, t.surface()));
    s.append(text(left, 34, title, t.primary(), 17, "600", "start"));
    s.append(text(left, 56, subtitle, t.secondary(), 13, "400", "start"));

    // Legend, one row
    int lx = left;
    for (int i = 0; i < series.size(); i++) {
      s.append(
          String.format(
              Locale.ROOT,
              "<rect x=\"%d\" y=\"%d\" width=\"12\" height=\"12\" rx=\"3\" fill=\"%s\"/>%n",
              lx,
              70,
              t.series()[i]));
      s.append(text(lx + 18, 81, series.get(i), t.secondary(), 13, "400", "start"));
      lx += 18 + series.get(i).length() * 7 + 28;
    }

    // Gridlines and ticks
    for (double tick = 0; tick <= axisMax + 1e-9; tick += step) {
      double x = plotLeft + (plotRight - plotLeft) * tick / axisMax;
      s.append(
          String.format(
              Locale.ROOT,
              "<line x1=\"%.1f\" y1=\"%d\" x2=\"%.1f\" y2=\"%d\" stroke=\"%s\" stroke-width=\"1\"/>%n",
              x,
              top - 8,
              x,
              plotBottom + 4,
              tick == 0 ? t.axis() : t.grid()));
      s.append(
          text(
              (int) Math.round(x),
              plotBottom + 22,
              tick(tick) + (Math.abs(tick - axisMax) < 1e-9 ? " " + unit : ""),
              t.muted(),
              12,
              "400",
              "middle"));
    }

    // Bars
    for (int r = 0; r < scenarios.size(); r++) {
      int groupTop = top + r * (groupHeight + groupGap);
      s.append(
          text(
              left,
              groupTop + groupHeight / 2 + 5,
              scenarios.get(r),
              t.primary(),
              13,
              "500",
              "start"));
      for (int i = 0; i < series.size(); i++) {
        double v = values[r][i];
        double w = Math.max(2, (plotRight - plotLeft) * Math.min(v, axisMax) / axisMax);
        int y = groupTop + i * (barHeight + gap);
        s.append(bar(plotLeft, y, w, barHeight, t.series()[i]));
        s.append(
            text(
                (int) Math.round(plotLeft + w + 6),
                y + barHeight / 2 + 4,
                number(v) + " " + unit + (v > axisMax ? " →" : ""),
                t.secondary(),
                12,
                "400",
                "start"));
      }
    }
    s.append("</svg>\n");
    return s.toString();
  }

  /** A tick spacing of 1, 2 or 5 times a power of ten giving at most about six ticks. */
  private static double niceStep(double max) {
    if (!(max > 0)) return 1.0;
    double raw = max / 6.0;
    double magnitude = Math.pow(10, Math.floor(Math.log10(raw)));
    for (double m : new double[] {1, 2, 5, 10}) {
      if (m * magnitude >= raw) return m * magnitude;
    }
    return 10 * magnitude;
  }

  /** A value with as many decimals as its size needs: 127, 8.3, 4.0, 0.52. */
  private static String number(double v) {
    if (v >= 100) return String.format(Locale.ROOT, "%.0f", v);
    if (v >= 1) return String.format(Locale.ROOT, "%.1f", v);
    // Two decimals only when the second carries information: 0.52, but 0.8 rather than 0.80
    String out = String.format(Locale.ROOT, "%.2f", v);
    return out.endsWith("0") ? out.substring(0, out.length() - 1) : out;
  }

  /** An axis tick without trailing zeros: 0, 0.2, 5, 40. */
  private static String tick(double v) {
    String out = String.format(Locale.ROOT, "%.2f", v);
    return out.replaceAll("0+$", "").replaceAll("\\.$", "");
  }

  /** A bar square at the baseline with a 4px rounded data end. */
  private static String bar(double x, double y, double w, double h, String fill) {
    double r = Math.min(4, w / 2);
    return String.format(
        Locale.ROOT,
        "<path d=\"M%.1f %.1f H%.1f Q%.1f %.1f %.1f %.1f V%.1f Q%.1f %.1f %.1f %.1f H%.1f Z\" fill=\"%s\"/>%n",
        x,
        y,
        x + w - r,
        x + w,
        y,
        x + w,
        y + r,
        y + h - r,
        x + w,
        y + h,
        x + w - r,
        y + h,
        x,
        fill);
  }

  private static String text(
      int x, int y, String content, String color, int size, String weight, String anchor) {
    return String.format(
        Locale.ROOT,
        "<text x=\"%d\" y=\"%d\" fill=\"%s\" font-family=\"%s\" font-size=\"%d\" font-weight=\"%s\" text-anchor=\"%s\">%s</text>%n",
        x,
        y,
        color,
        FONT,
        size,
        weight,
        anchor,
        escape(content));
  }

  private static String escape(String s) {
    return s.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace("\"", "&quot;");
  }
}
