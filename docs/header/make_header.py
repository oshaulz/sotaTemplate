#!/usr/bin/env python3
"""Generates the README header: sota-github-header.svg (1280 x 260).

thx claude

Stdlib only. Run: python3 docs/header/make_header.py
"""

import math
import random
from pathlib import Path

W, H = 1280, 260
WHITE = "#ffffff"
GREY = "#8c8c8c"
DIM = "#3a3a3a"
ACCENT = "#B7A6D6"  # Soft purple, as in the CMRA header
ACCENT_DEEP = "#4F2683"

FONT = "Helvetica Neue, Helvetica, Arial, sans-serif"

# Right-hand panel (the "field")
PX0, PX1, PY0, PY1 = 640, 1236, 30, 230

random.seed(4476)


def path_point(t):
    """Robot path, t in [0, 1]: a sweeping S across the panel."""
    x = PX0 + 40 + t * (PX1 - PX0 - 90)
    y = (PY0 + PY1) / 2 + 52 * math.sin(2.2 * math.pi * t + 0.4) * (0.55 + 0.45 * t)
    return x, y


def heading(t):
    x0, y0 = path_point(max(0.0, t - 0.01))
    x1, y1 = path_point(min(1.0, t + 0.01))
    return math.atan2(y1 - y0, x1 - x0)


def f(v):
    return f"{v:.1f}"


def robot(x, y, th, size, stroke, fill, width=1.4, opacity=1.0):
    """A square swerve chassis with a heading tick."""
    c, s = math.cos(th), math.sin(th)
    h = size / 2
    corners = [(-h, -h), (h, -h), (h, h), (-h, h)]
    pts = " ".join(f"{f(x + c * px - s * py)},{f(y + s * px + c * py)}" for px, py in corners)
    tip = (x + c * size * 0.95, y + s * size * 0.95)
    return (
        f'<g opacity="{opacity}">'
        f'<polygon points="{pts}" fill="{fill}" stroke="{stroke}" stroke-width="{width}" '
        f'stroke-linejoin="round"/>'
        f'<line x1="{f(x)}" y1="{f(y)}" x2="{f(tip[0])}" y2="{f(tip[1])}" stroke="{stroke}" '
        f'stroke-width="{width}" stroke-linecap="round"/></g>'
    )


def apriltag(x, y, size, solid=True, bits=None):
    """A small AprilTag glyph: white border, black body, a 4x4 bit pattern."""
    out = []
    if solid:
        out.append(
            f'<rect x="{f(x - size / 2)}" y="{f(y - size / 2)}" width="{f(size)}" '
            f'height="{f(size)}" fill="{WHITE}"/>'
        )
        inner = size * 0.72
        out.append(
            f'<rect x="{f(x - inner / 2)}" y="{f(y - inner / 2)}" width="{f(inner)}" '
            f'height="{f(inner)}" fill="#000"/>'
        )
        cell = inner / 6
        for i in range(4):
            for j in range(4):
                if bits[i * 4 + j]:
                    out.append(
                        f'<rect x="{f(x - inner / 2 + cell * (j + 1))}" '
                        f'y="{f(y - inner / 2 + cell * (i + 1))}" width="{f(cell)}" '
                        f'height="{f(cell)}" fill="{WHITE}"/>'
                    )
    else:
        out.append(
            f'<rect x="{f(x - size / 2)}" y="{f(y - size / 2)}" width="{f(size)}" '
            f'height="{f(size)}" fill="none" stroke="{GREY}" stroke-width="1" '
            f'stroke-dasharray="2 2"/>'
        )
    return "".join(out)


def main():
    svg = []
    svg.append(
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" role="img" '
        f'aria-label="sotaTemplate: a 2027 FRC swerve template with built-in SLAM">'
    )
    svg.append(
        "<defs>"
        f'<radialGradient id="glow"><stop offset="0" stop-color="{ACCENT}" stop-opacity="0.55"/>'
        f'<stop offset="1" stop-color="{ACCENT}" stop-opacity="0"/></radialGradient>'
        f'<linearGradient id="fade" x1="0" x2="1"><stop offset="0" stop-color="#000" '
        f'stop-opacity="1"/><stop offset="0.12" stop-color="#000" stop-opacity="0"/>'
        f"</linearGradient>"
        "</defs>"
    )
    svg.append(f'<rect width="{W}" height="{H}" fill="#000"/>')

    # ---- Wordmark ----
    svg.append(
        f'<text x="72" y="122" fill="{WHITE}" font-family="{FONT}" font-size="64" '
        f'font-weight="700" letter-spacing="-1.5">sota<tspan font-weight="300">Template</tspan>'
        f"</text>"
    )
    svg.append(
        f'<text x="75" y="160" fill="{WHITE}" font-family="{FONT}" font-size="21" '
        f'font-weight="400">FRC swerve with built-in SLAM</text>'
    )
    svg.append(
        f'<text x="75" y="192" fill="{GREY}" font-family="{FONT}" font-size="14" '
        f'font-weight="400" letter-spacing="0.3">2027 Systemcore · Commands v3 · '
        f"AdvantageKit · BLine</text>"
    )

    # Divider, as in the CMRA header
    svg.append(f'<line x1="598" y1="40" x2="598" y2="220" stroke="{GREY}" stroke-width="1"/>')

    # ---- Field: faint grid ----
    for gx in range(PX0, PX1 + 1, 40):
        svg.append(
            f'<line x1="{gx}" y1="{PY0}" x2="{gx}" y2="{PY1}" stroke="{DIM}" '
            f'stroke-width="0.6" opacity="0.55"/>'
        )
    for gy in range(PY0, PY1 + 1, 40):
        svg.append(
            f'<line x1="{PX0}" y1="{gy}" x2="{PX1}" y2="{gy}" stroke="{DIM}" '
            f'stroke-width="0.6" opacity="0.55"/>'
        )

    # ---- Tags (landmarks) ----
    tags = [
        (720, 52), (850, 210), (930, 48), (1040, 212), (1120, 50), (1205, 205),
    ]
    moved = 3  # This tag's layout position is off; the map learned where it really is
    moved_layout = (tags[moved][0] - 13, tags[moved][1] + 9)
    bits = [[random.random() < 0.45 for _ in range(16)] for _ in tags]

    # ---- Keyframes along the path ----
    n = 17
    ts = [i / (n - 1) for i in range(n)]
    sees = {}  # keyframe index -> tags in view
    for i, t in enumerate(ts):
        x, y = path_point(t)
        th = heading(t)
        visible = []
        for k, (tx, ty) in enumerate(tags):
            d = math.hypot(tx - x, ty - y)
            # Front and back cameras: anything within ~60 deg of the heading axis, in range
            ang = math.atan2(ty - y, tx - x) - th
            ang = math.atan2(math.sin(ang), math.cos(ang))
            if d < 190 and (abs(ang) < 1.05 or abs(abs(ang) - math.pi) < 1.05):
                visible.append(k)
        # Leave stretches with no vision so the ellipses visibly grow
        if i in (4, 5, 6, 11, 12):
            visible = []
        sees[i] = visible

    # Uncertainty: grows with distance while dead reckoning, collapses on a tag sighting
    sigma = []
    s = 3.0
    for i in range(n):
        if i > 0:
            s += 2.4
        if sees[i]:
            s = 2.2 + 1.2 / len(sees[i])
        sigma.append(s)

    # ---- Raw odometry: same path, slowly rotating and stretching away (dashed) ----
    ox0, oy0 = path_point(0)
    odo = []
    for k in range(121):
        t = k / 120
        x, y = path_point(t)
        drift = 0.11 * t * t
        dx, dy = (x - ox0) * (1 + 0.035 * t), (y - oy0) * (1 + 0.035 * t)
        odo.append(
            (ox0 + dx * math.cos(drift) - dy * math.sin(drift),
             oy0 + dx * math.sin(drift) + dy * math.cos(drift))
        )
    svg.append(
        '<polyline points="'
        + " ".join(f"{f(x)},{f(y)}" for x, y in odo)
        + f'" fill="none" stroke="{GREY}" stroke-width="1.3" stroke-dasharray="5 4" '
        f'opacity="0.85"/>'
    )

    # ---- Odometry factors: the estimated path through the keyframes ----
    est = [path_point(k / 120) for k in range(121)]
    svg.append(
        '<polyline points="'
        + " ".join(f"{f(x)},{f(y)}" for x, y in est)
        + f'" fill="none" stroke="{WHITE}" stroke-width="1.2" opacity="0.55"/>'
    )

    # ---- Vision factors: rays from keyframes to tags ----
    for i, t in enumerate(ts):
        x, y = path_point(t)
        recent = i / (n - 1)
        for k in sees[i]:
            tx, ty = tags[k]
            svg.append(
                f'<line x1="{f(x)}" y1="{f(y)}" x2="{tx}" y2="{ty}" stroke="{ACCENT}" '
                f'stroke-width="{0.6 + 0.6 * recent:.2f}" opacity="{0.18 + 0.5 * recent:.2f}"/>'
            )

    # ---- Uncertainty ellipses ----
    for i, t in enumerate(ts):
        x, y = path_point(t)
        th = math.degrees(heading(t))
        r = sigma[i]
        svg.append(
            f'<ellipse cx="{f(x)}" cy="{f(y)}" rx="{f(r * 1.6)}" ry="{f(r)}" '
            f'transform="rotate({f(th)} {f(x)} {f(y)})" fill="{ACCENT_DEEP}" fill-opacity="0.22" '
            f'stroke="{ACCENT}" stroke-width="0.7" stroke-opacity="0.6"/>'
        )

    # ---- Tags on top of the rays ----
    lx, ly = moved_layout
    mx, my = tags[moved]
    svg.append(apriltag(lx, ly, 13, solid=False))
    svg.append(
        f'<line x1="{lx}" y1="{ly}" x2="{f(mx - 5)}" y2="{f(my + 3.5)}" stroke="{ACCENT}" '
        f'stroke-width="1" opacity="0.9"/>'
    )
    for k, (tx, ty) in enumerate(tags):
        svg.append(apriltag(tx, ty, 13, solid=True, bits=bits[k]))

    # ---- Keyframe robots (older ones fade) ----
    for i, t in enumerate(ts[:-1]):
        x, y = path_point(t)
        svg.append(robot(x, y, heading(t), 7.5, WHITE, "#000", 1.1, 0.35 + 0.6 * i / (n - 1)))

    # ---- The robot now ----
    x, y = path_point(1.0)
    svg.append(f'<circle cx="{f(x)}" cy="{f(y)}" r="30" fill="url(#glow)"/>')
    svg.append(robot(x, y, heading(1.0), 15, WHITE, "#000", 1.8))

    # Soften the panel's left edge into the divider
    svg.append(f'<rect x="{PX0 - 2}" y="0" width="{PX1 - PX0 + 4}" height="{H}" fill="url(#fade)"/>')

    # ---- Legend, small and quiet like the CMRA figures ----
    ly0 = 247
    items = [
        ("line", WHITE, "SLAM estimate"),
        ("dash", GREY, "raw odometry"),
        ("line", ACCENT, "camera → tag"),
        ("ell", ACCENT, "uncertainty"),
        ("tag", WHITE, "tag mapped"),
    ]
    lx0 = 668
    for kind, color, label in items:
        if kind == "line":
            svg.append(
                f'<line x1="{lx0}" y1="{ly0 - 4}" x2="{lx0 + 18}" y2="{ly0 - 4}" '
                f'stroke="{color}" stroke-width="1.4"/>'
            )
        elif kind == "dash":
            svg.append(
                f'<line x1="{lx0}" y1="{ly0 - 4}" x2="{lx0 + 18}" y2="{ly0 - 4}" '
                f'stroke="{color}" stroke-width="1.4" stroke-dasharray="4 3"/>'
            )
        elif kind == "ell":
            svg.append(
                f'<ellipse cx="{lx0 + 9}" cy="{ly0 - 4}" rx="9" ry="4.5" fill="{ACCENT_DEEP}" '
                f'fill-opacity="0.35" stroke="{color}" stroke-width="0.8"/>'
            )
        else:
            svg.append(apriltag(lx0 + 4, ly0 - 4, 8, solid=False))
            svg.append(apriltag(lx0 + 15, ly0 - 4, 8, solid=True, bits=bits[0]))
        svg.append(
            f'<text x="{lx0 + 25}" y="{ly0}" fill="{GREY}" font-family="{FONT}" '
            f'font-size="11">{label}</text>'
        )
        lx0 += 25 + len(label) * 5.9 + 26

    svg.append("</svg>")
    out = Path(__file__).with_name("sota-github-header.svg")
    out.write_text("\n".join(svg) + "\n")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
