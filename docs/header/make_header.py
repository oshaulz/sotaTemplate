#!/usr/bin/env python3
"""Generates the README header: sota-github-header.svg (1280 x 360).

thx claude

A small 3D line renderer, stdlib only: a robot localizing itself on a night-lit FRC field, drawn
in outline. Rays run from its cameras to the four corners of each AprilTag they see (the SLAM
reprojects corners, not centers). Behind it, its past poses sit on the carpet with their
uncertainty, which grows while it drives blind and collapses when it sees tags; raw odometry
drifts off to one side.

Run: python3 docs/header/make_header.py
"""

import math
from pathlib import Path

W, H = 1280, 360
FONT = "Helvetica Neue, Helvetica, Arial, sans-serif"

BLACK = "#000000"
WHITE = "#ffffff"
GREY = "#8c8c8c"
# Neon on black: everything but the tags is light, not paint. Three colors, one job each.
CYAN = "#00c8ff"  # Viper cyan blue: the robot, its cameras and where it has been
PURPLE = "#b98cff"  # Purple: what the cameras see
YELLOW = "#ffd23f"  # Waffles yellow: the SLAM's answer, the solved path and its uncertainty
GRID = "#2b1f5c"  # Purple, dimmed for the carpet
EDGE = "#5b46a8"  # Purple, dimmed for the field's edge

# ---- Camera ------------------------------------------------------------------------------------

EYE = (-2.0, -8.5, 3.0)
TARGET = (9.8, 4.0, 0.8)
FOCAL = 1250.0
CX, CY = 930.0, 160.0


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def norm(a):
    n = math.sqrt(dot(a, a))
    return (a[0] / n, a[1] / n, a[2] / n)


FWD = norm(sub(TARGET, EYE))
RIGHT = norm(cross(FWD, (0, 0, 1)))
UP = cross(RIGHT, FWD)


def project(p):
    """World point (x, y, z up, meters) to screen (x, y)."""
    d = sub(p, EYE)
    z = dot(d, FWD)
    return (CX + FOCAL * dot(d, RIGHT) / z, CY - FOCAL * dot(d, UP) / z)


def f(v):
    return f"{v:.1f}"


def pts(ps):
    return " ".join(f"{f(x)},{f(y)}" for x, y in (project(p) for p in ps))


def line(a, b, color, width=1.0, opacity=1.0, dash=None):
    pa, pb = project(a), project(b)
    extra = f' stroke-dasharray="{dash}"' if dash else ""
    return (
        f'<line x1="{f(pa[0])}" y1="{f(pa[1])}" x2="{f(pb[0])}" y2="{f(pb[1])}" '
        f'stroke="{color}" stroke-width="{width}" opacity="{opacity}" '
        f'stroke-linecap="round"{extra}/>'
    )


def footprint(x, y, yaw, size, z=0.01):
    """A robot's square footprint on the carpet, with a nose tick, as world points."""
    c, s = math.cos(yaw), math.sin(yaw)
    h = size / 2
    sq = [(x + c * u - s * v, y + s * u + c * v, z) for u, v in ((h, h), (-h, h), (-h, -h), (h, -h))]
    nose = ((x + c * h * 0.4, y + s * h * 0.4, z), (x + c * h * 1.25, y + s * h * 1.25, z))
    return sq, nose


CAMERA_HEIGHT = 0.25
CAMERA_FOV = math.radians(90)


def robot(x, y, yaw, color, width, opacity=1.0, dash=None, cameras=False):
    """
    A swerve robot in outline: the bumper box, edges only. With cameras, two sit low over the
    front swerve modules, each yawed 45 degrees outward; returns their (position, yaw).
    """
    c, s = math.cos(yaw), math.sin(yaw)

    def at(u, v, z):
        return (x + c * u - s * v, y + s * u + c * v, z)

    out = []
    # True scale: a 26 x 26 in frame (0.66 m) inside 3.5 in bumpers (0.84 m outside)
    h, z0, z1 = 0.42, 0.02, 0.15
    lo = [at(u, v, z0) for u, v in ((h, h), (-h, h), (-h, -h), (h, -h))]
    hi = [at(u, v, z1) for u, v in ((h, h), (-h, h), (-h, -h), (h, -h))]
    for i in range(4):
        j = (i + 1) % 4
        out.append(line(lo[i], lo[j], color, width, opacity, dash))
        out.append(line(hi[i], hi[j], color, width, opacity, dash))
        out.append(line(lo[i], hi[i], color, width, opacity, dash))
    frame = [at(u, v, z1) for u, v in ((0.33, 0.33), (-0.33, 0.33), (-0.33, -0.33), (0.33, -0.33))]
    for i in range(4):
        out.append(line(frame[i], frame[(i + 1) % 4], color, width * 0.6, opacity * 0.7, dash))
    nose = at(h * 0.3, 0.0, z1), at(h * 1.2, 0.0, z1)
    out.append(line(*nose, color, width, opacity, dash))
    if not cameras:
        return out, []

    mounts = []
    for side in (1, -1):
        # Over the front swerve module, yawed 45 degrees outward
        cam_yaw = yaw + side * math.pi / 4
        pos = at(0.27, side * 0.27, CAMERA_HEIGHT)
        mounts.append((pos, cam_yaw))
        cc, cs = math.cos(cam_yaw), math.sin(cam_yaw)

        def cam(u, v, w):
            return (pos[0] + cc * u - cs * v, pos[1] + cs * u + cc * v, pos[2] + w)

        # A Limelight-sized body (10 x 8 x 5 cm), then a short frustum showing where it looks
        body = [cam(u, v, w) for w in (-0.025, 0.025)
                for u, v in ((0.0, 0.04), (-0.1, 0.04), (-0.1, -0.04), (0.0, -0.04))]
        for i in range(4):
            j = (i + 1) % 4
            out.append(line(body[i], body[j], color, width * 0.6, opacity))
            out.append(line(body[i + 4], body[j + 4], color, width * 0.6, opacity))
            out.append(line(body[i], body[i + 4], color, width * 0.6, opacity))
        far = [cam(0.2, u, v) for u, v in ((0.11, 0.07), (-0.11, 0.07), (-0.11, -0.07), (0.11, -0.07))]
        for i in range(4):
            out.append(line(pos, far[i], color, width * 0.5, opacity * 0.8))
            out.append(line(far[i], far[(i + 1) % 4], color, width * 0.5, opacity * 0.8))
    return out, mounts


def tag(center, yaw, size, code):
    """A filled AprilTag facing yaw: white margin, black border, a 4x4 code. Returns corners."""
    nx, ny = math.cos(yaw), math.sin(yaw)
    ux, uy = -ny, nx  # Across the face, horizontally
    cx, cy, cz = center
    h = size / 2

    def at(u, v):
        return (cx + ux * u, cy + uy * u, cz + v)

    def cell(i, j, n, color):
        """Cell (row i from the top, column j) of an n x n grid over the tag."""
        step = size / n
        u0, v0 = -h + j * step, h - (i + 1) * step
        quad = [at(u0, v0), at(u0 + step, v0), at(u0 + step, v0 + step), at(u0, v0 + step)]
        return f'<polygon points="{pts(quad)}" fill="{color}"/>'

    out = [cell(0, 0, 1, WHITE)]
    inner = [at(-h * 0.75, -h * 0.75), at(h * 0.75, -h * 0.75), at(h * 0.75, h * 0.75),
             at(-h * 0.75, h * 0.75)]
    out.append(f'<polygon points="{pts(inner)}" fill="{BLACK}"/>')
    for i in range(4):
        for j in range(4):
            if code[i * 4 + j]:
                out.append(cell(i + 2, j + 2, 8, WHITE))
    out.append(line((cx, cy, 0.0), (cx, cy, cz - h), GREY, 1.0, 0.4))
    # The detector measures the black square's corners, not the white margin's
    return out, inner


def path(t):
    """The robot's true path across the field, t in [0, 1]."""
    x = 2.6 + 5.4 * t
    y = 0.2 + 3.0 * t + 0.6 * math.sin(math.pi * 1.7 * t)
    return x, y


def heading(t):
    x0, y0 = path(max(0.0, t - 0.01))
    x1, y1 = path(min(1.0, t + 0.01))
    return math.atan2(y1 - y0, x1 - x0)


def main():
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" '
        f'height="{H}" role="img" aria-label="sotaTemplate: a 2027 FRC swerve template with '
        f'built-in SLAM">',
        "<defs>",
        f'<linearGradient id="veil" x1="0" x2="1">'
        f'<stop offset="0" stop-color="{BLACK}" stop-opacity="1"/>'
        f'<stop offset="0.3" stop-color="{BLACK}" stop-opacity="0.9"/>'
        f'<stop offset="0.48" stop-color="{BLACK}" stop-opacity="0"/></linearGradient>',
        '<filter id="glow" x="-10%" y="-10%" width="120%" height="120%">'
        '<feGaussianBlur stdDeviation="2" result="b"/>'
        '<feMerge><feMergeNode in="b"/><feMergeNode in="SourceGraphic"/></feMerge></filter>',
        "</defs>",
        f'<rect width="{W}" height="{H}" fill="{BLACK}"/>',
    ]

    svg.append('<g filter="url(#glow)">')

    # ---- Carpet grid, fading toward the far wall, and the field's edge ----
    for gx in range(0, 17):
        svg.append(line((gx, -2.0, 0), (gx, 8.2, 0), GRID, 1.0, max(0.2, 1 - gx / 20)))
    for gy in range(-2, 9):
        svg.append(line((0, gy, 0), (16.5, gy, 0), GRID, 1.0, max(0.2, 1 - (gy + 2) / 13)))
    edge = [(0, -2.0, 0), (16.5, -2.0, 0), (16.5, 8.2, 0), (0, 8.2, 0)]
    wall = [(x, y, 0.5) for x, y, _ in edge]
    for i in range(4):
        j = (i + 1) % 4
        svg.append(line(edge[i], edge[j], EDGE, 1.0, 0.6))
        svg.append(line(wall[i], wall[j], EDGE, 0.8, 0.35))
        svg.append(line(edge[i], wall[i], EDGE, 0.8, 0.35))

    # ---- Tags on posts around the field, facing in ----
    tags = [
        ((13.6, 2.6, 0.9), math.radians(175)),
        ((8.8, 7.6, 1.2), math.radians(-100)),
        ((15.6, 5.6, 1.1), math.radians(-170)),
    ]
    codes = [
        [0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0],
        [1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1],
        [0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1],
    ]
    tag_corners = []
    svg.append("</g>")
    for (center, yaw), code in zip(tags, codes):
        # True scale: a 6.5 in (0.165 m) black square, about 8.7 in with its white margin
        s, corners = tag(center, yaw, 0.165 / 0.75, code)
        svg.extend(s)
        tag_corners.append(corners)

    svg.append('<g filter="url(#glow)">')

    # ---- Raw odometry drifts away; its ghost robot ends up beside the real one ----
    t_end = 0.97
    odo = [path(k / 59 * t_end) for k in range(60)]
    odo = [(x + 0.3 * k / 59, y - 1.0 * (k / 59) ** 2, 0.01) for k, (x, y) in enumerate(odo)]
    svg.append(
        f'<polyline points="{pts(odo)}" fill="none" stroke="{GREY}" stroke-width="1.1" '
        f'stroke-dasharray="4 5" opacity="0.7"/>'
    )
    gx, gy, _ = odo[-1]
    ghost, _ = robot(gx, gy, heading(t_end) - 0.25, GREY, 1.0, 0.6, dash="3 4")
    svg.extend(ghost)

    # ---- Past poses: footprints and uncertainty, growing blind, collapsing on a sighting ----
    est = [(*path(k / 59 * t_end), 0.01) for k in range(60)]
    svg.append(
        f'<polyline points="{pts(est)}" fill="none" stroke="{YELLOW}" stroke-width="1.4" '
        f'opacity="0.85"/>'
    )
    n = 11
    seen = {0, 1, 6, 7, 10}
    sigma = 0.08
    for i in range(n):
        t = i / (n - 1) * 0.86
        sigma = 0.08 if i in seen else sigma + 0.1
        x, y = path(t)
        ring = [(x + sigma * 2.6 * math.cos(a), y + sigma * 2.6 * math.sin(a), 0.01)
                for a in (2 * math.pi * k / 32 for k in range(32))]
        svg.append(
            f'<polygon points="{pts(ring)}" fill="none" stroke="{YELLOW}" '
            f'stroke-opacity="0.6" stroke-width="0.9"/>'
        )
        fade = 0.25 + 0.45 * i / (n - 1)
        sq, nose = footprint(x, y, heading(t), 0.84)
        svg.append(
            f'<polygon points="{pts(sq)}" fill="none" stroke="{CYAN}" stroke-width="1" '
            f'opacity="{fade:.2f}" stroke-linejoin="round"/>'
        )
        svg.append(line(*nose, CYAN, 1.0, fade))
        if i in seen:
            k = min(range(len(tags)), key=lambda q: math.dist((x, y), tags[q][0][:2]))
            svg.append(line((x, y, CAMERA_HEIGHT), tags[k][0], PURPLE, 0.8, 0.25))

    # ---- The robot and its two cameras ----
    rx, ry = path(t_end)
    body, cameras = robot(rx, ry, heading(t_end), CYAN, 2.0, cameras=True)
    svg.extend(body)

    # ---- Rays from each camera to every corner of the tags in its view ----
    for pos, cam_yaw in cameras:
        for (center, _), corners in zip(tags, tag_corners):
            bearing = math.atan2(center[1] - pos[1], center[0] - pos[0]) - cam_yaw
            if abs(math.atan2(math.sin(bearing), math.cos(bearing))) > CAMERA_FOV / 2:
                continue
            for corner in corners:
                svg.append(line(pos, corner, PURPLE, 1.0, 0.85))
                p = project(corner)
                svg.append(f'<circle cx="{f(p[0])}" cy="{f(p[1])}" r="2.3" fill="{PURPLE}"/>')
        c0 = project(pos)
        svg.append(f'<circle cx="{f(c0[0])}" cy="{f(c0[1])}" r="2.8" fill="{PURPLE}"/>')
    svg.append("</g>")

    # ---- Wordmark over a veil of dark ----
    svg.append(f'<rect width="{W}" height="{H}" fill="url(#veil)"/>')
    svg.append(
        f'<text x="72" y="168" fill="{WHITE}" font-family="{FONT}" font-size="76" '
        f'font-weight="700" letter-spacing="-2">sota<tspan font-weight="300">Template</tspan>'
        f"</text>"
    )
    svg.append(
        f'<text x="76" y="212" fill="{WHITE}" font-family="{FONT}" font-size="24" '
        f'font-weight="400">FRC swerve with built-in SLAM</text>'
    )
    svg.append(
        f'<text x="76" y="246" fill="{GREY}" font-family="{FONT}" font-size="15" '
        f'font-weight="400">For 2027 Systemcore, with Commands v3, AdvantageKit and BLine</text>'
    )
    svg.append("</svg>")

    out = Path(__file__).with_name("sota-github-header.svg")
    out.write_text("\n".join(svg) + "\n")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
