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

EYE = (1.0, -9.5, 3.6)
TARGET = (11.0, 4.0, 0.6)
FOCAL = 1150.0
CX, CY = 860.0, 175.0


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


# Neon glow from wide, faint strokes under each crisp one. Pure vector, so it stays sharp at any
# size; an SVG blur filter is rasterized at the file's size and goes soft on high-DPI screens.
GLOW = ((5.0, 0.07), (2.6, 0.16))


def line(a, b, color, width=1.0, opacity=1.0, dash=None, glow=False):
    pa, pb = project(a), project(b)
    extra = f' stroke-dasharray="{dash}"' if dash else ""
    layers = [(width * k, opacity * o) for k, o in GLOW] if glow else []
    return "".join(
        f'<line x1="{f(pa[0])}" y1="{f(pa[1])}" x2="{f(pb[0])}" y2="{f(pb[1])}" '
        f'stroke="{color}" stroke-width="{w:.2f}" opacity="{o:.3f}" '
        f'stroke-linecap="round"{extra}/>'
        for w, o in layers + [(width, opacity)]
    )


def shape(kind, ps, color, width=1.0, opacity=1.0, dash=None, glow=False):
    """An outlined polyline or polygon in 3D, optionally glowing."""
    extra = f' stroke-dasharray="{dash}"' if dash else ""
    layers = [(width * k, opacity * o) for k, o in GLOW] if glow else []
    return "".join(
        f'<{kind} points="{pts(ps)}" fill="none" stroke="{color}" stroke-width="{w:.2f}" '
        f'opacity="{o:.3f}" stroke-linejoin="round"{extra}/>'
        for w, o in layers + [(width, opacity)]
    )


def footprint(x, y, yaw, size, z=0.01):
    """A robot's square footprint on the carpet, with a nose tick, as world points."""
    c, s = math.cos(yaw), math.sin(yaw)
    h = size / 2
    square = ((h, h), (-h, h), (-h, -h), (h, -h))
    sq = [(x + c * u - s * v, y + s * u + c * v, z) for u, v in square]
    nose = ((x + c * h * 0.4, y + s * h * 0.4, z), (x + c * h * 1.25, y + s * h * 1.25, z))
    return sq, nose


CAMERA_HEIGHT = 0.25
CAMERA_FOV = math.radians(90)
CAMERA_RANGE = 10.0

# The real 2026 FRC field (REBUILT, welded): WPILib's 2026-rebuilt-welded.json, as
# (id, x, y, z, yaw in degrees). The layout gives tags only, so the field elements below are
# outlined from where their tags sit.
FIELD_LENGTH, FIELD_WIDTH = 16.541, 8.069

# Each alliance's barrier, wall to wall: trench, bump, hub, bump, trench. Dimensions from the 2026
# Game Manual (section 5) where it gives them; "est." marks ones read off its figures.
HUB_X = (4.022, 11.312)  # Faces of each hub, from its tags
HUB_SIZE = 1.207  # 47 in square, plus its tag panels
HUB_Y = (3.431, 4.638)
HUB_HEIGHT = 1.35  # The body (est.)...
FUNNEL_TOP = 1.83  # ...then a hexagonal hopper flaring out to an opening 72 in off the carpet,
FUNNEL_RIM = 0.612  # 41.7 in across its flats,
FUNNEL_BASE = 0.34  # from a narrower base (est.)
TRENCH_X = (4.626, 11.916)  # Center of each arm: its two tags sit back to back
TRENCH_OPENING, TRENCH_WIDTH = 1.279, 1.668  # From the guardrail: the opening, then a pedestal
TRENCH_CLEARANCE, TRENCH_ARM = 0.565, 0.1  # 22.25 in under the arm; the arm (est.)
TRENCH_DEPTH, TRENCH_TOP = 1.194, 1.022  # 47 in deep at the pedestal; 40.25 in tall overall
TAG_PANEL = 0.267  # 10.5 in square panels hold the tags
BUMP_LENGTH, BUMP_DEPTH, BUMP_HEIGHT = 1.854, 1.128, 0.1654  # A ridge of two 15 degree ramps
WALL_HEIGHT, WALL_PANEL = 0.508, 1.5  # Guardrails: 20 in of clear polycarbonate, and a rail
DS_BASE, DS_TOP = 0.935, 2.0  # Driver stations: diamond plate base, then a clear sheet and rail
FIELD_TAGS = [
    (1, 11.878, 7.425, 0.889, 180), (2, 11.915, 4.638, 1.124, 90), (3, 11.312, 4.390, 1.124, 180),
    (4, 11.312, 4.035, 1.124, 180), (5, 11.915, 3.431, 1.124, -90), (6, 11.878, 0.644, 0.889, 180),
    (7, 11.953, 0.644, 0.889, 0), (8, 12.271, 3.431, 1.124, -90), (9, 12.519, 3.679, 1.124, 0),
    (10, 12.519, 4.035, 1.124, 0), (11, 12.271, 4.638, 1.124, 90), (12, 11.953, 7.425, 0.889, 0),
    (13, 16.533, 7.403, 0.552, 180), (14, 16.533, 6.972, 0.552, 180),
    (15, 16.533, 4.324, 0.552, 180), (16, 16.533, 3.892, 0.552, 180),
    (17, 4.663, 0.644, 0.889, 0), (18, 4.626, 3.431, 1.124, -90), (19, 5.229, 3.679, 1.124, 0),
    (20, 5.229, 4.035, 1.124, 0), (21, 4.626, 4.638, 1.124, 90), (22, 4.663, 7.425, 0.889, 0),
    (23, 4.588, 7.425, 0.889, 180), (24, 4.270, 4.638, 1.124, 90), (25, 4.022, 4.390, 1.124, 180),
    (26, 4.022, 4.035, 1.124, 180), (27, 4.270, 3.431, 1.124, -90), (28, 4.588, 0.644, 0.889, 180),
    (29, 0.008, 0.666, 0.552, 0), (30, 0.008, 1.098, 0.552, 0), (31, 0.008, 3.746, 0.552, 0),
    (32, 0.008, 4.178, 0.552, 0),
]


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
        out.append(line(lo[i], lo[j], color, width, opacity, dash, glow=dash is None))
        out.append(line(hi[i], hi[j], color, width, opacity, dash, glow=dash is None))
        out.append(line(lo[i], hi[i], color, width, opacity, dash, glow=dash is None))
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
        view = ((0.11, 0.07), (-0.11, 0.07), (-0.11, -0.07), (0.11, -0.07))
        far = [cam(0.2, u, v) for u, v in view]
        for i in range(4):
            out.append(line(pos, far[i], color, width * 0.5, opacity * 0.8))
            out.append(line(far[i], far[(i + 1) % 4], color, width * 0.5, opacity * 0.8))
    return out, mounts


def tag(center, yaw, size, code, post=False):
    """
    A filled AprilTag facing yaw, laid out like 36h11: a 10 x 10 grid of a white margin, a black
    border and 6 x 6 data cells. Returns the black square's corners.
    """
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

    b = h * 0.8
    inner = [at(-b, -b), at(b, -b), at(b, b), at(-b, b)]
    if nx * (EYE[0] - cx) + ny * (EYE[1] - cy) <= 0:
        return [], inner  # Seen from behind: hidden by whatever it's mounted on
    out = [cell(0, 0, 1, WHITE), f'<polygon points="{pts(inner)}" fill="{BLACK}"/>']
    for i in range(6):
        for j in range(6):
            if code[i * 6 + j]:
                out.append(cell(i + 2, j + 2, 10, WHITE))
    if post:
        out.append(line((cx, cy, 0.0), (cx, cy, cz - h), GREY, 1.0, 0.4))
    # The detector measures the black square's corners, not the white margin's
    return out, inner


def prism(center, radius, height, sides, rotation, color, opacity):
    """A field element in outline: a regular prism standing on the carpet."""
    cx, cy = center
    ring = [(cx + radius * math.cos(rotation + 2 * math.pi * k / sides),
             cy + radius * math.sin(rotation + 2 * math.pi * k / sides)) for k in range(sides)]
    out = []
    for k in range(sides):
        a, b = ring[k], ring[(k + 1) % sides]
        out.append(line((*a, 0.0), (*b, 0.0), color, 1.0, opacity))
        out.append(line((*a, height), (*b, height), color, 1.0, opacity))
        out.append(line((*a, 0.0), (*a, height), color, 0.8, opacity * 0.8))
    return out


def blocked(a, b):
    """Whether a hub stands between two points (in plan; hubs are taller than any sightline)."""
    for x0 in HUB_X:
        # Shrunk a little, so a tag on the hub's own face doesn't block itself
        lo = (x0 + 0.02, HUB_Y[0] + 0.02)
        hi = (x0 + HUB_SIZE - 0.02, HUB_Y[1] - 0.02)
        t0, t1 = 0.0, 1.0
        for axis in (0, 1):
            d = b[axis] - a[axis]
            if abs(d) < 1e-9:
                if not lo[axis] <= a[axis] <= hi[axis]:
                    break
                continue
            ta, tb = (lo[axis] - a[axis]) / d, (hi[axis] - a[axis]) / d
            t0, t1 = max(t0, min(ta, tb)), min(t1, max(ta, tb))
        else:
            if t0 <= t1:
                return True
    return False


def tags_in_view(pos, cam_yaw, tags):
    """The tags a camera at pos, looking along cam_yaw, can see the front of: in its field of view
    and range, and not behind a hub."""
    seen = []
    for k, (center, yaw) in enumerate(tags):
        bearing = math.atan2(center[1] - pos[1], center[0] - pos[0]) - cam_yaw
        facing = math.cos(yaw) * (pos[0] - center[0]) + math.sin(yaw) * (pos[1] - center[1])
        in_fov = abs(math.atan2(math.sin(bearing), math.cos(bearing))) <= CAMERA_FOV / 2
        in_range = math.dist(pos[:2], center[:2]) <= CAMERA_RANGE
        if in_fov and facing > 0 and in_range and not blocked(pos, center):
            seen.append(k)
    return seen


def solid(faces, color, opacity):
    """
    A convex solid in outline. Its front faces are filled with the background's black, so it hides
    whatever is behind it without adding any color; faces are wound counter-clockwise from outside.
    """
    visible = []
    for face in faces:
        normal = cross(sub(face[1], face[0]), sub(face[2], face[0]))
        if dot(normal, sub(EYE, face[0])) > 0:
            # Far to near, so a part nearer the viewer is drawn over one behind it
            center = tuple(sum(v[i] for v in face) / len(face) for i in range(3))
            visible.append((math.dist(EYE, center), face))
    return [
        f'<polygon points="{pts(face)}" fill="{BLACK}" stroke="{color}" '
        f'stroke-width="1" stroke-opacity="{opacity}" stroke-linejoin="round"/>'
        for _, face in sorted(visible, key=lambda v: -v[0])
    ]


def box(x0, x1, y0, y1, z0, z1):
    """The faces of an axis-aligned box."""
    return [
        [(x0, y0, z0), (x0, y1, z0), (x1, y1, z0), (x1, y0, z0)],
        [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)],
        [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)],
        [(x0, y1, z0), (x0, y1, z1), (x1, y1, z1), (x1, y1, z0)],
        [(x0, y0, z0), (x0, y0, z1), (x0, y1, z1), (x0, y1, z0)],
        [(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)],
    ]


def bump(cx, y0, y1):
    """The faces of a bump: a ramp up to a ridge and straight down again, from y0 to y1."""
    b, h = BUMP_DEPTH / 2, BUMP_HEIGHT
    return [
        [(cx + b, y0, 0), (cx + b, y1, 0), (cx, y1, h), (cx, y0, h)],
        [(cx - b, y0, 0), (cx, y0, h), (cx, y1, h), (cx - b, y1, 0)],
        [(cx - b, y0, 0), (cx + b, y0, 0), (cx, y0, h)],
        [(cx - b, y1, 0), (cx, y1, h), (cx + b, y1, 0)],
    ]


def side_wall(y):
    """A side wall of the perimeter. It's clear, so it's drawn as its frame: rail, base, posts."""
    out = [
        line((0, y, WALL_HEIGHT), (FIELD_LENGTH, y, WALL_HEIGHT), EDGE, 1.4, 0.95, glow=True),
        line((0, y, 0), (FIELD_LENGTH, y, 0), EDGE, 1.0, 0.7),
    ]
    posts = round(FIELD_LENGTH / WALL_PANEL)
    for k in range(posts + 1):
        x = FIELD_LENGTH * k / posts
        out.append(line((x, y, 0), (x, y, WALL_HEIGHT), EDGE, 0.9, 0.6))
    return out


def funnel(cx, cy):
    """The faces of the hub's hexagonal hopper, flaring outward, and the outline of its open rim."""

    def ring(radius, z):
        # Corners point along the field's width, as in the manual's top view
        angles = [math.pi / 2 + math.pi * k / 3 for k in range(6)]
        return [(cx + radius * math.cos(a), cy + radius * math.sin(a), z) for a in angles]

    lo, hi = ring(FUNNEL_BASE, HUB_HEIGHT), ring(FUNNEL_RIM, FUNNEL_TOP)
    faces = [[lo[k], lo[(k + 1) % 6], hi[(k + 1) % 6], hi[k]] for k in range(6)]
    # The top is open: the whole rim shows, its far edges through the opening
    return faces, [shape("polygon", hi, EDGE, 1.2, 0.9, glow=True)]


def trench(cx, side):
    """
    The faces of a trench, from the guardrail at y = 0 (side 1) or the far one (side -1): an arm
    over the opening, the bracket for its tags on top, and the pedestal beyond. Returns the faces
    and the opening's center.
    """
    def y(d):
        return d if side > 0 else FIELD_WIDTH - d

    def span(d0, d1):
        return min(y(d0), y(d1)), max(y(d0), y(d1))

    arm = box(cx - TRENCH_ARM / 2, cx + TRENCH_ARM / 2, *span(0.0, TRENCH_WIDTH),
              TRENCH_CLEARANCE, TRENCH_CLEARANCE + TRENCH_ARM)
    mid = TRENCH_OPENING / 2
    bracket = box(cx - 0.0375, cx + 0.0375, *span(mid - TAG_PANEL / 2, mid + TAG_PANEL / 2),
                  TRENCH_CLEARANCE + TRENCH_ARM, TRENCH_TOP)
    # The pedestal: sloped front and back, like a squat ridge, between the opening and the bump
    y0, y1 = span(TRENCH_OPENING, TRENCH_WIDTH)
    b, t, h = TRENCH_DEPTH / 2, 0.15, TRENCH_CLEARANCE + TRENCH_ARM
    pedestal = [
        [(cx - t, y0, h), (cx + t, y0, h), (cx + t, y1, h), (cx - t, y1, h)],
        [(cx + b, y0, 0), (cx + b, y1, 0), (cx + t, y1, h), (cx + t, y0, h)],
        [(cx - b, y0, 0), (cx - t, y0, h), (cx - t, y1, h), (cx - b, y1, 0)],
        [(cx - b, y0, 0), (cx + b, y0, 0), (cx + t, y0, h), (cx - t, y0, h)],
        [(cx - b, y1, 0), (cx - t, y1, h), (cx + t, y1, h), (cx + b, y1, 0)],
    ]
    return arm + bracket + pedestal, y(mid)


def driver_station_glass(x):
    """The clear sheet above an alliance wall's base: its frame, rail and station dividers."""
    out = [line((x, 0, DS_TOP), (x, FIELD_WIDTH, DS_TOP), EDGE, 1.3, 0.9, glow=True),
           line((x, 0, DS_BASE), (x, FIELD_WIDTH, DS_BASE), EDGE, 1.0, 0.7)]
    for k in range(4):
        y = FIELD_WIDTH * k / 3
        out.append(line((x, y, DS_BASE), (x, y, DS_TOP), EDGE, 1.0, 0.7))
    return out


def path(t):
    """The robot's true path, t in [0, 1]: across midfield toward the red hub."""
    x = 6.0 + 3.6 * t
    y = 0.9 + 2.2 * t + 0.4 * math.sin(math.pi * 1.7 * t)
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
        f'<stop offset="0.22" stop-color="{BLACK}" stop-opacity="0.85"/>'
        f'<stop offset="0.42" stop-color="{BLACK}" stop-opacity="0"/></linearGradient>',
        "</defs>",
        f'<rect width="{W}" height="{H}" fill="{BLACK}"/>',
    ]

    # ---- Carpet grid, fading toward the far wall, and the field's edge ----
    for gx in range(0, 17):
        svg.append(line((gx, 0, 0), (gx, FIELD_WIDTH, 0), GRID, 1.0, max(0.2, 1 - gx / 20)))
    for gy in range(0, 9):
        svg.append(line((0, gy, 0), (FIELD_LENGTH, gy, 0), GRID, 1.0, max(0.2, 1 - gy / 11)))
    # The far side wall is behind everything; the near one is drawn last, in front
    svg.extend(side_wall(FIELD_WIDTH))

    # ---- Every tag on the real field ----
    tags = [((x, y, z), math.radians(yaw)) for _, x, y, z, yaw in FIELD_TAGS]
    codes = [[(tid * 0x9E3779B97F4A7C15 >> k) & 1 for k in range(36)] for tid, *_ in FIELD_TAGS]
    tag_svg, tag_corners = [], []
    for (center, yaw), code in zip(tags, codes):
        # True scale: 8.125 in overall, a 6.5 in black square
        s, corners = tag(center, yaw, 0.2064, code)
        tag_svg.append(s)
        tag_corners.append(corners)

    # ---- Field elements, drawn far to near, each with the tags mounted on it ----
    elements = []  # (faces, center, anything drawn on top)
    for x in (0.0, FIELD_LENGTH):  # Alliance walls: a solid base, then clear glass
        base = box(x - 0.01, x + 0.01, 0.0, FIELD_WIDTH, 0.0, DS_BASE)
        elements.append((base, (x, FIELD_WIDTH / 2), driver_station_glass(x)))
    for x0, cx in zip(HUB_X, TRENCH_X):
        cone, extras = funnel(x0 + HUB_SIZE / 2, sum(HUB_Y) / 2)
        body = box(x0, x0 + HUB_SIZE, *HUB_Y, 0.0, HUB_HEIGHT)
        elements.append((body + cone, (cx, sum(HUB_Y) / 2), extras))
        for y0, y1 in ((HUB_Y[0] - BUMP_LENGTH, HUB_Y[0]), (HUB_Y[1], HUB_Y[1] + BUMP_LENGTH)):
            elements.append((bump(cx, y0, y1), (cx, (y0 + y1) / 2), []))
        for side in (1, -1):
            faces, yc = trench(cx, side)
            elements.append((faces, (cx, yc), []))

    def owner(center):
        """The element a tag is mounted on: the nearest, in plan."""
        return min(range(len(elements)),
                   key=lambda e: math.dist(center[:2], elements[e][1]))

    mounted = {e: [] for e in range(len(elements))}
    for k, (center, _) in enumerate(tags):
        mounted[owner(center)].append(k)
    for e in sorted(range(len(elements)),
                    key=lambda e: -math.dist(EYE[:2], elements[e][1])):
        svg.extend(solid(elements[e][0], EDGE, 0.75))
        svg.extend(elements[e][2])
        for k in mounted[e]:
            svg.extend(tag_svg[k])

    # ---- Raw odometry drifts away; its ghost robot ends up beside the real one ----
    t_end = 0.97
    odo = [path(k / 59 * t_end) for k in range(60)]
    odo = [(x + 0.3 * k / 59, y - 1.0 * (k / 59) ** 2, 0.01) for k, (x, y) in enumerate(odo)]
    svg.append(shape("polyline", odo, GREY, 1.1, 0.7, dash="4 5"))
    gx, gy, _ = odo[-1]
    ghost, _ = robot(gx, gy, heading(t_end) - 0.25, GREY, 1.0, 0.6, dash="3 4")
    svg.extend(ghost)

    # ---- Past poses: footprints and uncertainty, growing blind, collapsing on a sighting ----
    est = [(*path(k / 59 * t_end), 0.01) for k in range(60)]
    svg.append(shape("polyline", est, YELLOW, 1.4, 0.85, glow=True))
    n = 11
    seen = {0, 1, 2, 6, 7, 8, 10}
    web, nodes = [], []
    sigma = 0.08
    for i in range(n):
        t = i / (n - 1) * 0.86
        sigma = 0.08 if i in seen else sigma + 0.1
        x, y = path(t)
        ring = [(x + sigma * 2.6 * math.cos(a), y + sigma * 2.6 * math.sin(a), 0.01)
                for a in (2 * math.pi * k / 32 for k in range(32))]
        svg.append(shape("polygon", ring, YELLOW, 0.9, 0.6))
        fade = 0.25 + 0.45 * i / (n - 1)
        sq, nose = footprint(x, y, heading(t), 0.84)
        svg.append(shape("polygon", sq, CYAN, 1.0, fade))
        svg.append(line(*nose, CYAN, 1.0, fade))
        nodes.append(project((x, y, 0.01)))
        # The factor graph: every sighting from this pose ties it to a tag
        if i in seen:
            _, mounts = robot(x, y, heading(t), CYAN, 1.0, cameras=True)
            for pos, cam_yaw in mounts:
                for k in tags_in_view(pos, cam_yaw, tags):
                    web.append(line(pos, tags[k][0], PURPLE, 0.6, 0.14))
    svg.extend(web)
    for nx, ny in nodes:
        svg.append(f'<circle cx="{f(nx)}" cy="{f(ny)}" r="2.2" fill="{CYAN}"/>')

    # ---- The robot and its two cameras ----
    rx, ry = path(t_end)
    body, cameras = robot(rx, ry, heading(t_end), CYAN, 2.0, cameras=True)
    svg.extend(body)

    # ---- Rays from each camera to every corner of the tags in its view ----
    for pos, cam_yaw in cameras:
        for k in tags_in_view(pos, cam_yaw, tags):
            for corner in tag_corners[k]:
                svg.append(line(pos, corner, PURPLE, 1.0, 0.85, glow=True))
                p = project(corner)
                svg.append(f'<circle cx="{f(p[0])}" cy="{f(p[1])}" r="2.3" fill="{PURPLE}"/>')
        c0 = project(pos)
        svg.append(f'<circle cx="{f(c0[0])}" cy="{f(c0[1])}" r="2.8" fill="{PURPLE}"/>')

    svg.extend(side_wall(0.0))

    # ---- Wordmark over a veil of dark, each line outlined in black so the scene never crowds it
    svg.append(f'<rect width="{W}" height="{H}" fill="url(#veil)"/>')

    def text(x, y, size, weight, color, content):
        return (
            f'<text x="{x}" y="{y}" fill="{color}" stroke="{BLACK}" stroke-width="6" '
            f'stroke-linejoin="round" paint-order="stroke" font-family="{FONT}" '
            f'font-size="{size}" font-weight="{weight}"{content}</text>'
        )

    svg.append(text(72, 168, 76, 700, WHITE,
                    ' letter-spacing="-2">sota<tspan font-weight="300">Template</tspan>'))
    svg.append(text(76, 212, 24, 400, WHITE, ">FRC swerve with built-in SLAM"))
    svg.append(text(76, 246, 15, 400, GREY,
                    ">For 2027 Systemcore, with Commands v3, AdvantageKit and BLine"))
    svg.append("</svg>")

    out = Path(__file__).with_name("sota-github-header.svg")
    out.write_text("\n".join(svg) + "\n")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
