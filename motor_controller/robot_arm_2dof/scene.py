"""Dependency-free perspective 3D meshes rendered on a Tk canvas.

Camera controls only change projection. All joint poses come from the GUI's
feedback model; this module has no access to a CAN bus or motor session.
"""

from __future__ import annotations

import math
import tkinter as tk
from dataclasses import dataclass

from robot_arm_2dof.kinematics import ArmGeometry


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def mul(a, factor):
    return (a[0] * factor, a[1] * factor, a[2] * factor)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def normal(a):
    length = math.sqrt(dot(a, a))
    return mul(a, 1 / length) if length > 1e-12 else (0, 0, 1)


def transform(point, origin, angle=0):
    x, y, z = point
    c, s = math.cos(angle), math.sin(angle)
    return add(origin, (c * x - s * z, y, s * x + c * z))


@dataclass(frozen=True)
class Face:
    vertices: tuple
    color: str
    edge: str = ""


def box(origin, size, color, angle=0):
    x, y, z = (v / 2 for v in size)
    vertices = [transform(p, origin, angle) for p in
                ((-x, -y, -z), (x, -y, -z), (x, y, -z), (-x, y, -z),
                 (-x, -y, z), (x, -y, z), (x, y, z), (-x, y, z))]
    indices = ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
               (3, 7, 6, 2), (0, 4, 7, 3), (1, 2, 6, 5))
    return [Face(tuple(vertices[i] for i in face), color) for face in indices]


def cylinder(origin, radius, depth, color, angle=0, segments=24, axis="y"):
    """Cylinder axis is Y; angle rotates its face/bolt pattern around the axis."""
    rings = []
    for y in (-depth / 2, depth / 2):
        ring = []
        for i in range(segments):
            x = radius * math.cos(math.tau * i / segments + angle)
            z = radius * math.sin(math.tau * i / segments + angle)
            ring.append(add(origin, (x, y, z) if axis == "y" else (x, -z, y)))
        rings.append(ring)
    faces = [Face(tuple(rings[0]), color), Face(tuple(reversed(rings[1])), color)]
    for i in range(segments):
        j = (i + 1) % segments
        faces.append(Face((rings[0][i], rings[1][i], rings[1][j], rings[0][j]), color))
    return faces


def motor_mesh(center, angle, radius, accent):
    faces = cylinder(center, radius, 0.105, "#424c51", segments=32)
    # Cooling rings, front metal plate, encoder hub and visible mounting bolts.
    for y in (-0.035, -0.018, 0, 0.018, 0.035):
        faces += cylinder(add(center, (0, y, 0)), radius * 1.04, 0.006, "#252f34", segments=24)
    for sign in (-1, 1):
        front = add(center, (0, sign * 0.057, 0))
        faces += cylinder(front, radius * 0.97, 0.009, "#a8b5b9", segments=32)
        faces += cylinder(add(front, (0, sign * 0.006, 0)), radius * 0.74, 0.006, "#273a35")
        faces += cylinder(add(front, (0, sign * 0.010, 0)), radius * 0.57, 0.004, accent)
        faces += cylinder(add(front, (0, sign * 0.014, 0)), radius * 0.37, 0.011, "#14231e")
        for i in range(6):
            phase = i * math.tau / 6 + angle
            bolt = add(front, (radius * 0.86 * math.cos(phase), sign * 0.008,
                               radius * 0.86 * math.sin(phase)))
            faces += cylinder(bolt, 0.0035, 0.007, "#dde6e7", segments=6)
    # One asymmetric output marker makes shaft rotation visible.
    marker = transform((radius * 0.25, -0.079, 0), center, angle)
    faces += box(marker, (radius * 0.42, 0.003, 0.006), "#f4fff7", angle)
    return faces


def link_mesh(start, length, angle, accent):
    faces = []
    # Twin aluminium side rails and a dark recessed central spine.
    for y in (-0.036, 0.036):
        center = transform((length / 2, y, 0), start, angle)
        faces += box(center, (max(0.01, length - 0.05), 0.015, 0.066), "#b6c4cb", angle)
        stripe = transform((length / 2, y + math.copysign(0.008, y), 0), start, angle)
        faces += box(stripe, (max(0.005, length - 0.10), 0.0025, 0.012), accent, angle)
    faces += box(transform((length / 2, 0, 0), start, angle),
                 (max(0.01, length - 0.08), 0.04, 0.036), "#26353d", angle)
    for distance in (length * 0.26, length * 0.74):
        for sign in (-1, 1):
            center = transform((distance, sign * 0.047, 0.020), start, angle)
            faces += cylinder(center, 0.004, 0.006, "#35434a", segments=6)
    return faces


def arm_mesh(geometry, shoulder, elbow):
    base, middle, tip = geometry.points(shoulder, elbow)
    faces = box((0, 0, 0.018), (0.23, 0.21, 0.036), "#303e44")
    faces += box((0, 0, 0.039), (0.18, 0.16, 0.008), "#bccad0")
    faces += box((0, 0, 0.063), (0.13, 0.10, 0.046), "#24372f")
    for x in (-0.09, 0.09):
        for y in (-0.078, 0.078):
            faces += cylinder((x, y, 0.048), 0.006, 0.008, "#d8e0e0", segments=6, axis="z")
    for y in (-0.065, 0.065):
        faces += box((0, y, (0.081 + base[2]) / 2),
                     (0.09, 0.015, max(0.04, base[2] - 0.04)), "#8a9ca6")
    faces += link_mesh(base, geometry.upper_length, shoulder, "#2bf393")
    faces += motor_mesh(base, shoulder, 0.063, "#36d990")
    faces += link_mesh(middle, geometry.forearm_length, shoulder + elbow, "#50b9f2")
    faces += motor_mesh(middle, shoulder + elbow, 0.052, "#58bbf5")
    # Passive wrist and a small industrial parallel gripper (not a third DOF).
    angle = shoulder + elbow
    faces += cylinder(tip, 0.034, 0.082, "#788e9a")
    faces += box(transform((0.032, 0, 0), tip, angle), (0.052, 0.08, 0.045), "#273a43", angle)
    for y in (-0.035, 0.035):
        faces += box(transform((0.069, y, 0), tip, angle), (0.04, 0.015, 0.027), "#cad6dc", angle)
        faces += box(transform((0.092, y * 0.72, 0), tip, angle), (0.017, 0.026, 0.027), "#3b4b54", angle)
    return faces


class Camera:
    def __init__(self):
        self.home()

    def home(self):
        self.yaw = math.radians(-65)
        self.pitch = math.radians(22)
        self.zoom = 1.0

    def orbit(self, dx, dy):
        self.yaw += dx * 0.008
        self.pitch = max(math.radians(-15), min(math.radians(80), self.pitch + dy * 0.006))

    def zoom_by(self, factor):
        self.zoom = max(0.5, min(2.2, self.zoom * factor))

    def basis(self, geometry):
        reach = geometry.upper_length + geometry.forearm_length
        target = (reach * 0.17, 0.0, geometry.shoulder_height + reach * 0.19)
        distance = max(0.6, reach * 2.2) / self.zoom
        offset = (math.cos(self.pitch) * math.cos(self.yaw),
                  math.cos(self.pitch) * math.sin(self.yaw), math.sin(self.pitch))
        eye = add(target, mul(offset, distance))
        forward = normal(sub(target, eye))
        right = normal(cross(forward, (0, 0, 1)))
        up = cross(right, forward)
        return eye, right, up, forward


class ArmScene(tk.Canvas):
    def __init__(self, parent, geometry=None, **kwargs):
        super().__init__(parent, background="#080e12", highlightthickness=0, **kwargs)
        self.geometry = geometry or ArmGeometry()
        self.angles = (math.radians(55), math.radians(-80))
        self.camera = Camera()
        self.source = "CHƯA KẾT NỐI · TƯ THẾ LẮP"
        self.stale = (True, True)
        self.motor_ids = (1, 2)
        self.show_labels = tk.BooleanVar(self, value=True)
        self._dirty = True
        self._drag = None
        self._faces = None
        self._projection_cache = {}
        self.bind("<Configure>", lambda _: self.invalidate())
        self.bind("<ButtonPress-1>", self._start_drag)
        self.bind("<B1-Motion>", self._orbit)
        self.bind("<ButtonRelease-1>", lambda _: self.configure(cursor=""))
        self.bind("<MouseWheel>", self._zoom)
        self.bind("<Button-4>", lambda _: self.zoom(1.1))
        self.bind("<Button-5>", lambda _: self.zoom(1 / 1.1))
        self.bind("<Double-Button-1>", lambda _: self.home())
        self.show_labels.trace_add("write", lambda *_: self.invalidate())

    def invalidate(self):
        self._dirty = True

    def set_pose(self, angles):
        if self.angles != tuple(angles):
            self.angles = tuple(angles)
            self._faces = None
            self.invalidate()

    def set_geometry(self, geometry):
        self.geometry = geometry
        self._faces = None
        self.invalidate()

    def _start_drag(self, event):
        self._drag = (event.x, event.y)
        self.configure(cursor="fleur")

    def _orbit(self, event):
        if self._drag is not None:
            self.camera.orbit(event.x - self._drag[0], event.y - self._drag[1])
            self._drag = (event.x, event.y)
            self.invalidate()

    def _zoom(self, event):
        if event.delta:
            self.zoom(1.1 if event.delta > 0 else 1 / 1.1)

    def zoom(self, factor):
        self.camera.zoom_by(factor)
        self.invalidate()

    def home(self):
        self.camera.home()
        self.invalidate()

    def _project(self, point):
        if point in self._projection_cache:
            return self._projection_cache[point]
        eye, right, up, forward = self._basis
        x, y, dz = point[0] - eye[0], point[1] - eye[1], point[2] - eye[2]
        z = x * forward[0] + y * forward[1] + dz * forward[2]
        if z <= 0.015:
            self._projection_cache[point] = None
            return None
        result = (self._width * 0.5 + self._focal * (x * right[0] + y * right[1] + dz * right[2]) / z,
                  self._height * 0.5 - self._focal * (x * up[0] + y * up[1] + dz * up[2]) / z, z)
        self._projection_cache[point] = result
        return result

    def _line3d(self, points, color, width=1, **kwargs):
        projected = [self._project(p) for p in points]
        if all(p is not None for p in projected):
            self.create_line(*[v for p in projected for v in p[:2]], fill=color, width=width, **kwargs)

    def render(self):
        if not self._dirty or self.winfo_width() < 20 or self.winfo_height() < 20:
            return
        self._dirty = False
        self.delete("all")
        self._basis = self.camera.basis(self.geometry)
        self._width, self._height = self.winfo_width(), self.winfo_height()
        self._focal = min(self._width, self._height) * 1.55
        self._projection_cache.clear()
        # Ground grid, units in meters, with the arm's reference frame at its base.
        reach = max(0.5, self.geometry.upper_length + self.geometry.forearm_length)
        step = reach / 8
        for i in range(-10, 11):
            a = i * step
            self._line3d(((a, -reach, 0), (a, reach, 0)), "#1a292f")
            self._line3d(((-reach, a, 0), (reach, a, 0)), "#1a292f")
        self._line3d(((0, 0, 0.002), (0.2, 0, 0.002)), "#45816d", 2)
        self._line3d(((0, 0, 0.002), (0, 0.2, 0.002)), "#395e92", 2)
        self._line3d(((0, 0, 0), (0, 0, 0.24)), "#77844c", 2)
        if self._faces is None:
            self._faces = arm_mesh(self.geometry, *self.angles)
        faces = []
        light = normal((-0.3, -0.6, 1))
        colors = {}
        for face in self._faces:
            n = normal(cross(sub(face.vertices[1], face.vertices[0]),
                             sub(face.vertices[2], face.vertices[0])))
            if dot(n, sub(self._basis[0], face.vertices[0])) <= 0:
                continue
            projected = [self._project(v) for v in face.vertices]
            if any(p is None for p in projected):
                continue
            brightness = 0.52 + 0.48 * max(0, dot(n, light))
            if face.color not in colors:
                colors[face.color] = [int(face.color[i:i + 2], 16) for i in (1, 3, 5)]
            rgb = colors[face.color]
            color = "#" + "".join(f"{min(255, int(v * brightness)):02x}" for v in rgb)
            faces.append((sum(p[2] for p in projected) / len(projected), projected, color))
        commands = []
        for _, points, color in sorted(faces, key=lambda face: face[0], reverse=True):
            commands.append((self._w, "create", "polygon", *[v for p in points for v in p[:2]],
                             "-fill", color, "-outline", color, "-width", 1))
        # Send structured Tcl lists in one call rather than a Python/Tk round
        # trip per face. Coordinates stay numeric; no text is interpolated.
        if commands:
            self.tk.call("foreach", "_arm_mesh_command", tuple(commands), "uplevel #0 $_arm_mesh_command")
        if self.show_labels.get():
            base, middle, tip = self.geometry.points(*self.angles)
            for i, point in enumerate((base, middle)):
                projected = self._project(add(point, (0, -0.08, 0)))
                if projected is None:
                    continue
                x, y = projected[:2]
                endx = max(130, min(self.winfo_width() - 110, x - 95 if i == 0 else x + 100))
                endy = max(85, min(self.winfo_height() - 60, y - 70))
                color = "#39ff9a" if i == 0 else "#58c5ff"
                self.create_line(x, y, endx, endy, fill=color, width=1)
                self.create_oval(x - 3, y - 3, x + 3, y + 3, fill=color, outline="")
                title = "J1 / VAI" if i == 0 else "J2 / KHUỶU"
                self.create_text(endx, endy - 21, text=f"{title} · CAN 0x{self.motor_ids[i]:02X}",
                                 fill=color, font=("Segoe UI", 10, "bold"))
                suffix = "  [cũ]" if self.stale[i] else ""
                self.create_text(endx, endy - 3, text=f"{math.degrees(self.angles[i]):+.1f}°{suffix}",
                                 fill="#f5fff9", font=("Consolas", 13, "bold"))
            p = self._project(tip)
            if p:
                self.create_oval(p[0] - 4, p[1] - 4, p[0] + 4, p[1] + 4,
                                 outline="#e4f3ff", width=2)
        self.create_text(20, 20, anchor="nw", text=self.source,
                         fill="#39ff9a", font=("Segoe UI", 10, "bold"))
        self.create_text(20, 45, anchor="nw", text="2 DOF  /  SHOULDER + ELBOW",
                         fill="#82969f", font=("Consolas", 10))
        self.create_text(20, self.winfo_height() - 20, anchor="sw",
                         text="Kéo chuột: xoay camera  ·  Cuộn: zoom  ·  Nhấp đúp: góc nhìn gốc",
                         fill="#a8bdc7", font=("Segoe UI", 9))
        self._orientation_widget()

    def _orientation_widget(self):
        x, y = self.winfo_width() - 65, self.winfo_height() - 66
        for axis, title, color in (((1, 0, 0), "X", "#6af9ab"),
                                   ((0, 1, 0), "Y", "#64bfff"),
                                   ((0, 0, 1), "Z", "#f4cf76")):
            dx, dy = dot(axis, self._basis[1]) * 28, -dot(axis, self._basis[2]) * 28
            self.create_line(x, y, x + dx, y + dy, fill=color, width=2, arrow="last")
            self.create_text(x + dx * 1.3, y + dy * 1.3, text=title, fill=color, font=("Consolas", 10, "bold"))
