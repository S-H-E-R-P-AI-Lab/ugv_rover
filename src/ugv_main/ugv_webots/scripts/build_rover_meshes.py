#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10,<3.13"
# dependencies = [
#   "cadquery-ocp==8.0.1.1.0",
#   "numpy>=1.26",
#   "scipy>=1.11",
#   "trimesh==5.1.1",
#   "pymeshlab==2025.7.post1",
#   "moderngl==5.13.0",
#   "pillow>=10",
# ]
# ///
# Copyright 2026 S-H-E-R-P-AI-Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Build the visual meshes of the UGV Rover from Waveshare's STEP model.

Run from the workspace root (needs uv, a GPU or Mesa for the offscreen
visibility pass, and the URDF written by generate_proto.py):

    python3 src/ugv_main/ugv_webots/scripts/generate_proto.py   # writes the URDF it needs
    uv run src/ugv_main/ugv_webots/scripts/build_rover_meshes.py
    python3 src/ugv_main/ugv_webots/scripts/generate_proto.py   # picks up the new meshes

Outputs (see docs/SIMULATOR.md, "Robot model"):
  assets/3d/webots/<link>.dae, tire_*.png, manifest.json   Webots meshes (Collada, one file per
                                                           link, one material group per material)
  assets/3d/glb/ugv_rover.glb                              the whole robot for three.js

Pipeline:
  1. Download the STEP (cached, SHA-256 checked) and tessellate every part with OCP.
  2. Sort the parts into URDF links and materials (RULES); drop screws, nuts and PCB parts.
  3. Place the parts in their link frames. The URDF frames of the deck sensors and the
     pan-tilt are lower than in the CAD of the Jetson Orin kit (docs/MODEL_CHANGES.md), so
     these groups are shifted onto the URDF frames (GROUP_SHIFT) instead of moving frames.
  4. Remove hidden triangles: render the robot from 192 directions with one colour per
     triangle and keep what is seen (plus a one-triangle border to avoid cracks).
  5. Decimate each link to its triangle budget (BUDGET, MeshLab quadric edge collapse),
     split normals at sharp edges.
  6. Wheels are rebuilt: a low-poly tyre of revolution that follows the CAD profile, with
     the tread baked into a normal map and a base colour map, plus the decimated hub.
  7. Export Collada for Webots and a GLB for three.js; print triangle counts.
"""

import argparse
import hashlib
import json
import math
import os
import pickle
import re
import sys
import time
import urllib.request
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
REPO = os.path.abspath(os.path.join(PKG, '..', '..', '..'))
OUT_WEBOTS = os.path.join(REPO, 'assets', '3d', 'webots')
OUT_GLB = os.path.join(REPO, 'assets', '3d', 'glb')
MATERIALS_FILE = os.path.join(REPO, 'assets', '3d', 'materials.json')
URDF_FILE = os.path.join(HERE, '.build', 'ugv_rover.urdf')
CACHE = os.path.join(os.path.expanduser('~'), '.cache', 'ugv_rover_model')

STEP_URL = ('https://files.waveshare.com/wiki/UGV%20Rover%20Jetson%20Orin%20ROS2/'
            'UGV_Rover_PT_Jetson_Orin_ROS2_Kit_STEP.zip')
STEP_ZIP_SHA256 = 'eaa50ebdd9a648da45f1b0b54321167efa6540411a908e44e8c80d40e33b01ff'
STEP_NAME = 'UGV Rover PT Jetson Orin ROS2 Kit v2.step'

# Tessellation of the STEP (mm): fine enough for the visibility pass and the decimation.
LINEAR_DEFLECTION = 0.15
ANGULAR_DEFLECTION = 0.35

# ---------------------------------------------------------------------------
# Part sorting. The CAD frame has x backwards, y to the right, z up, in millimetres, with
# the origin at the top face of the chassis between the wheel axles; CAD_TO_BASE maps it to
# base_link. RULES map the part path (top level names joined with ' / ') to a group and a
# material, optionally with a minimum size in mm (default MIN_PART_EXTENT) that applies to
# the part, or to each connected piece of it when the fifth element is True. The first
# matching rule wins. Group None drops the part, material None picks the material from the
# CAD colour (color_material).
# ---------------------------------------------------------------------------
RULES = [
    (r'螺丝|螺钉|螺母|NUT-M|PA2X6|S-M3x10|KM4\*10|六角铜柱 M2\*4', None, None),  # screws, nuts
    (r'^80mm轮胎', 'wheel', 'rubber'),
    (r'^80mm轮毂', 'wheel', 'hub'),
    (r'^LD19', 'lidar', 'plastic'),
    (r'^DM9095.*/ 602-00001', 'oak', 'lens'),
    (r'^DM9095.*/ 612-00002', None, None),
    (r'^DM9095', 'oak', 'plastic'),
    (r'^PT-PRO.*BEARING-51105', 'pt_base', 'steel', 10.0, True),  # races and cage, no balls
    (r'^PT-PRO.*圆形隔离柱', 'pt_base', 'steel'),
    (r'^PT-PRO.*SCS215 v6:1 / 金属舵盘', 'pt_base', 'steel'),
    (r'^PT-PRO.*SCS215 v6:1', 'pt_base', 'plastic'),
    (r'^PT-PRO.*(PT-BASE-2MM|轴承定位片)', 'pt_base', 'anodized'),
    (r'^PT-PRO.*PT-T-HOLDER', 'pt_pan', 'anodized'),
    (r'^PT-PRO.*金属舵盘', 'pt_tilt', 'steel'),
    (r'^PT-PRO.*SCS215', 'pt_tilt', 'plastic'),
    (r'^PT-PRO.*12V LED.*/ (GLASS|LED):', 'pt_tilt', 'led'),
    (r'^PT-PRO.*12V LED', 'pt_tilt', 'plastic'),
    (r'^PT-PRO.*IMX335', 'pt_tilt', None),
    (r'^PT-PRO', 'pt_tilt', 'anodized'),
    (r'^12V LED.*/ (GLASS|LED):', 'deck', 'led'),
    (r'^12V LED', 'deck', 'plastic'),
    (r'^UGV EXP', 'deck', 'anodized'),
    (r'^Component13(17|21|26|31):', 'deck', 'gold'),  # antenna SMA nuts and washers
    (r'^Component13', 'deck', 'plastic'),             # antennas
    (r'^jetson-orin', 'deck', None, 6.0),
    (r'^六角铜柱 M2.5\*25', 'deck', 'steel'),
    (r'^(BASE-STR|SURFACE|DOWN-SIDE):', 'chassis', 'anodized'),
    (r'^1020铝型材', 'chassis', 'anodized'),
    (r'^1020堵头', 'chassis', 'plastic'),
    (r'^黑色半透明亚克力', 'chassis', 'lens'),
    (r'^(自锁开关|DC5521|扣式塞头)', 'chassis', 'plastic'),
    (r'^(ROS Driver|RPI Audio|WIFI-AT)', 'chassis', None, 6.0),
    (r'^(ROS FAN|散热|UPS|OLED|PCB-HOLDER)', 'chassis', None),
    (r'^JGB37-520.*PLASTIC COVER', 'chassis', 'plastic'),
    (r'^JGB37-520', 'chassis', 'steel'),
    (r'^(COUPLING|F624ZZ|BEAR-HOLDER|尼龙柱|喇叭|Speaker|六角铜柱)', 'chassis', None),
]
# Parts smaller than this (largest extent, mm) are dropped (PCB components, pins), except
# the LED parts which are small but bright.
MIN_PART_EXTENT = 2.5

# Which URDF link each group belongs to.
GROUP_LINK = {
    'chassis': 'base_link',
    'deck': 'base_link',
    'lidar': 'base_lidar_link',
    'oak': 'oak_base',
    'pt_base': 'pt_base_link',
    'pt_pan': 'pt_link1',
    'pt_tilt': 'pt_link2',
}

# Shift of each group in base_link (metres), applied after CAD_TO_BASE. The upstream URDF
# places the deck sensors and the pan-tilt lower than the CAD of the Jetson Orin kit
# (measured in docs/MODEL_CHANGES.md). Frames stay as they are, so the visuals are moved:
#   deck:      deck top 49.6 mm (CAD) onto 40.0 mm (top of the URDF base mesh)
#   lidar:     LD19 rotation axis (39.8, 0.1) mm and base 50 mm onto the URDF frame
#   oak:       OAK-D Lite body onto the URDF box (fit residual 0.9 mm)
#   pt_*:      pan axis x -8.7 mm onto -14.4 mm, tilt axis z 140.6 mm onto 121.3 mm
GROUP_SHIFT = {
    'chassis': (0.0, 0.0, 0.0),
    'deck': (0.0, 0.0, -0.0096),
    'lidar': (-0.0081, -0.0011, -0.0100),
    'oak': (-0.0031, 0.0, -0.0101),
    'pt_base': (-0.0057, 0.0, -0.0193),
    'pt_pan': (-0.0057, 0.0, -0.0193),
    'pt_tilt': (-0.0057, 0.0, -0.0193),
}

# Triangle budget per output mesh (the wheel mesh is drawn six times).
BUDGET = {
    'base_link': 24000,
    'base_lidar_link': 2500,
    'oak_base': 1500,
    'pt_base_link': 3500,
    'pt_link1': 1200,
    'pt_link2': 6000,
    'wheel_hub': 1500,
}
# Share of a link budget per material: proportional to (visible CAD triangles)^0.75 times
# this weight (the shell is what you see first, small shiny parts need little).
MATERIAL_WEIGHT = {'anodized': 2.0, 'steel': 0.7, 'gold': 0.4, 'pcb': 0.5}
TYRE_SEGMENTS = 48         # around the tyre
TYRE_PROFILE_POINTS = 12   # across the tread
TYRE_TEXTURE = (1024, 256)  # tread maps (u around the tyre, v across)
CREASE_ANGLE = math.radians(35)

# The CAD wheel used as the template (left front, hub facing +y) and the URDF joint it
# sits on. The middle wheels (passive, on F624ZZ bearings) are visual only, at these
# centres in base_link (measured from the CAD tyres).
TEMPLATE_TYRE = '80mm轮胎 v2:4'
TEMPLATE_HUB = '80mm轮毂 v3:1'
TEMPLATE_JOINT = 'left_up_wheel_link'
MIDDLE_WHEELS = {'left_mid_wheel': (0.0, 0.0883, -0.0412),
                 'right_mid_wheel': (0.0, -0.0883, -0.0412)}

LINKS = ['base_link', 'base_lidar_link', 'oak_base', 'pt_base_link', 'pt_link1', 'pt_link2']


# ---------------------------------------------------------------------------
# Small math helpers
# ---------------------------------------------------------------------------
def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def transform(xyz, rot):
    m = np.eye(4)
    m[:3, :3] = rot
    m[:3, 3] = xyz
    return m


def apply(m, v):
    return v @ m[:3, :3].T + m[:3, 3]


CAD_TO_BASE = transform((0, 0, 0), rpy_matrix(0, 0, math.pi) * 0.001)


# ---------------------------------------------------------------------------
# URDF (zero joint positions)
# ---------------------------------------------------------------------------
def urdf_frames(path):
    """Pose of every link in base_link, and {child: (joint, type, parent, T, axis)}."""
    root = ET.parse(path).getroot()

    def origin(el):
        o = el.find('origin')
        xyz = [float(v) for v in o.get('xyz', '0 0 0').split()] if o is not None else [0, 0, 0]
        rpy = [float(v) for v in o.get('rpy', '0 0 0').split()] if o is not None else [0, 0, 0]
        return transform(xyz, rpy_matrix(*rpy))

    joints = {}
    for j in root.findall('joint'):
        axis = j.find('axis')
        joints[j.find('child').get('link')] = (
            j.get('name'), j.get('type'), j.find('parent').get('link'), origin(j),
            [float(v) for v in axis.get('xyz').split()] if axis is not None else [1, 0, 0])
    poses = {'base_link': np.eye(4)}

    def pose(link):
        if link not in poses:
            poses[link] = pose(joints[link][2]) @ joints[link][3]
        return poses[link]

    for link in joints:
        if link != 'base_footprint' and joints[link][2] != 'base_footprint':
            try:
                pose(link)
            except KeyError:
                pass
    return poses, joints


def ensure_urdf(path):
    if os.path.exists(path):
        return path
    sys.exit(f'{path} is missing: run generate_proto.py once first (it writes the URDF).')


# ---------------------------------------------------------------------------
# STEP reading and tessellation (cached)
# ---------------------------------------------------------------------------
def fetch_step(step_arg):
    if step_arg:
        return step_arg
    os.makedirs(CACHE, exist_ok=True)
    zpath = os.path.join(CACHE, 'step.zip')
    if not os.path.exists(zpath):
        print(f'Downloading {STEP_URL}')
        urllib.request.urlretrieve(STEP_URL, zpath)
    digest = hashlib.sha256(open(zpath, 'rb').read()).hexdigest()
    if digest != STEP_ZIP_SHA256:
        sys.exit(f'{zpath}: SHA-256 {digest} does not match {STEP_ZIP_SHA256}. '
                 'Waveshare changed the file: check it and update STEP_ZIP_SHA256.')
    step = os.path.join(CACHE, STEP_NAME)
    if not os.path.exists(step):
        with zipfile.ZipFile(zpath) as z:
            z.extract(STEP_NAME, CACHE)
    return step


def read_step(path):
    """Return [{'path': [names], 'color': (r, g, b) or None, 'v': (n, 3) mm, 'f': (m, 3)}]."""
    key = hashlib.sha256(f'{os.path.getsize(path)}:{LINEAR_DEFLECTION}:{ANGULAR_DEFLECTION}'
                         .encode()).hexdigest()[:16]
    cache = os.path.join(CACHE, f'parts_{key}.pkl')
    if os.path.exists(cache):
        with open(cache, 'rb') as f:
            return pickle.load(f)
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.collections import Sequence_TDF_Label
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.Quantity import Quantity_Color
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDF import TDF_Label
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS
    from OCP.XCAFDoc import (XCAFDoc_ColorGen, XCAFDoc_ColorSurf, XCAFDoc_ColorTool,
                             XCAFDoc_DocumentTool)

    print(f'Reading {path} (about 20 s) and tessellating (about 60 s) ...')
    doc = TDocStd_Document(TCollection_ExtendedString('step'))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(True)
    if reader.ReadFile(path) != IFSelect_RetDone:
        sys.exit(f'Cannot read {path}')
    reader.Transfer(doc)
    shapes = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    colors = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())

    def name_of(label):
        attr = TDataStd_Name()
        found = label.FindAttribute(TDataStd_Name.GetID_s(), attr)
        return attr.Get().ToExtString() if found else ''

    def label_color(label):
        c = Quantity_Color()
        for kind in (XCAFDoc_ColorSurf, XCAFDoc_ColorGen):
            if XCAFDoc_ColorTool.GetColor_s(label, kind, c):
                return (c.Red(), c.Green(), c.Blue())
        return None

    def shape_color(shape):
        c = Quantity_Color()
        for kind in (XCAFDoc_ColorSurf, XCAFDoc_ColorGen):
            if colors.GetColor(shape, kind, c):
                return (c.Red(), c.Green(), c.Blue())
        return None

    parts = []

    def leaf(label, loc, names, color):
        shape = shapes.GetShape_s(label)
        BRepMesh_IncrementalMesh(shape, LINEAR_DEFLECTION, False, ANGULAR_DEFLECTION, True)
        groups = {}
        explorer = TopExp_Explorer(shape, TopAbs_FACE)
        while explorer.More():
            face = TopoDS.Face(explorer.Current())
            face_color = shape_color(face) or color
            face_loc = TopLoc_Location()
            tri = BRep_Tool.Triangulation_s(face, face_loc)
            if tri is not None:
                trsf = loc.Multiplied(face_loc).Transformation()
                nodes = (tri.Node(i).Transformed(trsf) for i in range(1, tri.NbNodes() + 1))
                pts = np.array([[p.X(), p.Y(), p.Z()] for p in nodes])
                idx = np.array([tri.Triangle(i).Get()
                                for i in range(1, tri.NbTriangles() + 1)]) - 1
                if face.Orientation() == TopAbs_REVERSED:
                    idx = idx[:, ::-1]
                g = groups.setdefault(face_color, [[], [], 0])
                g[0].append(pts)
                g[1].append(idx + g[2])
                g[2] += len(pts)
            explorer.Next()
        for col, (pts, idx, _) in groups.items():
            parts.append({'path': names, 'color': col, 'v': np.concatenate(pts),
                          'f': np.concatenate(idx)})

    def walk(label, loc, names, color):
        color = label_color(label) or color
        if shapes.IsAssembly_s(label):
            comps = Sequence_TDF_Label()
            shapes.GetComponents_s(label, comps)
            for i in range(1, comps.Length() + 1):
                comp = comps.Value(i)
                ref = TDF_Label()
                shapes.GetReferredShape_s(comp, ref)
                walk(ref, loc.Multiplied(shapes.GetLocation_s(comp)),
                     names + [name_of(comp) or name_of(ref)], label_color(comp) or color)
        else:
            leaf(label, loc, names, color)

    free = Sequence_TDF_Label()
    shapes.GetFreeShapes(free)
    for i in range(1, free.Length() + 1):
        walk(free.Value(i), TopLoc_Location(), [name_of(free.Value(i))], None)
    with open(cache, 'wb') as f:
        pickle.dump(parts, f)
    return parts


# ---------------------------------------------------------------------------
# Sorting
# ---------------------------------------------------------------------------
def color_material(color):
    if color is None:
        return 'steel'
    r, g, b = color
    if r > 0.8 and g > 0.5 and b < 0.4:
        return 'gold'
    if g > r + 0.03 and g > b:
        return 'pcb'
    if b > r + 0.1:
        return 'lens'
    if max(color) > 0.5:
        return 'steel'
    return 'plastic'


def sort_parts(parts):
    rules = [(re.compile(r[0]), r[1], r[2], r[3] if len(r) > 3 else MIN_PART_EXTENT,
              len(r) > 4 and r[4]) for r in RULES]
    sorted_parts = []
    unmatched = set()
    for part in parts:
        path = ' / '.join(part['path'][1:])
        for regex, group, material, min_extent, pieces in rules:
            if regex.search(path):
                break
        else:
            unmatched.add(part['path'][1])
            continue
        if group is None:
            continue
        if pieces:
            m = trimesh.Trimesh(part['v'], part['f'])
            keep = [c for c in m.split(only_watertight=False) if c.extents.max() >= min_extent]
            m = trimesh.util.concatenate(keep)
            part = dict(part, v=m.vertices, f=m.faces)
        extent = (part['v'].max(0) - part['v'].min(0)).max()
        if extent < min_extent and material != 'led':
            continue
        material = material or color_material(part['color'])
        sorted_parts.append(dict(part, group=group, material=material, name=path))
    if unmatched:
        sys.exit(f'Parts without a rule (add them to RULES): {sorted(unmatched)}')
    return sorted_parts


# ---------------------------------------------------------------------------
# Visibility (GPU, one colour per triangle)
# ---------------------------------------------------------------------------
def fibonacci_sphere(n):
    i = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * i / n)
    theta = math.pi * (1 + 5 ** 0.5) * i
    return np.stack([np.cos(theta) * np.sin(phi), np.sin(theta) * np.sin(phi), np.cos(phi)], 1)


def look_at(eye, target):
    """World to camera matrix (camera looks along -z)."""
    f = target - eye
    f /= np.linalg.norm(f)
    up = np.array([0, 0, 1.0]) if abs(f[2]) < 0.999 else np.array([1.0, 0, 0])
    s = np.cross(f, up)
    s /= np.linalg.norm(s)
    u = np.cross(s, f)
    m = np.eye(4)
    m[0, :3], m[1, :3], m[2, :3] = s, u, -f
    m[:3, 3] = -m[:3, :3] @ eye
    return m


def visible_triangles(meshes, views=192, resolution=2048):
    """
    Return one boolean array of seen faces for each of meshes, [(V, F)] in one frame.

    Renders every triangle with its index as an integer colour (OpenGL through EGL, no
    back-face culling) from directions spread evenly on a sphere, orthographic.
    """
    import moderngl
    tris = np.concatenate([v[f] for v, f in meshes]).reshape(-1, 3)
    n = len(tris) // 3
    lo, hi = tris.min(0), tris.max(0)
    centre, radius = (lo + hi) / 2, np.linalg.norm(hi - lo) / 2
    ctx = moderngl.create_context(standalone=True, backend='egl')
    prog = ctx.program(
        vertex_shader='#version 330\nin vec3 pos; in uint tid; uniform mat4 mvp;\n'
                      'flat out uint vid;\n'
                      'void main() { gl_Position = mvp * vec4(pos, 1.0); vid = tid; }',
        fragment_shader='#version 330\nflat in uint vid; out uint id;\nvoid main() { id = vid; }')
    vbo = ctx.buffer(((tris - centre) / radius).astype('f4').tobytes())
    ibo = ctx.buffer(np.repeat(np.arange(1, n + 1, dtype='u4'), 3).tobytes())
    vao = ctx.vertex_array(prog, [(vbo, '3f', 'pos'), (ibo, '1u', 'tid')])
    fbo = ctx.framebuffer([ctx.renderbuffer((resolution, resolution), components=1, dtype='u4')],
                          ctx.depth_renderbuffer((resolution, resolution)))
    fbo.use()
    ctx.enable(moderngl.DEPTH_TEST)
    proj = np.diag([1 / 1.02, 1 / 1.02, -0.5, 1.0])  # orthographic, depth 1 to 5 radii
    proj[2, 3] = -1.5
    seen = np.zeros(n + 1, bool)
    for d in fibonacci_sphere(views):
        mvp = (proj @ look_at(d * 2.5, np.zeros(3))).astype('f4')
        prog['mvp'].write(mvp.T.copy().tobytes())
        fbo.clear(0, 0, 0, 0, depth=1.0)
        vao.render()
        seen[np.unique(np.frombuffer(fbo.read(components=1, dtype='u4'), 'u4'))] = True
    ctx.release()
    seen = seen[1:]
    out, start = [], 0
    for _, f in meshes:
        out.append(seen[start:start + len(f)])
        start += len(f)
    return out


def grow(faces, mask):
    """Add the faces that share a vertex with a seen face (avoids cracks at the border)."""
    used = np.zeros(faces.max() + 1, bool)
    used[faces[mask].ravel()] = True
    return mask | used[faces].any(1)


# ---------------------------------------------------------------------------
# Mesh processing
# ---------------------------------------------------------------------------
def weld(v, f, digits=6):
    m = trimesh.Trimesh(v, f, process=False)
    m.merge_vertices(digits_vertex=digits)
    m.update_faces(m.nondegenerate_faces())
    m.remove_unreferenced_vertices()
    return m.vertices, m.faces


def decimate(v, f, target):
    """Quadric edge collapse (MeshLab) down to target triangles, keeping flat areas flat."""
    import pymeshlab
    if len(f) <= target:
        return v, f
    ms = pymeshlab.MeshSet()
    ms.add_mesh(pymeshlab.Mesh(v.astype(np.float64), f.astype(np.int32)))
    # optimalplacement moves vertices off the surface on open borders (spikes of several cm)
    ms.meshing_decimation_quadric_edge_collapse(targetfacenum=int(target), preservenormal=True,
                                                planarquadric=True, qualitythr=0.3, autoclean=True,
                                                optimalplacement=False)
    m = ms.current_mesh()
    v2, f2 = m.vertex_matrix(), m.face_matrix()
    if (v2.min(0) < v.min(0) - 1e-4).any() or (v2.max(0) > v.max(0) + 1e-4).any():
        sys.exit('Decimation moved vertices outside the input bounds')
    return v2, f2


def split_normals(v, f, uv=None):
    """Duplicate vertices at edges sharper than CREASE_ANGLE; return v, n, f, uv."""
    m = trimesh.Trimesh(v, f, process=False)
    fn = m.face_normals
    corners = f.reshape(-1)
    face_of = np.repeat(np.arange(len(f)), 3)
    # vertex normal per corner: average of adjacent face normals within the crease angle
    adj = [[] for _ in range(len(v))]
    for c, fi in zip(corners, face_of):
        adj[c].append(fi)
    cos_crease = math.cos(CREASE_ANGLE)
    area = m.area_faces
    out_v, out_n, out_uv, key_index = [], [], [], {}
    new_f = np.empty(len(corners), np.int64)
    for k, (c, fi) in enumerate(zip(corners, face_of)):
        near = [g for g in adj[c] if fn[g] @ fn[fi] >= cos_crease]
        normal = (fn[near] * area[near, None]).sum(0)
        normal /= np.linalg.norm(normal) + 1e-12
        key = (c, tuple(np.round(normal, 3)))
        if key not in key_index:
            key_index[key] = len(out_v)
            out_v.append(v[c])
            out_n.append(normal)
            if uv is not None:
                out_uv.append(uv[c])
        new_f[k] = key_index[key]
    return (np.array(out_v), np.array(out_n), new_f.reshape(-1, 3),
            np.array(out_uv) if uv is not None else None)


# ---------------------------------------------------------------------------
# Wheel
# ---------------------------------------------------------------------------
def rasterize_max(px, py, pz, faces, width, height):
    """Z-buffer (keep max z) of triangles given in pixel coordinates."""
    buf = np.full((height, width), -np.inf)
    for a, b, c in faces:
        xs, ys, zs = px[[a, b, c]], py[[a, b, c]], pz[[a, b, c]]
        x0, x1 = int(max(math.floor(xs.min()), 0)), int(min(math.ceil(xs.max()), width - 1))
        y0, y1 = int(max(math.floor(ys.min()), 0)), int(min(math.ceil(ys.max()), height - 1))
        if x1 < x0 or y1 < y0:
            continue
        det = (ys[1] - ys[2]) * (xs[0] - xs[2]) + (xs[2] - xs[1]) * (ys[0] - ys[2])
        if abs(det) < 1e-12:
            continue
        gx, gy = np.meshgrid(np.arange(x0, x1 + 1), np.arange(y0, y1 + 1))
        l0 = ((ys[1] - ys[2]) * (gx - xs[2]) + (xs[2] - xs[1]) * (gy - ys[2])) / det
        l1 = ((ys[2] - ys[0]) * (gx - xs[2]) + (xs[0] - xs[2]) * (gy - ys[2])) / det
        l2 = 1 - l0 - l1
        inside = (l0 >= -1e-6) & (l1 >= -1e-6) & (l2 >= -1e-6)
        z = l0 * zs[0] + l1 * zs[1] + l2 * zs[2]
        sub = buf[y0:y1 + 1, x0:x1 + 1]
        np.maximum(sub, np.where(inside, z, -np.inf), out=sub)
    return buf


def build_tyre(v, f):
    """
    Build a low-poly tyre of revolution around y, plus tread normal and colour maps.

    v, f: CAD tyre in the wheel frame (metres). Returns (V, F, UV), normal map, colour map.
    """
    from scipy.ndimage import gaussian_filter, maximum_filter1d
    r = np.hypot(v[:, 0], v[:, 2])
    y = v[:, 1]
    y0, y1 = y.min(), y.max()
    bins = np.linspace(y0, y1, 61)
    which = np.clip(np.digitize(y, bins) - 1, 0, 59)
    r_env = np.array([r[which == i].max() for i in range(60)])
    r_in = r.min()
    # profile across the tread: smooth envelope, simplified to about a dozen points
    ys = (bins[:-1] + bins[1:]) / 2
    keep = np.unique(np.round(np.linspace(0, 59, TYRE_PROFILE_POINTS)).astype(int)).tolist()
    prof_y = np.concatenate([[y1], ys[keep][::-1], [y0]])
    prof_r = np.concatenate([[r_env[59]], r_env[keep][::-1], [r_env[0]]])
    # The sidewalls curl in to the bead (radius r_in), which sits over the hub flange: start
    # each sidewall at the bead so no gap shows between tyre and hub.
    bead = r < r_in + 0.0005
    inset = min(y1 - y[bead].max(), y[bead].min() - y0)
    ring_y = np.concatenate([[y1 - inset], prof_y, [y0 + inset]])
    ring_r = np.concatenate([[r_in], prof_r, [r_in]])
    width = y1 - y0
    ring_v = np.concatenate([[0.0], (y1 - prof_y) / width, [1.0]])
    n = TYRE_SEGMENTS
    theta = np.linspace(0, 2 * math.pi, n + 1)
    V, UV = [], []
    for ry, rr, rv in zip(ring_y, ring_r, ring_v):
        for k, t in enumerate(theta):
            V.append((rr * math.cos(t), ry, rr * math.sin(t)))
            UV.append((k / n, rv))
    V, UV = np.array(V), np.array(UV)
    F = []
    cols = n + 1
    for i in range(len(ring_y) - 1):
        for k in range(n):
            a, b = i * cols + k, i * cols + k + 1
            c, d = (i + 1) * cols + k, (i + 1) * cols + k + 1
            F += [(a, b, d), (a, d, c)]
    F = np.array(F)
    # orient every face outwards: sidewalls away from the mid plane, the rest away from the axis
    m = trimesh.Trimesh(V, F, process=False)
    centre = m.triangles_center
    out = centre.copy()
    out[:, 1] = 0
    side = np.abs(m.face_normals[:, 1]) > 0.7
    out[side] = 0
    out[side, 1] = np.sign(centre[side, 1] - (y0 + y1) / 2)
    flip = (m.face_normals * out).sum(1) < 0
    F[flip] = F[flip][:, ::-1]

    # Tread height map in (u = angle, v = across) of the CAD tyre, relative to the envelope.
    tw, th = TYRE_TEXTURE
    ang = np.arctan2(v[:, 2], v[:, 0]) % (2 * math.pi)
    px = ang / (2 * math.pi) * tw
    py = (y1 - y) / width * th
    tread = r > r_in + 0.008  # skip the bead and inner sidewall
    keepf = tread[f].all(1)
    ff = f[keepf]
    # triangles across the 0/2pi seam: shift their small-angle corners up by tw
    wrap = (px[ff].max(1) - px[ff].min(1)) > tw / 2
    pxx = np.concatenate([px, px + tw])
    ffw = ff.copy()
    lowc = px[ff[wrap]] < tw / 2
    ffw_wrap = np.where(lowc, ff[wrap] + len(px), ff[wrap])
    ffw[wrap] = ffw_wrap
    height = rasterize_max(pxx, np.concatenate([py, py]), np.concatenate([r, r]), ffw, 2 * tw, th)
    height = np.maximum(height[:, :tw], height[:, tw:])
    env_rows = np.interp((np.arange(th) + 0.5) / th * width, (y1 - ys)[::-1], r_env[::-1])
    hole = ~np.isfinite(height)
    height[hole] = r_in
    h = np.clip(height - env_rows[:, None], -0.004, 0.0)
    h = np.maximum(h, -maximum_filter1d(-h, 3, axis=1))  # close one pixel gaps
    h = gaussian_filter(h, 0.7, mode=('nearest', 'wrap'))
    du = 2 * math.pi * env_rows[:, None] / tw  # metres per pixel around
    dv = width / th
    gu = (np.roll(h, -1, 1) - np.roll(h, 1, 1)) / (2 * du)
    gv = np.gradient(h, axis=0) / dv
    # tangent space: +x along u (around), +y along v; image rows grow with v, OpenGL
    # normal maps expect +y towards decreasing rows, hence the sign of gv.
    nrm = np.stack([-gu, gv, np.ones_like(h)], -1)
    nrm /= np.linalg.norm(nrm, axis=-1, keepdims=True)
    edge = max(2, th // 40)
    nrm[:edge] = nrm[-edge:] = (0, 0, 1)
    normal_img = ((nrm * 0.5 + 0.5) * 255).round().astype(np.uint8)
    ao = np.clip(1.0 + h / 0.004 * 0.55, 0.4, 1.0)
    ao[:edge] = ao[-edge:] = 1.0
    color_img = (ao * 255).round().astype(np.uint8)
    return (V, F, UV), normal_img, color_img


def build_wheel(sorted_parts, poses):
    tyre = next(p for p in sorted_parts if p['name'] == TEMPLATE_TYRE)
    hub = next(p for p in sorted_parts if p['name'] == TEMPLATE_HUB)
    to_link = np.linalg.inv(poses[TEMPLATE_JOINT]) @ CAD_TO_BASE
    tv, tf_ = apply(to_link, tyre['v']), tyre['f']
    hv, hf = apply(to_link, hub['v']), hub['f']
    centre = np.array([0.0, (tv[:, 1].min() + tv[:, 1].max()) / 2, 0.0])
    print(f'  wheel: CAD tyre centre in the {TEMPLATE_JOINT} frame: '
          f'{np.round((tv.max(0) + tv.min(0)) / 2 * 1000, 2)} mm')
    tyre_mesh, normal_img, color_img = build_tyre(tv - centre, tf_)
    # hub: drop what the tyre and the chassis hide, then decimate
    seen = visible_triangles([(hv - centre, hf), (tv - centre, tf_)], views=128,
                             resolution=1024)[0]
    hv2, hf2 = weld(hv - centre, hf[grow(hf, seen)])
    hv2, hf2 = decimate(hv2, hf2, BUDGET['wheel_hub'])
    V, F, UV = tyre_mesh
    return {'tyre': (V + centre, F, UV), 'hub': (hv2 + centre, hf2)}, normal_img, color_img


# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------
def fmt(a, digits):
    return ' '.join(f'{x:.{digits}f}'.rstrip('0').rstrip('.') if x != 0 else '0'
                    for x in a.ravel())


def collada_source(sid, data, digits, params):
    """Return a Collada <source> of float tuples (positions, normals or texture coordinates)."""
    names = ''.join(f'<param name="{p}" type="float"/>' for p in params)
    return (f'<source id="{sid}"><float_array id="{sid}-a" count="{data.size}">'
            f'{fmt(data, digits)}</float_array><technique_common>'
            f'<accessor source="#{sid}-a" count="{len(data)}" stride="{len(params)}">'
            f'{names}</accessor></technique_common></source>')


def write_collada(path, groups, materials):
    """
    Write groups [(material name, V, N, F, UV or None)], in material index order.

    Webots' Mesh node picks a group with materialIndex: assimp numbers the materials in
    the order of their ids, hence the two digit prefix.
    """
    fx, mats, geos, nodes = [], [], [], []
    for i, (mat, v, n, f, uv) in enumerate(groups):
        c = materials[mat]['baseColor']
        mid = f'{i:02d}_{mat}'
        fx.append(f'<effect id="{mid}-fx"><profile_COMMON><technique sid="t"><lambert>'
                  f'<diffuse><color>{c[0]} {c[1]} {c[2]} 1</color></diffuse>'
                  '</lambert></technique></profile_COMMON></effect>')
        mats.append(f'<material id="{mid}" name="{mid}">'
                    f'<instance_effect url="#{mid}-fx"/></material>')
        inputs = (f'<input semantic="VERTEX" source="#{mid}-v" offset="0"/>'
                  f'<input semantic="NORMAL" source="#{mid}-n" offset="0"/>')
        sources = (collada_source(f'{mid}-p', v, 5, 'XYZ') +
                   collada_source(f'{mid}-n', n, 3, 'XYZ'))
        if uv is not None:
            inputs += f'<input semantic="TEXCOORD" source="#{mid}-t" offset="0" set="0"/>'
            sources += collada_source(f'{mid}-t', uv, 4, 'ST')
        geos.append(f'<geometry id="{mid}-g" name="{mid}"><mesh>{sources}'
                    f'<vertices id="{mid}-v">'
                    f'<input semantic="POSITION" source="#{mid}-p"/></vertices>'
                    f'<triangles material="{mid}-s" count="{len(f)}">{inputs}'
                    f'<p>{" ".join(map(str, f.ravel()))}</p></triangles></mesh></geometry>')
        nodes.append(f'<node id="{mid}-node" name="{mid}"><instance_geometry url="#{mid}-g">'
                     '<bind_material><technique_common>'
                     f'<instance_material symbol="{mid}-s" target="#{mid}"/>'
                     '</technique_common></bind_material></instance_geometry></node>')
    doc = ('<?xml version="1.0" encoding="utf-8"?>\n'
           '<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">'
           '<asset><contributor><authoring_tool>ugv_webots build_rover_meshes.py'
           '</authoring_tool></contributor><unit name="meter" meter="1"/>'
           '<up_axis>Z_UP</up_axis></asset>'
           f'<library_effects>{"".join(fx)}</library_effects>'
           f'<library_materials>{"".join(mats)}</library_materials>'
           f'<library_geometries>{"".join(geos)}</library_geometries>'
           f'<library_visual_scenes><visual_scene id="scene">{"".join(nodes)}</visual_scene>'
           '</library_visual_scenes>'
           '<scene><instance_visual_scene url="#scene"/></scene></COLLADA>\n')
    with open(path, 'w') as fh:
        fh.write(doc)


def pbr(materials, name, textures=None):
    from PIL import Image
    m = materials[name]
    kw = dict(name=name, baseColorFactor=list(m['baseColor']) + [1.0],
              metallicFactor=m['metalness'], roughnessFactor=m['roughness'])
    if m.get('emissive'):
        kw['emissiveFactor'] = m['emissive']
    if textures:
        if 'normal' in textures:
            kw['normalTexture'] = Image.open(textures['normal'])
        if 'color' in textures:
            kw['baseColorTexture'] = Image.open(textures['color']).convert('RGB')
    return trimesh.visual.material.PBRMaterial(**kw)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--step', help='STEP file (default: download and cache Waveshare\'s)')
    parser.add_argument('--urdf', default=URDF_FILE, help='URDF from generate_proto.py')
    parser.add_argument('--report', action='store_true',
                        help='list the visible CAD triangles per part (to tune RULES)')
    args = parser.parse_args()
    t_start = time.time()
    materials = json.load(open(MATERIALS_FILE))['materials']
    poses, joints = urdf_frames(ensure_urdf(args.urdf))
    parts = sort_parts(read_step(fetch_step(args.step)))
    print(f'{len(parts)} parts kept after the rules')

    # ---- place every part in its link frame ----
    placed = {}  # link -> [(material, V, F, name)]
    for p in parts:
        if p['group'] == 'wheel':
            continue
        link = GROUP_LINK[p['group']]
        shift = transform(GROUP_SHIFT[p['group']], np.eye(3))
        m = np.linalg.inv(poses[link]) @ shift @ CAD_TO_BASE
        v, f = weld(apply(m, p['v']), p['f'])
        placed.setdefault(link, []).append((p['material'], v, f, p['name']))

    # ---- visibility: fixed links together (with wheels as occluders), moving links alone ----
    wheels = [p for p in parts if p['group'] == 'wheel']
    wheel_occluders = [(apply(CAD_TO_BASE, p['v']), p['f']) for p in wheels]
    fixed = ['base_link', 'base_lidar_link', 'oak_base', 'pt_base_link']
    passes = [fixed, ['pt_link1'], ['pt_link2']]
    kept = {}
    for links in passes:
        entries = [(link, i) for link in links for i in range(len(placed[link]))]
        meshes = [(apply(poses[link], placed[link][i][1]), placed[link][i][2])
                  for link, i in entries]
        occ = wheel_occluders if 'base_link' in links else []
        seen = visible_triangles(meshes + occ)
        for (link, i), s in zip(entries, seen):
            mat, v, f, name = placed[link][i]
            s = grow(f, s)
            if s.any():
                kept.setdefault(link, []).append((mat, v, f[s], name))
        before = sum(len(placed[link][i][2]) for link, i in entries)
        after = sum(len(e[2]) for link in links for e in kept.get(link, []))
        print(f'  visibility {links}: {before} -> {after} triangles')
    if args.report:
        for link in LINKS:
            rows = {}
            for mat, _, f, name in kept[link]:
                key = (name.split(' / ')[0], mat)
                rows[key] = rows.get(key, 0) + len(f)
            print(f'  {link}:')
            for (name, mat), n in sorted(rows.items(), key=lambda r: -r[1])[:25]:
                print(f'    {n:7d}  {mat:9s} {name}')

    # ---- merge per material, decimate to the budget, split normals ----
    os.makedirs(OUT_WEBOTS, exist_ok=True)
    os.makedirs(OUT_GLB, exist_ok=True)
    manifest = {'source': STEP_URL, 'step_zip_sha256': STEP_ZIP_SHA256,
                'group_shift_m': GROUP_SHIFT, 'links': {}}
    glb_meshes = {}
    report = []
    for link in LINKS:
        by_mat = {}
        for mat, v, f, _ in kept[link]:
            by_mat.setdefault(mat, []).append((v, f))
        counts = {mat: sum(len(f) for _, f in lst) for mat, lst in by_mat.items()}
        total = sum(counts.values())
        weights = {mat: c ** 0.75 * MATERIAL_WEIGHT.get(mat, 1.0) for mat, c in counts.items()}
        wsum = sum(weights.values())
        groups = []
        for mat in sorted(by_mat, key=lambda m: list(materials).index(m)):
            v, f = weld(*trimesh.util.append_faces([x[0] for x in by_mat[mat]],
                                                   [x[1] for x in by_mat[mat]]))
            if os.environ.get('UGV_MESH_DUMP'):
                np.savez(os.path.join(os.environ['UGV_MESH_DUMP'], f'{link}_{mat}.npz'), v=v, f=f)
            target = max(60, int(BUDGET[link] * weights[mat] / wsum))
            v, f = decimate(v, f, min(target, counts[mat]))
            if args.report:
                print(f'    {link} {mat}: {counts[mat]} -> target {target} -> {len(f)}')
            vv, nn, ff, _ = split_normals(v, f)
            groups.append((mat, vv, nn, ff, None))
        write_collada(os.path.join(OUT_WEBOTS, f'{link}.dae'), groups, materials)
        tri = {g[0]: len(g[3]) for g in groups}
        manifest['links'][link] = {'file': f'{link}.dae', 'materials': [g[0] for g in groups],
                                   'triangles': tri, 'cad_triangles_after_culling': total}
        glb_meshes[link] = groups
        report.append((link, sum(tri.values()), tri))

    # ---- wheel ----
    from PIL import Image
    wheel, normal_img, color_img = build_wheel(parts, poses)
    Image.fromarray(normal_img).save(os.path.join(OUT_WEBOTS, 'tire_normal.png'), optimize=True)
    Image.fromarray(color_img).save(os.path.join(OUT_WEBOTS, 'tire_color.png'), optimize=True)
    tv, tf_, tuv = wheel['tyre']
    v1, n1, f1, uv1 = split_normals(tv, tf_, tuv)
    hv, hf = wheel['hub']
    v2, n2, f2, _ = split_normals(hv, hf)
    wheel_groups = [('rubber', v1, n1, f1, uv1), ('hub', v2, n2, f2, None)]
    write_collada(os.path.join(OUT_WEBOTS, 'wheel.dae'), wheel_groups, materials)
    manifest['links']['wheel'] = {'file': 'wheel.dae', 'materials': ['rubber', 'hub'],
                                  'triangles': {'rubber': len(f1), 'hub': len(f2)},
                                  'textures': {'rubber': {'normal': 'tire_normal.png',
                                                          'color': 'tire_color.png'}},
                                  'mirror_for': ['right_up_wheel_link', 'right_down_wheel_link',
                                                 'right_mid_wheel'],
                                  'middle_wheels': MIDDLE_WHEELS}
    report.append(('wheel (x6)', len(f1) + len(f2), {'rubber': len(f1), 'hub': len(f2)}))

    # ---- GLB: URDF hierarchy, joints at zero, root turned to glTF's y up ----
    scene = trimesh.Scene()
    root_tf = transform((0, 0, 0), rpy_matrix(-math.pi / 2, 0, 0))
    scene.graph.update(frame_to='ugv_rover', frame_from=scene.graph.base_frame, matrix=root_tf)
    tex = {'rubber': {'normal': os.path.join(OUT_WEBOTS, 'tire_normal.png'),
                      'color': os.path.join(OUT_WEBOTS, 'tire_color.png')}}

    def add_link(name, parent, matrix, groups):
        scene.graph.update(frame_to=name, frame_from=parent, matrix=matrix)
        for mat, v, n, f, uv in groups:
            material = pbr(materials, mat, tex.get(mat) if uv is not None else None)
            visual = trimesh.visual.TextureVisuals(uv=uv, material=material)
            mesh = trimesh.Trimesh(v, f, vertex_normals=n, visual=visual, process=False)
            scene.add_geometry(mesh, node_name=f'{name}_{mat}', geom_name=f'{name}_{mat}',
                               parent_node_name=name)

    add_link('base_footprint', 'ugv_rover', np.eye(4), [])
    add_link('base_link', 'base_footprint', joints['base_link'][3], glb_meshes['base_link'])
    for link in LINKS[1:]:
        name, jtype, parent, jm, axis = joints[link]
        add_link(link, parent, jm, glb_meshes[link])
    flip = transform((0, 0, 0), rpy_matrix(0, 0, math.pi))
    for link in ('left_up_wheel_link', 'left_down_wheel_link', 'right_up_wheel_link',
                 'right_down_wheel_link'):
        add_link(link, 'base_link', joints[link][3], [])
        add_link(link + '_visual', link, flip if link.startswith('right') else np.eye(4),
                 wheel_groups)
    for name, xyz in MIDDLE_WHEELS.items():
        add_link(name, 'base_link', transform(xyz, flip[:3, :3] if name.startswith('right')
                                              else np.eye(3)), wheel_groups)
    glb_path = os.path.join(OUT_GLB, 'ugv_rover.glb')
    scene.export(glb_path)
    manifest['glb'] = os.path.relpath(glb_path, REPO)
    manifest['joints'] = {link: {'joint': j[0], 'type': j[1], 'parent': j[2],
                                 'axis': j[4], 'origin_xyz': [round(x, 6) for x in j[3][:3, 3]]}
                          for link, j in joints.items()}
    with open(os.path.join(OUT_WEBOTS, 'manifest.json'), 'w') as fh:
        json.dump(manifest, fh, indent=1, ensure_ascii=False)

    print('\nTriangles per mesh:')
    total = 0
    for name, n, per in report:
        drawn = n * 6 if name.startswith('wheel') else n
        total += drawn
        print(f'  {name:18s} {n:6d}  {per}')
    print(f'  {"robot (drawn)":18s} {total:6d}')
    for fn in sorted(os.listdir(OUT_WEBOTS)) + ['../glb/ugv_rover.glb']:
        p = os.path.join(OUT_WEBOTS, fn)
        print(f'  {fn:24s} {os.path.getsize(p) / 1024:8.1f} KiB')
    print(f'Done in {time.time() - t_start:.0f} s')


if __name__ == '__main__':
    main()
