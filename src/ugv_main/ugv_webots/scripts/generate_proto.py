#!/usr/bin/env python3
"""
Regenerate protos/UgvRover.proto from the upstream URDF.

Run from the workspace root after building, with ROS sourced:

    source install/setup.bash
    python3 src/ugv_main/ugv_webots/scripts/generate_proto.py

Pipeline:
  1. xacro: ugv_description/urdf/bases/ugv_rover.xacro without the Gazebo plugin
     include (ugv_webots/description.py, the same function the launch file uses).
  2. urdf2webots (bundled with webots_ros2_importer): URDF to a raw PROTO.
  3. Hand tuning, applied here so it is reproducible (see docs/SIMULATOR.md):
     a. Links joined by fixed joints become one rigid body. Their masses,
        centres of mass and inertias are combined (parallel axis theorem) and put
        on the body root. urdf2webots leaves the Robot root without Physics and
        glues the other links with implicit fixed joints, which makes the robot
        kinematic and the contacts soft.
     b. Every STL mesh is inlined as an IndexedFaceSet so the PROTO is a single
        self-contained file (needed for the macOS shared-folder mode).
     c. Collision geometry: the chassis mesh reaches the ground (it includes the
        six-wheel side frames), so it is replaced by two boxes that clear the
        wheels. Sensor and pan-tilt meshes are replaced by their bounding boxes.
     d. Wheels get contactMaterial "ugv_wheel" and realistic motor limits.
     e. Sensors are added: LD19 lidar, pan-tilt camera, OAK RGB camera and OAK
        depth (RangeFinder). Specs and frames follow ugv_gazebo and the real LD19.
  4. Inertia sanity checks on the URDF values (printed, and the script fails on
     non physical tensors).
"""

import math
import os
import re
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, PKG)  # use ugv_webots/description.py from the source tree

from ugv_webots.description import upstream_urdf  # noqa: E402

BUILD = os.path.join(HERE, '.build')
OUTPUT = os.path.join(PKG, 'protos', 'UgvRover.proto')
URDF2PROTO = '/opt/ros/humble/lib/webots_ros2_importer/urdf2proto'

# ---------------------------------------------------------------------------
# Tuning values. Documented in docs/SIMULATOR.md, keep both in sync.
# ---------------------------------------------------------------------------
WHEEL_MAX_VELOCITY = 40.0   # rad/s at the wheel (1.6 m/s), above the 1.5 m/s controller limit
WHEEL_MAX_TORQUE = 1.2      # N*m per wheel
WHEEL_MATERIAL = 'ugv_wheel'

# Chassis collision boxes in the base_link frame: (centre xyz, size xyz).
# Lower box sits between the wheels (inner wheel faces at |y| = 0.066), upper box is the
# top plate above the wheels (wheel tops are at z = -0.001 in base_link).
BASE_COLLISION_BOXES = [
    ((0.0, 0.0, -0.0125), (0.240, 0.128, 0.045)),
    ((0.0, 0.0, 0.0200), (0.245, 0.230, 0.040)),
]

LIDAR = '''Lidar {
  translation 0 0 0.02
  name "ld19_lidar"
  fieldOfView 6.283185
  horizontalResolution 480
  numberOfLayers 1
  near 0.01
  minRange 0.2
  maxRange 12
  noise 0.000833
  resolution 0.01
  defaultFrequency 10
  type "fixed"
}'''

PT_CAMERA = '''Camera {
  name "pt_camera"
  fieldOfView 1.02974
  width 640
  height 480
  near 0.02
  far 100
  noise 0.007
}'''

OAK_RGB = '''Camera {
  name "oak_rgb"
  fieldOfView 1.02974
  width 640
  height 480
  near 0.02
  far 100
  noise 0.007
}'''

OAK_DEPTH = '''RangeFinder {
  name "oak_depth"
  fieldOfView 1.02974
  width 640
  height 480
  near 0.02
  minRange 0.2
  maxRange 8
}'''

DEVICES = {
    'base_lidar_link': [LIDAR],
    'pt_camera_link': [PT_CAMERA],
    'oak_base': [OAK_RGB, OAK_DEPTH],
}

# DEF names must not start with a digit in VRML.
DEF_RENAMES = {'3d_camera_link': 'oak_body'}


# ---------------------------------------------------------------------------
# URDF helpers
# ---------------------------------------------------------------------------
def rpy_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matvec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def transpose(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def parse_origin(element):
    origin = element.find('origin') if element is not None else None
    xyz = [0.0, 0.0, 0.0]
    rpy = [0.0, 0.0, 0.0]
    if origin is not None:
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
    return xyz, rpy_matrix(*rpy)


def load_urdf(text):
    root = ET.fromstring(text)
    links = {}
    for link in root.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            links[link.get('name')] = None
            continue
        xyz, rot = parse_origin(inertial)
        i = inertial.find('inertia')
        tensor = [[float(i.get('ixx')), float(i.get('ixy')), float(i.get('ixz'))],
                  [float(i.get('ixy')), float(i.get('iyy')), float(i.get('iyz'))],
                  [float(i.get('ixz')), float(i.get('iyz')), float(i.get('izz'))]]
        # Inertia about the COM expressed in the link frame.
        tensor = matmul(matmul(rot, tensor), transpose(rot))
        links[link.get('name')] = (float(inertial.find('mass').get('value')), xyz, tensor)
    joints = []
    for joint in root.findall('joint'):
        xyz, rot = parse_origin(joint)
        joints.append((joint.get('name'), joint.get('type'), joint.find('parent').get('link'),
                       joint.find('child').get('link'), xyz, rot))
    return links, joints


def rigid_bodies(links, joints):
    """Map each body root link to [(link, xyz, rot) in the body root frame]."""
    children = {}
    has_parent = set()
    for name, jtype, parent, child, xyz, rot in joints:
        children.setdefault(parent, []).append((jtype, child, xyz, rot))
        has_parent.add(child)
    bodies = {}

    def walk(link, body, xyz, rot):
        bodies[body].append((link, xyz, rot))
        for jtype, child, cxyz, crot in children.get(link, []):
            if jtype == 'fixed':
                walk(child, body, [a + b for a, b in zip(xyz, matvec(rot, cxyz))],
                     matmul(rot, crot))
            else:
                bodies[child] = []
                walk(child, child, [0.0, 0.0, 0.0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    for link in links:
        if link not in has_parent:
            bodies[link] = []
            walk(link, link, [0.0, 0.0, 0.0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    return bodies


def combine(links, members):
    """Combine (mass, com, inertia about com) of the links of one rigid body."""
    parts = []
    for link, xyz, rot in members:
        if links.get(link) is None:
            continue
        mass, com, tensor = links[link]
        parts.append((mass, [a + b for a, b in zip(xyz, matvec(rot, com))],
                      matmul(matmul(rot, tensor), transpose(rot))))
    total = sum(p[0] for p in parts)
    com = [sum(p[0] * p[1][k] for p in parts) / total for k in range(3)]
    inertia = [[0.0] * 3 for _ in range(3)]
    for mass, c, tensor in parts:
        d = [c[k] - com[k] for k in range(3)]
        d2 = sum(x * x for x in d)
        for i in range(3):
            for j in range(3):
                inertia[i][j] += tensor[i][j] + mass * ((d2 if i == j else 0.0) - d[i] * d[j])
    return total, com, inertia


def check_inertia(name, mass, tensor):
    ixx, iyy, izz = tensor[0][0], tensor[1][1], tensor[2][2]
    problems = []
    if mass <= 0:
        problems.append('mass <= 0')
    if min(ixx, iyy, izz) <= 0:
        problems.append('non positive principal moment')
    if ixx + iyy < izz or ixx + izz < iyy or iyy + izz < ixx:
        problems.append('violates the triangle inequality')
    return problems


# ---------------------------------------------------------------------------
# STL to IndexedFaceSet
# ---------------------------------------------------------------------------
def read_stl(path):
    with open(path, 'rb') as f:
        data = f.read()
    triangles = []
    if data[:5] == b'solid' and b'facet' in data[:512]:
        values = [tuple(map(float, m.groups())) for m in
                  re.finditer(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)', data)]
        triangles = [values[i:i + 3] for i in range(0, len(values), 3)]
    else:
        count = struct.unpack('<I', data[80:84])[0]
        for i in range(count):
            offset = 84 + i * 50 + 12
            triangles.append([struct.unpack('<3f', data[offset + 12 * k:offset + 12 * k + 12])
                              for k in range(3)])
    return triangles


def indexed_face_set(path):
    index = {}
    points = []
    faces = []
    for triangle in read_stl(path):
        face = []
        for v in triangle:
            key = tuple(round(c, 6) for c in v)
            if key not in index:
                index[key] = len(points)
                points.append(key)
            face.append(index[key])
        if len(set(face)) == 3:
            faces.append(face)
    coords = ', '.join('%g %g %g' % p for p in points)
    indices = ', '.join('%d %d %d -1' % tuple(f) for f in faces)
    box = [(min(p[k] for p in points), max(p[k] for p in points)) for k in range(3)]
    text = ('IndexedFaceSet {\n  coord Coordinate {\n    point [ %s ]\n  }\n'
            '  coordIndex [ %s ]\n  creaseAngle 0.6\n}' % (coords, indices))
    return text, box


# ---------------------------------------------------------------------------
# PROTO text helpers
# ---------------------------------------------------------------------------
def block_end(text, start):
    """Index just after the node or list starting at the first { or [ at/after start."""
    i = start
    while text[i] not in '{[':
        i += 1
    depth = 0
    in_string = False
    while True:
        c = text[i]
        if c == '"':
            in_string = not in_string
        elif not in_string:
            if c in '{[':
                depth += 1
            elif c in '}]':
                depth -= 1
                if depth == 0:
                    return i + 1
        i += 1


def enclosing_node_start(text, index):
    """Index of the '{' of the node that directly contains position index."""
    depth = 0
    in_string = False
    i = index
    while i > 0:
        i -= 1
        c = text[i]
        if c == '"':
            in_string = not in_string
        elif not in_string:
            if c in '}]':
                depth += 1
            elif c in '{[':
                if depth == 0 and c == '{':
                    return i
                depth -= 1
    raise ValueError('no enclosing node')


def find_solid_field(text, solid, field):
    """(start, end) of 'field <node>' inside the Solid named solid, or None."""
    name_at = text.index(f'name "{solid}"')
    node_start = enclosing_node_start(text, name_at)
    node_end = block_end(text, node_start)
    # Fields of this Solid come after its children list; search from the name.
    m = re.compile(r'\n(\s*)' + field + r' ').search(text, name_at, node_end)
    if not m:
        return None
    start = m.end() - len(field) - 1
    if text.startswith('USE ', m.end()):
        return start, text.index('\n', m.end())
    return start, block_end(text, m.end())


def line_indent(text, index):
    """Indentation (in spaces) of the line containing index."""
    line_start = text.rfind('\n', 0, index) + 1
    line = text[line_start:index]
    return len(line) - len(line.lstrip(' '))


def indent(block, spaces):
    pad = ' ' * spaces
    return '\n'.join(pad + line if line else line for line in block.splitlines())


def physics_block(mass, com, inertia):
    return ('physics Physics {\n  density -1\n  mass %.6f\n  centerOfMass [ %.6f %.6f %.6f ]\n'
            '  inertiaMatrix [\n    %.6e %.6e %.6e\n    %.6e %.6e %.6e\n  ]\n}' % (
                mass, com[0], com[1], com[2],
                inertia[0][0], inertia[1][1], inertia[2][2],
                inertia[0][1], inertia[0][2], inertia[1][2]))


def axis_angle(rot):
    """Webots rotation 'x y z angle' of a 3x3 rotation matrix."""
    angle = math.acos(max(-1.0, min(1.0, (rot[0][0] + rot[1][1] + rot[2][2] - 1.0) / 2.0)))
    if angle < 1e-9:
        return (0.0, 0.0, 1.0, 0.0)
    if abs(angle - math.pi) < 1e-6:
        # axis from the largest diagonal term of (R + I) / 2
        d = [math.sqrt(max(0.0, (rot[i][i] + 1.0) / 2.0)) for i in range(3)]
        k = d.index(max(d))
        axis = [rot[k][j] / (2.0 * d[k]) if j != k else d[k] for j in range(3)]
        return (axis[0], axis[1], axis[2], angle)
    s = 2.0 * math.sin(angle)
    return ((rot[2][1] - rot[1][2]) / s, (rot[0][2] - rot[2][0]) / s,
            (rot[1][0] - rot[0][1]) / s, angle)


def box_bounding(boxes):
    """Build a boundingObject Group of boxes [(centre xyz, axis-angle or None, size xyz)]."""
    nodes = []
    for centre, rotation, size in boxes:
        rot = ''
        if rotation and abs(rotation[3]) > 1e-9:
            rot = '  rotation %.6f %.6f %.6f %.6f\n' % rotation
        nodes.append('Pose {\n  translation %.6g %.6g %.6g\n%s  children [\n'
                     '    Box {\n      size %.6g %.6g %.6g\n    }\n  ]\n}' % (
                         centre[0], centre[1], centre[2], rot, size[0], size[1], size[2]))
    return 'boundingObject Group {\n  children [\n%s\n  ]\n}' % indent('\n'.join(nodes), 4)


def solid_to_pose(text, solid):
    """Turn the Solid named solid into a Pose (keeps translation, rotation and children)."""
    name_at = text.index(f'name "{solid}"')
    start = enclosing_node_start(text, name_at)
    keyword = text.rfind('Solid', 0, start)
    if text[keyword:start].strip() != 'Solid':
        raise ValueError(f'{solid} is not a Solid node')
    for field in ('physics', 'boundingObject'):
        span = find_solid_field(text, solid, field)
        if span:
            text = text[:span[0]].rstrip(' \n') + text[span[1]:]
    name_at = text.index(f'name "{solid}"')
    line_start = text.rfind('\n', 0, name_at)
    text = text[:line_start] + text[text.index('\n', name_at):]
    return text[:keyword] + 'Pose' + text[keyword + len('Solid'):]


def main():
    os.makedirs(BUILD, exist_ok=True)
    urdf_text = upstream_urdf('ugv_rover')
    urdf_path = os.path.join(BUILD, 'ugv_rover.urdf')
    with open(urdf_path, 'w') as f:
        f.write(urdf_text)

    # ---- 4. inertia checks (done first so a bad URDF fails early) ----
    links, joints = load_urdf(urdf_text)
    print('URDF inertia check (mass kg, principal moments kg*m^2):')
    bad = False
    for name, value in links.items():
        if value is None:
            print(f'  {name:24s} no inertial (frame only)')
            continue
        mass, _, tensor = value
        problems = check_inertia(name, mass, tensor)
        bad |= bool(problems)
        print(f'  {name:24s} m={mass:.4f}  '
              f'I=({tensor[0][0]:.3e}, {tensor[1][1]:.3e}, {tensor[2][2]:.3e})'
              f'  {"; ".join(problems) if problems else "ok"}')
    if bad:
        sys.exit('Non physical inertia in the URDF, fix it before generating the PROTO.')

    # ---- 2. urdf2webots ----
    raw = os.path.join(BUILD, 'UgvRover.proto')
    subprocess.run([sys.executable, URDF2PROTO, f'--input={urdf_path}', f'--output={raw}'],
                   check=True, cwd=BUILD, stdout=subprocess.DEVNULL)
    with open(raw) as f:
        text = f.read()

    # ---- 3b. inline meshes, keep their bounding boxes for 3c ----
    mesh_boxes = {}
    mesh_def = re.compile(r'DEF (\w+) Mesh \{\s*url "file:([^"]+)"\s*\}')
    while True:
        m = mesh_def.search(text)
        if not m:
            break
        def_name = DEF_RENAMES.get(m.group(1), m.group(1))
        ifs, box = indexed_face_set(m.group(2))
        mesh_boxes[m.group(1)] = box
        body = indent(ifs, line_indent(text, m.start())).lstrip(' ')
        text = text[:m.start()] + f'DEF {def_name} ' + body + text[m.end():]
    for old, new in DEF_RENAMES.items():
        text = text.replace(f'USE {old}\n', f'USE {new}\n')

    # Collision boxes of each link, in the link frame: [(centre, size)].
    def mesh_box(mesh, turned=False):
        (x0, x1), (y0, y1), (z0, z1) = mesh_boxes[mesh]
        if turned:  # visual turned by pi around z: mirror x and y
            x0, x1, y0, y1 = -x1, -x0, -y1, -y0
        return (((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2),
                (max(x1 - x0, 1e-3), max(y1 - y0, 1e-3), max(z1 - z0, 1e-3)))

    link_boxes = {
        'base_link': BASE_COLLISION_BOXES,
        'base_lidar_link': [mesh_box('base_lidar_link')],
        'oak_base': [mesh_box('3d_camera_link')],
        'pt_base_link': [mesh_box('pt_base_link', turned=True)],
        'pt_link1': [mesh_box('pt_link1')],
        'pt_link2': [mesh_box('pt_link2')],
        'pt_camera_link': [mesh_box('pt_camera_link')],
    }

    # ---- 3d. wheels ----
    for side in ('left_up', 'left_down', 'right_up', 'right_down'):
        motor = f'name "{side}_wheel_link_joint"\n'
        at = text.index(motor)
        pad = ' ' * line_indent(text, at)
        text = text.replace(
            motor + pad + 'maxTorque 10000\n',
            motor + pad + f'maxVelocity {WHEEL_MAX_VELOCITY}\n'
            + pad + f'maxTorque {WHEEL_MAX_TORQUE}\n', 1)
        at = text.index(f'name "{side}_wheel_link"')
        pad = ' ' * line_indent(text, at)
        text = text[:at] + f'name "{side}_wheel_link"\n{pad}contactMaterial "{WHEEL_MATERIAL}"' + \
            text[at + len(f'name "{side}_wheel_link"'):]

    # ---- 3e. sensors ----
    for solid, devices in DEVICES.items():
        name_at = text.index(f'name "{solid}"')
        node_start = enclosing_node_start(text, name_at)
        children = text.index('children [', node_start) + len('children [')
        pad = line_indent(text, name_at) + 2
        text = text[:children] + ''.join('\n' + indent(d, pad) for d in devices) + text[children:]

    # ---- 3a + 3c. one Webots body per rigid body ----
    # Webots ignores the boundingObject of a Solid without physics inside a Solid with physics
    # (WbSolid.cpp: "As 'physics' is set to NULL, collisions will have no effect"), and joints
    # below such a Solid do not attach to the moving body. So every link fixed to a body root
    # becomes a plain Pose, and the root gets the combined physics and all collision boxes.
    bodies = rigid_bodies(links, joints)
    print('Rigid bodies (links joined by fixed joints):')
    for body, members in bodies.items():
        mass, com, inertia = combine(links, members)
        print(f'  {body:24s} {mass:.4f} kg  com=({com[0]:+.4f}, {com[1]:+.4f}, {com[2]:+.4f})  '
              f'links={[m[0] for m in members]}')
        physics = physics_block(mass, com, inertia)
        boxes = []
        for link, xyz, rot in members:
            for centre, size in link_boxes.get(link, []):
                boxes.append(([a + b for a, b in zip(xyz, matvec(rot, centre))],
                              axis_angle(rot), size))
        bounding = box_bounding(boxes) if boxes else None

        if body == 'base_footprint':
            anchor = 'selfCollision IS selfCollision\n'
            text = text.replace(
                anchor, anchor + indent(bounding, 4) + '\n' + indent(physics, 4) + '\n', 1)
        else:
            span = find_solid_field(text, body, 'physics')
            block = indent(physics, line_indent(text, span[0])).lstrip(' ')
            text = text[:span[0]] + block + text[span[1]:]
            if bounding:
                span = find_solid_field(text, body, 'boundingObject')
                block = indent(bounding, line_indent(text, span[0])).lstrip(' ')
                text = text[:span[0]] + block + text[span[1]:]

        for link, _, _ in members:
            if link != body and f'name "{link}"' in text:
                text = solid_to_pose(text, link)
    # ---- header ----
    text = re.sub(r'^#VRML_SIM \S+ utf8', '#VRML_SIM R2025a utf8', text)
    text = re.sub(r'# This is a proto file for Webots for the UgvRover\n'
                  r'# Extracted from: [^\n]*\n',
                  '# Waveshare UGV Rover (UGV_MODEL=ugv_rover) for Webots.\n'
                  '# GENERATED by ugv_webots/scripts/generate_proto.py '
                  'from ugv_description/urdf/bases/ugv_rover.xacro.\n'
                  '# Do not edit by hand: change the script and regenerate '
                  '(see docs/SIMULATOR.md).\n'
                  '# keywords: robot/wheeled\n'
                  '# template language: javascript\n', text)
    text = text.replace('field  SFString    name            "UgvRover"',
                        'field  SFString    name            "ugv_rover"')
    text = text.replace('field  SFString    controller      "void"',
                        'field  SFString    controller      "<extern>"')
    text = text.replace('field  SFBool      supervisor      FALSE',
                        'field  SFBool      supervisor      TRUE ')
    if 'controller      "<extern>"' not in text or 'supervisor      TRUE' not in text:
        sys.exit('Unexpected urdf2webots header, update generate_proto.py')

    with open(OUTPUT, 'w') as f:
        f.write(text)
    print(f'Wrote {os.path.relpath(OUTPUT)} ({len(text) // 1024} KiB)')


if __name__ == '__main__':
    main()
