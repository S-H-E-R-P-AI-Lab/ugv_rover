#!/usr/bin/env python3
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

r"""
Render the UGV Rover in Webots from fixed camera poses (for model comparisons).

Run with ROS sourced, after building ugv_webots:

    python3 src/ugv_main/ugv_webots/scripts/render_views.py \\
        --views docs/reference/comparisons/views.json --out /tmp/renders --detail high

Starts Webots on a studio world (white background, office light probe, light grey floor,
key and fill lights),
puts the robot at the origin with every joint at zero, and saves one PNG per view from a
Camera moved by a supervisor. A view is {"name", "position": [x, y, z],
"target": [x, y, z], "fov": horizontal rad, "width", "height", optional "pan", "tilt"}.
Webots' own messages go to <out>/webots.log.
"""

import argparse
import json
import math
import os
import subprocess
import sys
import time

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from ugv_webots.webots_host import is_wsl, webots_host  # noqa: E402

PORT = 1239
BACKGROUNDS = ('https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/'
               'backgrounds/protos')

WORLD = '''#VRML_SIM R2025a utf8
EXTERNPROTO "{bg}/TexturedBackground.proto"
EXTERNPROTO "../protos/UgvRover.proto"
WorldInfo {{
  basicTimeStep 10
}}
Viewpoint {{
  orientation -0.2 0.2 0.96 1.6
  position 0.9 -0.9 0.6
}}
TexturedBackground {{
  texture "{light}"
  luminosity {luminosity}
  skybox FALSE
  skyColor [ 1 1 1 ]
}}
DirectionalLight {{
  direction -0.55 0.35 -1
  intensity 4
}}
DirectionalLight {{
  direction 0.6 -0.7 -0.5
  intensity 2
}}
DirectionalLight {{
  direction 0.2 0.9 -0.3
  intensity 1.2
}}
Solid {{
  name "floor"
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor {floor} {floor} {floor}
        roughness 1
        metalness 0
      }}
      geometry Plane {{
        size 10 10
      }}
    }}
  ]
  boundingObject Plane {{
    size 10 10
  }}
}}
UgvRover {{
  controller "<none>"
  detail "{detail}"
}}
Robot {{
  name "render_views"
  controller "<extern>"
  supervisor TRUE
  children [
    Camera {{
      name "cam"
      width {width}
      height {height}
      fieldOfView 0.5
      near 0.01
      far 20
      antiAliasing TRUE
      zoom Zoom {{
        maxFieldOfView 1.5
        minFieldOfView 0.02
      }}
    }}
  ]
}}
'''


def look_rotation(position, target):
    """Webots axis-angle that points a camera (looking along +x, z up) at target."""
    x = np.asarray(target, float) - np.asarray(position, float)
    x /= np.linalg.norm(x)
    # same convention as build_rover_meshes.look_at: z is up unless looking straight down
    up = np.array([0, 0, 1.0]) if abs(x[2]) < 0.999 else np.array([1.0, 0, 0])
    y = np.cross(up, x)
    y /= np.linalg.norm(y)
    z = np.cross(x, y)
    r = np.stack([x, y, z], 1)
    angle = math.acos(max(-1.0, min(1.0, (np.trace(r) - 1) / 2)))
    if angle < 1e-9:
        return [0, 0, 1, 0]
    if abs(angle - math.pi) < 1e-6:
        d = np.sqrt(np.maximum(0, (np.diag(r) + 1) / 2))
        k = int(np.argmax(d))
        axis = r[k] / (2 * d[k])
        axis[k] = d[k]
        return list(axis) + [angle]
    axis = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    axis /= 2 * math.sin(angle)
    return list(axis) + [angle]


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--views', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--detail', default='high', choices=['high', 'low'])
    parser.add_argument('--light', default='empty_office')
    parser.add_argument('--luminosity', type=float, default=1.0, help='background light probe')
    parser.add_argument('--floor', type=float, default=0.85, help='floor grey level (linear)')
    args = parser.parse_args()
    views = json.load(open(args.views))['views']
    os.makedirs(args.out, exist_ok=True)
    width = max(v['width'] for v in views)
    height = max(v['height'] for v in views)

    from ament_index_python.packages import get_package_share_directory
    worlds = os.path.join(get_package_share_directory('ugv_webots'), 'worlds')
    world = os.path.join(worlds, '.render_views.wbt')
    with open(world, 'w') as f:
        f.write(WORLD.format(bg=BACKGROUNDS, detail=args.detail, width=width, height=height,
                             light=args.light, luminosity=args.luminosity, floor=args.floor))
    webots_home = os.environ.get('WEBOTS_HOME', '/usr/local/webots')
    if is_wsl():
        exe = os.path.join(webots_home, 'msys64', 'mingw64', 'bin', 'webots.exe')
        world_arg = subprocess.check_output(['wslpath', '-w', world]).decode().strip()
    else:
        exe = os.path.join(webots_home, 'webots')
        world_arg = world
    log = open(os.path.join(args.out, 'webots.log'), 'w')
    t0 = time.time()
    webots = subprocess.Popen([exe, '--batch', '--mode=fast', '--minimize', '--no-rendering',
                               '--stdout', '--stderr', f'--port={PORT}', world_arg],
                              stdout=log, stderr=subprocess.STDOUT)
    host = webots_host() or '127.0.0.1'
    os.environ['WEBOTS_CONTROLLER_URL'] = f'tcp://{host}:{PORT}/render_views'
    if is_wsl():  # the controller library is the Linux one shipped with webots_ros2_driver
        from ament_index_python.packages import get_package_prefix
        os.environ['WEBOTS_HOME'] = get_package_prefix('webots_ros2_driver')
    from controller import Supervisor
    try:
        robot = Supervisor()
        print(f'Webots connected after {time.time() - t0:.1f} s')
        step = int(robot.getBasicTimeStep())
        cam = robot.getDevice('cam')
        cam.enable(step)
        me = robot.getSelf()
        rover = None
        for node_index in range(robot.getRoot().getField('children').getCount()):
            node = robot.getRoot().getField('children').getMFNode(node_index)
            if node.getTypeName() == 'UgvRover':
                rover = node
        for _ in range(60):  # let the robot settle on its wheels
            robot.step(step)
        for view in views:
            me.getField('translation').setSFVec3f(list(view['position']))
            me.getField('rotation').setSFRotation(look_rotation(view['position'], view['target']))
            cam.setFov(view['fov'])
            for _ in range(3):
                robot.step(step)
            img = np.frombuffer(cam.getImage(), np.uint8).reshape(height, width, 4)
            img = img[:, :, [2, 1, 0]]
            w, h = view['width'], view['height']
            y0, x0 = (height - h) // 2, (width - w) // 2
            Image.fromarray(img[y0:y0 + h, x0:x0 + w]).save(
                os.path.join(args.out, view['name'] + '.png'))
            print('saved', view['name'])
        if rover is not None:
            pos = rover.getPosition()
            print('robot settled at', [round(p, 4) for p in pos])
        robot.simulationQuit(0)
        robot.step(step)
    finally:
        try:
            webots.wait(30)
        except subprocess.TimeoutExpired:
            webots.kill()
        os.remove(world)
        log.close()


if __name__ == '__main__':
    main()
