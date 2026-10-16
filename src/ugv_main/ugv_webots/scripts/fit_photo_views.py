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
Find the camera pose of each reference photo, for the model comparison renders.

    uv run src/ugv_main/ugv_webots/scripts/fit_photo_views.py

Waveshare's product photos are renders of the same CAD model on white. For each photo in
PHOTOS this renders the silhouette of the CAD (in base_link, wheels on the ground) and
searches the camera position, target and field of view that maximise the overlap
(intersection over union) with the photo's silhouette. The result goes to
docs/reference/comparisons/views.json for render_views.py; the IoU of each fit is printed.
"""

import json
import math
import os
import sys

import numpy as np
from PIL import Image
from scipy.optimize import minimize

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import build_rover_meshes as brm  # noqa: E402

PHOTOS_DIR = os.path.join(brm.REPO, 'docs', 'reference', 'photos')
OUT = os.path.join(brm.REPO, 'docs', 'reference', 'comparisons', 'views.json')
# photo file -> (view name, initial azimuth deg (0 = from the front, 90 = from the left),
# elevation deg, horizontal field of view deg)
PHOTOS = {
    'product-1.jpg': ('front_three_quarter', 40, 22, 20),
    'product-2.jpg': ('side_right', -90, 2, 10),
    'product-3.jpg': ('side_left', 90, 2, 10),
    'product-4.jpg': ('front', 0, 2, 10),
    'product-5.jpg': ('rear', 180, 2, 10),
    'product-6.jpg': ('top', 90, 85, 10),  # front to the left (OLED window on the right)
    'product-7.jpg': ('rear_three_quarter', 140, 22, 20),
}
FIT_SIZE = (320, 240)
BASE_HEIGHT = 0.081  # base_link above the ground (URDF base_joint)


def silhouette_photo(path):
    img = np.asarray(Image.open(path).convert('RGB').resize(FIT_SIZE, Image.BILINEAR)).astype(int)
    return (img.min(2) < 235)


class Silhouette:
    def __init__(self, tris):
        import moderngl
        self.ctx = moderngl.create_context(standalone=True, backend='egl')
        self.prog = self.ctx.program(
            vertex_shader='#version 330\nin vec3 pos; uniform mat4 mvp;\n'
                          'void main() { gl_Position = mvp * vec4(pos, 1.0); }',
            fragment_shader='#version 330\nout vec4 c;\nvoid main() { c = vec4(1.0); }')
        vbo = self.ctx.buffer(tris.astype('f4').tobytes())
        self.vao = self.ctx.vertex_array(self.prog, [(vbo, '3f', 'pos')])
        self.fbo = self.ctx.simple_framebuffer(FIT_SIZE, components=1)

    def render(self, eye, target, hfov):
        w, h = FIT_SIZE
        f = 1 / math.tan(hfov / 2)
        near, far = 0.05, 20.0
        proj = np.array([[f, 0, 0, 0], [0, f * w / h, 0, 0],
                         [0, 0, -(far + near) / (far - near), -2 * far * near / (far - near)],
                         [0, 0, -1, 0]])
        mvp = (proj @ brm.look_at(np.asarray(eye, float), np.asarray(target, float))).astype('f4')
        self.prog['mvp'].write(mvp.T.copy().tobytes())
        self.fbo.use()
        self.fbo.clear(0, 0, 0, 1)
        self.vao.render()
        img = np.frombuffer(self.fbo.read(components=1), np.uint8).reshape(h, w)[::-1]
        return img > 0


def camera(params):
    az, el, dist, hfov, tx, ty, tz = params
    el = min(el, 85.0)  # keep the azimuth meaningful (it sets the image rotation from above)
    d = np.array([math.cos(math.radians(el)) * math.cos(math.radians(az)),
                  math.cos(math.radians(el)) * math.sin(math.radians(az)),
                  math.sin(math.radians(el))])
    target = np.array([tx, ty, tz])
    return target + d * dist, target, math.radians(hfov)


def main():
    parts = brm.read_step(brm.fetch_step(None))
    tris = np.concatenate([brm.apply(brm.CAD_TO_BASE, p['v'])[p['f']] for p in parts])
    tris = tris.reshape(-1, 3)
    tris[:, 2] += BASE_HEIGHT  # in the world: base_footprint on the ground
    sil = Silhouette(tris)
    views = []
    for photo, (name, az, el, hfov) in PHOTOS.items():
        path = os.path.join(PHOTOS_DIR, photo)
        if not os.path.exists(path):
            print(f'skip {photo}: not in {PHOTOS_DIR}')
            continue
        target_mask = silhouette_photo(path)

        def cost(p):
            m = sil.render(*camera(p))
            return 1 - (m & target_mask).sum() / max(1, (m | target_mask).sum())

        # distance that frames the 0.3 m robot like the photo, from the field of view
        best = None
        for daz in (-10, 0, 10):
            for dfov in (0.7, 1.0, 1.4):
                p0 = [az + daz, el, 0.45 / math.tan(math.radians(hfov * dfov) / 2), hfov * dfov,
                      0, 0, 0.13]
                r = minimize(cost, p0, method='Nelder-Mead',
                             options={'maxiter': 600, 'xatol': 1e-3, 'fatol': 1e-4,
                                      'initial_simplex': None})
                if best is None or r.fun < best.fun:
                    best = r
        eye, target, fov = camera(best.x)
        print(f'{photo}: {name} IoU {1 - best.fun:.3f} az {best.x[0]:.1f} '
              f'el {min(best.x[1], 85):.1f} fov {math.degrees(fov):.1f} deg')
        views.append({'name': name, 'photo': photo, 'iou': round(1 - best.fun, 4),
                      'position': [round(x, 5) for x in eye],
                      'target': [round(x, 5) for x in target],
                      'fov': round(fov, 5), 'width': 1600, 'height': 1200})
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w') as f:
        json.dump({'note': 'Camera poses fitted to the reference photos by '
                           'fit_photo_views.py (silhouette IoU of the CAD). World frame: '
                           'base_footprint at the origin.',
                   'views': views}, f, indent=1)
    print(f'wrote {OUT}')


if __name__ == '__main__':
    main()
