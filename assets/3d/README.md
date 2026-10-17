# UGV Rover 3D models

Visual models of the Waveshare UGV Rover PT Jetson Orin kit, built from Waveshare's STEP
model by `src/ugv_main/ugv_webots/scripts/build_rover_meshes.py`. Do not edit these files
by hand: change the script or `materials.json` and regenerate (see
[docs/SIMULATOR.md, Robot model](../../docs/SIMULATOR.md#robot-model)).

| Path | What | Used by |
|---|---|---|
| `materials.json` | PBR materials (linear base colour, metalness, roughness, emissive) | both exports, `generate_proto.py` |
| `webots/<link>.dae` | One Collada file per URDF link, one material group per material, in the link frame, metres, `Z_UP` | Webots (`ugv_webots/protos/meshes` links here) |
| `webots/wheel.dae`, `tire_normal.png`, `tire_color.png` | One wheel (tyre and hub) and the tread maps, used for all six wheels | Webots |
| `webots/manifest.json` | Files, material order, triangle counts, joints, the visual shifts | `generate_proto.py`, tools |
| `glb/ugv_rover.glb` | The whole robot with the URDF link hierarchy | three.js and other web viewers |

Visuals only: no collision shapes, no masses. About 57k triangles for the whole robot
(per mesh numbers in `webots/manifest.json`).

## Using the GLB in three.js

```js
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
const gltf = await new GLTFLoader().loadAsync('ugv_rover.glb');
scene.add(gltf.scene);
const pan = gltf.scene.getObjectByName('pt_link1');      // rotate about its local z
const tilt = gltf.scene.getObjectByName('pt_link2');     // rotate about its local z
const wheel = gltf.scene.getObjectByName('left_up_wheel_link');  // rotate about its local y
```

* Units are metres. The root node `ugv_rover` turns the robot from ROS (z up, x forward) to
  glTF (y up); everything below it is in ROS coordinates, so URDF joint values apply as is.
* Node names are the URDF link names: `base_footprint`, `base_link`, `base_lidar_link`,
  `oak_base`, `pt_base_link`, `pt_link1` (pan, local z, -3.14 to 3.14 rad), `pt_link2`
  (tilt, local z, -0.52 to 1.57 rad), and the four wheel links (local y). Each node's
  transform is the URDF joint origin, joints at zero. `left_mid_wheel` and
  `right_mid_wheel` are the unpowered middle wheels (fixed to the chassis, spin them with
  the others if you like).
* Positions follow the CAD and the URDF of the Jetson Orin kit (`ugv_rover.xacro`
  with `UGV_KIT` unset or `jetson_orin`, see [docs/MODEL_CHANGES.md](../../docs/MODEL_CHANGES.md)).
