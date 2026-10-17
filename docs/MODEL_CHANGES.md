# Robot model versus Waveshare's CAD

The Webots robot model (`ugv_webots/protos/UgvRover.proto`) takes its kinematics from the
upstream URDF (`ugv_description/urdf/bases/ugv_rover.xacro`) and, since the high detail
model, its visuals from Waveshare's STEP model of the **UGV Rover PT Jetson Orin ROS2 Kit**
(see [SIMULATOR.md, Robot model](SIMULATOR.md#robot-model)). This page records where the
two disagree, how it was measured, and what was changed.

**Nothing was changed yet.** No link frame, joint origin, joint axis, sensor pose, mass or
inertia differs from upstream. The discrepancies below are moved into the URDF in a
separate pull request, so they can be reviewed on their own.

## How it was measured

* The STEP was tessellated with OpenCascade (`build_rover_meshes.py`) and put in
  `base_link`: the CAD frame has x pointing backwards and y to the right, so
  `base_link = Rz(180 deg) * CAD / 1000`. Its origin is the top face of the chassis between
  the wheel axles, which is also the URDF `base_link` origin (wheel centres agree to 0.1 mm,
  see below).
* Every URDF mesh (`ugv_description/meshes`) was placed at its link pose with joints at
  zero and fitted to the CAD part it represents with ICP (6000 points on the URDF mesh,
  200000 on the CAD, 90 % inliers). The fitted translation is the offset of that link, the
  residual (RMS distance after the fit) shows the URDF mesh really is that CAD part.
* Joint axes were read from the CAD directly: the servo horns (`金属舵盘`) give the pan
  and tilt axes, the LD19 dome gives the lidar rotation axis.
* The 2D drawing from the same Waveshare page (`docs/reference/ugv_rover_pt_jetson_orin_2d_drawing.pdf`)
  agrees with the STEP: length 252.4 mm, width 230.42 mm, height to the camera rail
  264.13 mm, to the antenna tips 283.09 mm (CAD: 252.4, 230.4, 264.1, 283.1).

![URDF meshes (red) over the CAD (grey), right side](reference/comparisons/urdf_vs_cad_side.png)

## Results (base_link frame, millimetres)

| Frame or part | URDF | CAD | CAD minus URDF | URDF mesh fit residual |
|---|---|---|---|---|
| Driven wheel centres (4) | x ±85.50, y ±87.25, z -41.21 | x ±85.5, y ±87.3, z -41.2 | below 0.1 | wheel radius 40.0 (URDF) vs 40.6 (CAD tyre) |
| Middle wheel centres (2) | not modelled | x 0, y ±88.3, z -41.2 | | |
| Chassis (`base_link` mesh, lower part) | | | below 0.2 in x and y | median 1.6 (simplified mesh) |
| Deck top (top of the URDF base mesh) | z 40.0 | z 49.6 | **+9.6** | |
| `base_lidar_link` (LD19 axis and base) | (31.7, -1.0, 40.0) | (39.8, 0.1, 50.0) | **(+8.1, +1.1, +10.0)** | 0.5 |
| `oak_base` (OAK-D Lite body) | (65.3, 0, 22.0) | (68.4, 0, 32.1) | **(+3.1, 0, +10.1)** | 0.9 |
| Pan axis (`pt_base_link_to_pt_link1`) | x -14.4, y 0 | x -8.7, y 0 | **+5.7 in x** | |
| `pt_base_link` (mount on the deck) | z 40.0 | z 49.6 | +9.6 | 4.6 (different standoffs) |
| `pt_link1` (pan bracket) | origin (-14.4, 0, 78.3) | | **(+5.6, 0, +19.1)** | 0.4 |
| Tilt axis (`pt_link1_to_pt_link2`) | x -14.4, z 121.3 | x -8.7, z 140.6 | **(+5.7, +19.3) in x, z** | |
| `pt_link2` (camera head) | | | (+4.8, -0.9, +18.9) | 3.1 |
| `pt_camera_link` (camera board) | (34.0, 0, 120.7) | | (+5.8, -0.5, +19.4) | 1.7 |

Wheel separation, wheel radius and the base of the robot agree. Everything mounted on the
upper deck sits 10 mm lower in the URDF than on this kit, and the pan-tilt head 19 mm lower
and 6 mm further back. The URDF meshes match the CAD parts almost exactly once shifted
(0.4 to 0.9 mm for the lidar, the OAK and the pan bracket), so the URDF describes the same
parts mounted differently: most likely the Raspberry Pi build of the rover, which has no
Jetson under the deck (lower standoffs) and shorter pan-tilt standoffs (`M3*35` in this kit).

## What the high detail model does about it

The visuals are moved onto the URDF frames instead of moving the frames
(`GROUP_SHIFT` in `build_rover_meshes.py`): the deck with the antennas and the Jetson
9.6 mm down, the LD19 and the OAK onto their URDF frames, the whole pan-tilt 5.7 mm back
and 19.3 mm down (the pan servo sinks into the deck to make room). So joints turn about the
axes you see, the lidar and the cameras sit where their sensors are, and TF and every
topic are unchanged. The price: the deck and the head look 10 and 19 mm lower than on the
real robot.

## Drivetrain: six wheels, four joints

The kit has six wheels. The CAD shows four `JGB37-520` gear motors (front and rear wheels,
shaft couplers `COUPLING 6mm`) and two **unpowered** middle wheels on `F624ZZ` flanged
bearings in `BEAR-HOLDER` brackets, with nylon spacers. The URDF models only the four
driven wheels as joints (`left_up`, `left_down`, `right_up`, `right_down`); its chassis
mesh `ugv_rover_base.stl` already draws all six wheels as part of the static body.

The high detail model keeps that split: the middle wheels are visual only, fixed to the
chassis (same wheel mesh, at the CAD centres). They were not made passive wheels with
contacts because the drive was tuned with four contacts (asymmetric friction plus
`wheel_separation_multiplier`, see [SIMULATOR.md](SIMULATOR.md#drive-and-contact-tuning)),
and that tuning already stands in for the scrub of the middle wheels; two more contacts
would change turning, odometry checks and the mass, which this change must not touch. The
cost is that the middle wheels do not spin in the simulation.
