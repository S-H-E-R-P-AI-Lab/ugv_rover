"""Build the robot description used by the Webots simulation.

The URDF is generated from the upstream xacro
(ugv_description/urdf/bases/<model>.xacro) so the simulation always uses the
same links, joints and frames as the real robot and the Gazebo setup.

Two things differ from a plain ``xacro ugv_rover.xacro`` call:

1. The include of ``urdf/gazebo/<model>.gazebo`` is removed before processing.
   That file only holds Gazebo plugins (diff drive, sensors, friction).
2. ``use_gazebo`` is set to true. With the Gazebo include gone, this flag only
   selects the simulation layout of the OAK camera frames (``oak_base`` plus
   the optical ``3d_camera_link``), which is what the simulated topics are
   stamped with. With ``use_gazebo:=false`` the xacro pulls in the frames of the
   real depthai driver instead.

``webots_urdf()`` then drops the upstream ``<ros2_control>``, ``<transmission>``
and ``<gazebo>`` blocks and appends ``resource/webots_extras.urdf`` (the
``webots_ros2_control`` hardware and the Webots sensor plugins).
"""

import argparse
import os
import re
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro

_GAZEBO_INCLUDE = re.compile(
    r'<xacro:if value="\$\{use_gazebo\}">\s*'
    r'<xacro:include filename="[^"]*/urdf/gazebo/[^"]*"\s*/>\s*'
    r'<xacro:gazebo[^>]*/>\s*'
    r'</xacro:if>',
    re.S)

_STRIPPED_TAGS = ('gazebo', 'ros2_control', 'transmission')


def upstream_urdf(model='ugv_rover'):
    """Return the upstream URDF for ``model`` without Gazebo plugins or ros2_control."""
    share = get_package_share_directory('ugv_description')
    bases = os.path.join(share, 'urdf', 'bases')
    with open(os.path.join(bases, model + '.xacro')) as f:
        text = f.read()
    text, count = _GAZEBO_INCLUDE.subn('', text)
    if count != 1:
        raise RuntimeError(
            f'Expected exactly one Gazebo include block in {model}.xacro, found {count}. '
            'The upstream xacro changed; update _GAZEBO_INCLUDE in ugv_webots/description.py.')

    # The xacro is processed from a temporary copy, so relative includes
    # (urdf_extras defaults to "empty.urdf") are pointed back at the original folder.
    with tempfile.NamedTemporaryFile('w', suffix='.xacro', delete=False) as tmp:
        tmp.write(text)
    try:
        doc = xacro.process_file(tmp.name, mappings={
            'use_gazebo': 'true',
            'GZ_VERSION': 'webots',
            'urdf_extras': os.path.join(bases, 'empty.urdf'),
        })
    finally:
        os.unlink(tmp.name)

    root = ET.fromstring(doc.toxml())
    for tag in _STRIPPED_TAGS:
        for element in root.findall(tag):
            root.remove(element)
    return ET.tostring(root, encoding='unicode')


def webots_urdf(model='ugv_rover'):
    """Return the URDF for robot_state_publisher and the Webots driver."""
    root = ET.fromstring(upstream_urdf(model))
    extras_path = os.path.join(
        get_package_share_directory('ugv_webots'), 'resource', 'webots_extras.urdf')
    for element in ET.parse(extras_path).getroot():
        root.append(element)
    return ET.tostring(root, encoding='unicode')


def write_webots_urdf(path, model='ugv_rover'):
    """Write the Webots URDF to ``path`` (the driver reads it from a file)."""
    with open(path, 'w') as f:
        f.write(webots_urdf(model))
    return path


def main():
    parser = argparse.ArgumentParser(description='Print the URDF used by the Webots simulation.')
    parser.add_argument('--model', default=os.environ.get('UGV_MODEL', 'ugv_rover'))
    parser.add_argument('--upstream-only', action='store_true',
                        help='print the URDF without the Webots extras (input of urdf2webots)')
    args = parser.parse_args()
    print(upstream_urdf(args.model) if args.upstream_only else webots_urdf(args.model))


if __name__ == '__main__':
    main()
