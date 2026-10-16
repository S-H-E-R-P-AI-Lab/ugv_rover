"""
Find the address of the machine running Webots when ROS runs in WSL or a VM.

webots_ros2 2025.0.0 (the Humble release) connects the robot controller to
Windows Webots at the first "nameserver" of /etc/resolv.conf. Recent WSL
versions enable DNS tunneling by default, which makes that address
10.255.255.254, not the Windows host, so the controller never connects.
Newer webots_ros2 sources use the default gateway instead, which is right for
WSL NAT networking but wrong for mirrored networking (there the gateway is your
LAN router and Windows is reachable at 127.0.0.1).

``patch_webots_ros2()`` replaces the address lookup of webots_ros2 with
``webots_host()`` below. Set UGV_WEBOTS_HOST to force an address.
"""

import os
import platform
import shutil
import subprocess


def is_wsl():
    return 'microsoft-standard' in platform.uname().release


def wsl_networking_mode():
    """Return 'nat', 'mirrored', ... or None if it cannot be determined."""
    if shutil.which('wslinfo'):
        try:
            out = subprocess.run(['wslinfo', '--networking-mode'],
                                 capture_output=True, text=True, timeout=5)
            mode = out.stdout.strip().lower()
            if mode:
                return mode
        except (OSError, subprocess.SubprocessError):
            pass
    return None


def default_gateway():
    try:
        out = subprocess.run(['ip', 'route', 'show', 'default'],
                             capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.SubprocessError):
        return None
    for line in out.stdout.splitlines():
        fields = line.split()
        if fields and fields[0] == 'default' and 'via' in fields:
            return fields[fields.index('via') + 1]
    return None


def webots_host():
    """Address of the Webots host, or None when Webots runs on this machine."""
    override = os.environ.get('UGV_WEBOTS_HOST')
    if override:
        return override
    if 'WEBOTS_SHARED_FOLDER' in os.environ:
        # macOS: ROS in a UTM VM, Webots on the Mac. With UTM's shared network
        # the Mac is the default gateway of the VM.
        return default_gateway()
    if is_wsl():
        if wsl_networking_mode() == 'mirrored':
            return '127.0.0.1'
        return default_gateway() or '127.0.0.1'
    return None


def patch_webots_ros2():
    """Make webots_ros2 connect controllers to ``webots_host()``. Returns the address used."""
    host = webots_host()
    if host is None:
        return None
    from webots_ros2_driver import utils
    utils.get_wsl_ip_address = lambda: host
    utils.get_host_ip = lambda: host
    return host
