"""Shared G10 data paths derived from the *installed workspace*, not HOME.

A root-run ROS node and an ordinary-user dashboard must see the same data.
This module does not create directories or modify ownership/permissions.
"""
from pathlib import Path


def workspace_from_install_prefix(package_prefix):
    """Support colcon isolated (install/pkg) and merged (install) layouts."""
    prefix = Path(package_prefix)
    for ancestor in (prefix, *prefix.parents):
        if ancestor.name == "install":
            return ancestor.parent
    raise ValueError(
        "Could not locate a colcon 'install' directory in package prefix %s; "
        "set log_directory and g10_calibration_file explicitly." % prefix)


def shared_ros_parameters(package_prefix):
    """Return absolute directories shared by root and desktop-user processes."""
    workspace = workspace_from_install_prefix(package_prefix)
    return {
        "log_directory": str(workspace / "measurements"),
        "g10_calibration_file": str(
            workspace / "calibration" / "g10_channel6.json"),
    }
