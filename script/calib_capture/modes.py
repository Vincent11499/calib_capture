from __future__ import annotations


STEREO_IMU_MODE = "stereo-imu"
LEGACY_STEREO_IMU_MODE = "camera-imu"
STEREO_BASELINK_MODE = "stereo-baselink-triggered"
LEGACY_STEREO_BASELINK_MODE = "stereo_base_link_triggered"
USB_CONTINUOUS_MODE = "usb-continuous"
USB_BASELINK_MODE = "usb-baselink-triggered"

USB_MODES = frozenset({USB_CONTINUOUS_MODE, USB_BASELINK_MODE})
STEREO_IMU_READ_MODES = frozenset({STEREO_IMU_MODE, LEGACY_STEREO_IMU_MODE})
STEREO_BASELINK_READ_MODES = frozenset(
    {STEREO_BASELINK_MODE, LEGACY_STEREO_BASELINK_MODE}
)


def is_usb_mode(mode: object) -> bool:
    return str(mode) in USB_MODES


def is_stereo_imu_mode(mode: object) -> bool:
    return str(mode) in STEREO_IMU_READ_MODES


def is_stereo_baselink_mode(mode: object) -> bool:
    return str(mode) in STEREO_BASELINK_READ_MODES
