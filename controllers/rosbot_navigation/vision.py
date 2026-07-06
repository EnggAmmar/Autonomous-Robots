"""Camera color detection for blue/yellow pillars and green forbidden floor."""
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

import config


@dataclass
class ColorDetection:
    visible: bool
    center_x: Optional[int] = None
    center_y: Optional[int] = None
    area: int = 0
    image_width: int = 0
    image_height: int = 0

    @property
    def horizontal_error(self) -> float:
        """Normalized horizontal error: -1 left, +1 right."""
        if not self.visible or self.center_x is None or self.image_width <= 0:
            return 0.0
        return (self.center_x - self.image_width / 2.0) / (self.image_width / 2.0)


def webots_camera_to_bgr(camera) -> Optional[np.ndarray]:
    """Convert Webots camera image to OpenCV BGR array."""
    if camera is None:
        return None
    width = camera.getWidth()
    height = camera.getHeight()
    raw = camera.getImage()
    if raw is None:
        return None
    # Webots camera image is BGRA bytes for Python controllers.
    img = np.frombuffer(raw, np.uint8).reshape((height, width, 4))
    bgr = img[:, :, :3].copy()
    return bgr


def detect_hsv_blob(bgr: np.ndarray, low: Tuple[int, int, int], high: Tuple[int, int, int]) -> ColorDetection:
    h, w = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return ColorDetection(False, image_width=w, image_height=h)

    largest = max(contours, key=cv2.contourArea)
    area = int(cv2.contourArea(largest))
    if area < config.MIN_COLOR_AREA:
        return ColorDetection(False, area=area, image_width=w, image_height=h)

    m = cv2.moments(largest)
    if m["m00"] == 0:
        return ColorDetection(False, area=area, image_width=w, image_height=h)

    cx = int(m["m10"] / m["m00"])
    cy = int(m["m01"] / m["m00"])
    return ColorDetection(True, cx, cy, area, w, h)


def detect_blue(bgr: np.ndarray) -> ColorDetection:
    return detect_hsv_blob(bgr, config.BLUE_HSV_LOW, config.BLUE_HSV_HIGH)


def detect_yellow(bgr: np.ndarray) -> ColorDetection:
    return detect_hsv_blob(bgr, config.YELLOW_HSV_LOW, config.YELLOW_HSV_HIGH)


def detect_green(bgr: np.ndarray) -> ColorDetection:
    return detect_hsv_blob(bgr, config.GREEN_HSV_LOW, config.GREEN_HSV_HIGH)


def target_reached(det: ColorDetection) -> bool:
    return det.visible and det.area >= config.TARGET_REACHED_AREA


def print_camera_hsv_stats(bgr: np.ndarray, brightness_thresh: int = 40):
    """Print the HSV range of the brighter pixels in the current frame.

    Point the camera at a pillar or the green floor while in CAMERA_CALIBRATION
    mode and read these ranges to tune config.py's *_HSV_LOW/HIGH thresholds
    against the real environment instead of guessing.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    mask = gray > brightness_thresh
    if not np.any(mask):
        print("[calibration] frame too dark, no pixels above brightness threshold")
        return
    pixels = hsv[mask]
    h_min, s_min, v_min = pixels.min(axis=0)
    h_max, s_max, v_max = pixels.max(axis=0)
    print(
        f"[calibration] all-bright   H=[{h_min},{h_max}] S=[{s_min},{s_max}] V=[{v_min},{v_max}] "
        f"pixels={pixels.shape[0]}"
    )

    # Gray walls/floor are low-saturation and skew the range above. Restricting
    # to saturated pixels isolates an actual colored target (pillar/green patch).
    saturated_mask = mask & (hsv[:, :, 1] > 80)
    if np.any(saturated_mask):
        sat_pixels = hsv[saturated_mask]
        sh_min, ss_min, sv_min = sat_pixels.min(axis=0)
        sh_max, ss_max, sv_max = sat_pixels.max(axis=0)
        print(
            f"[calibration] saturated>80 H=[{sh_min},{sh_max}] S=[{ss_min},{ss_max}] V=[{sv_min},{sv_max}] "
            f"pixels={sat_pixels.shape[0]}"
        )

    # A wide fieldOfView can catch background sky/hills alongside the target.
    # If the target is centered in frame, this crop isolates it from that background.
    h, w = hsv.shape[:2]
    cy0, cy1 = int(0.55 * h), int(0.95 * h)
    cx0, cx1 = int(0.35 * w), int(0.65 * w)
    center = hsv[cy0:cy1, cx0:cx1]
    ch_min, cs_min, cv_min = center.reshape(-1, 3).min(axis=0)
    ch_max, cs_max, cv_max = center.reshape(-1, 3).max(axis=0)
    print(
        f"[calibration] center-crop  H=[{ch_min},{ch_max}] S=[{cs_min},{cs_max}] V=[{cv_min},{cv_max}]"
    )

    center_saturated = center[center[:, :, 1] > 80]
    if center_saturated.size:
        csh_min, css_min, csv_min = center_saturated.min(axis=0)
        csh_max, css_max, csv_max = center_saturated.max(axis=0)
        print(
            f"[calibration] center+sat80 H=[{csh_min},{csh_max}] S=[{css_min},{css_max}] V=[{csv_min},{csv_max}] "
            f"pixels={center_saturated.shape[0]}"
        )
