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
