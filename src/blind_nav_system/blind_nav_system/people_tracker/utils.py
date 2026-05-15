import math
import numpy as np
import cv2


COLORS = [
    (255, 56, 56), (255, 157, 151), (255, 112, 31), (255, 178, 29),
    (207, 210, 49), (72, 249, 10), (146, 204, 23), (61, 219, 134),
    (26, 147, 52), (0, 212, 187), (44, 153, 168), (0, 194, 255),
    (52, 69, 147), (100, 115, 255), (0, 24, 236), (132, 56, 255),
    (82, 0, 133), (203, 56, 255), (255, 149, 200), (255, 55, 199),
]


def get_color(track_id: int) -> tuple[int, int, int]:
    return COLORS[int(track_id) % len(COLORS)]


def center_of_box(x1: float, y1: float, x2: float, y2: float) -> tuple[float, float]:
    return (x1 + x2) / 2.0, (y1 + y2) / 2.0


def bottom_center_of_box(x1: float, y1: float, x2: float, y2: float) -> tuple[float, float]:
    return (x1 + x2) / 2.0, y2


def angle_degrees(vx: float, vy: float) -> float:
    return math.degrees(math.atan2(-vy, vx))


def speed(vx: float, vy: float) -> float:
    return math.sqrt(vx * vx + vy * vy)


def draw_text_with_bg(
    img: np.ndarray,
    text: str,
    pos: tuple[int, int],
    font_scale: float = 0.5,
    thickness: int = 1,
    fg: tuple = (255, 255, 255),
    bg: tuple = (0, 0, 0),
):
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    x, y = pos
    cv2.rectangle(img, (x - 2, y - th - 4), (x + tw + 2, y + baseline), bg, -1)
    cv2.putText(img, text, (x, y), font, font_scale, fg, thickness, cv2.LINE_AA)
