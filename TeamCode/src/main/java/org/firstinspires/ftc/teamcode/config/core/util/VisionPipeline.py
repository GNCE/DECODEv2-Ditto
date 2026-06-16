# noinspection PyUnresolvedReferences
"""
Limelight Python SnapScript — DECODE artifact (ball) detector.

Detects PURPLE and GREEN artifacts that are circular but OVERLAP each other, so a
full circle is rarely visible. The camera is tilted slightly down from horizontal
and mounted only a little above the balls, so balls appear as partially-occluded
arcs/blobs rather than clean circles.

This script only does the per-frame DETECTION and reports each ball as a pair of
camera angles (so the geometry is independent of the robot's pose). All of the
"turn this into a field coordinate", "track velocity" and "plan a path" work is
done ON THE ROBOT in Java (see BallLocalizer.java and BallPathPlanner.java),
because that side knows the robot's odometry pose and can remove the robot's own
motion from the apparent ball motion.

----------------------------------------------------------------------------------
llpython output layout (max 32 doubles, read on the robot via LLResult.getPythonOutput())
    [0]                = N, number of balls reported (0..MAX_BALLS)
    for ball i (0-based), with base = 1 + 4*i:
        [base + 0]     = angle_x  (deg, + = target is to the RIGHT of center)
        [base + 1]     = angle_y  (deg, + = target is ABOVE center)
        [base + 2]     = radius   (px, apparent inscribed radius — distance hint only)
        [base + 3]     = color    (1 = PURPLE, 2 = GREEN), matches Artifact enum order
Balls are sorted largest-radius (closest) first, so if more than MAX_BALLS are
visible the robot still gets the most reliable ones.
----------------------------------------------------------------------------------

NOTE: This file is stored in the repo for reference/versioning only. It never runs
on the Control Hub — it is uploaded to the Limelight. The `# noinspection` comment
on line 1 tells Android Studio / PyCharm not to flag the cv2 / numpy imports
(there is no Python interpreter configured in this Android project).
"""

import cv2
import numpy as np

# ======================================================================================
# TUNABLES — edit these to match your lighting and camera. All are plain module globals
# so they are easy to find and change.
# ======================================================================================

# HSV color gates (OpenCV HSV: H 0-179, S 0-255, V 0-255).
GREEN_LOW = (60, 70, 70)
GREEN_HIGH = (85, 255, 255)
PURPLE_LOW = (115, 50, 40)
PURPLE_HIGH = (160, 255, 255)

# Camera field of view in degrees. LL3A defaults are ~54.5 H / ~42 V at 4:3; retune if
# you change the Limelight resolution/crop. These drive the pixel -> angle conversion.
H_FOV_DEG = 54.5
V_FOV_DEG = 42.0

MORPH_KERNEL = 5          # px, structuring-element size for open/close cleanup
MIN_BALL_RADIUS_PX = 6.0  # ignore distance-transform peaks smaller than this
MIN_PEAK_SEPARATION = 12  # px, merge peaks closer than this (same ball detected twice)
PEAK_NEIGHBORHOOD = 9     # px, window used to find local maxima of the distance transform
MAX_BALLS = 7             # 1 + 4*7 = 29 <= 32 doubles available in llpython

PURPLE_FLAG = 1.0
GREEN_FLAG = 2.0

# ======================================================================================
# Reusable kernel (built once, not per frame).
# ======================================================================================
_MORPH = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (MORPH_KERNEL, MORPH_KERNEL))


def _clean_mask(mask):
    """Open (remove speckle) then close (fill pinholes) a binary color mask."""
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _MORPH)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _MORPH)
    return mask


def _find_centers(mask):
    """
    Separate overlapping balls of one color and return their centers.

    Overlapping same-color balls merge into one blob, so a plain contour gives one
    fused shape. Instead we take the distance transform of the mask: each ball center
    is a local maximum whose value is the distance to the nearest edge (~the ball's
    radius). Local maxima cleanly separate touching convex blobs, which is exactly the
    overlapping-circles case, and it degrades gracefully when a ball is only a partial
    arc.

    Returns a list of (cx, cy, radius_px).
    """
    dist = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
    if dist.max() <= 0:
        return []

    # A pixel is a local maximum if it equals the max over its neighborhood (and the
    # mask is set there). Dilation gives the per-pixel neighborhood max in one shot.
    nbhd = cv2.getStructuringElement(cv2.MORPH_RECT, (PEAK_NEIGHBORHOOD, PEAK_NEIGHBORHOOD))
    local_max = cv2.dilate(dist, nbhd)
    peaks = (dist >= local_max) & (dist >= MIN_BALL_RADIUS_PX)

    ys, xs = np.where(peaks)
    cand = sorted(
        (((float(x), float(y)), float(dist[y, x])) for x, y in zip(xs, ys)),
        key=lambda c: c[1],
        reverse=True,
    )

    # Non-maximum suppression: keep the strongest peak, drop anything too close to one
    # we already kept (the same ball can light up several adjacent pixels).
    kept = []
    for (cx, cy), r in cand:
        if all((cx - kx) ** 2 + (cy - ky) ** 2 >= MIN_PEAK_SEPARATION ** 2 for kx, ky, _ in kept):
            kept.append((cx, cy, r))
    return kept


def _pixel_to_angles(cx, cy, width, height):
    """
    Convert a pixel location to camera angles using a pinhole model.

    angle_x: + when the target is to the right of the image center.
    angle_y: + when the target is above the image center (note image y grows downward).
    """
    fx = (width * 0.5) / np.tan(np.radians(H_FOV_DEG) * 0.5)
    fy = (height * 0.5) / np.tan(np.radians(V_FOV_DEG) * 0.5)
    angle_x = np.degrees(np.arctan((cx - width * 0.5) / fx))
    angle_y = np.degrees(np.arctan((height * 0.5 - cy) / fy))
    return angle_x, angle_y


def _draw(image, balls, width, height):
    """Annotate the streamed image so the detections can be eyeballed in the dashboard."""
    cv2.putText(image, "DECODE ball detector", (4, height - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    for cx, cy, r, flag in balls:
        bgr = (255, 0, 255) if flag == PURPLE_FLAG else (0, 255, 0)
        cv2.circle(image, (int(cx), int(cy)), int(max(r, 3)), bgr, 2)
        cv2.drawMarker(image, (int(cx), int(cy)), bgr, cv2.MARKER_CROSS, 8, 1)


# runPipeline() is called every frame by Limelight's backend.
def runPipeline(image, llrobot):
    height, width = image.shape[:2]
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    purple_mask = _clean_mask(cv2.inRange(img_hsv, PURPLE_LOW, PURPLE_HIGH))
    green_mask = _clean_mask(cv2.inRange(img_hsv, GREEN_LOW, GREEN_HIGH))

    # Collect (cx, cy, radius_px, color_flag) for every ball of every color.
    balls = []
    for cx, cy, r in _find_centers(purple_mask):
        balls.append((cx, cy, r, PURPLE_FLAG))
    for cx, cy, r in _find_centers(green_mask):
        balls.append((cx, cy, r, GREEN_FLAG))

    # Closest (largest apparent radius) first, then cap to what fits in llpython.
    balls.sort(key=lambda b: b[2], reverse=True)
    balls = balls[:MAX_BALLS]

    llpython = [0.0] * 32
    llpython[0] = float(len(balls))
    for i, (cx, cy, r, flag) in enumerate(balls):
        angle_x, angle_y = _pixel_to_angles(cx, cy, width, height)
        base = 1 + 4 * i
        llpython[base + 0] = float(angle_x)
        llpython[base + 1] = float(angle_y)
        llpython[base + 2] = float(r)
        llpython[base + 3] = flag

    _draw(image, balls, width, height)

    # Limelight wants a contour back for its "largest target" overlay. Hand it the
    # combined mask's largest contour, or an empty array if nothing was found.
    combined = cv2.bitwise_or(purple_mask, green_mask)
    contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest_contour = max(contours, key=cv2.contourArea) if contours else np.array([[]])

    return largest_contour, image, llpython
