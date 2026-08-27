"""OpenCV helpers that are deterministic and independent of camera hardware."""

from math import sqrt

import cv2
import cv2.aruco as aruco
import numpy as np


class ArucoDetectorCompat:
    """Expose one detector API across OpenCV 4.6 and newer releases."""

    def __init__(self, dictionary_id=aruco.DICT_6X6_250):
        self.dictionary = aruco.getPredefinedDictionary(dictionary_id)
        if hasattr(aruco, "ArucoDetector"):
            parameters = aruco.DetectorParameters()
            self._detector = aruco.ArucoDetector(self.dictionary, parameters)
            self._parameters = None
        else:
            self._detector = None
            self._parameters = aruco.DetectorParameters_create()

    def detect_markers(self, gray_image):
        if self._detector is not None:
            return self._detector.detectMarkers(gray_image)
        return aruco.detectMarkers(
            gray_image,
            self.dictionary,
            parameters=self._parameters,
        )


def detect_color_regions(
    bgr_image,
    lower_hsv=(35, 43, 46),
    upper_hsv=(77, 255, 255),
    minimum_area=500.0,
):
    """Return an annotated copy, binary mask, and accepted contour metadata."""
    annotated = bgr_image.copy()
    blurred = cv2.GaussianBlur(bgr_image, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.asarray(lower_hsv, dtype=np.uint8),
        np.asarray(upper_hsv, dtype=np.uint8),
    )
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detections = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < minimum_area:
            continue
        rectangle = cv2.minAreaRect(contour)
        box = np.rint(cv2.boxPoints(rectangle)).astype(np.intp)
        center = (int(rectangle[0][0]), int(rectangle[0][1]))
        detections.append({"area": area, "center": center, "box": box})
        cv2.drawContours(annotated, [box], 0, (255, 0, 0), 2)
        cv2.circle(annotated, center, 4, (0, 0, 255), -1)
        cv2.putText(
            annotated,
            f"Area: {int(area)}",
            (center[0] - 40, center[1] - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
        )
    return annotated, mask, detections


def rotation_matrix_to_quaternion(matrix) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to a normalized (x, y, z, w) quaternion."""
    matrix = np.asarray(matrix, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = sqrt(trace + 1.0) * 2.0
        quaternion = (
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
            0.25 * scale,
        )
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            quaternion = (
                0.25 * scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
            )
        elif index == 1:
            scale = sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            quaternion = (
                (matrix[0, 1] + matrix[1, 0]) / scale,
                0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
            )
        else:
            scale = sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            quaternion = (
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                0.25 * scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            )
    norm = sqrt(sum(value * value for value in quaternion))
    return tuple(value / norm for value in quaternion)


def detect_aruco_poses(
    bgr_image,
    detector,
    marker_size,
    camera_matrix,
    distortion_coefficients,
    axis_length=0.03,
):
    """Detect, estimate and annotate marker poses in a BGR image."""
    annotated = bgr_image.copy()
    gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detect_markers(gray)
    if ids is None:
        return annotated, []

    aruco.drawDetectedMarkers(annotated, corners, ids)
    rotation_vectors, translation_vectors, _ = aruco.estimatePoseSingleMarkers(
        corners,
        marker_size,
        camera_matrix,
        distortion_coefficients,
    )
    detections = []
    for index, marker in enumerate(ids):
        rotation_vector = rotation_vectors[index]
        translation_vector = translation_vectors[index]
        cv2.drawFrameAxes(
            annotated,
            camera_matrix,
            distortion_coefficients,
            rotation_vector,
            translation_vector,
            axis_length,
        )
        rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
        detections.append(
            {
                "id": int(marker[0]),
                "rotation_vector": rotation_vector,
                "translation_vector": translation_vector,
                "quaternion": rotation_matrix_to_quaternion(rotation_matrix),
            }
        )
    return annotated, detections
