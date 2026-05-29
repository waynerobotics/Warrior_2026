from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


@dataclass
class OpenCVEquirectLaneAdapter:
    """Classical lane-marking extractor for equirectangular frames.

    This adapter is intentionally lightweight and dependency-free beyond OpenCV.
    It detects bright/low-saturation white markings and yellow markings inside
    the lower equirectangular image band, then cleans the result with morphology.
    It is a real automated image-based backend, but it is not a trained CLRNet or
    LaneATT neural model.
    """

    checkpoint: str = ""
    device: str = "cpu"
    score_threshold: float = 0.5
    roi_top_fraction: float = 0.45
    min_component_area: int = 80
    max_component_fraction: float = 0.08
    morph_kernel: int = 5
    white_value_min: int = 150
    white_saturation_max: int = 150
    lab_lightness_min: int = 145
    lab_chroma_max: int = 42
    yellow_hue_min: int = 12
    yellow_hue_max: int = 45
    yellow_saturation_min: int = 55
    yellow_value_min: int = 90
    canny_low: int = 40
    canny_high: int = 140
    use_edges: bool = True
    use_tophat: bool = True
    tophat_kernel: int = 31
    tophat_threshold: int = 18
    use_hough: bool = True
    hough_threshold: int = 35
    min_line_length: int = 40
    max_line_gap: int = 20
    hough_draw_thickness: int = 7
    hough_min_retained_fraction: float = 0.02

    def __init__(self, checkpoint: str = "", device: str = "cpu", score_threshold: float = 0.5, **kwargs: Any) -> None:
        self.checkpoint = checkpoint
        self.device = device
        self.score_threshold = score_threshold
        for key, value in kwargs.items():
            if not hasattr(self, key):
                raise ValueError(f"Unknown OpenCV lane adapter option: {key}")
            setattr(self, key, value)

    def predict(self, image_bgr: np.ndarray) -> dict[str, Any]:
        if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
            raise ValueError("Expected BGR image with shape HxWx3")

        h, w = image_bgr.shape[:2]
        roi = np.zeros((h, w), dtype=np.uint8)
        roi_top = int(max(0.0, min(1.0, float(self.roi_top_fraction))) * h)
        roi[roi_top:, :] = 255

        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        hue, sat, val = cv2.split(hsv)
        lab = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2LAB)
        lightness, lab_a, lab_b = cv2.split(lab)

        white = ((val >= int(self.white_value_min)) & (sat <= int(self.white_saturation_max))).astype(np.uint8) * 255
        neutral_chroma = np.sqrt(
            (lab_a.astype(np.float32) - 128.0) ** 2 + (lab_b.astype(np.float32) - 128.0) ** 2
        )
        lab_white = (
            (lightness >= int(self.lab_lightness_min))
            & (neutral_chroma <= float(self.lab_chroma_max))
        ).astype(np.uint8) * 255
        yellow = (
            (hue >= int(self.yellow_hue_min))
            & (hue <= int(self.yellow_hue_max))
            & (sat >= int(self.yellow_saturation_min))
            & (val >= int(self.yellow_value_min))
        ).astype(np.uint8) * 255

        mask = cv2.bitwise_or(cv2.bitwise_or(white, lab_white), yellow)
        if self.use_tophat:
            gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
            kernel_size = max(3, int(self.tophat_kernel))
            if kernel_size % 2 == 0:
                kernel_size += 1
            line_kernel_h = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, 3))
            line_kernel_v = cv2.getStructuringElement(cv2.MORPH_RECT, (3, kernel_size))
            tophat = cv2.max(
                cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, line_kernel_h),
                cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, line_kernel_v),
            )
            _, tophat_mask = cv2.threshold(tophat, int(self.tophat_threshold), 255, cv2.THRESH_BINARY)
            mask = cv2.bitwise_or(mask, tophat_mask)

        if self.use_edges:
            gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, int(self.canny_low), int(self.canny_high))
            mask = cv2.bitwise_or(mask, cv2.bitwise_and(mask, cv2.dilate(edges, np.ones((3, 3), np.uint8))))

        mask = cv2.bitwise_and(mask, roi)
        if self.use_hough:
            mask = self._line_mask(mask)
        kernel_size = max(1, int(self.morph_kernel))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = self._filter_components(mask)

        confidence = self._confidence(mask, h, w)
        return {"mask": mask, "confidence": confidence}

    def _line_mask(self, candidate_mask: np.ndarray) -> np.ndarray:
        lines = cv2.HoughLinesP(
            candidate_mask,
            rho=1,
            theta=np.pi / 180.0,
            threshold=int(self.hough_threshold),
            minLineLength=int(self.min_line_length),
            maxLineGap=int(self.max_line_gap),
        )
        output = np.zeros_like(candidate_mask)
        if lines is None:
            return candidate_mask
        thickness = max(1, int(self.hough_draw_thickness))
        for line in lines[:, 0, :]:
            x1, y1, x2, y2 = [int(v) for v in line]
            length = float(np.hypot(x2 - x1, y2 - y1))
            if length < float(self.min_line_length):
                continue
            cv2.line(output, (x1, y1), (x2, y2), 255, thickness=thickness)
        support = cv2.dilate(candidate_mask, np.ones((max(3, thickness * 2), max(3, thickness * 2)), np.uint8))
        output = cv2.bitwise_and(output, support)
        candidate_pixels = int((candidate_mask > 0).sum())
        output_pixels = int((output > 0).sum())
        if candidate_pixels > 0 and output_pixels < candidate_pixels * float(self.hough_min_retained_fraction):
            return candidate_mask
        return output

    def _filter_components(self, mask: np.ndarray) -> np.ndarray:
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats((mask > 0).astype(np.uint8), connectivity=8)
        if num_labels <= 1:
            return np.zeros_like(mask)
        max_area = int(mask.size * float(self.max_component_fraction))
        output = np.zeros_like(mask)
        for label_idx in range(1, num_labels):
            area = int(stats[label_idx, cv2.CC_STAT_AREA])
            if area < int(self.min_component_area):
                continue
            if max_area > 0 and area > max_area:
                continue
            output[labels == label_idx] = 255
        return output

    @staticmethod
    def _confidence(mask: np.ndarray, h: int, w: int) -> float:
        lane_pixels = int((mask > 0).sum())
        if lane_pixels == 0:
            return 0.0
        lane_fraction = lane_pixels / float(h * w)
        num_labels, _, stats, _ = cv2.connectedComponentsWithStats((mask > 0).astype(np.uint8), connectivity=8)
        components = max(0, num_labels - 1)
        largest = int(stats[1:, cv2.CC_STAT_AREA].max()) if components else 0
        component_score = min(1.0, components / 8.0)
        coverage_score = min(1.0, lane_fraction / 0.025)
        largest_score = min(1.0, largest / max(1.0, 0.004 * h * w))
        return float(max(0.05, min(0.99, 0.25 + 0.35 * coverage_score + 0.25 * component_score + 0.15 * largest_score)))


class CLRNetEquirectAdapter(OpenCVEquirectLaneAdapter):
    """Compatibility alias for the existing clrnet backend slot.

    Use this local class when no external CLRNet package is available yet.
    """


class LaneATTEquirectAdapter(OpenCVEquirectLaneAdapter):
    """Compatibility alias for the existing laneatt backend slot.

    Use this local class when no external LaneATT package is available yet.
    """
