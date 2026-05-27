"""Overlay drawing helpers for live inference display."""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np


def draw_stats_block(frame_bgr: np.ndarray, stats: dict[str, Any]) -> np.ndarray:
    """Draw a compact semi-transparent stats block in the top-left corner."""

    lines = [
        f"FPS: {stats['current_fps']:.1f} | Avg: {stats['rolling_fps']:.1f}",
        f"Cap: {stats['capture_ms']:.1f} ms | Pre: {stats['preprocess_ms']:.1f} ms",
        f"Inf: {stats['inference_ms']:.1f} ms | Post: {stats['postprocess_ms']:.1f} ms",
        f"Total: {stats['total_ms']:.1f} ms",
        f"Frame: {stats['frame_idx']} | Res: {stats['resolution']}",
        f"Device: {stats['device']} | Saved: {stats['saved_count']}",
    ]

    overlay = frame_bgr.copy()
    x0, y0 = 12, 12
    line_height = 22
    block_height = 14 + line_height * len(lines)
    block_width = 420
    cv2.rectangle(overlay, (x0, y0), (x0 + block_width, y0 + block_height), (20, 20, 20), thickness=-1)
    blended = cv2.addWeighted(overlay, 0.55, frame_bgr, 0.45, 0)

    for idx, line in enumerate(lines):
        y = y0 + 24 + idx * line_height
        cv2.putText(blended, line, (x0 + 10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (240, 240, 240), 1, cv2.LINE_AA)

    return blended
