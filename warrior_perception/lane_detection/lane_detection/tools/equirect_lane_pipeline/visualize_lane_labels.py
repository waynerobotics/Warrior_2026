from __future__ import annotations

import argparse
import json
import random
from pathlib import Path
from typing import Any

import cv2
import numpy as np


def read_jsonl(path: str | Path) -> list[dict[str, Any]]:
    rows = []
    with Path(path).open("r", encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                rows.append(json.loads(line))
    return rows


def load_rows(path: str | Path) -> list[dict[str, Any]]:
    p = Path(path)
    if p.suffix.lower() == ".jsonl":
        return read_jsonl(p)
    data = json.loads(p.read_text(encoding="utf-8"))
    rows = data.get("samples", data) if isinstance(data, dict) else data
    if not isinstance(rows, list):
        raise ValueError(f"Expected JSONL records or JSON sample list in {p}")
    return rows


def resolve_record_paths(row: dict[str, Any]) -> tuple[str, str, str]:
    image_path = str(row.get("image_path") or row.get("image") or "")
    mask_path = str(row.get("seg_mask_path") or row.get("seg_mask") or "")
    frame_id = str(row.get("frame_id") or Path(image_path).stem)
    if not image_path or not mask_path:
        raise ValueError(f"Record is missing image or segmentation mask path: {row}")
    return frame_id, image_path, mask_path


def load_mask(path: str | None, shape: tuple[int, int]) -> np.ndarray | None:
    if not path:
        return None
    mask = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if mask is None:
        raise FileNotFoundError(path)
    if mask.shape[:2] != shape:
        mask = cv2.resize(mask, (shape[1], shape[0]), interpolation=cv2.INTER_NEAREST)
    return mask


def make_preview(image: np.ndarray, lane_mask: np.ndarray, roi_mask: np.ndarray | None) -> np.ndarray:
    lane_overlay = image.copy()
    lane_overlay[lane_mask > 0] = np.array([0, 255, 0], dtype=np.uint8)
    if roi_mask is not None:
        roi_edges = cv2.Canny((roi_mask > 0).astype(np.uint8) * 255, 50, 150)
        lane_overlay[roi_edges > 0] = np.array([255, 0, 255], dtype=np.uint8)
    overlay = cv2.addWeighted(image, 0.68, lane_overlay, 0.32, 0)
    lane_bgr = cv2.cvtColor((lane_mask > 0).astype(np.uint8) * 255, cv2.COLOR_GRAY2BGR)
    panels = [image, lane_bgr, overlay]
    if roi_mask is not None:
        panels.insert(1, cv2.cvtColor((roi_mask > 0).astype(np.uint8) * 255, cv2.COLOR_GRAY2BGR))
    return np.concatenate(panels, axis=1)


def visualize(args: argparse.Namespace) -> None:
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = load_rows(args.records)
    rng = random.Random(args.seed)
    if args.num_samples > 0 and len(rows) > args.num_samples:
        rows = rng.sample(rows, args.num_samples)
    for row in rows:
        frame_id, image_path, mask_path = resolve_record_paths(row)
        image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
        if image is None or mask is None:
            continue
        if mask.shape[:2] != image.shape[:2]:
            mask = cv2.resize(mask, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST)
        roi_ref = args.roi_mask or row.get("seg_roi_mask")
        roi = load_mask(str(roi_ref), image.shape[:2]) if roi_ref else None
        preview = make_preview(image, mask, roi)
        cv2.imwrite(str(output_dir / f"{frame_id}.jpg"), preview)
    print(f"output_dir={output_dir}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize generated equirectangular lane labels.")
    parser.add_argument("--records", required=True)
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--num_samples", type=int, default=50)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--roi_mask", default="")
    return parser.parse_args()


if __name__ == "__main__":
    visualize(parse_args())
