from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

LOGGER = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")


def load_labelme(path: Path) -> dict[str, Any] | None:
    data = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(data, dict):
        return None
    if not isinstance(data.get("shapes"), list):
        return None
    if "imagePath" not in data:
        return None
    return data


def resolve_image_path(annotation_path: Path, image_ref: str, image_root: Path | None) -> Path:
    candidates = []
    image_path = Path(image_ref)
    if image_path.is_absolute():
        candidates.append(image_path)
    else:
        candidates.append(annotation_path.parent / image_path)
        if image_root is not None:
            candidates.append(image_root / image_path)
            candidates.append(image_root / image_path.name)
    candidates.append(annotation_path.with_suffix(".jpg"))
    candidates.append(annotation_path.with_suffix(".jpeg"))
    candidates.append(annotation_path.with_suffix(".png"))

    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError(f"Could not resolve imagePath '{image_ref}' for {annotation_path}")


def label_matches(actual: str, expected: str, case_sensitive: bool) -> bool:
    if case_sensitive:
        return actual == expected
    return actual.lower() == expected.lower()


def points_to_array(points: Any) -> np.ndarray | None:
    if not isinstance(points, list) or len(points) < 2:
        return None
    coords = []
    for point in points:
        if not isinstance(point, list) or len(point) < 2:
            return None
        coords.append([float(point[0]), float(point[1])])
    return np.asarray(coords, dtype=np.float32)


def draw_shape(mask: np.ndarray, shape: dict[str, Any]) -> None:
    points = points_to_array(shape.get("points"))
    if points is None:
        return

    shape_type = str(shape.get("shape_type") or "polygon").lower()
    if shape_type == "rectangle" and len(points) >= 2:
        x1, y1 = points[0]
        x2, y2 = points[1]
        polygon = np.asarray(
            [[x1, y1], [x2, y1], [x2, y2], [x1, y2]],
            dtype=np.float32,
        )
    elif shape_type == "polygon" and len(points) >= 3:
        polygon = points
    else:
        return

    h, w = mask.shape[:2]
    polygon[:, 0] = np.clip(polygon[:, 0], 0, w - 1)
    polygon[:, 1] = np.clip(polygon[:, 1], 0, h - 1)
    cv2.fillPoly(mask, [np.rint(polygon).astype(np.int32)], 255)


def convert_one(annotation_path: Path, data: dict[str, Any], args: argparse.Namespace, masks_dir: Path) -> dict[str, Any] | None:
    matching_shapes = [
        shape
        for shape in data["shapes"]
        if isinstance(shape, dict) and label_matches(str(shape.get("label", "")), args.label, bool(args.case_sensitive))
    ]
    if not matching_shapes and not args.keep_empty:
        return None

    image_root = Path(args.image_root) if args.image_root else None
    image_path = resolve_image_path(annotation_path, str(data["imagePath"]), image_root)
    image_h = int(data.get("imageHeight") or 0)
    image_w = int(data.get("imageWidth") or 0)
    if image_h <= 0 or image_w <= 0:
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise FileNotFoundError(image_path)
        image_h, image_w = image.shape[:2]

    mask = np.zeros((image_h, image_w), dtype=np.uint8)
    matched_shapes = 0
    for shape in matching_shapes:
        before = int((mask > 0).sum())
        draw_shape(mask, shape)
        matched_shapes += int((mask > 0).sum() > before)

    num_lane_pixels = int((mask == 255).sum())
    if num_lane_pixels == 0 and not args.keep_empty:
        return None

    frame_id = image_path.stem
    mask_path = masks_dir / f"{frame_id}.png"
    if not cv2.imwrite(str(mask_path), mask):
        raise RuntimeError(f"Failed to write mask: {mask_path}")

    return {
        "frame_id": frame_id,
        "image_path": str(image_path),
        "seg_mask_path": str(mask_path),
        "seg_confidence": 1.0,
        "backend": "manual",
        "num_lane_pixels": num_lane_pixels,
        "num_shapes": matched_shapes,
    }


def convert(args: argparse.Namespace) -> None:
    annotations_dir = Path(args.annotations_dir)
    output_dir = Path(args.output_dir)
    masks_dir = output_dir / "masks"
    masks_dir.mkdir(parents=True, exist_ok=True)

    annotation_paths = sorted(annotations_dir.rglob("*.json") if args.recursive else annotations_dir.glob("*.json"))
    records = []
    scanned = 0
    for annotation_path in annotation_paths:
        data = load_labelme(annotation_path)
        if data is None:
            continue
        scanned += 1
        record = convert_one(annotation_path, data, args, masks_dir)
        if record is not None:
            records.append(record)
        if args.limit and len(records) >= args.limit:
            break

    records_path = output_dir / "records.jsonl"
    with records_path.open("w", encoding="utf-8") as handle:
        for record in records:
            handle.write(json.dumps(record) + "\n")

    LOGGER.info(
        "records=%s masks=%s scanned_labelme=%d wrote_records=%d label=%s",
        records_path,
        masks_dir,
        scanned,
        len(records),
        args.label,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert LabelMe tape/lane polygons to binary segmentation masks.")
    parser.add_argument("--annotations_dir", default="data")
    parser.add_argument("--image_root", default="data")
    parser.add_argument("--output_dir", default="outputs/equirect_run/manual_tape_labels")
    parser.add_argument("--label", default="tape")
    parser.add_argument("--case_sensitive", action="store_true")
    parser.add_argument("--keep_empty", action="store_true")
    parser.add_argument("--recursive", action="store_true")
    parser.add_argument("--limit", type=int, default=0)
    return parser.parse_args()


if __name__ == "__main__":
    try:
        convert(parse_args())
    except Exception as exc:
        LOGGER.error("%s", exc)
        raise SystemExit(1) from None
