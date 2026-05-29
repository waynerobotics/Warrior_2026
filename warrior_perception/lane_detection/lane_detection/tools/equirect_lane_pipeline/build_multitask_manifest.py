from __future__ import annotations

import argparse
import json
import random
from pathlib import Path
from typing import Any


def read_jsonl(path: str | Path) -> list[dict[str, Any]]:
    rows = []
    with Path(path).open("r", encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                rows.append(json.loads(line))
    return rows


def load_detection_annotations(path: str | Path) -> dict[str, dict[str, Any]]:
    if not path:
        return {}
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(p)
    if p.suffix.lower() == ".jsonl":
        rows = read_jsonl(p)
    else:
        data = json.loads(p.read_text(encoding="utf-8"))
        rows = data.get("samples", data) if isinstance(data, dict) else data
    output = {}
    for row in rows:
        image = str(row.get("image") or row.get("image_path") or "")
        frame_id = str(row.get("frame_id") or Path(image).stem)
        output[frame_id] = {"boxes": row.get("boxes", []), "labels": row.get("labels", [])}
    return output


def build(args: argparse.Namespace) -> None:
    images_dir = Path(args.images_dir)
    image_paths = sorted([p for p in images_dir.iterdir() if p.suffix.lower() in {".jpg", ".jpeg", ".png"}])
    if not image_paths:
        raise RuntimeError(f"No images found in {images_dir}")

    detections = load_detection_annotations(args.detection_annotations)
    lane_rows = read_jsonl(args.lane_records) if args.lane_records else []
    lanes = {str(row.get("frame_id") or Path(row.get("image_path", "")).stem): row for row in lane_rows}
    image_ids = {p.stem for p in image_paths}
    unknown_detection_ids = sorted(set(detections) - image_ids)
    unknown_lane_ids = sorted(set(lanes) - image_ids)
    if unknown_detection_ids:
        raise RuntimeError(f"Detection annotations include {len(unknown_detection_ids)} ids not found in images_dir, first={unknown_detection_ids[:5]}")
    if unknown_lane_ids:
        raise RuntimeError(f"Lane records include {len(unknown_lane_ids)} ids not found in images_dir, first={unknown_lane_ids[:5]}")

    samples = []
    missing_seg = []
    for image_path in image_paths:
        frame_id = image_path.stem
        det = detections.get(frame_id, {"boxes": [], "labels": []})
        lane = lanes.get(frame_id)
        if args.require_segmentation and lane is None:
            missing_seg.append(frame_id)
            continue
        sample = {
            "image": str(image_path),
            "boxes": det.get("boxes", []),
            "labels": det.get("labels", []),
        }
        if lane is not None:
            sample["seg_mask"] = lane["seg_mask_path"]
            sample["seg_confidence"] = float(lane.get("seg_confidence", 1.0))
            sample["label_source"] = lane.get("backend", "unknown")
        if args.shared_roi_mask:
            sample["seg_roi_mask"] = args.shared_roi_mask
        samples.append(sample)

    if not samples:
        raise RuntimeError("No samples remained after manifest merge")
    rng = random.Random(args.seed)
    rng.shuffle(samples)
    val_count = max(1, int(round(len(samples) * args.val_ratio))) if len(samples) > 1 else 0
    val_samples = samples[:val_count]
    train_samples = samples[val_count:]

    Path(args.train_manifest).parent.mkdir(parents=True, exist_ok=True)
    Path(args.val_manifest).parent.mkdir(parents=True, exist_ok=True)
    Path(args.train_manifest).write_text(json.dumps(train_samples, indent=2), encoding="utf-8")
    Path(args.val_manifest).write_text(json.dumps(val_samples, indent=2), encoding="utf-8")
    skipped = f" skipped_missing_seg={len(missing_seg)}" if args.require_segmentation else ""
    print(f"train={len(train_samples)} val={len(val_samples)}{skipped}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build multitask manifests from detections and lane pseudo-labels.")
    parser.add_argument("--images_dir", required=True)
    parser.add_argument("--detection_annotations", required=True)
    parser.add_argument("--lane_records", required=True)
    parser.add_argument("--train_manifest", required=True)
    parser.add_argument("--val_manifest", required=True)
    parser.add_argument("--val_ratio", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--require_segmentation", action="store_true")
    parser.add_argument("--shared_roi_mask", default="")
    return parser.parse_args()


if __name__ == "__main__":
    build(parse_args())
