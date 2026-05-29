"""
Build train/val/test manifests and binary lane segmentation masks from raw
LabelMe JSON annotations in data/.

Class map (detection, 0-indexed foreground — background is implicit):
  0  barrel
  1  pedestrian       (BOM-prefixed variant ﻿pedestrian is normalised)
  2  stop_sign
  3  unknown          (soup_sign remapped here; existing unknown kept)
  4  tire
  5  pothole

Dropped silently: cone_large, cone_small

Segmentation: binary lane masks → data/seg_masks/<stem>.png
  0 = background,  255 = lane  (dataset binarises >0 on load)

Split (seeded): 80 % train / 10 % val / 10 % test of annotated images.
Unannotated images → data/unlabeled_manifest.json for pseudo-labelling.

Usage (from project root):
  python tools/prep/build_manifest.py
  python tools/prep/build_manifest.py --train_frac 0.8 --val_frac 0.1 --seed 42
"""
from __future__ import annotations

import argparse
import json
import random
import sys
from pathlib import Path

from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

# ---------------------------------------------------------------------------
# Class mapping
# ---------------------------------------------------------------------------

CLASS_MAP: dict[str, int] = {
    "barrel": 0,
    "pedestrian": 1,
    "stop_sign": 2,
    "unknown": 3,
    "tire": 4,
    "pothole": 5,
}

REMAP: dict[str, str] = {
    "soup_sign": "unknown",
}

DROP: set[str] = {"cone_large", "cone_small"}

SEG_LABEL = "lane"
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png"}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def normalise_label(raw: str) -> str:
    """Strip BOM, strip whitespace, apply REMAP table."""
    cleaned = raw.lstrip("﻿").strip()
    return REMAP.get(cleaned, cleaned)


def parse_json(json_path: Path) -> dict:
    """
    Returns dict with keys:
      image_w, image_h  (int | None from JSON header)
      det_boxes         list[[x1,y1,x2,y2]] in original pixel coords
      det_labels        list[int]
      lane_shapes       list[list[[x,y]]] — polygon vertices
    """
    with json_path.open("r", encoding="utf-8-sig") as fh:  # utf-8-sig strips file-level BOM
        ann = json.load(fh)

    image_w: int | None = ann.get("imageWidth")
    image_h: int | None = ann.get("imageHeight")
    det_boxes: list[list[float]] = []
    det_labels: list[int] = []
    lane_shapes: list[list[list[float]]] = []

    for shape in ann.get("shapes", []):
        label = normalise_label(shape.get("label", ""))
        stype = shape.get("shape_type", "")
        pts = shape.get("points", [])

        if stype == "rectangle":
            if label in DROP:
                continue
            class_id = CLASS_MAP.get(label)
            if class_id is None:
                continue
            if len(pts) != 2:
                continue
            x1, y1 = float(pts[0][0]), float(pts[0][1])
            x2, y2 = float(pts[1][0]), float(pts[1][1])
            if x1 > x2:
                x1, x2 = x2, x1
            if y1 > y2:
                y1, y2 = y2, y1
            det_boxes.append([x1, y1, x2, y2])
            det_labels.append(class_id)

        elif stype == "polygon" and label == SEG_LABEL:
            if len(pts) >= 3:
                lane_shapes.append([[float(p[0]), float(p[1])] for p in pts])

    return {
        "image_w": int(image_w) if image_w is not None else None,
        "image_h": int(image_h) if image_h is not None else None,
        "det_boxes": det_boxes,
        "det_labels": det_labels,
        "lane_shapes": lane_shapes,
    }


def image_dims(image_path: Path) -> tuple[int, int]:
    with Image.open(image_path) as img:
        return img.size  # (width, height)


def clip_boxes(
    boxes: list[list[float]],
    labels: list[int],
    w: float,
    h: float,
) -> tuple[list[list[float]], list[int]]:
    out_b, out_l = [], []
    for box, lbl in zip(boxes, labels):
        x1, y1, x2, y2 = box
        x1 = max(0.0, min(x1, w))
        y1 = max(0.0, min(y1, h))
        x2 = max(0.0, min(x2, w))
        y2 = max(0.0, min(y2, h))
        if x2 - x1 >= 2.0 and y2 - y1 >= 2.0:
            out_b.append([round(x1, 2), round(y1, 2), round(x2, 2), round(y2, 2)])
            out_l.append(lbl)
    return out_b, out_l


def rasterise_lanes(lane_shapes: list, w: int, h: int) -> Image.Image:
    mask = Image.new("L", (w, h), 0)
    draw = ImageDraw.Draw(mask)
    for pts in lane_shapes:
        poly = [(float(x), float(y)) for x, y in pts]
        draw.polygon(poly, fill=255)
    return mask


def relative_posix(path: Path, base: Path) -> str:
    return path.relative_to(base).as_posix()


# ---------------------------------------------------------------------------
# Main build
# ---------------------------------------------------------------------------

def build(
    data_dir: Path,
    masks_dir: Path,
    project_root: Path,
    train_frac: float,
    val_frac: float,
    seed: int,
) -> dict:
    random.seed(seed)
    masks_dir.mkdir(parents=True, exist_ok=True)

    all_images = sorted(
        p for p in data_dir.iterdir()
        if p.is_file() and p.suffix.lower() in IMAGE_EXTENSIONS
    )
    json_map = {p.stem: p for p in data_dir.glob("*.json")}

    annotated: list[dict] = []
    unlabeled_paths: list[Path] = []
    stats = {
        "total_images": len(all_images),
        "det_only": 0,
        "seg_only": 0,
        "det_and_seg": 0,
        "skipped_no_annotation": 0,
        "seg_masks_written": 0,
    }

    for img_path in all_images:
        stem = img_path.stem
        if stem not in json_map:
            unlabeled_paths.append(img_path)
            continue

        parsed = parse_json(json_map[stem])
        iw = parsed["image_w"]
        ih = parsed["image_h"]
        if iw is None or ih is None:
            iw, ih = image_dims(img_path)

        boxes, labels = clip_boxes(parsed["det_boxes"], parsed["det_labels"], iw, ih)
        lane_shapes = parsed["lane_shapes"]

        if not boxes and not lane_shapes:
            unlabeled_paths.append(img_path)
            stats["skipped_no_annotation"] += 1
            continue

        seg_mask_ref: str | None = None
        if lane_shapes:
            mask = rasterise_lanes(lane_shapes, iw, ih)
            mask_path = masks_dir / f"{stem}.png"
            mask.save(str(mask_path))
            seg_mask_ref = relative_posix(mask_path, project_root)
            stats["seg_masks_written"] += 1

        if boxes and lane_shapes:
            stats["det_and_seg"] += 1
        elif boxes:
            stats["det_only"] += 1
        else:
            stats["seg_only"] += 1

        sample: dict = {
            "image": relative_posix(img_path, project_root),
            "boxes": boxes,
            "labels": labels,
            "seg_confidence": 1.0,
            "label_source": "manual",
        }
        if seg_mask_ref:
            sample["seg_mask"] = seg_mask_ref

        annotated.append(sample)

    random.shuffle(annotated)
    n = len(annotated)
    n_train = int(n * train_frac)
    n_val = int(n * val_frac)
    train = annotated[:n_train]
    val = annotated[n_train : n_train + n_val]
    test = annotated[n_train + n_val :]

    unlabeled = [
        {
            "image": relative_posix(p, project_root),
            "boxes": [],
            "labels": [],
            "label_source": "unlabeled",
        }
        for p in unlabeled_paths
    ]

    stats.update(
        {
            "annotated": n,
            "unlabeled": len(unlabeled),
            "train": len(train),
            "val": len(val),
            "test": len(test),
        }
    )

    return {"train": train, "val": val, "test": test, "unlabeled": unlabeled, "stats": stats}


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="Build manifests from LabelMe annotations.")
    parser.add_argument("--data_dir", type=Path, default=ROOT / "data",
                        help="Directory containing frame_*.jpg and frame_*.json files")
    parser.add_argument("--output_dir", type=Path, default=ROOT / "data",
                        help="Where to write *_manifest.json and class_map.json")
    parser.add_argument("--masks_subdir", default="seg_masks",
                        help="Sub-directory inside data_dir for lane mask PNGs")
    parser.add_argument("--train_frac", type=float, default=0.8)
    parser.add_argument("--val_frac", type=float, default=0.1)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    data_dir: Path = args.data_dir.resolve()
    output_dir: Path = args.output_dir.resolve()
    masks_dir: Path = data_dir / args.masks_subdir
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Scanning {data_dir} ...")
    result = build(data_dir, masks_dir, ROOT, args.train_frac, args.val_frac, args.seed)

    splits = [("train", result["train"]), ("val", result["val"]),
              ("test", result["test"]), ("unlabeled", result["unlabeled"])]
    for name, samples in splits:
        out = output_dir / f"{name}_manifest.json"
        out.write_text(json.dumps(samples, indent=2), encoding="utf-8")
        print(f"  {out.name:<30s}  {len(samples):>5d} samples")

    class_map_path = output_dir / "class_map.json"
    class_map_path.write_text(json.dumps(CLASS_MAP, indent=2), encoding="utf-8")
    print(f"  {class_map_path.name}")

    print("\nStats:")
    for k, v in result["stats"].items():
        print(f"  {k}: {v}")


if __name__ == "__main__":
    main()
