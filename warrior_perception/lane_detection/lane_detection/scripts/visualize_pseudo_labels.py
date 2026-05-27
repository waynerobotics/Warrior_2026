"""
Visualize pseudo-label results from pseudo_label.py output.

Reads the pseudo_train_manifest.json (or any manifest) and for each pseudo-
labelled sample draws detection boxes + segmentation mask overlay, then saves
annotated JPGs to --output_dir.

Usage:
  python scripts/visualize_pseudo_labels.py
  python scripts/visualize_pseudo_labels.py --limit 50 --output_dir outputs/pseudo_viz
  python scripts/visualize_pseudo_labels.py --manifest data/pseudo_train_manifest.json --limit 100
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

# ── Constants ─────────────────────────────────────────────────────────────────

CLASS_NAMES = ["barrel", "pedestrian", "stop_sign", "unknown", "tire", "pothole"]

CLASS_COLORS_BGR: dict[int, tuple[int, int, int]] = {
    0: (50,  50,  220),   # barrel     — red
    1: (50,  220,  50),   # pedestrian — green
    2: (0,   200, 240),   # stop_sign  — yellow
    3: (180, 180, 180),   # unknown    — silver
    4: (0,   130, 240),   # tire       — orange
    5: (220,  0,  150),   # pothole    — purple
}

SEG_COLOR_BGR = (220, 180, 0)   # cyan lane overlay
SEG_ALPHA     = 0.40
SKY_ZONE_FRAC = 0.35            # suppress seg above this row fraction


# ── Drawing helpers ───────────────────────────────────────────────────────────

def _text_with_bg(
    canvas: np.ndarray,
    text: str,
    origin: tuple[int, int],
    font_scale: float,
    color_bgr: tuple[int, int, int],
    thickness: int = 1,
) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    x, y = origin
    cv2.rectangle(canvas, (x, y - th - baseline), (x + tw + 2, y + baseline), (20, 20, 20), -1)
    cv2.putText(canvas, text, (x + 1, y), font, font_scale, color_bgr, thickness, cv2.LINE_AA)


def annotate_sample(
    bgr: np.ndarray,
    boxes: list[list[float]],
    labels: list[int],
    seg_mask: np.ndarray | None,
    label_source: str,
    seg_confidence: float,
) -> np.ndarray:
    canvas = bgr.copy()
    H, W = canvas.shape[:2]
    font_scale   = max(0.35, W / 3200.0)
    box_thickness = max(1, int(W / 640))
    label_line_h  = int(font_scale * 20) + 6

    # Segmentation overlay
    if seg_mask is not None and seg_mask.any():
        lane_mask = seg_mask > 0
        sky_cutoff = int(H * SKY_ZONE_FRAC)
        lane_mask[:sky_cutoff, :] = False
        if lane_mask.any():
            overlay = canvas.copy()
            overlay[lane_mask] = SEG_COLOR_BGR
            canvas = cv2.addWeighted(canvas, 1.0 - SEG_ALPHA, overlay, SEG_ALPHA, 0)
            contours, _ = cv2.findContours(
                lane_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            cv2.drawContours(canvas, contours, -1, SEG_COLOR_BGR, max(1, box_thickness - 1))

    # Detection boxes — coordinates are stored as pixel values (xyxy)
    strip_w = max(100, W // 16)
    label_slots: dict[int, int] = {}
    num_det = 0
    for box, lbl in zip(boxes, labels):
        x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
        num_det += 1
        color = CLASS_COLORS_BGR.get(int(lbl), (200, 200, 200))
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, box_thickness)

        cls_name   = CLASS_NAMES[int(lbl)] if int(lbl) < len(CLASS_NAMES) else str(lbl)
        label_text = cls_name

        strip_idx = x1 // strip_w
        label_y   = max(label_line_h, label_slots.get(strip_idx, y1))
        if y1 > label_slots.get(strip_idx, 0):
            label_y = max(label_line_h, y1)
        label_slots[strip_idx] = label_y + label_line_h
        _text_with_bg(canvas, label_text, (x1, label_y), font_scale, color)

    # Header bar
    bar_h = max(36, int(H * 0.055))
    header = np.zeros((bar_h, W, 3), dtype=np.uint8)
    lane_pct = 0.0
    if seg_mask is not None:
        lane_pct = 100.0 * (seg_mask > 0).mean()
    src_tag  = f"[{label_source}]"
    conf_tag = f"seg_conf={seg_confidence:.2f}" if seg_confidence > 0 else "seg=none"
    header_text = f"{src_tag}  dets={num_det}  lane={lane_pct:.1f}%  {conf_tag}"
    hf = max(0.4, font_scale * 1.0)
    cv2.putText(header, header_text, (10, bar_h - 8),
                cv2.FONT_HERSHEY_SIMPLEX, hf, (200, 220, 255), 1, cv2.LINE_AA)

    return np.concatenate([header, canvas], axis=0)


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Visualize pseudo-labelled samples.")
    parser.add_argument("--manifest",   default="data/pseudo_train_manifest.json")
    parser.add_argument("--output_dir", default="outputs/pseudo_viz")
    parser.add_argument("--limit",      type=int, default=100,
                        help="Max pseudo samples to visualize (0 = all)")
    parser.add_argument("--pseudo_only", action="store_true", default=True,
                        help="Skip non-pseudo (manually labelled) samples")
    parser.add_argument("--image_size", nargs=2, type=int, default=[0, 0],
                        metavar=("H", "W"),
                        help="Resize images before drawing (0 0 = keep original)")
    args = parser.parse_args()

    manifest_path = ROOT / args.manifest
    if not manifest_path.exists():
        print(f"[ERROR] Manifest not found: {manifest_path}")
        sys.exit(1)

    with open(manifest_path, encoding="utf-8") as fh:
        all_samples: list[dict] = json.load(fh)

    # Filter to pseudo samples only
    samples = [s for s in all_samples if s.get("label_source") == "pseudo"] if args.pseudo_only else all_samples
    if args.limit > 0:
        samples = samples[: args.limit]

    print(f"Manifest: {manifest_path.name}  total={len(all_samples)}  "
          f"pseudo={len([s for s in all_samples if s.get('label_source')=='pseudo'])}  "
          f"visualizing={len(samples)}")

    out_dir = ROOT / args.output_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    saved = skipped = 0
    for sample in samples:
        img_path = Path(sample["image"])
        if not img_path.is_absolute():
            img_path = ROOT / img_path
        if not img_path.exists():
            print(f"  [SKIP] image not found: {img_path}")
            skipped += 1
            continue

        bgr = cv2.imread(str(img_path))
        if bgr is None:
            print(f"  [SKIP] cannot read: {img_path}")
            skipped += 1
            continue

        target_h, target_w = args.image_size
        if target_h > 0 and target_w > 0:
            bgr = cv2.resize(bgr, (target_w, target_h), interpolation=cv2.INTER_LINEAR)

        # Load seg mask if present
        seg_mask: np.ndarray | None = None
        mask_ref = sample.get("seg_mask")
        if mask_ref:
            mask_path = Path(mask_ref)
            if not mask_path.is_absolute():
                mask_path = ROOT / mask_path
            if mask_path.exists():
                seg_gray = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
                if seg_gray is not None:
                    H, W = bgr.shape[:2]
                    if seg_gray.shape != (H, W):
                        seg_gray = cv2.resize(seg_gray, (W, H), interpolation=cv2.INTER_NEAREST)
                    seg_mask = seg_gray

        boxes  = sample.get("boxes", [])
        labels = sample.get("labels", [])
        label_source   = sample.get("label_source", "unknown")
        seg_confidence = float(sample.get("seg_confidence", 0.0))

        annotated = annotate_sample(bgr, boxes, labels, seg_mask, label_source, seg_confidence)

        out_path = out_dir / f"{img_path.stem}.jpg"
        cv2.imwrite(str(out_path), annotated, [cv2.IMWRITE_JPEG_QUALITY, 90])
        saved += 1

    print(f"\nSaved {saved} images -> {out_dir}")
    if skipped:
        print(f"Skipped {skipped} images (not found / unreadable)")
    print(f"\nOpen the folder to review:\n  {out_dir}")


if __name__ == "__main__":
    main()
