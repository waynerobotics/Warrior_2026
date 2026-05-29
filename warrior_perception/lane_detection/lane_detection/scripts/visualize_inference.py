"""
Visualize ONNX model inference on images with no ground truth.

Runs both resnet18 and convnext_base ONNX models and saves side-by-side
comparison images:  [ResNet-18 predictions] | [ConvNeXt-B predictions]

Each panel shows detection boxes (class + score) overlaid with the lane
segmentation mask.

Outputs (under --output_dir):
  comparison/<frame>.jpg    side-by-side model comparison
  resnet18/<frame>.jpg      ResNet-18 only
  convnext_base/<frame>.jpg ConvNeXt-B only
  summary.json              detection/seg stats per image

Usage:
  python scripts/visualize_inference.py
  python scripts/visualize_inference.py --num_samples 12
  python scripts/visualize_inference.py --images data/frame_01069.jpg data/frame_01070.jpg
  python scripts/visualize_inference.py --score_threshold 0.15 --output_dir outputs/viz
"""
from __future__ import annotations

import argparse
import json
import random
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

try:
    import onnxruntime as ort
except ImportError:
    print("[ERROR] onnxruntime not installed. Run: pip install onnxruntime")
    sys.exit(1)

from models.detection_postprocess import decode_detections

# ── Constants ────────────────────────────────────────────────────────────────

IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32)

CLASS_NAMES = ["barrel", "pedestrian", "stop_sign", "unknown", "tire", "pothole"]

# Per-class BGR colors for detection boxes
CLASS_COLORS_BGR: dict[int, tuple[int, int, int]] = {
    0: (50,  50,  220),   # barrel      — red
    1: (50,  220,  50),   # pedestrian  — green
    2: (0,   200, 240),   # stop_sign   — yellow
    3: (180, 180, 180),   # unknown     — silver
    4: (0,   130, 240),   # tire        — orange
    5: (220,  0,  150),   # pothole     — purple
}

SEG_COLOR_BGR = (220, 180, 0)   # cyan-ish lane overlay
SEG_ALPHA     = 0.40

# Fraction of image height below which detections are suppressed (robot body zone)
ROBOT_BODY_FRAC = 0.85
# Fraction of image height above which segmentation is suppressed (sky zone)
SKY_ZONE_FRAC   = 0.35


# ── ORT Session ──────────────────────────────────────────────────────────────

def load_session(onnx_path: Path) -> ort.InferenceSession:
    opts = ort.SessionOptions()
    opts.log_severity_level = 3
    return ort.InferenceSession(str(onnx_path), opts, providers=["CPUExecutionProvider"])


# ── Preprocessing ─────────────────────────────────────────────────────────────

def preprocess(image_path: Path, target_hw: tuple[int, int]) -> tuple[np.ndarray, np.ndarray]:
    """Load → resize → normalise. Returns (input_chw_float32, display_bgr_uint8)."""
    bgr = cv2.imread(str(image_path))
    if bgr is None:
        raise FileNotFoundError(f"Cannot read image: {image_path}")
    h, w = target_hw
    bgr_resized = cv2.resize(bgr, (w, h), interpolation=cv2.INTER_LINEAR)
    rgb = bgr_resized[..., ::-1].astype(np.float32) / 255.0
    rgb = (rgb - IMAGENET_MEAN) / IMAGENET_STD
    inp = rgb.transpose(2, 0, 1)[np.newaxis]   # (1, 3, H, W)
    return inp.astype(np.float32), bgr_resized


# ── Inference ─────────────────────────────────────────────────────────────────

def run_onnx(
    sess: ort.InferenceSession,
    inp: np.ndarray,
    image_hw: tuple[int, int],
    score_threshold: float,
    nms_threshold: float,
) -> dict[str, Any]:
    """Run ORT session, decode detections (via PyTorch NMS), return results."""
    det_p3, det_p4, det_p5, seg_logits = sess.run(None, {"input": inp})

    # Decode detections using existing PyTorch postprocessor
    level_outputs = {
        "p3": torch.from_numpy(det_p3),
        "p4": torch.from_numpy(det_p4),
        "p5": torch.from_numpy(det_p5),
    }
    dets = decode_detections(
        level_outputs=level_outputs,
        image_size=image_hw,
        score_threshold=score_threshold,
        nms_threshold=nms_threshold,
        max_detections=100,
    )[0]   # batch index 0

    # Segmentation: argmax over class dim → (H, W) uint8
    seg_mask = np.argmax(seg_logits[0], axis=0).astype(np.uint8)   # (H, W)

    return {
        "boxes":    dets["boxes"].numpy(),    # (N, 4) xyxy
        "scores":   dets["scores"].numpy(),   # (N,)
        "labels":   dets["labels"].numpy(),   # (N,) int
        "seg_mask": seg_mask,
    }


# ── Drawing ───────────────────────────────────────────────────────────────────

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
    # background rect
    cv2.rectangle(canvas, (x, y - th - baseline), (x + tw + 2, y + baseline), (20, 20, 20), -1)
    cv2.putText(canvas, text, (x + 1, y), font, font_scale, color_bgr, thickness, cv2.LINE_AA)


def annotate(
    bgr: np.ndarray,
    result: dict[str, Any],
    model_label: str,
    score_threshold: float,
) -> np.ndarray:
    """Draw seg mask + detection boxes on a copy of bgr."""
    canvas = bgr.copy()
    H, W = canvas.shape[:2]
    font_scale = max(0.35, W / 3200.0)
    box_thickness = max(1, int(W / 640))
    label_line_h = int(font_scale * 20) + 6  # vertical step for stacked labels

    # Segmentation overlay — clipped to rows below sky zone
    sky_cutoff = int(H * SKY_ZONE_FRAC)
    raw_lane_mask = result["seg_mask"] > 0
    lane_mask = raw_lane_mask.copy()
    lane_mask[:sky_cutoff, :] = False          # suppress sky pixels
    if lane_mask.any():
        overlay = canvas.copy()
        overlay[lane_mask] = SEG_COLOR_BGR
        canvas = cv2.addWeighted(canvas, 1.0 - SEG_ALPHA, overlay, SEG_ALPHA, 0)
        contours, _ = cv2.findContours(
            lane_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        cv2.drawContours(canvas, contours, -1, SEG_COLOR_BGR, max(1, box_thickness - 1))

    # Detection boxes — suppressed in robot body zone (bottom fraction)
    robot_cutoff = int(H * ROBOT_BODY_FRAC)
    boxes  = result["boxes"]
    scores = result["scores"]
    labels = result["labels"]

    # label_slots: maps x-strip → next available y pixel for label text
    # Divide image into strips of ~100px to allow independent columns to stack freely
    strip_w = max(100, W // 16)
    label_slots: dict[int, int] = {}

    num_det = 0
    for box, score, label in zip(boxes, scores, labels):
        if score < score_threshold:
            continue
        x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
        cy = (y1 + y2) // 2
        if cy >= robot_cutoff:          # box centre inside robot body zone — skip
            continue
        num_det += 1
        color = CLASS_COLORS_BGR.get(int(label), (200, 200, 200))
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, box_thickness)

        cls_name = CLASS_NAMES[int(label)] if int(label) < len(CLASS_NAMES) else str(label)
        label_text = f"{cls_name} {score:.2f}"

        # Collision-aware label placement: find the next free y in this x-strip
        strip_idx = x1 // strip_w
        min_y = label_line_h
        label_y = max(min_y, label_slots.get(strip_idx, y1))
        # If the natural box position is below our current slot, use box position
        if y1 > label_slots.get(strip_idx, 0):
            label_y = max(min_y, y1)
        label_slots[strip_idx] = label_y + label_line_h

        _text_with_bg(canvas, label_text, (x1, label_y), font_scale, color)

    # Model header bar — taller for readability
    bar_h = max(36, int(H * 0.055))
    header = np.zeros((bar_h, W, 3), dtype=np.uint8)
    seg_pct = 100.0 * lane_mask.sum() / max(1, H * W)
    header_text = f"{model_label}   dets={num_det}   lane={seg_pct:.1f}%"
    header_font = max(0.45, font_scale * 1.1)
    cv2.putText(header, header_text, (10, bar_h - 8),
                cv2.FONT_HERSHEY_SIMPLEX, header_font, (200, 220, 255), 1, cv2.LINE_AA)

    return np.concatenate([header, canvas], axis=0)


def make_comparison(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """Concatenate two annotated panels horizontally with a thin divider."""
    h = max(left.shape[0], right.shape[0])
    divider = np.full((h, 3, 3), 60, dtype=np.uint8)

    def pad(img: np.ndarray) -> np.ndarray:
        if img.shape[0] < h:
            pad_rows = np.zeros((h - img.shape[0], img.shape[1], 3), dtype=np.uint8)
            return np.concatenate([img, pad_rows], axis=0)
        return img

    return np.concatenate([pad(left), divider, pad(right)], axis=1)


# ── Main ──────────────────────────────────────────────────────────────────────

def collect_images(args: argparse.Namespace) -> list[Path]:
    if args.images:
        return [Path(p) for p in args.images]

    # Pull from manifest if given, else glob data/
    candidates: list[Path] = []
    if args.manifest:
        with open(args.manifest, encoding="utf-8") as f:
            rows = json.load(f)
        rows = rows.get("samples", rows) if isinstance(rows, dict) else rows
        for row in rows:
            p = Path(row.get("image", ""))
            if not p.is_absolute():
                p = ROOT / p
            if p.exists():
                candidates.append(p)
    else:
        candidates = sorted((ROOT / "data").glob("*.jpg"))

    if not candidates:
        raise RuntimeError("No images found. Pass --images or --manifest.")

    rng = random.Random(args.seed)
    if args.num_samples > 0 and len(candidates) > args.num_samples:
        candidates = sorted(rng.sample(candidates, args.num_samples))
    return candidates


def main() -> None:
    parser = argparse.ArgumentParser(description="Visualize ONNX inference (no ground truth).")
    parser.add_argument("--onnx_dir",   default="outputs/onnx",
                        help="Directory containing resnet18.onnx and convnext_base.onnx")
    parser.add_argument("--output_dir", default="outputs/visualizations")
    parser.add_argument("--manifest",   default="data/val_manifest.json",
                        help="JSON manifest to sample images from (ignored if --images given)")
    parser.add_argument("--images",     nargs="*", default=[],
                        help="Explicit image paths (overrides --manifest)")
    parser.add_argument("--num_samples",  type=int, default=12)
    parser.add_argument("--score_threshold", type=float, default=0.15,
                        help="Detection score threshold (lower = more boxes)")
    parser.add_argument("--nms_threshold",   type=float, default=0.5)
    parser.add_argument("--image_size",   nargs=2, type=int, default=[640, 1280],
                        metavar=("H", "W"))
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    onnx_dir   = ROOT / args.onnx_dir
    output_dir = ROOT / args.output_dir
    image_hw   = tuple(args.image_size)   # (H, W)

    # Load ONNX sessions
    models: dict[str, dict[str, Any]] = {
        "resnet18":     {"onnx": onnx_dir / "resnet18.onnx",     "label": "ResNet-18  (epoch 21)"},
        "convnext_base":{"onnx": onnx_dir / "convnext_base.onnx","label": "ConvNeXt-B (epoch  5)"},
    }
    sessions: dict[str, ort.InferenceSession] = {}
    for name, meta in models.items():
        path = meta["onnx"]
        if not path.exists():
            print(f"[WARN] {path} not found — skipping {name}")
            continue
        print(f"  Loading {name} from {path.name}...")
        sessions[name] = load_session(path)

    if not sessions:
        print("[ERROR] No ONNX models found.")
        sys.exit(1)

    # Output dirs
    (output_dir / "comparison").mkdir(parents=True, exist_ok=True)
    for name in sessions:
        (output_dir / name).mkdir(parents=True, exist_ok=True)

    # Collect images
    images = collect_images(args)
    print(f"\nRunning inference on {len(images)} image(s)  "
          f"score_thresh={args.score_threshold}  image_size={image_hw}")

    summary: list[dict[str, Any]] = []
    for img_path in images:
        try:
            inp, bgr_display = preprocess(img_path, image_hw)
        except Exception as exc:
            print(f"  [SKIP] {img_path.name}: {exc}")
            continue

        frame_id = img_path.stem
        panels: dict[str, np.ndarray] = {}
        record: dict[str, Any] = {"frame": frame_id, "models": {}}

        for name, sess in sessions.items():
            try:
                result = run_onnx(sess, inp, image_hw, args.score_threshold, args.nms_threshold)
            except Exception as exc:
                print(f"  [ERROR] {name} on {frame_id}: {exc}")
                continue

            panel = annotate(bgr_display, result, models[name]["label"], args.score_threshold)
            panels[name] = panel

            out_path = output_dir / name / f"{frame_id}.jpg"
            cv2.imwrite(str(out_path), panel, [cv2.IMWRITE_JPEG_QUALITY, 92])

            record["models"][name] = {
                "num_detections": int((result["scores"] >= args.score_threshold).sum()),
                "lane_coverage_pct": round(100.0 * (result["seg_mask"] > 0).mean(), 2),
            }

        # Side-by-side comparison
        if "resnet18" in panels and "convnext_base" in panels:
            comp = make_comparison(panels["resnet18"], panels["convnext_base"])
            comp_path = output_dir / "comparison" / f"{frame_id}.jpg"
            cv2.imwrite(str(comp_path), comp, [cv2.IMWRITE_JPEG_QUALITY, 92])

        summary.append(record)
        n_models = len(record["models"])
        stats = "  ".join(
            f"{m}: dets={v['num_detections']} lane={v['lane_coverage_pct']}%"
            for m, v in record["models"].items()
        )
        print(f"  {frame_id}  [{n_models} models]  {stats}")

    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    print(f"\nDone. Outputs -> {output_dir}")
    print(f"  comparison/ : {len(summary)} side-by-side images")
    for name in sessions:
        print(f"  {name}/     : {len(summary)} individual images")


if __name__ == "__main__":
    main()
