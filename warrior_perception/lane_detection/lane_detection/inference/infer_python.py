#!/usr/bin/env python3
"""
Python TensorRT 10 inference for the multitask perception model.

Mirrors the C++ pipeline exactly:
  resize → ImageNet normalise → TRT engine → FCOS decode → NMS → seg argmax → draw

Uses PyTorch CUDA tensors for GPU memory (no pycuda required).
Requires TRT 10 API: set_tensor_address + execute_async_v3.

Usage:
  # Batch images → annotated JPEGs in infer_out/
  python inference/infer_python.py --engine outputs/engines/resnet18_fp16.trt \
      --image_dir data/frames/ --output_dir infer_out/

  # Explicit image paths
  python inference/infer_python.py --engine model.trt \
      --image img1.jpg img2.jpg --save_json

  # Live camera / video file
  python inference/infer_python.py --engine model.trt --source 0
  python inference/infer_python.py --engine model.trt --source video.mp4 --save out.mp4

Dependencies:
  pip install tensorrt torch opencv-python numpy
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch
import tensorrt as trt

# ── Model constants ────────────────────────────────────────────────────────────

INPUT_H     = 640
INPUT_W     = 1280
NUM_CLASSES = 6
PRED_DIM    = 11   # 4 LTRB + 1 objectness + 6 class logits

IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32)

CLASS_NAMES = ["barrel", "pedestrian", "stop_sign", "unknown", "tire", "pothole"]

# BGR colours matching inference/src/main.cpp
CLASS_COLORS = [
    (  0, 165, 255),   # barrel      — orange
    (255,  50,  50),   # pedestrian  — blue-ish
    ( 50, 220,  50),   # stop_sign   — green
    (160, 160, 160),   # unknown     — grey
    ( 50, 220, 220),   # tire        — yellow
    (220,  50, 220),   # pothole     — magenta
]

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".tif"}

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)


# ── Config ────────────────────────────────────────────────────────────────────

class Config:
    score_threshold: float = 0.20
    nms_threshold:   float = 0.50
    max_detections:  int   = 100
    topk_per_level:  int   = 100
    sky_zone_frac:   float = 0.35
    robot_body_frac: float = 0.85
    edge_excl_frac:  float = 0.05
    apply_morph:     bool  = False


Det = dict[str, Any]   # keys: x1, y1, x2, y2, score, class_id


# ── TensorRT engine wrapper ───────────────────────────────────────────────────

class MultitaskEngine:
    """TensorRT 10 inference engine using PyTorch CUDA tensors for GPU memory."""

    def __init__(self, engine_path: str, cfg: Config | None = None) -> None:
        self.cfg = cfg or Config()
        self._load_engine(engine_path)
        self._allocate_buffers()

    # ── Initialisation ─────────────────────────────────────────────────────────

    def _load_engine(self, path: str) -> None:
        runtime = trt.Runtime(TRT_LOGGER)
        with open(path, "rb") as f:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(f"Failed to deserialise TRT engine: {path}")
        self.context = self.engine.create_execution_context()

        # Verify all required tensor names exist
        tensor_names = {
            self.engine.get_tensor_name(i)
            for i in range(self.engine.num_io_tensors)
        }
        required = {"input", "det_p3", "det_p4", "det_p5", "seg_logits"}
        missing = required - tensor_names
        if missing:
            raise RuntimeError(f"Engine is missing tensors: {missing}\n"
                               f"  Found: {tensor_names}")

        print(f"[INFO] Engine loaded : {path}")
        print(f"[INFO] TRT version   : {trt.__version__}")
        print(f"[INFO] IO tensors    : {sorted(tensor_names)}")

    def _allocate_buffers(self) -> None:
        self._stream = torch.cuda.Stream()

        # Tensor sizes in float32 elements
        _sz = {
            "input":      1 * 3       * INPUT_H * INPUT_W,   # 2 457 600
            "det_p3":     1 * PRED_DIM * 80      * 160,       #   140 800
            "det_p4":     1 * PRED_DIM * 40      *  80,       #    35 200
            "det_p5":     1 * PRED_DIM * 20      *  40,       #     8 800
            "seg_logits": 1 * 2       * INPUT_H * INPUT_W,   # 1 638 400
        }

        # Contiguous CUDA tensors — .data_ptr() gives the raw device pointer for TRT
        self._d: dict[str, torch.Tensor] = {
            k: torch.empty(n, dtype=torch.float32, device="cuda").contiguous()
            for k, n in _sz.items()
        }

        # Pinned host tensors for fast D→H copies (reused each frame)
        self._h: dict[str, torch.Tensor] = {
            k: torch.empty(n, dtype=torch.float32, pin_memory=True)
            for k, n in _sz.items()
            if k != "input"
        }

        # Register tensor addresses with TRT context once (they don't change)
        for name, tensor in self._d.items():
            self.context.set_tensor_address(name, tensor.data_ptr())

    # ── Per-frame pipeline ──────────────────────────────────────────────────────

    def _preprocess(self, bgr_frame: np.ndarray) -> None:
        """Resize → RGB → float32 /255 → ImageNet normalise → NCHW → upload."""
        resized = cv2.resize(bgr_frame, (INPUT_W, INPUT_H), interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0

        plane = INPUT_H * INPUT_W
        h_input = self._h.get("_input_staging")
        if h_input is None:
            h_input = torch.empty(
                1 * 3 * INPUT_H * INPUT_W, dtype=torch.float32, pin_memory=True
            )
            self._h["_input_staging"] = h_input

        buf = h_input.numpy()
        for c in range(3):
            buf[c * plane : (c + 1) * plane] = (
                (rgb[:, :, c] - IMAGENET_MEAN[c]) / IMAGENET_STD[c]
            ).ravel()

        with torch.cuda.stream(self._stream):
            self._d["input"].copy_(h_input, non_blocking=True)

    def infer(self, bgr_frame: np.ndarray) -> dict[str, Any]:
        t0 = time.perf_counter()

        self._preprocess(bgr_frame)

        t_gpu0 = time.perf_counter()
        with torch.cuda.stream(self._stream):
            self.context.execute_async_v3(self._stream.cuda_stream)
        self._stream.synchronize()
        t_gpu1 = time.perf_counter()

        # D→H for all outputs
        with torch.cuda.stream(self._stream):
            for name in ("det_p3", "det_p4", "det_p5", "seg_logits"):
                self._h[name].copy_(self._d[name], non_blocking=True)
        self._stream.synchronize()

        # Post-process on CPU
        all_dets: list[Det] = []
        all_dets += self._decode_level(self._h["det_p3"].numpy(), 80, 160)
        all_dets += self._decode_level(self._h["det_p4"].numpy(), 40,  80)
        all_dets += self._decode_level(self._h["det_p5"].numpy(), 20,  40)

        all_dets = self._nms(all_dets)
        all_dets = self._spatial_filter(all_dets)
        seg_mask = self._decode_seg(self._h["seg_logits"].numpy())

        t1 = time.perf_counter()

        return {
            "detections": all_dets,
            "seg_mask":   seg_mask,
            "infer_ms":   (t_gpu1 - t_gpu0) * 1e3,
            "total_ms":   (t1 - t0) * 1e3,
        }

    # ── FCOS decode ────────────────────────────────────────────────────────────

    def _decode_level(
        self, feat_flat: np.ndarray, feat_h: int, feat_w: int
    ) -> list[Det]:
        feat = feat_flat.reshape(PRED_DIM, feat_h, feat_w)

        stride_y = INPUT_H / feat_h
        stride_x = INPUT_W / feat_w

        cx_grid = (np.arange(feat_w, dtype=np.float32) + 0.5) * stride_x
        cy_grid = (np.arange(feat_h, dtype=np.float32) + 0.5) * stride_y
        cx_grid = np.broadcast_to(cx_grid[None, :], (feat_h, feat_w))
        cy_grid = np.broadcast_to(cy_grid[:, None], (feat_h, feat_w))

        l = np.maximum(0.0, feat[0]) * stride_x
        t = np.maximum(0.0, feat[1]) * stride_y
        r = np.maximum(0.0, feat[2]) * stride_x
        b = np.maximum(0.0, feat[3]) * stride_y
        obj = 1.0 / (1.0 + np.exp(-feat[4]))

        # Numerically stable softmax over class logits (channels 5–10)
        cls_logits = feat[5:]
        cls_logits = cls_logits - cls_logits.max(axis=0, keepdims=True)
        exp_l = np.exp(cls_logits)
        cls_probs = exp_l / exp_l.sum(axis=0, keepdims=True)

        best_cls  = cls_probs.argmax(axis=0)
        best_prob = cls_probs.max(axis=0)
        scores    = obj * best_prob

        keep = scores >= self.cfg.score_threshold
        if not keep.any():
            return []

        x1 = np.clip(cx_grid[keep] - l[keep], 0.0, float(INPUT_W))
        y1 = np.clip(cy_grid[keep] - t[keep], 0.0, float(INPUT_H))
        x2 = np.clip(cx_grid[keep] + r[keep], 0.0, float(INPUT_W))
        y2 = np.clip(cy_grid[keep] + b[keep], 0.0, float(INPUT_H))
        sc = scores[keep]
        cl = best_cls[keep]

        dets: list[Det] = [
            {"x1": float(x1[i]), "y1": float(y1[i]),
             "x2": float(x2[i]), "y2": float(y2[i]),
             "score": float(sc[i]), "class_id": int(cl[i])}
            for i in range(len(sc))
        ]

        if len(dets) > self.cfg.topk_per_level:
            dets.sort(key=lambda d: d["score"], reverse=True)
            dets = dets[: self.cfg.topk_per_level]

        return dets

    # ── Class-agnostic greedy NMS ───────────────────────────────────────────────

    def _nms(self, dets: list[Det]) -> list[Det]:
        if not dets:
            return dets
        dets.sort(key=lambda d: d["score"], reverse=True)
        suppressed = [False] * len(dets)
        kept: list[Det] = []
        for i, d in enumerate(dets):
            if suppressed[i]:
                continue
            kept.append(d)
            if len(kept) >= self.cfg.max_detections:
                break
            for j in range(i + 1, len(dets)):
                if not suppressed[j] and _iou(d, dets[j]) > self.cfg.nms_threshold:
                    suppressed[j] = True
        return kept

    def _spatial_filter(self, dets: list[Det]) -> list[Det]:
        body_row = self.cfg.robot_body_frac * INPUT_H
        edge_lo  = self.cfg.edge_excl_frac * INPUT_W
        edge_hi  = (1.0 - self.cfg.edge_excl_frac) * INPUT_W
        return [
            d for d in dets
            if not (
                (d["y1"] + d["y2"]) * 0.5 >= body_row
                or (d["x1"] + d["x2"]) * 0.5 < edge_lo
                or (d["x1"] + d["x2"]) * 0.5 > edge_hi
            )
        ]

    def _decode_seg(self, seg_flat: np.ndarray) -> np.ndarray:
        seg  = seg_flat.reshape(2, INPUT_H, INPUT_W)
        mask = (seg[1] > seg[0]).astype(np.uint8)

        sky_row  = int(self.cfg.sky_zone_frac   * INPUT_H)
        body_row = int(self.cfg.robot_body_frac * INPUT_H)
        mask[:sky_row,  :] = 0
        mask[body_row:, :] = 0

        if self.cfg.apply_morph:
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)

        return mask


# ── Helpers ───────────────────────────────────────────────────────────────────

def _iou(a: Det, b: Det) -> float:
    ix1 = max(a["x1"], b["x1"])
    iy1 = max(a["y1"], b["y1"])
    ix2 = min(a["x2"], b["x2"])
    iy2 = min(a["y2"], b["y2"])
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    if inter == 0.0:
        return 0.0
    area_a = (a["x2"] - a["x1"]) * (a["y2"] - a["y1"])
    area_b = (b["x2"] - b["x1"]) * (b["y2"] - b["y1"])
    return inter / (area_a + area_b - inter + 1e-6)


# ── Visualisation ─────────────────────────────────────────────────────────────

def overlay_seg(bgr: np.ndarray, mask: np.ndarray) -> np.ndarray:
    out   = bgr.astype(np.float32)
    tint  = np.zeros_like(out)
    tint[:, :, 1] = 220.0   # G
    tint[:, :, 2] = 80.0    # R (BGR: B=0, G=220, R=80)
    alpha = (mask > 0)[:, :, None].astype(np.float32) * 0.25
    out   = out * (1.0 - alpha) + tint * alpha
    return np.clip(out, 0, 255).astype(np.uint8)


def draw_detections(img: np.ndarray, dets: list[Det]) -> None:
    for d in dets:
        ci  = max(0, min(d["class_id"], NUM_CLASSES - 1))
        col = CLASS_COLORS[ci]
        cv2.rectangle(img,
                      (int(d["x1"]), int(d["y1"])),
                      (int(d["x2"]), int(d["y2"])),
                      col, 2, cv2.LINE_AA)
        label = f"{CLASS_NAMES[ci]}  {d['score']:.2f}"
        ts, baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1)
        lx = int(d["x1"])
        ly = max(int(d["y1"]) - baseline - 3, ts[1] + 2)
        cv2.rectangle(img, (lx, ly - ts[1] - 1), (lx + ts[0] + 2, ly + baseline), col, cv2.FILLED)
        cv2.putText(img, label, (lx + 1, ly),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)


def draw_hud(img: np.ndarray, n_dets: int,
             infer_ms: float, total_ms: float, frame_idx: int) -> None:
    text = (f"Frame {frame_idx:<5d} | Dets: {n_dets:2d} "
            f"| GPU {infer_ms:.1f} ms | Wall {total_ms:.1f} ms")
    cv2.putText(img, text, (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (230, 230, 230), 1, cv2.LINE_AA)


# ── Image / directory mode ────────────────────────────────────────────────────

def run_images(engine: MultitaskEngine, args: argparse.Namespace) -> None:
    if args.image:
        image_paths = [Path(p) for p in args.image]
    elif args.image_dir:
        image_paths = sorted(
            p for p in Path(args.image_dir).iterdir()
            if p.is_file() and p.suffix.lower() in IMAGE_EXTS
        )
        print(f"[INFO] Found {len(image_paths)} images in {args.image_dir}")
    else:
        raise RuntimeError("No input — use --image, --image_dir, or --source")

    if not image_paths:
        print("[WARN] No images found.")
        return

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    results: list[dict] = []
    total_infer_ms = 0.0
    total_dets = 0

    for idx, img_path in enumerate(image_paths):
        frame = cv2.imread(str(img_path))
        if frame is None:
            print(f"[WARN] Cannot read: {img_path}")
            continue

        result = engine.infer(frame)
        total_infer_ms += result["infer_ms"]
        total_dets     += len(result["detections"])

        display = cv2.resize(frame, (INPUT_W, INPUT_H), interpolation=cv2.INTER_LINEAR)
        display = overlay_seg(display, result["seg_mask"])
        draw_detections(display, result["detections"])

        stem = img_path.stem
        cv2.putText(display, stem, (6, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(display, stem, (6, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)

        out_path = out_dir / (stem + "_pred.jpg")
        cv2.imwrite(str(out_path), display, [cv2.IMWRITE_JPEG_QUALITY, 92])

        print(f"[{idx + 1:4d}/{len(image_paths)}]  {img_path.name:<40}  "
              f"dets={len(result['detections']):<3d}  GPU={result['infer_ms']:.1f} ms")

        if args.save_json:
            results.append({
                "image":    str(img_path),
                "infer_ms": round(result["infer_ms"], 1),
                "dets": [
                    {"class_id":   d["class_id"],
                     "class_name": CLASS_NAMES[d["class_id"]],
                     "score":      round(d["score"], 3),
                     "box":        [d["x1"], d["y1"], d["x2"], d["y2"]]}
                    for d in result["detections"]
                ],
            })

    n = len(image_paths)
    if n:
        print(f"\n─── Summary ─────────────────────────────────────────────────────")
        print(f"  Images     : {n}")
        print(f"  Total dets : {total_dets}  (avg {total_dets / n:.1f} / image)")
        print(f"  Avg GPU    : {total_infer_ms / n:.1f} ms/image")
        print(f"  Output dir : {out_dir}")

    if args.save_json and results:
        json_path = out_dir / "results.json"
        json_path.write_text(json.dumps(results, indent=2))
        print(f"  JSON       : {json_path}")

    print("─────────────────────────────────────────────────────────────────\n")


# ── Video / webcam mode ───────────────────────────────────────────────────────

def run_video(engine: MultitaskEngine, args: argparse.Namespace) -> None:
    source = args.source
    try:
        cam_idx = int(source)
        cap    = cv2.VideoCapture(cam_idx)
        is_cam = True
    except (ValueError, TypeError):
        cap    = cv2.VideoCapture(source)
        is_cam = False

    if not cap.isOpened():
        raise RuntimeError(f"Cannot open source: {source}")

    if is_cam:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  3840)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1920)
        cap.set(cv2.CAP_PROP_FPS,          30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
        print(f"[INFO] Camera: {cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}"
              f"×{cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f}"
              f" @ {cap.get(cv2.CAP_PROP_FPS):.0f} fps")

    writer = None
    if args.save:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(args.save, fourcc, 30.0, (INPUT_W, INPUT_H), True)
        if not writer.isOpened():
            raise RuntimeError(f"Cannot open output: {args.save}")
        print(f"[INFO] Saving to: {args.save}")

    frame_idx = 0
    print("[INFO] Running — press 'q' or Ctrl-C to stop.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                break

            result = engine.infer(frame)

            display = cv2.resize(frame, (INPUT_W, INPUT_H), interpolation=cv2.INTER_LINEAR)
            display = overlay_seg(display, result["seg_mask"])
            draw_detections(display, result["detections"])
            draw_hud(display, len(result["detections"]),
                     result["infer_ms"], result["total_ms"], frame_idx)

            if writer:
                writer.write(display)

            if not args.no_display:
                cv2.imshow("Multitask Perception", display)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break

            if args.verbose or frame_idx % 60 == 0:
                print(f"[{frame_idx:5d}]  dets={len(result['detections']):<3d}  "
                      f"GPU={result['infer_ms']:.1f} ms  wall={result['total_ms']:.1f} ms")

            frame_idx += 1

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if writer:
            writer.release()
        cv2.destroyAllWindows()

    print(f"[INFO] Stopped after {frame_idx} frames.")


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Python TRT 10 inference for the multitask perception model."
    )
    p.add_argument("--engine", required=True, help="TensorRT .trt / .engine file")

    src = p.add_mutually_exclusive_group()
    src.add_argument("--source",    help="Camera index (e.g. 0) or video file path")
    src.add_argument("--image",     nargs="+", metavar="PATH")
    src.add_argument("--image_dir", metavar="DIR")

    p.add_argument("--output_dir", default="infer_out")
    p.add_argument("--score",      type=float, default=0.20)
    p.add_argument("--nms",        type=float, default=0.50)
    p.add_argument("--morph",      action="store_true")
    p.add_argument("--no_display", action="store_true")
    p.add_argument("--save",       default="", metavar="PATH")
    p.add_argument("--save_json",  action="store_true")
    p.add_argument("--verbose",    action="store_true")
    return p.parse_args()


def main() -> None:
    args = parse_args()

    cfg = Config()
    cfg.score_threshold = args.score
    cfg.nms_threshold   = args.nms
    cfg.apply_morph     = args.morph

    print(f"[INFO] Loading engine: {args.engine}")
    engine = MultitaskEngine(args.engine, cfg)
    print("[INFO] Engine ready.\n")

    if args.source is not None:
        run_video(engine, args)
    else:
        run_images(engine, args)


if __name__ == "__main__":
    main()
