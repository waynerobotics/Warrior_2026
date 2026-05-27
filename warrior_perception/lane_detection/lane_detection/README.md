# Multi-Task Perception for Robotics

Real-time object detection and lane segmentation from 360° equirectangular camera frames, built for mobile robotics deployment.

[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-blue)](https://www.python.org/)
[![PyTorch](https://img.shields.io/badge/PyTorch-2.x-orange)](https://pytorch.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green)](LICENSE)

---

## What This Does

A single neural network that simultaneously:

1. **Detects objects** in 6 categories — barrel, pedestrian, stop sign, unknown obstacle, tire, pothole
2. **Segments the drivable lane area** as a binary mask (lane vs. background)

The model consumes equirectangular (360°) camera frames at full resolution (640 × 1280 px) and is designed to run on-robot with GPU acceleration. An anchor-free FCOS-style detection head and a bilinear segmentation decoder share a common backbone and Feature Pyramid Network.

---

## Results

Backbone comparison trained on ~1,400 manually annotated frames:

| Backbone      | mAP   | mAP@50 | Lane IoU | Lane Dice | Latency  | FPS    | GPU Memory |
| ------------- | ----- | ------ | -------- | --------- | -------- | ------ | ---------- |
| **ResNet-18** | **0.248** | **0.559** | **0.901** | **0.945** | 15.9 ms | **62.9** | 452 MB |
| ConvNeXt-Base | 0.224 | 0.461  | 0.884    | 0.933     | 206.4 ms | 4.8    | 1,771 MB   |

ResNet-18 is the recommended backbone — best speed/accuracy trade-off for embedded robotics deployment. Training used ~1,400 manually annotated 640 × 1280 frames; both runs triggered early stopping, so these numbers are conservative.

### Training curves

![ResNet-18 training curves](docs/assets/curves_resnet18.png)

### Inference example

![Inference sample](docs/assets/inference_sample.jpg)

---

## Architecture

```text
Equirectangular Frame  640 × 1280 px
          │
    ┌─────┴──────┐
    │  Backbone  │   ResNet-18 · ConvNeXt-B · Swin-B · HRNet
    └─────┬──────┘
          │  C3, C4, C5 feature maps
    ┌─────┴──────┐
    │    FPN     │   Feature Pyramid Network → P3 (stride 8) · P4 (stride 16) · P5 (stride 32)
    └──┬──────┬──┘
       │      │
  ┌────┴────┐ ┌──────┴──────┐
  │Detection│ │Segmentation │
  │  Head   │ │   Head      │
  └────┬────┘ └──────┬──────┘
       │              │
Anchor-free        Binary lane mask
FCOS boxes         640 × 1280 px
6 classes
```

- **Detection head** — 4 LTRB offsets + 1 objectness score + 6 class logits, decoded per FPN level then merged with NMS.
- **Segmentation head** — bilinear upsampling from fused P3/P4/P5 features to full input resolution.
- **Loss** — focal detection loss + cross-entropy (or Dice) segmentation loss, masked to a configurable ROI and optionally scaled by per-sample pseudo-label confidence.

See [`docs/architecture.md`](docs/architecture.md) for a full technical breakdown.

---

## Quick Start

### 1. Clone and create a virtual environment

```bash
git clone https://github.com/yourname/mutli-task-perception.git
cd mutli-task-perception

python -m venv .venv

# Windows
.venv\Scripts\activate
# macOS / Linux
source .venv/bin/activate

pip install -r requirements.txt
```

### 2. Prepare your dataset

Build a training manifest from your annotated images (see [Dataset Format](#dataset-format) below for the schema). If you already have images, detection annotations, and lane masks, point the manifest builder at them:

```bash
python tools/equirect_lane_pipeline/build_multitask_manifest.py \
  --images_dir path/to/frames \
  --detection_annotations path/to/detections.json \
  --lane_records path/to/lane_records.jsonl \
  --train_manifest work/train_manifest.json \
  --val_manifest work/val_manifest.json \
  --val_ratio 0.2 \
  --shared_roi_mask path/to/roi_mask.png
```

### 3. Train

```bash
python scripts/train.py \
  --config configs/multitask/multitask_resnet18.yaml \
  --train_manifest work/train_manifest.json \
  --val_manifest work/val_manifest.json \
  --output_dir outputs/resnet18_run \
  --device auto
```

Training writes checkpoints, a `metrics.jsonl` log, and an MLflow run (if `mlflow` is installed) to the output directory.

**Smoke test** — verify the pipeline runs before committing to a full training run:

```bash
python scripts/train.py \
  --config configs/multitask/multitask_resnet18.yaml \
  --train_manifest work/train_manifest.json \
  --val_manifest work/val_manifest.json \
  --output_dir outputs/smoke \
  --epochs 1 --batch_size 1 --max_steps_per_epoch 1 --device cpu
```

### 4. Compare backbones

```bash
python scripts/run_backbone_sweep.py \
  --config configs/multitask/multitask_resnet18.yaml \
  --train_manifest work/train_manifest.json \
  --val_manifest work/val_manifest.json \
  --output_dir outputs/backbone_sweep

python scripts/generate_report.py --sweep_dir outputs/backbone_sweep
```

### 5. Export to ONNX

```bash
python scripts/export_onnx.py \
  --checkpoint outputs/resnet18_run/best.pth \
  --config configs/multitask/multitask_resnet18.yaml \
  --output_path outputs/model.onnx
```

The script exports the model and verifies numerical divergence against the PyTorch reference (expected max absolute difference < 1e-5). See [`inference/README.md`](inference/README.md) for TensorRT conversion and the C++ deployment pipeline.

---

## Dataset Format

Training is manifest-driven. A manifest is a JSON list where each entry describes one annotated frame:

```json
[
  {
    "image": "path/to/frame.jpg",
    "boxes": [[10, 20, 50, 80], [120, 60, 200, 150]],
    "labels": [1, 0],
    "seg_mask": "path/to/lane_mask.png",
    "seg_confidence": 0.92,
    "seg_roi_mask": "path/to/roi_mask.png"
  }
]
```

| Field             | Type                        | Description                                                |
| ----------------- | --------------------------- | ---------------------------------------------------------- |
| `image`           | string                      | Path to the RGB frame (relative to repo root)              |
| `boxes`           | list of [x1, y1, x2, y2]   | Detection boxes in pixel coordinates                       |
| `labels`          | list of int                 | Class index per box (see table below)                      |
| `seg_mask`        | string                      | Single-channel PNG: 0 = background, 1 = lane               |
| `seg_confidence`  | float 0–1                   | Pseudo-label confidence — scales segmentation loss weight  |
| `seg_roi_mask`    | string                      | Binary mask PNG: 255 = supervise, 0 = ignore               |

Frames with empty `boxes` / `labels` are valid for segmentation-only training. See [`data/README.md`](data/README.md) for the full schema and dataset loader API.

### Detection class indices

| Index | Class       |
| ----- | ----------- |
| 0     | barrel      |
| 1     | pedestrian  |
| 2     | stop_sign   |
| 3     | unknown     |
| 4     | tire        |
| 5     | pothole     |

---

## Configuration

All experiment settings live in YAML files under `configs/multitask/`. Key options in `multitask_resnet18.yaml`:

| Key                           | Default    | Description                                          |
| ----------------------------- | ---------- | ---------------------------------------------------- |
| `backbone.name`               | `resnet18` | `resnet18`, `convnext_base`, `swin_b`, `hrnet_w32`   |
| `detection.num_classes`       | `6`        | Number of object categories                          |
| `optimizer.lr`                | `0.0001`   | Learning rate (AdamW with cosine annealing)          |
| `epochs`                      | `50`       | Maximum training epochs                              |
| `batch_size`                  | `4`        | Samples per batch                                    |
| `device`                      | `auto`     | `auto`, `cuda`, or `cpu`                             |
| `mixed_precision.enabled`     | `true`     | FP16 mixed precision (requires CUDA)                 |
| `early_stopping.patience`     | `12`       | Stop if val loss doesn't improve for N epochs        |
| `segmentation.loss`           | `ce`       | `ce` (cross-entropy) or `dice`                       |

Change config values rather than editing Python code wherever possible.

---

## Project Layout

```text
mutli-task-perception/
├── configs/                         # YAML experiment configs — edit these first
│   └── multitask/
│       ├── multitask_resnet18.yaml  # Recommended baseline
│       ├── multitask_convnext.yaml
│       ├── multitask_swinb.yaml
│       └── multitask_hrnet.yaml
├── data/                            # Manifest-driven dataset loader
├── models/                          # Neural network: backbone, FPN, heads, loss
├── engine/                          # Training loop, evaluation, metrics
├── scripts/                         # Command-line entry points
│   ├── train.py                     # Train one config
│   ├── run_backbone_sweep.py        # Compare multiple backbones
│   ├── export_onnx.py               # Export to ONNX
│   ├── visualize_inference.py       # Side-by-side ONNX inference comparison
│   └── generate_report.py           # Backbone sweep comparison report
├── tools/
│   └── equirect_lane_pipeline/      # Frame extraction and pseudo-label generation
├── inference/                       # ONNX → TensorRT deployment
├── tests/                           # Smoke tests
└── docs/                            # Architecture and deployment reference
```

---

## Running Tests

```bash
pytest tests/
```

The smoke test validates dataset loading, model forward pass, loss computation, and metrics without needing data on disk.

---

## Troubleshooting

**`CUDA unavailable`** — `device: auto` falls back to CPU automatically. CPU training is slow but works.

**Empty manifest error** — `MultitaskDataset` requires at least one sample. Verify your manifest file is a non-empty JSON list and all paths resolve from the repo root.

**Config mismatch** — changing `detection.num_classes` or `segmentation.num_classes` requires rebuilding all checkpoints. Model output shapes are fixed at construction time from the config.

**Segmentation masks misaligned** — verify your masks were generated at the same resolution as `dataset.image_size` in your config (default: 640 × 1280).

---

## Roadmap

- [ ] Expand training data with robot-collected and pseudo-labelled frames
- [ ] ONNX export validation against TensorRT engine on Jetson Orin
- [ ] ROS 2 node wrapper for real-time robot integration
- [ ] INT8 quantisation and latency benchmarking

---

## License

MIT — see [LICENSE](LICENSE).
