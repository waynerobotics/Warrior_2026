# Equirectangular Lane Supervision Pipeline

Converts raw drive videos into training-ready manifests for the multitask model. Everything stays in equirectangular image space — no perspective unwarping is performed.

The full pipeline runs in eight steps. Steps 1–5 produce manifests. Steps 6–7 train and compare models. Step 8 is a one-command orchestrator that wraps the entire flow.

---

## Step 1 — Extract Frames

```bash
python tools/equirect_lane_pipeline/extract_frames.py \
  --input_video data/raw/drive.mp4 \
  --output_dir work/equirect_run/frames \
  --fps 5 \
  --resize_h 640 \
  --resize_w 1280
```

Outputs `frames/` (JPEG files) and `metadata.jsonl` (per-frame timestamps and filenames).

---

## Step 2 — Build an ROI Mask

The ROI mask restricts lane supervision to the part of the frame where lanes actually appear (typically the lower half). Pixels outside the ROI contribute zero gradient during training.

**Band mode** (rectangular region):

```bash
python tools/equirect_lane_pipeline/build_roi_masks.py \
  --image_h 640 --image_w 1280 \
  --mode manual_band \
  --top 320 --bottom 640 --left 0 --right 1280 \
  --output_mask work/equirect_run/roi/roi_mask.png
```

**Polygon mode** (arbitrary shape from a JSON point list):

```bash
python tools/equirect_lane_pipeline/build_roi_masks.py \
  --image_h 640 --image_w 1280 \
  --mode polygon \
  --polygon_json roi_points.json \
  --output_mask work/equirect_run/roi/roi_mask.png
```

Output pixel values: 255 = valid, 0 = ignored.

---

## Step 3 — Generate Lane Pseudo-Labels

```bash
python tools/equirect_lane_pipeline/generate_lane_masks.py \
  --backend clrnet \
  --frames_dir work/equirect_run/frames \
  --output_dir work/equirect_run/lane_labels \
  --config configs/lane_backends/opencv_equirect_lane.yaml \
  --device auto \
  --score_threshold 0.6 \
  --mask_thickness 6 \
  --roi_mask work/equirect_run/roi/roi_mask.png \
  --save_overlay
```

**Backends**: `dummy` (plumbing test only), `clrnet`, `laneatt`. The `clrnet` slot also accepts any custom adapter via `--config` as long as the adapter exposes a `predict(image_bgr)` method that returns one of:

```python
{"mask": mask_array, "confidence": 0.91}
{"lanes": [[(x0, y0), (x1, y1), ...]], "confidence": 0.91}
(lanes, confidence)
```

Dense masks are binarised; lane point sequences are rasterised with `--mask_thickness` pixels.

---

## Step 4 — Filter Lane Masks

Removes low-quality pseudo-labels before training:

```bash
python tools/equirect_lane_pipeline/filter_lane_masks.py \
  --input_records work/equirect_run/lane_labels/records.jsonl \
  --output_records work/equirect_run/lane_labels/filtered_records.jsonl \
  --min_confidence 0.6 \
  --min_lane_pixels 1 \
  --drop_empty_masks
```

---

## Step 5 — Build Training Manifests

Combines images, detection annotations, and filtered lane records into train/val manifests:

```bash
python tools/equirect_lane_pipeline/build_multitask_manifest.py \
  --images_dir work/equirect_run/frames \
  --detection_annotations data/detections.json \
  --lane_records work/equirect_run/lane_labels/filtered_records.jsonl \
  --train_manifest work/equirect_run/train_manifest.json \
  --val_manifest work/equirect_run/val_manifest.json \
  --val_ratio 0.2 \
  --require_segmentation \
  --shared_roi_mask work/equirect_run/roi/roi_mask.png
```

Manifest schema per sample:

```json
{
  "image": "path/to/frame.jpg",
  "boxes": [[10, 20, 50, 80]],
  "labels": [1],
  "seg_mask": "path/to/lane_mask.png",
  "seg_confidence": 0.92,
  "seg_roi_mask": "path/to/roi_mask.png"
}
```

See [`data/README.md`](../../data/README.md) for the full field reference.

---

## Step 6 — Train One Config

```bash
python scripts/train.py \
  --config configs/multitask/multitask_resnet18.yaml \
  --train_manifest work/equirect_run/train_manifest.json \
  --val_manifest work/equirect_run/val_manifest.json
```

---

## Step 7 — Backbone Sweep

```bash
python scripts/run_backbone_sweep.py \
  --config configs/multitask/multitask_resnet18.yaml \
  --train_manifest work/equirect_run/train_manifest.json \
  --val_manifest work/equirect_run/val_manifest.json \
  --output_dir work/equirect_run/experiments
```

Trains ResNet-18, ConvNeXt-Base, and Swin-B (all with detection + segmentation). Add `--include_exploratory` to also run the HRNet fallback.

---

## Step 8 — Full Orchestration (single command)

```bash
python scripts/retrain_all_backbones.py \
  --input_video data/raw/drive.mp4 \
  --detection_annotations data/detections.json \
  --work_dir work/equirect_run \
  --lane_backend clrnet \
  --config configs/lane_backends/opencv_equirect_lane.yaml \
  --device auto \
  --batch_size 4 \
  --fps 5 \
  --min_confidence 0.6 \
  --val_ratio 0.2 \
  --roi_top 320 \
  --roi_bottom 640
```

---

## Advanced: ROI-Masked Segmentation Loss

`models/loss.py` computes unreduced cross-entropy, then applies the binary ROI mask before averaging — pixels outside the ROI never contribute gradient. Dice loss is also supported:

```yaml
segmentation:
  loss: dice
```

---

## Advanced: Confidence-Weighted Pseudo-Labels

Each manifest entry may include a `seg_confidence` float. The segmentation loss for that sample is scaled by this value at training time. This allows low-confidence pseudo-labels to contribute without dominating the gradient signal.

---

## Advanced: Seam-Aware Augmentation

Equirectangular images are horizontally cyclic. Enable circular shift augmentation to exploit this:

```yaml
augment:
  circular_shift:
    enabled: true
    max_fraction: 0.15
    shift_boxes: false
```

Images, segmentation masks, and ROI masks are all shifted with wrap-around. Detection box shifting is disabled by default because boxes that cross the seam are split — enable `shift_boxes` only if your labels can tolerate that behaviour.
