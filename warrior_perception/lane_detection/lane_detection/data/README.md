# Data Package

Manifest-driven dataset loader for multitask training. Large local images, masks, and manifests should live under `data/` but are excluded from Git by default.

---

## Manifest Format

A manifest is a JSON list. Each object describes one training sample:

```json
[
  {
    "image": "path/to/frame.jpg",
    "boxes": [[10, 20, 50, 80], [120, 60, 200, 150]],
    "labels": [1, 0],
    "seg_mask": "path/to/lane_mask.png",
    "seg_confidence": 0.92,
    "seg_roi_mask": "path/to/roi_mask.png"
  },
  {
    "image": "path/to/seg_only_frame.jpg",
    "boxes": [],
    "labels": [],
    "seg_mask": "path/to/lane_mask_2.png",
    "seg_confidence": 0.75,
    "seg_roi_mask": "path/to/roi_mask.png"
  }
]
```

### Field Reference

| Field            | Required | Type                      | Description                                                        |
| ---------------- | -------- | ------------------------- | ------------------------------------------------------------------ |
| `image`          | yes      | string                    | Path to the RGB frame, relative to the repo root                   |
| `boxes`          | yes      | list of [x1, y1, x2, y2] | Bounding boxes in pixel coordinates; empty list for seg-only frames |
| `labels`         | yes      | list of int               | Class index for each box; must be the same length as `boxes`       |
| `seg_mask`       | no       | string                    | Single-channel PNG: pixel value 0 = background, 1 = lane          |
| `seg_confidence` | no       | float 0–1                 | Scales the segmentation loss for this sample (pseudo-label weight) |
| `seg_roi_mask`   | no       | string                    | Binary PNG: 255 = include in loss, 0 = ignore                      |

- Samples with empty `boxes` and `labels` are valid — they contribute only segmentation loss.
- Samples without `seg_mask` contribute only detection loss.
- All paths are resolved relative to the repository root unless `image_root` / `seg_root` overrides are set in the config.

### Detection Class Indices

| Index | Class      |
| ----- | ---------- |
| 0     | barrel     |
| 1     | pedestrian |
| 2     | stop_sign  |
| 3     | unknown    |
| 4     | tire       |
| 5     | pothole    |

---

## Dataset Loader API

```python
from data.dataset import MultitaskDataset
from data.collate import collate_fn
from torch.utils.data import DataLoader

dataset = MultitaskDataset(
    manifest_path="work/train_manifest.json",
    image_size=(640, 1280),
    transform=None,          # pass an albumentations transform here
    image_root=".",
    seg_root=".",
)

loader = DataLoader(dataset, batch_size=4, collate_fn=collate_fn, num_workers=4)

for batch in loader:
    images   = batch["image"]        # (B, 3, H, W) float32, ImageNet-normalised
    boxes    = batch["boxes"]        # list of (N_i, 4) tensors
    labels   = batch["labels"]       # list of (N_i,) int tensors
    seg_mask = batch["seg_mask"]     # (B, H, W) long, -100 where no mask
    seg_conf = batch["seg_confidence"]   # (B,) float
    roi_mask = batch["seg_roi_mask"] # (B, H, W) bool
```

### Manifest Validation

Call `validate_manifest(path)` to catch missing files or mismatched box/label lengths before training:

```python
from data.dataset import validate_manifest
validate_manifest("work/train_manifest.json")
```

---

## Building Manifests

Use the pipeline tool to combine images, detections, and lane records into a manifest:

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

Or use the simple builder for detection-only datasets:

```bash
python tools/prep/build_manifest.py \
  --images_dir path/to/images \
  --annotations path/to/coco_annotations.json \
  --output work/manifest.json
```
