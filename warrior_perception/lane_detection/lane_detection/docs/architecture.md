# Model Architecture

Technical reference for the multitask perception model. For high-level context, see the root [`README.md`](../README.md).

---

## Overview

The model is a single forward pass that produces both detection boxes and a segmentation mask from one set of shared features. This is more efficient than running two separate models and allows the backbone to learn representations that are jointly useful for both tasks.

```
Input (1, 3, 640, 1280)
       │
  ┌────┴─────┐
  │ Backbone │   Extracts hierarchical feature maps at 3 scales
  └────┬─────┘
       │  C3 (stride 8), C4 (stride 16), C5 (stride 32)
  ┌────┴─────┐
  │   FPN    │   Fuses multi-scale features with top-down lateral connections
  └──┬─────┬─┘
     │     │  P3, P4, P5
  ┌──┴──┐ ┌┴──────────────┐
  │ Det │ │  Seg Head      │
  │ Head│ │                │
  └──┬──┘ └───────┬────────┘
     │             │
Boxes + classes  Lane mask (640×1280)
```

---

## Backbone

**Role** — extract rich visual features at multiple spatial resolutions from the input image.

Supported backbones (configured via `backbone.name` in the YAML config):

| Name           | Params  | ImageNet Top-1 | Notes                                         |
| -------------- | ------- | -------------- | --------------------------------------------- |
| `resnet18`     | 11 M    | 69.8%          | Recommended — fastest, lowest memory          |
| `convnext_base`| 89 M    | 85.8%          | Higher accuracy, 13× slower than ResNet-18    |
| `swin_b`       | 88 M    | 85.2%          | Transformer backbone                          |
| `hrnet_w32`    | 29 M    | 78.5%          | Custom lightweight fallback (not full HRNet)  |

All backbones are pretrained on ImageNet and loaded via `torchvision.models` or `timm`. The backbone registry in `models/backbone.py` maps config names to builder functions — add new backbones there with the `@register_backbone` decorator.

The backbone outputs three feature maps:
- **C3** — stride 8 relative to input, captures fine detail (small objects, lane edges)
- **C4** — stride 16, mid-level semantics
- **C5** — stride 32, high-level context (scene understanding)

---

## Feature Pyramid Network (FPN)

**Role** — combine features from all three backbone levels so every output scale sees both fine texture and high-level context.

```
C5 (stride 32) ──lateral conv──▶ P5
                                   │  upsample ×2
C4 (stride 16) ──lateral conv──▶ P4 = P4 + upsampled(P5)
                                   │  upsample ×2
C3 (stride 8)  ──lateral conv──▶ P3 = P3 + upsampled(P4)
```

All lateral convolutions project to 256 channels. The three output feature maps `{P3, P4, P5}` are fed independently to the detection head and jointly to the segmentation head.

Implemented in `models/fpn.py`.

---

## Detection Head (FCOS-style)

**Role** — predict object bounding boxes and class labels at each FPN level without pre-defined anchor boxes.

Each spatial cell at each FPN level predicts:

| Output          | Channels | Description                                                    |
| --------------- | -------- | -------------------------------------------------------------- |
| LTRB offsets    | 4        | Distances to left, top, right, bottom edges of the box        |
| Objectness      | 1        | Sigmoid score: is there an object centred at this cell?       |
| Class logits    | 6        | One logit per detection class                                  |

**Total per cell: 11 values** → output shapes `(B, 11, H/8, W/8)` at P3, `(B, 11, H/16, W/16)` at P4, etc.

**Post-processing** (inference only):
1. Sigmoid objectness; threshold at `score_threshold` (default 0.20).
2. Decode LTRB from each surviving cell's centre coordinates.
3. Score = objectness × softmax class confidence.
4. Concatenate predictions across P3/P4/P5.
5. Non-maximum suppression with `iou_threshold = 0.5`, keep top 100.

Implemented in `models/detection_head.py` and `models/detection_postprocess.py`.

---

## Segmentation Head

**Role** — produce a full-resolution binary lane mask from the FPN features.

```
P3 ──────────────────────────────────┐
P4 ── upsample to P3 resolution ─────┤ concat → 1×1 conv → upsample to 640×1280
P5 ── upsample to P3 resolution ─────┘
```

All three FPN levels are bilinearly upsampled to P3's resolution (H/8, W/8), concatenated along the channel axis, reduced with a 1×1 convolution to `num_classes` (default 2), then bilinearly upsampled to full input resolution.

Output: `(B, 2, 640, 1280)` logits. Taking argmax gives the class map: `0 = background`, `1 = lane`.

Implemented in `models/segmentation_head.py`.

---

## Loss Function

**Multitask loss** = weighted sum of detection loss and segmentation loss.

```
L_total = w_det × L_det  +  w_seg × L_seg
```

Weights are set in config under `loss_weights` (both default to 1.0).

### Detection Loss

Focal loss on objectness + cross-entropy on class logits + L1 regression on LTRB offsets, summed over positive cells. Positive assignment follows FCOS rules: cells within the centre region of a ground-truth box at the appropriate FPN scale.

### Segmentation Loss

Cross-entropy (default) or Dice loss between the full-resolution prediction and the ground-truth lane mask.

Two mechanisms restrict supervision to reliable pixels:

**ROI masking** — a binary mask (`seg_roi_mask`) defines the valid supervision region (e.g., the lower half of the equirectangular frame where lanes appear). Pixels outside the ROI contribute zero gradient.

**Pseudo-label confidence weighting** — each sample's `seg_confidence` scalar scales its segmentation loss contribution. Auto-generated pseudo-labels with low confidence are down-weighted without being discarded entirely.

Implemented in `models/loss.py` and `models/segmentation_roi.py`.

---

## Full Model

`models/multitask_model.py` wires backbone → FPN → detection head + segmentation head into a single `nn.Module`:

```python
from models.multitask_model import MultiTaskPerceptionModel
import yaml

with open("configs/multitask/multitask_resnet18.yaml") as f:
    cfg = yaml.safe_load(f)

model = MultiTaskPerceptionModel(cfg)

outputs = model(images)
# outputs["det_p3"]    → (B, 11, 80, 160)
# outputs["det_p4"]    → (B, 11, 40, 80)
# outputs["det_p5"]    → (B, 11, 20, 40)
# outputs["seg_logits"]→ (B, 2, 640, 1280)
```

During training, `engine/train.py` feeds `outputs` and the batch ground truth to `MultiTaskLoss`. During export, the same `outputs` dict maps directly to the four ONNX output nodes.
