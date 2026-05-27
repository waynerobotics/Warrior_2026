# Inference Post-Processing Reference

Intended audience: C++ / ROS 2 implementation of the multitask perception model on Jetson Orin.
All numbers come from the trained ONNX models and the Python reference implementation in
`scripts/visualize_inference.py` and `models/detection_postprocess.py`.

---

## 1. Model Inputs and Outputs

### Input

| Field | Value |
|---|---|
| Name | `input` |
| Shape | `(1, 3, 640, 1280)` — NCHW, batch fixed to 1 |
| Dtype | `float32` |
| Channel order | RGB (not BGR) |
| Normalisation | ImageNet: subtract `[0.485, 0.456, 0.406]`, divide by `[0.229, 0.224, 0.225]` per channel |

Preprocessing steps in order:
1. Resize raw frame to `1280 × 640` (W × H) with bilinear interpolation.
2. Convert BGR → RGB.
3. Cast to `float32`, divide by `255.0`.
4. Subtract mean, divide by std, channel-wise.
5. Transpose from HWC to CHW, add batch dimension → `(1, 3, 640, 1280)`.

### Outputs

| Name | Shape | Description |
|---|---|---|
| `det_p3` | `(1, 11, 80, 160)` | FCOS predictions — stride 8, small objects |
| `det_p4` | `(1, 11, 40, 80)` | FCOS predictions — stride 16, medium objects |
| `det_p5` | `(1, 11, 20, 40)` | FCOS predictions — stride 32, large objects |
| `seg_logits` | `(1, 2, 640, 1280)` | Lane segmentation logits — full resolution |

`pred_dim = 11 = 4 (LTRB offsets) + 1 (objectness) + 6 (class logits)`

Detection class indices:

| Index | Name |
|---|---|
| 0 | barrel |
| 1 | pedestrian |
| 2 | stop_sign |
| 3 | unknown |
| 4 | tire |
| 5 | pothole |

ONNX numerical divergence vs PyTorch (measured at export, opset 17):
- All outputs: `max_abs_diff < 6e-6`, well within any practical threshold.

---

## 2. Detection Post-Processing

Run identically for each of the three FPN levels, then merge and NMS.

### 2a. Per-Level Decode

For a feature map of shape `(1, 11, feat_h, feat_w)` and input image `(H=640, W=1280)`:

```
stride_y = H / feat_h
stride_x = W / feat_w
```

Build a grid of cell centres (0-indexed, half-pixel offset):

```
cx[i, j] = (j + 0.5) * stride_x    # pixel x
cy[i, j] = (i + 0.5) * stride_y    # pixel y
```

Split the 11-channel output along channel axis:

```
box_raw   = relu(pred[0:4])   # (4, feat_h, feat_w) — LTRB, clamped ≥ 0
obj_logit = pred[4]           # (feat_h, feat_w)
cls_logit = pred[5:11]        # (6, feat_h, feat_w)
```

Decode LTRB offsets to image-space xyxy boxes (clamp to image boundary):

```
x1 = clamp(cx - box_raw[0] * stride_x,  0, W)
y1 = clamp(cy - box_raw[1] * stride_y,  0, H)
x2 = clamp(cx + box_raw[2] * stride_x,  0, W)
y2 = clamp(cy + box_raw[3] * stride_y,  0, H)
```

Compute final score (objectness × class confidence):

```
cls_probs          = softmax(cls_logit, axis=0)   # (6, feat_h, feat_w)
cls_score, cls_id  = max(cls_probs, axis=0)       # (feat_h, feat_w) each
score              = sigmoid(obj_logit) * cls_score
```

Keep only cells where `score >= score_threshold` (default: `0.20`).
If more than `topk=100` cells survive per level, keep the top-100 by score.

### 2b. Merge and NMS

Concatenate surviving boxes, scores, and labels from all three levels.
Apply class-agnostic NMS (`iou_threshold = 0.5`).
Keep at most `max_detections = 100`.

Output per image: list of `(x1, y1, x2, y2, score, class_id)` in pixel coordinates.

### 2c. Spatial Filters (post-NMS, apply before publishing to ROS)

These are not inside the model — apply them after NMS in your node:

**Robot body exclusion**
The bottom 15 % of the frame contains the robot's own camera mount and wiring.
Drop any detection whose centre row satisfies:

```
cy = (y1 + y2) / 2
if cy >= 0.85 * H:  discard
```

**Equirectangular edge exclusion** (optional but recommended)
Objects wrapping the left/right seam of the equirectangular image are distorted and often misdetected. Drop detections whose centre column is in the outer 5 %:

```
cx = (x1 + x2) / 2
if cx < 0.05 * W or cx > 0.95 * W:  discard
```

---

## 3. Segmentation Post-Processing

### 3a. Argmax

```
seg_mask = argmax(seg_logits[0], axis=0)   # shape (640, 1280), dtype uint8
```

Values: `0 = background`, `1 = lane`.

### 3b. Sky Zone Suppression

The top 35 % of the equirectangular frame is sky — lane predictions there are model noise.
Zero out before publishing:

```
sky_cutoff = int(0.35 * H)   # row 224 for H=640
seg_mask[:sky_cutoff, :] = 0
```

### 3c. Robot Body Zone Suppression

The bottom 15 % of the frame is the robot body. Same logic:

```
body_cutoff = int(0.85 * H)  # row 544 for H=640
seg_mask[body_cutoff:, :] = 0
```

### 3d. Optional: Morphological Cleanup

If the raw mask is noisy (speckling), apply a small closing then opening before publishing:

```
kernel = 5x5 rectangle
mask = morphologyEx(mask, MORPH_CLOSE, kernel)
mask = morphologyEx(mask, MORPH_OPEN,  kernel)
```

This is not applied in the Python pipeline — add it only if needed at runtime.

---

## 4. Recommended Thresholds

| Parameter | Default | Notes |
|---|---|---|
| `score_threshold` | `0.20` | From config. Lower = more recall, more FP |
| `nms_threshold` (IoU) | `0.50` | Standard. Lower = more aggressive merging |
| `max_detections` | `100` | Cap after NMS |
| `sky_zone_frac` | `0.35` | Suppress seg above row 224 (H=640) |
| `robot_body_frac` | `0.85` | Suppress seg and det below row 544 (H=640) |

These are tuneable constants — define them in your ROS 2 node as parameters so they can be adjusted without recompilation.

---

## 5. TensorRT / Engine File Notes

- Export from ONNX opset 17.
- Use **fixed shape** `(1, 3, 640, 1280)` at `trtexec` conversion time — do not use dynamic axes for the engine. This lets TRT fuse ops more aggressively.
- **FP16 mode** is recommended on Orin. Verify parity: run the engine and the ONNX side-by-side on the same input and check `max_abs_diff` on `seg_logits` (the upsampling path accumulates the most error in FP16).
- The ONNX was exported with `do_constant_folding=False` to work around an ORT bug with the shared DetectionHead stem. TensorRT has its own graph optimizer and is unaffected — the engine will still fold constants correctly.
- INT8 quantization: if you pursue it, use a calibration set of ~200 representative frames drawn from the same distribution as the training data. Avoid calibrating only on sunny/clear frames — the model sees a wide range of lighting.

---

## 6. ROS 2 Node Checklist

- [ ] Publish `seg_mask` as `sensor_msgs/Image` (encoding `mono8`, latched if useful for visualisation)
- [ ] Publish detections as `vision_msgs/Detection2DArray`; populate `results[i].hypothesis.class_id` with the string class name and `score`
- [ ] Expose all thresholds from section 4 as `rcl_interfaces/ParameterDescriptor` so they can be tuned with `ros2 param set` at runtime
- [ ] Time-stamp both outputs with the header stamp from the input camera frame, not the node's wall clock
- [ ] Run inference in a dedicated thread or use a `SingleThreadedExecutor` with the camera callback triggering inference directly — avoid queuing latency

---

*Reference implementation: `scripts/visualize_inference.py`, `models/detection_postprocess.py`*
*ONNX export: `scripts/export_onnx.py`*
*Divergence measurements: `outputs/onnx/export_summary.json`*
