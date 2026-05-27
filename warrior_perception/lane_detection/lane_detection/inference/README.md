# Inference and Deployment

Pipeline for taking a trained PyTorch checkpoint all the way to a production TensorRT engine, with both Python and C++ reference implementations.

```
scripts/export_onnx.py        →   model.onnx
inference/convert_to_engine.py →   model.engine  (TensorRT FP16/INT8)
inference/infer_python.py      →   Python TensorRT inference
inference/src/                 →   C++ TensorRT reference
```

---

## Step 1 — Export to ONNX

Run from the repo root:

```bash
python scripts/export_onnx.py \
  --checkpoint outputs/resnet18_run/best.pth \
  --config configs/multitask/multitask_resnet18.yaml \
  --output_path outputs/model.onnx
```

The script:
- Traces the model with a dummy input `(1, 3, 640, 1280)`
- Exports at opset 17 with dynamic batch size disabled (batch is fixed to 1 for Jetson deployment)
- Runs a numerical divergence check — expected max absolute difference vs PyTorch is < 1e-5

Outputs written alongside the `.onnx` file:
- `model_meta.json` — input/output shape specs, class names, normalisation parameters
- `export_summary.json` — divergence statistics and export timestamp

---

## Step 2 — Convert to TensorRT Engine

Requires TensorRT 10 installed (typically via Jetson SDK or pip on x86 with CUDA).

```bash
python inference/convert_to_engine.py \
  --onnx outputs/model.onnx \
  --engine outputs/model.engine \
  --precision fp16
```

For INT8 precision, supply a calibration dataset:

```bash
python inference/convert_to_engine.py \
  --onnx outputs/model.onnx \
  --engine outputs/model_int8.engine \
  --precision int8 \
  --calibration_dir path/to/calibration/frames
```

---

## Step 3 — Python Inference

Validates the TensorRT engine and provides a reference for integration:

```bash
python inference/infer_python.py \
  --engine outputs/model.engine \
  --image path/to/frame.jpg
```

Outputs a JSON blob with decoded detections and the segmentation mask as a PNG.

---

## Step 4 — C++ Deployment

The `inference/src/` directory contains a CMake-based TensorRT inference library targeting Jetson Orin.

```bash
mkdir inference/build && cd inference/build
cmake .. -DTENSORRT_ROOT=/usr/local/tensorrt
make -j4
./multitask_infer --engine ../../outputs/model.engine --image ../../path/to/frame.jpg
```

See `inference/CMakeLists.txt` for dependency paths.

---

## Model Inputs and Outputs

### Input

| Field     | Value                                 |
| --------- | ------------------------------------- |
| Name      | `input`                               |
| Shape     | `(1, 3, 640, 1280)` — NCHW, batch = 1 |
| Dtype     | `float32`                             |
| Channels  | RGB (not BGR)                         |
| Normalisation | ImageNet: subtract `[0.485, 0.456, 0.406]`, divide by `[0.229, 0.224, 0.225]` |

Preprocessing order:
1. Resize raw frame to 1280 × 640 (W × H) with bilinear interpolation.
2. Convert BGR → RGB.
3. Cast to `float32`, divide by 255.
4. Subtract mean, divide by std per channel.
5. Transpose HWC → CHW, add batch dimension → `(1, 3, 640, 1280)`.

### Outputs

| Name         | Shape              | Description                                    |
| ------------ | ------------------ | ---------------------------------------------- |
| `det_p3`     | `(1, 11, 80, 160)` | FCOS predictions — stride 8, small objects     |
| `det_p4`     | `(1, 11, 40, 80)`  | FCOS predictions — stride 16, medium objects   |
| `det_p5`     | `(1, 11, 20, 40)`  | FCOS predictions — stride 32, large objects    |
| `seg_logits` | `(1, 2, 640, 1280)`| Lane segmentation logits — full resolution     |

`pred_dim = 11 = 4 (LTRB offsets) + 1 (objectness) + 6 (class logits)`

### Detection Post-Processing

For each FPN level:
1. Apply sigmoid to objectness and class logits.
2. Threshold objectness by `score_threshold` (default 0.20).
3. Decode LTRB offsets from each cell centre to absolute pixel coordinates.
4. Compute final score = objectness × class confidence.
5. Merge predictions from P3/P4/P5.
6. Apply NMS with `iou_threshold = 0.5`, keep top `max_detections = 100`.

### Segmentation Post-Processing

```
argmax(seg_logits, dim=1)  →  class map (0 = background, 1 = lane)
```

Optionally apply the ROI mask to zero-out predictions outside the valid driving band.

See [`docs/postprocessing.md`](../docs/postprocessing.md) for the complete C++ / ROS 2 implementation reference.
