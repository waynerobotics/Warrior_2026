# Perception Stack — Jetson Setup Guide

End-to-end bring-up for the Warrior perception pipeline on Jetson Orin.

**Pipeline:**
```
Insta360 (USB-C UVC)
  └─ insta360_node  →  camera/image_raw
                              └─ multitask_node  →  ai/seg_mask
                                                 →  ai/detections
```

---

## Prerequisites

### Hardware
- NVIDIA Jetson Orin (AGX or NX)
- Insta360 X4 or X5 camera
- USB-C cable (data-capable, not charge-only)

### Software versions
| Component | Required |
|---|---|
| JetPack | 6.x (TensorRT 10.x, CUDA 12.x) |
| ROS 2 | Humble |
| OpenCV | 4.x (ships with JetPack or `ros-humble-opencv`) |
| vision_msgs | `ros-humble-vision-msgs` |
| cv_bridge | `ros-humble-cv-bridge` |

Install ROS 2 deps if not already present:
```bash
sudo apt install \
  ros-humble-vision-msgs \
  ros-humble-cv-bridge \
  ros-humble-image-transport
```

---

## Step 0 — Unitree L2 LiDAR network

The L2 streams over Ethernet and expects a host NIC on `192.168.1.0/24`.
One-time NetworkManager setup:

```bash
sudo nmcli con modify "<your-eth-conn>" \
    connection.id unitree-l2 \
    ipv4.method manual \
    ipv4.addresses 192.168.1.2/24 \
    ipv4.gateway "" ipv4.never-default yes \
    ipv6.method ignore connection.autoconnect yes
sudo nmcli con up unitree-l2
```

Not needed if you are only bringing up the camera + inference path.

---

## Step 1 — Put the Insta360 into webcam mode

1. Power on the camera.
2. Swipe down → **Connections** → **USB Mode** → select **Webcam**.
3. Plug in the USB-C cable to the Jetson.
4. Verify it enumerates:
   ```bash
   v4l2-ctl --list-devices
   # Should show something like:
   # Insta360 X4 (usb-...-..):
   #         /dev/video0
   #         /dev/video1
   ```
5. Check the VID matches `0x2e1a`:
   ```bash
   lsusb | grep -i insta
   # Bus 002 Device 003: ID 2e1a:xxxx Insta360 ...
   ```
   If the VID is different from `2e1a`, update `INSTA360_VID` in
   `insta360_camera/src/insta360_node.cpp` before building.

---

## Step 2 — Convert the TRT engine to FP16

The repo ships `neural_engine/inference/resnet_fp32.engine` built on a desktop.
**You must rebuild it on the Jetson** — TRT engines are not portable across GPU
architectures. Rebuild as FP16 for best Jetson throughput (~2× faster than FP32).

### 2a. Locate the ONNX model
```bash
ls warrior_perception/neural_engine/inference/resnet18.onnx
```

### 2b. Build the FP16 engine with `trtexec`
```bash
/usr/src/tensorrt/bin/trtexec \
  --onnx=warrior_perception/neural_engine/inference/resnet18.onnx \
  --fp16 \
  --saveEngine=/opt/warrior/models/resnet18_fp16.engine \
  --minShapes=input:1x3x640x1280 \
  --optShapes=input:1x3x640x1280 \
  --maxShapes=input:1x3x640x1280 \
  --workspace=2048
```

`trtexec` is at `/usr/src/tensorrt/bin/trtexec` on JetPack 6. The build takes
3–10 minutes. The output engine is specific to this Jetson — do not copy it to
another machine.

Expected output at the end:
```
[I] Throughput: ... qps
[I] mean: ... ms
```

Typical ResNet18 640×1280 on Orin NX with FP16: **~6–12 ms/frame**.

---

## Step 3 — Build the ROS 2 packages

```bash
cd ~/Warrior_2026

colcon build --packages-select \
  insta360_camera \
  neural_engine \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

If TensorRT is not on the default library path (unlikely on JetPack), pass:
```bash
--cmake-args -DCMAKE_BUILD_TYPE=Release -DTENSORRT_ROOT=/usr/local/tensorrt
```

Verify both nodes are installed:
```bash
ros2 pkg executables insta360_camera
# insta360_camera insta360_node

ros2 pkg executables neural_engine
# neural_engine multitask_node
# neural_engine seg_viz_node
```

---

## Step 4 — Configure

Edit `warrior_perception/neural_engine/config/multitask.yaml`:
```yaml
multitask_node:
  ros__parameters:
    engine_path: /opt/warrior/models/resnet18_fp16.engine   # path from Step 2b
    image_topic: camera/image_raw
    seg_topic: ai/seg_mask
    detections_topic: ai/detections
    score_threshold: 0.20
    nms_threshold: 0.50
    apply_morph: false
```

`insta360_camera/config/insta360.yaml` defaults are fine for most cases:
```yaml
insta360_node:
  ros__parameters:
    device: ""               # empty = auto-discover by USB VID
    frame_id: insta360_optical_frame
    width: 1920
    height: 1080
    fps: 30.0
```

If you want to pin a specific device (e.g. `/dev/video0`), set `device` explicitly.
See CLAUDE.md Rule 1 before doing this — port numbers change on replug.

---

## Step 5 — Launch

### Terminal 1 — camera
```bash
source ~/Warrior_2026/install/setup.bash
ros2 launch insta360_camera insta360.launch.py
```

Expected output:
```
[insta360_node]: insta360_node streaming /dev/video0 @ 1920x1080/30 fps
```

### Terminal 2 — inference
```bash
source ~/Warrior_2026/install/setup.bash
ros2 launch neural_engine multitask.launch.py \
  engine_path:=/opt/warrior/models/resnet18_fp16.engine
```

Expected output:
```
=== TensorRT IO Tensors ===
[0] input
[1] det_p3
[2] det_p4
[3] det_p5
[4] seg_logits
===========================
[multitask_node]: multitask_node ready (engine=..., img=camera/image_raw, ...)
```

---

## Step 6 — Verify the pipeline

### Check topics are publishing
```bash
ros2 topic list
# Should include:
#   /camera/image_raw
#   /ai/seg_mask
#   /ai/detections

ros2 topic hz /camera/image_raw      # should be ~30 Hz
ros2 topic hz /ai/seg_mask           # should be ~30 Hz (or slightly less)
ros2 topic hz /ai/detections         # same
```

### Inspect detections
```bash
ros2 topic echo /ai/detections --once
```

Each detection has:
- `bbox.center.x / .y` — box centre in pixels (at 640×1280 model scale)
- `bbox.size_x / .size_y` — width and height
- `results[0].hypothesis.class_id` — one of: `barrel pedestrian stop_sign unknown tire pothole`
- `results[0].hypothesis.score` — confidence [0–1]

### Visualise in RViz2
```bash
rviz2
```
Add displays:
- **Image** → topic `/camera/image_raw`
- **Image** → topic `/ai/seg_mask` (mono8 — lane mask, white = lane)
- No native Detection2D display in base RViz2; use `image_view` or a custom overlay node.

---

## Performance notes

| Stage | Typical on Orin NX |
|---|---|
| V4L2 grab (1920×1080) | ~2 ms |
| ROS publish / subscribe | <1 ms (shared memory DDS, localhost) |
| CPU preprocess (resize + normalize) | ~8–15 ms |
| TRT FP16 inference (ResNet18 640×1280) | ~6–12 ms |
| Post-process (FCOS decode + NMS) | ~2 ms |
| **Total wall time per frame** | **~18–30 ms** |

At 30 fps you have 33 ms per frame. If the inference node falls behind it will
drop frames rather than queue them — the subscription uses `SensorDataQoS` with
depth 1, so it always processes the latest image.

### Optional: improve preprocessing throughput
The resize + normalize runs on CPU. If you need lower latency, replace the
`preprocess()` function in `neural_engine/src/multitask_node.cpp` with a
CUDA kernel or Jetson VPI pipeline. This is not needed unless the wall time
above consistently exceeds 33 ms.

---

## Troubleshooting

### Camera not found at startup
```
[insta360_node]: no Insta360 UVC device found in /sys/class/video4linux
```
- Check webcam mode is selected on the camera (Step 1).
- Check `lsusb | grep 2e1a` — if the VID differs, update `INSTA360_VID` in the source.
- Try `v4l2-ctl --list-devices` to confirm the device enumerates.
- The node retries on every timer tick — plug in the camera and it will come up without restarting.

### Engine fails to load
```
[multitask_node]: Failed to deserialize TensorRT engine
```
- The engine was built on a different GPU. Rebuild with `trtexec` on this Jetson (Step 2b).
- Check the engine path in `multitask.yaml` is correct.

### Status 2 / SPARK MAX unrelated faults during perception bring-up
The inference node has no dependency on the drive stack. Run it standalone — you do not need the SPARK MAXes or Arduinos powered.

### Low detection confidence / no detections
- Confirm the camera is outputting an equirectangular (360°) image, not a cropped view.
- Confirm `score_threshold` is not set too high. Default is 0.20.
- Check `ros2 topic echo /ai/seg_mask --once` to confirm the model is running (non-zero data).

### Inference falls below 30 fps
1. Confirm the engine is FP16 — check `trtexec` output from Step 2b.
2. Check CPU frequency scaling: `sudo jetson_clocks` to lock clocks.
3. Check thermal throttling: `tegrastats | grep -i cpu`.
