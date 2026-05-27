# Live Inference Demo

Lightweight OpenCV + PyTorch live inference app for a webcam, 360 USB camera, video file, or stream URL. It shows live performance stats, overlays optional model predictions, saves every Nth rendered frame, writes per-frame CSV metrics, and writes a summary JSON on exit.

## Project Layout

```text
live_inference_demo/
  app.py
  config.yaml
  requirements.txt
  README.md
  adapters/
    __init__.py
    base_adapter.py
    multitask_adapter.py
  utils/
    __init__.py
    video_source.py
    timer.py
    stats.py
    overlay.py
    saver.py
    logging_utils.py
  outputs/
    .gitkeep
```

## Setup

```bash
cd live_inference_demo
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

## Run From Camera

```bash
python app.py --camera-index 0
```

## Run From Video File

```bash
python app.py --video path/to/demo.mp4
```

## Run From Stream URL

```bash
python app.py --stream-url rtsp://user:pass@host:554/stream
```

## Common Overrides

Change save cadence:

```bash
python app.py --camera-index 0 --save-every-n 2
```

Change inference resolution:

```bash
python app.py --camera-index 0 --infer-width 960 --infer-height 480
```

Change crop mode for 360 input:

```bash
python app.py --camera-index 0 --crop-mode lower_band
```

## Keyboard Controls

- `q`: quit cleanly
- `s`: force-save current displayed frame
- `p`: pause/resume
- `o`: toggle overlay on/off

## Config Defaults

Edit [`config.yaml`](/c:/Users/jaide/robotics/mutli-task-perception/live_inference_demo/config.yaml) for default source, display size, inference size, crop mode, save cadence, checkpoint path, and rolling stats window.

## Plug In Your Model

The app talks to the model only through the adapter interface in [`adapters/base_adapter.py`](/c:/Users/jaide/robotics/mutli-task-perception/live_inference_demo/adapters/base_adapter.py).

Update these methods in [`adapters/multitask_adapter.py`](/c:/Users/jaide/robotics/mutli-task-perception/live_inference_demo/adapters/multitask_adapter.py):

1. `load_model()`
   Point this at your model constructor and checkpoint loading logic.

2. `preprocess(frame_bgr)`
   Replace the generic RGB-to-tensor conversion if your model needs normalization, channel changes, batching rules, or custom transforms.

3. `postprocess(raw_output, original_frame)`
   Translate your model output into the generic dictionary used by the app:
   - `boxes`: `[N, 4]` in `xyxy`
   - `scores`: `[N]`
   - `labels`: `[N]`
   - `mask`: optional segmentation output

You usually do not need to edit [`app.py`](/c:/Users/jaide/robotics/mutli-task-perception/live_inference_demo/app.py) when swapping models.

## Expected Output Files

Each run creates a folder under [`outputs/`](/c:/Users/jaide/robotics/mutli-task-perception/live_inference_demo/outputs) like:

```text
outputs/run_YYYYMMDD_HHMMSS/
  metrics.csv
  summary.json
  frames/
    2026-04-10T17-00-00-123_frame_000002.jpg
```

## Troubleshooting

- If the wrong camera opens, try `--camera-index 1`, `--camera-index 2`, and so on.
- If CUDA is unavailable, the app automatically falls back to CPU.
- If your checkpoint loads but no predictions appear, update the TODO blocks in [`adapters/multitask_adapter.py`](/c:/Users/jaide/robotics/mutli-task-perception/live_inference_demo/adapters/multitask_adapter.py) so the adapter understands your checkpoint and output structure.
- If a stream disconnects temporarily, the app retries frame reads a few times before exiting cleanly.
