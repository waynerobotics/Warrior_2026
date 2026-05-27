"""Live inference demo app for camera, video, or stream sources."""

from __future__ import annotations

import argparse
import logging
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch
import yaml

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from adapters import MultiTaskAdapter
from utils.logging_utils import setup_logging
from utils.overlay import draw_stats_block
from utils.saver import RunSaver
from utils.stats import FrameMetrics, StatsTracker
from utils.timer import measure_time
from utils.video_source import VideoSource


LOGGER = logging.getLogger(__name__)


def parse_args() -> argparse.Namespace:
    """Parse CLI overrides for the live inference demo."""

    parser = argparse.ArgumentParser(description="Live inference demo for camera, video, or stream input.")
    parser.add_argument("--config", default=str(CURRENT_DIR / "config.yaml"), help="Path to YAML config file.")
    parser.add_argument("--camera-index", type=int, default=None, help="Camera index to open.")
    parser.add_argument("--video", default=None, help="Video file path to open.")
    parser.add_argument("--stream-url", default=None, help="RTSP or stream URL to open.")
    parser.add_argument("--save-every-n", type=int, default=None, help="Save every Nth processed frame.")
    parser.add_argument("--infer-width", type=int, default=None, help="Inference width override.")
    parser.add_argument("--infer-height", type=int, default=None, help="Inference height override.")
    parser.add_argument("--display-width", type=int, default=None, help="Display width override.")
    parser.add_argument("--display-height", type=int, default=None, help="Display height override.")
    parser.add_argument("--crop-mode", choices=["full_frame", "center_crop", "lower_band"], default=None)
    parser.add_argument("--run-every-n-frames", type=int, default=None, help="Run inference every K frames.")
    parser.add_argument("--checkpoint", default=None, help="Model checkpoint override.")
    return parser.parse_args()


def load_config(config_path: str | Path) -> dict[str, Any]:
    """Load YAML config into a dictionary."""

    with Path(config_path).open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"Expected config mapping in {config_path}")
    return data


def apply_cli_overrides(config: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    """Apply runtime CLI overrides onto the config dictionary."""

    if args.camera_index is not None:
        config["source"]["type"] = "camera"
        config["source"]["camera_index"] = args.camera_index
        config["source"]["path"] = ""
    if args.video:
        config["source"]["type"] = "video"
        config["source"]["path"] = args.video
    if args.stream_url:
        config["source"]["type"] = "stream"
        config["source"]["path"] = args.stream_url
    if args.save_every_n is not None:
        config["saving"]["save_every_n_frames"] = max(1, args.save_every_n)
    if args.infer_width is not None:
        config["inference"]["width"] = args.infer_width
    if args.infer_height is not None:
        config["inference"]["height"] = args.infer_height
    if args.display_width is not None:
        config["display"]["width"] = args.display_width
    if args.display_height is not None:
        config["display"]["height"] = args.display_height
    if args.crop_mode is not None:
        config["inference"]["crop_mode"] = args.crop_mode
    if args.run_every_n_frames is not None:
        config["inference"]["run_every_n_frames"] = max(1, args.run_every_n_frames)
    if args.checkpoint is not None:
        config["model"]["checkpoint_path"] = args.checkpoint
    return config


def resolve_device(device_name: str) -> torch.device:
    """Resolve configured device, falling back safely from CUDA to CPU."""

    if device_name == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device_name == "cuda" and not torch.cuda.is_available():
        LOGGER.warning("CUDA requested but not available. Falling back to CPU.")
        return torch.device("cpu")
    return torch.device(device_name)


def resolve_source(config: dict[str, Any]) -> tuple[Any, dict[str, Any]]:
    """Resolve configured input source into an OpenCV-compatible source."""

    source_cfg = config["source"]
    source_type = str(source_cfg.get("type", "camera"))
    if source_type == "camera":
        source = int(source_cfg.get("camera_index", 0))
    else:
        source = str(source_cfg.get("path", ""))
    return source, {"type": source_type, "value": source}


def get_crop_rect(frame_bgr: np.ndarray, crop_mode: str) -> tuple[int, int, int, int]:
    """Return crop rectangle as ``x1, y1, x2, y2`` in original frame coordinates."""

    height, width = frame_bgr.shape[:2]
    if crop_mode == "full_frame":
        return 0, 0, width, height
    if crop_mode == "center_crop":
        crop_width = max(1, int(width * 0.6))
        x1 = (width - crop_width) // 2
        return x1, 0, x1 + crop_width, height
    if crop_mode == "lower_band":
        y1 = int(height * 0.5)
        return 0, y1, width, height
    raise ValueError(f"Unsupported crop mode: {crop_mode}")


def prepare_inference_frame(frame_bgr: np.ndarray, config: dict[str, Any]) -> tuple[np.ndarray, dict[str, Any]]:
    """Crop and resize frame for inference, returning transform metadata."""

    crop_mode = str(config["inference"].get("crop_mode", "full_frame"))
    x1, y1, x2, y2 = get_crop_rect(frame_bgr, crop_mode)
    cropped = frame_bgr[y1:y2, x1:x2]
    infer_width = int(config["inference"]["width"])
    infer_height = int(config["inference"]["height"])
    resized = cv2.resize(cropped, (infer_width, infer_height), interpolation=cv2.INTER_LINEAR)
    return resized, {"crop_rect": (x1, y1, x2, y2), "original_shape": frame_bgr.shape[:2], "inference_shape": resized.shape[:2]}


def prepare_display_frame(frame_bgr: np.ndarray, config: dict[str, Any]) -> np.ndarray:
    """Resize frame for display without altering the original capture."""

    display_width = int(config["display"]["width"])
    display_height = int(config["display"]["height"])
    return cv2.resize(frame_bgr, (display_width, display_height), interpolation=cv2.INTER_LINEAR)


def should_save_frame(frame_idx: int, save_every_n_frames: int) -> bool:
    """Return True when the current frame should be saved by cadence."""

    return frame_idx % max(1, save_every_n_frames) == 0


def build_live_stats(
    metrics: FrameMetrics,
    resolution: tuple[int, int],
    device: torch.device,
    saved_count: int,
) -> dict[str, Any]:
    """Build the overlay stats payload."""

    return {
        "current_fps": metrics.instantaneous_fps,
        "rolling_fps": metrics.rolling_fps,
        "capture_ms": metrics.capture_ms,
        "preprocess_ms": metrics.preprocess_ms,
        "inference_ms": metrics.inference_ms,
        "postprocess_ms": metrics.postprocess_ms,
        "total_ms": metrics.total_ms,
        "frame_idx": metrics.frame_idx,
        "resolution": f"{resolution[0]}x{resolution[1]}",
        "device": str(device).upper(),
        "saved_count": saved_count,
    }


def map_predictions_to_original(
    predictions: dict[str, Any],
    transform: dict[str, Any],
) -> dict[str, Any]:
    """Map prediction coordinates from inference space back to original frame space."""

    x1, y1, x2, y2 = transform["crop_rect"]
    crop_width = max(1, x2 - x1)
    crop_height = max(1, y2 - y1)
    infer_height, infer_width = transform["inference_shape"]
    scale_x = crop_width / max(1, infer_width)
    scale_y = crop_height / max(1, infer_height)
    original_height, original_width = transform["original_shape"]

    mapped = {
        "boxes": [],
        "scores": list(predictions.get("scores", [])),
        "labels": list(predictions.get("labels", [])),
        "mask": None,
    }

    for box in predictions.get("boxes", []):
        bx1, by1, bx2, by2 = box
        mapped["boxes"].append(
            [
                bx1 * scale_x + x1,
                by1 * scale_y + y1,
                bx2 * scale_x + x1,
                by2 * scale_y + y1,
            ]
        )

    mask = predictions.get("mask")
    if isinstance(mask, np.ndarray):
        crop_mask = cv2.resize(mask, (crop_width, crop_height), interpolation=cv2.INTER_NEAREST)
        if crop_mask.ndim == 2:
            full_mask = np.zeros((original_height, original_width), dtype=crop_mask.dtype)
            full_mask[y1:y2, x1:x2] = crop_mask
        else:
            full_mask = np.zeros((original_height, original_width, crop_mask.shape[2]), dtype=crop_mask.dtype)
            full_mask[y1:y2, x1:x2] = crop_mask
        mapped["mask"] = full_mask

    return mapped


def handle_keypress(key: int, paused: bool, overlay_enabled: bool) -> tuple[bool, bool, bool, bool]:
    """Interpret keyboard commands.

    Returns:
        quit_requested, paused, overlay_enabled, force_save
    """

    force_save = False
    quit_requested = False

    if key == ord("q"):
        quit_requested = True
    elif key == ord("p"):
        paused = not paused
    elif key == ord("o"):
        overlay_enabled = not overlay_enabled
    elif key == ord("s"):
        force_save = True

    return quit_requested, paused, overlay_enabled, force_save


def main() -> None:
    """Run the live inference demo loop."""

    args = parse_args()
    setup_logging()
    config = apply_cli_overrides(load_config(args.config), args)
    device = resolve_device(str(config["model"].get("device", "auto")))
    source, source_info = resolve_source(config)

    adapter = MultiTaskAdapter(config=config, device=device)
    stats = StatsTracker(rolling_window=int(config["stats"].get("rolling_window", 30)))

    output_dir_cfg = Path(str(config["saving"].get("output_dir", "outputs")))
    output_dir = output_dir_cfg if output_dir_cfg.is_absolute() else CURRENT_DIR / output_dir_cfg
    saver = RunSaver(
        output_dir=output_dir,
        saving_enabled=bool(config["saving"].get("enabled", True)),
    )

    LOGGER.info("Opening source: %s", source_info)
    LOGGER.info("Using device: %s", device)

    video_source: VideoSource | None = None
    window_name = "Live Inference Demo"
    paused = False
    overlay_enabled = True
    last_rendered_frame: np.ndarray | None = None
    last_predictions: dict[str, Any] = {"boxes": [], "scores": [], "labels": [], "mask": None}
    last_inference_ms = 0.0
    run_every_n_frames = max(1, int(config["inference"].get("run_every_n_frames", 1)))
    save_every_n_frames = max(1, int(config["saving"].get("save_every_n_frames", 2)))
    frame_idx = 0

    try:
        video_source = VideoSource(source=source)

        if bool(config["display"].get("show_window", True)):
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        while True:
            if paused and last_rendered_frame is not None:
                paused_frame = last_rendered_frame.copy()
                cv2.putText(paused_frame, "PAUSED", (20, paused_frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)
                if bool(config["display"].get("show_window", True)):
                    cv2.imshow(window_name, paused_frame)
                    key = cv2.waitKey(50) & 0xFF
                    quit_requested, paused, overlay_enabled, force_save = handle_keypress(key, paused, overlay_enabled)
                    if force_save:
                        saved = saver.save_frame(paused_frame, frame_idx, datetime.now().isoformat(timespec="milliseconds"))
                        if saved is not None:
                            stats.saved_frames += 1
                    if quit_requested:
                        break
                continue

            with measure_time() as capture_timer:
                video_frame = video_source.read()
            capture_ms = capture_timer.duration_ms

            if not video_frame.ok or video_frame.frame is None:
                LOGGER.error("Stream unavailable after retries. Exiting cleanly.")
                break

            raw_frame = video_frame.frame
            frame_idx += 1

            run_inference = (frame_idx - 1) % run_every_n_frames == 0
            if run_inference:
                with measure_time() as preprocess_timer:
                    inference_frame, transform = prepare_inference_frame(raw_frame, config)
                    model_input = adapter.preprocess(inference_frame)
                preprocess_ms = preprocess_timer.duration_ms
                with measure_time() as inference_timer:
                    raw_output = adapter.infer(model_input)
                with measure_time() as postprocess_timer:
                    inference_predictions = adapter.postprocess(raw_output, inference_frame)
                    last_predictions = map_predictions_to_original(inference_predictions, transform)
                last_inference_ms = inference_timer.duration_ms
                postprocess_ms = postprocess_timer.duration_ms
            else:
                with measure_time() as preprocess_timer:
                    _, _ = prepare_inference_frame(raw_frame, config)
                preprocess_ms = preprocess_timer.duration_ms
                last_inference_ms = 0.0
                postprocess_ms = 0.0

            display_frame = prepare_display_frame(raw_frame, config)
            display_predictions = last_predictions
            if display_frame.shape[:2] != raw_frame.shape[:2]:
                scale_x = display_frame.shape[1] / raw_frame.shape[1]
                scale_y = display_frame.shape[0] / raw_frame.shape[0]
                display_predictions = rescale_predictions(last_predictions, scale_x, scale_y, display_frame.shape[:2])

            total_ms = capture_ms + preprocess_ms + last_inference_ms + postprocess_ms
            instantaneous_fps = 1000.0 / total_ms if total_ms > 0 else 0.0
            rolling_fps_value = (
                (sum(stats.fps_window) + instantaneous_fps) / (len(stats.fps_window) + 1)
                if stats.fps_window
                else instantaneous_fps
            )

            save_by_cadence = should_save_frame(frame_idx, save_every_n_frames)
            frame_saved = False

            metrics = FrameMetrics(
                frame_idx=frame_idx,
                timestamp=video_frame.timestamp,
                capture_ms=capture_ms,
                preprocess_ms=preprocess_ms,
                inference_ms=last_inference_ms,
                postprocess_ms=postprocess_ms,
                total_ms=total_ms,
                instantaneous_fps=instantaneous_fps,
                rolling_fps=rolling_fps_value,
                saved_flag=0,
            )

            metrics.saved_flag = int(save_by_cadence)
            stats.update(metrics)
            metrics.rolling_fps = stats.rolling_fps()

            output_frame = adapter.draw_predictions(display_frame, display_predictions) if overlay_enabled else display_frame.copy()
            if overlay_enabled:
                stats_payload = build_live_stats(
                    metrics=metrics,
                    resolution=(display_frame.shape[1], display_frame.shape[0]),
                    device=device,
                    saved_count=stats.saved_frames,
                )
                output_frame = draw_stats_block(output_frame, stats_payload)

            if save_by_cadence:
                frame_saved = saver.save_frame(output_frame, frame_idx, video_frame.timestamp) is not None

            metrics.saved_flag = int(frame_saved)
            saver.append_metrics(metrics)
            last_rendered_frame = output_frame

            if bool(config["display"].get("show_window", True)):
                cv2.imshow(window_name, output_frame)
                key = cv2.waitKey(1) & 0xFF
                quit_requested, paused, overlay_enabled, force_save = handle_keypress(key, paused, overlay_enabled)
                if force_save:
                    saved = saver.save_frame(output_frame, frame_idx, datetime.now().isoformat(timespec="milliseconds"))
                    if saved is not None:
                        stats.saved_frames += 1
                if quit_requested:
                    break

    except KeyboardInterrupt:
        LOGGER.info("Interrupted by user. Shutting down.")
    except Exception as exc:
        LOGGER.exception("Live inference demo failed: %s", exc)
        raise
    finally:
        if video_source is not None:
            video_source.release()
        saver.save_summary(
            stats.build_summary(
                device=str(device).upper(),
                source_info=source_info,
                model_info=getattr(adapter, "model_info", None),
            )
        )
        saver.close()
        cv2.destroyAllWindows()


def rescale_predictions(
    predictions: dict[str, Any],
    scale_x: float,
    scale_y: float,
    target_shape: tuple[int, int],
) -> dict[str, Any]:
    """Rescale generic prediction structures for display resolution."""

    scaled = {
        "boxes": [],
        "scores": list(predictions.get("scores", [])),
        "labels": list(predictions.get("labels", [])),
        "mask": predictions.get("mask"),
    }

    for box in predictions.get("boxes", []):
        x1, y1, x2, y2 = box
        scaled["boxes"].append([x1 * scale_x, y1 * scale_y, x2 * scale_x, y2 * scale_y])

    mask = predictions.get("mask")
    if isinstance(mask, np.ndarray) and mask.shape[:2] != target_shape:
        scaled["mask"] = cv2.resize(mask, (target_shape[1], target_shape[0]), interpolation=cv2.INTER_NEAREST)

    return scaled


if __name__ == "__main__":
    main()
