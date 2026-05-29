from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import cv2


def extract_frames(args: argparse.Namespace) -> None:
    output_dir = Path(args.output_dir)
    frames_dir = output_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)
    metadata_path = output_dir / "metadata.jsonl"

    cap = cv2.VideoCapture(str(args.input_video))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video: {args.input_video}")

    source_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    step = max(1, round(source_fps / float(args.fps))) if args.fps else 1
    frame_index = 0
    saved = 0

    with metadata_path.open("w", encoding="utf-8") as handle:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if frame_index % step != 0:
                frame_index += 1
                continue
            if args.resize_h and args.resize_w:
                frame = cv2.resize(frame, (int(args.resize_w), int(args.resize_h)), interpolation=cv2.INTER_AREA)
            height, width = frame.shape[:2]
            frame_id = f"frame_{saved:06d}"
            image_path = frames_dir / f"{frame_id}.{args.img_ext.lstrip('.')}"
            cv2.imwrite(str(image_path), frame)
            row: dict[str, Any] = {
                "frame_id": frame_id,
                "image_path": str(image_path),
                "source_video": str(args.input_video),
                "frame_index": frame_index,
                "timestamp_sec": frame_index / source_fps,
                "height": height,
                "width": width,
            }
            handle.write(json.dumps(row) + "\n")
            saved += 1
            frame_index += 1
            if args.max_frames and saved >= int(args.max_frames):
                break
    cap.release()
    print(f"saved_frames={saved} metadata={metadata_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Extract equirectangular frames without unwarping.")
    parser.add_argument("--input_video", required=True)
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument("--max_frames", type=int, default=0)
    parser.add_argument("--resize_h", type=int, default=0)
    parser.add_argument("--resize_w", type=int, default=0)
    parser.add_argument("--img_ext", default="jpg")
    return parser.parse_args()


if __name__ == "__main__":
    extract_frames(parse_args())
