from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def build(args: argparse.Namespace) -> None:
    mask = np.zeros((int(args.image_h), int(args.image_w)), dtype=np.uint8)
    if args.mode == "manual_band":
        top = max(0, int(args.top))
        bottom = min(mask.shape[0], int(args.bottom) if args.bottom >= 0 else mask.shape[0])
        left = max(0, int(args.left))
        right = min(mask.shape[1], int(args.right) if args.right >= 0 else mask.shape[1])
        mask[top:bottom, left:right] = 255
    elif args.mode == "polygon":
        if not args.polygon_json:
            raise ValueError("--polygon_json is required for polygon mode")
        data = json.loads(Path(args.polygon_json).read_text(encoding="utf-8"))
        points = data.get("points", data)
        pts = np.asarray(points, dtype=np.int32).reshape((-1, 1, 2))
        cv2.fillPoly(mask, [pts], 255)
    else:
        raise ValueError(args.mode)
    output = Path(args.output_mask)
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), mask)
    print(f"roi_mask={output} valid_fraction={float((mask > 0).mean()):.4f}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build equirectangular ROI masks for lane supervision.")
    parser.add_argument("--image_h", type=int, required=True)
    parser.add_argument("--image_w", type=int, required=True)
    parser.add_argument("--mode", choices=["manual_band", "polygon"], default="manual_band")
    parser.add_argument("--top", type=int, default=0)
    parser.add_argument("--bottom", type=int, default=-1)
    parser.add_argument("--left", type=int, default=0)
    parser.add_argument("--right", type=int, default=-1)
    parser.add_argument("--polygon_json", default="")
    parser.add_argument("--output_mask", required=True)
    return parser.parse_args()


if __name__ == "__main__":
    build(parse_args())
