from __future__ import annotations

import argparse
import json
from pathlib import Path


def read_jsonl(path: str | Path) -> list[dict]:
    rows = []
    with Path(path).open("r", encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                rows.append(json.loads(line))
    return rows


def filter_records(args: argparse.Namespace) -> None:
    rows = read_jsonl(args.input_records)
    kept = []
    counts = {"input": len(rows), "low_confidence": 0, "too_few_pixels": 0, "too_many_pixels": 0, "empty": 0, "kept": 0}
    for row in rows:
        pixels = int(row.get("num_lane_pixels", 0))
        confidence = float(row.get("seg_confidence", 0.0))
        if confidence < args.min_confidence:
            counts["low_confidence"] += 1
            continue
        if args.drop_empty_masks and pixels <= 0:
            counts["empty"] += 1
            continue
        if pixels < args.min_lane_pixels:
            counts["too_few_pixels"] += 1
            continue
        if args.max_lane_pixels > 0 and pixels > args.max_lane_pixels:
            counts["too_many_pixels"] += 1
            continue
        kept.append(row)
    counts["kept"] = len(kept)

    output = Path(args.output_records)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as handle:
        for row in kept:
            handle.write(json.dumps(row) + "\n")
    summary = output.with_suffix(".summary.json")
    summary.write_text(json.dumps(counts, indent=2), encoding="utf-8")
    print(json.dumps(counts, indent=2))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Filter weak lane pseudo-label records.")
    parser.add_argument("--input_records", required=True)
    parser.add_argument("--output_records", required=True)
    parser.add_argument("--min_confidence", type=float, default=0.6)
    parser.add_argument("--min_lane_pixels", type=int, default=1)
    parser.add_argument("--max_lane_pixels", type=int, default=0)
    parser.add_argument("--drop_empty_masks", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    filter_records(parse_args())
