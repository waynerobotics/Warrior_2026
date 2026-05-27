from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any


def read_jsonl(path: str | Path) -> list[dict[str, Any]]:
    rows = []
    with Path(path).open("r", encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                rows.append(json.loads(line))
    return rows


def frame_id(row: dict[str, Any]) -> str:
    return str(row.get("frame_id") or Path(str(row.get("image_path", row.get("image", "")))).stem)


def select_diverse(rows: list[dict[str, Any]], count: int) -> list[dict[str, Any]]:
    if count <= 0 or len(rows) <= count:
        return rows
    ranked = sorted(rows, key=lambda row: int(row.get("num_lane_pixels", 0)), reverse=True)
    buckets = [
        ("high_prediction", ranked[: max(1, len(ranked) // 3)]),
        ("medium_prediction", ranked[max(1, len(ranked) // 3) : max(2, 2 * len(ranked) // 3)]),
        ("low_prediction", ranked[max(2, 2 * len(ranked) // 3) :]),
    ]
    selected: list[dict[str, Any]] = []
    per_bucket = max(1, count // len(buckets))
    for reason, bucket in buckets:
        stride = max(1, len(bucket) // per_bucket)
        for row in bucket[::stride]:
            if len(selected) >= count:
                break
            row = dict(row)
            row["selection_reason"] = reason
            selected.append(row)
        if len(selected) >= count:
            break

    if len(selected) < count:
        selected_ids = {frame_id(row) for row in selected}
        for row in ranked:
            if frame_id(row) in selected_ids:
                continue
            row = dict(row)
            row["selection_reason"] = "fill"
            selected.append(row)
            if len(selected) >= count:
                break
    return selected


def main(args: argparse.Namespace) -> None:
    predictions = read_jsonl(args.prediction_records)
    existing = {frame_id(row) for row in read_jsonl(args.existing_records)}
    unlabeled = [row for row in predictions if frame_id(row) not in existing]
    unlabeled = [row for row in unlabeled if int(row.get("num_lane_pixels", 0)) >= args.min_pred_pixels]
    selected = select_diverse(unlabeled, args.num_candidates)

    output_csv = Path(args.output_csv)
    output_json = Path(args.output_json) if args.output_json else output_csv.with_suffix(".json")
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    output_json.parent.mkdir(parents=True, exist_ok=True)

    fields = ["frame_id", "image_path", "seg_mask_path", "num_lane_pixels", "selection_reason"]
    with output_csv.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in selected:
            writer.writerow({field: row.get(field, "") if field != "frame_id" else frame_id(row) for field in fields})
    output_json.write_text(json.dumps(selected, indent=2), encoding="utf-8")
    print(f"candidates={len(selected)} unlabeled_pool={len(unlabeled)} csv={output_csv} json={output_json}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Select unlabeled frames for the next manual tape annotation batch.")
    parser.add_argument("--prediction_records", required=True)
    parser.add_argument("--existing_records", required=True)
    parser.add_argument("--output_csv", required=True)
    parser.add_argument("--output_json", default="")
    parser.add_argument("--num_candidates", type=int, default=40)
    parser.add_argument("--min_pred_pixels", type=int, default=1)
    return parser.parse_args()


if __name__ == "__main__":
    main(parse_args())
