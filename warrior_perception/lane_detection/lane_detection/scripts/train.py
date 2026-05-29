from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.train import load_yaml, train


def apply_overrides(cfg: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    if args.train_manifest:
        cfg["dataset"]["train_manifest"] = args.train_manifest
    if args.val_manifest:
        cfg["dataset"]["val_manifest"] = args.val_manifest
    if args.epochs is not None:
        cfg["epochs"] = args.epochs
    if args.batch_size is not None:
        cfg["batch_size"] = args.batch_size
    if args.device:
        cfg["device"] = args.device
    if args.max_steps_per_epoch:
        cfg["max_steps_per_epoch"] = args.max_steps_per_epoch
    if args.output_dir:
        cfg["checkpoint_root"] = f"{args.output_dir}/checkpoints"
    return cfg


def main() -> None:
    parser = argparse.ArgumentParser(description="Train the canonical multitask perception model.")
    parser.add_argument("--config", default="configs/multitask/multitask_resnet18.yaml")
    parser.add_argument("--train_manifest", default="")
    parser.add_argument("--val_manifest", default="")
    parser.add_argument("--output_dir", default="outputs/multitask")
    parser.add_argument("--epochs", type=int, default=None)
    parser.add_argument("--batch_size", type=int, default=None)
    parser.add_argument("--device", default="")
    parser.add_argument("--max_steps_per_epoch", type=int, default=0)
    args = parser.parse_args()

    cfg = apply_overrides(load_yaml(args.config), args)
    summary = train(cfg, config_path=args.config)
    print(summary)


if __name__ == "__main__":
    main()
