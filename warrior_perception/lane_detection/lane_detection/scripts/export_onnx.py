"""
Export multitask backbone checkpoints to ONNX with divergence verification.

The model forward pass returns nested dicts which ONNX cannot represent
directly. A thin wrapper flattens it to a named tuple of four tensors:
  det_p3      (B, pred_dim, H/8,  W/8)   FCOS predictions — small objects
  det_p4      (B, pred_dim, H/16, W/16)  FCOS predictions — medium objects
  det_p5      (B, pred_dim, H/32, W/32)  FCOS predictions — large objects
  seg_logits  (B, seg_classes, H, W)     lane segmentation logits (full res)

pred_dim = 4 (LTRB) + 1 (objectness) + num_det_classes

Outputs (under --output_dir):
  <backbone>.onnx          ONNX model
  <backbone>_meta.json     export metadata + per-output divergence stats
  export_summary.json      combined summary for all backbones

Usage:
  python scripts/export_onnx.py
  python scripts/export_onnx.py --backbones resnet18 convnext_base
  python scripts/export_onnx.py --atol 0.05 --opset 17 --output_dir outputs/onnx
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch import nn

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

try:
    import onnx
    import onnx.checker
except ImportError:
    onnx = None  # type: ignore[assignment]

try:
    import onnxruntime as ort
except ImportError:
    ort = None  # type: ignore[assignment]

from engine.train import load_yaml
from models import MultiTaskPerceptionModel


# Output tensor names — used as ONNX graph output names and divergence report keys
OUTPUT_NAMES: list[str] = ["det_p3", "det_p4", "det_p5", "seg_logits"]


class _ExportWrapper(nn.Module):
    """Flattens MultiTaskPerceptionModel outputs to a plain tuple for ONNX."""

    def __init__(self, model: MultiTaskPerceptionModel) -> None:
        super().__init__()
        self.model = model

    def forward(
        self, x: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        out = self.model(x, postprocess=False)
        det = out["detection"]
        return det["p3"], det["p4"], det["p5"], out["segmentation"]


def _load_model(
    checkpoint_path: Path, device: torch.device
) -> tuple[MultiTaskPerceptionModel, dict[str, Any], dict[str, Any]]:
    ckpt: dict[str, Any] = torch.load(checkpoint_path, map_location=device)
    cfg: dict[str, Any] = ckpt["cfg"]
    model = MultiTaskPerceptionModel(cfg).to(device)
    model.load_state_dict(ckpt["state_dict"])
    model.eval()
    return model, cfg, ckpt


def _dummy_input(cfg: dict[str, Any], device: torch.device) -> torch.Tensor:
    h, w = cfg["dataset"]["image_size"]
    return torch.randn(1, 3, h, w, device=device)


def _dynamic_axes() -> dict[str, dict[int, str]]:
    axes: dict[str, dict[int, str]] = {"input": {0: "batch"}}
    for name in OUTPUT_NAMES:
        axes[name] = {0: "batch"}
    return axes


def _check_divergence(
    wrapper: _ExportWrapper,
    dummy: torch.Tensor,
    onnx_path: Path,
    atol: float,
) -> dict[str, Any]:
    """Run PyTorch and ORT on the same input and compare each output."""
    with torch.no_grad():
        torch_outs = wrapper(dummy)

    sess_opts = ort.SessionOptions()
    sess_opts.log_severity_level = 3  # suppress ORT info spam
    sess = ort.InferenceSession(
        str(onnx_path), sess_opts, providers=["CPUExecutionProvider"]
    )
    ort_outs: list[np.ndarray] = sess.run(None, {"input": dummy.cpu().numpy()})

    stats: dict[str, Any] = {}
    all_passed = True
    for name, pt_out, ort_out in zip(OUTPUT_NAMES, torch_outs, ort_outs):
        pt_np = pt_out.cpu().float().numpy()
        abs_diff = np.abs(pt_np - ort_out)
        max_abs = float(abs_diff.max())
        mean_abs = float(abs_diff.mean())
        passed = max_abs <= atol
        all_passed = all_passed and passed
        stats[name] = {
            "shape": list(pt_np.shape),
            "max_abs_diff": round(max_abs, 7),
            "mean_abs_diff": round(mean_abs, 7),
            "atol": atol,
            "passed": passed,
        }
        verdict = "PASS" if passed else "FAIL"
        print(
            f"        {name:<14} shape={list(pt_np.shape)}"
            f"  max_abs={max_abs:.3e}  mean_abs={mean_abs:.3e}  [{verdict}]"
        )

    return {"outputs": stats, "all_passed": all_passed}


def export_backbone(
    backbone: str,
    checkpoint_path: Path,
    output_dir: Path,
    opset: int,
    atol: float,
    device: torch.device,
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "backbone": backbone,
        "checkpoint": str(checkpoint_path),
        "status": "failed",
        "passed": False,
    }

    sep = "=" * 62
    print(f"\n{sep}")
    print(f"  Backbone : {backbone}")
    print(f"  Ckpt     : {checkpoint_path}")
    print(sep)

    # 1 — Load ------------------------------------------------------------
    print("  [1/4] Loading checkpoint...")
    try:
        model, cfg, ckpt = _load_model(checkpoint_path, device)
    except Exception as exc:
        result["error"] = f"load failed: {exc}"
        print(f"  [ERROR] {exc}")
        return result

    image_size: list[int] = cfg["dataset"]["image_size"]
    num_det_classes: int = cfg["detection"]["num_classes"]
    pred_dim = 4 + 1 + num_det_classes
    result.update(
        {
            "epoch": ckpt.get("epoch", "?"),
            "image_size": image_size,
            "num_det_classes": num_det_classes,
            "pred_dim": pred_dim,
            "opset": opset,
        }
    )
    print(
        f"        epoch={result['epoch']}  image_size={image_size}"
        f"  det_classes={num_det_classes}  pred_dim={pred_dim}"
    )

    wrapper = _ExportWrapper(model).cpu().eval()
    dummy = _dummy_input(cfg, torch.device("cpu"))  # CPU for ORT compat

    # 2 — Export ----------------------------------------------------------
    onnx_path = output_dir / f"{backbone}.onnx"
    print(f"  [2/4] Exporting to ONNX (opset={opset})...")
    t0 = time.perf_counter()
    try:
        with torch.no_grad():
            torch.onnx.export(
                wrapper,
                dummy,
                str(onnx_path),
                input_names=["input"],
                output_names=OUTPUT_NAMES,
                dynamic_axes=_dynamic_axes(),
                opset_version=opset,
                # Must be False: constant folding misoptimises the shared DetectionHead
                # stem (Conv+BN+ReLU called 3× for p3/p4/p5), causing large ORT divergence.
                do_constant_folding=False,
            )
    except Exception as exc:
        result["error"] = f"export failed: {exc}"
        print(f"  [ERROR] {exc}")
        return result

    export_sec = round(time.perf_counter() - t0, 2)
    size_mb = round(onnx_path.stat().st_size / (1024 ** 2), 2)
    result["export_time_sec"] = export_sec
    result["onnx_size_mb"] = size_mb
    print(f"        -> {onnx_path}  ({size_mb} MB, {export_sec}s)")

    # 3 — Graph validation ------------------------------------------------
    print("  [3/4] Validating ONNX graph...")
    if onnx is not None:
        try:
            proto = onnx.load(str(onnx_path))
            onnx.checker.check_model(proto)
            result["onnx_graph_check"] = "pass"
            print("        onnx.checker : PASS")
        except Exception as exc:
            result["onnx_graph_check"] = f"fail: {exc}"
            print(f"        onnx.checker : FAIL — {exc}")
    else:
        result["onnx_graph_check"] = "skipped (onnx not installed)"
        print("        onnx.checker : skipped — pip install onnx")

    # 4 — Divergence check ------------------------------------------------
    print(f"  [4/4] Divergence check (atol={atol})...")
    if ort is None:
        result["divergence"] = "skipped (onnxruntime not installed)"
        result["status"] = "exported_no_verify"
        print("        onnxruntime  : skipped — pip install onnxruntime")
        return result

    try:
        div = _check_divergence(wrapper, dummy, onnx_path, atol)
    except Exception as exc:
        result["error"] = f"divergence check failed: {exc}"
        result["status"] = "verify_error"
        print(f"  [ERROR] {exc}")
        return result

    result["divergence"] = div
    result["passed"] = div["all_passed"]
    result["status"] = "ok" if div["all_passed"] else "divergence_fail"
    overall = "PASS" if div["all_passed"] else "FAIL"
    print(f"        Overall : [{overall}]")

    # Write per-backbone metadata sidecar
    meta_path = output_dir / f"{backbone}_meta.json"
    meta_path.write_text(json.dumps(result, indent=2, default=str), encoding="utf-8")
    print(f"        Metadata : {meta_path}")

    return result


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Export multitask backbones to ONNX and verify divergence."
    )
    parser.add_argument("--sweep_dir", default="outputs/backbone_sweep")
    parser.add_argument("--output_dir", default="outputs/onnx")
    parser.add_argument(
        "--backbones",
        nargs="*",
        default=["resnet18", "convnext_base"],
        help="Backbone names to export (default: resnet18 convnext_base)",
    )
    parser.add_argument("--opset", type=int, default=17, help="ONNX opset version")
    parser.add_argument("--atol", type=float, default=0.05, help="Absolute tolerance for divergence check")
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    sweep_dir = Path(args.sweep_dir)

    missing_deps: list[str] = []
    if onnx is None:
        missing_deps.append("onnx")
    if ort is None:
        missing_deps.append("onnxruntime")
    if missing_deps:
        print(f"[WARN] Missing packages: {', '.join(missing_deps)}")
        print(f"       pip install {' '.join(missing_deps)}")

    print(f"\n{'='*62}")
    print(f"  ONNX Export — {len(args.backbones)} backbone(s)")
    print(f"  sweep_dir  : {sweep_dir}")
    print(f"  output_dir : {output_dir}")
    print(f"  opset      : {args.opset}")
    print(f"  atol       : {args.atol}")
    print(f"{'='*62}")

    all_results: list[dict[str, Any]] = []
    for backbone in args.backbones:
        ckpt_path = sweep_dir / "checkpoints" / backbone / "best.pt"
        if not ckpt_path.exists():
            print(f"\n[SKIP] {backbone}: best.pt not found at {ckpt_path}")
            all_results.append({"backbone": backbone, "status": "skipped: no best.pt", "passed": False})
            continue
        result = export_backbone(backbone, ckpt_path, output_dir, args.opset, args.atol, torch.device("cpu"))
        all_results.append(result)

    # Write combined summary
    summary_path = output_dir / "export_summary.json"
    summary_path.write_text(json.dumps(all_results, indent=2, default=str), encoding="utf-8")

    # Print table
    print(f"\n{'='*62}")
    print("  Summary")
    print(f"{'='*62}")
    for r in all_results:
        icon = "+" if r.get("passed") else "-"
        b = r["backbone"]
        s = r.get("status", "?")
        size = r.get("onnx_size_mb", "")
        size_str = f"  {size} MB" if size else ""
        print(f"  [{icon}] {b:<22} {s}{size_str}")
    print(f"\n  Summary -> {summary_path}")
    print()


if __name__ == "__main__":
    main()
