#!/usr/bin/env python3
"""
Convert a multitask ONNX model to a serialised TensorRT engine.

Run this on the target device (Jetson Orin / any CUDA machine with TRT installed).
The engine is device-specific and cannot be transferred to a different GPU.

Usage:
  python inference/convert_to_engine.py \
      --onnx   outputs/onnx/resnet18.onnx \
      --engine outputs/engines/resnet18_fp16.trt

  # FP32 (slower, useful for debugging):
  python inference/convert_to_engine.py --onnx ... --engine ... --fp32

  # Explicit workspace (default 4 GB):
  python inference/convert_to_engine.py --onnx ... --engine ... --workspace 6

Requires:
  tensorrt (pre-installed in JetPack 5.x, or: pip install tensorrt on x86-64)
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import tensorrt as trt

TRT_LOGGER = trt.Logger(trt.Logger.INFO)


def build_engine(
    onnx_path: Path,
    engine_path: Path,
    fp16: bool = True,
    workspace_gb: int = 4,
    timing_cache_path: Path | None = None,
) -> None:
    print(f"[INFO] ONNX input    : {onnx_path}")
    print(f"[INFO] Engine output : {engine_path}")
    print(f"[INFO] Precision     : {'FP16' if fp16 else 'FP32'}")
    print(f"[INFO] Workspace     : {workspace_gb} GB")
    print(f"[INFO] TensorRT ver  : {trt.__version__}")

    builder = trt.Builder(TRT_LOGGER)
    # TRT 10: explicit batch is the only mode; create_network takes no flags.
    network = builder.create_network()
    parser = trt.OnnxParser(network, TRT_LOGGER)

    print("[INFO] Parsing ONNX model ...")
    with open(onnx_path, "rb") as f:
        if not parser.parse(f.read()):
            for i in range(parser.num_errors):
                print(f"  [ONNX ERROR] {parser.get_error(i)}", file=sys.stderr)
            raise RuntimeError("ONNX parsing failed")

    print("[INFO] Network bindings:")
    for i in range(network.num_inputs):
        t = network.get_input(i)
        print(f"  input  [{i}] {t.name:<20} {tuple(t.shape)}")
    for i in range(network.num_outputs):
        t = network.get_output(i)
        print(f"  output [{i}] {t.name:<20} {tuple(t.shape)}")

    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, workspace_gb << 30)

    if fp16:
        if builder.platform_has_fast_fp16:
            config.set_flag(trt.BuilderFlag.FP16)
            print("[INFO] FP16 flag set (platform supports fast FP16)")
        else:
            print("[WARN] Platform does not have fast FP16 — building FP32 engine")


    # The ONNX was exported with dynamic batch axis (-1).
    # Fix batch=1 via an optimization profile so TRT can specialize the kernels.
    profile = builder.create_optimization_profile()
    inp = network.get_input(0)
    h, w = inp.shape[2], inp.shape[3]   # 640, 1280
    profile.set_shape(inp.name,
                      min=(1, 3, h, w),
                      opt=(1, 3, h, w),
                      max=(1, 3, h, w))
    config.add_optimization_profile(profile)
    print(f"[INFO] Optimization profile: batch=1 fixed, input ({1}, 3, {h}, {w})")

    # Timing cache: load existing bytes if the file exists, otherwise start fresh.
    # Saved after a successful build so the next build reuses profiling results
    # and completes in seconds instead of minutes.
    if timing_cache_path is None:
        timing_cache_path = engine_path.with_suffix(".trt.timing_cache")

    cache_blob = b""
    if timing_cache_path.exists():
        cache_blob = timing_cache_path.read_bytes()
        print(f"[INFO] Timing cache  : loaded {len(cache_blob) // 1024} KB from {timing_cache_path}")
    else:
        print(f"[INFO] Timing cache  : none found — full profiling run (will be saved to {timing_cache_path})")

    timing_cache = config.create_timing_cache(cache_blob)
    config.set_timing_cache(timing_cache, ignore_mismatch=False)

    print("[INFO] Building serialised network (may take several minutes on first run) ...")
    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise RuntimeError("Engine build failed — check TRT logs above")

    engine_path.parent.mkdir(parents=True, exist_ok=True)
    with open(engine_path, "wb") as f:
        f.write(serialized)

    # Persist the timing cache so the next rebuild is fast.
    updated_cache = timing_cache.serialize()
    timing_cache_path.write_bytes(updated_cache)
    print(f"[INFO] Timing cache  : saved {len(updated_cache) // 1024} KB to {timing_cache_path}")

    size_mb = engine_path.stat().st_size / (1024 ** 2)
    print(f"[INFO] Engine saved  : {engine_path}  ({size_mb:.1f} MB)")
    print()
    print("Next step — run Python inference:")
    print(f"  python inference/infer_python.py --engine {engine_path} --image_dir data/frames/")
    print()
    print("Or build & run the C++ binary on Jetson:")
    print(f"  cd inference && ./build.sh")
    print(f"  ./inference/build/infer_images --engine {engine_path} --image_dir data/frames/")


def main() -> None:
    p = argparse.ArgumentParser(
        description="Convert multitask ONNX model to TensorRT engine."
    )
    p.add_argument("--onnx",      required=True, help="Input ONNX file path")
    p.add_argument(
        "--engine",
        default=None,
        help="Output engine path (default: same dir as ONNX, with .trt extension)",
    )
    p.add_argument(
        "--fp32",
        action="store_true",
        help="Build FP32 engine instead of FP16",
    )
    p.add_argument(
        "--workspace",
        type=int,
        default=4,
        help="Builder workspace in GB (default: 4)",
    )
    p.add_argument(
        "--timing_cache",
        default=None,
        help="Path to timing cache file (default: <engine>.timing_cache). "
             "Saves profiling results so rebuilds take seconds instead of minutes.",
    )
    args = p.parse_args()

    onnx_path = Path(args.onnx)
    if not onnx_path.exists():
        raise FileNotFoundError(f"ONNX file not found: {onnx_path}")

    if args.engine is None:
        engine_path = onnx_path.with_suffix(".trt")
    else:
        engine_path = Path(args.engine)

    build_engine(
        onnx_path=onnx_path,
        engine_path=engine_path,
        fp16=not args.fp32,
        workspace_gb=args.workspace,
        timing_cache_path=Path(args.timing_cache) if args.timing_cache else None,
    )


if __name__ == "__main__":
    main()
