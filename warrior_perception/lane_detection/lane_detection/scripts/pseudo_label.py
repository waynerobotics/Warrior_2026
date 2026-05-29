"""
Semi-supervised pseudo-labelling pipeline.

Strategy (combined approach chosen for best label quality):
  Detection  — run best checkpoint on unlabeled images, keep boxes with
               score >= DET_THRESHOLD (default 0.65).
  Segmentation — for each unlabeled image run the seg head, take softmax
               probability for the lane class and threshold at SEG_THRESHOLD
               (default 0.75 per pixel). Save as binary PNG.
               Only save masks where > MIN_LANE_FRACTION of pixels are lane
               (filters out empty / noisy predictions).
  Confidence  — pseudo-labeled samples enter training with seg_confidence=0.65
               and label_source="pseudo" so the loss function down-weights them
               vs manually-labelled data.
  Iteration   — can be re-run after a second training round with a stronger
               checkpoint to refine labels.

Outputs:
  data/pseudo_masks/<stem>.png           lane mask for each pseudo-seg sample
  data/pseudo_train_manifest.json        original train + accepted pseudo samples
  data/pseudo_label_report.json          per-image acceptance/rejection stats

Usage (from project root):
  python scripts/pseudo_label.py \\
      --checkpoint outputs/backbone_sweep/checkpoints/resnet18/best.pt \\
      --unlabeled_manifest data/unlabeled_manifest.json \\
      --train_manifest data/train_manifest.json
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import torch
import torch.nn.functional as F
from PIL import Image
from torch.utils.data import DataLoader, Dataset

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from models import MultiTaskPerceptionModel
from models.detection_postprocess import decode_detections

IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png"}

DET_THRESHOLD = 0.65
SEG_THRESHOLD = 0.75
MIN_LANE_FRACTION = 0.01   # ignore predictions where < 1% pixels are lane
PSEUDO_SEG_CONFIDENCE = 0.65
BATCH_SIZE = 4


# ---------------------------------------------------------------------------
# Minimal dataset for unlabeled images
# ---------------------------------------------------------------------------

class UnlabeledDataset(Dataset):  # type: ignore[type-arg]
    def __init__(self, samples: list[dict], image_size: tuple[int, int], image_root: Path) -> None:
        self.samples = samples
        self.target_h, self.target_w = image_size
        self.image_root = image_root

        _mean = torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1)
        _std = torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1)
        self._mean = _mean
        self._std = _std

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int) -> dict[str, Any]:
        sample = self.samples[idx]
        img_path = Path(sample["image"])
        if not img_path.is_absolute():
            img_path = self.image_root / img_path
        img = Image.open(img_path).convert("RGB").resize(
            (self.target_w, self.target_h), Image.BILINEAR
        )
        tensor = torch.from_numpy(__import__("numpy").array(img, dtype="float32")).permute(2, 0, 1) / 255.0
        tensor = (tensor - self._mean) / self._std
        return {"image": tensor, "stem": img_path.stem, "image_path": str(img_path)}


# ---------------------------------------------------------------------------
# Checkpoint loading
# ---------------------------------------------------------------------------

def load_checkpoint(checkpoint_path: Path, device: torch.device) -> tuple[MultiTaskPerceptionModel, dict]:
    ckpt = torch.load(checkpoint_path, map_location=device)
    cfg: dict = ckpt["cfg"]
    model = MultiTaskPerceptionModel(cfg).to(device)
    model.load_state_dict(ckpt["state_dict"])
    model.eval()
    return model, cfg


# ---------------------------------------------------------------------------
# Inference helpers
# ---------------------------------------------------------------------------

@torch.no_grad()
def infer_batch(
    model: MultiTaskPerceptionModel,
    images: torch.Tensor,
    cfg: dict,
    device: torch.device,
    det_threshold: float,
) -> tuple[list[dict], torch.Tensor]:
    images = images.to(device)
    preds = model(images, postprocess=False)

    det_decoded = decode_detections(
        level_outputs=preds["detection"],
        image_size=(images.shape[-2], images.shape[-1]),
        score_threshold=det_threshold,
        nms_threshold=float(cfg["detection"].get("nms_threshold", 0.5)),
        max_detections=int(cfg["detection"].get("max_detections", 100)),
    )

    seg_probs = F.softmax(preds["segmentation"].float(), dim=1)
    lane_probs = seg_probs[:, 1, :, :]  # (B, H, W) — probability of being lane

    return det_decoded, lane_probs


def boxes_to_list(
    boxes: torch.Tensor, labels: torch.Tensor, scores: torch.Tensor, det_threshold: float
) -> tuple[list, list]:
    out_boxes, out_labels = [], []
    for box, lbl, score in zip(boxes.tolist(), labels.tolist(), scores.tolist()):
        if score >= det_threshold:
            out_boxes.append([round(v, 2) for v in box])
            out_labels.append(int(lbl))
    return out_boxes, out_labels


def save_lane_mask(
    lane_prob: torch.Tensor, out_path: Path, seg_threshold: float, min_lane_fraction: float
) -> bool:
    """Returns True if the mask passes the min-lane-fraction filter."""
    hard_mask = (lane_prob >= seg_threshold).cpu().numpy().astype("uint8") * 255
    fraction = float(hard_mask.mean()) / 255.0
    if fraction < min_lane_fraction:
        return False
    Image.fromarray(hard_mask, mode="L").save(str(out_path))
    return True


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="Generate pseudo-labels for unlabeled images.")
    parser.add_argument("--checkpoint", required=True,
                        help="Path to best.pt from backbone sweep")
    parser.add_argument("--unlabeled_manifest", default="data/unlabeled_manifest.json")
    parser.add_argument("--train_manifest", default="data/train_manifest.json",
                        help="Existing train manifest — pseudo samples are appended to it")
    parser.add_argument("--output_manifest", default="data/pseudo_train_manifest.json")
    parser.add_argument("--masks_dir", default="data/pseudo_masks")
    parser.add_argument("--det_threshold", type=float, default=DET_THRESHOLD)
    parser.add_argument("--seg_threshold", type=float, default=SEG_THRESHOLD)
    parser.add_argument("--min_lane_fraction", type=float, default=MIN_LANE_FRACTION)
    parser.add_argument("--batch_size", type=int, default=BATCH_SIZE)
    parser.add_argument("--limit", type=int, default=0,
                        help="Process only the first N unlabeled images (0 = all)")
    parser.add_argument("--device", default="auto")
    args = parser.parse_args()

    device = torch.device(
        "cuda" if (args.device == "auto" and torch.cuda.is_available()) else args.device
        if args.device != "auto" else "cpu"
    )

    checkpoint_path = Path(args.checkpoint)
    masks_dir = (ROOT / args.masks_dir).resolve()
    masks_dir.mkdir(parents=True, exist_ok=True)

    print(f"Loading checkpoint: {checkpoint_path}")
    model, cfg = load_checkpoint(checkpoint_path, device)
    image_size = tuple(cfg["dataset"]["image_size"])
    class_names = cfg.get("detection", {}).get("class_names", [str(i) for i in range(cfg["detection"]["num_classes"])])
    print(f"  backbone={cfg['backbone']['name']}  "
          f"det_classes={cfg['detection']['num_classes']}  "
          f"image_size={image_size}")

    print(f"Loading unlabeled manifest: {args.unlabeled_manifest}")
    with open(args.unlabeled_manifest, encoding="utf-8") as fh:
        unlabeled_samples: list[dict] = json.load(fh)
    print(f"  {len(unlabeled_samples)} unlabeled images")
    if args.limit > 0:
        unlabeled_samples = unlabeled_samples[: args.limit]
        print(f"  Limiting to {len(unlabeled_samples)} images (--limit {args.limit})")

    print(f"Loading train manifest: {args.train_manifest}")
    with open(args.train_manifest, encoding="utf-8") as fh:
        train_samples: list[dict] = json.load(fh)
    print(f"  {len(train_samples)} existing training samples")

    dataset = UnlabeledDataset(unlabeled_samples, image_size, ROOT)
    loader = DataLoader(dataset, batch_size=args.batch_size, shuffle=False,
                        num_workers=2, pin_memory=device.type == "cuda")

    new_samples: list[dict] = []
    report: list[dict] = []
    accepted_det = accepted_seg = accepted_both = skipped = 0

    print(f"\nRunning inference on {len(unlabeled_samples)} images ...")
    for batch_idx, batch in enumerate(loader):
        images: torch.Tensor = batch["image"]
        stems: list[str] = batch["stem"]
        image_paths: list[str] = batch["image_path"]

        det_preds, lane_probs = infer_batch(model, images, cfg, device, args.det_threshold)

        for i, (stem, img_path) in enumerate(zip(stems, image_paths)):
            pred = det_preds[i]
            det_boxes, det_labels = boxes_to_list(
                pred["boxes"], pred["labels"], pred["scores"], args.det_threshold
            )

            # Segmentation mask
            seg_mask_ref: str | None = None
            mask_path = masks_dir / f"{stem}.png"
            lane_prob_np = lane_probs[i]  # (H, W)
            has_seg = save_lane_mask(lane_prob_np, mask_path, args.seg_threshold, args.min_lane_fraction)
            if has_seg:
                seg_mask_ref = mask_path.relative_to(ROOT).as_posix()

            has_det = len(det_boxes) > 0
            if not has_det and not has_seg:
                skipped += 1
                report.append({"image": img_path, "accepted": False, "reason": "no confident predictions"})
                continue

            sample: dict = {
                "image": str(Path(img_path).relative_to(ROOT).as_posix()),
                "boxes": det_boxes,
                "labels": det_labels,
                "seg_confidence": PSEUDO_SEG_CONFIDENCE if has_seg else 0.0,
                "label_source": "pseudo",
            }
            if seg_mask_ref:
                sample["seg_mask"] = seg_mask_ref

            new_samples.append(sample)
            report.append({
                "image": img_path,
                "accepted": True,
                "det_boxes": len(det_boxes),
                "has_seg": has_seg,
            })

            if has_det and has_seg:
                accepted_both += 1
            elif has_det:
                accepted_det += 1
            else:
                accepted_seg += 1

        if (batch_idx + 1) % 20 == 0:
            done = min((batch_idx + 1) * args.batch_size, len(unlabeled_samples))
            print(f"  {done}/{len(unlabeled_samples)} images processed ...")

    print(f"\n--- Pseudo-label results ---")
    print(f"  accepted det-only:  {accepted_det}")
    print(f"  accepted seg-only:  {accepted_seg}")
    print(f"  accepted det+seg:   {accepted_both}")
    print(f"  skipped (no conf):  {skipped}")
    print(f"  total new samples:  {len(new_samples)}")

    combined = train_samples + new_samples
    output_path = Path(args.output_manifest)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(combined, indent=2), encoding="utf-8")
    print(f"\nWrote {output_path}  ({len(combined)} total samples)")

    report_path = Path("data/pseudo_label_report.json")
    report_path.write_text(json.dumps({
        "checkpoint": str(checkpoint_path),
        "det_threshold": args.det_threshold,
        "seg_threshold": args.seg_threshold,
        "min_lane_fraction": args.min_lane_fraction,
        "summary": {
            "unlabeled_input": len(unlabeled_samples),
            "accepted_det_only": accepted_det,
            "accepted_seg_only": accepted_seg,
            "accepted_both": accepted_both,
            "skipped": skipped,
        },
        "per_image": report,
    }, indent=2), encoding="utf-8")
    print(f"Wrote {report_path}")

    print("\nNext step — retrain with pseudo-labels:")
    print(f"  python scripts/run_backbone_sweep.py --train_manifest {output_path}")


if __name__ == "__main__":
    main()
