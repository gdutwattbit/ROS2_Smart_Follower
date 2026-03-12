#!/usr/bin/env python3
"""Export a MobileNetV2 ReID model to ONNX (128-d embedding)."""

import argparse
from pathlib import Path

import torch
import torchreid


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", required=True, help="Path to .pth checkpoint")
    parser.add_argument("--output", default="reid_mobilenetv2_128.onnx")
    parser.add_argument("--height", type=int, default=128)
    parser.add_argument("--width", type=int, default=64)
    parser.add_argument("--opset", type=int, default=12)
    args = parser.parse_args()

    model = torchreid.models.build_model(
        name="mobilenetv2_x1_0",
        num_classes=1000,
        loss="softmax",
        pretrained=False,
    )

    checkpoint = torch.load(args.weights, map_location="cpu")
    if isinstance(checkpoint, dict) and "state_dict" in checkpoint:
        checkpoint = checkpoint["state_dict"]
    model.load_state_dict(checkpoint, strict=False)
    model.eval()

    dummy = torch.randn(1, 3, args.height, args.width, dtype=torch.float32)
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)

    torch.onnx.export(
        model,
        dummy,
        out.as_posix(),
        export_params=True,
        opset_version=args.opset,
        do_constant_folding=True,
        input_names=["images"],
        output_names=["embeddings"],
        dynamic_axes=None,
    )


if __name__ == "__main__":
    main()
