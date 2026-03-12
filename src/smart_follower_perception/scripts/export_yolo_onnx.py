#!/usr/bin/env python3
"""Export Ultralytics YOLO26n model to ONNX for C++ runtime."""

import argparse
from pathlib import Path

from ultralytics import YOLO


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", default="yolo26n.pt", help="YOLO checkpoint path")
    parser.add_argument("--output", default="yolo26n.onnx", help="Output ONNX path")
    parser.add_argument("--imgsz", type=int, nargs=2, default=[480, 640], help="H W")
    parser.add_argument("--opset", type=int, default=12)
    args = parser.parse_args()

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)

    model = YOLO(args.weights)
    model.export(format="onnx", opset=args.opset, imgsz=args.imgsz, simplify=True, dynamic=False)

    generated = Path(args.weights).with_suffix(".onnx")
    if generated.exists() and generated.resolve() != out.resolve():
      generated.replace(out)


if __name__ == "__main__":
    main()
