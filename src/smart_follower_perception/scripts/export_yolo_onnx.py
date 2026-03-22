#!/usr/bin/env python3
"""Export Ultralytics YOLO26 model to ONNX for C++ runtime."""

import argparse
from pathlib import Path

from ultralytics import YOLO


def str2bool(value: str) -> bool:
    if isinstance(value, bool):
        return value
    lowered = value.lower()
    if lowered in {"1", "true", "yes", "y", "on"}:
        return True
    if lowered in {"0", "false", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"invalid boolean value: {value}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", default="yolo26n.pt", help="YOLO checkpoint path")
    parser.add_argument("--output", default="yolo26n.onnx", help="Output ONNX path")
    parser.add_argument("--imgsz", type=int, nargs=2, default=[480, 640], help="H W")
    parser.add_argument("--opset", type=int, default=12)
    parser.add_argument("--dynamic", type=str2bool, default=False)
    parser.add_argument("--simplify", type=str2bool, default=True)
    parser.add_argument("--end2end", type=str2bool, default=True)
    args = parser.parse_args()

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)

    model = YOLO(args.weights)
    exported = model.export(
        format="onnx",
        opset=args.opset,
        imgsz=args.imgsz,
        dynamic=args.dynamic,
        simplify=args.simplify,
        end2end=args.end2end,
    )

    exported_path = Path(exported)
    if exported_path.resolve() != out.resolve():
        out.write_bytes(exported_path.read_bytes())

    print(f"Exported ONNX: {out}")
    print(
        "settings="
        f"imgsz={args.imgsz}, dynamic={args.dynamic}, simplify={args.simplify}, end2end={args.end2end}, opset={args.opset}"
    )


if __name__ == "__main__":
    main()
