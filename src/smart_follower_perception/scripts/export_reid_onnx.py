#!/usr/bin/env python3
"""Export a TorchReID ResNet50 checkpoint to ONNX (2048-d embedding)."""

import argparse
from pathlib import Path

import torch
import torchreid



def infer_num_classes(state_dict: dict, fallback: int) -> int:
    for key, value in state_dict.items():
        if key.endswith('classifier.weight') and value.ndim == 2:
            return int(value.shape[0])
    return fallback


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', required=True, help='Path to torchreid checkpoint (.pth/.tar)')
    parser.add_argument('--output', default='models/reid_resnet50_2048.onnx')
    parser.add_argument('--height', type=int, default=256)
    parser.add_argument('--width', type=int, default=128)
    parser.add_argument('--opset', type=int, default=12)
    parser.add_argument('--num-classes', type=int, default=751, help='Fallback class count when checkpoint does not expose classifier weight')
    args = parser.parse_args()

    checkpoint = torch.load(args.weights, map_location='cpu', weights_only=False)
    state_dict = checkpoint['state_dict'] if isinstance(checkpoint, dict) and 'state_dict' in checkpoint else checkpoint

    num_classes = infer_num_classes(state_dict, args.num_classes)
    model = torchreid.models.build_model(
        name='resnet50',
        num_classes=num_classes,
        loss='softmax',
        pretrained=False,
    )
    model.load_state_dict(state_dict, strict=False)
    model.eval()

    dummy = torch.randn(1, 3, args.height, args.width, dtype=torch.float32)
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    torch.onnx.export(
        model,
        dummy,
        output_path.as_posix(),
        export_params=True,
        opset_version=args.opset,
        do_constant_folding=True,
        input_names=['images'],
        output_names=['embeddings'],
        dynamic_axes=None,
    )

    print(f'Exported ONNX to: {output_path}')


if __name__ == '__main__':
    main()
