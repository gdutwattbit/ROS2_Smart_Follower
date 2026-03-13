#!/usr/bin/env python3
"""Validate ReID ONNX model I/O shape and random inference sanity."""

import argparse
import numpy as np
import onnx
import onnxruntime as ort


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True, help='Path to ONNX model')
    parser.add_argument('--height', type=int, default=256)
    parser.add_argument('--width', type=int, default=128)
    parser.add_argument('--feature-dim', type=int, default=2048)
    args = parser.parse_args()

    model = onnx.load(args.model)
    onnx.checker.check_model(model)

    sess = ort.InferenceSession(args.model, providers=['CPUExecutionProvider'])
    input_meta = sess.get_inputs()[0]
    output_meta = sess.get_outputs()[0]
    print(f'Input:  name={input_meta.name}, shape={input_meta.shape}')
    print(f'Output: name={output_meta.name}, shape={output_meta.shape}')

    x = np.random.rand(1, 3, args.height, args.width).astype(np.float32)
    y = sess.run([output_meta.name], {input_meta.name: x})[0]

    if not np.all(np.isfinite(y)):
        raise RuntimeError('ONNX output contains NaN/Inf')

    flat = y.reshape(-1)
    if flat.size != args.feature_dim:
        raise RuntimeError(f'Feature dim mismatch: expected {args.feature_dim}, got {flat.size}')

    n = np.linalg.norm(flat)
    if n < 1e-12:
        raise RuntimeError('Feature norm is near zero, invalid embedding')

    normalized = flat / n
    n2 = np.linalg.norm(normalized)
    if not (0.999 <= n2 <= 1.001):
        raise RuntimeError(f'Normalized feature norm out of range: {n2}')

    print('Validation passed: shape, finiteness, and normalized L2 norm are valid.')


if __name__ == '__main__':
    main()
