#!/usr/bin/env python3
"""Train or resume TorchReID ResNet50 on Market1501-compatible layout."""

import argparse
from pathlib import Path

import torch
import torchreid


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--data-root', default='reid-data')
    parser.add_argument('--source', default='market1501')
    parser.add_argument('--target', default='market1501')
    parser.add_argument('--save-dir', default='log/resnet50')
    parser.add_argument('--resume', default='', help='checkpoint path, e.g. log/resnet50/model/model.pth.tar-30')
    parser.add_argument('--start-epoch', type=int, default=0)
    parser.add_argument('--max-epoch', type=int, default=60)
    parser.add_argument('--eval-freq', type=int, default=10)
    parser.add_argument('--print-freq', type=int, default=10)
    parser.add_argument('--batch-size-train', type=int, default=32)
    parser.add_argument('--batch-size-test', type=int, default=100)
    parser.add_argument('--height', type=int, default=256)
    parser.add_argument('--width', type=int, default=128)
    parser.add_argument('--lr', type=float, default=3e-4)
    args = parser.parse_args()

    datamanager = torchreid.data.ImageDataManager(
        root=args.data_root,
        sources=args.source,
        targets=args.target,
        height=args.height,
        width=args.width,
        batch_size_train=args.batch_size_train,
        batch_size_test=args.batch_size_test,
        transforms=['random_flip', 'random_crop'],
    )

    model = torchreid.models.build_model(
        name='resnet50',
        num_classes=datamanager.num_train_pids,
        loss='softmax',
        pretrained=True,
    )

    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    model = model.to(device)

    optimizer = torchreid.optim.build_optimizer(model, optim='adam', lr=args.lr)
    scheduler = torchreid.optim.build_lr_scheduler(optimizer, lr_scheduler='single_step', stepsize=20)

    start_epoch = args.start_epoch
    if args.resume:
        ckpt = torch.load(args.resume, map_location='cpu', weights_only=False)
        if isinstance(ckpt, dict) and 'state_dict' in ckpt:
            model.load_state_dict(ckpt['state_dict'], strict=False)
            if 'optimizer' in ckpt:
                optimizer.load_state_dict(ckpt['optimizer'])
            if 'scheduler' in ckpt:
                scheduler.load_state_dict(ckpt['scheduler'])
            if 'epoch' in ckpt:
                start_epoch = int(ckpt['epoch'])
        print(f'Resumed from {args.resume}, start_epoch={start_epoch}')

    Path(args.save_dir).mkdir(parents=True, exist_ok=True)

    engine = torchreid.engine.ImageSoftmaxEngine(
        datamanager,
        model,
        optimizer=optimizer,
        scheduler=scheduler,
        label_smooth=True,
    )

    engine.run(
        save_dir=args.save_dir,
        start_epoch=start_epoch,
        max_epoch=args.max_epoch,
        eval_freq=args.eval_freq,
        print_freq=args.print_freq,
        test_only=False,
    )


if __name__ == '__main__':
    main()
