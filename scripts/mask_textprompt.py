#!/usr/bin/env python3
"""Experiment 3 — language-prompted segmentation (text → box → SAM).

The LLM-reasoning route: no rig geometry, no clicks — a text query does the
localisation. An open-vocabulary grounding model (OWLv2) is asked for
"a beetle" / "an insect"; the best-scoring box is handed to SAM, and the
best-scoring SAM candidate wins. This is the same pipeline family as
LLM-Seg / LISA (language model reasons about the query, SAM produces the
mask) — those exact repos are CUDA/DeepSpeed-only and cannot run on this
machine, so the language tower here is the lightweight grounding model.

    pixi run -e masking mask-textprompt -- datasets/phone_<ts> [--limit N]

Writes, under <session>/masks/textprompt/:
    stack_<n>.png       full-res binary mask (255 = insect; empty if no
                        detection above --det-threshold)
    preview/<n>.jpg     overlay preview
    debug/<n>.jpg       detected box + query + score
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mask_common import (find_stacks, load_image, overlay_preview, pick_device,
                         save_mask)
from mask_sam_geom import draw_debug, keep_component_at


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session')
    ap.add_argument('--queries', nargs='+',
                    default=['a beetle', 'an insect'],
                    help='text queries for the grounding model')
    ap.add_argument('--det-model',
                    default='google/owlv2-base-patch16-ensemble')
    ap.add_argument('--sam-model', default='facebook/sam-vit-large')
    ap.add_argument('--det-threshold', type=float, default=0.10)
    ap.add_argument('--crop-scale', type=float, default=1.8,
                    help='SAM crop size relative to the detected box '
                         '(image-space zoom, no geometry involved)')
    ap.add_argument('--out', default=None)
    ap.add_argument('--limit', type=int, default=None)
    args = ap.parse_args()

    out_dir = args.out or os.path.join(args.session.rstrip('/'),
                                       'masks', 'textprompt')
    composites = find_stacks(args.session)
    if args.limit:
        composites = composites[:args.limit]

    import torch
    from transformers import (Owlv2ForObjectDetection, Owlv2Processor,
                              SamModel, SamProcessor)
    device = pick_device()
    print(f'loading {args.det_model} + {args.sam_model} on {device} …')
    det_proc = Owlv2Processor.from_pretrained(args.det_model)
    det = Owlv2ForObjectDetection.from_pretrained(args.det_model) \
        .to(device).eval()
    sam_proc = SamProcessor.from_pretrained(args.sam_model)
    sam = SamModel.from_pretrained(args.sam_model).to(device).eval()

    def to_dev(inputs):
        return {k: (v.to(torch.float32) if v.dtype == torch.float64 else v)
                .to(device) for k, v in inputs.items()}

    for n, path in composites:
        img = load_image(path)
        W, H = img.size

        inputs = to_dev(det_proc(text=[args.queries], images=img,
                                 return_tensors='pt'))
        with torch.no_grad():
            out = det(**inputs)
        res = det_proc.post_process_grounded_object_detection(
            out, target_sizes=torch.tensor([[H, W]]),
            threshold=args.det_threshold)[0]
        if len(res['scores']) == 0:
            print(f'stack {n:2d}: NO detection above '
                  f'{args.det_threshold} — empty mask')
            save_mask(np.zeros((H, W), np.float32),
                      os.path.join(out_dir, f'stack_{n:02d}.png'))
            overlay_preview(img, np.zeros((H, W), np.float32),
                            os.path.join(out_dir, 'preview', f'{n:02d}.jpg'))
            continue
        k = int(res['scores'].argmax())
        score = float(res['scores'][k])
        query = args.queries[int(res['labels'][k])]
        box = [float(v) for v in res['boxes'][k]]          # x0 y0 x1 y1

        # zoom SAM onto the detection (image-space, geometry-free)
        bw, bh = box[2] - box[0], box[3] - box[1]
        cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
        ch = max(bw, bh) / 2 * args.crop_scale
        cx0 = int(max(0, min(cx - ch, W - 2 * ch)))
        cy0 = int(max(0, min(cy - ch, H - 2 * ch)))
        cx1, cy1 = int(min(W, cx0 + 2 * ch)), int(min(H, cy0 + 2 * ch))
        crop = img.crop((cx0, cy0, cx1, cy1))
        cbox = [box[0] - cx0, box[1] - cy0, box[2] - cx0, box[3] - cy0]

        inputs = to_dev(sam_proc(crop, input_boxes=[[cbox]],
                                 return_tensors='pt'))
        with torch.no_grad():
            sout = sam(**inputs)
        cands = sam_proc.image_processor.post_process_masks(
            sout.pred_masks.cpu(), inputs['original_sizes'].cpu(),
            inputs['reshaped_input_sizes'].cpu())[0][0]
        scores = sout.iou_scores[0, 0].cpu().numpy()
        best = int(np.argmax(scores))
        cm = keep_component_at(cands[best].numpy().astype(np.uint8),
                               (cx - cx0, cy - cy0))
        m = np.zeros((H, W), dtype=np.float32)
        m[cy0:cy1, cx0:cx1] = cm.astype(np.float32)

        save_mask(m, os.path.join(out_dir, f'stack_{n:02d}.png'))
        overlay_preview(img, m, os.path.join(out_dir, 'preview',
                                             f'{n:02d}.jpg'))
        draw_debug(img, box, None, [],
                   os.path.join(out_dir, 'debug', f'{n:02d}.jpg'),
                   extra=[(f'{query} {score:.2f}',
                           (box[0], box[1]), (255, 220, 40))])
        print(f'stack {n:2d}: "{query}" {score:.2f}, sam iou '
              f'{scores[best]:.3f}, mask {m.mean() * 100:5.2f}% of frame')

    print('done.')


if __name__ == '__main__':
    main()
