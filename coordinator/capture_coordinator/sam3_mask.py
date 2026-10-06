"""Beetle mask with SAM 3 (Meta, "Segment Anything with Concepts", 2025) from a
text prompt, for the merge's focus measure (stack_merge.beetle_sharpness).

Runs in the optitrack pixi env `masking` (torch + transformers >= 5 with
Sam3Model, MPS on the Mac); the merge runs in the default env and calls this
as a subprocess on 8-bit images it decoded itself (no rawpy needed here).
Weights: facebook/sam3 from the Hugging Face cache (~3.4 GB).

    python -m capture_coordinator.sam3_mask [--prompt insect] in.jpg out.png [in2.jpg out2.png ...]

For every pair: the highest-scoring instance for the prompt -> out.png
(255 = insect, same size as the input). Prints one JSON line per pair:
{"in": ..., "out": ..., "score": 0.97, "instances": 1} (score null = nothing
found, no file written). One model load per call (~7 s), ~5-25 s per image.
"""
import argparse
import json
import os
import sys

MODEL = "facebook/sam3"


def masking_python(optitrack_repo):
    """The `masking` env interpreter of the optitrack pixi project, or None."""
    env = os.path.join(optitrack_repo, ".pixi", "envs", "masking")
    for p in (os.path.join(env, "bin", "python"), os.path.join(env, "python.exe")):      # unix, Windows
        if os.path.exists(p):
            return p
    return None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pairs", nargs="+", help="in.jpg out.png [in2 out2 ...]")
    ap.add_argument("--prompt", default="insect")
    ap.add_argument("--threshold", type=float, default=0.3, help="instance score threshold")
    ap.add_argument("--device", default=None, help="mps | cuda | cpu (default: best available)")
    a = ap.parse_args(argv)
    if len(a.pairs) % 2:
        sys.exit("give input/output pairs")
    import numpy as np
    import torch
    from PIL import Image
    from transformers import Sam3Model, Sam3Processor
    dev = a.device or ("mps" if torch.backends.mps.is_available() else "cuda" if torch.cuda.is_available() else "cpu")
    model = Sam3Model.from_pretrained(MODEL).to(dev).eval()
    proc = Sam3Processor.from_pretrained(MODEL)
    for src, dst in zip(a.pairs[0::2], a.pairs[1::2]):
        im = Image.open(src).convert("RGB")
        inp = proc(images=im, text=a.prompt, return_tensors="pt").to(dev)
        with torch.no_grad():
            out = model(**inp)
        r = proc.post_process_instance_segmentation(out, threshold=a.threshold, mask_threshold=0.5,
                                                    target_sizes=inp.get("original_sizes").tolist())[0]
        scores = r["scores"].float().cpu().numpy()
        res = {"in": src, "out": dst, "score": None, "instances": int(len(scores)), "prompt": a.prompt}
        if len(scores):
            i = int(scores.argmax())
            m = r["masks"][i].cpu().numpy().astype(np.uint8) * 255
            Image.fromarray(m).save(dst)
            res["score"] = round(float(scores[i]), 4)
        print(json.dumps(res), flush=True)


if __name__ == "__main__":
    main()
