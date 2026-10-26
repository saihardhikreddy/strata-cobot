#!/usr/bin/env python3
'''
Do the colour classifiers work on real minerals, which are never one clean colour?

Runs on the museum photographs extracted by extract_mineral_photos.py. A photo has no
depth, so the specimen is separated from its plain backdrop by colour: pixels that differ
from the median border colour by more than a set distance in Lab. Then:

  * baseline: the first detector's tight colour ranges; the class with the most pixels
    wins if it covers at least 1% of the specimen;
  * voting:   classify_face() from ore_perception.py on the specimen mask.

The vote's thresholds were calibrated on photos 000-029 of each mineral
(calibrate_colour.py); photos 030-059 are held out.

    python3 experiments/real_mineral_photos.py

Writes experiments/results/real_mineral_photos.csv
'''
import csv
import glob
import importlib.util
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
spec = importlib.util.spec_from_file_location('ore_perception', os.path.join(ROOT, 'src/strata_cobot/scripts/ore_perception.py'))
P = importlib.util.module_from_spec(spec)
spec.loader.exec_module(P)

baseline_ranges = {
    'azurite_ore':    ((98, 200, 100), (112, 255, 255)),
    'malachite_ore':  ((64, 200, 120), (84, 255, 255)),
    'vanadinite_ore': ((5, 225, 180), (14, 255, 255)),
}
LAB_DIST = 22


def specimen_mask(bgr):
    '''Pixels that differ from the photo's backdrop, taken as the median border colour.'''
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
    b = np.concatenate((lab[:4].reshape(-1, 3), lab[-4:].reshape(-1, 3), lab[:, :4].reshape(-1, 3), lab[:, -4:].reshape(-1, 3)))
    dist = np.linalg.norm(lab - np.median(b, axis=0), axis=2)
    m = (dist > LAB_DIST).astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab_, stats, _ = cv2.connectedComponentsWithStats(m)
    if n <= 1:
        return np.full(m.shape, 255, np.uint8)
    keep = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))      # the specimen = largest region
    return (lab_ == keep).astype(np.uint8) * 255


def baseline_classify(hsv, mask):
    total = max(1, cv2.countNonZero(mask))
    counts = {m: cv2.countNonZero(cv2.inRange(hsv, np.array(lo), np.array(hi)) & mask) for m, (lo, hi) in baseline_ranges.items()}
    best = max(counts, key=counts.get)
    return best if counts[best] / total >= 0.01 else None


def main():
    rows = []
    for cls in ('azurite_ore', 'malachite_ore', 'vanadinite_ore'):
        files = sorted(glob.glob(os.path.join(HERE, 'mineral_photos', cls, '*.jpg')))
        for f in files:
            bgr = cv2.imread(f)
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            mask = specimen_mask(bgr)
            voted, shares = P.classify_face(hsv, mask)
            rows.append({'photo': os.path.relpath(f, HERE), 'true': cls, 'baseline': baseline_classify(hsv, mask) or 'none',
                         'voting': voted or 'uncertain', **{f'share_{k.split("_")[0]}': round(v, 3) for k, v in shares.items()}})
    with open(os.path.join(HERE, 'results', 'real_mineral_photos.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    held = [x for x in rows if int(os.path.basename(x['photo'])[:3]) >= 30]
    print('held-out photos (030-059 of each mineral; calibrate_colour.py tunes on 000-029):')
    summary(held)
    print('all photos:')
    summary(rows)


def summary(rows):
    for cls in ('azurite_ore', 'malachite_ore', 'vanadinite_ore'):
        r = [x for x in rows if x['true'] == cls]
        n = len(r)
        b = sum(x['baseline'] == cls for x in r)
        v = sum(x['voting'] == cls for x in r)
        vw = sum(x['voting'] not in (cls, 'uncertain') for x in r)
        print(f'{cls:15s} n={n:3d}  baseline {b / n:.0%}  voting {v / n:.0%} correct, {vw / n:.0%} wrong, '
              f'{sum(x["voting"] == "uncertain" for x in r) / n:.0%} uncertain')
    n = len(rows)
    print(f'all            n={n:3d}  baseline {sum(x["baseline"] == x["true"] for x in rows) / n:.0%}  '
          f'voting {sum(x["voting"] == x["true"] for x in rows) / n:.0%} correct, '
          f'{sum(x["voting"] not in (x["true"], "uncertain") for x in rows) / n:.0%} wrong')


if __name__ == '__main__':
    os.makedirs(os.path.join(HERE, 'results'), exist_ok=True)
    main()
