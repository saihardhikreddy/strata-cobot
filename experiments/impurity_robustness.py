#!/usr/bin/env python3
'''
How well do the two ore detectors cope with ores that are not one clean colour?

Takes a real frame from the simulated RealSense (experiments/data), paints impurities over
part of each ore's coloured face - dark rock, brown and white matrix, and veins of the
other two minerals - and varies the lighting. It then runs:

  * baseline: the first detector, which thresholds each mineral's colour tightly and keeps
    blobs that are big and square enough;
  * voting:   the current detector (src/strata_cobot/scripts/ore_perception.py), which finds
    each face by its height in the depth image and classifies it by a vote over its pixels.

An ore counts as correct when a detection of the right mineral lies within 30 mm of it.
Writes experiments/results/impurity_robustness.csv and a plot in docs/images/.

    python3 experiments/impurity_robustness.py
'''
import csv
import importlib.util
import json
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
spec = importlib.util.spec_from_file_location('ore_perception', os.path.join(ROOT, 'src/strata_cobot/scripts/ore_perception.py'))
P = importlib.util.module_from_spec(spec)
spec.loader.exec_module(P)

LEVELS = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]   # share of each ore's coloured area covered
TRIALS = 40
MATCH_M = 0.03
rng = np.random.default_rng(2026)

# ---------------- the first detector, kept here for comparison ----------------
baseline_ranges = {
    'azurite_ore':    ((98, 200, 100), (112, 255, 255)),
    'malachite_ore':  ((64, 200, 120), (84, 255, 255)),
    'vanadinite_ore': ((5, 225, 180), (14, 255, 255)),
}


def baseline_detect(bgr, xyz):
    '''Tight colour thresholds; blobs of 60-4000 px filling 60% of their rectangle;
    position = median of the blob's 3D points.'''
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    out = []
    for mineral, (lo, hi) in baseline_ranges.items():
        m = cv2.inRange(hsv, np.array(lo), np.array(hi))
        m = cv2.morphologyEx(cv2.morphologyEx(m, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, k)
        cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cs:
            a = cv2.contourArea(c)
            (_, _), (w, h), _ = cv2.minAreaRect(c)
            if not 60 <= a <= 4000 or w * h <= 0 or a / (w * h) < 0.6:
                continue
            face = np.zeros(m.shape, np.uint8)
            cv2.drawContours(face, [c], -1, 255, cv2.FILLED)
            pts = xyz[face > 0]
            pts = pts[np.all(np.isfinite(pts), axis=1)]
            if len(pts) >= 5:
                out.append((mineral, np.median(pts, axis=0)))
    return out


def voting_detect(bgr, xyz):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    return [(P.classify_face(hsv, mask)[0], c) for mask, c in P.find_faces(xyz)]


# ---------------- impurities and lighting ----------------
def hsv_colour(h, s, v):
    return cv2.cvtColor(np.uint8([[[h, s, v]]]), cv2.COLOR_HSV2BGR)[0, 0].astype(int)


mineral_hue = {'azurite_ore': (100, 125), 'malachite_ore': (50, 80), 'vanadinite_ore': (4, 16)}


def impurity_colour(own):
    r = rng.random()
    if r < 0.40:                                          # dark rock
        g = rng.integers(15, 70)
        return np.array([g, g, g]) + rng.integers(-8, 9, 3)
    if r < 0.70:                                          # brown, iron-stained matrix
        return hsv_colour(rng.integers(8, 20), rng.integers(60, 130), rng.integers(50, 120))
    if r < 0.85:                                          # white quartz or calcite
        return hsv_colour(rng.integers(0, 180), rng.integers(0, 30), rng.integers(170, 240))
    other = rng.choice([m for m in mineral_hue if m != own])   # a vein of another mineral
    lo, hi = mineral_hue[other]
    return hsv_colour(rng.integers(lo, hi), rng.integers(150, 255), rng.integers(130, 255))


def contaminate(bgr, patches, level):
    '''Cover `level` of each ore's coloured patch with random blotches, then vary the light.'''
    img = bgr.copy()
    for own, patch in patches:
        ys, xs = np.nonzero(patch)
        target = int(level * len(ys))
        covered = np.zeros(patch.shape, bool)
        while covered[patch].sum() < target:
            i = rng.integers(len(ys))
            blot = np.zeros(patch.shape, np.uint8)
            axes = (int(rng.integers(1, 4)), int(rng.integers(1, 4)))
            cv2.ellipse(blot, (int(xs[i]), int(ys[i])), axes, float(rng.uniform(0, 180)), 0, 360, 255, -1)
            blot = (blot > 0) & patch
            img[blot] = np.clip(impurity_colour(own), 0, 255)
            covered |= blot
    gain = rng.uniform(0.6, 1.3) * rng.uniform(0.92, 1.08, 3)        # brightness and colour cast
    img = img.astype(np.float32) * gain + rng.normal(0, 5, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def score(found, truth):
    '''Per ore: correct, wrong (only a wrong mineral nearby), or missed.'''
    res = []
    for t in truth:
        near = [m for m, c in found if np.linalg.norm(np.asarray(c) - t['pos']) < MATCH_M]
        res.append('correct' if t['kind'] in near else ('wrong' if any(m for m in near) else 'missed'))
    return res


def main():
    d = os.path.join(HERE, 'data')
    bgr = cv2.imread(os.path.join(d, 'color.png'))
    depth = np.load(os.path.join(d, 'depth.npy')).astype(np.float32)
    depth[~np.isfinite(depth) | (depth <= 0)] = np.nan
    meta = json.load(open(os.path.join(d, 'meta.json')))
    truth = [{'kind': t['kind'], 'pos': np.array(t['pos'])} for t in meta['truth']]
    xyz = P.back_project(depth, meta['k'], np.load(os.path.join(d, 'cam_to_base.npy')))

    # Each ore's coloured patch: the face pixels showing its own colour in the clean frame.
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    patches = []
    for mask, c in P.find_faces(xyz):
        t = min(truth, key=lambda t: np.linalg.norm(c - t['pos']))
        own = np.zeros(mask.shape, np.uint8)
        for lo, hi in P.mineral_colours[t['kind']]:
            own |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        patches.append((t['kind'], (own > 0) & (mask > 0)))

    rows = []
    for level in LEVELS:
        tally = {m: {'correct': 0, 'wrong': 0, 'missed': 0} for m in ('baseline', 'voting')}
        for _ in range(TRIALS):
            img = contaminate(bgr, patches, level)
            for name, fn in (('baseline', baseline_detect), ('voting', voting_detect)):
                for r in score(fn(img, xyz), truth):
                    tally[name][r] += 1
        n = TRIALS * len(truth)
        for name, t in tally.items():
            rows.append({'impurity': level, 'method': name, 'ores': n,
                         'correct': t['correct'] / n, 'wrong': t['wrong'] / n, 'missed': t['missed'] / n})
        print(f"impurity {level:.0%}: baseline {tally['baseline']['correct'] / n:.0%} correct, "
              f"voting {tally['voting']['correct'] / n:.0%} correct "
              f"({tally['voting']['wrong'] / n:.0%} wrong, {tally['voting']['missed'] / n:.0%} uncertain)")
        if level in (0.0, 0.4, 0.6):
            cv2.imwrite(os.path.join(HERE, 'results', f'contaminated_{int(level * 100)}.png'), img)

    with open(os.path.join(HERE, 'results', 'impurity_robustness.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


if __name__ == '__main__':
    os.makedirs(os.path.join(HERE, 'results'), exist_ok=True)
    sys.exit(main())
