#!/usr/bin/env python3
'''
Calibrate the colour vote on real photographs, then test it on photographs it never saw.

The photos fetched by fetch_commons_photos.py are split per mineral: 000-029 calibrate,
030-059 are held out. A grid search on the calibration half picks the value floors for
blue and green, and the vote's minimum share and ratio, maximising
    correct - 2 x wrong
(a wrong mineral sends an ore to the wrong container; 'uncertain' only leaves it). The
orange limits stay as they are, to keep brown rock out of vanadinite.

    python3 experiments/calibrate_colour.py

Prints the chosen values and the held-out accuracy; writes
experiments/results/colour_calibration.csv.
'''
import csv
import glob
import importlib.util
import itertools
import os

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


P = load('ore_perception', os.path.join(ROOT, 'src/strata_cobot/scripts/ore_perception.py'))
R = load('real_mineral_photos', os.path.join(HERE, 'real_mineral_photos.py'))
CLASSES = ('azurite_ore', 'malachite_ore', 'vanadinite_ore')


def photos(split):
    out = []
    for cls in CLASSES:
        for f in sorted(glob.glob(os.path.join(HERE, 'mineral_photos', cls, '*.jpg'))):
            n = int(os.path.basename(f)[:3])
            if (split == 'calibration') == (n < 30):
                bgr = cv2.imread(f)
                out.append((cls, cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV), R.specimen_mask(bgr)))
    return out


def configure(v_blue, v_green, share, ratio):
    P.mineral_colours['azurite_ore'] = [((95, 80, v_blue), (130, 255, 255))]
    P.mineral_colours['malachite_ore'] = [((40, 80, v_green), (90, 255, 255))]
    P.min_share, P.min_ratio = share, ratio


def evaluate(data):
    correct = wrong = 0
    for cls, hsv, mask in data:
        got, _ = P.classify_face(hsv, mask)
        correct += got == cls
        wrong += got is not None and got != cls
    return correct, wrong, len(data)


def main():
    cal, held = photos('calibration'), photos('held-out')
    grid = list(itertools.product((15, 25, 40), (15, 25, 40), (0.03, 0.05, 0.10), (1.5, 2.0, 3.0)))
    rows = []
    for g in grid:
        configure(*g)
        c, w, n = evaluate(cal)
        rows.append({'v_blue': g[0], 'v_green': g[1], 'min_share': g[2], 'min_ratio': g[3],
                     'correct': c, 'wrong': w, 'n': n, 'objective': c - 2 * w})
    best = max(rows, key=lambda r: (r['objective'], r['min_ratio'], r['min_share']))
    with open(os.path.join(HERE, 'results', 'colour_calibration.csv'), 'w', newline='') as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0]))
        wr.writeheader()
        wr.writerows(rows)
    print('chosen on calibration photos:', {k: best[k] for k in ('v_blue', 'v_green', 'min_share', 'min_ratio')},
          f"-> {best['correct']}/{best['n']} correct, {best['wrong']} wrong")

    for label, params in (('simulator values', (40, 40, 0.10, 2.0)),
                          ('calibrated values', (best['v_blue'], best['v_green'], best['min_share'], best['min_ratio']))):
        configure(*params)
        print(f'held-out photos, {label}:')
        for cls in CLASSES:
            c, w, n = evaluate([d for d in held if d[0] == cls])
            print(f'   {cls:15s} {c}/{n} correct ({c / n:.0%}), {w} wrong, {n - c - w} uncertain')
        c, w, n = evaluate(held)
        print(f'   {"all":15s} {c}/{n} correct ({c / n:.0%}), {w} wrong ({w / n:.0%})')


if __name__ == '__main__':
    os.makedirs(os.path.join(HERE, 'results'), exist_ok=True)
    main()
