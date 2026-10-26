#!/usr/bin/env python3
'''
Extract real mineral photographs for the classification test from the public
MineralImage5k-98 dataset (Fersman Mineralogical Museum photos, MIT licence), as published
on Hugging Face at Nech-C/mineralimage5K-98. Needs pyarrow.

The dataset has azurite (blue) and malachite (green) but no vanadinite, so carnelian, an
orange-red stone, stands in for the orange class.

    python3 experiments/extract_mineral_photos.py <folder with the test-*.parquet files>

Writes experiments/mineral_photos/<class>/<n>.jpg
'''
import io
import os
import sys

import pyarrow.parquet as pq
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
WANTED = {79: 'azurite_ore', 55: 'malachite_ore', 45: 'vanadinite_ore'}   # 45 = carnelian, the orange stand-in
MAX_SIDE = 480


def main(folder):
    counts = {c: 0 for c in WANTED.values()}
    for f in sorted(os.listdir(folder)):
        if not f.endswith('.parquet'):
            continue
        t = pq.read_table(os.path.join(folder, f), columns=['image', 'name']).to_pydict()
        for img, label in zip(t['image'], t['name']):
            if label not in WANTED:
                continue
            cls = WANTED[label]
            im = Image.open(io.BytesIO(img['bytes'])).convert('RGB')
            im.thumbnail((MAX_SIDE, MAX_SIDE))
            out = os.path.join(HERE, 'mineral_photos', cls)
            os.makedirs(out, exist_ok=True)
            im.save(os.path.join(out, f'{counts[cls]:03d}.jpg'), quality=90)
            counts[cls] += 1
    print(counts)


if __name__ == '__main__':
    main(sys.argv[1])
