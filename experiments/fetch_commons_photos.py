#!/usr/bin/env python3
'''
Fetch real mineral photographs for the classification test from Wikimedia Commons.

Takes a fixed random sample of photo files from Category:Azurite, Category:Malachite and
Category:Vanadinite (top level only), downloads a 480-pixel thumbnail of each, and writes
a manifest with each file's source page, author and licence. The photos themselves are
not committed to this repository; run this script to fetch them again.

    python3 experiments/fetch_commons_photos.py [photos per mineral, default 60]

Writes experiments/mineral_photos/<class>/<n>.jpg and experiments/results/commons_manifest.csv
'''
import csv
import json
import os
import random
import re
import sys
import time
import urllib.parse
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
API = 'https://commons.wikimedia.org/w/api.php'
UA = 'StrataCobotCourseProject/1.0 (robotics course project; contact saihardhikreddy@gmail.com)'
CATEGORIES = {'azurite_ore': 'Azurite', 'malachite_ore': 'Malachite', 'vanadinite_ore': 'Vanadinite'}


def api(params):
    url = API + '?' + urllib.parse.urlencode({**params, 'format': 'json'})
    req = urllib.request.Request(url, headers={'User-Agent': UA})
    with urllib.request.urlopen(req, timeout=60) as r:
        return json.load(r)


def category_photos(cat):
    files, cont = [], {}
    while True:
        d = api({'action': 'query', 'list': 'categorymembers', 'cmtitle': f'Category:{cat}',
                 'cmtype': 'file', 'cmlimit': 500, **cont})
        files += [m['title'] for m in d['query']['categorymembers'] if re.search(r'\.(jpe?g|png)$', m['title'], re.I)]
        if 'continue' not in d:
            return sorted(files)
        cont = d['continue']


def strip_html(s):
    return re.sub(r'<[^>]+>', '', s or '').strip()


def main(per_class):
    rng = random.Random(2026)
    rows = []
    for cls, cat in CATEGORIES.items():
        titles = rng.sample(category_photos(cat), per_class)
        out = os.path.join(HERE, 'mineral_photos', cls)
        os.makedirs(out, exist_ok=True)
        for i in range(0, len(titles), 25):
            batch = titles[i:i + 25]
            d = api({'action': 'query', 'titles': '|'.join(batch), 'prop': 'imageinfo',
                     'iiprop': 'url|extmetadata', 'iiurlwidth': 480})
            for page in d['query']['pages'].values():
                info = page['imageinfo'][0]
                meta = info.get('extmetadata', {})
                n = len([r for r in rows if r['class'] == cls])
                path = os.path.join(out, f'{n:03d}.jpg')
                req = urllib.request.Request(info['thumburl'], headers={'User-Agent': UA})
                with urllib.request.urlopen(req, timeout=60) as r, open(path, 'wb') as f:
                    f.write(r.read())
                rows.append({'class': cls, 'file': os.path.relpath(path, HERE), 'title': page['title'],
                             'source': info['descriptionurl'],
                             'author': strip_html(meta.get('Artist', {}).get('value'))[:120],
                             'licence': meta.get('LicenseShortName', {}).get('value', '')})
                time.sleep(0.2)
        print(cls, len([r for r in rows if r['class'] == cls]))
    os.makedirs(os.path.join(HERE, 'results'), exist_ok=True)
    with open(os.path.join(HERE, 'results', 'commons_manifest.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


if __name__ == '__main__':
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 60)
