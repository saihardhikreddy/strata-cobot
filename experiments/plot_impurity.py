#!/usr/bin/env python3
'''Plot experiments/results/impurity_robustness.csv to docs/images/impurity_robustness.png.'''
import csv
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
rows = list(csv.DictReader(open(os.path.join(HERE, 'results', 'impurity_robustness.csv'))))
series = {m: [(float(r['impurity']) * 100, float(r['correct']) * 100, float(r['wrong']) * 100)
              for r in rows if r['method'] == m] for m in ('baseline', 'voting')}

plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11, 'axes.spines.top': False, 'axes.spines.right': False})
fig, ax = plt.subplots(figsize=(7, 4))
style = {'baseline': ('#8a9399', 'First detector: tight colour thresholds'),
         'voting': ('#c4501f', 'New detector: depth shape + colour vote')}
for m, pts in series.items():
    x, y, _ = zip(*pts)
    c, label = style[m]
    ax.plot(x, y, 'o-', color=c, lw=2.4 if m == 'voting' else 1.8, label=label)
    for xi, yi in zip(x, y):
        if xi in (0, 20, 40, 60, 70):
            ax.annotate(f'{yi:.0f}%', (xi, yi), textcoords='offset points', xytext=(0, 8 if m == 'voting' else -15),
                        ha='center', fontsize=9, color=c)
wrong = max(w for _, _, w in series['voting'])
ax.text(24, 64, f'New detector: {wrong:.0f}% wrong at every level;\nwhat it cannot tell is flagged "uncertain"',
        fontsize=9, color='#5f6b74')
ax.set_xlabel('Share of each ore\'s coloured face covered by impurities (%)')
ax.set_ylabel('Ores correctly found and classified (%)')
ax.set_ylim(-12, 112)
ax.set_title('Voting over the face keeps working as ores get less pure', loc='left', color='#1f2a33')
ax.legend(frameon=False, loc='lower left', fontsize=9)
fig.tight_layout()
fig.savefig(os.path.join(ROOT, 'docs', 'images', 'impurity_robustness.png'), dpi=180)
