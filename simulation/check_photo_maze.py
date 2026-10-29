"""Photo-derived topology test, NOT a physical/Arduino emulator.

Runs the actual classifier expressions extracted from cases_right_hand.cpp.
Assumes perfect turns, centered travel, and correctly confirmed openings for
route tests. Separately exercises ambiguous sensor inputs. Coordinates are
topological grid locations, not measurements extracted from perspective pixels.
Run: python simulation/check_photo_maze.py
"""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / 'Essentials/cases_right_hand.cpp').read_text()
RULES = re.findall(r'if \((.*?)\) scenario = (\d);', SOURCE)

def constant(name):
    return float(re.search(r'const (?:float|unsigned long) ' + name + r' = ([\d.]+)', SOURCE)[1])

def classify(left, mid, right, left_open=False, right_open=False):
    values = dict(distLeft=left, distMid=mid, distRight=right,
                  leftGap=left == 5, rightGap=right == 5,
                  leftOpening=left_open, rightOpening=right_open, abs=abs)
    result = 0
    for expression, number in RULES:
        expression = expression.replace('&&', ' and ').replace('||', ' or ')
        expression = expression.replace('!rightOpening', 'not rightOpening')
        if eval(expression, {'__builtins__': {}}, values):
            result = int(number)
    return result

# Image interpreted with north toward its top. Start arrows are ambiguous;
# both east- and west-facing initial headings are tested at both starts.
POINTS = {
    'start1': (3, 0), 'east_end': (3.5, 0), 'bottom_T': (2, 0),
    'start2': (1, 0), 'west_end': (.5, 0),
    'S1': (2, 1), 'S2': (3, 1), 'top_right': (3, 3),
    'top_inner': (2, 3), 'inner_corner': (2, 2),
    'upper_T': (1, 2), 'upper_short_end': (1, 3),
    'lower_corner': (1, 1), 'left_T': (0, 1),
    'lower_short_end': (0, 0), 'FINISH': (0, 3),
}
EDGES = [
    ('east_end', 'start1'), ('start1', 'bottom_T'),
    ('bottom_T', 'start2'), ('start2', 'west_end'),
    ('bottom_T', 'S1'), ('S1', 'S2'), ('S2', 'top_right'),
    ('top_right', 'top_inner'), ('top_inner', 'inner_corner'),
    ('inner_corner', 'upper_T'), ('upper_T', 'upper_short_end'),
    ('upper_T', 'lower_corner'), ('lower_corner', 'left_T'),
    ('left_T', 'lower_short_end'), ('left_T', 'FINISH'),
]
GRAPH = {name: [] for name in POINTS}
for a, b in EDGES:
    GRAPH[a].append(b)
    GRAPH[b].append(a)

def direction(a, b):
    x, y = POINTS[a]
    xx, yy = POINTS[b]
    return 0 if yy > y else 1 if xx > x else 2 if yy < y else 3

def route(start, heading):
    node = start
    visited = set()
    trace = []
    for _ in range(100):
        state = node, heading
        if state in visited:
            return 'LOOP', trace
        visited.add(state)
        exits = {direction(node, other): other for other in GRAPH[node]}
        left, front, right = ((heading - 1) % 4, heading, (heading + 1) % 4)
        case = classify(5 if left in exits else 3,
                        5 if front in exits else 2,
                        5 if right in exits else 3,
                        left in exits, right in exits)
        trace.append(f'{node}: case {case}')
        if case == 4:
            # Ideal entry-reference lengths, based on supplied 20 cm panels,
            # 7 cm posts and 5 cm stopping clearance, NOT simulated odometry.
            length = 42 if node == 'FINISH' else 15
            if constant('LONG_MIN_CM') <= length <= constant('LONG_MAX_CM'):
                return ('PASS' if node == 'FINISH' and heading == 0 else 'FALSE FINISH'), trace
        relative = {1: 0, 2: 1, 3: -1, 4: 2}.get(case)
        if relative is None:
            return 'STUCK', trace
        heading = (heading + relative) % 4
        if heading not in exits:
            return 'INVALID TURN', trace
        node = exits[heading]
    return 'LIMIT', trace

def main():
    output = ['# Photo maze logic simulation', '',
              'This is an ideal topology/classifier test, not execution of the complete Arduino sketch.',
              'Turns, body clearance, sensor rays, gap recognition and distance estimates are idealised.',
              'The maze graph was manually transcribed from the supplied photograph.', '']
    for start in ('start1', 'start2'):
        for heading, label in ((1, 'east'), (3, 'west')):
            status, trace = route(start, heading)
            assert status == 'PASS', (start, label, status, trace)
            output += [f'## {start}, facing {label}: {status}', '', ' -> '.join(trace), '']
    # Direct checks against extracted classifier, including failure conditions.
    assert classify(3, 5, 5) == 1
    assert classify(3, 5, 5, right_open=True) == 2
    assert classify(4, 2, 4) == 0
    assert classify(3, 2, 3) == 4
    assert classify(3, 2, 5, right_open=True) == 2
    # Current qualification bypasses the distance test at a blocked front.
    elapsed_ms, travelled_cm, front = 150, 0, 2
    qualifies = elapsed_ms >= constant('OPENING_CONFIRM_MS') and (
        travelled_cm >= constant('OPENING_CONFIRM_CM') or front <= 2)
    assert qualifies
    output += [
        '## Failure probes', '',
        '- L/M/R = 4/2/4 selects case 0 indefinitely if readings persist, despite a front wall.',
        '- L/M/R = 3/2/5 for 150 ms qualifies a right opening without any forward travel. '
        'A missed right wall at a dead end can therefore trigger a turn rather than finish detection.',
        '- A 7 cm post is rejected as an opening by the nominal 10 cm moving-gap filter, but this protection '
        'is bypassed when the middle reading is <= 2.',
        '- At the stated wheel speeds, the 950 rather than 1000 turn multiplier requests '
        '85.5 degrees for a quarter-turn and 171 degrees for a U-turn (before integer rounding). '
        'The ideal route test assumes exact turns and does not validate this calibration.',
        '- Finish length is supplied as ideal 42 cm; the route pass does not validate '
        'when real side sensors establish straightKnown, or the resulting measured length.',
        '', 'Conclusion: both starts pass the ideal route test, but the current sketch is '
        'not demonstrated to finish reliably on the physical maze.',
    ]
    report = '\n'.join(output) + '\n'
    (ROOT / 'simulation/photo_maze_results.md').write_text(report)
    print(report)

if __name__ == '__main__':
    main()
