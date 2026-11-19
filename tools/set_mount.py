"""Write measured mount values into config/mount.yaml, keeping the comments.

    python3 tools/set_mount.py base_to_livox z=0.40 roll=0.0 pitch=0.7854
    python3 tools/set_mount.py base_to_zed x=0.10 yaw=0.0

Values are metres (x, y, z) and radians (roll, pitch, yaw), as in mount.yaml.
Use the numbers printed by `ros2 run elev_bringup check_floor`."""
import os
import re
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
PATH = os.path.join(ROOT, 'src', 'elev_bringup', 'config', 'mount.yaml')
KEYS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')


def set_mount(text, block, values):
    lines = text.splitlines(keepends=True)
    start = next((i for i, l in enumerate(lines) if re.match(r'^%s:\s*$' % re.escape(block), l)), None)
    if start is None:
        raise SystemExit('block %r not found in mount.yaml' % block)
    end = next((i for i in range(start + 1, len(lines)) if lines[i].strip() and not lines[i][0].isspace()),
               len(lines))
    seen = set()
    for i in range(start + 1, end):
        m = re.match(r'^(\s+)(\w+):(\s*)(\S+)(.*)$', lines[i].rstrip('\n'))
        if m and m.group(2) in values:
            lines[i] = '%s%s:%s%s%s\n' % (m.group(1), m.group(2), m.group(3) or ' ',
                                          '%.6f' % values[m.group(2)], m.group(5))
            seen.add(m.group(2))
    missing = set(values) - seen
    if missing:
        raise SystemExit('keys not found in block %r: %s' % (block, sorted(missing)))
    return ''.join(lines)


def main(argv):
    if len(argv) < 3:
        raise SystemExit(__doc__)
    block = argv[1]
    values = {}
    for a in argv[2:]:
        k, _, v = a.partition('=')
        if k not in KEYS:
            raise SystemExit('unknown key %r (use one of %s)' % (k, ', '.join(KEYS)))
        values[k] = float(v)
    with open(PATH) as f:
        text = f.read()
    with open(PATH, 'w') as f:
        f.write(set_mount(text, block, values))
    print('updated %s %s' % (block, values))


if __name__ == '__main__':
    main(sys.argv)
