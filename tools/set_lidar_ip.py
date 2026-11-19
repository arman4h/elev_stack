"""Set the Mid-360 IP in config/MID360_config.json from its serial number.
Mid-360 default IP is 192.168.1.1XX, where XX are the last two digits of the serial number.

    python3 tools/set_lidar_ip.py 47MDL5Q00100123      # last two digits "23" -> 192.168.1.123

Prints the lidar IP and the serial to pass as the launch argument lidar_sn."""
import json
import os
import re
import sys

PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'elev_bringup',
                    'config', 'MID360_config.json')


def lidar_ip(serial):
    serial = serial.strip()
    digits = re.findall(r'\d', serial)
    if len(digits) < 2:
        raise SystemExit('serial %r has no digits' % serial)
    return '192.168.1.1%s' % ''.join(digits[-2:])


def main(argv):
    if len(argv) != 2:
        raise SystemExit(__doc__)
    serial = argv[1]
    ip = lidar_ip(serial)
    with open(PATH) as f:
        cfg = json.load(f)
    cfg['lidar_configs'][0]['ip'] = ip
    with open(PATH, 'w') as f:
        json.dump(cfg, f, indent=2)
        f.write('\n')
    print('lidar ip = %s' % ip)
    print('launch with: ros2 launch elev_bringup sensors.launch.py lidar_sn:=%s' % serial)


if __name__ == '__main__':
    main(sys.argv)
