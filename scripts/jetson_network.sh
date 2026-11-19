#!/bin/bash
# Put the Jetson's lidar Ethernet port on 192.168.1.5/24 and check that the Mid-360 answers.
# Usage: scripts/jetson_network.sh <ethernet-iface> <lidar-serial>
#   e.g. scripts/jetson_network.sh eth0 47MDL5Q00100123
set -euo pipefail

IFACE="${1:?usage: $0 <ethernet-iface> <lidar-serial>}"
SERIAL="${2:?usage: $0 <ethernet-iface> <lidar-serial>}"
HOST_IP=192.168.1.5/24
LIDAR_IP="192.168.1.1$(echo -n "$SERIAL" | grep -o '[0-9]' | tail -n 2 | tr -d '\n')"
CON=elev-lidar

if ! ip link show "$IFACE" >/dev/null 2>&1; then
  echo "no interface $IFACE (list them with: ip -br link)" >&2
  exit 1
fi

if nmcli -t -f NAME con show | grep -qx "$CON"; then
  sudo nmcli con modify "$CON" ifname "$IFACE" ipv4.method manual ipv4.addresses "$HOST_IP" \
       ipv4.never-default yes ipv6.method disabled
else
  sudo nmcli con add type ethernet ifname "$IFACE" con-name "$CON" ipv4.method manual \
       ipv4.addresses "$HOST_IP" ipv4.never-default yes ipv6.method disabled
fi
sudo nmcli con up "$CON" >/dev/null
sudo sysctl -q -w net.core.rmem_max=67108864 net.core.rmem_default=67108864 || true

echo "host $IFACE: $(ip -br addr show "$IFACE" | awk '{print $3}')"
echo "pinging lidar at $LIDAR_IP (give it ~20 s after power on)..."
if ping -c 3 -W 1 "$LIDAR_IP" >/dev/null; then
  echo "OK: lidar reachable at $LIDAR_IP"
else
  echo "FAIL: no reply from $LIDAR_IP. Check the cable, power (Mid-360 needs 9-27 V), and that"
  echo "      tools/set_lidar_ip.py was run with this serial." >&2
  exit 2
fi
