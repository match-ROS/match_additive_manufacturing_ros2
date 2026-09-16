#!/usr/bin/env bash
# Run from this PC. SSH uses the key; sudo prompts on the robot's terminal.
set -euo pipefail

ssh -t -o BatchMode=yes -o StrictHostKeyChecking=yes robot@192.168.0.200 'sudo sh -c '\''
set -eu
directory=/etc/systemd/timesyncd.conf.d
configuration=$directory/99-am-operator-pc.conf
mkdir -p "$directory"
if [ -f "$configuration" ]; then
    cp -p "$configuration" "$configuration.backup.$(date +%Y%m%d%H%M%S)"
fi
printf "%s\n" "[Time]" "NTP=" "NTP=192.168.0.222" "FallbackNTP=" > "$configuration"
chmod 644 "$configuration"
systemctl enable --now systemd-timesyncd
systemctl restart systemd-timesyncd
timedatectl status
timedatectl timesync-status
'\'''
