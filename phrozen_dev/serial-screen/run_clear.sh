#!/bin/bash
# Script: run_clear.sh
# Purpose: Clear system cache, clear /var/log files, clean apt package cache, and restart udp_server

PASSWORD="makerbase"

# Sync data to disk
echo "$PASSWORD" | sudo -S sync

# Clear system cache
echo "$PASSWORD" | sudo -S sh -c 'echo 3 > /proc/sys/vm/drop_caches'

# Stop old udp_server process
echo "$PASSWORD" | sudo -S killall udp_server 2> /dev/null || true

# Brief delay to ensure process exits safely
sleep 1

# Safely truncate all regular log files in /var/log (without deleting them)
# Skip special files (sockets, fifos, etc.), only truncate regular files
echo "Clearing log files in /var/log..."
echo "$PASSWORD" | sudo -S find /var/log -type f -exec sh -c 'echo -n "" > "$1"' _ {} \;

# Optional: if using systemd-journal, vacuum journal logs to prevent excessive space usage
if command -v journalctl &> /dev/null; then
    echo "Vacuuming systemd journal logs..."
    echo "$PASSWORD" | sudo -S journalctl --vacuum-size=10M 2> /dev/null || true
fi

# Clean apt package cache to free disk space
echo "Cleaning apt package cache..."
echo "$PASSWORD" | sudo -S apt-get clean 2> /dev/null || true

# Start new udp_server (background)
echo "$PASSWORD" | sudo -S nohup /root/udp_server &> /dev/null &

# Check if execution succeeded (note: nohup usually returns 0, more reliable to check if process exists)
sleep 1
if pgrep -x "udp_server" > /dev/null; then
    echo "Cache cleared, logs cleared, and udp_server restarted successfully."
else
    echo "Failed to restart udp_server. Please check manually."
    exit 1
fi
