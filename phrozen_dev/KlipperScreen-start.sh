#!/bin/bash
# KlipperScreen-start.sh — KAOS-patched startup for Phrozen Arco (MKS board)
# Launched by klipperscreen.service on boot.

# Create phrozen working directory
mkdir -p /home/mks/phrozen_dir

# Stop conflicting makerbase UI service
systemctl stop makerbase-client.service

# Set permissions on phrozen_dev and data directories
chmod -R 777 /home/mks/klipper/klippy/extras/phrozen_dev
chmod -R 777 /home/mks/hdlDat
chmod -R 777 /home/mks/klipper/klippy/extras/phrozen_dev/serial-screen
sleep 1

# Launch serial touchscreen driver (TJC/voronFDM)
/home/mks/klipper/klippy/extras/phrozen_dev/serial-screen/voronFDM >/dev/null 2>&1 &
sleep 1

# NTP time sync (no battery-backed RTC on this board)
/usr/sbin/ntpdate pool.ntp.org &

# Keep service alive
while true; do
    sleep 1
done
