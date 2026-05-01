#!/usr/bin/env bash
# RPi one-time setup script for Project NUEVO.
#
# Covers:
#   1. Hardware UART enable (Arduino bridge on /dev/ttyAMA0)
#   2. SysRq hardening (prevents accidental read-only filesystem corruption)
#   3. Docker + Docker Compose installation
#
# Run on the Raspberry Pi:
#   sudo bash setup_rpi.sh

set -e

if [ "$(id -u)" -ne 0 ]; then
    echo "Error: run this script with sudo." >&2
    exit 1
fi

BOOT=/boot/firmware

# Determine the active cmdline.txt location
# Ubuntu RPi uses current/; standard RPi OS uses /boot/firmware/ directly
if [ -f "$BOOT/current/cmdline.txt" ]; then
    CMDLINE="$BOOT/current/cmdline.txt"
else
    CMDLINE="$BOOT/cmdline.txt"
fi

CONFIG="$BOOT/config.txt"

# ── 1. UART Setup (section 3.1 of Lab 1) ────────────────────────────────────
# The bridge communicates with the Arduino over the RPi hardware UART.
# enable_uart=1 and dtoverlay=uart0-pi5 must be present in config.txt,
# and the OS must not claim the port as a serial console.

echo "=== [1/3] Enabling hardware UART ==="

mount -o remount,rw "$BOOT"

if grep -q "enable_uart=1" "$CONFIG"; then
    echo "  enable_uart=1 already present, skipping."
else
    echo "enable_uart=1" >> "$CONFIG"
    echo "  Added enable_uart=1"
fi

if grep -q "dtoverlay=uart0-pi5" "$CONFIG"; then
    echo "  dtoverlay=uart0-pi5 already present, skipping."
else
    echo "dtoverlay=uart0-pi5" >> "$CONFIG"
    echo "  Added dtoverlay=uart0-pi5"
fi

echo "  Removing serial console entries from $CMDLINE ..."
cp "$CMDLINE" "${CMDLINE}.bak"
sed -i 's/console=serial0,[0-9]* \?//g' "$CMDLINE"
sed -i 's/console=ttyAMA0,[0-9]* \?//g' "$CMDLINE"
sed -i 's/console=serial0 \?//g'        "$CMDLINE"
sed -i 's/console=ttyAMA0 \?//g'        "$CMDLINE"
echo "  Backup saved to ${CMDLINE}.bak"
echo "  cmdline: $(cat "$CMDLINE")"

# ── 2. SysRq Hardening ───────────────────────────────────────────────────────
# With console=serial0 removed above, sysrq-u can no longer be triggered by
# the Arduino UART. This disables sysrq entirely as defence-in-depth.

echo ""
echo "=== [2/3] Disabling SysRq ==="
echo "kernel.sysrq=0" > /etc/sysctl.d/99-disable-sysrq.conf
sysctl -w kernel.sysrq=0 > /dev/null
echo "  kernel.sysrq=$(cat /proc/sys/kernel/sysrq) (persists across reboots)"

# ── 3. Docker + Docker Compose (section 3.4 of Lab 1) ───────────────────────

echo ""
echo "=== [3/3] Installing Docker ==="

if command -v docker &>/dev/null; then
    echo "  Docker already installed: $(docker --version)"
else
    echo "  Downloading and running Docker install script ..."
    curl -fsSL https://get.docker.com -o /tmp/get-docker.sh
    sh /tmp/get-docker.sh
    rm /tmp/get-docker.sh
    echo "  Docker installed: $(docker --version)"
fi

# Add the invoking user (the one who ran sudo) to the docker group
ACTUAL_USER="${SUDO_USER:-$USER}"
if id -nG "$ACTUAL_USER" | grep -qw docker; then
    echo "  $ACTUAL_USER is already in the docker group."
else
    usermod -aG docker "$ACTUAL_USER"
    echo "  Added $ACTUAL_USER to the docker group."
    echo "  Note: log out and back in (or run 'newgrp docker') for this to take effect."
fi

if docker compose version &>/dev/null; then
    echo "  Docker Compose: $(docker compose version)"
else
    echo "  Warning: 'docker compose' not available — may need a newer Docker version."
fi

# ── Summary ──────────────────────────────────────────────────────────────────

echo ""
echo "========================================"
echo " Setup complete. Summary:"
echo "========================================"
echo "  UART          : enable_uart=1 + dtoverlay=uart0-pi5 in config.txt"
echo "  Serial console: removed from cmdline.txt"
echo "  SysRq         : disabled (0)"
echo "  Docker        : $(docker --version 2>/dev/null || echo 'installed')"
echo ""
echo "Reboot to apply UART and cmdline changes:"
echo "  sudo reboot"
echo ""
echo "After reboot, verify:"
echo "  ls -l /dev/ttyAMA0          # should exist, owned by dialout group"
echo "  mount | grep 'on / '        # should show (rw,...)"
echo "  cat /proc/sys/kernel/sysrq  # should be 0"
