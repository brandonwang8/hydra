#!/usr/bin/env bash
# upload_2040.sh — safely copy code.py to Pimoroni Motor 2040 (CIRCUITPY)

set -e

CODE_FILE="motor2040_code/code.py"
MOUNT_POINT="/media/pi/CIRCUITPY"

echo "Searching for CIRCUITPY device..."
DEV_PATH=$(lsblk -rno NAME,LABEL | grep CIRCUITPY | awk '{print "/dev/" $1}')

if [ -z "$DEV_PATH" ]; then
    echo "No CIRCUITPY device found. Please plug in your Motor 2040."
    exit 1
fi

echo "Found device at $DEV_PATH"

# Create mount directory if it doesn't exist
if [ ! -d "$MOUNT_POINT" ]; then
    echo "Creating mount point at $MOUNT_POINT..."
    sudo mkdir -p "$MOUNT_POINT"
fi

# Check if already mounted
if mount | grep -q "$MOUNT_POINT"; then
    echo "Already mounted."
else
    echo "Mounting CIRCUITPY..."
    sudo mount -o uid=$(id -u),gid=$(id -g) "$DEV_PATH" "$MOUNT_POINT"
fi

# Double-check that it’s writable
if [ ! -w "$MOUNT_POINT" ]; then
    echo "CIRCUITPY is not writable (may be in safe mode or read-only)."
    echo "   Try unplugging and replugging the board."
    exit 1
fi

# Copy code.py
echo "Copying $CODE_FILE to CIRCUITPY..."
cp "$CODE_FILE" "$MOUNT_POINT/code.py"
sync
echo "Upload complete."

# Optional: unmount to ensure flush
echo "Flushing and unmounting..."
sudo umount "$MOUNT_POINT" || echo "⚠️ Could not unmount (device might be in use)."

echo "Done!"
