#!/usr/bin/env bash
# Script to launch Kerbal Space Program via Windows shortcut from WSL

# Determine the script's directory and project root
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
LNK_FILE="$PROJECT_ROOT/Kerbal Minimal.lnk"

# Check if the shortcut exists
if [ ! -f "$LNK_FILE" ]; then
    echo "Error: Shortcut '$LNK_FILE' not found in $PROJECT_ROOT"
    exit 1
fi

# Convert WSL path to Windows path
WIN_PATH=$(wslpath -w "$LNK_FILE")

# Launch the shortcut using Windows command processor
# Using start "" "" to handle paths with spaces and avoid treating the first quoted string as window title
cmd.exe /c start "" "$WIN_PATH"

echo "Launched Kerbal Space Program via Windows shortcut."