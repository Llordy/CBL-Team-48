#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/.venv"


if [[ ! -d "$VENV_DIR" ]]; then
    echo "[setup] Creating venv at $VENV_DIR"
    bash "$SCRIPT_DIR/make_venv.sh"
else
    echo "[setup] Using existing venv at $VENV_DIR"
fi

# ─────────────────────────────────────────────────────────────────
#  Process Management & Cleanup Logic
# ─────────────────────────────────────────────────────────────────
PID_SENSOR=""
PID_DASH=""

cleanup() {
    echo -e "\n[shutdown] Stopping all nodes safely..."
    
    # Send SIGINT (2) to both backgrounds so their try/except blocks can catch it
    if [[ -n "$PID_SENSOR" ]]; then
        kill -2 "$PID_SENSOR" 2>/dev/null || true
    fi
    if [[ -n "$PID_DASH" ]]; then
        kill -2 "$PID_DASH" 2>/dev/null || true
    fi

    # Wait for them to cleanly exit their shutdown routines
    wait "$PID_SENSOR" 2>/dev/null || true
    wait "$PID_DASH" 2>/dev/null || true
    echo "[shutdown] Goodbye."
}

# Trap SIGINT, SIGTERM, and normal script EXIT to ensure cleanup runs
trap cleanup SIGINT SIGTERM EXIT

# ─────────────────────────────────────────────────────────────────
#  Execution Pipeline
# ─────────────────────────────────────────────────────────────────

echo "[run] Launching Sensor Node..."
python3 "$SCRIPT_DIR/sensor.py" &
PID_SENSOR=$!

sleep 1

echo "[run] Launching Dashboard Node..."
# Note: Fixed a small typo from your snippet here ("VENV_DIR" -> "$VENV_DIR")
"$VENV_DIR/bin/python" "$SCRIPT_DIR/dash.py" &
PID_DASH=$!

echo "[run] Nodes are active. Press Ctrl+C to stop both."

# Wait indefinitely for both background tasks to exit natively or via signal
wait "$PID_SENSOR" "$PID_DASH"


