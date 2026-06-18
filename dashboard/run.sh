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

    # 2. Start a 10-second background watchdog timer
    (
        echo "[shutdown] waiting 10s" >/dev/tty
        sleep 10
        # If they are still running after 10 seconds, force kill them
        echo "[shutdwon] 10 seconds elapsed. killing.">/dev/tty
        if [[ -n "$PID_SENSOR" ]] && kill -0 "$PID_SENSOR" 2>/dev/null; then
            echo "[shutdown] Sensor didn't exit in time. Force killing...">/dev/tty
            kill -9 "$PID_SENSOR" 2>/dev/null || true
        fi
        if [[ -n "$PID_DASH" ]] && kill -0 "$PID_DASH" 2>/dev/null; then
            echo "[shutdown] Dash didn't exit in time. Force killing...">/dev/tty
            kill -9 "$PID_DASH" 2>/dev/null || true
        fi
    ) &
    TIMER_PID=$!

    # 3. Wait for the processes to exit naturally
    wait "$PID_SENSOR" 2>/dev/null || true
    wait "$PID_DASH" 2>/dev/null || true

    # 4. Clean up the watchdog timer if it's still running (i.e., they exited early)
    kill "$TIMER_PID" 2>/dev/null || true
    wait "$TIMER_PID" 2>/dev/null || true

    echo "[shutdown] Goodbye."
}

# Trap SIGINT, SIGTERM, and normal script EXIT to ensure cleanup runs
trap cleanup SIGINT SIGTERM EXIT

# ─────────────────────────────────────────────────────────────────
#  Execution Pipeline
# ─────────────────────────────────────────────────────────────────

echo "[run] Launching Sensor Node..."
python3 "$SCRIPT_DIR/sensor.py" >/dev/null &
PID_SENSOR=$!

sleep 1

echo "[run] Launching Dashboard Node..."
# Note: Fixed a small typo from your snippet here ("VENV_DIR" -> "$VENV_DIR")
"$VENV_DIR/bin/python" "$SCRIPT_DIR/dash.py" >/dev/null &
PID_DASH=$!

echo "[run] Nodes are active. Press Ctrl+C to stop both."

# Wait indefinitely for both background tasks to exit natively or via signal
wait "$PID_SENSOR" "$PID_DASH"


