#!/bin/bash
# test_nav.sh
# Builds gps_nav_interfaces + gps_nav, launches the navigator node,
# sends a goal to (0.02, 0.03), and kills everything on Ctrl+C.

set -e

WS=/ws/team48
SRC=$WS/src

# ── colours ──────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()    { echo -e "${GREEN}[test_nav]${NC} $*"; }
warn()    { echo -e "${YELLOW}[test_nav]${NC} $*"; }
die()     { echo -e "${RED}[test_nav]${NC} $*"; exit 1; }

# ── track child PIDs for cleanup ─────────────────────────────────────────────
PIDS=()

cleanup() {
    echo ""
    warn "Caught Ctrl+C — shutting down..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null && info "Killed PID $pid"
    done
    wait 2>/dev/null
    info "All done."
    exit 0
}
trap cleanup SIGINT SIGTERM

# ── 1. build ──────────────────────────────────────────────────────────────────
info "Building gps_nav_interfaces..."
cd $WS
colcon build --packages-select gps_nav_interfaces \
    --event-handlers console_cohesion+ \
    || die "gps_nav_interfaces build failed"

info "Building gps_nav..."
colcon build --packages-select gps_nav \
    --event-handlers console_cohesion+ \
    || die "gps_nav build failed"

# ── 2. source ─────────────────────────────────────────────────────────────────
info "Sourcing workspace..."
source $WS/install/setup.bash

# ── 3. launch navigator node ──────────────────────────────────────────────────
info "Starting gps_navigator node..."
ros2 run gps_nav gps_navigator &
NAV_PID=$!
PIDS+=($NAV_PID)
info "Navigator PID: $NAV_PID"

# Give the node a moment to initialise
info "Waiting for node to start..."
sleep 3

# Check it's still alive
if ! kill -0 $NAV_PID 2>/dev/null; then
    die "Navigator node died on startup — check your ROS environment"
fi

# ── 4. send goal ──────────────────────────────────────────────────────────────
info "Sending goal: lat=0.02  lon=0.03  radius=2.0 m"
ros2 action send_goal \
    /gps_navigator/navigate_to_gps \
    gps_nav_interfaces/action/GpsGoal \
    "{latitude: 0.02, longitude: 0.03, arrival_radius: 2.0}" \
    --feedback &
GOAL_PID=$!
PIDS+=($GOAL_PID)

# ── 5. wait (Ctrl+C triggers cleanup) ────────────────────────────────────────
info "Navigator running. Press Ctrl+C to stop everything."
wait $NAV_PID
warn "Navigator node exited on its own (PID $NAV_PID)."
cleanup
