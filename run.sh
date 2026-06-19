#!/bin/env bash

LOG=/dev/null

#This script must be run from the  root of the bundle
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash

echo "[launch] building workspace"
colcon build
source install/setup.bash
export TURTLEBOT3_MODEL=burger
echo MODEL= $TURTLEBOT3_MODEL

PIDS=()

if [ $1 == '-g' ] || [ $1 == "--gazebo" ];then 
    GAZEBO=1
    echo "[launch] launching gazebo"
    ros2 launch my_tb3_world new_world.launch.py &>$LOG &
    PIDS+=($!)

else 
    echo "[launch] skipping gazebo"
    echo "-g or --gazebo not passed. assuming connected turtlebot."
fi

# launch nav
echo "[launch] navigation"
python3 NavNode.py &>$LOG &
PIDS+=($!)

#launch obstacle avoidance
echo "[launch] launching obstacle avoidance"
ros2 launch tb3_twin_avoidance twin_avoidance.launch.py &>$LOG &
PIDS+=($!)

#launch battery monitor
echo "[launch] launching battery monitor"
python3 ./BatteryMonitor.py &>$LOG &
PIDS+=($!)
python3 ./ReturnToBase.py &>$LOG &
PIDS+=($!)

#launch dashboard
echo "[launch] launching dashboard"
cd ./dashboard/ 
./run.sh &
RUN_PID=$!
PIDS+=($RUN_PID)
cd ..

echo "[launch] The dashboard will be available at http://localhost:8080"

echo "[launch] All launched"
 
cleanup() {
    echo -e "\n[launch] Caught termination signal. Sending SIGINT to all processes..."
    
    echo "dashboard at $RUN_PID"
    kill -2 $RUN_PID

    # 1. Send SIGINT to all tracked processes
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -s SIGINT "$pid" 2>/dev/null
        fi
    done

    # 2. Wait up to 10 seconds for them to exit gracefully
    echo "[launch] Waiting up to 15 seconds for processes to exit gracefully..."
    for i in {1..15}; do
        # Check if any processes are still alive
        local alive=0
        for pid in "${PIDS[@]}"; do
            if kill -0 "$pid" 2>/dev/null; then
                alive=$((alive + 1))
            fi
        done
        
        # If everything is dead, we can exit early
        if [ "$alive" -eq 0 ]; then
            echo "[launch] All processes exited cleanly."
            exit 0
        fi
        
        sleep 1
    done

    pkill -9 -P $RUN_PID
    # 3. Forcefully kill anything still running after 10 seconds
    echo "[launch] 10 seconds elapsed. Forcefully terminating remaining processes (SIGKILL)..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "[launch] Force killing PID $pid"
            kill -s SIGKILL "$pid" 2>/dev/null
        fi
    done

    exit 0
}

trap cleanup SIGINT SIGTERM

echo "[launch] waiting for background processes to finish. Press Ctrl+C to stop all."

# Main loop to wait while background processes are alive
while [ ${#PIDS[@]} -gt 0 ]; do
    for i in "${!PIDS[@]}"; do
        pid=${PIDS[$i]}
        if ! kill -0 "$pid" 2>/dev/null; then
            unset 'PIDS[i]'
        fi
    done
    PIDS=("${PIDS[@]}")
    sleep 1
done

echo "[launch] All background scripts have finished."

