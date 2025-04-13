#!/bin/bash
# Data Collection Script for MSI Cubi Robotic Arm Computer

# Prompt for participant and session IDs
read -p "Enter participant ID: " PARTICIPANT_ID
read -p "Enter session ID: " SESSION_ID

# Define base directory and session directory
BASE_DIR="$HOME/VR_Interface/usertesting"
PARTICIPANT_DIR="${BASE_DIR}/${PARTICIPANT_ID}"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
SESSION_DIR="${PARTICIPANT_DIR}/${SESSION_ID}_${TIMESTAMP}"

# Create the session directory (and participant directory if necessary)
mkdir -p "$SESSION_DIR"
echo "Data will be saved in: $SESSION_DIR"

# Define the ROS2 topics to record and the output bag filename
BAG_FILENAME="session_${SESSION_ID}_${TIMESTAMP}"
TOPICS="/zed/zed_node/stereo/image_rect_color/compressed /controller_movement /franka_robot_state_broadcaster/current_pose /ft_sensor/data /new_goal_pose /rumble_output /joint_states"

# Capture start time of recording
recording_start=$(date +"%Y-%m-%dT%H:%M:%S")
start_sec=$(date +%s)

# Start ROS2 bag recording in the background
echo "Starting ROS2 bag recording..."
ros2 bag record -o "${SESSION_DIR}/${BAG_FILENAME}" $TOPICS &
BAG_PID=$!

echo "Recording... Press ENTER to stop."
read

# Capture end time of recording
recording_stop=$(date +"%Y-%m-%dT%H:%M:%S")
end_sec=$(date +%s)
duration=$((end_sec - start_sec))

# Stop the ROS2 bag recording process
kill $BAG_PID
echo "ROS2 bag recording stopped. Data saved in: ${SESSION_DIR}/${BAG_FILENAME}.db3"

# Prompt for additional notes
echo "Enter any additional notes for the session (press ENTER when done):"
read ADDITIONAL_NOTES

# Prompt for "Task achieved" binary question
while true; do
    read -p "Was the task achieved? (yes/no): " TASK_ACHIEVED
    case $TASK_ACHIEVED in
        [Yy]* ) TASK_ACHIEVED="yes"; break;;
        [Nn]* ) TASK_ACHIEVED="no"; break;;
        * ) echo "Please answer with yes or no."; continue;;
    esac
done

# Write standardized session info in YAML format to a YAML file
INFO_FILE="${SESSION_DIR}/session_info.yaml"
cat <<EOF > "$INFO_FILE"
participant_id: ${PARTICIPANT_ID}
session_id: ${SESSION_ID}
rosbag_file: "${BAG_FILENAME}.db3"
recording_start: ${recording_start}
recording_stop: ${recording_stop}
recording_duration_seconds: ${duration}
additional_notes: "${ADDITIONAL_NOTES}"
task_achieved: "${TASK_ACHIEVED}"
EOF

echo "Session info saved in: ${INFO_FILE}"
