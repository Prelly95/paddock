# Paddock — farm inspection drone autonomy stack. Task runner.
# Run `just` with no args to see this list.
#
# Typical first-run order:
#   just build && just px4-clone && just ws-clone && just px4-build
# Then, in three terminals: `just agent`, `just sitl`, `just topics`.

# Every recipe below execs through this so ROS/Gazebo env vars are always
# loaded. `docker exec -it ... bash -c "..."` runs a NON-interactive shell
# (the -c flag overrides that even with a tty attached), so ~/.bashrc is
# never auto-sourced here. We source /opt/ros/jazzy/setup.bash directly
# rather than routing through .bashrc, since some base images guard
# .bashrc with an "if not interactive, exit early" check that would
# silently defeat this too. GZ_IP/GZ_PARTITION are also set as real
# container env vars in docker-compose.yml (those aren't shell-sourced, so
# they're already present) — re-exporting here is belt-and-braces.
exec_prefix := "docker exec -it paddock bash -c"
ros_env := "source /opt/ros/jazzy/setup.bash && [ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash; export GZ_IP=127.0.0.1 GZ_PARTITION=paddock &&"

# PX4 release we target. Must match PX4_VERSION in the Dockerfile, and
# px4_msgs must be on the matching release branch or the uORB message
# definitions won't line up and topics silently fail to deserialize.
px4_version := "v1.17.0"
px4_msgs_branch := "release/1.17"

default:
    @just --list

# One-time (or after Dockerfile changes): build the image
build:
    docker compose build

# Force a rebuild (use after editing the Dockerfile) and restart the container
rebuild:
    docker compose up -d --build

# Safe to run every time — xhost grants are per-session and don't persist.
# Grant X11/XWayland access and start the container
up:
    xhost +local:docker
    docker compose up -d
    @echo "Container running. Use 'just shell' to enter it."

# This one's a real interactive shell, so .bashrc sources normally —
# no need for the ros_env prefix here.
# Drop into a shell in the container (main way you'll work day to day)
shell: up
    docker exec -it paddock bash

# Stop and remove the container (image, ./ws, ./px4 and ccache are kept)
down:
    docker compose down

# Nuke everything including the image and the ccache volume
clean:
    docker compose down --rmi all -v

# Quick sanity check that GPU/X11 rendering works before doing anything else
verify: up
    {{exec_prefix}} "{{ros_env}} gz sim shapes.sdf"

# --- one-time source checkouts -------------------------------------------
# Both clone into bind-mounted dirs, so the source lands on the host where
# your editor can reach it, owned by you (container runs as uid 1000).

# Clone PX4-Autopilot at the pinned tag into ./px4
px4-clone: up
    {{exec_prefix}} "test -d ~/PX4-Autopilot/.git && echo 'PX4 already cloned — skipping' || \
      git clone --recursive --shallow-submodules --depth 1 --branch {{px4_version}} \
        https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot"

# Clone px4_msgs (matching release branch) into ./ws/src
ws-clone: up
    {{exec_prefix}} "test -d ~/ros2_ws/src/px4_msgs/.git && echo 'px4_msgs already cloned — skipping' || \
      git clone --depth 1 --branch {{px4_msgs_branch}} \
        https://github.com/PX4/px4_msgs.git ~/ros2_ws/src/px4_msgs"

# --- build ----------------------------------------------------------------

# Compile PX4 SITL (slow the first time, ccache-warm after)
px4-build: up
    {{exec_prefix}} "{{ros_env}} cd ~/PX4-Autopilot && make px4_sitl -j$(nproc)"

# Build the ROS 2 workspace (px4_msgs + your own packages under ./ws/src)
colcon-build: up
    {{exec_prefix}} "{{ros_env}} cd ~/ros2_ws && colcon build --symlink-install"

# --- run ------------------------------------------------------------------

# Start this FIRST and leave it running in its own terminal — PX4 SITL
# starts the client side and connects out to it.
# Run the uXRCE-DDS agent (the PX4 <-> ROS 2 bridge)
agent: up
    {{exec_prefix}} "MicroXRCEAgent udp4 -p 8888"

# PX4 SITL + Gazebo with the stock standard_vtol airframe
sitl: up
    {{exec_prefix}} "{{ros_env}} cd ~/PX4-Autopilot && make px4_sitl gz_standard_vtol"

# Same, headless (no Gazebo GUI) — much lighter when you only need topics
sitl-headless: up
    {{exec_prefix}} "{{ros_env}} cd ~/PX4-Autopilot && HEADLESS=1 make px4_sitl gz_standard_vtol"

# Apply docker/sitl-params.txt to the running PX4. Run once after each
# `just sitl` — SITL starts from airframe defaults every time, and without
# this the vehicle refuses to arm with a generic health-check failure.
sim-params: up
    docker cp docker/sitl-params.txt paddock:/tmp/sitl-params.txt
    {{exec_prefix}} "cd ~/PX4-Autopilot/build/px4_sitl_default && \
      grep -vE '^[[:space:]]*(#|$)' /tmp/sitl-params.txt | while read -r name value; do \
        PATH=\$PATH:\$PWD/bin px4-param set \"\$name\" \"\$value\"; \
      done"

# Fly the inspection sortie. Needs `just agent` and `just sitl` already up.
mission: up
    {{exec_prefix}} "{{ros_env}} ros2 run paddock_mission mission_node \
      --ros-args --params-file ~/ros2_ws/install/paddock_mission/share/paddock_mission/config/paddocks.yaml"

# --- inspect --------------------------------------------------------------

# List topics — the check that the bridge is actually alive (expect /fmu/...)
topics: up
    {{exec_prefix}} "{{ros_env}} ros2 topic list"

# PX4 1.16+ version-suffixes a subset of the DDS topics; the authoritative
# list is src/modules/uxrce_dds_client/dds_topics.yaml in the PX4 tree.
# Watch the EKF2 local position estimate stream
ekf: up
    {{exec_prefix}} "{{ros_env}} ros2 topic echo /fmu/out/vehicle_local_position_v1"

# Vehicle state: arming state, nav state, and (for VTOL) current mode
status: up
    {{exec_prefix}} "{{ros_env}} ros2 topic echo /fmu/out/vehicle_status_v1"

# VTOL-specific: hover vs fixed-wing vs mid-transition
vtol: up
    {{exec_prefix}} "{{ros_env}} ros2 topic echo /fmu/out/vtol_vehicle_status"

# Open rviz2 to visualize estimator output
rviz: up
    {{exec_prefix}} "{{ros_env}} rviz2"
