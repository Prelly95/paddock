# Paddock dev image: ROS 2 Jazzy + Gazebo Harmonic + PX4 SITL toolchain
# + the uXRCE-DDS agent that bridges PX4 <-> ROS 2.
FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Which PX4 release we target. px4_msgs must be checked out on the matching
# release/X.Y branch or the uORB message definitions won't line up and topics
# will silently fail to deserialize. See the justfile `ws-clone` recipe.
ARG PX4_VERSION=v1.17.0
# Agent version recommended by the PX4 v1.17 uXRCE-DDS docs.
ARG XRCE_AGENT_VERSION=v2.4.3

# ---------------------------------------------------------------------------
# Base tooling.
#
# Deliberately NOT installed: ros-jazzy-ros-gz. It pulls ROS "vendor" builds of
# Gazebo into /opt/ros/jazzy/opt/gz_*_vendor, which collide with the OSRF
# Gazebo Harmonic that PX4 compiles and links against. Paddock doesn't need
# ros_gz at all: PX4 owns the Gazebo process, and ROS 2 talks to PX4 over
# uXRCE-DDS rather than over gz-transport.
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates \
      curl \
      wget \
      gnupg \
      lsb-release \
      sudo \
      git \
      python3-pip \
      python3-colcon-common-extensions \
      x11-apps \
      iproute2 \
      nano \
      less \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# PX4 toolchain + Gazebo Harmonic, installed by PX4's own setup script so we
# get exactly the dependency set upstream tests against (including the easily
# missed libunwind-dev / cppzmq-dev that the Gazebo bridge needs on 24.04).
#
# --no-nuttx skips the ARM cross-compiler; we only ever build SITL here.
# Simulation tools are intentionally left ON: that's the step that adds the
# packages.osrfoundation.org repo and installs gz-harmonic.
# ---------------------------------------------------------------------------
ENV RUNS_IN_DOCKER=true
COPY docker/px4-constraints.txt /tmp/px4-setup/px4-constraints.txt
RUN cd /tmp/px4-setup \
    && curl -fsSL -O https://raw.githubusercontent.com/PX4/PX4-Autopilot/${PX4_VERSION}/Tools/setup/ubuntu.sh \
    && curl -fsSL -O https://raw.githubusercontent.com/PX4/PX4-Autopilot/${PX4_VERSION}/Tools/setup/requirements.txt \
    && PIP_CONSTRAINT=/tmp/px4-setup/px4-constraints.txt bash ubuntu.sh --no-nuttx \
    && rm -rf /tmp/px4-setup /var/lib/apt/lists/*

# Fail the build loudly here rather than mysteriously at runtime if the pip
# step above ever manages to shadow ROS's numpy.
RUN python3 -c "import numpy; assert numpy.__version__.startswith('1.'), \
    'numpy %s will break ROS 2 Jazzy' % numpy.__version__; print('numpy', numpy.__version__, 'ok')"

# ---------------------------------------------------------------------------
# Micro XRCE-DDS Agent. This is the process that turns PX4's uORB topics into
# real DDS traffic ROS 2 can subscribe to. PX4 SITL starts the *client* side
# automatically; this is the other half of that link.
# ---------------------------------------------------------------------------
RUN git clone -b ${XRCE_AGENT_VERSION} --depth 1 \
      https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/xrce \
    && cmake -S /tmp/xrce -B /tmp/xrce/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/xrce/build -j"$(nproc)" \
    && cmake --install /tmp/xrce/build \
    && ldconfig /usr/local/lib \
    && rm -rf /tmp/xrce

# ---------------------------------------------------------------------------
# Run as uid 1000 (the 'ubuntu' user Ubuntu 24.04 images ship with), which
# matches the host user. Anything written into the bind-mounted ws/ and px4/
# directories is then owned by you on the host and editable without sudo.
# ---------------------------------------------------------------------------
RUN echo 'ubuntu ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/ubuntu \
    && chmod 0440 /etc/sudoers.d/ubuntu

USER ubuntu
WORKDIR /home/ubuntu

# ccache lives here and is mounted as a named volume so PX4 rebuilds stay fast
# across container recreations. Creating it as `ubuntu` matters: Docker seeds a
# new named volume from the image directory, ownership included.
RUN mkdir -p /home/ubuntu/.ccache

# Interactive shells get ROS + the workspace overlay. Note that `docker exec
# ... bash -c` is NON-interactive and never reads this file — the justfile
# sources setup.bash explicitly for that reason.
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /home/ubuntu/.bashrc \
    && echo 'if [ -f /home/ubuntu/ros2_ws/install/setup.bash ]; then source /home/ubuntu/ros2_ws/install/setup.bash; fi' >> /home/ubuntu/.bashrc

CMD ["bash"]
