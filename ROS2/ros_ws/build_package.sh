#!/bin/bash

# === CONFIGURATION ===
ROS_WS="/home/mstarks/Documents/Hero/heroswarmv2/ROS2/ros_ws"
BUILD_DIR="$ROS_WS/build"
INSTALL_DIR="$ROS_WS/pi-zero/install"
SYSROOT="/home/mstarks/yocto_sdks/pi-zero/sysroots/cortexa53-poky-linux"
TOOLCHAIN_FILE="$ROS_WS/toolchains/raspi-yocoto-toolchain.cmake"
LAUNCH_FILE="/home/mstarks/Documents/Hero/heroswarmv2/ROS2/ros_ws/src/roboto_controller/launch/robot_controller.launch.py"

# Host Python/NumPy
HOST_PY=$(command -v python3)
HOST_NUMPY_INCLUDE="$SYSROOT/usr/lib/python3.12/site-packages/numpy/core/include"

# Target Python
PY_INC="$SYSROOT/usr/include/python3.12"
PY_LIB="$(ls -1 $SYSROOT/usr/lib/libpython3.12*.so | head -n1)"

# === SANITY CHECKS ===
echo "Using host python: $HOST_PY"
echo "Using host NumPy include: $HOST_NUMPY_INCLUDE"
echo "Using target Python include: $PY_INC"
echo "Using target Python lib: $PY_LIB"

[[ -f "$PY_INC/Python.h" ]] || { echo "ERROR: $PY_INC/Python.h not found (install python3-dev in SDK)"; exit 1; }
[[ -f "$PY_LIB" ]] || { echo "ERROR: libpython3.12.so not found in $SYSROOT/usr/lib (install python3-dev in SDK)"; exit 1; }

# === ENVIRONMENT FIXES ===
# Remove any conflicting gmake symlink in sysroot
if [[ -L "$SYSROOT/usr/bin/gmake" ]]; then
    rm "$SYSROOT/usr/bin/gmake"
fi

# Force host make for CMake and scripts
export MAKE_PROGRAM=/usr/bin/make
export MAKE=/usr/bin/make
export GNUMAKE=/usr/bin/make
alias gmake=/usr/bin/make
export MAKEFLAGS="-j$(nproc)"

# Ensure host binaries take precedence
export PATH=/usr/bin:/usr/local/bin:$PATH

# Source Yocto SDK
source ~/yocto_sdks/pi-zero/environment-setup-cortexa53-poky-linux
source /opt/ros/jazzy/setup.sh

# === COPY HARD-CODED LIBS (if needed) ===
# Hack for foonathan-memory
cp /home/mstarks/Documents/Hero/poky/build/tmp/work/cortexa53-poky-linux/foonathan-memory/0.6.2+git/build/src/libfoonathan_memory-0.6.2.a \
   "$SYSROOT/usr/lib"

# === BUILD ===
mkdir -p "$BUILD_DIR"
mkdir -p "$INSTALL_DIR"

echo "Starting build..."
export ROS_LOCAL_INSTALL=$PWD/install
colcon build \
  --merge-install \
  --build-base "$BUILD_DIR" \
  --install-base "$INSTALL_DIR" \
  --cmake-args \
    -G "Unix Makefiles" \
    -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN_FILE" \
    -DCMAKE_MAKE_PROGRAM="$MAKE_PROGRAM" \
    -DPython3_FIND_DEBUG=ON \
    -DPython3_NumPy_INCLUDE_DIRS="$HOST_NUMPY_INCLUDE" \
    -DPython3_NumPy_INCLUDE_DIR="$HOST_NUMPY_INCLUDE" \
    -DPython3_FIND_STRATEGY=LOCATION

# === POST INSTALL ===
bash post_install_head_cmd_replacement.sh "$INSTALL_DIR"

# === DEPLOY ===
scp -r "$INSTALL_DIR" root@192.168.1.132:/home/root
scp "$LAUNCH_FILE" root@192.168.1.132:/home/root

echo "Build and deployment complete!"
