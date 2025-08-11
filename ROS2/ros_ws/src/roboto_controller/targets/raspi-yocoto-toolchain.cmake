# ============================================================
# Raspberry Pi Zero 2 W Cross-Compile Toolchain (64-bit)
# Target: Cortex-A53 (ARMv8-A)
# ============================================================

# Target system
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# --- [ Adjust these paths to your setup ] ---
# Path to cross-compilers
# TOOD: cross-compilers 
set(CMAKE_C_COMPILER   /path/to/toolchain/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /path/to/toolchain/bin/aarch64-linux-gnu-g++)
set(CMAKE_ASM_COMPILER /path/to/toolchain/bin/aarch64-linux-gnu-gcc)

# Path to target sysroot
#TODO: sysroot path
set(CMAKE_SYSROOT /path/to/sysroot)
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})

# --- [ Compiler Flags for Pi Zero 2 W 64-bit ] ---
# NEON is part of the baseline in ARMv8-A, just tune for Cortex-A53
set(COMMON_FLAGS "-march=armv8-a -mtune=cortex-a53")

set(CMAKE_C_FLAGS_INIT   "${COMMON_FLAGS}")
set(CMAKE_CXX_FLAGS_INIT "${COMMON_FLAGS}")
set(CMAKE_ASM_FLAGS_INIT "${COMMON_FLAGS}")

# Ensure CMake doesn’t override our flags later
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS_INIT}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_INIT}" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS_INIT}" CACHE STRING "" FORCE)

# --- [ pkg-config setup for sysroot ] ---
set(ENV{PKG_CONFIG_SYSROOT_DIR} ${CMAKE_SYSROOT})
set(ENV{PKG_CONFIG_PATH} ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/pkgconfig)

# --- [ Search path modes ] ---
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)  # Programs from host
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)   # Libraries from sysroot
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)   # Headers from sysroot
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)   # Packages from sysroot

# --- [ Extra: hint to Eigen & OpenCV to use NEON ] ---
set(EIGEN_ENABLE_NEON ON CACHE BOOL "" FORCE)
set(WITH_NEON ON CACHE BOOL "" FORCE)

# ============================================================
# Usage Example:
# colcon build \
#   --merge-install \
#   --cmake-args \
#     -DCMAKE_TOOLCHAIN_FILE=/path/to/pi_zero2w_64_toolchain.cmake \
#     -DCMAKE_BUILD_TYPE=Release
# ============================================================