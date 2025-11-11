# ============================================================
# Raspberry Pi Zero 2 W Cross-Compile Toolchain (64-bit)
# Target: Cortex-A53 (ARMv8-A)
# ============================================================

# --- Target system ---
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# --- Cross-compiler paths ---
set(CMAKE_C_COMPILER   /home/mstarks/yocto_sdks/pi-zero/sysroots/x86_64-pokysdk-linux/usr/bin/aarch64-poky-linux/aarch64-poky-linux-gcc)
set(CMAKE_CXX_COMPILER /home/mstarks/yocto_sdks/pi-zero/sysroots/x86_64-pokysdk-linux/usr/bin/aarch64-poky-linux/aarch64-poky-linux-g++)
set(CMAKE_ASM_COMPILER /home/mstarks/yocto_sdks/pi-zero/sysroots/x86_64-pokysdk-linux/usr/bin/aarch64-poky-linux/aarch64-poky-linux-gcc)

# --- Target sysroot ---
set(CMAKE_SYSROOT /home/mstarks/yocto_sdks/pi-zero/sysroots/cortexa53-poky-linux)
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})

# --- Compiler flags for Cortex-A53 ---
set(COMMON_FLAGS "-march=armv8-a -mtune=cortex-a53")
set(CMAKE_C_FLAGS_INIT   "${COMMON_FLAGS}")
set(CMAKE_CXX_FLAGS_INIT "${COMMON_FLAGS}")
set(CMAKE_ASM_FLAGS_INIT "${COMMON_FLAGS}")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS_INIT}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_INIT}" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS_INIT}" CACHE STRING "" FORCE)

# --- pkg-config for target sysroot ---
set(ENV{PKG_CONFIG_SYSROOT_DIR} ${CMAKE_SYSROOT})
set(ENV{PKG_CONFIG_PATH} ${CMAKE_SYSROOT}/usr/lib/pkgconfig)

# --- Search path modes ---
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)  # Programs from host
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)   # Libraries from sysroot only
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)   # Headers from sysroot only
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)   # Packages from sysroot only

# --- NEON optimizations ---
set(EIGEN_ENABLE_NEON ON CACHE BOOL "" FORCE)
set(WITH_NEON ON CACHE BOOL "" FORCE)

# --- ROS prefix paths ---
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_SYSROOT}/opt/ros/jazzy)
if(DEFINED ENV{ROS_LOCAL_INSTALL})
    list(APPEND CMAKE_PREFIX_PATH $ENV{ROS_LOCAL_INSTALL})
endif()

# --- Explicit Python target settings ---
set(Python3_EXECUTABLE ${CMAKE_SYSROOT}/usr/bin/python3 CACHE STRING "" FORCE)
set(Python3_INCLUDE_DIR ${CMAKE_SYSROOT}/usr/include/python3.12 CACHE STRING "" FORCE)
set(Python3_LIBRARY ${CMAKE_SYSROOT}/usr/lib/libpython3.12.so CACHE STRING "" FORCE)

# --- Notes ---
# Usage:
# colcon build \
#   --merge-install \
#   --cmake-args \
#     -DCMAKE_TOOLCHAIN_FILE=/path/to/pi_zero2w_64_toolchain.cmake \
#     -DCMAKE_BUILD_TYPE=Release
