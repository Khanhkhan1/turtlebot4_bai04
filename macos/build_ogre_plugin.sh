#!/usr/bin/env bash
# Builds a patched Ignition Rendering 6 "ogre" engine plugin for macOS (Apple Silicon,
# RoboStack ros_env) and installs it into <workspace>/ext/render_plugins.
#
# Why: with Apple's OpenGL (on Metal) the stock ogre plugin returns every gpu_lidar ray
# at range_min (its stitching pass draws GL points, which rasterize nothing) and crashes
# when a world has two depth cameras (duplicate shader resource). The patch:
#   * OgreGpuRays: render targets RGBA32F, and on __APPLE__ stitch the first-pass
#     textures into the scan on the CPU (same texel mapping as the original mesh);
#   * OgreMaterial: reuse an existing GLSL program instead of creating a duplicate.
# sim.launch.py points the robot's Sensors system at this plugin when it exists.
#
#   source /path/to/conda.sh && conda activate ros_env
#   ./build_ogre_plugin.sh [workspace_dir]        (default: ~/tb4_ws)
set -euo pipefail
WS="${1:-$HOME/tb4_ws}"
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$WS/ext/gz-rendering"
OUT="$WS/ext/render_plugins"

if [ ! -d "$SRC" ]; then
  git clone --depth 1 -b ignition-rendering6_6.6.3 https://github.com/gazebosim/gz-rendering.git "$SRC"
  git -C "$SRC" apply "$HERE/gz-rendering6-ogre-macos.patch"
fi
cmake -S "$SRC" -B "$SRC/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" \
  -DSKIP_ogre2=true -DSKIP_optix=true -DBUILD_TESTING=OFF -DBUILD_DOCS=OFF
make -C "$SRC/build" -j8 ignition-rendering6-ogre

mkdir -p "$OUT"
cp "$SRC/build/lib/libignition-rendering6-ogre.6.6.3.dylib" "$OUT/"
ln -sf libignition-rendering6-ogre.6.6.3.dylib "$OUT/libignition-rendering6-ogre.dylib"
echo "Installed $OUT/libignition-rendering6-ogre.dylib"
