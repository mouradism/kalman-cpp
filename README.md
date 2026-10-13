# Automotive Radar Multi-Target Tracker

A C++17 radar tracking project that estimates target position and velocity from
range, azimuth, and radial-velocity detections. The C++ core uses an Extended
Kalman Filter (EKF), gated Hungarian data association, and a track lifecycle
that handles tentative tracks, confirmation, coasting, and deletion.

The tracker is designed around fixed-size buffers for up to 64 tracks and 128
detections, with a nominal 40 ms (25 Hz) update cycle. Safety-validity flags
and input sanity checks are included as software features; this project is not
a claim of functional-safety certification or production readiness.

## Features

- Constant-velocity EKF with radar-coordinate measurements.
- Gated global-nearest-neighbor association using the Hungarian algorithm.
- Track confirmation, missed-detection coasting, and track cleanup.
- Deterministic C++ demo and acceptance tests covering tracking, occlusion,
  clutter, crossing targets, braking, and execution time.
- Optional Dear ImGui visualization for Windows with a DirectX 11 backend.
- Python reference model for simulation and test-vector generation.

## Requirements

- CMake 3.20 or newer.
- A C++17-capable compiler.
- Python 3 and NumPy to run the optional Python reference model. SciPy is
  needed when that model performs data association.
- The GUI target is intended for Windows with DirectX 11.

## Build

From the repository root:

```sh
cmake -S . -B build
cmake --build build --config Release
```

With a single-configuration generator, select Release at configure time if
desired:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

CMake places executables under `build/bin` (or the configuration-specific
subdirectory for multi-configuration generators).

## Run the demo

Run the `radar_tracker` executable produced by the build. It simulates an
approaching vehicle and prints the estimated track state and status for each
frame.

On Windows, the optional visualizer target is named `radar_tracker_gui`.

## Run tests

```sh
ctest --test-dir build -C Release --output-on-failure
```

The acceptance-test executable is also available as `test_tracker`. Its cases
check constant-velocity accuracy, coasting and reacquisition, clutter
rejection, crossing-target identity, braking response, and a 64-track /
128-detection timing benchmark.

## Python reference model

Install NumPy and SciPy in your Python environment, then run:

```sh
python3 python/radar_tracker.py
```

The Python implementation is a reference prototype; the C++ tracker is the
primary implementation.

## Repository layout

```text
include/radar/   Tracker types, EKF, association, and tracker interfaces
src/             C++ tracker implementation and command-line demo
src/gui/         Windows Dear ImGui / DirectX 11 visualizer
tests/           C++ acceptance tests
python/          Python reference model
third_party/     Dear ImGui dependency
```
