#pragma once
#include <cstdint>
#include <cstddef>
#include <array>

namespace radar::tracker {

constexpr std::size_t MAX_TRACKS = 64;
constexpr std::size_t MAX_DETECTIONS = 128;
constexpr float CYCLE_TIME_SEC = 0.040f; // 40 ms (25 Hz)

enum class TrackStatus : uint8_t {
    FREE       = 0,
    TENTATIVE  = 1,
    CONFIRMED  = 2,
    COASTED    = 3,
    DELETED    = 4
};

struct Detection {
    float range_m;       // Slant range [m]
    float azimuth_rad;   // Azimuth angle [rad]
    float doppler_mps;   // Radial velocity [m/s]
    float snr_db;        // Signal-to-noise ratio [dB]
    uint32_t timestamp_ms;
};

struct TrackOutput {
    uint32_t track_id;
    TrackStatus status;
    float x_m;           // Longitudinal position [m]
    float y_m;           // Lateral position [m]
    float vx_mps;        // Longitudinal velocity [m/s]
    float vy_mps;        // Lateral velocity [m/s]
    float var_x;         // Position variance x [m^2]
    float var_y;         // Position variance y [m^2]
    float var_vx;        // Velocity variance vx [(m/s)^2]
    float var_vy;        // Velocity variance vy [(m/s)^2]
    uint16_t age_frames;
    uint8_t coast_count;
    bool is_valid_safety; // ISO 26262 ASIL health flag
};

struct TrackerConfig {
    float sigma_a = 2.5f;          // Process noise acceleration std [m/s^2]
    float sigma_r = 0.25f;         // Range measurement noise std [m]
    float sigma_theta = 0.0087f;   // Angle measurement noise std [rad] (~0.5 deg)
    float sigma_vr = 0.15f;        // Radial velocity noise std [m/s]
    float gate_threshold = 12.84f; // Chi-Square 3-DOF 99.5% threshold
    uint8_t confirm_hits = 3;      // M-of-N confirmation hits
    uint8_t confirm_window = 5;    // M-of-N confirmation window
    uint8_t max_coast_frames = 5;  // Maximum coasting duration
};

class IMultiTargetTracker {
public:
    virtual ~IMultiTargetTracker() = default;
    virtual void init(const TrackerConfig& config) = 0;
    virtual void processCycle(
        const Detection* detections,
        std::size_t num_detections,
        TrackOutput* out_tracks,
        std::size_t& out_num_tracks
    ) = 0;
};

} // namespace radar::tracker

