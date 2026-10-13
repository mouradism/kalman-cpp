#pragma once

#include "radar/TrackerTypes.hpp"
#include "radar/Matrix.hpp"
#include "radar/EKF.hpp"
#include "radar/DataAssociation.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace radar::tracker {

struct InternalTrack {
    uint32_t track_id{0};
    TrackStatus status{TrackStatus::FREE};

    math::Vector<4> x{};       // [x, y, vx, vy]^T
    math::Matrix<4, 4> P{};    // Covariance matrix

    math::Vector<4> x_pred{};  // Predicted state in current cycle
    math::Matrix<4, 4> P_pred{};
    math::Matrix<3, 3> S_inv{};
    bool S_valid{false};

    float prev_vx{0.0f};
    float prev_vy{0.0f};

    uint8_t hit_count{0};
    uint16_t age{0};
    uint8_t coast_count{0};

    uint32_t total_active_cycles{0};
    bool is_valid_safety{true};

    void reset() {
        track_id = 0;
        status = TrackStatus::FREE;
        x = math::Vector<4>{};
        P = math::Matrix<4, 4>{};
        x_pred = math::Vector<4>{};
        P_pred = math::Matrix<4, 4>{};
        S_inv = math::Matrix<3, 3>{};
        S_valid = false;
        prev_vx = 0.0f;
        prev_vy = 0.0f;
        hit_count = 0;
        age = 0;
        coast_count = 0;
        total_active_cycles = 0;
        is_valid_safety = true;
    }
};

class MultiTargetTracker final : public IMultiTargetTracker {
public:
    MultiTargetTracker();
    ~MultiTargetTracker() override = default;

    void init(const TrackerConfig& config) override;

    void processCycle(
        const Detection* detections,
        std::size_t num_detections,
        TrackOutput* out_tracks,
        std::size_t& out_num_tracks
    ) override;

    // Direct inspection methods for testing & diagnostics
    const std::array<InternalTrack, MAX_TRACKS>& getInternalTracks() const {
        return tracks_;
    }

    std::size_t getActiveTrackCount() const;

private:
    bool isDetectionSanityValid(const Detection& det) const;

    TrackerConfig config_{};
    math::Matrix<3, 3> R_{};

    std::array<InternalTrack, MAX_TRACKS> tracks_{};
    uint32_t next_track_id_{1};

    HungarianMatcher matcher_{};
    float cost_matrix_[MAX_TRACKS][MAX_DETECTIONS]{};

    // Static buffer for active track indices
    std::array<std::size_t, MAX_TRACKS> active_track_indices_{};
    // Static buffer for valid detection indices
    std::array<std::size_t, MAX_DETECTIONS> valid_det_indices_{};
};

} // namespace radar::tracker

