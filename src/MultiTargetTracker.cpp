#include "radar/MultiTargetTracker.hpp"
#include <cmath>
#include <algorithm>

namespace radar::tracker {

MultiTargetTracker::MultiTargetTracker() {
    init(TrackerConfig{});
}

void MultiTargetTracker::init(const TrackerConfig& config) {
    config_ = config;

    R_ = math::Matrix<3, 3>::diagonal({
        config_.sigma_r * config_.sigma_r,
        config_.sigma_theta * config_.sigma_theta,
        config_.sigma_vr * config_.sigma_vr
    });

    for (auto& track : tracks_) {
        track.reset();
    }
    next_track_id_ = 1;
}

std::size_t MultiTargetTracker::getActiveTrackCount() const {
    std::size_t count = 0;
    for (const auto& track : tracks_) {
        if (track.status != TrackStatus::FREE && track.status != TrackStatus::DELETED) {
            count++;
        }
    }
    return count;
}

bool MultiTargetTracker::isDetectionSanityValid(const Detection& det) const {
    if (!std::isfinite(det.range_m) || !std::isfinite(det.azimuth_rad) || !std::isfinite(det.doppler_mps)) {
        return false;
    }

    // Input sanity limits per Section 6 (ISO 26262 ASIL B)
    constexpr float MIN_RANGE = 0.2f;
    constexpr float MAX_RANGE = 200.0f;
    constexpr float MAX_AZIMUTH_RAD = 1.04719755f; // ~60 degrees
    constexpr float MAX_DOPPLER = 70.0f;

    if (det.range_m < MIN_RANGE || det.range_m > MAX_RANGE) {
        return false;
    }
    if (std::abs(det.azimuth_rad) > MAX_AZIMUTH_RAD) {
        return false;
    }
    if (std::abs(det.doppler_mps) > MAX_DOPPLER) {
        return false;
    }

    return true;
}

void MultiTargetTracker::processCycle(
    const Detection* detections,
    std::size_t num_detections,
    TrackOutput* out_tracks,
    std::size_t& out_num_tracks
) {
    // 1. Gather valid detections
    std::size_t num_valid_det = 0;
    const std::size_t max_input_det = std::min(num_detections, MAX_DETECTIONS);
    for (std::size_t j = 0; j < max_input_det; ++j) {
        if (isDetectionSanityValid(detections[j])) {
            valid_det_indices_[num_valid_det++] = j;
        }
    }

    // 2. Gather active tracks & perform Step 1: EKF Time Prediction
    std::size_t num_active_tracks = 0;
    for (std::size_t i = 0; i < MAX_TRACKS; ++i) {
        auto& track = tracks_[i];
        if (track.status == TrackStatus::TENTATIVE ||
            track.status == TrackStatus::CONFIRMED ||
            track.status == TrackStatus::COASTED) {

            active_track_indices_[num_active_tracks++] = i;

            // Preserve prior velocity for acceleration monitoring
            track.prev_vx = track.x(2, 0);
            track.prev_vy = track.x(3, 0);

            // Prediction
            ExtendedKalmanFilter::predict(
                track.x,
                track.P,
                CYCLE_TIME_SEC,
                config_.sigma_a,
                track.x_pred,
                track.P_pred
            );

            // Compute measurement Jacobian and Innovation Covariance S
            math::Matrix<3, 4> H;
            ExtendedKalmanFilter::computeJacobian(track.x_pred, H);

            math::Matrix<3, 3> S;
            float det_S = 0.0f;
            track.S_valid = ExtendedKalmanFilter::computeInnovationCovariance(
                track.P_pred,
                H,
                R_,
                S,
                track.S_inv,
                det_S
            );
            if (det_S <= 0.0f) {
                track.S_valid = false;
            }
        }
    }

    // 3. Step 2: Compute Cost Matrix & Spatial Gating (Mahalanobis Distance)
    for (std::size_t i = 0; i < num_active_tracks; ++i) {
        const std::size_t t_idx = active_track_indices_[i];
        const auto& track = tracks_[t_idx];

        math::Vector<3> z_pred;
        ExtendedKalmanFilter::computeObservation(track.x_pred, z_pred);

        for (std::size_t j = 0; j < num_valid_det; ++j) {
            const std::size_t d_idx = valid_det_indices_[j];
            const auto& det = detections[d_idx];

            math::Vector<3> z_meas;
            z_meas(0, 0) = det.range_m;
            z_meas(1, 0) = det.azimuth_rad;
            z_meas(2, 0) = det.doppler_mps;

            math::Vector<3> y;
            ExtendedKalmanFilter::computeInnovation(z_meas, z_pred, y);

            if (track.S_valid) {
                const float d2 = ExtendedKalmanFilter::mahalanobisDistanceSq(y, track.S_inv);
                if (d2 <= config_.gate_threshold && d2 >= 0.0f) {
                    cost_matrix_[i][j] = d2;
                } else {
                    cost_matrix_[i][j] = COST_INF;
                }
            } else {
                cost_matrix_[i][j] = COST_INF;
            }
        }
    }

    // 4. Step 3: Data Association via Hungarian Matching
    const AssociationResult assoc = matcher_.match(
        cost_matrix_,
        num_active_tracks,
        num_valid_det,
        config_.gate_threshold
    );

    // 5. Step 4: Measurement Update and Lifecycle FSM for Active Tracks
    for (std::size_t i = 0; i < num_active_tracks; ++i) {
        const std::size_t t_idx = active_track_indices_[i];
        auto& track = tracks_[t_idx];

        const int assigned_det_idx = assoc.track_to_det[i];
        if (assigned_det_idx >= 0) {
            // Associated detection found: execute EKF measurement update
            const std::size_t d_idx = valid_det_indices_[static_cast<std::size_t>(assigned_det_idx)];
            const auto& det = detections[d_idx];

            math::Vector<3> z_meas;
            z_meas(0, 0) = det.range_m;
            z_meas(1, 0) = det.azimuth_rad;
            z_meas(2, 0) = det.doppler_mps;

            math::Vector<3> z_pred;
            ExtendedKalmanFilter::computeObservation(track.x_pred, z_pred);

            math::Vector<3> y;
            ExtendedKalmanFilter::computeInnovation(z_meas, z_pred, y);

            math::Matrix<3, 4> H;
            ExtendedKalmanFilter::computeJacobian(track.x_pred, H);

            math::Matrix<4, 3> K;
            ExtendedKalmanFilter::computeKalmanGain(track.P_pred, H, track.S_inv, K);

            ExtendedKalmanFilter::updateJoseph(
                track.x_pred,
                track.P_pred,
                y,
                H,
                R_,
                K,
                track.x,
                track.P
            );

            track.hit_count++;
            track.age++;
            track.total_active_cycles++;

            if (track.status == TrackStatus::TENTATIVE) {
                // Promotion check (M-of-N: hits >= confirm_hits within age <= confirm_window)
                if (track.hit_count >= config_.confirm_hits && track.age <= config_.confirm_window) {
                    track.status = TrackStatus::CONFIRMED;
                }
            } else {
                // CONFIRMED or COASTED -> re-acquire as CONFIRMED
                track.status = TrackStatus::CONFIRMED;
                track.coast_count = 0;
            }
        } else {
            // Unassociated track: propagate kinematic prediction only
            track.x = track.x_pred;
            track.P = track.P_pred;
            track.age++;
            track.total_active_cycles++;

            if (track.status == TrackStatus::TENTATIVE) {
                const uint16_t misses = track.age - track.hit_count;
                if (track.age >= config_.confirm_window && track.hit_count < config_.confirm_hits) {
                    track.status = TrackStatus::DELETED;
                } else if (misses > (config_.confirm_window - config_.confirm_hits)) {
                    track.status = TrackStatus::DELETED;
                }
            } else if (track.status == TrackStatus::CONFIRMED) {
                track.status = TrackStatus::COASTED;
                track.coast_count = 1;
            } else if (track.status == TrackStatus::COASTED) {
                track.coast_count++;
                if (track.coast_count > config_.max_coast_frames) {
                    track.status = TrackStatus::DELETED;
                }
            }
        }
    }

    // 6. Initialize Unassigned Detections into Candidate Tracks
    for (std::size_t j = 0; j < num_valid_det; ++j) {
        if (assoc.det_to_track[j] == -1) {
            // Find first FREE slot
            for (auto& slot : tracks_) {
                if (slot.status == TrackStatus::FREE) {
                    const std::size_t d_idx = valid_det_indices_[j];
                    const auto& det = detections[d_idx];

                    slot.track_id = next_track_id_++;
                    slot.status = TrackStatus::TENTATIVE;

                    const float r = det.range_m;
                    const float th = det.azimuth_rad;
                    const float vr = det.doppler_mps;

                    // Polar to Cartesian initialization
                    slot.x(0, 0) = r * std::cos(th);
                    slot.x(1, 0) = r * std::sin(th);
                    slot.x(2, 0) = vr * std::cos(th);
                    slot.x(3, 0) = vr * std::sin(th);

                    slot.prev_vx = slot.x(2, 0);
                    slot.prev_vy = slot.x(3, 0);

                    slot.hit_count = 1;
                    slot.age = 1;
                    slot.coast_count = 0;
                    slot.total_active_cycles = 1;
                    slot.is_valid_safety = true;

                    // Initial covariance matrix
                    const float var_r = config_.sigma_r * config_.sigma_r;
                    const float lateral_var = (r * config_.sigma_theta) * (r * config_.sigma_theta);
                    const float var_x = std::max(var_r, 1.0f);
                    const float var_y = std::max(lateral_var, 1.0f);
                    constexpr float INIT_VEL_VAR = 25.0f; // (m/s)^2

                    slot.P = math::Matrix<4, 4>::diagonal({var_x, var_y, INIT_VEL_VAR, INIT_VEL_VAR});
                    break;
                }
            }
        }
    }

    // 7. Step 5 & 6: ISO 26262 Plausibility, Covariance Guards & Lifecycle Cleanup
    for (auto& track : tracks_) {
        if (track.status == TrackStatus::TENTATIVE ||
            track.status == TrackStatus::CONFIRMED ||
            track.status == TrackStatus::COASTED) {

            const float vx = track.x(2, 0);
            const float vy = track.x(3, 0);
            const float speed = std::sqrt(vx * vx + vy * vy);

            // Kinematic Plausibility Guard: Speed <= 85 m/s (~306 km/h)
            if (speed > 85.0f) {
                track.is_valid_safety = false;
            }

            // Dynamic Kinematic Guard: Acceleration <= 15 m/s^2
            if (track.age > 1) {
                const float ax = (vx - track.prev_vx) / CYCLE_TIME_SEC;
                const float ay = (vy - track.prev_vy) / CYCLE_TIME_SEC;
                const float accel = std::sqrt(ax * ax + ay * ay);
                if (accel > 15.0f) {
                    track.is_valid_safety = false;
                }
            }

            // Covariance Divergence Health Monitor
            const float tr = track.P.trace();
            constexpr float MAX_COVARIANCE_TRACE = 1000.0f;
            if (tr > MAX_COVARIANCE_TRACE || !std::isfinite(tr)) {
                track.status = TrackStatus::DELETED;
            }

            // Sensor Field of View Plausibility Boundary
            const float px = track.x(0, 0);
            const float py = track.x(1, 0);
            if (px < 0.2f || px > 250.0f || std::abs(py) > (px * 1.8f + 15.0f)) {
                track.status = TrackStatus::DELETED;
            }
        }

        // Slot recycling
        if (track.status == TrackStatus::DELETED) {
            track.reset();
        }
    }

    // 8. Publish Confirmed & Active Tracks to downstream output list
    std::size_t out_count = 0;
    for (const auto& track : tracks_) {
        if (track.status == TrackStatus::CONFIRMED || track.status == TrackStatus::COASTED) {
            // Ghost target suppression: stationary targets (< 0.5 m/s) active for < 150 ms are suppressed
            const float speed = std::sqrt(track.x(2, 0) * track.x(2, 0) + track.x(3, 0) * track.x(3, 0));
            if (speed < 0.5f && (static_cast<float>(track.total_active_cycles) * CYCLE_TIME_SEC) < 0.150f) {
                continue;
            }

            if (out_count < MAX_TRACKS && out_tracks != nullptr) {
                auto& out = out_tracks[out_count++];
                out.track_id = track.track_id;
                out.status = track.status;
                out.x_m = track.x(0, 0);
                out.y_m = track.x(1, 0);
                out.vx_mps = track.x(2, 0);
                out.vy_mps = track.x(3, 0);
                out.var_x = track.P(0, 0);
                out.var_y = track.P(1, 1);
                out.var_vx = track.P(2, 2);
                out.var_vy = track.P(3, 3);
                out.age_frames = track.age;
                out.coast_count = track.coast_count;
                out.is_valid_safety = track.is_valid_safety;
            }
        }
    }
    out_num_tracks = out_count;
}

} // namespace radar::tracker
