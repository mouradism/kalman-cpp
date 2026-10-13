#pragma once

#include "radar/TrackerTypes.hpp"
#include <array>
#include <cstddef>
#include <algorithm>
#include <cmath>

namespace radar::tracker {

constexpr float COST_INF = 1e8f;

struct AssociationResult {
    // track_to_det[i] holds detection index associated with active track i, or -1 if unassigned
    std::array<int, MAX_TRACKS> track_to_det{};
    // det_to_track[j] holds track index associated with detection j, or -1 if unassigned
    std::array<int, MAX_DETECTIONS> det_to_track{};

    std::size_t num_matches{0};
};

class HungarianMatcher {
public:
    static constexpr std::size_t MAX_DIM = MAX_DETECTIONS > MAX_TRACKS ? MAX_DETECTIONS : MAX_TRACKS;

    HungarianMatcher() = default;

    // Zero-heap rectangular bipartite matching (O(n * m^2))
    AssociationResult match(
        const float cost_matrix[MAX_TRACKS][MAX_DETECTIONS],
        std::size_t num_tracks,
        std::size_t num_detections,
        float gate_threshold
    ) {
        AssociationResult result;
        result.track_to_det.fill(-1);
        result.det_to_track.fill(-1);
        result.num_matches = 0;

        if (num_tracks == 0 || num_detections == 0) {
            return result;
        }

        const bool transposed = (num_tracks > num_detections);
        const std::size_t n = transposed ? num_detections : num_tracks;
        const std::size_t m = transposed ? num_tracks : num_detections;

        // Fill rectangular cost table (1-indexed for Hungarian: n rows, m cols, n <= m)
        for (std::size_t i = 1; i <= n; ++i) {
            for (std::size_t j = 1; j <= m; ++j) {
                const float c = transposed ? cost_matrix[j - 1][i - 1] : cost_matrix[i - 1][j - 1];
                cost_table_[i][j] = (c <= gate_threshold) ? c : COST_INF;
            }
        }

        u_.fill(0.0f);
        v_.fill(0.0f);
        p_.fill(0);
        way_.fill(0);

        for (std::size_t i = 1; i <= n; ++i) {
            p_[0] = static_cast<int>(i);
            std::size_t j0 = 0;
            minv_.fill(COST_INF);
            used_.fill(false);

            do {
                used_[j0] = true;
                const std::size_t i0 = static_cast<std::size_t>(p_[j0]);
                float delta = COST_INF;
                std::size_t j1 = 0;

                for (std::size_t j = 1; j <= m; ++j) {
                    if (!used_[j]) {
                        const float cur = cost_table_[i0][j] - u_[i0] - v_[j];
                        if (cur < minv_[j]) {
                            minv_[j] = cur;
                            way_[j] = static_cast<int>(j0);
                        }
                        if (minv_[j] < delta) {
                            delta = minv_[j];
                            j1 = j;
                        }
                    }
                }

                for (std::size_t j = 0; j <= m; ++j) {
                    if (used_[j]) {
                        u_[static_cast<std::size_t>(p_[j])] += delta;
                        v_[j] -= delta;
                    } else {
                        minv_[j] -= delta;
                    }
                }
                j0 = j1;
            } while (p_[j0] != 0);

            do {
                const std::size_t j1 = static_cast<std::size_t>(way_[j0]);
                p_[j0] = p_[j1];
                j0 = j1;
            } while (j0 != 0);
        }

        // Extract assignments from potential array
        for (std::size_t j = 1; j <= m; ++j) {
            const int r = p_[j];
            if (r >= 1 && r <= static_cast<int>(n)) {
                const std::size_t row_idx = static_cast<std::size_t>(r - 1);
                const std::size_t col_idx = j - 1;

                const std::size_t t_idx = transposed ? col_idx : row_idx;
                const std::size_t d_idx = transposed ? row_idx : col_idx;

                if (t_idx < num_tracks && d_idx < num_detections) {
                    const float original_cost = cost_matrix[t_idx][d_idx];
                    if (original_cost <= gate_threshold) {
                        result.track_to_det[t_idx] = static_cast<int>(d_idx);
                        result.det_to_track[d_idx] = static_cast<int>(t_idx);
                        result.num_matches++;
                    }
                }
            }
        }

        return result;
    }

private:
    float cost_table_[MAX_DIM + 1][MAX_DIM + 1]{};
    std::array<float, MAX_DIM + 1> u_{};
    std::array<float, MAX_DIM + 1> v_{};
    std::array<int, MAX_DIM + 1> p_{};
    std::array<int, MAX_DIM + 1> way_{};
    std::array<float, MAX_DIM + 1> minv_{};
    std::array<bool, MAX_DIM + 1> used_{};
};

} // namespace radar::tracker

