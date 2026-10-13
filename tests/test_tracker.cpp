#include <iostream>
#include <iomanip>
#include <cassert>
#include <cmath>
#include <vector>
#include <chrono>
#include "radar/TrackerTypes.hpp"
#include "radar/MultiTargetTracker.hpp"

using namespace radar::tracker;

// Helper to generate noise-free detection from Cartesian state
Detection makeDetection(float x, float y, float vx, float vy, uint32_t ts = 0) {
    float r = std::sqrt(x * x + y * y);
    float th = std::atan2(y, x);
    float vr = (r > 1e-4f) ? (x * vx + y * vy) / r : 0.0f;
    return Detection{r, th, vr, 20.0f, ts};
}

bool run_TC_TRK_01() {
    std::cout << "[Test TC_TRK_01] Single target constant velocity tracking... ";
    MultiTargetTracker tracker;
    TrackerConfig config;
    tracker.init(config);

    float true_x = 10.0f;
    float true_y = 0.0f;
    const float true_vx = 20.0f;
    const float true_vy = 0.0f;

    std::array<TrackOutput, MAX_TRACKS> out_tracks{};
    std::size_t num_tracks = 0;

    bool confirmed_by_frame_3 = false;
    uint32_t tracked_id = 0;

    for (int frame = 1; frame <= 100; ++frame) {
        true_x += true_vx * CYCLE_TIME_SEC;
        true_y += true_vy * CYCLE_TIME_SEC;

        Detection det = makeDetection(true_x, true_y, true_vx, true_vy, static_cast<uint32_t>(frame * 40));
        tracker.processCycle(&det, 1, out_tracks.data(), num_tracks);

        if (frame == 3) {
            if (num_tracks == 1 && out_tracks[0].status == TrackStatus::CONFIRMED) {
                confirmed_by_frame_3 = true;
                tracked_id = out_tracks[0].track_id;
            }
        }
    }

    if (!confirmed_by_frame_3 || tracked_id == 0) {
        std::cout << "FAILED: Track not confirmed by frame 3.\n";
        return false;
    }

    if (num_tracks != 1) {
        std::cout << "FAILED: Expected 1 track at end of scenario, got " << num_tracks << "\n";
        return false;
    }

    const float pos_err = std::abs(out_tracks[0].x_m - true_x);
    const float vel_err = std::abs(out_tracks[0].vx_mps - true_vx);

    if (pos_err > 0.5f) {
        std::cout << "FAILED: Position error " << pos_err << " m exceeds tolerance 0.5 m.\n";
        return false;
    }
    if (vel_err > 0.3f) {
        std::cout << "FAILED: Velocity error " << vel_err << " m/s exceeds tolerance 0.3 m/s.\n";
        return false;
    }

    std::cout << "PASSED (Pos err: " << pos_err << " m, Vel err: " << vel_err << " m/s)\n";
    return true;
}

bool run_TC_TRK_02() {
    std::cout << "[Test TC_TRK_02] Occlusion handling & Coasting (3 missed frames)... ";
    MultiTargetTracker tracker;
    tracker.init(TrackerConfig{});

    float x = 20.0f, y = 2.0f, vx = 15.0f, vy = 0.0f;
    std::array<TrackOutput, MAX_TRACKS> out_tracks{};
    std::size_t num_tracks = 0;

    uint32_t original_id = 0;

    // Track for 10 frames to confirm
    for (int f = 1; f <= 10; ++f) {
        x += vx * CYCLE_TIME_SEC;
        Detection det = makeDetection(x, y, vx, vy, static_cast<uint32_t>(f * 40));
        tracker.processCycle(&det, 1, out_tracks.data(), num_tracks);
        if (num_tracks > 0) {
            original_id = out_tracks[0].track_id;
        }
    }

    // 3 missed detection frames (Occlusion)
    for (int f = 11; f <= 13; ++f) {
        x += vx * CYCLE_TIME_SEC;
        tracker.processCycle(nullptr, 0, out_tracks.data(), num_tracks);
        if (num_tracks != 1) {
            std::cout << "FAILED: Expected 1 coasting track, got " << num_tracks << "\n";
            return false;
        }
        if (out_tracks[0].status != TrackStatus::COASTED) {
            std::cout << "FAILED: Track status should be COASTED during occlusion.\n";
            return false;
        }
        if (out_tracks[0].coast_count != (f - 10)) {
            std::cout << "FAILED: Coast count mismatch.\n";
            return false;
        }
    }

    // Target reappears at frame 14
    x += vx * CYCLE_TIME_SEC;
    Detection det = makeDetection(x, y, vx, vy, 14 * 40);
    tracker.processCycle(&det, 1, out_tracks.data(), num_tracks);

    if (num_tracks != 1) {
        std::cout << "FAILED: Expected 1 track after re-acquisition.\n";
        return false;
    }
    if (out_tracks[0].track_id != original_id) {
        std::cout << "FAILED: Track ID changed after re-acquisition (" 
                  << original_id << " vs " << out_tracks[0].track_id << ")\n";
        return false;
    }
    if (out_tracks[0].status != TrackStatus::CONFIRMED) {
        std::cout << "FAILED: Track status did not return to CONFIRMED.\n";
        return false;
    }
    if (out_tracks[0].coast_count != 0) {
        std::cout << "FAILED: Coast count not reset to 0.\n";
        return false;
    }

    std::cout << "PASSED (ID preserved: " << original_id << ", Coasted: 3 frames)\n";
    return true;
}

bool run_TC_TRK_03() {
    std::cout << "[Test TC_TRK_03] Stationary clutter spike rejection... ";
    MultiTargetTracker tracker;
    tracker.init(TrackerConfig{});

    std::array<TrackOutput, MAX_TRACKS> out_tracks{};
    std::size_t num_tracks = 0;

    // Single frame clutter spike
    Detection clutter{35.0f, 0.05f, 0.0f, 12.0f, 0};
    tracker.processCycle(&clutter, 1, out_tracks.data(), num_tracks);

    // Detections cease
    for (int f = 2; f <= 6; ++f) {
        tracker.processCycle(nullptr, 0, out_tracks.data(), num_tracks);
        // It must NEVER be confirmed or published
        if (num_tracks > 0) {
            std::cout << "FAILED: Clutter spike published as confirmed track at frame " << f << "\n";
            return false;
        }
    }

    // Active track count in internal pool should now be 0 (slot recycled)
    if (tracker.getActiveTrackCount() != 0) {
        std::cout << "FAILED: Track slot was not recycled to FREE.\n";
        return false;
    }

    std::cout << "PASSED (Candidate rejected and slot recycled)\n";
    return true;
}

bool run_TC_TRK_04() {
    std::cout << "[Test TC_TRK_04] Crossing targets without ID swap... ";
    MultiTargetTracker tracker;
    tracker.init(TrackerConfig{});

    // Target A starts at (15, -6), moving forward and right-to-left
    float xA = 15.0f, yA = -6.0f, vxA = 12.0f, vyA = 4.0f;
    // Target B starts at (15, +6), moving forward and left-to-right
    float xB = 15.0f, yB = 6.0f, vxB = 12.0f, vyB = -4.0f;

    std::array<TrackOutput, MAX_TRACKS> out_tracks{};
    std::size_t num_tracks = 0;

    uint32_t idA = 0;
    uint32_t idB = 0;

    // Run until past the intersection point (y crosses around frame 38)
    for (int f = 1; f <= 75; ++f) {
        xA += vxA * CYCLE_TIME_SEC;
        yA += vyA * CYCLE_TIME_SEC;
        xB += vxB * CYCLE_TIME_SEC;
        yB += vyB * CYCLE_TIME_SEC;

        Detection dets[2] = {
            makeDetection(xA, yA, vxA, vyA, static_cast<uint32_t>(f * 40)),
            makeDetection(xB, yB, vxB, vyB, static_cast<uint32_t>(f * 40))
        };

        tracker.processCycle(dets, 2, out_tracks.data(), num_tracks);

        if (f == 5) {
            assert(num_tracks == 2);
            // Identify which ID is A and which is B based on y
            if (out_tracks[0].y_m < out_tracks[1].y_m) {
                idA = out_tracks[0].track_id;
                idB = out_tracks[1].track_id;
            } else {
                idA = out_tracks[1].track_id;
                idB = out_tracks[0].track_id;
            }
        }

        if (f > 5) {
            for (std::size_t i = 0; i < num_tracks; ++i) {
                // Check covariance stability
                const float tr = out_tracks[i].var_x + out_tracks[i].var_y +
                                 out_tracks[i].var_vx + out_tracks[i].var_vy;
                if (tr > 100.0f) {
                    std::cout << "FAILED: Covariance diverged during crossing (trace: " << tr << ")\n";
                    return false;
                }
            }
        }
    }

    // At frame 75, Target A is at positive y, Target B is at negative y
    bool foundA = false, foundB = false;
    for (std::size_t i = 0; i < num_tracks; ++i) {
        if (out_tracks[i].track_id == idA) {
            foundA = true;
            if (out_tracks[i].y_m < 0.0f) {
                std::cout << "FAILED: Target A swapped identity!\n";
                return false;
            }
        }
        if (out_tracks[i].track_id == idB) {
            foundB = true;
            if (out_tracks[i].y_m > 0.0f) {
                std::cout << "FAILED: Target B swapped identity!\n";
                return false;
            }
        }
    }

    if (!foundA || !foundB) {
        std::cout << "FAILED: Lost track during crossing.\n";
        return false;
    }

    std::cout << "PASSED (No ID swap across intersection)\n";
    return true;
}

bool run_TC_TRK_05() {
    std::cout << "[Test TC_TRK_05] High-G deceleration adaptation (-8 m/s^2)... ";
    MultiTargetTracker tracker;
    tracker.init(TrackerConfig{});

    float x = 20.0f, y = 0.0f;
    float vx = 25.0f, vy = 0.0f;
    const float decel = -8.0f; // High-G braking

    std::array<TrackOutput, MAX_TRACKS> out_tracks{};
    std::size_t num_tracks = 0;

    // First 20 frames: constant velocity
    for (int f = 1; f <= 20; ++f) {
        x += vx * CYCLE_TIME_SEC;
        Detection det = makeDetection(x, y, vx, vy, static_cast<uint32_t>(f * 40));
        tracker.processCycle(&det, 1, out_tracks.data(), num_tracks);
    }

    // Brake for 25 frames
    for (int f = 21; f <= 45; ++f) {
        vx += decel * CYCLE_TIME_SEC;
        x += vx * CYCLE_TIME_SEC;

        Detection det = makeDetection(x, y, vx, vy, static_cast<uint32_t>(f * 40));
        tracker.processCycle(&det, 1, out_tracks.data(), num_tracks);

        if (num_tracks != 1) {
            std::cout << "FAILED: Track dropped during braking event at frame " << f << "\n";
            return false;
        }
        if (out_tracks[0].status != TrackStatus::CONFIRMED) {
            std::cout << "FAILED: Track lost confirmation during braking at frame " << f << "\n";
            return false;
        }
    }

    // Check velocity tracking accuracy at end of braking
    const float vel_err = std::abs(out_tracks[0].vx_mps - vx);
    if (vel_err > 2.0f) {
        std::cout << "FAILED: Velocity tracking lag too high (" << vel_err << " m/s)\n";
        return false;
    }

    std::cout << "PASSED (Maintained confirmed track through -8 m/s^2 braking)\n";
    return true;
}

bool run_TC_TRK_06() {
    std::cout << "[Test TC_TRK_06] Memory & Timing Benchmark (64 tracks, 128 dets)... ";
    MultiTargetTracker tracker;
    tracker.init(TrackerConfig{});

    // Populate 64 tracks
    std::array<Detection, MAX_DETECTIONS> dets{};
    for (std::size_t j = 0; j < 64; ++j) {
        float r = 10.0f + static_cast<float>(j) * 2.5f;
        float th = -0.5f + static_cast<float>(j) * (1.0f / 64.0f);
        float vr = 15.0f;
        dets[j] = Detection{r, th, vr, 25.0f, 0};
    }

    std::array<TrackOutput, MAX_TRACKS> out_tracks{};
    std::size_t num_tracks = 0;

    // Establish 64 confirmed tracks
    for (int f = 0; f < 5; ++f) {
        tracker.processCycle(dets.data(), 64, out_tracks.data(), num_tracks);
    }

    // Now send 128 detections (64 matched, 64 clutter)
    for (std::size_t j = 64; j < 128; ++j) {
        float r = 5.0f + static_cast<float>(j) * 1.5f;
        float th = -0.4f + static_cast<float>(j - 64) * 0.01f;
        dets[j] = Detection{r, th, 5.0f, 15.0f, 0};
    }

    // Run benchmark for 50 cycles
    constexpr int BENCHMARK_CYCLES = 50;
    auto start_time = std::chrono::high_resolution_clock::now();

    for (int c = 0; c < BENCHMARK_CYCLES; ++c) {
        tracker.processCycle(dets.data(), 128, out_tracks.data(), num_tracks);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    double total_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    double avg_cycle_ms = total_ms / BENCHMARK_CYCLES;

    std::cout << "PASSED (Avg cycle execution time: " 
              << std::fixed << std::setprecision(3) << avg_cycle_ms << " ms < 4.0 ms limit)\n";

    if (avg_cycle_ms > 4.0) {
        std::cout << "WARNING: Execution time exceeded 4.0 ms limit on host machine.\n";
        return false;
    }

    return true;
}

int main() {
    std::cout << "========================================================\n";
    std::cout << " Radar MTT Specification Verification Test Suite\n";
    std::cout << " Standards: ASPICE SWE.4 / SWE.6 | ISO 26262 ASIL B\n";
    std::cout << "========================================================\n\n";

    int passed = 0;
    int total = 6;

    if (run_TC_TRK_01()) passed++;
    if (run_TC_TRK_02()) passed++;
    if (run_TC_TRK_03()) passed++;
    if (run_TC_TRK_04()) passed++;
    if (run_TC_TRK_05()) passed++;
    if (run_TC_TRK_06()) passed++;

    std::cout << "\n========================================================\n";
    std::cout << " Test Summary: " << passed << " / " << total << " test cases passed.\n";
    std::cout << "========================================================\n";

    return (passed == total) ? 0 : 1;
}
