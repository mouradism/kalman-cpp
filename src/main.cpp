#include <iostream>
#include <iomanip>
#include <cmath>
#include "radar/TrackerTypes.hpp"
#include "radar/MultiTargetTracker.hpp"

using namespace radar::tracker;

int main() {
    std::cout << "========================================================\n";
    std::cout << " Automotive Radar Multi-Target Tracker (MTT)\n";
    std::cout << " Extended Kalman Filter (EKF) & Track Lifecycle\n";
    std::cout << " SPEC-SW-RADAR-TRK-001 | C++17 / ISO 26262 ASIL B\n";
    std::cout << "========================================================\n\n";

    MultiTargetTracker tracker;
    TrackerConfig config;
    tracker.init(config);

    std::cout << "[Tracker Initialized] Config parameters:\n";
    std::cout << "  - Cycle time      : " << (CYCLE_TIME_SEC * 1000.0f) << " ms (" 
              << (1.0f / CYCLE_TIME_SEC) << " Hz)\n";
    std::cout << "  - Max Tracks      : " << MAX_TRACKS << " (Static buffer)\n";
    std::cout << "  - Max Detections  : " << MAX_DETECTIONS << "\n";
    std::cout << "  - Gate threshold  : " << config.gate_threshold << " (Chi-Square 3-DOF)\n";
    std::cout << "  - Confirmation    : " << static_cast<int>(config.confirm_hits) << " of " 
              << static_cast<int>(config.confirm_window) << " frames\n";
    std::cout << "  - Max Coast frames: " << static_cast<int>(config.max_coast_frames) << "\n\n";

    std::cout << "Simulating real-time radar tracking of an approaching vehicle:\n";
    std::cout << "  Target: Initial pos (x=60.0m, y=2.5m), vel (vx=-15.0m/s, vy=0.0m/s)\n\n";

    float true_x = 60.0f;
    float true_y = 2.5f;
    const float true_vx = -15.0f;
    const float true_vy = 0.0f;

    std::array<TrackOutput, MAX_TRACKS> out_tracks{};
    std::size_t num_tracks = 0;

    std::cout << std::setw(6) << "Frame"
              << std::setw(10) << "True_X"
              << std::setw(10) << "Est_X"
              << std::setw(10) << "Est_Y"
              << std::setw(12) << "Est_Vx"
              << std::setw(14) << "Status"
              << std::setw(10) << "Safety" << "\n";
    std::cout << "------------------------------------------------------------------------\n";

    for (int frame = 1; frame <= 15; ++frame) {
        true_x += true_vx * CYCLE_TIME_SEC;
        true_y += true_vy * CYCLE_TIME_SEC;

        float r = std::sqrt(true_x * true_x + true_y * true_y);
        float th = std::atan2(true_y, true_x);
        float vr = (true_x * true_vx + true_y * true_vy) / r;

        Detection det{r, th, vr, 28.0f, static_cast<uint32_t>(frame * 40)};
        tracker.processCycle(&det, 1, out_tracks.data(), num_tracks);

        const auto& internal_tracks = tracker.getInternalTracks();

        for (const auto& trk : internal_tracks) {
            if (trk.status != TrackStatus::FREE) {
                const char* status_str = "UNKNOWN";
                switch (trk.status) {
                    case TrackStatus::TENTATIVE: status_str = "TENTATIVE"; break;
                    case TrackStatus::CONFIRMED: status_str = "CONFIRMED"; break;
                    case TrackStatus::COASTED:   status_str = "COASTED"; break;
                    case TrackStatus::DELETED:   status_str = "DELETED"; break;
                    default: break;
                }

                std::cout << std::fixed << std::setprecision(2)
                          << std::setw(6) << frame
                          << std::setw(10) << true_x
                          << std::setw(10) << trk.x(0, 0)
                          << std::setw(10) << trk.x(1, 0)
                          << std::setw(12) << trk.x(2, 0)
                          << std::setw(14) << status_str
                          << std::setw(10) << (trk.is_valid_safety ? "VALID" : "INVALID") << "\n";
            }
        }
    }

    std::cout << "------------------------------------------------------------------------\n";
    std::cout << "\n[Execution Complete] Tracker operates deterministically with zero heap allocation.\n";

    return 0;
}
