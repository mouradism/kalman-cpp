#pragma once

#include "radar/TrackerTypes.hpp"
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>

namespace radar::sim {

enum class ScenarioType {
    APPROACHING_VEHICLE,
    CROSSING_TARGETS,
    OCCLUSION_COASTING,
    HIGH_G_BRAKING,
    CLUTTER_STORM
};

struct SimulatedObject {
    float x{0.0f};
    float y{0.0f};
    float vx{0.0f};
    float vy{0.0f};
    float ax{0.0f};
    float ay{0.0f};
    bool is_occluded{false};
};

class ScenarioManager {
public:
    ScenarioManager() : rng_(1337) {
        setScenario(ScenarioType::APPROACHING_VEHICLE);
    }

    void setScenario(ScenarioType type) {
        current_scenario_ = type;
        frame_idx_ = 0;
        objects_.clear();

        switch (type) {
            case ScenarioType::APPROACHING_VEHICLE:
                // Lead vehicle approaching at -15 m/s
                objects_.push_back({65.0f, 2.0f, -15.0f, 0.0f, 0.0f, 0.0f, false});
                break;

            case ScenarioType::CROSSING_TARGETS:
                // Target A: left to right
                objects_.push_back({30.0f, -12.0f, 8.0f, 6.0f, 0.0f, 0.0f, false});
                // Target B: right to left
                objects_.push_back({30.0f, 12.0f, 8.0f, -6.0f, 0.0f, 0.0f, false});
                break;

            case ScenarioType::OCCLUSION_COASTING:
                // Vehicle moving forward, occluded between frames 25 and 30
                objects_.push_back({25.0f, -1.5f, 12.0f, 0.0f, 0.0f, 0.0f, false});
                break;

            case ScenarioType::HIGH_G_BRAKING:
                // Fast target braking hard (-8 m/s^2) starting at frame 20
                objects_.push_back({50.0f, 0.5f, 30.0f, 0.0f, 0.0f, 0.0f, false});
                break;

            case ScenarioType::CLUTTER_STORM:
                // 3 real vehicles surrounded by false clutter spikes
                objects_.push_back({45.0f, -3.5f, -10.0f, 0.0f, 0.0f, 0.0f, false});
                objects_.push_back({70.0f, 3.5f, 15.0f, 0.0f, 0.0f, 0.0f, false});
                objects_.push_back({30.0f, 0.0f, 8.0f, 1.0f, 0.0f, 0.0f, false});
                break;
        }
    }

    void reset() {
        setScenario(current_scenario_);
    }

    void step(
        float dt,
        float noise_factor,
        int clutter_count,
        tracker::Detection* out_detections,
        std::size_t& out_count
    ) {
        frame_idx_++;
        out_count = 0;

        std::normal_distribution<float> noise_r(0.0f, 0.25f * noise_factor);
        std::normal_distribution<float> noise_th(0.0f, 0.0087f * noise_factor);
        std::normal_distribution<float> noise_vr(0.0f, 0.15f * noise_factor);

        std::uniform_real_distribution<float> clutter_r(5.0f, 85.0f);
        std::uniform_real_distribution<float> clutter_th(-0.95f, 0.95f);
        std::uniform_real_distribution<float> clutter_vr(-20.0f, 20.0f);

        // Update ground truth objects
        for (auto& obj : objects_) {
            // High-G braking logic
            if (current_scenario_ == ScenarioType::HIGH_G_BRAKING && frame_idx_ >= 20 && obj.vx > 2.0f) {
                obj.ax = -8.0f;
            }

            // Occlusion logic
            if (current_scenario_ == ScenarioType::OCCLUSION_COASTING) {
                obj.is_occluded = (frame_idx_ >= 25 && frame_idx_ <= 29);
            }

            // Kinematic integration
            obj.vx += obj.ax * dt;
            obj.vy += obj.ay * dt;
            obj.x += obj.vx * dt;
            obj.y += obj.vy * dt;

            // Generate detection if not occluded and inside sensor FoV
            if (!obj.is_occluded && obj.x > 0.5f && obj.x < 150.0f && std::abs(obj.y) < (obj.x * 1.7f + 5.0f)) {
                float r_true = std::sqrt(obj.x * obj.x + obj.y * obj.y);
                float th_true = std::atan2(obj.y, obj.x);
                float vr_true = (obj.x * obj.vx + obj.y * obj.vy) / r_true;

                if (out_count < tracker::MAX_DETECTIONS) {
                    out_detections[out_count++] = tracker::Detection{
                        std::max(0.2f, r_true + noise_r(rng_)),
                        th_true + noise_th(rng_),
                        vr_true + noise_vr(rng_),
                        26.0f,
                        static_cast<uint32_t>(frame_idx_ * 40)
                    };
                }
            }
        }

        // Generate random clutter detections
        for (int c = 0; c < clutter_count && out_count < tracker::MAX_DETECTIONS; ++c) {
            out_detections[out_count++] = tracker::Detection{
                clutter_r(rng_),
                clutter_th(rng_),
                clutter_vr(rng_),
                12.0f,
                static_cast<uint32_t>(frame_idx_ * 40)
            };
        }
    }

    const std::vector<SimulatedObject>& getObjects() const {
        return objects_;
    }

    ScenarioType getScenarioType() const {
        return current_scenario_;
    }

    uint32_t getFrame() const {
        return frame_idx_;
    }

private:
    ScenarioType current_scenario_{ScenarioType::APPROACHING_VEHICLE};
    std::vector<SimulatedObject> objects_;
    uint32_t frame_idx_{0};
    std::mt19937 rng_;
};

} // namespace radar::sim

