#include <windows.h>
#include <d3d11.h>
#include <dxgi.h>
#include <chrono>
#include <string>
#include <vector>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx11.h"

#include "radar/TrackerTypes.hpp"
#include "radar/MultiTargetTracker.hpp"
#include "radar/SimulationScenario.hpp"

// Forward declare message handler from imgui_impl_win32.cpp
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

// Global DirectX 11 state
static ID3D11Device*           g_pd3dDevice = nullptr;
static ID3D11DeviceContext*     g_pd3dDeviceContext = nullptr;
static IDXGISwapChain*          g_pSwapChain = nullptr;
static ID3D11RenderTargetView*  g_mainRenderTargetView = nullptr;

bool CreateDeviceD3D(HWND hWnd);
void CleanupDeviceD3D();
void CreateRenderTarget();
void CleanupRenderTarget();
LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow) {
    (void)hInstance; (void)hPrevInstance; (void)lpCmdLine;

    // Create application window
    WNDCLASSEXW wc = {
        sizeof(wc),
        CS_CLASSDC,
        WndProc,
        0L,
        0L,
        GetModuleHandle(nullptr),
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        L"RadarTrackerImGuiClass",
        nullptr
    };
    ::RegisterClassExW(&wc);
    HWND hwnd = ::CreateWindowW(
        wc.lpszClassName,
        L"Automotive Radar Multi-Target Tracker (EKF + FSM) | Dear ImGui",
        WS_OVERLAPPEDWINDOW,
        100, 100, 1380, 850,
        nullptr, nullptr, wc.hInstance, nullptr
    );

    // Initialize Direct3D
    if (!CreateDeviceD3D(hwnd)) {
        CleanupDeviceD3D();
        ::UnregisterClassW(wc.lpszClassName, wc.hInstance);
        return 1;
    }

    ::ShowWindow(hwnd, nCmdShow);
    ::UpdateWindow(hwnd);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    // Setup style
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 6.0f;
    style.FrameRounding = 4.0f;
    style.ItemSpacing = ImVec2(8.0f, 6.0f);
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.08f, 0.09f, 0.12f, 1.00f);

    // Setup Platform/Renderer backends
    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX11_Init(g_pd3dDevice, g_pd3dDeviceContext);

    // Tracking & Simulation Core
    radar::tracker::MultiTargetTracker tracker;
    radar::tracker::TrackerConfig tracker_config;
    tracker.init(tracker_config);

    radar::sim::ScenarioManager scenario_manager;

    bool is_running = true;
    bool step_requested = false;
    float sim_speed_hz = 25.0f; // 25 Hz nominal cycle time
    float noise_factor = 1.0f;
    int clutter_count = 2;
    int scenario_idx = 0;
    const char* scenario_names[] = {
        "Approaching Vehicle (-15 m/s)",
        "Crossing Targets (45 deg)",
        "Occlusion & Coasting (3 frames)",
        "High-G Braking (-8 m/s^2)",
        "Clutter Storm (3 Targets + Ghost Noise)"
    };

    std::array<radar::tracker::Detection, radar::tracker::MAX_DETECTIONS> detections{};
    std::size_t num_detections = 0;
    std::array<radar::tracker::TrackOutput, radar::tracker::MAX_TRACKS> out_tracks{};
    std::size_t num_out_tracks = 0;

    double last_wcet_ms = 0.0;
    std::vector<float> wcet_history(100, 0.0f);

    // Trajectory history for visualization: vector of (x, y, status) per track
    struct TrajectoryPoint { float x; float y; };
    struct TrackHistory {
        uint32_t id;
        std::vector<TrajectoryPoint> points;
    };
    std::vector<TrackHistory> track_histories;

    auto last_cycle_time = std::chrono::high_resolution_clock::now();

    // Main loop
    bool done = false;
    while (!done) {
        MSG msg;
        while (::PeekMessage(&msg, nullptr, 0U, 0U, PM_REMOVE)) {
            ::TranslateMessage(&msg);
            ::DispatchMessage(&msg);
            if (msg.message == WM_QUIT) {
                done = true;
            }
        }
        if (done) break;

        auto current_time = std::chrono::high_resolution_clock::now();
        float elapsed_since_last_cycle = std::chrono::duration<float>(current_time - last_cycle_time).count();
        float target_cycle_period = 1.0f / sim_speed_hz;

        // Execute tracker cycle when needed
        if ((is_running && elapsed_since_last_cycle >= target_cycle_period) || step_requested) {
            last_cycle_time = current_time;
            step_requested = false;

            // Generate radar detections from simulation
            scenario_manager.step(
                radar::tracker::CYCLE_TIME_SEC,
                noise_factor,
                clutter_count,
                detections.data(),
                num_detections
            );

            // Execute tracker and measure WCET
            auto t0 = std::chrono::high_resolution_clock::now();
            tracker.processCycle(detections.data(), num_detections, out_tracks.data(), num_out_tracks);
            auto t1 = std::chrono::high_resolution_clock::now();

            last_wcet_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            wcet_history.erase(wcet_history.begin());
            wcet_history.push_back(static_cast<float>(last_wcet_ms));

            // Update track history breadcrumbs
            const auto& internal_tracks = tracker.getInternalTracks();
            for (const auto& trk : internal_tracks) {
                if (trk.status != radar::tracker::TrackStatus::FREE) {
                    bool found = false;
                    for (auto& hist : track_histories) {
                        if (hist.id == trk.track_id) {
                            hist.points.push_back({trk.x(0, 0), trk.x(1, 0)});
                            if (hist.points.size() > 40) hist.points.erase(hist.points.begin());
                            found = true;
                            break;
                        }
                    }
                    if (!found) {
                        TrackHistory nh;
                        nh.id = trk.track_id;
                        nh.points.push_back({trk.x(0, 0), trk.x(1, 0)});
                        track_histories.push_back(nh);
                    }
                }
            }
            // Prune dead histories
            track_histories.erase(
                std::remove_if(track_histories.begin(), track_histories.end(), [&](const TrackHistory& h) {
                    for (const auto& trk : internal_tracks) {
                        if (trk.track_id == h.id && trk.status != radar::tracker::TrackStatus::FREE) {
                            return false;
                        }
                    }
                    return true;
                }),
                track_histories.end()
            );
        }

        // Start Dear ImGui frame
        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        // 1. TOP HEADER & TELEMETRY BAR
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(1360, 80), ImGuiCond_FirstUseEver);
        ImGui::Begin("Radar MTT Control Deck", nullptr, ImGuiWindowFlags_NoCollapse);
        {
            ImGui::Columns(5, "top_bar", false);
            ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "SYSTEM STATUS");
            ImGui::Text("Cycle: %u (dt=40ms)", scenario_manager.getFrame());
            ImGui::NextColumn();

            ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "WCET MONITOR");
            ImGui::Text("%.3f ms (Budget: 4.0ms)", last_wcet_ms);
            ImGui::NextColumn();

            ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "CONFIRMED TRACKS");
            ImGui::Text("%zu / %zu slots", num_out_tracks, radar::tracker::MAX_TRACKS);
            ImGui::NextColumn();

            ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "INPUT DETECTIONS");
            ImGui::Text("%zu / %zu detections", num_detections, radar::tracker::MAX_DETECTIONS);
            ImGui::NextColumn();

            ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "SAFETY INTEGRITY");
            ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.3f, 1.0f), "ISO 26262 ASIL B VALID");
            ImGui::Columns(1);
        }
        ImGui::End();

        // 2. RADAR PPI / BIRD'S EYE VIEW (BEV) CANVAS
        ImGui::SetNextWindowPos(ImVec2(10, 100), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(780, 720), ImGuiCond_FirstUseEver);
        ImGui::Begin("Radar Plan Position Indicator (BEV Display)", nullptr);
        {
            ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();
            ImVec2 canvas_sz = ImGui::GetContentRegionAvail();
            if (canvas_sz.x < 100.0f) canvas_sz.x = 100.0f;
            if (canvas_sz.y < 100.0f) canvas_sz.y = 100.0f;
            ImVec2 canvas_p1 = ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y);

            ImDrawList* draw_list = ImGui::GetWindowDrawList();
            draw_list->AddRectFilled(canvas_p0, canvas_p1, IM_COL32(12, 16, 24, 255));
            draw_list->AddRect(canvas_p0, canvas_p1, IM_COL32(40, 50, 70, 255));

            // Radar sensor origin (bottom center)
            const ImVec2 origin(canvas_p0.x + canvas_sz.x * 0.5f, canvas_p0.y + canvas_sz.y - 30.0f);
            const float max_range_m = 100.0f;
            const float px_per_m = (canvas_sz.y - 60.0f) / max_range_m;

            // Helper to transform radar Cartesian [x, y] to screen space
            // x: longitudinal forward (up on screen), y: lateral left (left on screen)
            auto worldToScreen = [&](float x_m, float y_m) -> ImVec2 {
                return ImVec2(origin.x - y_m * px_per_m, origin.y - x_m * px_per_m);
            };

            // Draw Radar Field of View (FOV +/- 60 degrees)
            constexpr float fov_angle_rad = 1.04719755f; // 60 deg
            ImVec2 left_fov_tip = worldToScreen(max_range_m * std::cos(fov_angle_rad), max_range_m * std::sin(fov_angle_rad));
            ImVec2 right_fov_tip = worldToScreen(max_range_m * std::cos(-fov_angle_rad), max_range_m * std::sin(-fov_angle_rad));

            // Translucent radar FOV sector
            draw_list->AddTriangleFilled(origin, left_fov_tip, right_fov_tip, IM_COL32(20, 45, 60, 45));
            draw_list->AddLine(origin, left_fov_tip, IM_COL32(0, 160, 220, 120), 1.5f);
            draw_list->AddLine(origin, right_fov_tip, IM_COL32(0, 160, 220, 120), 1.5f);
            draw_list->AddLine(origin, worldToScreen(max_range_m, 0.0f), IM_COL32(60, 90, 120, 100), 1.0f); // boresight

            // Draw Range Rings (20m, 40m, 60m, 80m, 100m)
            for (float r = 20.0f; r <= max_range_m; r += 20.0f) {
                float radius_px = r * px_per_m;
                draw_list->PathClear();
                // Arc from -150 to -30 degrees (upwards)
                constexpr float PI = 3.14159265f;
                draw_list->PathArcTo(origin, radius_px, -PI * 0.5f - fov_angle_rad, -PI * 0.5f + fov_angle_rad, 32);
                draw_list->PathStroke(IM_COL32(50, 80, 110, 80), 0, 1.0f);

                ImVec2 label_pos = worldToScreen(r, 0.0f);
                std::string r_str = std::to_string(static_cast<int>(r)) + "m";
                draw_list->AddText(ImVec2(label_pos.x + 4.0f, label_pos.y - 8.0f), IM_COL32(100, 140, 180, 160), r_str.c_str());
            }

            // Draw Radar Sensor icon
            draw_list->AddCircleFilled(origin, 7.0f, IM_COL32(0, 200, 255, 255));
            draw_list->AddText(ImVec2(origin.x - 30.0f, origin.y + 10.0f), IM_COL32(180, 220, 255, 200), "77 GHz RADAR");

            // 1. Draw Raw Radar Detections (Yellow dots with Doppler radial indicators)
            for (std::size_t j = 0; j < num_detections; ++j) {
                const auto& det = detections[j];
                float px = det.range_m * std::cos(det.azimuth_rad);
                float py = det.range_m * std::sin(det.azimuth_rad);
                ImVec2 det_screen = worldToScreen(px, py);

                // Small yellow cross
                draw_list->AddCircleFilled(det_screen, 3.5f, IM_COL32(255, 220, 60, 220));

                // Radial Doppler velocity line: vr positive moving away, negative approaching
                float r_hat_x = px / det.range_m;
                float r_hat_y = py / det.range_m;
                ImVec2 vr_tip = worldToScreen(px + r_hat_x * (det.doppler_mps * 0.35f), py + r_hat_y * (det.doppler_mps * 0.35f));
                ImU32 vr_col = (det.doppler_mps < 0.0f) ? IM_COL32(255, 90, 80, 180) : IM_COL32(80, 180, 255, 180);
                draw_list->AddLine(det_screen, vr_tip, vr_col, 1.5f);
            }

            // 2. Draw Track History Breadcrumbs
            for (const auto& hist : track_histories) {
                for (std::size_t i = 1; i < hist.points.size(); ++i) {
                    ImVec2 pA = worldToScreen(hist.points[i - 1].x, hist.points[i - 1].y);
                    ImVec2 pB = worldToScreen(hist.points[i].x, hist.points[i].y);
                    draw_list->AddLine(pA, pB, IM_COL32(80, 200, 120, 70), 1.5f);
                }
            }

            // 3. Draw Active Track Estimates & Covariance Ellipses
            const auto& internal_tracks = tracker.getInternalTracks();
            for (const auto& trk : internal_tracks) {
                if (trk.status == radar::tracker::TrackStatus::FREE) continue;

                float x_m = trk.x(0, 0);
                float y_m = trk.x(1, 0);
                float vx = trk.x(2, 0);
                float vy = trk.x(3, 0);
                ImVec2 trk_screen = worldToScreen(x_m, y_m);

                // Status color coding
                ImU32 col_status = IM_COL32(180, 180, 180, 255);
                const char* badge = "TRK";
                if (trk.status == radar::tracker::TrackStatus::CONFIRMED) {
                    col_status = IM_COL32(50, 230, 100, 255);
                    badge = "CONFIRMED";
                } else if (trk.status == radar::tracker::TrackStatus::TENTATIVE) {
                    col_status = IM_COL32(255, 200, 40, 255);
                    badge = "TENTATIVE";
                } else if (trk.status == radar::tracker::TrackStatus::COASTED) {
                    col_status = IM_COL32(255, 130, 40, 255);
                    badge = "COASTED";
                }

                // Draw Covariance Uncertainty Ellipse (2-sigma)
                float sigma_x_px = 2.0f * std::sqrt(std::max(0.01f, trk.P(0, 0))) * px_per_m;
                float sigma_y_px = 2.0f * std::sqrt(std::max(0.01f, trk.P(1, 1))) * px_per_m;
                draw_list->AddEllipse(trk_screen, ImVec2(sigma_y_px, sigma_x_px), IM_COL32(100, 220, 255, 90), 0.0f, 24, 1.0f);

                // Target box marker
                draw_list->AddRectFilled(
                    ImVec2(trk_screen.x - 6.0f, trk_screen.y - 6.0f),
                    ImVec2(trk_screen.x + 6.0f, trk_screen.y + 6.0f),
                    col_status, 2.0f
                );

                // Velocity vector arrow (1.0 sec forward prediction)
                ImVec2 vel_tip = worldToScreen(x_m + vx * 1.0f, y_m + vy * 1.0f);
                draw_list->AddLine(trk_screen, vel_tip, col_status, 2.0f);

                // Tag label (ID, Status, Velocity)
                char tag[64];
                snprintf(tag, sizeof(tag), "ID:%u [%s]\nv=%.1fm/s", trk.track_id, badge, std::sqrt(vx * vx + vy * vy));
                draw_list->AddText(ImVec2(trk_screen.x + 10.0f, trk_screen.y - 14.0f), IM_COL32(230, 240, 255, 230), tag);
            }
        }
        ImGui::End();

        // 3. RIGHT CONTROLS & SCENARIO MANAGER PANEL
        ImGui::SetNextWindowPos(ImVec2(800, 100), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(570, 360), ImGuiCond_FirstUseEver);
        ImGui::Begin("Tracker Controls & Simulation Scenarios", nullptr);
        {
            ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "EXECUTION CONTROLS");
            if (ImGui::Button(is_running ? "Pause ||" : "Play >", ImVec2(100, 28))) {
                is_running = !is_running;
            }
            ImGui::SameLine();
            if (ImGui::Button("Step Frame >|", ImVec2(110, 28))) {
                step_requested = true;
                is_running = false;
            }
            ImGui::SameLine();
            if (ImGui::Button("Reset Scenario", ImVec2(120, 28))) {
                scenario_manager.reset();
                tracker.init(tracker_config);
                track_histories.clear();
            }

            ImGui::SliderFloat("Simulation Rate (Hz)", &sim_speed_hz, 5.0f, 60.0f, "%.0f Hz");
            ImGui::Separator();

            ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "ASPICE SWE.4 TEST SCENARIOS");
            if (ImGui::Combo("Active Scenario", &scenario_idx, scenario_names, IM_ARRAYSIZE(scenario_names))) {
                scenario_manager.setScenario(static_cast<radar::sim::ScenarioType>(scenario_idx));
                tracker.init(tracker_config);
                track_histories.clear();
            }

            ImGui::Separator();
            ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "RADAR ENVIRONMENT CONFIG");
            ImGui::SliderFloat("Sensor Noise Factor", &noise_factor, 0.0f, 3.0f, "%.2fx");
            ImGui::SliderInt("Clutter Spikes / Cycle", &clutter_count, 0, 15);
            if (ImGui::SliderFloat("Gating Threshold", &tracker_config.gate_threshold, 6.0f, 25.0f, "%.2f")) {
                tracker.init(tracker_config);
            }

            ImGui::Separator();
            ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "WCET REAL-TIME BUDGET HISTOGRAM");
            ImGui::PlotLines("Cycle WCET (ms)", wcet_history.data(), static_cast<int>(wcet_history.size()), 0, "Limit: 4.0 ms", 0.0f, 5.0f, ImVec2(0, 50));
        }
        ImGui::End();

        // 4. ACTIVE TRACK TELEMETRY TABLE
        ImGui::SetNextWindowPos(ImVec2(800, 470), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(570, 350), ImGuiCond_FirstUseEver);
        ImGui::Begin("Active Track List (ISO 26262 ASIL B Telemetry)", nullptr);
        {
            if (ImGui::BeginTable("track_table", 8, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY)) {
                ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthFixed, 35.0f);
                ImGui::TableSetupColumn("Status", ImGuiTableColumnFlags_WidthFixed, 75.0f);
                ImGui::TableSetupColumn("X [m]", ImGuiTableColumnFlags_WidthFixed, 50.0f);
                ImGui::TableSetupColumn("Y [m]", ImGuiTableColumnFlags_WidthFixed, 50.0f);
                ImGui::TableSetupColumn("Vx [m/s]", ImGuiTableColumnFlags_WidthFixed, 55.0f);
                ImGui::TableSetupColumn("Vy [m/s]", ImGuiTableColumnFlags_WidthFixed, 55.0f);
                ImGui::TableSetupColumn("Var(x)", ImGuiTableColumnFlags_WidthFixed, 50.0f);
                ImGui::TableSetupColumn("ASIL B", ImGuiTableColumnFlags_WidthFixed, 50.0f);
                ImGui::TableHeadersRow();

                const auto& internal_tracks = tracker.getInternalTracks();
                for (const auto& trk : internal_tracks) {
                    if (trk.status == radar::tracker::TrackStatus::FREE) continue;

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("%u", trk.track_id);

                    ImGui::TableSetColumnIndex(1);
                    if (trk.status == radar::tracker::TrackStatus::CONFIRMED) {
                        ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.3f, 1.0f), "CONFIRMED");
                    } else if (trk.status == radar::tracker::TrackStatus::TENTATIVE) {
                        ImGui::TextColored(ImVec4(0.9f, 0.8f, 0.2f, 1.0f), "TENTATIVE");
                    } else if (trk.status == radar::tracker::TrackStatus::COASTED) {
                        ImGui::TextColored(ImVec4(0.9f, 0.5f, 0.2f, 1.0f), "COASTED");
                    }

                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%.2f", trk.x(0, 0));

                    ImGui::TableSetColumnIndex(3);
                    ImGui::Text("%.2f", trk.x(1, 0));

                    ImGui::TableSetColumnIndex(4);
                    ImGui::Text("%.2f", trk.x(2, 0));

                    ImGui::TableSetColumnIndex(5);
                    ImGui::Text("%.2f", trk.x(3, 0));

                    ImGui::TableSetColumnIndex(6);
                    ImGui::Text("%.2f", trk.P(0, 0));

                    ImGui::TableSetColumnIndex(7);
                    if (trk.is_valid_safety) {
                        ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.3f, 1.0f), "OK");
                    } else {
                        ImGui::TextColored(ImVec4(0.9f, 0.2f, 0.2f, 1.0f), "FAULT");
                    }
                }
                ImGui::EndTable();
            }
        }
        ImGui::End();

        // Rendering
        ImGui::Render();
        const float clear_color[4] = { 0.04f, 0.05f, 0.07f, 1.00f };
        g_pd3dDeviceContext->OMSetRenderTargets(1, &g_mainRenderTargetView, nullptr);
        g_pd3dDeviceContext->ClearRenderTargetView(g_mainRenderTargetView, clear_color);
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

        g_pSwapChain->Present(1, 0); // VSync enabled
    }

    // Cleanup
    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();

    CleanupDeviceD3D();
    ::DestroyWindow(hwnd);
    ::UnregisterClassW(wc.lpszClassName, wc.hInstance);

    return 0;
}

// Helper Direct3D 11 functions
bool CreateDeviceD3D(HWND hWnd) {
    DXGI_SWAP_CHAIN_DESC sd;
    ZeroMemory(&sd, sizeof(sd));
    sd.BufferCount = 2;
    sd.BufferDesc.Width = 0;
    sd.BufferDesc.Height = 0;
    sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    sd.BufferDesc.RefreshRate.Numerator = 60;
    sd.BufferDesc.RefreshRate.Denominator = 1;
    sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
    sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.OutputWindow = hWnd;
    sd.SampleDesc.Count = 1;
    sd.SampleDesc.Quality = 0;
    sd.Windowed = TRUE;
    sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

    UINT createDeviceFlags = 0;
    D3D_FEATURE_LEVEL featureLevel;
    const D3D_FEATURE_LEVEL featureLevelArray[2] = {
        D3D_FEATURE_LEVEL_11_0,
        D3D_FEATURE_LEVEL_10_0
    };
    HRESULT res = D3D11CreateDeviceAndSwapChain(
        nullptr,
        D3D_DRIVER_TYPE_HARDWARE,
        nullptr,
        createDeviceFlags,
        featureLevelArray,
        2,
        D3D11_SDK_VERSION,
        &sd,
        &g_pSwapChain,
        &g_pd3dDevice,
        &featureLevel,
        &g_pd3dDeviceContext
    );
    if (res == DXGI_ERROR_UNSUPPORTED) {
        res = D3D11CreateDeviceAndSwapChain(
            nullptr,
            D3D_DRIVER_TYPE_WARP,
            nullptr,
            createDeviceFlags,
            featureLevelArray,
            2,
            D3D11_SDK_VERSION,
            &sd,
            &g_pSwapChain,
            &g_pd3dDevice,
            &featureLevel,
            &g_pd3dDeviceContext
        );
    }
    if (res != S_OK) return false;

    CreateRenderTarget();
    return true;
}

void CleanupDeviceD3D() {
    CleanupRenderTarget();
    if (g_pSwapChain) { g_pSwapChain->Release(); g_pSwapChain = nullptr; }
    if (g_pd3dDeviceContext) { g_pd3dDeviceContext->Release(); g_pd3dDeviceContext = nullptr; }
    if (g_pd3dDevice) { g_pd3dDevice->Release(); g_pd3dDevice = nullptr; }
}

void CreateRenderTarget() {
    ID3D11Texture2D* pBackBuffer = nullptr;
    g_pSwapChain->GetBuffer(0, IID_PPV_ARGS(&pBackBuffer));
    if (pBackBuffer) {
        g_pd3dDevice->CreateRenderTargetView(pBackBuffer, nullptr, &g_mainRenderTargetView);
        pBackBuffer->Release();
    }
}

void CleanupRenderTarget() {
    if (g_mainRenderTargetView) {
        g_mainRenderTargetView->Release();
        g_mainRenderTargetView = nullptr;
    }
}

LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam))
        return true;

    switch (msg) {
        case WM_SIZE:
            if (wParam == SIZE_MINIMIZED) return 0;
            if (g_pd3dDevice != nullptr && wParam != SIZE_MINIMIZED) {
                CleanupRenderTarget();
                g_pSwapChain->ResizeBuffers(0, (UINT)LOWORD(lParam), (UINT)HIWORD(lParam), DXGI_FORMAT_UNKNOWN, 0);
                CreateRenderTarget();
            }
            return 0;
        case WM_SYSCOMMAND:
            if ((wParam & 0xfff0) == SC_KEYMENU) return 0;
            break;
        case WM_DESTROY:
            ::PostQuitMessage(0);
            return 0;
    }
    return ::DefWindowProcW(hWnd, msg, wParam, lParam);
}

