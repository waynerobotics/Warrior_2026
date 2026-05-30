// Standalone C++ port of warrior_serial/warrior_serial/nudge_sparks.py.
//
// Opens every SPARK MAX on USB, waits for each to report device_id +
// position, then nudges all of them by +5 motor rotations and prints
// before/after.
//
// Run:  ros2 run warrior_driver nudge_sparks_cli
// or:   ./build/warrior_driver/nudge_sparks_cli

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <thread>
#include <vector>

#include "warrior_driver/sparkmax/sparkmax_session.hpp"

using warrior::driver::SparkMaxSession;
using warrior::driver::list_sparkmax_ports;

namespace {
constexpr float NUDGE_ROT          = 5.0f;
constexpr auto  DISCOVERY_TIMEOUT  = std::chrono::seconds(10);
constexpr auto  HOLD               = std::chrono::seconds(5);
}  // namespace

int main()
{
    const auto ports = list_sparkmax_ports();
    if (ports.empty()) {
        std::printf("No SPARK MAX USB devices found.\n");
        return 1;
    }
    std::printf("Found %zu SPARK MAX port(s):", ports.size());
    for (const auto & p : ports) std::printf(" %s", p.c_str());
    std::printf("\n");

    std::vector<std::unique_ptr<SparkMaxSession>> sessions;
    sessions.reserve(ports.size());
    for (const auto & port : ports) {
        auto s = std::make_unique<SparkMaxSession>();
        if (!s->open(port)) {
            std::printf("[%s] open failed (busy or no permission?)\n", port.c_str());
            continue;
        }
        std::printf("[%s] opened.\n", port.c_str());
        sessions.push_back(std::move(s));
    }
    if (sessions.empty()) return 1;

    std::printf("Waiting up to %lds for all controllers to publish device_id + position...\n",
                static_cast<long>(std::chrono::duration_cast<std::chrono::seconds>(DISCOVERY_TIMEOUT).count()));
    std::printf("  (live counters: S0=Status0/applied+faults, S2=Status2/position, "
                "tx=heartbeat ticks)\n");
    std::printf("  S0 climbing => SLCAN channel open OK.  S0>0 but S2=0 => telemetry-"
                "enable not taking.\n");
    const auto t0 = std::chrono::steady_clock::now();
    auto next_diag = t0;
    while (std::chrono::steady_clock::now() - t0 < DISCOVERY_TIMEOUT) {
        bool all_ready = true;
        for (const auto & s : sessions) {
            if (s->device_id() < 0 || std::isnan(s->position_rotations())) {
                all_ready = false; break;
            }
        }
        if (all_ready) break;
        // Live diagnostics every 1 s so we can see exactly where it stalls.
        if (std::chrono::steady_clock::now() >= next_diag) {
            const double t_rel = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();
            for (const auto & s : sessions) {
                const float pos = s->position_rotations();
                std::printf("  t=%4.1fs [%s] dev=%2d S0=%-5lu S2=%-5lu other=%-5lu "
                            "tx=%-5lu pos=%s\n",
                            t_rel, s->port().c_str(), s->device_id(),
                            s->status_0_count(), s->status_2_count(),
                            s->other_frame_count(), s->tx_count(),
                            std::isnan(pos) ? "   nan" : "  ok");
            }
            next_diag += std::chrono::seconds(1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::vector<SparkMaxSession *> ready;
    for (const auto & s : sessions) {
        if (s->device_id() >= 0 && !std::isnan(s->position_rotations())) {
            ready.push_back(s.get());
        } else {
            std::printf("[%s] not ready -- device_id=%d pos=%.3f "
                        "S0=%lu S2=%lu other=%lu tx=%lu\n",
                        s->port().c_str(), s->device_id(),
                        s->position_rotations(),
                        s->status_0_count(), s->status_2_count(),
                        s->other_frame_count(), s->tx_count());
        }
    }
    if (ready.empty()) {
        std::printf("No controllers became ready.\n");
        return 2;
    }
    std::printf("Ready:");
    for (auto * s : ready) std::printf(" (%s,dev=%d)", s->port().c_str(), s->device_id());
    std::printf("\n");

    std::vector<float> starts;
    starts.reserve(ready.size());
    for (auto * s : ready) {
        const float start = s->position_rotations();
        const float target = start + NUDGE_ROT;
        starts.push_back(start);
        s->set_target_position(target);
        s->enable();
        std::printf("[%s] dev=%d start=%+.3f -> target=%+.3f (delta=%+.3f)\n",
                    s->port().c_str(), s->device_id(), start, target, NUDGE_ROT);
    }

    const auto deadline = std::chrono::steady_clock::now() + HOLD;
    auto next_log = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < deadline) {
        if (std::chrono::steady_clock::now() >= next_log) {
            const double t_rel = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();
            for (auto * s : ready) {
                std::printf("[%s] t=%4.1fs dev=%d pos=%+7.3f out=%+6.1f%% "
                            "faults=0x%04X tx=%lu\n",
                            s->port().c_str(), t_rel, s->device_id(),
                            s->position_rotations(),
                            s->applied_output_percent(),
                            s->faults(), s->tx_count());
            }
            next_log += std::chrono::seconds(1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    for (auto * s : ready) s->disable();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    int moved = 0;
    for (std::size_t i = 0; i < ready.size(); ++i) {
        auto * s = ready[i];
        const float final_pos = s->position_rotations();
        const float delta = final_pos - starts[i];
        std::printf("[%s] dev=%d final=%+.3f moved=%+.3f last_out=%+.1f%% "
                    "last_faults=0x%04X\n",
                    s->port().c_str(), s->device_id(), final_pos, delta,
                    s->applied_output_percent(), s->faults());
        if (std::fabs(delta) > 0.1f) ++moved;
    }
    std::printf("Done. %d/%zu moved.\n", moved, ready.size());
    return (moved == static_cast<int>(ready.size())) ? 0 : 2;
}
