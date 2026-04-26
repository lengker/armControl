#include "ruckig_smooth.h"

#include <ruckig/ruckig.hpp>

#include <array>
#include <stdio.h>

namespace {

constexpr size_t DOFs = 3;

struct AxisRuckigState {
    ruckig::Ruckig<1> otg;
    ruckig::InputParameter<1> in;
    ruckig::OutputParameter<1> out;
    bool active;
    bool finished;

    AxisRuckigState() : otg(0.02), active(false), finished(false) {}
};

std::array<AxisRuckigState, DOFs> g_axes;
bool g_active = false;

constexpr double V_MAX[DOFs] = {120.0, 120.0, 160.0};      // deg/s
constexpr double A_MAX[DOFs] = {600.0, 600.0, 800.0};      // deg/s^2
constexpr double J_MAX[DOFs] = {6000.0, 6000.0, 8000.0};   // deg/s^3

}  // namespace

extern "C" {

void RuckigSmooth_Init(float dt_s) {
    (void)dt_s;

    for (size_t i = 0; i < DOFs; ++i) {
        g_axes[i].in = ruckig::InputParameter<1>();
        g_axes[i].out = ruckig::OutputParameter<1>();
        g_axes[i].active = false;
        g_axes[i].finished = false;
        g_axes[i].otg.reset();
    }
    g_active = false;
}

int RuckigSmooth_Start(const float q_current_deg[3], const float q_target_deg[3]) {
    size_t i;

    if (!q_current_deg || !q_target_deg) {
        return -10;
    }

    for (i = 0; i < DOFs; ++i) {
        g_axes[i].in.current_position = {q_current_deg[i]};
        g_axes[i].in.current_velocity = {0.0};
        g_axes[i].in.current_acceleration = {0.0};

        g_axes[i].in.target_position = {q_target_deg[i]};
        g_axes[i].in.target_velocity = {0.0};
        g_axes[i].in.target_acceleration = {0.0};

        g_axes[i].in.max_velocity = {V_MAX[i]};
        g_axes[i].in.max_acceleration = {A_MAX[i]};
        g_axes[i].in.max_jerk = {J_MAX[i]};

        if (!g_axes[i].in.validate<false>(false, true)) {
            g_active = false;
            return (-13 - (int)i);
        }

        g_axes[i].out = ruckig::OutputParameter<1>();
        g_axes[i].active = true;
        g_axes[i].finished = false;
        g_axes[i].otg.reset();
    }

    g_active = true;
    return 0;
}

int RuckigSmooth_Step(float q_out_deg[3]) {
    size_t i;
    bool all_finished = true;

    if (!q_out_deg) {
        return -20;
    }
    if (!g_active) {
        return -21;
    }

    for (i = 0; i < DOFs; ++i) {
        if (g_axes[i].finished) {
            q_out_deg[i] = (float)g_axes[i].in.target_position[0];
            continue;
        }

        if (g_axes[i].active) {
            const auto res = g_axes[i].otg.update(g_axes[i].in, g_axes[i].out);
            q_out_deg[i] = (float)g_axes[i].out.new_position[0];

            if (res == ruckig::Result::Working) {
                g_axes[i].out.pass_to_input(g_axes[i].in);
                all_finished = false;
                continue;
            }

            if (res == ruckig::Result::Finished) {
                g_axes[i].finished = true;
                g_axes[i].active = false;
                q_out_deg[i] = (float)g_axes[i].out.new_position[0];
                continue;
            }

            printf("[RuckigCore] axis=%u step failed res=%d\r\n", (unsigned int)i, (int)res);
            g_active = false;
            return (-30 - (int)i);
        }

        all_finished = false;
    }

    if (all_finished) {
        g_active = false;
        return 1;
    }

    return 0;
}

}  // extern "C"
