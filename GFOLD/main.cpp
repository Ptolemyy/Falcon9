#define NOMINMAX
#include <matplotlibcpp17/mplot3d.h>
#include <matplotlibcpp17/pyplot.h>
#include "GFOLD_solver.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace py = pybind11;

int main()
{
    // Problem setup
    GFOLDConfig cfg;
    cfg.tf = 57.29;
    cfg.g0 = 3.71;
    cfg.Isp = 2000.0/3.71;
    cfg.T_max = 24000.0;
    cfg.throttle_min = 0.2;
    cfg.throttle_max = 0.8;
    cfg.m0 = 2000.0;
    cfg.r0[0] = 2400.00000;
    cfg.r0[1] = 450.00000;
    cfg.r0[2] = -330.00000;
    cfg.v0[0] = -10.0000;
    cfg.v0[1] = -40.00;
    cfg.v0[2] = 10.0000;
    cfg.glide_slope_deg = 30.0;
    cfg.max_angle_deg = 45.0;

    const std::vector<int> n_list = {10, 25, 50, 100};
    bool ok_n100 = false;
    GFOLDThrustProfile profile_n100;
    GFOLDConfig plot_cfg_n100 = cfg;

    for (const int n : n_list) {
        GFOLDConfig run_cfg = cfg;
        run_cfg.steps = n;
        run_cfg.solver_n = n;

        GFOLDSolver solver(run_cfg);
        auto start = std::chrono::steady_clock::now();
        const bool ok = solver.solve(n);
        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        std::cout << "[main] p4 N=" << n
                  << " solve time: " << duration.count() << " ms"
                  << " status=" << solver.status()
                  << " ok=" << (ok ? 1 : 0) << "\n";

        if (n == 100 && ok) {
            ok_n100 = true;
            profile_n100 = solver.compute_thrust_profile();
            plot_cfg_n100 = solver.config();
        }
    }

    if (!ok_n100) {
        std::cout << "Infeasible for N=100, skip plot.\n";
        return 1;
    }

    // ---- plot ----
    py::scoped_interpreter guard{};
    auto plt = matplotlibcpp17::pyplot::import();
    auto fig = plt.figure(Args(), Kwargs("figsize"_a = py::make_tuple(10, 6)));

    auto ax1 = fig.add_subplot(Args(2, 1, 1));
    ax1.plot(Args(profile_n100.t, profile_n100.thrust_N), Kwargs("linewidth"_a = 2.0));
    const double T_max  = plot_cfg_n100.T_max;
    ax1.plot(Args(profile_n100.t, std::vector<double>(profile_n100.t.size(), T_max * plot_cfg_n100.throttle_max)), Kwargs("linestyle"_a="--"));
    ax1.plot(Args(profile_n100.t, std::vector<double>(profile_n100.t.size(), T_max * plot_cfg_n100.throttle_min)), Kwargs("linestyle"_a="--"));
    ax1.set_title(Args("Thrust (N)"));
    ax1.grid(Args(true));

    auto ax2 = fig.add_subplot(Args(2, 1, 2));
    ax2.plot(Args(profile_n100.t, profile_n100.angle_deg), Kwargs("linewidth"_a = 2.0));
    ax2.set_title(Args("Thrust angle (deg)"));
    ax2.grid(Args(true));

    plt.show();
    return 0;
}
