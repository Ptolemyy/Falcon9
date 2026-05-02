#include "planner_mission.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

struct Scenario {
    double rp_km;
    double ra_km;
    double inc_deg;
    double payload_kg;
    unsigned worker_count = 0;
    bool coarse_1s = false;
};

const char* scenario_mode(const Scenario& s) {
    return s.coarse_1s ? "coarse1s" : "refined";
}

bool has_reserve_report_line(const falcon9::MissionResult& r) {
    for (const std::wstring& line : r.lines) {
        if (line.rfind(L"[Reserve Fuel]", 0) == 0) return true;
    }
    return false;
}

size_t separation_series_point_count(const falcon9::MissionResult& r, const std::wstring& name) {
    for (const falcon9::Series& series : r.separation_time_series) {
        if (series.name == name) return series.pts.size();
    }
    return 0;
}

void print_result(const Scenario& s, const falcon9::MissionResult& r) {
    const bool reserve_report_line = has_reserve_report_line(r);
    const size_t s1_sweep_pts = separation_series_point_count(r, L"Stage1 Reserve Propellant");
    const size_t s2_sweep_pts = separation_series_point_count(r, L"Stage2 Remaining Propellant");
    const size_t margin_sweep_pts = separation_series_point_count(r, L"Landing Margin");

    if (!std::isfinite(r.best_candidate.score)) {
        std::string status_ascii;
        status_ascii.reserve(r.status.size());
        for (wchar_t c : r.status) status_ascii.push_back((c >= 0 && c <= 0x7f) ? static_cast<char>(c) : '?');
        std::cout << std::fixed << std::setprecision(2)
                  << "scenario rp=" << s.rp_km
                  << " ra=" << s.ra_km
                  << " inc=" << s.inc_deg
                  << " payload=" << s.payload_kg
                  << " workers=" << s.worker_count
                  << " mode=" << scenario_mode(s)
                  << " -> status=" << status_ascii
                  << " payload_ok=0 no_accepted_candidate=1\n";
        std::cout << "reserve_line=" << (reserve_report_line ? "1" : "0")
                  << " s1_sweep_pts=" << s1_sweep_pts
                  << " s2_sweep_pts=" << s2_sweep_pts
                  << " margin_sweep_pts=" << margin_sweep_pts
                  << "\n";
        return;
    }

    const double sep_alt_km = (r.stage1.sep.r - falcon9::kRe) / 1000.0;
    const double sep_speed_mps = std::hypot(r.stage1.sep.vr, r.stage1.sep.vt);
    const double cutoff_alt_km = (r.stage2.seco.r - falcon9::kRe) / 1000.0;
    double min_s2_alt_km = 1e9;
    double max_s2_alt_km = -1e9;
    for (const auto& pt : r.stage2.traj) {
        min_s2_alt_km = std::min(min_s2_alt_km, pt.z_km);
        max_s2_alt_km = std::max(max_s2_alt_km, pt.z_km);
    }
    std::string status_ascii;
    status_ascii.reserve(r.status.size());
    for (wchar_t c : r.status) status_ascii.push_back((c >= 0 && c <= 0x7f) ? static_cast<char>(c) : '?');
    std::cout << std::fixed << std::setprecision(2)
              << "scenario rp=" << s.rp_km
              << " ra=" << s.ra_km
              << " inc=" << s.inc_deg
              << " payload=" << s.payload_kg
              << " workers=" << s.worker_count
              << " mode=" << scenario_mode(s)
              << " -> status=" << status_ascii
              << " payload_ok=" << (r.payload_search_ok ? "1" : "0")
              << " sep_t=" << r.best_candidate.sep_time_s
              << " sep_tgt_alt=" << r.best_candidate.sep_alt_target_km
              << " sep_tgt_speed=" << r.best_candidate.sep_speed_target_mps
              << " sep_tgt_gamma=" << r.best_candidate.sep_gamma_target_deg
              << " sep_alt=" << sep_alt_km
              << " sep_speed=" << sep_speed_mps
              << " ign_t=" << r.stage2.ignition_s
              << " cut_t=" << r.stage2.cutoff_s
              << " s2_burn=" << r.stage2.burn_s
              << " s2_used=" << r.stage2.used_prop
              << " s2_rem=" << r.stage2.rem_prop
              << " s2_alt_min=" << min_s2_alt_km
              << " s2_alt_max=" << max_s2_alt_km
              << " cutoff_alt=" << cutoff_alt_km
              << " cut_r_err=" << r.stage2.target_r_err_km
              << " rp_err=" << r.stage2.target_rp_err_km
              << " ra_err=" << r.stage2.target_ra_err_km
              << " fpa_err=" << r.stage2.target_fpa_err_deg
              << " s1_rem=" << r.stage1.rem_prop
              << " landing_prop=" << r.recovery.landing_prop_kg
              << " landing_margin=" << r.recovery.margin_kg
              << " td_speed=" << r.recovery.touchdown_speed_mps
              << " rec_feasible=" << (r.recovery.feasible ? "1" : "0")
              << " rec_converged=" << (r.recovery.converged ? "1" : "0")
              << " ship_lat=" << r.ship_lat_deg
              << " ship_lon=" << r.ship_lon_deg
              << " reserve_line=" << (reserve_report_line ? "1" : "0")
              << " s1_sweep_pts=" << s1_sweep_pts
              << " s2_sweep_pts=" << s2_sweep_pts
              << " margin_sweep_pts=" << margin_sweep_pts
              << "\n";
}

bool payload_positive_margin(const falcon9::MissionResult& result) {
    return falcon9::mission_payload_search_ok(result) &&
           result.recovery.feasible &&
           result.recovery.margin_kg >= -1e-3;
}

void run_max_payload_smoke(Scenario s) {
    constexpr double kMarginTolKg = 5.0;
    constexpr double kPayloadSearchCapKg = 30000.0;
    const double coarse_steps[] = {4000.0, 1000.0, 200.0, 50.0};
    constexpr double kFinalStepKg = 10.0;
    const double payload_cap_kg = std::max(kPayloadSearchCapKg, s.payload_kg);
    bool hit_search_cap = false;

    auto eval_payload = [&](double payload_kg) {
        falcon9::MissionRequest req;
        req.perigee_km = s.rp_km;
        req.apogee_km = s.ra_km;
        req.cutoff_alt_km = s.rp_km;
        req.incl_deg = s.inc_deg;
        req.payload_kg = payload_kg;
        req.lat_deg = 28.561857;
        req.launch_lon_deg = -80.577366;
        req.s1_prop_kg = 411000.0;
        req.s2_prop_kg = 107500.0;
        return falcon9::solve_mission(req, {nullptr, s.worker_count});
    };

    double best_payload_kg = 0.0;
    double first_fail_payload_kg = std::numeric_limits<double>::quiet_NaN();
    falcon9::MissionResult best = eval_payload(0.0);
    bool have_positive_margin = payload_positive_margin(best);

    for (double step_kg : coarse_steps) {
        int consecutive_failures = 0;
        double local_first_fail_payload_kg = std::numeric_limits<double>::quiet_NaN();
        while (true) {
            const double raw_probe_payload_kg = best_payload_kg + step_kg * static_cast<double>(consecutive_failures + 1);
            const double probe_payload_kg = std::min(raw_probe_payload_kg, payload_cap_kg);
            const bool capped_probe = raw_probe_payload_kg > payload_cap_kg + 1e-6;
            falcon9::MissionResult probe = eval_payload(probe_payload_kg);
            if (!payload_positive_margin(probe)) {
                if (!have_positive_margin) {
                    if (capped_probe) break;
                    best_payload_kg = probe_payload_kg;
                    continue;
                }
                if (!std::isfinite(local_first_fail_payload_kg)) local_first_fail_payload_kg = probe_payload_kg;
                ++consecutive_failures;
                if (consecutive_failures >= 3 || capped_probe) {
                    first_fail_payload_kg = local_first_fail_payload_kg;
                    break;
                }
                continue;
            }
            have_positive_margin = true;
            best_payload_kg = probe_payload_kg;
            best = std::move(probe);
            consecutive_failures = 0;
            local_first_fail_payload_kg = std::numeric_limits<double>::quiet_NaN();
            if (capped_probe) {
                hit_search_cap = true;
                break;
            }
            if (best.recovery.margin_kg <= kMarginTolKg) break;
        }
        if (hit_search_cap) break;
        if (!have_positive_margin) break;
    }

    if (!have_positive_margin) {
        s.payload_kg = 0.0;
        print_result(s, best);
        std::cout << "max_payload_result=NA first_fail=0.00 coarse_scan_no_feasible=1\n";
        return;
    }

    if (!hit_search_cap && std::isfinite(first_fail_payload_kg)) {
        double lo = best_payload_kg;
        double hi = std::max(first_fail_payload_kg, lo + kFinalStepKg);
        while (hi - lo > kFinalStepKg) {
            const double probe_payload_kg = 0.5 * (lo + hi);
            falcon9::MissionResult probe = eval_payload(probe_payload_kg);
            if (payload_positive_margin(probe)) {
                lo = probe_payload_kg;
                best_payload_kg = probe_payload_kg;
                best = std::move(probe);
            } else {
                hi = probe_payload_kg;
                first_fail_payload_kg = probe_payload_kg;
            }
        }
    }

    if (!std::isfinite(first_fail_payload_kg)) first_fail_payload_kg = hit_search_cap ? payload_cap_kg : best_payload_kg + kFinalStepKg;
    s.payload_kg = best_payload_kg;
    print_result(s, best);
    std::cout << std::fixed << std::setprecision(2)
              << "max_payload_result=" << best_payload_kg
              << " first_fail=" << first_fail_payload_kg
              << " margin=" << best.recovery.margin_kg
              << " hit_search_cap=" << (hit_search_cap ? "1" : "0")
              << "\n";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc == 6 && std::string(argv[1]) == "--max-payload") {
        run_max_payload_smoke({
            std::atof(argv[2]),
            std::atof(argv[3]),
            std::atof(argv[4]),
            0.0,
            static_cast<unsigned>(std::strtoul(argv[5], nullptr, 10)),
        });
        return 0;
    }

    std::vector<Scenario> scenarios;
    if ((argc == 6 || argc == 7) && std::string(argv[1]) == "--coarse1s") {
        scenarios.push_back({
            std::atof(argv[2]),
            std::atof(argv[3]),
            std::atof(argv[4]),
            std::atof(argv[5]),
            argc == 7 ? static_cast<unsigned>(std::strtoul(argv[6], nullptr, 10)) : 0u,
            true,
        });
    } else if (argc == 5 || argc == 6) {
        scenarios.push_back({
            std::atof(argv[1]),
            std::atof(argv[2]),
            std::atof(argv[3]),
            std::atof(argv[4]),
            argc == 6 ? static_cast<unsigned>(std::strtoul(argv[5], nullptr, 10)) : 0u,
        });
    } else {
        scenarios = {
            {200.0, 200.0, 28.5, 12000.0, 0},
            {200.0, 550.0, 28.5, 12000.0, 0},
            {550.0, 550.0, 43.0, 12000.0, 0},
        };
    }

    for (const Scenario& s : scenarios) {
        falcon9::MissionRequest req;
        req.perigee_km = s.rp_km;
        req.apogee_km = s.ra_km;
        req.cutoff_alt_km = s.rp_km;
        req.incl_deg = s.inc_deg;
        req.payload_kg = s.payload_kg;
        req.lat_deg = 28.561857;
        req.launch_lon_deg = -80.577366;
        req.s1_prop_kg = 411000.0;
        req.s2_prop_kg = 107500.0;
        const falcon9::MissionResult result = falcon9::solve_mission(
            req,
            {
                nullptr,
                s.worker_count,
                s.coarse_1s ? falcon9::SeparationSearchMode::Coarse1s : falcon9::SeparationSearchMode::RefinedDefault,
            });
        print_result(s, result);
    }
    return 0;
}
