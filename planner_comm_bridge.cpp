#define NOMINMAX
#include <windows.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <thread>

namespace fs = std::filesystem;

std::string trim_ascii_copy(std::string s) {
    auto is_ws = [](char c) {
        return c == ' ' || c == '\t' || c == '\r' || c == '\n';
    };
    size_t b = 0;
    size_t e = s.size();
    while (b < e && is_ws(s[b])) ++b;
    while (e > b && is_ws(s[e - 1])) --e;
    return s.substr(b, e - b);
}

std::string lower_ascii_copy(std::string s) {
    for (char& c : s) {
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
    }
    return s;
}

std::string canonical_key(std::string key) {
    key = lower_ascii_copy(trim_ascii_copy(key));
    if (key == "stage1_dry_kg") return "s1_dry_kg";
    if (key == "stage1_prop_kg") return "s1_prop_kg";
    if (key == "stage1_isp_s") return "s1_isp_s";
    if (key == "stage1_thrust_kn") return "s1_thrust_kn";
    if (key == "stage2_dry_kg") return "s2_dry_kg";
    if (key == "stage2_prop_kg") return "s2_prop_kg";
    if (key == "stage2_isp_s") return "s2_isp_s";
    if (key == "stage2_thrust_kn") return "s2_thrust_kn";
    if (key == "launch_lat_deg" || key == "launch_latitude_deg") return "lat_deg";
    if (key == "launch_lon_deg" || key == "launch_longitude_deg" || key == "lon_deg") return "launch_lon_deg";
    return key;
}

bool parse_send_file(const fs::path& send_file, std::set<std::string>& found_keys, std::string& err) {
    std::ifstream in(send_file, std::ios::in);
    if (!in) {
        err = "cannot open send file";
        return false;
    }

    int recognized = 0;
    std::string line;
    while (std::getline(in, line)) {
        if (const size_t p = line.find('#'); p != std::string::npos) line = line.substr(0, p);
        if (const size_t p = line.find("//"); p != std::string::npos) line = line.substr(0, p);
        line = trim_ascii_copy(line);
        if (line.empty()) continue;

        const size_t eq = line.find('=');
        if (eq == std::string::npos) continue;

        std::string key = canonical_key(line.substr(0, eq));
        std::string val = trim_ascii_copy(line.substr(eq + 1));
        if (key.empty() || val.empty()) continue;
        try {
            (void)std::stod(val);
        } catch (...) {
            continue;
        }
        found_keys.insert(key);
        recognized++;
    }

    if (recognized <= 0) {
        err = "no numeric key=value entries found";
        return false;
    }
    return true;
}

std::wstring quote_arg(const std::wstring& s) {
    std::wstring out = L"\"";
    for (wchar_t c : s) {
        if (c == L'"') out += L"\\\"";
        else out += c;
    }
    out += L"\"";
    return out;
}

bool launch_planner_with_config(const fs::path& planner_exe, const fs::path& cfg_file, std::wstring& err) {
    std::wstring cmd = quote_arg(planner_exe.wstring()) + L" --vehicle-config " + quote_arg(cfg_file.wstring());
    std::wstring cmd_mut = cmd;

    STARTUPINFOW si{};
    si.cb = sizeof(si);
    PROCESS_INFORMATION pi{};
    BOOL ok = CreateProcessW(
        nullptr,
        cmd_mut.data(),
        nullptr,
        nullptr,
        FALSE,
        0,
        nullptr,
        nullptr,
        &si,
        &pi);
    if (!ok) {
        err = L"CreateProcessW failed with code " + std::to_wstring(GetLastError());
        return false;
    }

    CloseHandle(pi.hThread);
    CloseHandle(pi.hProcess);
    return true;
}

int wmain(int argc, wchar_t** argv) {
    fs::path send_file =
        (argc >= 2)
            ? fs::path(argv[1])
            : fs::path(LR"(C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program\Ships\Script\send.txt)");

    fs::path planner_exe;
    if (argc >= 3) {
        planner_exe = fs::path(argv[2]);
    } else {
        wchar_t module_buf[MAX_PATH]{};
        GetModuleFileNameW(nullptr, module_buf, MAX_PATH);
        fs::path self_path(module_buf);
        planner_exe = self_path.parent_path() / L"falcon9_gui_planner.exe";
    }

    const int timeout_sec = (argc >= 4) ? std::max(1, _wtoi(argv[3])) : 120;
    const auto t0 = std::chrono::steady_clock::now();
    while (true) {
        std::error_code ec;
        if (fs::exists(send_file, ec) && fs::is_regular_file(send_file, ec) && fs::file_size(send_file, ec) > 0) {
            break;
        }
        const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - t0).count();
        if (elapsed >= timeout_sec) {
            std::wcerr << L"[bridge] timeout waiting for send file: " << send_file.wstring() << L"\n";
            return 2;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    std::set<std::string> found;
    std::string parse_err;
    if (!parse_send_file(send_file, found, parse_err)) {
        std::wcerr << L"[bridge] invalid send file: " << send_file.wstring()
                   << L" (" << std::wstring(parse_err.begin(), parse_err.end()) << L")\n";
        return 3;
    }

    const std::set<std::string> required = {
        "payload_kg", "s1_dry_kg", "s1_prop_kg", "s1_isp_s", "s1_thrust_kn",
        "s2_dry_kg", "s2_prop_kg", "s2_isp_s", "s2_thrust_kn",
        "lat_deg", "launch_lon_deg"
    };
    for (const std::string& k : required) {
        if (found.find(k) == found.end()) {
            std::wcout << L"[bridge] warning: missing key in send.txt -> "
                       << std::wstring(k.begin(), k.end()) << L"\n";
        }
    }

    if (!fs::exists(planner_exe)) {
        std::wcerr << L"[bridge] planner executable not found: " << planner_exe.wstring() << L"\n";
        return 4;
    }

    std::wstring launch_err;
    if (!launch_planner_with_config(planner_exe, send_file, launch_err)) {
        std::wcerr << L"[bridge] failed to launch planner: " << launch_err << L"\n";
        return 5;
    }

    std::wcout << L"[bridge] launched planner with config: " << send_file.wstring() << L"\n";
    return 0;
}
