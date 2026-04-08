#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <thread>

namespace fs = std::filesystem;

// Read whole file, trim trailing newlines.
std::string read_all(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    if (!in) return "";
    std::string s((std::istreambuf_iterator<char>(in)),
                  std::istreambuf_iterator<char>());
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r'))
        s.pop_back();
    return s;
}

// Atomic write: tmp -> rename
bool atomic_write(const fs::path& p, const std::string& content) {
    fs::path tmp = p;
    tmp += ".tmp";

    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    if (!out) return false;
    out << content;
    out.close();

    std::error_code ec;
    fs::remove(p, ec);
    fs::rename(tmp, p, ec);
    return !ec;
}

long long now_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(
        steady_clock::now().time_since_epoch()).count();
}

double rand_range(std::mt19937& rng, double lo, double hi) {
    std::uniform_real_distribution<double> dist(lo, hi);
    return dist(rng);
}

int main() {
    // Update this path to your KSP Scripts folder if needed.
    fs::path comm =
        R"(C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program\Ships\Script\comm.txt)";
    fs::path ready = comm;
    ready.replace_filename("comm_ready.txt");
    fs::path ack = comm;
    ack.replace_filename("comm_ack.txt");

    {
        std::error_code ec;
        fs::remove(comm, ec);
        fs::remove(ready, ec);
        fs::remove(ack, ec);
        std::ofstream clear(comm, std::ios::trunc);
    }

    const int line_count = 10;
    std::mt19937 rng(static_cast<unsigned int>(now_ms()));
    const double base_time = static_cast<double>(now_ms()) / 1000.0;
    const std::string seq = std::to_string(now_ms());

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);

    for (int i = 0; i < line_count; ++i) {
        const double u_mag = rand_range(rng, 0.0, 20.0);
        const double yaw_deg = rand_range(rng, 0.0, 360.0);
        const double pitch_deg = rand_range(rng, -10.0, 90.0);
        const double t_abs = base_time + (0.1 * static_cast<double>(i));

        oss << u_mag << ","
            << yaw_deg << ","
            << pitch_deg << ","
            << t_abs;

        if (i + 1 < line_count) oss << "\n";
    }

    if (!atomic_write(comm, oss.str())) {
        std::cerr << "Write failed. Try run as Administrator.\n";
        system("pause");
        return 1;
    }

    if (!atomic_write(ready, "READY," + seq + "\n")) {
        std::cerr << "Write ready failed. Try run as Administrator.\n";
        system("pause");
        return 1;
    }

    const long long t_send = now_ms();
    std::cout << "Wrote " << line_count << " U lines (mag,yaw,pitch,t) to " << comm << "\n";
    std::cout << "Seq (t0) = " << std::fixed << std::setprecision(3) << base_time << "\n";

    const int timeout_ms = 15000;
    const long long deadline = t_send + timeout_ms;
    while (now_ms() < deadline) {
        if (fs::exists(ack)) {
            std::string s = read_all(ack);
            std::string expect = "ACK," + seq + ",";
            if (s.rfind(expect, 0) == 0) {
                const long long t_recv = now_ms();
                std::cout << "Ack: " << s << "\n";
                std::cout << "CPP ack RTT(ms): " << (t_recv - t_send) << "\n";
                system("pause");
                return 0;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cerr << "Timeout waiting for ACK\n";
    system("pause");
    return 2;
}
