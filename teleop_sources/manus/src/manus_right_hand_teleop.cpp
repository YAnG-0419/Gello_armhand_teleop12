// This software contains source code provided by Manus Technology Group B.V.
//
// Right-only MANUS ergonomics adapter for the existing LinkerHand UDP bridge.

#include "manus_teleop/retarget.hpp"

#include "ManusSDK.h"
#include "ManusSDKTypeInitializers.h"
#include "ManusSDKTypes.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

namespace
{
using Clock = std::chrono::steady_clock;

constexpr std::array<const char*, manus_teleop::kRobotJoints> kJointNames = {
    "pinky_mcp_roll",
    "pinky_mcp_pitch",
    "pinky_pip",
    "pinky_dip",
    "ring_mcp_roll",
    "ring_mcp_pitch",
    "ring_pip",
    "ring_dip",
    "middle_mcp_roll",
    "middle_mcp_pitch",
    "middle_pip",
    "middle_dip",
    "index_mcp_roll",
    "index_mcp_pitch",
    "index_pip",
    "index_dip",
    "thumb_cmc_yaw",
    "thumb_cmc_roll",
    "thumb_cmc_pitch",
    "thumb_mcp",
    "thumb_dip",
};

struct Options
{
    std::string host = "127.0.0.1";
    uint16_t port = 5570;
    double rate_hz = 30.0;
    double stale_timeout_s = 0.25;
    double duration_s = 0.0;
    uint32_t discovery_s = 1;
    bool loopback_only = true;
    bool send = false;
    std::string calibration_file = MANUS_DEFAULT_CALIBRATION_FILE;
};

std::atomic<bool> g_running{true};
std::mutex g_frame_mutex;
std::unordered_set<uint32_t> g_right_glove_ids;
manus_teleop::ErgonomicHand g_latest{};
Clock::time_point g_received_at{};
uint64_t g_frame_sequence = 0;
bool g_has_frame = false;

void Stop(const int)
{
    g_running.store(false);
}

void OnLandscape(const Landscape* const landscape)
{
    if (landscape == nullptr)
    {
        return;
    }
    std::lock_guard<std::mutex> lock(g_frame_mutex);
    g_right_glove_ids.clear();
    for (uint32_t index = 0;
         index < landscape->gloveDevices.gloveCount;
         ++index)
    {
        const GloveLandscapeData& glove =
            landscape->gloveDevices.gloves[index];
        if (glove.side == Side_Right)
        {
            g_right_glove_ids.insert(glove.id);
        }
    }
}

void OnErgonomics(const ErgonomicsStream* const stream)
{
    if (stream == nullptr || !g_running.load())
    {
        return;
    }
    for (uint32_t index = 0; index < stream->dataCount; ++index)
    {
        const ErgonomicsData& data = stream->data[index];
        if (data.isUserID)
        {
            continue;
        }
        std::lock_guard<std::mutex> lock(g_frame_mutex);
        if (g_right_glove_ids.count(data.id) == 0)
        {
            continue;
        }
        for (std::size_t value = 0;
             value < manus_teleop::kErgonomicValues;
             ++value)
        {
            g_latest[value] = data.data[20 + value];
        }
        g_received_at = Clock::now();
        ++g_frame_sequence;
        g_has_frame = true;
    }
}

void Check(const char* const operation, const SDKReturnCode result)
{
    if (result != SDKReturnCode_Success)
    {
        throw std::runtime_error(
            std::string(operation) + " returned MANUS SDK code " +
            std::to_string(static_cast<int32_t>(result)));
    }
}

Options ParseOptions(const int argc, char** argv)
{
    Options options;
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument(argv[index]);
        auto next = [&]() -> std::string {
            if (++index >= argc)
            {
                throw std::invalid_argument(argument + " needs a value");
            }
            return argv[index];
        };
        if (argument == "--host")
        {
            options.host = next();
        }
        else if (argument == "--port")
        {
            const int value = std::stoi(next());
            if (value <= 0 || value > 65535)
            {
                throw std::invalid_argument("--port must be in [1, 65535]");
            }
            options.port = static_cast<uint16_t>(value);
        }
        else if (argument == "--rate")
        {
            options.rate_hz = std::stod(next());
        }
        else if (argument == "--stale-timeout")
        {
            options.stale_timeout_s = std::stod(next());
        }
        else if (argument == "--duration")
        {
            options.duration_s = std::stod(next());
        }
        else if (argument == "--discovery-seconds")
        {
            const int value = std::stoi(next());
            if (value < 0)
            {
                throw std::invalid_argument(
                    "--discovery-seconds must not be negative");
            }
            options.discovery_s = static_cast<uint32_t>(value);
        }
        else if (argument == "--network-discovery")
        {
            options.loopback_only = false;
        }
        else if (argument == "--calibration")
        {
            options.calibration_file = next();
        }
        else if (argument == "--no-calibration")
        {
            options.calibration_file.clear();
        }
        else if (argument == "--send")
        {
            options.send = true;
        }
        else if (argument == "--help")
        {
            std::cout
                << "Usage: manus_right_hand_teleop [options]\n"
                << "  --send                 send UDP hand commands (default: print only)\n"
                << "  --host HOST            hand bridge host (default: 127.0.0.1)\n"
                << "  --port PORT            hand bridge port (default: 5570)\n"
                << "  --rate HZ              output rate, <=60 (default: 30)\n"
                << "  --stale-timeout SEC    stop after no MANUS frame (default: 0.25)\n"
                << "  --duration SEC         stop automatically; 0 runs until Ctrl+C\n"
                << "  --discovery-seconds N  MANUS discovery wait (default: 1)\n"
                << "  --network-discovery    discover beyond localhost\n"
                << "  --calibration FILE     right .mcal override\n"
                << "  --no-calibration       use MANUS Core's current calibration\n";
            std::exit(0);
        }
        else
        {
            throw std::invalid_argument("unknown option: " + argument);
        }
    }
    if (!(options.rate_hz > 0.0 && options.rate_hz <= 60.0))
    {
        throw std::invalid_argument("--rate must be in (0, 60]");
    }
    if (!(options.stale_timeout_s > 0.0))
    {
        throw std::invalid_argument("--stale-timeout must be positive");
    }
    if (options.duration_s < 0.0)
    {
        throw std::invalid_argument("--duration must not be negative");
    }
    return options;
}

std::vector<unsigned char> ReadFile(const std::string& path)
{
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    if (!input)
    {
        throw std::runtime_error("could not open calibration file: " + path);
    }
    const std::streamsize size = input.tellg();
    if (size <= 0)
    {
        throw std::runtime_error("calibration file is empty: " + path);
    }
    std::vector<unsigned char> bytes(static_cast<std::size_t>(size));
    input.seekg(0);
    if (!input.read(
            reinterpret_cast<char*>(bytes.data()),
            static_cast<std::streamsize>(bytes.size())))
    {
        throw std::runtime_error("could not read calibration file: " + path);
    }
    return bytes;
}

class DatagramSocket
{
public:
    DatagramSocket(const std::string& host, const uint16_t port)
    {
        descriptor_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (descriptor_ < 0)
        {
            throw std::runtime_error(
                "could not create UDP socket: " + std::string(std::strerror(errno)));
        }
        address_.sin_family = AF_INET;
        address_.sin_port = htons(port);
        if (inet_pton(AF_INET, host.c_str(), &address_.sin_addr) != 1)
        {
            close(descriptor_);
            descriptor_ = -1;
            throw std::invalid_argument(
                "--host must be an IPv4 address, got " + host);
        }
    }

    ~DatagramSocket()
    {
        if (descriptor_ >= 0)
        {
            close(descriptor_);
        }
    }

    void Send(const std::string& payload) const
    {
        const ssize_t sent = sendto(
            descriptor_,
            payload.data(),
            payload.size(),
            0,
            reinterpret_cast<const sockaddr*>(&address_),
            sizeof(address_));
        if (sent != static_cast<ssize_t>(payload.size()))
        {
            throw std::runtime_error(
                "could not send hand datagram: " +
                std::string(std::strerror(errno)));
        }
    }

private:
    int descriptor_ = -1;
    sockaddr_in address_{};
};

double UnixSeconds()
{
    return std::chrono::duration<double>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

std::string Packet(
    const char* const stream,
    const uint64_t sequence,
    const char* const side,
    const manus_teleop::RobotPose& pose)
{
    std::ostringstream output;
    output << std::setprecision(10)
           << "{\"version\":1,\"stream_id\":\"" << stream
           << "\",\"sequence\":" << sequence
           << ",\"timestamp\":" << UnixSeconds()
           << ",\"side\":\"" << side << "\",\"joint_names\":[";
    for (std::size_t index = 0; index < kJointNames.size(); ++index)
    {
        if (index != 0)
        {
            output << ',';
        }
        output << '"' << kJointNames[index] << '"';
    }
    output << "],\"qpos\":[";
    for (std::size_t index = 0; index < pose.size(); ++index)
    {
        if (index != 0)
        {
            output << ',';
        }
        output << pose[index];
    }
    output << "]}";
    return output.str();
}

void PrintFrame(
    const uint64_t sequence,
    const manus_teleop::ErgonomicHand& values,
    const manus_teleop::RobotPose& pose)
{
    std::cout << "frame=" << sequence << " raw_deg=[";
    for (std::size_t index = 0; index < values.size(); ++index)
    {
        if (index != 0)
        {
            std::cout << ',';
        }
        std::cout << std::fixed << std::setprecision(1) << values[index];
    }
    std::cout << "] right_qpos=[";
    for (std::size_t index = 0; index < pose.size(); ++index)
    {
        if (index != 0)
        {
            std::cout << ',';
        }
        std::cout << std::fixed << std::setprecision(3) << pose[index];
    }
    std::cout << "]\n";
}
} // namespace

int main(int argc, char** argv)
{
    bool sdk_initialized = false;
    try
    {
        const Options options = ParseOptions(argc, argv);
        std::signal(SIGINT, Stop);
        std::signal(SIGTERM, Stop);

        Check("CoreSdk_InitializeIntegrated", CoreSdk_InitializeIntegrated());
        sdk_initialized = true;
        Check(
            "CoreSdk_RegisterCallbackForLandscapeStream",
            CoreSdk_RegisterCallbackForLandscapeStream(OnLandscape));
        Check(
            "CoreSdk_RegisterCallbackForErgonomicsStream",
            CoreSdk_RegisterCallbackForErgonomicsStream(OnErgonomics));

        CoordinateSystemVUH coordinate_system{};
        CoordinateSystemVUH_Init(&coordinate_system);
        coordinate_system.handedness = Side_Right;
        coordinate_system.up = AxisPolarity_PositiveZ;
        coordinate_system.view = AxisView_XFromViewer;
        coordinate_system.unitScale = 1.0F;
        Check(
            "CoreSdk_InitializeCoordinateSystemWithVUH",
            CoreSdk_InitializeCoordinateSystemWithVUH(
                coordinate_system, false));

        Check(
            "CoreSdk_LookForHosts",
            CoreSdk_LookForHosts(options.discovery_s, options.loopback_only));
        uint32_t host_count = 0;
        Check(
            "CoreSdk_GetNumberOfAvailableHostsFound",
            CoreSdk_GetNumberOfAvailableHostsFound(&host_count));
        if (host_count == 0)
        {
            throw std::runtime_error("MANUS SDK did not discover a host");
        }
        std::vector<ManusHost> hosts(host_count);
        for (ManusHost& host : hosts)
        {
            ManusHost_Init(&host);
        }
        Check(
            "CoreSdk_GetAvailableHostsFound",
            CoreSdk_GetAvailableHostsFound(hosts.data(), host_count));
        Check("CoreSdk_ConnectToHost", CoreSdk_ConnectToHost(hosts.front()));

        DatagramSocket datagram(options.host, options.port);
        std::cerr
            << "Powered by Manus. Connected to MANUS host "
            << hosts.front().hostName
            << "; right glove -> udp://" << options.host << ':' << options.port
            << (options.send ? " (SEND ENABLED)" : " (print-only dry run)")
            << '\n';

        const auto period =
            std::chrono::duration<double>(1.0 / options.rate_hz);
        const auto started = Clock::now();
        auto next = started;
        uint64_t output_sequence = 0;
        uint64_t last_printed_frame = 0;
        bool was_live = false;
        bool calibration_applied = options.calibration_file.empty();
        std::vector<unsigned char> calibration =
            calibration_applied
                ? std::vector<unsigned char>{}
                : ReadFile(options.calibration_file);
        while (g_running.load())
        {
            const auto now = Clock::now();
            if (options.duration_s > 0.0 &&
                std::chrono::duration<double>(now - started).count() >=
                    options.duration_s)
            {
                break;
            }

            manus_teleop::ErgonomicHand values{};
            Clock::time_point received_at{};
            uint64_t frame_sequence = 0;
            bool has_frame = false;
            uint32_t right_glove_id = 0;
            {
                std::lock_guard<std::mutex> lock(g_frame_mutex);
                values = g_latest;
                received_at = g_received_at;
                frame_sequence = g_frame_sequence;
                has_frame = g_has_frame;
                if (!g_right_glove_ids.empty())
                {
                    right_glove_id = *g_right_glove_ids.begin();
                }
            }
            if (!calibration_applied && right_glove_id != 0)
            {
                SetGloveCalibrationReturnCode calibration_result =
                    SetGloveCalibrationReturnCode_Error;
                Check(
                    "CoreSdk_SetGloveCalibration",
                    CoreSdk_SetGloveCalibration(
                        right_glove_id,
                        calibration.data(),
                        static_cast<uint32_t>(calibration.size()),
                        &calibration_result));
                if (calibration_result !=
                    SetGloveCalibrationReturnCode_Success)
                {
                    throw std::runtime_error(
                        "MANUS rejected right calibration with code " +
                        std::to_string(
                            static_cast<int32_t>(calibration_result)));
                }
                {
                    std::lock_guard<std::mutex> lock(g_frame_mutex);
                    g_has_frame = false;
                }
                has_frame = false;
                calibration_applied = true;
                std::cerr << "Applied right-glove calibration from "
                          << options.calibration_file << ".\n";
            }
            const bool live =
                calibration_applied && has_frame &&
                std::chrono::duration<double>(now - received_at).count() <=
                    options.stale_timeout_s;

            if (live)
            {
                const auto right = manus_teleop::RetargetRight(values);
                if (options.send)
                {
                    datagram.Send(Packet(
                        "manus-right",
                        output_sequence,
                        "right",
                        right));
                    datagram.Send(Packet(
                        "manus-left-default",
                        output_sequence,
                        "left",
                        manus_teleop::DefaultLeftPose()));
                    ++output_sequence;
                }
                else if (frame_sequence != last_printed_frame)
                {
                    PrintFrame(frame_sequence, values, right);
                    last_printed_frame = frame_sequence;
                }
            }
            if (live != was_live)
            {
                std::cerr << (live
                                  ? "Right MANUS stream live.\n"
                                  : "Right MANUS stream stale; output stopped.\n");
                was_live = live;
            }

            next += std::chrono::duration_cast<Clock::duration>(period);
            std::this_thread::sleep_until(next);
            if (Clock::now() - next > period)
            {
                next = Clock::now();
            }
        }

        std::cerr << "Stopping; the hand bridge watchdog will hold position.\n";
        Check("CoreSdk_ShutDown", CoreSdk_ShutDown());
        sdk_initialized = false;
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "error: " << error.what() << '\n';
        if (sdk_initialized)
        {
            CoreSdk_ShutDown();
        }
        return 2;
    }
}
