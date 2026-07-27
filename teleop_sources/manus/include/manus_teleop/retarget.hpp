#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace manus_teleop
{
constexpr std::size_t kErgonomicValues = 20;
constexpr std::size_t kRobotJoints = 21;
constexpr double kDegreesToRadians = 0.017453292519943295;

using ErgonomicHand = std::array<float, kErgonomicValues>;
using RobotPose = std::array<double, kRobotJoints>;

inline double Clamp(const double value, const double lower, const double upper)
{
    return std::max(lower, std::min(upper, value));
}

inline double Flexion(const float degrees, const double upper)
{
    if (!std::isfinite(degrees))
    {
        throw std::invalid_argument("MANUS ergonomics contains a non-finite angle");
    }
    return Clamp(static_cast<double>(degrees) * kDegreesToRadians, 0.0, upper);
}

inline double Spread(const float degrees)
{
    if (!std::isfinite(degrees))
    {
        throw std::invalid_argument("MANUS ergonomics contains a non-finite angle");
    }
    return Clamp(static_cast<double>(degrees) * kDegreesToRadians, -0.17, 0.17);
}

// Convert the MANUS right-hand ergonomics order into the repository's existing
// 21-name L20 packet order. MANUS angles are degrees. The G20 bridge remains
// responsible for its established L20-to-G20 projection and output slew limit.
inline RobotPose RetargetRight(const ErgonomicHand& values)
{
    RobotPose output{};

    // MANUS finger blocks are thumb, index, middle, ring, pinky. Each block is
    // spread, MCP stretch, PIP stretch, DIP stretch. The L20 packet is ordered
    // pinky, ring, middle, index, then thumb.
    const std::array<std::size_t, 4> input_fingers = {16, 12, 8, 4};
    for (std::size_t finger = 0; finger < input_fingers.size(); ++finger)
    {
        const std::size_t input = input_fingers[finger];
        const std::size_t robot = finger * 4;
        output[robot] = Spread(values[input]);
        output[robot + 1] = Flexion(values[input + 1], 1.4);
        output[robot + 2] = Flexion(values[input + 2], 1.57);
        output[robot + 3] = Flexion(values[input + 3], 1.4);
    }

    // Keep the hardware-validated right-thumb opposition. MANUS curl drives the
    // remaining three flexion coordinates directly.
    output[16] = 1.10;  // thumb_cmc_yaw
    output[17] = 0.52;  // thumb_cmc_roll
    output[18] = Flexion(values[1], 0.79);
    output[19] = Flexion(values[2], 1.05);
    output[20] = Flexion(values[3], 1.22);
    return output;
}

inline RobotPose DefaultLeftPose()
{
    return RobotPose{};
}
} // namespace manus_teleop
