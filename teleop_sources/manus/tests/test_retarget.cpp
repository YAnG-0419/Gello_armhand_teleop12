#include "manus_teleop/retarget.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace
{
void Check(const bool condition, const char* const expression)
{
    if (!condition)
    {
        std::cerr << "check failed: " << expression << '\n';
        std::exit(1);
    }
}
} // namespace

int main()
{
    manus_teleop::ErgonomicHand values{};
    values[4] = 5.0F;
    values[5] = 45.0F;
    values[6] = 90.0F;
    values[7] = 120.0F;
    values[16] = -20.0F;
    values[17] = 200.0F;
    values[1] = 30.0F;
    values[2] = 60.0F;
    values[3] = 90.0F;

    const auto pose = manus_teleop::RetargetRight(values);
    Check(
        std::abs(pose[12] - 5.0 * manus_teleop::kDegreesToRadians) < 1e-12,
        "index spread");
    Check(
        std::abs(pose[13] - 45.0 * manus_teleop::kDegreesToRadians) < 1e-12,
        "index MCP");
    Check(pose[14] == 1.57, "index PIP clipping");
    Check(pose[15] == 1.4, "index DIP clipping");
    Check(pose[0] == -0.17, "pinky spread clipping");
    Check(pose[1] == 1.4, "pinky MCP clipping");
    Check(pose[16] == 1.10, "thumb opposition yaw");
    Check(pose[17] == 0.52, "thumb opposition roll");
    Check(
        std::abs(pose[18] - 30.0 * manus_teleop::kDegreesToRadians) < 1e-12,
        "thumb CMC pitch");
    Check(
        std::abs(pose[19] - 60.0 * manus_teleop::kDegreesToRadians) < 1e-12,
        "thumb MCP");
    Check(pose[20] == 1.22, "thumb DIP clipping");

    values[5] = std::numeric_limits<float>::quiet_NaN();
    bool rejected = false;
    try
    {
        (void)manus_teleop::RetargetRight(values);
    }
    catch (const std::invalid_argument&)
    {
        rejected = true;
    }
    Check(rejected, "non-finite input rejected");

    for (const double value : manus_teleop::DefaultLeftPose())
    {
        Check(value == 0.0, "left default is zero");
    }
    return 0;
}
