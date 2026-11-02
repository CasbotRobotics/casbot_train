#include "rl_controller/loco_observation.hpp"

#include <iomanip>
#include <iostream>

// Read a clock, command, phase-frame count, reset flag and four base frames.
// No ROS node or robot connection is created.
int main()
{
  double time, vx, vy, yaw;
  int phaseFrames, resetHistory;
  std::array<float, 180> history{};
  rl::loco::GaitPhaseHistory phaseHistory;
  std::cout << std::setprecision(9);
  while (std::cin >> time >> vx >> vy >> yaw >> phaseFrames >> resetHistory)
  {
    if (resetHistory) phaseHistory.reset();
    for (float &value : history)
      if (!(std::cin >> value)) return 1;
    const auto phase = rl::loco::gaitPhaseFeatures(time, vx, vy, yaw);
    const float *phaseData = nullptr;
    if (phaseFrames == 1) phaseData = phase.data();
    if (phaseFrames == 4) phaseData = phaseHistory.append(phase);
    std::vector<float> observation;
    rl::loco::packObservation(history.data(), 4, phaseData, phaseFrames,
                              observation);
    for (const float value : observation) std::cout << value << ' ';
    std::cout << '\n';
  }
}
