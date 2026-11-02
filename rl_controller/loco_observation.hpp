#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace rl::loco
{
  constexpr int kFrameSize = 45;
  constexpr int kPhaseSize = 4;
  constexpr int kHistorySize = 4;

  // sim2sim gait_phase_features(): one-second clock, left/right offset 0/0.5.
  // A stopped command zeros the phases, so features are [0, 1, 0, 1].
  inline std::array<float, kPhaseSize> gaitPhaseFeatures(
      double policyTimeSec, double vx, double vy, double yaw)
  {
    constexpr double twoPi = 6.28318530717958647692;
    const bool stopped = std::hypot(vx, vy) + std::abs(yaw) <= 1e-6;
    const double left = stopped ? 0.0 : std::fmod(policyTimeSec, 1.0);
    const double right = stopped ? 0.0 : std::fmod(policyTimeSec + 0.5, 1.0);
    return {static_cast<float>(std::sin(twoPi * left)),
            static_cast<float>(std::cos(twoPi * left)),
            static_cast<float>(std::sin(twoPi * right)),
            static_cast<float>(std::cos(twoPi * right))};
  }

  // sim2sim phase_history: fill all four frames on first use, then slide.
  class GaitPhaseHistory
  {
  public:
    void reset() { initialized_ = false; }

    const float *append(const std::array<float, kPhaseSize> &phase)
    {
      if (!initialized_)
      {
        for (int frame = 0; frame < kHistorySize; ++frame)
          std::copy(phase.begin(), phase.end(), values_.begin() + frame * kPhaseSize);
        initialized_ = true;
      }
      else
      {
        std::move(values_.begin() + kPhaseSize, values_.end(), values_.begin());
        std::copy(phase.begin(), phase.end(), values_.end() - kPhaseSize);
      }
      return values_.data();
    }

  private:
    std::array<float, kPhaseSize * kHistorySize> values_{};
    bool initialized_{false};
  };

  // Oldest -> newest within each term. Append 0, 1 or 4 phase frames;
  // each phase frame is [sin(left), cos(left), sin(right), cos(right)].
  inline void packObservation(const float *history, int historySize,
                              const float *phase, int phaseFrames,
                              std::vector<float> &observation)
  {
    if (historySize <= 0)
      throw std::invalid_argument("Observation history_size must be positive");
    if (phaseFrames != 0 && phaseFrames != 1 && phaseFrames != historySize)
      throw std::invalid_argument("Phase must have zero, one or history_size frames");
    if (phaseFrames > 0 && !phase)
      throw std::invalid_argument("Missing gait phase features");
    constexpr std::array<int, 6> dims{3, 3, 3, 12, 12, 12};
    observation.resize(kFrameSize * historySize + kPhaseSize * phaseFrames);
    int frameOffset = 0;
    int outputOffset = 0;
    for (const int dim : dims)
    {
      for (int frame = 0; frame < historySize; ++frame)
      {
        std::copy_n(history + frame * kFrameSize + frameOffset, dim,
                    observation.begin() + outputOffset);
        outputOffset += dim;
      }
      frameOffset += dim;
    }
    if (phaseFrames > 0)
      std::copy_n(phase, kPhaseSize * phaseFrames, observation.begin() + outputOffset);
  }
}
