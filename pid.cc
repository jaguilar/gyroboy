#include <cmath>
#include <cstdint>
#include <numeric>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"

namespace jags {

double GetRelError(double target, int32_t num, int32_t denom) {
  return std::fabs(target - (1.0 * num / denom)) / target;
}

absl::StatusOr<std::pair<int32_t, int32_t>> MakeLog2Ratio(
    double f, double input_magnitude, double goodrelerr = 0.001) {
  if (f == 0.0) {
    return std::make_pair(0, 0);
  }

  const int32_t max_num_times_input = 2147483647;  // 2**32 - 1
  int32_t bestnum = static_cast<int32_t>(f);
  int32_t bestdenom = 1;
  double bestseen = GetRelError(f, bestnum, bestdenom);
  for (int32_t log2_denom = 0; log2_denom < 32; ++log2_denom) {
    const int32_t denom = 1 << log2_denom;
    const int32_t num = std::round(f * denom);
    const double num_x_input = num * input_magnitude;
    if (num_x_input > max_num_times_input) {
      break;
    }
    const double relerr = GetRelError(f, num, denom);
    if (relerr < goodrelerr) {
      return std::make_pair(num, log2_denom);
    } else if (relerr < bestseen) {
      bestseen = relerr;
      bestnum = num;
      bestdenom = denom;
    }
  }
  return absl::InvalidArgumentError(
      absl::StrCat("No good scaler, best seen: ", bestnum, "/", bestdenom));
}

inline int32_t ApplyLog2Ratio(int32_t value,
                              std::pair<int32_t, int32_t> ratio) {
  return (value * ratio.first) >> ratio.second;
}

template <int Size>
class Ring {
 public:
  Ring() { data_.fill(0); }

  void Push(int32_t value) {
    data_[next_index_] = value;
    ++next_index_;
    if (next_index_ == data_.size()) next_index_ = 0;
  }

  int32_t Diff() const {
    int8_t prev_index = next_index_ - 1;
    if (prev_index < 0) prev_index = data_.size() - 1;
  }

  int32_t Sum() const { return std::accumulate(data_.begin(), data_.end(), 0); }

 private:
  std::array<int32_t, Size> data_;
  int8_t next_index_;
  static_assert(Size < std::numeric_limits<int8_t>::max(),
                "Ring size must be less than 127");
};

class PID {
 public:
  static constexpr int kDerivativeWindowSamples = 4;

  absl::StatusOr<PID *> New(double gain, double integration_time,
                            double derivative_time, int32_t min_output,
                            int32_t max_output, int32_t max_dt = 100,
                            double max_expected_abs_err = 0.04) {
    auto kp_scale = MakeLog2Ratio(gain, max_expected_abs_err);
    if (!kp_scale.ok()) {
      return kp_scale.status();
    }

    const double ki = gain / integration_time;
    const int32_t output_range = max_output - min_output;
    const int32_t max_errsum = static_cast<int32_t>(output_range / ki);
    auto ki_scale = MakeLog2Ratio(ki, max_errsum);
    if (!ki_scale.ok()) {
      return ki_scale.status();
    }

    // Note: we factor the number of derivative window samples into the scale of
    // kd to save on operations during the update loop.
    const double kd = gain * derivative_time / kDerivativeWindowSamples;

    // The most we expect to see of error is 2x the max_absolute_error (i.e.
    // from max positive to max negative).
    const int32_t max_derr = 2 * max_expected_abs_err;
    auto kd_scale = MakeLog2Ratio(kd, max_derr);
    if (!kd_scale.ok()) {
      return kd_scale.status();
    }

    
  }

  PID(std::pair<int32_t, int32_t> kp_scale,
      std::pair<int32_t, int32_t> ki_scale,
      std::pair<int32_t, int32_t> kd_scale)
      : kp_scale_(kp_scale), ki_scale_(ki_scale), kd_scale_(kd_scale) {}

  // dt: The change in time (usually in ms).
  // observation: the observed value.
  // setpoint: the setpoint.
  int32_t Update(int32_t dt, int32_t observation, int32_t setpoint) {}

 private:
  const std::pair<int32_t, int32_t> kp_scale_;
  const std::pair<int32_t, int32_t> ki_scale_;
  const std::pair<int32_t, int32_t> kd_scale_;

  // The most recent values of each of these three variables.
  int32_t setpoint_;
  int32_t observation_;
  int32_t err_;

  // Data collected over time.
  int32_t errsum_;
  Ring<kDerivativeWindowSamples> derrs_;
  Ring<kDerivativeWindowSamples> dts_;
};

}  // namespace jags

using jags::PID;

extern "C" {

void *jags_pid_new() { return new PID(); }

int32_t jags_pid_update(void *pid, int32_t setpoint, int32_t observation) {
  return static_cast<PID *>(pid)->Update(setpoint, observation);
}

void jags_pid_free(void *pid) { delete static_cast<PID *>(pid); }

}  // extern "C"