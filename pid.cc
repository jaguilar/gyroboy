#include <cmath>
#include <cstdint>
#include <utility>

namespace jags {

std::pair<int32_t, int32_t> make_log2_ratio(float f, float input_magnitude,
                                            float goodrelerr = 0.001) {
  if (f == 0.0) {
    return {0, 0};
  }

  const int32_t max_num_times_input = 2147483647;  // 2**32 - 1
  int32_t bestnum = static_cast<int32_t>(f);
  int32_t bestdenom = 1;
  float bestseen = std::fabs(f - (bestnum / bestdenom)) / f;
  for (int32_t log2_denom = 0; log2_denom < 32; ++log2_denom) {
    int32_t denom = 1 << log2_denom;
    int32_t num = std::round(f * denom);
    if (num * input_magnitude > max_num_times_input) {
      break;
    }
    float relerr = std::fabs(f - (num / denom)) / f;
    if (relerr < goodrelerr) {
      return {num, log2_denom};
    } else if (relerr < bestseen) {
      bestseen = relerr;
      bestnum = num;
      bestdenom = denom;
    }
  }
}

#if 0
    def make_log2_ratio(f: float, input_magnitude: float, goodrelerr=0.001):
#We can store values up to 2 ^ 31 - 1(leaving a bit for the sign).
    def error(f, num, denom):
        return math.fabs(f - (num / denom)) / f

    if f == 0.0:
        return 0, 0
    max_num_times_input = const(2147483647)  # 2**32 - 1
    bestnum, bestdenom = int(f), 1
    bestseen = error(f, bestnum, bestdenom)
    for log2_denom in range(32):
        denom = 1 << log2_denom
        num = round(f * denom)
        if num * input_magnitude > max_num_times_input:
            break
        relerr = math.fabs(f - (num / denom)) / f
        if relerr < goodrelerr:
            return num, log2_denom
        elif relerr < bestseen:
            bestseen = relerr
            bestnum = num
            bestdenom = denom
    raise Exception(
        "No good int scaler for {0} {1} {2} best seen ratio {3} {4}/{5}".format(
            f, input_magnitude, goodrelerr, bestseen, bestnum, bestdenom
        )
    )
#endif

class PID {
 public:
  int32_t Update(int32_t observation, int32_t setpoint) {}
};

}  // namespace jags

using jags::PID;

extern "C" {

void* jags_pid_new() { return new PID(); }

int32_t jags_pid_update(void* pid, int32_t setpoint, int32_t observation) {
  return static_cast<PID*>(pid)->Update(setpoint, observation);
}

void jags_pid_free(void* pid) { delete static_cast<PID*>(pid); }

}  // extern "C"