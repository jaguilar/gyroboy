#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define ring_size 4

#define int error

typedef struct jags_ring {
  int32_t data[ring_size];
  int32_t index;
} jags_ring;

void jags_ring_init(jags_ring *ring) {
  ring->index = 0;
  memset(ring->data, 0, ring_size * sizeof(int32_t));
}

void jags_ring_push(jags_ring *ring, int32_t value) {
  ring->data[ring->index++] = value;
  if (ring->index == ring_size) {
    ring->index = 0;
  }
}

int32_t jags_ring_diff(const jags_ring *ring) {
  int32_t prev_index = ring->index - 1;
  if (prev_index < 0) prev_index = ring_size - 1;
  // The most recently written value is in prev_index.
  return ring->data[prev_index] - ring->data[ring->index];
}

int32_t jags_ring_sum(const jags_ring *ring) {
  int32_t sum = 0;
  for (int32_t i = 0; i < ring_size; i++) {
    sum += ring->data[i];
  }
  return sum;
}

typedef struct jags_ratio {
  int32_t num;
  int32_t log2_denom;
} jags_ratio;

double jags_calculate_relative_error(double target, int32_t num,
                                     int32_t log2_denom) {
  const double found = 1.0 * num / (1 << log2_denom);
  const double err = fabs(found - target) / target;
  printf("%f %d/%d %f\n", target, num, 1 << log2_denom, err);
  return err;
}

// Initialize the ratio with the closest numerator and log2_denom to the target
// value. Asserts that the ratio is with the tolerance provided. (This isn't
// very good as a general approach but it will be okay for this.)
bool jags_ratio_init(jags_ratio *ratio, double target, int32_t max_input,
                     double tolerance) {
  printf("%f %d %f\n", target, max_input, tolerance);
  const double quit_early = tolerance / 10;
  ratio->log2_denom = 0;
  ratio->num = 0;
  if (target == 0.0) {
    return true;
  }

  const int32_t max_num_times_input = INT32_MAX;
  int32_t num = (int32_t)target;
  double bestrelerr = 1.0;
  for (int32_t log2_denom = 0; log2_denom < 32; ++log2_denom) {
    const int32_t denom = 1 << log2_denom;
    num = round(target * denom);
    if (1.0 * num * max_input > max_num_times_input) {
      break;
    }
    const double relerr =
        jags_calculate_relative_error(target, num, log2_denom);
    if (relerr < bestrelerr) {
      ratio->num = num;
      ratio->log2_denom = log2_denom;
      bestrelerr = relerr;
    }
    if (relerr < quit_early) {
      break;
    }
  }
  if (bestrelerr > tolerance) {
    printf(
        "jags_ratio_init: best rel err %f > tolerance %f (target:%f "
        "max_input:%d)\n",
        bestrelerr, tolerance, target, max_input);
    assert(0);
    return false;
  }
  printf("found ratio %f %d/%d\n", target, ratio->num, 1 << ratio->log2_denom);
  return true;
}

int32_t jags_ratio_apply(const jags_ratio *ratio, int32_t input) {
  return (input * ratio->num) >> ratio->log2_denom;
}

typedef struct jags_pid {
  jags_ratio kp_r, ki_r, kd_r;
  int32_t min_out, max_out;

  // Integrator and differentiator state.
  int32_t err_sum;
  jags_ring derr_samps, dt_samps;

  FILE *logfile;
  int32_t logfreq, nextlog, time;
} jags_pid;

extern void *jags_pid_new(double gain, double integration_time,
                          double derivative_time, int32_t min_output,
                          int32_t max_output, int32_t max_expected_err,
                          const char *logfile_name, int32_t max_dt,
                          double max_ratio_error, int32_t logfreq) {
  jags_pid *pid = (jags_pid *)malloc(sizeof(jags_pid));
  pid->logfile = 0;

  if (!jags_ratio_init(&pid->kp_r, gain, max_expected_err, max_ratio_error)) {
    goto err;
  }
  if (integration_time > 0) {
    if (!jags_ratio_init(&pid->ki_r, gain / integration_time, max_expected_err,
                         max_ratio_error)) {
      goto err;
    }
  } else {
    pid->ki_r.num = 0;
  }
  if (derivative_time > 0) {
    if (!jags_ratio_init(&pid->kd_r, gain * derivative_time,
                         2 * max_expected_err, max_ratio_error)) {
      goto err;
    }
  } else {
    pid->kd_r.num = 0;
  }

  pid->min_out = min_output;
  pid->max_out = max_output;
  jags_ring_init(&pid->derr_samps);
  jags_ring_init(&pid->dt_samps);

  pid->logfile = 0;
  if (logfile_name && strlen(logfile_name) > 0) {
    pid->logfile = fopen(logfile_name, "w+");
    if (!pid->logfile) {
      printf("jags_pid_new: logfile (%s) open err: %d", logfile_name, errno);
      goto err;
    }
    fprintf(pid->logfile, "t,setpoint,observation,p,i,d,output\n");
    pid->time = pid->nextlog = 0;
    pid->logfreq = logfreq;
  }

  // Print all variables in pid on one line.
  printf(
      "pid: kp_r(%f, %d/%d), ki_r(%f, %d/%d), kd_r(%f, %d/%d), min_out(%d), "
      "max_out(%d), err_sum(%d), logfreq(%d), nextlog(%d), time(%d)\n",
      gain, pid->kp_r.num, 1 << pid->kp_r.log2_denom, gain / integration_time,
      pid->ki_r.num, 1 << pid->ki_r.log2_denom, gain * derivative_time,
      pid->kd_r.num, 1 << pid->kd_r.log2_denom, pid->min_out, pid->max_out,
      pid->err_sum, pid->logfreq, pid->nextlog, pid->time);

  return pid;

err:
  if (pid->logfile) fclose(pid->logfile);
  free(pid);
  return 0;
}

extern int32_t jags_pid_update(void *pid_void, int32_t dt, int32_t setpoint,
                               int32_t observation) {
  jags_pid *pid = (jags_pid *)pid_void;
  const int32_t error = setpoint - observation;

  const int32_t p = jags_ratio_apply(&pid->kp_r, error);

  int32_t i = 0;
  int32_t errsum_change = 0;
  if (dt > 0 && pid->ki_r.num != 0) {
    errsum_change = error * dt;
    pid->err_sum += errsum_change;
    i = jags_ratio_apply(&pid->ki_r, pid->err_sum);
  }

  int32_t d = 0;
  if (dt > 0) {
    const int32_t derr_diff = jags_ring_diff(&pid->derr_samps);
    const int32_t dt_sum = jags_ring_sum(&pid->dt_samps);
    d = jags_ratio_apply(&pid->kd_r, derr_diff) / dt_sum;
  }

  int32_t output = p + i + d;

  if (output < pid->min_out) {
    output = pid->min_out;
    pid->err_sum -= errsum_change;  // Anti-windup.
  } else if (output > pid->max_out) {
    output = pid->max_out;
    pid->err_sum -= errsum_change;  // Anti-windup.
  }

  if (pid->logfile) {
    pid->time += dt;
    if (pid->time > pid->nextlog) {
      pid->nextlog += pid->logfreq;
      fprintf(pid->logfile, "%d,%d,%d,%d,%d,%d,%d\n", pid->time, setpoint,
              observation, p, i, d, output);
    }
  }

  return output;
}

extern void jags_pid_free(void *pid_void) {
  jags_pid *pid = pid_void;
  if (pid->logfile) {
    fclose(pid->logfile);
  }
  free(pid);
}
