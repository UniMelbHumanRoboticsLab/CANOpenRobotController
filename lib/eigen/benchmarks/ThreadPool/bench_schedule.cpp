// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#define EIGEN_USE_THREADS
#include <benchmark/benchmark.h>
#include <Eigen/ThreadPool>

static void BM_ScheduleBatch(benchmark::State& state) {
  const int threads = static_cast<int>(state.range(0));
  const unsigned tasks = static_cast<unsigned>(state.range(1));
  Eigen::ThreadPool pool(threads, state.range(2) != 0);
  std::atomic<int64_t> completed{0};
  for (auto _ : state) {
    Eigen::Barrier done(tasks);
    for (unsigned i = 0; i < tasks; ++i) {
      pool.Schedule([&] {
        completed.fetch_add(1, std::memory_order_relaxed);
        done.Notify();
      });
    }
    done.Wait();
  }
  if (completed.load() != state.iterations() * tasks) state.SkipWithError("A scheduled task did not complete");
  state.SetItemsProcessed(state.iterations() * tasks);
}
BENCHMARK(BM_ScheduleBatch)
    ->ArgNames({"threads", "tasks", "spinning"})
    ->ArgsProduct({{1, 4, 16}, {1, 32, 256}, {0, 1}})
    ->UseRealTime();
