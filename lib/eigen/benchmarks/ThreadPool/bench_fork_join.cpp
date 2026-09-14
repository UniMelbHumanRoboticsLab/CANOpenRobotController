// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2026 Rasmus Munk Larsen <rmlarsen@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

// Benchmarks for ForkJoinScheduler::ParallelFor / ParallelForAsync.
//
// Four scenarios are covered:
//
//   BM_ParallelFor_Throughput      Dispatch overhead with cheap (single-add) tasks,
//                                  swept over (num_threads, num_tasks, granularity).
//   BM_ParallelFor_Granularity     Fixed total work, sweep granularity. Shows the
//                                  classic too-fine-vs-too-coarse trade-off.
//   BM_ParallelFor_HeavyCapture    Functor with a 2 KiB by-value capture, std::move'd
//                                  into ParallelFor. Exercises the entry-point
//                                  forwarding path (the one fix in MR !2556 that has
//                                  observable codegen).
//   BM_ParallelForAsync_Batch      Async API: schedule N concurrent ParallelForAsync
//                                  calls and wait on a single Barrier.

#define EIGEN_USE_THREADS

#include <benchmark/benchmark.h>

#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <utility>

#include "Eigen/ThreadPool"

using Eigen::Barrier;
using Eigen::ForkJoinScheduler;
using Eigen::Index;
using Eigen::ThreadPool;

namespace {

// One ThreadPool per (benchmark_name, num_threads) so we don't pay pool-startup
// cost inside the timed loop. Google Benchmark calls the benchmark function once
// per Args() row; the pool lives across all `for (auto _ : state)` iterations
// of that row and is destroyed when the function returns.
std::unique_ptr<ThreadPool> MakePool(int num_threads) { return std::make_unique<ThreadPool>(num_threads); }

}  // namespace

// ---------------------------------------------------------------------------
// 1. Dispatch throughput: cheap per-task body (a single atomic increment).
// Args: {num_threads, num_tasks, granularity}
// ---------------------------------------------------------------------------
static void BM_ParallelFor_Throughput(benchmark::State& state) {
  const int num_threads = static_cast<int>(state.range(0));
  const Index num_tasks = state.range(1);
  const Index granularity = state.range(2);
  auto pool = MakePool(num_threads);
  std::atomic<int64_t> counter{0};
  auto do_func = [&counter](Index i, Index j) { counter.fetch_add(j - i, std::memory_order_relaxed); };
  for (auto _ : state) {
    ForkJoinScheduler::ParallelFor(0, num_tasks, granularity, do_func, pool.get());
    benchmark::DoNotOptimize(counter);
  }
  state.SetItemsProcessed(state.iterations() * num_tasks);
}
BENCHMARK(BM_ParallelFor_Throughput)
    ->ArgNames({"threads", "tasks", "gran"})
    ->Args({1, 1024, 1})
    ->Args({4, 1024, 1})
    ->Args({8, 1024, 1})
    ->Args({4, 1024, 16})
    ->Args({4, 8192, 1})
    ->Args({4, 8192, 64})
    ->Args({8, 8192, 64})
    ->UseRealTime();

// ---------------------------------------------------------------------------
// 2. Granularity sweep: fixed 65536 trivial tasks across 8 threads, sweep
// granularity through the regime where too-fine == dispatch-bound and
// too-coarse == load-imbalance.
// Args: {granularity}
// ---------------------------------------------------------------------------
static void BM_ParallelFor_Granularity(benchmark::State& state) {
  constexpr int kNumThreads = 8;
  constexpr Index kNumTasks = 1 << 16;
  const Index granularity = state.range(0);
  auto pool = MakePool(kNumThreads);
  std::atomic<int64_t> counter{0};
  auto do_func = [&counter](Index i, Index j) { counter.fetch_add(j - i, std::memory_order_relaxed); };
  for (auto _ : state) {
    ForkJoinScheduler::ParallelFor(0, kNumTasks, granularity, do_func, pool.get());
    benchmark::DoNotOptimize(counter);
  }
  state.SetItemsProcessed(state.iterations() * kNumTasks);
}
BENCHMARK(BM_ParallelFor_Granularity)->ArgName("gran")->RangeMultiplier(4)->Range(1, 1 << 12)->UseRealTime();

// ---------------------------------------------------------------------------
// 3. Heavy capture: a do_func with a ~2 KiB by-value payload, std::move'd
// into ParallelFor on every iteration. The functor is copy/move-constructed
// exactly once per call (when ParallelForAsync captures it into the entry-
// point lambda). The per-task body is cheap so the copy/move is visible at
// the call rate.
// Args: {num_threads, num_tasks, granularity}
// ---------------------------------------------------------------------------
static void BM_ParallelFor_HeavyCapture(benchmark::State& state) {
  const int num_threads = static_cast<int>(state.range(0));
  const Index num_tasks = state.range(1);
  const Index granularity = state.range(2);
  auto pool = MakePool(num_threads);
  std::atomic<int64_t> counter{0};

  // ~2 KiB of by-value capture. The destination address of the data dictates
  // the per-task observable, so the captured array isn't dead-stripped.
  struct HeavyFn {
    std::array<double, 256> payload;
    std::atomic<int64_t>* counter;
    void operator()(Index i, Index j) const {
      // Use one element so the payload isn't optimized away; correctness of the
      // count is preserved by adding (j - i).
      counter->fetch_add(static_cast<int64_t>(j - i) + static_cast<int64_t>(payload[0] == 0.0),
                         std::memory_order_relaxed);
    }
  };
  HeavyFn proto;
  proto.payload.fill(1.0);
  proto.counter = &counter;

  for (auto _ : state) {
    // Fresh copy each iteration so std::move into ParallelFor is meaningful.
    HeavyFn fn = proto;
    ForkJoinScheduler::ParallelFor(0, num_tasks, granularity, std::move(fn), pool.get());
    benchmark::DoNotOptimize(counter);
  }
  state.SetItemsProcessed(state.iterations() * num_tasks);
  state.SetBytesProcessed(state.iterations() * static_cast<int64_t>(sizeof(HeavyFn)));
}
BENCHMARK(BM_ParallelFor_HeavyCapture)
    ->ArgNames({"threads", "tasks", "gran"})
    ->Args({4, 64, 1})
    ->Args({4, 1024, 16})
    ->Args({8, 1024, 16})
    ->Args({8, 8192, 64})
    ->UseRealTime();

// ---------------------------------------------------------------------------
// 4. Async batch: schedule kBatch concurrent ParallelForAsync calls and
// wait on a single Barrier. Mirrors the dominant usage pattern from
// TensorDeviceThreadPool.
// Args: {num_threads, batch, tasks_per_call, granularity}
// ---------------------------------------------------------------------------
static void BM_ParallelForAsync_Batch(benchmark::State& state) {
  const int num_threads = static_cast<int>(state.range(0));
  const int batch = static_cast<int>(state.range(1));
  const Index num_tasks = state.range(2);
  const Index granularity = state.range(3);
  auto pool = MakePool(num_threads);
  std::atomic<int64_t> counter{0};
  std::function<void(Index, Index)> do_func = [&counter](Index i, Index j) {
    counter.fetch_add(j - i, std::memory_order_relaxed);
  };
  for (auto _ : state) {
    Barrier barrier(batch);
    std::function<void()> done = [&barrier]() { barrier.Notify(); };
    for (int k = 0; k < batch; ++k) {
      ForkJoinScheduler::ParallelForAsync(0, num_tasks, granularity, do_func, done, pool.get());
    }
    barrier.Wait();
    benchmark::DoNotOptimize(counter);
  }
  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(batch) * num_tasks);
}
BENCHMARK(BM_ParallelForAsync_Batch)
    ->ArgNames({"threads", "batch", "tasks", "gran"})
    ->Args({4, 4, 1024, 16})
    ->Args({8, 8, 1024, 16})
    ->Args({8, 16, 1024, 16})
    ->Args({8, 8, 8192, 64})
    ->UseRealTime();
