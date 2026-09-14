// Benchmarks for the masked partial-packet ("segment") tail of vectorized
// assignments -- the path gated by internal::has_packet_segment<Packet>.
// See libeigen/eigen#3083 (the regression) and #3086 (segment-tail placement).
//
// Part A measures end-to-end assignments whose length is not a multiple of
// the packet size, so the tail loop is on the hot path:
//   * fixed-size  -> LinearVectorizedTraversal / CompleteUnrolling
//                    (compile-time tail count; the #3083 regression shape)
//   * dynamic     -> LinearVectorizedTraversal / NoUnrolling
//                    (runtime tail count)
// Part A's destinations are not read back, so it does NOT exercise the
// store-to-load forwarding hazard -- see Part C.
//
// Part B is a direct A/B of the segment primitives (ploaduSegment /
// pstoreuSegment) against an equivalent scalar loop, swept over the tail
// length. This isolates the masked load/store hardware cost (e.g. AVX2
// vmaskmov) and the store-to-load forwarding stall, and is the data that
// drives the per-ISA has_packet_segment decision.
//
// Part C runs the #3083 regression kernels: chained small fixed-size matrix
// ops where each result is consumed by the next. A partial-packet tail store
// then collides with the consumer's packet load -- the store-to-load
// forwarding hazard that makes a masked tail far more costly than a scalar
// one. This is the cost Parts A and B cannot see in isolation. Chained/Rotation
// additionally covers the lazy-product shape, whose tail is reached through
// SliceVectorizedTraversal with InnerUnrolling and through the product
// evaluator's own partial loads, not through LinearVectorizedTraversal.
//
// Part D takes the same traversal as Chained/Rotation but streams over a runtime
// outer size with nothing reloading the destination, so the segment amortizes a
// source evaluation instead of stalling a consumer. It is the shape that decides
// how narrowly the scalar tail may be applied.
//
// The active packet size and the value of has_packet_segment per scalar type
// are emitted as Google Benchmark custom context, so a captured run is
// self-describing. To sweep ISAs, build once per target, e.g.:
//   g++ -O3 -DNDEBUG -std=c++17 -msse4.2      bench_segment_tail.cpp ...
//   g++ -O3 -DNDEBUG -std=c++17 -mavx         bench_segment_tail.cpp ...
//   g++ -O3 -DNDEBUG -std=c++17 -mavx2 -mfma  bench_segment_tail.cpp ...
//   g++ -O3 -DNDEBUG -std=c++17 -march=native bench_segment_tail.cpp ...
// or, via CMake, configure benchmarks/ with -DCMAKE_CXX_FLAGS=<isa flags>.
//
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#include <benchmark/benchmark.h>

#include <complex>
#include <cstdint>
#include <string>
#include <tuple>

#include <Eigen/Core>
#include <Eigen/LU>

using namespace Eigen;

namespace {

// Capacity of the Part B scratch buffers (Buffers<T>). Must be >= the largest
// packet size of any benchmarked scalar type, since the primitives load and
// store a full packet; the static_assert in Buffers enforces it.
constexpr int kMaxCount = 16;

// ---------------------------------------------------------------------------
// Records, per scalar type, the packet size in use and the value of
// has_packet_segment for the active ISA. Emitted as Google Benchmark custom
// context so it appears once in the report header and in any --benchmark_out
// JSON, making the captured baseline self-describing.
// ---------------------------------------------------------------------------
template <typename T>
void add_trait_context(const char* name) {
  using Packet = typename internal::packet_traits<T>::type;
  constexpr int PacketSize = internal::unpacket_traits<Packet>::size;
  constexpr bool HasSegment = internal::has_packet_segment<Packet>::value;
  benchmark::AddCustomContext(
      std::string("segment.") + name,
      "PacketSize=" + std::to_string(PacketSize) + " has_packet_segment=" + (HasSegment ? "true" : "false"));
}

// ===========================================================================
// Part A : assignment-level tail benchmarks
// ===========================================================================

// Fixed-size column vector: dst = a + b, with N a compile-time constant.
// When N % PacketSize != 0 the completely-unrolled assignment ends in a
// compile-time-length segment tail.
template <typename T, int N>
void BM_FixedAssign(benchmark::State& state) {
  using Vec = Matrix<T, N, 1>;
  Vec a = Vec::Random();
  Vec b = Vec::Random();
  Vec dst = Vec::Random();
  for (auto _ : state) {
    benchmark::DoNotOptimize(a.data());
    benchmark::DoNotOptimize(b.data());
    dst = a + b;
    benchmark::DoNotOptimize(dst.data());
    benchmark::ClobberMemory();
  }
  state.SetBytesProcessed(state.iterations() * static_cast<int64_t>(N) * sizeof(T) * 3);
}

// Dynamic-size column vector: dst = a + b, with a runtime length that is not a
// multiple of the packet size, so the NoUnrolling tail loop runs every call.
template <typename T>
void BM_DynamicAssign(benchmark::State& state) {
  const Index size = state.range(0);
  Matrix<T, Dynamic, 1> a = Matrix<T, Dynamic, 1>::Random(size);
  Matrix<T, Dynamic, 1> b = Matrix<T, Dynamic, 1>::Random(size);
  Matrix<T, Dynamic, 1> dst = Matrix<T, Dynamic, 1>::Random(size);
  for (auto _ : state) {
    benchmark::DoNotOptimize(a.data());
    benchmark::DoNotOptimize(b.data());
    dst = a + b;
    benchmark::DoNotOptimize(dst.data());
    benchmark::ClobberMemory();
  }
  state.SetBytesProcessed(state.iterations() * static_cast<int64_t>(size) * sizeof(T) * 3);
}

// ===========================================================================
// Part B : segment-primitive A/B micro-benchmarks
//
// Each pair (segment vs scalar) is registered over the same count range so the
// reports line up. count is taken from state.range(0): runtime for the
// non-CT variants (matches the dynamic tail), compile-time for the *_CT
// variants (matches the completely-unrolled tail).
// ===========================================================================

template <typename T>
struct Buffers {
  static_assert(kMaxCount >= internal::unpacket_traits<typename internal::packet_traits<T>::type>::size,
                "kMaxCount is smaller than this type's packet size: the scratch buffers cannot hold a full packet");
  EIGEN_ALIGN_MAX T src[kMaxCount];
  EIGEN_ALIGN_MAX T dst[kMaxCount];
  Buffers() {
    for (int i = 0; i < kMaxCount; ++i) {
      src[i] = T(i + 1);
      dst[i] = T(0);
    }
  }
};

// --- runtime-count store: masked segment store vs scalar store loop ---------
template <typename T>
void BM_PrimSegmentStore(benchmark::State& state) {
  using Packet = typename internal::packet_traits<T>::type;
  constexpr int PacketSize = internal::unpacket_traits<Packet>::size;
  const Index count = state.range(0);
  if (PacketSize <= 1 || count > PacketSize) {
    state.SkipWithError("count exceeds packet size");
    return;
  }
  Buffers<T> buf;
  Packet p = internal::ploadu<Packet>(buf.src);
  for (auto _ : state) {
    benchmark::DoNotOptimize(p);
    internal::pstoreuSegment<T, Packet>(buf.dst, p, 0, count);
    benchmark::DoNotOptimize(buf.dst);
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations() * count);
}

template <typename T>
void BM_PrimScalarStore(benchmark::State& state) {
  using Packet = typename internal::packet_traits<T>::type;
  constexpr int PacketSize = internal::unpacket_traits<Packet>::size;
  const Index count = state.range(0);
  if (PacketSize <= 1 || count > PacketSize) {
    state.SkipWithError("count exceeds packet size");
    return;
  }
  Buffers<T> buf;
  for (auto _ : state) {
    benchmark::DoNotOptimize(buf.src);
    for (Index k = 0; k < count; ++k) buf.dst[k] = buf.src[k];
    benchmark::DoNotOptimize(buf.dst);
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations() * count);
}

// --- runtime-count store+reload: captures store-to-load forwarding stalls ---
template <typename T>
void BM_PrimSegmentStoreReload(benchmark::State& state) {
  using Packet = typename internal::packet_traits<T>::type;
  constexpr int PacketSize = internal::unpacket_traits<Packet>::size;
  const Index count = state.range(0);
  if (PacketSize <= 1 || count > PacketSize) {
    state.SkipWithError("count exceeds packet size");
    return;
  }
  Buffers<T> buf;
  Packet p = internal::ploadu<Packet>(buf.src);
  Packet acc = internal::pset1<Packet>(T(0));
  for (auto _ : state) {
    benchmark::DoNotOptimize(p);
    internal::pstoreuSegment<T, Packet>(buf.dst, p, 0, count);
    benchmark::DoNotOptimize(buf.dst);
    Packet q = internal::ploaduSegment<Packet>(buf.dst, 0, count);
    acc = internal::padd(acc, q);
    benchmark::DoNotOptimize(acc);
  }
  state.SetItemsProcessed(state.iterations() * count);
}

template <typename T>
void BM_PrimScalarStoreReload(benchmark::State& state) {
  using Packet = typename internal::packet_traits<T>::type;
  constexpr int PacketSize = internal::unpacket_traits<Packet>::size;
  const Index count = state.range(0);
  if (PacketSize <= 1 || count > PacketSize) {
    state.SkipWithError("count exceeds packet size");
    return;
  }
  Buffers<T> buf;
  for (auto _ : state) {
    benchmark::DoNotOptimize(buf.src);
    for (Index k = 0; k < count; ++k) buf.dst[k] = buf.src[k];
    benchmark::DoNotOptimize(buf.dst);
    // acc is reset every iteration: a persistent accumulator would grow without
    // bound over the benchmark's iterations -- signed-integer overflow (UB) for
    // integer T. Reset keeps the reload + add live without that.
    T acc = T(0);
    for (Index k = 0; k < count; ++k) acc += buf.dst[k];
    benchmark::DoNotOptimize(acc);
  }
  state.SetItemsProcessed(state.iterations() * count);
}

// --- overlapping full-width store+reload: one unmasked packet, count-free ---
// Models the "store a full packet ending at size" tail strategy: valid for a
// plain assignment when size >= PacketSize. Mask-free, so it is available on
// every ISA (including SSE/NEON), and a full-width store forwards cleanly to a
// same-width reload -- unlike a masked store. Cost is independent of the tail
// length, so this is registered once per type rather than swept over count.
template <typename T>
void BM_PrimOverlapStoreReload(benchmark::State& state) {
  using Packet = typename internal::packet_traits<T>::type;
  constexpr int PacketSize = internal::unpacket_traits<Packet>::size;
  if (PacketSize <= 1) {
    state.SkipWithError("type not vectorized");
    return;
  }
  Buffers<T> buf;
  Packet p = internal::ploadu<Packet>(buf.src);
  Packet acc = internal::pset1<Packet>(T(0));
  for (auto _ : state) {
    benchmark::DoNotOptimize(p);
    internal::pstoreu<T, Packet>(buf.dst, p);
    benchmark::DoNotOptimize(buf.dst);
    Packet q = internal::ploadu<Packet>(buf.dst);
    acc = internal::padd(acc, q);
    benchmark::DoNotOptimize(acc);
  }
  state.SetItemsProcessed(state.iterations() * PacketSize);
}

// --- runtime-count load: masked segment load vs scalar load loop -----------
template <typename T>
void BM_PrimSegmentLoad(benchmark::State& state) {
  using Packet = typename internal::packet_traits<T>::type;
  constexpr int PacketSize = internal::unpacket_traits<Packet>::size;
  const Index count = state.range(0);
  if (PacketSize <= 1 || count > PacketSize) {
    state.SkipWithError("count exceeds packet size");
    return;
  }
  Buffers<T> buf;
  Packet acc = internal::pset1<Packet>(T(0));
  for (auto _ : state) {
    benchmark::DoNotOptimize(buf.src);
    Packet q = internal::ploaduSegment<Packet>(buf.src, 0, count);
    acc = internal::padd(acc, q);
    benchmark::DoNotOptimize(acc);
  }
  state.SetItemsProcessed(state.iterations() * count);
}

template <typename T>
void BM_PrimScalarLoad(benchmark::State& state) {
  using Packet = typename internal::packet_traits<T>::type;
  constexpr int PacketSize = internal::unpacket_traits<Packet>::size;
  const Index count = state.range(0);
  if (PacketSize <= 1 || count > PacketSize) {
    state.SkipWithError("count exceeds packet size");
    return;
  }
  Buffers<T> buf;
  for (auto _ : state) {
    benchmark::DoNotOptimize(buf.src);
    // acc is reset every iteration -- see BM_PrimScalarStoreReload.
    T acc = T(0);
    for (Index k = 0; k < count; ++k) acc += buf.src[k];
    benchmark::DoNotOptimize(acc);
  }
  state.SetItemsProcessed(state.iterations() * count);
}

// --- compile-time-count store: matches the completely-unrolled tail --------
// Here the scalar alternative is itself fully unrolled (no loop), which is why
// it can beat a masked store for tiny tails.
template <typename T, int Count>
void BM_PrimSegmentStoreCT(benchmark::State& state) {
  using Packet = typename internal::packet_traits<T>::type;
  constexpr int PacketSize = internal::unpacket_traits<Packet>::size;
  if (PacketSize <= 1 || Count > PacketSize) {
    state.SkipWithError("count exceeds packet size");
    return;
  }
  Buffers<T> buf;
  Packet p = internal::ploadu<Packet>(buf.src);
  for (auto _ : state) {
    benchmark::DoNotOptimize(p);
    internal::pstoreuSegment<T, Packet>(buf.dst, p, 0, Count);
    benchmark::DoNotOptimize(buf.dst);
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations() * Count);
}

template <typename T, int Count>
void BM_PrimScalarStoreCT(benchmark::State& state) {
  using Packet = typename internal::packet_traits<T>::type;
  constexpr int PacketSize = internal::unpacket_traits<Packet>::size;
  if (PacketSize <= 1 || Count > PacketSize) {
    state.SkipWithError("count exceeds packet size");
    return;
  }
  Buffers<T> buf;
  for (auto _ : state) {
    benchmark::DoNotOptimize(buf.src);
    EIGEN_UNROLL_LOOP
    for (int k = 0; k < Count; ++k) buf.dst[k] = buf.src[k];
    benchmark::DoNotOptimize(buf.dst);
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(state.iterations() * Count);
}

// ===========================================================================
// Part C : chained fixed-size kernels (results consumed immediately)
//
// The #3083 regression kernels. Unlike Part A, each assignment's result feeds
// the next op, so a partial-packet tail store is followed by a packet load of
// the same destination. That store-to-load forwarding hazard is what makes a
// masked (or overlapping) tail far more costly than a scalar one -- it is only
// visible when the result is consumed, which these benchmarks do and Part A
// deliberately does not. These are the kernels that reflect the real cost.
// ===========================================================================

// #3083 Example 2: double 3x3 inverse() * t -- the headline regression.
EIGEN_DONT_INLINE void kernel_inverse(const Vector3d& a, const Vector3d& b, const Vector3d& c, const Matrix3d& R,
                                      const Vector3d& t, Vector3d& result) {
  Matrix3d A;
  A << a, -R * b, c;
  result = A.inverse() * t;
}

void BM_Chained_Inverse3x3(benchmark::State& state) {
  Vector3d a = Vector3d::Random(), b = Vector3d::Random(), c = Vector3d::Random(), t = Vector3d::Random(), result;
  Matrix3d R = Matrix3d::Random();
  for (auto _ : state) {
    benchmark::DoNotOptimize(a);
    benchmark::DoNotOptimize(b);
    benchmark::DoNotOptimize(R);
    kernel_inverse(a, b, c, R, t, result);
    benchmark::DoNotOptimize(result);
    benchmark::ClobberMemory();
  }
}

// #3083 Example 1: camera kernel chaining Matrix<double,2,3> blocks.
using Blk23 = Matrix<double, 2, 3, RowMajor>;
using Blocks5 = std::tuple<Blk23, Blk23, Blk23, Blk23, Blk23>;

EIGEN_DONT_INLINE void kernel_camera(const Matrix3d& A, const Matrix3d& B, const Vector3d& u, const Vector3d& v,
                                     const Vector3d& w, const Vector3d& x, const Vector4d& params, Vector2d& output,
                                     Blocks5& blocks) {
  const Vector3d a = A * u + v;
  const Vector3d b = B.transpose() * (a - w);
  const Vector3d c = b - x;
  const double iz = 1.0 / c.z();
  const double iz2 = iz * iz;
  const double k0 = params(0), k1 = params(1);
  Blk23 J;
  J << k0 * iz, 0.0, -k0 * c.x() * iz2, 0.0, k1 * iz, -k1 * c.y() * iz2;
  std::get<0>(blocks) = J * B.transpose();
  std::get<1>(blocks) = std::get<0>(blocks) * A;
  std::get<2>(blocks) = -std::get<1>(blocks);
  std::get<3>(blocks) = std::get<0>(blocks) + std::get<1>(blocks);
  std::get<4>(blocks) = std::get<3>(blocks) - std::get<2>(blocks);
  output << k0 * c.x() * iz + params(2), k1 * c.y() * iz + params(3);
}

void BM_Chained_Camera(benchmark::State& state) {
  Matrix3d A = Matrix3d::Random(), B = Matrix3d::Random();
  Vector3d u = Vector3d::Random(), v = Vector3d::Random(), w = Vector3d::Random(), x = Vector3d::Random();
  Vector4d params = Vector4d::Random();
  Vector2d output;
  Blocks5 blocks;
  for (auto _ : state) {
    benchmark::DoNotOptimize(A);
    benchmark::DoNotOptimize(B);
    benchmark::DoNotOptimize(u);
    kernel_camera(A, B, u, v, w, x, params, output, blocks);
    benchmark::DoNotOptimize(&blocks);
    benchmark::DoNotOptimize(output);
    benchmark::ClobberMemory();
  }
}

// Direct chained Matrix<double,2,3> sums: isolates the 6-element tail with a
// read-after-write dependency between consecutive assignments.
void BM_Chained_Block23(benchmark::State& state) {
  Blk23 m0 = Blk23::Random(), m1 = Blk23::Random(), m2 = Blk23::Random(), m3 = Blk23::Random();
  for (auto _ : state) {
    benchmark::DoNotOptimize(m0);
    m1 = m0 + m1;
    m2 = m1 + m2;
    m3 = m2 + m3;
    benchmark::DoNotOptimize(m3);
    benchmark::ClobberMemory();
  }
}

// #3083 Example A: the matrix algebra of one IMU integration step. Its 3x3
// products are lazy, so unlike the kernels above the destination assignment
// takes SliceVectorizedTraversal with InnerUnrolling, and the product evaluator
// itself loads a partial packet. Neither path was covered by the kernels above,
// which is why this one kept a masked tail after !2581.
void BM_Chained_Rotation(benchmark::State& state) {
  Matrix3d S = Matrix3d::Random(), R = Matrix3d::Random(), Xi = Matrix3d::Random();
  Vector3d a = Vector3d::Random(), v = Vector3d::Zero();
  const double dt = 0.005;
  for (auto _ : state) {
    benchmark::DoNotOptimize(S);
    benchmark::DoNotOptimize(R);
    benchmark::DoNotOptimize(a);
    const Matrix3d S2 = S * S;
    Xi = dt * Matrix3d::Identity() + 0.5 * dt * dt * S + (dt * dt * dt / 6.0) * S2;
    v += R * Xi * a;
    benchmark::DoNotOptimize(Xi);
    benchmark::DoNotOptimize(v);
    benchmark::ClobberMemory();
  }
}

// ===========================================================================
// Part D : streaming slices (compile-time inner size, runtime outer size)
//
// SliceVectorizedTraversal with InnerUnrolling, as in Chained/Rotation, but the
// tail is paid once per column of a long runtime outer loop and no consumer
// reloads the destination. Sqrt makes one packet evaluation of the source far
// cheaper than PacketSize scalar ones, so unlike Part C this shape wants the
// masked segment; the copy variant is the cheap-source control.
// ===========================================================================

template <typename T, int Rows>
void BM_SliceCopy(benchmark::State& state) {
  const Index cols = state.range(0);
  using Mat = Matrix<T, Dynamic, Dynamic>;
  Mat src = Mat::Random(Rows + 9, cols), dst = Mat::Zero(Rows + 9, cols);
  for (auto _ : state) {
    benchmark::DoNotOptimize(src.data());
    dst.template topRows<Rows>() = src.template topRows<Rows>();
    benchmark::DoNotOptimize(dst.data());
    benchmark::ClobberMemory();
  }
  state.SetBytesProcessed(state.iterations() * Rows * cols * static_cast<int64_t>(sizeof(T)) * 2);
}

template <typename T, int Rows>
void BM_SliceSqrt(benchmark::State& state) {
  const Index cols = state.range(0);
  using Mat = Matrix<T, Dynamic, Dynamic>;
  Mat src = Mat::Random(Rows + 9, cols).cwiseAbs(), dst = Mat::Zero(Rows + 9, cols);
  for (auto _ : state) {
    benchmark::DoNotOptimize(src.data());
    dst.template topRows<Rows>() = src.template topRows<Rows>().array().sqrt().matrix();
    benchmark::DoNotOptimize(dst.data());
    benchmark::ClobberMemory();
  }
  state.SetBytesProcessed(state.iterations() * Rows * cols * static_cast<int64_t>(sizeof(T)) * 2);
}

// ===========================================================================
// Registration
//
// Done programmatically (rather than via the BENCHMARK macro) so that Part B's
// tail count is clamped to the packet size actually in use: the report then
// contains no inapplicable count > PacketSize entries, whatever ISA the source
// was compiled for.
// ===========================================================================

template <typename T>
const char* type_tag();
template <>
const char* type_tag<float>() {
  return "f32";
}
template <>
const char* type_tag<double>() {
  return "f64";
}
template <>
const char* type_tag<int32_t>() {
  return "i32";
}
template <>
const char* type_tag<int64_t>() {
  return "i64";
}
template <>
const char* type_tag<std::complex<float>>() {
  return "cf32";
}
template <>
const char* type_tag<std::complex<double>>() {
  return "cf64";
}

template <typename T>
int packet_size() {
  return internal::unpacket_traits<typename internal::packet_traits<T>::type>::size;
}

// Part A : fixed-size tails. Exact multiples (8/16/32/64) act as no-tail
// controls; the rest cover a spread of remainders across SSE/AVX/AVX-512.
template <typename T, int N>
void add_fixed() {
  benchmark::RegisterBenchmark(std::string("FixedAssign/") + type_tag<T>() + "/" + std::to_string(N),
                               &BM_FixedAssign<T, N>);
}

template <typename T>
void add_all_fixed() {
  add_fixed<T, 8>();
  add_fixed<T, 9>();
  add_fixed<T, 11>();
  add_fixed<T, 15>();
  add_fixed<T, 16>();
  add_fixed<T, 17>();
  add_fixed<T, 23>();
  add_fixed<T, 32>();
  add_fixed<T, 33>();
  add_fixed<T, 47>();
  add_fixed<T, 64>();
  add_fixed<T, 65>();
}

// Part A : dynamic-size tails. Small sizes keep the tail a meaningful fraction;
// 1025 confirms there is no regression once the tail is amortized.
template <typename T>
void add_dynamic() {
  auto* b = benchmark::RegisterBenchmark(std::string("DynAssign/") + type_tag<T>(), &BM_DynamicAssign<T>);
  for (int s : {15, 17, 31, 33, 63, 65, 127, 129, 255, 1025}) b->Arg(s);
}

// Part B : segment vs scalar primitives, runtime count 1..PacketSize.
void add_prim_range(const std::string& name, void (*fn)(benchmark::State&), int ps) {
  auto* b = benchmark::RegisterBenchmark(name, fn);
  for (int c = 1; c <= ps; ++c) b->Arg(c);
}

template <typename T>
void add_prim() {
  const int ps = packet_size<T>();
  if (ps <= 1) return;  // type not vectorized on this ISA
  const std::string tag = type_tag<T>();
  add_prim_range("Prim/Store/seg/" + tag, &BM_PrimSegmentStore<T>, ps);
  add_prim_range("Prim/Store/scl/" + tag, &BM_PrimScalarStore<T>, ps);
  add_prim_range("Prim/StoreReload/seg/" + tag, &BM_PrimSegmentStoreReload<T>, ps);
  add_prim_range("Prim/StoreReload/scl/" + tag, &BM_PrimScalarStoreReload<T>, ps);
  benchmark::RegisterBenchmark("Prim/StoreReload/ovl/" + tag, &BM_PrimOverlapStoreReload<T>);
  add_prim_range("Prim/Load/seg/" + tag, &BM_PrimSegmentLoad<T>, ps);
  add_prim_range("Prim/Load/scl/" + tag, &BM_PrimScalarLoad<T>, ps);
}

// Part B : compile-time count store (matches the completely-unrolled tail).
template <typename T, int Count>
void add_prim_ct() {
  if (Count > packet_size<T>()) return;
  const std::string tag = std::string(type_tag<T>()) + "/" + std::to_string(Count);
  benchmark::RegisterBenchmark("PrimCT/Store/seg/" + tag, &BM_PrimSegmentStoreCT<T, Count>);
  benchmark::RegisterBenchmark("PrimCT/Store/scl/" + tag, &BM_PrimScalarStoreCT<T, Count>);
}

template <typename T>
void add_all_prim_ct() {
  add_prim_ct<T, 1>();
  add_prim_ct<T, 3>();
  add_prim_ct<T, 5>();
  add_prim_ct<T, 7>();
}

// Part D : one fixed inner size that is not a packet multiple for any supported
// packet width, swept over outer sizes that span the cache hierarchy.
template <typename T>
void add_slice() {
  const std::string tag = std::string("/") + type_tag<T>() + "/7";
  for (auto entry : {std::make_pair("Slice/copy" + tag, &BM_SliceCopy<T, 7>),
                     std::make_pair("Slice/sqrt" + tag, &BM_SliceSqrt<T, 7>)}) {
    auto* b = benchmark::RegisterBenchmark(entry.first, entry.second);
    for (int c : {64, 1024, 4096}) b->Arg(c);
  }
}

int RegisterAll() {
  add_trait_context<float>("f32");
  add_trait_context<double>("f64");
  add_trait_context<int32_t>("i32");
  add_trait_context<int64_t>("i64");
  add_trait_context<std::complex<float>>("cf32");
  add_trait_context<std::complex<double>>("cf64");

  add_all_fixed<float>();
  add_all_fixed<double>();
  add_dynamic<float>();
  add_dynamic<double>();

  add_prim<float>();
  add_prim<double>();
  add_prim<int32_t>();
  add_prim<int64_t>();
  add_prim<std::complex<float>>();
  add_prim<std::complex<double>>();

  add_all_prim_ct<float>();
  add_all_prim_ct<double>();

  benchmark::RegisterBenchmark("Chained/Inverse3x3", &BM_Chained_Inverse3x3);
  benchmark::RegisterBenchmark("Chained/Camera", &BM_Chained_Camera);
  benchmark::RegisterBenchmark("Chained/Block23", &BM_Chained_Block23);
  benchmark::RegisterBenchmark("Chained/Rotation", &BM_Chained_Rotation);

  add_slice<float>();
  add_slice<double>();
  return 0;
}

const int registered = RegisterAll();

}  // namespace

// main() is provided by benchmark::benchmark_main (linked via eigen_add_benchmark).
