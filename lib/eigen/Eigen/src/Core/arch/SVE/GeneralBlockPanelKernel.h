// IWYU pragma: private
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0
#include "../../InternalHeaderCheck.h"

namespace Eigen {
namespace internal {

// The generic gebp_traits declares nr = 4, and until this header existed the SVE
// backend took it: an SVE build measured a 6x4 register block where the NEON
// build of the same source measured 6x8. Isolated by forcing the NEON build to
// nr = 4, which cost it about 15% of its double GEMM rate -- the whole of the
// remaining gap between the two backends once double was vectorized at all.
//
// Only nr is overridden. NEON additionally replaces the RHS machinery because
// vfmaq_laneq_f64 needs an immediate lane index into a real vector; SVE's
// equivalent selects a lane within each 128-bit segment rather than across the
// vector, so it is not a drop-in at vector lengths above 128 bits, and the
// generic QuadPacket path -- pbroadcast4 to svdup, madd to svmla -- already
// emits what a hand-written version would.
#ifndef EIGEN_SVE_GEBP_NR
#define EIGEN_SVE_GEBP_NR 8
#endif

template <>
struct gebp_traits<float, float, false, false, Architecture::SVE, GEBPPacketFull>
    : gebp_traits<float, float, false, false, Architecture::Generic, GEBPPacketFull> {
  enum { nr = EIGEN_SVE_GEBP_NR };
};

template <>
struct gebp_traits<double, double, false, false, Architecture::SVE, GEBPPacketFull>
    : gebp_traits<double, double, false, false, Architecture::Generic, GEBPPacketFull> {
  enum { nr = EIGEN_SVE_GEBP_NR };
};

}  // namespace internal
}  // namespace Eigen
