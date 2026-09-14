// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

#ifndef HIP_VECTOR_COMPATIBILITY_H
#define HIP_VECTOR_COMPATIBILITY_H

namespace hip_impl {
template <typename, typename, unsigned int>
struct Scalar_accessor;
}  // end namespace hip_impl

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {
namespace internal {

#define HIP_SCALAR_ACCESSOR_BUILDER(NAME)           \
  template <typename T, typename U, unsigned int n> \
  struct NAME<hip_impl::Scalar_accessor<T, U, n>> : NAME<T> {};

#define HIP_SCALAR_ACCESSOR_BUILDER_IMPL(NAME)      \
  template <typename T, typename U, unsigned int n> \
  struct NAME##_impl<hip_impl::Scalar_accessor<T, U, n>> : NAME##_impl<T> {};

#define HIP_SCALAR_ACCESSOR_BUILDER_IGAMMA(NAME)                                \
  template <typename T, typename U, unsigned int n, IgammaComputationMode mode> \
  struct NAME<hip_impl::Scalar_accessor<T, U, n>, mode> : NAME<T, mode> {};

HIP_SCALAR_ACCESSOR_BUILDER(betainc_helper)
HIP_SCALAR_ACCESSOR_BUILDER(incbeta_cfe)

HIP_SCALAR_ACCESSOR_BUILDER_IMPL(erf)
HIP_SCALAR_ACCESSOR_BUILDER_IMPL(erfc)
HIP_SCALAR_ACCESSOR_BUILDER_IMPL(igammac)
HIP_SCALAR_ACCESSOR_BUILDER_IMPL(lgamma)
HIP_SCALAR_ACCESSOR_BUILDER_IMPL(ndtri)
HIP_SCALAR_ACCESSOR_BUILDER_IMPL(polygamma)

HIP_SCALAR_ACCESSOR_BUILDER_IGAMMA(igamma_generic_impl)

HIP_SCALAR_ACCESSOR_BUILDER(digamma_impl_maybe_poly)
HIP_SCALAR_ACCESSOR_BUILDER(zeta_impl_series)

HIP_SCALAR_ACCESSOR_BUILDER_IMPL(bessel_i0)
HIP_SCALAR_ACCESSOR_BUILDER_IMPL(bessel_i0e)
HIP_SCALAR_ACCESSOR_BUILDER_IMPL(bessel_i1)
HIP_SCALAR_ACCESSOR_BUILDER_IMPL(bessel_i1e)
HIP_SCALAR_ACCESSOR_BUILDER_IMPL(bessel_j0)
HIP_SCALAR_ACCESSOR_BUILDER_IMPL(bessel_j1)
HIP_SCALAR_ACCESSOR_BUILDER_IMPL(bessel_k0)
HIP_SCALAR_ACCESSOR_BUILDER_IMPL(bessel_k0e)
HIP_SCALAR_ACCESSOR_BUILDER_IMPL(bessel_k1)
HIP_SCALAR_ACCESSOR_BUILDER_IMPL(bessel_k1e)
HIP_SCALAR_ACCESSOR_BUILDER_IMPL(bessel_y0)
HIP_SCALAR_ACCESSOR_BUILDER_IMPL(bessel_y1)
HIP_SCALAR_ACCESSOR_BUILDER_IMPL(betainc)
HIP_SCALAR_ACCESSOR_BUILDER_IMPL(digamma)
HIP_SCALAR_ACCESSOR_BUILDER_IMPL(gamma_sample_der_alpha)
HIP_SCALAR_ACCESSOR_BUILDER_IMPL(igamma_der_a)
HIP_SCALAR_ACCESSOR_BUILDER_IMPL(igamma)
HIP_SCALAR_ACCESSOR_BUILDER_IMPL(zeta)

HIP_SCALAR_ACCESSOR_BUILDER_IGAMMA(igamma_series_impl)
HIP_SCALAR_ACCESSOR_BUILDER_IGAMMA(igammac_cf_impl)

#undef HIP_SCALAR_ACCESSOR_BUILDER
#undef HIP_SCALAR_ACCESSOR_BUILDER_IMPL
#undef HIP_SCALAR_ACCESSOR_BUILDER_IGAMMA

}  // end namespace internal
}  // end namespace Eigen

#endif  // HIP_VECTOR_COMPATIBILITY_H
