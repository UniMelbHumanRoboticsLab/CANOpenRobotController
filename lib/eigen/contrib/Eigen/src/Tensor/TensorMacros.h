// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2015 Benoit Steiner <benoit.steiner.goog@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_TENSOR_TENSOR_META_MACROS_H
#define EIGEN_TENSOR_TENSOR_META_MACROS_H

/** use this macro in sfinae selection in templated functions
 *
 *   template<typename T,
 *            std::enable_if_t< isBanana<T>::value , int > = 0
 *   >
 *   void foo(){}
 *
 *   becomes =>
 *
 *   template<typename T,
 *           EIGEN_SFINAE_ENABLE_IF( isBanana<T>::value )
 *   >
 *   void foo(){}
 */

#define EIGEN_SFINAE_ENABLE_IF(__condition__) std::enable_if_t<(__condition__), int> = 0

// Define a macro to use a reference on the host but a value on the device
#if defined(SYCL_DEVICE_ONLY)
#define EIGEN_DEVICE_REF
#else
#define EIGEN_DEVICE_REF &
#endif

// Define a macro for catching SYCL exceptions if exceptions are enabled
#define EIGEN_SYCL_TRY_CATCH(X)                                                                                        \
  do {                                                                                                                 \
    EIGEN_TRY { X; }                                                                                                   \
    EIGEN_CATCH(const cl::sycl::exception& e) {                                                                        \
      EIGEN_THROW_X(std::runtime_error("SYCL exception at " + std::string(__FILE__) + ":" + std::to_string(__LINE__) + \
                                       "\n" + e.what()));                                                              \
    }                                                                                                                  \
  } while (false)

// Define a macro if local memory flags are unset or one of them is set
// Setting both flags is the same as unsetting them
#if (!defined(EIGEN_SYCL_LOCAL_MEM) && !defined(EIGEN_SYCL_NO_LOCAL_MEM)) || \
    (defined(EIGEN_SYCL_LOCAL_MEM) && defined(EIGEN_SYCL_NO_LOCAL_MEM))
#define EIGEN_SYCL_LOCAL_MEM_UNSET_OR_ON 1
#define EIGEN_SYCL_LOCAL_MEM_UNSET_OR_OFF 1
#elif defined(EIGEN_SYCL_LOCAL_MEM) && !defined(EIGEN_SYCL_NO_LOCAL_MEM)
#define EIGEN_SYCL_LOCAL_MEM_UNSET_OR_ON 1
#elif !defined(EIGEN_SYCL_LOCAL_MEM) && defined(EIGEN_SYCL_NO_LOCAL_MEM)
#define EIGEN_SYCL_LOCAL_MEM_UNSET_OR_OFF 1
#endif

#endif
