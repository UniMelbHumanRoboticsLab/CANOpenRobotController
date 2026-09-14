// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#ifndef EIGEN_AMBIVECTOR_H
#define EIGEN_AMBIVECTOR_H

// IWYU pragma: private
#include "./InternalHeaderCheck.h"

namespace Eigen {

namespace internal {

/** \internal
 * Hybrid sparse/dense vector class designed for intensive read-write operations.
 *
 * See BasicSparseLLT and SparseProduct for usage examples.
 */
template <typename Scalar_, typename StorageIndex_>
class AmbiVector {
 public:
  using Scalar = Scalar_;
  using StorageIndex = StorageIndex_;

  explicit AmbiVector(Index size)
      : m_buffer(0),
        m_zero(0),
        m_size(0),
        m_end(0),
        m_allocatedSize(0),
        m_allocatedElements(0),
        m_denseConstructed(0),
        m_mode(-1) {
    resize(size);
  }

  void init(double estimatedDensity);
  void init(int mode);

  Index nonZeros() const;

  /** Specifies a sub-vector to work on */
  void setBounds(Index start, Index end) {
    m_start = convert_index(start);
    m_end = convert_index(end);
  }

  void setZero();

  void restart();
  Scalar& coeffRef(Index i);
  Scalar& coeff(Index i);

  class Iterator;

  ~AmbiVector() {
    destructElements();
    internal::aligned_free(m_buffer);
  }

  void resize(Index size) {
    if (m_allocatedSize < size) reallocate(size);
    m_size = convert_index(size);
    // The bounds describe a sub-vector of the old size, so they cannot survive a
    // resize that reuses the allocation: a smaller one would leave the iterators
    // running past the end, a larger one would stop them short of it.
    m_start = 0;
    m_end = m_size;
  }

  StorageIndex size() const { return m_size; }

 protected:
  // element type of the linked list
  struct ListEl {
    StorageIndex next;
    StorageIndex index;
    Scalar value;
  };

  StorageIndex convert_index(Index idx) { return internal::convert_index<StorageIndex>(idx); }

  ListEl* listElements() { return static_cast<ListEl*>(static_cast<void*>(m_buffer)); }
  const ListEl* listElements() const { return static_cast<const ListEl*>(static_cast<const void*>(m_buffer)); }

  void reallocate(Index size) {
    // if the size of the matrix is not too large, let's allocate a bit more than needed such
    // that we can handle dense vector even in sparse mode.
    destructElements();
    internal::aligned_free(m_buffer);
    Index allocSize;
    if (size < 1000) {
      allocSize = numext::div_ceil<Index>(size * sizeof(ListEl), sizeof(Scalar));
      m_allocatedElements = convert_index((allocSize * sizeof(Scalar)) / sizeof(ListEl));
    } else {
      allocSize = size;
      m_allocatedElements = convert_index((size * sizeof(Scalar)) / sizeof(ListEl));
    }
    // The buffer is raw storage: init() constructs the elements the mode needs.
    m_buffer = static_cast<Scalar*>(internal::aligned_malloc(allocSize * sizeof(Scalar)));
    m_allocatedSize = convert_index(allocSize);
    m_mode = -1;
  }

  void reallocateSparse() {
    Index copyElements = m_llSize;
    StorageIndex newAllocatedElements = (std::min)(StorageIndex(m_allocatedElements * 1.5), m_size);
    Index allocSize = newAllocatedElements * sizeof(ListEl);
    allocSize = numext::div_ceil<Index>(allocSize, sizeof(Scalar));
    Scalar* newBuffer = static_cast<Scalar*>(internal::aligned_malloc(allocSize * sizeof(Scalar)));
    ListEl* newElements = static_cast<ListEl*>(static_cast<void*>(newBuffer));
    // A throwing move leaves the nodes where they are, so the vector stays
    // destructible; the new buffer, which nothing points to yet, must be
    // released, and the capacity must not describe it either.
    EIGEN_TRY { internal::move_construct_elements_of_array(newElements, listElements(), copyElements); }
    EIGEN_CATCH(...) {
      internal::aligned_free(newBuffer);
      EIGEN_THROW;
    }
    internal::destruct_elements_of_array(listElements(), copyElements);
    internal::aligned_free(m_buffer);
    m_buffer = newBuffer;
    m_allocatedElements = newAllocatedElements;
    m_allocatedSize = convert_index(allocSize);
  }

  // Constructs a node holding a zero coefficient. Initializing the coefficient
  // as part of the construction keeps it atomic: a throwing Scalar leaves no
  // ListEl behind, so the caller can commit the node to the list - and to
  // m_llSize, which is what destructElements() destroys - only once it exists.
  static ListEl* constructListEl(ListEl* dst, StorageIndex index, StorageIndex next) {
    return ::new (static_cast<void*>(dst)) ListEl{next, index, Scalar(0)};
  }

  // Destroy whatever elements are currently alive in the raw buffer.
  void destructElements() {
    if (m_mode == IsDense) {
      internal::destruct_elements_of_array(m_buffer, m_denseConstructed);
      m_denseConstructed = 0;
    } else if (m_mode == IsSparse) {
      internal::destruct_elements_of_array(listElements(), m_llSize);
      m_llSize = 0;
    }
  }

  // used to store data in both modes
  Scalar* m_buffer;
  Scalar m_zero;
  StorageIndex m_size;
  StorageIndex m_start;
  StorageIndex m_end;
  StorageIndex m_allocatedSize;
  StorageIndex m_allocatedElements;
  StorageIndex m_denseConstructed;  // number of live Scalar objects in dense mode
  StorageIndex m_mode;

  // linked list mode
  StorageIndex m_llStart;
  StorageIndex m_llCurrent;
  StorageIndex m_llSize;
};

/** \returns the number of non zeros in the current sub vector */
template <typename Scalar_, typename StorageIndex_>
Index AmbiVector<Scalar_, StorageIndex_>::nonZeros() const {
  if (m_mode == IsSparse)
    return m_llSize;
  else
    return m_end - m_start;
}

template <typename Scalar_, typename StorageIndex_>
void AmbiVector<Scalar_, StorageIndex_>::init(double estimatedDensity) {
  if (estimatedDensity > 0.1)
    init(IsDense);
  else
    init(IsSparse);
}

template <typename Scalar_, typename StorageIndex_>
void AmbiVector<Scalar_, StorageIndex_>::init(int mode) {
  if (mode != m_mode) {
    destructElements();
    m_mode = convert_index(mode);
  } else if (m_mode == IsSparse) {
    // Re-initializing in sparse mode discards the previous list.
    internal::destruct_elements_of_array(listElements(), m_llSize);
  }
  if (m_mode == IsDense && m_denseConstructed < m_size) {
    // Construct the dense coefficients this mode reads and writes; they stay
    // alive across subsequent dense inits, like the values they carry.
    internal::default_construct_elements_of_array(m_buffer + m_denseConstructed, m_size - m_denseConstructed);
    m_denseConstructed = m_size;
  }
  // This is only necessary in sparse mode, but we set these unconditionally to avoid some maybe-uninitialized warnings
  // if (m_mode==IsSparse)
  {
    m_llSize = 0;
    m_llStart = -1;
  }
}

/** Must be called whenever we might perform a write access
 * with an index smaller than the previous one.
 *
 * Don't worry, this function is extremely cheap.
 */
template <typename Scalar_, typename StorageIndex_>
void AmbiVector<Scalar_, StorageIndex_>::restart() {
  m_llCurrent = m_llStart;
}

/** Set all coefficients of current subvector to zero */
template <typename Scalar_, typename StorageIndex_>
void AmbiVector<Scalar_, StorageIndex_>::setZero() {
  if (m_mode == IsDense) {
    for (Index i = m_start; i < m_end; ++i) m_buffer[i] = Scalar(0);
  } else {
    eigen_assert(m_mode == IsSparse);
    // The nodes being dropped own their coefficients, and a later coeffRef()
    // constructs its node in place over this storage.
    internal::destruct_elements_of_array(listElements(), m_llSize);
    m_llSize = 0;
    m_llStart = -1;
  }
}

template <typename Scalar_, typename StorageIndex_>
Scalar_& AmbiVector<Scalar_, StorageIndex_>::coeffRef(Index i) {
  if (m_mode == IsDense)
    return m_buffer[i];
  else {
    ListEl* EIGEN_RESTRICT llElements = listElements();
    // TODO: factor out the following code to reduce code generation
    eigen_assert(m_mode == IsSparse);
    if (m_llSize == 0) {
      // this is the first element
      ListEl& el = *constructListEl(llElements, convert_index(i), -1);
      m_llStart = 0;
      m_llCurrent = 0;
      m_llSize = 1;
      return el.value;
    } else if (i < llElements[m_llStart].index) {
      // this is going to be the new first element of the list
      ListEl& el = *constructListEl(llElements + m_llSize, convert_index(i), m_llStart);
      m_llStart = m_llSize;
      m_llCurrent = m_llStart;
      ++m_llSize;
      return el.value;
    } else {
      StorageIndex nextel = llElements[m_llCurrent].next;
      eigen_assert(i >= llElements[m_llCurrent].index &&
                   "you must call restart() before inserting an element with lower or equal index");
      while (nextel >= 0 && llElements[nextel].index <= i) {
        m_llCurrent = nextel;
        nextel = llElements[nextel].next;
      }

      if (llElements[m_llCurrent].index == i) {
        // the coefficient already exists and we found it !
        return llElements[m_llCurrent].value;
      } else {
        if (m_llSize >= m_allocatedElements) {
          reallocateSparse();
          llElements = listElements();
        }
        eigen_internal_assert(m_llSize < m_allocatedElements && "internal error: overflow in sparse mode");
        // let's insert a new coefficient
        ListEl& el = *constructListEl(llElements + m_llSize, convert_index(i), llElements[m_llCurrent].next);
        llElements[m_llCurrent].next = m_llSize;
        ++m_llSize;
        return el.value;
      }
    }
  }
}

template <typename Scalar_, typename StorageIndex_>
Scalar_& AmbiVector<Scalar_, StorageIndex_>::coeff(Index i) {
  if (m_mode == IsDense)
    return m_buffer[i];
  else {
    ListEl* EIGEN_RESTRICT llElements = listElements();
    eigen_assert(m_mode == IsSparse);
    if ((m_llSize == 0) || (i < llElements[m_llStart].index)) {
      return m_zero;
    } else {
      Index elid = m_llStart;
      while (elid >= 0 && llElements[elid].index < i) elid = llElements[elid].next;

      if (elid >= 0 && llElements[elid].index == i)
        return llElements[elid].value;
      else
        return m_zero;
    }
  }
}

/** Iterator over the nonzero coefficients */
template <typename Scalar_, typename StorageIndex_>
class AmbiVector<Scalar_, StorageIndex_>::Iterator {
 public:
  using Scalar = Scalar_;
  using RealScalar = typename NumTraits<Scalar>::Real;

  /** Default constructor
   * \param vec the vector on which we iterate
   * \param epsilon the minimal value used to prune zero coefficients.
   * In practice, all coefficients having a magnitude smaller than \a epsilon
   * are skipped.
   */
  explicit Iterator(const AmbiVector& vec, const RealScalar& epsilon = 0) : m_vector(vec) {
    using std::abs;
    m_epsilon = epsilon;
    m_isDense = m_vector.m_mode == IsDense;
    if (m_isDense) {
      m_currentEl = 0;    // this is to avoid a compilation warning
      m_cachedValue = 0;  // this is to avoid a compilation warning
      m_cachedIndex = m_vector.m_start - 1;
      ++(*this);
    } else {
      const ListEl* EIGEN_RESTRICT llElements = m_vector.listElements();
      m_currentEl = m_vector.m_llStart;
      while (m_currentEl >= 0 && abs(llElements[m_currentEl].value) <= m_epsilon)
        m_currentEl = llElements[m_currentEl].next;
      if (m_currentEl < 0) {
        m_cachedValue = 0;  // this is to avoid a compilation warning
        m_cachedIndex = -1;
      } else {
        m_cachedIndex = llElements[m_currentEl].index;
        m_cachedValue = llElements[m_currentEl].value;
      }
    }
  }

  StorageIndex index() const { return m_cachedIndex; }
  Scalar value() const { return m_cachedValue; }

  operator bool() const { return m_cachedIndex >= 0; }

  Iterator& operator++() {
    using std::abs;
    if (m_isDense) {
      do {
        ++m_cachedIndex;
      } while (m_cachedIndex < m_vector.m_end && abs(m_vector.m_buffer[m_cachedIndex]) <= m_epsilon);
      if (m_cachedIndex < m_vector.m_end)
        m_cachedValue = m_vector.m_buffer[m_cachedIndex];
      else
        m_cachedIndex = -1;
    } else {
      const ListEl* EIGEN_RESTRICT llElements = m_vector.listElements();
      do {
        m_currentEl = llElements[m_currentEl].next;
      } while (m_currentEl >= 0 && abs(llElements[m_currentEl].value) <= m_epsilon);
      if (m_currentEl < 0) {
        m_cachedIndex = -1;
      } else {
        m_cachedIndex = llElements[m_currentEl].index;
        m_cachedValue = llElements[m_currentEl].value;
      }
    }
    return *this;
  }

 protected:
  const AmbiVector& m_vector;  // the target vector
  StorageIndex m_currentEl;    // the current element in sparse/linked-list mode
  RealScalar m_epsilon;        // epsilon used to prune zero coefficients
  StorageIndex m_cachedIndex;  // current coordinate
  Scalar m_cachedValue;        // current value
  bool m_isDense;              // mode of the vector
};

}  // end namespace internal

}  // end namespace Eigen

#endif  // EIGEN_AMBIVECTOR_H
