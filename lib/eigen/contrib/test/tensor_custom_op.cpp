// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2014 Benoit Steiner <benoit.steiner.goog@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0

#include "main.h"

#include <Eigen/Tensor>

using Eigen::Tensor;

struct InsertZeros {
  DSizes<DenseIndex, 2> dimensions(const Tensor<float, 2>& input) const {
    DSizes<DenseIndex, 2> result;
    result[0] = input.dimension(0) * 2;
    result[1] = input.dimension(1) * 2;
    return result;
  }

  template <typename Output, typename Device>
  void eval(const Tensor<float, 2>& input, Output& output, const Device& device) const {
    array<DenseIndex, 2> strides;
    strides[0] = 2;
    strides[1] = 2;
    output.stride(strides).device(device) = input;

    Eigen::DSizes<DenseIndex, 2> offsets(1, 1);
    Eigen::DSizes<DenseIndex, 2> extents(output.dimension(0) - 1, output.dimension(1) - 1);
    output.slice(offsets, extents).stride(strides).device(device) = input.constant(0.0f);
  }
};

static void test_custom_unary_op() {
  Tensor<float, 2> tensor(3, 5);
  tensor.setRandom();

  Tensor<float, 2> result = tensor.customOp(InsertZeros());
  VERIFY_IS_EQUAL(result.dimension(0), 6);
  VERIFY_IS_EQUAL(result.dimension(1), 10);

  for (int i = 0; i < 6; i += 2) {
    for (int j = 0; j < 10; j += 2) {
      VERIFY_IS_EQUAL(result(i, j), tensor(i / 2, j / 2));
    }
  }
  for (int i = 1; i < 6; i += 2) {
    for (int j = 1; j < 10; j += 2) {
      VERIFY_IS_EQUAL(result(i, j), 0);
    }
  }
}

struct BatchMatMul {
  DSizes<DenseIndex, 3> dimensions(const Tensor<float, 3>& input1, const Tensor<float, 3>& input2) const {
    DSizes<DenseIndex, 3> result;
    result[0] = input1.dimension(0);
    result[1] = input2.dimension(1);
    result[2] = input2.dimension(2);
    return result;
  }

  template <typename Output, typename Device>
  void eval(const Tensor<float, 3>& input1, const Tensor<float, 3>& input2, Output& output,
            const Device& device) const {
    typedef Tensor<float, 3>::DimensionPair DimPair;
    array<DimPair, 1> dims;
    dims[0] = DimPair(1, 0);
    for (int i = 0; i < output.dimension(2); ++i) {
      output.template chip<2>(i).device(device) = input1.chip<2>(i).contract(input2.chip<2>(i), dims);
    }
  }
};

static void test_custom_binary_op() {
  Tensor<float, 3> tensor1(2, 3, 5);
  tensor1.setRandom();
  Tensor<float, 3> tensor2(3, 7, 5);
  tensor2.setRandom();

  Tensor<float, 3> result = tensor1.customOp(tensor2, BatchMatMul());
  for (int i = 0; i < 5; ++i) {
    typedef Tensor<float, 3>::DimensionPair DimPair;
    array<DimPair, 1> dims;
    dims[0] = DimPair(1, 0);
    Tensor<float, 2> reference = tensor1.chip<2>(i).contract(tensor2.chip<2>(i), dims);
    TensorRef<Tensor<float, 2> > val = result.chip<2>(i);
    for (int j = 0; j < 2; ++j) {
      for (int k = 0; k < 7; ++k) {
        VERIFY_IS_APPROX(val(j, k), reference(j, k));
      }
    }
  }
}

template <int DataLayout>
struct InsertZerosLayout {
  DSizes<DenseIndex, 2> dimensions(const Tensor<float, 2, DataLayout>& input) const {
    DSizes<DenseIndex, 2> result;
    result[0] = input.dimension(0) * 2;
    result[1] = input.dimension(1) * 2;
    return result;
  }

  template <typename Output, typename Device>
  void eval(const Tensor<float, 2, DataLayout>& input, Output& output, const Device& device) const {
    output.device(device) = output.constant(0.0f);

    array<DenseIndex, 2> strides;
    strides[0] = 2;
    strides[1] = 2;
    output.stride(strides).device(device) = input;
  }
};

template <int DataLayout>
static float insert_zeros_reference(const Tensor<float, 2, DataLayout>& input, Index i, Index j) {
  return i % 2 == 0 && j % 2 == 0 ? input(i / 2, j / 2) : 0.0f;
}

template <int DataLayout>
static void test_custom_op_raw_access() {
  // The custom-op evaluators materialize into a dense buffer, so they must
  // advertise raw access like TensorForcedEvalOp.
  typedef Tensor<float, 2, DataLayout> Tensor2;

  Tensor2 tensor(13, 17);
  tensor.setRandom();

  auto custom_expr = tensor.customOp(InsertZerosLayout<DataLayout>());
  typedef TensorEvaluator<const decltype(custom_expr), DefaultDevice> CustomEval;
  static_assert(CustomEval::RawAccess == 1, "custom op must expose its materialized buffer");

  // Padding consumes the custom op directly through data().
  Eigen::array<std::pair<Index, Index>, 2> paddings;
  paddings[0] = {2, 3};
  paddings[1] = {1, 4};
  Tensor2 result = custom_expr.pad(paddings);

  VERIFY_IS_EQUAL(result.dimension(0), 2 + 26 + 3);
  VERIFY_IS_EQUAL(result.dimension(1), 1 + 34 + 4);
  for (Index i = 0; i < result.dimension(0); ++i) {
    for (Index j = 0; j < result.dimension(1); ++j) {
      const bool interior = i >= 2 && i < 2 + 26 && j >= 1 && j < 1 + 34;
      if (interior) {
        const Index ii = i - 2;
        const Index jj = j - 1;
        VERIFY_IS_EQUAL(result(i, j), insert_zeros_reference(tensor, ii, jj));
      } else {
        VERIFY_IS_EQUAL(result(i, j), 0.0f);
      }
    }
  }
}

template <int DataLayout>
struct AddInputsLayout {
  DSizes<DenseIndex, 2> dimensions(const Tensor<float, 2, DataLayout>& lhs, const Tensor<float, 2, DataLayout>&) const {
    return lhs.dimensions();
  }

  template <typename Output, typename Device>
  void eval(const Tensor<float, 2, DataLayout>& lhs, const Tensor<float, 2, DataLayout>& rhs, Output& output,
            const Device& device) const {
    output.device(device) = lhs + rhs;
  }
};

template <int DataLayout>
static void test_custom_op_block_access() {
  typedef Tensor<float, 2, DataLayout> Tensor2;

  Tensor2 lhs(259, 263);
  Tensor2 rhs(259, 263);
  lhs.setRandom();
  rhs.setRandom();

  // Exceed TensorSlicingOp's coefficient threshold so it delegates to the
  // custom-op evaluator's block() method instead of its memcpy path.
  Eigen::array<Index, 2> offsets = {1, 2};
  Eigen::array<Index, 2> extents;
  extents[0] = DataLayout == ColMajor ? 129 : 257;
  extents[1] = DataLayout == ColMajor ? 257 : 129;

  auto unary_expr = lhs.customOp(InsertZerosLayout<DataLayout>());
  typedef TensorEvaluator<const decltype(unary_expr), DefaultDevice> UnaryEval;
  static_assert(UnaryEval::BlockAccess == 1, "custom unary op must support block access");
  Tensor2 unary_result = unary_expr.slice(offsets, extents);

  for (Index i = 0; i < extents[0]; ++i) {
    for (Index j = 0; j < extents[1]; ++j) {
      const Index source_i = i + offsets[0];
      const Index source_j = j + offsets[1];
      VERIFY_IS_EQUAL(unary_result(i, j), insert_zeros_reference(lhs, source_i, source_j));
    }
  }

  auto binary_expr = lhs.customOp(rhs, AddInputsLayout<DataLayout>());
  typedef TensorEvaluator<const decltype(binary_expr), DefaultDevice> BinaryEval;
  static_assert(BinaryEval::RawAccess == 1, "custom binary op must expose its materialized buffer");
  static_assert(BinaryEval::BlockAccess == 1, "custom binary op must support block access");
  Tensor2 binary_result = binary_expr.slice(offsets, extents);

  for (Index i = 0; i < extents[0]; ++i) {
    for (Index j = 0; j < extents[1]; ++j) {
      const Index source_i = i + offsets[0];
      const Index source_j = j + offsets[1];
      VERIFY_IS_EQUAL(binary_result(i, j), lhs(source_i, source_j) + rhs(source_i, source_j));
    }
  }
}

struct RowSumOp {
  // The rank-changing functor from the customOp() documentation example.
  template <typename Input>
  DSizes<Index, 1> dimensions(const Input& input) const {
    return DSizes<Index, 1>(input.dimension(0));
  }

  template <typename Input, typename Output, typename Device>
  void eval(const Input& input, Output& output, const Device& device) const {
    array<Index, 1> reduce_dims{1};
    output.device(device) = input.sum(reduce_dims);
  }
};

struct RowSumExprOp {
  // Rank-changing functor accepting arbitrary input expressions. Lazy
  // expressions do not expose dimension(), so sizes are computed with a
  // TensorEvaluator, whose constructor determines the dimensions without
  // evaluating the expression.
  template <typename Input>
  DSizes<Index, 1> dimensions(const Input& input) const {
    DefaultDevice device;
    TensorEvaluator<const Input, DefaultDevice> eval(input, device);
    return DSizes<Index, 1>(eval.dimensions()[0]);
  }

  template <typename Input, typename Output, typename Device>
  void eval(const Input& input, Output& output, const Device& device) const {
    array<Index, 1> reduce_dims{1};
    output.device(device) = input.sum(reduce_dims);
  }
};

template <int DataLayout>
static void test_custom_unary_op_output_rank() {
  // Regression test for issue #3106: the output rank must come from the
  // functor's dimensions() method, not from the input expression.
  Tensor<float, 2, DataLayout> tensor(3, 5);
  tensor.setRandom();

  Tensor<float, 1, DataLayout> row_sums = tensor.customOp(RowSumOp());
  VERIFY_IS_EQUAL(row_sums.dimension(0), 3);
  for (Index i = 0; i < 3; ++i) {
    float expected = 0.0f;
    for (Index j = 0; j < 5; ++j) {
      expected += tensor(i, j);
    }
    VERIFY_IS_APPROX(row_sums(i), expected);
  }

  // The input may be an arbitrary expression rather than a plain tensor.
  Tensor<float, 1, DataLayout> doubled_row_sums = (tensor + tensor).customOp(RowSumExprOp());
  VERIFY_IS_EQUAL(doubled_row_sums.dimension(0), 3);
  for (Index i = 0; i < 3; ++i) {
    VERIFY_IS_APPROX(doubled_row_sums(i), 2.0f * row_sums(i));
  }
}

struct OuterProductOp {
  template <typename Lhs, typename Rhs>
  DSizes<Index, 2> dimensions(const Lhs& lhs, const Rhs& rhs) const {
    return DSizes<Index, 2>(lhs.dimension(0), rhs.dimension(0));
  }

  template <typename Lhs, typename Rhs, typename Output, typename Device>
  void eval(const Lhs& lhs, const Rhs& rhs, Output& output, const Device& device) const {
    const array<IndexPair<Index>, 0> contract_dims{};
    output.device(device) = lhs.contract(rhs, contract_dims);
  }
};

template <typename OuterTensor, typename VectorTensor>
static void verify_outer_product(const OuterTensor& outer, const VectorTensor& lhs, const VectorTensor& rhs) {
  using OuterIndex = typename OuterTensor::Index;
  VERIFY_IS_EQUAL(outer.dimension(0), lhs.dimension(0));
  VERIFY_IS_EQUAL(outer.dimension(1), rhs.dimension(0));
  for (OuterIndex i = 0; i < outer.dimension(0); ++i) {
    for (OuterIndex j = 0; j < outer.dimension(1); ++j) {
      VERIFY_IS_APPROX(outer(i, j), lhs(i) * rhs(j));
    }
  }
}

template <int DataLayout>
static void test_custom_binary_op_output_rank() {
  Tensor<float, 1, DataLayout> lhs(3);
  Tensor<float, 1, DataLayout> rhs(4);
  lhs.setRandom();
  rhs.setRandom();

  Tensor<float, 2, DataLayout> outer = lhs.customOp(rhs, OuterProductOp());
  verify_outer_product(outer, lhs, rhs);
}

struct OuterProductIntIndexOp {
  // Returns dimensions with index type int regardless of the expressions'
  // index type.
  template <typename Lhs, typename Rhs>
  DSizes<int, 2> dimensions(const Lhs& lhs, const Rhs& rhs) const {
    return DSizes<int, 2>(static_cast<int>(lhs.dimension(0)), static_cast<int>(rhs.dimension(0)));
  }

  template <typename Lhs, typename Rhs, typename Output, typename Device>
  void eval(const Lhs& lhs, const Rhs& rhs, Output& output, const Device& device) const {
    const array<IndexPair<typename Lhs::Index>, 0> contract_dims{};
    output.device(device) = lhs.contract(rhs, contract_dims);
  }
};

template <int DataLayout>
static void test_custom_binary_op_custom_index() {
  // Operands with a non-default index type: their DSizes<int, ...> dimensions
  // are promoted into the evaluator's Dimensions and output map.
  Tensor<float, 1, DataLayout, int> lhs(3);
  Tensor<float, 1, DataLayout, int> rhs(4);
  lhs.setRandom();
  rhs.setRandom();

  Tensor<float, 2, DataLayout, int> outer = lhs.customOp(rhs, OuterProductIntIndexOp());
  verify_outer_product(outer, lhs, rhs);

  // The functor may also return dimensions with a narrower index type than the
  // expressions'; the evaluator promotes them.
  Tensor<float, 1, DataLayout> lhs_default(3);
  Tensor<float, 1, DataLayout> rhs_default(4);
  lhs_default.setRandom();
  rhs_default.setRandom();

  Tensor<float, 2, DataLayout> outer_default = lhs_default.customOp(rhs_default, OuterProductIntIndexOp());
  verify_outer_product(outer_default, lhs_default, rhs_default);
}

// An index type as wide as DenseIndex but distinct from it where the platform
// provides one (long long versus long on LP64), so the output map's index type
// is pinned on a width tie and not just when the operands are narrower.
using EqualWidthIndex =
    std::conditional_t<sizeof(long long) == sizeof(DenseIndex) && !std::is_same<long long, DenseIndex>::value,
                       long long,
                       std::conditional_t<sizeof(long) == sizeof(DenseIndex) && !std::is_same<long, DenseIndex>::value,
                                          long, DenseIndex> >;

template <int DataLayout, typename IndexType>
struct AddExactOutput {
  DSizes<IndexType, 2> dimensions(const Tensor<float, 2, DataLayout, IndexType>& lhs,
                                  const Tensor<float, 2, DataLayout, IndexType>&) const {
    return DSizes<IndexType, 2>(lhs.dimension(0), lhs.dimension(1));
  }

  // eval() spells out the exact Output type the evaluator constructs: the
  // DenseIndex-typed map, for any operand index type no wider than DenseIndex.
  template <typename Device>
  void eval(const Tensor<float, 2, DataLayout, IndexType>& lhs, const Tensor<float, 2, DataLayout, IndexType>& rhs,
            TensorMap<Tensor<float, 2, DataLayout> >& output, const Device& device) const {
    output.device(device) = lhs + rhs;
  }
};

template <int DataLayout, typename IndexType>
static void verify_exact_output_type() {
  Tensor<float, 2, DataLayout, IndexType> lhs(3, 4);
  Tensor<float, 2, DataLayout, IndexType> rhs(3, 4);
  lhs.setRandom();
  rhs.setRandom();

  Tensor<float, 2, DataLayout, IndexType> sum = lhs.customOp(rhs, AddExactOutput<DataLayout, IndexType>());
  VERIFY_IS_EQUAL(sum.dimension(0), 3);
  VERIFY_IS_EQUAL(sum.dimension(1), 4);
  for (IndexType i = 0; i < 3; ++i) {
    for (IndexType j = 0; j < 4; ++j) {
      VERIFY_IS_APPROX(sum(i, j), lhs(i, j) + rhs(i, j));
    }
  }
}

template <int DataLayout>
static void test_custom_binary_op_exact_output_type() {
  // The Output parameter of eval() is a user customization point: functors
  // written against the concrete DenseIndex-typed TensorMap must keep
  // compiling for operand index types no wider than DenseIndex, including a
  // distinct type of the same width.
  verify_exact_output_type<DataLayout, int>();
  verify_exact_output_type<DataLayout, EqualWidthIndex>();
}

EIGEN_DECLARE_TEST(tensor_custom_op) {
  CALL_SUBTEST(test_custom_unary_op());
  CALL_SUBTEST(test_custom_binary_op());
  CALL_SUBTEST(test_custom_op_raw_access<ColMajor>());
  CALL_SUBTEST(test_custom_op_raw_access<RowMajor>());
  CALL_SUBTEST(test_custom_op_block_access<ColMajor>());
  CALL_SUBTEST(test_custom_op_block_access<RowMajor>());
  CALL_SUBTEST(test_custom_unary_op_output_rank<ColMajor>());
  CALL_SUBTEST(test_custom_unary_op_output_rank<RowMajor>());
  CALL_SUBTEST(test_custom_binary_op_output_rank<ColMajor>());
  CALL_SUBTEST(test_custom_binary_op_output_rank<RowMajor>());
  CALL_SUBTEST(test_custom_binary_op_custom_index<ColMajor>());
  CALL_SUBTEST(test_custom_binary_op_custom_index<RowMajor>());
  CALL_SUBTEST(test_custom_binary_op_exact_output_type<ColMajor>());
  CALL_SUBTEST(test_custom_binary_op_exact_output_type<RowMajor>());
}
