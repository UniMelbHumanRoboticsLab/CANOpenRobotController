// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-FileCopyrightText: The Eigen Authors
// SPDX-License-Identifier: MPL-2.0

// clang-format off
#include "main.h"
#include <Eigen/Tensor>
#include "../Eigen/SpecialFunctions"
// clang-format on

using Eigen::internal::TensorBlockDescriptor;
using Eigen::internal::TensorExecutor;

template <typename T, int NumDims, int Layout>
static void setRandomForBinaryProduct(Tensor<T, NumDims, Layout>& tensor) {
  EIGEN_IF_CONSTEXPR ((std::is_integral<T>::value && !std::is_same<T, bool>::value)) {
    setRandomDataInRange(tensor, T(-1000), T(1000));
  } else {
    tensor.setRandom();
  }
}

// -------------------------------------------------------------------------- //
// Utility functions to generate random tensors, blocks, and evaluate them.

template <int NumDims>
static DSizes<Index, NumDims> RandomDims(Index min, Index max) {
  DSizes<Index, NumDims> dims;
  for (int i = 0; i < NumDims; ++i) {
    dims[i] = internal::random<Index>(min, max);
  }
  return DSizes<Index, NumDims>(dims);
}

// Block offsets and extents allows to construct a TensorSlicingOp corresponding
// to a TensorBlockDescriptor.
template <int NumDims>
struct TensorBlockParams {
  DSizes<Index, NumDims> offsets;
  DSizes<Index, NumDims> sizes;
  TensorBlockDescriptor<NumDims, Index> desc;
};

template <int Layout, int NumDims>
static TensorBlockParams<NumDims> RandomBlock(DSizes<Index, NumDims> dims, Index min, Index max) {
  // Choose random offsets and sizes along all tensor dimensions.
  DSizes<Index, NumDims> offsets(RandomDims<NumDims>(min, max));
  DSizes<Index, NumDims> sizes(RandomDims<NumDims>(min, max));

  // Make sure that offset + size do not overflow dims.
  for (int i = 0; i < NumDims; ++i) {
    offsets[i] = numext::mini(dims[i] - 1, offsets[i]);
    sizes[i] = numext::mini(sizes[i], dims[i] - offsets[i]);
  }

  Index offset = 0;
  DSizes<Index, NumDims> strides = Eigen::internal::strides<Layout>(dims);
  for (int i = 0; i < NumDims; ++i) {
    offset += strides[i] * offsets[i];
  }

  return {offsets, sizes, TensorBlockDescriptor<NumDims, Index>(offset, sizes)};
}

// Generate block with block sizes skewed towards inner dimensions. This type of
// block is required for evaluating broadcast expressions.
template <int Layout, int NumDims>
static std::enable_if_t<NumDims == 0, void> SetSkewedInnerBlockInnerOffset(DSizes<Index, NumDims>& /*offsets*/,
                                                                           Index /*index*/) {}

template <int Layout, int NumDims>
static std::enable_if_t<(NumDims > 0), void> SetSkewedInnerBlockInnerOffset(DSizes<Index, NumDims>& offsets,
                                                                            Index index) {
  EIGEN_IF_CONSTEXPR (static_cast<int>(Layout) == static_cast<int>(ColMajor)) {
    offsets[0] = index;
  } else {
    offsets[NumDims - 1] = index;
  }
}

template <int Layout, int NumDims>
static TensorBlockParams<NumDims> SkewedInnerBlock(DSizes<Index, NumDims> dims) {
  using BlockMapper = internal::TensorBlockMapper<NumDims, Layout, Index>;
  BlockMapper block_mapper(
      dims,
      {internal::TensorBlockShapeType::kSkewedInnerDims, internal::random<size_t>(1, dims.TotalSize()), {0, 0, 0}});

  Index total_blocks = block_mapper.blockCount();
  Index block_index = internal::random<Index>(0, total_blocks - 1);
  auto block = block_mapper.blockDescriptor(block_index);
  DSizes<Index, NumDims> sizes = block.dimensions();

  auto strides = internal::strides<Layout>(dims);
  DSizes<Index, NumDims> offsets;

  // Compute offsets for the first block coefficient.
  Index index = block.offset();
  EIGEN_IF_CONSTEXPR (static_cast<int>(Layout) == static_cast<int>(ColMajor)) {
    for (int i = NumDims - 1; i > 0; --i) {
      const Index idx = index / strides[i];
      index -= idx * strides[i];
      offsets[i] = idx;
    }
  } else {
    for (int i = 0; i < NumDims - 1; ++i) {
      const Index idx = index / strides[i];
      index -= idx * strides[i];
      offsets[i] = idx;
    }
  }
  SetSkewedInnerBlockInnerOffset<Layout, NumDims>(offsets, index);

  return {offsets, sizes, block};
}

template <int NumDims>
static TensorBlockParams<NumDims> FixedSizeBlock(DSizes<Index, NumDims> dims) {
  DSizes<Index, NumDims> offsets;
  for (int i = 0; i < NumDims; ++i) offsets[i] = 0;

  return {offsets, dims, TensorBlockDescriptor<NumDims, Index>(0, dims)};
}

inline Eigen::IndexList<Index, Eigen::type2index<1>> NByOne(Index n) {
  Eigen::IndexList<Index, Eigen::type2index<1>> ret;
  ret.set(0, n);
  return ret;
}
inline Eigen::IndexList<Eigen::type2index<1>, Index> OneByM(Index m) {
  Eigen::IndexList<Eigen::type2index<1>, Index> ret;
  ret.set(1, m);
  return ret;
}

// -------------------------------------------------------------------------- //
// Verify that block expression evaluation produces the same result as a
// TensorSliceOp (reading a tensor block is same to taking a tensor slice).

template <typename T, int NumDims, int Layout, typename Expression, typename GenBlockParams>
static void VerifyBlockEvaluator(Expression expr, GenBlockParams gen_block) {
  using Device = DefaultDevice;
  auto d = Device();

  // Scratch memory allocator for block evaluation.
  typedef internal::TensorBlockScratchAllocator<Device> TensorBlockScratch;
  TensorBlockScratch scratch(d);

  // TensorEvaluator is needed to produce tensor blocks of the expression.
  auto eval = TensorEvaluator<const decltype(expr), Device>(expr, d);
  eval.evalSubExprsIfNeeded(nullptr);

  // Choose a random offsets, sizes and TensorBlockDescriptor.
  TensorBlockParams<NumDims> block_params = gen_block();

  // Evaluate TensorBlock expression into a tensor.
  Tensor<T, NumDims, Layout> block(block_params.desc.dimensions());

  // Dimensions for the potential destination buffer.
  DSizes<Index, NumDims> dst_dims;
  if (internal::random<bool>()) {
    dst_dims = block_params.desc.dimensions();
  } else {
    for (int i = 0; i < NumDims; ++i) {
      Index extent = internal::random<Index>(0, 5);
      dst_dims[i] = block_params.desc.dimension(i) + extent;
    }
  }

  // Maybe use this tensor as a block desc destination.
  Tensor<T, NumDims, Layout> dst(dst_dims);
  dst.setZero();
  if (internal::random<bool>()) {
    block_params.desc.template AddDestinationBuffer<Layout>(dst.data(), internal::strides<Layout>(dst.dimensions()));
  }

  const bool root_of_expr = internal::random<bool>();
  auto tensor_block = eval.block(block_params.desc, scratch, root_of_expr);

  if (tensor_block.kind() == internal::TensorBlockKind::kMaterializedInOutput) {
    // Copy data from destination buffer.
    if (dimensions_match(dst.dimensions(), block.dimensions())) {
      block = dst;
    } else {
      DSizes<Index, NumDims> offsets;
      for (int i = 0; i < NumDims; ++i) offsets[i] = 0;
      block = dst.slice(offsets, block.dimensions());
    }

  } else {
    // Assign to block from expression.
    auto b_expr = tensor_block.expr();

    // We explicitly disable vectorization and tiling, to run a simple coefficient
    // wise assignment loop, because it's very simple and should be correct.
    using BlockAssign = TensorAssignOp<decltype(block), const decltype(b_expr)>;
    using BlockExecutor = TensorExecutor<const BlockAssign, Device, false, internal::TiledEvaluation::Off>;
    BlockExecutor::run(BlockAssign(block, b_expr), d);
  }

  // Cleanup temporary buffers owned by a tensor block.
  tensor_block.cleanup();

  // Compute a Tensor slice corresponding to a Tensor block.
  Tensor<T, NumDims, Layout> slice(block_params.desc.dimensions());
  auto s_expr = expr.slice(block_params.offsets, block_params.sizes);

  // Explicitly use coefficient assignment to evaluate slice expression.
  using SliceAssign = TensorAssignOp<decltype(slice), const decltype(s_expr)>;
  using SliceExecutor = TensorExecutor<const SliceAssign, Device, false, internal::TiledEvaluation::Off>;
  SliceExecutor::run(SliceAssign(slice, s_expr), d);

  // Tensor block and tensor slice must be the same.
  for (Index i = 0; i < block.dimensions().TotalSize(); ++i) {
    VERIFY_IS_EQUAL(block.coeff(i), slice.coeff(i));
  }

  // Release evaluator-owned temporaries (e.g. the materialized buffer that
  // TensorScan / TensorFFT allocate during evalSubExprsIfNeeded).
  eval.cleanup();
}

// -------------------------------------------------------------------------- //

template <typename T, int NumDims, int Layout>
static void test_eval_tensor_block() {
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(10, 20);
  Tensor<T, NumDims, Layout> input(dims);
  input.setRandom();

  // Identity tensor expression transformation.
  VerifyBlockEvaluator<T, NumDims, Layout>(input, [&dims]() { return RandomBlock<Layout>(dims, 1, 10); });
}

template <typename T, int NumDims, int Layout>
static void test_eval_tensor_unary_expr_block() {
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(10, 20);
  Tensor<T, NumDims, Layout> input(dims);
  input.setRandom();

  VerifyBlockEvaluator<T, NumDims, Layout>(input.abs(), [&dims]() { return RandomBlock<Layout>(dims, 1, 10); });
}

template <typename T, int NumDims, int Layout>
static void test_eval_tensor_binary_expr_block() {
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(10, 20);
  Tensor<T, NumDims, Layout> lhs(dims), rhs(dims);
  setRandomForBinaryProduct(lhs);
  setRandomForBinaryProduct(rhs);

  VerifyBlockEvaluator<T, NumDims, Layout>(lhs * rhs, [&dims]() { return RandomBlock<Layout>(dims, 1, 10); });
}

template <typename T, int NumDims, int Layout>
static void test_eval_tensor_binary_with_unary_expr_block() {
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(10, 20);
  Tensor<T, NumDims, Layout> lhs(dims), rhs(dims);
  lhs.setRandom();
  rhs.setRandom();

  VerifyBlockEvaluator<T, NumDims, Layout>((lhs.abs() + rhs.abs()).sqrt(),
                                           [&dims]() { return RandomBlock<Layout>(dims, 1, 10); });
}

template <typename T, int NumDims, int Layout>
static void test_eval_tensor_broadcast() {
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(1, 10);
  Tensor<T, NumDims, Layout> input(dims);
  input.setRandom();

  DSizes<Index, NumDims> bcast = RandomDims<NumDims>(1, 5);

  DSizes<Index, NumDims> bcasted_dims;
  for (int i = 0; i < NumDims; ++i) bcasted_dims[i] = dims[i] * bcast[i];

  VerifyBlockEvaluator<T, NumDims, Layout>(input.broadcast(bcast),
                                           [&bcasted_dims]() { return SkewedInnerBlock<Layout>(bcasted_dims); });

  VerifyBlockEvaluator<T, NumDims, Layout>(input.broadcast(bcast),
                                           [&bcasted_dims]() { return RandomBlock<Layout>(bcasted_dims, 5, 10); });

  VerifyBlockEvaluator<T, NumDims, Layout>(input.broadcast(bcast),
                                           [&bcasted_dims]() { return FixedSizeBlock(bcasted_dims); });

  // Check that desc.destination() memory is not shared between two broadcast
  // materializations.
  Tensor<T, NumDims, Layout> product_input(dims);
  setRandomForBinaryProduct(product_input);
  VerifyBlockEvaluator<T, NumDims, Layout>(product_input.broadcast(bcast) * product_input.abs().broadcast(bcast),
                                           [&bcasted_dims]() { return SkewedInnerBlock<Layout>(bcasted_dims); });
}

template <typename T, int NumDims, int Layout>
static void test_eval_tensor_scan() {
  // Scan eagerly materializes m_output during evalSubExprsIfNeeded; this
  // exercises the block() wrapper around that buffer.
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(4, 12);
  Tensor<T, NumDims, Layout> input(dims);
  input.setRandom();

  constexpr Index axis = NumDims == 1 ? 0 : NumDims / 2;

  VerifyBlockEvaluator<T, NumDims, Layout>(input.cumsum(axis), [&dims]() { return RandomBlock<Layout>(dims, 1, 5); });

  VerifyBlockEvaluator<T, NumDims, Layout>(input.cumsum(axis), [&dims]() { return FixedSizeBlock(dims); });
}

template <typename T, int NumDims, int Layout>
static void test_eval_tensor_fft() {
  // FFT eagerly materializes m_data during evalSubExprsIfNeeded; this
  // exercises the block() wrapper around that buffer. Use RealPart so
  // the output type matches the input type and the harness's slice
  // comparison stays straightforward.
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(4, 12);
  Tensor<T, NumDims, Layout> input(dims);
  input.setRandom();

  Eigen::array<int, 1> fft_dims = {0};

  VerifyBlockEvaluator<T, NumDims, Layout>(input.template fft<RealPart, FFT_FORWARD>(fft_dims),
                                           [&dims]() { return RandomBlock<Layout>(dims, 1, 5); });

  VerifyBlockEvaluator<T, NumDims, Layout>(input.template fft<RealPart, FFT_FORWARD>(fft_dims),
                                           [&dims]() { return FixedSizeBlock(dims); });
}

// A destination buffer whose strides do not match the block's dense strides
// cannot back a tensor expression, but it can still absorb the block when the
// block is the root of the expression tree, because nothing reads the result
// back through `expr()`. Verify that such a destination is used rather than
// routed through scratch, and that the values landing in it are correct.
template <typename T, int NumDims, int Layout, typename Expression>
static void VerifyStridedDestinationAtRoot(Expression expr, const DSizes<Index, NumDims>& expr_dims) {
  using Device = DefaultDevice;
  using Desc = TensorBlockDescriptor<NumDims, Index>;
  auto d = Device();

  internal::TensorBlockScratchAllocator<Device> scratch(d);

  auto eval = TensorEvaluator<const Expression, Device>(expr, d);
  eval.evalSubExprsIfNeeded(nullptr);

  // A block strictly inside the expression, and a destination strictly larger
  // than the block, so that the destination strides are dense in no dimension
  // but the innermost.
  DSizes<Index, NumDims> offsets, sizes, dst_dims;
  for (int i = 0; i < NumDims; ++i) {
    offsets[i] = 1;
    sizes[i] = expr_dims[i] - 2;
    dst_dims[i] = sizes[i] + 2;
  }

  Index offset = 0;
  const DSizes<Index, NumDims> expr_strides = internal::strides<Layout>(expr_dims);
  for (int i = 0; i < NumDims; ++i) offset += expr_strides[i] * offsets[i];

  Desc desc(offset, sizes);
  Tensor<T, NumDims, Layout> dst(dst_dims);
  dst.setZero();
  desc.template AddDestinationBuffer<Layout>(dst.data(), internal::strides<Layout>(dst.dimensions()));
  VERIFY(desc.destination().kind() == Desc::DestinationBuffer::kStrided);

  auto tensor_block = eval.block(desc, scratch, /*root_of_expr_ast=*/true);
  VERIFY(tensor_block.kind() == internal::TensorBlockKind::kMaterializedInOutput);
  tensor_block.cleanup();

  DSizes<Index, NumDims> zeros;
  for (int i = 0; i < NumDims; ++i) zeros[i] = 0;
  Tensor<T, NumDims, Layout> block = dst.slice(zeros, sizes);

  // Reference: the same slice evaluated coefficient-wise.
  Tensor<T, NumDims, Layout> slice(sizes);
  auto s_expr = expr.slice(offsets, sizes);
  using SliceAssign = TensorAssignOp<decltype(slice), const decltype(s_expr)>;
  using SliceExecutor = TensorExecutor<const SliceAssign, Device, false, internal::TiledEvaluation::Off>;
  SliceExecutor::run(SliceAssign(slice, s_expr), d);

  for (Index i = 0; i < sizes.TotalSize(); ++i) {
    VERIFY_IS_EQUAL(block.coeff(i), slice.coeff(i));
  }

  eval.cleanup();
}

template <typename T, int NumDims, int Layout>
static void test_eval_tensor_layout_swap() {
  // The swap_layout expression has the opposite layout of its operand. Build
  // the input with the opposite layout and assert the block evaluator on the
  // resulting expression matches the slice-based reference.
  constexpr int InputLayout = (Layout == ColMajor) ? RowMajor : ColMajor;
  DSizes<Index, NumDims> input_dims = RandomDims<NumDims>(10, 20);
  Tensor<T, NumDims, InputLayout> input(input_dims);
  setRandomForBinaryProduct(input);

  DSizes<Index, NumDims> swapped_dims;
  for (int i = 0; i < NumDims; ++i) {
    swapped_dims[i] = input_dims[NumDims - 1 - i];
  }

  VerifyBlockEvaluator<T, NumDims, Layout>(input.swap_layout(),
                                           [&swapped_dims]() { return RandomBlock<Layout>(swapped_dims, 1, 10); });

  VerifyBlockEvaluator<T, NumDims, Layout>(input.swap_layout(),
                                           [&swapped_dims]() { return FixedSizeBlock(swapped_dims); });

  // An operand without a raw buffer: the block request is forwarded to the
  // operand with reversed dimensions. A shuffle serves materialized blocks
  // (re-wrapped without a copy), while a cwise expression on top of it serves
  // lazy blocks (materialized by the layout swap itself).
  DSizes<Index, NumDims> reversing_shuffle;
  for (int i = 0; i < NumDims; ++i) reversing_shuffle[i] = NumDims - 1 - i;

  VerifyBlockEvaluator<T, NumDims, Layout>(input.shuffle(reversing_shuffle).swap_layout(),
                                           [&input_dims]() { return RandomBlock<Layout>(input_dims, 1, 10); });

  VerifyBlockEvaluator<T, NumDims, Layout>(input.shuffle(reversing_shuffle).swap_layout(),
                                           [&input_dims]() { return FixedSizeBlock(input_dims); });

  VerifyBlockEvaluator<T, NumDims, Layout>(
      (input.shuffle(reversing_shuffle) * input.shuffle(reversing_shuffle)).swap_layout(),
      [&input_dims]() { return RandomBlock<Layout>(input_dims, 1, 10); });

  // Both forwarding branches must take a strided destination at the root of
  // the expression tree instead of paying for a scratch round trip. A
  // one-dimensional destination is never strided, so it has nothing to pin.
  EIGEN_IF_CONSTEXPR (NumDims > 1) {
    VerifyStridedDestinationAtRoot<T, NumDims, Layout>(input.shuffle(reversing_shuffle).swap_layout(), input_dims);
    VerifyStridedDestinationAtRoot<T, NumDims, Layout>(
        (input.shuffle(reversing_shuffle) * input.shuffle(reversing_shuffle)).swap_layout(), input_dims);
  }
}

// Regression for the original failure mode this MR fixes: TensorPaddingOp's
// BlockAccess gates on m_impl.RawAccess (true for Scan/FFT/LayoutSwap/Contraction)
// and its getResourceRequirements() unconditionally calls the operand's. Before
// the operand-side block plumbing was added, instantiating the executor's
// Tiling=On path through these compositions failed to compile. Driving block()
// over the composed expressions exercises both sides of that compile path.
template <typename T, int NumDims, int Layout>
static void test_eval_composed_block_ops() {
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(4, 8);
  array<std::pair<Index, Index>, NumDims> paddings;
  DSizes<Index, NumDims> padded_dims;
  for (int i = 0; i < NumDims; ++i) {
    paddings[i] = std::make_pair(1, 2);
    padded_dims[i] = dims[i] + 3;
  }

  constexpr Index axis = NumDims == 1 ? 0 : NumDims / 2;

  Tensor<T, NumDims, Layout> input(dims);
  input.setRandom();

  // cumsum(...).pad(...) — TensorScan + TensorPadding.
  VerifyBlockEvaluator<T, NumDims, Layout>(input.cumsum(axis).pad(paddings),
                                           [&padded_dims]() { return RandomBlock<Layout>(padded_dims, 1, 5); });

  // swap_layout().pad(...) — operand built with the opposite layout so the
  // composed expression evaluates in the test's Layout.
  constexpr int InputLayout = (Layout == ColMajor) ? RowMajor : ColMajor;
  Tensor<T, NumDims, InputLayout> swap_input(dims);
  swap_input.setRandom();
  DSizes<Index, NumDims> swap_padded_dims;
  for (int i = 0; i < NumDims; ++i) {
    swap_padded_dims[i] = dims[NumDims - 1 - i] + 3;
  }
  VerifyBlockEvaluator<T, NumDims, Layout>(swap_input.swap_layout().pad(paddings), [&swap_padded_dims]() {
    return RandomBlock<Layout>(swap_padded_dims, 1, 5);
  });
}

// 2D-specific regression: contract(...).pad(...) hit the same composition bug
// because TensorContraction has RawAccess=true but lacked getResourceRequirements().
template <typename T, int Layout>
static void test_eval_contract_pad_composition() {
  Tensor<T, 2, Layout> A(8, 6);
  Tensor<T, 2, Layout> B(6, 4);
  A.setRandom();
  B.setRandom();

  Eigen::array<IndexPair<Index>, 1> contract_dims = {IndexPair<Index>(1, 0)};
  array<std::pair<Index, Index>, 2> paddings = {std::pair<Index, Index>{1, 1}, std::pair<Index, Index>{2, 2}};
  DSizes<Index, 2> padded_dims(8 + 2, 4 + 4);

  VerifyBlockEvaluator<T, 2, Layout>(A.contract(B, contract_dims).pad(paddings),
                                     [&padded_dims]() { return RandomBlock<Layout>(padded_dims, 1, 5); });
}

template <typename T, int NumDims, int Layout>
static void test_eval_tensor_reshape() {
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(1, 10);

  DSizes<Index, NumDims> shuffled = dims;
  std::shuffle(&shuffled[0], &shuffled[NumDims - 1], std::mt19937(g_seed));

  Tensor<T, NumDims, Layout> input(dims);
  input.setRandom();

  VerifyBlockEvaluator<T, NumDims, Layout>(input.reshape(shuffled),
                                           [&shuffled]() { return RandomBlock<Layout>(shuffled, 1, 10); });

  VerifyBlockEvaluator<T, NumDims, Layout>(input.reshape(shuffled),
                                           [&shuffled]() { return SkewedInnerBlock<Layout>(shuffled); });
}

template <typename T, int NumDims, int Layout>
static void test_eval_tensor_cast() {
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(10, 20);
  Tensor<T, NumDims, Layout> input(dims);
  input.setRandom();

  VerifyBlockEvaluator<T, NumDims, Layout>(input.template cast<int>().template cast<T>(),
                                           [&dims]() { return RandomBlock<Layout>(dims, 1, 10); });
}

template <typename T, int NumDims, int Layout>
static void test_eval_tensor_ternary() {
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(10, 20);
  Tensor<T, NumDims, Layout> a(dims);
  Tensor<T, NumDims, Layout> b(dims);
  Tensor<T, NumDims, Layout> x(dims);
  // betainc requires a > 0, b > 0 and x in [0, 1].
  a.setRandom();
  b.setRandom();
  x.setRandom();
  a = a.abs() + a.constant(T(0.5));
  b = b.abs() + b.constant(T(0.5));
  x = (x + x.constant(T(1))) * x.constant(T(0.5));

  VerifyBlockEvaluator<T, NumDims, Layout>(Eigen::betainc(a, b, x),
                                           [&dims]() { return RandomBlock<Layout>(dims, 1, 20); });
  VerifyBlockEvaluator<T, NumDims, Layout>(Eigen::betainc(a, b, x), [&dims]() { return FixedSizeBlock(dims); });
}

template <typename T, int NumDims, int Layout>
static void test_eval_tensor_nullary() {
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(10, 20);
  Tensor<T, NumDims, Layout> input(dims);
  input.setRandom();

  VerifyBlockEvaluator<T, NumDims, Layout>(input.constant(T(7)),
                                           [&dims]() { return RandomBlock<Layout>(dims, 1, 10); });
  VerifyBlockEvaluator<T, NumDims, Layout>(input.constant(T(7)), [&dims]() { return FixedSizeBlock(dims); });

  // Nullary leaf composed under a block-capable binary expression.
  VerifyBlockEvaluator<T, NumDims, Layout>(input * input.constant(T(1)),
                                           [&dims]() { return RandomBlock<Layout>(dims, 1, 10); });
}

template <typename T, int NumDims, int Layout>
static void test_eval_tensor_random_nullary() {
  // Each draw is a pure function of the generator's seed and the coefficient's
  // tensor-linear index, so a block must reproduce the coefficient evaluation
  // of the same expression exactly.
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(10, 20);
  Tensor<T, NumDims, Layout> input(dims);
  auto expr = input.random();

  VerifyBlockEvaluator<T, NumDims, Layout>(expr, [&dims]() { return RandomBlock<Layout>(dims, 5, 10); });
  VerifyBlockEvaluator<T, NumDims, Layout>(expr, [&dims]() { return FixedSizeBlock(dims); });

  // The full tensor always has >= 10 elements; all of them collapsing to one
  // value means the generator was not actually invoked per element.
  Tensor<T, NumDims, Layout> full = expr;
  bool all_equal = true;
  for (Index i = 0; i < full.size(); ++i) {
    VERIFY(full.coeff(i) >= T(0) && full.coeff(i) < T(1));
    all_equal = all_equal && (full.coeff(i) == full.coeff(0));
  }
  VERIFY(!all_equal);
}

// A zero-argument operator() alone does not imply index independence: when an
// indexed overload also exists, nullary_wrapper dispatches to it, so blocks
// must be materialized with true tensor-linear indices rather than served
// lazily with indices restarting at each block origin.
template <typename T, int NumDims, int Layout>
static void test_eval_tensor_indexed_nullary() {
  struct BothOverloads {
    T operator()() const { return T(-1); }
    T operator()(Index i) const { return static_cast<T>(i); }
  };

  DSizes<Index, NumDims> dims = RandomDims<NumDims>(10, 20);
  Tensor<T, NumDims, Layout> input(dims);

  VerifyBlockEvaluator<T, NumDims, Layout>(input.nullaryExpr(BothOverloads()),
                                           [&dims]() { return RandomBlock<Layout>(dims, 1, 10); });
  VerifyBlockEvaluator<T, NumDims, Layout>(input.nullaryExpr(BothOverloads()),
                                           [&dims]() { return FixedSizeBlock(dims); });
}

template <typename T, int NumDims, int Layout>
static void test_eval_tensor_select() {
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(10, 20);
  Tensor<T, NumDims, Layout> lhs(dims);
  Tensor<T, NumDims, Layout> rhs(dims);
  Tensor<bool, NumDims, Layout> cond(dims);
  lhs.setRandom();
  rhs.setRandom();
  cond.setRandom();

  VerifyBlockEvaluator<T, NumDims, Layout>(cond.select(lhs, rhs),
                                           [&dims]() { return RandomBlock<Layout>(dims, 1, 20); });
}

template <typename T, int NumDims, int Layout>
static void test_eval_tensor_padding() {
  constexpr int inner_dim = Layout == static_cast<int>(ColMajor) ? 0 : NumDims - 1;

  DSizes<Index, NumDims> dims = RandomDims<NumDims>(10, 20);
  Tensor<T, NumDims, Layout> input(dims);
  input.setRandom();

  DSizes<Index, NumDims> pad_before = RandomDims<NumDims>(0, 4);
  DSizes<Index, NumDims> pad_after = RandomDims<NumDims>(0, 4);
  array<std::pair<Index, Index>, NumDims> paddings;
  for (int i = 0; i < NumDims; ++i) {
    paddings[i] = std::make_pair(pad_before[i], pad_after[i]);
  }

  // Test squeezing reads from inner dim.
  if (internal::random<bool>()) {
    pad_before[inner_dim] = 0;
    pad_after[inner_dim] = 0;
    paddings[inner_dim] = std::make_pair(0, 0);
  }

  DSizes<Index, NumDims> padded_dims;
  for (int i = 0; i < NumDims; ++i) {
    padded_dims[i] = dims[i] + pad_before[i] + pad_after[i];
  }

  VerifyBlockEvaluator<T, NumDims, Layout>(input.pad(paddings),
                                           [&padded_dims]() { return FixedSizeBlock(padded_dims); });

  VerifyBlockEvaluator<T, NumDims, Layout>(input.pad(paddings),
                                           [&padded_dims]() { return RandomBlock<Layout>(padded_dims, 1, 10); });

  VerifyBlockEvaluator<T, NumDims, Layout>(input.pad(paddings),
                                           [&padded_dims]() { return SkewedInnerBlock<Layout>(padded_dims); });
}

template <typename T, int NumDims, int Layout>
static void test_eval_tensor_chipping() {
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(10, 20);
  Tensor<T, NumDims, Layout> input(dims);
  input.setRandom();

  Index chip_dim = internal::random<int>(0, NumDims - 1);
  Index chip_offset = internal::random<Index>(0, dims[chip_dim] - 2);

  DSizes<Index, NumDims - 1> chipped_dims;
  for (Index i = 0; i < chip_dim; ++i) {
    chipped_dims[i] = dims[i];
  }
  for (Index i = chip_dim + 1; i < NumDims; ++i) {
    chipped_dims[i - 1] = dims[i];
  }

  // Block buffer forwarding.
  VerifyBlockEvaluator<T, NumDims - 1, Layout>(input.chip(chip_offset, chip_dim),
                                               [&chipped_dims]() { return FixedSizeBlock(chipped_dims); });

  VerifyBlockEvaluator<T, NumDims - 1, Layout>(input.chip(chip_offset, chip_dim),
                                               [&chipped_dims]() { return RandomBlock<Layout>(chipped_dims, 1, 10); });

  // Block expression assignment.
  VerifyBlockEvaluator<T, NumDims - 1, Layout>(input.abs().chip(chip_offset, chip_dim),
                                               [&chipped_dims]() { return FixedSizeBlock(chipped_dims); });

  VerifyBlockEvaluator<T, NumDims - 1, Layout>(input.abs().chip(chip_offset, chip_dim),
                                               [&chipped_dims]() { return RandomBlock<Layout>(chipped_dims, 1, 10); });
}

template <typename T, int NumDims, int Layout>
static void test_eval_tensor_concatenation() {
  DSizes<Index, NumDims> lhs_dims = RandomDims<NumDims>(5, 12);
  DSizes<Index, NumDims> rhs_dims = lhs_dims;

  const Index axis = internal::random<int>(0, NumDims - 1);
  rhs_dims[axis] = internal::random<Index>(1, 10);

  Tensor<T, NumDims, Layout> lhs(lhs_dims);
  Tensor<T, NumDims, Layout> rhs(rhs_dims);
  lhs.setRandom();
  rhs.setRandom();

  DSizes<Index, NumDims> out_dims = lhs_dims;
  out_dims[axis] = lhs_dims[axis] + rhs_dims[axis];

  VerifyBlockEvaluator<T, NumDims, Layout>(lhs.concatenate(rhs, axis),
                                           [&out_dims]() { return FixedSizeBlock(out_dims); });

  VerifyBlockEvaluator<T, NumDims, Layout>(lhs.concatenate(rhs, axis),
                                           [&out_dims]() { return RandomBlock<Layout>(out_dims, 1, 5); });

  VerifyBlockEvaluator<T, NumDims, Layout>(lhs.concatenate(rhs, axis),
                                           [&out_dims]() { return SkewedInnerBlock<Layout>(out_dims); });
}

template <typename T, int NumDims>
struct SimpleTensorGenerator {
  T operator()(const array<Index, NumDims>& coords) const {
    T result = static_cast<T>(0);
    for (int i = 0; i < NumDims; ++i) {
      result += static_cast<T>((i + 1) * coords[i]);
    }
    return result;
  }
};

// Boolean specialization to avoid -Wint-in-bool-context warnings on GCC.
template <int NumDims>
struct SimpleTensorGenerator<bool, NumDims> {
  bool operator()(const array<Index, NumDims>& coords) const {
    bool result = false;
    for (int i = 0; i < NumDims; ++i) {
      result = result != (coords[i] != 0);
    }
    return result;
  }
};

template <typename T, int NumDims, int Layout>
static void test_eval_tensor_generator() {
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(10, 20);
  Tensor<T, NumDims, Layout> input(dims);
  input.setRandom();

  auto generator = SimpleTensorGenerator<T, NumDims>();

  VerifyBlockEvaluator<T, NumDims, Layout>(input.generate(generator), [&dims]() { return FixedSizeBlock(dims); });

  VerifyBlockEvaluator<T, NumDims, Layout>(input.generate(generator),
                                           [&dims]() { return RandomBlock<Layout>(dims, 1, 10); });
}

template <typename T, int NumDims, int Layout>
static void test_eval_tensor_reverse() {
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(10, 20);
  Tensor<T, NumDims, Layout> input(dims);
  input.setRandom();

  // Randomly reverse dimensions.
  Eigen::DSizes<bool, NumDims> reverse;
  for (int i = 0; i < NumDims; ++i) reverse[i] = internal::random<bool>();

  VerifyBlockEvaluator<T, NumDims, Layout>(input.reverse(reverse), [&dims]() { return FixedSizeBlock(dims); });

  VerifyBlockEvaluator<T, NumDims, Layout>(input.reverse(reverse),
                                           [&dims]() { return RandomBlock<Layout>(dims, 1, 10); });
}

template <typename T, int NumDims, int Layout>
static void test_eval_tensor_slice() {
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(10, 20);
  Tensor<T, NumDims, Layout> input(dims);
  input.setRandom();

  // Pick a random slice of an input tensor.
  DSizes<Index, NumDims> slice_start = RandomDims<NumDims>(5, 10);
  DSizes<Index, NumDims> slice_size = RandomDims<NumDims>(5, 10);

  // Make sure that slice start + size do not overflow tensor dims.
  for (int i = 0; i < NumDims; ++i) {
    slice_start[i] = numext::mini(dims[i] - 1, slice_start[i]);
    slice_size[i] = numext::mini(slice_size[i], dims[i] - slice_start[i]);
  }

  VerifyBlockEvaluator<T, NumDims, Layout>(input.slice(slice_start, slice_size),
                                           [&slice_size]() { return FixedSizeBlock(slice_size); });

  VerifyBlockEvaluator<T, NumDims, Layout>(input.slice(slice_start, slice_size),
                                           [&slice_size]() { return RandomBlock<Layout>(slice_size, 1, 10); });
}

// Exercise the block evaluator for bool slices as a sub-expression of a
// block-aware parent op at sizes spanning Packet16b (16-lane bool packet)
// boundaries. Before the BlockAccess fix for bool slicing, the parent op's
// BlockAccess would be forced to false and this composition would never
// dispatch through the block path.
template <int NumDims, int Layout>
static void test_eval_tensor_slice_bool_composite() {
  const Index boundary_sizes[] = {15, 16, 17, 31, 32, 33, 47, 48, 49};
  for (Index sz : boundary_sizes) {
    DSizes<Index, NumDims> dims;
    for (int i = 0; i < NumDims; ++i) dims[i] = sz;

    Tensor<bool, NumDims, Layout> lhs(dims);
    Tensor<bool, NumDims, Layout> rhs(dims);
    lhs.setRandom();
    rhs.setRandom();

    // Slice skewed off the boundary so the block layout straddles packet
    // boundaries of the underlying tensor.
    DSizes<Index, NumDims> slice_start;
    DSizes<Index, NumDims> slice_size;
    for (int i = 0; i < NumDims; ++i) {
      slice_start[i] = sz >= 2 ? 1 : 0;
      slice_size[i] = sz - slice_start[i];
    }

    auto expr = lhs.slice(slice_start, slice_size) && rhs.slice(slice_start, slice_size);

    VerifyBlockEvaluator<bool, NumDims, Layout>(expr, [&slice_size]() { return FixedSizeBlock(slice_size); });
    VerifyBlockEvaluator<bool, NumDims, Layout>(expr,
                                                [&slice_size, sz]() { return RandomBlock<Layout>(slice_size, 1, sz); });
  }
}

template <typename T, int NumDims, int Layout>
static void test_eval_tensor_shuffle() {
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(5, 15);
  Tensor<T, NumDims, Layout> input(dims);
  input.setRandom();

  DSizes<Index, NumDims> shuffle;
  for (int i = 0; i < NumDims; ++i) shuffle[i] = i;

  do {
    DSizes<Index, NumDims> shuffled_dims;
    for (int i = 0; i < NumDims; ++i) shuffled_dims[i] = dims[shuffle[i]];

    VerifyBlockEvaluator<T, NumDims, Layout>(input.shuffle(shuffle),
                                             [&shuffled_dims]() { return FixedSizeBlock(shuffled_dims); });

    VerifyBlockEvaluator<T, NumDims, Layout>(input.shuffle(shuffle),
                                             [&shuffled_dims]() { return RandomBlock<Layout>(shuffled_dims, 1, 5); });

    break;

  } while (std::next_permutation(&shuffle[0], &shuffle[0] + NumDims));
}

template <typename T, int NumDims, int Layout>
static void test_eval_tensor_inflation() {
  // Keep the input small: inflation multiplies every dimension by its stride.
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(2, 5);
  Tensor<T, NumDims, Layout> input(dims);
  input.setRandom();

  DSizes<Index, NumDims> strides;
  DSizes<Index, NumDims> inflated_dims;
  for (int i = 0; i < NumDims; ++i) {
    strides[i] = internal::random<Index>(1, 3);
    inflated_dims[i] = (dims[i] - 1) * strides[i] + 1;
  }

  VerifyBlockEvaluator<T, NumDims, Layout>(input.inflate(strides),
                                           [&inflated_dims]() { return RandomBlock<Layout>(inflated_dims, 1, 10); });

  VerifyBlockEvaluator<T, NumDims, Layout>(input.inflate(strides),
                                           [&inflated_dims]() { return SkewedInnerBlock<Layout>(inflated_dims); });

  VerifyBlockEvaluator<T, NumDims, Layout>(input.inflate(strides),
                                           [&inflated_dims]() { return FixedSizeBlock(inflated_dims); });
}

template <typename T, int NumDims, int Layout>
static void test_eval_tensor_strided_slice() {
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(10, 20);
  Tensor<T, NumDims, Layout> input(dims);
  input.setRandom();

  // Sweep unit, dilated, and negative strides; start/stop are chosen in-range
  // so that clamping does not kick in.
  const Index stride_choices[] = {1, 2, 3, -1, -2};
  for (const Index s : stride_choices) {
    DSizes<Index, NumDims> start, stop, strides, out_dims;
    for (int i = 0; i < NumDims; ++i) {
      strides[i] = s;
      if (s > 0) {
        start[i] = 1;
        stop[i] = dims[i] - 1;
      } else {
        start[i] = dims[i] - 2;
        stop[i] = 0;
      }
      const Index interval = stop[i] - start[i];
      out_dims[i] = interval / strides[i] + (interval % strides[i] != 0 ? 1 : 0);
    }

    VerifyBlockEvaluator<T, NumDims, Layout>(input.stridedSlice(start, stop, strides),
                                             [&out_dims]() { return RandomBlock<Layout>(out_dims, 1, 5); });
    VerifyBlockEvaluator<T, NumDims, Layout>(input.stridedSlice(start, stop, strides),
                                             [&out_dims]() { return FixedSizeBlock(out_dims); });
  }

  // The identity strided slice serves blocks straight from the input buffer.
  DSizes<Index, NumDims> zeros, ones;
  for (int i = 0; i < NumDims; ++i) {
    zeros[i] = 0;
    ones[i] = 1;
  }
  VerifyBlockEvaluator<T, NumDims, Layout>(input.stridedSlice(zeros, dims, ones),
                                           [&dims]() { return RandomBlock<Layout>(dims, 1, 5); });
}

template <typename T, int Layout>
static void test_eval_tensor_reshape_with_bcast() {
  Index dim = internal::random<Index>(1, 100);

  Tensor<T, 2, Layout> lhs(1, dim);
  Tensor<T, 2, Layout> rhs(dim, 1);
  lhs.setRandom();
  rhs.setRandom();

  auto reshapeLhs = NByOne(dim);
  auto reshapeRhs = OneByM(dim);

  auto bcastLhs = OneByM(dim);
  auto bcastRhs = NByOne(dim);

  DSizes<Index, 2> dims(dim, dim);

  VerifyBlockEvaluator<T, 2, Layout>(
      lhs.reshape(reshapeLhs).broadcast(bcastLhs) * rhs.reshape(reshapeRhs).broadcast(bcastRhs),
      [dims]() { return SkewedInnerBlock<Layout, 2>(dims); });
}

template <typename T, int Layout>
static void test_eval_tensor_forced_eval() {
  Index dim = internal::random<Index>(1, 100);

  Tensor<T, 2, Layout> lhs(dim, 1);
  Tensor<T, 2, Layout> rhs(1, dim);
  lhs.setRandom();
  rhs.setRandom();

  auto bcastLhs = OneByM(dim);
  auto bcastRhs = NByOne(dim);

  DSizes<Index, 2> dims(dim, dim);

  VerifyBlockEvaluator<T, 2, Layout>((lhs.broadcast(bcastLhs) * rhs.broadcast(bcastRhs)).eval().reshape(dims),
                                     [dims]() { return SkewedInnerBlock<Layout, 2>(dims); });

  VerifyBlockEvaluator<T, 2, Layout>((lhs.broadcast(bcastLhs) * rhs.broadcast(bcastRhs)).eval().reshape(dims),
                                     [dims]() { return RandomBlock<Layout, 2>(dims, 1, 50); });
}

template <typename T, int Layout>
static void test_eval_tensor_chipping_of_bcast() {
  if (Layout != static_cast<int>(RowMajor)) return;

  Index dim0 = internal::random<Index>(1, 10);
  Index dim1 = internal::random<Index>(1, 10);
  Index dim2 = internal::random<Index>(1, 10);

  Tensor<T, 3, Layout> input(1, dim1, dim2);
  input.setRandom();

  Eigen::array<Index, 3> bcast = {{dim0, 1, 1}};
  DSizes<Index, 2> chipped_dims(dim0, dim2);

  VerifyBlockEvaluator<T, 2, Layout>(input.broadcast(bcast).chip(0, 1),
                                     [chipped_dims]() { return FixedSizeBlock(chipped_dims); });

  VerifyBlockEvaluator<T, 2, Layout>(input.broadcast(bcast).chip(0, 1),
                                     [chipped_dims]() { return SkewedInnerBlock<Layout, 2>(chipped_dims); });

  VerifyBlockEvaluator<T, 2, Layout>(input.broadcast(bcast).chip(0, 1),
                                     [chipped_dims]() { return RandomBlock<Layout, 2>(chipped_dims, 1, 5); });
}

template <typename T, int Layout>
static void test_eval_tensor_patch() {
  DSizes<Index, 3> dims = RandomDims<3>(4, 10);
  Tensor<T, 3, Layout> input(dims);
  input.setRandom();

  DSizes<Index, 3> patch_dims;
  Index num_patches = 1;
  for (int i = 0; i < 3; ++i) {
    patch_dims[i] = internal::random<Index>(1, dims[i]);
    num_patches *= (dims[i] - patch_dims[i] + 1);
  }

  DSizes<Index, 4> out_dims;
  if (static_cast<int>(Layout) == static_cast<int>(ColMajor)) {
    out_dims = DSizes<Index, 4>(patch_dims[0], patch_dims[1], patch_dims[2], num_patches);
  } else {
    out_dims = DSizes<Index, 4>(num_patches, patch_dims[0], patch_dims[1], patch_dims[2]);
  }

  VerifyBlockEvaluator<T, 4, Layout>(input.extract_patches(patch_dims),
                                     [&out_dims]() { return RandomBlock<Layout>(out_dims, 1, 10); });

  VerifyBlockEvaluator<T, 4, Layout>(input.extract_patches(patch_dims),
                                     [&out_dims]() { return SkewedInnerBlock<Layout>(out_dims); });

  VerifyBlockEvaluator<T, 4, Layout>(input.extract_patches(patch_dims),
                                     [&out_dims]() { return FixedSizeBlock(out_dims); });

  // Dispatch check: stride() serves neither blocks nor raw buffers, so this
  // pins down that the patch block path needs no capability bit from its
  // argument, only coeff().
  {
    const array<Index, 3> strides = {{2, 2, 2}};
    auto strided = input.stride(strides);
    typedef TensorEvaluator<const decltype(strided), DefaultDevice> StridedEval;
    static_assert(!StridedEval::BlockAccess && !StridedEval::RawAccess,
                  "stride() must stay a coeff()-only argument for this check to be meaningful");

    DSizes<Index, 3> strided_dims;
    DSizes<Index, 3> strided_patch_dims;
    Index strided_num_patches = 1;
    for (int i = 0; i < 3; ++i) {
      strided_dims[i] = (dims[i] - 1) / 2 + 1;
      strided_patch_dims[i] = internal::random<Index>(1, strided_dims[i]);
      strided_num_patches *= (strided_dims[i] - strided_patch_dims[i] + 1);
    }
    DSizes<Index, 4> strided_out_dims;
    if (static_cast<int>(Layout) == static_cast<int>(ColMajor)) {
      strided_out_dims =
          DSizes<Index, 4>(strided_patch_dims[0], strided_patch_dims[1], strided_patch_dims[2], strided_num_patches);
    } else {
      strided_out_dims =
          DSizes<Index, 4>(strided_num_patches, strided_patch_dims[0], strided_patch_dims[1], strided_patch_dims[2]);
    }

    auto strided_patch = strided.extract_patches(strided_patch_dims);
    typedef TensorEvaluator<const decltype(strided_patch), DefaultDevice> PatchEval;
    static_assert(PatchEval::BlockAccess && PatchEval::PreferBlockAccess,
                  "the patch block path must dispatch for a coeff()-only argument");
    VerifyBlockEvaluator<T, 4, Layout>(strided_patch,
                                       [&strided_out_dims]() { return RandomBlock<Layout>(strided_out_dims, 1, 10); });
  }
}

template <typename T, int Layout>
static void test_eval_tensor_image_patch() {
  static constexpr bool kColMajor = static_cast<int>(Layout) == static_cast<int>(ColMajor);
  const Index depth = internal::random<Index>(1, 5);
  const Index rows = internal::random<Index>(6, 12);
  const Index cols = internal::random<Index>(6, 12);
  const Index batch = internal::random<Index>(1, 3);

  DSizes<Index, 4> input_dims =
      kColMajor ? DSizes<Index, 4>(depth, rows, cols, batch) : DSizes<Index, 4>(batch, cols, rows, depth);
  Tensor<T, 4, Layout> input(input_dims);
  input.setRandom();

  const Index pr = internal::random<Index>(1, 3);
  const Index pc = internal::random<Index>(1, 3);

  auto make_out_dims = [&](Index out_r, Index out_c) {
    return kColMajor ? DSizes<Index, 5>(depth, pr, pc, out_r * out_c, batch)
                     : DSizes<Index, 5>(batch, out_r * out_c, pc, pr, depth);
  };

  // No padding, unit strides.
  {
    DSizes<Index, 5> out_dims = make_out_dims(rows - pr + 1, cols - pc + 1);
    VerifyBlockEvaluator<T, 5, Layout>(input.extract_image_patches(pr, pc, 1, 1, 1, 1, PADDING_VALID),
                                       [&out_dims]() { return RandomBlock<Layout>(out_dims, 1, 10); });
    VerifyBlockEvaluator<T, 5, Layout>(input.extract_image_patches(pr, pc, 1, 1, 1, 1, PADDING_VALID),
                                       [&out_dims]() { return SkewedInnerBlock<Layout>(out_dims); });
    VerifyBlockEvaluator<T, 5, Layout>(input.extract_image_patches(pr, pc, 1, 1, 1, 1, PADDING_VALID),
                                       [&out_dims]() { return FixedSizeBlock(out_dims); });
  }

  // Same padding with strided patch extraction (blocks contain padding runs).
  {
    DSizes<Index, 5> out_dims = make_out_dims(numext::div_ceil(rows, Index(2)), numext::div_ceil(cols, Index(2)));
    VerifyBlockEvaluator<T, 5, Layout>(input.extract_image_patches(pr, pc, 2, 2, 1, 1, PADDING_SAME),
                                       [&out_dims]() { return RandomBlock<Layout>(out_dims, 1, 10); });
    VerifyBlockEvaluator<T, 5, Layout>(input.extract_image_patches(pr, pc, 2, 2, 1, 1, PADDING_SAME),
                                       [&out_dims]() { return FixedSizeBlock(out_dims); });
  }

  // Dilated patches (non-unit in-row/in-col strides).
  {
    const Index pr_eff = pr + (pr - 1);
    const Index pc_eff = pc + (pc - 1);
    DSizes<Index, 5> out_dims = make_out_dims(rows - pr_eff + 1, cols - pc_eff + 1);
    VerifyBlockEvaluator<T, 5, Layout>(input.extract_image_patches(pr, pc, 1, 1, 2, 2, PADDING_VALID),
                                       [&out_dims]() { return RandomBlock<Layout>(out_dims, 1, 10); });
    VerifyBlockEvaluator<T, 5, Layout>(input.extract_image_patches(pr, pc, 1, 1, 2, 2, PADDING_VALID),
                                       [&out_dims]() { return FixedSizeBlock(out_dims); });
  }

  // Inflated (zero-interleaved) input. Only the long overload can set the
  // inflate strides, and they are what makes an in-range coordinate land
  // between samples, which is a branch none of the cases above reach.
  {
    const Index rows_eff = (rows - 1) * 2 + 1;
    const Index cols_eff = (cols - 1) * 2 + 1;
    DSizes<Index, 5> out_dims = make_out_dims(rows_eff - pr + 1, cols_eff - pc + 1);
    auto inflated = input.extract_image_patches(pr, pc, 1, 1, 1, 1, 2, 2, 0, 0, 0, 0, T(0));
    VerifyBlockEvaluator<T, 5, Layout>(inflated, [&out_dims]() { return RandomBlock<Layout>(out_dims, 1, 10); });
    VerifyBlockEvaluator<T, 5, Layout>(inflated, [&out_dims]() { return FixedSizeBlock(out_dims); });
  }

  // Dispatch check: stride() serves neither blocks nor raw buffers, so this
  // pins down that the block path needs no capability bit from its argument.
  {
    const array<Index, 4> strides = {{1, 2, 2, 1}};
    auto strided_patch = input.stride(strides).extract_image_patches(pr, pc, 1, 1, 1, 1, PADDING_VALID);
    typedef TensorEvaluator<const decltype(strided_patch), DefaultDevice> PatchEval;
    static_assert(PatchEval::BlockAccess && PatchEval::PreferBlockAccess,
                  "the image-patch block path must dispatch for a coeff()-only argument");
    const Index rows_s = (rows - 1) / 2 + 1;
    const Index cols_s = (cols - 1) / 2 + 1;
    DSizes<Index, 5> out_dims = make_out_dims(rows_s - pr + 1, cols_s - pc + 1);
    VerifyBlockEvaluator<T, 5, Layout>(strided_patch, [&out_dims]() { return RandomBlock<Layout>(out_dims, 1, 10); });
  }
}

template <typename T, int Layout>
static void test_eval_tensor_volume_patch() {
  static constexpr bool kColMajor = static_cast<int>(Layout) == static_cast<int>(ColMajor);
  const Index depth = internal::random<Index>(1, 4);
  const Index planes = internal::random<Index>(5, 8);
  const Index rows = internal::random<Index>(5, 8);
  const Index cols = internal::random<Index>(5, 8);
  const Index batch = internal::random<Index>(1, 2);

  DSizes<Index, 5> input_dims = kColMajor ? DSizes<Index, 5>(depth, planes, rows, cols, batch)
                                          : DSizes<Index, 5>(batch, cols, rows, planes, depth);
  Tensor<T, 5, Layout> input(input_dims);
  input.setRandom();

  const Index pp = internal::random<Index>(1, 3);
  const Index pr = internal::random<Index>(1, 3);
  const Index pc = internal::random<Index>(1, 3);

  auto make_out_dims = [&](Index out_p, Index out_r, Index out_c) {
    return kColMajor ? DSizes<Index, 6>(depth, pp, pr, pc, out_p * out_r * out_c, batch)
                     : DSizes<Index, 6>(batch, out_p * out_r * out_c, pc, pr, pp, depth);
  };

  // No padding, unit strides.
  {
    DSizes<Index, 6> out_dims = make_out_dims(planes - pp + 1, rows - pr + 1, cols - pc + 1);
    VerifyBlockEvaluator<T, 6, Layout>(input.extract_volume_patches(pp, pr, pc, 1, 1, 1, PADDING_VALID),
                                       [&out_dims]() { return RandomBlock<Layout>(out_dims, 1, 10); });
    VerifyBlockEvaluator<T, 6, Layout>(input.extract_volume_patches(pp, pr, pc, 1, 1, 1, PADDING_VALID),
                                       [&out_dims]() { return SkewedInnerBlock<Layout>(out_dims); });
    VerifyBlockEvaluator<T, 6, Layout>(input.extract_volume_patches(pp, pr, pc, 1, 1, 1, PADDING_VALID),
                                       [&out_dims]() { return FixedSizeBlock(out_dims); });
  }

  // Same padding with strided patch extraction (blocks contain padding runs).
  {
    DSizes<Index, 6> out_dims = make_out_dims(numext::div_ceil(planes, Index(2)), numext::div_ceil(rows, Index(2)),
                                              numext::div_ceil(cols, Index(2)));
    VerifyBlockEvaluator<T, 6, Layout>(input.extract_volume_patches(pp, pr, pc, 2, 2, 2, PADDING_SAME),
                                       [&out_dims]() { return RandomBlock<Layout>(out_dims, 1, 10); });
    VerifyBlockEvaluator<T, 6, Layout>(input.extract_volume_patches(pp, pr, pc, 2, 2, 2, PADDING_SAME),
                                       [&out_dims]() { return FixedSizeBlock(out_dims); });
  }

  // Inflated (zero-interleaved) input, reaching the branch that rejects an
  // in-range coordinate for landing between samples.
  {
    const Index planes_eff = (planes - 1) * 2 + 1;
    const Index rows_eff = (rows - 1) * 2 + 1;
    const Index cols_eff = (cols - 1) * 2 + 1;
    DSizes<Index, 6> out_dims = make_out_dims(planes_eff - pp + 1, rows_eff - pr + 1, cols_eff - pc + 1);
    auto inflated = input.extract_volume_patches(pp, pr, pc, 1, 1, 1, 2, 2, 2, 0, 0, 0, 0, 0, 0, T(0));
    VerifyBlockEvaluator<T, 6, Layout>(inflated, [&out_dims]() { return RandomBlock<Layout>(out_dims, 1, 10); });
    VerifyBlockEvaluator<T, 6, Layout>(inflated, [&out_dims]() { return FixedSizeBlock(out_dims); });
  }

  // Dispatch check: stride() serves neither blocks nor raw buffers, so this
  // pins down that the block path needs no capability bit from its argument.
  {
    const array<Index, 5> strides = {{1, 2, 2, 2, 1}};
    auto strided_patch = input.stride(strides).extract_volume_patches(pp, pr, pc, 1, 1, 1, PADDING_VALID);
    typedef TensorEvaluator<const decltype(strided_patch), DefaultDevice> PatchEval;
    static_assert(PatchEval::BlockAccess && PatchEval::PreferBlockAccess,
                  "the volume-patch block path must dispatch for a coeff()-only argument");
    const Index planes_s = (planes - 1) / 2 + 1;
    const Index rows_s = (rows - 1) / 2 + 1;
    const Index cols_s = (cols - 1) / 2 + 1;
    DSizes<Index, 6> out_dims = make_out_dims(planes_s - pp + 1, rows_s - pr + 1, cols_s - pc + 1);
    VerifyBlockEvaluator<T, 6, Layout>(strided_patch, [&out_dims]() { return RandomBlock<Layout>(out_dims, 1, 10); });
  }
}

// -------------------------------------------------------------------------- //
// Verify that assigning block to a Tensor expression produces the same result
// as an assignment to TensorSliceOp (writing a block is is identical to
// assigning one tensor to a slice of another tensor).

// `Layout` is the layout of the expression and the blocks written through it;
// `TensorLayout` is the layout of the underlying destination tensor. They only
// differ when the expression contains a layout swap.
template <typename T, int NumDims, int Layout, int NumExprDims = NumDims, int TensorLayout = Layout,
          typename Expression, typename GenBlockParams>
static void VerifyBlockAssignment(Tensor<T, NumDims, TensorLayout>& tensor, Expression expr, GenBlockParams gen_block) {
  using Device = DefaultDevice;
  auto d = Device();

  // We use tensor evaluator as a target for block and slice assignments.
  auto eval = TensorEvaluator<decltype(expr), Device>(expr, d);

  // Generate a random block, or choose a block that fits in full expression.
  TensorBlockParams<NumExprDims> block_params = gen_block();

  // Generate random data of the selected block size.
  Tensor<T, NumExprDims, Layout> block(block_params.desc.dimensions());
  block.setRandom();

  // ************************************************************************ //
  // (1) Assignment from a block.

  // Construct a materialize block from a random generated block tensor.
  internal::TensorMaterializedBlock<T, NumExprDims, Layout> blk(internal::TensorBlockKind::kView, block.data(),
                                                                block.dimensions());

  // Reset all underlying tensor values to zero.
  tensor.setZero();

  // Use evaluator to write block into a tensor.
  eval.writeBlock(block_params.desc, blk);

  // Make a copy of the result after assignment.
  Tensor<T, NumDims, TensorLayout> block_assigned = tensor;

  // ************************************************************************ //
  // (2) Assignment to a slice

  // Reset all underlying tensor values to zero.
  tensor.setZero();

  // Assign block to a slice of original expression
  auto s_expr = expr.slice(block_params.offsets, block_params.sizes);

  // Explicitly use coefficient assignment to evaluate slice expression.
  using SliceAssign = TensorAssignOp<decltype(s_expr), const decltype(block)>;
  using SliceExecutor = TensorExecutor<const SliceAssign, Device, false, internal::TiledEvaluation::Off>;
  SliceExecutor::run(SliceAssign(s_expr, block), d);

  // Make a copy of the result after assignment.
  Tensor<T, NumDims, TensorLayout> slice_assigned = tensor;

  for (Index i = 0; i < tensor.dimensions().TotalSize(); ++i) {
    VERIFY_IS_EQUAL(block_assigned.coeff(i), slice_assigned.coeff(i));
  }
}

// -------------------------------------------------------------------------- //

template <typename T, int NumDims, int Layout>
static void test_assign_to_tensor() {
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(10, 20);
  Tensor<T, NumDims, Layout> tensor(dims);

  TensorMap<Tensor<T, NumDims, Layout>> map(tensor.data(), dims);

  VerifyBlockAssignment<T, NumDims, Layout>(tensor, map, [&dims]() { return RandomBlock<Layout>(dims, 10, 20); });
  VerifyBlockAssignment<T, NumDims, Layout>(tensor, map, [&dims]() { return FixedSizeBlock(dims); });
}

template <typename T, int NumDims, int Layout>
static void test_assign_to_tensor_reshape() {
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(10, 20);
  Tensor<T, NumDims, Layout> tensor(dims);

  TensorMap<Tensor<T, NumDims, Layout>> map(tensor.data(), dims);

  DSizes<Index, NumDims> shuffled = dims;
  std::shuffle(&shuffled[0], &shuffled[NumDims - 1], std::mt19937(g_seed));

  VerifyBlockAssignment<T, NumDims, Layout>(tensor, map.reshape(shuffled),
                                            [&shuffled]() { return RandomBlock<Layout>(shuffled, 1, 10); });

  VerifyBlockAssignment<T, NumDims, Layout>(tensor, map.reshape(shuffled),
                                            [&shuffled]() { return SkewedInnerBlock<Layout>(shuffled); });

  VerifyBlockAssignment<T, NumDims, Layout>(tensor, map.reshape(shuffled),
                                            [&shuffled]() { return FixedSizeBlock(shuffled); });
}

template <typename T, int NumDims, int Layout>
static void test_assign_to_tensor_chipping() {
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(10, 20);
  Tensor<T, NumDims, Layout> tensor(dims);

  Index chip_dim = internal::random<int>(0, NumDims - 1);
  Index chip_offset = internal::random<Index>(0, dims[chip_dim] - 2);

  DSizes<Index, NumDims - 1> chipped_dims;
  for (Index i = 0; i < chip_dim; ++i) {
    chipped_dims[i] = dims[i];
  }
  for (Index i = chip_dim + 1; i < NumDims; ++i) {
    chipped_dims[i - 1] = dims[i];
  }

  TensorMap<Tensor<T, NumDims, Layout>> map(tensor.data(), dims);

  VerifyBlockAssignment<T, NumDims, Layout, NumDims - 1>(
      tensor, map.chip(chip_offset, chip_dim), [&chipped_dims]() { return RandomBlock<Layout>(chipped_dims, 1, 10); });

  VerifyBlockAssignment<T, NumDims, Layout, NumDims - 1>(
      tensor, map.chip(chip_offset, chip_dim), [&chipped_dims]() { return SkewedInnerBlock<Layout>(chipped_dims); });

  VerifyBlockAssignment<T, NumDims, Layout, NumDims - 1>(tensor, map.chip(chip_offset, chip_dim),
                                                         [&chipped_dims]() { return FixedSizeBlock(chipped_dims); });
}

template <typename T, int NumDims, int Layout>
static void test_assign_to_tensor_slice() {
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(10, 20);
  Tensor<T, NumDims, Layout> tensor(dims);

  // Pick a random slice of tensor.
  DSizes<Index, NumDims> slice_start = RandomDims<NumDims>(5, 10);
  DSizes<Index, NumDims> slice_size = RandomDims<NumDims>(5, 10);

  // Make sure that slice start + size do not overflow tensor dims.
  for (int i = 0; i < NumDims; ++i) {
    slice_start[i] = numext::mini(dims[i] - 1, slice_start[i]);
    slice_size[i] = numext::mini(slice_size[i], dims[i] - slice_start[i]);
  }

  TensorMap<Tensor<T, NumDims, Layout>> map(tensor.data(), dims);

  VerifyBlockAssignment<T, NumDims, Layout>(tensor, map.slice(slice_start, slice_size),
                                            [&slice_size]() { return RandomBlock<Layout>(slice_size, 1, 10); });

  VerifyBlockAssignment<T, NumDims, Layout>(tensor, map.slice(slice_start, slice_size),
                                            [&slice_size]() { return SkewedInnerBlock<Layout>(slice_size); });

  VerifyBlockAssignment<T, NumDims, Layout>(tensor, map.slice(slice_start, slice_size),
                                            [&slice_size]() { return FixedSizeBlock(slice_size); });
}

template <typename T, int NumDims, int Layout>
static void test_assign_to_tensor_shuffle() {
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(5, 15);
  Tensor<T, NumDims, Layout> tensor(dims);

  DSizes<Index, NumDims> shuffle;
  for (int i = 0; i < NumDims; ++i) shuffle[i] = i;

  TensorMap<Tensor<T, NumDims, Layout>> map(tensor.data(), dims);

  do {
    DSizes<Index, NumDims> shuffled_dims;
    for (int i = 0; i < NumDims; ++i) shuffled_dims[i] = dims[shuffle[i]];

    VerifyBlockAssignment<T, NumDims, Layout>(tensor, map.shuffle(shuffle),
                                              [&shuffled_dims]() { return FixedSizeBlock(shuffled_dims); });

    VerifyBlockAssignment<T, NumDims, Layout>(tensor, map.shuffle(shuffle),
                                              [&shuffled_dims]() { return RandomBlock<Layout>(shuffled_dims, 1, 5); });

  } while (std::next_permutation(&shuffle[0], &shuffle[0] + NumDims));
}

template <typename T, int NumDims, int Layout>
static void test_assign_to_tensor_layout_swap() {
  // The swap_layout lvalue has the opposite layout of the underlying tensor.
  // static so the lambdas below can name it without capturing: MSVC 14.29
  // rejects an uncaptured non-static constexpr local there (error C3493).
  static constexpr int SwappedLayout = (Layout == ColMajor) ? RowMajor : ColMajor;
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(5, 15);
  Tensor<T, NumDims, Layout> tensor(dims);

  DSizes<Index, NumDims> swapped_dims;
  for (int i = 0; i < NumDims; ++i) swapped_dims[i] = dims[NumDims - 1 - i];

  TensorMap<Tensor<T, NumDims, Layout>> map(tensor.data(), dims);

  VerifyBlockAssignment<T, NumDims, SwappedLayout>(
      tensor, map.swap_layout(), [&swapped_dims]() { return RandomBlock<SwappedLayout>(swapped_dims, 1, 10); });

  VerifyBlockAssignment<T, NumDims, SwappedLayout>(tensor, map.swap_layout(),
                                                   [&swapped_dims]() { return FixedSizeBlock(swapped_dims); });
}

template <typename T, int NumDims, int Layout>
static void test_assign_to_tensor_strided_slice() {
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(10, 20);
  Tensor<T, NumDims, Layout> tensor(dims);

  TensorMap<Tensor<T, NumDims, Layout>> map(tensor.data(), dims);

  // Sweep unit, dilated, and negative strides; start/stop are chosen in-range
  // so that clamping does not kick in.
  const Index stride_choices[] = {1, 2, -1, -2};
  for (const Index s : stride_choices) {
    DSizes<Index, NumDims> start, stop, strides, out_dims;
    for (int i = 0; i < NumDims; ++i) {
      strides[i] = s;
      if (s > 0) {
        start[i] = 1;
        stop[i] = dims[i] - 1;
      } else {
        start[i] = dims[i] - 2;
        stop[i] = 0;
      }
      const Index interval = stop[i] - start[i];
      out_dims[i] = interval / strides[i] + (interval % strides[i] != 0 ? 1 : 0);
    }

    VerifyBlockAssignment<T, NumDims, Layout>(tensor, map.stridedSlice(start, stop, strides),
                                              [&out_dims]() { return RandomBlock<Layout>(out_dims, 1, 5); });
    VerifyBlockAssignment<T, NumDims, Layout>(tensor, map.stridedSlice(start, stop, strides),
                                              [&out_dims]() { return FixedSizeBlock(out_dims); });
  }
}

// A lazy rhs expression has no raw buffer, so the tiled executor drives
// writeBlock with a block expression: assigned directly into the destination
// for a unit inner stride, and through a materialized temporary otherwise.
// VerifyBlockAssignment cannot reach these paths because it always feeds
// writeBlock a materialized block.
template <typename T, int NumDims, int Layout>
static void test_assign_expr_to_tensor_strided_slice() {
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(10, 20);
  Tensor<T, NumDims, Layout> tensor(dims);

  TensorMap<Tensor<T, NumDims, Layout>> map(tensor.data(), dims);

  const Index stride_choices[] = {1, 2, -1, -2};
  for (const Index s : stride_choices) {
    DSizes<Index, NumDims> start, stop, strides, out_dims;
    for (int i = 0; i < NumDims; ++i) {
      strides[i] = s;
      if (s > 0) {
        start[i] = 1;
        stop[i] = dims[i] - 1;
      } else {
        start[i] = dims[i] - 2;
        stop[i] = 0;
      }
      const Index interval = stop[i] - start[i];
      out_dims[i] = interval / strides[i] + (interval % strides[i] != 0 ? 1 : 0);
    }

    Tensor<T, NumDims, Layout> rhs(out_dims);
    rhs.setRandom();

    auto lhs_expr = map.stridedSlice(start, stop, strides);
    auto rhs_expr = rhs + rhs.constant(T(1));
    using Assign = TensorAssignOp<decltype(lhs_expr), const decltype(rhs_expr)>;

    tensor.setZero();
    TensorExecutor<const Assign, DefaultDevice, /*Vectorizable=*/true, internal::TiledEvaluation::On>::run(
        Assign(lhs_expr, rhs_expr), DefaultDevice());
    Tensor<T, NumDims, Layout> tiled = tensor;

    tensor.setZero();
    TensorExecutor<const Assign, DefaultDevice, /*Vectorizable=*/false, internal::TiledEvaluation::Off>::run(
        Assign(lhs_expr, rhs_expr), DefaultDevice());

    for (Index i = 0; i < tensor.size(); ++i) {
      VERIFY_IS_EQUAL(tiled.coeff(i), tensor.coeff(i));
    }
  }
}

template <typename T, int NumDims, int Layout>
static void test_assign_to_tensor_reverse() {
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(5, 15);
  Tensor<T, NumDims, Layout> tensor(dims);

  TensorMap<Tensor<T, NumDims, Layout>> map(tensor.data(), dims);

  array<bool, NumDims> reverse;
  for (int i = 0; i < NumDims; ++i) reverse[i] = false;
  VerifyBlockAssignment<T, NumDims, Layout>(tensor, map.reverse(reverse),
                                            [&dims]() { return RandomBlock<Layout>(dims, 1, 10); });

  for (int i = 0; i < NumDims; ++i) reverse[i] = true;
  VerifyBlockAssignment<T, NumDims, Layout>(tensor, map.reverse(reverse),
                                            [&dims]() { return RandomBlock<Layout>(dims, 1, 10); });
  VerifyBlockAssignment<T, NumDims, Layout>(tensor, map.reverse(reverse), [&dims]() { return FixedSizeBlock(dims); });

  for (int i = 0; i < NumDims; ++i) reverse[i] = internal::random<bool>();
  VerifyBlockAssignment<T, NumDims, Layout>(tensor, map.reverse(reverse),
                                            [&dims]() { return RandomBlock<Layout>(dims, 1, 10); });
}

template <typename T, int NumDims, int Layout>
static void test_assign_to_tensor_roll() {
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(5, 15);
  Tensor<T, NumDims, Layout> tensor(dims);

  TensorMap<Tensor<T, NumDims, Layout>> map(tensor.data(), dims);

  array<Index, NumDims> rolls;
  for (int i = 0; i < NumDims; ++i) rolls[i] = 0;
  VerifyBlockAssignment<T, NumDims, Layout>(tensor, map.roll(rolls),
                                            [&dims]() { return RandomBlock<Layout>(dims, 1, 10); });

  // Shifts beyond the dimension size and negative shifts must wrap.
  for (int i = 0; i < NumDims; ++i) rolls[i] = internal::random<Index>(-2 * dims[i], 2 * dims[i]);
  VerifyBlockAssignment<T, NumDims, Layout>(tensor, map.roll(rolls),
                                            [&dims]() { return RandomBlock<Layout>(dims, 1, 10); });
  VerifyBlockAssignment<T, NumDims, Layout>(tensor, map.roll(rolls), [&dims]() { return FixedSizeBlock(dims); });
}

template <typename T, int NumDims, int Layout>
static void test_assign_to_tensor_concatenation() {
  // A concatenation writes into two operands, which VerifyBlockAssignment's
  // single-tensor contract cannot express; compare writeBlock against a
  // scalar reference instead. This is also the only place the RowMajor lvalue
  // evaluator is instantiated: its coeffRef/writePacket are ColMajor-only, so
  // a slice-assignment reference would not compile.
  DSizes<Index, NumDims> dims = RandomDims<NumDims>(5, 15);
  const int axis = internal::random<int>(0, NumDims - 1);
  const Index left_axis_size = internal::random<Index>(1, dims[axis] - 1);

  DSizes<Index, NumDims> left_dims = dims;
  left_dims[axis] = left_axis_size;
  DSizes<Index, NumDims> right_dims = dims;
  right_dims[axis] = dims[axis] - left_axis_size;

  Tensor<T, NumDims, Layout> left(left_dims);
  Tensor<T, NumDims, Layout> right(right_dims);

  TensorMap<Tensor<T, NumDims, Layout>> left_map(left.data(), left_dims);
  TensorMap<Tensor<T, NumDims, Layout>> right_map(right.data(), right_dims);

  auto expr = left_map.concatenate(right_map, axis);
  auto eval = TensorEvaluator<decltype(expr), DefaultDevice>(expr, DefaultDevice());

  const DSizes<Index, NumDims> strides = internal::strides<Layout>(dims);

  // Whether a block straddles the concat axis decides which of the two copies
  // in writeBlock run, so pin all three cases instead of leaving it to chance:
  // a random block, one that spans the seam, and one wholly in the right
  // operand.
  auto verify = [&](TensorBlockParams<NumDims> params) {
    Tensor<T, NumDims, Layout> block(params.desc.dimensions());
    block.setRandom();

    left.setZero();
    right.setZero();
    internal::TensorMaterializedBlock<T, NumDims, Layout> blk(internal::TensorBlockKind::kView, block.data(),
                                                              block.dimensions());
    eval.writeBlock(params.desc, blk);

    // Scalar reference: land every block coefficient in the expected operand.
    Tensor<T, NumDims, Layout> expected_left(left_dims);
    Tensor<T, NumDims, Layout> expected_right(right_dims);
    expected_left.setZero();
    expected_right.setZero();

    DSizes<Index, NumDims> it;
    for (int i = 0; i < NumDims; ++i) it[i] = 0;
    for (Index count = 0; count < block.size(); ++count) {
      array<Index, NumDims> dst;
      for (int i = 0; i < NumDims; ++i) dst[i] = params.offsets[i] + it[i];
      if (dst[axis] < left_axis_size) {
        expected_left(dst) = block(it);
      } else {
        dst[axis] -= left_axis_size;
        expected_right(dst) = block(it);
      }
      for (int i = 0; i < NumDims; ++i) {
        if (++it[i] < params.desc.dimension(i)) break;
        it[i] = 0;
      }
    }

    for (Index i = 0; i < left.size(); ++i) VERIFY_IS_EQUAL(left.coeff(i), expected_left.coeff(i));
    for (Index i = 0; i < right.size(); ++i) VERIFY_IS_EQUAL(right.coeff(i), expected_right.coeff(i));
  };

  // Builds the descriptor for an explicit offset/extent box.
  auto make_params = [&](DSizes<Index, NumDims> offsets, DSizes<Index, NumDims> sizes) {
    Index offset = 0;
    for (int i = 0; i < NumDims; ++i) offset += strides[i] * offsets[i];
    return TensorBlockParams<NumDims>{offsets, sizes, TensorBlockDescriptor<NumDims, Index>(offset, sizes)};
  };

  verify(RandomBlock<Layout>(dims, 1, 10));

  // Straddling the seam: both the left and the right copy run.
  {
    DSizes<Index, NumDims> offsets;
    DSizes<Index, NumDims> sizes;
    for (int i = 0; i < NumDims; ++i) {
      offsets[i] = 0;
      sizes[i] = numext::mini(dims[i], Index(3));
    }
    offsets[axis] = left_axis_size - 1;
    sizes[axis] = numext::mini(Index(2), dims[axis] - offsets[axis]);
    verify(make_params(offsets, sizes));
  }

  // Wholly inside the right operand: only the right copy runs.
  {
    DSizes<Index, NumDims> offsets;
    DSizes<Index, NumDims> sizes;
    for (int i = 0; i < NumDims; ++i) {
      offsets[i] = 0;
      sizes[i] = numext::mini(dims[i], Index(3));
    }
    offsets[axis] = left_axis_size;
    sizes[axis] = numext::mini(Index(2), dims[axis] - offsets[axis]);
    verify(make_params(offsets, sizes));
  }
}

// -------------------------------------------------------------------------- //

#define CALL_SUBTEST_PART(PART) CALL_SUBTEST_##PART

#define CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(PART, NAME)     \
  CALL_SUBTEST_PART(PART)((NAME<float, 1, RowMajor>())); \
  CALL_SUBTEST_PART(PART)((NAME<float, 2, RowMajor>())); \
  CALL_SUBTEST_PART(PART)((NAME<float, 3, RowMajor>())); \
  CALL_SUBTEST_PART(PART)((NAME<float, 4, RowMajor>())); \
  CALL_SUBTEST_PART(PART)((NAME<float, 5, RowMajor>())); \
  CALL_SUBTEST_PART(PART)((NAME<float, 1, ColMajor>())); \
  CALL_SUBTEST_PART(PART)((NAME<float, 2, ColMajor>())); \
  CALL_SUBTEST_PART(PART)((NAME<float, 3, ColMajor>())); \
  CALL_SUBTEST_PART(PART)((NAME<float, 4, ColMajor>())); \
  CALL_SUBTEST_PART(PART)((NAME<float, 5, ColMajor>())); \
  CALL_SUBTEST_PART(PART)((NAME<int, 1, RowMajor>()));   \
  CALL_SUBTEST_PART(PART)((NAME<int, 2, RowMajor>()));   \
  CALL_SUBTEST_PART(PART)((NAME<int, 3, RowMajor>()));   \
  CALL_SUBTEST_PART(PART)((NAME<int, 4, RowMajor>()));   \
  CALL_SUBTEST_PART(PART)((NAME<int, 5, RowMajor>()));   \
  CALL_SUBTEST_PART(PART)((NAME<int, 1, ColMajor>()));   \
  CALL_SUBTEST_PART(PART)((NAME<int, 2, ColMajor>()));   \
  CALL_SUBTEST_PART(PART)((NAME<int, 3, ColMajor>()));   \
  CALL_SUBTEST_PART(PART)((NAME<int, 4, ColMajor>()));   \
  CALL_SUBTEST_PART(PART)((NAME<int, 5, ColMajor>()));   \
  CALL_SUBTEST_PART(PART)((NAME<bool, 1, RowMajor>()));  \
  CALL_SUBTEST_PART(PART)((NAME<bool, 2, RowMajor>()));  \
  CALL_SUBTEST_PART(PART)((NAME<bool, 3, RowMajor>()));  \
  CALL_SUBTEST_PART(PART)((NAME<bool, 4, RowMajor>()));  \
  CALL_SUBTEST_PART(PART)((NAME<bool, 5, RowMajor>()));  \
  CALL_SUBTEST_PART(PART)((NAME<bool, 1, ColMajor>()));  \
  CALL_SUBTEST_PART(PART)((NAME<bool, 2, ColMajor>()));  \
  CALL_SUBTEST_PART(PART)((NAME<bool, 3, ColMajor>()));  \
  CALL_SUBTEST_PART(PART)((NAME<bool, 4, ColMajor>()));  \
  CALL_SUBTEST_PART(PART)((NAME<bool, 5, ColMajor>()))

#define CALL_SUBTESTS_DIMS_LAYOUTS(PART, NAME)           \
  CALL_SUBTEST_PART(PART)((NAME<float, 1, RowMajor>())); \
  CALL_SUBTEST_PART(PART)((NAME<float, 2, RowMajor>())); \
  CALL_SUBTEST_PART(PART)((NAME<float, 3, RowMajor>())); \
  CALL_SUBTEST_PART(PART)((NAME<float, 4, RowMajor>())); \
  CALL_SUBTEST_PART(PART)((NAME<float, 5, RowMajor>())); \
  CALL_SUBTEST_PART(PART)((NAME<float, 1, ColMajor>())); \
  CALL_SUBTEST_PART(PART)((NAME<float, 2, ColMajor>())); \
  CALL_SUBTEST_PART(PART)((NAME<float, 3, ColMajor>())); \
  CALL_SUBTEST_PART(PART)((NAME<float, 4, ColMajor>())); \
  CALL_SUBTEST_PART(PART)((NAME<float, 5, ColMajor>()))

#define CALL_SUBTESTS_LAYOUTS_TYPES(PART, NAME)       \
  CALL_SUBTEST_PART(PART)((NAME<float, RowMajor>())); \
  CALL_SUBTEST_PART(PART)((NAME<float, ColMajor>())); \
  CALL_SUBTEST_PART(PART)((NAME<bool, RowMajor>()));  \
  CALL_SUBTEST_PART(PART)((NAME<bool, ColMajor>()))

EIGEN_DECLARE_TEST(tensor_block_eval) {
  // clang-format off
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(1, test_eval_tensor_block);
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(1, test_eval_tensor_binary_expr_block);
  CALL_SUBTESTS_DIMS_LAYOUTS(1, test_eval_tensor_unary_expr_block);
  CALL_SUBTESTS_DIMS_LAYOUTS(2, test_eval_tensor_binary_with_unary_expr_block);
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(2, test_eval_tensor_broadcast);
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(2, test_eval_tensor_reshape);
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(2, test_eval_tensor_layout_swap);
  CALL_SUBTEST_PART(2)((test_eval_tensor_scan<float, 2, RowMajor>()));
  CALL_SUBTEST_PART(2)((test_eval_tensor_scan<float, 3, RowMajor>()));
  CALL_SUBTEST_PART(2)((test_eval_tensor_scan<float, 4, RowMajor>()));
  CALL_SUBTEST_PART(2)((test_eval_tensor_scan<float, 2, ColMajor>()));
  CALL_SUBTEST_PART(2)((test_eval_tensor_scan<float, 3, ColMajor>()));
  CALL_SUBTEST_PART(2)((test_eval_tensor_scan<float, 4, ColMajor>()));
  CALL_SUBTEST_PART(2)((test_eval_tensor_fft<float, 2, RowMajor>()));
  CALL_SUBTEST_PART(2)((test_eval_tensor_fft<float, 3, RowMajor>()));
  CALL_SUBTEST_PART(2)((test_eval_tensor_fft<float, 4, RowMajor>()));
  CALL_SUBTEST_PART(2)((test_eval_tensor_fft<float, 2, ColMajor>()));
  CALL_SUBTEST_PART(2)((test_eval_tensor_fft<float, 3, ColMajor>()));
  CALL_SUBTEST_PART(2)((test_eval_tensor_fft<float, 4, ColMajor>()));
  CALL_SUBTEST_PART(2)((test_eval_composed_block_ops<float, 2, RowMajor>()));
  CALL_SUBTEST_PART(2)((test_eval_composed_block_ops<float, 3, RowMajor>()));
  CALL_SUBTEST_PART(2)((test_eval_composed_block_ops<float, 4, RowMajor>()));
  CALL_SUBTEST_PART(2)((test_eval_composed_block_ops<float, 2, ColMajor>()));
  CALL_SUBTEST_PART(2)((test_eval_composed_block_ops<float, 3, ColMajor>()));
  CALL_SUBTEST_PART(2)((test_eval_composed_block_ops<float, 4, ColMajor>()));
  CALL_SUBTEST_PART(2)((test_eval_contract_pad_composition<float, RowMajor>()));
  CALL_SUBTEST_PART(2)((test_eval_contract_pad_composition<float, ColMajor>()));
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(3, test_eval_tensor_cast);
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(3, test_eval_tensor_nullary);
  CALL_SUBTESTS_DIMS_LAYOUTS(3, test_eval_tensor_random_nullary);
  CALL_SUBTESTS_DIMS_LAYOUTS(3, test_eval_tensor_indexed_nullary);
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(3, test_eval_tensor_select);
  CALL_SUBTESTS_DIMS_LAYOUTS(3, test_eval_tensor_ternary);
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(3, test_eval_tensor_padding);
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(4, test_eval_tensor_chipping);
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(4, test_eval_tensor_concatenation);
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(4, test_eval_tensor_generator);
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(4, test_eval_tensor_reverse);
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(5, test_eval_tensor_slice);
  CALL_SUBTEST_PART(5)((test_eval_tensor_slice_bool_composite<1, RowMajor>()));
  CALL_SUBTEST_PART(5)((test_eval_tensor_slice_bool_composite<2, RowMajor>()));
  CALL_SUBTEST_PART(5)((test_eval_tensor_slice_bool_composite<3, RowMajor>()));
  CALL_SUBTEST_PART(5)((test_eval_tensor_slice_bool_composite<1, ColMajor>()));
  CALL_SUBTEST_PART(5)((test_eval_tensor_slice_bool_composite<2, ColMajor>()));
  CALL_SUBTEST_PART(5)((test_eval_tensor_slice_bool_composite<3, ColMajor>()));
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(5, test_eval_tensor_shuffle);
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(5, test_eval_tensor_strided_slice);

  CALL_SUBTESTS_LAYOUTS_TYPES(6, test_eval_tensor_reshape_with_bcast);
  CALL_SUBTESTS_LAYOUTS_TYPES(6, test_eval_tensor_forced_eval);
  CALL_SUBTESTS_LAYOUTS_TYPES(6, test_eval_tensor_chipping_of_bcast);
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(6, test_eval_tensor_inflation);
  CALL_SUBTESTS_LAYOUTS_TYPES(6, test_eval_tensor_patch);
  CALL_SUBTESTS_LAYOUTS_TYPES(6, test_eval_tensor_image_patch);
  CALL_SUBTESTS_LAYOUTS_TYPES(6, test_eval_tensor_volume_patch);

  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(7, test_assign_to_tensor);
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(7, test_assign_to_tensor_reshape);
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(7, test_assign_to_tensor_chipping);
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(8, test_assign_to_tensor_slice);
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(8, test_assign_to_tensor_shuffle);
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(8, test_assign_to_tensor_layout_swap);
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(8, test_assign_to_tensor_strided_slice);
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(8, test_assign_expr_to_tensor_strided_slice);
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(9, test_assign_to_tensor_reverse);
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(9, test_assign_to_tensor_roll);
  CALL_SUBTESTS_DIMS_LAYOUTS_TYPES(9, test_assign_to_tensor_concatenation);

  // Force CMake to split this test.
  // EIGEN_SUFFIXES;1;2;3;4;5;6;7;8;9

  // clang-format on
}
