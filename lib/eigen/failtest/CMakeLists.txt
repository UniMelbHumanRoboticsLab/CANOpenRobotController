# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

ei_add_failtest("failtest_sanity_check")

ei_add_failtest("block_nonconst_ctor_on_const_xpr_0")
ei_add_failtest("block_nonconst_ctor_on_const_xpr_1")
ei_add_failtest("block_nonconst_ctor_on_const_xpr_2")
ei_add_failtest("transpose_nonconst_ctor_on_const_xpr")
ei_add_failtest("diagonal_nonconst_ctor_on_const_xpr")
ei_add_failtest("cwiseunaryview_nonconst_ctor_on_const_xpr")
ei_add_failtest("triangularview_nonconst_ctor_on_const_xpr")
ei_add_failtest("selfadjointview_nonconst_ctor_on_const_xpr")

ei_add_failtest("const_qualified_block_method_retval_0")
ei_add_failtest("const_qualified_block_method_retval_1")
ei_add_failtest("const_qualified_transpose_method_retval")
ei_add_failtest("const_qualified_diagonal_method_retval")

ei_add_failtest("map_nonconst_ctor_on_const_ptr_0")
ei_add_failtest("map_nonconst_ctor_on_const_ptr_1")
ei_add_failtest("map_nonconst_ctor_on_const_ptr_2")
ei_add_failtest("map_nonconst_ctor_on_const_ptr_3")
ei_add_failtest("map_nonconst_ctor_on_const_ptr_4")

ei_add_failtest("map_on_const_type_actually_const_0")
ei_add_failtest("map_on_const_type_actually_const_1")
ei_add_failtest("block_on_const_type_actually_const_0")
ei_add_failtest("block_on_const_type_actually_const_1")
ei_add_failtest("transpose_on_const_type_actually_const")
ei_add_failtest("diagonal_on_const_type_actually_const")
ei_add_failtest("cwiseunaryview_on_const_type_actually_const")
ei_add_failtest("triangularview_on_const_type_actually_const")
ei_add_failtest("selfadjointview_on_const_type_actually_const")

ei_add_failtest("ref_1")
ei_add_failtest("ref_2")
ei_add_failtest("ref_3")
ei_add_failtest("ref_4")
ei_add_failtest("ref_5")
ei_add_failtest("ref_6")
ei_add_failtest("ref_7")

ei_add_failtest("swap_1")
ei_add_failtest("swap_2")

ei_add_failtest("ternary_1")
ei_add_failtest("ternary_2")

ei_add_failtest("sparse_ref_1")
ei_add_failtest("sparse_ref_2")
ei_add_failtest("sparse_ref_3")
ei_add_failtest("sparse_ref_4")
ei_add_failtest("sparse_ref_5")

ei_add_failtest("sparse_storage_mismatch")

ei_add_failtest("partialpivlu_int")
ei_add_failtest("fullpivlu_int")
ei_add_failtest("llt_int")
ei_add_failtest("ldlt_int")
ei_add_failtest("qr_int")
ei_add_failtest("colpivqr_int")
ei_add_failtest("fullpivqr_int")
ei_add_failtest("jacobisvd_int")
ei_add_failtest("bdcsvd_int")
ei_add_failtest("eigensolver_int")
ei_add_failtest("eigensolver_cplx")
ei_add_failtest("initializer_list_1")
ei_add_failtest("initializer_list_2")

ei_add_failtest("erf_no_scalar_overload")
ei_add_failtest("erfc_no_scalar_overload")
ei_add_failtest("vandermonde_int")
ei_add_failtest("vandermonde_rectangular_square")
ei_add_failtest("bjorckpereyra_rectangular")

ei_add_failtest("structured_bindings_dynamic_matrix")
ei_add_failtest("structured_bindings_dynamic_array")
ei_add_failtest("structured_bindings_rowmajor")

# Structured bindings require C++17 — the header is a no-op otherwise and the
# "_ok" targets would not compile. Force the standard on these four targets
# (two per failtest: the _ok and _ko variants).
foreach(_sb_target
    structured_bindings_dynamic_matrix_ok structured_bindings_dynamic_matrix_ko
    structured_bindings_dynamic_array_ok  structured_bindings_dynamic_array_ko
    structured_bindings_rowmajor_ok       structured_bindings_rowmajor_ko)
  set_target_properties(${_sb_target} PROPERTIES
                        CXX_STANDARD 17
                        CXX_STANDARD_REQUIRED ON
                        CXX_EXTENSIONS OFF)
endforeach()
