# SPDX-FileCopyrightText: The Eigen Authors
# SPDX-License-Identifier: MPL-2.0

# ei_cuda_use_compiler_launcher() is the only way a compiler launcher reaches
# nvcc, because FindCUDA's cuda_add_executable() runs CUDA_NVCC_EXECUTABLE as a
# single quoted COMMAND.  CMAKE_<LANG>_COMPILER_LAUNCHER is a list whose
# elements are argv entries, and an element may contain spaces, so the wrapper
# has to preserve argument boundaries rather than flatten the list into one
# line of shell source.
#
# No CUDA installation is involved: the launcher and nvcc are stand-ins that
# report the argv they were handed.

include("${EIGEN_SOURCE_DIR}/cmake/EigenTesting.cmake")

# A cmd.exe wrapper cannot be executed from a POSIX host, so its quoting rules
# are pinned on the rendering instead, on every host.
ei_quote_command_for_batch(rendered "C:/a dir/ccache.exe" "--opt=x y" "C:/cuda/nvcc.exe")
bs_assert_streq("${rendered}"
                "\"C:/a dir/ccache.exe\" \"--opt=x y\" \"C:/cuda/nvcc.exe\""
                "cmd.exe rendering of a launcher command")

if(CMAKE_HOST_WIN32)
  return()
endif()

# Put the stand-ins under a directory whose name contains a space, so the paths
# the wrapper bakes in are quoted too, not only the launcher's own arguments.
set(bin_dir "${WORK_DIR}/fake bin")
file(MAKE_DIRECTORY "${bin_dir}")

set(fake_nvcc "${bin_dir}/fake nvcc")
file(WRITE "${fake_nvcc}" "#!/bin/sh\nfor a in \"$@\"; do echo \"nvcc-arg=[$a]\"; done\n")
execute_process(COMMAND chmod +x "${fake_nvcc}")

# Reports the one launcher argument it expects, then runs what follows it --
# which is how ccache is invoked, and what proves nvcc is still a separate argv
# entry rather than part of the launcher's.
set(fake_launcher "${bin_dir}/fake launcher")
file(WRITE "${fake_launcher}" "#!/bin/sh\necho \"launcher-arg=[$1]\"\nshift\nexec \"$@\"\n")
execute_process(COMMAND chmod +x "${fake_launcher}")

set(CMAKE_CXX_COMPILER_LAUNCHER "${fake_launcher}" "--launcher opt")
set(CUDA_NVCC_EXECUTABLE "${fake_nvcc}")
set(CMAKE_CURRENT_BINARY_DIR "${WORK_DIR}")
ei_cuda_use_compiler_launcher()

if(CUDA_NVCC_EXECUTABLE STREQUAL "${fake_nvcc}")
  bs_fail("ei_cuda_use_compiler_launcher() left CUDA_NVCC_EXECUTABLE unwrapped")
endif()

bs_run(WHAT "generated nvcc wrapper" OUTPUT_VARIABLE output
       COMMAND "${CUDA_NVCC_EXECUTABLE}" "-Ia dir" "-DQ=\"a b\"" "plain")

foreach(expected "launcher-arg=[--launcher opt]"
                 "nvcc-arg=[-Ia dir]"
                 "nvcc-arg=[-DQ=\"a b\"]"
                 "nvcc-arg=[plain]")
  string(FIND "${output}" "${expected}" position)
  if(position LESS 0)
    bs_fail("the wrapper did not pass through ${expected}\n----\n${output}\n----")
  endif()
endforeach()
