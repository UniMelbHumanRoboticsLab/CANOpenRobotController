This directory hosts modules that satisfy a looser API-stability contract
than the rest of Eigen. Most of the content here is mature and widely
depended on (Tensor, AutoDiff, FFT, MatrixFunctions, Polynomials, Splines,
NNLS, the GPU host-dispatch layer, ...), but the interfaces may evolve
between minor releases and a small subset is genuinely experimental.

Up to and including Eigen 4.x this directory was named "unsupported/".
The tree was renamed to "contrib/" in Eigen 5.x; legacy
<unsupported/Eigen/...> include paths continue to work via forwarding
header shims under ../unsupported/.

In order to use a contrib module either:

 - add path_to_eigen/contrib to your include path and do:
   #include <Eigen/ModuleHeader>

 - or directly:
   #include <contrib/Eigen/ModuleHeader>


If you are interested in contributing to one of them, or have other stuff
you would like to share, feel free to contact us via the upstream project
on GitLab: https://gitlab.com/libeigen/eigen.

Any kind of contributions are much appreciated, even very preliminary ones.
However, it:
 - must rely on Eigen,
 - must be highly related to math,
 - should have some general purpose in the sense that it could
   potentially become a core Eigen module (or be merged into another one).

In doubt feel free to contact us. For instance, if your addon is very
specific but shows an interesting way of using Eigen, it could make for a
nice demo.


This directory is organized as follow:

contrib/Eigen/ModuleHeader1
contrib/Eigen/ModuleHeader2
contrib/Eigen/...
contrib/Eigen/src/Module1/SourceFile1.h
contrib/Eigen/src/Module1/SourceFile2.h
contrib/Eigen/src/Module1/...
contrib/Eigen/src/Module2/SourceFile1.h
contrib/Eigen/src/Module2/SourceFile2.h
contrib/Eigen/src/Module2/...
contrib/Eigen/src/...
contrib/doc/snippets/.cpp   <- code snippets for the doc
contrib/doc/examples/.cpp   <- examples for the doc
contrib/doc/TutorialModule1.dox
contrib/doc/TutorialModule2.dox
contrib/doc/...
contrib/test/.cpp           <- unit test files

The documentation is generated at the same time as the main Eigen
documentation. The .html files are generated in: build_dir/doc/html/contrib/
