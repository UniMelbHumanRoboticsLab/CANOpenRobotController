# CONTRIBUTION GUIDELINES

## Before contributing

Welcome to [CORC Project](https://github.com/UniMelbHumanRoboticsLab/CANOpenRobotController)! Before submitting pull requests, please make sure that you have **read the whole guidelines**. If you have any doubts about this contribution guide, please open [an issue](https://github.com/UniMelbHumanRoboticsLab/CANOpenRobotController/issues/new/choose) and clearly state your concerns.

## Contributing

### Contributor

We are very happy that you consider implementing CORC with a currently supported or new platform! This repository is referred to and used by researchers and developers from a wide range of backgrounds around the globe. Being one of our contributors, you agree and confirm that:

- You did your own work.
  - No plagiarism allowed. Any plagiarized work will not be merged.
- Your work will be distributed under [Apache 2](License) once your pull request has been merged.
- You submitted work fulfils or mostly fulfils our styles and standards.

**New implementation** New implementation are welcome!
**Improving comments** and **adding tests** to existing code are much appreciated.

### Making Changes

#### Code

- Please use the directory structure of the repository.
- File extension for code should be _.h _.cpp.
- Build new applications using the existing Base Class structure - include directory.
- You can suggest reasonable changes to existing base Class code.
- Strictly use UpperCase in filenames: InputDevice.cpp.
- If you have added or modified code, please make sure the code compiles before submitting.
<!-- - TODO: MAKE THIS HAPPEN -> ur automated testing runs [**cpplint**](https://github.com/cpplint/cpplint) on all pull requests so please be sure that your code passes before submitting. -->
- **Be consistent in use of these guidelines.**

#### New File Name guidelines

- Use Upercase words without a separator
- For instance

```
my_new_cpp_class.c   is incorrect
MyNewCppClass.CPP     is correct format
```

#### New Directory guidelines

- We recommend adding files to existing directories as much as possible.
- Use Upercase words without a separator ( no spaces or `"-"` allowed )
- For instance

```
some_new_fancy_category         is correct
SomeNewFancyCategory          is incorrect
```

- Filepaths must be added to the makefiles modules for automatic compilation.
- No Filepath validation exists
- Verify by compiling your code

#### Commit Guidelines

- It is recommended to keep your changes grouped logically within individual commits. Maintainers find it easier to understand changes that are logically spilt across multiple commits. Try to modify just one or two files in the same directory. Pull requests that span multiple directories are tedious to manage and may be rejected.

```
git add file_xyz.cpp
git commit -m "your message"
```

Examples of commit messages with semantic prefixes:

```
fix: xyz algorithm bug
feat: add xyx algorithm, class xyz
test: add test for xyz algorithm
docs: add comments and explanation to xyz algorithm
```

Common prefixes:

- fix: A bug fix
- feat: A new feature
- docs: Documentation changes
- test: Correct existing tests or add new ones

#### Documentation

- Write self documenting code whenever possible and use DOXYGEN commenting when needed.
- Document method and class definitions using DOXYGEN in .h files.
- Make sure you put useful comments in your code. Do not comment things that are obvious.
- Please avoid creating new directories if at all possible. Try to fit your work into the existing directory structure. If you want to create a new directory, then please check if a similar category has been recently suggested or created by other pull requests.
- If you have modified/added documentation, please ensure that your language is concise and contains no grammar errors.
<!-- - Please follow the provided DOXYGEN styleguide () -->
- Do not update README.md along with other changes, first create an issue and then link to that issue in your pull request to suggest specific changes required to README.md

#### Test

- Make sure to add examples and test cases in a testfile.
- If you find any class or document without tests, please feel free to create a pull request or issue describing suggested changes.

- For system chages, please add to your pull request, system test results. - e.g. Fully opperational Robotic motion etc.
<!-- #### cpplint

To see if [**cpplint**](https://github.com/cpplint/cpplint) is already installed, do:

- `cpplint --version` # currently returns "cpplint 1.4.4"
  If cpplint is **_not_** installed then do:
- `python3 -m pip install cpplint` # If that does not work then try...
- `py -m pip install cpplint` # If that does not work then try...
- `pip install cpplint`
  Once cpplint is installed, test your file(s) with:
- `cpplint --filter=-legal my_file.cpp my_other_file.cpp` # Fix any issues and try again.

The [**clang-format**](https://clang.llvm.org/docs/ClangFormat.html) tool can fix whitespace related _cpplint_ issues.

- On Macs only: `brew install clang-format` # Only needs to be installed once.
- All platforms: `clang-format -i -style="{IndentWidth: 4}" my_file.cpp` -->

Most importantly,

- Happy coding!
