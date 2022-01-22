# Getting Started with Windows - Installation Instructions
A compile CORC application can only run on a Linux machine, and as such, Windows can only be used as a development environment. Therefore, you must install the cross-compilation toolchains to compile for a Linux device. Here, we provide instructions only for the arm-linux-gnueabihf toolchain (which have been tested by the development team on various Beaglebone devices), however, cross compilation toolchains for other Linux devices may also be used. 

## Git 
CORC is hosted on Github, and uses Git for its version control. Whilst technically not required (all the files can be downloaded from the Github repository), it is highly recommended that Git is used. 

On Windows, you have the following two options:

- [Git for Windows](https://git-scm.com/download/win) - A command-line based utility
- [Github Desktop](https://desktop.github.com/) - A GUI specifically for Github Integration 

## Other Tools
It is also useful to have a development environment. If you do not have a preferred environment, VSCode (https://code.visualstudio.com/) is popular and can be configured with a number of extensions which make development easier. 

## Cross-compilation Tools 
Windows Tools chain for Beaglebone (which uses the arm-linux-gnueabihf) can be downloaded here: https://gnutoolchains.com/beaglebone/. Note that the tested environment is the `bone-debian-9.5-2018-10-07` Image with the `beaglebone-gcc6.3.0-r2.exe` toolchain.