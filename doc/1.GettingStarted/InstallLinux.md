# Getting Started with Linux - Installation Instructions

This document describes the setup process required to get started with a clean installation of Ubuntu Linux. 

When developing with CORC on a Linux-based computer, you can choose to deploy on either your host (development) computer, or on a remote computer. Given that local-machine testing is possible (and likely more convenient), these instructions will detail how to set up and run the software on the development computer. Instructions specific to deploying to am `arm-linux-gnueabihf` remote system (as per the Beaglebone Black example) will be tagged with [DEPLOY-REMOTE]. 

The following installation instructions are to be executed from Terminal. A `sudo apt update` is suggested to ensure that the latest packages are available. 

## Git 
CORC is hosted on Github, and uses Git for its version control. Whilst technically not required (all the files can be downloaded from the Github repository), it is highly recommended that Git is used. 

```bash
$ sudo apt install git
$ git config --global user.name [USERNAME]
$ git config --global user.email [EMAIL]
```

Where [USERNAME] and [EMAIL] correspond to your own Github username and email.

## Host Compilation Tools

CORC is written in C++ and C, and thus requires an appropriate compiler. A simple way of installing this is to install the `build-essential` package: 
```bash
$ sudo apt install build-essential
```

Additionally, CORC uses CMAKE, which can be installed with:
```bash
$ sudo apt install cmake
```

> Note that the integration of ROS elements into CORC require the installation of ROS and catkin. This is not yet in this guide, but will be added in the future. 

## Cross-compilation Tools [DEPLOY-REMOTE]
If you are to be deploying onto remote system, you also require cross-compilation toolchains. These instructions are written for the Beaglebone Black running Debian Stretch 9.5 Firmware, and require the `arm-linux-gnueabihf` compilation packages. These can be installed as follows:

```bash 
$ sudo apt install gcc-arm-linux-gnueabihf
$ sudo apt install g++-arm-linux-gnueabihf
```

## Networking Tools
CORC uses CANOpen, thus some networking tools are required to configure and access the CAN bus. These can be installed using the following commands:

```bash
$ sudo apt install net-tools
$ sudo apt install can-utils
```

`net-tools` includes `ifconfig` which can be used to detect and configure network interfaces (including ethernet and CAN), and `can-utils` provides tools which allow the configuration of the CAN devices. 

> Note: This guide at this stage only includes instructions to test using a virtual CAN device. Use of a real CAN device is possible using (for example) a USB-CAN adapter. These instructions are to be added at a later date.


## Other Tools
It is also useful to have a development environment. If you do not have a preferred environment, VSCode (https://code.visualstudio.com/) is popular and can be configured with a number of extensions which make development easier. 