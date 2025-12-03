# Offline Magnetometer Calibration
Library and command-line application for offline magnetometer calibration.

## Usage
See the following user guides:
 * [Application](application/README.md#cli-usage) 
 * [Library]()                                          

## Install
See the [Releases](https://github.com/HBK-MicroStrain-Internal/mag-cal/releases) page for downloads.

### Pre-built
Releases are currently distributed as a compressed folder containing the pre-built binaries.

Each component is distributed separately. Download the folder for the desired component and version
and unzip it.

### Building manually
This project uses CMake. To configure it, first create a build directory in the project root:
```
mkdir build
```
Then, run the configure command:
```
cmake -S . -B build
```

Once the project has been configured, each component can be built individually. See the following 
build guides:
* [Application](application/README.md#building-manually)
* [Library]()
