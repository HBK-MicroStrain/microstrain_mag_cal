# MicroStrain Mag Cal
Suite of library and tools for magnetometer calibration.

## Usage
See the following user guides:
 * [Application](application/README.md#cli-usage) 
 * [Library](library/README.md#usage)                                          

## Install
See the [Releases](https://github.com/HBK-MicroStrain-Internal/mag-cal/releases) page for downloads.

### Pre-built
Releases are currently distributed as a compressed folder containing the pre-built binaries.

Each component is distributed separately. Download the folder for the desired component and version
and unzip it.

### Building manually
This project uses CMake. To configure it, first create a build directory in the project root:
```
mkdir <build-dir>
```
Then, run the configure command:
```
cmake -S . -B <build-dir>
```

Once the project has been configured, build it:
```
cmake --build <build-dir>
```

After building, run this if you would like to create a distributable package as well:
```
cmake --build <build_dir> --target package_microstrain_mag_cal
```
The package will be in `<build-dir>`.

## Development
The library and application are developed together under the same project. While they are each
standalone, they should be treated as one during development.
### Running tests
Unit tests should be run after every code change to catch errors early:
```
ctest --test-dir build -L unit --output-on-failure --verbose --parallel
```
