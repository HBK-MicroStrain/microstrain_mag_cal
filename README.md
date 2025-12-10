# MicroStrain Mag Cal
[![CI](https://github.com/HBK-MicroStrain-Internal/mag-cal/actions/workflows/ci.yml/badge.svg)](https://github.com/HBK-MicroStrain-Internal/mag-cal/actions/workflows/ci.yml)
[![Release](https://img.shields.io/github/v/release/HBK-MicroStrain-Internal/mag-cal?label=Release)](https://github.com/HBK-MicroStrain-Internal/mag-cal/releases/latest)
[![Pre-Release](https://img.shields.io/github/v/release/HBK-MicroStrain-Internal/mag-cal?include_prereleases&label=Pre-Release)](https://github.com/HBK-MicroStrain-Internal/mag-cal/releases/latest)

![Linux](https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black)
![Windows](https://img.shields.io/badge/Windows-0078D6?logoColor=white)
![macOS](https://img.shields.io/badge/macOS-000000?logo=apple&logoColor=white)

Suite of library and tools for magnetometer calibration.

## Usage
See the following usage guides:
* [Application](app/README.md#cli-usage)
* [Library](lib/README.md#usage)

### Fit functions
The core functionality in this suite revolves around fitting calibration coefficients. The currently
supported fit functions are:

| Function        | Correction                                                                  | Coefficients calibrated                                                                              |
|-----------------|-----------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------|
| Spherical fit   | $\mathbf{m}\_\text{corr} = s \cdot (\mathbf{m}\_\text{raw} - \mathbf{b})$   | - Hard-iron offset<br/> - Uniform scale factor                                                       |
| Ellipsoidal fit | $\mathbf{m}\_\text{corr} = \mathbf{A}(\mathbf{m}\_\text{raw} - \mathbf{b})$ | - Hard-iron offset<br/> - Non-uniform scale factor<br/>- Symmetric (non-uniform) cross-axis coupling |

where,

$$
\begin{aligned}
\mathbf{m}_\text{corr} &= \text{Corrected measurement vector} \\
\mathbf{m}_\text{raw} &= \text{Raw measurement vector} \\
\mathbf{b} &= \text{Hard-iron offset vector} \\
s &= \text{Uniform scale factor} \\
\mathbf{A} &= \text{Soft-iron matrix}
\end{aligned}
$$

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
cmake --build <build-dir> --parallel
```

After building, run this if you would like to create a distributable package as well:
```
cmake --build <build_dir> --target package_microstrain_mag_cal
```
The package will be in `<build-dir>`.

## Development
The library and application are developed together under the same project. They should be treated as 
one during development.

### Running tests
Unit tests should be run after every code change to catch errors early:
```
ctest --test-dir <build-dir> -L unit --output-on-failure --parallel
```
