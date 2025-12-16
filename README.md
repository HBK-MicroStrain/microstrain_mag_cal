# MicroStrain MagCal
<!-- Build and CI/CD Status -->
[![CI](https://github.com/HBK-MicroStrain-Internal/mag-cal/actions/workflows/ci.yml/badge.svg)](https://github.com/HBK-MicroStrain-Internal/mag-cal/actions/workflows/ci.yml)
[![CD](https://github.com/HBK-MicroStrain-Internal/mag-cal/actions/workflows/cd.yml/badge.svg)](https://github.com/HBK-MicroStrain-Internal/mag-cal/actions/workflows/cd.yml)
[![Stable](https://img.shields.io/github/v/release/HBK-MicroStrain-Internal/mag-cal?label=Stable)](https://github.com/HBK-MicroStrain-Internal/mag-cal/releases/latest)
[![Latest](https://img.shields.io/github/v/release/HBK-MicroStrain-Internal/mag-cal?include_prereleases&label=Latest)](https://github.com/HBK-MicroStrain-Internal/mag-cal/releases/latest)

<!-- Project Info -->
![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20Linux%20%7C%20macOS-blue)

<!-- Development/Standards -->
[![Keep a Changelog](https://img.shields.io/badge/changelog-Keep%20a%20Changelog%20v1.1.0-E05735)](https://keepachangelog.com/en/1.1.0/)

Suite of library and tools for magnetometer calibration.

## Calibration Coefficients
The following coefficients are used to provide corrections to the device:

### Hard-Iron Offset

$$
\mathbf{b} = \begin{bmatrix} b_x \\
b_y \\
b_z \end{bmatrix}
$$

where,

$$
\begin{aligned}
b_x &= \text{Hard-iron bias in x-axis (Gauss)} \\
b_y &= \text{Hard-iron bias in y-axis (Gauss)} \\
b_z &= \text{Hard-iron bias in z-axis (Gauss)}
\end{aligned}
$$

### Soft-Iron Matrix

$$
\mathbf{A} = \begin{bmatrix} a_{xx} & a_{xy} & a_{xz} \\
a_{yx} & a_{yy} & a_{yz} \\
a_{zx} & a_{zy} & a_{zz} \end{bmatrix}
$$

where,

$$
\begin{aligned}
\mathbf{A} = \text{Soft-iron (matrix)}
\end{aligned}
$$

## Usage
See the following usage guides:
* [Application](app/README.md#cli-usage)
* [Library](lib/README.md#usage)

### Fit Functions
The core functionality in this suite revolves around fitting the calibration coefficients. The currently
supported fit functions are:

| Function        | Correction                                                                  | Coefficients calibrated                                                                              |
|-----------------|-----------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------|
| Spherical fit   | $\mathbf{m}\_\text{corr} = s \cdot (\mathbf{m}\_\text{raw} - \mathbf{b})$   | - Hard-iron offset<br/> - Uniform scale factor                                                       |
| Ellipsoidal fit | $\mathbf{m}\_\text{corr} = \mathbf{A}(\mathbf{m}\_\text{raw} - \mathbf{b})$ | - Hard-iron offset<br/> - Non-uniform scale factor<br/>- Symmetric (non-uniform) cross-axis coupling |

where,

$$
\begin{aligned}
\mathbf{m}_\text{corr} &= \text{Corrected measurement (vector)} \\
\mathbf{m}_\text{raw} &= \text{Raw measurement (vector)} \\
\mathbf{b} &= \text{Hard-iron offset (vector)} \\
s &= \text{Uniform scale factor (scalar)} \\
\mathbf{A} &= \text{Soft-iron (matrix)}
\end{aligned}
$$

### When To Use Each

In general, *Spherical Fit* is more reliable but less accurate. *Ellipsoidal Fit* is much more
accurate, but requires high spatial coverage (*> 60%*).

A good rule of thumb is to start with *Spherical Fit* for a baseline. Then, an *Ellipsoidal Fit* can
be run to see if it improves calibration over the baseline.

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
