# Library User Guide

## Usage
The library is organized into three modules:

| Module                                                 | Purpose                                    |
|--------------------------------------------------------|--------------------------------------------|
| [Preprocessing](microstrain_mag_cal/preprocessing.hpp) | Filters and prepares data for calibration. |
| [Calibration](microstrain_mag_cal/calibration.hpp)     | Estimates calibration coefficients.        |
| [Analysis](microstrain_mag_cal/analysis.hpp)           | Analyzes data and evaluates fit quality.   |

## Building Manually
Make sure the project has been [configured](../README.md#building-manually) first. Once configured, build the library:
```
cmake --build build/library
```

After building, run this if you would like to create a distributable package as well:
```
cmake --build build/library --target package_microstrain_mag_cal
```

The package will be in `build/library`.
