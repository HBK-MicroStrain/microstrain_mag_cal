# Changelog
This project follows [Semantic Versioning](https://semver.org/).

## Unreleased

### Added 
- `libmicrostrain-mag-cal`: PointManager metrics for tracking total number of points seen and number of points after 
                            filtering.

### Changed
- `libmicrostrain-mag-cal`: Changed target constructor for PointManager to pass by value instead of reference.

## 0.3.0-pre-alpha - 2025-12-18

_First major changes from feedback_.

### Added
- `libmicrostrain-mag-cal`: Factory method to get a FitResult with error and no correction to apply.
- `libmicrostrain-mag-cal`: Utilities for JSON serialization/deserialization for FitResult.
- `libmicrostrain-mag-cal`: Utility to convert eigen containers to std::vector.
- `microstrain-mag-cal`: Option to output calibration results to json file(s).
- `microstrain-mag-cal`: Verbose analysis option for data analysis output.
- `microstrain-mag-cal-apply`: Separate CLI tool to apply calibrations to a device.
- `Documentation`: Guide for new apply tool to application usage guide.
- `Documentation`: Example workflow guide for both app tools together.
- `Documentation`: Section for calibration coefficients to README.
- `Documentation`: Section for when to use each fit function to README.

### Changed
- `Documentation`: Renamed app and library READMEs to USAGE_GUIDE.
- `Documentation`: Made more comprehensive guide for getting reference field strength from NOAA. 
- `microstrain-mag-cal`: Moved spatial coverage option into verbose analysis option.
- `microstrain-mag-cal`: Moved other analysis outputs into verbose analysis option.
- `microstrain-mag-cal`: Updated app run output and help message.
- New dependency on [nlohmann/json](https://github.com/nlohmann/json/tree/v3.12.0?tab=readme-ov-file#serialization--deserialization).
- New dependency on [Neargye/magic_enum](https://github.com/Neargye/magic_enum).

## 0.2.0-pre-alpha - 2025-12-11

_Interface, packaging, and documentation changes. No new features were introduced._

### Changed
- Set library and application to be built, packaged, and released together.
- Significantly updated documentation.
- Refactored some interface names.

### Added
- Calibration library
- Offline mag cal cli tool
- Script to apply coefficients to the device

## 0.1.0-pre-alpha - 2025-12-03

_First release._

### Added
- Calibration library 
- Offline mag cal cli tool 
- Script to apply coefficients to the device 
