# Changelog
This project follows [Semantic Versioning](https://semver.org/).

## [0.6.0-alpha] - 2026-01-16

_First alpha release._

### Changed
- `libmicrostrain-mag-cal`: Changed hard-iron offset in FitResult to be a column vector instead of a row 
                            vector to match common format in papers.
- `everything`: Updated interfaces to take in the hard-iron offset as a column vector instead of a 
                row vector.
- `libmicrostrain-mag-cal`: Updated documentation for fit functions to explicitly state that they return
                            the correction matrix and not the distortion matrix for soft-iron.
  - `libmicrostrain-mag-cal`: Moved separate `libmicrostrain-mag-cal-apply` into `apply` module.
  
### Removed
- Separate `libmicrostrain-mag-cal-apply` library.

## [0.5.0-pre-alpha] - 2026-01-12

_Added calibration composition support and fixed fit algorithm issues._

### Added
- `libmicrostrain-mag-cal-apply`: New library module dedicated to features used in applying calibrations
                                  to a device.
- `libmicrostrain-mag-cal-apply`: Function to compose two calibration corrections into a single combined 
                                  correction. The composed correction can then be written to the device
                                  in a single step. Used to apply a new calibration on top of an old
                                  one (if the data sampled for the new calibration came from already
                                  calibrated data).
- `microstrain-mag-cal-apply`: Command-line argument to bypass the default composing behavior and 
                               overwrite the old calibration completely.
- `libmicrostrain-mag-cal-apply`: Function to convert from raw row-major data representation of the 
                                  soft-iron matrix to Eigen matrix representation (useful for 
                                  converting raw data from the device to work with the library).
- `libmicrostrain-mag-cal-apply`: Function to convert from raw data representation of the hard-iron 
                                  offset to Eigen matrix representation (useful for converting raw 
                                  data from the device to work with the library).
                                  
### Changed
- `microstrain-mag-cal-apply`: Changed the default behavior for applying calibrations. The new default
                               composes the new calibration onto the old one before applying. This is
                               done automatically.
                               
### Fixed
- `libmicrostrain-mag-cal`: Fit algorithms returning an error when they converged early.
- `libmicrostrain-mag-cal`: Warnings of potentially escaped addresses with FitResult.


## [0.4.0-pre-alpha] - 2025-12-19

_Updated interface to provide more descriptive metrics for used points_.

### Added 
- `libmicrostrain-mag-cal`: PointManager metrics for tracking total number of points seen, number of points after 
                            filtering, and the point retention percentage after filtering.

### Changed
- `libmicrostrain-mag-cal`: Changed target constructor for PointManager to pass by value instead of reference.
- `libmicrostrain-mag-cal`: Made `PointManager::getMatrix()` const-qualified.
- `microstrain-mag-cal`: Points used output changed to the following format: `POINTS_USED / TOTAL_POINTS (PERCENTAGE%)`

## [0.3.0-pre-alpha] - 2025-12-18

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

## [0.2.0-pre-alpha] - 2025-12-11

_Interface, packaging, and documentation changes. No new features were introduced._

### Changed
- Set library and application to be built, packaged, and released together.
- Significantly updated documentation.
- Refactored some interface names.

### Added
- Calibration library
- Offline mag cal cli tool
- Script to apply coefficients to the device

## [0.1.0-pre-alpha] - 2025-12-03

_First release._

### Added
- Calibration library 
- Offline mag cal cli tool 
- Script to apply coefficients to the device 
