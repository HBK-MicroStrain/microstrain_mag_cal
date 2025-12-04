# Library User Guide

<!-- TODO: Convert to library usage guide
## CLI Usage
The CLI is run by the following command:
```
offline_mag_cal [OPTIONS]
```

For a list of options, run:
```
offline_mag_cal --help
```

### Example
This example reads magnetometer data from the file `mag_cal_data.bin` and runs spherical and
ellipsoidal fits on the data:
```
offline-mag-cal "mag_cal_test.bin" -s -e
```

### Reference Field Strength
Specifying the reference field strength for the location where data is sampled from will improve
calibration.

The reference field strength can be retrieved from the
[NOAA calculator](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm). It will be
the `Total Field` attribute in the calculator results. The result will likely be in
[Nanotesla](https://en.wikipedia.org/wiki/Tesla_(unit)), while the application expects it
to be [Gauss](https://en.wikipedia.org/wiki/Gauss_(unit)). This conversion must be
done manually; the application does not support it automatically (yet).

The conversion is done using the following formula:
```
R_Gauss = R_Nanotesla * 0.00001
```

It is recommended to use the *World Magnetic Model* (WMM) for the model in the calculator.

-->

## Building Manually
Make sure the project has been [configured](../README.md#building-manually) first. Once configured, build the library:
```
cmake --build build/library
```

After building, run this if you would like to create a distributable package as well:
```
cmake --build build/application --target package_offline_mag_cal_cli
```