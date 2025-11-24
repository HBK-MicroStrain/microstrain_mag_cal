# Offline Magnetometer Calibration
MVP library and commandline application for offline magnetometer calibration.

## CLI Usage
<!-- TODO: Add full interface usage docs -->
The CLI is run by the following command:
```
./offline_mag_cal [OPTIONS]
```

For a list of options, run:
```
./offline_mag_cal --help
```

### Example
This example reads magnetometer data from the file `mag_cal_data.bin` and runs spherical and 
ellipsoidal fits on the data:
```
./offline-mag-cal "mag_cal_test.bin" -s -e
```

### Reference Field Strength
Specifying the reference field strength for the location where data is sampled from will improve 
calibration. The reference field strength can be retrieved from the
[NOAA calculator](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm). 

It will be the `Total Field` attribute in the calculator results.
