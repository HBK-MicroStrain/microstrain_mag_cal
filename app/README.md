# Application User Guide

## Calibration Fitting
Calibration fitting is done through the `microstrain-mag-cal` tool. For a list of options, run:
```
microstrain-mag-cal --help
```

### Example
This example reads magnetometer data from the file `mag_cal_data.bin` and runs spherical and
ellipsoidal fits on the data using the reference field strength:
```
microstrain-mag-cal "mag_cal_example.bin" -f 0.12345 -s -e
```

### Reference Field Strength
The tool will automatically estimate the field strength given the input data if no reference field
strength is given. 

However, specifying the reference field strength for the location where data is sampled from will 
greatly improve calibration.

The reference field strength can be retrieved from the
[NOAA calculator](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm). It will be
the `Total Field` attribute in the calculator results:

![NOAA calculator total field attribute](../images/NOAA_total_field.png)

 The result will likely be in
[Nanotesla](https://en.wikipedia.org/wiki/Tesla_(unit)), while the application expects it
to be [Gauss](https://en.wikipedia.org/wiki/Gauss_(unit)). This conversion must be
done manually; the application does not support it automatically (yet).

The conversion is done using the following formula:
```
R_Gauss = R_Nanotesla * 0.00001
```

It is recommended to use the *World Magnetic Model* (WMM) for the model in the calculator.
