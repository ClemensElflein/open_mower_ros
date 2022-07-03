# VESC
## ROS Interfaces for VESC Motor Drivers

## Purpose of this fork
This implementation aims to provide a stable, high performance VESC driver implementation.

I've read through the original code and there were multiple parts which I didn't like about it. E.g. "TODO: we need a mutex here". Additionally, I didn't like the threading model of the driver implementation.

Additionally, I've added an abstraction layer so that we can drive multiple xESC motor controllers using the same ROS interface `xesc_driver`. Currently the xesc_mini and the xesc_2040 are supported.


## License
This repositry is licensed under the [Apache 2.0 License](https://www.apache.org/licenses/LICENSE-2.0.html).

## Note
A part of these packages had been developed by Michael T. Boulet at MIT under the BSD 3-clause License until Dec. 2016 in the repository named [mit-racecar/vesc](https://github.com/mit-racecar/vesc). Since Nov. 2019, Softbank Corp. takes over development as new packages.
