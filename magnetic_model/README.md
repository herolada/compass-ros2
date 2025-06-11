<!-- SPDX-License-Identifier: BSD-3-Clause -->
<!-- SPDX-FileCopyrightText: Czech Technical University in Prague -->

# magnetic\_model

This package contains utilities for working with the
[World Magnetic Model](https://en.wikipedia.org/wiki/World_Magnetic_Model), e.g. computing magnetic declination.

The package also embeds the IGRF-14 model for predictions dating back to 1900.

There is also special support for the Gazebo Magnetometer model, which is a special instance of IGRF-14.

## C++ Libraries

### magnetic\_model::MagneticModel

A class that can examine the Earth magnetic model and answer questions about it like field strength and components.

### magnetic\_model::MagneticModelManager

A manager class for `MagneticModel`s that can automatically load and return the most suitable model.