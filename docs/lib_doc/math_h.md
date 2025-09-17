---
sidebar_position: 5
---

Math
===

This header provides a collection of lightweight utilities for performing base mathematical operations.

```cpp
// The gateway to the math utilities
#include <refx/math.h>
```

## Angle

These functions are basic angular operations and conversions.

### Angle Unit Specifier

The `AngleUnit` enum provides a type-safe way to specify the desired angular units in function parameters, preventing ambiguity between degrees and radians.

**API**

```cpp
enum class AngleUnit {
    Deg,  ///< Angle is expressed in degrees.
    Rad   ///< Angle is expressed in radians.
};
```

This is used, for example, in the accessor methods of [`Coordinate3D<lla>`](geometry_h#latitude-longitude-altitude) to retrieve latitude or longitude in the desired format.

-----

### Angular Conversion Functions

The library provides a set of simple, inline-free functions for converting between the most common angular representations used in robotics and geodesy.

#### Degrees ↔ Radians

These functions are essential for interfacing with standard C++ trigonometric functions (e.g., `std::sin`, `std::cos`), which require inputs in radians, and for converting results back to a human-readable degree format.

**API**

  * `T deg2rad(T deg)`: Converts an angle from decimal degrees to radians.
  * `T rad2deg(T rad)`: Converts an angle from radians to decimal degrees.

**Usage**

```cpp
// Convert 45.0 degrees to radians for use in a trig function
double angle_deg = 45.0;
double angle_rad = refx::deg2rad(angle_deg); // Result: ~0.7854

// Convert the output of atan2 back to degrees
double angle_deg_again = refx::rad2deg(angle_rad); // Result: 45.0
```

-----

#### Decimal Degrees ↔ DMS

These functions convert between a floating-point decimal degree representation and the traditional **DMS (Degrees, Minutes, Seconds)** sexagesimal format used for geographic coordinates.

**API**

  * `std::tuple<int, int, T> degToDMS(T decimal_deg)`: Decomposes a decimal degree value into its integer degrees, integer minutes, and floating-point seconds components.
  * `T DMSToDeg(int deg, int min, T sec)`: Composes a decimal degree value from its DMS components, correctly handling negative degree values.

**Usage**

```cpp
// A latitude value in decimal degrees
double lat_decimal = 44.49421;

// Convert to Degrees, Minutes, Seconds
auto [d, m, s] = refx::degToDMS(lat_decimal);
// Result: d=44, m=29, s=~39.156

// Convert back to decimal degrees
double lat_decimal_again = refx::DMSToDeg(d, m, s); // Result: 44.49421
```