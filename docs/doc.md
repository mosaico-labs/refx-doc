---
sidebar_position: 1
---

# The `refx` Library

`refx` is a modern C++ header-only library for robust robotics and navigation applications. It leverages the C++ type system to enforce coordinate frame correctness at compile-time, preventing an entire class of common runtime errors by making invalid geometric operations a compilation failure.

The library's core philosophy is the separation of a frame's name, its axis structure, and its semantic classification. It provides a suite of frame-aware geometric types, a powerful transformation API, and a set of standard physical models for high-fidelity geodetic calculations.

## Key Features

  * **Compile-Time Safety**: Enforces frame-aware mathematics for vectors, rotations, and transformations. Operations between incompatible frames are caught by the compiler.
  * **Expressive Geometry Types**: Provides strongly-typed classes like [`Vector3D<Frame>`](lib_doc/geometry_h#vectors), [`Coordinate3D<Frame>`](lib_doc/geometry_h#coordinates), [`Rotation<To, From>`](lib_doc/geometry_h#rotation), and [`Transformation<To, From>`](lib_doc/geometry_h#transformations).
  * **Comprehensive Frame System**: Includes a rich set of predefined geodetic, local-tangent, body, and sensor frames ([`lla`](lib_doc/frames_h#latitude-longitude-altitude-lla), [`ned`](lib_doc/frames_h#north-east-down-ned), [`frd`](lib_doc/frames_h#forward-right-down-frd), [`imu`](lib_doc/frames_h#imu-imu), etc.) and is fully extensible with user-defined frames.
  * **High-Fidelity Models**: Ships with standard [geodetic models](lib_doc/models_h#earth-model), including [**WGS-84**](https://en.wikipedia.org/wiki/World_Geodetic_System) and [**GRS-80**](https://en.wikipedia.org/wiki/Geodetic_Reference_System_1980) [reference ellipsoids](lib_doc/models_h#reference-ellipsoid) and [gravity models](lib_doc/models_h#gravity-model), for accurate real-world calculations.
  * **Powerful Transformation API**: Offers a unified API with [`frame_cast`](lib_doc/transformations_h#co-origin-conversion-frame_cast) for simple co-origin conversions and [`frame_transform`](lib_doc/transformations_h#complex-transformations-frame_transform) for complex, model-based geodetic projections.

## Safety Through Types

In complex navigation systems, ensuring the semantic correctness of coordinate frames is critical. Implicit conventions often lead to runtime errors when, for example, a vector in a body frame is incorrectly added to a vector in a world frame.

`refx` prevents this by encoding the frame into the type itself. The following code will not compile, saving hours of potential debugging:

```cpp
#include <refx/geometry.h>

int main() {
    // A velocity vector in the world (NED) frame
    refx::Vector3D<refx::ned> velocity_world(10.0, 5.0, 0.0);

    // A velocity vector in the vehicle's body (FRD) frame
    refx::Vector3D<refx::frd> velocity_body(1.0, 0.0, 0.0);

    // COMPILE-TIME ERROR: static assertion failed due to different FrameTags.
    auto invalid_sum = velocity_world + velocity_body;

    return 0;
}
```
