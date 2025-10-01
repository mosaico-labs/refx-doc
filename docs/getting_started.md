---
sidebar_position: 2
---

# Getting Started

Let's discover **refx in less than 5 minutes**.

Using `refx` is straightforward. The following steps demonstrate the core features of the library: creating frame-aware vectors and coordinates, leveraging compile-time safety, and performing transformations.

## Installation

### CMake
Here's the code you'd add to your project's `CMakeLists.txt` file to import the refx library.
```cmake
include(FetchContent)
FetchContent_Declare(
  refx
  GIT_REPOSITORY https://github.com/mosaico-labs/refx.git/
  GIT_TAG        v0.1.0
)
FetchContent_MakeAvailable(refx)
```

From this point on, the `refx::refx` target is available to your project and this command can be used to to link refx.
```cmake
target_link_libraries(my_target PRIVATE refx::refx)
```

### Conan
You can install refx using Conan via the [klin](https://github.com/conan-kiln/kiln) repository.

- Set up Conan Kiln as a remote following the [instructions](https://github.com/conan-kiln/kiln#setup)
- Include `refx/[>=0.2.1 <1]` as a requirement under your `conanfile.txt` or `conanfile.py`.
- Follow the regular [instructions](https://docs.conan.io/2/tutorial.html) for integrating Conan packages into your project

Special thanks to [@valgur](https://github.com/valgur) for adding Conan support.

## Requirements

Developing with refx requires only a C++17 compatible compiler. Running library unit-tests requires [Google Test](https://github.com/google/googletest) as dependency (downloaded automatically by CMake).

## Ecosystem Integration
refx is designed to be a self-contained, lightweight library. However, for maximum utility, it provides optional, first-class support for [Eigen3](short_doc#appendix-eigen3-integration).

## Hands-On Implementation

The code below simulates a common scenario: taking a vehicle's body-frame velocity, rotating it into the world frame, and projecting a global GPS coordinate into a local navigation frame.

```cpp
//getting_started.cpp
#include <iostream>
#include <cmath>
#include <refx/geometry.h>  // for Vector3D, Coordinate3D, Rotation and YawPitchRoll
#include <refx/transformations.h> // frame_cast, frame_transform

using namespace refx;

int main() {
    // 1. Create frame-aware vectors. Types are tagged with their frame.
    Vector3D<ned> velocity_ned{10.0, -2.0, 0.5};  // {N, E, D}
    Vector3D<frd> omega_body{0.0, 0.0, 0.03};     // {F, R, D}

    std::cout << "Velocity in NED frame: " << velocity_ned << std::endl;

    // 2. COMPILE-TIME SAFETY: Mixing frames is a compiler error.
    // Uncommenting the line below will cause a compile-time error, preventing a common bug.
    // velocity_ned + thrust_in_body;  // ERROR: Incompatible frames!

    // 3. Define a rotation from the body frame to the world (NED) frame.
    // Let's assume a 45-degree yaw (pi/4 radians).
    auto yaw_pitch_roll = YawPitchRoll<double>(M_PI / 4.0, 0.0, 0.0);
    auto R_world_from_body = Rotation<ned, frd>(yaw_pitch_roll);

    // 4. Correctly transform the thrust vector to the world frame.
    Vector3D<frd> velocity_body = R_world_from_body.inverse() * velocity_ned;
    // This will generate a compile-time error:
    // Vector3D<frd> velocity_body = R_world_from_body * velocity_ned; //need .inverse() to rotation
    // Vector3D<flu> velocity_body = R_world_from_body.inverse() * velocity_ned; //result is <frd>

    std::cout << "Velocity in Body Frame: " << velocity_body << std::endl;

    // 5. Compute centripetal acceleration.
    Vector3D<frd> centripetal_acceleration_body = cross(omega_body, velocity_body);
    std::cout << "Centripetal Acceleration Body:  " << centripetal_acceleration_body << std::endl;

    std::cout << "\n--- Coordinate Transformations ---\n" << std::endl;

    // 6. Use `frame_cast` for simple, co-origin conversions (e.g., NED -> ENU).
    // This is a zero-cost, compile-time operation.
    Vector3D<enu> velocity_in_enu = frame_cast<enu>(velocity_ned);
    std::cout << "Velocity in ENU: " << velocity_in_enu << std::endl;

    // 7. Use `frame_transform` for complex projections (e.g., LLA -> NED).
    // This requires a physical context (an origin point).
    // Let's use a location in Rome, Italy.
    Coordinate3D<lla> local_origin(41.9028, 12.4964, 50.0);  // Lat/Lon in deg, Alt in m
    Coordinate3D<lla> target_gps_point(41.9030, 12.4966, 60.0);

    auto earthmodel = EarthModelWGS84<double>();

    // This performs a runtime projection of the GPS point onto a flat plane at the origin.
    Coordinate3D<ned> target_in_local_ned =
        frame_transform<ned>(target_gps_point, local_origin, earthmodel);

    std::cout << "Target GPS in Local NED Frame: " << target_in_local_ned << " (meters)"
              << std::endl;   
    // Convert a LLA position to ECEF
    auto local_origin_ecef = frame_transform<ecef>(local_origin, earthmodel);
    std::cout << "Local Origin in ECEF: " << local_origin_ecef << std::endl;

    // The same local NED position can be obtained with the same origin expressed in ECEF frame
    target_in_local_ned = frame_transform<ned>(target_gps_point, local_origin_ecef, earthmodel);
    std::cout << "Target GPS in Local NED Frame (from ecef origin): " << target_in_local_ned
              << " (meters)" << std::endl;

    return 0;
}
```

### Expected Output

```
Velocity in NED frame: [10, -2, 0.5]
Velocity in Body Frame: [5.65685, -8.48528, 0.5]
Centripetal Acceleration Body:  [0.254558, 0.169706, -0]

--- Coordinate Transformations ---

Velocity in ENU: [-2, 10, -0.5]
Local Origin in ECEF: [4.64162e+06, 1.02872e+06, 4.23761e+06]
Target GPS in Local NED Frame: [22.2145, 16.5955, -10] (meters)
Target GPS in Local NED Frame (from ecef origin): [22.2145, 16.5955, -10] (meters)
```

