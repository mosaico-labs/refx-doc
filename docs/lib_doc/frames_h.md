---
sidebar_position: 1
---

Frames
===

This header defines the architectural building blocks for the library's type-safe coordinate frame system. 
It introduces the concepts of **[Frame Tags](#frame-tags)** for high-level classification, **[Axis Systems](#axis-systems)** to describe a frame's geometric or semantic structure, and finally, the specific **[Frame](#frames-1)** definitions used throughout the library. The core architectural philosophy is the **separation of concerns** between a frame's semantic name (e.g., `ned`), its underlying axis structure and its semantic classification.

```cpp
// The gateway to the frame definitions
#include <refx/frames.h>
```

## Frame Tags

The `FrameTag` enum provides a high-level, semantic classification for different families of coordinate frames. This allows for compile-time checks to ensure that operations are only performed between conceptually compatible frames (e.g., preventing the direct conversion of a sensor-frame vector to a geocentric frame without an intermediate transformation).

```cpp
enum class FrameTag {
    Geocentric,   // frame-tag for geocentric (Earth-centered) coordinate frames.
    LocalTangent, // frame-tag for local tangent plane coordinate frames
    Body,         // frame-tag for frames rigidly attached to a moving body
    Sensor        // frame-tag for frames rigidly attached to a sensor mounted on a body
};
```

#### Geocentric

A tag for geocentric (Earth-centered) coordinate frames. These frames have their origin at the center of mass of the Earth and are used for absolute, planet-wide positioning.

  * **Examples**: [`ecef`](#earth-centered-earth-fixed-ecef) (Earth-Centered, Earth-Fixed), [`lla`](#latitude-longitude-altitude-lla) (Latitude, Longitude, Altitude).

#### LocalTangent

A tag for local tangent plane coordinate frames. These are Cartesian systems anchored to a specific point on the Earth's surface or on a moving platform, representing a "flat Earth" approximation valid for local-area navigation.

  * **Examples**: [`ned`](#north-east-down-ned) (North-East-Down), [`enu`](#east-north-up-enu) (East-North-Up).

#### Body

A tag for frames rigidly attached to a moving body. These frames have their origin at a reference point on a vehicle or object (e.g., its center of gravity) and move and rotate with it.

  * **Examples**: [`frd`](#forward-right-down-frd) (Forward-Right-Down), [`flu`](#forward-left-up-flu) (Forward-Left-Up).

#### Sensor

A tag for frames rigidly attached to a sensor mounted on a body. The relationship between a *sensor* frame and a *body* frame is defined by an extrinsic calibration [`Transformation`](geometry_h#transformations).

  * **Examples**: [`imu`](#imu-imu), [`camera`](#camera-camera).


## Axis Systems

The library separates a frame's semantic name (e.g., *ned*) from its underlying axis structure. This allows compile-time validation and enables generic algorithms to adapt their behavior based on a frame's properties.

### Component Mathematical Properties

The `AxisDomain` enum tags each component of a coordinate or vector with its mathematical properties, allowing arithmetic operators to apply the correct logic at compile time (e.g., linear subtraction vs. shortest-angle logic for angular elements).

```cpp
enum class AxisDomain {
    Linear,
    WrappedAngular90,
    WrappedAngular180,
    WrappedAngular360,
    WrappedAngularPi2,
    WrappedAngularPi,
    WrappedAngular2Pi
};
```

  * `Linear`: A standard Euclidean component.
  * `WrappedAngular90` / `WrappedAngularPi2`: An angular component clamped to a 90-degree (`[-90°, +90°]`) or `π/2` radian (`[-π/2, +π/2]`) range (e.g., Latitude).
  * `WrappedAngular180` / `WrappedAngularPi`: An angular component that wraps at ±180° or ±π radians (e.g., Longitude).
  * `WrappedAngular360` / `WrappedAngular2Pi`: An angular component that wraps at 360° or 2π radians (e.g., Azimuth).

### Directional (Cartesian) Axis

This system defines right-handed Cartesian frames using three orthogonal geometric directions from the `AxisDirection` enum.

```cpp
// Defines the six cardinal directions for constructing Cartesian axes.
enum class AxisDirection { 
    Up,
    Down, 
    Forw, 
    Back, 
    Left, 
    Right 
};
```
The library provides [all of the 24](frameaxis#directional-axis-cartesian) possible right-handed frame axis configurations.

```cpp
// A template "tag" struct to define a Cartesian axis system
template <AxisDirection x, AxisDirection y, AxisDirection z>
struct DirectionalAxis {};

// Example: Forward-Right-Down axis system
using axis_frd = DirectionalAxis<AxisDirection::Forw, AxisDirection::Right, AxisDirection::Down>;
// Example: Forward-Left-Up axis system
using axis_flu = DirectionalAxis<AxisDirection::Forw, AxisDirection::Left, AxisDirection::Up>;
```

### Semantic (Non-Cartesian) Axis

This system defines non-Cartesian frames using [semantic tags](frameaxis#semantic-axis-mixed-sphericallinear) for each component from the `AxisSemantic` enum.

```cpp
// Defines the physical meaning of a component in a non-Cartesian frame.
enum class AxisSemantic {
    Latitude, 
    Longitude, 
    Azimuth, 
    Elevation, 
    Altitude, 
    Down, 
    Range, 
    Length
};
```

With these tags, the axis configurations for non-Cartesian axis system can be defined.
```cpp
// A template "tag" struct to define a non-Cartesian axis system
template <AxisSemantic x, AxisSemantic y, AxisSemantic z>
struct SemanticAxis {};

// Example: Latitude-Longitude-Altitude axis system
using axis_lla = SemanticAxis<AxisSemantic::Latitude, AxisSemantic::Longitude, AxisSemantic::Altitude>;
// Example: Azimuth-Elevation-Range axis system
using axis_aer = SemanticAxis<AxisSemantic::Azimuth, AxisSemantic::Elevation, AxisSemantic::Range>;
```

## Frames

This is the central registry for all coordinate frames supported by the library. Each frame is an empty struct that provides a `name`, an `axis` definition, and a `tag` from `FrameTag`. These are the types used to parameterize [geometric](geometry_h) objects like [`Vector3D`](geometry_h#vectors) and [`Coordinate3D`](geometry_h#coordinates).

### Geocentric Frames

These frames have their origin at the center of mass of the Earth and are used for absolute, planet-wide positioning.

#### Earth-Centered, Earth-Fixed (ecef)

A global, right-handed Cartesian frame whose origin is at the Earth's center of mass. It rotates with the Earth.

  - **+X-axis**: Points to the intersection of the equator and the prime meridian.
  - **+Y-axis**: Points to 90° East longitude on the equator.
  - **+Z-axis**: Points to the North Pole.

<!-- end list -->

```cpp
struct ecef {
    static constexpr auto name = "ecef";
    using axis = axis_flu;
    static constexpr FrameTag tag = FrameTag::Geocentric;
};
```

#### Latitude, Longitude, Altitude (lla)

A geographic polar (non-Cartesian) coordinate system.

  - **Latitude**: Angle from the equatorial plane (-90° to +90°).
  - **Longitude**: Angle from the prime meridian (-180° to +180°).
  - **Altitude**: Height in meters above a reference ellipsoid.

<!-- end list -->

```cpp
struct lla {
    static constexpr auto name = "lla";
    using axis = axis_lla;
    static constexpr FrameTag tag = FrameTag::Geocentric;
};
```

#### Latitude, Longitude, Down (lld)

A variant of LLA where the third component is a positive-downward distance (depth).

  - **Latitude**: Angle from the equatorial plane (-90° to +90°).
  - **Longitude**: Angle from the prime meridian (-180° to +180°).
  - **Down**: Depth in meters below a reference ellipsoid.

<!-- end list -->

```cpp
struct lld {
    static constexpr auto name = "lld";
    using axis = axis_lld;
    static constexpr FrameTag tag = FrameTag::Geocentric;
};
```

### Local Tangent Frames

These are Cartesian systems anchored to a specific point on the Earth's surface or on a moving platform, representing a "flat Earth" approximation valid for local-area navigation.

#### North-East-Down (ned)

A right-handed Cartesian frame defined on a plane tangent to the Earth's surface.

  - **+X-axis**: Points to geographic North.
  - **+Y-axis**: Points to geographic East.
  - **+Z-axis**: Points Down, towards the Earth's center.

<!-- end list -->

```cpp
struct ned {
    static constexpr auto name = "ned";
    using axis = axis_frd;
    static constexpr FrameTag tag = FrameTag::LocalTangent;
};
```

#### East-North-Up (enu)

A right-handed Cartesian frame defined on a plane tangent to the Earth's surface.

  - **+X-axis**: Points to geographic East.
  - **+Y-axis**: Points to geographic North.
  - **+Z-axis**: Points Up, away from the Earth's center.

<!-- end list -->

```cpp
struct enu {
    static constexpr auto name = "enu";
    using axis = axis_rfu;
    static constexpr FrameTag tag = FrameTag::LocalTangent;
};
```

#### North-West-Up (nwu)

A right-handed Cartesian frame defined on a plane tangent to the Earth's surface.

  - **+X-axis**: Points to geographic North.
  - **+Y-axis**: Points to geographic West.
  - **+Z-axis**: Points Up.

<!-- end list -->

```cpp
struct nwu {
    static constexpr auto name = "nwu";
    using axis = axis_flu;
    static constexpr FrameTag tag = FrameTag::LocalTangent;
};
```

#### Wander Azimuth (`wa`, `wa_generic`)

A **Wander Azimuth (WA)** frame is a local-tangent, right-handed Cartesian frame whose orientation is not necessarily aligned with geographic North. It's commonly used in strapdown inertial navigation systems to avoid singularities at the poles. The frame's orientation is defined by its rotation from a North-aligned frame (like `ned` or `enu`) by a value called the **wander angle** ($\alpha$).

To support various system conventions, the library provides a flexible template, `wa_generic`, rather than a single hard-coded definition for a specific frame. This allows you to create WA frame variants that are rotationally consistent with any primary local-tangent frame (e.g., [`ned`](#north-east-down-ned), [`enu`](#east-north-up-enu)) by providing the appropriate [`DirectionalAxis`](#directional-cartesian-axis) as a template parameter.

```cpp
template <typename Axis>
struct wa_generic {
    static constexpr auto name = "wa";
    using axis = Axis;
    static constexpr FrameTag tag = FrameTag::LocalTangent;
};
```

For practical use, the library defines aliases for the two most common associations: `ned` and `enu`. Using these ensures the WA frame is rotationally equivalent and directly compatible with the chosen primary navigation frame.

  * `wa_ned`: Defines a WA frame with a `Forward-Right-Down` (`axis_frd`) system, making it compatible with `ned`.
  * `wa_enu`: Defines a WA frame with a `Right-Forward-Up` (`axis_rfu`) system, making it compatible with `enu`.
  * `wa`: The default alias for Wander Azimuth, which corresponds to the `ned`-based convention (`wa_ned`).

<!-- end list -->

```cpp
// A Wander-Azimuth frame rotationally consistent with the NED convention.
using wa_ned = wa_generic<axis_frd>;

// The standard alias for Wander-Azimuth frame (NED related).
using wa = wa_ned;

// A Wander-Azimuth frame rotationally consistent with the ENU convention.
using wa_enu = wa_generic<axis_rfu>;
```
#### Azimuth, Elevation, Range (aer)

A polar (non-Cartesian) coordinate system relative to a local origin.

  - **Azimuth**: Horizontal angle, typically clockwise from North.
  - **Elevation**: Vertical angle from the horizontal plane.
  - **Range**: Straight-line distance from the origin.

<!-- end list -->

```cpp
struct aer {
    static constexpr auto name = "aer";
    using axis = axis_aer;
    static constexpr FrameTag tag = FrameTag::LocalTangent;
};
```

### Body-Fixed Frames

These frames have their origin at a reference point on a vehicle or object (e.g., its center of gravity) and move and rotate with it.

#### Forward-Right-Down (frd)

A right-handed Cartesian frame attached to a moving vehicle.

  - **+X-axis**: Points out the front of the vehicle (roll axis).
  - **+Y-axis**: Points out the right side of the vehicle (pitch axis).
  - **+Z-axis**: Points down through the vehicle (yaw axis).

<!-- end list -->

```cpp
struct frd {
    static constexpr auto name = "frd";
    using axis = axis_frd;
    static constexpr FrameTag tag = FrameTag::Body;
};
```

#### Forward-Left-Up (flu)

A right-handed Cartesian frame attached to a moving vehicle.

  - **+X-axis**: Points out the front of the vehicle.
  - **+Y-axis**: Points out the left side of the vehicle.
  - **+Z-axis**: Points up through the vehicle.

<!-- end list -->

```cpp
struct flu {
    static constexpr auto name = "flu";
    using axis = axis_flu;
    static constexpr FrameTag tag = FrameTag::Body;
};
```

#### Right-Forward-Up (rfu)

A body-fixed frame used in computer vision and some robotics libraries.

  - **+X-axis**: Points out the right side of the vehicle.
  - **+Y-axis**: Points out the front of the vehicle.
  - **+Z-axis**: Points up through the vehicle.

<!-- end list -->

```cpp
struct rfu {
    static constexpr auto name = "rfu";
    using axis = axis_rfu;
    static constexpr FrameTag tag = FrameTag::Body;
};
```

#### Surge-Sway-Heave (ssh)

A frame used in marine robotics to describe body-fixed linear velocities, typically aligned with [`frd`](#forward-right-down-frd).

  - **+X-axis**: Forward/backward motion (Surge).
  - **+Y-axis**: Right/left motion (Sway).
  - **+Z-axis**: Down/up motion (Heave).

<!-- end list -->

```cpp
struct ssh {
    static constexpr auto name = "ssh";
    using axis = axis_frd;
    static constexpr FrameTag tag = FrameTag::Body;
};
```

### Sensor-Fixed Frames

These frames have their origin at a reference point on a specific sensor and move and rotate with it. The relationship between a *sensor* frame and a *body* frame is defined by an extrinsic calibration [`Transformation`](geometry_h#transformations).

#### IMU (imu)

Defines a standard reference frame for an Inertial Measurement Unit. The standard convention is [`frd`](#forward-right-down-frd) (Forward-Right-Down).

  - **+X-axis**: Forward.
  - **+Y-axis**: Right.
  - **+Z-axis**: Down.

<!-- end list -->

```cpp
struct imu {
    static constexpr auto name = "imu";
    using axis = axis_frd;
    static constexpr FrameTag tag = FrameTag::Sensor;
};
```

#### Camera (camera)

Defines a standard reference frame for a camera, with its origin at the optical center. The standard convention is *Right-Down-Forward*.

  - **+X-axis**: Points to the right.
  - **+Y-axis**: Points down.
  - **+Z-axis**: Points forward, out of the lens.

<!-- end list -->

```cpp
struct camera {
    static constexpr auto name = "camera";
    using axis = axis_rdf;
    static constexpr FrameTag tag = FrameTag::Sensor;
};
```

## Appendix: List of Directional Axis Types

The used conventiom for axis names is: `axis_<x-dir><y-dir><z-dir>`, e.g.
  * `axis_frd` means: `x` **f**orward, `y` **r**ight, `z` **d**own.
  * `axis_lbu` means: `x` **l**eft, `y` **b**ack, `z` **u**p.
  * and so on.

| Axis | Description |
| :--- | :--- |
| **X: Forward or Backward** | |
| `axis_frd` | `DirectionalAxis<AxisDirection::Forw, AxisDirection::Right, AxisDirection::Down>`|
| `axis_flu` | `DirectionalAxis<AxisDirection::Forw, AxisDirection::Left, AxisDirection::Up>`|
| `axis_fur` | `DirectionalAxis<AxisDirection::Forw, AxisDirection::Up, AxisDirection::Right>`|
| `axis_fdl` | `DirectionalAxis<AxisDirection::Forw, AxisDirection::Down, AxisDirection::Left>`|
| `axis_bru` | `DirectionalAxis<AxisDirection::Back, AxisDirection::Right, AxisDirection::Up>`|
| `axis_bld` | `DirectionalAxis<AxisDirection::Back, AxisDirection::Left, AxisDirection::Down>`|
| `axis_bul` | `DirectionalAxis<AxisDirection::Back, AxisDirection::Up, AxisDirection::Left>`|
| `axis_bdr` | `DirectionalAxis<AxisDirection::Back, AxisDirection::Down, AxisDirection::Right>`|
| **X: Right or Left** | |
| `axis_rfu` | `DirectionalAxis<AxisDirection::Right, AxisDirection::Forw, AxisDirection::Up>`|
| `axis_rbd` | `DirectionalAxis<AxisDirection::Right, AxisDirection::Back, AxisDirection::Down>`|
| `axis_rub` | `DirectionalAxis<AxisDirection::Right, AxisDirection::Up, AxisDirection::Back>`|
| `axis_rdf` | `DirectionalAxis<AxisDirection::Right, AxisDirection::Down, AxisDirection::Forw>`|
| `axis_lfd` | `DirectionalAxis<AxisDirection::Left, AxisDirection::Forw, AxisDirection::Down>`|
| `axis_lbu` | `DirectionalAxis<AxisDirection::Left, AxisDirection::Back, AxisDirection::Up>`|
| `axis_luf` | `DirectionalAxis<AxisDirection::Left, AxisDirection::Up, AxisDirection::Forw>`|
| `axis_ldb` | `DirectionalAxis<AxisDirection::Left, AxisDirection::Down, AxisDirection::Back>`|
| **X: Up or Down** | |
| `axis_urf` | `DirectionalAxis<AxisDirection::Up, AxisDirection::Right, AxisDirection::Forw>`|
| `axis_ulb` | `DirectionalAxis<AxisDirection::Up, AxisDirection::Left, AxisDirection::Back>`|
| `axis_ufl` | `DirectionalAxis<AxisDirection::Up, AxisDirection::Forw, AxisDirection::Left>`|
| `axis_ubr` | `DirectionalAxis<AxisDirection::Up, AxisDirection::Back, AxisDirection::Right>`|
| `axis_drb` | `DirectionalAxis<AxisDirection::Down, AxisDirection::Right, AxisDirection::Back>`|
| `axis_dlf` | `DirectionalAxis<AxisDirection::Down, AxisDirection::Left, AxisDirection::Forw>`|
| `axis_dfr` | `DirectionalAxis<AxisDirection::Down, AxisDirection::Forw, AxisDirection::Right>`|
| `axis_dbl` | `DirectionalAxis<AxisDirection::Down, AxisDirection::Back, AxisDirection::Left>`|

## Appendix: List of Semantic Axis Types

| Axis | Description |
| :--- | :--- |
| `axis_lla` | `SemanticAxis<AxisSemantic::Latitude, AxisSemantic::Longitude, AxisSemantic::Altitude>`|
| `axis_lld` | `SemanticAxis<AxisSemantic::Latitude, AxisSemantic::Longitude, AxisSemantic::Down>`|
| `axis_aer` | `SemanticAxis<AxisSemantic::Azimuth, AxisSemantic::Elevation, AxisSemantic::Range>`|
