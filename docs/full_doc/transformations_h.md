---
sidebar_position: 4
---

Transformations
===

This header provides the high-level user-facing API for converting and transforming geometric entities between different reference frames. The library makes a distinction between two types of operations:
1.  [**`frame_cast`**](#co-origin-conversion-frame_cast): For simple, compile-time conversions between [`DirectionalAxis`](frames_h#directional-cartesian-axis) frames that **share the same origin** (co-origin). This involves only axis shuffling.
2.  [**`frame_transform`**](#complex-transformations-frame_transform): For complex transformations that may involve a **change of origin**, projection, or require a physical [`EarthModel`](models_h#earth-model) as context.


```cpp
// The gateway to the transformation functions
#include <refx/transformations.h>
```

## Co-Origin Conversion: `frame_cast`

The `frame_cast` function is a type-safe utility for re-expressing a [`Vector3D`](geometry_h#vectors) or [`Coordinate3D`](geometry_h#coordinates) in a new reference frame that **shares the same origin** as the source frame. It uses a syntax similar to C++'s `static_cast` for an expressive and intuitive API. This operation is restricted to conversions between [`DirectionalAxis`](frames_h#directional-cartesian-axis) (Cartesian) frames that share the same origin and [`FrameTag`](frames_h#frame-tags), such as [**NED ↔ ENU**](frames_h#local-tangent-frames) or [**FRD ↔ FLU**](frames_h#body-fixed-frames). This constraint ensures the transformation is a simple, constant rotation that relies only on axis shuffling and does not require a physical context like an [`EarthModel`](models_h#earth-model).

The template-based API takes an object expressed in a source frame (`frameFrom`) and returns the equivalent object re-expressed in the destination frame (`frameTo`). 

```cpp
template <typename frameTo, typename frameFrom, /* ... */>
VecType<frameTo, T> frame_cast(const VecType<frameFrom, T>& from);
```
#### API Template Parameters

  * **`frameFrom` / `frameTo`**
      * **Constraint**: Must be [`DirectionalAxis`](frames_h#directional-cartesian-axis) (Cartesian) frames that share the same origin and [`FrameTag`](frames_h#frame-tags).
      * **Valid Types by `FrameTag`**:
          * [**`LocalTangent`**](frames_h#localtangent): [`ned`](frames_h#north-east-down-ned), [`enu`](frames_h#east-north-up-enu), [`nwu`](frames_h#north-west-up-nwu), *`user-defined`*;
          * [**`Body`**](frames_h#body): [`frd`](frames_h#forward-right-down-frd), [`flu`](frames_h#forward-left-up-flu), [`rfu`](frames_h#right-forward-up-rfu), [`ssh`](frames_h#surge-sway-heave-ssh), *`user-defined`*;
          * [**`Sensor`**](frames_h#sensor): [`imu`](frames_h#imu-imu), [`camera`](frames_h#camera-camera), *`user-defined`*
  * **`VecType`** (Geometric Container, *deduced*)
      * **Valid Types**: [`Vector3D`](geometry_h#vectors) or [`Coordinate3D`](geometry_h#coordinates).

**Usage**

```cpp
// A vector representing velocity in the NED local-tangent frame
refx::Vector3D<refx::ned> vel_ned(10.0, -5.0, 2.0); // {North, East, Down}

// Use frame_cast to re-express the same velocity in the ENU local-tangent frame
auto vel_enu = refx::frame_cast<refx::enu>(vel_ned);
// vel_enu now contains {-5.0, 10.0, -2.0} // {East, North, Up}

// A vector representing the position of a target object in the FRD body frame
refx::Coordinate3D<refx::frd> target_body(10.0, -5.0, 2.0); // {Forward, Right, Down}
// Use frame_cast to re-express the same postion in the FLU body frame
auto target_flu = refx::frame_cast<refx::flu>(target_body); // result is Coordinate3D

```

:::info
The API automatically supports any [user-defined](../short_doc#types-customization) [`DirectionalAxis`](frames_h#directional-cartesian-axis) frame without requiring extension of the library code.
:::

**Frame-safety**

If a conversion between the specified frames is not allowed, a compile-time error will be generated, ensuring frame safety.

```cpp
// This is a compile-time error! 'FrameTag' of 'ned' and 'frd' are different
auto vel_frd = refx::frame_cast<refx::frd>(vel_ned);
                                     ^---^
```

## Complex Transformations: `frame_transform`

The `frame_transform` function is an overloaded utility designed for complex transformations that go beyond simple axis-shuffles. These operations often require a physical context, such as an [`EarthModel`](models_h#earth-model) to account for the planet's curvature or a [`Transformation`](geometry_h#transformations) object to define a complete pose.


### Geocentric Transformations (e.g., LLA ↔ ECEF)

This overload handles transformations between frames with [`FrameTag::Geocentric`](frames_h#geocentric-frames), sharing the Earth's center as their origin. The conversion requires an [`EarthModel`](models_h#earth-model) to provide the parameters of the reference ellipsoid (e.g., [WGS-84](models_h#wgs-84-2)).

```cpp
template <typename frameTo, typename frameFrom, /* ... */>
Coordinate3D<frameTo, T> frame_transform(const Coordinate3D<frameFrom, T>& from,
                                         const EarthModel<T>& em);
```

#### API Template Parameters

  * **`frameFrom` / `frameTo`**
      * **Constraint**: Must be frames with [`FrameTag::Geocentric`](frames_h#geocentric) frame-tag.
      * **Valid Types**: [`lla`](frames_h#latitude-longitude-altitude-lla), [`lld`](frames_h#latitude-longitude-down-lld), [`ecef`](frames_h#earth-centered-earth-fixed-ecef);
  * Accept only [`Coordinate3D`](geometry_h#coordinates) geometric types.

**Usage**

```cpp
// Define the Earth model (WGS-84)
refx::EarthModelWGS84<double> earth;

// A geodetic coordinate (Latitude, Longitude, Altitude)
refx::Coordinate3D<refx::lla> lla_coord(45.0, -75.0, 500.0);

// Transform the LLA coordinate to the ECEF (Earth-Centered, Earth-Fixed) frame
auto ecef_coord = refx::frame_transform<refx::ecef>(lla_coord, earth);
```

:::note
In the versions 0.2.x of the library, the current transformation should be used for converting between `lla` and `lld` also, despite the two frames do not need the `EarthModel`context and a simple axis-shuffle would be sufficient for conversion. This issue will be fixed in version 0.3.0.

The customization of this transformation for [user-defined](../short_doc#types-customization) frames with [`FrameTag::Geocentric`](frames_h#geocentric) frame-tag needs extension of the library and it is still **not supported**.
:::

**Frame-safety**

If a conversion between the specified frames is not allowed, a compile-time error will be generated, ensuring frame safety.

```cpp
// This is a compile-time error: invalid 'FrameTag' for 'ned' (`LocalGeodetic`, must be `Geocentric`);
// moreover the two frames are not co-origin.
auto ned_coord = refx::frame_transform<refx::ned>(lla_coord, earth);
                                            ^---^
```

### Geocentric ↔ Local-Tangent Transformations (e.g., LLA ↔ NED)

This overload performs the projection of a global geodetic coordinate onto a local "flat Earth" tangent plane (like [`ned`](frames_h#north-east-down-ned) or [`enu`](frames_h#east-north-up-enu)), or the inverse. This requires two pieces of context:

1.  **`origin`**: A geodetic coordinate defining the origin position of the local tangent frame on the Earth.
2.  **`em`**: The [`EarthModel`](models_h#earth-model) to provide the necessary geodetic parameters for the projection.

```cpp
template <typename frameTo, typename frameFrom, /* ... */>
Coordinate3D<frameTo, T> frame_transform(const Coordinate3D<frameFrom, T>& from,
                                         const Coordinate3D<frameOrigin, T>& origin,
                                         const EarthModel<T>& em);
```
#### API Template Parameters

* **`frameFrom` / `frameTo`**
    * **Constraint**: One must be a [**`FrameTag::Geocentric`**](frames_h#geocentric) frame and the other must be a [**`FrameTag::LocalTangent`**](frames_h#localtangent) [`DirectionalAxis`](frames_h#directional-cartesian-axis) frame.
        * **Accepted Geocentric Types**: [`lla`](frames_h#latitude-longitude-altitude-lla), [`lld`](frames_h#latitude-longitude-down-lld), [`ecef`](frames_h#earth-centered-earth-fixed-ecef).
        * **Accepted Local-Tangent Types**: [`ned`](frames_h#north-east-down-ned), [`enu`](frames_h#east-north-up-enu), [`nwu`](frames_h#north-west-up-nwu), *`user-defined`* frames.

* **`frameOrigin`** (Frame Tag)
    * **Constraint**: Must be a [**`FrameTag::Geocentric`**](frames_h#geocentric) frame.
    * **Accepted Types**: [`lla`](frames_h#latitude-longitude-altitude-lla), [`lld`](frames_h#latitude-longitude-down-lld), [`ecef`](frames_h#earth-centered-earth-fixed-ecef).
* Accept only [`Coordinate3D`](geometry_h#coordinates) geometric types.

**Usage**

```cpp
// Define the Earth model and the origin of the local NED frame
refx::EarthModelWGS84<double> earth;
refx::Coordinate3D<refx::lla> local_origin(44.49, 11.34, 100.0);

// A target point defined in global LLA coordinates
refx::Coordinate3D<refx::lla> target_lla(44.491, 11.341, 120.0);

// Transform the global target point into the local NED frame relative to the origin
auto target_ned = refx::frame_transform<refx::ned>(target_lla, local_origin, earth);

// Transform back the local NED frame relative to the origin into the global target point 
auto target_lla_again = refx::frame_transform<refx::lla>(target_ned, local_origin, earth);
```

:::note
The API automatically supports any [user-defined](../short_doc#types-customization) [`DirectionalAxis`](frames_h#directional-cartesian-axis) frames with [`FrameTag::LocalTangent`](frames_h#localtangent) without requiring extension of the library.

The customization of this transformation for user-defined frames with [`FrameTag::Geocentric`](frames_h#geocentric) needs extension of the library and it is still **not supported**.

The current version of the library does not allow to pass [**`FrameTag::LocalTangent`**](frames_h#localtangent) frame with [`SemanticAxis`](frames_h#semantic-non-cartesian-axis) axis category, like the [`Azimuth-Elevation-Range (aer)`](frames_h#azimuth-elevation-range-aer) frame. This issue will be fixed in future versions. With the current version of the library, any transformation `aer`↔*Geocentric* can be made by chaining the conversion via an intermediate Cartesian local-tangent frame (like `ned`): `aer`↔**`ned`**↔*Geocentric*. The first conversion can be made by calling the [**Polar ↔ Cartesian Transformation** API](#polar--cartesian-transformations-eg-ned--aer), and then perform the final conversion `ned`↔*Geocentric* as already showed in this section.
:::

**Frame-safety**

If a conversion between the specified frames is not allowed, a compile-time error will be generated, ensuring frame safety.

```cpp
// This is a compile-time error: 'FrameTag' of return must be `LocalTangent`
auto target_frd = refx::frame_transform<refx::frd>(target_lla, local_origin, earth);
                                             ^---^
```

### Polar ↔ Cartesian Transformations (e.g., NED ↔ AER)

This overload is used to transform a point from a frame with [**`axis_aer`**](frameaxis#semantic-axis-mixed-sphericallinear) axis configuration to a frame with  [`DirectionalAxis`](frames_h#directional-cartesian-axis) (Cartesian) axis that share the same origin and [`FrameTag`](frames_h#frame-tags), or vice-versa.

```cpp
template <typename frameTo, typename frameFrom, /* ... */>
VecType<frameTo, T> frame_transform(const VecType<frameFrom, T>& from);
```
#### API Template Parameters

* **`frameFrom` / `frameTo`**
    * **Constraints**: One must be [`DirectionalAxis`](frames_h#directional-cartesian-axis) (Cartesian) frame, one must be [`axis_aer`](frameaxis#semantic-axis-mixed-sphericallinear) frame. Both must share the same origin and [`FrameTag`](frames_h#frame-tags).
    * **Accepted Types**: [`ned`](frames_h#north-east-down-ned), [`enu`](frames_h#east-north-up-enu), [`nwu`](frames_h#north-west-up-nwu), [`aer`](frames_h#azimuth-elevation-range-aer), *`user-defined`* frames.
* **`VecType`** (Geometric Container, *deduced*)
      * **Valid Types**: [`Vector3D`](geometry_h#vectors) or [`Coordinate3D`](geometry_h#coordinates).

**Usage**

```cpp
// A point defined in the local NED frame
refx::Coordinate3D<refx::ned> point_ned(100.0, 100.0, -50.0);

// Transform the Cartesian point to Polar AER (Azimuth, Elevation, Range) coordinates
auto point_aer = refx::frame_transform<refx::aer>(point_ned);

// point_aer.azimuth() will be ~45 degrees
// point_aer.range() will be ~150 meters
```


:::tip
While the library's only built-in frame in [`axis_aer`](frameaxis#semantic-axis-mixed-sphericallinear) axis semantics is [`aer`](frames_h#azimuth-elevation-range-aer), the API will automatically support any [user-defined](../short_doc#types-customization) frame that uses the same `axis_aer` semantics.
For example, the user could define a `radar` [`FrameTag::Sensor`](frames_h#sensor) frame and directly transform its coordinates into another (Cartesian) **sensor** frame, such as [`imu`](frames_h#imu-imu):
```cpp
// file: my_frames.h
// Define a radar frame using the azimuth-elevation-range axis configuration.
struct radar {
   static constexpr auto name = "radar";
   using axis = refx::axis_aer;
   static constexpr FrameTag tag = refx::FrameTag::Sensor;
};
// file: main.cpp
// This code compiles and works without any library modification.
refx::Coordinate3D<radar> p_radar(45.0, 30.0, 100.0);
auto p_imu = refx::frame_transform<refx::imu>(p_radar);
```
:::

**Frame-safety**

If a conversion between the specified frames is not allowed, a compile-time error will be generated, ensuring frame safety.

```cpp
// This is a compile-time error: axis type or return must be `axis_aer`
// (`SemanticAxis<AxisSemantic::Azimuth, AxisSemantic::Elevation, AxisSemantic::Range>`)
auto point_ecef = refx::frame_transform<refx::ecef>(point_ned);
                                             ^----^
```

### Applying a Pose (SE(3) Transformation)

This overload applies a full rigid body [`Transformation`](geometry_h#transformations) to a [`Vector3D`](geometry_h#vectors) or [`Coordinate3D`](geometry_h#coordinates). It is the primary method for transforming quantities between frames with different origins and orientations in a **Cartesian space**, for example from a vehicle's **[`FrameTag::Body`](frames_h#body) or [`FrameTag::Sensor`](frames_h#sensor) frame to a (navigation) [`FrameTag::LocalTangent`](frames_h#localtangent) frame**.

```cpp
template <typename frameTo, typename frameFrom, /* ... */>
VecType<frameTo, T> frame_transform(const Transformation<frameTo, frameFrom, T>& pose,
                                    const VecType<frameFrom, T>& p_from);
```

This is the API version of the `Transform` [product operator](geometry_h#geometric-operators-2) `p_to = pose * p_from`.

#### API Template Parameters

* **`frameFrom` / `frameTo`**
    * **Constraints**: Both must be [`DirectionalAxis`](frames_h#directional-cartesian-axis) (Cartesian) frames.
* **`VecType`** (Geometric Container, *deduced*)
    * **Valid Types**: [`Vector3D`](geometry_h#vectors) or [`Coordinate3D`](geometry_h#coordinates).

**Usage**

```cpp
// Pose of the vehicle (frd) in the world (ned)
refx::Rotation<refx::ned, refx::frd> R_ned_frd(/* 90 deg yaw */);
refx::Vector3D<refx::ned> t_ned_frd(100.0, 50.0, -10.0);
refx::Transformation<refx::ned, refx::frd> pose_ned_frd(R_ned_frd, t_ned_frd);

// A position measured in the vehicle's frame
refx::Coordinate3D<refx::frd> point_frd(5.0, 0.0, 0.0);

// Transform the point from the vehicle frame to the world frame
auto point_ned = refx::frame_transform(pose_ned_frd, point_frd);
```

**Frame-safety**

If a conversion between the specified frames is not allowed, a compile-time error will be generated, ensuring frame safety.

```cpp
// A position measured in the vehicle's frame, in FLU axis
refx::Coordinate3D<refx::flu> point_flu(5.0, 0.0, 0.0);

// This is a compile-time error: right-hand-side must be expressed in 'frd'
auto point_ned = refx::frame_transform(pose_ned_frd, point_flu);
                                               ^---^      ^---^
```
