---
sidebar_position: 6
---

# Frame Axis Reference

This page provides a comprehensive list of the standard frame axis references available in the library.

## Directional Axis (Cartesian)

This is the set of all the possible right-handed axis reference for a standard Cartesian coordinate system (X, Y, Z). The library offers pre-defined axis configurations for various conventions used in robotics and navigation. Each entry in the table corresponds to a static axis type that you can use when defining a custom frame.

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

## Semantic Axis (Mixed Spherical/Linear)

This type of axis reference is used for more specialized coordinate systems, such as those combining spherical coordinates (e.g., Latitude, Longitude, Azimuth) with linear measurements (e.g., Altitude, Down, Range). These are essential for applications like GPS, geospatial data, and sensor data.

| Axis | Description |
| :--- | :--- |
| `axis_lla` | `SemanticAxis<AxisSemantic::Latitude, AxisSemantic::Longitude, AxisSemantic::Altitude>`|
| `axis_lld` | `SemanticAxis<AxisSemantic::Latitude, AxisSemantic::Longitude, AxisSemantic::Down>`|
| `axis_aer` | `SemanticAxis<AxisSemantic::Azimuth, AxisSemantic::Elevation, AxisSemantic::Range>`|
