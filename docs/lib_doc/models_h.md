---
sidebar_position: 3
---

Models
===

This header provides a suite of classes to [model the Earth's](#earth-model) physical properties, including its [shape](#reference-ellipsoid), [gravitational field](#gravity-model), and [magnetic field](#magnetic-field-model). These models are the foundation for all high-fidelity geodetic and navigation-related calculations, serving as the primary context for converting between coordinate systems and accounting for environmental forces.

```cpp
// The gateway to the Earth models
#include <refx/models.h>
```

## Reference Ellipsoid

The `ReferenceEllipsoid<T>` struct provides a mathematical model of the Earth's shape as an oblate spheroid. This model is a simplified surface that approximates the Earth's shape and serves as the **datum** for all high-accuracy geodetic computations. All transformations between geodetic coordinates (like [`lla`](frames_h#latitude-longitude-altitude-lla)) and Cartesian frames (like [`ecef`](frames_h#earth-centered-earth-fixed-ecef)) depend on the parameters of a specific ellipsoid. The struct is templated with respect to the type that defines the numerical precision employed for representing the constants (`T`). The default value for the numerical precision is `double`.

The available **Constructor** is:

  * `ReferenceEllipsoid() = delete`: The default constructor is deleted to ensure every model is explicitly and completely initialized.
  * `ReferenceEllipsoid(name, semi_major_axis, semi_minor_axis, ...)`: Constructs an ellipsoid from its defining physical constants:
    * `name`: The official name of the ellipsoid standard (e.g., "WGS-84").
    * `semi_major_axis`: The equatorial radius, denoted 'a', in **[m]**.
    * `semi_minor_axis`: The polar radius, denoted 'b', in éé[m]**.
    * `inverse_flattening`: The inverse of the flattening factor, `1/f`.
    * `eccentricity`: The first eccentricity of the ellipsoid, `e`.
    * `angular_velocity`: The mean angular velocity of the Earth, `ω`, in **[rad/s]**.
    * `reference_epoch`: The official epoch for which the constants are defined.

The type provides the following **member functions**:

  * `.semi_major_axis() const`: Returns the equatorial radius 'a' in **[m]**.
  * `.semi_minor_axis() const`: Returns the polar radius 'b' in **[m]**.
  * `.inverse_flattening() const`: Returns the inverse flattening `1/f`, a measure of the ellipsoid's compression.
  * `.eccentricity() const`: Returns the first eccentricity `e`.
  * `.angular_velocity() const`: Returns the mean angular velocity of the Earth `ω` in **[rad/s]**.
  * `.reference_epoch() const`: Returns the reference epoch (e.g., "2024.00") for the constant definitions.
  * `.name() const`: Returns the official name of the standard (e.g., "WGS-84").

### Pre-defined Standards

The library provides pre-configured classes for the most common international standards.

#### WGS-84

`ReferenceEllipsoidWGS84` is the implementation of the reference ellipsoid physical constants relative to the [**World Geodetic System 1984**](https://en.wikipedia.org/wiki/World_Geodetic_System). It is the standard for the Global Positioning System (GPS) and the most widely used datum in navigation and robotics. This should be the default choice for most applications.

```cpp
// Construct a WGS-84 ellipsoid with its standard defining constants
refx::ReferenceEllipsoidWGS84<double> wgs84_datum;
```

#### GRS-80

`ReferenceEllipsoidGRS80` is the implementation of the reference ellipsoid physical constants relative to the [**Geodetic Reference System 1980**](https://en.wikipedia.org/wiki/Geodetic_Reference_System_1980). It is a global reference ellipsoid widely used in geodesy, very similar to WGS-84, with minor differences in defining constants.

```cpp
// Construct a GRS-80 ellipsoid with its standard defining constants
refx::ReferenceEllipsoidGRS80<double> grs80_datum;
```

## Gravity Model

The `GravityModel<T>` struct encapsulates the mathematical formulas for the Earth's theoretical "normal" gravity. The struct is templated with respect to the type that defines the numerical precision employed for representing the scalar elements (`T`). The default value for the numerical precision is `double`.

The enum `GravityHeightCorrection` specifies the model used to account for the decrease in gravity with altitude.

```cpp
class GravityHeightCorrection {
     // No height correction is applied.
    None,
    // A spherical free-air correction is applied.
    FreeAir,
    // An ellipsoidal correction is applied (Moritz, 1980).
    Ellipsoidal
};
```
  * `None`: No correction; gravity is calculated on the ellipsoid surface.
  * `FreeAir`: A common and computationally efficient approximation, reasonably accurate for altitudes typical of most robotics and aerospace applications.
  * `Ellipsoidal`: A higher accuracy by accounting for the non-spherical (ellipsoidal) shape of the Earth.

The available **Constructor** is:

  * `GravityModel() = delete`: The default constructor is deleted to ensure that a gravity model is
     * always explicitly initialized with a complete and valid set of constants.
  * `GravityModel(T gamma_e, T gamma_p, T GM, const ReferenceEllipsoid<T>& datum)`: Constructs a gravity model from its defining physical constants:
    * `gamma_e`: The magnitude of normal gravity at the equator in **[m/s²]**.
    * `gamma_p`: The magnitude of normal gravity at the poles in **[m/s²]**.
    * `GM`: The gravitational constant of the Earth (mass × G) in **[m³/s²]**.
    * `datum`: The reference ellipsoid that this gravity field refers to.

The type provides the following **member functions**:

  * `.gamma_e() const`: Returns normal gravity at the equator (`γe`) in **[m/s²]**.
  * `.gamma_p() const`: Returns normal gravity at the poles (`γp`) in **[m/s²]**.
  * `.kappa() const`: Returns the Somigliana constant `k`, used in the normal gravity formula.
  * `.GM() const`: Returns the Earth's standard gravitational parameter `GM` in **[m³/s²]**.
  * `.reference_ellipsoid() const`: Returns the const reference to the `ReferenceEllipsoid<T>` associated with this gravity model.
  * `.normal_gravity_on_ellipsoid(T latitude)`: Computes gravity on the ellipsoid surface using the [**Somigliana formula**](https://en.wikipedia.org/wiki/Theoretical_gravity#Somigliana_equation).
  * `.apply_height(T gamma_phi, T lat, T alt, GravityHeightCorrection hc)`: Applies a specified height correction to a surface gravity value. The input parameters are:
    * `gamma_phi`: The normal gravity at sea-level for the given latitude, `φ`, in **[m/s²]**.
    * `lat`: The geodetic latitude in **radians**.
    * `alt`: The altitude above the reference ellipsoid in **[m]**.
    * `hc`: The height correction model to apply.

### Pre-defined Standards

#### WGS-84

The `GravityModelWGS84` model provides the standard gravity constants associated with the [`ReferenceEllipsoidWGS84`](#wgs-84).

```cpp
// Construct a WGS-84 gravity model
refx::GravityModelWGS84<double> wgs84_gravity;
```

#### GRS-80

The `GravityModelGRS80` model provides the standard gravity constants associated with the [`ReferenceEllipsoidGRS80`](#grs-80).

```cpp
// Construct a GRS-80 gravity model
refx::GravityModelGRS80<double> grs80_gravity;
```

## Magnetic Field Model
> [!WARNING]
> The Magnetic model is still in development and struct and API interface may change between minor revisions. 

The `MagneticFieldModel<T>` class serves as an interface for modeling the Earth's magnetic field. The base class is an interface whose default implementation returns a zero-field vector, allowing it to be an optional component in the main [`EarthModel`](#earth-model).

The core function of the interface is:

  * `virtual get_field(const Coordinate3D<lla, T>&, T decimal_year) const`: Computes the magnetic field vector at a specific location ([`Coordinate3D<lla, T>`](geometry_h#latitude-longitude-altitude)) and time, returning a [`Vector3D<ned>`](geometry_h#north-east-down) in **[nanoTeslas, nT]**.

<!-- >
### Pre-defined Standards

#### World Magnetic Model (WMM)

The `MagneticFieldModelWMM` class is a concrete implementation of the **World Magnetic Model**, the standard model used in global navigation. This implementation uses the **WMM2020** coefficients, which are valid from 2020.0 to 2025.0, and is critical for high-fidelity heading estimation.

```cpp
// Construct a World Magnetic Model (WMM2020)
refx::MagneticFieldModelWMM<double> wmm_model;

// Compute the magnetic field at a specific location and time
auto location = refx::Coordinate3D<refx::lla>::from_radians(0.72, -1.48, 2000.0);
refx::Vector3D<refx::ned> mag_field = wmm_model.get_field(location, 2024.5);
```
< -->

## Earth Model

The `EarthModel<T>` class is a high-level utility that consolidates the fundamental physical models ([`ReferenceEllipsoid`](#reference-ellipsoid), [`GravityModel`](#gravity-model), and an optional [`MagneticFieldModel`](#magnetic-field-model)) into a single, coherent context object. It serves as the primary input for advanced navigation algorithms and provides convenient helper functions to compute critical values derived from the underlying models.

The available **Constructor** is:

  * `EarthModel() = delete`: The default constructor is deleted to ensure that an Earth model is always explicitly initialized with valid physical models.
  * `EarthModel(const ReferenceEllipsoid&, const GravityModel&, const MagneticFieldModel&)`: Constructs a unified Earth model from its constituent components. The `MagneticFieldModel` parameter is defaulted to a zero-field model.

The type provides the following **member functions**:

  * `.reference_ellipsoid() const`: Returns a const reference to the associated [`ReferenceEllipsoid`](#reference-ellipsoid).
  * `.gravity_model() const`: Returns a const reference to the associated [`GravityModel`](#gravity-model).
  * `.gravity(lat)`: Computes the magnitude of normal gravity on the surface of the ellipsoid (zero-height).
  * `.gravity(const Coordinate3D<lla, T>&, const GravityHeightCorrection&)`: Computes the magnitude of normal gravity at a specific 3D geodetic coordinate. The **default value** for the [`GravityHeightCorrection`](#gravity-model) parameter is `GravityHeightCorrection::None`, that is the gravity is computed without height correction (returns the same result of `.gravity(lat)`).
  * `.equatorial_radius() const`: Returns the semi-major axis 'a' of the ellipsoid in **[m]**.
  * `.polar_radius() const`: Returns the semi-minor axis 'b' of the ellipsoid in **[m]**.
  * `.mean_radius() const`: Returns the mean radius `(2a + b) / 3` in **[m]**.
  * `.meridian_radius(latitude)`: Computes the North-South radius of curvature, used for relating North-South velocity to latitude changes.
  * `.normal_radius(latitude)`: Computes the East-West (transverse) radius of curvature, used for relating East-West velocity to longitude changes.

### Pre-defined Standards

#### WGS-84

The `EarthModelWGS84` is a pre-configured model based on the WGS-84 standard. This is the most widely used model and combines the [`ReferenceEllipsoidWGS84`](#wgs-84) and [`GravityModelWGS84`](#wgs-84-1). It is the recommended choice for most GPS-based applications.

```cpp
// Construct a unified WGS-84 Earth model
refx::EarthModelWGS84<double> earth;

// Use the model to perform high-level calculations
double r_mean = earth.mean_radius();

auto location = refx::Coordinate3D<refx::lla>(45.0, -75.0, 1000.0);
double g = earth.gravity(location);
// the same as: earth.gravity(location, refx::GravityHeightCorrection::None);
std::cout << g << std::endl; // '9.8062'

g = earth.gravity(location, refx::GravityHeightCorrection::FreeAir);
std::cout << g << std::endl; // '9.80312'

g = earth.gravity(location, refx::GravityHeightCorrection::FreeAir);
std::cout << g << std::endl; // '9.80311'
```