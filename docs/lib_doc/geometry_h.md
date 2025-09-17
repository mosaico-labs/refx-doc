---
sidebar_position: 2
---

Geometry
===

This header defines the frame-aware geometric types of the library, for representing type-safe 3D [Vectors](#vectors), [Coordinates](#coordinates), [Rotations](#rotation) and [Transformations](#transformations).

```cpp
// The gateway to the geometric entities
#include <refx/geometry.h>
```
## Vectors

The class needed to represent frame-aware **3D vectors** is `Vector3D<Frame, T>`. The type is a template that accepts two parameters: the [reference frame](frames_h#frames-1) (`Frame`) and the type that defines the numerical precision employed for representing the vector elements (`T`). The default value for the numerical precision is `double`.

The generic (non-specialized) template struct can be used with all the default (or [user-defined](../intro#types-customization)) [frame](frames_h#frames-1) types with **[`DirectionalAxis`](frames_h#directional-cartesian-axis) `axis` configuration** (Cartesian frames) 
Both the generic template struct and its specializations publicly inherit from a user-restricted `internal::VectorContainer3D`, allowing them to leverage the same **data storage, constructors and base vector accessors**.

The list of non-specialized frame-aware vectors that the library provides by default are:
  * `Vector<ecef, T>`: Specialization for [`ecef`](frames_h#earth-centered-earth-fixed-ecef) ([`FrameTag::Geocentric`](frames_h#geocentric)) frame;
  * `Vector<wa_generic, T>`: Specialization for template [`wa_generic`](frames_h#wander-azimuth-wa-wa_generic) ([`FrameTag::LocalTangent`](frames_h#localtangent)) frame;
  * `Vector<wa, T>`: Specialization for [`wa` (`wa_ned`)](frames_h#wander-azimuth-wa-wa_generic) ([`FrameTag::LocalTangent`](frames_h#localtangent)) frame;
  * `Vector<wa_enu, T>`: Specialization for [`wa_enu`](frames_h#wander-azimuth-wa-wa_generic) ([`FrameTag::LocalTangent`](frames_h#localtangent)) frame;
  * `Vector<imu, T>`: Specialization for [`imu`)](frames_h#wander-azimuth-wa-wa_generic) ([`FrameTag::Sensor`](frames_h#sensor)) frame;
  * `Vector<camera, T>`: Specialization for [`camera`)](frames_h#camera-camera) ([`FrameTag::Sensor`](frames_h#sensor)) frame;

The available **Constructor** of the generic template is:
  * `Vector3D(T x, T y, T z)`: Constructs a `Vector3D` from the three components X, Y and Z in the specified `Frame`. The constructor default input parameters value is `T(0)`. The **units** of the components depend on the entity the vector is representing (position/displacement, velocity, acceleration, etc.), and are expressed in **International System Units (SI)**.

```cpp
// In this way a 3D vector in the frd (body) reference frame is zero-initialized. 
refx::Vector3D<refx::frd> frd_vec;

// In this way a single-precision, 3D vector in the camera (sensor) reference frame is initialized.
refx::Vector3D<refx::camera, float> camera_vec(2.0, -0.5, 1.8);

// With the same syntax we can define vectors in all the cartesian reference frames:
// wa, imu, ... [and user defined frames]
```
##### Base vector accessors
The generic template type and all its specializations inherit the following **base vector accessors** from the struct `internal::VectorContainer3D`:
  * `T& x()`: Returns a reference to the vector's first component.
  * `T& y()`: Returns a reference to the vector's second component.
  * `T& z()`: Returns a reference to the vector's third component.
  * `T x() const`: Returns the vector's first component by copy. Can be used with `const` objects.
  * `T y() const`: Returns the vector's second component by copy. Can be used with `const` objects.
  * `T z() const`: Returns the vector's third component by copy. Can be used with `const` objects.

```cpp
// Access to the first component
double elem_x = frd_vec.x(); 
// Access to the second component
double elem_y = frd_vec.y(); 
// Writes to the third component
frd_vec.z() = 0.6; 
```

The type provides the following **member functions**:

  * `.data()`: Returns the vector's data as a `const` reference to the internal container `std::array`.


```cpp
// Get the std::array<double, 3> from the vector
const std::array<double, 3>& ecef_array = ecef_vec.data();
```

### Specializations for Cartesian Frames

The library specializes the template struct for the default [Cartesian frames](frames_h#directional-cartesian-axis) for which it is convenient to customize the semantic accessors.

#### North, East, Down

The specialization `Vector3D<ned>` represents a vector in the [**North, East, Down (NED)**](frames_h#north-east-down-ned) [`FrameTag::LocalTangent`](frames_h#localtangent) frame. The available **Constructor** is:

  * `Vector3D<ned>(T n, T e, T d)`: Constructor taking the North, East and Down components. The default input parameters value is `T(0)`. The **units** of the components depend on the entity the vector is representing (position/displacement, velocity, acceleration, etc.), and are expressed in **International System Units (SI)**.

<!-- end list -->

```cpp
// In this way a 3D vector in the ned reference frame is zero-initialized. 
refx::Vector3D<refx::ned> ned_vec; //ned_vec(0.0, 0.0, 0.0)

// In this way a 3D vector in the ned reference frame is initialized. 
refx::Vector3D<refx::ned> new_ned_vec(44.49, 11.34, 101.8);
```

The type inherits the **base vector accessors** from the parent `internal::VectorContainer3D` and defines contextual semantic accessors associated with the `ned` frame:

**Inherited**
  * [**base vector accessors**](#base-vector-accessors)

**Contextual**
  * `T& north()`: Returns a reference to the North component in **its corresponding SI unit**.
  * `T& east()`: Returns a reference to the East component in **its corresponding SI unit**.
  * `T& down()`: Returns a reference to the Down component in **its corresponding SI unit**.
  * `T north() const`: Returns the North component in **its corresponding SI unit**, by copy. Can be used with `const` objects.
  * `T east() const`: Returns the East component in **its corresponding SI unit**, by copy. Can be used with `const` objects.
  * `T down() const`: Returns the Down component in **its corresponding SI unit**, by copy. Can be used with `const` objects.

<!-- end list -->

```cpp
// Overwrite the North value. Calls T& north();
new_ned_vec.north() = 44.49;
// It would be the same with the inherited accessor internal::VectorContainer3D::x()
// new_ned_vec.x() = 44.49;

//Access the East value
double east_val = new_ned_vec.east(); // returns 11.34
```
The type provides the following **member functions**:

  * `.data()`: Returns the coordinate's data as a const reference to the internal container `std::array`.

#### East, North, Up

The specialization `Vector3D<enu>` represents a vector in the[ **East, North, Up (ENU)**](frames_h#east-north-up-enu) [`FrameTag::LocalTangent`](frames_h#localtangent) frame. The available **Constructor** is:

  * `Vector3D<enu>(T e, T n, T u)`: Constructor taking the East, North and Up components. The constructor default input parameters value is `T(0)`. The **units** of the components depend on the entity the vector is representing (position/displacement, velocity, acceleration, etc.), and are expressed in **International System Units (SI)**.

```cpp
// In this way a 3D vector in the enu reference frame is zero-initialized. 
refx::Vector3D<refx::enu> enu_vec; //enu_vec(0.0, 0.0, 0.0)

// In this way a 3D vector in the enu reference frame is initialized. 
refx::Vector3D<refx::enu> new_enu_vec(44.49, 11.34, 101.8);
```

The type inherits the **base vector accessors** from the parent `internal::VectorContainer3D` and defines contextual semantic accessors associated with the `enu` frame:

**Inherited**
  * [**base vector accessors**](#base-vector-accessors)

**Contextual**
  * `T& east()`: Returns a reference to the East component in **its corresponding SI unit**.
  * `T& north()`: Returns a reference to the North component in **its corresponding SI unit**.
  * `T& up()`: Returns a reference to the Up component in **its corresponding SI unit**.
  * `T east() const`: Returns the East component in **its corresponding SI unit**, by copy. Can be used with `const` objects.
  * `T north() const`: Returns the North component in **its corresponding SI unit**, by copy. Can be used with `const` objects.
  * `T up() const`: Returns the Up component in **its corresponding SI unit**, by copy. Can be used with `const` objects.

<!-- end list -->

```cpp
// Overwrite the North value. Calls T& north();
new_enu_vec.north() = 44.49;
// It would be the same with the inherited accessor internal::VectorContainer3D::x()
// new_enu_vec.x() = 44.49;

//Access the East value
double east_val = new_enu_vec.east(); // returns 11.34
```
The type provides the following **member functions**:

  * `.data()`: Returns the coordinate's data as a const reference to the internal container `std::array`.


#### North, West, Up

The specialization `Vector3D<nwu>` represents a vector in the [**North, West, Up (NWU)**](frames_h#north-west-up-nwu) [`FrameTag::LocalTangent`](frames_h#localtangent) frame. The available **Constructor** is:

  * `Vector3D<nwu>(T n, T w, T u)`: Constructor taking the North, West and Up components. The constructor default input parameters value is `T(0)`. The **units** of the components depend on the entity the vector is representing (position/displacement, velocity, acceleration, etc.), and are expressed in **International System Units (SI)**.

```cpp
// In this way a 3D vector in the nwu reference frame is zero-initialized. 
refx::Vector3D<refx::nwu> nwu_vec; //nwu_vec(0.0, 0.0, 0.0)

// In this way a 3D vector in the nwu reference frame is initialized. 
refx::Vector3D<refx::nwu> new_nwu_vec(44.49, -11.34, 101.8);
```

The type inherits the **base vector accessors** from the parent `internal::VectorContainer3D` and defines contextual semantic accessors associated with the `nwu` frame:

**Inherited**
  * [**base vector accessors**](#base-vector-accessors)

**Contextual**
  * `T& north()`: Returns a reference to the North component in **its corresponding SI unit**.
  * `T& west()`: Returns a reference to the West component in **its corresponding SI unit**.
  * `T& up()`: Returns a reference to the Up component in **its corresponding SI unit**.
  * `T north() const`: Returns the North component in **its corresponding SI unit**, by copy. Can be used with `const` objects.
  * `T west() const`: Returns the West component in **its corresponding SI unit**, by copy. Can be used with `const` objects.
  * `T up() const`: Returns the Up component in **its corresponding SI unit**, by copy. Can be used with `const` objects.

```cpp
// Overwrite the North value. Calls T& north();
new_nwu_vec.north() = 44.49;
// It would be the same with the inherited accessor internal::VectorContainer3D::x()
// new_nwu_vec.x() = 44.49;

//Access the West value
double west_val = new_enu_vec.west(); // returns -11.34
```
The type provides the following **member functions**:

  * `.data()`: Returns the coordinate's data as a const reference to the internal container `std::array`.


#### Forward, Right, Down

The specialization `Vector3D<frd>` represents a vector in the [**Forward, Right, Down (FRD)**](frames_h#forward-right-down-frd) [`FrameTag::Body`](frames_h#body) frame. The available **Constructor** is:

  * `Vector3D<frd>(T f, T r, T d)`: Constructor taking the Forward, Right and Down components. The constructor default input parameters value is `T(0)`. The **units** of the components depend on the entity the vector is representing (position/displacement, velocity, acceleration, etc.), and are expressed in **International System Units (SI)**.

```cpp
// In this way a 3D vector in the frd reference frame is zero-initialized. 
refx::Vector3D<refx::frd> frd_vec; //frd_vec(0.0, 0.0, 0.0)

// In this way a 3D vector in the frd reference frame is initialized. 
refx::Vector3D<refx::frd> new_frd_vec(44.49, -11.34, 101.8);
```

The type inherits the **base vector accessors** from the parent `internal::VectorContainer3D` and defines contextual semantic accessors associated with the `frd` frame:

**Inherited**
  * [**base vector accessors**](#base-vector-accessors)

**Contextual**
  * `T& forward()`: Returns a reference to the Forward component in **its corresponding SI unit**.
  * `T& right()`: Returns a reference to the Right component in **its corresponding SI unit**.
  * `T& down()`: Returns a reference to the Down component in **its corresponding SI unit**.
  * `T forward() const`: Returns the Forward component in **its corresponding SI unit**, by copy. Can be used with `const` objects.
  * `T right() const`: Returns the Right component in **its corresponding SI unit**, by copy. Can be used with `const` objects.
  * `T down() const`: Returns the Down component in **its corresponding SI unit**, by copy. Can be used with `const` objects.

```cpp
// Overwrite the Forward value. Calls T& forward();
new_frd_vec.forward() = 44.49;
// It would be the same with the inherited accessor internal::VectorContainer3D::x()
// new_frd_vec.x() = 44.49;

//Access the Right value
double right_val = new_frd_vec.right(); // returns -11.34
```
The type provides the following **member functions**:

  * `.data()`: Returns the coordinate's data as a const reference to the internal container `std::array`.


#### Forward, Left, Up

The specialization `Vector3D<flu>` represents a vector in the [**Forward, Left, Up (FLU)**](frames_h#forward-left-up-flu) [`FrameTag::Body`](frames_h#body) frame. The available **Constructor** is:

  * `Vector3D<flu>(T f, T l, T u)`: Constructor taking the Forward, Left and Up components. The constructor default input parameters value is `T(0)`. The **units** of the components depend on the entity the vector is representing (position/displacement, velocity, acceleration, etc.), and are expressed in **International System Units (SI)**.

```cpp
// In this way a 3D vector in the flu reference frame is zero-initialized. 
refx::Vector3D<refx::flu> flu_vec; //flu_vec(0.0, 0.0, 0.0)

// In this way a 3D vector in the flu reference frame is initialized. 
refx::Vector3D<refx::flu> new_flu_vec(44.49, -11.34, 101.8);
```

The type inherits the **base vector accessors** from the parent `internal::VectorContainer3D` and defines contextual semantic accessors associated with the `flu` frame:

**Inherited**
  * [**base vector accessors**](#base-vector-accessors)

**Contextual**
  * `T& forward()`: Returns a reference to the Forward component in **its corresponding SI unit**.
  * `T& left()`: Returns a reference to the Left component in **its corresponding SI unit**.
  * `T& up()`: Returns a reference to the Up component in **its corresponding SI unit**.
  * `T forward() const`: Returns the Forward component in **its corresponding SI unit**, by copy. Can be used with `const` objects.
  * `T left() const`: Returns the Left component in **its corresponding SI unit**, by copy. Can be used with `const` objects.
  * `T up() const`: Returns the Up component in **its corresponding SI unit**, by copy. Can be used with `const` objects.

```cpp
// Overwrite the Forward value. Calls T& forward();
new_flu_vec.forward() = 44.49;
// It would be the same with the inherited accessor internal::VectorContainer3D::x()
// new_flu_vec.x() = 44.49;

//Access the Right value
double left_val = new_flu_vec.left(); // returns -11.34
```
The type provides the following **member functions**:

  * `.data()`: Returns the coordinate's data as a const reference to the internal container `std::array`.


#### Right, Forward, Up

The specialization `Vector3D<rfu>` represents a vector in the [**Right, Forward, Up (RFU)**](frames_h#right-forward-up-rfu) [`FrameTag::Body`](frames_h#body) frame. The available **Constructor** is:

  * `Vector3D<rfu>(T r, T f, T u)`: Constructor taking the Right, Forward and Up components. The constructor default input parameters value is `T(0)`. The **units** of the components depend on the entity the vector is representing (position/displacement, velocity, acceleration, etc.), and are expressed in **International System Units (SI)**.

```cpp
// In this way a 3D vector in the rfu reference frame is zero-initialized. 
refx::Vector3D<refx::rfu> rfu_vec; //rfu_vec(0.0, 0.0, 0.0)

// In this way a 3D vector in the rfu reference frame is initialized. 
refx::Vector3D<refx::rfu> new_rfu_vec(44.49, -11.34, 101.8);
```

The type inherits the **base vector accessors** from the parent `internal::VectorContainer3D` and defines contextual semantic accessors associated with the `rfu` frame:

**Inherited**
  * [**base vector accessors**](#base-vector-accessors)

**Contextual**
  * `T& right()`: Returns a reference to the Right component in **its corresponding SI unit**.
  * `T& forward()`: Returns a reference to the Forward component in **its corresponding SI unit**.
  * `T& up()`: Returns a reference to the Up component in **its corresponding SI unit**.
  * `T right() const`: Returns the Right component in **its corresponding SI unit**, by copy. Can be used with `const` objects.
  * `T forward() const`: Returns the Forward component in **its corresponding SI unit**, by copy. Can be used with `const` objects.
  * `T up() const`: Returns the Up component in **its corresponding SI unit**, by copy. Can be used with `const` objects.

```cpp
// Overwrite the Forward value. Calls T& forward();
new_rfu_vec.forward() = 44.49;
// It would be the same with the inherited accessor internal::VectorContainer3D::x()
// new_rfu_vec.x() = 44.49;

//Access the Right value
double right_val = new_rfu_vec.right(); // returns -11.34
```
The type provides the following **member functions**:

  * `.data()`: Returns the coordinate's data as a const reference to the internal container `std::array`.


#### Surge, Sway, Heave

The specialization `Vector3D<ssh>` represents a vector in the [**Surge, Sway, Heave (SSH)**](frames_h#surge-sway-heave-ssh) [`FrameTag::Body`](frames_h#body) frame. The available **Constructor** is:

  * `Vector3D<ssh>(T s, T s, T h)`: Constructor taking the Surge, Sway and Heave components. The constructor default input parameters value is `T(0)`. The **units** of the components depend on the entity the vector is representing (position/displacement, velocity, acceleration, etc.), and are expressed in **International System Units (SI)**.

```cpp
// In this way a 3D vector in the ssh reference frame is zero-initialized. 
refx::Vector3D<refx::ssh> ssh_vec; //ssh_vec(0.0, 0.0, 0.0)

// In this way a 3D vector in the ssh reference frame is initialized. 
refx::Vector3D<refx::ssh> new_ssh_vec(44.49, -11.34, 101.8);
```

The type inherits the **base vector accessors** from the parent `internal::VectorContainer3D` and defines contextual semantic accessors associated with the `ssh` frame:

**Inherited**
  * [**base vector accessors**](#base-vector-accessors)

**Contextual**
  * `T& surge()`: Returns a reference to the Surge component in **its corresponding SI unit**.
  * `T& sway()`: Returns a reference to the Sway component in **its corresponding SI unit**.
  * `T& heave()`: Returns a reference to the Heave component in **its corresponding SI unit**.
  * `T surge() const`: Returns the Surge component in **its corresponding SI unit**, by copy. Can be used with `const` objects.
  * `T sway() const`: Returns the Sway component in **its corresponding SI unit**, by copy. Can be used with `const` objects.
  * `T heave() const`: Returns the Heave component in **its corresponding SI unit**, by copy. Can be used with `const` objects.

```cpp
// Overwrite the Surge value. Calls T& surge();
new_ssh_vec.surge() = 44.49;
// It would be the same with the inherited accessor internal::VectorContainer3D::x()
// new_ssh_vec.x() = 44.49;

//Access the Sway value
double right_val = new_ssh_vec.sway(); // returns -11.34
```
The type provides the following **member functions**:

  * `.data()`: Returns the coordinate's data as a const reference to the internal container `std::array`.


### Specializations for Non-Cartesian Frames

These specializations are for frames with [`SemanticAxis`](frames_h#semantic-non-cartesian-axis) `axis` type (non-cartesian, mixed frames). They represent **differential quantities** and are typically the result of subtracting two [`Coordinate3D`](#coordinates) objects. Standard geometric operations like cross/dot products are not physically meaningful for these types and are forbidden via compile-time error.

All angular vector components are stored internally in **degrees**.

#### Delta Latitude, Longitude, Altitude

The specialization `Vector3D<lla>` represents a geodetic difference vector in the [**Latitude, Longitude, Altitude (LLA)**](frames_h#latitude-longitude-altitude-lla) [`FrameTag::Geocentric`](frames_h#geocentric) frame. The available **Constructors** are:

  * `Vector3D<lla>(T d_lat, T d_lon, T d_alt)`: Constructor taking the difference in Latitude and Longitude in **degrees** and the difference in Altitude in **meters**.

The available **Factories** are:
  * `static from_radians(T d_lat_rad, T d_lon_rad, T d_alt)`: Factory to construct from differences in Latitude and Longitude in **radians** and Altitude in **meters**.

```cpp
// In this way a 3D difference vector in the lla reference frame is zero-initialized. 
refx::Vector3D<refx::lla> lla_vec; //lla_vec(0.0, 0.0, 0.0)

// In this way a 3D difference vector in the lla reference frame is initialized. 
refx::Vector3D<refx::lla> new_lla_vec(0.001, -0.0005, 1.2);

// The same difference vector as the previous, initialized with angles in radians
new_lla_vec = refx::Vector3D<refx::lla>::from_radians(1.745e-5, -8.72e-6, 1.2);
```
The type inherits the **base vector accessors** from the parent `internal::VectorContainer3D` and defines contextual semantic accessors associated to the differential quantities:

**Inherited**
  * [**base vector accessors**](#base-vector-accessors)

**Contextual**
  * `T& delta_latitude()`: Returns a reference to the Latitude difference in **degrees**.
  * `T& delta_longitude()`: Returns a reference to the Longitude difference in **degrees**.
  * `T& delta_altitude()`: Returns a reference to the Altitude difference in **degrees**.
  * `T delta_latitude(AngleUnit) const`: Returns the Latitude difference, with the **desired angular unit**, by copy. Can be used with `const` objects.
  * `T delta_longitude(AngleUnit) const`: Returns the Longitude difference, with the **desired angular unit**, by copy. Can be used with `const` objects.
  * `T delta_altitude() const`: Returns the Altitude difference in **meters**, by copy. Can be used with `const` objects.

```cpp
//Access the longitude difference value in radians
double d_lon_rad = new_lla_vec.delta_longitude(AngleUnit::Rad); // returns -8.72e-6
```

#### Delta Latitude, Longitude, Down

The specialization `Vector3D<lld>` represents a geodetic difference vector in the [**Latitude, Longitude, Down (LLD)**](frames_h#latitude-longitude-down-lld) [`FrameTag::Geocentric`](frames_h#geocentric) frame. The available **Constructors** are:

  * `Vector3D<lld>(T d_lat, T d_lon, T d_dow)`: Constructor taking the difference in Latitude and Longitude in **degrees** and the difference in negative Altitude in **meters**.

The available **Factories** are:
  * `static from_radians(T d_lat_rad, T d_lon_rad, T d_dow)`: Factory to construct from differences in Latitude and Longitude in **radians** and negative Altitude in **meters**.

```cpp
// In this way a 3D difference vector in the lld reference frame is zero-initialized. 
refx::Vector3D<refx::lld> lld_vec; //lld_vec(0.0, 0.0, 0.0)

// In this way a 3D difference vector in the lld reference frame is initialized. 
refx::Vector3D<refx::lld> new_lld_vec(0.001, -0.0005, 1.2);

// The same difference vector as the previous, initialized with angles in radians
new_lld_vec = refx::Vector3D<refx::lld>::from_radians(1.745e-5, -8.72e-6, 1.2);
```

The type inherits the **base vector accessors** from the parent `internal::VectorContainer3D` and defines contextual semantic accessors associated to the differential quantities:

**Inherited**
  * [**base vector accessors**](#base-vector-accessors)

**Contextual**
  * `T& delta_latitude()`: Returns a reference to the Latitude difference in **degrees**.
  * `T& delta_longitude()`: Returns a reference to the Longitude difference in **degrees**.
  * `T& delta_down()`: Returns a reference to the negative Altitude difference in **degrees**.
  * `T delta_latitude(AngleUnit) const`: Returns the Latitude difference with the **desired angular unit**, by copy. Can be used with `const` objects.
  * `T delta_longitude(AngleUnit) const`: Returns the Longitude difference with the **desired angular unit**, by copy. Can be used with `const` objects.
  * `T delta_down() const`: Returns the negative Altitude difference in **meters**, by copy. Can be used with `const` objects.

```cpp
//Access the longitude difference value in radians
double d_lon_rad = new_lld_vec.delta_longitude(AngleUnit::Rad); // returns -8.72e-6
```

#### Delta Azimuth, Elevation, Range

The specialization `Vector3D<aer>` represents a geodetic difference vector in the [**Azimuth, Elevation, Range (AER)**](frames_h#azimuth-elevation-range-aer) [`FrameTag::LocalTangent`](frames_h#localtangent) frame. The available **Constructors** are:

  * `Vector3D<aer>(T d_az, T d_el, T d_rg)`: Constructor taking the difference in Azimuth and Elevation in **degrees** and the difference in Range in **meters**.

The available **Factories** are:
  * `static from_radians(T d_az_rad, T d_el_rad, T d_rg)`: Factory to construct from differences in Azimuth and Elevation in **radians** and the difference in Range in **meters**.

```cpp
// In this way a 3D difference vector in the aer reference frame is zero-initialized. 
refx::Vector3D<refx::aer> aer_vec; //aer_vec(0.0, 0.0, 0.0)

// In this way a 3D difference vector in the aer reference frame is initialized. 
refx::Vector3D<refx::aer> new_aer_vec(0.001, -0.0005, 1.2);

// The same difference vector as the previous, initialized with angles in radians
new_aer_vec = refx::Vector3D<refx::aer>::from_radians(1.745e-5, -8.72e-6, 1.2);
```

The type inherits the **base vector accessors** from the parent `internal::VectorContainer3D` and defines contextual semantic accessors associated to the differential quantities:

**Inherited**
  * [**base vector accessors**](#base-vector-accessors)

**Contextual**
  * `T& delta_azimuth()`: Returns a reference to the Azimuth difference in **degrees**.
  * `T& delta_elevation()`: Returns a reference to the Elevation difference in **degrees**.
  * `T& delta_range()`: Returns a reference to the Range difference in **degrees**.
  * `T delta_azimuth(AngleUnit) const`: Returns the Azimuth difference with the **desired angular unit**, by copy. Can be used with `const` objects.
  * `T delta_elevation(AngleUnit) const`: Returns the Elevation difference with the **desired angular unit**, by copy. Can be used with `const` objects.
  * `T delta_range() const`: Returns the Range difference in **meters**, by copy. Can be used with `const` objects.

```cpp
//Access the longitude difference value in radians
double d_lon_rad = new_aer_vec.delta_longitude(AngleUnit::Rad); // returns -8.72e-6
```


### Arithmetic Operators

The header defines a set of operators that enforce compile-time frame safety: operations between vectors of different frames will result in a compile-time error.

  * `operator+ (Vector3D, Vector3D)`: Returns the **sum** of the two vectors. For frames with [`SemanticAxis`](frames_h#semantic-non-cartesian-axis) `axis` type, this uses specialized logic like shortest-angle difference.

  * `operator- (Vector3D, Vector3D)`: Returns the **difference** between the two vectors. For frames with [`SemanticAxis`](frames_h#semantic-non-cartesian-axis) `axis` type, this uses specialized logic like shortest-angle difference.
  
  * `operator* (Vector3D, T)` or `(T, Vector3D)`: Returns a new `Vector3D` scaled by a scalar value. For frames with [`SemanticAxis`](frames_h#semantic-non-cartesian-axis) `axis` type, this uses specialized logic like shortest-angle difference.
  
  * `operator/ (Vector3D, T)`: Returns a new `Vector3D` with each component divided by a scalar value. For frames with [`SemanticAxis`](frames_h#semantic-non-cartesian-axis) `axis` type, this uses specialized logic like shortest-angle difference.
  
  * `cross(Vector3D, Vector3D)`: Computes the **cross product**. Meaningful only for Cartesian frames. The operator is forbidden for frames with [`SemanticAxis`](frames_h#semantic-non-cartesian-axis) `axis` type.
  
  * `dot(Vector3D, Vector3D)`: Computes the **dot product**. Meaningful only for Cartesian frames. The operator is forbidden for frames with [`SemanticAxis`](frames_h#semantic-non-cartesian-axis) `axis` type.

<!-- end list -->

```cpp
Vector3D<ned> v1(10, 5, 2);
Vector3D<ned> v2(1, 2, 3);
Vector3D<frd> v3;

// Lecit: Compute the sum of two vectors in the same frame
auto v_sum = v1 + v2; // Result: [11, 7, 5]
// This is a compile-time error!
// auto v_err = v1 + v3; // ERROR: Incompatible frames
                  ^----^

// Lecit: Compute the cross product
auto v_cross = cross(v1, v2); // Result: [11, -28, 15]
// This is a compile-time error!
// auto v_cross = cross(v1, v3); // ERROR: Incompatible frames
                           ^--^
```

## Coordinates

The class needed to represent **3D positions** is `Coordinate3D<Frame, T>`. The type is a template which accepts two parameters: the reference frame (`Frame`) and the type which defines the numerical precision employed for representing the coordinate elements (`T`). The default value for the numerical precision is `double`.

The generic (non-specialized) template struct, can be used with all the default (or [user-defined](../intro#types-customization)) [frame](frames_h#frames-1) types with **[`DirectionalAxis`](frames_h#directional-cartesian-axis) `axis` configuration** (Cartesian frames).
The struct publicly inherits from [`Vector3D<Frame, T>`](#vectors), allowing it to leverage the same **data storage, constructors and accessors** defined for the vector specializations in the same `Frame`. The class inherit **the standard vector operations** that are mathematically valid for linear, Euclidean spaces. 

```cpp
// In this way a 3D position in the ned (local-tangent) reference frame is zero-initialized. 
// Calls the inherited Vector3D<ned>(0, 0, 0)
refx::Coordinate3D<refx::ned> ned_coord;

// In this way a 3D position in the ned (local-tangent) reference frame is initialized. 
// Calls the inherited Vector3D<ned>(10.5, 0.8, -101.8)
refx::Coordinate3D<refx::ned> new_ned_coord(10.5, 0.8, -101.8);

// In this way a single-precision, 3D position in the frd (body) reference frame is initialized.
// Calls the inherited Vector3D<frd>(2.0, -0.5, 1.8)
refx::Coordinate3D<refx::frd, float> frd_coord(2.0, -0.5, 1.8);

// With the same syntax we can define positions in all the cartesian reference frames:
// nwu, frd, flu, imu, ... [and user defined frames]
```

The type inherits all the [**base vector accessors**](#base-vector-accessors) from the struct [`Vector3D<Frame, T>`](#vectors):
  * `T& x()`, `T x() const`;
  * `T& y()`, `T y() const`;
  * ...
  * other inherited **contextual accessors** derive from the [parent specializations](#specializations-for-cartesian-frames) of [`Vector3D<Frame, T>`](#vectors), depend on the `Frame` type:
    * `T& north()`, `T north() const`;
    * ...
    * `T& right()`, `T right() const`;
    * ...

```cpp
// Access to the North component via .north() accessor (inherited from Vector3D<ned>)
double elem_n = new_ned_coord.north(); // returns 10.5
double elem_e = new_ned_coord.east(); // returns 0.8
// Generic base accessors from Vector3D<ned>::VectorContainer3D, 
double elem_n_again = new_ned_coord.x(); // returns 10.5
double elem_e_again = new_ned_coord.y(); // returns 0.8

// Access to the Forward component via .forward() accessor (inherited from Vector3D<frd>)
double elem_f = frd_coord.forward();
// Generic accessors from internal::VectorContainer3D, 
double elem_f_again = frd_coord.x();
```

The type provides the following **member functions**:
  * `.as_vector()`, returns the coordinate's data as a [`Vector3D<Frame, T>`](#vectors) object.

```cpp
// Get the Vector3D<ned> from Coordinate
refx::Vector3D<refx::ned> ned_vec = ned_coord.as_vector();
```

### Specializations for Non-Cartesian Frames

The specializations of the template struct `Coordinate3D<Frame, T>` are provided for the frames with [`SemanticAxis`](frames_h#semantic-non-cartesian-axis) `axis` type (non-cartesian, mixed frames), that necessitate a different accessors and allowed operators. These specializations inherit from the user-restricted struct `internal::VectorContainer3D` the [base vector accessors](#base-vector-accessors) and data container; this allows disabling meaningless vector operations at compile time (like [*cross/dot* products](#arithmetic-operators) defined for [`Vector3D<Frame, T>`](#vectors)).

All the angular coordinate components are stored internally in **degrees**.

#### Latitude, Longitude, Altitude

The specialization `Coordinate<lla>` represents a geographic position in the [**Latitude, Longitude, Altitude (LLA)**](frames_h#latitude-longitude-altitude-lla) [`FrameTag::Geocentric`](frames_h#geocentric) frame. The available **Costructors** are:

 * `Coordinate3D<lla>(T lat, T lon, T alt)`: Constructor taking Latitude and Longitude in **degrees** and Altitude in **meters**.

The available **Factories** are:
 * `static from_radians(T lat_rad, T lon_rad, T alt)`: Factory to construct from Latitude and Longitude in **radians**, and Altitude in **meters**

```cpp
// In this way a 3D position in the lla reference frame is zero-initialized. 
refx::Coordinate3D<refx::lla> lla_pos; //lla_pos(0.0, 0.0, 0.0)

// In this way a 3D position in the lla reference frame is initialized. 
refx::Coordinate3D<refx::lla> new_lla_pos(44.49421, 11.3467, 101.8);

// The same position as the previous, initialized with angles in radians
new_lla_pos = refx::Coordinate3D<refx::lla>::from_radians(0.77657, 0.19804, 101.8);
```

The type inherits the **base vector accessors** from the parent `internal::VectorContainer3D` and defines specific accessors associated to the [`lla`](frames_h#latitude-longitude-altitude-lla) frame:

**Inherited**  
 * [**base vector accessors**](#base-vector-accessors)

**Contextual**
 * `T& latitude()`: Returns a reference to Latitude in **degrees**.
 * `T& longitude()`: Returns a reference to Longitude in **degrees**.
 * `T& altitude()`: Returns a reference to Altitude in **meters**
 * `T latitude(AngleUnit) const`: Returns the Latitude with **desired angular unit**, by copy. Can be used with `const` objects.
 * `T longitude(AngleUnit) const`: Returns the Longitude with **desired angular unit**, by copy. Can be used with `const` objects.
 * `T altitude() const`: Returns the Altitude in **meters**, by copy. Can be used with `const` objects.

```cpp
// Overwrite the latitude value. Calls T& latitude();
new_lla_pos.latitude() = 44.50607;
// It would be the same with the inherited accessor internal::VectorContainer3D::x()
// new_lla_pos.x() = 44.50607;

//Access the longitude value in radians
double lon_rad = new_lla_pos.longitude(AngleUnit::Rad); // returns 0.19804
```

#### Latitude, Longitude, Down

The specialization `Coordinate<lld>` represents a geographic position in the [**Latitude, Longitude, Down (LLD)**](frames_h#latitude-longitude-down-lld) [`FrameTag::Geocentric`](frames_h#geocentric) frame. The available **Costructors** are:

 * `Coordinate3D<lld>(T lat, T lon, T down)`: Constructor taking Latitude and Longitude in **degrees** and negative Altitude in **meters**.

The available **Factories** are:
 * `static from_radians(T lat_rad, T lon_rad, T down)`: Factory to construct from Latitude and Longitude in **radians**, and negative Altitude in **meters**

```cpp
// In this way a 3D position in the lld reference frame is zero-initialized. 
refx::Coordinate3D<refx::lld> lld_pos; //lld_pos(0.0, 0.0, 0.0)

// In this way a 3D position in the lld reference frame is initialized. 
refx::Coordinate3D<refx::lld> new_lld_pos(44.49421, 11.3467, -101.8);

// The same position as the previous, initialized with angles in radians
new_lld_pos = refx::Coordinate3D<refx::lld>::from_radians(0.77657, 0.19804, -101.8);
```

The type inherits the **base vector accessors** from the parent `internal::VectorContainer3D` and defines specific accessors associated to the [`lld`](frames_h#latitude-longitude-down-lld) frame:

**Inherited**  
 * [**base vector accessors**](#base-vector-accessors)

**Contextual**
 * `T& latitude()`: Returns a reference to Latitude in **degrees**.
 * `T& longitude()`: Returns a reference to Longitude in **degrees**.
 * `T& down()`: Returns a reference to negative Altitude in **meters**.
 * `T latitude(AngleUnit) const`: Returns the Latitude with **desired angular unit**, by copy. Can be used with `const` objects.
 * `T longitude(AngleUnit) const`: Returns the Longitude with **desired angular unit**, by copy. Can be used with `const` objects.
 * `T down() const`: Returns the negative Altitude in **meters**, by copy. Can be used with `const` objects.

```cpp
// Overwrite the negative altitude value. Calls T& down();
new_lla_pos.down() = 104.0;
// It would be the same with the inherited accessor internal::VectorContainer3D::y() 
// new_lla_pos.z() = 104.0;

//Access the longitude value in radians
double lat_rad = new_lla_pos.latitude(AngleUnit::Rad); // returns 0.77657
```


#### Azimuth, Elevation, Range

The specialization `Coordinate<aer>` represents a geographic position in the [**Azimuth, Elevation, Range (AER)**](frames_h#azimuth-elevation-range-aer) [`FrameTag::LocalTangent`](frames_h#localtangent) frame. The available **Costructors** are:

 * `Coordinate3D<aer>(T az, T el, T rg)`: Constructor taking azimith and Elevation in **degrees** and Range in **meters**.

The available **Factories** are:
 * `static from_radians(T az_rad, T az_rad, T rg)`: Factory to construct from azimith and Elevation in **radians**, and Range in **meters**


```cpp
// In this way a 3D position in the aer reference frame is zero-initialized. 
refx::Coordinate3D<refx::aer> aer_pos; //aer_pos(0.0, 0.0, 0.0)

// In this way a 3D position in the aer reference frame is initialized. 
refx::Coordinate3D<refx::aer> new_aer_pos(-178.5, 10.1234, 21.34);

// The same position as the previous, initialized with angles in radians
new_lld_pos = refx::Coordinate3D<refx::aer>::from_radians(-3.1154, 0.1767, 21.34);
```
The type inherits the **base vector accessors** from the parent `internal::VectorContainer3D` and defines specific accessors associated to the `aer` frame:

**Inherited**  
 * [**base vector accessors**](#base-vector-accessors)

**Contextual**
 * `T& azimuth()`: Returns a reference to Azimuth in **degrees**.
 * `T& elevation()`: Returns a reference to Elevation in **degrees**.
 * `T& range()`: Returns a reference to Range in **meters**.
 * `T azimuth(AngleUnit) const`: Returns the Azimuth with **desired angular unit**, by copy. Can be used with `const` objects.
 * `T elevation(AngleUnit) const`: Returns the Elevation with **desired angular unit**, by copy. Can be used with `const` objects.
 * `T range() const`: Returns the Range in **meters**, by copy. Can be used with `const` objects.


```cpp
// Overwrite the longitude value. Calls T& longitude();
new_aer_pos.azimuth() = 104.1276;
// It would be the same with the inherited accessor internal::VectorContainer3D::x() 
// new_aer_pos.x() = 104.1276;

//Access the longitude value in radians
double el_rad = new_aer_pos.elevation(AngleUnit::Rad); // returns 0.1767
```

### Arithmetic Operators

The header defines a set of operators that enforce correct affine space mathematics at compile time.

* `operator+ (Coordinate3D, Coordinate3D)`: This operator results in a compile-time error via `static_assert(false, ...)`. Adding two absolute positions is assumed a physically [meaningless operation](../intro#mathematical-operations).

* `operator- (Coordinate3D, Coordinate3D)`: Returns the **displacement** [`Vector3D<Frame, T>`](#vectors) in the same `Frame` of the input coordinates, directed from the left-side operand to the right-side operand. For non-cartesian frames like [`lla`](frames_h#latitude-longitude-altitude-lla), this operation uses specialized logic to handle angular wrap-arounds.

* `operator+ (Coordinate3D, Vector3D)`: Returns a new `Coordinate3D` which is the translation of the input coordinate by the input displacement vector. For non-cartesian frames like [`lla`](frames_h#latitude-longitude-altitude-lla), this operation uses specialized logic to handle angular wrap-arounds.

* `operator+ (Vector3D, Coordinate3D)`: Returns the same result as the previous operator.

```cpp
Coordinate3D<lla> local_origin(41.9028, 12.4964, 50.0);  // Lat/Lon in deg, Alt in m
Coordinate3D<lla> target_gps_point(41.9030, 12.4966, 60.0);

// Lecit: Compute the vector displacement in lla
Vector3D<lla> lla_displ = target_gps_point - local_origin;
// Access the displacement elements, prints: 'latitude displacement: 0.0002'
std::cout << "latitude displacement: " << lla_displ.delta_latitude() << std::endl;

// These are a compile-time error!
// target_gps_point + local_origin // deleted
// Coordinate3D<lla> lla_displ = target_gps_point - local_origin; // result is a Vector3D
  ^------------^

// Lecit: Computes back a Coordinate3D, which equals `target_gps_point`
auto target_gps_point_again = local_origin + lla_displ;
```

## Rotation

The class needed to represent **3D rotations** is `Rotation<FrameTo, FrameFrom, T>`. The rotation describes mathematically the orientation of `FromFrame` with respect to `ToFrame` and is internally implemented with a [`UnitQuaternion`](#unitquaternion), which represents its container unit. The type is a template which accepts three parameters: the target reference frame (`FrameTo`), the source reference frame (`FrameFrom`), and the type which defines the numerical precision employed for representing the rotation internal elements (`T`). The default value for the numerical precision is `double`. 

The available **Constructors** are:
  * `Rotation()`: Default constructor, creates an identity (zero-angle) rotation.
  * `explicit Rotation(const UnitQuaternion<T>&)`: Constructs a `Rotation` object from a frame-agnostic [`UnitQuaternion<T>`](#unitquaternion) object.
  * `explicit Rotation(const EulerAngles<Seq, T>&)`: Constructs a `Rotation` object from a template [`EulerAngles<Seq, T>`](#eulerangles) object.

The available **Factories** are:
  * `static from_axis_angle(const Vector3D<FromFrame, T>&, T)`: Returns a `Rotation` from a frame-agnostic axis-angle angular parametrization. The input [`Vector3D<Frame, T>`](#vectors) is normalized to ensure unit-norm.

```cpp
// An identity rotation from body (frd) to world (ned)
refx::Rotation<refx::ned, refx::frd> R_ned_frd;

// A rotation from a YawPitchRoll data container: 90 deg yaw
auto ypr = refx::YawPitchRoll<double>(refx::deg2rad(90.0), 0.0, 0.0);
refx::Rotation<refx::ned, refx::frd> R_from_ypr(ypr);
```

The type provides the following **member functions**:

  * `.to_quaternion()`: Returns the orientation as a [`UnitQuaternion<T>`](#unitquaternion) object.
  * `.to_euler_angles<Seq>()`: Returns the orientation as an [`EulerAngles<Seq, T>`](#eulerangles) object.
  * `.inverse()`: Computes the inverse rotation, returning a `Rotation<FromFrame, ToFrame, T>`.
  * `.normalize()`: Normalizes the internal [`UnitQuaternion`](#unitquaternion) implementing the `Rotation` object. Returns `void`.

#### Eigen3 Support
If the Eigen3 support is enabled and the library is found, the macro `REFX_ENABLE_EIGEN_SUPPORT` is automatically defined. The following supplementary constructors are made available:
  * `explicit Rotation(const Eigen::Quaternion<T>&)`: Constructs a `Rotation` object from a Eigen3 quaternion.
  * `explicit Rotation(const Eigen::Matrix<T, 3, 3>&)`: Constructs a `Rotation` object from a Eigen3 rotation matrix.

The library, provides the following supplementary member functions:
  * `.to_eigen_quat()`: Returns a `Eigen::Quaternion<T>`.
  * `.to_eigen_matrix()`: Returns the rotation matrix as `Eigen::Matrix<T, 3, 3>` .

#### Geometric Operators

The header defines a set of operators that enforce compile-time frame safety.

  * `operator* (const Rotation<FrameA, FrameB, T>&, const Rotation<FrameB, FrameC>&)`: Composes two rotations (`R_ac = R_ab * R_bc`). The operation is only valid if the inner frames of `R_ab` and `R_bc` match.
  * `operator* (const Rotation<ToFrame, FromFrame, T>&, const Vector3D<FromFrame>&)`: Rotates a vector from the source frame to the destination frame. The operation is only valid if the vector's frame matches the rotation's source reference frame (`FromFrame`).


```cpp
// Define two rotations
auto R_ned_frd = refx::Rotation<refx::ned, refx::frd>(
    refx::YawPitchRoll<double>(refx::deg2rad(90.0), 0.0, 0.0));
auto R_frd_imu = refx::Rotation<refx::frd, refx::imu>(
    refx::YawPitchRoll<double>(0.0, 0.0, refx::deg2rad(180.0)));

// Lecit: Compose rotations to get the direct rotation from imu to ned
auto R_ned_imu = R_ned_frd * R_frd_imu;
// This is a compilet-time error!
// auto R_ned_imu = R_frd_imu * R_ned_frd; // Inner frame (imu|ned) is different
                         ^---^   ^---^

// A vector in the imu frame
refx::Vector3D<refx::imu> accel_imu(0.1, 0.2, 9.8);

// Lecit: Rotate the vector into the ned frame
refx::Vector3D<refx::ned> accel_ned = R_ned_imu * accel_imu;
// This is a compile-time error! The vector is not in the 'imu' frame
// auto accel_err = R_ned_imu * refx::Vector3D<refx::frd>();
                                                    ^---^
```

### UnitQuaternion

The `UnitQuaternion<T>` struct is a frame-agnostic container for the four components of a unit quaternion (`w, x, y, z`). It serves as a plain data object for creating or extracting orientation data from the main [`Rotation`](#rotation) class and provides the fundamental operators for quaternion algebra. The type inherit the following **base (4D) vector accessors** from the struct `internal::VectorContainer4D`:
  * `T& w()`: Returns a reference to the quaternion's scalar term.
  * `T& x()`: Returns a reference to the quaternion's first component.
  * `T& y()`: Returns a reference to the quaternion's second component.
  * `T& z()`: Returns a reference to the quaternion's third component.
  * `T w() const`: Returns the quaternion's scalar term by copy. Can be used with `const` objects.
  * `T x() const`: Returns the quaternion's first component by copy. Can be used with `const` objects.
  * `T y() const`: Returns the quaternion's second component by copy. Can be used with `const` objects.
  * `T z() const`: Returns the quaternion's third component by copy. Can be used with `const` objects.

The available **Constructors** are:

  * `UnitQuaternion()`: Default constructor, initializes to an identity `UnitQuaternion` (w=1, x=0, y=0, z=0).
  * `UnitQuaternion(T w, T x, T y, T z)`: Constructs a `UnitQuaternion` from its four components, in the order: `w`, `x`, `y` and `z`.

The available  **Factories** are:
  * `static from_rotation_x(T angle)`: Returns a `UnitQuaternion` representing the elementary rotation around the X axis. The angle must be in **radians**.
  * `static from_rotation_y(T angle)`: Returns a `UnitQuaternion` representing the elementary rotation around the Y axis. The angle must be in **radians**.
  * `static from_rotation_z(T angle)`: Returns a `UnitQuaternion` representing the elementary rotation around the Z axis. The angle must be in **radians**.

```cpp
// Default constructor: identity quaternion
refx::UnitQuaternion<double> q_identity; // w=1, x=0, y=0, z=0

// Component-wise constructor
refx::UnitQuaternion<double> q_custom(0.707, 0.707, 0.0, 0.0);

// Factory for a 90-degree rotation around the Z-axis
auto q_from_z = refx::UnitQuaternion<double>::from_rotation_z(refx::deg2rad(90.0));
```

The type provides the following **member functions**:

  * `.conjugate()`: Computes the conjugate of the quaternion, which is its inverse for a unit quaternion.
  * `.normalize()`: Normalizes the quaternion in-place to ensure it has unit length.

The type provides the following **operators**:
  * `operator*=(const UnitQuaternion<T>& q)`: perfoms an in-place quaternion multiplication (Hamilton product). Returns `*this`.
  * `operator*(UnitQuaternion<T> lhs, const UnitQuaternion<T>& rhs)`: perfoms a quaternion multiplication (Hamilton product). Returns a `UnitQuaternion<T>`.
  * `operator<<(std::ostream& os, const UnitQuaternion<T>& q)`: convenient stream output operator for debugging quaternions. Formats the quaternion's components as `[w, x, y, z]`.

The following free functions API are provided for utility conversions:
  * `quat_to_euler<Seq, T>(const UnitQuaternion<T>&)`: Returns the [`EulerAngles<Seq,T>`](#eulerangles) corresponding to the input `UnitQuaternion`. The function is templated with respect to the desired `EulerSequence` of the ouput Euler angles struct.
  * `euler_to_quat(const EulerAngles<Seq, T>&)`: Returns the `UnitQuaternion<T>` corresponding to the input [`EulerAngles<Seq,T>`](#eulerangles).

#### Eigen3 Support
If the Eigen3 support is enabled and the library is found, the macro `REFX_ENABLE_EIGEN_SUPPORT` is automatically defined. The following supplementary constructors are made available:
  * `explicit UnitQuaternion(const Eigen::Quaternion<T>&)`: Constructs a `UnitQuaternion` object from a Eigen3 quaternion.
  * `explicit UnitQuaternion(const Eigen::Matrix<T, 3, 3>&)`: Constructs a `UnitQuaternion` object from a Eigen3 rotation matrix.

The library, provides the following supplementary member functions:
  * `.to_eigen()`: Returns a `Eigen::Quaternion<T>`.


#### Geometric Operators

The following operators are provided for the `UnitQuaternion<T>` struct:
  * `operator*(const UnitQuaternion<T>& ql, const UnitQuaternion<T>& qr)`: Returns the `UnitQuaterion<T>` resulting from the Hamilton product `ql * qr`.

### EulerAngles

The `EulerAngles<Seq, T>` struct is a frame-agnostic container for Tait-Bryan Euler angles. It avoids ambiguity by making the rotation sequence a compile-time template parameter, via the enum `EulerSequence Seq`. Such enum defines the six available Tait-Bryant rotation sequences:

```cpp
enum class EulerSequence {
    ZYX,  ///< Standard for aerospace and navigation (Yaw, Pitch, Roll).
    ZXY,  ///< e.g., for camera pointing.
    YZX,  ///< A valid Tait-Bryan sequence.
    YXZ,  ///< e.g., for character animation.
    XYZ,  ///< Common in robotics and computer graphics.
    XZY   ///< A valid Tait-Bryan sequence.
};
```

The generic template type cannot be instantiated; instead, it is specialized per each `EulerSequence` value. Per each specialization, the type's constructor expects the input angles **in the same order of the sequence** the specialized type refers to.  
All angles are stored and handled in **radians**.

##### Base angle accessors
All the specializations inherit the data container, the [base vector accessors](#base-vector-accessors) and the following **contextual angle-accessors** from the user-restricted struct `internal::EulerBase<T>`:

  * `T& angle_x()`: returns a reference to the first angle, corresponding to the first axis in the sequence.
  * `T& angle_y()`: returns a reference to the second angle, corresponding to the second axis in the sequence.
  * `T& angle_z()`: returns a reference to the third angle, corresponding to the third axis in the sequence.
  * `T angle_x() const`: returns the angle corresponding to the first axis in the sequence, by copy. Can be used with `const` objects.
  * `T angle_y() const`: returns the angle corresponding to the second axis in the sequence, by copy. Can be used with `const` objects.
  * `T angle_z() const`: returns the angle corresponding to the third axis in the sequence, by copy. Can be used with `const` objects.

The following free functions API are provided for utility conversions:
  * `quat_to_euler<Seq, T>(const UnitQuaternion<T>&)`: Returns the `EulerAngles<Seq,T>` corresponding to the input [`UnitQuaternion<T>`](#unitquaternion). The function is templated with respect to the desired `EulerSequence` of the ouput Euler angles struct.
  * `euler_to_quat(const EulerAngles<Seq, T>&)`: Returns the [`UnitQuaternion<T>`](#unitquaternion) corresponding to the input `EulerAngles`.


#### Sequence: ZYX (Yaw-Pitch-Roll)
The available **Constructor** is:
  * `EulerAngles<EulerSequence::ZYX, T>()`: Constructs an identity (zero-angle) rotation ZYX Euler angle object.
  * `EulerAngles<EulerSequence::ZYX, T>(T yaw, T pitch, T roll)`: Constructs a ZYX Euler angle object, in the order `yaw`, `pitch` and `roll`. All angles must be in **radians**.

For this sequence, the library provides the convenient alias `YawPitchRoll<T>` also.  

```cpp
using EulerAnglesZYXd = refx::EulerAngles<EulerSequence::ZYX,double>;
// A YPR sequence representing a 45-degree yaw and 10-degree pitch
auto ypr_angles = EulerAnglesZYXd(refx::deg2rad(45.0), refx::deg2rad(10.0), 0.0);

// The same object is constructed by:
// auto ypr_angles = refx::YawPitchRoll<double>(refx::deg2rad(45.0), refx::deg2rad(10.0), 0.0);
```

The struct contains the following `static constexpr` members, which allow for compile-time reflection on the properties of the specific Euler sequence.

  * `axis_order`: maps the order of operations (first, second, third rotation) to the frame's axes (`Axis0` for X, `Axis1` for Y, `Axis2` for Z).

    ```cpp
    /// @brief Defines the axis rotation order for a ZYX sequence.
    static constexpr AxisOrder axis_order[3] = {
        AxisOrder::Axis2,  ///< First rotation around the z-axis.
        AxisOrder::Axis1,  ///< Second rotation around the y-axis.
        AxisOrder::Axis0,  ///< Third rotation around the x-axis.
    };
    ```

  * `angles_wrap` defines the **mathematical domain** for each of the three angles in the sequence.

    ```cpp
    /// @brief Defines the angle range for each angle in a ZYX sequence.
    static constexpr AxisDomain angles_wrap[3] = {
        AxisDomain::WrappedAngularPi,   ///< Roll wraps around [-pi, pi].
        AxisDomain::WrappedAngularPi2,  ///< Pitch wraps around at [-pi/2, pi/2].
        AxisDomain::WrappedAngularPi    ///< Yaw wraps around [-pi, pi].
    };
    ```

The type provides the following **accessors**:

**Inherited**  
 * [**base angle accessors**](#base-angle-accessors)

**Contextual**
 * `T& roll()`: Returns a reference to Roll in **radians**.
 * `T& pitch()`: Returns a reference to Picth in **radians**.
 * `T& yaw()`: Returns a reference to Yaw in **radians**.
 * `T roll() const`: Returns the Roll in **radians**, by copy. Can be used with `const` objects.
 * `T pitch() const`: Returns the Pitch in **radians**, by copy. Can be used with `const` objects.
 * `T yaw() const`: Returns the Yaw in **radians**, by copy. Can be used with `const` objects.

```cpp
// Access the pitch angle
double pitch_rad = ypr_angles.pitch(); // returns 0.1745...
```

The type provides the following **member functions**:
  * `inverse()`: Returns a `EulerAngles<EulerSequence::XYZ, T>` object, correspondent to the inverse rotation of `*this`

#### Sequence: ZXY
The available **Constructor** is:
  * `EulerAngles<EulerSequence::ZXY, T>()`: Constructs an identity (zero-angle) rotation ZXY Euler angle object.
  * `EulerAngles<EulerSequence::ZXY, T>(T ang_z, T ang_x, T ang_y)`: Constructs a ZXY Euler angle object, in the order `angle_z`, `angle_x` and `angle_y`. All angles must be in **radians**.

```cpp
using EulerAnglesZXYd = refx::EulerAngles<EulerSequence::ZXY,double>;
// A ZXY sequence representing a 45-degree z-axis rotation and 10-degree x-axis
auto zxy_angles = EulerAnglesZXYd(refx::deg2rad(45.0), refx::deg2rad(10.0), 0.0);
```

The struct contains the following `static constexpr` members, which allow for compile-time reflection on the properties of the specific Euler sequence.

  * `axis_order`: maps the order of operations (first, second, third rotation) to the frame's axes (`Axis0` for X, `Axis1` for Y, `Axis2` for Z).

    ```cpp
    /// @brief Defines the axis rotation order for a ZXY sequence.
    static constexpr AxisOrder axis_order[3] = {
        AxisOrder::Axis2,  ///< First rotation around the z-axis.
        AxisOrder::Axis0,  ///< Second rotation around the x-axis.
        AxisOrder::Axis1,  ///< Third rotation around the y-axis.
    };
    ```

  * `angles_wrap` defines the **mathematical domain** for each of the three angles in the sequence.

    ```cpp
    /// @brief Defines the angle range for each angle in a ZXY sequence.
    static constexpr AxisDomain angles_wrap[3] = {
        AxisDomain::WrappedAngularPi2,  ///< ang_x (2nd angle) wraps around at [-pi/2, pi/2].
        AxisDomain::WrappedAngularPi,   ///< ang_y (3rd angle) wraps around [-pi, pi].
        AxisDomain::WrappedAngularPi    ///< ang_z (1st angle) wraps around [-pi, pi].
    };
    ```

The type provides the [**base angle accessors**](#base-angle-accessors) inherited by `internal::EulerBase<T>`.

```cpp
// Access the angle relative to the **second** axis of rotation
double angle_x_rad = zxy_angles.angle_x(); // returns 0.1745...
```

The type provides the following **member functions**:
  * `inverse()`: Returns a `EulerAngles<EulerSequence::YXZ, T>` object, correspondent to the inverse rotation of `*this`

#### Sequence: XYZ
The available **Constructor** is:
  * `EulerAngles<EulerSequence::XYZ, T>()`: Constructs an identity (zero-angle) rotation XYZ Euler angle object.
  * `EulerAngles<EulerSequence::XYZ, T>(T ang_x, T ang_y, T ang_z)`: Constructs a XYZ Euler angle object, in the order `angle_x`, `angle_y` and `angle_z`. All angles must be in **radians**.

```cpp
using EulerAnglesXYZd = refx::EulerAngles<EulerSequence::XYZ,double>;
// A XYZ sequence representing a 45-degree x-axis rotation and 10-degree y-axis
auto xyz_angles = EulerAnglesXYZd(refx::deg2rad(45.0), refx::deg2rad(10.0), 0.0);
```
The struct contains the following `static constexpr` members, which allow for compile-time reflection on the properties of the specific Euler sequence.

  * `axis_order`: maps the order of operations (first, second, third rotation) to the frame's axes (`Axis0` for X, `Axis1` for Y, `Axis2` for Z).

    ```cpp
    /// @brief Defines the axis rotation order for a XYZ sequence.
    static constexpr AxisOrder axis_order[3] = {
        AxisOrder::Axis0,  ///< First rotation around the x-axis.
        AxisOrder::Axis1,  ///< Second rotation around the y-axis.
        AxisOrder::Axis2,  ///< Third rotation around the z-axis.
    };
    ```

  * `angles_wrap` defines the **mathematical domain** for each of the three angles in the sequence.

    ```cpp
    /// @brief Defines the angle range for each angle in a XYZ sequence.
    static constexpr AxisDomain angles_wrap[3] = {
        AxisDomain::WrappedAngularPi,   ///< ang_x (1st angle) wraps around at [-pi, pi].
        AxisDomain::WrappedAngularPi2,  ///< ang_y (2nd angle) wraps around [-pi/2, pi/2].
        AxisDomain::WrappedAngularPi    ///< ang_z (3rd angle) wraps around [-pi, pi].
    };
    ```

The type provides the [**base angle accessors**](#base-angle-accessors) inherited by `internal::EulerBase<T>`.

```cpp
// Access the angle relative to the **first** axis of rotation
double angle_x_rad = xyz_angles.angle_x(); // returns 0.7853...
```

The type provides the following **member functions**:
  * `inverse()`: Returns a `EulerAngles<EulerSequence::ZYX, T>` object, correspondent to the inverse rotation of `*this`

#### Sequence: XZY
The available **Constructor** is:
  * `EulerAngles<EulerSequence::XZY, T>()`: Constructs an identity (zero-angle) rotation XZY Euler angle object.
  * `EulerAngles<EulerSequence::XZY, T>(T ang_x, T ang_z, T ang_y)`: Constructs a XZY Euler angle object, in the order `angle_x`, `angle_z` and `angle_y`. All angles must be in **radians**.

```cpp
using EulerAnglesXZYd = refx::EulerAngles<EulerSequence::XZY,double>;
// A XZY sequence representing a 45-degree x-axis rotation and 10-degree z-axis
auto xzy_angles = EulerAnglesXZYd(refx::deg2rad(45.0), refx::deg2rad(10.0), 0.0);
```

The struct contains the following `static constexpr` members, which allow for compile-time reflection on the properties of the specific Euler sequence.

  * `axis_order`: maps the order of operations (first, second, third rotation) to the frame's axes (`Axis0` for X, `Axis1` for Y, `Axis2` for Z).

    ```cpp
    /// @brief Defines the axis rotation order for a XZY sequence.
    static constexpr AxisOrder axis_order[3] = {
        AxisOrder::Axis0,  ///< First rotation around the x-axis.
        AxisOrder::Axis2,  ///< Second rotation around the z-axis.
        AxisOrder::Axis1,  ///< Third rotation around the y-axis.
    };
    ```

  * `angles_wrap` defines the **mathematical domain** for each of the three angles in the sequence.

    ```cpp
    /// @brief Defines the angle range for each angle in a XZY sequence.
    static constexpr AxisDomain angles_wrap[3] = {
        AxisDomain::WrappedAngularPi,  ///< ang_x (1st angle) wraps around at [-pi, pi].
        AxisDomain::WrappedAngularPi,  ///< ang_y (3rd angle) wraps around [-pi, pi].
        AxisDomain::WrappedAngularPi2  ///< ang_z (2nd angle) wraps around [-pi/2, pi/2].
    };
    ```

The type provides the [**base angle accessors**](#base-angle-accessors) inherited by `internal::EulerBase<T>`.

```cpp
// Access the angle relative to the **first** axis of rotation
double angle_x_rad = xzy_angles.angle_x(); // returns 0.7853...
```

The type provides the following **member functions**:
  * `inverse()`: Returns a `EulerAngles<EulerSequence::YZX, T>` object, correspondent to the inverse rotation of `*this`

#### Sequence: YXZ
The available **Constructor** is:
  * `EulerAngles<EulerSequence::YXZ, T>()`: Constructs an identity (zero-angle) rotation YXZ Euler angle object.
  * `EulerAngles<EulerSequence::YXZ, T>(T ang_y, T ang_x, T ang_z)`: Constructs a YXZ Euler angle object, in the order `angle_y`, `angle_x` and `angle_z`. All angles must be in **radians**.

```cpp
using EulerAnglesYXZd = refx::EulerAngles<EulerSequence::YXZ,double>;
// A YXZ sequence representing a 45-degree y-axis rotation and 10-degree x-axis
auto yxz_angles = EulerAnglesYXZdrefx::deg2rad(45.0), refx::deg2rad(10.0), 0.0);
```

The struct contains the following `static constexpr` members, which allow for compile-time reflection on the properties of the specific Euler sequence.

  * `axis_order`: maps the order of operations (first, second, third rotation) to the frame's axes (`Axis0` for X, `Axis1` for Y, `Axis2` for Z).

    ```cpp
    /// @brief Defines the axis rotation order for a YXZ sequence.
    static constexpr AxisOrder axis_order[3] = {
        AxisOrder::Axis1,  ///< First rotation around the y-axis.
        AxisOrder::Axis0,  ///< Second rotation around the x-axis.
        AxisOrder::Axis2,  ///< Third rotation around the z-axis.
    };
    ```

  * `angles_wrap` defines the **mathematical domain** for each of the three angles in the sequence.

    ```cpp
    /// @brief Defines the angle range for each angle in a YXZ sequence.
    static constexpr AxisDomain angles_wrap[3] = {
        AxisDomain::WrappedAngularPi2,  ///< ang_x (2nd angle) wraps around at [-pi/2, pi/2].
        AxisDomain::WrappedAngularPi,   ///< ang_y (1st angle) wraps around [-pi, pi].
        AxisDomain::WrappedAngularPi    ///< ang_z (3rd angle) wraps around [-pi, pi].
    };
    ```

The type provides the [**base angle accessors**](#base-angle-accessors) inherited by `internal::EulerBase<T>`.

```cpp
// Access the angle relative to the **second** axis of rotation
double angle_x_rad = yxz_angles.angle_x(); // returns 0.1745...
```

The type provides the following **member functions**:
  * `inverse()`: Returns a `EulerAngles<EulerSequence::ZXY, T>` object, correspondent to the inverse rotation of `*this`

#### Sequence: YZX
The available **Constructor** is:
  * `EulerAngles<EulerSequence::YZX, T>()`: Constructs an identity (zero-angle) rotation YZX Euler angle object.
  * `EulerAngles<EulerSequence::YZX, T>(T ang_y, T ang_z, T ang_x)`: Constructs a YZX Euler angle object, in the order `angle_y`, `angle_z` and `angle_x`. All angles must be in **radians**.

```cpp
using EulerAnglesYZXd = refx::EulerAngles<EulerSequence::YZX,double>;
// A YZX sequence representing a 45-degree y-axis rotation and 10-degree x-axis
auto yzx_angles = EulerAnglesYZXd(refx::deg2rad(45.0), refx::deg2rad(10.0), 0.0);
```

The struct contains the following `static constexpr` members, which allow for compile-time reflection on the properties of the specific Euler sequence.

  * `axis_order`: maps the order of operations (first, second, third rotation) to the frame's axes (`Axis0` for X, `Axis1` for Y, `Axis2` for Z).

    ```cpp
    /// @brief Defines the axis rotation order for a YZX sequence.
    static constexpr AxisOrder axis_order[3] = {
        AxisOrder::Axis1,  ///< First rotation around the y-axis.
        AxisOrder::Axis2,  ///< Second rotation around the z-axis.
        AxisOrder::Axis0,  ///< Third rotation around the x-axis.
    };
    ```

  * `angles_wrap` defines the **mathematical domain** for each of the three angles in the sequence.

    ```cpp
    /// @brief Defines the angle range for each angle in a YZX sequence.
    static constexpr AxisDomain angles_wrap[3] = {
        AxisDomain::WrappedAngularPi,  ///< ang_x (3rd angle) wraps around at [-pi, pi].
        AxisDomain::WrappedAngularPi,  ///< ang_y (1st angle) wraps around [-pi, pi].
        AxisDomain::WrappedAngularPi2  ///< ang_z (2nd angle) wraps around [-pi/2, pi/2].
    };
    ```

The type provides the [**base angle accessors**](#base-angle-accessors) inherited by `internal::EulerBase<T>`.

```cpp
// Access the angle relative to the **third** axis of rotation
double angle_x_rad = yzx_angles.angle_x(); // returns 0.0
```

The type provides the following **member functions**:
  * `inverse()`: Returns a `EulerAngles<EulerSequence::XZY, T>` object, correspondent to the inverse rotation of `*this`

## Transformations

The class needed to represent a full **3D pose** (position and orientation) is `Transformation<ToFrame, FromFrame, T>`. This struct models a rigid body transformation in the Special Euclidean group SE(3), describing the pose of `FromFrame` with respect to `ToFrame`. It combines a rotation and a translation such that the transformation of a point `P` is defined by the operation: `P_in_ToFrame = rotation * P_in_FromFrame + translation`.

The available **Constructors** are:

  * `Transformation()`: Constructs the identity `Transformation` (zero-angle, zero-translation).
  * `Transformation(const Rotation&, const Vector3D&)`: Constructs a `Transformation` from a [`Rotation`](#rotation) and a [`Vector3D`](#vectors) translation component.
  * `Transformation(const Rotation&, const Coordinate3D&)`: Constructs a `Transformation` from a [`Rotation`](#rotation) and a [`Coordinate3D`](#coordinates) position, internally converting the coordinate to its vector representation.

<!-- end list -->

```cpp
// The orientation of the vehicle (frd) in the world (ned)
auto R_ned_frd = refx::Rotation<refx::ned, refx::frd>(
    refx::YawPitchRoll<double>(refx::deg2rad(20.0), 0.0, 0.0));

// The position of the vehicle in the world
auto p_ned = refx::Coordinate3D<refx::ned>(100.0, 50.0, -5.0);

// The complete pose of the vehicle in the world
refx::Transformation<refx::ned, refx::frd> T_ned_frd(R_ned_frd, p_ned);
```

The `Transformation` struct contains two public **members**:

  * `rotation`: A `Rotation<ToFrame, FromFrame, T>` object representing the orientation of the `FromFrame` relative to the `ToFrame`.
  * `translation`: A `Vector3D<ToFrame, T>` object representing the displacement between the `FromFrame`'s origin and the `ToFrame`'s origin, expressed in `ToFrame` reference frame.

The type provides the following **member functions**:

  * `.inverse()`: Computes the inverse transformation, returning a `Transformation<FromFrame, ToFrame, T>`. This is useful for changing the direction of the relationship between two frames.

### Geometric Operators

The header defines operators for applying and composing transformations.

  * `operator* (const Transformation<ToFrame, FromFrame, T>&, const Vector3D<FromFrame>&)`: Applies the full transformation to the input vector. The operation is only valid if the vector's frame matches the transformation's `FromFrame`.
  * `operator* (const Transformation<FrameA, FrameB, T>&, const Transformation<FrameB, FrameC, T>&)`: Composes (chains) two transformations. `T_ac = T_ab * T_bc`. The operation is only valid if the inner frames (`FrameB`) match.

<!-- end list -->

```cpp
// Pose of the camera relative to the vehicle (frd)
auto T_frd_cam = refx::Transformation<refx::frd, refx::camera>(R_frd_cam, p_frd);

// Pose of the vehicle (frd) relative to the world (ned)
auto T_ned_frd = refx::Transformation<refx::ned, refx::frd>(R_ned_frd, p_ned);

// Lecit: Compose transformations to get the camera's pose in the world
auto T_ned_cam = T_ned_frd * T_frd_cam;

// A point detected by the camera
refx::Vector3D<refx::camera> point_cam(0, 0, 20.0);

// Lecit: Transform the point directly into the world frame
refx::Vector3D<refx::ned> point_ned = T_ned_cam * point_cam;
```