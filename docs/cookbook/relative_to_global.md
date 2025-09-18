# From Sensor Measurement to Global Position

This recipe shows how to take a sensor measurement from a vehicle's body-fixed frame and correctly determine its absolute position in a global navigation frame. This is a fundamental task in robotics, autonomous driving, and aviation.

## The Goal

An ego-vehicle, equipped with a sensor like a radar, detects another vehicle (a target). We know our own absolute position and orientation in the world. Our goal is to find the **absolute world position of the target vehicle**.

To achieve this, we will first need to **personalize the library by [defining a custom](/short_doc#types-customization) `radar` frame** to represent our sensor. This is a crucial first step in building a complete and accurate frame transformation pipeline.

## Ingredients

  * [`refx::Coordinate3D`](/full_doc/geometry_h#coordinates): To represent specific points in a frame.
  * [`refx::Rotation`](/full_doc/geometry_h#rotation): To represent orientation.
  * [`refx::frame_transform`](/full_doc/transformations_h#complex-transformations-frame_transform): The function that correctly changes a coordinate's frame of reference.
  * **Frames**:
      * [`refx::lla`](/full_doc/frames_h#latitude-longitude-altitude-lla): A global geocentric frame for absolute localization.
      * [`refx::ned`](/full_doc/frames_h#north-east-down-ned): A local **N**orth-**E**ast-**D**own frame as intermediate navigation frame.
      * [`refx::frd`](/full_doc/frames_h#forward-right-down-frd): A body-fixed **F**orward-**R**ight-**D**own frame attached to our ego-vehicle.
      * *`my_robot::radar`*: A customized sensor-fixed frame, attached to our radar sensor.


## Step-by-Step Instructions

### Step 0: Customize the library with the radar sensor frame

Since the library does not provide a `radar` frame, it is a good occasion for library customization. All we need to do is to define the `radar` frame by defining a simple C++ struct.

```cpp
// file my_robot_frames.h

// Include 'developer-level' headers
#include <refx/frames/axis.h>
#include <refx/frames/tags.h>

namespace my_robot {

// A custom frame for a radar sensor.
// It is configured with a standard Forward-Right-Down (FRD) axis orientation.
struct radar {
    // The frame's name for identification.
    static constexpr auto name = "radar";

    // Specifies the axis convention. `axis_frd` is for Forward-Right-Down.
    using axis = refx::axis_frd;  
    
    // Assigns the frame a `Sensor` tag for categorization.
    static constexpr refx::FrameTag tag = refx::FrameTag::Sensor;
};

}  // namespace my_robot
```

We don't need to specialize geometric container accessors, the [default ones](/full_doc/geometry_h#base-vector-accessors) are sufficient for our scopes. This simple struct is all that's required for `refx` to recognize and use your custom radar frame.

### Step 1: Define the Ego-Vehicle's State

We need to know our own state in the world. This consists of our position and orientation.

  * Our **absolute position** is a specific point in the [`lla`](/full_doc/frames_h#latitude-longitude-altitude-lla) frame, so it's a [`Coordinate3D<lla>`](/full_doc/geometry_h#latitude-longitude-altitude).
  * Our **orientation** describes how our [`frd`](/full_doc/frames_h#forward-right-down-frd) frame is rotated relative to the local [`ned`](/full_doc/frames_h#north-east-down-ned) frame, so it's a [`Rotation<ned, frd>`](/full_doc/geometry_h#rotation).

<!-- end list -->

```cpp
#include <refx/geometry.h>
#include "my_robot_frames.h"

using namespace refx;

// Our ego-vehicle is at a known global position.
auto ego_position_global = Coordinate3D<lla>(44.3040729, 11.9530427);

// It's pointing -45 degrees off the North (a -45-degree yaw).
auto ego_orientation_global =
    Rotation<ned, frd>(YawPitchRoll<double>(deg2rad(-45.0), 0.0, 0.0));
```

### Step 2: Define the Sensor Measurement

Our radar provides the position of the target vehicle, in its own brand-new `radar` axis; since it is **position** measurement, it can be modeled as a [`Coordinate3D<frd>`](/full_doc/geometry_h#coordinates). The radar is not on the vehicle's origin reference frame, therefore we need to define the **relative calibration** between the sensor and the vehicle body frame.

```cpp
// The complete pose of the radar wrt body frame (relative mounting)
auto T_frd_radar = Transformation<frd, my_robot::radar>(
  // The relative rotation from the radar frame to the body frame
    Rotation<frd, my_robot::radar>(
      YawPitchRoll<double>(deg2rad(2.0), -deg2rad(1.5), 0.0)
    ),
    // The relative translation between the radar and the body frame origins,
    // expressed in the **body frame**
    Vector3D<frd>(1.0, -0.3, 0.15));

// The radar detects the target 75 meters directly in front of us
// and 10 meters to the left. 
auto target_position_relative = Coordinate3D<my_robot::radar>(75.0, -10.0, 0.0);
// The radar measurement in the body frame
auto target_position_relative_body = T_frd_radar * target_position_relative;

std::cout << "Target position relative (Body): " << target_position_relative_body << " [m]" << std::endl;
```

### Step 3: Convert and Transform

To obtain the absolute (geodetic) position of the target vehicle detected by the radar, we must perform a two-step transformation: first, from the sensor's frame to the local navigation frame, and second, from the local navigation frame to the global geodetic frame.

```cpp
// This transformation finds the position of the target vehicle in the ned frame attached to our ego-vehicle
Coordinate3D<ned> target_position_relative_ned = ego_orientation_global * target_position_relative_body;
std::cout << "Target position relative (NED): " << target_position_relative_ned << " [m]" << std::endl;

```

Then this coordinate expressed in a local tangent reference frame **rigidly attached to our ego-vehicle** can be used in a [`frame_transform`](/full_doc/transformations_h#complex-transformations-frame_transform), by employing our global position as origin. What we need, is an [`EarthModel`](/full_doc/models_h#earth-model):

```cpp
// This transform fully defines the target vehicle global position.
Coordinate3D<lla> target_position_global = frame_transform<lla>(
    target_position_relative_ned, ego_position_global, EarthModelWGS84<double>());

// We can now use target_position_global for navigation, tracking, etc.
std::cout << "Target's Absolute Position (LLA): " << target_position_global << " [m]" << std::endl;

```

## Expected Output

```
Target position relative (Body): [76.2776, -7.67734, 2.11327] [m]
Target position relative (NED): [48.5077, -59.3651, 2.11327] [m]
Target's Absolute Position (LLA): [44.3045, 11.9523, -2.11327] [m]
```
-----

## The Main Takeaway

This recipe demonstrates the core philosophy of refx:

  * A **`Coordinate3D`** is a point relative to a specific frame's origin. The radar measurement is a coordinate in the `frd` frame.
  * To find a point's coordinates in a different frame, you must apply a **`frame_transform`**. This avoids ambiguity and prevents common mathematical errors, like adding a relative position to a global one without rotating it first.