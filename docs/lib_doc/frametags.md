---
sidebar_position: 6
---

# Frame Tags Reference

| Tag | Description |
| :--- | :--- |
| **Geocentric / Global Frames** | |
| `ecef` | Earth-Centered, Earth-Fixed. A global Cartesian frame. |
| `lla` | Geodetic Latitude, Longitude, Altitude. |
| `lld` | Geodetic Latitude, Longitude, Down. |
| **Local Tangent Frames** | |
| `ned` | North-East-Down. A right-handed local Cartesian frame. |
| `enu` | East-North-Up. A right-handed local Cartesian frame. |
| `nwu` | North-West-Up. A right-handed local Cartesian frame. |
| `wa_generic` | Generic Wander Azimuth. A generic local Cartesian frame, templated wrt axis orientation. |
| `wa_ned` | Wander Azimuth. A standard generic local Cartesian frame (underlying NED). |
| `wa` | Alias for `wa_ned`. |
| `wa_enu` | Wander Azimuth. A generic local Cartesian frame (underlying ENU). |
| `aer` | Azimuth, Elevation, Range. A local spherical frame. |
| **Body-Fixed Frames** | |
| `frd` | Forward-Right-Down. A right-handed body frame (aerospace standard). |
| `flu` | Forward-Left-Up. A right-handed body frame (robotics standard, ROS). |
| `rfu` | Right-Forward-Up. A right-handed body frame (vision standard). |
| `ssh` | Surge-Sway-Heave. A velocity frame for marine robotics. |
| **Sensor-Fixed Frames** | |
| `imu` | [Forward-Left-Up]. A right-handed frame usually related to Inertial Sensor suites. |
| `camera` | [Right-Down-Forward]. A right-handed camera (pinhole) frame. |