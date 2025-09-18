---
id: index
title: Cookbook
slug: /cookbook
sidebar_label: The refx Cookbook
---

# The refx Cookbook

Welcome to the **refx Cookbook**! This is a collection of practical, real-world recipes designed to help you solve common problems in robotics and navigation. While the [Full Documentation](category/full-documentation) explains *what* each component does, this cookbook shows you *how* to put them together to build robust, safe, and readable software.

Each recipe is a handout in the refx philosophy of *thinking in frames*. You'll learn how to leverage the refx type system to prevent bugs before they happen and write code that is a clear reflection of the underlying mathematics.

## Table of Contents

Here you will find a list of available recipes, each tackling a specific challenge. The list is constantly updated.

### [**From Sensor Measurement to Global Position**](relative_to_global.md)
**Goal**: Take a sensor detection from a vehicle's local frame and find its absolute position in a global navigation frame.  
**Concepts Covered**: [`Coordinate3D`](full_doc/geometry_h#coordinates), [`frame_transform`](full_doc/transformations_h#complex-transformations-frame_transform).


### [**IMU Measurements Compensation for Navigation Filters**](imu_comp.md)
**Goal**: Correct raw IMU measurements for the effects of gravity and the Earth's rotation to prepare them for integration in a navigation filter.  
**Concepts Covered**: Using high-fidelity [`EarthModel`](full_doc/models_h#earth-model)s, transforming physical vectors ([`gravity`](full_doc/models_h#gravity-model), etc.) between frames, practical application of [`Rotation`](full_doc/geometry_h#rotation) and [`Vector3D`](full_doc/geometry_h#vectors) for sensor fusion.
