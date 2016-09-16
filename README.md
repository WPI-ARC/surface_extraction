# Surface Extraction

This project is an on-demand surface extraction framework designed for humanoid locomotion planning. Its key feature is the ability to incrementally process the scene in order to waste unnecessary effort processing areas of the environment for which surface information is not required.

## Setup

Surface Extraction runs on Ubuntu 16.04 and ROS Kinetic (http://wiki.ros.org/kinetic/Installation/Ubuntu)

1. Install PCL version 1.8. As of this writing PCL 1.8 is not available by PPA, so it must be installed from source as described here: http://pointclouds.org/documentation/tutorials/compiling_pcl_posix.php
2. Install CGAL with `apt-get install libcgal-dev`
3. Download the `arc_utilities` package and put it in your Catkin workspace
4. Download `surface_extraction` and put it in your Catkin workspace
5. Compile with `catkin_make`

## Usage

Surface Extraction is configured entirely through ROS parameters, recieves data on ROS topics, and is queried using ROS services.

### Configuration

According to ROS convention, all distances are expressed in meters and all angles are expressed in radians.

Parameters:
- `target_frame` (string, default `"/world"`: The frame that surfaces will be computed relative to. Assumed to be static. Requesting surfaces in a different frame than `target_frame` is currently not supported.
- `camera_frame` (boolean, default `"/left_camera_frame"`): The frame, or one of the frames, associated with the camera. Used to identify the direction of each plane's normal.
- `discretization` (float, default `0.02`): The discretization resolution to use when discretizing input point clouds. Smaller values give greater accuracy at the cost of greater computation time. This value must be smaller than all distance thresholds.
- `parallel_distance` (float, default `0.06`): The threshold distance along each plane that separates points which are considered to be part of this plane from those which are assumed to be outside the plane.
- `perpendicular_distance` (float, default `0.04`): The threshold distance above and below each plane which separates points which are considered to be a part of the plane from points which are assumed to be outside the plane.
- `point_inside_threshold` (float, default `0.1`): Newly-recieved points which are this close to an existing surface will not be recorded, and are assumed to be either part of the existing surface or belong to an object on the surface.
- `mls_radius` (float, default `0.12`): The radius to use when computing neighbors for normals estimation.
- `min_points_per_surface` (integer, default `50`): Surfaces with fewer than this number of points are assumed to be too small to be useful and are discarded.
- `min_plane_width` (float, default `0.15`): If the spread of points in any direction along the surface plane is smaller than this value, the surface is assumed to be too small to be useful and is discarded.
- `extrusion_distance` (float, default `0.02`): The assumed thickness of every surface. This is used when computing the surface mesh.
- `optimistic` (boolean, default `true`): Controls whether the entire area of every newly detected surface is expanded (`true`) or just the area within the query volume (`false`). NOTE The implementation of pessimistic mode is currently broken. 
- `enable_visualization` (boolean, default `true`): If this is true the query box and the polygons, meshes, and normals of every detected surface are published on `surface_markers` and the unprocessed points are published on the topic specified by the `unprocessed_points_topic` parameter.
- `initial_surface` (float, optional): Add an implicit starting surface at the robot's feet, which cannot be seen because the robot is in the way but whose existence can be inferred from the fact that the robot is standing on it. The surface is a square in the XY plane with the given side length and centered at (0, 0, 0). If this is not specified, no initial surface is added.
- `wait_for_points` (integer, optional): The number of points which must be collected before the robot can expect to find surfaces. The service will not be advertised until the specified number of points is collected. Clients should use `waitForService()` to delay until the service is available. If this is not specified, the service is advertised immediately.

Input:
- `input_points_topic` (string, default `"/scan_cloud"`): The topic on which to recieve new points. A transform from the point clouds' frame to `target_frame` must exist.

Output:
- `unprocessed_points_topic` (string, default `"/pending_points"`): The topic on which to publish the collected but unprocessed points.

### Calling
