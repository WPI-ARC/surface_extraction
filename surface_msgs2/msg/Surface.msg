# A sequential numeric ID uniquely identifies the surface
uint32 id

# Each surface has a unique random color which helps to visuually identify it across different visualizations
std_msgs/ColorRGBA color

# Describes the plane using a plane equation in the for ax + by + cz + d = 0, where ||<a, b, c>|| = 1
pcl_msgs/ModelCoefficients model

# The surface pose. The position is always in the surface plane (but not necessarily inside the polygon). The x and y
# axes are always on the plane. The pose begins with the position in the center (mean) of the polygon and with the x
# axis aligned to the polygon's largest dimension. When new area is added to the surface, the plane equation is re-
# calculated and the pose is moved as little as possible while also remaining on the plane. Poses relative to the
# surface pose will always be on the surface and have as high a probability as possible of remaining valid for their
# original purpose
geometry_msgs/Pose pose

# A point cloud containing all the points on the plane
sensor_msgs/PointCloud2 inliers

# A list of boundary points, where the largest gap between points is controlled by a parameter to the surface detection
sensor_msgs/PointCloud2 boundary

# The surface polygon, described as a list of simple polygons, each of which is an ordered list of indices into the
# inliers point cloud. The first item in the list is the outer boundary, and all subsequent items describe holes. Every
# hole polygon will be completely contained within the outer boundary, but it is possible for a hole to share a vertex
# with the outer boundary. Both the outer polygon and all holes are indexed counter-clockwise with respect to the
# plane normal. Unlike the boundary, there is no maximum distance between points (leading to simpler polygons)
pcl_msgs/Vertices[] polygons

# A triangle mesh describing the surface. The mesh is a thin (width controlled by a parameter to the surface detection)
# extrusion of the surface polygon in the direction opposite the normal vector.
shape_msgs/Mesh mesh
