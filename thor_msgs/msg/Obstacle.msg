# An obstacle for the control barrier function: a sphere, or a capsule
# (a cylinder with hemispherical ends) if half_axis is not zero.
# Vectors are in the frame of the ObstacleArray header.

# Sphere center, or midpoint of the capsule axis [m].
geometry_msgs/Point position
# Velocity [m/s].
geometry_msgs/Vector3 velocity
# Acceleration [m/s^2]; zero if unknown.
geometry_msgs/Vector3 acceleration
# Sphere or capsule radius [m] (>= 0).
float64 radius
# Capsule half axis [m]: the axis goes from position - half_axis to
# position + half_axis, fixed in orientation. Zero for a sphere.
geometry_msgs/Vector3 half_axis
# Distance estimation error [m] (>= 0), added to the radius by the barrier.
# Negative means the default of the controller (thor.cbf.distance_error).
float64 delta_h -1.0
