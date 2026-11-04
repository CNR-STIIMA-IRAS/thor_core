# State of a THOR controller, published every cycle.

uint8 STATUS_OK = 0
# The QP had no solution: the controller brakes within the acceleration
# limits and the trajectory time stops.
uint8 STATUS_BRAKING = 1
# Protective stop (thor.protective_stop): an obstacle closer than the stop
# distance; the robot brakes and holds still until the distance is restored.
uint8 STATUS_PROTECTIVE_STOP = 2
# Back into the tube (thor.limits.tube) after a stop: the trajectory time is
# stopped while the robot returns to the reference.
uint8 STATUS_RECOVERY = 3

std_msgs/Header header
uint8 status

# Time scaling applied to the trajectory (1 = nominal speed).
float64 scaling
# Wall time of the MPC solve [s].
float64 solve_time

# Number of obstacles used in this cycle, and whether the last obstacle
# message is older than thor.obstacles.timeout.
uint32 n_obstacles
bool obstacles_stale

# Distance between the surfaces of the robot spheres and of the closest
# obstacle at the current state [m]; infinite without obstacles.
float64 min_distance
# min_distance reduced by the perception errors (distance_error and
# sigma_bound * sigma of the position): the distance of the protective stop.
float64 guard_distance
# Largest distance of a joint from the reference now (0 without the tube).
float64 tube_error

# Barrier on the current state [m]: the actual safety margin.
float64 h_now
# The robot-obstacle pair with the lowest barrier over the horizon; zero
# with the CBF disabled. See thor_math CbfDiagnostics: its distance and
# velocities may be on a predicted state of the nominal trajectory.
float64 h_min
float64 distance
float64 v_robot
float64 v_obstacle
string robot_frame
int32 obstacle_index

# Joint states: the command written to the robot and the reference of the
# nominal trajectory at the current trajectory time.
string[] joint_names
float64[] command_positions
float64[] command_velocities
float64[] command_accelerations
float64[] reference_positions
