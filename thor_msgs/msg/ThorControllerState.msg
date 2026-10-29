# State of a THOR controller, published every cycle.

uint8 STATUS_OK = 0
# The QP had no solution: the controller brakes within the acceleration
# limits and the trajectory time stops.
uint8 STATUS_BRAKING = 1

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

# The robot-obstacle pair with the lowest barrier over the horizon; zero
# with the CBF disabled. See thor_math CbfDiagnostics.
float64 h_now
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
