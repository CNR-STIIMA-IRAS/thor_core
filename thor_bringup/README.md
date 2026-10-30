# thor_bringup

UR10 with ros2_control mock hardware and the THOR controller: launch file,
integration test and an RViz demo with a simulated person.

## Demo

```bash
ros2 launch thor_bringup ur10_thor.launch.py                 # ThorTrajectoryController
ros2 launch thor_bringup ur10_thor.launch.py chained:=true   # JTC -> ThorChainableController
```

In RViz:

- **The person** (blue capsule): give it a goal with **2D Goal Pose** in the
  toolbar, as with nav2. It walks there at 1 m/s and is sent to the
  controller as four obstacles (body, head, hands).
- **The robot** starts a pick-and-place loop. The orange ball at the tool is
  an interactive marker: drag it and release to send the tool there (inverse
  kinematics with the tool pointing down). Right click on it for *Go here*,
  *Start loop*, *Stop loop*.
- **The robot spheres** are green when far from the person and turn red
  near the safety margin. The text above the robot shows the time scaling,
  the barrier `h` and the actual distance.

Walk the person into the path of the arm: the robot moves around the person
or slows down, and resumes the trajectory when the person leaves.

Arguments: `rviz:=false`, `demo:=false` (only robot and controllers),
`chained:=true`, `plot:=true` (PlotJuggler with `config/thor_demo.xml`: barrier,
projected velocities, separation distance with the thresholds, scaling, status
and solve time from `/thor_controller/thor_state`).

| Node | Role |
| --- | --- |
| `human_sim.py` | Person walking to `/goal_pose`; publishes `/thor_controller/obstacles` and markers. |
| `robot_goal.py` | Pick-and-place loop and interactive marker; goals on `/thor_controller/follow_joint_trajectory`. |
| `thor_markers.py` | Robot spheres colored by distance and state text. |

## Configuration

- [urdf/ur10.urdf.xacro](urdf/ur10.urdf.xacro): UR10 (kinematics of
  `thor_math/test/data/ur10`, collision meshes also as visuals) on a 0.75 m
  pedestal, with `mock_components/GenericSystem`.
- [config/ur10_controllers.yaml](config/ur10_controllers.yaml):
  `ThorTrajectoryController` at 500 Hz, ten spheres covering the arm.
- [config/ur10_controllers_chained.yaml](config/ur10_controllers_chained.yaml):
  `joint_trajectory_controller` chained to `ThorChainableController`, with
  the same topics.

## Integration test

```bash
colcon test --packages-select thor_bringup --event-handlers console_direct+
launch_test src/thor_core/thor_bringup/test/test_ur10_thor.launch.py chained:=true
```

[test/test_ur10_thor.launch.py](test/test_ur10_thor.launch.py) starts the
launch file without RViz, for both controllers, and checks that:

1. a goal is reached at nominal speed;
2. with a person standing on the path of the tool for 3 s, the goal is still
   reached, the actual distance never goes below the safety margin, and the
   robot slows down or moves around the person.
