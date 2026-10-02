from curses import version
import numpy as np
import pandas as pd
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import meshcat
import meshcat.geometry as mg
import time
import importlib.metadata



# ---- CONFIGURE YOUR PATHS HERE ----
print("Pinocchio version:", pin.__version__)


print("Meshcat version:", importlib.metadata.version("meshcat"))
urdf_path = '/home/galileo/projects/thor_ws/src/thor_core/thor_math/ur10/ur10.urdf'
mesh_dir = '/home/galileo/projects/thor_ws/src/thor_core/thor_math/ur10/ur_description/meshes'  # Folder that contains 'ur_description' package
traj_path = '/home/galileo/projects/thor_ws/src/thor_core/thor_math/trajectory_log.csv'

# ---- LOAD TRAJECTORY LOG ----
df = pd.read_csv(traj_path)
print("Columns in df:", df.columns)
q_cols = [c for c in df.columns if c.startswith('q')]
ph_cols = ['ph_x', 'ph_y', 'ph_z']
h_cols = ['h']
# ---- LOAD ROBOT MODEL (WITH GEOMETRY!) ----
model, collision_model, visual_model = pin.buildModelsFromUrdf(
    urdf_path)
#     mesh_dir=mesh_dir
# )
print("Number of visual objects loaded:", len(visual_model.geometryObjects))  # Should be > 0

# ---- INITIALIZE MESHCAT VISUALIZER ----
viz = MeshcatVisualizer(model, collision_model, visual_model)
viz.initViewer(open=True)
viz.loadViewerModel()

# ---- ADD HUMAN SPHERE (RED) ----

viz.viewer['/human'].set_object(
    mg.Sphere(0.05),
    mg.MeshLambertMaterial(color=0xff0000)
)
for jid in range(model.njoints):
    print(f"Joint id: {jid}, name: {model.names[jid]}")

joint_id = model.getJointId("wrist_3_joint")  # Get joint ID for end effector
print("Joint ID for end effector:", joint_id)
name = f"/joints/joint_{joint_id}"
viz.viewer[name].set_object(
    mg.Sphere(0.025),  # smaller than human marker
    mg.MeshLambertMaterial(color=0xffff00)  # yellow
)


# ---- MAIN ANIMATION LOOP ----
while True:
    for idx, row in df.iterrows():
        # Set robot configuration
        q = np.zeros(model.nq)
        q[:len(q_cols)] = row[q_cols].values.astype(float)
        viz.display(q)

        pin.forwardKinematics(model, viz.data, q)
        # pin.updateFramePlacements(model, viz.data)
        # pin.updateJointPlacements(model, viz.data)

        # Move yellow spheres to each joint's position
        # for jid in joint_ids:
        pos = viz.data.oMi[joint_id].translation
        T = pin.SE3(np.eye(3), pos)
        viz.viewer[f"/joints/joint_{joint_id}"].set_transform(T.homogeneous)

        # Set human marker position
        ph = row[ph_cols].values.astype(float)
        T = pin.SE3(np.eye(3), ph)
        viz.viewer['/human'].set_transform(T.homogeneous)
        print(f"h: {df.iloc[idx]['h']}")
        print(f"n_c: {df.iloc[idx]['n_c']}")
        print(f"frameId: {df.iloc[idx]['frameId']}")
        print(f"d: {df.iloc[idx]['d']}")
        print(f"vr: {df.iloc[idx]['vr']}")
        print(f"vh: {df.iloc[idx]['vh']}")
        time.sleep(0.002)  # Adjust for speed
    print("Restarting animation from beginning.")
    time.sleep(0.5)
