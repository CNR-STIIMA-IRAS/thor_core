import time
import numpy as np
import pandas as pd

import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import meshcat.geometry as g

from meshcat.visualizer import Visualizer



urdf_path = '/home/galileo/projects/thor_ws/src/thor_core/thor_math/ur10/ur10_with_intermediates.urdf'
mesh_dir = '/home/galileo/projects/thor_ws/src/thor_core/thor_math/ur10/ur_description/meshes'  # Folder that contains 'ur_description' package
traj_path = '/home/galileo/projects/thor_ws/src/thor_core/thor_math/trajectory_log.csv'

# ---- LOAD TRAJECTORY LOG ----
df = pd.read_csv(traj_path)
print("Columns in df:", df.columns)
q_cols = [c for c in df.columns if c.startswith('q')]
ph_cols = ['ph_x', 'ph_y', 'ph_z']
h_cols = ['h']



# --- Load robot model (skip visuals, we don’t use them) ---
model, _, _ = pin.buildModelsFromUrdf(urdf_path)
viz = MeshcatVisualizer(model)
viz.viewer = Visualizer()  # Directly create Meshcat viewer
viz.viewer.open()

# --- Prepare sphere geometry ---
sphere = g.Sphere(0.03)  # size of the sphere
material = g.MeshPhongMaterial(color=0xFFFF00)  # yellow
for i, f in enumerate(model.frames):
    
    print(f"Frame {i}: {f.name}, {f.type}")
# --- Create one sphere per frame (excluding OP_FRAME) ---

frame_ids = [
    i for i, f in enumerate(model.frames)
    if f.type == pin.FrameType.JOINT or "intermediate" in f.name
      # skip world/base if needed
]
for fid in frame_ids:
    name = f"sphere/frame_{fid}_{model.frames[fid].name}"
    viz.viewer[name].set_object(sphere, material)
    print(f"Added sphere for frame {fid}: {model.frames[fid].name}")

print("QUI")
viz.viewer['/human'].set_object(
    g.Sphere(0.05),
    g.MeshLambertMaterial(color=0xff0000)
)
print("Added human marker")

# --- Animate through the trajectory ---
data = model.createData()
while True:
    for idx, row in df.iterrows():
        q = np.array(row[q_cols])
        
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)

        for fid in frame_ids:
            # print(f"Frame {fid}: {model.frames[fid].name}")
            placement = data.oMf[fid].homogeneous
            # pos = data.oMf[fid].translation
            # print(f"Position for frame {fid}: {pos}")
            name = f"sphere/frame_{fid}_{model.frames[fid].name}"
            viz.viewer[name].set_transform(placement)

        # Set human marker position
        ph = row[ph_cols].values.astype(float)
        T = pin.SE3(np.eye(3), ph)
        viz.viewer['/human'].set_transform(T.homogeneous)
        time.sleep(0.001)  # adjust speed if needed
    print("End of trajectory, restarting...")
    time.sleep(1)  # wait before restarting the trajectory