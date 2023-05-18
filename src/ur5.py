#!/usr/bin/env python

import zmqRemoteApi
import numpy as np
from time import sleep
import pandas as pd
from utils import rotation_3d_deg
from utils.simulation import reset_arm

client = zmqRemoteApi.RemoteAPIClient()

sim = client.getObject("sim")

UR5_sim_obj_id = "/UR5"
base_id = sim.getObject(UR5_sim_obj_id)
no_joints = 6
joint_sim_names = ["" for _ in range(6)]

joint_sim_names[0] = f"{UR5_sim_obj_id}/joint"

for i in range(1, len(joint_sim_names)):
    link_joint_str = "/link/joint" * i
    joint_sim_names[i] = f"{UR5_sim_obj_id}{link_joint_str}"

connection_name = f"{UR5_sim_obj_id}/connection"

joint_ids = [sim.getObject(name) for name in joint_sim_names]
connection_id = sim.getObject(connection_name)


real_final_coords: np.ndarray = np.array([])

try:
    sim.startSimulation()
    reset_arm(sim, joint_ids)
    sleep(1)
    real_final_coords = sim.getObjectPosition(joint_ids[-1], sim.handle_world)
except Exception as e:
    sim.stopSimulation()
    raise e
finally:
    sim.stopSimulation()
