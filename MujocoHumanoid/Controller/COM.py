import numpy as np

def compute_com(data):

    com_x = data.qpos[0]
    com_z = data.qpos[2]

    com_vx = data.qvel[0]

    return {

        "x": com_x,
        "z": com_z,
        "vx": com_vx
    }