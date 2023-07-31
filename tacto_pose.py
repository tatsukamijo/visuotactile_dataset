import time
import logging

import hydra
import numpy as np
import pybullet as p

import pybulletX as px

import tacto

from sawyer_gripper import SawyerGripper
import math
import cv2

log = logging.getLogger(__name__)
 
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

# Load the config YAML file from examples/conf/grasp.yaml
@hydra.main(config_path="conf", config_name="mmd")
def main(cfg):
    # Initialize digits
    digits = tacto.Sensor(**cfg.tacto)

    # Initialize World
    log.info("Initializing world")
    px.init()

    # p.resetDebugVisualizerCamera(**cfg.pybullet_camera)
    p.resetDebugVisualizerCamera(**cfg.pybullet_camera_from_side)

    vmat = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0.5,0,0.08], distance=0.3, yaw=90, pitch=-20, roll=0, upAxisIndex=2)
    pmat = p.computeProjectionMatrixFOV(fov=60, aspect=(128 / 128), nearVal=0.01, farVal=5) 
    img = p.getCameraImage(128, 128, viewMatrix=vmat, projectionMatrix=pmat, renderer=p.ER_BULLET_HARDWARE_OPENGL)

    robot = SawyerGripper(**cfg.sawyer_gripper)

    # [21, 24]
    digits.add_camera(robot.id, robot.digit_links)

    # Add object to pybullet and digit simulator
    obj = px.Body(**cfg.object)
    digits.add_body(obj)

    np.set_printoptions(suppress=True)

    # run p.stepSimulation in another thread
    t = px.utils.SimulationThread(real_time_factor=1.0)
    t.start()

    robot.reset()

    panel = px.gui.RobotControlPanel(robot)
    panel.start()
    time.sleep(3)
    count = 0
    while True:
        color, depth = digits.render()
        digits.updateGUI(color, depth)
        width, height, rgbImg, _, _ = p.getCameraImage(256, 256, viewMatrix=vmat, projectionMatrix=pmat, renderer=p.ER_BULLET_HARDWARE_OPENGL)
        pose = obj.get_base_pose()[1]
        pose_euler = euler_from_quaternion(pose[0], pose[1], pose[2], pose[3])
        roll = math.degrees(pose_euler[0])
        pitch = math.degrees(pose_euler[1])
        yaw = math.degrees(pose_euler[2])
        if abs(pitch) > 60:
            break
        print(f"pitch: {pitch}")
        # Set your desired result path and save the images
        # cv2.imwrite(f'./results_90/front/{count}_front_{pitch}.png', np.reshape(rgbImg, (height, width, 4))[:, :, :3])
        # cv2.imwrite(f'./results_90/tactile/{count}_tactile_left_{pitch}.png', cv2.cvtColor(color[0], cv2.COLOR_RGB2BGR))
        count += 1
        time.sleep(0.1)


if __name__ == "__main__":
    main()
