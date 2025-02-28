# Copied from run_soft_hand to try out monitoring etc 
# Date of creation 3.04.2024

import os
import importlib
FOUND_CNOID = importlib.util.find_spec("cnoid") is not None
if FOUND_CNOID: 
    from cnoid.Util import *
    from cnoid.Base import *
    from cnoid.Body import *
    from cnoid.BodyPlugin import *
    from cnoid.GraspPlugin import *
    from cnoid.BinPicking import *

import time
import datetime
import numpy as np
np.set_printoptions(suppress=True)
import os
from bpbot.binpicking import *
from bpbot.config import BinConfig
from bpbot.robotcon.nxt.nxtrobot_client import NxtRobot
from bpbot.device import DynPickClient, DynPickControl

# from bpbot.motion import PickAndPlaceActor, PickSoftHandActor
# from bpbot.robotcon.nxt.nxtrobot_client import NxtRobot
#Following library files copied from runseppick closeloop to run the robot

nxt = NxtRobot(host='[::]:15005')

import timeit
start = timeit.default_timer()


# ---------------------- get config info -------------------------

# root_dir = os.path.realpath(os.path.join(os.path.realpath(__file__), "../../"))
root_dir = "/home/hlab/bpbot"
config_path = os.path.join(root_dir, "config/config_ikura.yaml")
print(f"Root directory: {root_dir} ")
print(f"Config path   : {config_path} ")

img_path = os.path.join(root_dir, "data/depth/depth.png")
crop_path = os.path.join(root_dir, "data/depth/cropped.png")
mf_path = os.path.join(root_dir, "data/motion/motion.dat")
mf_path_2 = os.path.join(root_dir, "data/motion/motion_rozdemo.dat")
draw_path = os.path.join(root_dir, "data/depth/result.png")

cfg = BinConfig(config_path=config_path)
cfgdata = cfg.data
camera_mode = cfgdata["exp"]["camera_mode"]
exp_mode = cfgdata["exp"]["mode"]
mf_mode = cfgdata["exp"]["motionfile_mode"]
lr_arm = cfgdata["exp"]["lr_arm"]
log_mode = cfgdata["exp"]["log_mode"]
#PLAY from run_picksep_closeloop file
#PLAY = cfgdata["exp"]["play_mode"]

# ---------------------- get depth img -------------------------
if camera_mode:
    point_array = capture_pc()
    print("Captured point cloud ... ")
    img, img_blur = pc2depth(point_array, cfgdata)
    cv2.imwrite(img_path, img_blur)
    crop = crop_roi(img_blur, cfgdata, bounding_size=0)
    cv2.imwrite(crop_path, crop)         #may 10th prashant
    
# pcd = o3d.io.read_point_cloud("./data/test/out.ply")
# point_array = pcd.points
"""
# ---------------------- compute grasps -------------------------

grasps = detect_grasp(n_grasp=10, 
                            img_path=crop_path, 
                            g_params=cfgdata['graspability'],
                            h_params=cfgdata["hand"][lr_arm])

if grasps is None:
    raise SystemError("Grasp detection failed! ")

best_grasp = grasps[0]
print("Grasp | Pixel: (%d,%d,%.1f)" % (*best_grasp,)) 
img_grasp = draw_grasp(grasps, crop_path, cfgdata["hand"][lr_arm], top_only=True)
cv2.imwrite(draw_path, img_grasp)

if camera_mode:
    obj_pose, eef_pose = transform_image_to_robot(best_grasp, point_array, cfgdata, arm=lr_arm)
    print("Grasp | Object 3D location: (%.3f,%.3f,%.3f)" % (*obj_pose,))
    print("Grasp | Robot EEF pose (%.3f,%.3f,%.3f,%.1f,%.1f,%.1f)" % (*eef_pose,)) 

    print("Geenrate motion file .. ")
    
    arm = lr_arm[0].upper()
    # actor = PickAndPlaceActor(mf_path, arm)
    actor = PickSoftHandActor(mf_path, cfgdata, arm)
    # rigid true: cold water, false -> hot water 
    actor.get_action(eef_pose, rigid=True) 

"""
"""
# Flatten the image array and get the indices of the top 5 points with highest intensity
top_intensity_indices = np.argsort(crop.ravel())[-5:]

# Convert the indices to 2D coordinates
top_intensity_points = np.unravel_index(top_intensity_indices, crop.shape)

# Print the coordinates of the top 5 points with highest intensity
print("Top 5 points with highest intensity:")
for point in zip(*top_intensity_points):
    print(point)

# Draw yellow circles over the top 5 points (optional)
image_bgr = cv2.cvtColor(crop, cv2.COLOR_GRAY2BGR)
for y, x in zip(*top_intensity_points):
    cv2.circle(image_bgr, (x, y), 5, (0, 255, 255), -1)  # Yellow color

# Display the image with the circles
cv2.imshow('Top 5 Points with Highest Intensity', image_bgr)"""

max_intensity_point_index = np.argmax(crop)

# Convert the index to 2D coordinates
# max_intensity_point = np.unravel_index(max_intensity_point_index, crop.shape)
pp = np.unravel_index(max_intensity_point_index, crop.shape)

max_intensity_point = np.array([pp[1], pp[0]])

# Print the coordinates of the point with the highest intensity
print("Point with highest intensity:", max_intensity_point)

# Draw a yellow circle over the point of highest intensity (optional)
image_bgr = cv2.cvtColor(crop, cv2.COLOR_GRAY2BGR)
cv2.circle(image_bgr, max_intensity_point[::1], 5, (0, 255, 255), -1)
#best_grasp=[0,0,0]

cv2.imshow('Top 5 Points with Highest Intensity', image_bgr)
cv2.waitKey()
cv2.destroyAllWindows()
# u, v = max_intensity_point
# u += cfgdata["main"]["area"]["left"]
# v += cfgdata["main"]["area"]["top"]
# u = int(u)
# v = int(v)

# point_mat = np.reshape(point_array, (cfgdata["height"],cfgdata["width"],3))
# p_c = point_mat[v,u]

# # ranformation
# g_rc = np.loadtxt(cfgdata["calibmat_path"])
# p_tcp = np.dot(g_rc, [*p_c, 1])  # unit: m 
# p_tcp = p_tcp[:3]
# print(p_tcp)
# transform_camera_to_robot()

if camera_mode:
    obj_pose = transform_image_to_robot(max_intensity_point, point_array, cfgdata, arm=lr_arm)
    print("Grasp | Object 3D location: (%.3f,%.3f,%.3f)" % (*obj_pose,))
    eef_pose = list(obj_pose)+[0,-90,90]
    print("Grasp | Robot EEF pose (%.3f,%.3f,%.3f,%.1f,%.1f,%.1f)" % (*eef_pose,)) 

# pg=[eef_pose[0]+0.1,eef_pose[1]+0.14,eef_pose[2]+0.5,eef_pose[3],eef_pose[4],eef_pose[5]]
# eef_pose= [*p_tcp[:2], 0.4, 0,-90,90]
# print(eef_pose)
eef_pose_pre=[eef_pose[0],eef_pose[1],0.200,eef_pose[3],eef_pose[4],eef_pose[5]]
eef_pose[2]=eef_pose[2]+ 0.15
print("eeeefffff",eef_pose)

"""
arm="left"
success = plan_move(arm,[eef_pose],[5])
motion_seq = get_motion()
clear_motion()

nxt.playMotion(motion_seq)
nxt.setInitial(arm='all', tm=3)
#type(best_grasp)
"""