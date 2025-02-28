# Reference from run_soft_hand  
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
config_path = os.path.join(root_dir, "config/config_soft_hand.yaml")
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
dc = DynPickClient()
fs = DynPickControl()
sensitivity = np.array(cfgdata["force_sensor"]["sensitivity"])
#F0_reset = np.zeros(6)

# ---------------------- get depth img -------------------------
"""if camera_mode:
    point_array = capture_pc()
    print("Captured point cloud ... ")
    img, img_blur = pc2depth(point_array, cfgdata)
    cv2.imwrite(img_path, img_blur)
    crop = crop_roi(img_blur, cfgdata, bounding_size=0)
    cv2.imwrite(crop_path, crop)
    
# pcd = o3d.io.read_point_cloud("./data/test/out.ply")
# point_array = pcd.points

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
    actor.get_action(eef_pose, rigid=True)  """




#------------------------Dropping the same weight of ikura------------------#

# This section will ensure that even if the ikur apickup isn't uniform,
# the weight dropping is precise as it'll adjust accordingly

def drop_ikura():

    print("Starting the dropping process")
    f_t=dc.get()                            #Getting the weight value right after we reach the rop of the bowl
    F_pre=f_t[2]
    print(F_pre)

    print("------ Weight_pre------")
    F_post=F_pre-50
    print("------ Weight_post------")
    print(F_post)

    while(F_pre>F_post):
    #arduino command to drop one bucket
        print("Dropping a bucket") 
        time.sleep(0.5)   
        f_t=dc.get()                        #Do we need to add a delay here to make sure the next ft value is after the motor is rotated??
        F_pre=f_t[2]
        print("current weight is "+ str(F_pre))

    print("Final weight is " + str(F_pre))
    print("Ikura dropping complete") 

    time.sleep(2)
#drop_ikura()

############# Execute on the robot############
arm="right"
p1=[0.483,-0.251,0.250,-90.0,-90.0,90.0]        #Leftmost Ikura bowl
p2=[0.483,-0.251,0.180,-90.0,-90.0,90.0]
p3=[0.396,0.004,0.250,-90.0,-90.0,90.0]         #Center target bowl
p4=[0.396,0.004,0.250,0.0,-90.0,90.0]

success = plan_move(arm,[p1,p2,p1,p3,p4],[5,5,5,5,5])
motion_seq = get_motion()
clear_motion()
nxt.playMotion(motion_seq)


###############Ikura drop starts##############

#drop_ikura()


arm="left"
p5=[0.461,0.227,0.250,-90.0,-90.0,90.0]     #Rightmost gari bowl
p6=[0.461,0.227,0.180,-90.0,-90.0,90.0]

success = plan_move(arm,[p5,p6,p5],[5,5,5])
motion_seq = get_motion()
clear_motion()
nxt.playMotion(motion_seq)
print("----- Gari has been placed -------")

# ---------------------- execute on robot -------------------------
#-----------------------Pickup and move Ikura----------------------
if FOUND_CNOID: 
    success = load_motionfile(mf_path)
    print(f"Planning succeeded? {success}")

    motion_seq = get_motion()
    print(motion_seq.shape)
    print(motion_seq)
    
    if success: 
        print(f"[*] Success! Total {motion_seq.shape[0]} motion sequences! ")
        nxt = NxtRobot(host='[::]:15005')

        nxt.playMotion(motion_seq)
    
end = timeit.default_timer()
print("[*] Time: {:.2f}s".format(end - start))