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

from bpbot.binpicking import *
from bpbot.config import BinConfig
# from bpbot.motion import PickAndPlaceActor, PickSoftHandActor
# from bpbot.robotcon.nxt.nxtrobot_client import NxtRobot
#Following library files copied from runseppick closeloop to run the robot
import time
import datetime
import numpy as np
np.set_printoptions(suppress=True)
from bpbot.binpicking import *
from bpbot.config import BinConfig
from bpbot.robotcon.nxt.nxtrobot_client import NxtRobot
#from bpbot.device import DynPickClient, DynPickControl
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
img_grasp = draw_grasp(grasps, crop_path, cfgdata["hand"][lr_arm], top_only=False)
cv2.imwrite(draw_path, img_grasp)

if camera_mode:
    obj_pose, eef_pose = transform_image_to_robot(best_grasp, point_array, cfgdata, arm=lr_arm)
    print("Grasp | Object 3D location: (%.3f,%.3f,%.3f)" % (*obj_pose,))
    print("Grasp | Robot EEF pose (%.3f,%.3f,%.3f,%.1f,%.1f,%.1f)" % (*eef_pose,)) 

    #----grasp_2 location
    obj_pose_2, eef_pose_2 = transform_image_to_robot(grasps[1], point_array, cfgdata, arm=lr_arm)
    print("Grasp | Object 3D location: (%.3f,%.3f,%.3f)" % (*obj_pose_2,))
    print("Grasp | Robot EEF pose (%.3f,%.3f,%.3f,%.1f,%.1f,%.1f)" % (*eef_pose_2,))

    #----grasp_3 location
    obj_pose_3, eef_pose_3 = transform_image_to_robot(grasps[2], point_array, cfgdata, arm=lr_arm)
    print("Grasp | Object 3D location: (%.3f,%.3f,%.3f)" % (*obj_pose_3,))
    print("Grasp | Robot EEF pose (%.3f,%.3f,%.3f,%.1f,%.1f,%.1f)" % (*eef_pose_3,))

    #----grasp_4 location
    obj_pose_4, eef_pose_4 = transform_image_to_robot(grasps[3], point_array, cfgdata, arm=lr_arm)
    print("Grasp | Object 3D location: (%.3f,%.3f,%.3f)" % (*obj_pose_4,))
    print("Grasp | Robot EEF pose (%.3f,%.3f,%.3f,%.1f,%.1f,%.1f)" % (*eef_pose_4,))

   

    
    
    
    
    
    #arm = lr_arm[0].upper()        #Changed on 20.4.2024 prashant
   # arm="left" 
    # actor = PickAndPlaceActor(mf_path, arm)
  
print("eef poses",eef_pose,eef_pose_2)
print("object poses",obj_pose,obj_pose_2)
print(best_grasp)

def pick_karage1(a):
    #nxt.openHandToolLft()
    #p1=[eef_pose[0],eef_pose[1],0.280,eef_pose[3],eef_pose[4],eef_pose[5]]
    #p2=[eef_pose[0],eef_pose[1],eef_pose[2]+0.070,eef_pose[3],eef_pose[4],eef_pose[5]]
    p1=[a[0],a[1],0.280,a[3],a[4],a[5]]
    p2=[a[0],a[1],a[2]+0.070,a[3],a[4],a[5]]
   # p3=                                      #final karaage dropping position. To be added laterr

    #p2=[eef_pose[0],eef_pose[1],eef_pose[2],eef_pose[3],eef_pose[4],eef_pose[5]]
    #p2=[eef_pose]
    arm="left"
    print(p1,p2,p1)
    success = plan_move(arm,[p1,p2],[3,3])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)
    nxt.closeHandToolLft()

    #time.sleep(10)
    success = plan_move(arm,[p1],[3])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)

    nxt.openHandToolLft()


#-------Right arm movement-----------#
def pick_karage2(b):
    arm="right"
    nxt.openHandToolRgt()
    #p1=[eef_pose_2[0],eef_pose_2[1],0.280,eef_pose_2[3],eef_pose_2[4],eef_pose_2[5]]
    #p2=[eef_pose_2[0],eef_pose_2[1],eef_pose[2]+0.070,eef_pose_2[3],eef_pose_2[4],eef_pose_2[5]]
    p1=[b[0],b[1],0.280,b[3],b[4],b[5]]
    p2=[b[0],b[1],b[2]+0.070,b[3],b[4],b[5]]

    #p2=[eef_pose[0],eef_pose[1],0.175,eef_pose[3],eef_pose[4],eef_pose[5]]
    #p2=[eef_pose]

    print(p1,p2,p1)
    success = plan_move(arm,[p1,p2],[3,3])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)
    nxt.closeHandToolRgt()

    #time.sleep(10)
    success = plan_move(arm,[p1],[3])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)

    nxt.openHandToolRgt()


#----------- Sending eef poses for the robot movement
pick_karage1(eef_pose)
pick_karage2(eef_pose_2)
pick_karage1(eef_pose_3)
pick_karage2(eef_pose_4)






nxt.setInitial(arm='all', tm=3)