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

# NX-ROS Series Package
from nxr_001_ftcg_client import controllerIcraClient

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
F0_reset = np.zeros(6)


controllerClient = controllerIcraClient()
while not controllerClient.getConnectionStatus():
        controllerClient.connect()

def gripperPickingMotion():
    print("OK Here")
    controllerClient.controlGripper("close_gripper")
    controllerClient.controlGripper("bothServos_up")
    controllerClient.controlGripper("bothServos_up")
    controllerClient.controlGripper("bothServos_up")
    controllerClient.controlGripper("bothServos_up")

def gripperPlaceMotion():
    controllerClient.controlGripper("bothServos_down")
    controllerClient.controlGripper("bothServos_down")

def gripperReleaseMotion():
    controllerClient.controlGripper("open_gripper")    

#controlResponse = controllerClient.controlGripper("close_gripper")
        #print(controlResponse)
#controlResponse = controllerClient.controlGripper("bothServos_up", 720, 720)



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



#controllerClient.controlGripper("close_gripper")
#time.sleep(2)
    
#i = 1
#test_loop = 1
degree = 634
degree_2=600
#degree_3=162
#degree_3=180
degree_3=185

controllerClient.setServoSpeed(30)
"""

#------------------------Dropping the same weight of ikura------------------#

# This section will ensure that even if the ikur apickup isn't uniform,
# the weight dropping is precise as it'll a
# djust accordingly
"""
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

    time.sleep(2) """
"""
h=0
for x in range(1,7):
#---------- Execute on the robot-----------#
    
    arm="right"
    p1=[0.560,-0.251,0.350,-90.0,-90.0,110.0]        #Leftmost Ikura bowl
    #p2=[0.560,-0.251,0.230,-90.0,-90.0,105.0]
    p2=[0.560,-0.221,0.233-h,-90.0,-90.0,110.0]
    p2_2=[0.560,-0.271,0.233-h,-90.0,-90.0,110.0]
    #p3=[0.396,0.004,0.350,-90.0,-90.0,90.0]         #Center target bowl
    #p4=[0.396,0.004,0.275,-90.0,-90.0,90.0] 
    p3=[0.450,0.004,0.350,-90.0,-90.0,110.0]         #Center target bowl
    p4=[0.450,0.004,0.270,-90.0,-90.0,110.0]
    p7=[0.450,-0.045,0.270,-90.0,-90.0,105.0]        #Zig zag
    p8=[0.450,0.035,0.270,-90.0,-90.0,105.0]        #Zig zag
    #p9=[0.396,0.004,0.250,0.0,-90.0,90.0]
    
    
    if x%2 !=0:

        success = plan_move(arm,[p1,p2],[5,5])
        motion_seq = get_motion()
        clear_motion()
        nxt.playMotion(motion_seq)
    else:
        success = plan_move(arm,[p1,p2_2],[5,5])
        motion_seq = get_motion()
        clear_motion()
        nxt.playMotion(motion_seq) 
        h=h+0.005

    print("value of h is ",h)
    

    #gripperPickingMotion()
    controllerClient.controlGripper("close_gripper")
    time.sleep(1)
    controllerClient.controlGripper("bothServos_up", degree, degree)
    time.sleep(1)

    #success = plan_move(arm,[p1,p2],[5,5])
    #motion_seq = get_motion()
    #clear_motion()
    #nxt.playMotion(motion_seq)

    #gripperPlaceMotion()
    #gripperPlaceMotion()


    ###############Ikura drop starts##############
    #drop_ikura()



    success = plan_move(arm,[p1,p3,p8],[3,3,3])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)

    #gripperReleaseMotion()

    #controllerClient.sendOffSignal()
    #controllerClient.controlGripper("open_gripper")
    #time.sleep(1)
    controllerClient.setServoSpeed(10)
    controllerClient.controlGripper("bothServos_down", degree_3,degree_3)       #Zig zag motion


    success = plan_move(arm,[p7],[0.5])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)

    controllerClient.setServoSpeed(10)
    controllerClient.controlGripper("bothServos_down", degree_3,degree_3)

    success = plan_move(arm,[p8],[0.5])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)

    controllerClient.setServoSpeed(10)
    controllerClient.controlGripper("bothServos_down", degree_3,degree_3)

    success = plan_move(arm,[p7],[0.5])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)

    controllerClient.setServoSpeed(10)
    controllerClient.controlGripper("bothServos_down", degree_3,degree_3)
    time.sleep(1)
    controllerClient.setServoSpeed(30)
    controllerClient.controlGripper("bothServos_up", (4*degree_3)-degree-10,(4*degree_3)-degree-10)




    success = plan_move(arm,[p3],[1])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)
    
    """
"""
    nxt.openHandToolLft()
    arm="left"
    p5=[0.461,0.227,0.250,-90.0,-90.0,90.0]     #Rightmost gari bowl
    p6=[0.461,0.227,0.180,-90.0,-90.0,90.0]

    success = plan_move(arm,[p5,p6],[3,3])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)



    nxt.closeHandToolLft()
    success = plan_move(arm,[p5,p3],[3,3])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)

    nxt.openHandToolLft()
    print("----- Gari has been placed -------") 
    """
    #controllerClient.controlGripper("bothServos_up", degree, degree)
    #time.sleep(55)

    #controllerClient.controlGripper("bothServos_down", degree_2, degree_2)
    #time.sleep(1)
"""

    
"""
"""
#---------------------- Turned around--------------------
h=0
for x in range(1,5):
#---------- Execute on the robot-----------#
    
    arm="right"
    p1=[0.558,-0.251,0.350,-90.0,-90.0,290.0]        #Leftmost Ikura bowl
    #p2=[0.560,-0.251,0.230,-90.0,-90.0,105.0]
    p2=[0.558,-0.221,0.234-h,-90.0,-90.0,290.0]
    p2_2=[0.558,-0.271,0.234-h,-90.0,-90.0,290.0]
    #p3=[0.396,0.004,0.350,-90.0,-90.0,90.0]         #Center target bowl
    #p4=[0.396,0.004,0.275,-90.0,-90.0,90.0] 
    p3=[0.450,0.004,0.350,-90.0,-90.0,110.0]         #Center target bowl
    p4=[0.450,0.004,0.270,-90.0,-90.0,110.0]
    p7=[0.450,-0.045,0.270,-90.0,-90.0,105.0]        #Zig zag
    p8=[0.450,0.035,0.270,-90.0,-90.0,105.0]        #Zig zag
    #p9=[0.396,0.004,0.250,0.0,-90.0,90.0]
    
    
    if x%2 !=0:

        success = plan_move(arm,[p1,p2],[5,5])
        motion_seq = get_motion()
        clear_motion()
        nxt.playMotion(motion_seq)
    else:
        success = plan_move(arm,[p1,p2_2],[5,5])
        motion_seq = get_motion()
        clear_motion()
        nxt.playMotion(motion_seq) 
        h=h+0.01

    print("value of h is ",h)
    

    #gripperPickingMotion()
    controllerClient.controlGripper("close_gripper")
    time.sleep(1)
    controllerClient.controlGripper("bothServos_up", degree, degree)
    time.sleep(1)

    #success = plan_move(arm,[p1,p2],[5,5])
    #motion_seq = get_motion()
    #clear_motion()
    #nxt.playMotion(motion_seq)

    #gripperPlaceMotion()
    #gripperPlaceMotion()


    ###############Ikura drop starts##############
    #drop_ikura()



    success = plan_move(arm,[p1,p3,p8],[3,3,3])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)

    #gripperReleaseMotion()

    #controllerClient.sendOffSignal()
    #controllerClient.controlGripper("open_gripper")
    #time.sleep(1)
    controllerClient.setServoSpeed(10)
    controllerClient.controlGripper("bothServos_down", degree_3,degree_3)       #Zig zag motion


    success = plan_move(arm,[p7],[0.5])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)

    controllerClient.setServoSpeed(10)
    controllerClient.controlGripper("bothServos_down", degree_3,degree_3)

    success = plan_move(arm,[p8],[0.5])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)

    controllerClient.setServoSpeed(10)
    controllerClient.controlGripper("bothServos_down", degree_3,degree_3)

    success = plan_move(arm,[p7],[0.5])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)

    controllerClient.setServoSpeed(10)
    controllerClient.controlGripper("bothServos_down", degree_3,degree_3)
    time.sleep(1)
    controllerClient.setServoSpeed(30)
    controllerClient.controlGripper("bothServos_up", (4*degree_3)-degree-10,(4*degree_3)-degree-10)




    success = plan_move(arm,[p3],[1])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)
    """
"""
    nxt.openHandToolLft()
    arm="left"
    p5=[0.461,0.227,0.250,-90.0,-90.0,90.0]     #Rightmost gari bowl
    p6=[0.461,0.227,0.180,-90.0,-90.0,90.0]

    success = plan_move(arm,[p5,p6],[3,3])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)



    nxt.closeHandToolLft()
    success = plan_move(arm,[p5,p3],[3,3])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)

    nxt.openHandToolLft()
    print("----- Gari has been placed -------") 
    """
    #controllerClient.controlGripper("bothServos_up", degree, degree)
    #time.sleep(55)

    #controllerClient.controlGripper("bothServos_down", degree_2, degree_2)
    #time.sleep(1)


#------------ Different container------
for x in range(0,4):
#---------- Execute on the robot-----------#
    
    arm="right"
    p1=[0.558,-0.251,0.350,-90.0,-90.0,110.0]        #Leftmost Ikura bowl
    #p2=[0.558-(x*0.015),-0.192,0.225,-90.0,-90.0,110.0]
    #p2_2=[0.558-(x*0.015),-0.291,0.225,-90.0,-90.0,110.0]
    #p2_3=[0.558-(x*0.015),-0.215,0.225,-90.0,-90.0,110.0]
    p2=[0.558-(x*0.02),-0.192,0.225,-90.0,-90.0,110.0]
    p2_2=[0.558-(x*0.02),-0.291,0.225,-90.0,-90.0,110.0]
    p3=[0.450,0.004,0.350,-90.0,-90.0,110.0]         #Center target bowl
    p4=[0.450,0.004,0.270,-90.0,-90.0,110.0]
    p7=[0.450,-0.045,0.270,-90.0,-90.0,105.0]        #Zig zag
    p8=[0.450,0.035,0.270,-90.0,-90.0,105.0]        #Zig zag
    #p9=[0.396,0.004,0.250,0.0,-90.0,90.0]
    
    
    def ikura_pick(a,b):

        success = plan_move(arm,[a,b],[2,2])
        motion_seq = get_motion()
        clear_motion()
        nxt.playMotion(motion_seq)


    def zigzag():
        controllerClient.setServoSpeed(10)
        controllerClient.controlGripper("bothServos_down", degree_3,degree_3)       #Zig zag motion


        success = plan_move(arm,[p7],[0.5])
        motion_seq = get_motion()
        clear_motion()
        nxt.playMotion(motion_seq)

        controllerClient.setServoSpeed(10)
        controllerClient.controlGripper("bothServos_down", degree_3,degree_3)

        success = plan_move(arm,[p8],[0.5])
        motion_seq = get_motion()
        clear_motion()
        nxt.playMotion(motion_seq)

        controllerClient.setServoSpeed(10)
        controllerClient.controlGripper("bothServos_down", degree_3,degree_3)

        success = plan_move(arm,[p7],[0.5])
        motion_seq = get_motion()
        clear_motion()
        nxt.playMotion(motion_seq)

        controllerClient.setServoSpeed(10)
        controllerClient.controlGripper("bothServos_down", degree_3,degree_3)
        time.sleep(1)
        controllerClient.setServoSpeed(30)
        controllerClient.controlGripper("bothServos_up", (4*degree_3)-degree-10,(4*degree_3)-degree-10)

    def gari_pick():
        nxt.openHandToolLft()
        arm="left"
        p5=[0.461,0.227,0.250,-90.0,-90.0,90.0]     #Rightmost gari bowl
        p6=[0.461-(x*0.01),0.227,0.125,-90.0,-90.0,90.0]

        success = plan_move(arm,[p5,p6],[2,2])
        motion_seq = get_motion()
        clear_motion()
        nxt.playMotion(motion_seq)



        nxt.closeHandToolLft()
        success = plan_move(arm,[p5,p4],[2,2])
        motion_seq = get_motion()
        clear_motion()
        nxt.playMotion(motion_seq)

        nxt.openHandToolLft()
        print("----- Gari has been placed -------")


    ikura_pick(p1,p2)

    #controllerClient.controlGripper("close_gripper")
    #time.sleep(1)
    controllerClient.controlGripper("bothServos_up", degree, degree)
    time.sleep(1)

    success = plan_move(arm,[p1,p3,p8],[2,2,2])             #Move to the ikura bowl
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)

    zigzag()

    success = plan_move(arm,[p3],[1])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)

    gari_pick()

    #------Cycle 2-------#
    ikura_pick(p1,p2_2)
    
    controllerClient.controlGripper("bothServos_up", degree, degree)
    time.sleep(1)

    success = plan_move(arm,[p1,p3,p8],[2,2,2])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)

    zigzag()

    success = plan_move(arm,[p3],[1])
    motion_seq = get_motion()
    clear_motion()
    nxt.playMotion(motion_seq)
    gari_pick()
    if(x>2):
        #--------Cycle 3-----------
        for z in range(1,3):
            p2_3=[0.558-(z*0.015),-0.215,0.225,-90.0,-90.0,110.0]
            ikura_pick(p1,p2_3)
    
            controllerClient.controlGripper("bothServos_up", degree, degree)
            time.sleep(1)

            success = plan_move(arm,[p1,p3,p8],[2,2,2])
            motion_seq = get_motion()
            clear_motion()
            nxt.playMotion(motion_seq)

            zigzag()

            success = plan_move(arm,[p3],[1])
            motion_seq = get_motion()
            clear_motion()
            nxt.playMotion(motion_seq)
            gari_pick()
