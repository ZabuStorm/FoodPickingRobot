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

import timeit
start = timeit.default_timer() 

# ---------------------- get config info -------------------------

# root_dir = os.path.realpath(os.path.join(os.path.realpath(__file__), "../../"))
root_dir = "/home/hlab/bpbot"
config_path = os.path.join(root_dir, "config/config_soft_hand.yaml")
print(f"Root directory: {root_dir} ")
print(f"Config path   : {config_path} ")

img_path = os.path.join(root_dir, "data/depth/depth_p.png")
crop_path = os.path.join(root_dir, "data/depth/cropped_p.png")
mf_path = os.path.join(root_dir, "data/motion/motion.dat")         # prashant
draw_path = os.path.join(root_dir, "data/depth/result.png")

cfg = BinConfig(config_path=config_path)
cfgdata = cfg.data
camera_mode = cfgdata["exp"]["camera_mode"]
exp_mode = cfgdata["exp"]["mode"]
mf_mode = cfgdata["exp"]["motionfile_mode"]
lr_arm = cfgdata["exp"]["lr_arm"]                              
log_mode = cfgdata["exp"]["log_mode"]

# ---------------------- get depth img -------------------------
if camera_mode:
    point_array = capture_pc()
    print("Captured point cloud ... ")
    img, img_blur = pc2depth(point_array, cfgdata)
    cv2.imwrite(img_path, img_blur)
    crop = crop_roi(img_blur, cfgdata, bounding_size=50)
    cv2.imwrite(crop_path, crop)
   
 #pcd = o3d.io.read_point_cloud("./data/test/out.ply")
 #point_array = pcd.points

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

# ---------------------- execute on robot -------------------------
if FOUND_CNOID and camera_mode: 
    success = load_motionfile(mf_path)
    print(f"Planning succeeded? {success}")
    
end = timeit.default_timer()
print("[*] Time: {:.2f}s".format(end - start))  """

"""if FOUND_CNOID: 
    plan_success = load_motionfile(mf_path)
    print(f"[*] Motion planning succeed? ==> {plan_success}")
    if plan_success.count(True) == len(plan_success):
        nxt = NxtRobot(host='[::]:15005')
        motion_seq = get_motion()
        num_seq = int(len(motion_seq)/20)
        print(f"[*] Total {num_seq} motion sequences! ")
        motion_seq = np.reshape(motion_seq, (num_seq, 20))

        os.system("bash /home/hlab/bpbot/script/start_ft.sh")
        nxt.playMotionFT(motion_seq)
        os.system("bash /home/hlab/bpbot/script/stop_ft.sh")
        from bpbot.device import DynPick
        ft = DynPick()
        ft.plot_file()
        if LOG_ON:
            print("[*] Save results")
            tdatetime = dt.now()
            tstr = tdatetime.strftime('%Y%m%d%H%M%S')
            save_dir = "/home/hlab/Desktop/exp" 
            tstr_dir = os.path.join(save_dir, tstr)
            if not os.path.exists(save_dir): os.mkdir(save_dir)
            if not os.path.exists(tstr_dir): os.mkdir(tstr_dir)
            import shutil
            shutil.copyfile(crop_path, os.path.join(tstr_dir, "depth.png"))
            shutil.copyfile(draw_path, os.path.join(tstr_dir, "grasp.png"))
            shutil.copyfile(os.path.join(root_dir, "data/force/out.txt"), os.path.join(tstr_dir, "force.txt"))
            shutil.copyfile(os.path.join(root_dir, "data/force/out.png"), os.path.join(tstr_dir, "force.png"))
    else:
        print("[!] Motion planning failed ...")
"""