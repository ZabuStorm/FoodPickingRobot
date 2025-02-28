import os
import random
import importlib
FOUND_CNOID = importlib.util.find_spec("cnoid") is not None
PLAY = True
if FOUND_CNOID: 
    from cnoid.Util import *
    from cnoid.Base import *
    from cnoid.Body import *
    from cnoid.BodyPlugin import *
    from cnoid.GraspPlugin import *
    from cnoid.BinPicking import *

from bpbot.binpicking import *
from bpbot.config import BinConfig
from bpbot.robotcon.nxt.nxtrobot_client import NxtRobot

import timeit
import numpy as np
start = timeit.default_timer()

# ---------------------- get config info -------------------------

root_dir = os.path.realpath(os.path.join(os.path.realpath(__file__), "../../"))
print(f"Root directory: {root_dir} ")

img_path = os.path.join(root_dir, "data/depth/depth.png")
crop_path = os.path.join(root_dir, "data/depth/cropped.png")
config_path = os.path.join(root_dir, "config/config.yaml")
mf_path = os.path.join(root_dir, "data/motion/motion.dat")
draw_path = os.path.join(root_dir, "data/depth/result.png")

cfg = BinConfig(config_path=config_path)
cfgdata = cfg.data
camera_mode = cfgdata["exp"]["camera_mode"]
exp_mode = cfgdata["exp"]["mode"]
mf_mode = cfgdata["exp"]["motionfile_mode"]
lr_arm = cfgdata["exp"]["lr_arm"]
destination = cfgdata["exp"]["destination"]
log_mode = cfgdata["exp"]["log_mode"]

# ---------------------- get depth img -------------------------
if camera_mode:
    point_array = capture_pc()
    print("Captured point cloud ... ")
    img, img_blur = pc2depth(point_array, cfgdata)
    cv2.imwrite(img_path, img_blur)
    crop = crop_roi(img_blur, cfgdata, bounding_size=50)
    cv2.imwrite(crop_path, crop)
    
# pcd = o3d.io.read_point_cloud("./data/test/out.ply")
# point_array = pcd.points

# ---------------------- compute grasps -------------------------

if exp_mode == "emap":

    h_params = {
        "finger_length": 25,
        "finger_width":  8, 
        "open_width":    60
    }
        
    g_params = {
        "rotation_step": 22.5, 
        "depth_step":    10,
        "hand_depth":    20
    }
    grasps = detect_nontangle_grasp(n_grasp=10, 
                                    img_path=crop_path, 
                                    g_params=g_params, 
                                    h_params=h_params,
                                    t_params=cfgdata["tangle"])

elif exp_mode == "picknet":
    grasps = picknet(img_path=crop_path, hand_config=cfgdata["hand"][lr_arm])


else:
    # default: "fge"
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

    if mf_mode:
        print("Geenrate motion file .. ")
        gen_motionfile_pick(mf_path, eef_pose, dest=destination, arm=lr_arm, action_idx=1)

# ---------------------- execute on robot -------------------------
if FOUND_CNOID and camera_mode: 
    if mf_mode:
        success = load_motionfile(mf_path)
    else:
        # compute place eef pose
        if destination == "front":
            place_eef_xyz = [0.48,0.30,0.25] 
        elif destination == "side":
            place_eef_xyz = [0.000, 0.552, 0.25]

        if lr_arm == "right":
            place_eef_xyz[1] = -place_eef_xyz[1]

        success = plan_binpicking(lr_arm, eef_pose[:3], eef_pose[3:], place_eef_xyz)
    
    motion_seq = get_motion()
    print(motion_seq.shape)
    print(motion_seq)
    
    if success and PLAY: 
        print(f"[*] Success! Total {motion_seq.shape[0]} motion sequences! ")
        nxt = NxtRobot(host='[::]:15005')

        nxt.playMotion(motion_seq)
        
        if log_mode:
            tdatetime = dt.now()
            tstr = tdatetime.strftime('%Y%m%d%H%M%S')
            save_dir = f"/home/hlab/Desktop/exp/{tstr}" 
            os.mkdir(save_dir)
            np.savetxt(os.path.join(save_dir, "out.txt"), np.asarray(grasps), delimiter=',')
            cv2.imwrite(os.path.join(save_dir, "grasp.png"), img_grasp)
            cv2.imwrite(os.path.join(save_dir, "depth.png"), crop)

end = timeit.default_timer()
print("[*] Time: {:.2f}s".format(end - start))
