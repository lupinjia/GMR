import argparse
import json
import pathlib
import os
import multiprocessing as mp
import yaml
import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm
from natsort import natsorted
from rich import print
import torch
import pickle

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting.utils.smpl import load_smplx_file, get_smplx_data_offline_fast
from general_motion_retargeting.kinematics_model import KinematicsModel
from general_motion_retargeting import IK_CONFIG_ROOT
import gc
import time
import psutil
import tracemalloc




def quat_to_rotmat(q):
    """
    将四元数转为旋转矩阵
    参数:
    q (ndarray): 四元数 [B, 4]，其中 B 是帧数，q 为 [w, x, y, z]
    
    返回:
    rotmat (ndarray): 旋转矩阵 [B, 3, 3]
    """
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    
    # 计算旋转矩阵 (3x3)
    rotmat = np.stack([
        1 - 2 * (y**2 + z**2), 2 * (x*y - z*w), 2 * (x*z + y*w),
        2 * (x*y + z*w), 1 - 2 * (x**2 + z**2), 2 * (y*z - x*w),
        2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x**2 + y**2)
    ], axis=-1).reshape(-1, 3, 3)

    return rotmat

def local_to_world(local_body_pos, root_rot, fk_root_pos):
    """
    local_body_pos: (B, N, 3)
    root_rot:       (B, 4)   wxyz
    fk_root_pos:    (B, 3)
    return:         (B, N, 3)
    """
    rotmat = quat_to_rotmat(root_rot)      # (B, 3, 3)

    # ⭐ 核心：对每一帧 b，把 R[b] 作用在所有 N 个点上
    world_pos = np.einsum('bij,bnj->bni', rotmat, local_body_pos)

    # 平移
    world_pos += fk_root_pos[:, None, :]

    return world_pos
# def local_to_world(local_body_pos, root_rot, fk_root_pos):
#     """
#     将 `local_body_pos` 从本地坐标系转换到世界坐标系.
    
#     参数:
#     local_body_pos (Tensor): [num_frames, 3] 本地坐标系下的位置
#     root_rot (Tensor): [num_frames, 4] 四元数表示的根节点旋转（w, x, y, z）
#     fk_root_pos (Tensor): [num_frames, 3] 根节点在世界坐标系中的位置
    
#     返回:
#     world_pos (Tensor): [num_frames, 3] 世界坐标系下的位置
#     """
    
#     # 四元数转旋转矩阵 (四元数是 [w, x, y, z])
#     def quat_to_rotmat(q):
#         w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
#         # 计算旋转矩阵 (3x3)
#         rotmat = torch.stack([
#             1 - 2 * (y**2 + z**2), 2 * (x*y - z*w), 2 * (x*z + y*w),
#             2 * (x*y + z*w), 1 - 2 * (x**2 + z**2), 2 * (y*z - x*w),
#             2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x**2 + y**2)
#         ], dim=-1).view(-1, 3, 3)
#         return rotmat

#     # 计算旋转矩阵
#     rotmat = quat_to_rotmat(root_rot)

#     # 将 local_body_pos 旋转到世界坐标系
#     world_pos_rotated = torch.bmm(local_body_pos.unsqueeze(1), rotmat).squeeze(1)

#     # 加上根节点位置，得到最终的世界坐标系位置
#     world_pos = world_pos_rotated + fk_root_pos

#     return world_pos

def check_memory(min_available_gb=50):
    mem = psutil.virtual_memory()

    used_gb = (mem.total - mem.available) / (1024 ** 3)
    available_gb = mem.available / (1024 ** 3)

    if available_gb < min_available_gb:
        print(
            f"[WARNING] Available memory low: "
            f"used={used_gb:.2f} GB, available={available_gb:.2f} GB "
            f"(min required {min_available_gb} GB)."
        )
        return True

    return False

HERE = pathlib.Path(__file__).parent


def process_file(smplx_file_path, tgt_file_path, tgt_robot, SMPLX_FOLDER, tgt_folder, total_files, verbose=False):
    def log_memory(message):
        if verbose:
            process = psutil.Process(os.getpid())
            memory_usage = process.memory_info().rss / (1024 ** 3)  # Convert to GB
            print(f"[MEMORY] {message}: {memory_usage:.2f} GB")
    
    # Start memory tracking if verbose
    if verbose:
        tracemalloc.start()
        
    # Initial checks (with optional logging)
    log_memory("Initial memory usage")
    
    num_pause = 0
    # while check_memory():
    #     print(f"[PAUSE] Paused processing {smplx_file_path} to prevent memory overflow. num_pause: {num_pause}")
    #     time.sleep(60*2)
    #     num_pause += 1
    #     if num_pause > 10:
    #         print(f"[ERROR] Memory usage is still high after 10 pauses. Exiting.")
    #         return

    try:
        smplx_data, body_model, smplx_output, actual_human_height = load_smplx_file(smplx_file_path, SMPLX_FOLDER)
        mocap_frame_rate = smplx_data["mocap_frame_rate"]
        log_memory("After loading SMPL-X data")
    except Exception as e:
        print(f"Error loading {smplx_file_path}: {e}")
        return
    
  
    tgt_fps = 50
    try:
        smplx_frame_data_list, aligned_fps = get_smplx_data_offline_fast(smplx_data, body_model, smplx_output, tgt_fps=tgt_fps)
    except Exception as e:
        print(f"Error processing {smplx_file_path}: {e}")
        return
    
    # retarget
    retargeter = GMR(
        src_human="smplx",
        tgt_robot=tgt_robot,
        actual_human_height=actual_human_height,
    )
    qpos_list = []
    qvel_list = []

    for smplx_frame_data in smplx_frame_data_list:
        qpos = retargeter.retarget(smplx_frame_data)[0]
        qvel = retargeter.retarget(smplx_frame_data)[2]
        # print(retargeter.retarget(smplx_frame_data).)
        qpos_list.append(qpos.copy())
        qvel_list.append(qvel.copy())



    # for smplx_frame_data in smplx_frame_data_list:
    #     try:
    #         ret = retargeter.retarget(smplx_data, frame_dt_target=1.0 / float(tgt_fps))
    #     except TypeError:
    #         ret = retargeter.retarget(smplx_data)

    #     # 解包：支持返回 2 或 3 项
    #     if isinstance(ret, tuple):
    #         if len(ret) == 3:
    #             qpos, _qvel_last, qvel = ret
    #         elif len(ret) == 2:
    #             qpos, qvel = ret
    #             _qvel_last = None
    #         else:
    #             raise RuntimeError(f"Unexpected retarget() return length: {len(ret)}")
    #     else:
    #         raise RuntimeError("retarget() should return a tuple")
        # qpos_list.append(qpos.copy())
        # qvel_list.append(qvel.copy())


    qpos_list = np.array(qpos_list)
    qvel_list = np.array(qvel_list)

    log_memory("After retargeting")
    
    device = "cuda:0"
    kinematics_model = KinematicsModel(retargeter.xml_file, device=device)

    try:
        root_pos = qpos_list[:, :3]
    except Exception as e:
        print(f"Error processing {smplx_file_path}: {e}")
        return
    root_rot = qpos_list[:, 3:7]
    lin_vel_world = qvel_list[:, 0:3]
    ang_vel_body = qvel_list[:, 3:6]
    root_rot = qpos_list[:, 3:7]
    root_rot_wxyz = qpos_list[:, 3:7]
    root_rot[:, [0, 1, 2, 3]] = root_rot[:, [1, 2, 3, 0]]
    dof_pos = qpos_list[:, 7:]
    dof_vel = qvel_list[:, 6:]
    num_frames = root_pos.shape[0]

    fk_root_pos = torch.zeros((num_frames, 3), device=device)
    fk_root_rot = torch.zeros((num_frames, 4), device=device)
    fk_root_rot[:, -1] = 1.0

    local_body_pos, local_body_rot = kinematics_model.forward_kinematics(
        fk_root_pos, fk_root_rot, torch.from_numpy(dof_pos).to(device=device, dtype=torch.float)
    )

    log_memory("After forward kinematics")

    body_names = kinematics_model.body_names
    
    HEIGHT_ADJUST = True
    if HEIGHT_ADJUST:
        # height adjust to ensure the lowerset part is on the ground
        body_pos, _ = kinematics_model.forward_kinematics(torch.from_numpy(root_pos).to(device=device, dtype=torch.float), 
                                                        torch.from_numpy(root_rot).to(device=device, dtype=torch.float), 
                                                        torch.from_numpy(dof_pos).to(device=device, dtype=torch.float)) # TxNx3
        ground_offset = 0.0


        lowerst_height = torch.min(body_pos[25:30,[9,18], 2]).item()
        print("lowerst_height: ", body_pos.size())
        root_pos[:, 2] = root_pos[:, 2] - lowerst_height + ground_offset # make sure motion on the ground
        
    ROOT_ORIGIN_OFFSET = True
    if ROOT_ORIGIN_OFFSET:
        # offset using the first frame
        root_pos[25:, :2] -= root_pos[25, :2]
        
    # print()
    lin_vel_world = np.gradient(root_pos, 1/tgt_fps, axis=0).astype(np.float32)

    # R_bw = R.from_quat(root_rot_wxyz, scalar_first=True).as_matrix()   # (T,3,3) body->world
    # R_wb = np.transpose(R_bw, (0, 2, 1))                               # (T,3,3) world->body
    # root_vel_body = np.einsum('tij,tj->ti', R_wb, lin_vel_world).astype(np.float32)  # (T,3)
    # print("root_vel_body: ", root_vel_body)   
    # print("root_vel_body: ", root_vel_body)   

    dof_vel = np.gradient(dof_pos, 0.02, axis=0).astype(np.float32)

    r = R.from_quat(root_rot)
    ang_vel_body = r.inv().apply(np.gradient(r.as_rotvec(), 1/tgt_fps, axis=0) )

    root_vel_body = r.inv().apply(lin_vel_world)
    world_pos = local_to_world(local_body_pos.detach().cpu().numpy(), root_rot, root_pos)
    # print(":ccc : ", local_body_pos.detach().cpu().numpy()[:,5,:])
    # print(root_vel_body)
    motion_data = {
        # "fps": aligned_fps,
        "root_trans": root_pos,
        "root_rot": root_rot,
        "base_lin_vel": root_vel_body,
        "base_ang_vel": ang_vel_body,
        "dof_pos": dof_pos,
        "dof_vel": dof_vel,
        "local_body_pos": local_body_pos.detach().cpu().numpy(),
        "link_rot_base": local_body_rot.detach().cpu().numpy(),
        # "link_body_list": body_names,
    }



    amp_motion = {
        "joints_list": [
            "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", 
            "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch", 
            "waist_yaw",
            "left_shoulder_pitch", "left_shoulder_roll",  "left_elbow_pitch",
            "right_shoulder_pitch", "right_shoulder_roll",  "right_elbow_pitch"
        ],
        "joint_positions": [],
        "root_position": [],
        "root_quaternion": [],
        "fps": tgt_fps,  # 或者你自己的 fps
    }

    # 转 numpy（如果是 tensor）
    root_pos_np = root_pos
    root_rot_np = root_rot
    dof_pos_np = dof_pos

    n_frames = root_pos_np.shape[0]

    for f in range(n_frames):
        amp_motion["joint_positions"].append(dof_pos_np[f])
        amp_motion["root_position"].append(root_pos_np[f])
        amp_motion["root_quaternion"].append(root_rot_np[f])

    # 转成 numpy array（推荐）
    amp_motion["joint_positions"] = np.array(amp_motion["joint_positions"])
    amp_motion["root_position"] = np.array(amp_motion["root_position"])
    amp_motion["root_quaternion"] = np.array(amp_motion["root_quaternion"])

    # 保存
    npy_save_path = tgt_file_path.replace(".npz", "_amp.npy")
    np.save(npy_save_path, amp_motion)

    print(f"AMP npy saved to: {npy_save_path}")




    np.savez_compressed(
        tgt_file_path,
        root_trans=root_pos[:],
        root_ori=root_rot[:],
        base_lin_vel=root_vel_body[:],
        base_ang_vel=ang_vel_body[:],
        dof_pos=dof_pos[:],
        dof_vel=dof_vel[:],
        link_pos_base=local_body_pos[:,[2,4,6,11,13,15,19,22,23,24,27,28,29]].detach().cpu().numpy(),
        link_rot_base=local_body_rot[:,[2,4,6,11,13,15,19,22,23,24,27,28,29]].detach().cpu().numpy(),
        # left_contact=left_contact[:],
        # right_contact=right_contact[:],


    )





    data_dict = {
    "root_trans": root_pos[:],
    "root_ori": root_rot[:],
    "base_lin_vel": root_vel_body[:],
    "base_ang_vel": ang_vel_body[:],
    "dof_pos": dof_pos[:],
    "dof_vel": dof_vel[:],
    "link_pos_base": local_body_pos[:,[2,4,6,11,13,15,19,22,23,24,27,28,29]].detach().cpu().numpy(),
    "link_rot_base": local_body_rot[:,[2,4,6,11,13,15,19,22,23,24,27,28,29]].detach().cpu().numpy(),
}
    yaml_path = os.path.splitext(tgt_file_path)[0] + ".yaml"

    # ⚠️ YAML 不支持 numpy array，需要转成 list
    yaml_dict = {k: v.tolist() for k, v in data_dict.items()}

    with open(yaml_path, "w") as f:
        yaml.safe_dump(yaml_dict, f)

    print(f"Saved to {tgt_file_path}")




            
    # Progress print based on tgt_folder
    done = 0
    for root, _, files in os.walk(tgt_folder):
        done += len([f for f in files if f.endswith('.npz')])
    print(f"Processed {done}/{total_files}: {tgt_file_path}")
    
    if verbose:
        # Get memory snapshot
        snapshot = tracemalloc.take_snapshot()
        top_stats = snapshot.statistics('lineno')
        
        print("\nTop 10 memory-consuming lines:")
        for stat in top_stats[:10]:
            print(stat)
        
        tracemalloc.stop()
        
    # clean cache
    torch.cuda.empty_cache()
    gc.collect()
    


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", default="phybot_c2")
    parser.add_argument("--src_folder", type=str,
                            default="./data/CMU/10",                        )
    parser.add_argument("--tgt_folder", type=str,
                        default="./output_c2", 
                        )
    
    parser.add_argument("--override", default=False, action="store_true")
    parser.add_argument("--num_cpus", default=2, type=int)
    args = parser.parse_args()
    
    # print the total number of cpus and gpus
    print(f"Total CPUs: {mp.cpu_count()}")
    print(f"Using {args.num_cpus} CPUs.")
    
    src_folder = args.src_folder
    tgt_folder = args.tgt_folder

    SMPLX_FOLDER = HERE / ".." / "assets" / "body_models"
    hard_motions_folder = HERE / ".." / "assets" / "hard_motions"

    verbose = False

    # hard_motions_paths = [hard_motions_folder / "0.txt", 
    #                       hard_motions_folder / "1.txt"]
    # hard_motions = []
    # for hard_motions_path in hard_motions_paths:
    #     with open(hard_motions_path, "r") as f:
    #         for line in f:
    #             if "Motion:" in line:
    #                 motion_path = line.split(":")[1].strip()
    #             else:
    #                 continue
    #             motion_path = motion_path.split(",")[0].strip().split(".")[0]
    #             hard_motions.append(motion_path)
                
                
    args_list = []
    for dirpath, _, filenames in os.walk(src_folder):
        for filename in natsorted(filenames):
            if filename.endswith("_stagei.npz"):
                continue
            if filename.endswith((".npz", ".npz")):
                smplx_file_path = os.path.join(dirpath, filename)
                tgt_file_path = smplx_file_path.replace(src_folder, tgt_folder).replace(".npz", ".npz")
                if not os.path.exists(tgt_file_path) or args.override:
                    args_list.append((smplx_file_path, tgt_file_path, args.robot, SMPLX_FOLDER, tgt_folder))
    print("full args_list:", args_list)
    
    # remove hard and infeasible motions
    exclude_file_content = ["BMLrub", "EKUT", "crawl", "_lie", "upstairs", "downstairs"]
    
    new_args_list = []
    for arguments in args_list:
        motion_name = arguments[0].split("/")[-1].split('.')[0]
        # if motion_name in hard_motions:
        #     continue
        if any(content in motion_name for content in exclude_file_content):
            continue
        new_args_list.append(arguments)
    args_list = new_args_list
    
    
    print("new args_list:", len(args_list))
    
    total_files = len(args_list)
    print(f"Total number of files to process: {total_files}")
    with mp.Pool(args.num_cpus) as pool:
        pool.starmap(process_file, [args + (total_files, verbose) for args in args_list])

    print("Done. Saved to ", tgt_folder)


if __name__ == "__main__":
    main()
