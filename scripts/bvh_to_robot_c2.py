import argparse
import pathlib
import time
from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting.kinematics_model import KinematicsModel
import torch
from general_motion_retargeting import RobotMotionViewer
from general_motion_retargeting.utils.lafan1 import load_bvh_file
from rich import print
from tqdm import tqdm
import os
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import yaml



def slerp_batch(q0, q1, t_vals):
    """
    q0, q1: (1,4)
    t_vals: (N,1)
    return: (N,4)
    """
    q0 = q0.repeat(len(t_vals), axis=0)
    q1 = q1.repeat(len(t_vals), axis=0)

    dot = np.sum(q0 * q1, axis=1, keepdims=True)

    # shortest path
    flip_mask = dot < 0
    q1[flip_mask[:,0]] *= -1
    dot = np.abs(dot)

    DOT_THRESHOLD = 0.9995

    result = np.zeros_like(q0)

    linear_mask = dot > DOT_THRESHOLD

    # 线性部分
    result[linear_mask[:,0]] = (
        q0[linear_mask[:,0]] +
        t_vals[linear_mask[:,0]] * (q1[linear_mask[:,0]] - q0[linear_mask[:,0]])
    )

    # slerp 部分
    theta_0 = np.arccos(np.clip(dot, -1.0, 1.0))
    sin_theta_0 = np.sin(theta_0)

    theta = theta_0 * t_vals
    sin_theta = np.sin(theta)

    s0 = np.sin(theta_0 - theta) / sin_theta_0
    s1 = sin_theta / sin_theta_0

    slerp_part = s0 * q0 + s1 * q1

    result[~linear_mask[:,0]] = slerp_part[~linear_mask[:,0]]

    # 归一化
    result /= np.linalg.norm(result, axis=1, keepdims=True)

    return result


default_dof = np.array([
    # left leg (7)
    -0.1,   0.0,  -0.0,   # hip xyz
    0.2,                 # knee
    -0.1,                # ankle pitch
    0.0,             # ankle roll/yaw
    
    # right leg (7)
    -0.1,   0.0,  -0.0,   # hip xyz
    0.2,                 # knee
    -0.1,                # ankle pitch
    0.0,   

    # torso + others (7)
    0.0, 
    0.0,  0.1, 0.0, -0.1,
    0.0,  -0.1, 0.0, -0.1,
])


default_root_pos = np.array([
    # left leg (7)
    -0.1,   0.0,  -0.0,   # hip xyz

])

def generate_contact_sequence(world_pos,  contact_threshold=0.05, velocity_threshold=0.015):
    """
    基于脚在世界坐标系中的位置和速度生成粗略的接触序列。
    
    :param world_pos: 脚在世界坐标系中的位置 (N, M, 3)，其中 N 为帧数，M 为脚数
    :param contact_threshold: 用于判断脚是否接触地面的 z 坐标阈值
    :param velocity_threshold: 用于判断脚的速度是否接近零的阈值
    :return: 接触序列，形状为 (N, M)，值为 1 表示接触，0 表示未接触
    """
    
    # 获取脚的 x, y, z 轴坐标
    xyz_positions = world_pos
    
    # 计算每一帧的速度（位置差分）
    # velocities = np.diff(xyz_positions, axis=0)  # 计算帧间差分
    velocities = np.zeros_like(xyz_positions)  # (N, M, 3)
    velocities[1:-1] = (xyz_positions[2:] - xyz_positions[:-2]) / 2  # 中间帧
    velocities[0] = xyz_positions[1] - xyz_positions[0]  # 第一帧
    velocities[-1] = xyz_positions[-1] - xyz_positions[-2]  # 最后一帧
    # print(np.linalg.norm(velocities, axis=  1))
    # 初始化接触序列
    contact_sequence = np.zeros_like(np.linalg.norm(velocities, axis=  1))  # (N, M)，只存储接触信息
    contact_sequence = np.zeros_like(np.linalg.norm(velocities, axis=  1))  # (N, M)，只存储接触信息
    # print("np.linalg.norm(velocities, axis=  1): ", np.min(np.linalg.norm(velocities, axis=  1)))

    velocity_threshold += np.min(np.linalg.norm(velocities, axis=  1))
    for j in range(len(contact_sequence)):
        # 判断脚是否接触：位置的 z 低于阈值，且速度在 x, y, z 方向上都接近零
        if xyz_positions[j, 2] > contact_threshold or np.linalg.norm(velocities[j]) > velocity_threshold:
            contact_sequence[ j] = 0  # 标记为接触
        else:
            contact_sequence[j] = 1  # 标记为未接触

    return contact_sequence


def quat_to_rot_matrix(q):
    """
    将四元数（x, y, z, w）转换为旋转矩阵。
    
    :param q: 四元数 (x, y, z, w)
    :return: 3x3的旋转矩阵
    """
    x, y, z, w = q
    # 计算旋转矩阵
    rot_matrix = np.array([
        [1 - 2 * (y ** 2 + z ** 2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x ** 2 + z ** 2), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x ** 2 + y ** 2)]
    ])
    return rot_matrix

def transform_to_world_from_quat(link_pos_base, quat, root_trans):
    """
    将多帧数据从本地坐标系转换到世界坐标系。
    
    :param link_pos_base: (N, M, 3) 本地坐标系下的链接位置，N 是帧数，M 是链接数
    :param quat: (N, 4) 四元数，表示每一帧的旋转 (x, y, z, w)
    :param root_trans: (N, 3) 每一帧的根节点世界坐标位置
    :return: 转换后的世界坐标系下的位置，形状为(N, M, 3)
    """
    N, M, _ = link_pos_base.shape  # N 是帧数，M 是链接数

    # 初始化存储结果的数组
    world_pos = np.zeros_like(link_pos_base)

    for i in range(N):
        # 提取当前帧的四元数和根节点位置
        current_quat = quat[i]
        current_root_trans = root_trans[i]

        # 将四元数转换为旋转矩阵
        rotation_matrix = quat_to_rot_matrix(current_quat)

        # 将当前帧的 link_pos_base 从本地坐标系转换到世界坐标系
        for j in range(M):
            # 当前链接的相对位置
            link_pos = link_pos_base[i, j]

            # 旋转并平移到世界坐标系
            world_pos[i, j] = np.dot(link_pos, rotation_matrix.T) + current_root_trans
    
    return world_pos


def _ensure_quat_hemisphere_wxyz(q_prev_wxyz: np.ndarray, q_curr_wxyz: np.ndarray) -> np.ndarray:
    """若两帧四元数（wxyz）点积为负，翻转当前帧到同一半球，避免差分跳变。"""
    if float(np.dot(q_prev_wxyz, q_curr_wxyz)) < 0.0:
        return -q_curr_wxyz
    return q_curr_wxyz


def get_bvh_frame_rate(bvh_file_path):
    """
    从BVH文件中提取帧率信息
    """
    try:
        with open(bvh_file_path, 'r') as f:
            lines = f.readlines()
        
        # 查找Frame Time行
        for line in lines:
            if line.strip().startswith('Frame Time'):
                frame_time = float(line.strip().split()[-1])
                fps = 1.0 / frame_time
                return fps
    except Exception as e:
        print(f"[WARN] Could not extract frame rate from BVH file: {e}")
    
    return None


def check_exported_root_velocities(qpos_seq: np.ndarray,
                                   qvel_seq: np.ndarray,
                                   dt_list: np.ndarray,
                                   lin_tol: float = 5e-3,
                                   ang_tol: float = 5e-3,
                                   strict: bool = False) -> None:
    """
    逐帧验证导出的 root 速度是否与位姿差分一致。
    约定：dt_list[i] 表示区间 (i-1 -> i) 的累计子步时长；dt_list[0] 无意义。
      qpos = [root_pos(3), root_quat(wxyz)(4), dof_pos...]
      qvel = [0:3 线速(world), 3:6 角速(body), 6: 关节速]
    """
    assert qpos_seq.ndim == 2 and qvel_seq.ndim == 2, "qpos/qvel must be (T, ...)"
    T = min(qpos_seq.shape[0], qvel_seq.shape[0], dt_list.shape[0])
    if T < 2:
        print("[check] sequence too short to validate (T < 2), skip.")
        return

    n_bad = 0
    for i in range(1, T):
        dt = float(dt_list[i])
        if not (dt > 0.0 and np.isfinite(dt)):
            print(f"[WARN][frame {i-1}->{i}] invalid dt (dt={dt}), skip check for this interval.")
            continue

        # 位姿
        p_prev = qpos_seq[i-1, :3]
        q_prev_wxyz = qpos_seq[i-1, 3:7]
        p_curr = qpos_seq[i, :3]
        q_curr_wxyz = qpos_seq[i, 3:7]
        q_curr_wxyz = _ensure_quat_hemisphere_wxyz(q_prev_wxyz, q_curr_wxyz)

        # R_{i-1}, R_i（body->world）
        R_prev = R.from_quat(q_prev_wxyz, scalar_first=True).as_matrix()
        R_curr = R.from_quat(q_curr_wxyz, scalar_first=True).as_matrix()

        # —— 帧间差分 —— #
        v_fd_world = (p_curr - p_prev) / dt
        R_delta = R_prev.T @ R_curr
        w_fd_body = R.from_matrix(R_delta).as_rotvec() / dt  # (i-1)body 表达

        # —— qvel —— #
        w_q_body = qvel_seq[i, 3:6]   # 体坐标角速度
        v_q_world = qvel_seq[i, 0:3]  # 世界系线速度

        lin_err = float(np.linalg.norm(v_fd_world - v_q_world))
        ang_err = float(np.linalg.norm(w_fd_body - w_q_body))

        ok = (lin_err <= lin_tol) and (ang_err <= ang_tol)
        tag = "OK  " if ok else "WARN"
        print(f"[{tag}][frame {i-1}->{i}] lin_err={lin_err:.3e}; ang_err={ang_err:.3e}; dt={dt:.6f}")

        if not ok:
            n_bad += 1
            if strict:
                raise RuntimeError(
                    f"[frame {i-1}->{i}] velocity mismatch (lin_tol={lin_tol}, ang_tol={ang_tol})."
                )

    if n_bad == 0:
        print("[check] all frame-to-frame root velocities consistent within tolerance.")
    else:
        print(f"[check] {n_bad} / {T-1} intervals exceeded tolerance "
              f"(lin_tol={lin_tol}, ang_tol={ang_tol}).")


def resample_bvh_data(bvh_data_frames, src_fps, tgt_fps):
    """
    简单的BVH数据重采样（线性插值位置，球面线性插值旋转）
    """
    if abs(src_fps - tgt_fps) < 1e-6:
        return bvh_data_frames
    
    print(f"[INFO] Resampling BVH data from {src_fps:.3f} Hz to {tgt_fps:.3f} Hz")
    
    src_interval = 1.0 / src_fps
    tgt_interval = 1.0 / tgt_fps
    
    # 计算目标时间线
    src_duration = len(bvh_data_frames) * src_interval
    tgt_frames_count = int(src_duration * tgt_fps)
    tgt_times = np.linspace(0, src_duration - src_interval, tgt_frames_count)
    
    resampled_data = []
    
    for t in tgt_times:
        src_frame_idx = t / src_interval
        idx_prev = int(np.floor(src_frame_idx))
        idx_next = min(idx_prev + 1, len(bvh_data_frames) - 1)
        alpha = src_frame_idx - idx_prev
        
        frame_prev = bvh_data_frames[idx_prev]
        frame_next = bvh_data_frames[idx_next]
        
        resampled_frame = {}
        
        for joint_name in frame_prev.keys():
            pos_prev, rot_prev = frame_prev[joint_name]
            pos_next, rot_next = frame_next[joint_name]
            
            # 线性插值位置
            pos_interp = (1 - alpha) * np.array(pos_prev) + alpha * np.array(pos_next)
            
            # 球面线性插值旋转
            rot_prev_r = R.from_quat(rot_prev, scalar_first=True)
            rot_next_r = R.from_quat(rot_next, scalar_first=True)
            # rot_interp_r = R.slerp(rot_prev_r, rot_next_r, alpha)

            # rot_prev_r, rot_next_r: Rotation 对象
            alpha = float(alpha)

            key_times = [0.0, 1.0]
            key_rots = R.from_quat([
                rot_prev_r.as_quat(),   # (x,y,z,w)
                rot_next_r.as_quat()
            ])

            slerp = Slerp(key_times, key_rots)
            rot_interp_r = slerp([alpha])[0]

            # scipy 默认是 (x,y,z,w)
            rot_interp = rot_interp_r.as_quat()
            rot_interp = rot_interp_r.as_quat(scalar_first=True)
            
            resampled_frame[joint_name] = [pos_interp, rot_interp]
        
        resampled_data.append(resampled_frame)
    
    return resampled_data


if __name__ == "__main__":
    
    HERE = pathlib.Path(__file__).parent

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--bvh_file",
        help="BVH motion file to load.",
        default="/home/kairan/Downloads/lafan1/dance2_subject1.bvh",
        type=str,
    )
    
    parser.add_argument(
        "--format",
        choices=["lafan1", "nokov","mocap"],
        default="lafan1",
    )
    
    parser.add_argument(
        "--loop",
        default=False,
        action="store_true",
        help="Loop the motion.",
    )
    
    parser.add_argument(
        "--robot",
        choices=["unitree_g1", "unitree_g1_with_hands", "booster_t1", "stanford_toddy", "phybot_c2", "fourier_n1", "engineai_pm01", "pal_talos"],
        default="phybot_c2",
    )
    
    parser.add_argument(
        "--record_video",
        action="store_true",
        default=False,
    )

    parser.add_argument(
        "--video_path",
        type=str,
        default="videos/example.mp4",
    )

    parser.add_argument(
        "--rate_limit",
        action="store_true",
        default=False,
    )

    parser.add_argument(
        "--save_path",
        default="/home/kairan/Documents/GMR_phybot/output_c2/lafan1",
        help="Path to save the robot motion.",
    )
    
    parser.add_argument(
        "--motion_fps",
        default=50,
        type=float,
        help="Target FPS for retargeting. If not specified, will use source FPS.",
    )
    
    # 校验阈值与严格模式
    parser.add_argument("--vel_check_strict", action="store_true", default=False)
    parser.add_argument("--lin_tol", type=float, default=5e-3)
    parser.add_argument("--ang_tol", type=float, default=5e-3)

    args = parser.parse_args()

    # Load BVH trajectory
    lafan1_data_frames, actual_human_height = load_bvh_file(args.bvh_file, format=args.format)
    
    # ========= 从BVH文件中检测源帧率 =========
    src_fps = get_bvh_frame_rate(args.bvh_file)
    if src_fps is None:
        src_fps = args.motion_fps if args.motion_fps is not None else 30.0
        print(f"[WARN] Could not detect frame rate from BVH file, using {src_fps:.3f} Hz")
    else:
        print(f"[INFO] Detected source FPS from BVH file: {src_fps:.3f} Hz")
    
    # 确定目标帧率
    tgt_fps = args.motion_fps if args.motion_fps is not None else src_fps
    print(f"[INFO] Target FPS: {tgt_fps:.3f} Hz")
    
    # 如果源帧率和目标帧率不同，进行重采样
    if abs(src_fps - tgt_fps) > 1e-6:
        lafan1_data_frames = resample_bvh_data(lafan1_data_frames, src_fps, tgt_fps)
        aligned_fps = tgt_fps
        print(f"[INFO] Resampled to {len(lafan1_data_frames)} frames at {aligned_fps:.3f} Hz")
    else:
        aligned_fps = src_fps
        print(f"[INFO] Using original {len(lafan1_data_frames)} frames at {aligned_fps:.3f} Hz")
    
    # Initialize the retargeting system
    retargeter = GMR(
        src_human=f"bvh_{args.format}",
        tgt_robot=args.robot,
        actual_human_height=actual_human_height,
    )

    robot_motion_viewer = RobotMotionViewer(robot_type=args.robot,
                                            motion_fps=aligned_fps,
                                            transparent_robot=0,
                                            record_video=args.record_video,
                                            video_path=args.video_path,
                                            # video_width=2080,
                                            # video_height=1170
                                            )
    
    # FPS measurement variables
    fps_counter = 0
    fps_start_time = time.time()
    fps_display_interval = 2.0  # Display FPS every 2 seconds
    
    print(f"Final motion FPS: {aligned_fps}")
    
    # 保存容器
    if args.save_path is not None:
        save_dir = os.path.dirname(args.save_path)
        if save_dir:  # Only create directory if it's not empty
            os.makedirs(save_dir, exist_ok=True)
        qpos_list = []
        qvel_list = []
        frame_dt_list = []

    # Create tqdm progress bar for the total number of frames
    pbar = tqdm(total=len(lafan1_data_frames), desc="Retargeting")
    
    # ====== 仅 --rate_limit 时，按墙钟限速 ======
    desired_dt = 1.0 / float(aligned_fps)
    next_frame_time = time.perf_counter()

    # Start the viewer
    i = 0
    
    target_dt = 1.0 / float(aligned_fps)  # 目标帧时长（传给 retarget）

    try:
        # 让 frame_dt 与索引对齐：先放一个占位 0.0（表示第 0 帧不存在的前置区间）
        if args.save_path is not None:
            frame_dt_list.append(0.0)

        while True:
            # 限速（仅开启时）
            if args.rate_limit:
                now = time.perf_counter()
                if now < next_frame_time:
                    time.sleep(next_frame_time - now)
                    next_frame_time += desired_dt
                else:
                    missed = int((now - next_frame_time) // desired_dt) + 1
                    next_frame_time += missed * desired_dt
            
            # FPS measurement
            fps_counter += 1
            current_time = time.time()
            if current_time - fps_start_time >= fps_display_interval:
                actual_fps = fps_counter / (current_time - fps_start_time)
                print(f"Actual rendering FPS: {actual_fps:.2f}")
                fps_counter = 0
                fps_start_time = current_time
                
            # Update progress bar
            pbar.update(1)

            # Update task targets.
            smplx_data = lafan1_data_frames[i]

            # retarget with frame_dt_target
            try:
                ret = retargeter.retarget(smplx_data, frame_dt_target=target_dt)
            except TypeError:
                ret = retargeter.retarget(smplx_data)

            # 解包：支持返回 2 或 3 项
            if isinstance(ret, tuple):
                if len(ret) == 3:
                    qpos, _qvel_last, qvel = ret
                elif len(ret) == 2:
                    qpos, qvel = ret
                    _qvel_last = None
                else:
                    raise RuntimeError(f"Unexpected retarget() return length: {len(ret)}")
            else:
                raise RuntimeError("retarget() should return a tuple")

            # dt：优先取 retarget.last_frame_dt；没有就用目标帧时长
            dt_this = getattr(retargeter, "last_frame_dt", None)
            if not (isinstance(dt_this, (float, np.floating)) and np.isfinite(dt_this) and dt_this > 0.0):
                dt_this = target_dt

            # 获取机器人对应连杆名称列表
            robot_frames = retargeter.ik_match_table1.keys()
            
            # 可视化
            robot_motion_viewer.step(
                root_pos=qpos[:3],
                root_rot=qpos[3:7],
                dof_pos=qpos[7:],
                human_motion_data=retargeter.scaled_human_data,
                human_pos_offset=np.array([0.0, 0.0, 0.0]),
                show_human_body_name=False,
                robot_frames=robot_frames,
                show_robot_body_name=False,
                rate_limit=args.rate_limit,
                # human_pos_offset=np.array([0.0, 0.0, 0.0])
            )
            time.sleep(0.02)

            if args.loop:
                i = (i + 1) % len(lafan1_data_frames)
            else:
                i += 1
                if i >= len(lafan1_data_frames):
                    break
   
            if args.save_path is not None:
                qpos_list.append(qpos)
                qvel_list.append(qvel)
                frame_dt_list.append(float(dt_this))  # 区间 (i-1 -> i) 的 dt
    
    finally:
        # —— 确保渲染与录制干净关闭，避免 GLXBadContext / segfault —— #
        try:
            if getattr(robot_motion_viewer, "stop_recording", None) and args.record_video:
                robot_motion_viewer.stop_recording()
        except Exception as e:
            print(f"[WARN] stop_recording failed: {e}")
        try:
            if getattr(robot_motion_viewer, "close", None):
                robot_motion_viewer.close()
            elif getattr(robot_motion_viewer, "destroy", None):
                robot_motion_viewer.destroy()
        except Exception as e:
            print(f"[WARN] viewer close failed: {e}")
        # 给后台线程（渲染/视频写入）一点收尾时间
        time.sleep(0.05)
    
    # Close progress bar
    pbar.close()
    
    # ====== 导出 ======
    if args.save_path is not None:
        import pickle
        from pathlib import Path

        # 1) 堆成数组
        qpos_arr = np.asarray(qpos_list)
        qvel_arr = np.asarray(qvel_list)
        frame_dt_arr = np.asarray(frame_dt_list, dtype=np.float32)
        # 说明：frame_dt_arr[0] = 0.0（占位）；区间 (i-1->i) 用 frame_dt_arr[i]

        # —— 导出前做逐帧校验（按每帧自己的 dt）——
        try:
            check_exported_root_velocities(
                qpos_seq=qpos_arr,          # (T, 7+Nd)
                qvel_seq=qvel_arr,          # (T, 6+Nd)
                dt_list=frame_dt_arr,       # (T+1,) 但我们只用 1..T 区间
                lin_tol=getattr(args, "lin_tol", 5e-3),
                ang_tol=getattr(args, "ang_tol", 5e-3),
                strict=getattr(args, "vel_check_strict", False),
            )
        except Exception as e:
            print(f"[ERROR] export-time velocity validation failed: {e}")
            # 如需硬失败可改为 raise

        # 2) 拆分（保持你当前的索引解释方式）
        root_pos = qpos_arr[:, :3]                                 # (T,3)
        root_rot_wxyz = qpos_arr[:, 3:7]                           # (T,4) wxyz
        root_rot_xyzw = root_rot_wxyz[:, [1, 2, 3, 0]]             # (T,4) xyzw
        dof_pos  = qpos_arr[:, 7:]                                 # (T,Nd)

        # qvel 0:3 线速(world), 3:6 角速(body)
        root_vel_world = qvel_arr[:, 0:3]                          # (T,3) 世界系线速度
        root_rot_vel   = qvel_arr[:, 3:6]                          # (T,3) 角速度（body）
        dof_vel        = qvel_arr[:, 6:]                           # (T,Nd)

        # 3) 长度对齐（以最短为准）(去前 5 帧异常）
        L = min(root_pos.shape[0], root_rot_xyzw.shape[0], dof_pos.shape[0],
                root_vel_world.shape[0], root_rot_vel.shape[0], dof_vel.shape[0], frame_dt_arr.shape[0])
        if L > 10:  # 只有帧数足够时才去掉前5帧
            root_pos       = root_pos[30:L].astype(np.float32)
            root_rot_wxyz  = root_rot_wxyz[30:L].astype(np.float32)
            root_rot_xyzw  = root_rot_xyzw[30:L].astype(np.float32)
            dof_pos        = dof_pos[30:L].astype(np.float32)
            root_vel_world = root_vel_world[30:L].astype(np.float32)
            root_rot_vel   = root_rot_vel[30:L].astype(np.float32)
            dof_vel        = dof_vel[30:L].astype(np.float32)
            frame_dt_arr   = frame_dt_arr[30:L].astype(np.float32)
        else:
            # 如果帧数太少，使用所有帧
            root_pos       = root_pos.astype(np.float32)
            root_rot_wxyz  = root_rot_wxyz.astype(np.float32)
            root_rot_xyzw  = root_rot_xyzw.astype(np.float32)
            dof_pos        = dof_pos.astype(np.float32)
            root_vel_world = root_vel_world.astype(np.float32)
            root_rot_vel   = root_rot_vel.astype(np.float32)
            dof_vel        = dof_vel.astype(np.float32)
            frame_dt_arr   = frame_dt_arr.astype(np.float32)




        T, D = dof_pos.shape

        num_interp = 30  # 插值帧数
        default_dof = default_dof.reshape(1, -1)  # (1, D)

        # ========== 1️⃣ 生成开头插值 ==========
        start = default_dof
        end = dof_pos[0:1]  # 第一帧

        alphas = np.linspace(0.0, 1.0, num_interp, endpoint=False).reshape(-1, 1)
        dof_start_interp = start * (1 - alphas) + end * alphas  # (num_interp, D)

        # ========== 2️⃣ 生成结尾插值 ==========
        start = dof_pos[-1:]
        end = default_dof

        alphas = np.linspace(0.0, 1.0, num_interp, endpoint=False).reshape(-1, 1)
        dof_end_interp = start * (1 - alphas) + end * alphas  # (num_interp, D)

        # ========== 3️⃣ 拼接 dof ==========
        dof_pos = np.concatenate(
            [dof_start_interp, dof_pos, dof_end_interp],
            axis=0
        )



        root_pos_start = np.repeat(root_pos[0:1], num_interp, axis=0)
        root_rot_start = np.repeat(root_rot_xyzw[0:1], num_interp, axis=0)
        root_rot_wxyz_start = np.repeat(root_rot_wxyz[0:1], num_interp, axis=0)
        root_vel_world_start = np.repeat(root_vel_world[0:1], num_interp, axis=0)
        root_rot_vel_start = np.repeat(root_rot_vel[0:1], num_interp, axis=0)
        dof_vel_start = np.repeat(dof_vel[0:1], num_interp, axis=0)

        # 复制最后一帧
        root_pos_end = np.repeat(root_pos[-1:], num_interp, axis=0)
        root_rot_end = np.repeat(root_rot_xyzw[-1:], num_interp, axis=0)
        root_rot_wxyz_end = np.repeat(root_rot_wxyz[-1:], num_interp, axis=0)
        root_vel_world_end = np.repeat(root_vel_world[-1:], num_interp, axis=0)
        root_rot_vel_end = np.repeat(root_rot_vel[-1:], num_interp, axis=0)
        dof_vel_end = np.repeat(dof_vel[-1:], num_interp, axis=0)

        # 拼接
        root_pos = np.concatenate(
            [root_pos_start, root_pos, root_pos_end],
            axis=0
        )

        root_rot_xyzw = np.concatenate(
            [root_rot_start, root_rot_xyzw, root_rot_end],
            axis=0
        )

        root_rot_wxyz = np.concatenate(
            [root_rot_wxyz_start, root_rot_wxyz, root_rot_wxyz_end],
            axis=0
        )
        
        root_vel_world = np.concatenate(
            [root_vel_world_start, root_vel_world, root_vel_world_end],
            axis=0
        )

        root_rot_vel = np.concatenate(
            [root_rot_vel_start, root_rot_vel, root_rot_vel_end],
            axis=0
        )

        dof_vel = np.concatenate(
            [dof_vel_start, dof_vel, dof_vel_end],
            axis=0
        )

        # 4) 计算 root_vel_body：v_body = R^T * v_world （R: body->world, 由 wxyz 四元数得到）
        R_bw = R.from_quat(root_rot_wxyz, scalar_first=True).as_matrix()   # (T,3,3) body->world
        R_wb = np.transpose(R_bw, (0, 2, 1))                               # (T,3,3) world->body
        root_vel_body = np.einsum('tij,tj->ti', R_wb, root_vel_world).astype(np.float32)  # (T,3)
        # lin_vel_world = np.gradient(dof_pos, 0.02, axis=0).astype(np.float32)
        lin_vel_world = np.gradient(root_pos, 0.02, axis=0).astype(np.float32)
        np.set_printoptions(threshold=np.inf)

        print("root_vel_world: ", root_vel_world[:200, 0])
        print("lin_vel_world: ", lin_vel_world[:200, 0])
        device = "cuda:0"
        kinematics_model = KinematicsModel(retargeter.xml_file, device=device)


        # root_pos = qpos_list[:, :3]
        # root_rot = qpos_list[:, 3:7]
        # root_rot[:, [0, 1, 2, 3]] = root_rot[:, [1, 2, 3, 0]]
        # dof_pos = qpos_list[:, 7:]
        num_frames = root_pos.shape[0]
        
        # obtain local body pos
        identity_root_pos = torch.zeros((num_frames, 3), device=device)
        identity_root_rot = torch.zeros((num_frames, 4), device=device)
        identity_root_rot[:, -1] = 1.0
        local_body_pos, local_body_rot = kinematics_model.forward_kinematics(
            identity_root_pos, 
            identity_root_rot, 
            torch.from_numpy(dof_pos).to(device=device, dtype=torch.float)
        )
        body_names = kinematics_model.body_names

        HEIGHT_ADJUST = True
        PERFRAME_ADJUST = True
        if HEIGHT_ADJUST:
            body_pos, _ = kinematics_model.forward_kinematics(
                torch.from_numpy(root_pos).to(device=device, dtype=torch.float),
                torch.from_numpy(root_rot_xyzw).to(device=device, dtype=torch.float),
                torch.from_numpy(dof_pos).to(device=device, dtype=torch.float)
            )
            ground_offset = 0.00
            if not PERFRAME_ADJUST:
                lowest_height = torch.min(body_pos[..., 2]).item()
                root_pos[:, 2] = root_pos[:, 2] - lowest_height + ground_offset
            else:
                for i in range(root_pos.shape[0]):
                    lowest_body_part = torch.min(body_pos[i, :, 2])
                    root_pos[i, 2] = root_pos[i, 2] - lowest_body_part + ground_offset



    r = R.from_quat(root_rot_xyzw)
    ang_vel_body = r.inv().apply(root_rot_vel)

    # link_pos_base=local_body_pos[:,[2,4,6,11,13,15,19,22,23,24,27,28,29]].detach().cpu().numpy()
    # link_rot_base=local_body_rot[:,[2,4,6,11,13,15,19,22,23,24,27,28,29]].detach().cpu().numpy(),
    # world_pos = transform_to_world_from_quat(link_pos_base, root_rot_xyzw, root_pos)



    # left_contact = generate_contact_sequence(world_pos[:,2,:])
    # right_contact = generate_contact_sequence(world_pos[:,5,:])
    
    # motion_data = {
    #         # "fps": aligned_fps,
    #         "root_trans": root_pos,
    #         "root_rot": root_rot_xyzw,
    #         "base_lin_vel": root_vel_body,
    #         "base_ang_vel": ang_vel_body,
    #         "dof_pos": dof_pos,
    #         "dof_vel": dof_vel,
    #         "local_body_pos": local_body_pos.detach().cpu().numpy(),
    #         "link_rot_base": local_body_rot.detach().cpu().numpy(),
    #         # "link_body_list": body_names,
    #     }



        # os.makedirs(os.path.dirname(tgt_file_path), exist_ok=True)
        # with open(tgt_file_path, "wb") as f:
        #     pickle.dump(motion_data, f)

        # print(type(motion_data))
        # save_file = save_file.with_suffix(".npz")

    # dof_pos[1077+30:1120+30,4] = 0    
        
    np.savez_compressed(
        args.save_path,
        root_trans=root_pos[:],
        root_ori=root_rot_xyzw[:],
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
    "root_ori": root_rot_xyzw[:],
    "base_lin_vel": root_vel_body[:],
    "base_ang_vel": ang_vel_body[:],
    "dof_pos": dof_pos[:],
    "dof_vel": dof_vel[:],
    "link_pos_base": local_body_pos[:,[2,4,6,11,13,15,19,22,23,24,27,28,29]].detach().cpu().numpy(),
    "link_rot_base": local_body_rot[:,[2,4,6,11,13,15,19,22,23,24,27,28,29]].detach().cpu().numpy(),
}
    yaml_path = os.path.splitext(args.save_path)[0] + ".yaml"

    # ⚠️ YAML 不支持 numpy array，需要转成 list
    yaml_dict = {k: v.tolist() for k, v in data_dict.items()}

    with open(yaml_path, "w") as f:
        yaml.safe_dump(yaml_dict, f)

    print(f"Saved to {args.save_path}")
    

        # # 5) 构造导出数据格式
        # seq_name = Path(args.bvh_file).stem
        # export_data = {
        #     "motion_file": seq_name,
        #     "root_pos": root_pos,                 # (T,3) world
        #     "root_vel": root_vel_world,           # (T,3) world（兼容字段）
        #     "root_vel_body": root_vel_body,       # (T,3) body（新增）
        #     "root_rot": root_rot_xyzw,            # (T,4) xyzw
        #     "root_rot_vel": root_rot_vel,         # (T,3) body
        #     "dof_pos": dof_pos,                   # (T,Nd)
        #     "dof_vel": dof_vel,                   # (T,Nd)
        #     "local_body_pos": local_body_pos,
        #     "link_body_list": None,
        #     "fps": float(aligned_fps),
        #     "meta": {
        #         "root_rot_convention": "xyzw",
        #         "root_ang_vel_space": "local",      # 体坐标
        #         "root_lin_vel_space_world": "world", # 线速度（root_vel）的坐标系
        #         "root_lin_vel_space_body": "local",  # 线速度（root_vel_body）的坐标系
        #         # "frame_dt_per_step": frame_dt_arr    # (T,) 每帧累计子步时长（与区间对齐）
        #     },
        # }

        # with open(args.save_path, "wb") as f:
        #     pickle.dump(export_data, f)
        # print(f"Saved to {args.save_path}")
