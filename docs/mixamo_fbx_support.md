# Mixamo FBX → GMR 支持记录

本文档记录将 **Mixamo FBX 动画数据**接入 GMR 重定向流程的完整过程、踩过的坑与最终方案。
针对 G1 (`unitree_g1`) 与 K1 (`booster_k1`) 均已验证。

## 一、数据流概览

```
Mixamo FBX (*.fbx)
  │  (fbx 环境, 需 Autodesk FBX Python SDK)
  ▼
third_party/poselib/fbx_importer.py --root-joint Hips --fps 60
  │  输出 pkl: 每帧 dict { joint_name: [pos(m), quat(wxyz)] }
  ▼
scripts/fbx_offline_to_robot.py --robot <robot> --src_human fbx_mixamo
  │  输出 pkl: { root_pos, root_rot(xyzw), dof_pos, ... }
  ▼
机器人运动数据
```

## 二、遇到的问题与解决

### 1. 缺少 `FbxCommon.py`（FBX 导入失败）

**现象**: `No module named 'FbxCommon'`

**原因**: PyPI 的 `fbx` wheel（2020.3.10）只包含编译好的 `fbx.cpython-311...so`，
不包含官方 SDK 中的 `FbxCommon.py` 包装模块（提供 `InitializeSdkObjects()` / `LoadScene()`）。

**解决**: 在 fbx 环境的 site-packages 手动创建 `FbxCommon.py`（标准 Autodesk 包装）。

### 2. FBX SDK 版本 API 变化

**现象**: `TypeError: GetFrameCount(self, FbxTime.EMode = ...): argument 1 has unexpected type 'bool'`

**原因**: poselib 的 `fbx_backend.py` 针对 FBX SDK 2020.2.1 编写，调用
`duration.GetFrameCount(True)`（旧版 bool 参数）；SDK 2020.3.10 改为 `EMode` 枚举。

**解决**: 改为 `duration.GetFrameCount(duration.GetGlobalTimeMode())`。

### 3. Mixamo 关节命名（`mixamorig:` 冒号前缀）

**现象**: `IndexError: list index out of range`（`to_retarget_motion_file` 导出 pkl 时）

**原因**: poselib 的 `to_retarget_motion_file()` 用 `name.split('_')[1]` 提取关节名，
只支持 `mixamorig_Hips`（下划线）风格；Mixamo 实际导出 `mixamorig:Hips`（冒号）。

**解决**: 改为 `name.replace(':', '_').split('_')[1]`。

### 4. `booster_k1` 未注册到 FBX 流程

**现象**: `argument --robot: invalid choice: 'booster_k1'`

**原因**: `scripts/fbx_offline_to_robot.py` 的 `--robot` choices 是硬编码的旧列表；
`params.py` 的 `fbx_offline` 字典也未注册 K1 的 config。

**解决**: 注册 `fbx_offline_to_k1.json` 并补充 choices；顺带新增 `--fps` 参数
（原脚本硬编码 120，而 Mixamo 数据以 60 fps 导出）。

### 5. 旋转对了但肢体是 T-pose、腿部扭曲（核心问题）

**现象**: 重定向后 yaw 旋转正确（约 90°），但 K1 呈 T-pose 站姿、手臂水平、
双腿严重扭曲。

**根因（两层）**:

**(a) poselib FK 丢失骨骼方向**：原 `fbx_backend.py` 用 `EvaluateLocalTransform()`
读取局部变换再做 FK。Mixamo 骨架的骨骼方向隐含在**平移**中（bind pose 时
旋转=0），局部旋转只含动画增量，导致 FK 出的全局旋转与 FBX 真实值差 100-133°。

**(b) offset 推导参考错误**：K1 的 `qpos=0` 是 **T-pose**（手臂水平，`Left_ShR=0`），
而 Mixamo frame-0 是自然站立（手臂下垂）。用 frame-0 推导 offset 时，
`q_offset = q_human(f0)⁻¹ * q_robot_default` 把"自然站"映射到"T-pose"，
IK 收敛后机器人保持 T-pose。G1 恰好 `qpos=0` 就是自然站，所以早期对 G1 有效、
对 K1 无效。

**解决（两步）**:

1. **修改 `fbx_backend.py` 改用全局变换**：用 `EvaluateGlobalTransform()` 读取，
   再通过父节点全局矩阵的相对运算还原局部变换（保持 poselib 的 FK 约定）：
   - FBX 返回的矩阵是列主序（row-major 视图下平移在最后一行），需先转置；
   - `local = inv(parent_global) @ child_global`，再转置回列主序输出。
   - 验证：修复后 pkl 与 FBX SDK 直接读取差恒为 90°（即 y-up→z-up 转换本身），
     数据正确。

2. **从 K1 自然站姿态推导 offset**（而非 qpos=0）：手工构造 K1 自然站 qpos，
   关键关节角（qpos 索引）：
   ```
   qpos[10] = -1.57   # Left_Shoulder_Roll: 手臂从水平(+Y)放下(-Z)
   qpos[14] = 1.57    # Right_Shoulder_Roll
   qpos[11] = 0.0     # Left_Elbow_Pitch
   qpos[15] = 0.0     # Right_Elbow_Pitch
   ```
   然后 `q_offset = q_human(f0)⁻¹ * q_robot(natural_stand)`。

**为什么 G1 也要用 frame-0 推导**: G1 `qpos=0` 恰好是自然站（手臂下垂），
frame-0 推导天然正确。

### 6. K1 手臂方向偏差（24.6°，可接受）

**现象**: 重定向后 K1 前臂方向与人体有约 25° 偏差（大腿仅 5°）。

**原因**: K1 是 **4-DOF 手臂**（ShP/ShR/ElP/ElY），无法完全匹配 Mixamo 人体
自由 3D 前臂朝向；`Left_Shoulder_Roll` 长期贴在下限 -1.74。这是 K1 硬件
（关节构型/限位）限制，不是配置错误。G1 的 7-DOF 手臂也有约 27° 偏差，
同为构型限制。

## 三、最终产物

| 文件 | 说明 |
| --- | --- |
| `ik_configs/fbx_offline_mixamo_to_g1.json` | Mixamo → G1 IK 配置 |
| `ik_configs/fbx_offline_mixamo_to_k1.json` | Mixamo → K1 IK 配置 |
| `params.py` | 新增 `fbx_mixamo` 源类型注册上述配置 |
| `scripts/fbx_offline_to_robot.py` | 新增 `--src_human fbx_mixamo` 与 `--fps` |
| `poselib/.../fbx_backend.py` | 改用 `EvaluateGlobalTransform`（保留骨骼方向） |

## 四、使用方式

```bash
# 1. FBX → pkl（fbx 环境，需 FBX SDK）
conda activate fbx
python third_party/poselib/fbx_importer.py \
  --input <path.fbx> --output <path.pkl> --root-joint Hips --fps 60

# 2. pkl → 机器人运动（gmr 环境）
conda activate gmr
python scripts/fbx_offline_to_robot.py \
  --motion_file <path.pkl> --robot booster_k1 \
  --src_human fbx_mixamo --save_path <out.pkl> \
  --rate_limit --fps 60
```

> ⚠️ 注意：
> - Mixamo 下载时选 **FBX Binary** + **Without Skin** + 帧率与 `--fps` 一致；
> - 不要勾选 Mixamo 的 **In Place**（会去掉 root motion，机器人原地不动）；
> - OptiTrack 数据继续用默认 `--src_human fbx_offline`，不受影响。

## 五、验证结果（Left Turn 90, 56 帧 @ 60fps）

| 机器人 | root yaw 旋转 | IK 误差 | 肢体状态 |
| --- | --- | --- | --- |
| Booster K1 | +90.6° | 0.975 | 手臂下垂、腿部自然、全部关节在限位内 |
| Unitree G1 | +93.5° | ~1.3 | 手臂下垂、腿部自然 |

## 六、相关推导方法

通用 rot_offset 推导见 [`rot_offset_derivation.md`](./rot_offset_derivation.md)。
本文的 K1 方案可视为其特例：**参考姿态必须取机器人"自然站"而不是 qpos=0**，
当目标机器人默认姿态不是自然站时尤其重要。
