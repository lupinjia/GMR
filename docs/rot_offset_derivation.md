# rot_offset（姿态偏移）推导方法

本文档总结如何为**新的人体数据格式 × 机器人**组合推导 IK 配置中的 `rot_offset`
（姿态偏移四元数），并以 `bvh_lafan1_to_k1.json` 的实际推导过程为例。

## 一、背景与核心思想

在 IK 配置的 `ik_match_table1/2` 中，每个匹配项的第 ⑤ 个字段是 `rot_offset`（wxyz，
scalar-first）。它把人体 body 的朝向变换到机器人连杆的朝向：

```python
# motion_retarget.py -> offset_human_data()
updated_quat = (R.from_quat(quat, scalar_first=True) * rot_offsets[body_name]).as_quat(scalar_first=True)
```

即 `q_robot_link = q_human_body * q_offset`（四元数**右乘**）。

`rot_offset` 同时依赖两个因素：

1. **人体数据格式**（SMPL-X / LAFAN1 / FBX 的坐标系与朝向约定不同）；
2. **机器人连杆**（不同机器人的连杆坐标系朝向约定不同）。

因此不能直接把 A 格式的 offset 用于 B 格式，也不能直接把机器人 X 的 offset 用于机器人 Y。

**核心洞察**：对于同一个机器人 pair（如 K1 vs G1），它们对**同一人体 body** 的
offset 之差 `bias` 只反映**两个机器人连杆朝向约定的差异**，与人体数据格式无关。
因此可以把该 `bias` 从一种人体格式迁移到另一种人体格式。

## 二、推导公式

假设已有三份配置：

- `X_to_A.json`：人体格式 X → 机器人 A（如 `smplx_to_g1.json`）
- `X_to_B.json`：人体格式 X → 机器人 B（如 `smplx_to_k1.json`）
- `Y_to_A.json`：人体格式 Y → 机器人 A（如 `bvh_lafan1_to_g1.json`）

目标：推导 `Y_to_B.json` 中每个连杆的 `rot_offset`。

**第 1 步：计算连杆偏置**（对每个两个机器人配置**共享的人体 body**）：

```
bias(body) = q_off_X_to_A(body)⁻¹ * q_off_X_to_B(body)
```

其中 `q_off` 为对应配置中该人体 body 的 rot_offset（scipy 四元数组合，`inv *` 表示
"从 A 连杆朝向转到 B 连杆朝向"的旋转）。

**第 2 步：迁移到新人体格式**：

```
q_off_Y_to_B(body) = q_off_Y_to_A(body) * bias(body)
                   = q_off_Y_to_A(body) * q_off_X_to_A(body)⁻¹ * q_off_X_to_B(body)
```

### 推导验证

`q_robotA = q_human_X * q_off_X_to_A`，`q_robotB = q_human_X * q_off_X_to_B`，
则 `q_robotB = q_robotA * bias`（`bias = q_off_X_to_A⁻¹ * q_off_X_to_B`）。
对格式 Y 施加同样的连杆偏置：`q_off_Y_to_B = q_off_Y_to_A * bias`，即保证"同一
连杆朝向约定差"在两种人体格式下都被正确补偿。✅

## 三、实际操作步骤

1. 从 `ik_match_table1`（或 table2）提取每个配置的「人体 body → rot_offset」映射；
2. 取两个 X 格式配置的**共享人体 body** 集合；
3. 对每个共享 body 计算 `bias`；
4. 对每个 Y 格式配置中出现的 body：`new = q_off_Y_to_A * bias`；
5. 将 `new` 写入 `Y_to_B.json` 的对应条目（table1/table2 都要写）。

> 实现参考（scipy）：
>
> ```python
> from scipy.spatial.transform import Rotation as R
>
> g1_smplx = {body: R.from_quat(off, scalar_first=True) for body, off in ...}  # X_to_A
> k1_smplx = {body: R.from_quat(off, scalar_first=True) for body, off in ...}  # X_to_B
> g1_laf   = {body: R.from_quat(off, scalar_first=True) for body, off in ...}  # Y_to_A
>
> for body in g1_laf:  # 仅对共享 body
>     bias = g1_smplx[body].inv() * k1_smplx[body]
>     new  = g1_laf[body] * bias
>     new_offset = new.as_quat(scalar_first=True)  # wxyz
> ```

## 四、实例：bvh_lafan1_to_k1.json 的推导

输入配置：

| 角色 | 配置 | 人体格式 | 机器人 |
| --- | --- | --- | --- |
| X_to_A | `smplx_to_g1.json` | SMPL-X | G1 |
| X_to_B | `smplx_to_k1.json` | SMPL-X | K1 |
| Y_to_A | `bvh_lafan1_to_g1.json` | LAFAN1 | G1 |
| 输出 | `bvh_lafan1_to_k1.json` | LAFAN1 | K1 |

人体 body 对应关系（SMPL-X ↔ LAFAN1 命名）：

| SMPL-X | LAFAN1 | K1 匹配的连杆 |
| --- | --- | --- |
| `spine3` | `Spine2` | `Trunk` |
| `left_hip` / `right_hip` | `LeftUpLeg` / `RightUpLeg` | `Left_Hip_Yaw` / `Right_Hip_Yaw` |
| `left_knee` / `right_knee` | `LeftLeg` / `RightLeg` | `Left_Shank` / `Right_Shank` |
| `left_foot` / `right_foot` | `LeftFootMod` / `RightFootMod` | `left_foot_link` / `right_foot_link` |
| `left_shoulder` / `right_shoulder` | `LeftArm` / `RightArm` | `Left_Arm_3` / `Right_Arm_3` |
| `left_elbow` / `right_elbow` | `LeftForeArm` / `RightForeArm` | `Left_Arm_4`(表1)/`Left_Hand_link`(表2) 等 |

### 计算得到的连杆偏置 bias（K1 相对 G1）

| SMPL-X body | bias（欧拉角，xyz，度） | 说明 |
| --- | --- | --- |
| `spine3` / `left_knee` / `right_knee` / `left_foot` / `right_foot` | `[0, 0, 0]` | 无偏置，K1 与 G1 约定一致 |
| `left_hip` / `right_hip` | `[0, 15.8, 0]` | K1 `Left_Hip_Yaw` 与 G1 `left_hip_roll_link` 相差约 16° yaw |
| `left_shoulder` / `right_shoulder` | `[∓90, 0, 0]` | K1 4-DOF 手臂与 G1 7-DOF 手臂连杆约定差异大 |
| `left_elbow` / `right_elbow` | 约 ±90°/120° | 同上，肘部连杆约定差异最大 |

### 最终 bvh_lafan1_to_k1.json 的 rot_offset（wxyz）

| human body | rot_offset |
| --- | --- |
| `Spine2`（Trunk） | `[0.5, 0.5, 0.5, 0.5]` |
| `LeftUpLeg` / `RightUpLeg` | `[0.56379311, -0.56379311, -0.4267755, 0.4267755]` |
| `LeftLeg` / `RightLeg` | `[0.5, -0.5, -0.5, 0.5]`（与 `[-0.5, 0.5, 0.5, -0.5]` 等价，q ≡ -q） |
| `LeftArm` / `RightArm` | 左 `[-0.70710678, 0.0, 0.0, 0.70710678]`；右 `[0.0, 0.70710678, -0.70710678, 0.0]` |
| `LeftForeArm` / `RightForeArm` | 左 `[-0.70710678, 0.0, 0.0, 0.70710678]`；右 `[0.0, -0.70710678, 0.70710678, 0.0]` |
| `LeftFootMod` / `RightFootMod` | `[0.70710678, -0.70710678, 0.0, 0.0]` |

> ⚠️ 注意**左右手不对称**：`LeftArm` 与 `RightArm`、`LeftForeArm` 与 `RightForeArm`
> 的计算值不同（G1 配置的左右肩/肘 rot_offset 本就镜像对称，经 bias 变换后保持镜像），
> 修改时左右必须分别填写。

## 五、注意事项

1. **只对共享人体 body 有效**：若某个 body 只存在于其中一个 X 格式配置
   （如 K1 有 `head` 而 G1 配置无），无法推导 bias。此时可保持原值或按同类
   body 近似（如 `Head` 取 `Spine2` 的值），并在注释/文档中说明。
2. **四元数符号等价**：`q` 与 `-q` 表示同一旋转。推导结果可能相对手工值出现
   整体符号翻转，这不影响结果（scipy / mink 内部会归一化），但对比 diff 时需注意。
3. **精度校验**：推导完成后可用
   `angle = (stored.inv() * expected).magnitude()` 校验每个条目与公式值的一致
   程度（实例中最大偏差约 1e-7°，纯浮点误差）。
4. **手臂差异过大时需复核结构**：若某两个机器人的关节数/连杆结构差异很大
   （如 G1 7-DOF 手臂 vs K1 4-DOF 手臂），rot_offset 虽可推导，但重定向效果
   可能仍不理想——此时应考虑更换匹配的连杆（如用 `Left_Arm_2`/`Left_Arm_1`），
   而不只是调整 offset。
5. **权重与 pos_offset 不参与推导**：本方法只推导第 ⑤ 字段 `rot_offset`。
   `human_scale_table`、权重、`pos_offset` 应另行按目标机器人的现有配置设置。
