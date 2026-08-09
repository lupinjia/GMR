# smplx_to_k1.json 参数详解

本文档解析 `general_motion_retargeting/ik_configs/smplx_to_k1.json` 中每一个参数的
含义与作用。该配置文件用于将 **SMPL-X 人体动作** 重定向到 **Booster K1 机器人**
（22 DoF），是 `GMR(src_human="smplx", tgt_robot="booster_k1")` 实际加载的 IK 配置。

## 一、参数总览

| 参数 | 当前值 | 一句话作用 |
| --- | --- | --- |
| `robot_root_name` | `"Trunk"` | 机器人 MuJoCo 模型中的根 body 名 |
| `human_root_name` | `"spine3"` | 人体数据缩放的根节点（基准点） |
| `ground_height` | `0.0` | 地面在机器人坐标系中的高度 |
| `human_height_assumption` | `1.8` | 配置所假定的真人身高（米） |
| `use_ik_match_table1` | `true` | 是否启用第一阶段 IK 求解 |
| `use_ik_match_table2` | `true` | 是否启用第二阶段 IK 求解 |
| `human_scale_table` | 全部 `0.6` | 人体各部位位置缩放系数 |
| `ik_match_table1` | 14 项 | 第一阶段：机器人 body ↔ 人体 body 匹配表 |
| `ik_match_table2` | 14 项 | 第二阶段：机器人 body ↔ 人体 body 匹配表 |

> 说明：`solver`、`damping`、`use_velocity_limit` 等参数**不在** JSON 中，它们是
> `GeneralMotionRetargeting` 类的构造参数，由调用方直接传入。

## 二、顶层参数

### 1. `robot_root_name`（当前：`"Trunk"`）

机器人在 MuJoCo 模型（`assets/booster_k1/K1_serial.xml`）中的根 body 名称。
K1 的根 body 是 `Trunk`，与 `params.py` 中 `ROBOT_BASE_DICT["booster_k1"] = "Trunk"`
保持一致。重定向输出的 `root_pos` / `root_rot`（即 `qpos[:3]` 与 `qpos[3:7]`）
就是该根 body 的自由关节（free joint）位姿。

### 2. `human_root_name`（当前：`"spine3"`）

人体数据的**缩放基准根节点**，必须是输入人体数据中存在的 body 名。
在 `scale_human_data()`（`motion_retarget.py:243`）中：

- 该节点的位置会被整体缩放：`scaled_root_pos = human_scale_table[human_root_name] * root_pos`
- 其他所有部位先变换到以它为原点的局部系再缩放：
  `(body_pos - root_pos) * scale`

> ⚠️ **约束**：`human_root_name` 必须出现在 `human_scale_table` 中，否则
> `scale_human_data()` 第 249 行会 `KeyError`。当前值 `"spine3"` 已包含在表中。
>
> K1 配置将其设为 `"spine3"`（脊柱顶段），配合 `ik_match_table` 中
> `"Trunk" → "spine3"`，使机器人躯干直接跟随 SMPL-X 的 spine3，而不是 pelvis。

### 3. `ground_height`（当前：`0.0`）

地面在机器人坐标系中的高度（米）。在 `motion_retarget.py:81` 中被转为
`self.ground = ground_height * [0, 0, 1]`，随后从每个匹配项的位置偏移中减去
（第 124、142 行：`np.array(pos_offset) - self.ground`）。K1 的地面即
`z = 0`，故为 `0.0`。

### 4. `human_height_assumption`（当前：`1.8`）

配置文件假定的真人身高（米），是重定向的核心标定值之一。
在 `motion_retarget.py:63-70` 中，当传入 `actual_human_height` 时：

```python
ratio = actual_human_height / ik_config["human_height_assumption"]
for key in ik_config["human_scale_table"].keys():
    ik_config["human_scale_table"][key] *= ratio
```

即：**整个人体缩放表会按实际身高 / 假定身高 的比例整体缩放**。若未传入
`actual_human_height`，`ratio = 1.0`。1.8 米也是 SMPL-X 中性体型（neutral）
的默认身高。

### 5. `use_ik_match_table1` / `use_ik_match_table2`（当前：均 `true`）

两个布尔开关，控制两阶段 IK 中每一阶段是否执行（`motion_retarget.py:160/166/177/197`）：

- `use_ik_match_table1 = true`：第一阶段用 `ik_match_table1` 求解；
- `use_ik_match_table2 = true`：第二阶段用 `ik_match_table2` 在当前位形上继续求解。

两阶段策略的作用：第一阶段以一组权重快速逼近全身目标位姿，第二阶段再以
另一组（通常更强调脚部）权重进行精修，从而同时兼顾"整体姿态"与"脚部落地"。

## 三、`human_scale_table`（人体缩放表）

`human_root_name` → 缩放系数的映射，**仅缩放位置、不改变旋转**。
K1 配置中所有部位均为 `0.6`（K1 尺寸约为 1.8 m 人体模型的 60%）：

```json
"human_scale_table": {
    "head": 0.6, "spine3": 0.6,
    "left_hip": 0.6, "right_hip": 0.6,
    "left_knee": 0.6, "right_knee": 0.6,
    "left_foot": 0.6, "right_foot": 0.6,
    "left_shoulder": 0.6, "right_shoulder": 0.6,
    "left_elbow": 0.6, "right_elbow": 0.6
}
```

**缩放公式**（`scale_human_data()`，第 243-266 行）：

- 根节点：`scaled_root_pos = scale[root] * root_pos`
- 其他部位：`(body_pos - root_pos) * scale[body] + scaled_root_pos`

> ⚠️ **关键约束**：`scale_human_data()` 会**丢弃不在缩放表中的人体部位**
> （第 253 行 `continue`）。因此：
> 1. `ik_match_table` 中引用的**每个人体 body 名**都必须在 `human_scale_table` 中，
>    否则 `update_targets()` 第 163/169 行 `human_data[body_name]` 会 `KeyError`；
> 2. 只有缩放表中列出的部位会参与后续 IK 匹配（其余部位不会传给任务）。

K1 的 `ik_match_table` 引用的人体 body 为：`right_knee / left_knee /
right_hip / left_hip / spine3 / right_elbow / left_elbow / right_shoulder /
left_shoulder / right_foot / left_foot / head`，与缩放表完全对应。

## 四、`ik_match_table1` / `ik_match_table2`（IK 匹配表）

两者的结构完全相同，均为「机器人 body 名 → 匹配参数数组」的映射，例如：

```json
"Trunk": [
    "spine3",          // ① 对应的人体 body 名
    0,                 // ② 位置跟踪权重 (position_cost)
    10,                // ③ 姿态跟踪权重 (orientation_cost)
    [0, 0, 0],         // ④ 位置偏移 (x, y, z)，单位米
    [-0.5, 0.5, 0.5, 0.5]  // ⑤ 姿态偏移，四元数，wxyz（scalar first）
]
```

### 各字段含义

| 字段 | 含义 | K1 取值 |
| --- | --- | --- |
| ① 人体 body 名 | 目标取自人体数据的哪个部位 | 见下表 |
| ② 位置权重 | 跟踪该点 3D 位置的成本权重（`position_cost`） | 表1 多为 0，表2 多为 10，脚部 100 |
| ③ 姿态权重 | 跟踪该点 3D 旋转的成本权重（`orientation_cost`） | 表1 多为 10，表2 多为 5，脚部 50 |
| ④ 位置偏移 | 施加到人体目标点上的固定位移，在旋转偏移后的局部系下应用 | 脚部有 ±0.02 的横向微调，其余 0 |
| ⑤ 姿态偏移 | 施加到人体目标旋转上的固定四元数旋转，用于对齐 SMPL-X 坐标系与机器人坐标系 | 全身统一 `[-0.5, 0.5, 0.5, 0.5]` |

### 处理流程（`setup_retarget_configuration()`，第 113-147 行）

对每个匹配项，若 `pos_weight != 0 or rot_weight != 0`，则创建
`mink.FrameTask(frame_name=机器人body名, frame_type="body", position_cost,
orientation_cost)`，并记录：

- `pos_offsets[人体body名] = pos_offset - ground`
- `rot_offsets[人体body名] = R.from_quat(rot_offset, scalar_first=True)`

### 目标更新流程（`update_targets()` → `offset_human_data()`，第 150-170、268-284 行）

每帧执行：

1. 人体数据缩放（见 `human_scale_table`）；
2. **先施加旋转偏移**：`updated_quat = R(quat) * rot_offset`；
3. **再施加位置偏移**（在旋转后的局部系下）：
   `pos += R(updated_quat).apply(pos_offset)`；
4. 将最终 `(pos, rot)` 设置为对应 `FrameTask` 的目标，交给 mink IK 求解。

### K1 两阶段匹配表对比

| 机器人 body | 人体 body | 表1 (pos, rot) | 表2 (pos, rot) | 说明 |
| --- | --- | --- | --- | --- |
| `Trunk` | `spine3` | (0, 10) | (10, 5) | 躯干，只跟随姿态→同时跟位置 |
| `Right_Shank` / `Left_Shank` | `right_knee` / `left_knee` | (0, 10) | (10, 5) | 小腿 |
| `Right_Hip_Yaw` / `Left_Hip_Yaw` | `right_hip` / `left_hip` | (0, 10) | (10, 5) | 髋 |
| `Right_Arm_3` / `Left_Arm_3` | `right_shoulder` / `left_shoulder` | (0, 10) | (10, 5) | 上臂 |
| `right_hand_link` / `left_hand_link` | `right_elbow` / `left_elbow` | (0, 10) | (10, 5) | 手（跟踪肘部） |
| `right_foot_link` / `left_foot_link` | `right_foot` / `left_foot` | (0, 10) | **(100, 50)** | 脚，第二阶段被大幅加权 |
| `Head_2` | `head` | (0, 10) | (10, 5) | 头 |

**设计意图**：

- 第一阶段（表1）：所有部位仅用姿态权重 `10` 跟踪旋转，`0` 位置权重 →
  先求一个"形似"的全身位形，避免位置约束过强导致 IK 发散发抖；
- 第二阶段（表2）：所有部位改为 `(10, 5)` 同时跟踪位置与姿态，而**脚部
  大幅提高为 `(100, 50)`**，强制脚掌贴地（`left_foot_link` 的
  `pos_offset = [0, 0.02, 0]`、`right_foot_link` 为 `[0, -0.02, 0]`，
  是让脚掌在 y 方向略微向外展开的微调），从而显著减少滑步与抬脚。

### 姿态偏移四元数 `[-0.5, 0.5, 0.5, 0.5]`

这是 SMPL-X 人体坐标系与 K1 机器人坐标系之间的固定旋转对齐量
（wxyz 顺序，与 MuJoCo 一致）。所有部位统一使用，表示对每个人体 body 的
姿态施加相同的坐标系旋转，使人体前向 / 上向与机器人对齐。

## 五、参数在重定向流程中的位置

```
初始化: GMR(src_human="smplx", tgt_robot="booster_k1")
   │ 读取 smplx_to_k1.json (motion_retarget.py:57)
   │ 按 human_height_assumption 缩放 human_scale_table (:63-70)
   │ setup_retarget_configuration(): 按两个匹配表建 mink.FrameTask (:107-147)
   ▼
每帧: retarget(human_data)  (:173)
   │ update_targets(): 缩放 → 旋转/位置偏移 → 设置任务目标 (:150-170)
   │ 第一阶段 IK 求解 (ik_match_table1, use_ik_match_table1) (:177-195)
   │ 第二阶段 IK 求解 (ik_match_table2, use_ik_match_table2) (:197-216)
   ▼
输出: qpos = [root_pos(3), root_rot(4), dof_pos(15)]  (K1 = 22 DoF)
```

## 六、修改建议与注意事项

1. **改人体对应关系**（如 `Trunk → spine3`）：必须**同步**修改
   `ik_match_table1`、`ik_match_table2` 以及 `human_scale_table` 三处；
   若还要把 `human_root_name` 一并改为新部位，则该部位必须存在于
   `human_scale_table`。
2. **缩放表不能删减匹配表引用的部位**，否则运行时报 `KeyError`。
3. **脚部权重（100/50）是防滑步的关键**：若出现脚部抖动/滑动，优先调
   表2 中脚部权重；若姿态跟不准，调表1 的姿态权重（10）。
4. **`human_height_assumption` 与 `actual_human_height` 的比值决定整体缩放**：
   不同身高的人体数据会自动按比例缩放，通常无需修改。

## 七、人体 body 命名的来源（SMPL-X 命名约定）

K1 配置中出现的所有人体 body 名（`spine3`、`left_knee`、`left_hip`、
`left_shoulder`、`left_elbow`、`right_foot`、`left_foot`、`head` 等）**全部来自
smplx Python 库的 `JOINT_NAMES`（全小写下划线风格）**，而不是 npz 资产文件里的命名。

`general_motion_retargeting/utils/smpl.py` 生成每帧人体数据时：

```python
from smplx.joint_names import JOINT_NAMES
joint_names = JOINT_NAMES[: len(body_model.parents)]   # SMPL-X 取前 55 个
# 每帧输出: {joint_name: (position, quat_wxyz)}
```

因此配置文件中的每个名字都必须与 `JOINT_NAMES[:55]` **精确一致（大小写敏感）**。

> ⚠️ **常见误区**：`SMPLX_MALE.npz` / `SMPLX_FEMALE.npz` / `SMPLX_NEUTRAL.npz`
> 资产文件内部还内嵌了 `joint2num` / `part2num` 两个命名表（混合大小写风格，
> 如 `Pelvis`、`L_Hip`、`Spine3`、`Global`、`L_Thigh`）。它们只是模型资产的
> 元数据，GMR **从不使用**它们作为人体 body 名。两套命名指向同一批 55 个关节、
> 顺序完全一致，唯一的命名差异是眼睛关节：npz 为 `L_Eye`/`R_Eye`，smplx 库为
> `left_eye_smplhf`/`right_eye_smplhf`。K1 配置未涉及手/眼关节；完整 55 关节
> 命名与对照表见 [`ik_config.md`](./ik_config.md) 的 "Human Body Naming Convention (SMPL-X)" 一节。
