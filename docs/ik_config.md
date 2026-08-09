# IK Config
In our ik config such as `smplx_to_g1.json`, you might find following params. I add annotations here for your understanding.
```json
"ik_match_table1": {
        "pelvis": [ # robot's body name
            "pelvis", # corresponding human body name, here we are using "pelvis" as example
            100, # weight to track 3D positions (xyz)
            10, # weight to track 3D rotations
            [
                0.0, # x offset added to human body "pelvis" x
                0.0, # y offset added to human body "pelvis" y
                0.0 # z offset added to human body "pelvis" z
            ],
            [
                # the rotation (represented as quaternion) applied to human body "pelvis". the order follows scalar first (wxyz)
                0.5,
                -0.5,
                -0.5,
                -0.5
            ]
        ],
      ...
```

## Human Body Naming Convention (SMPL-X)

All human body names referenced by an ik config (`human_root_name`, `human_scale_table`,
`ik_match_table1/2`, e.g. `pelvis`, `spine3`, `left_knee`) come from the **smplx Python
package** — **not** from the npz asset file. `general_motion_retargeting/utils/smpl.py` does:

```python
from smplx.joint_names import JOINT_NAMES
joint_names = JOINT_NAMES[: len(body_model.parents)]   # first 55 for SMPL-X
```

and each frame is emitted as `{joint_name: (position, quat_wxyz)}`. So every human body
name in a config must exactly match one of the following 55 lowercase snake_case names:

**Body (idx 0–24):** `pelvis`, `left_hip`, `right_hip`, `spine1`, `left_knee`,
`right_knee`, `spine2`, `left_ankle`, `right_ankle`, `spine3`, `left_foot`,
`right_foot`, `neck`, `left_collar`, `right_collar`, `head`, `left_shoulder`,
`right_shoulder`, `left_elbow`, `right_elbow`, `left_wrist`, `right_wrist`, `jaw`,
`left_eye_smplhf`, `right_eye_smplhf`

**Left hand (idx 25–39):** `left_index1..3`, `left_middle1..3`, `left_pinky1..3`,
`left_ring1..3`, `left_thumb1..3`

**Right hand (idx 40–54):** `right_index1..3`, `right_middle1..3`, `right_pinky1..3`,
`right_ring1..3`, `right_thumb1..3`

### npz `joint2num` / `part2num` vs `JOINT_NAMES`

The model asset (`SMPLX_MALE.npz` / `SMPLX_FEMALE.npz` / `SMPLX_NEUTRAL.npz`) embeds its
**own** naming dicts: `joint2num` (mixed-case, e.g. `Pelvis`, `L_Hip`, `Spine3`) and
`part2num` (e.g. `Global`, `L_Thigh`, `Spine`). They are asset metadata only and are
**never used** by GMR. Both sets describe the same 55 joints in the same order; the only
naming difference is the eye joints:

| npz `joint2num` | smplx `JOINT_NAMES` |
| --- | --- |
| `L_Eye` (idx 23) | `left_eye_smplhf` (idx 23) |
| `R_Eye` (idx 24) | `right_eye_smplhf` (idx 24) |

> If you read joint names directly from the npz file, convert them to the lowercase
> `JOINT_NAMES` style before matching against an ik config.
