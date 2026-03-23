# GMR Motion Editor

一个基于PyQt6的GMR机器人运动数据可视化编辑器，支持导入、剪辑和导出GMR格式数据。

## 功能特性

- **导入/导出**: 支持加载和保存 `.pkl` 格式的GMR运动数据
- **可视化**: 使用MuJoCo实时渲染机器人运动
- **剪辑**: 简单的起止时间裁剪功能，导出选定片段
- **多机器人支持**: 支持项目中所有17种机器人模型

## 安装依赖

确保已安装PyQt6：

```bash
pip install PyQt6
```

其他依赖（mujoco, numpy等）已在GMR项目中安装。

## 使用方法

### 启动编辑器

```bash
# 从motion_editor目录运行
cd motion_editor
python motion_editor.py

# 或带文件路径启动
python motion_editor.py /path/to/motion_data.pkl
```

### 界面说明

1. **机器人选择**: 从下拉菜单选择对应的机器人类型
2. **播放控制**: 
   - ▶ Play / ⏸ Pause: 播放/暂停
   - ⏹ Stop: 停止并重置到起始位置
   - ⏮ / ⏭: 上一帧/下一帧
   - ⏮⏮ / ⏭⏭: 跳到裁剪范围开始/结束
3. **时间轴**:
   - 蓝色手柄: 裁剪起点
   - 红色手柄: 裁剪终点
   - 黄色竖线: 当前帧位置
4. **导出**: 点击"📤 Export Clip"导出裁剪后的片段

### 快捷键

- `Space`: 播放/暂停
- `← / →`: 上一帧/下一帧
- `Home`: 跳到裁剪范围开始
- `End`: 跳到裁剪范围结束
- `Ctrl+O`: 打开文件
- `Ctrl+S`: 保存文件
- `Ctrl+Shift+S`: 另存为

## 示例工作流程

1. 运行 `python motion_editor.py`
2. File → Open，选择一个 `.pkl` 运动数据文件
3. 在机器人选择下拉框中选择对应的机器人类型
4. 点击播放按钮查看运动
5. 拖动时间轴上的蓝色和红色手柄设置裁剪范围
6. 点击"Export Clip"导出裁剪后的片段

## 项目结构

```
motion_editor/
├── docs/                           # 文档
│   ├── gmr_visualizer_design.md   # 设计文档
│   └── implementation_plan.md     # 实施计划
├── src/                           # 源代码
│   └── gui/                       # GUI模块
│       ├── __init__.py
│       ├── gmr_manager.py        # 数据管理
│       ├── motion_controller.py  # 播放控制
│       ├── timeline_widget.py    # 时间轴控件
│       └── main_window.py        # 主窗口
├── tests/                         # 测试文件
│   ├── test_gmr_manager.py
│   ├── test_timeline_widget.py
│   └── test_motion_controller.py
├── motion_editor.py              # 启动脚本
├── path_config.py                # 路径配置
└── README.md                     # 本文件
```

## 支持的机器人

支持GMR项目中的所有17种机器人模型：

- Unitree G1 (29 DOF)
- Unitree G1 with Hands (43 DOF)
- Unitree H1 (19 DOF)
- Unitree H1 2 (27 DOF)
- Booster T1
- Booster T1 29dof
- Booster K1 (22 DOF)
- Stanford ToddlerBot
- Fourier N1
- ENGINEAI PM01
- HighTorque Hi (25 DOF)
- Galaxea R1 Pro (24 DOF)
- Kuavo S45 (28 DOF)
- Berkeley Humanoid Lite (22 DOF)
- PND Adam Lite (25 DOF)
- Tienkung (20 DOF)
- PAL Robotics' Talos (30 DOF)
- Fourier GR3 (31 DOF)

## 开发说明

### 运行测试

```bash
cd motion_editor
python tests/test_gmr_manager.py
python tests/test_timeline_widget.py
python tests/test_motion_controller.py
```

### 技术栈

- Python 3.10+
- PyQt6 (GUI框架)
- MuJoCo (3D渲染)
- NumPy (数据处理)

## 许可证

本项目基于GMR项目，遵循MIT许可证。
