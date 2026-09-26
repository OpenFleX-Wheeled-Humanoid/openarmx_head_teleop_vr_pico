# openarmx_head_teleop_vr_pico

[English](./README.md) | 中文

---

![封面](./image/cover.gif)


使用 Pico VR 头显方向追踪遥操作 OpenArmX 头部的节点。

## 概述

本 Python 包提供一个 ROS 2 节点，将 Pico VR 头显的方向映射为 OpenArmX 头部关节指令。实现了相对控制（按键切换激活）、平滑步进限制、软关节限位、启动回零和紧急停止支持。

## 功能特性

- **相对方向控制**：按右手柄 B 键切换头部跟踪。切换开启时捕获参考四元数，相对旋转映射到头部偏航/俯仰。
- **步进限制运动**：每个控制周期的最大角度步进受限，确保运动平滑（可配置慢速/快速模式）。
- **软关节限位**：在关节边界附近平滑过渡，避免硬停止（可配置边距和过渡宽度）。
- **启动回零**：启动时自动将头部移回零位，完成后才接受 VR 输入。
- **紧急停止**：订阅 `/vr_estop_active` 以暂停头部控制。
- **可选 TF 可视化**：发布调试用 TF 坐标系，显示当前指令目标。

## 编译

```bash
cd ~/openflex_ws
colcon build --packages-select openarmx_head_teleop_vr_pico
source install/setup.bash
```

## 启动

```bash
ros2 launch openarmx_head_teleop_vr_pico head_teleop_vr_pico.launch.py
```

### 启动参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `control_rate` | `50.0` | 控制循环频率 (Hz) |
| `button_b_topic` | `/pico_right_controller/button_b` | B 键切换话题 (std_msgs/Bool) |
| `head_pose_topic` | `/pico_head/pose` | VR 头显姿态话题 (geometry_msgs/PoseStamped) |
| `rate_topic` | `/pico_left_controller/rate` | 速度模式值 (std_msgs/Float32, 0-1) |
| `command_topic` | `/head_forward_position_controller/commands` | 输出指令话题 |
| `slow_max_step_deg` | `2.0` | 慢速模式每周期最大步进（度） |
| `fast_max_step_deg` | `5.0` | 快速模式每周期最大步进（度） |
| `yaw_scale` | `1.0` | 偏航运动缩放系数 |
| `pitch_scale` | `1.0` | 俯仰运动缩放系数 |
| `invert_yaw` | `false` | 反转偏航方向 |
| `invert_pitch` | `false` | 反转俯仰方向 |
| `enable_soft_limits` | `true` | 启用软关节限位过渡 |
| `yaw_soft_margin_deg` | `10.0` | 偏航软限位距物理极限的边距（度） |
| `pitch_soft_margin_deg` | `5.0` | 俯仰软限位边距（度） |
| `soft_limit_blend_deg` | `10.0` | 软限位过渡区宽度（度） |
| `startup_home_enabled` | `true` | 启用启动回零 |
| `startup_home_step_deg` | `1.0` | 回零每周期步进大小（度） |
| `startup_home_tolerance_deg` | `1.0` | 回零完成容差（度） |
| `head_pose_timeout_sec` | `0.3` | VR 头部位姿超时后保持当前位置（秒） |
| `publish_visualization_tf` | `false` | 发布调试 TF 坐标系 |

## 订阅话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/pico_head/pose` | `geometry_msgs/PoseStamped` | VR 头显方向 |
| `/pico_right_controller/button_b` | `std_msgs/Bool` | B 键，用于开启/关闭控制 |
| `/pico_left_controller/rate` | `std_msgs/Float32` | 速度模式 (0.0=慢, 1.0=快) |
| `/joint_states` | `sensor_msgs/JointState` | 当前头部关节位置 |
| `/vr_estop_active` | `std_msgs/Bool` | 紧急停止（transient local QoS） |

## 发布话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/head_forward_position_controller/commands` | `std_msgs/Float64MultiArray` | 位置指令 [yaw, pitch]（弧度） |

## TF（可选）

当 `publish_visualization_tf` 为 `true` 时：
- 坐标系 `head_teleop_target`（父坐标系 `head_base_link`）：表示当前指令目标方向。

## 控制流程

1. 启动时，若 `startup_home_enabled` 为 true，节点逐步将头部移至零位。
2. 回零完成后，等待 B 键按下。
3. B 键上升沿：切换控制开/关。
   - 开启：捕获当前 VR 四元数作为参考，当前关节位置作为锚点。
   - 关闭：保持当前位置。
4. 控制启用时：VR 相对旋转映射为偏航/俯仰指令，应用缩放、步进限制和软限位。

## 依赖

- `rclpy`
- `geometry_msgs`、`sensor_msgs`、`std_msgs`
- `tf2_ros`

## 前置条件

- `openarmx_head_bringup` 必须正在运行（提供控制器和关节状态）。
- Pico VR 头显流式传输节点必须正在发布姿态数据。

## 许可证

本作品采用知识共享 署名-非商业性使用-相同方式共享 4.0 国际许可协议 (CC BY-NC-SA 4.0) 进行许可。

版权所有 (c) 2026 成都长数机器人有限公司 (Chengdu Changshu Robot Co., Ltd.)

详情请参阅 [LICENSE_CN.md](LICENSE) 文件或访问：http://creativecommons.org/licenses/by-nc-sa/4.0/

## 致谢

本包是 OpenArmX 机器人平台生态系统的一部分，专为协作机器人领域的研究和工业应用而开发。

---

## 📞 联系我们

### 成都长数机器人有限公司
**Chengdu Changshu Robotics Co., Ltd.**

| 联系方式 | 信息 |
|---------|------|
| 📧 邮箱 | openarmrobot@gmail.com |
| 📱 电话/微信 | +86-17746530375 |
| 🌐 官网 | <https://openarmx.com/> |
| 🌐 文档 | <http://docs.openarmx.com/> |
| 📍 地址 | 天津经济技术开发区西区新业八街11号华诚机械厂 |
| 👤 联系人 | 王先生 |
