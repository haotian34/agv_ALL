# 2026-11-18 相机与夹爪诊断初稿

## 用途

`camera_gripper_key_test.py` 用于同时启动 RealSense 和夹爪键盘诊断：

- 相机先启动并确认彩色/深度帧正常；
- 只有按 `G` 才闭合夹爪；
- 按 `O` 张开夹爪；
- 按 `Q` / `ESC` 退出；
- 机械臂本体和底盘不发送任何运动指令；
- 每 0.5 秒读取并记录夹爪位置反馈，辅助排查“指令返回成功但动作延迟”的问题。

## 状态

这是现场诊断初稿。结合此前记录，夹爪延迟闭合的控制器原因尚未完全查明；`MoveGripper` 返回 0 不等于夹爪已经闭合，不能仅凭状态接口自动提升机械臂。

## 运行环境

```text
C:\Users\Admin\Desktop\现场调试脚本
D:\anaconda3\envs\agv\python.exe camera_gripper_key_test.py
```

依赖 `pyrealsense2`、`opencv-python`、`numpy` 和现场 Fairino SDK。

## 安全

- 夹爪两指之间不得有手指、线缆或杂物；
- 需现场人员守护实体急停；
- 本诊断不是自动抓取程序，不授权机械臂运动。
