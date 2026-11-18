# -*- coding: utf-8 -*-
"""
handeye_collect.py — eye-in-hand 手眼标定数据采集（2026-11-16）
原理：固定靶点不动，换 9~15 组机械臂姿态；每组记录
    TCP 位姿（SDK） + 靶点像素（鼠标点选） + 该点深度（RealSense）
输出：handeye_data.jsonl（每行一组，断点续采，崩溃不丢数据）

安全：本脚本【不驱动任何运动】，换姿态请在 WebApp 手动低速点动。
要求：
- 全程使用同一个工具坐标（toolcoord2=夹爪尖端中心），不要切换！
- 靶点固定不动，相机/夹爪不能磕碰。
- 姿态要求：覆盖不同位置、不同距离、不同倾角（姿态差尽量>20~30度），
  靶点尽量出现在画面不同区域（中间/四角都要）。

依赖：fairino 文件夹、pyrealsense2、numpy、opencv-python（均已装）。
运行：D:\\anaconda3\\envs\\agv\\python.exe handeye_collect.py
操作：
    鼠标左键点靶点  -> 记录一组（终端打印编号和数值）
    u              -> 撤销最后一组
    q / ESC / 点X  -> 退出
"""
import sys
import os
import json
import time
sys.stdout.reconfigure(encoding="utf-8", line_buffering=True)

import cv2
import numpy as np
import pyrealsense2 as rs
from fairino import Robot

IP = "192.168.192.160"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "handeye_data.jsonl")
WIN = "handeye collect"

# ---------------- 机械臂 ----------------
print(f"[1/3] 连接机械臂 {IP} ...")
robot = Robot.RPC(IP)
err, tcp = robot.GetActualTCPPose(0)
print(f"      当前 TCP 位姿: {tcp}")

# ---------------- 相机 ----------------
print("[2/3] 启动 RealSense（彩色+深度，深度对齐到彩色）...")
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
profile = pipeline.start(config)
align = rs.align(rs.stream.color)
intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
print(f"      内参: fx={intr.fx:.1f} fy={intr.fy:.1f} cx={intr.ppx:.1f} cy={intr.ppy:.1f}")

# ---------------- 数据 ----------------
samples = []
if os.path.exists(OUT):
    with open(OUT, "r", encoding="utf-8") as fp:
        for line in fp:
            line = line.strip()
            if line:
                samples.append(json.loads(line))
    print(f"      发现已有数据 {len(samples)} 组，将续采（如需重采请先删除 handeye_data.jsonl）")

latest = {"frame": None, "depth_frame": None}

def record(u, v):
    df = latest["depth_frame"]
    if df is None:
        print("      尚无深度帧，稍后重试")
        return
    # 5x5 邻域取有效深度中位数，抗跳点
    zs = []
    for du in range(-2, 3):
        for dv in range(-2, 3):
            uu, vv = u + du, v + dv
            if 0 <= uu < 640 and 0 <= vv < 480:
                z = df.get_distance(uu, vv)
                if z > 0:
                    zs.append(z)
    if not zs:
        print("      该点深度无效（全为0），请换个角度或靠近些再点")
        return
    z = float(np.median(zs))
    X = (u - intr.ppx) / intr.fx * z
    Y = (v - intr.ppy) / intr.fy * z
    err, tcp = robot.GetActualTCPPose(0)
    if err != 0:
        print(f"      读取 TCP 失败 err={err}，未记录")
        return
    s = {
        "tcp_pose": list(tcp),          # mm / deg，基座坐标系
        "pixel": [int(u), int(v)],
        "depth_m": z,
        "cam_point_m": [X, Y, z],       # 相机坐标系（m）
        "time": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    samples.append(s)
    with open(OUT, "a", encoding="utf-8") as fp:
        fp.write(json.dumps(s) + "\n")
    print(f"      已记录第 {len(samples)} 组: 像素=({u},{v}) 深度={z:.3f}m TCP={[round(t,1) for t in tcp]}")

def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        record(x, y)

cv2.namedWindow(WIN)
cv2.setMouseCallback(WIN, on_mouse)

print("[3/3] 开始采集。WebApp 点动换姿态 -> 鼠标点靶点记录；u 撤销；q/ESC 退出。")
print(f"      当前已有 {len(samples)} 组，目标 9~15 组。")
try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            continue
        latest["depth_frame"] = depth_frame
        img = np.asanyarray(color_frame.get_data())

        for s in samples:  # 已点过的位置画绿点（仅参考，不代表当前姿态位置）
            pass
        cv2.putText(img, f"samples: {len(samples)}  (click target, u=undo, q=quit)",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.imshow(WIN, img)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break
        if key == ord("u") and samples:
            samples.pop()
            with open(OUT, "w", encoding="utf-8") as fp:
                for s in samples:
                    fp.write(json.dumps(s) + "\n")
            print(f"      已撤销最后一组，剩 {len(samples)} 组")
        if cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    robot.CloseRPC()
    print(f"已退出，共 {len(samples)} 组，数据保存在 handeye_data.jsonl。把该文件发回即可求解。")
