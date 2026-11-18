# 平面标定工具

这是单站点三平面视觉标定与静态定位工具，只读取RealSense彩色图和机械臂TCP，不发送机械臂、夹爪或AGV运动命令。

## 内容

- `plane_math.py`：坐标变换、九点单应拟合和平面求解。
- `plane_session.py`：追加式会话、撤销、断点恢复和参考图完整性校验。
- `plane_calibrate.py`：A4九点只读采集与结果生成。
- `plane_locate.py`：静态像素定位验证。
- `plane_report.py`：标定报告生成。
- `plane_profiles.json`：三套配置登记表。
- `PLANE_CALIBRATION_GUIDE.md`：现场操作指南。
- `test_plane_*.py`：离线测试。

## 配置

- `warehouse_raw_1`：仓库第一行原件。
- `warehouse_raw_2`：仓库第二行原件。
- `machine_finished_pick`：加工区加工件抓取。

## 运行

在车载电脑的脚本目录中执行：

```powershell
D:\anaconda3\envs\agv\python.exe .\plane_calibrate.py --help
```

完整流程、安全条件和命令见 `PLANE_CALIBRATION_GUIDE.md`。

## 边界

当前仅完成离线实现与静态验证，未完成真实相机、Fairino和现场平面标定。所有结果均标记 `motion_authorized=false`，诊断Z不得直接用于运动。
