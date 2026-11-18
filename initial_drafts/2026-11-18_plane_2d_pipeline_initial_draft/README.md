# 2026-11-18 二维平面标定初稿

## 状态

这是基于 A4 九点目标的 2D 平面标定代码初稿，包含只读采集、会话持久化、静态定位和报告。

该目录中的代码用于支持固定拍照姿态下的像素到工作平面映射。它不是正式生产版本，也没有替代现场安全验收。

## 文件用途

| 文件 | 用途 |
|---|---|
| `plane_calibrate.py` | 在静止的 A4 图像上交互采集九点，并读取当前 TCP；不驱动设备 |
| `plane_math.py` | Fairino 位姿、单应矩阵、本地点平面等纯数学工具 |
| `plane_session.py` | 追加式保存标定会话、校验和与事件 |
| `plane_locate.py` | 使用带版本信息的平面配置进行静态定位 |
| `plane_report.py` | 输出静态标定报告和追溯信息 |
| `plane_profiles.json` | 当前配置结构；其中 Z 参数和现场结果仍需现场填写/生成 |
| `PLANE_CALIBRATION_GUIDE.md` | 现场九点 A4 标定操作与安全指南 |
| `test_*.py` | 离线测试 |

## 未包含的数据

`plane_results/`、现场会话 JSONL、现场图片和任何实际标定结果未上传。仓库中的配置和测试不得被解释为已经完成现场标定。

## 重要边界

- 采集脚本只读 TCP 和相机，所有机械臂姿态由人员在 WebApp 中手动完成；
- 脚本不授权机器人、夹爪或 AGV 自动运动；
- 每套平面配置必须独立采集、独立验证；
- 参考拍照 RPY、工作距离、工具坐标和机械臂基座关系不得混用；
- 现场 Z 未配置时保持 `null`，不能把空值解释为 0 或运动许可。

## 依赖

- `numpy`；
- `opencv-python`；
- `pyrealsense2`；
- Fairino Windows Python SDK，按现场目录放置。

## 测试提示

在中文 Windows 上运行命令行测试时，建议先设置 UTF-8，避免子进程帮助信息因系统代码页导致解码错误：

```powershell
$env:PYTHONIOENCODING = 'utf-8'
$env:PYTHONUTF8 = '1'
```
