# 2026-11-18 三维手眼、抓取偏移与抓取安全初稿

## 状态

这是眼在手上三维手眼标定及抓取安全门槛的**阶段性初稿**，用于记录当时的实现、数据结构和离线测试，不是正式视觉抓取版本。

已知结论：使用 RealSense 深度和人工点选小圆柱靶点时，Z 方向单帧深度稳定性不足；离线拟合/留一指标和独立静态复测均不能替代现场抓取验收。不要直接使用本目录代码自动实抓。

## 文件用途

| 文件 | 用途 |
|---|---|
| `handeye_collect.py` | 手动调整姿态时采集固定靶点的像素、深度和 TCP 位姿 |
| `handeye_solve.py` | 离线求解相机到 TCP 的三维变换 |
| `handeye_validate.py` | 独立静态复测，只读取 TCP 和相机，不驱动机械臂 |
| `grasp_offset_calibrate.py` | 使用视觉靶点与人工安全抓取位姿计算抓取偏移 |
| `pregrasp_dry_run.py` | 从视觉靶点计算预抓取位姿，只执行一次空跑运动并带高度防护 |
| `grasp_lift_test.py` | 受保护的下降、夹爪闭合、人工确认和短距离提升试验 |
| `test_*.py` | 对应模块的离线单元测试 |

## 未包含的数据

`handeye_data.jsonl`、`handeye_result.json`、`handeye_report.txt`、`grasp_offset.json` 以及运行日志均未上传。这些文件含现场标定结果和设备位姿，应在受控目录中生成和保存。

## 依赖

- Python 3.11 现场环境；
- `numpy`、`scipy`；
- `pyrealsense2`、`opencv-python`；
- Fairino Windows Python SDK，按现场目录放置，不能通过普通 pip 包代替。

## 安全说明

- `handeye_collect.py`、`handeye_validate.py`、`grasp_offset_calibrate.py` 设计为只读 TCP/相机、由人员手动点动；
- `pregrasp_dry_run.py` 和 `grasp_lift_test.py` 具备真实运动/夹爪控制能力；
- 任何真实运动前都必须核对 `toolcoord2`、使能状态、报警、路径、夹爪状态和实体急停；
- `MoveGripper` 返回 0、固定等待或某些状态反馈不能单独证明夹爪已闭合；
- 本目录代码未获得自动抓取或生产使用授权。
