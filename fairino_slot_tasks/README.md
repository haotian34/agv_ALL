# Fairino 车上槽位四任务节点

本目录提供四个**相互独立、自包含**的 Fairino 机械臂任务程序。每个 `.py` 文件都可以单独复制到车载电脑运行，也可以由上层 Python 节点导入后调用。

每个程序只执行一个任务，结束后停在 `A_middle`，不会自动执行下一任务，不包含 AGV/底盘控制，也不包含 YOLO、视觉标定或槽位传感器逻辑。

## 版本与归档

`fairino_slot_tasks/` 是 Fairino 槽位任务的正式版本，也是 `main` 分支中的唯一任务入口。每个任务只保留一个文件：

```text
run_task1.py   生件放入 B 槽
run_task2.py   从 B 槽取生件
run_task3.py   成品放入 C 槽
run_task4.py   从 C 槽取成品
```

旧版 `b_slot_tasks/` 只覆盖 B 槽任务1/任务2，并且依赖 `run_b_cycle.py`，已归档到 GitHub 的 `legacy` 分支：

```text
https://github.com/haotian34/agv_ALL/tree/legacy
```

归档分支只用于追溯历史，不用于现场部署，也不作为上层程序的导入路径。现场交接和后续电信号调用统一使用本目录。


## 1. 文件与任务

| 文件 | 任务 | 起点夹爪 | 终点夹爪 |
|---|---|---|---|
| `run_task1.py` | 生件放入 B 槽 | 闭合且检测到工件 | 张开且未检测到物体 |
| `run_task2.py` | 从 B 槽取生件 | 张开且未检测到物体 | 闭合且检测到工件 |
| `run_task3.py` | 成品放入 C 槽 | 闭合且检测到工件 | 张开且未检测到物体 |
| `run_task4.py` | 从 C 槽取成品 | 张开且未检测到物体 | 闭合且检测到工件 |

本版本不判断工件是生件还是成品，四个任务按现场时序调用。夹爪检查只用于判断“有工件/空夹爪”。

## 2. 依赖的 WebApp 点位

所有点位必须已经在 Fairino WebApp 中示教并保存：

```text
A_middle
B_1high
B_2
B_3
B_4low
C_1high
C_2
C_3low
```

运行前后必须满足：

- 工具坐标：`toolcoord2`，即 `tool=2`
- 工件坐标：`wobj=0`
- 不自动切换工具坐标
- 机械臂处于自动模式、已使能、无报警、无急停、无碰撞
- 任务开始前机器人必须停在 `A_middle`

程序会在运动前读取当前 WebApp 点位并自动检查以上状态。

## 3. 任务路径

```text
任务1：A_middle → B_1high → B_3 → B_4low
       → 张爪 → B_3 → B_2 → B_1high → A_middle

任务2：A_middle → B_1high → B_3 → B_4low
       → 闭爪 → B_3 → B_2 → B_1high → A_middle

任务3：A_middle → C_1high → C_2 → C_3low
       → 张爪 → C_2 → C_1high → A_middle

任务4：A_middle → C_1high → C_2 → C_3low
       → 闭爪 → PTP 到 C_2 → C_1high → A_middle
```

任务4第一次从 `C_3low` 抬升到 `C_2` 使用现场验证成功的 PTP，其余路径使用 LIN。

## 4. 命令行直接运行

```powershell
cd C:\Users\Admin\Desktop\现场调试脚本

D:\anaconda3\envs\agv\python.exe run_task1.py --sdk-root C:\Users\Admin\Desktop\现场调试脚本
D:\anaconda3\envs\agv\python.exe run_task2.py --sdk-root C:\Users\Admin\Desktop\现场调试脚本
D:\anaconda3\envs\agv\python.exe run_task3.py --sdk-root C:\Users\Admin\Desktop\现场调试脚本
D:\anaconda3\envs\agv\python.exe run_task4.py --sdk-root C:\Users\Admin\Desktop\现场调试脚本
```

任务中间没有人工按回车节点。程序通过机器人状态和夹爪反馈自动判断是否继续。

## 5. 作为 Python 节点调用

每个文件都提供相同形式的入口：

```python
execute(session, **kwargs) -> str
```

返回值是本次执行的 JSON 日志绝对路径。

### 任务1/2：B 槽

```python
from fairino import Robot
import run_task1
import run_task2

robot = Robot.RPC("192.168.192.160")
session = run_task1.FairinoSession(robot)
try:
    log_path = run_task1.execute(session)
    # 或 run_task2.execute(session)
finally:
    session.close()
```

### 任务3/4：C 槽

```python
from fairino import Robot
import run_task3
import run_task4

robot = Robot.RPC("192.168.192.160")
session = run_task3.FairinoCSession(robot)
try:
    log_path = run_task3.execute(session)
    # 或 run_task4.execute(session)
finally:
    session.close()
```

注意：

- 任务1/2使用 `FairinoSession`
- 任务3/4使用 `FairinoCSession`
- 每次调用只执行一个任务
- 上层节点负责按现场时序选择任务，程序不会自动衔接下一任务

## 6. 常用调用参数

任务1/2：

```python
run_task1.execute(
    session,
    speed=100.0,              # MoveL 指令速度
    global_speed=30.0,        # 控制器全局速度
    extract_speed=100.0,      # 任务2首次抬升速度
    extract_acceleration=100.0,
    grip_force=70.0,
)
```

任务3/4：

```python
run_task3.execute(
    session,
    speed=30.0,               # 普通 LIN 速度
    global_speed=30.0,
    lift_speed=30.0,          # 任务4首次 PTP 抬升速度
    grip_force=70.0,
)
```

任务1/3的放件任务也会读取这些参数，但不会实际执行闭爪或抬升动作。

命令行参数：

```text
通用：--ip --sdk-root --speed --global-speed --grip-force
任务1/2：--extract-speed --extract-acc
任务3/4：--lift-speed
```

默认 IP 为 `192.168.192.160`。

## 7. 异常和安全行为

任一检查失败时：

1. 程序写入失败日志；
2. 抛出 `CycleError`；
3. 节点入口调用 `StopMotion`；
4. 不自动张爪；
5. 不自动返回 `A_middle`；
6. 不自动执行下一任务。

可能出现的中止条件包括：

- 起点不在 `A_middle` 或起点误差超限；
- 工具坐标不是 `tool=2`；
- 报警、急停、安全停止、碰撞、通讯异常；
- WebApp 点位缺失；
- 夹爪起点状态与任务不符；
- 张爪后仍检测到物体或夹爪没有张开；
- 闭爪后没有检测到物体；
- 运动中机器人状态异常或超时。

异常后必须由现场人员检查并决定恢复方式，程序不会自动尝试恢复。

## 8. 日志

日志写入程序所在目录的 `logs/`：

```text
logs/b_task1_*.json
logs/b_task2_*.json
logs/c_task3_*.json
logs/c_task4_*.json
```

日志包括：

- 任务编号和名称；
- WebApp 点位快照；
- 起点机器人状态；
- 速度、夹持力；
- 每段运动记录；
- 夹爪动作和反馈；
- 异常原因和发生时间；
- 成功结束时间和最终位置检查结果。

## 9. 现场使用前检查

1. 确认已保存并核对所有 WebApp 点位；
2. 确认工具坐标仍为 `toolcoord2`；
3. 确认机械臂在 `A_middle`，一人守实体急停；
4. 首次验证应使用低速并确认路径无碰撞；
5. 若夹爪位置/状态反馈阈值与本机不一致，应先记录实际反馈值再调整代码，不要直接绕过检查；
6. 当前夹爪反馈不能区分生件和成品，工件类型必须由现场时序和上游任务编排保证。
