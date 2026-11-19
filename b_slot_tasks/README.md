# Fairino B槽任务1/任务2

Fairino 机械臂 B 槽固定点位 LIN 搬运程序。程序直接读取 WebApp 当前示教点：

```text
A_middle
B_1high
B_3
B_4low
B_2
```

## 两个独立程序

| 文件 | 作用 | 起点 | 终点 |
|---|---|---|---|
| `run_task1.py` | 生件放入生件槽 | A_middle，夹爪闭合夹持 | A_middle，夹爪张开 |
| `run_task2.py` | 从生件槽取出工件 | A_middle，夹爪张开 | A_middle，夹爪闭合夹持 |

每次运行只执行一个任务，结束后停在 `A_middle`，不会自动执行下一任务。

## 手动运行

```powershell
cd C:\Users\Admin\Desktop\现场调试脚本
D:\anaconda3\envs\agv\python.exe run_task1.py --sdk-root C:\Users\Admin\Desktop\现场调试脚本
```

```powershell
D:\anaconda3\envs\agv\python.exe run_task2.py --sdk-root C:\Users\Admin\Desktop\现场调试脚本
```

也可以使用带任务选择的总入口：

```powershell
D:\anaconda3\envs\agv\python.exe run_b_cycle.py --task 1 --sdk-root C:\Users\Admin\Desktop\现场调试脚本
D:\anaconda3\envs\agv\python.exe run_b_cycle.py --task 2 --sdk-root C:\Users\Admin\Desktop\现场调试脚本
```

## 信号调用

后续信号控制系统可以直接复用核心函数：

```python
from run_b_cycle import FairinoSession, execute_task_1, execute_task_2

session = FairinoSession(robot)
execute_task_1(session)
execute_task_2(session)
```

`execute_task_1()` 和 `execute_task_2()` 在异常时会调用 `StopMotion`，不会自动张爪。

## 默认参数

- 工具坐标：`toolcoord2`，`tool=2`
- 工件坐标：`wobj=0`
- 控制器全局速度：`30%`
- MoveL 指令速度：`100%`
- 首次抬升加速度：`100%`
- 夹爪闭合力：`70%`

## 安全检查

- 起点必须为 `A_middle`
- 检查自动模式、使能、报警、急停、安全停止、碰撞和通信状态
- 任务1要求夹爪闭合
- 任务2要求夹爪张开
- 任务2闭爪后、首次抬升后、返回 A 后均检查夹爪保持闭合
- 任务1张爪后检查机器人状态，正常才返回 A
- 任一步失败立即中止，不自动张爪、不继续返回 A、不执行下一任务
- 日志分别写入 `logs/b_task1_*.json` 和 `logs/b_task2_*.json`

当前夹爪位置反馈只有张开/闭合状态，电流反馈可能为 `0`。因此程序能够检测夹爪是否重新张开，但无法判断工件在夹爪保持闭合时是否滑出；如需检测该情况，应增加工件存在传感器。

## 测试

```powershell
python -m unittest discover -s tests
```
