# 夹爪实时键盘控制

| 文件 | 代码作用 |
|---|---|
| `gripper_keyboard_control.py` | 实时读取夹爪位置，按键控制夹爪张开和闭合 |
| `test_gripper_keyboard_control.py` | 验证命令非阻塞和实时位置轮询 |

操作键：

| 按键 | 作用 |
|---|---|
| `O` | 张开夹爪 |
| `G` | 闭合夹爪 |
| `S` | 立即读取位置 |
| `Q` | 退出 |

启动命令：

```bat
cd /d C:\Users\Admin\Desktop\现场调试脚本
D:\anaconda3\envs\agv\python.exe gripper_keyboard_control.py
```
