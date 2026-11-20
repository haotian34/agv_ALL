# Fairino 槽位任务旧版归档

本分支是 Fairino 车上 B/C 槽搬运任务的旧版与历史实现归档分支，不作为现场部署入口。

## 当前正式版本

正式版本只保留在 `main` 分支：

```text
fairino_slot_tasks/
  README.md
  run_task1.py   生件放入 B 槽
  run_task2.py   从 B 槽取生件
  run_task3.py   成品放入 C 槽
  run_task4.py   从 C 槽取成品
```

正式版每个任务只有一个自包含入口文件，四个任务结束后均停在 `A_middle`，不自动执行下一任务。

## 本分支保留的旧实现

```text
b_slot_tasks/
  run_b_cycle.py   旧版 B 槽统一核心程序
  run_task1.py     旧版任务1入口
  run_task2.py     旧版任务2入口
  tests/           旧版 B 槽测试
```

旧版 `b_slot_tasks` 已被 `main` 中的 `fairino_slot_tasks` 取代，原因如下：

- 旧版只覆盖 B 槽任务1/任务2，正式版覆盖 B/C 槽四个任务。
- 旧版入口依赖 `run_b_cycle.py`，正式版任务文件可单独复制运行。
- 两套任务1/任务2同时保留会造成现场选错版本和重复维护。
- 正式版统一了四任务的调用接口、夹爪反馈检查、起点检查和日志说明。

## 使用规则

- 现场运行、交接和后续电信号调用只使用 `main` 的 `fairino_slot_tasks`。
- 本分支只用于追溯历史、对比调试过程和恢复旧资料。
- 不要把本分支路径配置到车载电脑的生产启动脚本中。
- 若必须复用旧代码，应先确认它与当前 WebApp 点位、工具坐标、夹爪参数和现场安全要求一致。
