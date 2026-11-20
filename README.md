> Fairino 槽位任务的正式代码只保留在 `fairino_slot_tasks/`。旧版 B 槽实现 `b_slot_tasks/` 已归档到 `legacy` 分支，不再在 `main` 中维护，也不得作为现场启动入口。


# AGV_ALL

本仓库归档AGV项目相关代码。

| 目录 | 代码作用 |
|---|---|
| `plane_calibration/` | A4九点平面标定、会话保存、静态定位和报告 |
| `initial_drafts/2026-11-18_handeye_3d_pipeline_initial_draft/` | 三维手眼标定、抓取偏移、预抓取和抓取测试 |
| `initial_drafts/2026-11-18_plane_2d_pipeline_initial_draft/` | 早期A4九点平面标定、静态定位和报告 |
| `initial_drafts/2026-11-18_camera_gripper_diagnostic_initial_draft/` | RealSense相机与夹爪联合诊断 |
| `initial_drafts/2026-11-19_gripper_keyboard_control_initial_draft/` | 夹爪实时键盘开合控制 |
| `fairino_slot_tasks/` | Fairino 车上 B/C 槽四个独立搬运任务节点、命令行入口和交接说明 |

目录命名格式：

```text
initial_drafts/<日期>_<用途>_initial_draft/
```