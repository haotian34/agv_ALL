# 三维手眼与抓取测试

| 文件 | 代码作用 |
|---|---|
| `handeye_collect.py` | 采集固定靶点的像素、深度和TCP位姿 |
| `handeye_solve.py` | 离线求解相机到TCP的三维变换 |
| `handeye_validate.py` | 静态复测手眼标定结果 |
| `grasp_offset_calibrate.py` | 计算视觉靶点到抓取TCP的偏移 |
| `pregrasp_dry_run.py` | 计算并执行预抓取空跑 |
| `grasp_lift_test.py` | 下降、闭合夹爪、确认夹持和短距离提升测试 |
| `test_*.py` | 离线单元测试 |
