# 单站点三平面静态标定现场指南

## 0. 当前状态与使用边界

本指南用于三块彼此独立的替代 A4 平面：`warehouse_raw_1`、`warehouse_raw_2`、`machine_finished_pick`。每块平面必须单独采集、单独生成结果、单独做额外静态验证。

**截至本指南生成时，三块物理平面均未完成现场采集；不得把任何一套测试夹具或离线结果称为“现场已标定”。** 只有完成现场九点采集、残差复核、独立中心误差复核和额外独立点静态验证后，才能进入下一阶段的现场评审。本工具不授权机器人、夹爪或 AGV 自动运动，也不授权真实抓取或放置。

核心安全原则：

- 全程只使用 `toolcoord2`；相机固定为 640×480 彩色图。
- 开始前必须确认物理急停可用，并由一名现场人员专职守护急停。
- 所有机械臂移动只能由人员在 WebApp 中低速手动完成。脚本只保存图像、追加会话事件和读取当前 TCP，不发送运动指令。
- 远程画面只能用于观察，不能代替现场急停观察员。
- 机器人必须完全停稳后，才能点击像素或按 Enter 读取 TCP。
- 同一会话内，A4 纸、硬板、相机分辨率、工具坐标、参考拍照几何和相对工作距离不得改变。纸张或硬板一旦移动，保留旧会话并重新开始，不要拼接数据。

## 1. 制作九点 A4 目标

1. 使用哑光白色 A4 纸，画九个细而清晰的黑色十字。十字中心就是采集点，不要用粗色块遮住中心。
2. 将纸平整固定到刚性板上；四个边和中间都不能翘起。采集和手动示教期间硬板不得移动。
3. 按下图的编号和顺序制作十字。`0` 到 `7` 是外圈拟合点，`8` 是最后采集的中心独立检查点：

```text
0 --------- 1 --------- 2
|                       |
3           8           4
|                       |
5 --------- 6 --------- 7

采集顺序：0 -> 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7 -> 8
```

4. 九个十字必须全部位于图像内，且尽量分布到视野外圈。中心点 `8` 不参与外圈八点的 holdout 拟合；外圈 holdout 只用点 `0` 到 `7` 预测点 `8`。点 `8` 会加入最终九点单应拟合以及最终平面拟合。
5. 保持纸面稳定、无强反光。发现十字不清晰、纸张起皱或硬板松动时，先处理目标，不开始有效采集。

## 2. 会话前的固定条件

每次采集前逐项确认并记录：

| 项目 | 要求 |
|---|---|
| 工具坐标 | `toolcoord2` |
| 相机流 | RealSense 彩色，640×480 |
| 目标板 | 同一张 A4、同一块硬板，九点编号和顺序不变 |
| 参考拍照几何 | 规定的标准拍照 RPY、工作距离和相对相机姿态不变 |
| 现场安全 | 物理急停可用，本地观察员在位，WebApp 可低速手动控制 |
| 配置选择 | 操作员显式选择一个 `profile_id`，禁止按高度自动猜测 |

`warehouse_raw_1` 与 `warehouse_raw_2` 必须使用**同一套仓库标准拍照 RPY**；`machine_finished_pick` 必须使用**另外一套加工区拍照 RPY**。三套配置不能互换 RPY，也不能把一个配置的 JSON 复制后改名冒充另一个配置。两个仓库平面仍各自拥有自己的参考 TCP、图像和结果。

`plane_profiles.json` 中的 `standard_photo_description` 只是拍照配置说明；三个条目的 `site_z_parameter` 当前均为显式 `null`，表示现场 Z 尚未配置，`null` 不代表可用的 Z、零值或任何默认值，也绝不能被解释为运动授权。

## 3. 车辆目录与依赖检查

车辆工作目录固定为：

```powershell
Set-Location 'C:\Users\Admin\Desktop\现场调试脚本'
```

确认解释器、脚本和配置存在：

```powershell
D:\anaconda3\envs\agv\python.exe --version
Get-Item .\plane_calibrate.py, .\plane_locate.py, .\plane_report.py, .\plane_profiles.json, .\PLANE_CALIBRATION_GUIDE.md
```

## 4. 三套配置的精确采集命令

一次只运行一个配置，不得开多个采集终端并行执行。以下命令中的 `--outer-tolerance-mm 1.0` 和 `--center-tolerance-mm 1.0` 都是首次现场试验的**临时起始门限**，不是已批准的最终验收标准。现场质量/工艺负责人必须先分别确认外圈 RMS/最大残差门限和中心独立误差门限；若未批准，停止执行。任何调整都必须记录原因，不能为了让某次采集通过而临时放宽。

每个配置开始前，先打印并人工核对 `PROFILE_ID`。结果文件名和命令中的 ID 必须完全一致。

### 4.1 `warehouse_raw_1`

```powershell
Set-Location 'C:\Users\Admin\Desktop\现场调试脚本'
Write-Host 'PROFILE_ID=warehouse_raw_1'
D:\anaconda3\envs\agv\python.exe .\plane_calibrate.py --profile-id warehouse_raw_1 --outer-tolerance-mm 1.0 --center-tolerance-mm 1.0
```

### 4.2 `warehouse_raw_2`

```powershell
Set-Location 'C:\Users\Admin\Desktop\现场调试脚本'
Write-Host 'PROFILE_ID=warehouse_raw_2'
D:\anaconda3\envs\agv\python.exe .\plane_calibrate.py --profile-id warehouse_raw_2 --outer-tolerance-mm 1.0 --center-tolerance-mm 1.0
```

### 4.3 `machine_finished_pick`

```powershell
Set-Location 'C:\Users\Admin\Desktop\现场调试脚本'
Write-Host 'PROFILE_ID=machine_finished_pick'
D:\anaconda3\envs\agv\python.exe .\plane_calibrate.py --profile-id machine_finished_pick --outer-tolerance-mm 1.0 --center-tolerance-mm 1.0
```

### 4.4 中断后继续同一会话

只有命令中打印的 ID 与界面/路径中的 ID 完全一致时才能续采。下面的命令自动选择该配置最新的 JSONL 会话，并在执行前打印会话路径：

```powershell
Set-Location 'C:\Users\Admin\Desktop\现场调试脚本'
$ProfileId = 'warehouse_raw_1'
$Session = Get-ChildItem -LiteralPath .\plane_sessions -Filter "$ProfileId`_*.jsonl" |
    Sort-Object LastWriteTime -Descending |
    Select-Object -First 1
if ($null -eq $Session) { throw '未找到该配置的会话文件' }
Write-Host "PROFILE_ID=$ProfileId SESSION=$($Session.FullName)"
D:\anaconda3\envs\agv\python.exe .\plane_calibrate.py --profile-id $ProfileId --outer-tolerance-mm 1.0 --center-tolerance-mm 1.0 --resume $Session.FullName
```

续采另一个配置时，只把 `$ProfileId` 改成 `warehouse_raw_2` 或 `machine_finished_pick`。不得跨 ID 续采，也不得编辑 JSONL 文件。

## 5. 每次会话的现场操作顺序

### 阶段 A：先采像素

1. 程序连接相机并保存一张不可覆盖的静态参考图。`session_start` 会记录图像文件大小 `image_file_size_bytes` 和 SHA-256 `image_sha256`。
2. 核对打印出的 `PROFILE_ID`、参考图和会话路径。续采、追加事件或最终求解前，程序都会重新校验参考图；若图像缺失、大小变化或 SHA-256 变化，必须停止并保留会话，不能继续拼接数据。
3. 在固定图像窗口按 `0` 到 `8` 的顺序点击九十字中心，顺序必须与纸面编号一致。
4. 点错时按 `u` 撤销最后一个点，再按正确顺序继续。不得用修改图像或删除事件来掩盖对应关系错误。
5. 九点齐全后按 Enter，进入 TCP 示教阶段。按 `q` 或 Esc 可中止；会话会保留，供安全续采。每个像素、示教点和 undo 事件都会在追加时写入带时区的 ISO 时间戳。

### 阶段 B：再手动示教 TCP

1. 保持参考图窗口可见。由现场人员通过 WebApp 低速手动移动机械臂。
2. 按 `0` 到 `8` 的同一顺序，将 `toolcoord2` 的 TCP 对准每个实体十字中心。
3. 每次对准后先让机器人完全停稳，再由现场急停观察员确认，最后按 Enter 读取当前 TCP。
4. TCP 点误差、顺序或纸面移动必须当场记录。不要为了“拟合更好”删除或替换异常点；先查清点选对应关系、纸张位移、工具坐标、TCP 对准和相机设置。
5. 九点齐全后按 Enter 完成并生成当前结果；按 `u` 撤销最后一个示教点，按 `q` 中止并保留会话。

完成采集后必须检查以下产物：

- `plane_sessions\<profile_id>_<timestamp>.png`：不可修改的参考图。
- `plane_sessions\<profile_id>_<timestamp>.jsonl`：追加式原始事件；首行含参考图大小/SHA-256，后续事件含时区时间戳。
- `plane_results\<profile_id>.json`：当前版本化结果。
- `plane_results\<profile_id>.txt`：UTF-8 可读报告。
- 若该配置已有旧结果，还会生成带 `.backup-<timestamp>.json` 的备份；不得删除旧备份或旧会话。

## 6. 结果复核：先看残差，再看中心独立误差

程序必须先只用外圈 `0` 到 `7` 拟合，计算外圈逐点残差、平面 RMS 和最大残差，并写入 `outer_fit_metrics`。外圈 RMS 和最大残差必须同时不超过本次批准的 `outer_tolerance_mm`；任一超限都必须在中心检查和最终九点拟合之前拒绝。随后程序用同一个外圈单应预测中心 `8`，其 `independent_center_error_xy_mm` 必须不超过 `center_tolerance_mm`；中心通过后才允许最终九点单应和平面拟合。`fit_metrics` 只是最终拟合指标，不能替代前两道门限。

逐点检查报告中的 `outer point 0` 到 `outer point 7` 残差，并重点核对编号、方向和量级；再逐点检查最终 `point 0` 到 `point 8` 残差。中心 `8` 不参与外圈 holdout 拟合，但会参与最终九点单应拟合和最终平面拟合。任一外圈或中心门限超限，都不得生成可用结果，更不能自动运动。

出现下列任一情况时，不要继续进入运动集成：

- 点位编号或点击/示教顺序不一致。
- `outer_fit_metrics` 的 RMS 或最大残差超过 `outer_tolerance_mm`。
- 中心独立误差超门限。
- 参考图、A4、硬板、相机分辨率或工具坐标在会话中发生变化。
- 报告中的 `profile_id`、参考 TCP、RPY 与手工记录不一致。

处理方式是保留原会话并重新检查；必要时新建独立会话重采。**禁止通过删点、改 JSONL、改结果 JSON 或临时放宽门限来让指标变好。**

## 7. 额外独立点的离线静态验证

完成结果复核后，还要在该替代平面上选择至少三个不在原九点内的独立十字/标记点，分布到不同区域。每个点都按以下顺序操作：人工通过 WebApp 手动移动到标准拍照位并停稳；记录当前 TCP；记录对应像素；最后用 `--tcp` 做纯离线计算。不要在连接自动运动程序的情况下执行本验证。

下面以 `warehouse_raw_1` 为例。`$U`、`$V` 是零基像素坐标；`$X`、`$Y`、`$Z` 单位为 mm；`$RX`、`$RY`、`$RZ` 单位为度。每次输入现场实测值，并在运行前打印核对：

```powershell
Set-Location 'C:\Users\Admin\Desktop\现场调试脚本'
$ProfileId = 'warehouse_raw_1'
$U = Read-Host '独立点 U（零基像素）'
$V = Read-Host '独立点 V（零基像素）'
$X = Read-Host '当前 TCP X（mm）'
$Y = Read-Host '当前 TCP Y（mm）'
$Z = Read-Host '当前 TCP Z（mm）'
$RX = Read-Host '当前 TCP RX（deg）'
$RY = Read-Host '当前 TCP RY（deg）'
$RZ = Read-Host '当前 TCP RZ（deg）'
Write-Host "PROFILE_ID=$ProfileId PIXEL=($U,$V) TCP=($X,$Y,$Z,$RX,$RY,$RZ)"
D:\anaconda3\envs\agv\python.exe .\plane_locate.py --profile ".\plane_results\$ProfileId.json" --profile-id $ProfileId --pixel $U $V --tcp $X $Y $Z $RX $RY $RZ
```

对另外两套配置做同样验证时，只改第一行，例如：

```powershell
$ProfileId = 'warehouse_raw_2'
```

或：

```powershell
$ProfileId = 'machine_finished_pick'
```

每次静态定位必须同时满足：

- 输出的 `profile_id` 与手工记录完全一致。
- 配置中的 `rpy_tolerance_deg` 必须精确等于 `0.5`；`0.49`、`0.51`、`1.0`、`2.0` 等值都必须被拒绝，不能手工编辑放宽。
- 当前 TCP 的 RPY 与配置参考 RPY 的旋转差不超过 `0.5` 度；超差时程序必须拒绝，不能绕过。
- 输出的 `base_xy_mm` 与独立实测/标尺结果在本次批准门限内。
- 输出字段恰为八项：`profile_id`、`image_size`、`schema_version`、`base_xy_mm`、`diagnostic_plane_z_mm`、`base_point_mm`、`pixel`、`motion_authorized`。
- `schema_version` 为整数 `1`，`motion_authorized` 必须为 `false`。
- 程序最后打印 `仅静态验证，诊断Z不得直接用于运动。`

把每个独立点的像素、TCP、基座 XY、误差、门限和结论写入现场记录。只有三套配置都完成该验证，才具备进入后续集成的评审条件。

## 8. 显式同步到车辆目录

新增脚本、配置和本指南必须**显式逐项同步**到 `C:\Users\Admin\Desktop\现场调试脚本\`。不得把整个旧目录覆盖过去，不得用同名新脚本覆盖旧的 `handeye_*` 或 `grasp_*` 文件。`plane_sessions` 和 `plane_results` 是一次性现场证据，永远不得被同步操作覆盖。

先运行以下**只做检查和建目录、不复制文件**的冲突预检。只要目标目录已有任一计划写入的同名文件，命令就停止，不会覆盖任何文件：

```powershell
$Source = 'C:\Users\飞机炸弹\Desktop\比邻星AGV 资料\现场调试脚本'
$Target = 'C:\Users\Admin\Desktop\现场调试脚本'

foreach ($Directory in @($Target, "$Target\plane_sessions", "$Target\plane_results")) {
    if (-not (Test-Path -LiteralPath $Directory -PathType Container)) {
        New-Item -ItemType Directory -Path $Directory | Out-Null
    }
}

$ScriptNames = @(
    'plane_math.py', 'plane_session.py', 'plane_calibrate.py',
    'plane_locate.py', 'plane_report.py', 'plane_profiles.json',
    'PLANE_CALIBRATION_GUIDE.md'
)
$Collisions = @()

foreach ($Name in $ScriptNames) {
    $Destination = Join-Path $Target $Name
    if (Test-Path -LiteralPath $Destination) {
        $Collisions += $Destination
    }
}

foreach ($Folder in @('plane_sessions', 'plane_results')) {
    $SourceFolder = Join-Path $Source $Folder
    if (Test-Path -LiteralPath $SourceFolder -PathType Container) {
        foreach ($SourceFile in Get-ChildItem -LiteralPath $SourceFolder -File) {
            $Destination = Join-Path (Join-Path $Target $Folder) $SourceFile.Name
            if (Test-Path -LiteralPath $Destination) {
                $Collisions += $Destination
            }
        }
    }
}

if ($Collisions.Count -gt 0) {
    $Collisions | ForEach-Object { Write-Host "CONFLICT: $_" }
    throw '同步已停止：目标存在同名文件，未覆盖任何文件。'
}
Write-Host '预检通过：目标不存在计划写入的同名文件。'
```

只有预检通过后，才执行以下**非 Force** 复制。脚本/配置如需更新已有版本，必须先单独备份并经批准后人工移走旧文件，再重新运行预检；会话和结果不得删除、移动或覆盖。

```powershell
foreach ($Name in $ScriptNames) {
    Copy-Item -LiteralPath (Join-Path $Source $Name) -Destination (Join-Path $Target $Name)
}

foreach ($Folder in @('plane_sessions', 'plane_results')) {
    $SourceFolder = Join-Path $Source $Folder
    if (Test-Path -LiteralPath $SourceFolder -PathType Container) {
        foreach ($SourceFile in Get-ChildItem -LiteralPath $SourceFolder -File) {
            $DestinationFolder = Join-Path $Target $Folder
            Copy-Item -LiteralPath $SourceFile.FullName -Destination (Join-Path $DestinationFolder $SourceFile.Name)
        }
    }
}

Get-ChildItem -LiteralPath $Target -File | Select-Object Name, LastWriteTime, Length
```

同步后复核版本化配置和非授权状态：

```powershell
Get-ChildItem -LiteralPath .\plane_results -Filter '*.json' | ForEach-Object {
    $Profile = Get-Content -Raw -LiteralPath $_.FullName | ConvertFrom-Json
    [pscustomobject]@{
        File = $_.Name
        ProfileId = $Profile.profile_id
        SchemaVersion = $Profile.schema_version
        MotionAuthorized = $Profile.motion_authorized
    }
} | Format-Table -AutoSize
```

当前结果、备份和会话都必须保留。不要复制、删除或覆盖旧 `handeye_data.jsonl`、`handeye_result.json`、`handeye_report.txt`、`grasp_offset.json`。

## 9. 明确不在本工具授权范围内的事项

- **真实现场 Z**：诊断 Z 只由替代平面几何推算，不能写入抓取或放置 Z。实际作业面下降量必须另行现场示教和验证。
- **仓库成品槽位位姿**：本工具不做槽位选择、成品槽映射或放置位姿安全验证。
- **车上取放位姿**：机械臂底座到车上工位的轨迹和位姿仍需独立示教、空跑和验收。
- **传感器与槽位状态**：光电/接近传感器、空位判定、选槽逻辑和异常处理未接入。
- **加工握手与任务状态**：PLC、加工完成握手、YOLO 任务、自动调度和状态机未实现，不能由本工具推断任务是否安全。
- **自动运动授权**：所有结果都带 `motion_authorized=false`；脚本没有 MoveL、MoveJ、夹爪、AGV 导航或自动抓取授权。

## 10. 停机条件与完成声明

遇到急停触发、相机丢帧、工具坐标错误、A4/硬板移动、TCP 读取异常、点序不确定、残差异常、RPY 超差、结果 ID 不匹配或任何人员对现场安全有疑问时，立即停止采集，保留会话，不生成或替换结果，等待现场负责人处理。

只有在三套配置分别完成以下全部工作后，才能向现场负责人提交“静态采集与静态验证完成”的结论：

1. 三套独立九点会话和版本化结果均已生成。
2. 外圈残差、最终拟合指标和独立中心误差均已复核。
3. 每套配置至少三个额外独立点的 `plane_locate.py --tcp` 静态验证通过。
4. 三套结果均确认 `schema_version=1`、八字段定位输出和 `motion_authorized=false`。
5. 旧 `handeye_*` 与 `grasp_*` 文件未被修改。

即便如此，也只能声明静态标定与静态验证完成，**不能声明真实抓取、自动运动、真实 Z、槽位放置或加工任务已经验收**。真实硬件验收必须另行执行。
