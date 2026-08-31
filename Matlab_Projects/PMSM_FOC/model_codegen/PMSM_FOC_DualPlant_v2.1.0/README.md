# PMSM FOC Dual Plant v2.1.0

该版本在 `PMSM_FOC_Native_v2.0.0` 的离散 FOC 闭环模型上加入可切换的 MathWorks Motor Control Blockset PMSM HDL 被控对象，并把控制调度重构为 100 us 快速电流/过流保护任务与 1 ms 速度/状态监督任务。原 v2.0.0 目录和模型保持不变；两个电机对象共用同一组件化 FOC 控制器、平均逆变器、状态机、工况和信号接口，可直接比较控制效果。

## 文件与模型

- `PMSM_FOC_DualPlant_ClosedLoop_v21.slx`：闭环仿真顶层模型。
- `PMSM_FOC_DualPlant_Controller_v21.slx`：可独立生成 ERT C 代码的控制器模型。
- `PMSM_FOC_Data.sldd`：31 个控制/保护参数、四类 Bus 对象及参数/接口元数据的唯一设计数据源。
- `create_pmsm_foc_data_dictionary_v21.m`：幂等创建/更新数据字典、参数分类、接口目录和版本/CRC 策略。
- `apply_pmsm_foc_bus_interface_v21.m`：将控制器标量内部实现封装为两个输入 Bus 和一个状态输出 Bus。
- `build_pmsm_foc_dualplant_v21.m`：配置模型、依次仿真两个电机对象、绘图、导出架构图、生成代码并写入验收报告。
- `verify_pmsm_foc_reproducibility_v21.m`：连续执行两次完全清理重建并比较模型语义校验和、配置、接口和数值结果。
- `refactor_pmsm_foc_controller_v21.m`：清空旧顶层连线，并按经典级联 FOC 拓扑重建两个 v2.1.0 模型。
- `add_pmsm_foc_stateflow_v21.m`：幂等集成 Stateflow 状态机、电流偏置校准、转子对齐、安全监测和 PWM 门控。
- `switch_pmsm_plant_v21.m`：停止仿真时切换 Variant 选择。
- `verification_report.txt`：最近一次自动验收数据。
- `baseline_reproducibility_report.txt`：两次 clean build 的可复现性对比证据。
- `PMSM_FOC_DualPlant_v21_results.png`：关键曲线对比。
- `PMSM_FOC_DualPlant_v21_stateflow_results.png`：正常启动与故障注入状态曲线。
- `doc/PMSM_FOC_DualPlant_v2.1.0_Architecture_and_Test_Manual.tex`：架构与测试手册 LaTeX 源文件。
- `doc/PMSM_FOC_DualPlant_v2.1.0_Architecture_and_Test_Manual.pdf`：编译后的完整手册。
- `doc/PMSM_FOC_v2.1.0_Requirements_Traceability_Matrix.md`：M0～M10 的 22 条唯一需求及模型、代码、测试、证据和风险映射。

## 模型架构

闭环信号链如下：

```text
速度给定 -> Speed PI (1 ms) -> Rate Transition -> Iq PI ─┐
Id*=0 --------------------------------------------> Id PI ├-> DQ 电压合成
                                                        │
PMSM Ia/Ib -> Clarke -> Park -> Id/Iq ------------------┘
                              │
PMSM 转速 --------------------┴-> DQ 解耦前馈 ----------┘

电角度 -> Shared sin/cos -> Park 与逆 Park
DQ 电压 -> 对齐覆盖 -> 逆 Park -> 逆 Clarke -> SVPWM -> 快速安全门 -> 平均逆变器 -> PMSM

1 ms Supervisor/Speed/慢保护 <-> 显式任务边界 <-> 100 us Current FOC/过流门控
```

顶层 Simulink 架构：

![闭环模型顶层架构](PMSM_FOC_DualPlant_v21_architecture.png)

可选 PMSM Variant 子系统：

![双电机 Variant 架构](PMSM_FOC_DualPlant_v21_plant_variants.png)

### FOC 控制组件及责任边界

控制器采用与经典级联 FOC 框图一致的顶层数据流：`Speed_PI_Controller_1ms` 形成速度外环，显式 `IqRef_Rate_Transition` 将 1 ms 的电流给定传递到 100 us 任务；Clarke、共享电角度三角函数、Park、D/Q 轴 PI、解耦前馈、DQ 电压合成、逆 Park、逆 Clarke 和 SVPWM 均直接显示为顶层独立组件。模型不再用一个 `Current_Control_100us` 大子系统遮蔽电流环结构，图中的前向控制链与电流/角度反馈链均可直接追踪。

![组件化 FOC 控制器架构](PMSM_FOC_DualPlant_v21_controller_architecture.png)

| 组件 | 单一责任 | 主要接口 | 关键参数或约束 |
| --- | --- | --- | --- |
| `Speed_PI_Controller_1ms` | 将速度误差转换为 q 轴电流给定，构成速度外环 | `SpeedReferenceRpm`, `SpeedRpm` → `IqReference` | 独立 1 ms 任务；`FOC_Native_KpSpeed`、`FOC_Native_KiSpeed`；±`FOC_Native_IqLimit` |
| `Current_Offset_Calibration_100us` | 对静止电流采样求均值并保持 A/B 相偏置，输出校正电流 | Raw Ia/Ib、校准使能/复位 → Corrected Ia/Ib、Offset、Done | 100 us；`PMSM_Calibration_SampleCount`；INIT/FAULT 清零 |
| `Clarke_Transform` | 两相采样电流转换到静止 α/β 坐标系 | `Ia`, `Ib` → `Ialpha`, `Ibeta` | `Ialpha=Ia`；`Ibeta=(Ia+2Ib)/sqrt(3)` |
| `Electrical_Angle_Trig_100us` | 每个快周期只计算一次电角度基函数并共享 | `ThetaElectrical` → `SinTheta`, `CosTheta` | 100 us；Park/逆 Park 共用一对 sin/cos |
| `Park_Transform` | α/β 电流转换到旋转 d/q 坐标系 | `Ialpha`, `Ibeta`, `SinTheta`, `CosTheta` → `Id`, `Iq` | 使用共享角度基函数；100 us 数据通路 |
| `D_Axis_Current_PI` | 将 d 轴电流调节到 0 A | `Reference=0`, `Id` → `VdPI` | 100 us；电流 PI 参数；±`FOC_Native_CurrentIntegratorLimit` |
| `Q_Axis_Current_PI` | 跟踪速度环给出的 q 轴电流 | `IqReference`, `Iq` → `VqPI` | 100 us；电流 PI 参数；±`FOC_Native_CurrentIntegratorLimit` |
| `DQ_Decoupling_Feedforward` | 补偿 d/q 轴交叉耦合与反电动势 | `SpeedRpm`, `Id`, `Iq` → `VdFeedforward`, `VqFeedforward` | 极对数、`Ld`、`Lq`、永磁磁链 |
| `DQ_Voltage_Command` | 合并 PI 与前馈项并形成有界电压指令 | PI/前馈电压 → `VdCommand`, `VqCommand` | d/q 轴分别限幅至 ±`FOC_Native_VoltageLimit` |
| `Alignment_DQ_Override_100us` | 仅在 ALIGN 状态选择固定对齐电压和角度基函数 | FOC Vd/Vq、反馈 sin/cos、`AlignmentEnable` → Applied Vd/Vq/Sin/Cos | `PMSM_Alignment_*` 字典参数 |
| `Inverse_Park_Transform` | d/q 电压转换回静止 α/β 坐标系 | `Vd`, `Vq`, `SinTheta`, `CosTheta` → `Valpha`, `Vbeta` | 与 Park 共用同一 sin/cos |
| `Inverse_Clarke_Transform` | α/β 电压转换为三相电压 | `Valpha`, `Vbeta` → `Va`, `Vb`, `Vc` | `Va=Valpha`；B/C 相使用 ±`sqrt(3)/2` |
| `SVPWM_Duty_Calculation` | 公共模注入、母线归一化和占空比限幅 | `Va`, `Vb`, `Vc`, `Vdc` → `DutyA/B/C` | `Voffset=-0.5(max+min)`；占空比 0.02～0.98 |

速度环与电流环不再混放在同一个顶层控制器块内。100 us 快任务只保留电流采样校准、Clarke/Park、D/Q 电流 PI、解耦、电压合成、对齐选择、逆变换、SVPWM 和过流最终门控；1 ms 慢任务负责速度 PI、Stateflow 模式转换、过速与母线电压监测。快到慢的锁存故障和校准完成量使用 Rate Transition；慢到快的命令/状态采用显式保持边界，由快任务读取最近的 1 ms 值，不复制一套 100 us 状态任务。

独立控制器根接口已由 7 入/8 出标量改为 `ControlCommandBus`、`MeasurementBus` 两个输入和 `ControlStatusBus` 一个输出，生成签名为 `2BI-1BO-13C-6S`。`CalibrationBus` 作为等价标定契约保存在数据字典中；全部 39 个 Bus 元素都在接口目录中记录类型、单位、范围和采样时间。`FaultResetRequest` 取代顶层 `FaultResetAck` 标量端口。为保持本轮只完成接口契约而不暗改运行语义，`StartRequest/StopRequest/EmergencyStop/Direction/TorqueReferenceNm` 暂作为 ARC-003 的接口占位，当前启动仍沿用速度阈值判定。重构脚本每次先恢复确定性的标量内部实现，再应用 Bus 包装，两个模型的悬空线检查均为 0。

### 数据字典与接口契约

`PMSM_FOC_Data.sldd` 是控制器设计数据的唯一来源。31 个参数分别标识为编译期常量或可标定参数；运行时信号和只读诊断量由 Bus 接口目录管理。每个参数记录名称、目标类型、单位、上下限、默认值、所有者、版本和说明。参数集版本为 `2.1.0`，兼容性标识为 `PMSM_FOC_DUALPLANT_V21`，CRC 字段当前为后续标定导出流水线的明确占位符。两个模型工作区中迁移参数的影子变量数量经自动检查为 0，Harness 专用测试激励仍保留在 Harness 工作区。

| 契约 | 关键元素 | 更新周期/用途 |
| --- | --- | --- |
| `ControlCommandBus` | Start/Stop/EmergencyStop、Direction、Speed/Torque Reference、FaultResetRequest | 外部命令；当前仅速度给定和故障复位已接入既有语义 |
| `MeasurementBus` | Ia/Ib/Ic、Vdc、电角度、机械速度、Valid、Timestamp、Freshness | 100 us 测量链与 1 ms 速度反馈 |
| `ControlStatusBus` | DutyA/B/C、Id/Iq、Vd/Vq、PwmEnable、StateCode、FaultBits/Code、限幅和测量状态 | 100 us 根输出；1 ms 状态经保持边界同步，不增加慢环执行次数 |
| `CalibrationBus` | 电流偏置、对齐电压/时间、样本数、参数版本和 CRC | 字典中的等价标定结构，不增加顶层零散参数线 |

本轮只冻结接口形状：`CurrentLimitActive` 暂映射快速过流锁存，`VoltageLimitActive` 暂为 `false`；完整电流限幅/电压矢量饱和语义分别由 ARC-003 和 CTL-001/002 实施。调用方不得把这两个占位诊断当作已完成的降额控制。

完整元素、范围、采样时间和 M0～M10 追溯关系见 `doc/PMSM_FOC_v2.1.0_Requirements_Traceability_Matrix.md`。

### Stateflow 电机运行状态管理

`Motor_Supervisor_1ms` 以 1 ms 周期管理运行模式和监督 PWM 请求。`SUPERVISED` 父状态统一包含 INIT、READY、CALIB、ALIGN、RUN，父状态只用一条故障迁移进入顶层 FAULT，避免每个状态各自连接故障线造成交叉和红线堆叠。最终 PWM permit 不由慢状态机独占：`Fast_Safety_Gate_100us` 可在同一快周期因过流强制关断。

![Stateflow 状态机架构](PMSM_FOC_DualPlant_v21_stateflow_architecture.png)

| 状态码 | 状态 | 责任 | 主要转换条件 | Control/PWM |
| ---: | --- | --- | --- | --- |
| 1 | INIT | 确定的上电/复位初态 | 1 tick 后进入 READY | 0 / 0 |
| 2 | READY | 等待启动命令 | `StartCmd` 后进入 CALIB | 0 / 0 |
| 3 | CALIB | 启动 100 样本电流偏置均值计算 | `CalibrationDone` 后进入 ALIGN；15 个 1 ms tick 超时进 FAULT | 0 / 0 |
| 4 | ALIGN | 施加 2 V d 轴电压、零 q 轴和固定角度 | 20 个 1 ms tick 后进入 RUN；撤销启动回 INIT | 0 / 1 |
| 5 | RUN | 放行速度指令和闭环 SVPWM | 撤销启动回 INIT；故障进 FAULT | 1 / 1 |
| 6 | FAULT | 关闭 PWM、清零校准和 PI 状态并保持锁存 | 仅 `FaultResetRequest && !FaultDetected && !StartCmd` 回 INIT | 0 / 0 |

`Fast_Safety_Gate_100us` 使用 `PMSM_Protection_MaxCurrent_A` 执行同周期 PWM 禁止并锁存快速故障；只有故障源消失且收到 `FaultResetRequest` 才清除锁存。`Slow_Safety_Monitor_1ms` 使用 `PMSM_Protection_MaxSpeed_Rpm`、`PMSM_Protection_MinDcBus_V` 和 `PMSM_Protection_MaxDcBus_V` 监测慢故障。快故障经 Rate Transition 送入 `Motor_Supervisor_1ms`，状态机负责进入锁存 FAULT；最终三相门控始终由快速安全门决定。ALIGN/RUN 以外的状态使用 `PMSM_SafeDuty`。闭环模型把测试信号收拢在 Harness 专用日志子系统中，该子系统不进入 ERT 目标。

两个电机对象的公共接口为：

| 方向 | 信号 | 含义 |
| --- | --- | --- |
| 输入 | `VAlpha`, `VBeta` | 静止 α/β 坐标系定子电压指令 |
| 输入 | `LoadTorque` | 负载转矩，N·m |
| 输出 | `SpeedRpm` | 机械转速，rpm |
| 输出 | `ThetaElectrical` | 电角度，rad |
| 输出 | `Ia`, `Ib` | A/B 相电流，A |
| 输出 | `TorqueNm` | 电磁转矩，N·m |
| 输出 | `Id`, `Iq` | d/q 轴电流，A |

MathWorks 分支将 α/β 电压转换为三相电压后送入官方 `mcbhdlplantlib/PMSM HDL` 块，再将官方块输出转换回上述公共接口。

## 仿真工况与关键参数

仿真使用 MATLAB/Simulink R2024a，求解器为 `FixedStepDiscrete`，仿真时长 2.0 s。

| 类别 | 参数 | 数值 |
| --- | --- | ---: |
| 工况 | 速度给定 | 0.05 s 时由 0 阶跃至 1000 rpm |
| 工况 | 负载转矩 | 0.5 s 时由 0 阶跃至 0.2 N·m |
| 电源 | 直流母线电压 | 48 V |
| 调度 | 电流环/基础步长 | 100 us |
| 调度 | 速度环周期 | 1 ms |
| 调度 | Stateflow 监督/慢保护周期 | 1 ms |
| 调度 | 快速过流门控周期 | 100 us |
| 速度环 | `Kp`, `Ki` | 0.02, 0.05 |
| 电流环 | `Kp`, `Ki` | 1.0, 500.0 |
| 限幅 | q 轴电流 | ±8 A |
| 限幅 | 电流积分器 | ±30 |
| 限幅 | 电压指令 | ±26 V |
| 限幅 | 三相占空比 | 0.02～0.98 |
| 电机 | 定子电阻 `Rs` | 0.4 Ω |
| 电机 | d/q 轴电感 `Ld`, `Lq` | 1 mH, 1 mH |
| 电机 | 永磁磁链 `λpm` | 0.05 Wb |
| 电机 | 极对数 | 4 |
| 电机 | 转动惯量 `J` | 0.002 kg·m² |
| 电机 | 黏性阻尼 `B` | 0.0001 N·m·s/rad |

## 关键曲线

下图由同一次脚本运行依次仿真两个 Variant 后生成，包含机械转速、q 轴电流、电磁转矩和 A 相占空比。实线为原生离散 PMSM，虚线为 MathWorks PMSM HDL；两条曲线在当前参数和工况下基本重合。

![双电机关键曲线](PMSM_FOC_DualPlant_v21_results.png)

## 仿真数据与验收结果

最近一次 MATLAB R2024a 自动验证结果：

| 指标 | Native 离散 PMSM | MathWorks PMSM HDL |
| --- | ---: | ---: |
| 最终转速 | 996.191467 rpm | 996.191345 rpm |
| 最大转速 | 1084.86560 rpm | 1084.86560 rpm |
| 最大绝对 `Iq` | 2.14349961 A | 2.14349937 A |
| Duty A 最小值 | 0.0849514306 | 0.0849514306 |
| Duty A 最大值 | 0.915057063 | 0.915057063 |
| 最终电磁转矩 | 0.201538101 N·m | 0.201538414 N·m |
| 闭环限值检查 | PASS | PASS |

两个分支的最终转速在记录精度内一致，最大转速差约 0.00012 rpm；关键曲线和稳态数据表明两个被控对象在当前平均值闭环工况下具有一致的控制响应。

Stateflow 正常启动测试依次访问 `[1 2 3 4 5]`，0.06 s 完成 100 样本电流校准，0.08 s 进入 RUN；注入的 A/B 相偏置 `+0.75/-0.50 A` 被准确估计，校准完成时校正电流均为 0 A。ALIGN 期间实际施加 `Vd=2 V`、`Vq=0 V` 且 PWM 有效。70 V 母线过压由 1 ms 慢保护在 0.001 s 进入 FAULT。另在 RUN 中 0.0903 s 注入 20 A、持续 0.2 ms 的过流脉冲，快速故障和 PWM 禁止均在 0.0903 s 生效，测得关断延迟 0 us；两类故障最终 Duty A 均为 0.5。

故障矩阵分别从 INIT、READY、CALIB、ALIGN 和 RUN 注入短故障脉冲；在故障源清除且没有确认时，五个场景均保持 FAULT。独立确认测试在 0.004 s 清除故障源，状态持续锁存至 0.008 s 的 `FaultResetRequest` 脉冲才退出，最终进入 READY。上述测试均 PASS。

![Stateflow 正常与故障测试](PMSM_FOC_DualPlant_v21_stateflow_results.png)

结构与代码检查同时确认：Variant 分支数为 2，官方块引用为 `mcbhdlplantlib/PMSM HDL`；13 个独立控制组件、6 个运行状态和显式任务边界均存在；控制器为 2 个 Bus 输入/1 个 Bus 输出，31 个数据参数、39 个接口元素、4 类 Bus 均通过检查，两个模型悬空线数均为 0。ERT 代码生成独立的 `ControlCommandBus.h`、`MeasurementBus.h` 和 `ControlStatusBus.h`，包含 100 us 基础步长和每 10 步执行一次的 1 ms 子任务、Stateflow、快/慢保护及 `initialize/step` 接口；共享角度组件使生成代码只有 1 次 `sinf` 和 1 次 `cosf` 调用，未检测到 S-Function 文本。

## 切换电机对象

模型工作区变量 `PMSM_PLANT_SELECTION` 控制 Variant：

- `1`：`Native_Discrete_PMSM`
- `2`：`MathWorks_MCB_PMSM_HDL`（默认）

仿真停止时，可单击模型中的 Dashboard `Switch_PMSM_Plant` 按钮切换电机分支；按钮会调用 `switch_pmsm_plant_v21.m` 并显示新的活动对象。下一次开始仿真时编译所选分支。旧版富文本超链接已删除，因为 Simulink 导出 PNG 时会把其 HTML 原文铺在架构图上。

## 重建与复核

在 MATLAB 中将当前目录切换到本目录并运行：

```matlab
build_pmsm_foc_dualplant_v21
```

脚本会依次验证两个被控对象、正常启动、数值校准、对齐输出、1 ms 过压、100 us 快速过流、五状态故障锁存矩阵和确认复位，刷新文档 PNG，核对 13 个控制组件、`2BI-1BO` 根接口、31 个参数、39 个 Bus 元素、6 个 Stateflow 状态、显式任务边界和接口连线，重建控制器 ERT C 代码并更新 `verification_report.txt`。生成 C 代码的目标仅为控制器模型；两个 PMSM 被控对象和 Harness 测试日志不进入控制器目标代码。

完全清理重建和无人值守运行可使用：

```matlab
build_pmsm_foc_dualplant_v21( ...
    'CleanBuild', true, 'BatchMode', true, 'ExportImages', false)
```

`CleanBuild=true` 会从只读基线目录 `PMSM_FOC_Native_v2.0.0` 覆盖两个 v2.1.0 模型，再确定性地应用组件化、双速率和 Stateflow 重构。重复构建或 CI 可用 `ExportImages=false` 降低峰值内存；功能曲线、全部测试、结构检查和代码生成仍会执行。常规交互构建默认导出架构 PNG。

冻结/复核基线时运行：

```matlab
verify_pmsm_foc_reproducibility_v21
```

该命令要求两次 clean build 均通过，并比较控制器/Harness 语义 checksum、`2BI-1BO-13C-6S` 接口签名、模型配置和关键数值指标。2026-08-31 本机 R2024a 双轮结果均为 PASS：控制器 checksum 为 `F3830E96A301CAE4DD5FADBB6B09AFD8`，Harness checksum 为 `5D7F7808CAEFDB7F3253245556780ACB`，Native/MCB 指标最大差值均为 0；完整证据记录在 `baseline_reproducibility_report.txt`。正式发布仍应在已提交、Git clean 的第二台同配置开发机复跑一次。

已验证环境为 Windows 11 x64、MATLAB R2024a（24.1）；所需产品为 Simulink、Stateflow、Motor Control Blockset、Simulink Coder 和 Embedded Coder，均为 24.1。完整环境、许可状态、求解器及代码生成配置见 `verification_report.txt`。
