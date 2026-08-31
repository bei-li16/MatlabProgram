# PMSM FOC v2.1.0 需求追溯矩阵

## 1. 范围与判定规则

本文将原始软件架构 `PMSM_FOC_Software_Architecture.tex` 的 M0～M10 拆分为唯一需求编号，并建立“需求 → 模型路径 → 生成代码 → 测试 → 证据”的闭环。矩阵状态只允许以下四种：

- **已满足**：实现、自动测试及证据均存在。
- **部分满足**：已有可验证实现，但原始验收范围尚未全部覆盖。
- **未满足**：尚无符合要求的实现或测试。
- **明确延期**：已评审并安排到后续里程碑，不得按完成统计。

本矩阵基于 2026-08-31 的完全清理重建。控制器 checksum 为 `F3830E96A301CAE4DD5FADBB6B09AFD8`，Harness checksum 为 `5D7F7808CAEFDB7F3253245556780ACB`，生成接口签名为 `2BI-1BO-13C-6S`，总门禁为 PASS。主要自动证据为根目录 `verification_report.txt`。

## 2. M0～M10 追溯矩阵

| 需求 ID | 来源 | 可验证需求 | 状态 | 模型/数据落点 | 生成代码符号或文件 | 测试与证据 | 偏离、理由与风险 |
|---|---|---|---|---|---|---|---|
| REQ-M0-001 | SRC-A8(M0)、SRC-A4 | 控制参数、保护阈值、单位、范围、所有者、版本和参数分类应由统一数据源管理。 | 已满足 | `PMSM_FOC_Data.sldd`；`PMSM_FOC_Parameter_Catalog`；两个模型均绑定该字典 | `PMSM_FOC_DualPlant_Controller_v21.c/.h` 中 `FOC_Native_*`、`PMSM_Protection_*`、`PMSM_Alignment_*` | `verification_report.txt`：31 个参数、版本 2.1.0、模型工作区影子数 0、总门禁 PASS | CRC 目前为导出流水线占位符，真实 CRC 计算留给 ARC-005；错误使用占位值上线会降低标定可追溯性。 |
| REQ-M0-002 | SRC-A8(M0)、SRC-A6 | 命令、测量、状态和标定接口应有类型化契约，并与生成代码一致。 | 已满足 | `ControlCommandBus`、`MeasurementBus`、`ControlStatusBus`、`CalibrationBus`；控制器根端口 2 入 1 出 | `ControlCommandBus.h`、`MeasurementBus.h`、`ControlStatusBus.h`；`ExtU/ExtY` 使用这些类型 | `verification_report.txt`：39 个接口元素、四类 Bus 元素验证、代码类型验证、`2BI-1BO-13C-6S` | Bus 中显式启停/急停字段已定义，但控制语义仍沿用速度阈值，属于 ARC-003；误认为已启用显式命令会造成集成风险。 |
| REQ-M1-001 | SRC-A8(M1) | 电气 RL 和机械参数应有量纲、范围和可复现实验基线。 | 部分满足 | 数据字典中的 `Ld/Lq/FluxPM/PolePairs`；`Native_Discrete_PMSM` | 参数在生成代码中可见；植物仅用于 Harness，不属于控制器 ERT 目标 | 双被控对象仿真均 PASS；速度、Iq、Duty、Torque 指标写入 `verification_report.txt` | 尚缺独立 RL/机械阶跃用例和参数辨识报告；当前闭环通过不能替代对象级验收。 |
| REQ-M1-002 | SRC-A8(M1) | 速度 PI 与 d/q 电流 PI 应离散化、限幅并可复位。 | 已满足 | `Speed_PI_Controller_1ms`、`D_Axis_Current_PI`、`Q_Axis_Current_PI` | `PMSM_FOC_DualPlant_Controller_v21_step` 中 PI 状态、限幅与复位逻辑 | 架构门禁检查 3 个 PI 复位端口；正常、故障复位场景均 PASS | 目前 d/q 电压独立限幅，尚无电压矢量圆和抗饱和回算；动态性能风险由 CTL-001/002 处理。 |
| REQ-M2-001 | SRC-A8(M2) | PMSM dq/离散对象应体现转矩、反电动势和机械动态。 | 部分满足 | `Selectable_PMSM_Plant/Native_Discrete_PMSM` 与 MathWorks PMSM HDL 变体 | 不生成到控制器代码 | Native 与 MCB 两对象闭环指标均 PASS | 尚未新增专门的 Iq→Torque、速度→BEMF 对象级断言，故不能标记完全满足。 |
| REQ-M2-002 | SRC-A8(M2) | 平均值逆变器与 PMSM 对象应可替换且默认选择有记录。 | 已满足 | `Native_Average_Inverter`、`Selectable_PMSM_Plant`，默认选择 2 | 控制器输出 `ControlStatus.DutyA/B/C` | 变体数 2、MCB 引用校验、单击切换控件和双对象仿真均 PASS | 目前逆变器不含死区/器件压降；这些非理想因素属于 M4。 |
| REQ-M3-001 | SRC-A8(M3) | Clarke、Park、反 Park、反 Clarke 和 SVPWM 应为职责独立组件。 | 已满足 | `Clarke_Transform`、`Park_Transform`、`Inverse_Park_Transform`、`Inverse_Clarke_Transform`、`SVPWM_Duty_Calculation` | `PMSM_FOC_DualPlant_Controller_v21.c`；每步仅 1 次 `sinf` 和 1 次 `cosf` | 13 个组件存在、无悬空线、代码无 S-Function、双对象仿真 PASS | 组件为虚拟子系统，代码优化后名称不一定逐一保留；追溯依赖模型路径和运算特征。 |
| REQ-M3-002 | SRC-A8(M3)、SRC-U4 | 快环 100 µs、慢环 1 ms 应有明确边界，慢环不得被复制到快环。 | 已满足 | `IqRef_Rate_Transition`、`FastFault_To_1ms`、`StatusState_To_100us`、`StatusFault_To_100us` 等 | `TaskCounters.TID[1]`，10:1 调度 | 速度/Stateflow/慢保护为 1 ms，电流/快保护为 100 µs；代码双速率门禁 PASS | 状态输出 Bus 在 100 µs 接口更新，但慢状态仅通过保持边界同步，未增加慢算法执行次数。 |
| REQ-M4-001 | SRC-A8(M4) | 电流采样应包含零偏校准、有效性和新鲜度信息。 | 部分满足 | `Current_Offset_Calibration_100us`；`MeasurementBus.Valid/TimestampSeconds/FreshnessTicks` | 校准累加、完成和偏移逻辑进入控制器 C；有效性字段已生成 | 100 点偏置注入与估计误差测试 PASS | `Valid/FreshnessTicks` 目前仅进入接口/诊断，尚未驱动安全降级；由 ARC-004/MEAS-001 完成。 |
| REQ-M4-002 | SRC-A8(M4) | 应建模双电阻可采样窗口、重构、死区和不可测区降级。 | 未满足 | 当前仅使用 Ia/Ib Clarke；无窗口判定、死区或重构状态机 | 无对应生成符号 | TODO：MEAS-001、MEAS-002、PLANT-001 | 这是硬件相关高风险缺口；在完成前不得把当前模型作为两电阻量产采样实现。 |
| REQ-M5-001 | SRC-A8(M5) | 编码器角度闭环、速度环和电流环应形成级联 FOC。 | 部分满足 | `ElectricalAngleRad` 测量输入、1 ms 速度 PI、100 µs 电流 PI | `Measurement.ElectricalAngleRad`、`Measurement.MechanicalSpeedRpm` | 正常启停、速度闭环和负载对象仿真 PASS | 角度直接由植物提供，尚无编码器计数、方向、零位和跳变诊断；由 MEAS-001 补齐。 |
| REQ-M5-002 | SRC-A8(M5) | 应验证正反转、速度阶跃、负载阶跃和停机过程。 | 部分满足 | Harness 的速度/负载源与现有日志 | 控制器支持有符号速度参考字段 | 当前正常场景和故障停机 PASS | 缺少自动正反转和多工况阶跃矩阵；由 TEST-220/TEST-221 扩展。 |
| REQ-M6-001 | SRC-A8(M6)、SRC-U3 | Stateflow 应实现 INIT、READY、CALIB、ALIGN、RUN、FAULT 及超时。 | 已满足 | `Motor_Supervisor_1ms`，6 状态和 `SUPERVISED` 域 | 生成代码中的 Stateflow 状态枚举、`is_active_`/状态逻辑 | 状态数 6；正常访问 `[1 2 3 4 5]`；RUN 进入 0.08 s；PASS | 启动命令仍由速度阈值隐式产生，须由 ARC-003 改为显式命令优先级。 |
| REQ-M6-002 | SRC-A8(M6) | 故障应从任意运行状态锁存，只有故障消失且收到确认复位才能退出。 | 已满足 | `Fast_Safety_Gate_100us`、`Slow_Safety_Monitor_1ms`、`Motor_Supervisor_1ms` | 快故障锁存、慢故障、FAULT→INIT 复位逻辑 | 全状态故障矩阵 PASS；快故障 PWM 延迟 0；确认复位 PASS | 故障位当前是聚合占位，缺少稳定故障编号和历史，属于 ARC-004。 |
| REQ-M7-001 | SRC-A8(M7) | 应实现 BEMF 观测器并与编码器角度比较。 | 明确延期 | 无传感器子系统尚未建立 | 无 | TODO：OBS-001 | 计划在 v2.3.0 实施；提前移除编码器会造成低速失锁风险。 |
| REQ-M7-002 | SRC-A8(M7) | 应实现 Tracking Observer 并验证加减速不发散。 | 明确延期 | 尚无 | 无 | TODO：OBS-002 | 依赖编码器真值基线和 M4 测量质量。 |
| REQ-M8-001 | SRC-A8(M8) | 应实现 Force/Tracking/Sensorless 三模式及进入/退出条件。 | 明确延期 | 尚无模式管理子系统 | 无 | TODO：OBS-003 | 计划在观察器稳定后实施；当前禁止宣称无传感器启动能力。 |
| REQ-M8-002 | SRC-A8(M8) | 模式切换应无明显电流冲击并具备回退策略。 | 明确延期 | 尚无 | 无 | TODO：OBS-003、TEST-230 | 需要负载矩阵和故障注入；当前风险为切换瞬态失控。 |
| REQ-M9-001 | SRC-A8(M9) | 控制器应生成可编译的 ERT C 代码，并保留双速率和接口类型。 | 已满足 | `PMSM_FOC_DualPlant_Controller_v21.slx` | `PMSM_FOC_DualPlant_Controller_v21_step`、`TaskCounters.TID[1]`、三个 Bus 头文件 | MinGW 编译成功；代码结构、双速率、Bus 类型、sin/cos 次数和无 S-Function 均 PASS | 目标硬件工具链尚未验证；当前通过仅代表 Windows x86-64 参考构建。 |
| REQ-M9-002 | SRC-A8(M9) | 应完成 SIL/PIL、目标实时任务映射、WCET 和竞态审查。 | 部分满足 | 有 ERT 代码与确定性 10:1 调度模型 | Windows 可执行文件已构建 | 当前仅模型仿真和主机构建 PASS | 尚无 SIL 数值比对、PIL、ISR/WCET 或并发竞态报告；由 TEST-222 和 HW-001 完成。 |
| REQ-M10-001 | SRC-A8(M10) | 应在低压实机验证相序、采样、保护和紧急停机。 | 明确延期 | 无硬件台架证据 | 无 | TODO：HW-001、HW-002 | 软件门禁不能替代上电检查；未完成前禁止实机高能量测试。 |
| REQ-M10-002 | SRC-A8(M10) | 应扩展到目标电压、温度和负载覆盖并冻结标定文件。 | 明确延期 | 无 | 无 | TODO：HW-003、REL-001 | 依赖 M4、M7～M9 和低压台架通过。 |

## 3. 状态汇总

| 状态 | 数量 | 需求 ID |
|---|---:|---|
| 已满足 | 9 | M0-001、M0-002、M1-002、M2-002、M3-001、M3-002、M6-001、M6-002、M9-001 |
| 部分满足 | 6 | M1-001、M2-001、M4-001、M5-001、M5-002、M9-002 |
| 未满足 | 1 | M4-002 |
| 明确延期 | 6 | M7-001、M7-002、M8-001、M8-002、M10-001、M10-002 |

## 4. 使用与维护

1. 每次关闭一级 TODO 时，必须更新对应需求行的模型路径、代码符号、测试和证据。
2. 状态由“部分满足/未满足/明确延期”改为“已满足”前，必须有自动测试结果或经评审的硬件报告。
3. 运行 `build_pmsm_foc_dualplant_v21('CleanBuild',true,'BatchMode',true,'ExportImages',false)` 后，以新 checksum、接口签名和 `verification_report.txt` 更新本文。
4. 接口、参数或故障编号变化时，必须同步 `PMSM_FOC_Data.sldd`、生成头文件、本文和 TeX/PDF 手册。
