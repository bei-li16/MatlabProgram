# PMSM FOC Native v2.0.0

这是 PMSM FOC 的第二版基础模型。控制器、平均逆变器和离散 PMSM 被控对象均使用 Simulink 标准组件搭建，不包含 S-Function、MATLAB Function、MATLAB System 或 Stateflow 图表。

## 文件说明

- `PMSM_FOC_Native_Controller_v20.slx`：可独立生成 C 代码的 FOC 控制器顶层模型。
- `PMSM_FOC_Native_ClosedLoop_v20.slx`：闭环仿真模型，包含速度给定、负载阶跃、FOC 控制器、平均逆变器和离散 PMSM 被控对象。
- `PMSM_FOC_Native_Controller_v20_ert_rtw/`：Embedded Coder（ERT）生成的 C/H 源码及接口文件。
- `build_pmsm_foc_native_v2.m`：模型生成、仿真、禁用块扫描、ERT 构建和代码检查脚本。
- `PMSM_FOC_Native_v2_results.png`：闭环仿真结果。
- `verification_report.txt`：自动验收报告。

`slprj/`、`*.slxc`、目标文件和本地可执行文件属于可重新生成的缓存或编译中间产物，已由仓库 `.gitignore` 排除。

## 模型结构

控制器包含：

1. Clarke 与 Park 变换。
2. 1 ms 速度 PI 环，输出受限的 q 轴电流参考值。
3. 100 us 的 d/q 电流 PI 环和交叉耦合前馈补偿。
4. 逆 Park 变换。
5. 基于相电压最大值/最小值公共模注入的 SVPWM 占空比计算。
6. 三相占空比限幅。

闭环对象包含：

- 三相平均逆变器：将 DutyA/B/C 和直流母线电压换算为 alpha/beta 电压。
- 离散 PMSM：由 Gain、Sum、Product、Trigonometric Function、Unit Delay 等标准块实现 alpha/beta 电流、电角度、机械速度和电磁转矩的离散状态更新。
- 48 V 直流母线、1000 rpm 速度阶跃和 0.2 N·m 负载阶跃。

该电机对象适合控制算法闭环联调和代码生成验证，不是开关器件级功率电路或高保真 Simscape Electrical 模型。

## 仿真与代码生成配置

- MATLAB/Simulink：R2024a。
- 求解器：FixedStepDiscrete。
- 控制器基础步长/电流环：0.0001 s（100 us）。
- 速度环：0.001 s（1 ms），生成代码中由 10 次基础步长计数触发。
- 仿真时间：2.0 s。
- 代码生成目标：`ert.tlc`（Embedded Coder），不是 AUTOSAR 目标。
- 数据类型：控制输入、输出、状态和可标定参数主要使用 `single`/`real32_T`。

## 控制器 C 接口

初始化和周期接口：

```c
void PMSM_FOC_Native_Controller_v20_initialize(void);
void PMSM_FOC_Native_Controller_v20_step(void);
void PMSM_FOC_Native_Controller_v20_terminate(void);
```

应用程序先给 `PMSM_FOC_Native_Controller_v2_U` 写入输入，每 100 us 调用一次 `PMSM_FOC_Native_Controller_v20_step()`，再从 `PMSM_FOC_Native_Controller_v2_Y` 读取输出。

输入字段：

- `SpeedReferenceRpm`
- `SpeedRpm`
- `PhaseCurrentA`
- `PhaseCurrentB`
- `ElectricalAngleRad`
- `DcBusVoltage`

输出字段：

- `DutyA`、`DutyB`、`DutyC`
- `IqReference`
- `IdMeasured`、`IqMeasured`
- `VdCommand`、`VqCommand`

## 可标定参数

以下参数使用 `ExportedGlobal` 生成，可由 ECU 标定层直接读写，而不是固化为表达式中的数字：

- `FOC_Native_KpSpeed`、`FOC_Native_KiSpeed`
- `FOC_Native_KpCurrent`、`FOC_Native_KiCurrent`
- `FOC_Native_IqLimit`、`FOC_Native_VoltageLimit`
- `FOC_Native_CurrentIntegratorLimit`
- `FOC_Native_DutyMin`、`FOC_Native_DutyMax`
- `FOC_Native_CurrentPeriod`、`FOC_Native_SpeedPeriod`
- `FOC_Native_Ld`、`FOC_Native_Lq`、`FOC_Native_FluxPM`、`FOC_Native_PolePairs`

标定值改变后无需重新生成代码，但量产集成时应在停机状态或受保护的标定会话中更新参数，并校验参数范围。

## 已验证结果

脚本扫描了控制器和闭环模型的 290 个块：禁用块数量为 0。2 秒闭环仿真结果如下：

- 最终速度：993.090 rpm。
- 最大速度：1084.071 rpm。
- 最大绝对 q 轴电流：2.138 A。
- DutyA 范围：0.08523～0.91476。
- 最终电磁转矩：0.2044 N·m。

ERT 构建、MinGW 编译和自动代码检查均通过；已确认生成代码包含输入/输出结构、`initialize/step` 入口及三角函数控制运算，并且未包含 S-Function 文本。

## 重建方法

在 MATLAB 中切换到本目录并运行：

```matlab
build_pmsm_foc_native_v2
```

脚本会复用本目录中完整的两个模型，应用当前调参值，重新仿真、生成结果图、调用 `slbuild` 并刷新验收报告。如果目录中只存在一个模型文件，脚本会停止，避免生成不完整的版本包。
