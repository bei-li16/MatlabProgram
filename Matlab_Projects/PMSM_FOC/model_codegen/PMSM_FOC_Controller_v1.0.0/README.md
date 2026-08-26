# PMSM FOC Controller v1.0.0

## 版本说明

- 软件包版本：`v1.0.0`
- 模型内部名称：`PMSM_FOC_Controller_Codegen_v2`
- 开发环境：MATLAB/Simulink R2024a
- 代码生成目标：Embedded Coder ERT（`ert.tlc`）
- 生成语言：C
- 数值类型：控制接口与主要运算使用 `single`/`real32_T`

`v1.0.0` 是第一个完成 ERT 代码生成、主机编译和 SIL 对比验证的可交付版本。文件名中的 `v2` 是建模过程中的内部迭代编号，不代表软件包版本。

## 模型用途和边界

本版本包同时归档基础闭环仿真模型和可部署控制器模型，避免后续新模型与本版本文件混放。控制器用于生成可集成到 MCU/ECU 工程中的 C 代码，主要功能包括：

- Clarke 变换和 Park 变换；
- 速度 PI 环；
- d/q 轴电流 PI 环；
- d/q 轴耦合补偿和反电动势前馈；
- 电压矢量限幅和积分抗饱和；
- 逆 Park 变换；
- 居中式 SVPWM 三相占空比计算。

`PMSM_FOC_Basic_v2.slx` 包含用于闭环验证的平均值逆变器和离散 PMSM 被控对象；`PMSM_FOC_Controller_Codegen_v2.slx` 只包含需要部署的软件控制器。电机、逆变器、机械负载和仿真信号源不应部署到目标控制器。

## 文件结构

```text
PMSM_FOC_Controller_v1.0.0/
├─ README.md
├─ PMSM_FOC_Basic_v2.slx
├─ PMSM_FOC_Basic_v2_results.png
├─ build_basic_pmsm_foc.m
├─ foc_controller_sfun.m
├─ average_inverter_sfun.m
├─ pmsm_plant_sfun.m
├─ PMSM_FOC_Controller_Codegen_v2.slx
├─ build_pmsm_foc_controller_codegen.m
├─ foc_controller_codegen_core.m
├─ verify_pmsm_foc_controller_sil.m
├─ PMSM_FOC_Controller_Codegen_v2_ert_rtw/
│  ├─ PMSM_FOC_Controller_Codegen_v2.c
│  ├─ PMSM_FOC_Controller_Codegen_v2.h
│  ├─ PMSM_FOC_Controller_Codegen_v2_private.h
│  ├─ PMSM_FOC_Controller_Codegen_v2_types.h
│  ├─ rtwtypes.h
│  └─ ert_main.c
└─ intermediate/                         # 可删除、不会提交到 Git
   └─ PMSM_FOC_Controller_Codegen_v2_sbs.mexw64
```

其中 `ert_main.c` 是主机独立运行示例。集成到实际 MCU 时，通常使用硬件工程自己的启动代码和定时中断，不直接使用该文件。

`intermediate` 中的 SIL MEX 文件当前可能被 MATLAB 会话占用。关闭模型或执行 `clear mex` 后可以删除；它受 `.gitignore` 保护，不属于版本交付物。

## 任务周期

| 任务 | 周期 | 实现方式 |
|---|---:|---|
| FOC 电流环及 SVPWM | 100 us / 10 kHz | 每次调用 `step()` 都执行 |
| 速度环 | 1 ms / 1 kHz | 在 100 us 基准任务内每 10 次执行一次 |

模型固定步长为 `0.0001 s`。生成代码中的 `speedDivider` 从 0 计数到 9，因此速度环每 10 次基准调用执行一次。

## 输入接口

所有输入均为 `real32_T`：

| 输入字段 | 单位 | 说明 |
|---|---|---|
| `SpeedReferenceRpm` | rpm | 目标机械转速 |
| `SpeedRpm` | rpm | 反馈机械转速 |
| `PhaseCurrentA` | A | A 相电流 |
| `PhaseCurrentB` | A | B 相电流，C 相由三相平衡关系隐含 |
| `ElectricalAngleRad` | rad | 转子电角度 |
| `DcBusVoltage` | V | 直流母线电压 |

输入结构体实例：

```c
PMSM_FOC_Controller_Codegen_v_U
```

## 输出接口

所有输出均为 `real32_T`：

| 输出字段 | 说明 |
|---|---|
| `DutyA` | A 相 PWM 占空比，限制在 `FOC_DutyMin`～`FOC_DutyMax` |
| `DutyB` | B 相 PWM 占空比 |
| `DutyC` | C 相 PWM 占空比 |
| `IqReference` | 速度环产生的 q 轴电流给定 |
| `IdMeasured` | d 轴反馈电流 |
| `IqMeasured` | q 轴反馈电流 |
| `VdCommand` | d 轴电压指令 |
| `VqCommand` | q 轴电压指令 |

输出结构体实例：

```c
PMSM_FOC_Controller_Codegen_v_Y
```

## 调用接口

系统上电时调用一次初始化函数：

```c
PMSM_FOC_Controller_Codegen_v2_initialize();
```

在 100 us 定时任务中写入输入、执行一步控制器并读取输出：

```c
PMSM_FOC_Controller_Codegen_v_U.SpeedReferenceRpm = speed_ref_rpm;
PMSM_FOC_Controller_Codegen_v_U.SpeedRpm = speed_rpm;
PMSM_FOC_Controller_Codegen_v_U.PhaseCurrentA = phase_current_a;
PMSM_FOC_Controller_Codegen_v_U.PhaseCurrentB = phase_current_b;
PMSM_FOC_Controller_Codegen_v_U.ElectricalAngleRad = electrical_angle_rad;
PMSM_FOC_Controller_Codegen_v_U.DcBusVoltage = dc_bus_voltage;

PMSM_FOC_Controller_Codegen_v2_step();

pwm_duty_a = PMSM_FOC_Controller_Codegen_v_Y.DutyA;
pwm_duty_b = PMSM_FOC_Controller_Codegen_v_Y.DutyB;
pwm_duty_c = PMSM_FOC_Controller_Codegen_v_Y.DutyC;
```

`terminate()` 在裸机控制器中通常不需要周期调用：

```c
PMSM_FOC_Controller_Codegen_v2_terminate();
```

## 可标定参数

以下参数以 `ExportedGlobal` 全局变量生成，可由标定层或应用软件修改：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `FOC_KpSpeed` | 0.2 | 速度环比例系数 |
| `FOC_KiSpeed` | 3.0 | 速度环积分系数 |
| `FOC_KpCurrent` | 1.0 | 电流环比例系数 |
| `FOC_KiCurrent` | 500.0 | 电流环积分系数 |
| `FOC_IqLimit` | 8.0 A | q 轴电流限幅 |
| `FOC_AntiWindupGain` | 0.2 | 抗饱和回算增益 |
| `FOC_DutyMin` | 0.02 | 最小占空比 |
| `FOC_DutyMax` | 0.98 | 最大占空比 |
| `FOC_VoltageUtilization` | 0.95 | 直流母线电压利用率 |
| `FOC_Ld` | 0.001 H | d 轴电感 |
| `FOC_Lq` | 0.001 H | q 轴电感 |
| `FOC_FluxPM` | 0.05 Wb | 永磁体磁链 |
| `FOC_PolePairs` | 4 | 极对数 |
| `FOC_CurrentPeriod` | 0.0001 s | 电流环算法周期 |
| `FOC_SpeedPeriod` | 0.001 s | 速度环算法周期 |

修改 `FOC_CurrentPeriod` 或 `FOC_SpeedPeriod` 不能自动修改硬件调度周期。调整任务周期时，必须同步修改 Simulink 固定步长、分频逻辑、参数值和 MCU 定时器配置。

## 重新生成代码

在 MATLAB 中将当前目录切换到本版本文件夹，然后运行：

```matlab
build_pmsm_foc_controller_codegen
```

脚本会重新创建控制器模型，并使用 `ert.tlc` 生成和编译代码。重新构建会再次产生 `.slxc`、`slprj`、对象文件和编译元数据，这些均属于可再生成的中间产物。

重新创建和运行基础闭环仿真模型：

```matlab
build_basic_pmsm_foc
```

## 验证结果

- ERT 代码生成成功；
- MinGW64 GCC 主机编译成功；
- 生成代码不包含动态加载 S-Function；
- Normal 模式与 SIL 生成代码使用同一组测试输入进行比较；
- 8 路输出的最大绝对差为 `1.90734863e-6`；
- 差异处于单精度浮点舍入范围内，采用 `5e-6` 绝对误差阈值判定通过。

运行验证：

```matlab
verify_pmsm_foc_controller_sil
```

## 集成注意事项

1. MCU 的 ADC 采样、电角度更新与 PWM 更新应和 100 us 控制任务同步。
2. 调用 `step()` 前必须更新全部输入，调用完成后再写入 PWM 比较寄存器。
3. 对标定变量增加范围检查，尤其是 PI 增益、电流限幅和占空比上下限。
4. 首次上电、故障恢复或控制器重新使能时调用 `initialize()`，避免保留旧积分状态。
5. 当前版本为基础闭环算法，尚未包含电流零偏校准、预充电、转子对齐、弱磁、MTPA、故障状态机和硬件保护逻辑。
