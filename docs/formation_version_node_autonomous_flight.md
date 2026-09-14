# 版本节点记录：Formation Autonomous Flight

## 1. 记录目的

- 将本版代码固化为一个**版本节点**，作为链翼编队飞行研究过程中的里程碑存档，便于后续回溯、对比与恢复。
- 代码基线：提交 `b65802c8cc`（autonomous flight），并包含工作区未提交的 `FORM_YAW_K` 参数范围调整（`@max` 由 3.0 放宽至 5.0，`@decimal` 由 2 提升至 3）。
- 记录本阶段已验证与未验证的功能边界，为下一阶段（控制律调整、试飞验证等）提供参照。

## 2. 具体内容

### 2.1 主机侧数据流总览

本节点记录链翼无人机**主机（master）侧**的指令链路：`formation_rates_sender` 主机订阅的 uORB 指令（调用），以及经 UAVCAN 广播给从机的 `ControlInput` 指令（发出）。

```text
主机飞控内部：
  vehicle_status / vehicle_attitude_setpoint / vehicle_rates_setpoint
      │  订阅（调用）
      ▼
  FormationRatesSender::periodic_update()          ← 定时器 300 Hz
      │  封装
      ▼
  dronecan::formation::ControlInput（广播）        ← 发出（从机侧接收）
```

### 2.2 主机调用的指令（输入端）

| uORB 主题 | 用到的字段 | 用途 |
| ---- | ---- | ---- |
| `vehicle_status` | `vehicle_type`、`in_transition_mode` | 发送门控：仅固定翼或过渡模式发送 |
| `vehicle_status` | `nav_state` | 偏航指令数据源选择（手动姿态模式 vs 自主模式） |
| `vehicle_attitude_setpoint` | `q_d`（四元数） | 反解欧拉角，得到主机滚转角设定 `roll_target`、俯仰角设定 `pitch` |
| `vehicle_attitude_setpoint` | `yaw_sp_move_rate` | 自主模式的偏航指令源 |
| `vehicle_attitude_setpoint` | `thrust_body[0]` | 主机推力指令，clamp 到 `[0, 1]` |
| `vehicle_attitude_setpoint` | `timestamp` | 新鲜度门控：超过 500 ms 不发送 |
| `vehicle_rates_setpoint` | `yaw` | 手动姿态模式的偏航指令源（含操纵杆偏航输入） |
| `vehicle_rates_setpoint` | `roll` | 主机**期望**滚转角速率（设定点），作为铰链修正参考量 |

### 2.3 发出的指令（输出端）：ControlInput

消息定义：`src/drivers/uavcan/dsdl_custom/dronecan/formation/20040.ControlInput.uavcan`（Message ID 20040），以 300 Hz 广播。

| 字段 | 类型 | 本版代码实际装载的数据源 | 物理含义 |
| ---- | ---- | ---- | ---- |
| `thrust` | `float16` | `att_sp.thrust_body[0]`，constrain 到 `[0,1]` | 主机推力指令 |
| `pitch` | `float16` | `euler.theta()` | 主机俯仰角设定，rad |
| `yaw` | `float16` | 见 2.4 节数据源切换 | 主机偏航角速率指令，rad/s |
| `roll_target` | `float16` | `euler.phi()` | 主机滚转角设定，rad |
| `master_roll_signed_err` | `float16` | `vehicle_rates_setpoint.roll` | 主机**期望**滚转角速率（设定点），rad/s |
| `flags` | `uint8` | `FLAG_VALID` (=1) | 消息有效标志 |

### 2.4 关键逻辑：偏航指令数据源切换

```text
nav_state ∈ {MANUAL, STAB, ACRO, ALTCTL, POSCTL}（主机处于人工姿态操纵）
  → 使用 rates.yaw（操纵杆偏航已折算在内）
否则（AUTO / MISSION / OFFBOARD 等自主/半自主模式）
  → 使用 att_sp.yaw_sp_move_rate（导航解算的偏航速率）
异常回退：优先值非有限时 → rates.yaw → 0.0
```

**问题背景**：手动模式下 `fw_att_control::vehicle_manual_poll()` 发布的 `vehicle_attitude_setpoint.yaw_sp_move_rate` 保持零初始化值（`0.0f` 是有限值，`PX4_ISFINITE()` 检查会**误通过**），若不做显式的 `nav_state` 判断，将使用恒为零的偏航指令而非真正的操纵杆偏航输入。

**物理意义**：主机在人工操纵与自主飞行两种状态下，偏航指令的"权威"来源不同——前者源于驾驶员操纵杆，后者源于导航/制导外环，必须显式区分，否则从机收到的偏航指令在人机切换瞬间会错误归零。

### 2.5 发送门控条件

定期回调中同时满足以下条件才广播：

1. `vehicle_type == VEHICLE_TYPE_FIXED_WING`，或 `in_transition_mode == true`；
2. `vehicle_attitude_setpoint.timestamp` 距今不超过 500 ms；
3. 定时器周期 `1000000 / MAX_RATE_HZ` = 300 Hz。

### 2.6 结论：DSDL 注释与代码的语义漂移

`ControlInput.uavcan` 中的字段注释仍描述旧版"原始操纵输入"语义，属于之前修改代码时未同步更新的**遗留问题**。本节点以**代码实际装载的数据语义**为准：

| 字段 | DSDL 注释（旧语义，遗留） | 本节点认定的实际语义 |
| ---- | ---- | ---- |
| `pitch` | Raw pitch input `[-1,1]` | 主机俯仰角设定，rad |
| `roll_target` | 由 raw roll × `FORM_ROLL_LIM` 预换算 | 主机滚转角设定（`euler.phi()`），rad |
| `master_roll_signed_err` | `master_roll - roll_target` 角度差 | 主机**期望**滚转角速率设定点（`vehicle_rates_setpoint.roll`），rad/s |

**处理结果**：已于本次节点记录时同步修正 `20040.ControlInput.uavcan` 的字段注释，使其与代码实际语义一致（注释改动不影响报文格式与兼容性）。重编译后 `ControlInput.hpp` 中的字段文档随之更新。

**注意区分**：`master_roll_signed_err` 装载的是 `vehicle_rates_setpoint.roll`，即主机姿态控制器的**期望滚转角速率（设定点）**，而非 `vehicle_angular_velocity` 的实测角速度。二者对铰链修正的影响不同：

- **设定点**：无传感器延迟、无噪声，代表主机"想要"的运动；但主机实际可能未跟上（存在姿态跟踪误差），且强机动时设定点可能骤变；
- **实测值**：反映主机真实运动，来自 EKF 估计，含噪声与延迟。

因此从机侧铰链修正参考的是**主机期望运动**（前馈式），而非主机的真实响应。若从机控制器同时具备对主机实际姿态的闭环（如后续接入实测姿态），二者构成的修正带宽将不同，需在从机侧控制律设计中明确该信号的相位属性。

## 3. 从机接收端：根据主机指令与自身姿态解算从机运动

### 3.1 定位与目的

本节记录从机侧 UAVCAN 接收器 `FormationRatesBridge`（`src/drivers/uavcan/sensors/formation_rates.{hpp,cpp}`）。

**目的**：从机接收主机广播的 `ControlInput`，**结合主机传来的指令（姿态设定、偏航速率、铰链修正量）与从机自身当前姿态**，在本机解算出"从机该如何运动"，并以本地姿态设定（`vehicle_attitude_setpoint`）的形式下发执行，实现链翼编队的协同运动。

```text
主机广播 ControlInput
      │  UAVCAN
      ▼
FormationRatesBridge::formation_rates_sub_cb()
      │  读取：vehicle_attitude（保持从机自身航向 self_yaw）
      │        vehicle_status（Offboard 模式门控）
      │        编队参数 FORM_POSITION / FORM_HINGE_K / FORM_ROLL_LIM
      ▼
发布 offboard_control_mode（attitude=true, body_rate=false）
  + vehicle_attitude_setpoint（q_d / yaw_sp_move_rate / thrust_body）
```

**语义印证**：从机侧解算直接按"弧度/角速率"新语义使用消息字段（`roll_target` 按弧度引用、`master_roll_signed_err` 直接乘增益叠加到俯仰角），证实 2.6 节结论——从机侧早已按实际语义适配，DSDL 旧注释确为遗留问题。

### 3.2 从机调用的输入

| 来源 | 字段/参数 | 用途 |
| ---- | ---- | ---- |
| UAVCAN `ControlInput` | `roll_target` | 主机滚转角设定，rad |
| UAVCAN `ControlInput` | `pitch` | 主机俯仰角设定，rad |
| UAVCAN `ControlInput` | `yaw` | 主机偏航角速率指令，rad/s |
| UAVCAN `ControlInput` | `thrust` | 主机推力指令 `[0,1]` |
| UAVCAN `ControlInput` | `master_roll_signed_err` | 主机期望滚转角速率，rad/s（铰链修正量） |
| UAVCAN `ControlInput` | `flags` | `FLAG_VALID` 有效性检查 |
| uORB `vehicle_attitude` | `q`（四元数） | 反解从机自身当前偏航 `self_yaw`，用于保持航向 |
| uORB `vehicle_status` | `nav_state` | 仅 `OFFBOARD` 状态发布姿态设定 |
| uORB `parameter_update` | — | 参数热更新，限频 1 s |
| 参数 `FORM_FOLLOWER_EN` | 0/1 | 从机模式总开关 |
| 参数 `FORM_POSITION` | 0/1/2 | 编队位置：0=CENTER，1=LEFT，2=RIGHT |
| 参数 `FORM_ROLL_LIM` | deg（默认 45） | 滚转/俯仰角限幅 |
| 参数 `FORM_HINGE_K` | 默认 1.0 | 铰链修正增益 |

### 3.3 解算核心：从机"该如何运动"

**位置符号**：`side_sign` 由编队位置决定——LEFT 为 $+1$，RIGHT 为 $-1$，CENTER 为 $0$（中心位置不做铰链修正）。

**① 滚转设定**（跟随主机，限幅）：

$$
\phi_{sp} = \mathrm{clip}\left(\phi_{tgt},\; -\delta_{lim},\; +\delta_{lim}\right), \qquad \delta_{lim} = \mathrm{FORM\_ROLL\_LIM}\ (\mathrm{rad})
$$

**② 俯仰设定**（主机俯仰 + 铰链修正）：

$$
\theta_{sp} = \mathrm{clip}\left(\theta_{tgt} + \mathrm{side\_sign}\cdot K_h \cdot p_{sp}^{master},\; -\delta_{lim},\; +\delta_{lim}\right)
$$

其中 $p_{sp}^{master}$ 即 `master_roll_signed_err`（主机期望滚转角速率）。

**③ 偏航**：四元数设定中的偏航角固定为**从机自身当前航向** $\psi_{sp}=\psi_{self}$（航向保持）；偏航运动通过 `yaw_sp_move_rate = msg.yaw` 单独注入。

**④ 推力**：`thrust_body[0]` 直接透传主机推力（偏航增推逻辑已移至 ControlAllocator 处理）。

**物理意义解读**

- **铰链修正项的量纲**：$\theta$ 修正量 $= K_h \cdot p$，rad $= [K_h]\cdot$ rad/s，故 **$K_h$ 的量纲是时间**（s），默认值 1.0 实际为 1.0 s。物理上它刻画了"主机滚转速率→从机俯仰补偿"的**运动学比例**：主机以角速率 $p$ 滚转时，翼尖铰链约束迫使两侧从机产生几何上必需的点头/抬头运动，$K_h$ 即该等效几何-气动耦合系数的时间尺度。
- **左右对称反向**：`side_sign` 保证左右从机的俯仰修正方向相反，与主机滚转的铰链运动学一致——这正对应链翼构型"铰链释放相对滚转、约束相对偏航"的约束特征。
- **偏航保持 + 速率跟随**：从机角位置指令保持自身航向，仅接收主机的偏航速率指令，体现从机航向被机械铰链约束、只能整体跟随主机转向的构型本质。
- **"判断"的含义**：从机并非简单复制主机位姿，而是以**主机姿态设定为基准**、以**主机滚转速率驱动的铰链修正**为补偿、以**自身航向**为约束，三者合成后才得到自身的姿态设定。

### 3.4 模式门控与发布

1. 收到有效消息（`FLAG_VALID`）后，先发布 `offboard_control_mode`（`attitude=true`，`body_rate=false`），用于维持/请求 Offboard 模式；
2. 检查 `vehicle_status.nav_state == NAVIGATION_STATE_OFFBOARD`，否则**丢弃本包指令**（存在切换期间的指令丢失窗口）；
3. 在 Offboard 状态下发布 `vehicle_attitude_setpoint`：

| 字段 | 值 |
| ---- | ---- |
| `q_d` | `Eulerf(roll_sp, pitch_sp, self_yaw)` |
| `yaw_sp_move_rate` | `msg.yaw`（偏航速率指令） |
| `thrust_body` | `[base_thrust, 0, 0]` |

### 3.5 存留事项：从机侧代码的注释/命名遗留

1. ✅ **`FORM_ROLL_LIM` 命名与用途不符**：参数名含 "rate"，实际用作**角度**限幅。已修正 `uavcan_params.c` 中该参数的描述注释（注明实际限幅对象及保留旧名的原因）；**参数名本身未改**，以免破坏 QGC 参数配置兼容。
2. ✅ **hpp 类注释过时**：已修正 `FormationRatesBridge` 的 `@brief`，改为发布 `vehicle_attitude_setpoint`（attitude 模式）+ `offboard_control_mode`。
3. ✅ **cpp 内注释不完整**：`msg.yaw` 处注释已改为说明双数据源（手动 `rates_setpoint.yaw` / 自主 `yaw_sp_move_rate`）；滚转设定处"遥控指令"表述一并修正为"主机滚转角设定"。
4. **跨机型一致性（待验证）**：`attitude=true, body_rate=false` 模式下 `yaw_sp_move_rate` 的前馈作用，注释提示 mc_att_control 可使用、fw_att_control 需配合补丁，本版代码中固定翼从机的偏航通道实际生效路径待验证。

## 4. 控制分配层：偏航执行与编队混控（ControlAllocator）

### 4.1 定位

`ControlAllocator`（`src/modules/control_allocator/ControlAllocator.{hpp,cpp}`）订阅 `vehicle_torque_setpoint` / `vehicle_thrust_setpoint`，经执行器有效性矩阵解算出 `actuator_motors` / `actuator_servos`。本版在其中加入了**从机编队定制**：滚转→俯仰混控与偏航增推。由此，第 3 节中提到的"偏航增推已移至 ControlAllocator"在此闭环。

### 4.2 编队相关参数与内部状态

| 参数 | 默认 | 用途 |
| ---- | ---- | ---- |
| `FORM_FOLLOWER_EN` | 0 | 1 表示本机为从机（`_is_follower`，判定容差 1e-3） |
| `FORM_POSITION` | 0 | `_side_sign`：LEFT=+1，RIGHT=−1，CENTER=0 |
| `FORM_YAW_K` | 0.3 | 偏航增推增益 `_yaw_throttle_gain` |
| `CA_RLL2PIT_K` | 0.0 | 滚转→俯仰混控增益 `_roll_to_pitch_mix`（自定义参数，见第 5 节） |

### 4.3 控制设定向量的合成

分配器控制目标 $c = [\tau_x,\ \tau_y,\ \tau_z,\ T_x,\ T_y,\ T_z]$，从机启用时：

$$c(1) = \tau_{y,sp} + \underbrace{\tau_{x,sp}\cdot K_{r2p}\cdot \mathrm{side\_sign}}_{\text{滚转→俯仰混控（仅从机）}}$$

$$c(3) = T_{x,sp} + \underbrace{\max\left(\mathrm{side\_sign}\cdot\tau_{z,sp},\ 0\right)\cdot K_{yaw}}_{\text{偏航增推（仅从机，单侧截断）}}$$

其余分量直接透传设定点。

**偏航增推的符号分析**（body 系 z 轴向下，$\tau_z>0$ 为右转）：左转（$\tau_z<0$）时 `side_sign=−1` 的右机满足 `side_sign·τz>0` → **右机增推**；右转时左机增推。即——**增推的始终是转弯外侧的从机**。

**物理意义**：

- **偏航增推**：编队整体转弯时，外侧从机走大半径圆弧，速度需求高于内侧，必须额外加推力维持队形与空速；增推量与偏航力矩幅值成正比（$|\tau_z|\cdot K_{yaw}$）。单侧截断（只增不减）避免内侧从机动力过剩；这也使控制律在 $\tau_z$ 过零处**不可微、非光滑**，与后续抗积分饱和设计存在交互问题。
- **滚转→俯仰混控**：机理由两部分组成——**① 几何高度协调（主导）**：从机绕主机相对滚转（上反角 $\delta$ 变化）时，质心高度 $z_f = \mp l\sin\delta$ 随之变化，需要法向力 $m a_z$ 支撑这段圆弧运动；副翼只能产生力矩、不能直接提供质心法向加速度，故必须由升降舵改变迎角产生升力来"喂"高差。② 结构耦合补偿（次要）：翼尖铰链约束传递的寄生俯仰力矩。两效应对左右从机符号相反（镜像几何，`side_sign`）。由于几何项含 $1/\bar{q}$ 依赖，低速工况所需增益更大，全程变速编队时需考虑增益调度。

### 4.4 偏航控制的完整闭环链路

```text
主机：驾驶员/导航 → rates.yaw 或 yaw_sp_move_rate → ControlInput.yaw（UAVCAN）
      ▼
从机：msg.yaw → vehicle_attitude_setpoint.yaw_sp_move_rate
      ▼
从机姿态控制器：跟踪生成 τ_z,sp（vehicle_torque_setpoint）
      ▼
ControlAllocator：
  ① 常规路径：τ_z 进入分配矩阵 → 电机差速/舵面（直接偏航执行）
  ② 从机增强：|τ_z|·FORM_YAW_K → 主推力增量（转弯外侧速度补偿）
  ③ 高度协调+耦合补偿：τ_x·CA_RLL2PIT_K·side_sign → 俯仰通道（质心高差升力前馈为主、铰链耦合为辅）
```

### 4.5 存留事项

1. **单侧截断的非线性**：增推条件 `side_sign·τz>0` 在 τz 过零处产生死区型非线性，从机推力通道等效增益随偏航方向跳变。调参（`FORM_YAW_K` 上限已放宽至 5.0）与抗积分饱和设计时需显式考虑该不连续点。
2. **多矩阵实例的差异**：分配器第二实例（`c[1]`，多矩阵/VTOL 场景）仅继承了滚→俯混控，**未**实现偏航增推；本版三机均为单一有效性矩阵（固定翼），该差异无影响，但需记录以防后续混构型误用。
3. `FORM_YAW_K` 量纲：归一化推力增量 per 单位偏航力矩（约 $1/\mathrm{Nm}$），默认 0.3 对应较小的增推强度。

## 5. 自定义参数汇总

以下 `FORM_*` 参数均定义于 `src/drivers/uavcan/uavcan_params.c`（`Formation Control` 参数组），供 QGC 地面站配置。

| 参数 | 默认值 | 量纲 | 使用位置 | 功能说明 |
| ---- | ---- | ---- | ---- | ---- |
| `FORM_FOLLOWER_EN` | 0 | — | UAVCAN 桥（从机）、ControlAllocator | 从机模式总开关（1 = 从机）；**重启生效**（`@reboot_required true`） |
| `FORM_POSITION` | 0 | — | UAVCAN 桥（从机）、ControlAllocator | 编队位置：0 = CENTER，1 = LEFT，2 = RIGHT；决定两处 `side_sign`（+1/−1/0） |
| `FORM_ROLL_LIM` | 45 | deg | UAVCAN 桥（从机） | 滚转/俯仰**角度**设定限幅（名称含 rate 但非速率限幅，见 3.5 节） |
| `FORM_HINGE_K` | 1.0 | s | UAVCAN 桥（从机） | 铰链修正增益：$\theta$ 修正 $= \mathrm{side\_sign}\cdot K_h \cdot p_{sp}^{master}$ |
| `FORM_YAW_K` | 0.3 | ≈ 1/Nm | ControlAllocator | 偏航增推增益：外侧从机增推 $\lvert\tau_z\rvert\cdot K$（上限已放宽至 5.0，见第 4 节） |
| `CA_RLL2PIT_K` | 0.0 | — | ControlAllocator | 滚转→俯仰混控增益；**自定义参数**（定义于 `src/modules/control_allocator/module.yaml`，范围 ±2.0，2026-04 随编队功能加入），仅从机模式生效：质心高差升力前馈为主、铰链耦合补偿为辅（见 4.3 节） |

## 6. 评审讨论结论：设计原则澄清与审阅意见修订

本节为对本节点的一次外部评审讨论所形成的结论，用于**固化设计哲学**并**修订此前不准确的判断**，不改变代码本身。

### 6.1 顶层设计原则（本次确立）

> **左右从机的作用是辅助主机修正其自身姿态，而非跟随主机的实际运动。因此，铰链修正通道的参考量必须取主机的"期望运动"（设定点），而非"实测运动"。**

§2.6 此前仅把"设定点 vs 实测"作为**信号属性**（噪声、延迟）来讨论；本次评审将其上升为**设计原则**，理由是一个稳定性层面的符号论证：

设主机受外界扰动 $\Delta L_{ext}$ 产生非指令滚转，此时：

- 若参考量取**实测角速率** $p_{meas}$：扰动使 $p_{meas}\neq 0$，从机按实测产生协助 → 协助方向与扰动引起的滚转**同向** → 构成**正反馈**（"助纣为虐"），恶化稳定性；
- 若参考量取**期望角速率** $p_{sp}$：受扰时主机外环发出的是**纠偏指令**，$p_{sp}$ 本身即反扰动的，从机协助的是"主机的纠偏意图" → **助其纠偏**。

因此本通道的物理本质是**前馈协助（feedforward aiding）**，而非**反馈跟随（feedback following）**。这从原理上解释了 `master_roll_signed_err` 装载 `vehicle_rates_setpoint.roll` 是**正确的架构选择**，而非权宜之计。

### 6.2 审阅意见修订记录

| 编号 | 初判意见 | 修订结论 | 依据 |
| ---- | ---- | ---- | ---- |
| 一 | $K_h$ 常增益假设欠严谨，建议闭式化 | **接受，但归入后续优化**。符号由 `side_sign` 完全确定；左右安装严格镜像对称，**无需**引入安装角差参数 | 构型对称性 |
| 二 | 建议对前馈量加低通 / 改用实测 $p$ | **撤回**。参考量必须为**期望值**；改用实测将构成正反馈（见 6.1） | 6.1 稳定性论证 |
| 三 | $\max(\cdot,0)$ 单侧截断的非光滑性 | 后续处理（方案见 7.2-D） | — |
| 四 | 从机侧消息新鲜度 / 丢包处理未见记录 | 后续处理（试飞安全项） | — |
| 五 | 固定翼 `yaw_sp_move_rate` 生效性 + `q_d` 偏航与速率双指令源仲裁 | **已验证通过**，后续优化 | 试飞 / SIL |
| 六 | roll 通道做同款有效性排查；字段名更名 | roll 通道**不需要**同款排查；`master_roll_signed_err` 后续更名 | — |

### 6.3 语义澄清：§3.3 措辞

§3.3 ① 中"滚转设定（跟随主机）"的表述易被**误读**为"从机跟随主机滚转运动"，与 6.1 所述设计原则冲突。精确表述应为：

> 从机以**主机的滚转姿态设定** $\phi_{tgt}$ 作为自身姿态基准（水平基准）；而"协助主机修正滚转姿态"的作用由**俯仰通道**的修正项 $\theta$（经翼尖铰链差动升力回馈至主机滚转轴）承担。

即：**滚转通道给基准，俯仰通道给协助**，二者是不同通道，不可混同。建议后续将 §3.3 ① 措辞修订为"滚转设定（以主机设定为姿态基准）"。

## 7. 架构级问题：装配体滚转的分布式执行器协调

本节记录一个**不属于"参数调优"、而属于"架构与稳定性"**的问题，并固化经讨论修订后的物理图像与解决方向。原 §7 中关于"协助饱和 windup"的内容按要求**搁置**（见 7.5）。

### 7.1 修订后的物理图像：主/从执行器的主次关系（关键更正）

**原判断（已否定）**：曾认为"主机副翼构成一个强而快的滚转回路，从机协助通路是叠加其上的附加通道，可能拖垮主机的相位裕度"。

**修订判断（本节点确立）**：主机翼尖两侧连接的是**两架与主机等大等重**的从机。主机副翼**没有足够效能**去拉动两个与自身同量级的质量——**副翼在装配体滚转中的作动效果是杯水车薪**。真正承担装配体滚转主力的，是**两侧从机由俯仰产生的差动升力**。

装配体滚转方程应写成**两个并联执行器**之和：

$$
I_{xx}\,\dot p \;=\; \underbrace{L_{\delta_a}\,\delta_a}_{\text{主机副翼：快、弱}} \;+\; \underbrace{2\,l\,\Delta L_s(\theta_{sp})}_{\text{两侧从机差动升力：慢、强}} \;-\; D_p\,p \;+\; \Delta L_{ext}
$$

其中 $l$ 为翼尖铰链到主机滚转轴的横向力臂，$\Delta L_s$ 为从机由俯仰产生的升力增量。二者效能之比约为**面积比**（从机升力变化作用在整个与主机等大的机翼面上、且力臂达半翼展量级；副翼只在翼尖一小段产生局部升力变化），故

$$
K_s \gg K_a \qquad (\text{约相差一到两个数量级})
$$

**结论**：装配体滚转的**带宽上限由从机协助通路的动力学 $G_s(s)$ 决定**，而非由主机副翼回路决定。主机副翼回路是"强回路缺席时的高频补丁"，不是"被拖累的主角"。

### 7.2 $K_h$ 与 $CA\_RLL2PIT\_K$ 的职能拆分（关键澄清）

原 §7.1 将 $K_h$ 与 $CA\_RLL2PIT\_K$ 当作"同轴双执行器"一并讨论，**属错误归并**。二者虽**都作用在俯仰通道**，但**目标不同、回路不同**，必须分别建模：

| 参数 | 服务对象 | 作用路径 | 回路性质 |
| ---- | ---- | ---- | ---- |
| `FORM_HINGE_K` ($K_h$) | **主机增稳** | 主机 $p_{sp}$ → 从机 $\theta_{sp}$ → 从机差动升力 → 经翼尖铰链以滚转力矩**回馈主机滚转轴** | 跨机**外回路**（协助主机） |
| `CA_RLL2PIT_K` ($K_{r2p}$) | **从机自身增稳** | 从机 $\tau_x$ → 从机 $\tau_y$ → 从机自身高度/姿态协调 | 从机**内回路**（自稳） |

因此：

- **§7.3 的联合回路整定（方案 2）只对 $K_h$ 那条外回路成立**，$K_{r2p}$ 不参与该联合回路，应按从机自身俯仰/高度回路的常规整定处理；
- $K_h$ 的作用机理是"**经铰链约束、由从机差动升力向主机滚转轴回馈力矩**"，这正是 7.1 修订图像中的**主执行器路径**。

### 7.3 解决方案修订

**原方案 1（带宽分层）的机理作废**：

原表述"让帮手慢一点，主机快回路不被拖垮"，其**预设前提是主机存在一个强而快的滚转回路**。按 7.1 修订图像，该前提不成立——主机副翼本就杯水车薪，**不存在可被拖垮的裕度**。故该方案的机理**予以撤回**。

> 说明：原方案 1 的"限制从机指令带宽"这一动作本身，即使换一个理由可能仍成立，那也是**另一条命题，须单独验证**，不能用以救活原方案。原方案 1 不再作为倾向性建议。

**修订后的解决方向**（按执行顺序）：

1. **先建模，后调参（前提项）**：量化从机协助通路的等效执行器传递 $G_s(s)$，确定装配体滚转的**带宽天花板**。这是后续一切设计的前提。
2. **主/从执行器频带划分（替代原方案 1）**：副翼快弱 → 承担**高频阻尼与快速扰动抑制**；从机差动升力慢强 → 承担**低频主体滚转**。二者若在同一频段都全力投入，会产生"争夺/相位打架"；应设计**显式指令分频器**，而非简单给从机加低通。
3. **从机前馈整形（$K_h$ 的升级方向）**：既然从机承担主体滚转，应将常数 $K_h$ 升级为**含超前补偿的整形器**——对冲 $G_s(s)$ 的已知滞后、并按动压 $\bar q$ 做增益调度。**开环前馈整形提高有效带宽却不抬高回路增益、不动相位裕度**，这是它相对"单纯加大 $K_h$"的根本优势。
4. **联合回路整定上限（原方案 2，仅对 $K_h$）**：建立"从机俯仰指令 → 主机滚转力矩"的等效传递 $G_{f\to m}(s)$，与主机滚转对象叠加后做根轨迹/裕度扫描，以**相位裕度 ≥ 30°（或幅值裕度 ≥ 6 dB）**确定 $K_h$ 的安全上限。
5. **守住开环前馈架构（原方案 4，保留）**：$K_h$ 通路**不得**引入对主机姿态误差的闭环，否则回路增益乘性增长。当前"以 $p_{sp}$ 前馈"恰好满足，应予以保持（与 6.1 一致）。
6. **统一协助权限因子（原方案 3，降级保留）**：引入 $\eta\in[0,1]$ 对 $K_h$ 做整体缩放，便于在稳定边界内快速调节协助强度。

### 7.4 下一步：$G_s(s)$ 的建模方法（待开展）

以铰链约束 + 从机气动写出"**从机俯仰指令 → 装配体滚转力矩**"的等效执行器传递，含力臂 $l$、从机机翼面积、动压 $\bar q$ 依赖。该模型同时为 7.3 第 4 项提供定量支撑，并闭合 §3.3 的机理链条。

### 7.5 搁置事项（原风险二）

原 §7.2"协助饱和 → 主机滚转回路第二类积分饱和（Windup）"及其对应方案**暂时搁置**。搁置原因：该问题在物理上受约束条件（从机机械限位、跨机不可观测）限制，现有方案（饱和观测器 / Back-Calculation / 指令投影 / 平滑化）均未见突破性价值，留待后续结合 $G_s(s)$ 建模结果再评估。

## 8. 设计资产存档：$G_s(s)$ 等效执行器传递与超前整形器（待开展）

> **状态：方法论存档，暂不实施。** 当前阶段的首要任务是**在新 PX4 架构下把基础链路搭通**——上述建模与整形器属于链路打通、可稳定飞之后的**性能优化**环节。本节仅把讨论结论固化，避免思路丢失。

### 8.1 建模方法论：三条腿并行、互相验证

| 路径 | 内容 | 作用 |
| ---- | ---- | ---- |
| A. 解析分层（白盒） | 按物理环节逐段写传递函数后串联 | 每项含物理意义，便于增益调度与故障归因 |
| B. 频域辨识（灰盒/黑盒） | SIL 注入 $p_{sp}$ chirp（0.1–10 rad/s），由响应辨识 | 补齐解析式遗漏的环节 |
| C. 降阶 | 主极点近似 / 平衡截断，压至 2–3 阶 | 得到控制器可用的低阶模型 |
| D. 交叉验证 | 解析式与辨识模型在穿越频率 ±10 dB 带宽内幅频误差 < 3 dB、相频 < 15° | 确认模型可用 |

**辨识中的关键技巧——力矩不可直测时的反算**：

$$
L_m(t) = I_{xx}\,\dot p(t) + D_p\,p(t) - K_a\,\delta_a(t)
$$

其中 $\delta_a$、$p$ 可测，$D_p$ 由**滚转阶跃后的自由衰减试验**辨识。得到 $L_m(t)$ 后再用 ARX / 子空间法辨识 $G_s$。

### 8.2 信号链分解

$$
p_{sp} \xrightarrow{\ K_h\ } \theta_{sp} \xrightarrow{\ \text{从机俯仰姿态环}\ } \theta \xrightarrow{\ \alpha=\theta-\gamma\ } \Delta\alpha \xrightarrow{\ \text{准静态气动}\ } \Delta L_s \xrightarrow{\ 2l\ } L_m
$$

### 8.3 逐段传递函数

**(a) 准静态气动-几何增益**（纯增益，无动态）：

$$
K_g(\bar q)=2\,l\,\bar q\,S_s\,C_{L\alpha}
$$

$l$ 为主机滚转轴到从机气动中心的横向距离，$S_s$ 为单侧从机机翼面积。**$K_g\propto\bar q$，这是增益调度的根源。**

> $\Delta\alpha\approx\Delta\theta$ 的成立条件：在滚转回路关注频段（$\gtrsim 0.5$ rad/s），① 非定常升力滞后时间尺度 $\omega c/(2V)\ll 1$，可视为准定常；② 航迹角 $\gamma$ 尚来不及跟随。**低频谱段的"轨迹协调"由 §7.2 的 $CA\_RLL2PIT\_K$ 内回路另行承担**，两者自洽。

**(b) 从机俯仰姿态闭环**（主滞后源）：

$$
G_{att}(s)=\frac{\omega_n^2}{s^2+2\zeta_n\omega_n s+\omega_n^2}
$$

舵机一阶滞后可单列或并入（$\frac{1}{\tau_e s+1}$，$\tau_e\sim 20\text{–}50$ ms）。**$\omega_n$ 是装配体滚转带宽天花板的核心参数。**

**(c) 传输与计算延迟**：$G_d(s)=e^{-\tau_d s}$，$\tau_d\approx 1.5\,T_{sample}=1.5/300\approx 5$ ms（UAVCAN 300 Hz 传输本身可忽略，从机调度 + 姿态环执行需实测确认）。

### 8.4 合成

$$
\boxed{\;G_s(s)\;\triangleq\;\frac{L_m(s)}{\theta_{sp}(s)}\;=\;\underbrace{2\,l\,\bar q\,S_s\,C_{L\alpha}}_{K_g(\bar q)}\cdot\underbrace{\frac{\omega_n^2}{s^2+2\zeta_n\omega_n s+\omega_n^2}}_{G_{att}(s)}\cdot\underbrace{e^{-\tau_d s}}_{G_d(s)}\;}
$$

含代码增益 $K_h$ 的协助通路：

$$
L_m(s)=K_h\,G_s(s)\,p_{sp}(s)
$$

**工程降阶形式**（供控制器设计）：

$$
\tilde G_s(s)\approx\frac{K_g(\bar q)}{\tau_n s+1},\qquad \tau_n\approx\frac{2\zeta_n}{\omega_n}
$$

### 8.5 量级对照（示例值，需代入真实参数复核）

取 $\bar q=245$ Pa（$V=20$ m/s）：

| 通路 | 表达式 | 示例值 |
| ---- | ---- | ---- |
| 从机差动升力（主） | $K_g=2l\bar q S_s C_{L\alpha}$ | $\approx 2\times1.75\times245\times0.6\times4.5\approx 2300$ Nm/rad |
| 主机副翼（辅） | $K_a=\bar q S_m b_m C_{l_{\delta_a}}$ | $\approx 22$ Nm/rad |

$$
\frac{K_g}{K_a}\sim\mathcal{O}(10^2)
$$

与"副翼拉不动两个等重机体"的判断量级吻合。**注意该比值与 $\bar q$ 无关**（两条通路均 $\propto\bar q$），故低/高速下主次关系不变，变的只是绝对效能。

### 8.6 结构事实：从机支路位于主机滚转回路**内部**

主机 `fw_att_control` 中 $p_{sp}=K_p(\phi_{sp}-\phi)$ 是**姿态误差的反馈输出**，因此从机支路实际是主机滚转回路的**并联执行支路**。闭环：

$$
I\dot p = \underbrace{K_a\,C_r(s)(p_{sp}-p)}_{\text{副翼支路}} + \underbrace{K_h\,G_s(s)\,p_{sp}}_{\text{从机支路}} - D_p p
$$

因 $K_h G_s(0)\gg K_a C_r(0)$，开环近似：

$$
\boxed{\;L_\phi(s)\approx\frac{K_p\,K_h\,G_s(s)}{s\left(Is+D_p+K_a C_r(s)\right)}\;}
$$

**两点结论**：

1. **穿越频率与相位裕度由 $G_s$ 主导**。主机姿态环 $K_p$ 虽按单机整定，回路实际绕过副翼、由从机动态决定——**原"风险一"的方向没错，错的只是把副翼当成了强回路**。
2. §7.3 第 5 条"守住开环前馈架构"的准确含义：从机支路**不得自行再引入一条对主机姿态误差的独立反馈**（会使回路增益乘性增长）；经 $p_{sp}$ 的这条**唯一通路是允许且必要的**。区别是"加一条新回路" vs "整形已有回路"。

**由此决定整形器的定位**：它不是"外挂滤波器"，而是**主机滚转回路补偿器的一部分**，会改裕度——**能修复相位，也必须重新校核裕度**。

### 8.7 含超前补偿的整形器

**(1) 为什么用超前（lead）而非低通（lag）**：低通**追加**相位滞后，而 $G_s$ 的滞后（$G_{att}$ 二阶 + $\tau_d$）已经过多。要让从机承担主体滚转，必须**提高有效带宽**，方向是超前。

**(2) 为何不能做精确逆**：$G_s^{-1}(s)=\dfrac{s^2+2\zeta_n\omega_n s+\omega_n^2}{\omega_n^2}\cdot\dfrac{e^{+\tau_d s}}{2 l\bar q S_s C_{L\alpha}}$ 存在**分子二次（非真、需微分）+ $e^{+\tau_d s}$（非因果）**，物理不可实现且无穷放大噪声。**可行路线：用真有理函数在有限带宽内逼近逆** —— 这正是超前补偿器的本质。

**(3) 设计公式**：

$$
\boxed{\;C(s)=\underbrace{\frac{K_h^0}{K_g(\bar q_0)}\cdot\frac{\bar q_0}{\bar q}}_{K_c(\bar q)\ \text{动压调度}}\cdot\underbrace{\frac{\tau_z s+1}{\tau_p s+1}}_{\text{超前环节}}\;}
$$

动压调度：因 $K_g\propto\bar q$，取 $K_c\propto 1/\bar q$ 使前向增益 $\bar q$-无关。**建议用从机自身动压**（它才是产生升力的那架）。

超前环节（设目标穿越频率 $\omega_c$）：

$$
\tau_z=\frac{\sqrt{\alpha}}{\omega_c},\qquad \tau_p=\frac{1}{\sqrt{\alpha}\,\omega_c},\qquad \alpha=\frac{\tau_z}{\tau_p}>1
$$

最大相位超前：

$$
\phi_{max}=\arcsin\frac{\alpha-1}{\alpha+1},\qquad \omega_{max}=\frac{1}{\sqrt{\tau_z\tau_p}}=\omega_c
$$

把零极点几何中心锁在 $\omega_c$，补偿量最大。$\alpha$ 取 **3–10**（对应 $30^\circ\text{–}55^\circ$）。

**(4) 关键权衡（待定）**：超前是**开环整形**，对 $G_{att}$ 的 $\omega_n$ 估计误差敏感——$\alpha$ 越大鲁棒性越差、高频噪声放大越强（输入 $p_{sp}$ 较 $p_{meas}$ 干净，但仍含姿态环噪声与杆噪）。**$\alpha$ 的上限应由"$G_{att}$ 不确定性下相位裕度最坏情况"反推**，而非经验拍定，须待 §8.1 辨识结果。

**(5) 与频带划分的关系**：超前把从机有效带宽从 $\omega_n$ 抬到约 $\omega_c$（$\approx 2\text{–}3\,\omega_n$）；**抬不上去的残余高频段，由主机副翼（快、弱）补相位阻尼**。完整方案：

$$
\text{从机 }C(s)\text{ 超前整形（主，扛低频大机动）} + \text{副翼（快弱，补高频阻尼与扰动抑制）}
$$

### 8.8 待定参数清单（落地障碍）

| 参数 | 含义 | 获取方式 |
| ---- | ---- | ---- |
| $\omega_n,\ \zeta_n$ | 从机俯仰姿态环带宽/阻尼 | SIL chirp 辨识，或读 $G_{att}$ 闭环阶跃响应 |
| $D_p$ | 装配体滚转气动阻尼 | roll step 自由衰减试验 |
| $l,\ S_s,\ C_{L\alpha}$ | 几何/气动参数 | 几何测量 + 风洞/估算 |

> 说明：$\omega_n$ 是整条推导中**唯一需要"测"的动力学量**，其余皆为几何与查表量。故后续开展此项时，第一步即在 SIL 中对从机俯仰做阶跃/chirp 标定 $\omega_n$。

### 8.9 实施顺序（待链路打通后）

1. 基础链路搭建（**当前阶段**）；
2. SIL chirp 辨识 $\omega_n,\zeta_n$ 与 $D_p$，标定 $G_s(s)$；
3. 以最坏情况相位裕度反推 $\alpha$ 上限；
4. 实现 $C(s)$（含 $\bar q$ 调度）；
5. 回到 §7.3 第 4 项做联合回路裕度校核。

## 9. 新架构（PX4 main）固定翼偏航链路调研记录（待决策）

> **状态：调研存档，暂不改动。** 本节的目的是为"从机偏航指令如何进入执行机构"这一决策留档。结论与旧系统（本文档 §1–§8 所描述者）差异较大，必须单独记录。

### 9.1 旧/新架构模块对照

旧系统固定翼为两个模块（姿态+速率合一、位置+制导合一）。新架构 `fw_pos_control_l1` 已被删除，拆分为四层：

| 模块 | 职责 | 与旧架构的关系 |
| ---- | ---- | ---- |
| `fw_mode_manager` | 模式编排、设定点分发 | 自 `fw_pos_control_l1` 拆出 |
| `fw_lateral_longitudinal_control` | 横航向 **NPFG** 制导 + 纵向 TECS | **替换 L1** |
| `fw_att_control` | **仅剩**姿态环 | 瘦身 |
| `fw_rate_control` | **独立**速率环 | 自 `fw_att_control` 拆出 |
| `fw_autotune_attitude_control` | 参数自整定 | 不变 |

信号链：

```text
fw_mode_manager  ──► fixed_wing_lateral_setpoint / fixed_wing_longitudinal_setpoint
      │
      ▼
fw_lateral_longitudinal_control ──► vehicle_attitude_setpoint (q_d, thrust_body)
      │
      ▼
fw_att_control                  ──► vehicle_rates_setpoint (roll/pitch/yaw rate)
      │
      ▼
fw_rate_control                 ──► vehicle_torque_setpoint + vehicle_thrust_setpoint
      │
      ▼
ControlAllocator                ──► actuator_motors / actuator_servos
```

注：编队发送端订阅的 `vehicle_attitude_setpoint`、`vehicle_rates_setpoint` 分别为第 2、3 层输出，订阅点正确。

### 9.2 核心结论：偏航**角**不是被控量，偏航**角速率**是

> **表述更正**：此前曾概括为"偏航不是被控轴"，不够精确。准确的区分是——偏航**角** $\psi$（承载于 `q_d`）被姿态环显式剥离；偏航**角速率** $\dot\psi$（承载于 `rates_setpoint.yaw`）由速率环正常 PID 跟踪。**偏航指令并未被忽略，而是被分流到速率通道。**

三条独立证据：

1. **姿态环显式消去偏航误差。** `fw_att_control` 的 `computeAttitudeError()`（`FixedwingAttitudeControl.hpp`）注释即写明 *"The yaw error is removed since fixed-wing aircraft have no direct yaw authority"*。实现上构造一个纯偏航旋转 $q_{yaw\_off}$（其角度等于 $q_{cur}$ 与 $q_{sp}$ 的航向差）后作
   $$
   q_{err}=\big(q_{cur}^{-1}\,q_{yaw\_off}\,q_{sp}\big)_{\text{canonical}},
   \qquad \text{att\_err}=2\,\mathrm{Im}(q_{err})
   $$
   使航向误差分量 $\approx 0$。

   **推论（决定性）**：任何写进 `q_d` 的偏航角**都不会产生控制作用**，会被这一步原样丢弃。

2. **制导层把设定航向取为当前航向。** `fw_lateral_longitudinal_control` 中 `yaw_body = _yaw`（当前航向）；横向控制量走 `lateral_accel_sp → mapLateralAccelerationToRollAngle()`，**输出滚转角指令**。航向是滚转产生的**结果**。`fw_mode_manager` 内同款注释：*"yaw is not controlled, so set setpoint to current yaw"*。

3. **速率环的偏航是"阻尼 + 补偿"。** 偏航仅为速率 PID 三轴之一，另加两类修正：协调转弯前馈 $\dot\psi_{ff}=g\,q_1/V$（$q_1=\sin\phi\cos\theta$，即把滚转天然产生的偏航率前馈掉）与不利偏航补偿 $\tau_{yaw}\mathrel{+}=K_{rll\to yaw}\tau_{roll}$（`FW_RLL_TO_YAW_FF`）。

### 9.3 驾驶员偏航杆的三条通路（不可假设恒定）

| 模式 | 杆量偏航注入点 |
| ---- | ---- |
| STAB / ALTCTL / POSCTL 等 | **加到** `rates_sp.yaw`（叠加于姿态环输出之上） |
| ACRO 且 `FW_ACRO_YAW_EN=0` | **直通** `torque.xyz[2]`，不做速率闭环 |
| ACRO 且 `FW_ACRO_YAW_EN=1` | 先生成 `rates_sp.yaw` 再闭环 |
| 地面滑行 | 独立轮控，走 `landing_gear_wheel` |

故编队发送端所采用的判据 `vehicle_control_mode.flag_control_manual_enabled` 是**生产者同源判据**（见 §9.6），可自动覆盖上述分支。

### 9.4 偏航力矩的来源

- **方向舵**：由 `ActuatorEffectivenessControlSurfaces` 效能矩阵配置，每舵面三轴系数 `CA_SV_CS{i}_TRQ_R/P/Y`。
- **差动推力（链翼关键项）**：新架构内置固定翼前飞的差动推力机制
  `VT_FW_DIFTHR_EN`（bit0=Yaw, bit1=Roll, bit2=Pitch）、`VT_FW_DIFTHR_S_Y/R/P`。
  但**仅在 `vtol_att_control`（tailsitter/tiltrotor）路径生效**；纯固定翼机型不走此路径。

### 9.5 方案评估（**已更正**）

| 方案 | 注入点 | 评估 |
| ---- | ---- | ---- |
| **A. 补丁 `fw_att_control`** | `rates_sp.yaw` | 让 `yaw_sp_move_rate` 作为**偏航速率前馈**叠加进速率设定点。语义与 §3 一致；须处理与协调转弯前馈的叠加、`FW_Y_RMAX` 限幅。**倾向方案** |
| **B. 接收端积分成航向写入 `q_d`** | `q_d` | ❌ **不成立**。按 §9.2 第 1 条，航向分量被 `computeAttitudeError` 消去，指令被原样丢弃。**此方案已否决** |
| **C. 分配层直驱** | ControlAllocator | 用偏航指令直接驱动差动推力/方向舵，绕开姿态环偏航消零；可复用 `VT_FW_DIFTHR_*` 范式。代价：绕开速率闭环，控制律责任下移，§8 频域分析失去明确被控对象 |
| **D. 转成滚转指令** | `roll` | ❌ 不适用。链翼需要的是铰链约束下的**整体转向**，而非靠压坡度转弯 |

**待澄清的物理事实**：从机偏航的执行机构究竟是**方向舵**还是**电机差速**。
- 若为方向舵 → 方案 A 直接可用（舵面本就在效能矩阵内）。
- 若为电机差速 → 纯固定翼下现架构无对应效能项，可能仍需方案 C 的补充改动。

### 9.5b 决策结论（已确认并实现）

**关键澄清（由设计讨论确认）**：链翼构型**仅释放相对滚转与俯仰约束，偏航角在机械上本来就相同**（$\psi_{master}=\psi_L=\psi_R$）。因此：

1. 不存在"三机偏航角协调"问题——它们被铰链强制同角；
2. 偏航控制的目标是**整机（装配体）航迹转向**，要克服的是装配体的 $I_{zz}$，而非单机；
3. 这正是"多机连接后航向转动惯量剧增、方向舵舵效不足"的根源，也是引入**电机差速增推**的原因。

**从机偏航执行机构的性质**：方向舵**未放弃**，但与副翼性质类似——舵效相对装配体惯量已不足。故电机差速是**补充**，不是替代。

**最终方案（三项，均已实现）**：

| # | 改动 | 说明 |
| ---- | ---- | ---- |
| ① | **发送端：手动模式改发"纯杆量偏航率"** | 主机发出 `manual_control_setpoint.yaw × FW_MAN_YR_MAX`（并 clamp 到 `FW_Y_RMAX`），**剔除协调转弯前馈**。各机用**自身**空速独立计算 $\dot\psi_{ff}=gq_1/V$，只共享驾驶员意图，避免从机双计前馈。自主模式下仍透传 `yaw_sp_move_rate`（该量在固定翼栈中无写入者，实际为 0） |
| ② | **从机：`fw_att_control` 增加偏航通道（方案 A）** | 在速率设定点处**叠加** `yaw_sp_move_rate`，与手动杆量走**同一注入点、同一限幅、同一下游**。门控：`FW_FORM_YAW_EN`（默认 0）+ `flag_control_offboard_enabled` + `PX4_ISFINITE`。因 `flag_control_manual_enabled` 在 OFFBOARD 下恒为假，必须显式门控（照抄手动杆判据会永不触发） |
| ③ | **分配层：增推驱动量保留 $\tau_z$（$\tan\phi_{tgt}$ 方案评估后未采纳）** | $\tan\phi$ 推导（见下）确认了稳态"持续需要增推"的需求，但坡度驱动在滚转进入/改出瞬态（逆偏航最大、最需要偏航权限的时刻）恰为零、平飞带偏航率指令时无输出；$\tau_z$ 在需要偏航权限的一切工况必然非零，瞬态与稳态均覆盖。主机按 `FORM_MSTR_YAW_SC`（$\beta\in[0,1]$，默认 0）施加 $|\tau_z|$ 的份额 |

**外翼稳态增推需求的推导**（$\tan\phi_{tgt}$ 方案的物理依据；最终实现保留 §4.3 的"与偏航力矩幅值成正比"表述）：

$$
\Delta V=\dot\psi\,l,\quad
\Delta D\approx\rho S C_D\,V\,\Delta V
\;\xrightarrow{\ \dot\psi=g\tan\phi/V\ }\;
\Delta D=\rho S C_D\,g\,l\,\tan\phi
\;\Longrightarrow\;
\boxed{\ \Delta T\propto\tan\phi\ }
$$

**物理意义**：外翼多飞的速度 $\Delta V$ 正比于 $V\dot\psi$，而协调转弯下 $\dot\psi\propto1/V$，**$V$ 恰好抵消**——故稳态额外阻力只由坡度角决定，与空速无关。这确认了稳态转弯中 $\tan\phi$ 持续非零、正好匹配"持续增推"的需求，是 $\tan\phi_{tgt}$ 方案的立论依据。

**为何实现仍取 $\tau_z$（最终裁决）**：$\tan\phi$ 只覆盖**稳态**增推需求，但该通道同时承担**瞬态偏航权限**。滚转进入/改出瞬态恰是逆偏航最大、最需要偏航权限的时刻，而坡度正在过零，$\tan\phi$ 驱动恰在此刻消失；平飞带偏航率指令（如驾驶员蹬杆修正）时 $\phi\approx0$，坡度驱动完全无输出。$\tau_z$（偏航控制需求）则在需要偏航权限的一切工况必然非零——瞬态、稳态、带指令平飞均覆盖。代价：稳态转弯中 $\tau_z$ 回落至配平值附近，稳态增推幅值需靠 $K_{yaw}$ 试飞标定，本节推导的 $\Delta T\propto\tan\phi$ 即稳态标定的理论参考值；若试飞表明稳态增推不足，可评估在此基础上叠加 $\tan\phi_{tgt}$ 前馈项。

**两侧均不含内侧减速**：增推为**单侧**（`max(side_sign·τz, 0)`，只增不减），内翼保持空速与升力；主机亦按 $\beta$ 施加 $|\tau_z|$ 份额的增推，使整机略微加速而非让内翼掉速。代价是转弯时多余动能需由姿态控制吸收。

**已知的非光滑点**：单侧截断在 $\tau_z=0$ 处不可微（偏航需求过零，而非坡度过零），与抗积分饱和设计存在交互（延续 §4.5-1 的记录）。

**已识别的通道语义更正**：接收端 `q_d` 中取 `self_yaw`（原 §3.3③"航向保持"）在姿态层是 **no-op**——航向分量被 `computeAttitudeError` 剥离。从机航向实际由"偏航速率指令为零 + 风标稳定性 + 铰链约束"自然维持，机制是**不干预**而非"闭环保持"。功能正确，但原表述的机理不成立。

**编译期约束**：发送端引用 `FW_MAN_YR_MAX`/`FW_Y_RMAX`（跨模块参数，为精确复现主机计算）。已在 `Kconfig` 为 `UAVCAN_FORMATION_CONTROLLER` 加 `depends on MODULES_FW_ATT_CONTROL`，使该依赖显式化。

### 9.6 已确定的相关实现（编队链路，已完成）

新架构下已完成并编译验证的部分（与本节决策无关，不受影响）：

1. **DSDL 消息** `nuaa.formation.ControlInput`（DTID 20040），置于 `src/drivers/uavcan/dsdl_custom/nuaa/formation/`。命名空间由 `DSDLC_INPUTS` 中**源目录 basename** 决定，故须加 `.../dsdl_custom/nuaa` 这一层。
2. **主机发送端** `FormationRatesSender`。偏航源判据采用 `flag_control_manual_enabled`——与 `fw_att_control` / `fw_rate_control` 决定"杆量是否进入 `rates_setpoint.yaw`"的判据**同源**，构造上不会随 `nav_state` 重新编号或新增人工模式而漂移。
3. **从机接收端** `FormationRatesBridge`。按 §3.3 解算并发布 `offboard_control_mode` + `vehicle_attitude_setpoint`；架构定位由旧系统的 `sensors/` 桥改为与发送端对称的 link 控制器（`UavcanSensorBridgeBase` 的通道/device-id 抽象不适用于控制设定点链路）。
4. **ControlAllocator 编队混控**（对应 §4，驱动量与 §4 相同为 $\tau_z$，裁决记录见 §9.5b ③）：滚→俯混控 `c(PITCH) += c(ROLL)·CA_RLL2PIT_K·side_sign`（从机、两矩阵均生效）；转弯增推 `c(THRUST_X) += drive·FORM_YAW_K`（仅矩阵 0），其中驱动量 $drive$ 对从机取 $\max(\mathrm{side\_sign}\cdot\tau_z,\,0)$、对主机取 $\mathrm{FORM\_MSTR\_YAW\_SC}\cdot|\tau_z|$，$\tau_z$ 即分配器控制向量中的偏航力矩分量（`control_sp(YAW)`），无新增订阅。
5. **从机偏航通道**：`fw_att_control` 新增 `FW_FORM_YAW_EN` 门控的偏航速率注入（方案 A，见 §9.5b ②）。

> **参数放置**：`FORM_FOLLOWER_EN`、`FORM_POSITION`、`FORM_ROLL_LIM`、`FORM_HINGE_K`、`FORM_YAW_K`、`FORM_MSTR_YAW_SC` 统一定义于 `src/drivers/uavcan/uavcan_params.yaml` 的 `Formation Control` 组；`CA_RLL2PIT_K` 定义于 `src/modules/control_allocator/module.yaml`；`FW_FORM_YAW_EN` 定义于 `src/modules/fw_att_control/fw_att_control_params.yaml`（就近于其消费模块，避免依赖倒置）。
>
> **参数名长度约束**：PX4 参数名上限 16 字符（`FORM_MASTER_YAW_SCALE` 超限被构建拒绝，已改为 `FORM_MSTR_YAW_SC`）。

> **已记录的耦合风险**：参数生成与模块启用绑定（`config_module_list` 仅含 Kconfig 启用的模块），因此 ControlAllocator 引用的 `FORM_*` 依赖 UAVCAN 模块被启用。当前链翼构型必须使用 UAVCAN 编队链路，语义上自洽；但若将来出现"启用 ControlAllocator、关闭 UAVCAN"的板级配置，需将共享参数（`FORM_FOLLOWER_EN`、`FORM_POSITION`）迁至 `control_allocator/module.yaml`（依赖方向应由**可选模块指向必需模块**）。发送端对 `FW_MAN_YR_MAX`/`FW_Y_RMAX` 的依赖已通过 Kconfig 的 `depends on MODULES_FW_ATT_CONTROL` 显式化。

