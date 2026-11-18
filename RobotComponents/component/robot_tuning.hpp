#pragma once
// ==================== 唯一电机数值调参入口 ====================
// 日常只改本文件#define右侧的数值。建议保留f后缀，例如0.375f；无需整数或固定步长。
// 第一次升级Main需两板烧录；之后本文件的数值改动只重建/烧录底盘Main。
// 云台参数经CAN完整校验，在有效遥控的右下急停档应用；运行时不换参。
// 四轮驱动/转向为已验收值，保持原样；yaw/pitch为待实机整定初值。
// 人为整定上限已取消；有限数、非负增益、协议边界、超速/机械保护继续执行。
// 控制模式开关、硬件ID/零位/安装方向不在此调参；不要用Ozone改只读诊断镜像。

// ==================== 一、四个M3508驱动轮：共用参数，各轮独立PID历史 ====================
// 速度环P；误差rad/s，输出A
#ifndef ROBOT_DRIVE_SPEED_KP
#define ROBOT_DRIVE_SPEED_KP 2.5f
#endif
// 速度环I
#ifndef ROBOT_DRIVE_SPEED_KI
#define ROBOT_DRIVE_SPEED_KI 1.0f
#endif
// 速度环D
#ifndef ROBOT_DRIVE_SPEED_KD
#define ROBOT_DRIVE_SPEED_KD 0.003f
#endif
// 最终电流绝对上限，单位A；C620可编码边界20A
#ifndef ROBOT_DRIVE_CURRENT_LIMIT_A
#define ROBOT_DRIVE_CURRENT_LIMIT_A 4.000f
#endif
// 速度环积分输出上限，单位A
#ifndef ROBOT_DRIVE_INTEGRAL_LIMIT_A
#define ROBOT_DRIVE_INTEGRAL_LIMIT_A (ROBOT_DRIVE_CURRENT_LIMIT_A*0.5f)
#endif
// 左杆满幅对应的轮轴转速上限，rpm；已经过减速箱
#ifndef ROBOT_DRIVE_TARGET_SPEED_RPM
#define ROBOT_DRIVE_TARGET_SPEED_RPM 100.0f
#endif
// 非零轮速目标变化率，rpm/s；回中立即给零速，由速度环主动制动
#ifndef ROBOT_DRIVE_ACCELERATION_RPM_S
#define ROBOT_DRIVE_ACCELERATION_RPM_S 480.0f
#endif
// 实际轮轴独立超速保护，rpm；必须严格高于目标上限
#ifndef ROBOT_DRIVE_TRIP_SPEED_RPM
#define ROBOT_DRIVE_TRIP_SPEED_RPM 480.0f
#endif

// ==================== 二、四个GM6020转向：位置外环rad→rad/s，速度内环rad/s→A ====================
// 位置环P
#ifndef ROBOT_STEER_POSITION_KP
#define ROBOT_STEER_POSITION_KP 10.0f
#endif
// 位置环I
#ifndef ROBOT_STEER_POSITION_KI
#define ROBOT_STEER_POSITION_KI 0.0f
#endif
// 位置环D
#ifndef ROBOT_STEER_POSITION_KD
#define ROBOT_STEER_POSITION_KD 0.0f
#endif
// 位置环积分输出上限，rad/s；0表示关闭积分输出
#ifndef ROBOT_STEER_POSITION_INTEGRAL_LIMIT_RAD_S
#define ROBOT_STEER_POSITION_INTEGRAL_LIMIT_RAD_S 0.0f
#endif
// 速度环P
#ifndef ROBOT_STEER_SPEED_KP
#define ROBOT_STEER_SPEED_KP 0.25f
#endif
// 速度环I
#ifndef ROBOT_STEER_SPEED_KI
#define ROBOT_STEER_SPEED_KI 0.0f
#endif
// 速度环D
#ifndef ROBOT_STEER_SPEED_KD
#define ROBOT_STEER_SPEED_KD 0.00f
#endif
// 最终电流绝对上限，A；GM6020电流协议可编码边界3A
#ifndef ROBOT_STEER_CURRENT_LIMIT_A
#define ROBOT_STEER_CURRENT_LIMIT_A 0.500f
#endif
// 速度环积分输出上限，A
#ifndef ROBOT_STEER_SPEED_INTEGRAL_LIMIT_A
#define ROBOT_STEER_SPEED_INTEGRAL_LIMIT_A (ROBOT_STEER_CURRENT_LIMIT_A*0.5f)
#endif
// 位置外环输出给速度内环的目标转速上限，rpm
#ifndef ROBOT_STEER_TARGET_SPEED_RPM
#define ROBOT_STEER_TARGET_SPEED_RPM 120.0f
#endif
// 舵角目标本身的变化率，折合rpm；不能超过目标转速上限
#ifndef ROBOT_STEER_TARGET_RATE_RPM
#define ROBOT_STEER_TARGET_RATE_RPM 120.0f
#endif
// 实际转向速度独立超速保护，rpm；必须严格高于目标转速上限
#ifndef ROBOT_STEER_TRIP_SPEED_RPM
#define ROBOT_STEER_TRIP_SPEED_RPM 180.0f
#endif

// ==================== 三、yaw水平轴DM-J4310（底盘执行）：以下为待实机整定初值 ====================
// yaw左右瞄准速度独立入口：右杆左右满幅时角度目标的稳态变化率，rpm；回中冻结角度目标。
// 不由第五组加速度决定；实际跟随还受位置环速度/目标变化率/力矩能力约束。
#ifndef ROBOT_YAW_AIM_RPM
#define ROBOT_YAW_AIM_RPM 30.0f
#endif
// 位置环P；角度误差rad→目标速度rad/s
#ifndef ROBOT_YAW_POSITION_KP
#define ROBOT_YAW_POSITION_KP 15.0f
#endif
// 位置环I
#ifndef ROBOT_YAW_POSITION_KI
#define ROBOT_YAW_POSITION_KI 0.0f
#endif
// 位置环D
#ifndef ROBOT_YAW_POSITION_KD
#define ROBOT_YAW_POSITION_KD 0.0f
#endif
// 位置环积分输出上限，rad/s
#ifndef ROBOT_YAW_POSITION_INTEGRAL_RAD_S
#define ROBOT_YAW_POSITION_INTEGRAL_RAD_S 0.0f
#endif
// 位置外环输出给速度内环的目标转速上限，rpm
#ifndef ROBOT_YAW_POSITION_SPEED_RPM
#define ROBOT_YAW_POSITION_SPEED_RPM (ROBOT_YAW_AIM_RPM*2.0f)
#endif
// 速度环P；速度误差rad/s→力矩Nm
#ifndef ROBOT_YAW_SPEED_KP
#define ROBOT_YAW_SPEED_KP 0.42f
#endif
// 速度环I；pitch悬停需要用积分补偿持续负载，仍须现场整定
#ifndef ROBOT_YAW_SPEED_KI
#define ROBOT_YAW_SPEED_KI 0.0f
#endif
// 速度环D
#ifndef ROBOT_YAW_SPEED_KD
#define ROBOT_YAW_SPEED_KD 0.0005f
#endif
// 最终力矩绝对上限，Nm；不是恒力矩输出，不保证足以悬停
#ifndef ROBOT_YAW_TORQUE_NM
#define ROBOT_YAW_TORQUE_NM 2.0f
#endif
// 速度环积分输出上限，Nm
#ifndef ROBOT_YAW_SPEED_INTEGRAL_NM
#define ROBOT_YAW_SPEED_INTEGRAL_NM ROBOT_YAW_TORQUE_NM
#endif
// 本轴位置目标变化率，rpm；限制进入Working回水平及瞄准目标的追随速度
#ifndef ROBOT_YAW_TARGET_RATE_RPM
#define ROBOT_YAW_TARGET_RATE_RPM (ROBOT_YAW_AIM_RPM*2.0f)
#endif
// 实际关节独立超速保护，rpm；必须严格高于位置环输出速度上限
#ifndef ROBOT_YAW_TRIP_RPM
#define ROBOT_YAW_TRIP_RPM (ROBOT_YAW_POSITION_SPEED_RPM*2.0f)
#endif

// ==================== 四、pitch俯仰轴DM-J4310（云台执行，参数由底盘下发）：以下为待实机整定初值 ====================
// pitch上下瞄准速度独立入口：右杆上下满幅时角度目标的稳态变化率，rpm；回中冻结角度目标。
// 不由第五组加速度决定；实际跟随还受位置环速度/目标变化率/力矩能力约束。
#ifndef ROBOT_PITCH_AIM_RPM
#define ROBOT_PITCH_AIM_RPM 20.0f
#endif
// 位置环P；角度误差rad→目标速度rad/s
#ifndef ROBOT_PITCH_POSITION_KP
#define ROBOT_PITCH_POSITION_KP 15.0f
#endif
// 位置环I
#ifndef ROBOT_PITCH_POSITION_KI
#define ROBOT_PITCH_POSITION_KI 0.0f
#endif
// 位置环D
#ifndef ROBOT_PITCH_POSITION_KD
#define ROBOT_PITCH_POSITION_KD 0.0f
#endif
// 位置环积分输出上限，rad/s
#ifndef ROBOT_PITCH_POSITION_INTEGRAL_RAD_S
#define ROBOT_PITCH_POSITION_INTEGRAL_RAD_S 0.0f
#endif
// 位置外环输出给速度内环的目标转速上限，rpm
#ifndef ROBOT_PITCH_POSITION_SPEED_RPM
#define ROBOT_PITCH_POSITION_SPEED_RPM (ROBOT_PITCH_AIM_RPM*2.0f)
#endif
// 速度环P；速度误差rad/s→力矩Nm
#ifndef ROBOT_PITCH_SPEED_KP
#define ROBOT_PITCH_SPEED_KP 1.0f
#endif
// 速度环I；pitch悬停需要用积分补偿持续负载，仍须现场整定
#ifndef ROBOT_PITCH_SPEED_KI
#define ROBOT_PITCH_SPEED_KI 0.0f
#endif
// 速度环D
#ifndef ROBOT_PITCH_SPEED_KD
#define ROBOT_PITCH_SPEED_KD 0.0f
#endif
// 最终力矩绝对上限，Nm；不是恒力矩输出，不保证足以悬停
#ifndef ROBOT_PITCH_TORQUE_NM
#define ROBOT_PITCH_TORQUE_NM 2.0f
#endif
// 速度环积分输出上限，Nm
#ifndef ROBOT_PITCH_SPEED_INTEGRAL_NM
#define ROBOT_PITCH_SPEED_INTEGRAL_NM ROBOT_PITCH_TORQUE_NM
#endif
// 本轴位置目标变化率，rpm；限制进入Working回水平及瞄准目标的追随速度
#ifndef ROBOT_PITCH_TARGET_RATE_RPM
#define ROBOT_PITCH_TARGET_RATE_RPM ROBOT_PITCH_AIM_RPM
#endif
// 实际关节独立超速保护，rpm；必须严格高于位置环输出速度上限
#ifndef ROBOT_PITCH_TRIP_RPM
#define ROBOT_PITCH_TRIP_RPM (ROBOT_PITCH_POSITION_SPEED_RPM*2.0f)
#endif

// ==================== 五、云台瞄准加速度（两轴共用，仅决定加减速快慢） ====================
// 单位rpm/s，不是转速上限：yaw/pitch稳态瞄准速度分别由各自ROBOT_*_AIM_RPM决定。
// 从静止到满幅的加速时间约为该轴AIM_RPM/本参数；0取消此平滑。回中立即冻结目标，闭环保持。
#ifndef ROBOT_AIM_ACCEL_RPM_S
#define ROBOT_AIM_ACCEL_RPM_S 120.0f
#endif

// ==================== 六、pitch目标角度限位（rad） ====================
// 1限制角度目标并启用边界保护带，0关闭。实测越界时保持闭环并禁止向外施力。
#ifndef ROBOT_PITCH_POSITION_LIMIT_ENABLED
#define ROBOT_PITCH_POSITION_LIMIT_ENABLED 1
#endif
static_assert(ROBOT_PITCH_POSITION_LIMIT_ENABLED==0 || ROBOT_PITCH_POSITION_LIMIT_ENABLED==1,"pitch角度限位开关只能为0或1");
// 标定后数值的最小俯仰角，换零位或机械结构后重新核对。
#ifndef ROBOT_PITCH_MIN_RAD
#define ROBOT_PITCH_MIN_RAD (-0.419f)
#endif
// 最大俯仰角；不得为了调PID向机械止挡放宽
#ifndef ROBOT_PITCH_MAX_RAD
#define ROBOT_PITCH_MAX_RAD 0.515f
#endif
// 边界保护带，rad；按剩余角度限制向外的速度目标，实际越界才阻断向外力矩。
#ifndef ROBOT_PITCH_GUARD_RAD
#define ROBOT_PITCH_GUARD_RAD 0.03f
#endif

// ==================== 七、历史单电机单位置环模式（Main/FourSteer/DriveSpeed/VectorDrive不使用） ====================
// 仅旧MotorDebugPower的位置P，A/rad；旧协议保留既有档位检查
#ifndef ROBOT_MOTOR_DEBUG_KP
#define ROBOT_MOTOR_DEBUG_KP 8.0f
#endif
// 仅旧MotorDebugPower的电流上限，A；不用于本次双环整定
#ifndef ROBOT_MOTOR_DEBUG_CURRENT_LIMIT_A
#define ROBOT_MOTOR_DEBUG_CURRENT_LIMIT_A 0.100f
#endif

// ==================== 八、云台前馈（和PID合计后受总力矩限幅） ====================
// yaw速度前馈Kv，Nm/(rad/s)：乘速度内环目标值；建议先保持原PID，只逐步调此系数。
#ifndef ROBOT_YAW_VELOCITY_FF
#define ROBOT_YAW_VELOCITY_FF 0.08f
#endif
// pitch重力前馈：A*cos(实际角度)+B*sin(实际角度)+C，Nm；由七组静止保持扭矩指令拟合。
#ifndef ROBOT_PITCH_GRAVITY_FF_ENABLED
#define ROBOT_PITCH_GRAVITY_FF_ENABLED 1
#endif
#ifndef ROBOT_PITCH_GRAVITY_FF_COS_NM
#define ROBOT_PITCH_GRAVITY_FF_COS_NM 0.6283247f
#endif
#ifndef ROBOT_PITCH_GRAVITY_FF_SIN_NM
#define ROBOT_PITCH_GRAVITY_FF_SIN_NM (-0.3856459f)
#endif
#ifndef ROBOT_PITCH_GRAVITY_FF_BIAS_NM
#define ROBOT_PITCH_GRAVITY_FF_BIAS_NM 0.0f
#endif
// 下面是实测拟合覆盖的角度区间；区间外采用曲线端点，不外推，也不限制枪口运动。
#ifndef ROBOT_PITCH_GRAVITY_FF_MIN_RAD
#define ROBOT_PITCH_GRAVITY_FF_MIN_RAD (-0.412754297f)
#endif
#ifndef ROBOT_PITCH_GRAVITY_FF_MAX_RAD
#define ROBOT_PITCH_GRAVITY_FF_MAX_RAD 0.531520844f
#endif
static_assert(ROBOT_PITCH_GRAVITY_FF_ENABLED==0 || ROBOT_PITCH_GRAVITY_FF_ENABLED==1,"重力前馈开关只能为0或1");
