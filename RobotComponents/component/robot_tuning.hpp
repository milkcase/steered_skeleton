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
#define ROBOT_DRIVE_CURRENT_LIMIT_A 2.000f
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
// 右杆本方向满幅时瞄准目标移动的角速度，rpm；回中冻结角度目标
#ifndef ROBOT_YAW_AIM_RPM
#define ROBOT_YAW_AIM_RPM 10.0f
#endif
// 位置环P；角度误差rad→目标速度rad/s
#ifndef ROBOT_YAW_POSITION_KP
#define ROBOT_YAW_POSITION_KP 4.0f
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
#define ROBOT_YAW_SPEED_KP 0.12f
#endif
// 速度环I；pitch悬停需要用积分补偿持续负载，仍须现场整定
#ifndef ROBOT_YAW_SPEED_KI
#define ROBOT_YAW_SPEED_KI 0.1f
#endif
// 速度环D
#ifndef ROBOT_YAW_SPEED_KD
#define ROBOT_YAW_SPEED_KD 0.0f
#endif
// 最终力矩绝对上限，Nm；不是恒力矩输出，不保证足以悬停
#ifndef ROBOT_YAW_TORQUE_NM
#define ROBOT_YAW_TORQUE_NM 0.5f
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
// 右杆本方向满幅时瞄准目标移动的角速度，rpm；回中冻结角度目标
#ifndef ROBOT_PITCH_AIM_RPM
#define ROBOT_PITCH_AIM_RPM 5.0f
#endif
// 位置环P；角度误差rad→目标速度rad/s
#ifndef ROBOT_PITCH_POSITION_KP
#define ROBOT_PITCH_POSITION_KP 5.0f
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
#define ROBOT_PITCH_SPEED_KP 0.2f
#endif
// 速度环I；pitch悬停需要用积分补偿持续负载，仍须现场整定
#ifndef ROBOT_PITCH_SPEED_KI
#define ROBOT_PITCH_SPEED_KI 0.5f
#endif
// 速度环D
#ifndef ROBOT_PITCH_SPEED_KD
#define ROBOT_PITCH_SPEED_KD 0.0f
#endif
// 最终力矩绝对上限，Nm；不是恒力矩输出，不保证足以悬停
#ifndef ROBOT_PITCH_TORQUE_NM
#define ROBOT_PITCH_TORQUE_NM 0.5f
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

// ==================== 五、云台瞄准平滑度（两轴共用） ====================
// 偏杆时目标瞄准角速度的变化率，rpm/s；回中立刻冻结目标角度，闭环继续保持
#ifndef ROBOT_AIM_ACCEL_RPM_S
#define ROBOT_AIM_ACCEL_RPM_S 20.0f
#endif

// ==================== 六、pitch现场已确认的机械安全区（rad） ====================
// 最小俯仰角，枪口仰起方向为负；仅换零位/机械结构后重新核对
#ifndef ROBOT_PITCH_MIN_RAD
#define ROBOT_PITCH_MIN_RAD (-0.419f)
#endif
// 最大俯仰角；不得为了调PID向机械止挡放宽
#ifndef ROBOT_PITCH_MAX_RAD
#define ROBOT_PITCH_MAX_RAD 0.515f
#endif
// 边界保护带，rad；进入保护带后禁止继续向外施力
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
